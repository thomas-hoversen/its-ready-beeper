/**
 * @file main.cpp
 * @brief Example with beep detection + captive portal 
 */

#include <Arduino.h>
#include <SPIFFS.h>
#include "BluetoothA2DPSource.h"
#include <arduinoFFT.h>
#include "esp_task_wdt.h"
#include <driver/i2s.h>

// Our beep logic classes
#include "BeepDetector.h"
#include "BeepHistory.h"

// Captive Portal
#include "CaptivePortal.h"

// ---------------------------------------------------------------------------
// Configuration Constants (RELAXED DETECTION)
// ---------------------------------------------------------------------------

// I2S
#define I2S_SCK  14
#define I2S_WS   15
#define I2S_SD   34
#define I2S_PORT I2S_NUM_0

static const int   I2S_SAMPLE_RATE      = 16000;
static const bool  USE_APLL             = false;
static const bool  TX_DESC_AUTO_CLEAR   = false;
static const int   FIXED_MCLK           = 0;
static const int   I2S_DMA_BUF_COUNT    = 4;
static const int   I2S_DMA_BUF_LEN      = 128;

// FFT & Beep Detection
#define SAMPLES                  512
#define SAMPLING_FREQUENCY       16000
#define FREQUENCY_MIN            1200
#define FREQUENCY_MAX            9000
static float BIN_AMPLITUDE_THRESHOLD    = 10;
static float AMPLITUDE_THRESHOLD        = 3000;
static const int   I2S_READ_CHUNK       = 64;
static const int   MIC_PROXIMITY_THRESHOLD = 100;

// Consecutive-window detection
static const int   MULTI_WINDOW_CONFIDENCE = 4;
static const unsigned long DETECTION_DELAY  = 1000; // 1s
static const unsigned long TIME_WINDOW_MS   = 5000;
static const int           THRESHOLD_COUNT  = 3;

// LED / Button
#define LED_PIN          2
#define BUTTON_PIN       25
static const int   LED_FLASH_DURATION = 500;

// ---------------------------------------------------------------------------
// Instantiate classes
// ---------------------------------------------------------------------------
BeepDetector beepDetector(MULTI_WINDOW_CONFIDENCE, DETECTION_DELAY);
BeepHistory  beepHistory(THRESHOLD_COUNT, TIME_WINDOW_MS);

// FFT arrays
float vReal[SAMPLES];
float vImag[SAMPLES];
#include <arduinoFFT.h>
ArduinoFFT<float> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

// Bluetooth + Audio
BluetoothA2DPSource a2dp_source;
const char* bluetoothSpeakerName = "C17A";
const char* audioFilePath        = "/audio1.wav";
File audioFile;  

// State
bool playingAudio         = false;
unsigned long ignoreBeepUntil = 0;  
int16_t maxSampleInFrame  = 0;

// Captive Portal
CaptivePortal captivePortal;

// ---------------------------------------------------------------------------
// Function Declarations
// ---------------------------------------------------------------------------
int32_t audioDataCallback(uint8_t* data, int32_t len);

void initPins();
void initWatchdogTimer();
void initSPIFFS();
void initI2S();
void initBluetooth();

void startAudioPlayback();
void handlePlaybackCompletion();
void collectAudioSamples();
void runFFT();
bool detectBeep();
void handleBeepDetection();

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    Serial.println("\n\nInitializing (Relaxed Detection)...");

    initPins();
    initWatchdogTimer();
    initSPIFFS();
    initI2S();
    initBluetooth();

    // Start the captive portal
    captivePortal.begin();

    Serial.println("System ready. Looking for beeps or button presses...");
}

// ---------------------------------------------------------------------------
// Loop
// ---------------------------------------------------------------------------
void loop() {
    // (1) Run captive portal server first
    captivePortal.handleClients();

    // (2) Button => immediate playback
    if (digitalRead(BUTTON_PIN) == LOW) {
        Serial.println("Button pressed => immediate playback");
        startAudioPlayback();
    }

    // (3) Check if playback ended
    handlePlaybackCompletion();

    // (4) Gather samples + analyze
    collectAudioSamples();
    runFFT();
    handleBeepDetection();
}

// ---------------------------------------------------------------------------
// Gather samples from I2S
// ---------------------------------------------------------------------------
void collectAudioSamples() {
    maxSampleInFrame = 0;
    int count = 0;

    while (count < SAMPLES) {
        uint8_t i2sBuffer[I2S_READ_CHUNK * 4];
        size_t bytesRead = 0;
        i2s_read(I2S_PORT, i2sBuffer, sizeof(i2sBuffer), &bytesRead, portMAX_DELAY);

        int frames = bytesRead / 4;
        for (int i = 0; i < frames && count < SAMPLES; i++) {
            uint8_t mid = i2sBuffer[i*4 + 2];
            uint8_t msb = i2sBuffer[i*4 + 3];

            int16_t sample16 = (int16_t)((msb << 8) | mid);
            if (abs(sample16) > abs(maxSampleInFrame)) {
                maxSampleInFrame = sample16;
            }

            vReal[count] = (float)sample16;
            vImag[count] = 0.0f;
            count++;
            esp_task_wdt_reset();
        }
    }
}

// ---------------------------------------------------------------------------
// Perform FFT
// ---------------------------------------------------------------------------
void runFFT() {
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();
}

// ---------------------------------------------------------------------------
// Check if there's a beep in this FFT window
// ---------------------------------------------------------------------------
bool detectBeep() {
    if (abs(maxSampleInFrame) <= MIC_PROXIMITY_THRESHOLD) {
        return false; 
    }
    float majorFrequency = FFT.majorPeak();
    float amplitude      = 0.0f;
    bool  sufficientBins = false;

    for (int i = 0; i < SAMPLES/2; i++) {
        float freq = (i * (float)SAMPLING_FREQUENCY) / (float)SAMPLES;
        if (freq >= FREQUENCY_MIN && freq <= FREQUENCY_MAX) {
            amplitude += vReal[i];
            if (vReal[i] > BIN_AMPLITUDE_THRESHOLD) {
                sufficientBins = true;
            }
        }
        esp_task_wdt_reset();
    }

    bool beepDetected = (
        majorFrequency >= FREQUENCY_MIN &&
        majorFrequency <= FREQUENCY_MAX &&
        amplitude      >  AMPLITUDE_THRESHOLD &&
        sufficientBins
    );
    return beepDetected;
}

// ---------------------------------------------------------------------------
// Use beepDetector + beepHistory => Possibly start audio
// ---------------------------------------------------------------------------
void handleBeepDetection() {
    if (playingAudio) return; 
    if (millis() < ignoreBeepUntil) return;

    bool beepInThisWindow = detectBeep();
    bool beepEvent = beepDetector.update(beepInThisWindow, millis());

    if (beepEvent) {
        Serial.println("-> A new beep event just got confirmed!");
        beepHistory.addBeep(millis());

        if (beepHistory.hasReachedThreshold()) {
            Serial.println("Beep history => threshold => Start playback!");
            beepHistory.clear();
            startAudioPlayback();
        } else {
            Serial.println("Not enough beep events yet...");
        }
    }
}

// ---------------------------------------------------------------------------
// Playback
// ---------------------------------------------------------------------------
int32_t audioDataCallback(uint8_t* data, int32_t len) {
    if (!audioFile || !audioFile.available()) {
        return 0;
    }
    size_t bytesRead = audioFile.read(data, len);
    return (int32_t)bytesRead;
}

void startAudioPlayback() {
    if (!a2dp_source.is_connected()) {
        Serial.println("No BT connection => skip playback");
        return;
    }
    if (audioFile) {
        audioFile.close();
    }
    audioFile = SPIFFS.open(audioFilePath, "r");
    if (!audioFile || !audioFile.available()) {
        Serial.println("Audio file not available => skip");
        return;
    }
    if (!audioFile.seek(0, SeekSet)) {
        Serial.println("Failed to rewind audio file => skip");
        return;
    }

    playingAudio = true;
    Serial.println("Starting audio playback...");

    // ONLY flash LED right before playing
    Serial.println("Flashing LED because audio is about to play...");
    digitalWrite(LED_PIN, HIGH);
    delay(LED_FLASH_DURATION);
    digitalWrite(LED_PIN, LOW);
}

void handlePlaybackCompletion() {
    if (playingAudio && (!audioFile || !audioFile.available())) {
        Serial.println("Playback ended.");
        playingAudio = false;
        if (audioFile) {
            audioFile.close();
        }
        ignoreBeepUntil = millis() + 1500;
    }
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------
void initPins() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    digitalWrite(LED_PIN, LOW);
    Serial.println("Pins init.");
}

void initWatchdogTimer() {
    esp_task_wdt_init(10, true);
    esp_task_wdt_add(NULL);
    Serial.println("Watchdog init.");
}

void initSPIFFS() {
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS mount failed!");
        while(1){delay(100);}
    }
    Serial.println("SPIFFS mounted.");
    File root = SPIFFS.open("/");
    File f = root.openNextFile();
    while(f) {
        Serial.print("File: ");
        Serial.println(f.name());
        f = root.openNextFile();
    }
}

void initI2S() {
    Serial.println("[initI2S] Installing driver...");
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = 0,
        .dma_buf_count = I2S_DMA_BUF_COUNT,
        .dma_buf_len = I2S_DMA_BUF_LEN,
        .use_apll = USE_APLL,
        .tx_desc_auto_clear = TX_DESC_AUTO_CLEAR,
        .fixed_mclk = FIXED_MCLK
    };
    esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if(err != ESP_OK) {
        Serial.printf("[initI2S] driver_install failed: %d\n", err);
        while(true){delay(100);}
    }
    i2s_pin_config_t pinConfig = {
        .bck_io_num = I2S_SCK,
        .ws_io_num  = I2S_WS,
        .data_out_num = -1,
        .data_in_num  = I2S_SD
    };
    err = i2s_set_pin(I2S_PORT, &pinConfig);
    if(err != ESP_OK){
        Serial.printf("[initI2S] set_pin failed: %d\n", err);
        while(true){delay(100);}
    }
    i2s_set_clk(I2S_PORT, I2S_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);
    i2s_start(I2S_PORT);
    Serial.println("[initI2S] done.");
}

void initBluetooth() {
    Serial.println("BT init...");
    a2dp_source.set_auto_reconnect(true);
    a2dp_source.set_on_connection_state_changed([](esp_a2d_connection_state_t state, void*){
        Serial.printf("[BT] Connection => %s\n", a2dp_source.to_str(state));
    });
    a2dp_source.start_raw(bluetoothSpeakerName, audioDataCallback);
    a2dp_source.set_volume(20);
    Serial.printf("BT name='%s' => connect speaker.\n", bluetoothSpeakerName);
}