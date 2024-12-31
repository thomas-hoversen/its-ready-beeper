#include <Arduino.h>
#include "MyA2DPSimple.h"
#include <SPIFFS.h>
#include <esp_task_wdt.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>

// ---------------------------------------------------------------------------
// BEEP DETECTION HEADERS
// ---------------------------------------------------------------------------
#include "BeepDetector.h"
#include "BeepHistory.h"

// ---------------------------------------------------------------------------
// WAV / Bluetooth streaming constants
// ---------------------------------------------------------------------------
static File audioFile;
#define READAHEAD_SIZE 2048
static uint8_t readAheadBuffer[READAHEAD_SIZE];
static size_t  bufferDataLen  = 0;
static size_t  bufferReadPos  = 0;

// Our custom A2DP object
static MyA2DPSimple a2dpSimple;

// ---------------------------------------------------------------------------
// LED / BUTTON PINS
// ---------------------------------------------------------------------------
#define LED_BLUE      16
#define LED_ORANGE    17
#define BUTTON_PIN    25

// Flag for WAV playback
static bool playingAudio = false;

// ---------------------------------------------------------------------------
// BEEP DETECTION CONFIG
// ---------------------------------------------------------------------------
#define I2S_SCK   14
#define I2S_WS    15
#define I2S_SD    34
#define I2S_PORT  I2S_NUM_0

static const int   I2S_SAMPLE_RATE        = 16000;
static const bool  USE_APLL               = false;
static const bool  TX_DESC_AUTO_CLEAR     = false;
static const int   FIXED_MCLK             = 0;
static const int   I2S_DMA_BUF_COUNT      = 4;
static const int   I2S_DMA_BUF_LEN        = 128;

#define SAMPLES                      512
#define SAMPLING_FREQUENCY          16000
#define FREQUENCY_MIN               1200
#define FREQUENCY_MAX               9000
static float BIN_AMPLITUDE_THRESHOLD      = 10;
static float AMPLITUDE_THRESHOLD          = 3000;
static const int   I2S_READ_CHUNK         = 64;
static const int   MIC_PROXIMITY_THRESHOLD= 100;
static const int   MULTI_WINDOW_CONFIDENCE= 4;
static const unsigned long DETECTION_DELAY= 1000;  // 1s
static const unsigned long TIME_WINDOW_MS = 5000;
static const int   THRESHOLD_COUNT        = 3;

// Arrays for FFT
static float vReal[SAMPLES];
static float vImag[SAMPLES];
ArduinoFFT<float> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

// beep detection logic
BeepDetector beepDetector(MULTI_WINDOW_CONFIDENCE, DETECTION_DELAY);
BeepHistory  beepHistory(THRESHOLD_COUNT, TIME_WINDOW_MS);

// beep detection state
static int16_t maxSampleInFrame  = 0;
static unsigned long ignoreBeepUntil = 0;

// ---------------------------------------------------------------------------
// LED logic
// ---------------------------------------------------------------------------
static bool     connectedLedActive  = false;  // true for 3s after connection
static uint32_t connectedLedStartMs = 0;
static bool     wasConnected        = false;

// Fading variables
static int fadeValue     = 0;
static int fadeIncrement = 5;

// ---------------------------------------------------------------------------
// WAV / Bluetooth STREAMING CALLBACK
// ---------------------------------------------------------------------------
int32_t audioDataCallback(uint8_t* data, int32_t len) {
    if (!audioFile || !audioFile.available()) {
        return 0;
    }
    if (bufferReadPos >= bufferDataLen) {
        bufferDataLen = audioFile.read(readAheadBuffer, READAHEAD_SIZE);
        bufferReadPos = 0;
        if (bufferDataLen == 0) {
            return 0;
        }
    }
    int32_t available = bufferDataLen - bufferReadPos;
    int32_t toCopy    = (available < len) ? available : len;
    memcpy(data, readAheadBuffer + bufferReadPos, toCopy);
    bufferReadPos += toCopy;
    return toCopy;
}

// ---------------------------------------------------------------------------
// AUDIO PLAYBACK CONTROL
// ---------------------------------------------------------------------------
void startAudioPlayback() {
    if (!a2dpSimple.isConnected()) {
        Serial.println("[MAIN] Not connected => skipping playback.");
        return;
    }
    if (audioFile) {
        audioFile.close();
    }
    audioFile = SPIFFS.open("/audio1.wav", "r");
    if (!audioFile) {
        Serial.println("[MAIN] Could not open /audio1.wav => silent streaming.");
        return;
    }
    size_t fileSize = audioFile.size();
    Serial.printf("[MAIN] Opened /audio1.wav, size=%u bytes\n", (unsigned)fileSize);

    if (!audioFile.seek(0, SeekSet)) {
        Serial.println("[MAIN] Failed to rewind => skip playback");
        audioFile.close();
        return;
    }
    bufferDataLen = 0;
    bufferReadPos = 0;
    playingAudio  = true;
    Serial.println("[MAIN] Starting audio playback...");
}

void handlePlaybackCompletion() {
    // If we were playing, but file ended or unavailable
    if (playingAudio && (!audioFile || !audioFile.available())) {
        Serial.println("[MAIN] Playback ended or file unavailable.");
        playingAudio = false;
        if (audioFile) {
            audioFile.close();
            Serial.println("[MAIN] audioFile closed after playback.");
        }
        // short wait before beep detection
        ignoreBeepUntil = millis() + 1500;
    }
}

// ---------------------------------------------------------------------------
// SPIFFS INIT
// ---------------------------------------------------------------------------
void initSPIFFS() {
    if (!SPIFFS.begin(true)) {
        Serial.println("[MAIN] SPIFFS mount failed!");
        while (true) { delay(100); }
    }
    Serial.println("[MAIN] SPIFFS successfully mounted.");

    // Optional: list SPIFFS files
    Serial.println("[MAIN] Listing SPIFFS files:");
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    while (file) {
        Serial.printf("  -> %s (%llu bytes)\n", file.name(), file.size());
        file = root.openNextFile();
    }
    Serial.println("[MAIN] Done listing SPIFFS files.\n");
}

// ---------------------------------------------------------------------------
// WATCHDOG INIT
// ---------------------------------------------------------------------------
void initWatchdogTimer() {
    esp_task_wdt_init(30, true); // 30s WDT
    esp_task_wdt_add(NULL);
    Serial.println("[MAIN] Watchdog init done.");
}

// ---------------------------------------------------------------------------
// I2S INIT (microphone)
// ---------------------------------------------------------------------------
void initI2S() {
    Serial.println("[initI2S] Installing driver...");
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        // for some mics, use I2S_CHANNEL_FMT_ONLY_RIGHT
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        // standard I2S (most mics)
        .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S),
        .intr_alloc_flags = 0,
        .dma_buf_count = I2S_DMA_BUF_COUNT,
        .dma_buf_len = I2S_DMA_BUF_LEN,
        .use_apll = USE_APLL,
        .tx_desc_auto_clear = TX_DESC_AUTO_CLEAR,
        .fixed_mclk = FIXED_MCLK
    };
    esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if(err != ESP_OK) {
        Serial.printf("[initI2S] driver_install failed => %d\n", err);
        while(true){delay(100);}
    }
    i2s_pin_config_t pinConfig = {
        .bck_io_num   = I2S_SCK,
        .ws_io_num    = I2S_WS,
        .data_out_num = -1,
        .data_in_num  = I2S_SD
    };
    err = i2s_set_pin(I2S_PORT, &pinConfig);
    if(err != ESP_OK){
        Serial.printf("[initI2S] set_pin failed => %d\n", err);
        while(true){delay(100);}
    }
    i2s_set_clk(I2S_PORT, I2S_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);
    i2s_start(I2S_PORT);
    Serial.println("[initI2S] Driver installed & started.");
}

// ---------------------------------------------------------------------------
// BEEP DETECTION
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
            // parse 32-bit data: top 16 bits are actual sample
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

void runFFT() {
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();
}

bool detectBeep() {
    // If audio is extremely low, skip
    if (abs(maxSampleInFrame) <= MIC_PROXIMITY_THRESHOLD) {
        return false;
    }
    float majorFrequency = FFT.majorPeak();
    float amplitude      = 0.0f;
    bool  sufficientBins = false;

    // Sum amplitude in [FREQUENCY_MIN, FREQUENCY_MAX]
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

void handleBeepDetection() {
    // If audio is playing or ignoring beeps => skip
    if (playingAudio) return;
    if (millis() < ignoreBeepUntil) return;

    bool beepInThisWindow = detectBeep();
    bool beepEvent        = beepDetector.update(beepInThisWindow, millis());

    if (beepEvent) {
        Serial.println("[BEEP] Confirmed beep detected!");
        beepHistory.addBeep(millis());

        if (beepHistory.hasReachedThreshold()) {
            Serial.println("[BEEP] Threshold reached => Start playback!");
            beepHistory.clear();
            startAudioPlayback();
        } else {
            Serial.println("[BEEP] Not enough beep events yet...");
        }
    }
}

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n[MAIN] System starting up...");

    // Basic config logs
    Serial.println("[MAIN] Startup Configuration:");
    Serial.printf("    I2S sample rate: %d\n", I2S_SAMPLE_RATE);
    Serial.printf("    Mic threshold: %d\n", MIC_PROXIMITY_THRESHOLD);
    Serial.printf("    Frequency range: %d-%d Hz\n", FREQUENCY_MIN, FREQUENCY_MAX);
    Serial.printf("    Audio read buffer: %d bytes\n", READAHEAD_SIZE);

    pinMode(LED_ORANGE, OUTPUT);
    digitalWrite(LED_ORANGE, LOW);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Blue LED => fade channel
    ledcSetup(0, 5000, 8);
    ledcAttachPin(LED_BLUE, 0);
    ledcWrite(0, 0);

    initWatchdogTimer();
    initSPIFFS();
    initI2S();

    a2dpSimple.begin("MyESP32_Simple", audioDataCallback);
    Serial.println("[MAIN] Setup complete.\n");
}

// ---------------------------------------------------------------------------
// LOOP
// ---------------------------------------------------------------------------
void loop() {
    // Feed the watchdog
    esp_task_wdt_reset();

    // 1) Button => start playback if not already
    if (digitalRead(BUTTON_PIN) == LOW) {
        if (!playingAudio) {
            Serial.println("[MAIN] Button => start playback...");
            startAudioPlayback();
        }
    }

    // 2) Check if playback ended
    handlePlaybackCompletion();

    // 3) LED: orange ON if playing
    digitalWrite(LED_ORANGE, playingAudio ? HIGH : LOW);

    // 4) Bluetooth states
    bool nowScanning   = a2dpSimple.isScanning();
    bool nowConnecting = a2dpSimple.isConnecting();
    bool nowConnected  = a2dpSimple.isConnected();

    // If newly connected => 3s bright
    if (nowConnected && !wasConnected) {
        connectedLedActive  = true;
        connectedLedStartMs = millis();
        ledcWrite(0, 255);  // full bright
        Serial.println("[MAIN] Blue LED => FULL BRIGHT (connected).");
    }

    // If we are in the 3s bright phase, check if 3s passed
    if (connectedLedActive) {
        if (millis() - connectedLedStartMs >= 3000) {
            connectedLedActive = false;
            // Turn off
            ledcWrite(0, 0);
            Serial.println("[MAIN] Blue LED => OFF (post-connection).");
        }
    } 
    else {
        // Not in the "connected bright" phase
        if (nowConnected) {
            // Already connected => LED off
            ledcWrite(0, 0);
        }
        else {
            // If not connected => fade if scanning or connecting
            if (nowScanning || nowConnecting) {
                fadeValue += fadeIncrement;
                if (fadeValue >= 255) {
                    fadeValue = 255;
                    fadeIncrement = -fadeIncrement;
                } else if (fadeValue <= 0) {
                    fadeValue = 0;
                    fadeIncrement = -fadeIncrement;
                }
                ledcWrite(0, fadeValue);
            } else {
                // Otherwise => LED off
                ledcWrite(0, 0);
            }
        }
    }

    wasConnected = nowConnected;

    // 5) If connected & not playing => beep detection
    if (nowConnected && !playingAudio) {
        collectAudioSamples();
        runFFT();
        handleBeepDetection();
    }

    delay(10);
}