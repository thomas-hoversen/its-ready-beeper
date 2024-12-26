/**
 * @file main_preDownloadedWav_I2S.cpp
 * @brief This program listens for a beep from an I2S microphone (e.g. INMP441),
 *        and upon beep detection (or button press), plays a WAV file from SPIFFS
 *        to a Bluetooth speaker. 
 *
 *        Features:
 *        - I2S audio sampling from an INMP441 for beep detection only (no recording saved).
 *        - Frequency domain analysis (FFT) to detect beep presence.
 *        - Automatic playback of an existing file in SPIFFS over BT.
 *        - Button press can also trigger immediate playback.
 *        - Watchdog timer + LED indicator.
 *
 * Note: This refactor assumes the mic is wired to:
 *       BCLK = GPIO14, LRCLK = GPIO15, DOUT = GPIO34, L/R pin => GND => "Left" channel.
 */

#include <Arduino.h>
#include <SPIFFS.h>
#include "BluetoothA2DPSource.h"
#include <arduinoFFT.h>
#include "esp_task_wdt.h"   // Watchdog Timer Reset
#include <driver/i2s.h>     // I2S driver for ESP32

// ---------------------------------------------------------------------------
// Configuration Constants
// ---------------------------------------------------------------------------

// I2S Mic Pin Definitions (INMP441 => 3.3V, GND, BCLK=14, LRCLK=15, DOUT=34)
#define I2S_SCK  14 // BCLK
#define I2S_WS   15 // LRCLK
#define I2S_SD   34 // DOUT
#define I2S_PORT I2S_NUM_0

// I2S Config: 16 kHz, 32-bit frames, "Only Left" channel
static const int   I2S_SAMPLE_RATE      = 16000;
static const bool  USE_APLL             = false;   
static const bool  TX_DESC_AUTO_CLEAR   = false;
static const int   FIXED_MCLK           = 0;
static const int   I2S_DMA_BUF_COUNT    = 4;
static const int   I2S_DMA_BUF_LEN      = 128;

// FFT & Beep Detection
#define SAMPLES            256    // Must be a power of 2
#define SAMPLING_FREQUENCY 16000  // matches I2S sample rate
#define FREQUENCY_MIN      2000
#define FREQUENCY_MAX      8000
static const float BIN_AMPLITUDE_THRESHOLD = 20;   // used to check "enough bins" in range
static const float AMPLITUDE_THRESHOLD      = 300; // total amplitude in frequency band
static const int   MULTI_WINDOW_CONFIDENCE  = 3;   // beep must appear in N consecutive windows
static const int   DETECTION_DELAY         = 1000; // min ms between beep detections
static const int   LED_FLASH_DURATION      = 500;  
static const int   I2S_READ_CHUNK         = 64;    // # of frames to read per loop iteration

// General Pins
#define LED_PIN          2
#define BUTTON_PIN       25

// FFT Arrays
float vReal[SAMPLES];
float vImag[SAMPLES];

unsigned long lastDetectionTime = 0;
int confirmedWindows            = 0;
bool playingAudio               = false;

// Create the FFT object
ArduinoFFT<float> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

// ---------------------------------------------------------------------------
// Bluetooth and Audio File Configuration
// ---------------------------------------------------------------------------

BluetoothA2DPSource a2dp_source;
const char* bluetoothSpeakerName = "C17A";
const char* audioFilePath        = "/audio1.wav";
File audioFile;  // The pre-downloaded WAV file

// The callback that streams data from `audioFile` to the speaker
int32_t audioDataCallback(uint8_t* data, int32_t len);

// ---------------------------------------------------------------------------
// Function Declarations
// ---------------------------------------------------------------------------

void initPins();
void initWatchdogTimer();
void initSPIFFS();
void initI2S();
void initBluetooth();

void startAudioPlayback();
void handlePlaybackCompletion();
void flashLED(int pin, int duration);

void collectAudioSamples();
void runFFT();
bool detectBeep();
void handleBeepDetection();

// ---------------------------------------------------------------------------
// Setup and Loop
// ---------------------------------------------------------------------------

void setup() {
    Serial.begin(115200);
    Serial.println("\n\nInitializing system...");

    initPins();
    initWatchdogTimer();
    initSPIFFS();
    initI2S();
    initBluetooth();

    Serial.println("System initialized. Listening for appliance beeps...");
}

void loop() {
    // Button: if pressed, try immediate playback
    if (digitalRead(BUTTON_PIN) == LOW) {
        Serial.println("Button pressed -> Immediate playback.");
        flashLED(LED_PIN, LED_FLASH_DURATION);
        startAudioPlayback();
    }

    // Check if any ongoing playback is done
    handlePlaybackCompletion();

    // Collect samples, run FFT, detect beep
    collectAudioSamples();
    runFFT();
    handleBeepDetection();
}

// ---------------------------------------------------------------------------
// I2S Data -> vReal[] for FFT
//  - We'll read 256 samples, 32 bits each, downshift to 16-bit, store in vReal
// ---------------------------------------------------------------------------
void collectAudioSamples() {
    // We'll gather exactly SAMPLES frames for the FFT each loop
    // Each frame is 32 bits from the INMP441. We only care about top 16 bits.

    int count = 0;
    while (count < SAMPLES) {
        // We read data in smaller chunks to ensure we don't block too long.
        // Each chunk = I2S_READ_CHUNK frames (4 bytes each) => I2S_READ_CHUNK*4 bytes
        uint8_t i2sBuffer[I2S_READ_CHUNK * 4];
        size_t bytesRead = 0;
        // Attempt to read up to I2S_READ_CHUNK frames
        i2s_read(I2S_PORT, i2sBuffer, sizeof(i2sBuffer), &bytesRead, portMAX_DELAY);

        // Number of 32-bit frames we actually got
        int frames = bytesRead / 4;
        for (int i = 0; i < frames && count < SAMPLES; i++) {
            // Each 32-bit sample layout:
            //   Byte0 => usually 0x00 or 0xE0
            //   Byte1 => LSB (often noise)
            //   Byte2 => Mid
            //   Byte3 => MSB
            uint8_t mid = i2sBuffer[i*4 + 2];
            uint8_t msb = i2sBuffer[i*4 + 3];

            // Combine => 16-bit
            int16_t sample16 = (int16_t)((msb << 8) | mid);

            // For the FFT, store as float
            vReal[count] = (float)sample16;
            vImag[count] = 0.0f;
            count++;

            // Feed the watchdog
            esp_task_wdt_reset();
        }
    }
}

// ---------------------------------------------------------------------------
// Run the FFT
// ---------------------------------------------------------------------------
void runFFT() {
    // Apply a window, then compute forward FFT, then convert to magnitude
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();
}

// ---------------------------------------------------------------------------
// Attempt beep detection by checking the FFT results
// ---------------------------------------------------------------------------
bool detectBeep() {
    float majorFrequency = FFT.majorPeak();
    float amplitude      = 0.0f;
    bool sufficientBins  = false;

    // Sum amplitude in the band [FREQUENCY_MIN, FREQUENCY_MAX]
    for (int i = 0; i < SAMPLES/2; i++) {
        float freq = (i * (float)SAMPLING_FREQUENCY) / (float)SAMPLES;
        if (freq >= FREQUENCY_MIN && freq <= FREQUENCY_MAX) {
            amplitude += vReal[i];
            if (vReal[i] > BIN_AMPLITUDE_THRESHOLD) {
                sufficientBins = true;
            }
        }
        // Watchdog
        esp_task_wdt_reset();
    }

    bool beepDetected = (majorFrequency >= FREQUENCY_MIN &&
                         majorFrequency <= FREQUENCY_MAX &&
                         amplitude > AMPLITUDE_THRESHOLD &&
                         sufficientBins);

    if (beepDetected) {
        Serial.println("Potential beep detected (pre-confirmation)...");
    }

    return beepDetected;
}

// ---------------------------------------------------------------------------
// Update beep-detection logic, and trigger playback if beep confirmed
// ---------------------------------------------------------------------------
void handleBeepDetection() {
    if (detectBeep()) {
        confirmedWindows++;
        Serial.print("Beep confirmation window: ");
        Serial.println(confirmedWindows);
    } else {
        confirmedWindows = 0;
    }

    if (confirmedWindows >= MULTI_WINDOW_CONFIDENCE) {
        unsigned long currentTime = millis();
        if ((currentTime - lastDetectionTime) >= DETECTION_DELAY) {
            Serial.println("Beep detected and confirmed! Starting playback...");
            confirmedWindows = 0;
            lastDetectionTime = currentTime;
            flashLED(LED_PIN, LED_FLASH_DURATION);
            startAudioPlayback();
        }
    }
}

// ---------------------------------------------------------------------------
// Bluetooth Callback: Streams data from audioFile to the speaker
// ---------------------------------------------------------------------------
int32_t audioDataCallback(uint8_t* data, int32_t len) {
    if (!audioFile || !audioFile.available()) {
        return 0; // No more data left
    }
    size_t bytesRead = audioFile.read(data, len);
    return static_cast<int32_t>(bytesRead);
}

// ---------------------------------------------------------------------------
// Start / End WAV Playback
// ---------------------------------------------------------------------------
void startAudioPlayback() {
    if (!a2dp_source.is_connected()) {
        Serial.println("No Bluetooth connection. Skipping audio playback.");
        return;
    }
    if (audioFile) {
        audioFile.close(); // close if left open
    }
    audioFile = SPIFFS.open(audioFilePath, "r");
    if (!audioFile || !audioFile.available()) {
        Serial.println("Audio file not available for playback.");
        return;
    }

    // Rewind to start
    bool seekSuccess = audioFile.seek(0, SeekSet);
    if (!seekSuccess) {
        Serial.println("Failed to seek audio file to start.");
        return;
    }

    Serial.println("Starting audio playback...");
    playingAudio = true;
}

void handlePlaybackCompletion() {
    if (playingAudio && (!audioFile || !audioFile.available())) {
        Serial.println("Audio playback complete.");
        playingAudio = false;
        if (audioFile) {
            audioFile.close();
            Serial.println("Audio file closed after playback.");
        }
    }
}

// ---------------------------------------------------------------------------
// LED Helper
// ---------------------------------------------------------------------------
void flashLED(int pin, int duration) {
    Serial.println("Flashing LED...");
    digitalWrite(pin, HIGH);
    delay(duration);
    digitalWrite(pin, LOW);
}

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------
void initPins() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    digitalWrite(LED_PIN, LOW);
    Serial.println("Pins initialized.");
}

void initWatchdogTimer() {
    esp_task_wdt_init(10, true);
    esp_task_wdt_add(NULL);
    Serial.println("Watchdog timer initialized.");
}

void initSPIFFS() {
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS mount failed!");
        while (1) { delay(100); }
    }

    Serial.println("Listing SPIFFS files:");
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    while (file) {
        Serial.print("File: ");
        Serial.println(file.name());
        file = root.openNextFile();
    }
    Serial.println("SPIFFS ready.");
}

void initI2S() {
    Serial.println("[initI2S] Installing I2S driver...");

    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        // For INMP441 with L/R => GND => typically "left" channel; 
        // but some boards actually wire data on "right." Adjust if needed.
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
    if (err != ESP_OK) {
        Serial.printf("[initI2S] i2s_driver_install failed: %d\n", err);
        while(true){ delay(100); }
    }
    Serial.println("[initI2S] i2s_driver_install OK.");

    i2s_pin_config_t pin_config = {
        .bck_io_num   = I2S_SCK,
        .ws_io_num    = I2S_WS,
        .data_out_num = -1,
        .data_in_num  = I2S_SD
    };
    err = i2s_set_pin(I2S_PORT, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("[initI2S] i2s_set_pin failed: %d\n", err);
        while(true){ delay(100); }
    }
    Serial.println("[initI2S] i2s_set_pin OK.");

    err = i2s_set_clk(I2S_PORT, I2S_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);
    if (err != ESP_OK) {
        Serial.printf("[initI2S] i2s_set_clk failed: %d\n", err);
        while(true){ delay(100); }
    }
    Serial.println("[initI2S] i2s_set_clk OK.");

    err = i2s_start(I2S_PORT);
    if (err != ESP_OK) {
        Serial.printf("[initI2S] i2s_start failed: %d\n", err);
        while(true){ delay(100); }
    }
    Serial.println("[initI2S] i2s_start OK.");

    // Optional: discard an initial buffer
    size_t dummyRead;
    char dummy[256];
    i2s_read(I2S_PORT, dummy, 256, &dummyRead, 100 / portTICK_RATE_MS);

    Serial.println("[initI2S] I2S init done.");
}

void initBluetooth() {
    Serial.println("Initializing Bluetooth A2DP source...");
    a2dp_source.set_auto_reconnect(true);

    // Optional: log connection states
    a2dp_source.set_on_connection_state_changed([](esp_a2d_connection_state_t state, void*){
        Serial.printf("[Bluetooth] Connection => %s\n", a2dp_source.to_str(state));
    });

    // Start in raw callback mode
    a2dp_source.start_raw(bluetoothSpeakerName, audioDataCallback);
    a2dp_source.set_volume(20);

    Serial.printf("BT name='%s', started. Connect to your speaker.\n", bluetoothSpeakerName);
}