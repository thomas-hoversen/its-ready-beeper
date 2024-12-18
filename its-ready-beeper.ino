#include <Arduino.h>
#include <SPIFFS.h>
#include "BluetoothA2DPSource.h"  // ESP32-A2DP Library
#include <arduinoFFT.h>
#include "esp_task_wdt.h"  // Watchdog Timer Reset

// Configuration
#define MIC_PIN 35                 // Analog pin connected to MAX9814 output
#define LED_PIN 2                  // Onboard LED pin
#define SAMPLES 64                 // Number of samples for FFT (must be power of 2)
#define SAMPLING_FREQUENCY 16000   // Sampling rate in Hz
#define FREQUENCY_MIN 2500         // Minimum frequency of the beep (in Hz)
#define FREQUENCY_MAX 3500         // Maximum frequency of the beep (in Hz)

int SILENCE_THRESHOLD = 100;       // Noise floor level in a quiet room
float AMPLITUDE_THRESHOLD = 500;   // Minimum amplitude for a loud beep
float BIN_AMPLITUDE_THRESHOLD = 50; // Minimum amplitude per frequency bin
#define MULTI_WINDOW_CONFIDENCE 5  // Number of consecutive windows for confirmation
#define DETECTION_DELAY 1500       // Minimum time between detections (in ms)
#define LED_FLASH_DURATION 500     // LED flash duration in milliseconds

// Buffers
float vReal[SAMPLES];
float vImag[SAMPLES];
unsigned long lastDetectionTime = 0;
unsigned long ledFlashStartTime = 0;
int confirmedWindows = 0;
int beepCount = 0;

// Bluetooth and Audio
BluetoothA2DPSource a2dp_source;
File audioFile;
const char* bluetoothSpeakerName = "C17A";
const char* audioFilePath = "/audio1.wav";

// FFT Object
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

// Audio Data Callback Function
int32_t audioDataCallback(uint8_t* data, int32_t len) {
    if (!audioFile || !audioFile.available()) {
        return 0; // No data available
    }
    size_t bytesRead = audioFile.read(data, len);
    return static_cast<int32_t>(bytesRead);
}

void setup() {
    Serial.begin(115200);
    pinMode(MIC_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Initialize SPIFFS
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS failed to mount!");
        while (1);
    }

    Serial.println("Checking files in SPIFFS...");
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    while (file) {
        Serial.print("File: ");
        Serial.println(file.name());
        file = root.openNextFile();
    }
    Serial.println("SPIFFS initialized.");

    // Open audio file
    audioFile = SPIFFS.open(audioFilePath, "r");
    if (!audioFile) {
        Serial.println("Failed to open audio file!");
    }

    // Start Bluetooth A2DP with raw data callback
    a2dp_source.start_raw(bluetoothSpeakerName, audioDataCallback);
    Serial.println("Bluetooth A2DP source started. Ready to connect to a speaker.");

    Serial.println("Calibrating baseline noise level...");
    calibrateBaseline();
    Serial.println("Listening for appliance beeps...");

    // Initialize Watchdog Timer with a 10-second timeout
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 10000,
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic = true,
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);  // Add current thread to WDT watch
}

void loop() {
    // Collect audio samples
    for (uint16_t i = 0; i < SAMPLES; i++) {
        vReal[i] = analogRead(MIC_PIN) - 2048; // Center the signal
        vImag[i] = 0;
        delayMicroseconds(1000000 / SAMPLING_FREQUENCY);
        esp_task_wdt_reset();  // Reset WDT to prevent timeout
    }

    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();

    if (detectBeep()) {
        confirmedWindows++;
    } else {
        confirmedWindows = 0;
    }

    if (confirmedWindows >= MULTI_WINDOW_CONFIDENCE) {
        unsigned long currentTime = millis();
        if (currentTime - lastDetectionTime >= DETECTION_DELAY) {
            beepCount++;
            Serial.print("IT'S READY! Total beeps: ");
            Serial.println(beepCount);

            // Flash LED
            digitalWrite(LED_PIN, HIGH);
            ledFlashStartTime = currentTime;

            // Play audio file
            playAudioFile();

            lastDetectionTime = currentTime;
            confirmedWindows = 0;
        }
    }

    // Turn off LED after flash duration
    if (ledFlashStartTime > 0 && millis() - ledFlashStartTime >= LED_FLASH_DURATION) {
        digitalWrite(LED_PIN, LOW);
        ledFlashStartTime = 0;
    }
}

// Play audio file function
void playAudioFile() {
    Serial.println("Playing audio file...");
    if (!audioFile || !audioFile.available()) {
        Serial.println("Audio file is not available.");
        return;
    }

    // Wait for Bluetooth to connect
    while (!a2dp_source.is_connected()) {
        Serial.println("Waiting for Bluetooth connection...");
        esp_task_wdt_reset();
        delay(500);
    }

    // Restart the audio file for playback
    audioFile.seek(0);
    uint8_t* buffer = (uint8_t*)malloc(512);  // Allocate buffer for audio
    while (audioFile.available() && a2dp_source.is_connected()) {
        size_t bytesRead = audioFile.read(buffer, 512);
        esp_task_wdt_reset();
        delay(1);  // Allow time for audio to stream
    }
    free(buffer);  // Free buffer memory
    Serial.println("Audio playback complete.");
}

// Calibrate baseline noise
void calibrateBaseline() {
    long sum = 0;
    for (int i = 0; i < 100; i++) {
        sum += analogRead(MIC_PIN);
        delay(10);
        esp_task_wdt_reset();
    }
    int baseline = sum / 100;
    Serial.print("Baseline noise level: ");
    Serial.println(baseline);

    if (baseline > SILENCE_THRESHOLD) {
        AMPLITUDE_THRESHOLD += baseline - SILENCE_THRESHOLD;
    }
}

// Beep detection function
bool detectBeep() {
    float majorFrequency = FFT.majorPeak();
    float amplitude = 0.0;
    bool sufficientBins = false;

    for (uint16_t i = 0; i < SAMPLES / 2; i++) {
        float frequency = (i * SAMPLING_FREQUENCY) / SAMPLES;
        if (frequency >= FREQUENCY_MIN && frequency <= FREQUENCY_MAX) {
            amplitude += vReal[i];
            if (vReal[i] > BIN_AMPLITUDE_THRESHOLD) {
                sufficientBins = true;
            }
        }
        esp_task_wdt_reset();
    }

    return (majorFrequency >= FREQUENCY_MIN && majorFrequency <= FREQUENCY_MAX &&
            amplitude > AMPLITUDE_THRESHOLD && sufficientBins);
}
