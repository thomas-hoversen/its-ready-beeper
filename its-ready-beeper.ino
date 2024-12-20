#include <Arduino.h>
#include <SPIFFS.h>
#include "BluetoothA2DPSource.h"  // ESP32-A2DP Library
#include <arduinoFFT.h>
#include "esp_task_wdt.h"  // Watchdog Timer Reset

// Configuration Constants
#define MIC_PIN 35                 // Analog pin connected to MAX9814 output
#define LED_PIN 2                  // Onboard LED pin
#define BUTTON_PIN 15              // Button connected to GND and P15
#define SAMPLES 64                 // Number of samples for FFT (must be power of 2)
#define SAMPLING_FREQUENCY 16000   // Sampling rate in Hz
#define FREQUENCY_MIN 2500         // Minimum frequency of the beep (in Hz)
#define FREQUENCY_MAX 3500         // Maximum frequency of the beep (in Hz)

const int SILENCE_THRESHOLD = 100;          // Noise floor level in a quiet room
float AMPLITUDE_THRESHOLD = 500;            // Minimum amplitude for a loud beep
const float BIN_AMPLITUDE_THRESHOLD = 50;   // Minimum amplitude per frequency bin
const int MULTI_WINDOW_CONFIDENCE = 5;      // Consecutive windows for confirmation
const int DETECTION_DELAY = 1000;           // Time between detections (in ms)
const int LED_FLASH_DURATION = 500;         // LED flash duration in milliseconds

// Global Variables
float vReal[SAMPLES];
float vImag[SAMPLES];
unsigned long lastDetectionTime = 0;
int confirmedWindows = 0;
bool playingAudio = false;

BluetoothA2DPSource a2dp_source;
const char* bluetoothSpeakerName = "C17A";
const char* audioFilePath = "/audio1.wav";
File audioFile;

ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

// Forward Declarations
bool detectBeep();
void flashLED(int pin, int duration);
void startAudioPlayback();

// Audio Data Callback Function
int32_t audioDataCallback(uint8_t* data, int32_t len) {
    if (!audioFile || !audioFile.available()) {
        // No more data to read
        return 0;
    }
    size_t bytesRead = audioFile.read(data, len);
    return static_cast<int32_t>(bytesRead);
}

void setup() {
    Serial.begin(115200);
    pinMode(MIC_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP); // Button uses internal pull-up
    digitalWrite(LED_PIN, LOW);

    // Initialize Watchdog Timer
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 10000,
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic = true,
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);

    // Initialize SPIFFS
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS failed to mount!");
        while (1);
    }

    // List files in SPIFFS
    Serial.println("Checking files in SPIFFS...");
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    while (file) {
        Serial.print("File: ");
        Serial.println(file.name());
        file = root.openNextFile();
    }

    // Start Bluetooth A2DP with raw callback
    a2dp_source.start_raw(bluetoothSpeakerName, audioDataCallback);
    Serial.println("Bluetooth A2DP source started. Ready to connect to a speaker.");

    // Calibrate baseline noise
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

    Serial.println("Listening for appliance beeps...");
}

void loop() {
    // Check for button press
    if (digitalRead(BUTTON_PIN) == LOW) {
        // Button pressed, flash LED
        flashLED(LED_PIN, LED_FLASH_DURATION);
    }

    // If we were playing audio and now it's done, print completion message and reset
    if (playingAudio && (!audioFile || !audioFile.available())) {
        Serial.println("Audio playback complete.");
        playingAudio = false;
        // Close the file after playback is done
        if (audioFile) {
            audioFile.close();
        }
    }

    // Collect audio samples for beep detection
    for (int i = 0; i < SAMPLES; i++) {
        vReal[i] = analogRead(MIC_PIN) - 2048;
        vImag[i] = 0;
        delayMicroseconds(1000000 / SAMPLING_FREQUENCY);
        esp_task_wdt_reset();
    }

    // Perform FFT
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();

    // Check for beep detection
    if (detectBeep()) {
        confirmedWindows++;
    } else {
        confirmedWindows = 0;
    }

    if (confirmedWindows >= MULTI_WINDOW_CONFIDENCE) {
        unsigned long currentTime = millis();
        if (currentTime - lastDetectionTime >= DETECTION_DELAY) {
            Serial.println("Beep detected!");

            // Reset windows
            confirmedWindows = 0;
            lastDetectionTime = currentTime;

            // Flash LED immediately for beep detection
            flashLED(LED_PIN, LED_FLASH_DURATION);

            // Attempt immediate audio playback
            startAudioPlayback();
        }
    }
}

void startAudioPlayback() {
    if (!a2dp_source.is_connected()) {
        Serial.println("No Bluetooth connection. Skipping audio playback.");
        return;
    }

    // Always re-open the audio file fresh before playback
    if (audioFile) {
        audioFile.close();
    }
    audioFile = SPIFFS.open(audioFilePath, "r");
    if (!audioFile || !audioFile.available()) {
        Serial.println("Audio file is not available.");
        return;
    }

    // Restart from the beginning of the file
    bool seekSuccess = audioFile.seek(0, SeekSet);
    if (!seekSuccess) {
        Serial.println("Failed to seek audio file to start.");
        return;
    }

    Serial.println("Playing audio file...");
    playingAudio = true;
    // The actual data transfer happens in the callback.
}

// Flash LED to indicate detection
void flashLED(int pin, int duration) {
    digitalWrite(pin, HIGH);
    delay(duration);
    digitalWrite(pin, LOW);
}

// Detect beep using majorPeak and amplitude thresholds
bool detectBeep() {
    float majorFrequency = FFT.majorPeak();
    float amplitude = 0.0;
    bool sufficientBins = false;

    // Sum amplitudes and check if bins exceed threshold
    for (int i = 0; i < SAMPLES / 2; i++) {
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
