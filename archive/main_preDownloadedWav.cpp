/**
 * @file main_preDownloadedWav.cpp
 * @brief This program listens for a beep from the MAX9814 microphone,
 *        and upon beep detection (or button press), plays a WAV file
 *        from SPIFFS to a Bluetooth speaker. This code is known to work
 *        with a pre-downloaded audio file, but it does not record audio.
 * 
 *        Features:
 *        - Audio sampling from a microphone for beep detection only.
 *        - Frequency domain analysis (FFT) to detect beep presence.
 *        - Automatic playback of an existing file in SPIFFS over BT.
 *        - Button press can also trigger immediate playback.
 *        - Watchdog timer + LED indicator.
 */

#include <Arduino.h>
#include <SPIFFS.h>
#include "BluetoothA2DPSource.h"
#include <arduinoFFT.h>
#include "esp_task_wdt.h"  // Watchdog Timer Reset

// ---------------------------------------------------------------------------
// Configuration Constants
// ---------------------------------------------------------------------------

#define MIC_PIN 35  
#define LED_PIN 2
#define BUTTON_PIN 15
#define SAMPLES 256
#define SAMPLING_FREQUENCY 16000
#define FREQUENCY_MIN 2000
#define FREQUENCY_MAX 8000
const int SILENCE_THRESHOLD = 100;
float AMPLITUDE_THRESHOLD = 300;
const float BIN_AMPLITUDE_THRESHOLD = 20;
const int MULTI_WINDOW_CONFIDENCE = 3;
const int DETECTION_DELAY = 1000;
const int LED_FLASH_DURATION = 500;

// ---------------------------------------------------------------------------
// Global Variables
// ---------------------------------------------------------------------------

float vReal[SAMPLES];    
float vImag[SAMPLES];    

unsigned long lastDetectionTime = 0;
int confirmedWindows = 0;
bool playingAudio = false;

// FFT
ArduinoFFT<float> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

// ---------------------------------------------------------------------------
// Bluetooth and Audio File Configuration
// ---------------------------------------------------------------------------

BluetoothA2DPSource a2dp_source;
const char* bluetoothSpeakerName = "C17A";
const char* audioFilePath = "/audio1.wav";
File audioFile;  // The pre-downloaded WAV file

// The callback that streams data from `audioFile` to the speaker
int32_t audioDataCallback(uint8_t* data, int32_t len);

// ---------------------------------------------------------------------------
// Function Declarations
// ---------------------------------------------------------------------------

void initPins();
void initWatchdogTimer();
void initSPIFFS();
void initBluetooth();
void calibrateBaselineNoise();
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
    initBluetooth();
    calibrateBaselineNoise();
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
// Callback that feeds the A2DP sink with data from audioFile
// ---------------------------------------------------------------------------
int32_t audioDataCallback(uint8_t* data, int32_t len) {
    if (!audioFile || !audioFile.available()) {
        return 0; // No more data left
    }
    size_t bytesRead = audioFile.read(data, len);
    return static_cast<int32_t>(bytesRead);
}

// ---------------------------------------------------------------------------
// Playback
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
// LED & Utility
// ---------------------------------------------------------------------------
void flashLED(int pin, int duration) {
    Serial.println("Flashing LED...");
    digitalWrite(pin, HIGH);
    delay(duration);
    digitalWrite(pin, LOW);
}

// ---------------------------------------------------------------------------
// Beep Detection
// ---------------------------------------------------------------------------
bool detectBeep() {
    float majorFrequency = FFT.majorPeak();
    float amplitude = 0.0f;
    bool sufficientBins = false;

    for (int i = 0; i < SAMPLES / 2; i++) {
        float frequency = (i * (float)SAMPLING_FREQUENCY) / (float)SAMPLES;
        if (frequency >= FREQUENCY_MIN && frequency <= FREQUENCY_MAX) {
            amplitude += vReal[i];
            if (vReal[i] > BIN_AMPLITUDE_THRESHOLD) {
                sufficientBins = true;
            }
        }
        esp_task_wdt_reset();
    }

    bool beepDetected = (majorFrequency >= FREQUENCY_MIN && majorFrequency <= FREQUENCY_MAX &&
                         amplitude > AMPLITUDE_THRESHOLD && sufficientBins);

    if (beepDetected) {
        Serial.println("Potential beep detected (pre-confirmation)...");
    }

    return beepDetected;
}

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
// Audio Sample Collection & FFT
// ---------------------------------------------------------------------------
void collectAudioSamples() {
    for (int i = 0; i < SAMPLES; i++) {
        float raw = (float)analogRead(MIC_PIN);
        vReal[i] = raw - 2048.0f; // recenter
        vImag[i] = 0.0f;
        delayMicroseconds((1000000UL / SAMPLING_FREQUENCY));
        esp_task_wdt_reset();
    }
}

void runFFT() {
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();
}

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------
void initPins() {
    pinMode(MIC_PIN, INPUT);
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

void initBluetooth() {
    a2dp_source.start_raw(bluetoothSpeakerName, audioDataCallback);
    Serial.println("Bluetooth A2DP source started. Connect to speaker now.");
}

void calibrateBaselineNoise() {
    long sum = 0;
    const int testSamples = 100;
    for (int i = 0; i < testSamples; i++) {
        sum += analogRead(MIC_PIN);
        delay(10);
        esp_task_wdt_reset();
    }
    int baseline = sum / testSamples;
    Serial.print("Baseline ADC level: ");
    Serial.println(baseline);
    // Adjust amplitude threshold if baseline is high
    if (baseline > SILENCE_THRESHOLD) {
        AMPLITUDE_THRESHOLD += (baseline - SILENCE_THRESHOLD);
    }
    Serial.print("AMPlitude threshold set to ");
    Serial.println(AMPLITUDE_THRESHOLD);
}