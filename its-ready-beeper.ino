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

const int SILENCE_THRESHOLD = 100;          // Expected noise floor in a quiet environment
float AMPLITUDE_THRESHOLD = 500;            // Adjusted based on baseline noise
const float BIN_AMPLITUDE_THRESHOLD = 50;   // Minimum amplitude per FFT bin in the target range
const int MULTI_WINDOW_CONFIDENCE = 5;      // Number of consecutive FFT windows needed to confirm a beep
const int DETECTION_DELAY = 1000;           // Milliseconds between confirmed detections
const int LED_FLASH_DURATION = 500;         // LED flash duration (ms) upon detection or button press

// Global Variables
float vReal[SAMPLES];    // Real part of the FFT input
float vImag[SAMPLES];    // Imaginary part of the FFT input (initially zero)
unsigned long lastDetectionTime = 0;  
int confirmedWindows = 0;  
bool playingAudio = false;  

BluetoothA2DPSource a2dp_source;
const char* bluetoothSpeakerName = "C17A";
const char* audioFilePath = "/audio1.wav";
File audioFile;

// FFT object initialization
ArduinoFFT<float> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

// Forward Declarations
bool detectBeep();
void flashLED(int pin, int duration);
void startAudioPlayback();
void handleButtonPress();
void handlePlaybackCompletion();
void collectAudioSamples();
void runFFT();
void handleBeepDetection();

// Audio Data Callback Function
// Called by the Bluetooth A2DP library to fetch audio data to send.
// Returns the number of bytes read from the file.
int32_t audioDataCallback(uint8_t* data, int32_t len) {
    if (!audioFile || !audioFile.available()) {
        return 0; // No more data available
    }
    size_t bytesRead = audioFile.read(data, len);
    return static_cast<int32_t>(bytesRead);
}

void setup() {
    Serial.begin(115200);

    pinMode(MIC_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP); // Use internal pull-up resistor
    digitalWrite(LED_PIN, LOW);

    // Initialize Watchdog Timer with a 10-second timeout
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 10000,
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic = true,
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);

    // Initialize SPIFFS file system
    if (!SPIFFS.begin(true)) {
        Serial.println("Error: SPIFFS failed to mount!");
        while (1);
    }

    // List files present in SPIFFS for reference
    Serial.println("Checking files in SPIFFS...");
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    while (file) {
        Serial.print("File: ");
        Serial.println(file.name());
        file = root.openNextFile();
    }

    // Start Bluetooth A2DP in raw mode
    a2dp_source.start_raw(bluetoothSpeakerName, audioDataCallback);
    Serial.println("Bluetooth A2DP source started. Ready to connect to a speaker.");

    // Calibrate baseline noise to adjust AMPLITUDE_THRESHOLD if needed
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
        // Adjust amplitude threshold based on a higher-than-expected baseline noise floor
        AMPLITUDE_THRESHOLD += (baseline - SILENCE_THRESHOLD);
    }

    Serial.println("System initialized. Listening for appliance beeps...");
}

void loop() {
    handleButtonPress();
    handlePlaybackCompletion();
    collectAudioSamples();
    runFFT();
    handleBeepDetection();
}


/**
 * @brief Checks if the user button is pressed and handles the event.
 * 
 * When the button is pressed, the LED is flashed and an attempt is made 
 * to start immediate audio playback.
 */
void handleButtonPress() {
    if (digitalRead(BUTTON_PIN) == LOW) {
        // Button pressed
        flashLED(LED_PIN, LED_FLASH_DURATION);
        startAudioPlayback();
    }
}


/**
 * @brief Monitors the completion of audio playback.
 * 
 * If the currently playing audio finishes, this method logs the completion,
 * resets the playing state, and closes the file.
 */
void handlePlaybackCompletion() {
    if (playingAudio && (!audioFile || !audioFile.available())) {
        Serial.println("Audio playback complete.");
        playingAudio = false;
        if (audioFile) {
            audioFile.close();
        }
    }
}


/**
 * @brief Collects raw audio samples from the microphone for FFT processing.
 * 
 * Reads SAMPLES data points at the MIC_PIN at a defined sampling frequency.
 * The imaginary part of the FFT input is set to zero.
 */
void collectAudioSamples() {
    for (int i = 0; i < SAMPLES; i++) {
        vReal[i] = analogRead(MIC_PIN) - 2048;  // Center around zero
        vImag[i] = 0;
        delayMicroseconds(1000000 / SAMPLING_FREQUENCY);
        esp_task_wdt_reset();
    }
}


/**
 * @brief Runs the FFT on the collected samples to compute their frequency spectrum.
 * 
 * Applies a Hamming window, then performs a forward FFT and converts the result 
 * into magnitudes. This prepares the data for beep detection analysis.
 */
void runFFT() {
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();
}


/**
 * @brief Checks for beep detection and manages the confidence window.
 * 
 * Increments a confidence counter when a beep is detected and resets it when not.
 * If the confidence threshold is reached, it checks the delay condition, logs the detection,
 * flashes the LED, and attempts audio playback.
 */
void handleBeepDetection() {
    if (detectBeep()) {
        confirmedWindows++;
    } else {
        confirmedWindows = 0;
    }

    if (confirmedWindows >= MULTI_WINDOW_CONFIDENCE) {
        unsigned long currentTime = millis();
        if (currentTime - lastDetectionTime >= DETECTION_DELAY) {
            Serial.println("Beep detected!");
            confirmedWindows = 0;
            lastDetectionTime = currentTime;

            flashLED(LED_PIN, LED_FLASH_DURATION);
            startAudioPlayback();
        }
    }
}


/**
 * @brief Starts audio playback over the Bluetooth A2DP connection.
 * 
 * Ensures a Bluetooth speaker is connected, then opens the audio file from SPIFFS 
 * and seeks to the start. Once configured, the actual audio data is fed to the speaker 
 * via the audioDataCallback function.
 */
void startAudioPlayback() {
    if (!a2dp_source.is_connected()) {
        Serial.println("No Bluetooth connection. Skipping audio playback.");
        return;
    }

    // Re-open the audio file before each playback attempt
    if (audioFile) {
        audioFile.close();
    }
    audioFile = SPIFFS.open(audioFilePath, "r");
    if (!audioFile || !audioFile.available()) {
        Serial.println("Audio file not available for playback.");
        return;
    }

    // Seek to the start of the file
    bool seekSuccess = audioFile.seek(0, SeekSet);
    if (!seekSuccess) {
        Serial.println("Failed to seek audio file to start.");
        return;
    }

    Serial.println("Starting audio playback...");
    playingAudio = true;
    // Actual reading and sending of data occurs in the audioDataCallback function.
}


/**
 * @brief Flashes the LED for the specified duration.
 * 
 * This is used to provide a visual indicator of certain events like button press 
 * or beep detection.
 * 
 * @param pin LED pin to control
 * @param duration Duration in milliseconds to keep LED lit
 */
void flashLED(int pin, int duration) {
    digitalWrite(pin, HIGH);
    delay(duration);
    digitalWrite(pin, LOW);
}


/**
 * @brief Analyzes the FFT result to detect if a beep is present.
 * 
 * Uses the FFT data to determine if there's a frequency peak within the target range 
 * (FREQUENCY_MIN to FREQUENCY_MAX) and if the amplitude exceeds certain thresholds.
 * 
 * @return true if a beep is detected, false otherwise
 */
bool detectBeep() {
    float majorFrequency = FFT.majorPeak();
    float amplitude = 0.0;
    bool sufficientBins = false;

    // Accumulate amplitude in the target frequency range
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

    // Check if the major peak and accumulated amplitude meet the criteria
    bool beepDetected = (majorFrequency >= FREQUENCY_MIN && majorFrequency <= FREQUENCY_MAX &&
                         amplitude > AMPLITUDE_THRESHOLD && sufficientBins);

    // Only print this log if we actually detect a beep candidate
    if (beepDetected) {
        Serial.println("Potential beep detected (pre-confirmation)...");
    }

    return beepDetected;
}
