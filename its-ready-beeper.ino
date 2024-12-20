#include <Arduino.h>
#include <SPIFFS.h>
#include "BluetoothA2DPSource.h"  // ESP32-A2DP Library
#include <arduinoFFT.h>
#include "esp_task_wdt.h"         // Watchdog Timer Reset

// ---------------------------------------------------------------------------
// Configuration Constants
// ---------------------------------------------------------------------------
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
const int MULTI_WINDOW_CONFIDENCE = 5;      // # of consecutive FFT windows needed for confirmation
const int DETECTION_DELAY = 1000;           // ms between confirmed detections
const int LED_FLASH_DURATION = 500;         // LED flash duration in ms

// ---------------------------------------------------------------------------
// Global Variables
// ---------------------------------------------------------------------------
float vReal[SAMPLES];    // Real part of the FFT input
float vImag[SAMPLES];    // Imaginary part of the FFT input
unsigned long lastDetectionTime = 0;
int confirmedWindows = 0;
bool playingAudio = false;

ArduinoFFT<float> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

// ---------------------------------------------------------------------------
// Bluetooth-Related Code
// ---------------------------------------------------------------------------
BluetoothA2DPSource a2dp_source;
const char* bluetoothSpeakerName = "C17A";

/**
 * @brief Callback function for Bluetooth A2DP audio data request.
 * 
 * Reads data from the audio file stored in SPIFFS and passes it to the
 * Bluetooth A2DP stack. The requested number of bytes (len) may be partially
 * or fully fulfilled depending on file availability.
 * 
 * @param data Pointer to the buffer to store audio data.
 * @param len  Maximum number of bytes requested.
 * @return int32_t Actual number of bytes read from the file.
 */
int32_t audioDataCallback(uint8_t* data, int32_t len);

// ---------------------------------------------------------------------------
// Audio-Related Code
// ---------------------------------------------------------------------------
const char* audioFilePath = "/audio1.wav";
File audioFile;

/**
 * @brief Starts audio playback over the Bluetooth A2DP connection.
 * 
 * Ensures a connected Bluetooth speaker. Opens the audio file from SPIFFS,
 * seeks to the start, and sets the flag indicating that playback is ongoing.
 * Actual data transfer is handled by the audioDataCallback function.
 */
void startAudioPlayback();

/**
 * @brief Monitors the completion of audio playback.
 * 
 * Checks if the audio file has no more data. If completed, logs a message,
 * resets the playingAudio flag, and closes the file. This should be called
 * regularly (e.g., in loop()) to detect when playback ends.
 */
void handlePlaybackCompletion();

/**
 * @brief Flashes the LED for a specified duration.
 * 
 * Turns the LED on for a given duration and then turns it off.
 * Used to provide a visual indicator of certain events such as button presses
 * and beep detections.
 * 
 * @param pin      The pin number of the LED.
 * @param duration Duration in milliseconds for which the LED should remain lit.
 */
void flashLED(int pin, int duration);

// ---------------------------------------------------------------------------
// Beep-Logic Related Code
// ---------------------------------------------------------------------------

/**
 * @brief Detects if a beep sound is present in the audio input.
 * 
 * Uses FFT results to check if there's a dominant frequency peak within
 * the defined beep frequency range. Also considers overall amplitude thresholds
 * and bin-level thresholds to avoid false positives.
 * 
 * @return true if a beep is detected, false otherwise.
 */
bool detectBeep();

/**
 * @brief Handles the logic of confirming a beep after multiple FFT windows.
 * 
 * Increments a confirmation counter each time detectBeep() returns true.
 * Once the counter reaches MULTI_WINDOW_CONFIDENCE and enough time has
 * passed since the last detection, it confirms a beep event, flashes the LED,
 * and triggers audio playback.
 */
void handleBeepDetection();

/**
 * @brief Collects raw audio samples from the microphone for FFT processing.
 * 
 * Samples the microphone input (MIC_PIN) at the defined sampling frequency.
 * The result is stored in vReal and vImag for subsequent FFT computation.
 * This function also periodically resets the watchdog timer.
 */
void collectAudioSamples();

/**
 * @brief Runs the FFT on the collected samples to compute their frequency spectrum.
 * 
 * Applies a Hamming window to the input data, performs a forward FFT, and
 * computes magnitudes of the frequency bins. The result is used by detectBeep()
 * to identify the presence of a beep tone.
 */
void runFFT();

// ---------------------------------------------------------------------------
// Support Functions for Setup
// ---------------------------------------------------------------------------

/**
 * @brief Initializes GPIO pins for microphone, LED, and button.
 */
void initPins() {
    pinMode(MIC_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    digitalWrite(LED_PIN, LOW);
    Serial.println("Pins initialized.");
}

/**
 * @brief Initializes the watchdog timer to prevent system hangs.
 */
void initWatchdogTimer() {
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 10000,
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic = true,
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);
    Serial.println("Watchdog timer initialized.");
}

/**
 * @brief Initializes the SPIFFS file system and lists available files.
 */
void initSPIFFS() {
    if (!SPIFFS.begin(true)) {
        Serial.println("Error: SPIFFS failed to mount!");
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
}

/**
 * @brief Initializes the Bluetooth A2DP source with a callback for audio data.
 */
void initBluetooth() {
    a2dp_source.start_raw(bluetoothSpeakerName, audioDataCallback);
    Serial.println("Bluetooth A2DP source started. Ready to connect to a speaker.");
}

/**
 * @brief Calibrates baseline noise level and adjusts AMPLITUDE_THRESHOLD if needed.
 * 
 * Averages 100 samples from the microphone to determine a baseline noise floor.
 * If baseline exceeds SILENCE_THRESHOLD, AMPLITUDE_THRESHOLD is increased accordingly.
 */
void calibrateBaselineNoise() {
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
        AMPLITUDE_THRESHOLD += (baseline - SILENCE_THRESHOLD);
    }
    Serial.print("Amplitude threshold set to: ");
    Serial.println(AMPLITUDE_THRESHOLD);
}

// ---------------------------------------------------------------------------
// Setup and Loop
// ---------------------------------------------------------------------------

/**
 * @brief Arduino setup function. Initializes all subsystems and prepares for operation.
 */
void setup() {
    Serial.begin(115200);
    Serial.println("Initializing system...");
    initPins();
    initWatchdogTimer();
    initSPIFFS();
    initBluetooth();
    calibrateBaselineNoise();
    Serial.println("System initialized. Listening for appliance beeps...");
}

/**
 * @brief Arduino loop function. Executes repeatedly.
 * 
 * Checks button presses, handles playback completion, collects samples, runs FFT,
 * and checks for beeps. If a beep is confirmed, triggers actions like LED flash and audio playback.
 */
void loop() {
    // Check for button press
    if (digitalRead(BUTTON_PIN) == LOW) {
        Serial.println("Button pressed. Flashing LED and attempting immediate audio playback.");
        flashLED(LED_PIN, LED_FLASH_DURATION);
        startAudioPlayback();
    }

    handlePlaybackCompletion();
    collectAudioSamples();
    runFFT();
    handleBeepDetection();
}

// ---------------------------------------------------------------------------
// Bluetooth-Related Code Implementations
// ---------------------------------------------------------------------------
int32_t audioDataCallback(uint8_t* data, int32_t len) {
    if (!audioFile || !audioFile.available()) {
        return 0; // No more data available
    }
    size_t bytesRead = audioFile.read(data, len);
    return static_cast<int32_t>(bytesRead);
}

// ---------------------------------------------------------------------------
// Audio-Related Code Implementations
// ---------------------------------------------------------------------------
void startAudioPlayback() {
    if (!a2dp_source.is_connected()) {
        Serial.println("No Bluetooth connection. Skipping audio playback.");
        return;
    }

    if (audioFile) {
        audioFile.close();
    }
    audioFile = SPIFFS.open(audioFilePath, "r");
    if (!audioFile || !audioFile.available()) {
        Serial.println("Audio file not available for playback.");
        return;
    }

    bool seekSuccess = audioFile.seek(0, SeekSet);
    if (!seekSuccess) {
        Serial.println("Failed to seek audio file to start.");
        return;
    }

    Serial.println("Starting audio playback...");
    playingAudio = true;
    // Actual data transfer handled in audioDataCallback.
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

void flashLED(int pin, int duration) {
    Serial.println("Flashing LED...");
    digitalWrite(pin, HIGH);
    delay(duration);
    digitalWrite(pin, LOW);
}

// ---------------------------------------------------------------------------
// Beep-Logic Related Code Implementations
// ---------------------------------------------------------------------------
bool detectBeep() {
    float majorFrequency = FFT.majorPeak();
    float amplitude = 0.0;
    bool sufficientBins = false;

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
        Serial.print("Beep confirmation window count: ");
        Serial.println(confirmedWindows);
    } else {
        confirmedWindows = 0;
    }

    if (confirmedWindows >= MULTI_WINDOW_CONFIDENCE) {
        unsigned long currentTime = millis();
        if (currentTime - lastDetectionTime >= DETECTION_DELAY) {
            Serial.println("Beep detected and confirmed!");
            confirmedWindows = 0;
            lastDetectionTime = currentTime;
            flashLED(LED_PIN, LED_FLASH_DURATION);
            startAudioPlayback();
        }
    }
}

void collectAudioSamples() {
    // Collect raw samples for FFT analysis
    for (int i = 0; i < SAMPLES; i++) {
        vReal[i] = analogRead(MIC_PIN) - 2048;  // Center around zero
        vImag[i] = 0;
        delayMicroseconds(1000000 / SAMPLING_FREQUENCY);
        esp_task_wdt_reset();
    }
    // Provide minimal logging to confirm samples collected
    // (If desired, this can be commented out to reduce log noise)
    // Serial.println("Audio samples collected.");
}

void runFFT() {
    // Compute FFT of the collected samples
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();
    // Serial.println("FFT computed.");  // Commented out to avoid excessive logs
}
