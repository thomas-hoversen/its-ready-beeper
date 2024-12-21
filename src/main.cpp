/**
 * @file main.cpp
 * @brief This program listens for a specific beep frequency from a microphone, and upon detection,
 *        it triggers audio playback to a connected Bluetooth speaker. It uses FFT analysis to
 *        detect the presence of the beep in the audio signal.
 * 
 *        Features:
 *        - Audio sampling from a microphone using an ADC pin.
 *        - Frequency domain analysis (FFT) to detect a beep within a certain frequency range.
 *        - Confirmation of detection over multiple sample windows.
 *        - Automatic playback of an audio file to a Bluetooth speaker once beep is confirmed.
 *        - Visual indication of events using an onboard LED.
 *        - Button-controlled audio playback for testing.
 *        - Watchdog timer to ensure long running stability.
 * 
 * @author thomas hoversen
 * @date
 */

#include <Arduino.h>
#include <SPIFFS.h>
#include "BluetoothA2DPSource.h"
#include <arduinoFFT.h>
#include "esp_task_wdt.h"  // Watchdog Timer Reset

// ---------------------------------------------------------------------------
// Configuration Constants
// ---------------------------------------------------------------------------

/** 
 * @brief Analog pin connected to the MAX9814 microphone module output.
 * 
 * The MAX9814 microphone module outputs an analog voltage proportional to audio input. 
 * This pin is read by the ADC.
 */
#define MIC_PIN 35  

/** @brief LED pin used for signaling detections. Typically the onboard LED on ESP32 boards. */
#define LED_PIN 2

/** 
 * @brief Pin connected to a button. The button is expected to be pulled-up internally, 
 * and shorted to ground when pressed.
 */
#define BUTTON_PIN 15

/** 
 * @brief Number of samples for the FFT. Higher values = better frequency resolution, but requires more computation.
 */
#define SAMPLES 256

/** 
 * @brief Sampling frequency in Hz. This should match the expected frequency range of the beep.
 */
#define SAMPLING_FREQUENCY 16000

/** 
 * @brief Minimum frequency in Hz considered for beep detection.
 * Adjust based on the expected beep frequency.
 */
#define FREQUENCY_MIN 2000

/** 
 * @brief Maximum frequency in Hz considered for beep detection.
 * Adjust based on the expected beep frequency.
 */
#define FREQUENCY_MAX 8000

/** 
 * @brief Threshold for the baseline silence level. If the environment is noisier, 
 * this may need to be increased.
 */
const int SILENCE_THRESHOLD = 100;

/** 
 * @brief Initial amplitude threshold for detecting a beep. This may be adjusted at runtime 
 * based on baseline noise.
 */
float AMPLITUDE_THRESHOLD = 300;

/** 
 * @brief Minimum bin amplitude required within the target frequency range.
 * Allows a finer check on actual signal presence across multiple FFT bins.
 */
const float BIN_AMPLITUDE_THRESHOLD = 20;

/** 
 * @brief Number of consecutive detection windows required to confirm a beep detection.
 * Higher values reduce false positives but increase detection latency.
 */
const int MULTI_WINDOW_CONFIDENCE = 3;

/** 
 * @brief Minimum delay (ms) after a confirmed beep before acknowledging another.
 * This prevents back-to-back triggers from a single beep event.
 */
const int DETECTION_DELAY = 1000;

/** 
 * @brief Duration (ms) the LED stays lit during a detection event or button press.
 */
const int LED_FLASH_DURATION = 500;

// ---------------------------------------------------------------------------
// Global Variables
// ---------------------------------------------------------------------------

/** @brief Arrays to hold real and imaginary parts of the sampled audio data for FFT processing. */
float vReal[SAMPLES];    
float vImag[SAMPLES];    

/** @brief Last time (in ms) a beep was confirmed to avoid immediate retriggers. */
unsigned long lastDetectionTime = 0;

/** @brief Counter for consecutive windows in which a beep is suspected. */
int confirmedWindows = 0;

/** @brief Flag indicating whether the system is currently playing audio. */
bool playingAudio = false;

/** @brief FFT object for performing the frequency analysis. */
ArduinoFFT<float> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

// ---------------------------------------------------------------------------
// Bluetooth and Audio File Configuration
// ---------------------------------------------------------------------------

/** @brief A2DP source object for Bluetooth audio output. */
BluetoothA2DPSource a2dp_source;

/** @brief Name of the Bluetooth speaker or endpoint to connect to. */
const char* bluetoothSpeakerName = "C17A";

/** 
 * @brief Path to the audio file in SPIFFS that will be played once the beep is detected or 
 * triggered by the button.
 */
const char* audioFilePath = "/audio1.wav";

/** @brief File object for the audio file to be streamed via Bluetooth. */
File audioFile;

/** 
 * @brief Callback function that provides audio data to the A2DP sink.
 * 
 * @param data Pointer to the buffer where audio data should be written.
 * @param len  Length of the audio data buffer in bytes.
 * @return The number of bytes actually written into the buffer.
 */
int32_t audioDataCallback(uint8_t* data, int32_t len);

// ---------------------------------------------------------------------------
// Function Declarations
// ---------------------------------------------------------------------------

/**
 * @brief Initialize GPIO pins and their modes.
 */
void initPins();

/**
 * @brief Initialize the watchdog timer to ensure the system resets if it hangs.
 */
void initWatchdogTimer();

/**
 * @brief Initialize the SPIFFS filesystem and list available files.
 */
void initSPIFFS();

/**
 * @brief Initialize Bluetooth A2DP source.
 */
void initBluetooth();

/**
 * @brief Perform a baseline noise calibration to adjust amplitude thresholds dynamically.
 */
void calibrateBaselineNoise();

/**
 * @brief Start audio playback from the selected audio file over Bluetooth.
 */
void startAudioPlayback();

/**
 * @brief Handle the completion of audio playback, close the file if needed.
 */
void handlePlaybackCompletion();

/**
 * @brief Flash the onboard LED to indicate an event.
 * 
 * @param pin      LED pin number.
 * @param duration Duration in milliseconds for the LED to remain lit.
 */
void flashLED(int pin, int duration);

/**
 * @brief Gather raw audio samples from the microphone and store in vReal/vImag buffers.
 */
void collectAudioSamples();

/**
 * @brief Execute FFT on the collected samples to convert them to the frequency domain.
 */
void runFFT();

/**
 * @brief Check FFT data to determine if a beep is present.
 * 
 * @return true if a beep is detected, false otherwise.
 */
bool detectBeep();

/**
 * @brief Handle the logic of confirming a detected beep over multiple windows, and trigger actions if confirmed.
 */
void handleBeepDetection();

// ---------------------------------------------------------------------------
// Setup Support Functions Implementations
// ---------------------------------------------------------------------------

void initPins() {
    pinMode(MIC_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    digitalWrite(LED_PIN, LOW);
    Serial.println("Pins initialized.");
}

void initWatchdogTimer() {
    // Set a 10-second timeout and enable panic on WDT timeout
    esp_task_wdt_init(10, true);
    esp_task_wdt_add(NULL);
    Serial.println("Watchdog timer initialized.");
}

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

void initBluetooth() {
    a2dp_source.start_raw(bluetoothSpeakerName, audioDataCallback);
    Serial.println("Bluetooth A2DP source started. Ready to connect to a speaker.");
}

void calibrateBaselineNoise() {
    long sum = 0;
    const int samplesForBaseline = 100;
    for (int i = 0; i < samplesForBaseline; i++) {
        sum += analogRead(MIC_PIN);
        delay(10);
        esp_task_wdt_reset();
    }
    int baseline = sum / samplesForBaseline;
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
    // Check if the button is pressed
    if (digitalRead(BUTTON_PIN) == LOW) {
        Serial.println("Button pressed. Flashing LED and attempting immediate audio playback.");
        flashLED(LED_PIN, LED_FLASH_DURATION);
        startAudioPlayback();
    }

    // Handle completion of any ongoing playback
    handlePlaybackCompletion();

    // Collect audio samples and perform FFT analysis
    collectAudioSamples();
    runFFT();

    // Handle beep detection logic
    handleBeepDetection();
}

// ---------------------------------------------------------------------------
// Bluetooth-Related Code
// ---------------------------------------------------------------------------

int32_t audioDataCallback(uint8_t* data, int32_t len) {
    if (!audioFile || !audioFile.available()) {
        return 0; // No more data available
    }
    size_t bytesRead = audioFile.read(data, len);
    return static_cast<int32_t>(bytesRead);
}

// ---------------------------------------------------------------------------
// Audio-Related Code
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
// Beep-Logic Related
// ---------------------------------------------------------------------------

bool detectBeep() {
    // Retrieve the major frequency from the FFT data
    float majorFrequency = FFT.majorPeak();
    float amplitude = 0.0f;
    bool sufficientBins = false;

    // Check all frequency bins within the target range
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

    // Determine if the conditions for a beep are met
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

    // If beep is confirmed over multiple windows and detection delay has passed
    if (confirmedWindows >= MULTI_WINDOW_CONFIDENCE) {
        unsigned long currentTime = millis();
        if ((currentTime - lastDetectionTime) >= DETECTION_DELAY) {
            Serial.println("Beep detected and confirmed!");
            confirmedWindows = 0;
            lastDetectionTime = currentTime;
            flashLED(LED_PIN, LED_FLASH_DURATION);
            startAudioPlayback();
        }
    }
}

void collectAudioSamples() {
    // Acquire audio samples, offsetting by 2048 to recenter around zero
    for (int i = 0; i < SAMPLES; i++) {
        float raw = (float)analogRead(MIC_PIN);
        vReal[i] = raw - 2048.0f;
        vImag[i] = 0.0f;
        delayMicroseconds((1000000UL / SAMPLING_FREQUENCY));
        esp_task_wdt_reset();
    }
}

void runFFT() {
    // Apply a Hamming window to reduce spectral leakage, then compute the FFT
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();
}
