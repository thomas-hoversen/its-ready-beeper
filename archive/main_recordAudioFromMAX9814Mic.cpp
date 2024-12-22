/**
 * @file main_broken.cpp
 * @brief Attempts to record audio from the MAX9814 microphone at 16 kHz, save to SPIFFS,
 *        and then play it back over Bluetooth with naive mono-to-stereo duplication.
 *        It transmits audio but sounds garbled, indicating a timing/resampling issue
 *        or hardware limitation. We keep this code as a reference for future debugging.
 */

#include <Arduino.h>
#include <SPIFFS.h>
// BluetoothA2DPSource is from https://github.com/pschatzmann/ESP32-A2DP.git
#include "BluetoothA2DPSource.h"
#include <arduinoFFT.h>
#include "esp_task_wdt.h"

// ---------------------------------------------------------------------------
// Configuration Constants
// ---------------------------------------------------------------------------

// Hardware Pins
#define MIC_PIN 35                  // Analog pin for the MAX9814 output
#define LED_PIN 2                   // Onboard LED
#define BUTTON_PIN 15               // Button pin (active LOW)

// FFT Configuration
#define SAMPLES 256                 // Number of samples for the FFT
#define SAMPLING_FREQUENCY 16000    // 16 kHz sampling

// Beep Detection Parameters
#define FREQUENCY_MIN 2000          // Minimum freq for beep detection
#define FREQUENCY_MAX 8000          // Maximum freq for beep detection
float AMPLITUDE_THRESHOLD = 300;    // Threshold for beep amplitude
#define BIN_AMPLITUDE_THRESHOLD 20.0f
#define MULTI_WINDOW_CONFIDENCE 3   // Number of consecutive windows to confirm beep
#define DETECTION_DELAY 1000        // Ms between beep detections
#define LED_FLASH_DURATION 500      // LED blink time

// Button Press for Recording
#define BUTTON_HOLD_THRESHOLD 1000  // 1 second to start recording

// Audio Recording
#define MAX_RECORDING_DURATION 4000 // 4 seconds max
#define RECORDING_AMPLIFICATION 4   // Some gain to the ADC sample
#define WRITE_BUFFER_SIZE 512       // Buffer to write to SPIFFS

// ---------------------------------------------------------------------------
// Global Variables
// ---------------------------------------------------------------------------

// For FFT
float vReal[SAMPLES];
float vImag[SAMPLES];
ArduinoFFT<float> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

unsigned long lastDetectionTime = 0;
int confirmedWindows = 0;

// Recording
bool isRecording = false;
unsigned long recordingStartTime = 0;
File recordingFile;
const char* audioFilePath = "/recorded_audio.raw";
int16_t writeBuffer[WRITE_BUFFER_SIZE];
size_t writeBufferIndex = 0;

// Playback
BluetoothA2DPSource a2dp_source;
const char* bluetoothSpeakerName = "C17A";

// In raw-callback mode, we provide audio data from a global file
static File playbackFile;
static bool isPlayingAudio = false; 
static bool playbackOpened = false;

// ---------------------------------------------------------------------------
// Forward Declarations
// ---------------------------------------------------------------------------
void initPins();
void initSPIFFS();
void initBluetooth();
void connection_state_changed(esp_a2d_connection_state_t state, void *ptr);

void startRecording();
void stopRecording();
void recordAudioSamples();

void collectAudioSamples();
void runFFT();
bool detectBeep();
void handleBeepDetection();
void flashLED(int pin, int duration);

void startAudioPlayback();
void checkPlaybackFinished();

// ---------------------------------------------------------------------------
// setup() and loop()
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    Serial.println("\n\n[setup] Initializing...");

    initPins();
    initSPIFFS();
    initBluetooth();

    Serial.println("[setup] System ready. Listening for beeps, button for recording.");
}

void loop() {
    // Button logic for recording
    static unsigned long buttonPressStart = 0;
    if (digitalRead(BUTTON_PIN) == LOW) {
        if (buttonPressStart == 0) {
            buttonPressStart = millis();
        }
        // If held longer than threshold, start recording (if not already)
        if ((millis() - buttonPressStart > BUTTON_HOLD_THRESHOLD) && !isRecording) {
            Serial.println("[loop] Button held => start recording...");
            startRecording();
        }
    } else {
        if (isRecording) {
            Serial.println("[loop] Button released => stop recording.");
            stopRecording();
        }
        buttonPressStart = 0;
    }

    // If we are recording, gather samples into file
    if (isRecording) {
        recordAudioSamples();
    } 
    else {
        // If not recording, do beep detection
        collectAudioSamples();
        runFFT();
        handleBeepDetection();
    }

    // Check if playback ended (file read done)
    checkPlaybackFinished();
}

// ---------------------------------------------------------------------------
// Recording Functions
// ---------------------------------------------------------------------------
void startRecording() {
    if (isPlayingAudio) {
        // If already playing, we might skip recording or forcibly stop playback
        Serial.println("[startRecording] WARNING: Already playing audio, might cause conflict!");
        return;
    }

    isRecording = true;
    recordingStartTime = millis();

    // Remove old file if exists
    if (SPIFFS.exists(audioFilePath)) {
        SPIFFS.remove(audioFilePath);
        Serial.println("[startRecording] Removed old /recorded_audio.raw");
    }

    // Open new file
    recordingFile = SPIFFS.open(audioFilePath, "w");
    if (!recordingFile) {
        Serial.println("[startRecording] ERROR: Failed to open file for recording.");
        isRecording = false;
        return;
    }
    writeBufferIndex = 0;

    digitalWrite(LED_PIN, HIGH);
    Serial.println("[startRecording] Recording started, LED on.");
}

void stopRecording() {
    if (!isRecording) return;

    isRecording = false;
    digitalWrite(LED_PIN, LOW);

    if (writeBufferIndex > 0) {
        recordingFile.write((uint8_t*)writeBuffer, writeBufferIndex * sizeof(int16_t));
        writeBufferIndex = 0;
    }
    recordingFile.close();
    Serial.println("[stopRecording] Recording stopped and file saved.");
}

void recordAudioSamples() {
    if (millis() - recordingStartTime >= MAX_RECORDING_DURATION) {
        Serial.println("[recordAudioSamples] Max duration reached => stop recording.");
        stopRecording();
        return;
    }

    int16_t sample = (analogRead(MIC_PIN) - 2048) * RECORDING_AMPLIFICATION;
    sample = constrain(sample, -32768, 32767);
    writeBuffer[writeBufferIndex++] = sample;

    if (writeBufferIndex >= WRITE_BUFFER_SIZE) {
        recordingFile.write((uint8_t*)writeBuffer, WRITE_BUFFER_SIZE * sizeof(int16_t));
        writeBufferIndex = 0;
    }
}

// ---------------------------------------------------------------------------
// Playback Functions
// ---------------------------------------------------------------------------

/**
 * @brief This function is called by the Bluetooth library in a dedicated task/thread
 *        whenever it needs more PCM data. We read from playbackFile (raw PCM, 16-bit mono),
 *        replicate each sample to both L/R channels.
 * @param data Buffer to fill with stereo PCM bytes.
 * @param len  Number of bytes requested by the library.
 * @return Number of bytes actually written.
 */
int32_t audioDataCallback(uint8_t* data, int32_t len) {
    if (!playbackOpened || !playbackFile || !playbackFile.available()) {
        return 0;
    }

    int stereoFramesRequested = len / 4;
    int totalBytesWritten = 0;

    for (int i = 0; i < stereoFramesRequested; i++) {
        uint8_t monoBytes[2];
        int actuallyRead = playbackFile.read(monoBytes, 2);
        if (actuallyRead < 2) {
            // EOF
            break;
        }
        // duplicate to stereo
        data[4*i + 0] = monoBytes[0];
        data[4*i + 1] = monoBytes[1];
        data[4*i + 2] = monoBytes[0];
        data[4*i + 3] = monoBytes[1];
        totalBytesWritten += 4;
    }

    return totalBytesWritten;
}

void startAudioPlayback() {
    if (isPlayingAudio) {
        Serial.println("[startAudioPlayback] Already playing, skip...");
        return;
    }
    if (!a2dp_source.is_connected()) {
        Serial.println("[startAudioPlayback] No Bluetooth connection, skip.");
        return;
    }
    if (!SPIFFS.exists(audioFilePath)) {
        Serial.println("[startAudioPlayback] Recorded file not found, skip.");
        return;
    }

    playbackFile = SPIFFS.open(audioFilePath, "rb");
    if (!playbackFile) {
        Serial.println("[startAudioPlayback] ERROR: Could not open recorded file for playback.");
        return;
    }

    playbackOpened = true;
    isPlayingAudio = true;
    Serial.println("[startAudioPlayback] Playback started => reading from recorded file...");
}

void checkPlaybackFinished() {
    if (!isPlayingAudio) return;

    if (!playbackFile.available()) {
        Serial.println("[checkPlaybackFinished] Playback completed. Closing file.");
        playbackFile.close();
        playbackOpened = false;
        isPlayingAudio = false;
    }
}

// ---------------------------------------------------------------------------
// Beep Detection
// ---------------------------------------------------------------------------
void collectAudioSamples() {
    for (int i = 0; i < SAMPLES; i++) {
        float raw = analogRead(MIC_PIN) - 2048.0;
        vReal[i] = raw;
        vImag[i] = 0.0;
        delayMicroseconds(1000000UL / SAMPLING_FREQUENCY);
    }
}

void runFFT() {
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();
}

bool detectBeep() {
    float majorFrequency = FFT.majorPeak();
    float amplitude = 0.0;
    bool sufficientBins = false;

    for (int i = 0; i < SAMPLES / 2; i++) {
        float freq = (i * (float)SAMPLING_FREQUENCY) / (float)SAMPLES;
        if (freq >= FREQUENCY_MIN && freq <= FREQUENCY_MAX) {
            amplitude += vReal[i];
            if (vReal[i] > BIN_AMPLITUDE_THRESHOLD) {
                sufficientBins = true;
            }
        }
    }

    if (majorFrequency >= FREQUENCY_MIN && majorFrequency <= FREQUENCY_MAX &&
        amplitude > AMPLITUDE_THRESHOLD && sufficientBins) {
        return true;
    }
    return false;
}

void handleBeepDetection() {
    if (detectBeep()) {
        confirmedWindows++;
    } else {
        confirmedWindows = 0;
    }

    if (confirmedWindows >= MULTI_WINDOW_CONFIDENCE &&
        millis() - lastDetectionTime >= DETECTION_DELAY) 
    {
        Serial.println("[handleBeepDetection] Beep detected & confirmed! Playback starting...");
        lastDetectionTime = millis();
        flashLED(LED_PIN, LED_FLASH_DURATION);
        startAudioPlayback();
    }
}

// ---------------------------------------------------------------------------
// Initialization / Helpers
// ---------------------------------------------------------------------------
void initPins() {
    pinMode(MIC_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    digitalWrite(LED_PIN, LOW);
}

void initSPIFFS() {
    if (!SPIFFS.begin(true)) {
        Serial.println("[initSPIFFS] ERROR: SPIFFS init failed.");
        while (true) { delay(100); }
    }
    Serial.println("[initSPIFFS] SPIFFS mounted OK.");
}

void connection_state_changed(esp_a2d_connection_state_t state, void *ptr) {
    Serial.print("[connection_state_changed] State: ");
    Serial.println(a2dp_source.to_str(state));
}

void initBluetooth() {
    Serial.println("[initBluetooth] Init Bluetooth...");
    a2dp_source.set_auto_reconnect(true);
    a2dp_source.set_on_connection_state_changed(connection_state_changed);

    // Start in raw callback mode
    a2dp_source.start_raw(bluetoothSpeakerName, audioDataCallback);
    a2dp_source.set_volume(20);

    Serial.println("[initBluetooth] Bluetooth started. Connect to speaker named: C17A");
}

void flashLED(int pin, int duration) {
    digitalWrite(pin, HIGH);
    delay(duration);
    digitalWrite(pin, LOW);
}