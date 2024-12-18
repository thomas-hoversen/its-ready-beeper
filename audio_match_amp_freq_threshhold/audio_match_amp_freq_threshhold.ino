#include <Arduino.h>
#include <arduinoFFT.h>

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
unsigned long lastDetectionTime = 0; // Time of the last beep detection
unsigned long ledFlashStartTime = 0; // Time when the LED starts flashing
int confirmedWindows = 0;            // Counter for multi-window confirmation
int beepCount = 0;                   // Counter for the total number of beeps detected

// Create FFT object
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

void setup() {
    Serial.begin(115200);
    pinMode(MIC_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW); // Ensure LED is off at startup

    Serial.println("Calibrating baseline noise level...");
    calibrateBaseline();
    Serial.println("Listening for appliance beeps...");
}

void loop() {
    // Collect audio samples
    for (uint16_t i = 0; i < SAMPLES; i++) {
        vReal[i] = analogRead(MIC_PIN) - 2048; // Center the signal
        vImag[i] = 0;                          // Clear imaginary part
        delayMicroseconds(1000000 / SAMPLING_FREQUENCY); // Sampling delay
    }

    // Perform FFT
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); // Apply window function
    FFT.compute(FFTDirection::Forward);                       // Compute FFT
    FFT.complexToMagnitude();                                 // Compute magnitudes

    // Check for a beep in the frequency range
    if (detectBeep()) {
        confirmedWindows++; // Increment the confirmation counter
    } else {
        confirmedWindows = 0; // Reset the counter if no beep detected
    }

    // Log beep detection and flash LED only if confirmed across multiple windows
    if (confirmedWindows >= MULTI_WINDOW_CONFIDENCE) {
        unsigned long currentTime = millis();

        // Ensure a minimum delay between consecutive detections
        if (currentTime - lastDetectionTime >= DETECTION_DELAY) {
            beepCount++; // Increment the beep counter
            Serial.print("IT'S READY! Total beeps: ");
            Serial.println(beepCount);

            // Flash the LED
            digitalWrite(LED_PIN, HIGH);
            ledFlashStartTime = currentTime;

            lastDetectionTime = currentTime; // Update the last detection time
            confirmedWindows = 0;            // Reset confirmation counter
        }
    }

    // Turn off the LED after the flash duration
    if (ledFlashStartTime > 0 && millis() - ledFlashStartTime >= LED_FLASH_DURATION) {
        digitalWrite(LED_PIN, LOW);
        ledFlashStartTime = 0;
    }
}

// Calibrate the baseline noise level
void calibrateBaseline() {
    long sum = 0;

    for (int i = 0; i < 100; i++) { // Collect 100 samples
        sum += analogRead(MIC_PIN);
        delay(10);
    }

    int baseline = sum / 100;
    Serial.print("Baseline noise level: ");
    Serial.println(baseline);

    // Adjust amplitude threshold based on baseline noise
    if (baseline > SILENCE_THRESHOLD) {
        Serial.println("Adjusting amplitude threshold based on baseline noise...");
        AMPLITUDE_THRESHOLD += baseline - SILENCE_THRESHOLD;
    }
}

// Detect a beep by analyzing FFT results
bool detectBeep() {
    float majorFrequency = FFT.majorPeak(); // Identify the dominant frequency
    float amplitude = 0.0;
    bool sufficientBins = false;

    // Sum up amplitudes in the target frequency range
    for (uint16_t i = 0; i < SAMPLES / 2; i++) {
        float frequency = (i * SAMPLING_FREQUENCY) / SAMPLES;
        if (frequency >= FREQUENCY_MIN && frequency <= FREQUENCY_MAX) {
            amplitude += vReal[i];
            if (vReal[i] > BIN_AMPLITUDE_THRESHOLD) {
                sufficientBins = true; // Ensure at least one bin exceeds the bin threshold
            }
        }
    }

    // Validate the beep
    if (majorFrequency >= FREQUENCY_MIN && majorFrequency <= FREQUENCY_MAX &&
        amplitude > AMPLITUDE_THRESHOLD && sufficientBins) {
        return true; // Beep detected
    }

    return false; // No beep detected
}
