#include <Arduino.h>
#include <SPIFFS.h>
#include "BluetoothA2DPSource.h"

// ---------------------------------------------------------------------------
// Pin Definitions
// ---------------------------------------------------------------------------
#define MIC_PIN       35
#define BUTTON_PIN    15
#define LED_PIN       2

// ---------------------------------------------------------------------------
// Recording Configuration
// ---------------------------------------------------------------------------
#define SOURCE_SAMPLE_RATE   44100        // ADC rate
#define RECORD_SECS          4
#define RECORDING_AMPLIFICATION 16
#define WRITE_BUF_SIZE       512

// We'll up-sample by factor 3: 16k → 48k
static const int UPSAMPLE_FACTOR = 3;

// ---------------------------------------------------------------------------
// File Paths
// ---------------------------------------------------------------------------
static const char* FILE_16K = "/recorded_audio.raw";
static const char* FILE_48K = "/recorded_audio_48k.raw";

// ---------------------------------------------------------------------------
// Bluetooth
// ---------------------------------------------------------------------------
static const char* BT_DEVICE_NAME = "C17A";
BluetoothA2DPSource a2dp_source;

// ---------------------------------------------------------------------------
// Global State
// ---------------------------------------------------------------------------
static bool isRecording    = false;
static bool isPlayingAudio = false;

File recordFile;
File playbackFile;

int16_t writeBuffer[WRITE_BUF_SIZE];
size_t writeIndex = 0;
unsigned long recordStartMillis = 0;

// Forward Declarations
void setupPins();
void setupSPIFFS();
void setupBluetooth();
void connectionStateChanged(esp_a2d_connection_state_t state, void* ptr);

void startRecording();
void stopRecording();
void recordAudioSample();
void printSampleStats(const int16_t* buf, size_t len);

void upsampleTo48k(const char* inFilePath, const char* outFilePath, int upFactor);
void startAudioPlayback();
void checkPlaybackFinished();
int32_t audioDataCallback(uint8_t* data, int32_t len);

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("\n[setup] Starting...");

  setupPins();
  setupSPIFFS();
  setupBluetooth();

  // Countdown
  for (int i = 3; i > 0; i--) {
    Serial.printf("[setup] Recording starts in %d...\n", i);
    delay(1000);
  }

  Serial.printf("[setup] Recording for %d seconds at %d Hz...\n", RECORD_SECS, SOURCE_SAMPLE_RATE);
  startRecording();
  recordStartMillis = millis();

  // Record for RECORD_SECS
  while ((millis() - recordStartMillis) < (RECORD_SECS * 1000UL)) {
    recordAudioSample();
    // ~16 kHz => 62.5 microseconds between samples
    delayMicroseconds(1000000UL / SOURCE_SAMPLE_RATE);
  }
  stopRecording();
  Serial.println("[setup] Recording complete! File stored in SPIFFS.");

  // Now offline up-sample to 48k
  upsampleTo48k(FILE_16K, FILE_48K, UPSAMPLE_FACTOR);

  // Ready to play when button is pressed
}

// ---------------------------------------------------------------------------
void loop() {
  checkPlaybackFinished();

  // Button triggers playback
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(50); // Debounce
    if (digitalRead(BUTTON_PIN) == LOW && !isPlayingAudio) {
      startAudioPlayback();
    }
  }

  delay(10);
  yield();
}

// ---------------------------------------------------------------------------
// Setup Helpers
// ---------------------------------------------------------------------------
void setupPins() {
  pinMode(MIC_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void setupSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("[setupSPIFFS] ERROR: SPIFFS init failed!");
    while(true) { delay(100); }
  }
  Serial.println("[setupSPIFFS] SPIFFS mounted OK.");
}

void setupBluetooth() {
  Serial.println("[setupBluetooth] Initializing Bluetooth A2DP...");
  a2dp_source.set_auto_reconnect(true);

  // If your library supports these calls, keep them. Otherwise, comment them out.
//   a2dp_source.set_sample_rate(48000);      // Force 48k if supported
//   a2dp_source.set_bits_per_sample(16);     // 16 bits
//   a2dp_source.set_channels(2);            // stereo output

  a2dp_source.set_on_connection_state_changed(connectionStateChanged);
  a2dp_source.start_raw(BT_DEVICE_NAME, audioDataCallback);
  a2dp_source.set_volume(20);

  Serial.printf("[setupBluetooth] Bluetooth started as '%s'.\n", BT_DEVICE_NAME);
  Serial.println("[setupBluetooth] Pair your speaker with this device to connect.");
}

void connectionStateChanged(esp_a2d_connection_state_t state, void* ptr) {
  const char* st = a2dp_source.to_str(state);
  Serial.printf("[Bluetooth] Connection state changed => %s\n", st);
  if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
    Serial.println("[Bluetooth] Speaker CONNECTED, ready to stream!");
  } else if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
    Serial.println("[Bluetooth] Speaker DISCONNECTED!");
  }
}

// ---------------------------------------------------------------------------
// Recording
// ---------------------------------------------------------------------------
void startRecording() {
  isRecording = true;
  digitalWrite(LED_PIN, HIGH);

  if (SPIFFS.exists(FILE_16K)) {
    SPIFFS.remove(FILE_16K);
    Serial.printf("[startRecording] Removed old %s.\n", FILE_16K);
  }

  recordFile = SPIFFS.open(FILE_16K, "w");
  if (!recordFile) {
    Serial.printf("[startRecording] ERROR: Could not open %s!\n", FILE_16K);
    isRecording = false;
  }
  writeIndex = 0;

  Serial.println("[startRecording] Recording started.");
}

void stopRecording() {
  if (!isRecording) return;
  isRecording = false;
  digitalWrite(LED_PIN, LOW);

  // Flush leftover
  if (writeIndex > 0) {
    recordFile.write((uint8_t*)writeBuffer, writeIndex * sizeof(int16_t));
    writeIndex = 0;
  }
  recordFile.close();

  Serial.println("[stopRecording] Recording stopped & file closed.");
}

void recordAudioSample() {
  if (!isRecording) return;

  int16_t sample = analogRead(MIC_PIN) - 2048;    // center around 0
  sample *= RECORDING_AMPLIFICATION;
  sample = constrain(sample, -32768, 32767);

  writeBuffer[writeIndex++] = sample;

  if (writeIndex >= WRITE_BUF_SIZE) {
    recordFile.write((uint8_t*)writeBuffer, WRITE_BUF_SIZE * sizeof(int16_t));
    printSampleStats(writeBuffer, WRITE_BUF_SIZE);
    writeIndex = 0;
  }
}

void printSampleStats(const int16_t* buf, size_t len) {
  long totalAbs = 0;
  for (size_t i = 0; i < len; i++) {
    totalAbs += abs(buf[i]);
  }
  float avg = (float)totalAbs / (float)len;
  Serial.printf("[stats] AvgAbsAmplitude = %.1f\n", avg);
}

// ---------------------------------------------------------------------------
// Offline Up-Sampling 16k → 48k
// ---------------------------------------------------------------------------
/**
 * Read the 16 kHz mono file. For each adjacent pair of samples:
 *   - Generate UPSAMPLE_FACTOR(=3) frames by linear interpolation,
 *   - Duplicate each frame for stereo,
 *   - Write them to /recorded_audio_48k.raw.
 *
 * Result => 3x as many samples, each frame is stereo => 4 bytes per sample.
 */
void upsampleTo48k(const char* inFilePath, const char* outFilePath, int upFactor) {
  if (!SPIFFS.exists(inFilePath)) {
    Serial.printf("[upsampleTo48k] ERROR: Missing input file: %s\n", inFilePath);
    return;
  }

  // Remove old 48k file, if any
  if (SPIFFS.exists(outFilePath)) {
    SPIFFS.remove(outFilePath);
  }

  File inFile  = SPIFFS.open(inFilePath, "rb");
  File outFile = SPIFFS.open(outFilePath, "wb");
  if (!inFile || !outFile) {
    Serial.println("[upsampleTo48k] ERROR: Could not open in/out files!");
    return;
  }

  Serial.println("[upsampleTo48k] Starting offline up-sample from 16k to 48k...");

  bool havePrev = false;
  int16_t prevSample = 0;

  // We'll read 2 bytes at a time (one 16-bit sample at 16 kHz)
  // For each pair (prevSample, nextSample), generate 3 frames
  // Then each frame is written as 2-ch (stereo).
  while (true) {
    int16_t nextSample;
    uint8_t sampleBytes[2];
    int got = inFile.read(sampleBytes, 2);
    if (got < 2) {
      // No more samples - we're done
      break;
    }
    nextSample = (int16_t)((sampleBytes[1] << 8) | sampleBytes[0]);

    if (!havePrev) {
      // For the very first iteration, set prevSample, skip output
      prevSample = nextSample;
      havePrev = true;
      continue;
    }

    // We have prevSample & nextSample => produce 'upFactor' frames
    // (including the fraction from 0/3 up to 2/3).
    for (int i = 0; i < upFactor; i++) {
      float frac = (float)i / (float)upFactor; 
      float interp = (1.0f - frac) * (float)prevSample + frac * (float)nextSample;
      int16_t finalSample = (int16_t)roundf(interp);

      // Write finalSample in stereo (4 bytes)
      uint8_t stereo[4];
      stereo[0] = (uint8_t)(finalSample & 0xFF);
      stereo[1] = (uint8_t)((finalSample >> 8) & 0xFF);
      stereo[2] = stereo[0];
      stereo[3] = stereo[1];

      outFile.write(stereo, 4);
    }

    // Shift next -> prev
    prevSample = nextSample;
  }

  inFile.close();
  outFile.close();
  Serial.println("[upsampleTo48k] Up-sampling complete! Created file at 48k stereo.");
}

// ---------------------------------------------------------------------------
// Playback from 48k Stereo File
// ---------------------------------------------------------------------------
/**
 * Now we have an actual 48k stereo RAW file. We just read chunks
 * and feed them out “as is.” The library is set to 48k, so we don't
 * do any interpolation in the callback.
 */
int32_t audioDataCallback(uint8_t* data, int32_t len) {
  if (!isPlayingAudio || !playbackFile || !playbackFile.available()) {
    return 0;
  }

  int32_t bytesToRead = min((int32_t)playbackFile.available(), len);
  int32_t actuallyRead = playbackFile.read(data, bytesToRead);
  return actuallyRead;
}

void startAudioPlayback() {
  if (isPlayingAudio) {
    Serial.println("[startAudioPlayback] Already playing. Skip.");
    return;
  }
  if (!a2dp_source.is_connected()) {
    Serial.println("[startAudioPlayback] No BT speaker connected. Skip.");
    return;
  }
  if (!SPIFFS.exists(FILE_48K)) {
    Serial.printf("[startAudioPlayback] 48k file '%s' not found. Skip.\n", FILE_48K);
    return;
  }

  playbackFile = SPIFFS.open(FILE_48K, "rb");
  if (!playbackFile) {
    Serial.println("[startAudioPlayback] Could not open 48k file. Skip.");
    return;
  }

  isPlayingAudio = true;
  Serial.printf("[startAudioPlayback] Playback started => reading %s\n", FILE_48K);
}

void checkPlaybackFinished() {
  if (!isPlayingAudio) return;

  // If file is at EOF, end
  if (!playbackFile.available()) {
    Serial.println("[checkPlaybackFinished] Playback complete!");
    playbackFile.close();
    isPlayingAudio = false;
  }
}