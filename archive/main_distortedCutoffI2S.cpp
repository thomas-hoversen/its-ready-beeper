/**
 * Combined I2S (INMP441 @16k, Left Channel) + Offline Upsample + Bluetooth Playback
 *
 * 1) Record from INMP441 at 16 kHz, 32-bit frames (top 24 bits used),
 *    storing as 16-bit PCM => /recorded_audio_16k.raw
 * 2) Offline up-sample from 16k => 48k stereo => /recorded_audio_48k.raw
 * 3) Stream the 48k file over Bluetooth with a raw callback
 * 4) Press button (GPIO25) to start playback
 *
 * Should play the entire 4 seconds of recorded audio.
 */

#include <Arduino.h>
#include <SPIFFS.h>
#include <driver/i2s.h>
#include "BluetoothA2DPSource.h"

// ---------------------------------------------------------------------------
// I2S Mic Pin Definitions (INMP441 => 3.3V, GND, BCLK=14, LRCLK=15, DOUT=32)
// L/R pin => GND => we use ONLY_LEFT channel
// ---------------------------------------------------------------------------
#define I2S_SCK  14 // BCLK
#define I2S_WS   15 // LRCLK
#define I2S_SD   34 // DOUT
#define I2S_PORT I2S_NUM_0

// ---------------------------------------------------------------------------
// I2S Record Config: 16 kHz, 32 bits, Left Channel (INMP441 => top 24 bits used)
// ---------------------------------------------------------------------------
static const int   I2S_SAMPLE_RATE      = 16000;
static const bool  USE_APLL             = false;   // or true if your board needs it
static const bool  TX_DESC_AUTO_CLEAR   = false;
static const int   FIXED_MCLK           = 0;
static const int   I2S_DMA_BUF_COUNT    = 4;
static const int   I2S_DMA_BUF_LEN      = 128;
static const int   I2S_READ_SAMPLES     = 64;  // how many 32-bit frames we read at once

// We'll record for 4 seconds at 16k
static const int   RECORD_SECONDS       = 4;     
static const char* FILE_16K             = "/recorded_audio_16k.raw";
static const char* FILE_48K             = "/recorded_audio_48k.raw";

// Offline up-sample factor (16k => 48k => ×3, stereo)
static const int   UPSAMPLE_FACTOR      = 3;

// ---------------------------------------------------------------------------
// Bluetooth
// ---------------------------------------------------------------------------
BluetoothA2DPSource a2dp_source;
static const char*  BT_DEVICE_NAME      = "C17A";
static bool isPlayingAudio              = false; 
static File playbackFile;

// ---------------------------------------------------------------------------
// Button/LED Pins
// ---------------------------------------------------------------------------
#define BUTTON_PIN   25
#define LED_PIN      2

// ---------------------------------------------------------------------------
// Forward Declarations
// ---------------------------------------------------------------------------
void setupPins();
void setupSPIFFS();
void initI2S();
void recordI2S_16k_toFile(const char* path, int seconds);
void upsample16kTo48k(const char* inPath, const char* outPath, int upFactor);

void initBluetooth();
void startAudioPlayback();
void checkPlaybackFinished();
int32_t audioDataCallback(uint8_t* data, int32_t len);

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("[setup] Starting...");

  setupPins();
  setupSPIFFS();
  initI2S();
  initBluetooth();

  // Simple 3-second countdown
  for (int i=3; i>0; i--) {
    Serial.printf("[setup] Recording in %d...\n", i);
    delay(1000);
  }

  // 1) Record from I2S at 16 kHz, store as 16-bit PCM
  Serial.println("[setup] Recording 16k for 4 seconds...");
  recordI2S_16k_toFile(FILE_16K, RECORD_SECONDS);

  // 2) Offline up-sample: 16k => 48k, stereo
  upsample16kTo48k(FILE_16K, FILE_48K, UPSAMPLE_FACTOR);

  Serial.printf("[setup] Done. Press button (GPIO %d) to play 48k file...\n", BUTTON_PIN);
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop() {
  checkPlaybackFinished();

  // Press button => start playback if not playing
  if (digitalRead(BUTTON_PIN) == LOW && !isPlayingAudio) {
    delay(50); // simple debounce
    if (digitalRead(BUTTON_PIN) == LOW) {
      startAudioPlayback();
    }
  }

  delay(10);
}

// ---------------------------------------------------------------------------
// Basic Pin Setup for LED / Button
// ---------------------------------------------------------------------------
void setupPins() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

// ---------------------------------------------------------------------------
// SPIFFS
// ---------------------------------------------------------------------------
void setupSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("[setupSPIFFS] ERROR: SPIFFS init failed.");
    while(true){ delay(100); }
  }
  Serial.println("[setupSPIFFS] SPIFFS ok.");
}

// ---------------------------------------------------------------------------
// initI2S()
//  - Master RX at 16 kHz, 32 bits, Only Left (L/R=GND on mic)
// ---------------------------------------------------------------------------
void initI2S() {
  Serial.println("[initI2S] Installing I2S driver...");

  i2s_config_t i2s_config = {
    .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate          = I2S_SAMPLE_RATE,
    .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format       = I2S_CHANNEL_FMT_ONLY_RIGHT,  // L/R => GND
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags     = 0,
    .dma_buf_count        = I2S_DMA_BUF_COUNT,
    .dma_buf_len          = I2S_DMA_BUF_LEN,
    .use_apll             = USE_APLL,
    .tx_desc_auto_clear   = TX_DESC_AUTO_CLEAR,
    .fixed_mclk           = FIXED_MCLK
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

  // Some boards need i2s_set_clk explicitly
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
  i2s_read(I2S_PORT, dummy, 256, &dummyRead, 100/portTICK_RATE_MS);

  Serial.println("[initI2S] I2S init done.");
}

// ---------------------------------------------------------------------------
// recordI2S_16k_toFile(path, secs)
//  - Reads top 24 bits from 32-bit sample => downshift => clamp to int16_t
//  - Writes a raw 16-bit PCM file at 16 kHz
// ---------------------------------------------------------------------------
void recordI2S_16k_toFile(const char* path, int secs) {
  if (SPIFFS.exists(path)) {
    SPIFFS.remove(path);
  }
  File f = SPIFFS.open(path, "w");
  if (!f) {
    Serial.printf("[recordI2S_16k_toFile] ERROR: Could not open %s!\n", path);
    return;
  }
  Serial.printf("[recordI2S_16k_toFile] Recording => %s for %d sec.\n", path, secs);

  unsigned long startMs = millis();
  unsigned long endMs   = startMs + secs*1000UL;
  size_t totalWritten   = 0;

  int32_t i2sBuffer[I2S_READ_SAMPLES]; // 32-bit frames
  while (millis() < endMs) {
    size_t bytesIn = 0;
    esp_err_t r = i2s_read(I2S_PORT, i2sBuffer, I2S_READ_SAMPLES * sizeof(int32_t),
                           &bytesIn, portMAX_DELAY);
    if (r == ESP_OK && bytesIn > 0) {
      int frames = bytesIn / sizeof(int32_t);
      for (int i=0; i<frames; i++){
        // shift down 8 => 24-bit
        int32_t val24 = i2sBuffer[i] >> 8;
        // clamp 24-bit => 16-bit
        if (val24 > 32767)  val24 = 32767;
        if (val24 < -32768) val24 = -32768;
        int16_t s16 = (int16_t)val24;

        f.write((uint8_t*)&s16, sizeof(int16_t));
        totalWritten += sizeof(int16_t);
      }
    }
  }

  f.close();
  Serial.printf("[recordI2S_16k_toFile] Done. Wrote %u bytes.\n", (unsigned)totalWritten);
}

// ---------------------------------------------------------------------------
// upsample16kTo48k()
//  - Reads 16-bit mono from 'inPath'
//  - For each pair of samples => produce 'upFactor' frames => stereo => outPath
// ---------------------------------------------------------------------------
void upsample16kTo48k(const char* inPath, const char* outPath, int upFactor) {
  if (!SPIFFS.exists(inPath)) {
    Serial.printf("[upsample16kTo48k] ERROR: Missing '%s'\n", inPath);
    return;
  }
  if (SPIFFS.exists(outPath)) {
    SPIFFS.remove(outPath);
  }

  File inFile  = SPIFFS.open(inPath,  "rb");
  File outFile = SPIFFS.open(outPath, "wb");
  if (!inFile || !outFile) {
    Serial.println("[upsample16kTo48k] ERROR opening files!");
    return;
  }

  Serial.printf("[upsample16kTo48k] 16k => 48k => '%s'...\n", outPath);

  bool havePrev = false;
  int16_t prevSample = 0;
  size_t totalOut = 0;

  while (true) {
    int16_t nextSample;
    uint8_t sampleBytes[2];
    int got = inFile.read(sampleBytes, 2);
    if (got < 2) break; // EOF

    nextSample = (int16_t)((sampleBytes[1] << 8) | sampleBytes[0]);
    if (!havePrev) {
      prevSample = nextSample;
      havePrev = true;
      continue;
    }

    // For each pair => produce 'upFactor' frames
    for (int i=0; i<upFactor; i++){
      float frac = (float)i / (float)upFactor;
      float valF = (1.0f - frac)*prevSample + frac*(float)nextSample;
      int16_t s16 = (int16_t)roundf(valF);

      // Write as stereo => 4 bytes
      uint8_t stereo[4];
      stereo[0] = (uint8_t)(s16 & 0xFF);
      stereo[1] = (uint8_t)((s16 >> 8) & 0xFF);
      // Duplicate for Right
      stereo[2] = stereo[0];
      stereo[3] = stereo[1];

      outFile.write(stereo, 4);
      totalOut += 4;
    }
    prevSample = nextSample;
  }

  inFile.close();
  outFile.close();
  Serial.printf("[upsample16kTo48k] Wrote %u bytes.\n", (unsigned)totalOut);
}

// ---------------------------------------------------------------------------
// Bluetooth
// ---------------------------------------------------------------------------
void initBluetooth() {
  Serial.println("[initBluetooth] Initializing Bluetooth...");
  a2dp_source.set_auto_reconnect(true);

  // optional: log connection states
  a2dp_source.set_on_connection_state_changed([](esp_a2d_connection_state_t state, void*){
    Serial.printf("[Bluetooth] Connection => %s\n", a2dp_source.to_str(state));
  });

  // Start in raw callback mode
  a2dp_source.start_raw(BT_DEVICE_NAME, audioDataCallback);
  a2dp_source.set_volume(20);

  Serial.printf("[initBluetooth] BT name='%s', started.\n", BT_DEVICE_NAME);
}

void startAudioPlayback() {
  if (isPlayingAudio) {
    Serial.println("[startAudioPlayback] Already playing. Skip.");
    return;
  }
  if (!a2dp_source.is_connected()) {
    Serial.println("[startAudioPlayback] No speaker connected. Skip.");
    return;
  }
  if (!SPIFFS.exists(FILE_48K)) {
    Serial.printf("[startAudioPlayback] Missing '%s'. Skip.\n", FILE_48K);
    return;
  }

  playbackFile = SPIFFS.open(FILE_48K, "rb");
  if (!playbackFile) {
    Serial.printf("[startAudioPlayback] Could not open '%s'.\n", FILE_48K);
    return;
  }

  isPlayingAudio = true;
  Serial.printf("[startAudioPlayback] Playing '%s'...\n", FILE_48K);
}

void checkPlaybackFinished() {
  if (!isPlayingAudio) return;

  if (!playbackFile.available()) {
    Serial.println("[checkPlaybackFinished] Playback finished!");
    playbackFile.close();
    isPlayingAudio = false;
  }
}

// Called by the Bluetooth library in raw mode to get more PCM bytes
int32_t audioDataCallback(uint8_t* data, int32_t len) {
  if (!isPlayingAudio || !playbackFile || !playbackFile.available()) {
    return 0;
  }

  int32_t toRead = min((int32_t)playbackFile.available(), len);
  int32_t got    = playbackFile.read(data, toRead);
  return got; 
}