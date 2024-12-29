/**
 * @file main.cpp
 * @brief Example with beep detection + captive portal + scanning for speakers
 */

#include <Arduino.h>
#include <SPIFFS.h>
#include "BluetoothA2DPSource.h"
#include <arduinoFFT.h>
#include "esp_task_wdt.h"
#include <driver/i2s.h>
#include <esp_bt.h>
#include <esp_gap_bt_api.h>
#include <esp_a2dp_api.h>

// Our beep logic classes
#include "BeepDetector.h"
#include "BeepHistory.h"

// DiscoveredDevice struct
#include "DiscoveredDevice.h"

// Captive Portal
#include "CaptivePortal.h"

// ---------------------------------------------------------------------------
// Configuration Constants (RELAXED DETECTION)
// ---------------------------------------------------------------------------

// I2S
#define I2S_SCK  14
#define I2S_WS   15
#define I2S_SD   34
#define I2S_PORT I2S_NUM_0

static const int   I2S_SAMPLE_RATE      = 16000;
static const bool  USE_APLL             = false;
static const bool  TX_DESC_AUTO_CLEAR   = false;
static const int   FIXED_MCLK           = 0;
static const int   I2S_DMA_BUF_COUNT    = 4;
static const int   I2S_DMA_BUF_LEN      = 128;

// FFT & Beep Detection
#define SAMPLES                  512
#define SAMPLING_FREQUENCY       16000
#define FREQUENCY_MIN            1200
#define FREQUENCY_MAX            9000
static float BIN_AMPLITUDE_THRESHOLD    = 10;
static float AMPLITUDE_THRESHOLD        = 3000;
static const int   I2S_READ_CHUNK       = 64;
static const int   MIC_PROXIMITY_THRESHOLD = 100;

// Consecutive-window detection
static const int   MULTI_WINDOW_CONFIDENCE = 4;
static const unsigned long DETECTION_DELAY  = 1000; // 1s
static const unsigned long TIME_WINDOW_MS   = 5000;
static const int           THRESHOLD_COUNT  = 3;

// LED / Button
#define LED_PIN          2
#define BUTTON_PIN       25
static const int   LED_FLASH_DURATION    = 500;

// ---------------------------------------------------------------------------
// Instantiate classes
// ---------------------------------------------------------------------------
BeepDetector beepDetector(MULTI_WINDOW_CONFIDENCE, DETECTION_DELAY);
BeepHistory  beepHistory(THRESHOLD_COUNT, TIME_WINDOW_MS);

// FFT arrays
float vReal[SAMPLES];
float vImag[SAMPLES];
#include <arduinoFFT.h>
ArduinoFFT<float> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

// Bluetooth + Audio
BluetoothA2DPSource a2dp_source;
File audioFile;  
const char* audioFilePath = "/audio1.wav";  

// State
bool playingAudio         = false;
unsigned long ignoreBeepUntil = 0;  
int16_t maxSampleInFrame  = 0;

// ---------------------------------------------------------------------------
// Captive Portal object
// ---------------------------------------------------------------------------
CaptivePortal captivePortal;

/**
 * These two globals match the 'extern' lines in CaptivePortal.cpp
 */
std::vector<DiscoveredDevice> discoveredDevices;  // real definition
bool scanning = false;                             // real definition

// Forward declarations
int32_t audioDataCallback(uint8_t* data, int32_t len);

// Extra
void initPins();
void initWatchdogTimer();
void initSPIFFS();
void initI2S();
void initBluetooth();

void startAudioPlayback();
void handlePlaybackCompletion();
void collectAudioSamples();
void runFFT();
bool detectBeep();
void handleBeepDetection();

// GAP + scanning
void startScan();
void stopScan();
void onDeviceFound(const DiscoveredDevice& dev);
static void btGapCallback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
bool connectToDeviceByMac(const String& macStr);

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    Serial.println("\n\n[MAIN] Initializing (Relaxed Detection)...");

    initPins();
    initWatchdogTimer();
    initSPIFFS();
    // initI2S();
    // initBluetooth();

    // Set up captive portal
    captivePortal.begin();
    Serial.println("[MAIN] CaptivePortal initialized.");

    Serial.println("[MAIN] System ready. Looking for beeps or button presses...");
}

// ---------------------------------------------------------------------------
// Loop
// ---------------------------------------------------------------------------
void loop() {
    // 1) Captive portal
    captivePortal.handleClients();

    // 2) Button => immediate playback
    if (digitalRead(BUTTON_PIN) == LOW) {
        Serial.println("[MAIN] Button pressed => immediate playback");
        startAudioPlayback();
    }
    // Reset the watchdog:
    esp_task_wdt_reset();

    // Give time to other tasks:
    delay(10);
    // 3) Check if playback ended
    // handlePlaybackCompletion();

    // // 4) Gather samples + run FFT
    // collectAudioSamples();
    // runFFT();
    // handleBeepDetection();
}

// ---------------------------------------------------------------------------
// collectAudioSamples()
// ---------------------------------------------------------------------------
void collectAudioSamples() {
    maxSampleInFrame = 0;
    int count = 0;

    while (count < SAMPLES) {
        uint8_t i2sBuffer[I2S_READ_CHUNK * 4];
        size_t bytesRead = 0;
        i2s_read(I2S_PORT, i2sBuffer, sizeof(i2sBuffer), &bytesRead, portMAX_DELAY);

        int frames = bytesRead / 4;
        for (int i = 0; i < frames && count < SAMPLES; i++) {
            uint8_t mid = i2sBuffer[i*4 + 2];
            uint8_t msb = i2sBuffer[i*4 + 3];

            int16_t sample16 = (int16_t)((msb << 8) | mid);
            if (abs(sample16) > abs(maxSampleInFrame)) {
                maxSampleInFrame = sample16;
            }

            vReal[count] = (float)sample16;
            vImag[count] = 0.0f;
            count++;
            // Kick watchdog
            esp_task_wdt_reset();
        }
    }
}

// ---------------------------------------------------------------------------
// runFFT()
// ---------------------------------------------------------------------------
void runFFT() {
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();
}

// ---------------------------------------------------------------------------
// detectBeep()
// ---------------------------------------------------------------------------
bool detectBeep() {
    // Provide a debug log for beep detection steps
    // (especially if you want to see how often it's called)

    if (abs(maxSampleInFrame) <= MIC_PROXIMITY_THRESHOLD) {
        return false; 
    }
    float majorFrequency = FFT.majorPeak();
    float amplitude      = 0.0f;
    bool  sufficientBins = false;

    for (int i = 0; i < SAMPLES/2; i++) {
        float freq = (i * (float)SAMPLING_FREQUENCY) / (float)SAMPLES;
        if (freq >= FREQUENCY_MIN && freq <= FREQUENCY_MAX) {
            amplitude += vReal[i];
            if (vReal[i] > BIN_AMPLITUDE_THRESHOLD) {
                sufficientBins = true;
            }
        }
        esp_task_wdt_reset();
    }

    bool beepDetected = (
        majorFrequency >= FREQUENCY_MIN &&
        majorFrequency <= FREQUENCY_MAX &&
        amplitude      >  AMPLITUDE_THRESHOLD &&
        sufficientBins
    );

    if (beepDetected) {
        Serial.println("[MAIN] Potential beep detected -> meets frequency & amplitude criteria.");
    }
    return beepDetected;
}

// ---------------------------------------------------------------------------
// handleBeepDetection()
// ---------------------------------------------------------------------------
void handleBeepDetection() {
    if (playingAudio) {
        Serial.println("[MAIN] Currently playing audio; skipping beep detection.");
        return; 
    }
    if (millis() < ignoreBeepUntil) {
        Serial.println("[MAIN] Still ignoring beeps (cooldown).");
        return;
    }

    bool beepInThisWindow = detectBeep();
    bool beepEvent = beepDetector.update(beepInThisWindow, millis());

    if (beepEvent) {
        Serial.println("[MAIN] -> A new beep event just got confirmed!");
        beepHistory.addBeep(millis());

        if (beepHistory.hasReachedThreshold()) {
            Serial.println("[MAIN] Beep history => threshold => Start playback!");
            beepHistory.clear();
            startAudioPlayback();
        } else {
            Serial.println("[MAIN] Not enough beep events yet...");
        }
    }
}

// ---------------------------------------------------------------------------
// Playback
// ---------------------------------------------------------------------------
int32_t audioDataCallback(uint8_t* data, int32_t len) {
    if (!audioFile || !audioFile.available()) {
        return 0;
    }
    size_t bytesRead = audioFile.read(data, len);
    return (int32_t)bytesRead;
}

void startAudioPlayback() {
    Serial.println("[MAIN] startAudioPlayback called...");
    if (!a2dp_source.is_connected()) {
        Serial.println("[MAIN] No BT connection => skip playback");
        return;
    }
    if (audioFile) {
        audioFile.close();
    }
    audioFile = SPIFFS.open(audioFilePath, "r");
    if (!audioFile || !audioFile.available()) {
        Serial.println("[MAIN] Audio file not available => skip");
        return;
    }
    if (!audioFile.seek(0, SeekSet)) {
        Serial.println("[MAIN] Failed to rewind audio file => skip");
        return;
    }

    playingAudio = true;
    Serial.println("[MAIN] Starting audio playback...");

    // ONLY flash LED right before playing
    Serial.println("[MAIN] Flashing LED because audio is about to play...");
    digitalWrite(LED_PIN, HIGH);
    delay(LED_FLASH_DURATION);
    digitalWrite(LED_PIN, LOW);
}

void handlePlaybackCompletion() {
    if (playingAudio && (!audioFile || !audioFile.available())) {
        Serial.println("[MAIN] Playback ended.");
        playingAudio = false;
        if (audioFile) {
            audioFile.close();
        }
        // Provide a short ignore window after playback ends
        ignoreBeepUntil = millis() + 1500;
    }
}

// ---------------------------------------------------------------------------
// initPins()
// ---------------------------------------------------------------------------
void initPins() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    digitalWrite(LED_PIN, LOW);
    Serial.println("[MAIN] Pins init done.");
}

// ---------------------------------------------------------------------------
// initWatchdogTimer()
// ---------------------------------------------------------------------------
void initWatchdogTimer() {
    esp_task_wdt_init(10, true);
    esp_task_wdt_add(NULL);
    Serial.println("[MAIN] Watchdog init done.");
}

// ---------------------------------------------------------------------------
// initSPIFFS()
// ---------------------------------------------------------------------------
void initSPIFFS() {
    if (!SPIFFS.begin(true)) {
        Serial.println("[MAIN] SPIFFS mount failed!");
        while(1){delay(100);}
    }
    Serial.println("[MAIN] SPIFFS mounted successfully.");
    File root = SPIFFS.open("/");
    File f = root.openNextFile();
    while(f) {
        Serial.printf("[MAIN] Found file: %s\n", f.name());
        f = root.openNextFile();
    }
}

// ---------------------------------------------------------------------------
// initI2S()
// ---------------------------------------------------------------------------
void initI2S() {
    Serial.println("[MAIN] initI2S: Installing I2S driver...");
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        // Note: I2S_COMM_FORMAT_I2S and I2S_COMM_FORMAT_I2S_MSB are deprecated,
        // but still used widely. Future versions may need an updated approach.
        .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = 0,
        .dma_buf_count = I2S_DMA_BUF_COUNT,
        .dma_buf_len = I2S_DMA_BUF_LEN,
        .use_apll = USE_APLL,
        .tx_desc_auto_clear = TX_DESC_AUTO_CLEAR,
        .fixed_mclk = FIXED_MCLK
    };
    esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if(err != ESP_OK) {
        Serial.printf("[MAIN][initI2S] driver_install failed: %d\n", err);
        while(true){delay(100);}
    }
    i2s_pin_config_t pinConfig = {
        .bck_io_num = I2S_SCK,
        .ws_io_num  = I2S_WS,
        .data_out_num = -1,
        .data_in_num  = I2S_SD
    };
    err = i2s_set_pin(I2S_PORT, &pinConfig);
    if(err != ESP_OK){
        Serial.printf("[MAIN][initI2S] set_pin failed: %d\n", err);
        while(true){delay(100);}
    }
    i2s_set_clk(I2S_PORT, I2S_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);
    i2s_start(I2S_PORT);
    Serial.println("[MAIN] initI2S done.");
}

// ---------------------------------------------------------------------------
// initBluetooth()
// ---------------------------------------------------------------------------
void initBluetooth() {
    Serial.println("[MAIN] initBluetooth: Starting Classic BT...");

    // Enable Classic BT if not started
    if (!btStarted() && !btStart()) {
        Serial.println("[MAIN][BT] Failed to start classic BT stack!");
        return;
    }

    // Register GAP callback
    esp_bt_gap_register_callback(btGapCallback);

    // Initialize A2DP source
    a2dp_source.set_auto_reconnect(true);
    a2dp_source.set_on_connection_state_changed([](esp_a2d_connection_state_t state, void*){
        Serial.printf("[MAIN][BT] Connection => %s\n", a2dp_source.to_str(state));
    });

    // Start A2DP (raw)
    a2dp_source.start_raw("ESP32_A2DP", audioDataCallback);
    a2dp_source.set_volume(20);
    Serial.println("[MAIN] initBluetooth => ESP32_A2DP");
}

// ---------------------------------------------------------------------------
// startScan(), stopScan()
// ---------------------------------------------------------------------------
void startScan() {
    if (scanning) {
        Serial.println("[MAIN][BT] Already scanning");
        return;
    }
    discoveredDevices.clear();
    scanning = true;
    // 10s general inquiry
    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
    Serial.println("[MAIN][BT] Starting discovery (10s)...");
}

void stopScan() {
    if (!scanning) {
        Serial.println("[MAIN][BT] Called stopScan but was not scanning.");
        return;
    }
    esp_bt_gap_cancel_discovery();
    scanning = false;
    Serial.println("[MAIN][BT] Scanning stopped.");
}

// ---------------------------------------------------------------------------
// onDeviceFound()
// ---------------------------------------------------------------------------
void onDeviceFound(const DiscoveredDevice& dev) {
    // Check duplicates
    for (auto &d : discoveredDevices) {
        if (d.address == dev.address) {
            Serial.println("[MAIN][BT] Device is already in discoveredDevices, skipping.");
            return;
        }
    }
    discoveredDevices.push_back(dev);
    Serial.printf("[MAIN][BT] Discovered: %s (%s)\n", dev.name.c_str(), dev.address.c_str());
}

// ---------------------------------------------------------------------------
// GAP Callback
// ---------------------------------------------------------------------------
static void btGapCallback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT: {
        // Found device(s)
        char addrStr[18];
        sprintf(addrStr, "%02X:%02X:%02X:%02X:%02X:%02X",
                param->disc_res.bda[0], param->disc_res.bda[1],
                param->disc_res.bda[2], param->disc_res.bda[3],
                param->disc_res.bda[4], param->disc_res.bda[5]);

        String devName;
        for (int i = 0; i < param->disc_res.num_prop; i++) {
            esp_bt_gap_dev_prop_t *p = param->disc_res.prop + i;
            if (p->type == ESP_BT_GAP_DEV_PROP_BDNAME) {
                char name[249] = {0};
                memcpy(name, (char*)p->val, p->len);
                devName = String(name);
            }
        }
        DiscoveredDevice dd;
        dd.address = String(addrStr);
        dd.name    = devName;
        onDeviceFound(dd);
        break;
    }

    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: {
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            scanning = false;
            Serial.println("[MAIN][BT] Discovery stopped (callback).");
        } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
            Serial.println("[MAIN][BT] Discovery started (callback).");
        }
        break;
    }

    default:
        // If you want to see all other events, you can log them here
        break;
    }
}

// ---------------------------------------------------------------------------
// connectToDeviceByMac()
// ---------------------------------------------------------------------------
bool connectToDeviceByMac(const String& macStr) {
    Serial.printf("[MAIN][BT] connectToDeviceByMac called with %s\n", macStr.c_str());
    // Convert "XX:XX:XX:XX:XX:XX" => 6-byte array
    uint8_t mac[6];
    int vals[6];
    if (sscanf(macStr.c_str(), "%x:%x:%x:%x:%x:%x",
               &vals[0], &vals[1], &vals[2],
               &vals[3], &vals[4], &vals[5]) == 6) {
        for (int i = 0; i < 6; i++) {
            mac[i] = (uint8_t) vals[i];
        }
        // Now attempt to connect
        esp_err_t err = esp_a2d_source_connect(mac);
        if (err == ESP_OK) {
            Serial.println("[MAIN][BT] Connect request sent successfully.");
            return true;
        } else {
            Serial.printf("[MAIN][BT] Connect request failed: 0x%x\n", err);
            return false;
        }
    }
    Serial.println("[MAIN][BT] Could not parse MAC.");
    return false;
}