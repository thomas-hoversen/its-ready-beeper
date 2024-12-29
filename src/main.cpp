/*****************************************************
 * Combined Code - Forcing Media Start, with updated onAudioStateChanged
 * 
 * Logic:
 * - We keep scanning logic, beep detection, etc.
 * - Once the device is "connected", we call
 *    esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY)
 *    esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START)
 *   to push data to the speaker if it never requests it.
 * 
 * - The onAudioStateChanged(...) callback logs when the
 *   speaker transitions states: STARTED, REMOTE_SUSPEND,
 *   or SUSPENDED.
 *****************************************************/

#include <Arduino.h>
#include <SPIFFS.h>
#include <arduinoFFT.h>
#include <esp_task_wdt.h>
#include <driver/i2s.h>
#include <esp_bt.h>
#include "BluetoothA2DPSource.h"
#include "BeepDetector.h"
#include "BeepHistory.h"

// ------------------------------------------------------
// Configuration Constants
// ------------------------------------------------------

// I2S microphone pins
#define I2S_SCK   14
#define I2S_WS    15
#define I2S_SD    34
#define I2S_PORT  I2S_NUM_0

static const int   I2S_SAMPLE_RATE       = 16000;
static const bool  USE_APLL              = false;
static const bool  TX_DESC_AUTO_CLEAR    = false;
static const int   FIXED_MCLK            = 0;
static const int   I2S_DMA_BUF_COUNT     = 4;
static const int   I2S_DMA_BUF_LEN       = 128;

// FFT & Beep Detection
#define SAMPLES                    512
#define SAMPLING_FREQUENCY         16000
#define FREQUENCY_MIN              1200
#define FREQUENCY_MAX              9000
static float BIN_AMPLITUDE_THRESHOLD     = 10;
static float AMPLITUDE_THRESHOLD         = 3000;
static const int   I2S_READ_CHUNK        = 64;
static const int   MIC_PROXIMITY_THRESHOLD = 100;

// Consecutive-window detection
static const int   MULTI_WINDOW_CONFIDENCE = 4;
static const unsigned long DETECTION_DELAY  = 1000;  
static const unsigned long TIME_WINDOW_MS   = 5000;
static const int   THRESHOLD_COUNT          = 3;

// LED / Button
#define LED_PIN           2
#define BUTTON_PIN        25
static const int   LED_FLASH_DURATION     = 500;

// ------------------------------------------------------
// Instantiate beep logic
// ------------------------------------------------------
BeepDetector beepDetector(MULTI_WINDOW_CONFIDENCE, DETECTION_DELAY);
BeepHistory  beepHistory(THRESHOLD_COUNT, TIME_WINDOW_MS);

// FFT arrays
#include <arduinoFFT.h>
float vReal[SAMPLES];
float vImag[SAMPLES];
ArduinoFFT<float> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

// ------------------------------------------------------
// Subclass so we can call protected esp_a2d_connect(...)
// ------------------------------------------------------
class MyA2DPSource : public BluetoothA2DPSource {
public:
  esp_err_t connectDevice(const esp_bd_addr_t peer_addr) {
    esp_bd_addr_t mutableAddr;
    memcpy(mutableAddr, peer_addr, sizeof(esp_bd_addr_t));
    return esp_a2d_connect(mutableAddr); 
  }
};

// Structure to store discovered device info
struct DiscoveredDevice {
    String address;
    String name;
    int8_t rssi;
};

// ------------------------------------------------------
// Global scanning & connection state
// ------------------------------------------------------
MyA2DPSource a2dp_source;  

static std::vector<DiscoveredDevice> discoveredDevices;
static bool isDiscoveryActive   = false;
static bool haveLastBestDevice  = false;
static bool connectInProgress   = false;
static DiscoveredDevice lastBestDevice;

// ------------------------------------------------------
// Audio file (pre-downloaded in SPIFFS)
// ------------------------------------------------------
File audioFile;  
const char* audioFilePath = "/audio1.wav";
bool playingAudio         = false;
unsigned long ignoreBeepUntil = 0;
int16_t maxSampleInFrame  = 0;

// ------------------------------------------------------
// Forward declarations
// ------------------------------------------------------
void initPins();
void initWatchdogTimer();
void initSPIFFS();
void initI2S();
void initBluetooth();

void startScanning();
void stopScanning();
void connectToBestDevice();
void gapCallback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);

void startAudioPlayback();
void handlePlaybackCompletion();

void collectAudioSamples();
void runFFT();
bool detectBeep();
void handleBeepDetection();
int32_t audioDataCallback(uint8_t* data, int32_t len);

// Additional callbacks
static void onConnectionStateChanged(esp_a2d_connection_state_t state, void*);
static void onAudioStateChanged(esp_a2d_audio_state_t state, void*);

// ------------------------------------------------------
// Setup
// ------------------------------------------------------
void setup() {
    Serial.begin(115200);
    Serial.println("\n\n[MAIN] Initializing...");

    initPins();
    initWatchdogTimer();
    initSPIFFS();
    initI2S();
    initBluetooth();

    startScanning(); // Start scanning once
    Serial.println("[MAIN] Setup complete. Will scan until connected.");
}

// ------------------------------------------------------
// Main loop
// ------------------------------------------------------
void loop() {
    esp_task_wdt_reset();
    delay(10);

    if (digitalRead(BUTTON_PIN) == LOW) {
        Serial.println("[MAIN] Button pressed => immediate playback");
        startAudioPlayback();
    }

    handlePlaybackCompletion();

    // Only run beep detection if connected
    if (a2dp_source.is_connected()) {
        collectAudioSamples();
        runFFT();
        handleBeepDetection();
    }
}

// ------------------------------------------------------
// Start scanning (10s inquiry)
// ------------------------------------------------------
void startScanning() {
    if (connectInProgress || a2dp_source.is_connected()) {
        Serial.println("[MAIN] Not starting scan (already connecting or connected).");
        return;
    }
    Serial.println("[MAIN] Starting 10s inquiry scan...");
    discoveredDevices.clear();
    haveLastBestDevice = false;
    isDiscoveryActive   = true;

    esp_bt_gap_register_callback(gapCallback);
    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
}

// ------------------------------------------------------
// Stop scanning
// ------------------------------------------------------
void stopScanning() {
    if (isDiscoveryActive) {
        Serial.println("[MAIN] Stop scanning...");
        esp_bt_gap_cancel_discovery();
        isDiscoveryActive = false;
    }
}

// ------------------------------------------------------
// Connect to best (highest RSSI) device
// ------------------------------------------------------
void connectToBestDevice() {
    if (discoveredDevices.empty()) {
        Serial.println("[MAIN] No devices found => let's scan again later...");
        isDiscoveryActive = false;
        return;
    }

    DiscoveredDevice bestDev;
    int8_t bestRssi = -128;
    for (auto &d : discoveredDevices) {
        if (d.rssi > bestRssi) {
            bestRssi = d.rssi;
            bestDev  = d;
        }
    }

    Serial.println("[MAIN] Summary of discovered devices:");
    for (auto &dev : discoveredDevices) {
        Serial.printf("    -> '%s' (%s), RSSI=%d\n", dev.name.c_str(), dev.address.c_str(), dev.rssi);
    }

    Serial.printf("[MAIN] Best device: '%s' (%s), RSSI=%d\n",
                  bestDev.name.c_str(), bestDev.address.c_str(), bestDev.rssi);

    uint8_t btAddr[6];
    if (sscanf(bestDev.address.c_str(), "%x:%x:%x:%x:%x:%x",
               &btAddr[0], &btAddr[1], &btAddr[2],
               &btAddr[3], &btAddr[4], &btAddr[5]) == 6) {
        stopScanning();
        connectInProgress = true;

        esp_bd_addr_t remoteBdAddr;
        memcpy(remoteBdAddr, btAddr, 6);

        esp_err_t err = a2dp_source.connectDevice(remoteBdAddr);
        if (err == ESP_OK) {
            lastBestDevice     = bestDev;
            haveLastBestDevice = true;
            Serial.printf("[MAIN] Connection request sent to '%s' (%s)\n",
                          bestDev.name.c_str(), bestDev.address.c_str());
        } else {
            connectInProgress = false;
            Serial.printf("[MAIN] Connection request FAILED (0x%x)\n", err);
        }
    } else {
        Serial.println("[MAIN] Could not parse MAC => no connect");
    }
}

// ------------------------------------------------------
// GAP Callback
// ------------------------------------------------------
void gapCallback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
        case ESP_BT_GAP_DISC_RES_EVT: {
            DiscoveredDevice dev;
            char addrStr[18] = {0};
            sprintf(addrStr, "%02X:%02X:%02X:%02X:%02X:%02X",
                    param->disc_res.bda[0], param->disc_res.bda[1],
                    param->disc_res.bda[2], param->disc_res.bda[3],
                    param->disc_res.bda[4], param->disc_res.bda[5]);

            dev.address = String(addrStr);
            dev.name    = "";
            dev.rssi    = -128;

            for (int i = 0; i < param->disc_res.num_prop; i++) {
                esp_bt_gap_dev_prop_t *p = &param->disc_res.prop[i];
                switch (p->type) {
                    case ESP_BT_GAP_DEV_PROP_BDNAME: {
                        char tempName[249] = {0};
                        memcpy(tempName, (char*)p->val, p->len);
                        dev.name = String(tempName);
                        break;
                    }
                    case ESP_BT_GAP_DEV_PROP_RSSI: {
                        dev.rssi = *(int8_t*)(p->val);
                        break;
                    }
                    default:
                        break;
                }
            }

            bool alreadyStored = false;
            for (auto &existing : discoveredDevices) {
                if (existing.address == dev.address) {
                    existing.name = dev.name;
                    existing.rssi = dev.rssi;
                    alreadyStored  = true;
                    break;
                }
            }
            if (!alreadyStored) {
                discoveredDevices.push_back(dev);
            }

            Serial.printf("[MAIN][GAP] Found device: '%s' (%s) RSSI=%d\n",
                          dev.name.c_str(), dev.address.c_str(), dev.rssi);
            break;
        }

        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: {
            if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
                Serial.println("[MAIN][GAP] Discovery ended => connect attempt");
                isDiscoveryActive = false;
                connectToBestDevice();
            }
            else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
                Serial.println("[MAIN][GAP] Discovery started...");
                isDiscoveryActive = true;
            }
            break;
        }

        default:
            break;
    }
}

// ------------------------------------------------------
// onConnectionStateChanged
// ------------------------------------------------------
static void onConnectionStateChanged(esp_a2d_connection_state_t state, void*) {
    if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
        connectInProgress = false;
        Serial.println("[MAIN] A2DP Source is now connected!");

        if (haveLastBestDevice) {
            Serial.printf("[MAIN] Connected to device: '%s' (%s), RSSI=%d\n",
                          lastBestDevice.name.c_str(),
                          lastBestDevice.address.c_str(),
                          lastBestDevice.rssi);
        }

        // Force A2DP media to start if speaker doesn't request
        esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY);
        esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);

    }
    else if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
        connectInProgress = false;
        Serial.println("[MAIN] Disconnected from speaker.");
        startScanning(); // re-scan
    }
}

// ------------------------------------------------------
// onAudioStateChanged
// ------------------------------------------------------
static void onAudioStateChanged(esp_a2d_audio_state_t state, void*) {
    // We want to log STARTED, REMOTE_SUSPEND, or SUSPENDED
    const char* stateStr = (state == ESP_A2D_AUDIO_STATE_STARTED)         ? "STARTED" :
                           (state == ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND) ? "REMOTE_SUSPEND" :
                           (state == ESP_A2D_AUDIO_STATE_STOPPED)        ? "STOPPED" :
                                                                           "UNKNOWN";
    Serial.printf("[MAIN] Audio state changed => %s\n", stateStr);

    if (state == ESP_A2D_AUDIO_STATE_STARTED) {
        Serial.println("[MAIN] Speaker has started requesting audio => streaming data now!");
    }
}

// ------------------------------------------------------
// Audio data callback
// ------------------------------------------------------
int32_t audioDataCallback(uint8_t* data, int32_t len) {
    if (!audioFile) {
        Serial.println("[MAIN][audioDataCallback] audioFile is NULL => returning 0");
        return 0;
    }
    if (!audioFile.available()) {
        Serial.println("[MAIN][audioDataCallback] audioFile has no more data => returning 0");
        return 0;
    }

    size_t bytesRead = audioFile.read(data, len);
    Serial.printf("[MAIN][audioDataCallback] Requested=%d, Read=%d\n", len, bytesRead);

    return (int32_t)bytesRead;
}

// ------------------------------------------------------
// Start playback
// ------------------------------------------------------
void startAudioPlayback() {
    if (!a2dp_source.is_connected()) {
        Serial.println("[MAIN] No Bluetooth connection => skip playback.");
        return;
    }
    if (audioFile) {
        Serial.println("[MAIN] Closing previously opened audio file before playback...");
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
    Serial.printf("[MAIN] audioFile.size()=%u\n", (unsigned)audioFile.size());

    digitalWrite(LED_PIN, HIGH);
    delay(LED_FLASH_DURATION);
    digitalWrite(LED_PIN, LOW);

    // Optionally, force media start here again if you want:
    // esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY);
    // esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);
}

// ------------------------------------------------------
// Handle playback completion
// ------------------------------------------------------
void handlePlaybackCompletion() {
    if (!playingAudio) return;

    if (audioFile) {
        uint32_t remain = audioFile.available();
        if (remain < 2000) {
            Serial.printf("[MAIN] handlePlaybackCompletion => file.available=%u\n", remain);
        }
    }

    if (playingAudio && (!audioFile || !audioFile.available())) {
        Serial.println("[MAIN] Playback ended.");
        playingAudio = false;
        if (audioFile) {
            audioFile.close();
            Serial.println("[MAIN] audioFile closed after playback.");
        }
        ignoreBeepUntil = millis() + 1500;
    }
}

// ------------------------------------------------------
// Mic reading for beep detection
// ------------------------------------------------------
void collectAudioSamples() {
    maxSampleInFrame = 0;
    int count = 0;

    while (count < SAMPLES) {
        esp_task_wdt_reset();
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
        }
    }
}

// ------------------------------------------------------
// Run FFT
// ------------------------------------------------------
void runFFT() {
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();
}

// ------------------------------------------------------
// Detect beep
// ------------------------------------------------------
bool detectBeep() {
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
    return beepDetected;
}

// ------------------------------------------------------
// Handle beep detection
// ------------------------------------------------------
void handleBeepDetection() {
    static bool beepDetectionLogged = false;
    if (!beepDetectionLogged) {
        Serial.println("[MAIN] Beep detection has started (connected).");
        beepDetectionLogged = true;
    }
    if (playingAudio) return;
    if (millis() < ignoreBeepUntil) return;

    bool beepInThisWindow = detectBeep();
    bool beepEvent = beepDetector.update(beepInThisWindow, millis());
    if (beepEvent) {
        Serial.println("[MAIN] -> A new beep event just got confirmed!");
        beepHistory.addBeep(millis());

        if (beepHistory.hasReachedThreshold()) {
            Serial.println("[MAIN] beepHistory => threshold => Start playback!");
            beepHistory.clear();
            startAudioPlayback();
        } else {
            Serial.println("[MAIN] Not enough beep events yet...");
        }
    }
}

// ------------------------------------------------------
// Pin & Peripherals Init
// ------------------------------------------------------
void initPins() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    digitalWrite(LED_PIN, LOW);
    Serial.println("[MAIN] Pins init.");
}

void initWatchdogTimer() {
    esp_task_wdt_init(15, true);
    esp_task_wdt_add(NULL);
    Serial.println("[MAIN] WDT init (15s).");
}

void initSPIFFS() {
    if (!SPIFFS.begin(true)) {
        Serial.println("[MAIN] SPIFFS mount failed!");
        while (1) { delay(100); }
    }
    Serial.println("[MAIN] SPIFFS mounted.");
    File root = SPIFFS.open("/");
    File f = root.openNextFile();
    while (f) {
        Serial.printf("[MAIN] File: %s\n", f.name());
        f = root.openNextFile();
    }
}

void initI2S() {
    Serial.println("[MAIN][I2S] Installing driver...");
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format =
            (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = 0,
        .dma_buf_count    = I2S_DMA_BUF_COUNT,
        .dma_buf_len      = I2S_DMA_BUF_LEN,
        .use_apll         = USE_APLL,
        .tx_desc_auto_clear = TX_DESC_AUTO_CLEAR,
        .fixed_mclk       = FIXED_MCLK
    };

    esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("[MAIN][I2S] driver_install failed: %d\n", err);
        while(true){ delay(100); }
    }
    i2s_pin_config_t pinConfig = {
        .bck_io_num     = I2S_SCK,
        .ws_io_num      = I2S_WS,
        .data_out_num   = -1,
        .data_in_num    = I2S_SD
    };

    err = i2s_set_pin(I2S_PORT, &pinConfig);
    if(err != ESP_OK){
        Serial.printf("[MAIN][I2S] set_pin failed: %d\n", err);
        while(true){ delay(100); }
    }

    i2s_set_clk(I2S_PORT,
                I2S_SAMPLE_RATE,
                I2S_BITS_PER_SAMPLE_32BIT,
                I2S_CHANNEL_MONO);
    i2s_start(I2S_PORT);
    Serial.println("[MAIN][I2S] done.");
}

void initBluetooth() {
    Serial.println("[MAIN] BT init...");

    // Start the A2DP source
    a2dp_source.start_raw("ITSREADY", audioDataCallback);
    a2dp_source.set_volume(20);

    // Provide robust logs on connection state changes
    a2dp_source.set_on_connection_state_changed(onConnectionStateChanged);

    // Also track audio state => print out STARTED or REMOTE_SUSPEND or STOPPED
    a2dp_source.set_on_audio_state_changed(onAudioStateChanged);

    Serial.println("[MAIN] BT init done. A2DP source started as 'C17A'.");
}