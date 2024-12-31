#include "MyA2DPSimple.h"
#include "BluetoothA2DPSource.h"
static const char* TAG = "MyA2DPSimple";

// Single static pointer to access from C callbacks
MyA2DPSimple* MyA2DPSimple::_instance = nullptr;

// -------------------------------------------------------
// Constructor / Destructor
// -------------------------------------------------------
MyA2DPSimple::MyA2DPSimple() {
    _instance = this;
}

MyA2DPSimple::~MyA2DPSimple() {
    end();
}

// -------------------------------------------------------
// Main begin()
// -------------------------------------------------------
void MyA2DPSimple::begin(const char* deviceName, MyAudioDataCB dataCB) {
    if (_initialized) {
        Serial.println("[MyA2DPSimple] Already initialized.");
        return;
    }

    Serial.println("STARTING BEGIN");

    _audioDataCB = dataCB;

    // ---------------------------------------------------
    // 1) NVS init
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // (Optional) Release BLE memory if you want:
    // ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    // ---------------------------------------------------
    // 2) Start the BT controller using bt_start()
    if (!btStart()) {
        Serial.println("[MyA2DPSimple] bt_start() failed => cannot proceed.");
        return;
    }

    // ---------------------------------------------------
    // 3) Initialize Bluedroid stack
    if (esp_bluedroid_init() != ESP_OK) {
        Serial.println("[MyA2DPSimple] Bluedroid init failed => cannot proceed.");
        return;
    }
    if (esp_bluedroid_enable() != ESP_OK) {
        Serial.println("[MyA2DPSimple] Bluedroid enable failed => cannot proceed.");
        return;
    }

    // ---------------------------------------------------
    // 4) Set local device name
    esp_bt_dev_set_device_name(deviceName);

    // 5) Register GAP callback
    esp_bt_gap_register_callback(&MyA2DPSimple::gapCallback);

    // 6) Initialize A2DP source
    esp_a2d_source_init();
    esp_a2d_register_callback(&MyA2DPSimple::a2dpCallback);
    esp_a2d_source_register_data_callback(&MyA2DPSimple::a2dpDataCallback);

    // ----------------------
    // (Option 6) If your SDK or Arduino core supports a direct SBC config call,
    // you can specify bitpool, sample rate, etc. For example:
    /*
    esp_a2d_sbc_config_t sbc_config = {
        .sample_freq = ESP_A2D_SBC_FREQ_44K, // 44.1kHz
        .block_len   = ESP_A2D_SBC_BLOCK_LEN_16,
        .num_subbands= ESP_A2D_SBC_SUBBAND_8,
        .alloc_method= ESP_A2D_SBC_ALLOC_METHOD_LOUDNESS,
        .channel_mode= ESP_A2D_SBC_CHANNEL_MODE_JOINT_STEREO,
        .min_bitpool = 2,
        .max_bitpool = 53 // default or can be changed
    };
    esp_a2d_source_set_sbc_params(&sbc_config);
    */
    // ----------------------

    // 7) Make device connectable/discoverable
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    // ---------------------------------------------------
    // 8) Create background task
    // Increase stack size a bit to handle heavier loads (option 2)
    _myTaskQueue = xQueueCreate(10, sizeof(my_bt_app_msg_t));
    if (xTaskCreatePinnedToCore(
            MyA2DPSimple::myTaskHandler,
            "myBtAppTask",
            8192, // bigger stack size for stability
            NULL,
            configMAX_PRIORITIES - 3, // fairly high priority
            &_myTaskHandle,
            0) != pdPASS)
    {
        Serial.println("[MyA2DPSimple] Could not create background task!");
    }

    // ---------------------------------------------------
    // Mark done & start scanning
    _initialized = true;
    Serial.println("[MyA2DPSimple] Initialization complete. Starting scan...");
    startScan();
}

// -------------------------------------------------------
// end()
// -------------------------------------------------------
void MyA2DPSimple::end() {
    if (!_initialized) return;

    stopScan();

    // Deinit A2DP
    esp_a2d_source_deinit();

    // Deinit Bluedroid
    esp_bluedroid_disable();
    esp_bluedroid_deinit();

    // Disable the BT controller
    esp_bt_controller_disable();
    esp_bt_controller_deinit();

    // Kill the background task
    if (_myTaskHandle) {
        vTaskDelete(_myTaskHandle);
        _myTaskHandle = nullptr;
    }
    if (_myTaskQueue) {
        vQueueDelete(_myTaskQueue);
        _myTaskQueue = nullptr;
    }

    _initialized = false;
    _connected   = false;
    _discovered.clear();
    Serial.println("[MyA2DPSimple] end() => Bluetooth shut down.");
}

// -------------------------------------------------------
// Start scanning
// -------------------------------------------------------
void MyA2DPSimple::startScan() {
    // (Option 5) do NOT scan if already connected
    if (!_initialized) {
        Serial.println("[MyA2DPSimple] Not initialized => no scan.");
        return;
    }
    if (_scanning) {
        Serial.println("[MyA2DPSimple] Already scanning!");
        return;
    }
    if (_connecting || _connected) {
        Serial.println("[MyA2DPSimple] Not starting scan (already connecting or connected).");
        return;
    }
    _discovered.clear();
    _scanning = true;
    Serial.println("[MyA2DPSimple] Starting inquiry scan for 10s...");
    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
}

// -------------------------------------------------------
// Stop scanning
// -------------------------------------------------------
void MyA2DPSimple::stopScan() {
    if (_scanning) {
        Serial.println("[MyA2DPSimple] Stop scanning...");
        esp_bt_gap_cancel_discovery();
        _scanning = false;
    }
}

// -------------------------------------------------------
// GAP Callback
// -------------------------------------------------------
void MyA2DPSimple::gapCallback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    if (_instance) _instance->handleGapEvent(event, param);
}

void MyA2DPSimple::handleGapEvent(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT: {
        // Found a device
        MyA2DPDevice dev;
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
            if (p->type == ESP_BT_GAP_DEV_PROP_BDNAME) {
                char n[249] = {0};
                memcpy(n, (char*)p->val, p->len);
                dev.name = String(n);
            } else if (p->type == ESP_BT_GAP_DEV_PROP_RSSI) {
                dev.rssi = *(int8_t*)(p->val);
            }
        }
        // Update or add
        bool found = false;
        for (auto &d : _discovered) {
            if (d.address == dev.address) {
                d.name = dev.name;
                d.rssi = dev.rssi;
                found  = true;
                break;
            }
        }
        if (!found) {
            _discovered.push_back(dev);
        }
        Serial.printf("[MyA2DPSimple][GAP] Found: '%s' (%s), RSSI=%d\n",
                      dev.name.c_str(), dev.address.c_str(), dev.rssi);
        break;
    }

    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: {
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            _scanning = false;
            Serial.println("[MyA2DPSimple] Discovery ended => connect to best device");
            // List all
            for (auto &dev : _discovered) {
                Serial.printf("    -> '%s' (%s), RSSI=%d\n",
                              dev.name.c_str(), dev.address.c_str(), dev.rssi);
            }
            // Dispatch an event to connect
            myWorkDispatch([](uint16_t e, void* p) {
                _instance->connectToBest();
            }, EVENT_CONNECT_BEST, nullptr, 0);
        }
        else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
            _scanning = true;
            Serial.println("[MyA2DPSimple] Discovery started...");
        }
        break;
    }

    default:
        break;
    }
}

// -------------------------------------------------------
// Connect to best device
// -------------------------------------------------------
void MyA2DPSimple::connectToBest() {
    if (_discovered.empty()) {
        Serial.println("[MyA2DPSimple] No devices found => do nothing");
        return;
    }
    // Find best RSSI
    int8_t bestRssi = -128;
    for (auto &d : _discovered) {
        if (d.rssi > bestRssi) {
            bestRssi = d.rssi;
            _bestDev = d;
        }
    }
    Serial.printf("[MyA2DPSimple] Attempting connect => '%s' (%s), RSSI=%d\n",
                  _bestDev.name.c_str(), _bestDev.address.c_str(), bestRssi);

    // Convert address
    uint8_t btAddr[6];
    if (sscanf(_bestDev.address.c_str(), "%x:%x:%x:%x:%x:%x",
               &btAddr[0], &btAddr[1], &btAddr[2],
               &btAddr[3], &btAddr[4], &btAddr[5]) == 6) {
        esp_bd_addr_t remoteBdAddr;
        memcpy(remoteBdAddr, btAddr, 6);
        _connecting = true;
        esp_err_t err = esp_a2d_source_connect(remoteBdAddr);
        if (err == ESP_OK) {
            Serial.printf("[MyA2DPSimple] Connect request => OK\n");
        } else {
            Serial.printf("[MyA2DPSimple] Connect request => FAILED (0x%x)\n", err);
            _connecting = false;
        }
    } else {
        Serial.println("[MyA2DPSimple] Could not parse MAC => no connect.");
    }
}

// -------------------------------------------------------
// A2DP Callback
// -------------------------------------------------------
void MyA2DPSimple::a2dpCallback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
    if (_instance) _instance->handleA2DPEvent(event, param);
}

void MyA2DPSimple::handleA2DPEvent(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
        if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
            _connecting = false;
            _connected  = true;
            Serial.println("[MyA2DPSimple] A2DP connected => streaming can start");

            // Force the speaker to begin playing (it has no Play button)
            esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY);
            esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);

        } else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            _connecting = false;
            _connected  = false;
            Serial.println("[MyA2DPSimple] A2DP disconnected => maybe re-scan");
            // If you want to automatically reconnect, do so here:
            startScan();
        }
        break;
    }
    case ESP_A2D_AUDIO_STATE_EVT: {
        if (param->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED) {
            Serial.println("[MyA2DPSimple] Audio started => streaming data now");
        } else if (param->audio_stat.state == ESP_A2D_AUDIO_STATE_STOPPED ||
                   param->audio_stat.state == ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND) {
            Serial.println("[MyA2DPSimple] Audio suspended/stopped");
        }
        break;
    }
    default:
        // Unhandled
        break;
    }
}

// -------------------------------------------------------
// Data Callback => must match esp_a2d_source_data_cb_t
// -------------------------------------------------------
int32_t MyA2DPSimple::a2dpDataCallback(uint8_t* data, int32_t len) {
    if (!_instance || !_instance->_audioDataCB) return 0;
    return dataCallbackImpl(data, len);
}

int32_t MyA2DPSimple::dataCallbackImpl(uint8_t* data, int32_t len) {
    if (!_instance || !_instance->_audioDataCB) return 0;
    return _instance->_audioDataCB(data, len);
}

// -------------------------------------------------------
// Background Task Handling
// -------------------------------------------------------
void MyA2DPSimple::myTaskHandler(void *arg) {
    my_bt_app_msg_t msg;
    for (;;) {
        if (_instance && xQueueReceive(_instance->_myTaskQueue, &msg, portMAX_DELAY)) {
            switch (msg.sig) {
            case MY_SIG_WORK_DISPATCH:
                _instance->myWorkDispatched(&msg);
                break;
            default:
                break;
            }
            if (msg.param) free(msg.param);
        }
    }
    vTaskDelete(NULL);
}

bool MyA2DPSimple::mySendMsg(my_bt_app_msg_t *msg) {
    if (!msg) return false;
    if (xQueueSend(_myTaskQueue, msg, 10 / portTICK_PERIOD_MS) != pdTRUE) {
        Serial.println("[MyA2DPSimple] xQueueSend failed!");
        return false;
    }
    return true;
}

void MyA2DPSimple::myWorkDispatched(my_bt_app_msg_t *msg) {
    if (msg->cb) {
        msg->cb(msg->event, msg->param);
    }
}

bool MyA2DPSimple::myWorkDispatch(my_bt_app_cb_t p_cback,
                                  uint16_t event, void *p_params, int param_len) {
    my_bt_app_msg_t msg;
    memset(&msg, 0, sizeof(my_bt_app_msg_t));

    msg.sig   = MY_SIG_WORK_DISPATCH;
    msg.event = event;
    msg.cb    = p_cback;

    if (param_len == 0) {
        return mySendMsg(&msg);
    } else if (p_params && param_len > 0) {
        msg.param = (void*)malloc(param_len);
        if (msg.param) {
            memcpy(msg.param, p_params, param_len);
            return mySendMsg(&msg);
        }
    }
    return false;
}