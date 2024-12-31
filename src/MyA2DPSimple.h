#pragma once

#include <Arduino.h>
#include <vector>
#include <esp_bt.h>
#include <esp_gap_bt_api.h>
#include <esp_a2dp_api.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include <nvs_flash.h>

// For background tasks and event dispatch
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// Simple struct for discovered device info
struct MyA2DPDevice {
    String name;
    String address;
    int8_t rssi;
};

// Callback type for incoming A2DP data requests
typedef int32_t (*MyAudioDataCB)(uint8_t* data, int32_t len);

// For event queue
typedef void (*my_bt_app_cb_t)(uint16_t event, void *p_param);

// Simple message structure for queue
struct my_bt_app_msg_t {
    uint16_t    sig;       // Event signal
    uint16_t    event;     // Message event
    my_bt_app_cb_t cb;     // Context switch callback
    void        *param;    // parameter area needs to be last
};

class MyA2DPSimple {
public:
    MyA2DPSimple();
    ~MyA2DPSimple();

    // Initialize the stack, start scanning, etc.
    void begin(const char* deviceName, MyAudioDataCB dataCB);
    // Disconnect, stop scanning, and clean up
    void end();

    // Check if we are currently connected to a remote device
    bool isConnected() const;
    // Check if we are currently scanning
    bool isScanning() const;

    // Start scanning for nearby devices
    void startScan();
    // Stop scanning if in progress
    void stopScan();

private:
    // GAP events
    static void gapCallback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
    void handleGapEvent(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);

    // A2DP events
    static void a2dpCallback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
    void handleA2DPEvent(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);

    // Must match esp_a2d_source_data_cb_t
    static int32_t a2dpDataCallback(uint8_t *data, int32_t len);
    static int32_t dataCallbackImpl(uint8_t* data, int32_t len);

    // Internal background task logic
    static void myTaskHandler(void *arg);
    bool mySendMsg(my_bt_app_msg_t *msg);
    void myWorkDispatched(my_bt_app_msg_t *msg);
    bool myWorkDispatch(my_bt_app_cb_t p_cback, uint16_t event, void *p_params, int param_len);

    // Device discovery => connect to the best discovered device
    void connectToBest();

private:
    static MyA2DPSimple* _instance;

    bool _initialized = false;
    bool _scanning    = false;
    bool _connecting  = false;
    bool _connected   = false;

    std::vector<MyA2DPDevice> _discovered;
    MyA2DPDevice              _bestDev;

    MyAudioDataCB _audioDataCB = nullptr;

    // Our background task
    TaskHandle_t  _myTaskHandle   = nullptr;
    QueueHandle_t _myTaskQueue    = nullptr;

    // Some placeholders for signals
    static const uint16_t MY_SIG_WORK_DISPATCH = 0x01;
    static const uint16_t EVENT_CONNECT_BEST   = 0x02;
};