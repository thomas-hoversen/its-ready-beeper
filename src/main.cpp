#include <Arduino.h>
#include "MyA2DPSimple.h"
#include <SPIFFS.h>

// We'll read from /audio1.wav in SPIFFS
static File audioFile;

#define READAHEAD_SIZE 2048
static uint8_t readAheadBuffer[READAHEAD_SIZE];
static size_t  bufferDataLen  = 0;
static size_t  bufferReadPos  = 0;

static MyA2DPSimple a2dpSimple;

// LED pins
#define LED_BLUE      16
#define LED_ORANGE    17

// Button pin
#define BUTTON_PIN    25

// Flag for WAV playback
static bool playingAudio = false;

int32_t audioDataCallback(uint8_t* data, int32_t len) {
    if (!audioFile || !audioFile.available()) {
        return 0;
    }
    if (bufferReadPos >= bufferDataLen) {
        bufferDataLen = audioFile.read(readAheadBuffer, READAHEAD_SIZE);
        bufferReadPos = 0;
        if (bufferDataLen == 0) {
            return 0;
        }
    }
    int32_t available = bufferDataLen - bufferReadPos;
    int32_t toCopy    = (available < len) ? available : len;
    memcpy(data, readAheadBuffer + bufferReadPos, toCopy);
    bufferReadPos += toCopy;
    return toCopy;
}

void startAudioPlayback() {
    if (!a2dpSimple.isConnected()) {
        Serial.println("[MAIN] Not connected => skipping playback.");
        return;
    }
    if (audioFile) {
        Serial.println("[MAIN] Closing previously opened audio file...");
        audioFile.close();
    }
    audioFile = SPIFFS.open("/audio1.wav", "r");
    if (!audioFile) {
        Serial.println("[MAIN] Could not open /audio1.wav => silent streaming.");
        return;
    }
    size_t fileSize = audioFile.size();
    Serial.printf("[MAIN] Opened /audio1.wav, size=%u bytes\n", (unsigned)fileSize);

    if (!audioFile.seek(0, SeekSet)) {
        Serial.println("[MAIN] Failed to rewind => skip");
        audioFile.close();
        return;
    }
    bufferDataLen = 0;
    bufferReadPos = 0;
    playingAudio  = true;
    Serial.println("[MAIN] Starting audio playback...");
}

void handlePlaybackCompletion() {
    if (playingAudio && (!audioFile || !audioFile.available())) {
        Serial.println("[MAIN] Playback ended or file unavailable.");
        playingAudio = false;
        if (audioFile) {
            audioFile.close();
            Serial.println("[MAIN] audioFile closed after playback.");
        }
    }
}

void initSPIFFS() {
    if (!SPIFFS.begin(true)) {
        Serial.println("[MAIN] SPIFFS mount failed!");
        while (true) { delay(100); }
    }
    Serial.println("[MAIN] SPIFFS successfully mounted.");

    // Optional: list files
    Serial.println("[MAIN] Listing SPIFFS files:");
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    while (file) {
        Serial.printf("  -> %s (%llu bytes)\n", file.name(), file.size());
        file = root.openNextFile();
    }
    Serial.println("[MAIN] Done listing SPIFFS files.\n");
}

// -- For 3s bright LED
static bool     connectedLedActive  = false;
static uint32_t connectedLedStartMs = 0;

// Track previous connection state
static bool wasConnected = false;

// Fade variables
static int fadeValue     = 0;
static int fadeIncrement = 5;

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[MAIN] Starting...");

    pinMode(LED_ORANGE, OUTPUT);
    digitalWrite(LED_ORANGE, LOW);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Configure LEDC for the blue LED
    ledcSetup(0, 5000, 8);
    ledcAttachPin(LED_BLUE, 0);
    ledcWrite(0, 0);

    initSPIFFS();

    a2dpSimple.begin("MyESP32_Simple", audioDataCallback);
    Serial.println("[MAIN] Setup complete. Now scanning for devices...");
}

void loop() {
    // Check button
    if (digitalRead(BUTTON_PIN) == LOW) {
        if (!playingAudio) {
            Serial.println("[MAIN] Button => start playback...");
            startAudioPlayback();
        } else {
            Serial.println("[MAIN] Button => already playing => ignore.");
        }
    }
    handlePlaybackCompletion();

    // Orange LED => on if playing
    digitalWrite(LED_ORANGE, playingAudio ? HIGH : LOW);

    // Check states
    bool nowScanning   = a2dpSimple.isScanning();
    bool nowConnecting = a2dpSimple.isConnecting();
    bool nowConnected  = a2dpSimple.isConnected();

    // If just connected => 3s bright
    if (nowConnected && !wasConnected) {
        connectedLedActive  = true;
        connectedLedStartMs = millis();
        ledcWrite(0, 255);
        Serial.println("[MAIN] Blue LED => FULL BRIGHT (connected).");
    }

    // If in bright phase, check if 3s passed
    if (connectedLedActive) {
        if (millis() - connectedLedStartMs >= 3000) {
            connectedLedActive = false;
            // Turn off after 3s
            ledcWrite(0, 0);
            Serial.println("[MAIN] Blue LED => OFF (post-connection).");
        }
    }
    else {
        // If not in the 3s bright phase:
        // => fade if scanning or connecting
        if (nowScanning || nowConnecting) {
            fadeValue += fadeIncrement;
            if (fadeValue >= 255) {
                fadeValue = 255;
                fadeIncrement = -fadeIncrement;
            } else if (fadeValue <= 0) {
                fadeValue = 0;
                fadeIncrement = -fadeIncrement;
            }
            ledcWrite(0, fadeValue);
        } else {
            // Otherwise, LED off
            ledcWrite(0, 0);
        }
    }

    wasConnected = nowConnected;
    delay(10);
}