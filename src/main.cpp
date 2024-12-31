#include <Arduino.h>
#include "MyA2DPSimple.h"
#include <SPIFFS.h>

/**
 * We'll read from /audio1.wav in SPIFFS. But to reduce SPIFFS read latencies,
 * we will do a small read-ahead buffering in RAM. The A2DP callback then
 * pulls from that buffer rather than calling file.read() repeatedly.
 */
static File audioFile;

// Read-ahead buffer to smooth out SPIFFS access
#define READAHEAD_SIZE 2048
static uint8_t readAheadBuffer[READAHEAD_SIZE];
static size_t  bufferDataLen  = 0;   // how many bytes in buffer
static size_t  bufferReadPos  = 0;   // current read offset within buffer

// Our instance of MyA2DPSimple
static MyA2DPSimple a2dpSimple;

// LED pins
#define LED_BLUE      16
#define LED_ORANGE    17

// Button pin for triggering playback
#define BUTTON_PIN 25

// Flag to track whether we are currently playing audio (local WAV -> BT)
static bool playingAudio = false;

// ---------------------------------------------------------------------------
// A2DP audio data callback
// ---------------------------------------------------------------------------
int32_t audioDataCallback(uint8_t* data, int32_t len) {
    // If file isn't open or no data left in file, return 0 => signals "no more audio"
    if (!audioFile || !audioFile.available()) {
        return 0;
    }

    // If our read-ahead buffer is empty or we consumed it, fill it again
    if (bufferReadPos >= bufferDataLen) {
        bufferDataLen = audioFile.read(readAheadBuffer, READAHEAD_SIZE);
        bufferReadPos = 0;
        // If still zero => no more data
        if (bufferDataLen == 0) {
            return 0;
        }
    }

    // Calculate how many bytes we can provide from our buffer
    int32_t available = bufferDataLen - bufferReadPos;
    int32_t toCopy    = (available < len) ? available : len;

    // Copy from read-ahead buffer into the output "data" buffer
    memcpy(data, readAheadBuffer + bufferReadPos, toCopy);
    bufferReadPos += toCopy;

    return toCopy;
}

// ---------------------------------------------------------------------------
// Playback control
// ---------------------------------------------------------------------------
/**
 * @brief Attempts to open and rewind /audio1.wav from SPIFFS, then sets playingAudio = true
 */
void startAudioPlayback() {
    // Only proceed if we have a valid BT connection
    if (!a2dpSimple.isConnected()) {
        Serial.println("[MAIN] Not connected to any speaker => skipping playback.");
        return;
    }

    // If audioFile is already open, close it
    if (audioFile) {
        Serial.println("[MAIN] Closing previously opened audio file before playback...");
        audioFile.close();
    }

    // Re-open the audio file from SPIFFS
    audioFile = SPIFFS.open("/audio1.wav", "r");
    if (!audioFile) {
        Serial.println("[MAIN] Could not open /audio1.wav => streaming will be silent.");
        return;
    }

    // Optional: log file size
    size_t fileSize = audioFile.size();
    Serial.printf("[MAIN] Opened /audio1.wav, size=%u bytes\n", (unsigned)fileSize);

    // Rewind to start
    if (!audioFile.seek(0, SeekSet)) {
        Serial.println("[MAIN] Failed to rewind audio file => skip");
        audioFile.close();
        return;
    }

    // Reset the read-ahead buffer
    bufferDataLen = 0;
    bufferReadPos = 0;

    // Mark as playing
    playingAudio = true;
    Serial.println("[MAIN] Starting audio playback...");
}

/**
 * @brief Check if the audioFile is done. If so, stop playback and close the file.
 */
void handlePlaybackCompletion() {
    // If we're playing but the file is done or invalid, end playback
    if (playingAudio && (!audioFile || !audioFile.available())) {
        Serial.println("[MAIN] Playback ended or file unavailable.");
        playingAudio = false;
        if (audioFile) {
            audioFile.close();
            Serial.println("[MAIN] audioFile closed after playback.");
        }
    }
}

/**
 * @brief Initialize (format if needed) and optionally list files in SPIFFS
 */
void initSPIFFS() {
    if (!SPIFFS.begin(true)) {
        Serial.println("[MAIN] SPIFFS mount failed!");
        while (true) {
            delay(100);
        }
    }
    Serial.println("[MAIN] SPIFFS successfully mounted.");

    // Optional: list all files in SPIFFS
    Serial.println("[MAIN] Listing SPIFFS files:");
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    while (file) {
        Serial.printf("  -> %s (%llu bytes)\n", file.name(), file.size());
        file = root.openNextFile();
    }
    Serial.println("[MAIN] Done listing SPIFFS files.\n");
}

// ---------------------------------------------------------------------------
// Standard setup/loop
// ---------------------------------------------------------------------------

// Variables to handle the "connected LED" 3-second bright phase
static bool     connectedLedActive    = false;
static uint32_t connectedLedStartMs   = 0;

// Keep track of whether we were connected previously, for detecting new connection
static bool wasConnected = false;

// For the blue LED fading while scanning
static int   fadeValue      = 0;   // current PWM value for LED_BLUE
static int   fadeIncrement  = 5;   // how fast to fade in/out (bigger = faster)

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[MAIN] Starting...");

    // Initialize pins
    pinMode(LED_ORANGE, OUTPUT);
    digitalWrite(LED_ORANGE, LOW);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // --- Use LEDC for BLUE LED to achieve a fade effect ---
    // Channel 0, 5kHz, 8-bit resolution
    ledcSetup(0, 5000, 8);
    // Attach the BLUE pin to channel 0
    ledcAttachPin(LED_BLUE, 0);
    // Start with the blue LED off
    ledcWrite(0, 0);

    // Initialize SPIFFS so we can open an audio file
    initSPIFFS();

    // Initialize custom A2DP library (automatically starts scanning)
    a2dpSimple.begin("MyESP32_Simple", audioDataCallback);

    Serial.println("[MAIN] Setup complete. Now scanning for devices...");
}

void loop() {
    // Check if the user pressed the button => start playback
    if (digitalRead(BUTTON_PIN) == LOW) {
        if (!playingAudio) {
            Serial.println("[MAIN] Button pressed => starting playback...");
            startAudioPlayback();
        } else {
            Serial.println("[MAIN] Button pressed => already playing => ignoring...");
        }
    }

    // Check if we're done playing
    handlePlaybackCompletion();

    // Turn orange LED on/off depending on audio playback
    if (playingAudio) {
        digitalWrite(LED_ORANGE, HIGH);
    } else {
        digitalWrite(LED_ORANGE, LOW);
    }

    // Track scanning & connected states for the blue LED
    bool nowScanning  = a2dpSimple.isScanning();
    bool nowConnected = a2dpSimple.isConnected();

    // 1) Handle the 3-second "connected" bright LED if we just connected
    if (nowConnected && !wasConnected) {
        connectedLedActive  = true;
        connectedLedStartMs = millis();
        ledcWrite(0, 255);
        Serial.println("[MAIN] Blue LED => FULL BRIGHT (connected).");
    }

    // 2) Connected bright phase
    if (connectedLedActive) {
        if (millis() - connectedLedStartMs >= 3000) {
            connectedLedActive = false;
            ledcWrite(0, 0);
            Serial.println("[MAIN] Blue LED => OFF (post-connection).");
        }
    }
    // 3) Fade while scanning & not connected
    else {
        if (nowScanning && !nowConnected) {
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
            ledcWrite(0, 0);
        }
    }

    wasConnected = nowConnected;

    delay(10);
}