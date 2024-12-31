#include <Arduino.h>
#include "MyA2DPSimple.h"
#include <SPIFFS.h>

/**
 * We'll read from /audio1.wav in SPIFFS. But to reduce SPIFFS read latencies,
 * we will do a small read-ahead buffering in RAM. The A2DP callback then
 * pulls from that buffer rather than calling file.read() repeatedly.
 */
static File audioFile;

// Read-ahead buffer to smooth out SPIFFS access (option 3)
#define READAHEAD_SIZE 2048
static uint8_t readAheadBuffer[READAHEAD_SIZE];
static size_t  bufferDataLen  = 0;   // how many bytes in buffer
static size_t  bufferReadPos  = 0;   // current read offset within buffer

// Our instance of MyA2DPSimple
static MyA2DPSimple a2dpSimple;

// For a simple LED blink when playback starts
#define LED_PIN    2
// Button pin for triggering playback
#define BUTTON_PIN 25
// Flag to track whether we are currently playing audio
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

    // **Minimal debug** to avoid overhead (option 2)
    // Serial.printf("[audioDataCallback] Requested=%d, Provided=%d\n", len, toCopy);

    return toCopy;
}

// ---------------------------------------------------------------------------
// Playback control
// ---------------------------------------------------------------------------
void flashLED(int pin, int durationMs) {
    digitalWrite(pin, HIGH);
    delay(durationMs);
    digitalWrite(pin, LOW);
}

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

    // Quick LED blink to show playback triggered
    flashLED(LED_PIN, 300);
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
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[MAIN] Starting...");

    // Initialize pins
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Initialize SPIFFS so we can open an audio file
    initSPIFFS();

    // Initialize custom A2DP library
    // (This begins scanning and will eventually connect automatically)
    a2dpSimple.begin("MyESP32_Simple", audioDataCallback);

    Serial.println("[MAIN] Setup complete. Now scanning for devices...");
}

void loop() {
    // Check if the user pressed the button => start playback
    if (digitalRead(BUTTON_PIN) == LOW) {
        Serial.println("[MAIN] Button pressed => starting playback...");
        startAudioPlayback();
    }

    // Check if we're done playing
    handlePlaybackCompletion();

    // Minimal delay to allow other tasks to run (option 2)
    delay(10);
}