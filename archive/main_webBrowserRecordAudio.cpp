/**
 * ================================================================
 * NOTE ON BROWSER MICROPHONE ACCESS (HTTP BLOCKING):
 * ------------------------------------------------
 * Many modern browsers (especially on iOS Safari) REQUIRE HTTPS
 * or a secure context to allow microphone access via getUserMedia().
 * 
 * Because this example serves a page over plain HTTP from 192.168.4.x,
 * the browser may BLOCK microphone access by default, preventing
 * actual recording. You might see an error like:
 * 
 *   "Could not access microphone. Check permissions."
 * 
 * WORKAROUNDS:
 *   1) Use a browser/device that allows mic on HTTP (rare nowadays).
 *   2) Override secure-origin policies in desktop Chrome (not recommended).
 *   3) Serve this page via HTTPS with a valid (or self-signed) cert
 *      to make browsers allow microphone usage.
 * 
 * In short, if recording does NOT work for you, it's due to this
 * HTTP vs. HTTPS security requirement. The rest of the code
 * for recording, uploading, decoding, and Bluetooth playback
 * should function if the mic is actually permitted.
 * ================================================================
 *
 * Demonstration of a single-page approach (simplified):
 *  - ESP32 serves a single "Record" page in AP mode (no static IP, no DNS).
 *  - The page uses MediaRecorder to record up to 4s of audio (MP3),
 *    or until a "Stop" button is clicked.
 *  - Playback in the browser before upload.
 *  - Upload that file to ESP32.
 *  - Decode from MP3 -> RAW using Helix MP3 calls.
 *  - Store raw data in SPIFFS.
 *  - Play that raw data via Bluetooth A2DP on a connected speaker.
 *  - Logging each step.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include "BluetoothA2DPSource.h"

// AudioTools & Helix MP3
#include <AudioTools.h>
#include <AudioTools/AudioCodecs/CodecMP3Helix.h>
#include "libhelix-mp3/mp3dec.h"

using namespace audio_tools;

/**
 * Some Helix libraries don’t define MP3_MAX_OUTPUT_SAMPLES by default.
 * According to Helix docs, a single MP3 frame in stereo can produce up to
 * 1152 samples per channel → 2304 total samples.
 * So we define 1152, and in the code we multiply by 2 for stereo.
 */
#ifndef MP3_MAX_OUTPUT_SAMPLES
#define MP3_MAX_OUTPUT_SAMPLES 1152
#endif

// Wi-Fi AP credentials
const char* AP_SSID     = "ESP_Audio";
const char* AP_PASSWORD = "12345678";  

// Bluetooth device name
const char* BT_DEVICE_NAME = "C17A";

// SPIFFS file paths
const char* TEMP_MP3_PATH  = "/temp_compressed.mp3";
const char* FINAL_RAW_PATH = "/uploaded_audio.raw";

// Webserver on port 80
AsyncWebServer server(80);

// Bluetooth A2DP
BluetoothA2DPSource a2dp_source;

// Playback state
static bool playing = false;
static File audioFile;

// ----------------------------------------------------------------------------
// Log helper
void logMsg(const String &msg) {
  Serial.println(msg);
}

// ----------------------------------------------------------------------------
// A2DP audio data callback
int32_t audioDataCallback(uint8_t* data, int32_t len) {
  if (!audioFile || !audioFile.available()) {
    return 0; // no more data
  }
  return audioFile.read(data, len);
}

// ----------------------------------------------------------------------------
// Start playing from RAW file
void startAudioPlayback() {
  logMsg("[startAudioPlayback] Attempting playback...");

  if (!a2dp_source.is_connected()) {
    logMsg("[startAudioPlayback] No Bluetooth speaker connected!");
    return;
  }

  audioFile = SPIFFS.open(FINAL_RAW_PATH, "r");
  if (!audioFile) {
    logMsg("[startAudioPlayback] ERROR: Could not open final raw file!");
    return;
  }

  playing = true;
  logMsg("[startAudioPlayback] Now playing from SPIFFS raw file...");
}

// ----------------------------------------------------------------------------
// Check if audio playback finished
void checkAudioFinished() {
  if (playing && (!audioFile || !audioFile.available())) {
    logMsg("[checkAudioFinished] Playback complete.");
    audioFile.close();
    playing = false;
  }
}

// ----------------------------------------------------------------------------
// Helix decode from MP3 -> RAW
void decodeMP3toRaw(const char* inPath, const char* outPath) {
  logMsg(String("[decodeMP3toRaw] Decoding from ") + inPath + " -> " + outPath);

  if (SPIFFS.exists(outPath)) {
    SPIFFS.remove(outPath);
    logMsg("[decodeMP3toRaw] Removed old output file.");
  }

  File inFile = SPIFFS.open(inPath, "r");
  if (!inFile) {
    logMsg("[decodeMP3toRaw] ERROR: Could not open input file!");
    return;
  }

  File outFile = SPIFFS.open(outPath, "w");
  if (!outFile) {
    logMsg("[decodeMP3toRaw] ERROR: Could not open output file!");
    inFile.close();
    return;
  }

  HMP3Decoder hDecoder = MP3InitDecoder();
  if (!hDecoder) {
    logMsg("[decodeMP3toRaw] ERROR: MP3InitDecoder failed!");
    inFile.close();
    outFile.close();
    return;
  }

  const size_t READ_SIZE = 1024;
  uint8_t inputBuf[READ_SIZE];
  int16_t outputBuf[MP3_MAX_OUTPUT_SAMPLES * 2]; // stereo

  while (true) {
    if (!inFile.available()) {
      break;
    }

    int bytesRead = inFile.read(inputBuf, READ_SIZE);
    if (bytesRead <= 0) {
      break;
    }

    int offset = 0;
    while (true) {
      int frameOffset = MP3FindSyncWord(&inputBuf[offset], bytesRead - offset);
      if (frameOffset < 0) {
        break;
      }
      offset += frameOffset;

      unsigned char* dataPtr = &inputBuf[offset];
      int bytesLeft = bytesRead - offset;

      int err = MP3Decode(hDecoder, &dataPtr, (int*)&bytesLeft, outputBuf, 0);
      if (err != 0) {
        if (err == ERR_MP3_INDATA_UNDERFLOW) {
          // Need more data
          break;
        }
        // Skip this frame
        break;
      }

      offset = bytesRead - bytesLeft;

      MP3FrameInfo frameInfo;
      MP3GetLastFrameInfo(hDecoder, &frameInfo);

      outFile.write((const uint8_t*)outputBuf, frameInfo.outputSamps * sizeof(int16_t));

      if (offset >= bytesRead) {
        break;
      }
    }
  }

  MP3FreeDecoder(hDecoder);
  inFile.close();
  outFile.close();

  logMsg("[decodeMP3toRaw] Decode complete. Output in SPIFFS.");
}

// ----------------------------------------------------------------------------
// HTML/JS for single-page recorder
const char record_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>Audio Recorder</title>
</head>
<body>
<h1>Audio Recorder</h1>
<p>Record up to 4 seconds, or press Stop whenever you want.</p>

<button id="btnStart">Start Recording</button>
<button id="btnStop" disabled>Stop Recording</button>
<button id="btnPlay" disabled>Play in Browser</button>
<button id="btnUpload" disabled>Upload to ESP32</button>
<button id="btnESP32Play">Play via Bluetooth</button>

<audio id="audioPlayback" controls style="display:none;"></audio>

<script>
let mediaRecorder;
let audioChunks = [];
let audioBlob;
let timeoutID;

const btnStart    = document.getElementById('btnStart');
const btnStop     = document.getElementById('btnStop');
const btnPlay     = document.getElementById('btnPlay');
const btnUpload   = document.getElementById('btnUpload');
const btnESP32Play= document.getElementById('btnESP32Play');
const audioPlayer = document.getElementById('audioPlayback');

async function initMedia() {
  // Request audio from mic
  const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
  // Attempt MP3 if supported
  const mimeType = 'audio/mp3';
  if (!MediaRecorder.isTypeSupported(mimeType)) {
    alert('MP3 not supported. Using default...');
  }
  mediaRecorder = new MediaRecorder(stream, { mimeType });

  mediaRecorder.ondataavailable = e => {
    audioChunks.push(e.data);
  };

  mediaRecorder.onstop = () => {
    // Combine chunks
    audioBlob = new Blob(audioChunks, { type: mediaRecorder.mimeType });
    audioChunks = [];

    // Show in browser
    const audioUrl = URL.createObjectURL(audioBlob);
    audioPlayer.src = audioUrl;
    audioPlayer.style.display = 'block';

    btnPlay.disabled = false;
    btnUpload.disabled = false;
    btnStop.disabled = true;
    btnStart.disabled = false;
    btnStart.textContent = "Start Recording";
  };

  btnStart.onclick = () => {
    startRecording();
    // auto-stop after 4s
    if (timeoutID) clearTimeout(timeoutID);
    timeoutID = setTimeout(() => {
      if (mediaRecorder.state === 'recording') {
        stopRecording();
      }
    }, 4000);
  };

  btnStop.onclick = () => {
    if (mediaRecorder.state === 'recording') {
      stopRecording();
    }
  };

  btnPlay.onclick = () => {
    audioPlayer.play();
  };

  btnUpload.onclick = async () => {
    if (!audioBlob) return;
    btnUpload.disabled = true;
    btnUpload.textContent = "Uploading...";
    try {
      const formData = new FormData();
      formData.append('audiofile', audioBlob, 'recorded.mp3');
      const res = await fetch('/upload', {
        method: 'POST',
        body: formData
      });
      const txt = await res.text();
      alert('Server says:\n' + txt);
    } catch (err) {
      console.error(err);
      alert('Upload failed');
    }
    btnUpload.textContent = "Upload to ESP32";
    btnUpload.disabled = false;
  };

  btnESP32Play.onclick = async () => {
    await fetch('/play');
    alert('Asked ESP32 to play over Bluetooth. Check speaker connection.');
  };
}

function startRecording() {
  if (mediaRecorder.state === 'inactive') {
    audioChunks = [];
    mediaRecorder.start();
    btnStart.disabled = true;
    btnStop.disabled = false;
    btnStart.textContent = "Recording...";
  }
}

function stopRecording() {
  mediaRecorder.stop();
}

window.addEventListener('load', () => {
  initMedia().catch(e => {
    alert('Could not access microphone. Check permissions.');
    console.error(e);
  });
});
</script>
</body>
</html>
)rawliteral";

// ----------------------------------------------------------------------------
// Handle MP3 upload
void handleUpload(
  AsyncWebServerRequest *request,
  String filename,
  size_t index,
  uint8_t *data,
  size_t len,
  bool final
) {
  if(!index) {
    logMsg("[handleUpload] Start: " + filename);
    if (SPIFFS.exists(TEMP_MP3_PATH)) {
      SPIFFS.remove(TEMP_MP3_PATH);
      logMsg("[handleUpload] Removed old temp MP3 file.");
    }
  }

  File f = SPIFFS.open(TEMP_MP3_PATH, (index == 0 ? "w" : "a"));
  if (f) {
    f.write(data, len);
    f.close();
  }

  if(final) {
    logMsg("[handleUpload] End: " + filename + " (" + String(index + len) + " bytes)");
    // Decode from MP3 -> RAW
    decodeMP3toRaw(TEMP_MP3_PATH, FINAL_RAW_PATH);
    SPIFFS.remove(TEMP_MP3_PATH);

    request->send(200, "text/plain",
      "Upload + decode complete. Stored as " + String(FINAL_RAW_PATH));
  }
}

// ----------------------------------------------------------------------------
// Setup
void setup() {
  Serial.begin(115200);
  logMsg("\n[setup] Starting...");

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    logMsg("[setup] SPIFFS init failed!");
    while(true) { delay(1000); }
  }
  logMsg("[setup] SPIFFS initialized.");

  // Wi-Fi AP mode, minimal config
  WiFi.mode(WIFI_AP);
  if(!WiFi.softAP(AP_SSID, AP_PASSWORD)) {
    logMsg("[setup] Failed to start AP!");
    while(true) { delay(1000); }
  }
  logMsg("[setup] AP started: " + String(AP_SSID));
  logMsg("[setup] AP IP: " + WiFi.softAPIP().toString());

  // Start Bluetooth A2DP
  a2dp_source.set_auto_reconnect(true);
  a2dp_source.start_raw(BT_DEVICE_NAME, audioDataCallback);
  a2dp_source.set_volume(20);
  logMsg("[setup] Bluetooth started as '" + String(BT_DEVICE_NAME) + "'");

  // Web routes
  server.on("/record", HTTP_GET, [](AsyncWebServerRequest *request){
    logMsg("[HTTP_GET /record] Serving page");
    request->send_P(200, "text/html", record_html);
  });

  server.on(
    "/upload", HTTP_POST,
    [](AsyncWebServerRequest *request){},
    handleUpload
  );

  server.on("/play", HTTP_GET, [](AsyncWebServerRequest *request){
    logMsg("[HTTP_GET /play] Playback triggered");
    request->send(200, "text/plain", "Playback started. Check speaker.");
    startAudioPlayback();
  });

  // Redirect any other path to /record
  server.onNotFound([](AsyncWebServerRequest *request){
    logMsg("[NotFound] Redirect => /record");
    request->redirect("/record");
  });

  server.begin();
  logMsg("[setup] Webserver on port 80");
}

// ----------------------------------------------------------------------------
// Loop
void loop() {
  checkAudioFinished();
}