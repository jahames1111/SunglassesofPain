/*
  Smartglasses pipeline (Groq Whisper -> Groq Chat -> WitAITTS)
  - Record -> Save WAV -> Upload to Groq Whisper -> Groq LLM (chat) -> WitAITTS (streams to I2S)
  - Manual WAV playback removed: WitAITTS handles playback itself
  - Stops PDM input while streaming TTS to avoid input/output interference
  - Requires keys.h with WIFI_SSID, WIFI_PASSWORD, GROQ_API_KEY, WIT_TOKEN
  - Requires libraries: AudioTools, ESP_I2S, SD, ArduinoJson, WitAITTS
*/

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <SPI.h>
#include <SD.h>
#include <ArduinoJson.h>
#include <AudioTools.h>
#include <ESP_I2S.h>
#include <WitAITTS.h>
#include "keys.h"   // define WIFI_SSID, WIFI_PASSWORD, GROQ_API_KEY, WIT_TOKEN

// ---------------- Hardware Pin Configuration ----------------
#define TOUCH_PIN 1            // Touch Pin for Recording
#define TOUCH_THRESHOLD 30000  // Touch threshold (user provided)
#define SD_CS 21               // SD Chip Select
#define SD_SCK 7               // SD SCK
#define SD_MISO 8              // SD MISO
#define SD_MOSI 9              // SD MOSI
#define PDM_CLK 42             // PDM Mic Clock
#define PDM_DATA 41            // PDM Mic Data
#define I2S_BCLK 5             // Speaker BCLK
#define I2S_WS 6               // Speaker WS
#define I2S_DOUT 4             // Speaker Data
#define SAMPLE_RATE 16000      // Recording sample rate (Whisper expects 16k)
const char* WAV_PATH = "/r.wav";

// ---------------- Groq endpoints ----------------
const char* WHISPER_HOST = "api.groq.com";
const char* WHISPER_PATH = "/openai/v1/audio/transcriptions";
const char* GROQ_CHAT_URL = "https://api.groq.com/openai/v1/chat/completions";

// ---------------- Globals ----------------
SPIClass spiSD(FSPI);
I2SClass I;
I2SStream O;
VolumeStream V(O);

// WitAITTS instance
WitAITTS tts(I2S_BCLK, I2S_WS, I2S_DOUT);
bool ttsReady = false;

bool debugVerbose = true;
const int STABLE_REQUIRED = 4;     // consecutive stable readings to accept state

// ---------------- Utility ----------------
static unsigned long nowMs() { return millis(); }

void dbg(const String &s) {
  if (debugVerbose) Serial.println(s);
}

// varargs debug print (printf-style)
void dbgf(const char *fmt, ...) {
  if (!debugVerbose) return;
  char buf[512];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.println(buf);
}
void dbgf(const String &s) { if (debugVerbose) Serial.println(s); }

// ---------------- WAV header helpers ----------------
void writeWavHeader(File &f, uint32_t sampleRate, uint32_t dataBytes) {
  f.seek(0);
  f.write((const uint8_t*)"RIFF", 4);
  uint32_t fileSizeMinus8 = 36 + dataBytes;
  f.write((const uint8_t*)&fileSizeMinus8, 4);
  f.write((const uint8_t*)"WAVE", 4);
  f.write((const uint8_t*)"fmt ", 4);
  uint32_t subChunk1Size = 16;
  f.write((const uint8_t*)&subChunk1Size, 4);
  uint16_t audioFormat = 1; // PCM
  uint16_t numChannels = 1;
  uint16_t bitsPerSample = 16;
  uint32_t byteRate = sampleRate * numChannels * (bitsPerSample / 8);
  uint16_t blockAlign = numChannels * (bitsPerSample / 8);
  f.write((const uint8_t*)&audioFormat, 2);
  f.write((const uint8_t*)&numChannels, 2);
  f.write((const uint8_t*)&sampleRate, 4);
  f.write((const uint8_t*)&byteRate, 4);
  f.write((const uint8_t*)&blockAlign, 2);
  f.write((const uint8_t*)&bitsPerSample, 2);
  f.write((const uint8_t*)"data", 4);
  f.write((const uint8_t*)&dataBytes, 4);
}

// ---------------- Touch helpers ----------------
inline bool isPressedRaw(int raw) {
  return raw > TOUCH_THRESHOLD;
}

// Debounced rising-edge detector that returns true once when press is detected
bool detectRisingEdgeDebounced() {
  static bool lastState = false;
  static int stableCount = 0;

  int raw = touchRead(TOUCH_PIN);
  bool cur = isPressedRaw(raw);
  dbgf("[TOUCH] raw=%d cur=%d last=%d stableCount=%d", raw, cur ? 1 : 0, lastState ? 1 : 0, stableCount);

  if (cur != lastState) {
    stableCount++;
    if (stableCount >= STABLE_REQUIRED) {
      bool rising = (!lastState && cur);
      lastState = cur;
      stableCount = 0;
      if (rising) {
        dbgf("[TOUCH] Rising edge accepted at %lu ms", nowMs());
        return true;
      }
    }
  } else {
    stableCount = 0;
  }
  delay(12);
  return false;
}

// Debounced stable state reader (used to stop recording)
bool readStablePressedState() {
  int ok = 0;
  for (int i = 0; i < STABLE_REQUIRED; ++i) {
    int raw = touchRead(TOUCH_PIN);
    if (isPressedRaw(raw)) ok++;
    delay(8);
  }
  bool stable = (ok == STABLE_REQUIRED);
  dbgf("[TOUCH] readStablePressedState ok=%d stable=%d", ok, stable ? 1 : 0);
  return stable;
}

// ---------------- Recording (uses stable stop detection) ----------------
void recordToSd() {
  dbgf("[REC] Start recording at %lu ms", nowMs());
  File f = SD.open(WAV_PATH, FILE_WRITE);
  if (!f) {
    dbg("[REC] ERROR: cannot open file for writing");
    return;
  }

  // Reserve header space (44 bytes)
  uint8_t zero44[44] = {0};
  f.write(zero44, 44);

  const size_t BUF_SZ = 512;
  static uint8_t buf[BUF_SZ]; // static to avoid stack pressure
  uint32_t totalWritten = 0;
  unsigned long start = nowMs();

  // Read while touch remains pressed (stable)
  while (readStablePressedState()) {
    int r = I.readBytes((char*)buf, BUF_SZ);
    if (r > 0) {
      f.write(buf, r);
      totalWritten += r;
    }
    delay(1);
  }

  unsigned long dur = nowMs() - start;
  dbgf("[REC] Raw bytes written: %u  duration_ms: %lu", totalWritten, dur);

  // finalize WAV header
  writeWavHeader(f, SAMPLE_RATE, totalWritten);
  f.close();
  dbg("[REC] Recording finished and WAV header written");
}

// ---------------- Transcription (Whisper) with TLS diagnostics and retries ----------------
String transcribeWithWhisper(const char* path, const char* modelName = "whisper-large-v3-turbo") {
  dbgf("[STT] transcribeWithWhisper start %lu ms", nowMs());
  File f = SD.open(path);
  if (!f) {
    dbg("[STT] ERROR: file not found on SD");
    return "";
  }

  uint32_t fileSize = f.size();
  dbgf("[STT] File opened: %s size=%u bytes", path, fileSize);

  // Diagnostics before TLS connect
  dbgf("[STT] Pre-connect diagnostics:");
  dbgf(String("  FreeHeap=") + String(ESP.getFreeHeap()));
  dbgf(String("  WiFi RSSI=") + String(WiFi.RSSI()));
  dbgf(String("  millis=") + String(millis()));

  WiFiClientSecure client;

  client.setInsecure();

  // small pause to let memory stabilize after heavy ops
  delay(120);
  yield();

  // DNS resolution step (non-invasive): print resolved IP if available
  IPAddress resolvedIP;
  bool dnsOk = false;
  dbgf("[STT] Resolving host via DNS: %s", WHISPER_HOST);
  if (WiFi.hostByName(WHISPER_HOST, resolvedIP)) {
    dbgf("[STT] DNS resolved %s -> %s", WHISPER_HOST, resolvedIP.toString().c_str());
    dnsOk = true;
  } else {
    dbgf("[STT] DNS resolution failed for %s", WHISPER_HOST);
  }

  // Try connect with retries
  const int MAX_CONNECT_TRIES = 3;
  bool connected = false;
  for (int attempt = 1; attempt <= MAX_CONNECT_TRIES; ++attempt) {
    dbgf("[STT] Attempt %d to connect...", attempt);

    // First try: original host-based connect
    if (client.connect(WHISPER_HOST, 443)) {
      connected = true;
      dbgf("[STT] TLS connect OK (host)");
      break;
    } else {
      dbgf("[STT] connect() returned: 0  (attempt %d)", attempt);
      dbgf(String("  FreeHeap after failed connect=") + String(ESP.getFreeHeap()));
      // If DNS resolved earlier, try direct IP connect as an extra fallback on the same attempt
      if (dnsOk) {
        dbgf("[STT] Attempting direct-IP connect to %s (fallback)", resolvedIP.toString().c_str());
        if (client.connect(resolvedIP, 443)) {
          connected = true;
          dbgf("[STT] TLS connect OK (direct IP)");
          break;
        } else {
          dbgf("[STT] direct-IP connect() returned: 0  (attempt %d)", attempt);
          dbgf(String("  FreeHeap after failed direct-IP connect=") + String(ESP.getFreeHeap()));
        }
      }
      delay(250 * attempt);
      yield();
    }
  }

  if (!connected) {
    Serial.println("[STT] ERROR: could not connect to host after retries");
    f.close();
    return "";
  }

  // Build multipart body header and tail
  String boundary = "ESP32Boundary" + String(millis());
  String head = "--" + boundary + "\r\n";
  head += "Content-Disposition: form-data; name=\"model\"\r\n\r\n";
  head += String(modelName) + "\r\n";
  head += "--" + boundary + "\r\n";
  head += "Content-Disposition: form-data; name=\"file\"; filename=\"r.wav\"\r\n";
  head += "Content-Type: audio/wav\r\n\r\n";
  String tail = "\r\n--" + boundary + "--\r\n";

  uint32_t contentLength = head.length() + fileSize + tail.length();
  dbgf("[STT] Calculated Content-Length: %u", contentLength);

  // Send request line and headers
  client.printf("POST %s HTTP/1.1\r\n", WHISPER_PATH);
  client.printf("Host: %s\r\n", WHISPER_HOST);
  client.printf("Authorization: Bearer %s\r\n", GROQ_API_KEY);
  client.printf("Content-Type: multipart/form-data; boundary=%s\r\n", boundary.c_str());
  client.printf("Content-Length: %u\r\n", contentLength);
  client.printf("Connection: close\r\n\r\n");

  // Send head
  client.print(head);
  dbg("[STT] Sent multipart head");

  // Stream file in chunks using static buffer
  const size_t CHUNK = 1024;
  static uint8_t buffer[CHUNK];
  uint32_t sentBytes = 0;
  unsigned long t0 = nowMs();
  while (f.available()) {
    size_t r = f.read(buffer, CHUNK);
    if (r == 0) break;
    size_t w = client.write(buffer, r);
    if (w != r) {
      dbgf("[STT] WARNING: wrote %u of %u bytes to client", w, r);
    }
    sentBytes += r;
    if ((sentBytes % 32768) < CHUNK) {
      dbgf("[STT] upload progress: %u/%u bytes", sentBytes, fileSize);
    }
    delay(1);
  }
  f.close();
  client.print(tail);
  dbgf("[STT] Finished upload at %lu ms, sentBytes=%u", nowMs(), sentBytes);

  // Read response: print headers and body lines
  dbg("[STT] Reading response headers and body (line-by-line):");
  String statusLine = client.readStringUntil('\n');
  statusLine.trim();
  dbgf("[STT] Status line: %s", statusLine.c_str());

  // Read headers
  String header;
  while (true) {
    header = client.readStringUntil('\n');
    header.trim();
    if (header.length() == 0) break;
    dbgf("[STT] Header: %s", header.c_str());
  }
  dbg("[STT] End of headers. Now reading body lines (raw):");

  String fullBody = "";
  unsigned long bodyStart = nowMs();
  unsigned long readStart = nowMs();
  while (client.connected() || client.available()) {
    if (client.available()) {
      String line = client.readStringUntil('\n');
      dbgf("[STT] BODY LINE: %s", line.c_str());
      fullBody += line;
      readStart = nowMs();
    } else {
      if ((nowMs() - readStart) > 5000) break;
      delay(10);
    }
  }
  unsigned long bodyDur = nowMs() - bodyStart;
  dbgf("[STT] Finished reading body, bytes=%u, time_ms=%lu", fullBody.length(), bodyDur);

  client.stop();

  // Try to find JSON substring that contains "text"
  String transcript = "";
  int idx = fullBody.indexOf("{\"text\"");
  if (idx >= 0) {
    String jsonPart = fullBody.substring(idx);
    int showLen = (int)jsonPart.length();
    if (showLen > 512) showLen = 512;
    dbgf("[STT] Found JSON start at idx=%d; jsonPart (truncated 512): %s", idx, jsonPart.substring(0, showLen).c_str());
    StaticJsonDocument<4096> doc;
    DeserializationError err = deserializeJson(doc, jsonPart);
    if (err) {
      dbgf("[STT] JSON parse error: %s", err.c_str());
      int braceIdx = fullBody.indexOf("\n{\"text\"");
      if (braceIdx >= 0) {
        String alt = fullBody.substring(braceIdx + 1);
        StaticJsonDocument<4096> doc2;
        DeserializationError err2 = deserializeJson(doc2, alt);
        if (!err2) {
          transcript = String((const char*)doc2["text"].as<const char*>());
        } else {
          dbgf("[STT] JSON parse alt failed: %s", err2.c_str());
        }
      }
    } else {
      transcript = String((const char*)doc["text"].as<const char*>());
    }
  } else {
    dbg("[STT] No JSON 'text' field found in response body");
    int snippetLen = (int)fullBody.length();
    if (snippetLen > 1024) snippetLen = 1024;
    dbgf("[STT] Response snippet (first %d chars): %s", snippetLen, fullBody.substring(0, snippetLen).c_str());
  }

  dbgf("[STT] Transcript result: '%s'", transcript.c_str());
  return transcript;
}

// ---------------- Chat (Groq Llama) with debugging ----------------
String queryGroqChat(const String &userPrompt, const char* modelName = "llama-3.1-8b-instant") {
  dbgf("[CHAT] queryGroqChat start at %lu ms", nowMs());
  if (userPrompt.length() == 0) {
    dbg("[CHAT] Empty prompt, returning empty reply");
    return "";
  }

  HTTPClient http;
  http.begin(GROQ_CHAT_URL);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", "Bearer " + String(GROQ_API_KEY));

  // Build JSON body with a small StaticJsonDocument
  StaticJsonDocument<2048> bodyDoc;
  bodyDoc["model"] = modelName;
  JsonArray messages = bodyDoc.createNestedArray("messages");
  JsonObject m = messages.createNestedObject();
  m["role"] = "user";
  m["content"] = userPrompt;

  String body;
  serializeJson(bodyDoc, body);

  // Safe substring printing
  {
    int blen = (int)body.length();
    int show = (blen < 1024) ? blen : 1024;
    dbg("[CHAT] Request body (truncated 1024):");
    dbg(body.substring(0, show));
  }

  unsigned long t0 = nowMs();
  int code = http.POST(body);
  unsigned long t1 = nowMs();
  dbgf("[CHAT] HTTP POST returned code=%d (took %lu ms)", code, t1 - t0);

  String payload = "";
  if (code > 0) {
    payload = http.getString();
    dbgf("[CHAT] Response payload length=%u", payload.length());
    {
      int plen = (int)payload.length();
      int showp = (plen < 2048) ? plen : 2048;
      dbg("[CHAT] Response payload (truncated 2048):");
      dbg(payload.substring(0, showp));
    }
  } else {
    dbgf("[CHAT] HTTP POST failed, error: %s", http.errorToString(code).c_str());
  }

  http.end();

  // Parse JSON
  String aiText = "";
  if (payload.length() > 0) {
    StaticJsonDocument<8192> resDoc;
    DeserializationError err = deserializeJson(resDoc, payload);
    if (err) {
      dbgf("[CHAT] JSON parse error: %s", err.c_str());
      int idx = payload.indexOf("\"choices\"");
      dbgf("[CHAT] choices idx=%d", idx);
    } else {
      if (resDoc.containsKey("choices") && resDoc["choices"].is<JsonArray>()) {
        JsonArray arr = resDoc["choices"].as<JsonArray>();
        if (arr.size() > 0 && arr[0].containsKey("message") && arr[0]["message"].containsKey("content")) {
          aiText = String((const char*)arr[0]["message"]["content"].as<const char*>());
        } else {
          dbg("[CHAT] Unexpected choices structure");
        }
      } else {
        dbg("[CHAT] No choices array in response");
      }
    }
  }

  dbgf("[CHAT] AI reply: '%s'", aiText.c_str());
  return aiText;
}

// ---------------- Estimate TTS duration helper ----------------
unsigned long estimateTtsDurationMs(const String &text) {
  int words = 0;
  bool inWord = false;
  for (size_t i = 0; i < text.length(); ++i) {
    char c = text[i];
    if (!isspace(c)) {
      if (!inWord) { words++; inWord = true; }
    } else inWord = false;
  }
  unsigned long ms = (unsigned long)words * 400UL;
  if (ms < 1500UL) ms = 1500UL;
  if (ms > 90000UL) ms = 90000UL;
  ms += 800UL;
  return ms;
}

// ---------------- TTS speak wrapper using WitAITTS ----------------
void speakText(const String &txt) {
  if (txt.length() == 0) {
    dbg("[TTS] Empty text, skipping TTS");
    return;
  }
  if (!ttsReady) {
    dbg("[TTS] WitAITTS not initialized, skipping TTS");
    return;
  }

  dbgf("[TTS] Preparing to speak text length=%u", txt.length());

  // Stop PDM input to avoid interference with I2S output
  I.end();
  dbgf("[TTS] PDM input stopped before TTS");

  // Lower AudioTools volume briefly to avoid clipping (if using same output)
  float prevVol = 1.0f;
  V.setVolume(0.7f);

  // Start streaming TTS to I2S (WitAITTS handles playback)
  tts.speak(txt);

  // Keep calling tts.loop() while waiting estimated duration
  unsigned long waitMs = estimateTtsDurationMs(txt);
  unsigned long start = nowMs();
  while (nowMs() - start < waitMs) {
    tts.loop();
    delay(10);
  }

  // Ensure any remaining streaming tasks are processed
  for (int i = 0; i < 20; ++i) {
    tts.loop();
    delay(10);
  }

  // Restore volume
  V.setVolume(prevVol);

  // Restart PDM input after TTS
  I.setPinsPdmRx(PDM_CLK, PDM_DATA);
  I.begin(I2S_MODE_PDM_RX, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
  dbgf("[TTS] PDM input restarted after TTS");

  dbgf("[TTS] TTS streaming complete");
}

// ---------------- Setup and loop ----------------
void setup() {
  Serial.begin(115200);
  delay(10);
  dbg("[SETUP] Starting setup");

  // WiFi connect with debug
  dbgf("[SETUP] Connecting to WiFi SSID='%s'", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long wifiStart = nowMs();
  while (WiFi.status() != WL_CONNECTED && (nowMs() - wifiStart) < 20000) {
    dbgf("[SETUP] WiFi status=%d, elapsed=%lu ms", WiFi.status(), nowMs() - wifiStart);
    delay(200);
  }
  if (WiFi.status() == WL_CONNECTED) {
    dbgf("[SETUP] WiFi connected, IP=%s", WiFi.localIP().toString().c_str());
  } else {
    dbgf("[SETUP] WiFi not connected after timeout; network ops may fail");
  }

  // SD init
  dbg("[SETUP] Initializing SD...");
  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, spiSD)) {
    dbg("[SETUP] ERROR: SD.begin failed");
  } else {
    dbg("[SETUP] SD initialized");
  }

  // Audio output init (AudioTools) - default sample rate
  auto cfg = O.defaultConfig(TX_MODE);
  cfg.pin_bck = I2S_BCLK;
  cfg.pin_ws = I2S_WS;
  cfg.pin_data = I2S_DOUT;
  cfg.sample_rate = SAMPLE_RATE;
  cfg.channels = 1;
  cfg.bits_per_sample = 16;
  O.begin(cfg);
  auto vcfg = V.defaultConfig();
  vcfg.copyFrom(cfg);
  vcfg.allow_boost = 1;
  V.begin(vcfg);
  V.setVolume(1.0);
  dbg("[SETUP] I2S output initialized");

  // PDM mic init
  I.setPinsPdmRx(PDM_CLK, PDM_DATA);
  I.begin(I2S_MODE_PDM_RX, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
  dbg("[SETUP] I2S PDM input initialized");

  // Touch sensor stabilization: discard noisy startup readings
  dbg("[SETUP] Stabilizing touch sensor...");
  delay(500);
  for (int i = 0; i < 8; ++i) {
    int v = touchRead(TOUCH_PIN);
    dbgf("[SETUP] touchRead discard #%d = %d", i, v);
    delay(20);
  }

  // Initialize WitAITTS
  dbgf("[SETUP] Initializing WitAITTS...");
  tts.setDebugLevel(DEBUG_INFO);
  if (tts.begin(WIFI_SSID, WIFI_PASSWORD, WIT_TOKEN)) {
    Serial.println("[TTS] WitAITTS Ready!");
    ttsReady = true;
    // Configure voice and params (adjust as desired)
    tts.setVoice("wit$Remi");
    tts.setStyle("default");
    tts.setSpeed(100);
    tts.setPitch(100);
    tts.setGain(0.8);
    tts.printConfig();
  } else {
    Serial.println("[TTS] WitAITTS initialization failed (check WiFi and WIT_TOKEN)");
    ttsReady = false;
  }

  dbg("[SETUP] Setup complete");
}

void loop() {
  // Keep TTS streaming alive
  tts.loop();

  // Detect rising edge (debounced)
  if (detectRisingEdgeDebounced()) {
    dbg("[LOOP] Rising edge detected -> start record flow");
    recordToSd();

    // After recording, show file info
    File f = SD.open(WAV_PATH);
    if (!f) {
      dbg("[LOOP] ERROR: WAV file missing after recording");
    } else {
      dbgf("[LOOP] WAV file size after record: %u bytes", f.size());
      f.close();
    }

    // Transcribe with Groq Whisper
    unsigned long t0 = nowMs();
    String transcript = transcribeWithWhisper(WAV_PATH);
    unsigned long t1 = nowMs();
    dbgf("[LOOP] Transcription took %lu ms", t1 - t0);
    dbgf("[LOOP] User: '%s'", transcript.c_str());

    // Query Groq chat (LLaMA)
    unsigned long c0 = nowMs();
    String aiReply = queryGroqChat(transcript);
    unsigned long c1 = nowMs();
    dbgf("[LOOP] Chat took %lu ms", c1 - c0);
    dbgf("[LOOP] AI: '%s'", aiReply.c_str());

    // TTS via WitAITTS (streamed to I2S).
    speakText(aiReply);

    dbg("[LOOP] Completed flow");
  }

  delay(50);
}