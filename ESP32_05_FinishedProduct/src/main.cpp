#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <time.h>
#include "config.h"

// =============================================================================
// ESP32 SENSOR 5 — FINISHED PRODUCT (PIPE OUTPUT) PRODUCTION COUNTER
// =============================================================================
// Node ID   : S-05
// Purpose   : Count manufactured pipes passing flying cutter / output station.
// Logic     : Pipe arrives (HIGH -> LOW) -> Increments counter, sends 'pulse' (active), LED ON.
//             Pipe leaves  (LOW -> HIGH) -> Counter re-armed for next pipe, LED OFF.
//             S-05 never participates in absence detection or downtime.
// =============================================================================

// Identity, WiFi, backend URL, pins and timing live in config.h.

// ── Runtime State ───────────────────────────────────────────────────────────
static int       currentRawReading      = HIGH;
static int       lastRawReading         = HIGH;
static uint32_t  lastDebounceTime       = 0;
static int       debouncedPinState      = HIGH;
static int       previousConfirmedState = HIGH;

static uint32_t  pipeProductionCount    = 0;
static uint32_t  lastHeartbeatTime      = 0;
static bool      activitySinceHb        = false;
static uint32_t  pendingPulseEvents     = 0;

static Preferences prefs;
static String      bootId;
static uint32_t    bootCounter          = 1;
static uint64_t    hbSequence           = 0;

// ── Helpers ─────────────────────────────────────────────────────────────────
String generateUuidV4() {
    uint8_t b[16];
    esp_fill_random(b, 16);
    b[6] = (b[6] & 0x0F) | 0x40;
    b[8] = (b[8] & 0x3F) | 0x80;
    char out[37];
    snprintf(out, sizeof(out),
        "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
        b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7],
        b[8], b[9], b[10], b[11], b[12], b[13], b[14], b[15]);
    return String(out);
}

String isoNow() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm timeinfo;
    gmtime_r(&tv.tv_sec, &timeinfo);
    char buf[64];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d.%03ldZ",
        timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
        timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, (long)(tv.tv_usec / 1000));
    return String(buf);
}

bool postToBackend(const char* path, const String& jsonBody) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("  [API] Skipped: WiFi disconnected"));
        return false;
    }
    HTTPClient http;
    http.setTimeout(4000);
    http.begin(String(API_BASE_URL) + path);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("x-device-id", DEVICE_ID);
    http.addHeader("x-device-key", DEVICE_KEY);

    int code = http.POST(jsonBody);
    if (code < 200 || code >= 300) {
        String resp = http.getString();
        Serial.printf("  [API Error %d] %s\n", code, resp.c_str());
    }
    http.end();
    return (code >= 200 && code < 300);
}

bool sendPulseEvent() {
    String meta = String("{\"rssi\":") + String(WiFi.RSSI()) +
                  ",\"pipeCount\":" + String(pipeProductionCount) + "}";
    String body = String("{") +
        "\"eventId\":\"" + generateUuidV4() + "\"," +
        "\"eventType\":\"pulse\"," +
        "\"signal\":\"active\"," +
        "\"recordedAt\":\"" + isoNow() + "\"," +
        "\"metadata\":" + meta +
    "}";
    if (postToBackend("/api/iot/events", body)) {
        Serial.printf("  [PIPE COUNTED] Total: %lu\n", (unsigned long)pipeProductionCount);
        return true;
    }
    return false;
}

void sendHeartbeat() {
    hbSequence++;
    String body = String("{") +
        "\"heartbeatId\":\"" + generateUuidV4() + "\"," +
        "\"bootId\":\"" + bootId + "\"," +
        "\"bootCounter\":\"" + String(bootCounter) + "\"," +
        "\"sequence\":\"" + String((unsigned long)hbSequence) + "\"," +
        "\"recordedAt\":\"" + isoNow() + "\"," +
        "\"activityObserved\":" + (activitySinceHb ? "true" : "false") +
    "}";
    if (postToBackend("/api/iot/heartbeats", body)) {
        Serial.printf("  [HEARTBEAT] Seq: %llu | Output count: %lu\n", hbSequence, (unsigned long)pipeProductionCount);
        activitySinceHb = false;
    }
}

void reconnectWifiIfNeeded() {
    static uint32_t lastReconnectAttempt = 0;
    if (WiFi.status() == WL_CONNECTED) return;
    uint32_t now = millis();
    if (now - lastReconnectAttempt < WIFI_RECONNECT_INTERVAL_MS) return;
    lastReconnectAttempt = now;
    Serial.println(F("  [WiFi] Disconnected, retrying..."));
    WiFi.reconnect();
}

// ── Setup ───────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    delay(200);

    pinMode(SENSOR_INPUT_PIN, INPUT_PULLUP);
    pinMode(ONBOARD_LED_PIN, OUTPUT);
    digitalWrite(ONBOARD_LED_PIN, LOW);

    prefs.begin("iot-node", false);
    bootCounter = prefs.getUInt("boot_cnt", 0) + 1;
    prefs.putUInt("boot_cnt", bootCounter);
    pipeProductionCount = prefs.getUInt("pipe_cnt", 0);
    prefs.end();
    bootId = generateUuidV4();

    Serial.println();
    Serial.println(F("=============================================================="));
    Serial.printf( "  %s — %s [%s]\n", NODE_ID, NODE_ROLE_NAME, DEVICE_ID);
    Serial.println(F("=============================================================="));
    Serial.printf( "  Backend URL   : %s\n", API_BASE_URL);
    Serial.printf( "  Sensor Pin    : GPIO %d (INPUT_PULLUP via PC817)\n", SENSOR_INPUT_PIN);
    Serial.printf( "  LED Pin       : GPIO %d (Pipe Indicator)\n", ONBOARD_LED_PIN);
    Serial.printf( "  Current Count : %lu pipes\n", (unsigned long)pipeProductionCount);
    Serial.println(F("=============================================================="));

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("Connecting to WiFi (%s)", WIFI_SSID);
    uint32_t wifiAttemptStart = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        if (millis() - wifiAttemptStart >= WIFI_RECONNECT_INTERVAL_MS) {
            wifiAttemptStart = millis();
            WiFi.disconnect();
            WiFi.begin(WIFI_SSID, WIFI_PASS);
        }
    }
    Serial.printf("\nWiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());

    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    struct tm timeinfo;
    Serial.print("Syncing NTP time");
    while (!getLocalTime(&timeinfo) || (timeinfo.tm_year + 1900) < 2026) {
        reconnectWifiIfNeeded();
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\nNTP Synced: %s\n", isoNow().c_str());

    int initialReading      = digitalRead(SENSOR_INPUT_PIN);
    currentRawReading       = initialReading;
    lastRawReading          = initialReading;
    debouncedPinState       = initialReading;
    previousConfirmedState  = initialReading;
    lastHeartbeatTime       = millis();

    digitalWrite(ONBOARD_LED_PIN, (initialReading == SENSOR_DETECTED_STATE) ? HIGH : LOW);
    sendHeartbeat();
}

// ── Main Loop ───────────────────────────────────────────────────────────────
void loop() {
    reconnectWifiIfNeeded();

    uint32_t now = millis();
    int rawReading = digitalRead(SENSOR_INPUT_PIN);

    if (rawReading != lastRawReading) {
        lastDebounceTime = now;
        lastRawReading   = rawReading;
    }

    if ((now - lastDebounceTime) >= SENSOR_DEBOUNCE_MS) {
        if (rawReading != debouncedPinState) {
            debouncedPinState = rawReading;

            // Detect PIPE ARRIVAL (HIGH -> LOW transition)
            if (previousConfirmedState == SENSOR_CLEARED_STATE && debouncedPinState == SENSOR_DETECTED_STATE) {
                pipeProductionCount++;
                activitySinceHb = true;

                digitalWrite(ONBOARD_LED_PIN, HIGH);

                prefs.begin("iot-node", false);
                prefs.putUInt("pipe_cnt", pipeProductionCount);
                prefs.end();

                if (!sendPulseEvent()) {
                    pendingPulseEvents++;
                }
            }
            // Detect PIPE CLEARANCE (LOW -> HIGH transition)
            else if (previousConfirmedState == SENSOR_DETECTED_STATE && debouncedPinState == SENSOR_CLEARED_STATE) {
                digitalWrite(ONBOARD_LED_PIN, LOW);
            }

            previousConfirmedState = debouncedPinState;
        }
    }

    if (now - lastHeartbeatTime >= HEARTBEAT_INTERVAL_MS) {
        lastHeartbeatTime = now;
        while (pendingPulseEvents > 0 && sendPulseEvent()) {
            pendingPulseEvents--;
        }
        sendHeartbeat();
    }

    delay(1);
}
