#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <time.h>
#include "config.h"

// ── Network & Backend Configuration ─────────────────────────────────────────
static const char* WIFI_SSID    = "Realme 11 280";
static const char* WIFI_PASS    = "12345678900987654321";
static const char* API_BASE_URL = "http://10.72.136.174:3000";
static const char* DEVICE_ID    = "esp32-m01-s03";
static const char* DEVICE_KEY   = "php-s-03-ee1ff4f1cd060acffb5b19b40dc2a1fe7be4";

// ── Test Configuration ──────────────────────────────────────────────────────
static const uint32_t DOWNTIME_THRESHOLD_MS    = 10000; // 10 seconds
static const uint32_t STATUS_PRINT_INTERVAL_MS = 2000;  // Print status every 2s
static const uint32_t DEBOUNCE_MS              = 50;    // 50ms PC817 edge debounce
static const uint32_t HEARTBEAT_INTERVAL_MS    = 10000; // Backend heartbeat every 10s

// ── State Variables ─────────────────────────────────────────────────────────
static int      currentState       = HIGH;   // Current debounced GPIO reading
static int      previousState      = HIGH;   // Previous reading (for edge detection)
static int      lastRawReading     = HIGH;   // Raw GPIO reading for debounce
static uint32_t lastDebounceTime   = 0;      // millis() of last raw GPIO change
static uint32_t stateStartTime     = 0;      // millis() when current state began
static bool     downtimeReported   = false;  // True if we already reported downtime
static bool     downtimeActive     = false;  // True while LED is ON (downtime in progress)
static uint32_t lastStatusPrint    = 0;      // For periodic status updates
static uint32_t lastHeartbeatSent  = 0;      // For 10s backend heartbeat
static bool     activitySinceHb    = false;  // Tracks sensor activity for heartbeat

static Preferences prefs;
static String      bootId;
static uint32_t    bootCounter         = 1;
static uint64_t    hbSequence          = 0;

// ── Helpers ─────────────────────────────────────────────────────────────────
const char* sensorStateLabel(int gpioState) {
    return (gpioState == LOW) ? "METAL_DETECTED (LOW)" : "NO_METAL (HIGH)";
}

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

bool sendSensorEvent(const char* eventType, const char* signal) {
    String meta = String("{\"rssi\":") + String(WiFi.RSSI()) +
                  ",\"sensorState\"😕"" + sensorStateLabel(currentState) + "\"";
    if (holdReason != nullptr) {
        meta += String(",\"reason\"😕"") + holdReason + "\"";
    }
    meta += "}";

    String body = String("{") +
        "\"eventId\":\"" + generateUuidV4() + "\"," +
        "\"eventType\":\"" + eventType + "\"," +
        "\"signal\":\"" + signal + "\"," +
        "\"recordedAt\":\"" + isoNow() + "\"," +
        "\"metadata\":" + meta +
    "}";

    if (postToBackend("/api/iot/events", body)) {
        Serial.printf("  [EVENT SENT] %s -> %s (pulse #%lu)\n", eventType, signal, (unsigned long)pulseCount);
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
        Serial.printf("  [HEARTBEAT] Seq: %llu | Activity: %s\n", hbSequence, activitySinceHb ? "YES" : "NO");
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

void markRotationEdge(uint32_t now) {
    lastEdgeTime = now;
    stoppedReported = false;
}

bool shouldReportStopped(uint32_t now) {
    if (stoppedReported) return false;
    if (now - lastEdgeTime < INACTIVITY_TIMEOUT_MS) return false;
    return now - lastIdleAttempt >= IDLE_RETRY_INTERVAL_MS;
}

void reportStoppedIfInactive(uint32_t now) {
    if (!shouldReportStopped(now)) return;
    lastIdleAttempt = now;
    stoppedReported = sendSensorEvent("idle", "idle");
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
    prefs.end();
    bootId = generateUuidV4();

    Serial.println();
    Serial.println(F("=============================================================="));
    Serial.printf( "  %s — MACHINE MAIN SPINDLE MONITOR [%s]\n", NODE_ID, DEVICE_ID);
    Serial.println(F("=============================================================="));
    Serial.printf( "  Backend URL : %s\n", API_BASE_URL);
    Serial.printf( "  Sensor Pin  : GPIO %d (INPUT_PULLUP via PC817)\n", SENSOR_INPUT_PIN);
    Serial.printf( "  LED Pin     : GPIO %d (Pulse Indicator)\n", ONBOARD_LED_PIN);
    Serial.println(F("  Logic       : Rotation toggles GPIO. Both constant HIGH"));
    Serial.println(F("                and constant LOW indicate stopped machine."));
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

    uint32_t epochNow = (uint32_t)time(nullptr);
    if (epochNow > bootCounter) {
        bootCounter = epochNow;
        prefs.begin("iot-node", false);
        prefs.putUInt("boot_cnt", bootCounter);
        prefs.end();
    }

    currentState      = digitalRead(SENSOR_INPUT_PIN);
    lastRawReading    = currentState;
    lastHeartbeatSent = millis();
    lastStatusPrint   = millis();
    lastEdgeTime      = millis();

    sendHeartbeat();
}

// ── Main Loop ───────────────────────────────────────────────────────────────
void loop() {
    reconnectWifiIfNeeded();

    uint32_t now = millis();
    int reading = digitalRead(SENSOR_INPUT_PIN);

    // 1. Debounce and detect transitions (both bolt arrival and departure)
    if (reading != lastRawReading) {
        lastDebounceTime = now;
        lastRawReading = reading;
    }

    if ((now - lastDebounceTime) >= SENSOR_DEBOUNCE_MS && reading != currentState) {
        currentState = reading;

        pulseCount++;
        hasRotatedSinceSend = true;
        activitySinceHb = true;
        markRotationEdge(now);

        digitalWrite(ONBOARD_LED_PIN, HIGH);
    }

    // 2. Transmit throttled rotation pulse while running (immediate on first edge after idle/boot)
    if (hasRotatedSinceSend && (lastPulseAttempt == 0 || (now - lastPulseAttempt >= MIN_PULSE_INTERVAL_MS))) {
        lastPulseAttempt = now;
        if (sendSensorEvent("pulse", "active")) {
            lastPulseSentTime = now;
            hasRotatedSinceSend = false;
        }
    }

    reportStoppedIfInactive(now);

    // Keep pulse indicator LED visible for 200ms after each detected edge
    if (now - lastDebounceTime > 200) {
        digitalWrite(ONBOARD_LED_PIN, LOW);
    }

    // ── 4. Periodic Backend Heartbeat (every 10s) ───────────────────────────
    if (now - lastHeartbeatSent >= HEARTBEAT_INTERVAL_MS) {
        lastHeartbeatSent = now;
        sendHeartbeat();
    }

    // 4. Status log every 3s
    if (now - lastStatusPrint >= STATUS_PRINT_INTERVAL_MS) {
        lastStatusPrint = now;
        uint32_t silentDurationMs = now - lastPulseSentTime;
        Serial.printf("[%7lu ms] State: %-22s | Pulses: %4lu | Silence: %5lu ms | WiFi: %ddBm\n",
                      (unsigned long)now, sensorStateLabel(currentState),
                      (unsigned long)pulseCount, (unsigned long)silentDurationMs, WiFi.RSSI());
    }

    delay(1);
}
