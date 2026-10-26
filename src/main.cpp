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
static const uint32_t BACKEND_HEARTBEAT_INTERVAL_MS = 10000; // Backend heartbeat every 10s

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
static uint32_t    bootCounter = 1;
static uint64_t    hbSequence  = 0;

// ── Helpers ─────────────────────────────────────────────────────────────────
const char* sensorStateLabel(int gpioState) {
    return (gpioState == LOW) ? "ON" : "OFF";
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
    char buf[32];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d.%03ldZ",
        timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
        timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, tv.tv_usec / 1000);
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
    String resp = http.getString();
    Serial.printf("  [API] POST %s -> HTTP %d\n", path, code);
    if (code < 200 || code >= 300) {
        Serial.printf("  [API Error] %s\n", resp.c_str());
    }
    http.end();
    return (code >= 200 && code < 300);
}

void sendSensorEvent(const char* eventType, const char* signal, const char* holdReason = nullptr) {
    String meta = String("{\"rssi\":") + String(WiFi.RSSI()) +
                  ",\"sensorState\":\"" + sensorStateLabel(currentState) + "\"";
    if (holdReason != nullptr) {
        meta += String(",\"reason\":\"") + holdReason + "\"";
    }
    meta += "}";

    String body = String("{") +
        "\"eventId\":\"" + generateUuidV4() + "\"," +
        "\"eventType\":\"" + eventType + "\"," +
        "\"signal\":\"" + signal + "\"," +
        "\"recordedAt\":\"" + isoNow() + "\"," +
        "\"metadata\":" + meta +
    "}";
    postToBackend("/api/iot/events", body);
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
        activitySinceHb = false;
    }
}

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    delay(200);

    // Configure GPIO 4 with internal pull-up (PC817 collector pulls to GND on detect)
    pinMode(SENSOR_INPUT_PIN, INPUT_PULLUP);
    pinMode(ONBOARD_LED_PIN, OUTPUT);
    digitalWrite(ONBOARD_LED_PIN, LOW);  // LED OFF = normal at boot

    // Persist monotonic bootCounter in NVS
    prefs.begin("iot-node", false);
    bootCounter = prefs.getUInt("boot_cnt", 0) + 1;
    prefs.putUInt("boot_cnt", bootCounter);
    prefs.end();
    bootId = generateUuidV4();

    // Connect WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("Connecting to WiFi (%s)", WIFI_SSID);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\nWiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());

    // Sync NTP time (required by Backend: recordedAt >= 2026-05-01)
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    struct tm timeinfo;
    Serial.print("Syncing NTP time");
    while (!getLocalTime(&timeinfo) || (timeinfo.tm_year + 1900) < 2026) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\nNTP Synced: %s\n", isoNow().c_str());

    // Read initial state
    currentState    = digitalRead(SENSOR_INPUT_PIN);
    previousState   = currentState;
    lastRawReading  = currentState;
    stateStartTime  = millis();
    lastStatusPrint = stateStartTime;
    lastHeartbeatSent = stateStartTime;

    // Boot banner
    Serial.println();
    Serial.println(F("=============================================================="));
    Serial.println(F("  HARDWARE VALIDATION TEST — 10-Second Downtime Detection"));
    Serial.println(F("=============================================================="));
    Serial.printf( "  Device ID      : %s\n", DEVICE_ID);
    Serial.printf( "  Backend URL    : %s\n", API_BASE_URL);
    Serial.printf( "  Sensor Pin     : GPIO %d (INPUT_PULLUP)\n", SENSOR_INPUT_PIN);
    Serial.printf( "  LED Pin        : GPIO %d (Downtime indicator)\n", ONBOARD_LED_PIN);
    Serial.println(F("  Sensor Logic   : LOW = metal detected, HIGH = no metal"));
    Serial.println(F("  LED Logic      : OFF = normal, ON = downtime"));
    Serial.printf( "  Threshold      : %u seconds of unchanged state\n", DOWNTIME_THRESHOLD_MS / 1000);
    Serial.println(F("=============================================================="));
    Serial.printf( "  Sensor state: %s\n", sensorStateLabel(currentState));
    Serial.println(F("  Timer started. Monitoring for downtime..."));
    Serial.println();

    // Send initial heartbeat & running pulse
    sendHeartbeat();
    sendSensorEvent("pulse", "active");
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        WiFi.reconnect();
    }

    uint32_t now = millis();
    int reading = digitalRead(SENSOR_INPUT_PIN);

    // Debounce raw GPIO input
    if (reading != lastRawReading) {
        lastDebounceTime = now;
        lastRawReading = reading;
    }

    // ── 1. Detect confirmed state change ────────────────────────────────────
    if ((now - lastDebounceTime) >= DEBOUNCE_MS && reading != currentState) {
        previousState  = currentState;
        currentState   = reading;
        stateStartTime = now;
        activitySinceHb = true;

        // If downtime was active, end it immediately & send recovery to backend
        if (downtimeActive) {
            digitalWrite(ONBOARD_LED_PIN, LOW);  // LED OFF immediately
            downtimeActive = false;
            Serial.println(F(""));
            Serial.println(F("────────────────────────────────────────────────────────────"));
            Serial.println(F("  DOWNTIME ENDED: Sensor state changed."));
            Serial.printf( "  Sensor state: %s\n", sensorStateLabel(currentState));
            Serial.println(F("  Timer reset -> Sending 'recovered' to Backend..."));
            Serial.println(F("────────────────────────────────────────────────────────────"));
            sendSensorEvent("recovered", "active");
        } else {
            Serial.println(F("────────────────────────────────────────────────────────────"));
            Serial.printf( "  Sensor state: %s\n", sensorStateLabel(currentState));
            Serial.println(F("  Timer reset"));
            Serial.println(F("────────────────────────────────────────────────────────────"));
            // Send pulse on metal detection (LOW)
            if (currentState == LOW) {
                sendSensorEvent("pulse", "active");
            }
        }

        downtimeReported = false;
    }

    // ── 2. Check if 10 seconds of unchanged state reached ───────────────────
    uint32_t elapsed = now - stateStartTime;

    if (elapsed >= DOWNTIME_THRESHOLD_MS && !downtimeReported) {
        downtimeReported = true;
        downtimeActive   = true;
        digitalWrite(ONBOARD_LED_PIN, HIGH);  // LED ON = downtime

        Serial.println();
        Serial.println(F("********************************************************"));
        if (currentState == LOW) {
            Serial.println(F("  DOWNTIME: Sensor ON for 10 seconds -> Sending to Backend"));
            sendSensorEvent("fault", "fault", "sensor_stuck_on_10s");
        } else {
            Serial.println(F("  DOWNTIME: Sensor OFF for 10 seconds -> Sending to Backend"));
            sendSensorEvent("fault", "fault", "sensor_no_metal_10s");
        }
        Serial.println(F("********************************************************"));
        Serial.println();
    }

    // ── 3. Periodic status print (every 2s) ─────────────────────────────────
    if (now - lastStatusPrint >= STATUS_PRINT_INTERVAL_MS) {
        lastStatusPrint = now;

        Serial.printf( "[%7u ms] Sensor: %-3s | Held: %5u ms / %u ms | LED: %-3s | %s\n",
                       now,
                       sensorStateLabel(currentState),
                       elapsed,
                       DOWNTIME_THRESHOLD_MS,
                       downtimeActive ? "ON" : "OFF",
                       downtimeActive ? "DOWNTIME" : (downtimeReported ? "DOWNTIME (reported)" : "normal"));
    }

    // ── 4. Periodic Backend Heartbeat (every 10s) ───────────────────────────
    if (now - lastHeartbeatSent >= BACKEND_HEARTBEAT_INTERVAL_MS) {
        lastHeartbeatSent = now;
        sendHeartbeat();
    }

    // ── 5. Yield to RTOS watchdog ───────────────────────────────────────────
    delay(1);
}