#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <time.h>

// =============================================================================
// ESP32 SENSOR 3 — MACHINE MAIN SENSOR (SPINDLE ROTATION MONITOR)
// =============================================================================
// Node ID   : S-03
// Purpose   : Monitor continuous spindle/shaft rotation via inductive proximity sensor.
//             - Rotating (signal toggles): transmits throttled activity pulses.
//             - Stopped (constant LOW or constant HIGH): stops emitting pulses.
//             - Backend absence watchdog evaluates silence and triggers downtime.
// =============================================================================

// ── 1. Node Identity & Backend Configuration ────────────────────────────────
static const char* NODE_ID      = "S-03";
static const char* DEVICE_ID    = "esp32-m01-s03";
static const char* DEVICE_KEY   = "php-s-03-ee1ff4f1cd060acffb5b19b40dc2a1fe7be4";
static const char* API_BASE_URL = "http://10.72.136.174:3000";

// ── 2. Network Configuration ────────────────────────────────────────────────
static const char* WIFI_SSID    = "Realme 11 280";
static const char* WIFI_PASS    = "12345678900987654321";

// ── 3. Hardware Pin Configuration (ESP32 DevKit V1) ─────────────────────────
static const uint8_t SENSOR_INPUT_PIN = 4;   // GPIO 4 (Collector of PC817 optocoupler)
static const uint8_t ONBOARD_LED_PIN  = 2;   // GPIO 2 (Activity pulse indicator)

// ── 4. Timing & Filter Settings ─────────────────────────────────────────────
static const uint32_t SENSOR_DEBOUNCE_MS       = 30;    // 30ms proximity edge debounce
static const uint32_t MIN_PULSE_INTERVAL_MS    = 1500;  // Max 1 HTTP event per 1.5s while rotating
static const uint32_t HEARTBEAT_INTERVAL_MS    = 10000; // Background heartbeat every 10s
static const uint32_t STATUS_PRINT_INTERVAL_MS = 3000;  // Serial status update interval
static const uint32_t SERIAL_BAUD_RATE         = 115200;

// ── 5. Runtime State ────────────────────────────────────────────────────────
static int       currentState        = HIGH;
static int       lastRawReading      = HIGH;
static uint32_t  lastDebounceTime    = 0;

static bool      hasRotatedSinceSend = false;
static uint32_t  lastPulseSentTime   = 0;
static uint32_t  lastHeartbeatSent   = 0;
static uint32_t  lastStatusPrint     = 0;
static bool      activitySinceHb     = false;
static uint32_t  pulseCount          = 0;

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
    if (code < 200 || code >= 300) {
        String resp = http.getString();
        Serial.printf("  [API Error %d] %s\n", code, resp.c_str());
    }
    http.end();
    return (code >= 200 && code < 300);
}

void sendSensorEvent(const char* eventType, const char* signal) {
    String meta = String("{\"rssi\":") + String(WiFi.RSSI()) +
                  ",\"pinState\":\"" + (currentState == LOW ? "LOW" : "HIGH") + "\"" +
                  ",\"totalPulses\":" + String(pulseCount) + "}";

    String body = String("{") +
        "\"eventId\":\"" + generateUuidV4() + "\"," +
        "\"eventType\":\"" + eventType + "\"," +
        "\"signal\":\"" + signal + "\"," +
        "\"recordedAt\":\"" + isoNow() + "\"," +
        "\"metadata\":" + meta +
    "}";

    if (postToBackend("/api/iot/events", body)) {
        Serial.printf("  [EVENT SENT] %s -> %s (pulse #%u)\n", eventType, signal, pulseCount);
    }
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

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("Connecting to WiFi (%s)", WIFI_SSID);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\nWiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());

    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    struct tm timeinfo;
    Serial.print("Syncing NTP time");
    while (!getLocalTime(&timeinfo) || (timeinfo.tm_year + 1900) < 2026) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\nNTP Synced: %s\n", isoNow().c_str());

    currentState      = digitalRead(SENSOR_INPUT_PIN);
    lastRawReading    = currentState;
    lastHeartbeatSent = millis();
    lastStatusPrint   = millis();

    sendHeartbeat();
    sendSensorEvent("pulse", "active");
    activitySinceHb = true;
}

// ── Main Loop ───────────────────────────────────────────────────────────────
void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        WiFi.reconnect();
    }

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

        digitalWrite(ONBOARD_LED_PIN, HIGH);
    }

    // 2. Transmit throttled rotation pulse while running
    if (hasRotatedSinceSend && (now - lastPulseSentTime >= MIN_PULSE_INTERVAL_MS)) {
        lastPulseSentTime = now;
        hasRotatedSinceSend = false;

        digitalWrite(ONBOARD_LED_PIN, LOW);
        sendSensorEvent("pulse", "active");
    }

    // Turn LED off when stationary
    if (!hasRotatedSinceSend && (now - lastDebounceTime > 200)) {
        digitalWrite(ONBOARD_LED_PIN, LOW);
    }

    // 3. 10s Heartbeat in background
    if (now - lastHeartbeatSent >= HEARTBEAT_INTERVAL_MS) {
        lastHeartbeatSent = now;
        sendHeartbeat();
    }

    // 4. Status log every 3s
    if (now - lastStatusPrint >= STATUS_PRINT_INTERVAL_MS) {
        lastStatusPrint = now;
        uint32_t silentDurationMs = now - lastPulseSentTime;
        Serial.printf("[%7u ms] State: %-22s | Pulses: %4u | Silence: %5u ms | WiFi: %ddBm\n",
                      now, sensorStateLabel(currentState), pulseCount, silentDurationMs, WiFi.RSSI());
    }

    delay(1);
}
