#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <time.h>

// =============================================================================
// ESP32 SENSOR 5 — FINISHED PRODUCT (PIPE OUTPUT) PRODUCTION COUNTER
// =============================================================================
// Node ID   : S-05
// Purpose   : Count manufactured pipes passing flying cutter / output station.
// Logic     : Pipe arrives (HIGH -> LOW) -> Increments counter, sends 'pulse' (active), LED ON.
//             Pipe leaves  (LOW -> HIGH) -> Counter re-armed for next pipe, LED OFF.
//             S-05 never participates in absence detection or downtime.
// =============================================================================

// ── 1. Node Identity & Backend Configuration ────────────────────────────────
static const char* NODE_ID        = "S-05";
static const char* NODE_ROLE_NAME = "Production Output Cutting Monitor";
static const char* DEVICE_ID      = "esp32-m01-s05";
static const char* DEVICE_KEY     = "php-s-05-device-key-placeholder";
static const char* API_BASE_URL   = "http://10.72.136.174:3000";

// ── 2. Network Configuration ────────────────────────────────────────────────
static const char* WIFI_SSID      = "Realme 11 280";
static const char* WIFI_PASS      = "12345678900987654321";

// ── 3. Hardware Pin Configuration (ESP32 DevKit V1) ─────────────────────────
static const uint8_t SENSOR_INPUT_PIN        = 4;     // GPIO 4 (Collector of PC817 optocoupler)
static const uint8_t ONBOARD_LED_PIN         = 2;     // GPIO 2 (Pipe detection indicator)
static const uint8_t SENSOR_DETECTED_STATE   = LOW;   // LOW  = Metal pipe present
static const uint8_t SENSOR_CLEARED_STATE    = HIGH;  // HIGH = Gap between pipes

// ── 4. Timing & Filter Settings ─────────────────────────────────────────────
static const uint32_t SENSOR_DEBOUNCE_MS     = 50;    // 50ms stable debounce duration
static const uint32_t HEARTBEAT_INTERVAL_MS  = 10000; // 10s background heartbeat
static const uint32_t SERIAL_BAUD_RATE       = 115200;

// ── 5. Runtime State ────────────────────────────────────────────────────────
static int       currentRawReading      = HIGH;
static int       lastRawReading         = HIGH;
static uint32_t  lastDebounceTime       = 0;
static int       debouncedPinState      = HIGH;
static int       previousConfirmedState = HIGH;

static uint32_t  pipeProductionCount    = 0;
static uint32_t  lastHeartbeatTime      = 0;
static bool      activitySinceHb        = false;

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

void sendPulseEvent() {
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
        Serial.printf("  [PIPE COUNTED] Total: %u\n", pipeProductionCount);
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
        Serial.printf("  [HEARTBEAT] Seq: %llu | Output count: %u\n", hbSequence, pipeProductionCount);
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
    Serial.printf( "  Current Count : %u pipes\n", pipeProductionCount);
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
    if (WiFi.status() != WL_CONNECTED) {
        WiFi.reconnect();
    }

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

                // Persist counter periodically in NVS
                if (pipeProductionCount % 10 == 0) {
                    prefs.begin("iot-node", false);
                    prefs.putUInt("pipe_cnt", pipeProductionCount);
                    prefs.end();
                }

                sendPulseEvent();
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
        sendHeartbeat();
    }

    delay(1);
}
