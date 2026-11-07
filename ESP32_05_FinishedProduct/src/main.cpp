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
// Logic     : 1. Metal detected (LOW confirmed)     -> Arms cycle (METAL_DETECTED), LED ON.
//             2. No metal detected (HIGH confirmed) -> Completes METAL -> NO METAL cycle,
//                LED OFF, sends 1 'pulse' ('active') output event to web backend.
//             3. S-05 is completely independent of idle, fault, and absence thresholds.
//             4. If machine downtime is active on the web, the backend responds with
//                outputAccepted = false (reason: 'machine_downtime') and S-05 does NOT
//                increment its accepted output count.
// =============================================================================

// Identity, WiFi, backend URL, pins and timing live in config.h.

enum OutputCycleState {
    WAITING_FOR_METAL = 0, // No metal present (HIGH) — waiting for pipe to enter sensor zone
    METAL_DETECTED    = 1  // Metal present (LOW) — waiting for transition to NO METAL to count 1 output
};

enum PulseDeliveryResult {
    PULSE_ACCEPTED    = 0, // Accepted by web backend; output count updated
    PULSE_REJECTED    = 1, // Explicitly rejected by web backend (e.g., machine_downtime); do not count or retry
    PULSE_RETRY_LATER = 2  // Network / transient server failure; keep queued with same eventId & recordedAt
};

struct PendingPulse {
    char     eventId[37];
    char     recordedAt[32];
    int32_t  rssi;
    uint32_t passDurationMs;
};

// ── Runtime State ───────────────────────────────────────────────────────────
static int              lastRawReading       = SENSOR_CLEARED_STATE;
static uint32_t         lastDebounceTime     = 0;
static int              debouncedPinState    = SENSOR_CLEARED_STATE;
static OutputCycleState cycleState           = WAITING_FOR_METAL;
static uint32_t         metalDetectedAtMs    = 0;

static uint32_t         pipeProductionCount  = 0;
static uint32_t         rejectedOutputCount  = 0;
static uint32_t         lastHeartbeatTime    = 0;
static uint32_t         lastEventRetryTime   = 0;
static uint32_t         lastStatusPrintTime  = 0;
static bool             activitySinceHb      = false;

static PendingPulse     pendingQueue[MAX_PENDING_PULSES];
static size_t           pendingHead          = 0;
static size_t           pendingCount         = 0;

static Preferences      prefs;
static String           bootId;
static uint32_t         bootCounter          = 1;
static uint64_t         hbSequence           = 0;

// ── Helpers ─────────────────────────────────────────────────────────────────
const char* getCycleStateString(OutputCycleState state) {
    return (state == METAL_DETECTED)
        ? "METAL_DETECTED (Waiting for NO METAL)"
        : "WAITING_FOR_METAL (No Metal)";
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

int parseJsonBooleanField(const String& json, const char* key) {
    String pattern = String("\"") + key + "\":";
    int idx = json.indexOf(pattern);
    if (idx < 0) return -1;
    idx += pattern.length();
    while (idx < (int)json.length() && (json[idx] == ' ' || json[idx] == '\t')) {
        idx++;
    }
    if (json.startsWith("true", idx)) return 1;
    if (json.startsWith("false", idx)) return 0;
    return -1;
}

String parseJsonStringField(const String& json, const char* key) {
    String pattern = String("\"") + key + "\":";
    int idx = json.indexOf(pattern);
    if (idx < 0) return "";
    idx += pattern.length();
    while (idx < (int)json.length() && (json[idx] == ' ' || json[idx] == '\t')) {
        idx++;
    }
    if (idx >= (int)json.length() || json[idx] != '"') return "";
    int start = idx + 1;
    int end = json.indexOf('"', start);
    if (end < 0) return "";
    return json.substring(start, end);
}

bool postHeartbeatToBackend(const String& jsonBody) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("  [API] Skipped heartbeat: WiFi disconnected"));
        return false;
    }
    HTTPClient http;
    http.setTimeout(4000);
    http.begin(String(API_BASE_URL) + "/api/iot/heartbeats");
    http.addHeader("Content-Type", "application/json");
    http.addHeader("x-device-id", DEVICE_ID);
    http.addHeader("x-device-key", DEVICE_KEY);

    int code = http.POST(jsonBody);
    if (code < 200 || code >= 300) {
        String resp = http.getString();
        Serial.printf("  [API Heartbeat Error %d] %s\n", code, resp.c_str());
    }
    http.end();
    return (code >= 200 && code < 300);
}

PulseDeliveryResult postPulseToBackend(const PendingPulse& pulse, String& outReason) {
    outReason = "";
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("  [API] Pulse queued: WiFi disconnected"));
        return PULSE_RETRY_LATER;
    }

    String meta = String("{\"rssi\":") + String(pulse.rssi) +
                  ",\"passDurationMs\":" + String((unsigned long)pulse.passDurationMs) + "}";
    String body = String("{") +
        "\"eventId\":\"" + String(pulse.eventId) + "\"," +
        "\"eventType\":\"pulse\"," +
        "\"signal\":\"active\"," +
        "\"recordedAt\":\"" + String(pulse.recordedAt) + "\"," +
        "\"metadata\":" + meta +
    "}";

    HTTPClient http;
    http.setTimeout(4000);
    http.begin(String(API_BASE_URL) + "/api/iot/events");
    http.addHeader("Content-Type", "application/json");
    http.addHeader("x-device-id", DEVICE_ID);
    http.addHeader("x-device-key", DEVICE_KEY);

    int code = http.POST(body);
    String resp = (code > 0) ? http.getString() : "";
    http.end();

    if (code >= 200 && code < 300) {
        int acceptedFlag = parseJsonBooleanField(resp, "outputAccepted");
        if (acceptedFlag == 0) {
            outReason = parseJsonStringField(resp, "outputRejectionReason");
            if (outReason.length() == 0) outReason = "rejected_by_server";
            return PULSE_REJECTED;
        }
        return PULSE_ACCEPTED;
    }

    if (code <= 0 || code == 408 || code == 429 || code >= 500) {
        Serial.printf("  [API Transient Error %d] Will retry pulse %s\n", code, pulse.eventId);
        return PULSE_RETRY_LATER;
    }

    Serial.printf("  [API Rejected %d] %s\n", code, resp.c_str());
    outReason = String("http_") + String(code);
    return PULSE_REJECTED;
}

bool enqueuePulseEvent(uint32_t passDurationMs) {
    String uuid = generateUuidV4();
    String timestamp = isoNow();

    if (pendingCount >= MAX_PENDING_PULSES) {
        // Drop oldest unconfirmed item only if buffer is completely full
        Serial.println(F("  [QUEUE] Pending buffer full — dropping oldest queued pulse"));
        pendingHead = (pendingHead + 1) % MAX_PENDING_PULSES;
        pendingCount--;
    }

    size_t tail = (pendingHead + pendingCount) % MAX_PENDING_PULSES;
    strncpy(pendingQueue[tail].eventId, uuid.c_str(), sizeof(pendingQueue[tail].eventId) - 1);
    pendingQueue[tail].eventId[sizeof(pendingQueue[tail].eventId) - 1] = '\0';
    strncpy(pendingQueue[tail].recordedAt, timestamp.c_str(), sizeof(pendingQueue[tail].recordedAt) - 1);
    pendingQueue[tail].recordedAt[sizeof(pendingQueue[tail].recordedAt) - 1] = '\0';
    pendingQueue[tail].rssi = WiFi.RSSI();
    pendingQueue[tail].passDurationMs = passDurationMs;
    pendingCount++;
    return true;
}

void flushPendingPulses() {
    while (pendingCount > 0) {
        const PendingPulse& front = pendingQueue[pendingHead];
        String rejectionReason;
        PulseDeliveryResult outcome = postPulseToBackend(front, rejectionReason);

        if (outcome == PULSE_ACCEPTED) {
            pipeProductionCount++;
            prefs.begin("iot-node", false);
            prefs.putUInt("pipe_cnt", pipeProductionCount);
            prefs.end();

            Serial.printf("  [OUTPUT ACCEPTED] Pipe counted on web! Total accepted: %lu\n",
                (unsigned long)pipeProductionCount);

            pendingHead = (pendingHead + 1) % MAX_PENDING_PULSES;
            pendingCount--;
        } else if (outcome == PULSE_REJECTED) {
            rejectedOutputCount++;
            Serial.printf("  [OUTPUT REJECTED] Reason: %s | Accepted count unchanged: %lu (Rejected total: %lu)\n",
                rejectionReason.c_str(),
                (unsigned long)pipeProductionCount,
                (unsigned long)rejectedOutputCount);

            pendingHead = (pendingHead + 1) % MAX_PENDING_PULSES;
            pendingCount--;
        } else {
            // PULSE_RETRY_LATER — keep front item intact for next retry window
            lastEventRetryTime = millis();
            break;
        }
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
    if (postHeartbeatToBackend(body)) {
        Serial.printf("  [HEARTBEAT] Seq: %llu | Accepted outputs: %lu | Rejected: %lu | Queued: %u\n",
            hbSequence,
            (unsigned long)pipeProductionCount,
            (unsigned long)rejectedOutputCount,
            (unsigned int)pendingCount);
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
    Serial.printf( "  %s — %s [%s] (v%s)\n", NODE_ID, NODE_ROLE_NAME, DEVICE_ID, FIRMWARE_VERSION);
    Serial.println(F("=============================================================="));
    Serial.printf( "  Backend URL   : %s\n", API_BASE_URL);
    Serial.printf( "  Sensor Pin    : GPIO %d (INPUT_PULLUP via PC817)\n", SENSOR_INPUT_PIN);
    Serial.printf( "  LED Pin       : GPIO %d (Metal Presence Indicator)\n", ONBOARD_LED_PIN);
    Serial.println(F("  Output Rule   : METAL (LOW) -> NO METAL (HIGH) = 1 Output"));
    Serial.println(F("  Isolation     : Unaffected by Idle/Fault; rejected only on Downtime"));
    Serial.printf( "  Current Count : %lu accepted pipes\n", (unsigned long)pipeProductionCount);
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

    // Monotonic bootCounter sync using Unix epoch so heartbeats are never marked stale
    uint32_t epochNow = (uint32_t)time(nullptr);
    if (epochNow > bootCounter) {
        bootCounter = epochNow;
        prefs.begin("iot-node", false);
        prefs.putUInt("boot_cnt", bootCounter);
        prefs.end();
    }

    uint32_t now       = millis();
    int initialReading = digitalRead(SENSOR_INPUT_PIN);
    lastRawReading     = initialReading;
    debouncedPinState  = initialReading;
    lastDebounceTime   = now;

    if (initialReading == SENSOR_DETECTED_STATE) {
        cycleState        = METAL_DETECTED;
        metalDetectedAtMs = now;
        digitalWrite(ONBOARD_LED_PIN, HIGH);
    } else {
        cycleState        = WAITING_FOR_METAL;
        metalDetectedAtMs = 0;
        digitalWrite(ONBOARD_LED_PIN, LOW);
    }

    lastHeartbeatTime   = now;
    lastStatusPrintTime = now;

    Serial.printf("Initial Cycle State: %s\n", getCycleStateString(cycleState));
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

            // Step 1 of cycle: METAL DETECTED (LOW confirmed) -> Arm cycle, do NOT count yet
            if (debouncedPinState == SENSOR_DETECTED_STATE) {
                if (cycleState == WAITING_FOR_METAL) {
                    cycleState        = METAL_DETECTED;
                    metalDetectedAtMs = now;
                    digitalWrite(ONBOARD_LED_PIN, HIGH);
                    Serial.printf("[%lu ms] [METAL DETECTED] Pipe present — waiting for NO METAL to count output\n",
                        (unsigned long)now);
                }
            }
            // Step 2 of cycle: NO METAL DETECTED (HIGH confirmed) after METAL -> 1 Completed Output!
            else if (debouncedPinState == SENSOR_CLEARED_STATE) {
                digitalWrite(ONBOARD_LED_PIN, LOW);

                if (cycleState == METAL_DETECTED) {
                    uint32_t passDurationMs = now - metalDetectedAtMs;
                    cycleState  = WAITING_FOR_METAL;
                    activitySinceHb = true;

                    Serial.printf("[%lu ms] [CYCLE COMPLETE] METAL -> NO METAL (%lu ms) — Sending 1 output pulse\n",
                        (unsigned long)now,
                        (unsigned long)passDurationMs);

                    enqueuePulseEvent(passDurationMs);
                    flushPendingPulses();
                }
            }
        }
    }

    // Retry any network-queued output pulses
    if (pendingCount > 0 && (now - lastEventRetryTime >= EVENT_RETRY_INTERVAL_MS)) {
        lastEventRetryTime = now;
        flushPendingPulses();
    }

    // Periodic heartbeat (online connectivity only; S-05 never triggers absence/fault/downtime)
    if (now - lastHeartbeatTime >= HEARTBEAT_INTERVAL_MS) {
        lastHeartbeatTime = now;
        if (pendingCount > 0) {
            flushPendingPulses();
        }
        sendHeartbeat();
    }

    // Periodic Serial status summary
    if (now - lastStatusPrintTime >= STATUS_PRINT_INTERVAL_MS) {
        lastStatusPrintTime = now;
        if (cycleState == METAL_DETECTED) {
            Serial.printf("[STATUS] %s (%lu ms) | Accepted: %lu | Rejected: %lu | Queued: %u | RSSI: %d dBm\n",
                getCycleStateString(cycleState),
                (unsigned long)(now - metalDetectedAtMs),
                (unsigned long)pipeProductionCount,
                (unsigned long)rejectedOutputCount,
                (unsigned int)pendingCount,
                WiFi.RSSI());
        } else {
            Serial.printf("[STATUS] %s | Accepted: %lu | Rejected: %lu | Queued: %u | RSSI: %d dBm\n",
                getCycleStateString(cycleState),
                (unsigned long)pipeProductionCount,
                (unsigned long)rejectedOutputCount,
                (unsigned int)pendingCount,
                WiFi.RSSI());
        }
    }

    delay(1);
}
