#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <esp_timer.h>
#include <soc/gpio_struct.h>
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
    uint32_t completedAtMs;
};

struct CompletedCut {
    uint32_t completedAtMs;
    uint32_t passDurationMs;
};

struct OutputIsrState {
    int              confirmedPin;
    int              candidatePin;
    uint32_t         candidateStartMs;
    OutputCycleState cycleState;
    uint32_t         metalDetectedAtMs;
    bool             metalDetectedPendingLog;
    uint32_t         lastMetalDetectedLogMs;
    CompletedCut     cuts[MAX_PENDING_PULSES];
    size_t           cutHead;
    size_t           cutCount;
};

// ── Runtime State ───────────────────────────────────────────────────────────
static portMUX_TYPE     sensorMux            = portMUX_INITIALIZER_UNLOCKED;
static OutputIsrState   isrState             = {
    SENSOR_CLEARED_STATE,
    SENSOR_CLEARED_STATE,
    0,
    WAITING_FOR_METAL,
    0,
    false,
    0,
    {},
    0,
    0
};

static int              debouncedPinState    = SENSOR_CLEARED_STATE;
static OutputCycleState cycleState           = WAITING_FOR_METAL;
static uint32_t         metalDetectedAtMs    = 0;

static uint32_t         pipeProductionCount  = 0;
static uint32_t         rejectedOutputCount  = 0;
static uint32_t         lastHeartbeatTime    = 0;
static uint32_t         lastEventRetryTime   = 0;
static uint32_t         lastStatusPrintTime  = 0;
static bool             activitySinceHb      = false;
static bool             ntpSynced            = false;

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

void IRAM_ATTR advanceOutputIsrState(OutputIsrState& s, uint32_t nowMs, int rawLevel) {
    if (s.candidatePin != s.confirmedPin && (nowMs - s.candidateStartMs) >= SENSOR_DEBOUNCE_MS) {
        uint32_t confirmMs = s.candidateStartMs + SENSOR_DEBOUNCE_MS;
        s.confirmedPin = s.candidatePin;
        if (s.confirmedPin == SENSOR_DETECTED_STATE) {
            if (s.cycleState == WAITING_FOR_METAL) {
                s.cycleState              = METAL_DETECTED;
                s.metalDetectedAtMs       = confirmMs;
                s.metalDetectedPendingLog = true;
                s.lastMetalDetectedLogMs  = confirmMs;
            }
        } else if (s.confirmedPin == SENSOR_CLEARED_STATE) {
            if (s.cycleState == METAL_DETECTED) {
                uint32_t passDurationMs = confirmMs - s.metalDetectedAtMs;
                s.cycleState = WAITING_FOR_METAL;
                if (s.cutCount >= MAX_PENDING_PULSES) {
                    s.cutHead = (s.cutHead + 1) % MAX_PENDING_PULSES;
                    s.cutCount--;
                }
                size_t tail = (s.cutHead + s.cutCount) % MAX_PENDING_PULSES;
                s.cuts[tail].completedAtMs  = confirmMs;
                s.cuts[tail].passDurationMs = passDurationMs;
                s.cutCount++;
            }
        }
    }

    if (rawLevel != s.candidatePin) {
        s.candidatePin     = rawLevel;
        s.candidateStartMs = nowMs;
    }
}

void IRAM_ATTR onSensorPinChangeISR() {
    portENTER_CRITICAL_ISR(&sensorMux);
    uint32_t nowMs = (uint32_t)(esp_timer_get_time() / 1000ULL);
    int rawLevel = (GPIO.in >> SENSOR_INPUT_PIN) & 0x1;
    advanceOutputIsrState(isrState, nowMs, rawLevel);
    portEXIT_CRITICAL_ISR(&sensorMux);
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

bool isClockSynced() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm timeinfo;
    gmtime_r(&tv.tv_sec, &timeinfo);
    return (timeinfo.tm_year + 1900) >= 2026;
}

String formatUint64(uint64_t val) {
    char buf[24];
    snprintf(buf, sizeof(buf), "%llu", (unsigned long long)val);
    return String(buf);
}

String isoAtAgeMs(uint32_t ageMs) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    int64_t totalUs = (int64_t)tv.tv_sec * 1000000LL + (int64_t)tv.tv_usec - (int64_t)ageMs * 1000LL;
    if (totalUs < 0) totalUs = 0;
    time_t sec = (time_t)(totalUs / 1000000LL);
    long ms = (long)((totalUs % 1000000LL) / 1000LL);
    struct tm timeinfo;
    gmtime_r(&sec, &timeinfo);
    char buf[64];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d.%03ldZ",
        timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
        timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, ms);
    return String(buf);
}

String isoNow() {
    return isoAtAgeMs(0);
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
    http.setConnectTimeout(2000);
    http.setTimeout(2500);
    http.begin(String(API_BASE_URL) + "/api/iot/heartbeats");
    http.addHeader("Content-Type", "application/json");
    http.addHeader("x-device-id", DEVICE_ID);
    http.addHeader("x-device-key", DEVICE_KEY);

    int code = http.POST(jsonBody);
    if (code < 200 || code >= 300) {
        String resp = (code > 0) ? http.getString() : "";
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
    http.setConnectTimeout(2000);
    http.setTimeout(2500);
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
        if (acceptedFlag == 1) {
            return PULSE_ACCEPTED;
        }
        Serial.printf("  [API Transient Error %d] Missing outputAccepted in response; will retry pulse %s\n", code, pulse.eventId);
        return PULSE_RETRY_LATER;
    }

    if (code <= 0 || code == 408 || code == 429 || code >= 500) {
        Serial.printf("  [API Transient Error %d] Will retry pulse %s\n", code, pulse.eventId);
        return PULSE_RETRY_LATER;
    }

    Serial.printf("  [API Rejected %d] %s\n", code, resp.c_str());
    outReason = String("http_") + String(code);
    return PULSE_REJECTED;
}

bool enqueuePulseEvent(uint32_t passDurationMs, uint32_t completedAtMs, uint32_t nowMs) {
    String uuid = generateUuidV4();
    uint32_t ageMs = (nowMs - completedAtMs) < 0x80000000UL ? (nowMs - completedAtMs) : 0;
    String timestamp = isClockSynced() ? isoAtAgeMs(ageMs) : "";

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
    pendingQueue[tail].completedAtMs = completedAtMs;
    pendingCount++;
    return true;
}

bool flushPendingPulses() {
    if (!isClockSynced()) {
        lastEventRetryTime = (uint32_t)(esp_timer_get_time() / 1000ULL);
        return false;
    }
    while (pendingCount > 0) {
        PendingPulse& front = pendingQueue[pendingHead];
        if (front.recordedAt[0] == '\0' && isClockSynced()) {
            uint32_t nowMs = (uint32_t)(esp_timer_get_time() / 1000ULL);
            uint32_t ageMs = (nowMs - front.completedAtMs) < 0x80000000UL ? (nowMs - front.completedAtMs) : 0;
            String syncedTs = isoAtAgeMs(ageMs);
            strncpy(front.recordedAt, syncedTs.c_str(), sizeof(front.recordedAt) - 1);
            front.recordedAt[sizeof(front.recordedAt) - 1] = '\0';
        }
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
            lastEventRetryTime = (uint32_t)(esp_timer_get_time() / 1000ULL);
            return false;
        }
    }
    return true;
}

void sendHeartbeat() {
    if (!isClockSynced()) {
        return;
    }
    hbSequence++;
    String body = String("{") +
        "\"heartbeatId\":\"" + generateUuidV4() + "\"," +
        "\"bootId\":\"" + bootId + "\"," +
        "\"bootCounter\":\"" + String(bootCounter) + "\"," +
        "\"sequence\":\"" + formatUint64(hbSequence) + "\"," +
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
    uint32_t wifiSetupStart = millis();
    uint32_t wifiAttemptStart = wifiSetupStart;
    while (WiFi.status() != WL_CONNECTED && (millis() - wifiSetupStart < 15000UL)) {
        delay(500);
        Serial.print(".");
        if (millis() - wifiAttemptStart >= WIFI_RECONNECT_INTERVAL_MS) {
            wifiAttemptStart = millis();
            WiFi.disconnect();
            WiFi.begin(WIFI_SSID, WIFI_PASS);
        }
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\nWiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println(F("\nWiFi not ready within 15 s — continuing offline; background reconnect active."));
    }

    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("Syncing NTP time");
        uint32_t ntpSetupStart = millis();
        while (!isClockSynced() && (millis() - ntpSetupStart < 15000UL)) {
            reconnectWifiIfNeeded();
            delay(500);
            Serial.print(".");
        }
    }
    if (isClockSynced()) {
        ntpSynced = true;
        Serial.printf("\nNTP Synced: %s\n", isoNow().c_str());
        uint32_t epochNow = (uint32_t)time(nullptr);
        if (epochNow > bootCounter) {
            bootCounter = epochNow;
            prefs.begin("iot-node", false);
            prefs.putUInt("boot_cnt", bootCounter);
            prefs.end();
        }
    } else {
        Serial.println(F("\nNTP not synced yet — will complete in background once online."));
    }

    uint32_t now = 0;
    int initialReading = SENSOR_CLEARED_STATE;
    portENTER_CRITICAL(&sensorMux);
    now = (uint32_t)(esp_timer_get_time() / 1000ULL);
    initialReading = (GPIO.in >> SENSOR_INPUT_PIN) & 0x1;
    isrState.confirmedPin            = initialReading;
    isrState.candidatePin            = initialReading;
    isrState.candidateStartMs        = now;
    isrState.cycleState              = (initialReading == SENSOR_DETECTED_STATE) ? METAL_DETECTED : WAITING_FOR_METAL;
    isrState.metalDetectedAtMs       = (initialReading == SENSOR_DETECTED_STATE) ? now : 0;
    isrState.metalDetectedPendingLog = false;
    isrState.lastMetalDetectedLogMs  = 0;
    isrState.cutHead                 = 0;
    isrState.cutCount                = 0;
    portEXIT_CRITICAL(&sensorMux);

    debouncedPinState   = initialReading;
    cycleState          = isrState.cycleState;
    metalDetectedAtMs   = isrState.metalDetectedAtMs;
    digitalWrite(ONBOARD_LED_PIN, (initialReading == SENSOR_DETECTED_STATE) ? HIGH : LOW);

    lastHeartbeatTime   = now;
    lastEventRetryTime  = now;
    lastStatusPrintTime = now;

    attachInterrupt(digitalPinToInterrupt(SENSOR_INPUT_PIN), onSensorPinChangeISR, CHANGE);

    Serial.printf("Initial Cycle State: %s\n", getCycleStateString(cycleState));
    sendHeartbeat();
}

// ── Main Loop ───────────────────────────────────────────────────────────────
void loop() {
    reconnectWifiIfNeeded();
    if (!ntpSynced && isClockSynced()) {
        ntpSynced = true;
        uint32_t epochNow = (uint32_t)time(nullptr);
        if (epochNow > bootCounter) {
            bootCounter = epochNow;
            prefs.begin("iot-node", false);
            prefs.putUInt("boot_cnt", bootCounter);
            prefs.end();
        }
        Serial.printf("  [NTP] Background sync complete: %s\n", isoNow().c_str());
    }

    uint32_t now = 0;
    int rawReading = SENSOR_CLEARED_STATE;
    bool logMetalDetected = false;
    uint32_t metalLogMs = 0;
    CompletedCut drainedCuts[MAX_PENDING_PULSES];
    size_t drainedCount = 0;

    portENTER_CRITICAL(&sensorMux);
    now = (uint32_t)(esp_timer_get_time() / 1000ULL);
    rawReading = (GPIO.in >> SENSOR_INPUT_PIN) & 0x1;
    advanceOutputIsrState(isrState, now, rawReading);

    debouncedPinState = isrState.confirmedPin;
    cycleState        = isrState.cycleState;
    metalDetectedAtMs = isrState.metalDetectedAtMs;

    if (isrState.metalDetectedPendingLog) {
        logMetalDetected = true;
        metalLogMs = isrState.lastMetalDetectedLogMs;
        isrState.metalDetectedPendingLog = false;
    }

    while (isrState.cutCount > 0 && drainedCount < MAX_PENDING_PULSES) {
        drainedCuts[drainedCount++] = isrState.cuts[isrState.cutHead];
        isrState.cutHead = (isrState.cutHead + 1) % MAX_PENDING_PULSES;
        isrState.cutCount--;
    }
    portEXIT_CRITICAL(&sensorMux);

    digitalWrite(ONBOARD_LED_PIN, (debouncedPinState == SENSOR_DETECTED_STATE) ? HIGH : LOW);

    if (logMetalDetected) {
        Serial.printf("[%lu ms] [METAL DETECTED] Pipe present — waiting for NO METAL to count output\n",
            (unsigned long)metalLogMs);
    }

    bool eventFailedThisPass = false;

    if (drainedCount > 0) {
        activitySinceHb = true;
        for (size_t i = 0; i < drainedCount; i++) {
            Serial.printf("[%lu ms] [CYCLE COMPLETE] METAL -> NO METAL (%lu ms) — Sending 1 output pulse\n",
                (unsigned long)drainedCuts[i].completedAtMs,
                (unsigned long)drainedCuts[i].passDurationMs);
            enqueuePulseEvent(drainedCuts[i].passDurationMs, drainedCuts[i].completedAtMs, now);
        }
        if (!flushPendingPulses()) {
            eventFailedThisPass = true;
        }
    }

    // Retry any network-queued output pulses
    if (pendingCount > 0 && !eventFailedThisPass && (now - lastEventRetryTime >= EVENT_RETRY_INTERVAL_MS)) {
        lastEventRetryTime = now;
        if (!flushPendingPulses()) {
            eventFailedThisPass = true;
        }
    }

    // Periodic heartbeat (online connectivity only; S-05 never triggers absence/fault/downtime)
    if (now - lastHeartbeatTime >= HEARTBEAT_INTERVAL_MS) {
        lastHeartbeatTime = now;
        if (pendingCount > 0 && !eventFailedThisPass) {
            if (!flushPendingPulses()) {
                eventFailedThisPass = true;
            }
        }
        if (!eventFailedThisPass) {
            sendHeartbeat();
        }
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
