#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <esp_timer.h>
#include <soc/gpio_struct.h>
#include <time.h>
#include "config.h"
#include "spindle_logic.h"

// =============================================================================
// ESP32 SENSOR 3 — MACHINE MAIN SENSOR (SPINDLE ROTATION MONITOR)
// =============================================================================
// Node ID   : S-03
// Role      : Machine Main Spindle Monitor
// Logic     : - Spindle rotating (ON/OFF transitions between LOW and HIGH) :
//                 * Running -> emits 'pulse' / 'recovered' (active), LED OFF
//             - Spindle stopped (Constant metal LOW OR Constant no-metal HIGH
//               for >= ROTATION_TIMEOUT_MS without ON/OFF transitions) :
//                 * Within dynamic trigger limit (< triggerLimitMs) : Idle  -> emits 'idle' (idle), LED ON
//                 * Past dynamic trigger limit   (>= triggerLimitMs): Fault -> emits 'fault' (fault), LED ON
//             - Recovery from Fault (ON/OFF transitions after Fault) :
//                 * Single metal detection does NOT resume Running.
//                 * Holds Fault status (SPINDLE_RECOVERING) while spindle continuously
//                   toggles ON/OFF for >= recoveryLimitMs, then transitions to Running.
//                 * If spindle stalls (constant metal or constant no-metal) before
//                   recoveryLimitMs elapses, it remains in Fault.
//             - Dynamic Settings Sync :
//                 * Fetches triggerSeconds, recoverySeconds, and absenceDetectionEnabled
//                   from GET /api/iot/config on boot and syncs response headers on
//                   every heartbeat/event POST.
// =============================================================================

// Identity, WiFi, backend URL, pins and timing live in config.h.

// ── Runtime State ───────────────────────────────────────────────────────────
static int                    currentRawReading    = HIGH;
static uint32_t               lastStatusPrintTime  = 0;
static uint32_t               lastHeartbeatTime    = 0;
static bool                   stateEventPending    = false;
static String                 settingsVersion      = "";

static SpindleThresholdConfig thresholdCfg = {
    DEFAULT_ROTATION_STALL_MS,
    DEFAULT_TRIGGER_LIMIT_MS,
    DEFAULT_RECOVERY_LIMIT_MS,
    true,
    false
};

static portMUX_TYPE           sensorMux            = portMUX_INITIALIZER_UNLOCKED;
static SpindleIsrState        isrState             = makeSpindleIsrState(HIGH, 0, DEFAULT_ROTATION_STALL_MS);
static SpindleRuntimeState    runtimeState         = makeSpindleRuntimeState(HIGH, 0);

static uint32_t               lastLowDwellMs       = 0;
static uint32_t               lastHighGapMs        = 0;
static String                 serialCmdBuffer      = "";

static Preferences  prefs;
static String       bootId;
static uint32_t     bootCounter             = 1;
static uint64_t     hbSequence              = 0;

// ── Helpers ─────────────────────────────────────────────────────────────────
const char* getSpindleStateString(SpindleState state) {
    switch (state) {
        case SPINDLE_RUNNING:    return "RUNNING (Spindle Rotating ON/OFF)";
        case SPINDLE_IDLE:       return "IDLE (Spindle Stopped - Within Trigger Limit)";
        case SPINDLE_FAULT:      return "FAULT (Spindle Stopped - Trigger Limit Exceeded)";
        case SPINDLE_RECOVERING: return "RECOVERING (Hold FAULT - Confirming Recovery Limit)";
        default:                 return "INITIALIZING";
    }
}

const char* getPinLevelLabel(int pinValue) {
    return (pinValue == SENSOR_ACTIVE_STATE) ? "CONSTANT_METAL (LOW)" : "CONSTANT_NO_METAL (HIGH)";
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

void IRAM_ATTR onSensorPinChangeISR() {
    portENTER_CRITICAL_ISR(&sensorMux);
    uint32_t nowMs = (uint32_t)(esp_timer_get_time() / 1000ULL);
    int rawLevel = (GPIO.in >> SENSOR_INPUT_PIN) & 0x1;
    spindleIsrOnPinSample(isrState, nowMs, rawLevel, SENSOR_ACTIVE_STATE, SENSOR_DEBOUNCE_MS);
    portEXIT_CRITICAL_ISR(&sensorMux);
}

void applyDynamicThresholds(
    int32_t triggerSec,
    int32_t recoverySec,
    int8_t absenceEnabled,
    int32_t stallMs,
    const String& version,
    bool fromSerial = false) {
    bool versionChanged = (!fromSerial && version.length() > 0 && settingsVersion != version);
    bool backendVersionAdvanced = (!fromSerial && version.length() > 0 && settingsVersion.length() > 0 && settingsVersion != version);
    if (versionChanged) {
        settingsVersion = version;
    }

    uint32_t nowMs = (uint32_t)(esp_timer_get_time() / 1000ULL);
    ThresholdUpdateOutcome outcome = applyThresholdUpdateLogic(
        thresholdCfg,
        &runtimeState,
        nowMs,
        triggerSec,
        recoverySec,
        absenceEnabled,
        stallMs,
        fromSerial,
        versionChanged,
        backendVersionAdvanced,
        MIN_ROTATION_STALL_MS,
        MAX_ROTATION_STALL_MS);

    uint32_t effectiveStallMs = computeEffectiveStallMs(
        thresholdCfg.absenceDetectionEnabled,
        thresholdCfg.rotationStallMs,
        thresholdCfg.triggerLimitMs);
    portENTER_CRITICAL(&sensorMux);
    isrState.effectiveStallMs = effectiveStallMs;
    portEXIT_CRITICAL(&sensorMux);

    if (outcome.shouldPersistNvs) {
        prefs.begin("iot-node", false);
        prefs.putUInt("stall_ms", thresholdCfg.rotationStallMs);
        prefs.putUInt("trig_ms", thresholdCfg.triggerLimitMs);
        prefs.putUInt("rec_ms", thresholdCfg.recoveryLimitMs);
        prefs.putBool("abs_en", thresholdCfg.absenceDetectionEnabled);
        prefs.end();
    }

    if (outcome.anyChanged) {
        Serial.printf("  [SETTINGS SYNC] Dynamic limits updated%s -> Stall: %lu ms%s | Trigger: %lu s (%lu ms) | Recovery: %lu s (%lu ms) | Absence Detection: %s\n",
            settingsVersion.length() > 0 ? (String(" (v") + settingsVersion + ")").c_str() : "",
            (unsigned long)thresholdCfg.rotationStallMs,
            thresholdCfg.serialStallOverride ? " [SERIAL OVERRIDE]" : "",
            (unsigned long)(thresholdCfg.triggerLimitMs / 1000),
            (unsigned long)thresholdCfg.triggerLimitMs,
            (unsigned long)(thresholdCfg.recoveryLimitMs / 1000),
            (unsigned long)thresholdCfg.recoveryLimitMs,
            thresholdCfg.absenceDetectionEnabled ? "ENABLED" : "DISABLED");
    }
}

void syncThresholdsFromHttp(HTTPClient& http) {
    int32_t triggerSec = -1;
    int32_t recoverySec = -1;
    int8_t absenceEnabled = -1;
    int32_t stallMs = -1;
    String version = "";

    if (http.hasHeader("x-trigger-seconds")) {
        triggerSec = http.header("x-trigger-seconds").toInt();
    }
    if (http.hasHeader("x-recovery-seconds")) {
        recoverySec = http.header("x-recovery-seconds").toInt();
    }
    if (http.hasHeader("x-absence-enabled")) {
        String val = http.header("x-absence-enabled");
        val.toLowerCase();
        if (val == "true" || val == "1") absenceEnabled = 1;
        else if (val == "false" || val == "0") absenceEnabled = 0;
    }
    if (http.hasHeader("x-rotation-stall-ms")) {
        stallMs = http.header("x-rotation-stall-ms").toInt();
    }
    if (http.hasHeader("x-settings-version")) {
        version = http.header("x-settings-version");
    }

    if (triggerSec > 0 || recoverySec > 0 || absenceEnabled != -1 || stallMs > 0 || version.length() > 0) {
        applyDynamicThresholds(triggerSec, recoverySec, absenceEnabled, stallMs, version);
    }
}

int32_t extractJsonInt(const String& json, const char* key) {
    String pattern = String("\"") + key + "\":";
    int idx = json.indexOf(pattern);
    if (idx < 0) return -1;
    idx += pattern.length();
    while (idx < (int)json.length() && json[idx] == ' ') idx++;
    if (idx >= (int)json.length() || !isDigit(json[idx])) return -1;
    return json.substring(idx).toInt();
}

bool fetchDeviceConfigFromBackend() {
    if (WiFi.status() != WL_CONNECTED) return false;

    static const char* headerKeys[] = {
        "x-trigger-seconds",
        "x-recovery-seconds",
        "x-absence-enabled",
        "x-rotation-stall-ms",
        "x-settings-version"
    };

    HTTPClient http;
    http.setTimeout(4000);
    http.begin(String(API_BASE_URL) + "/api/iot/config");
    http.addHeader("x-device-id", DEVICE_ID);
    http.addHeader("x-device-key", DEVICE_KEY);
    http.collectHeaders(headerKeys, 5);

    int code = http.GET();
    if (code >= 200 && code < 300) {
        String resp = http.getString();
        int32_t trigSec = http.hasHeader("x-trigger-seconds")
            ? http.header("x-trigger-seconds").toInt()
            : extractJsonInt(resp, "triggerSeconds");
        int32_t recSec = http.hasHeader("x-recovery-seconds")
            ? http.header("x-recovery-seconds").toInt()
            : extractJsonInt(resp, "recoverySeconds");
        int32_t stallMs = http.hasHeader("x-rotation-stall-ms")
            ? http.header("x-rotation-stall-ms").toInt()
            : extractJsonInt(resp, "rotationStallMs");
        int8_t absEn = -1;
        if (http.hasHeader("x-absence-enabled")) {
            String val = http.header("x-absence-enabled");
            val.toLowerCase();
            if (val == "true" || val == "1") absEn = 1;
            else if (val == "false" || val == "0") absEn = 0;
        } else if (resp.indexOf("\"absenceDetectionEnabled\":true") >= 0) {
            absEn = 1;
        } else if (resp.indexOf("\"absenceDetectionEnabled\":false") >= 0) {
            absEn = 0;
        }
        String version = http.hasHeader("x-settings-version") ? http.header("x-settings-version") : "";
        applyDynamicThresholds(trigSec, recSec, absEn, stallMs, version, false);
    }
    http.end();
    return (code >= 200 && code < 300);
}

bool postToBackend(const char* path, const String& jsonBody) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("  [API] Skipped: WiFi disconnected"));
        return false;
    }

    static const char* headerKeys[] = {
        "x-trigger-seconds",
        "x-recovery-seconds",
        "x-absence-enabled",
        "x-rotation-stall-ms",
        "x-settings-version"
    };

    HTTPClient http;
    http.setTimeout(4000);
    http.begin(String(API_BASE_URL) + path);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("x-device-id", DEVICE_ID);
    http.addHeader("x-device-key", DEVICE_KEY);
    http.collectHeaders(headerKeys, 5);

    int code = http.POST(jsonBody);
    if (code >= 200 && code < 300) {
        syncThresholdsFromHttp(http);
    } else {
        String resp = http.getString();
        Serial.printf("  [API Error %d] %s\n", code, resp.c_str());
    }
    http.end();
    return (code >= 200 && code < 300);
}

bool sendSensorEvent(const char* eventType, const char* signal) {
    const SpindleState state = runtimeState.currentState;
    const char* detectionLabel = (state == SPINDLE_RUNNING)
        ? "RUNNING"
        : (state == SPINDLE_RECOVERING
            ? "RECOVERING"
            : (state == SPINDLE_FAULT ? "FAULT" : "IDLE"));
    const bool spindleRotating = (state == SPINDLE_RUNNING || state == SPINDLE_RECOVERING);
    const char* spindleCondition = spindleRotating
        ? "ROTATING_ON_OFF"
        : (runtimeState.debouncedPinState == SENSOR_ACTIVE_STATE ? "CONSTANT_METAL_DOWNTIME" : "CONSTANT_NO_METAL_DOWNTIME");
    String meta = String("{\"rssi\":") + String(WiFi.RSSI()) +
                  ",\"pinState\":\"" + (runtimeState.debouncedPinState == SENSOR_ACTIVE_STATE ? "LOW" : "HIGH") +
                  "\",\"spindleState\":\"" + spindleCondition +
                  "\",\"detectionState\":\"" + detectionLabel +
                  "\",\"totalPulses\":" + String(runtimeState.pulseCount) +
                  ",\"rotationStallMs\":" + String(thresholdCfg.rotationStallMs) +
                  ",\"lastEdgeGapMs\":" + String(runtimeState.lastEdgeGapMs) +
                  (strcmp(eventType, "recovered") == 0 ? ",\"recoveryConfirmed\":true" : "") + "}";
    String body = String("{") +
        "\"eventId\":\"" + generateUuidV4() + "\"," +
        "\"eventType\":\"" + eventType + "\"," +
        "\"signal\":\"" + signal + "\"," +
        "\"recordedAt\":\"" + isoNow() + "\"," +
        "\"metadata\":" + meta +
    "}";
    if (postToBackend("/api/iot/events", body)) {
        Serial.printf("  [EVENT SENT] %s -> %s (Detection = %s | Condition = %s)\n",
            eventType, signal, detectionLabel, spindleCondition);
        return true;
    }
    return false;
}

bool syncCurrentStateEvent() {
    if (runtimeState.currentState == SPINDLE_RUNNING) {
        if (runtimeState.wasFaulted) {
            if (sendSensorEvent("recovered", "active")) {
                runtimeState.wasFaulted = false;
                return true;
            }
            return false;
        }
        return sendSensorEvent("pulse", "active");
    }
    if (runtimeState.currentState == SPINDLE_FAULT || runtimeState.currentState == SPINDLE_RECOVERING) {
        return sendSensorEvent("fault", "fault");
    }
    return sendSensorEvent("idle", "idle");
}

void sendHeartbeat() {
    hbSequence++;
    bool activityObserved = (runtimeState.currentState == SPINDLE_RUNNING) || runtimeState.activitySinceHb;
    String body = String("{") +
        "\"heartbeatId\":\"" + generateUuidV4() + "\"," +
        "\"bootId\":\"" + bootId + "\"," +
        "\"bootCounter\":\"" + String(bootCounter) + "\"," +
        "\"sequence\":\"" + String((unsigned long)hbSequence) + "\"," +
        "\"recordedAt\":\"" + isoNow() + "\"," +
        "\"activityObserved\":" + (activityObserved ? "true" : "false") +
    "}";
    if (postToBackend("/api/iot/heartbeats", body)) {
        Serial.printf("  [HEARTBEAT] Seq: %llu | Activity: %s\n", hbSequence, activityObserved ? "YES" : "NO");
        if (runtimeState.currentState != SPINDLE_RUNNING) {
            runtimeState.activitySinceHb = false;
        }
    }
}

void updateVisualIndicator(SpindleState state) {
    digitalWrite(ONBOARD_LED_PIN, (state == SPINDLE_RUNNING) ? LED_NORMAL_STATE : LED_ALERT_STATE);
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

void pollSerialCommands() {
    while (Serial.available() > 0) {
        char ch = (char)Serial.read();
        if (ch == '\r') continue;
        if (ch != '\n') {
            if (serialCmdBuffer.length() < 64) serialCmdBuffer += ch;
            continue;
        }
        String cmd = serialCmdBuffer;
        serialCmdBuffer = "";
        cmd.trim();
        if (cmd.length() == 0) continue;

        String upper = cmd;
        upper.toUpperCase();
        if (upper == "STATUS") {
            uint32_t effectiveStallMs = computeEffectiveStallMs(
                thresholdCfg.absenceDetectionEnabled,
                thresholdCfg.rotationStallMs,
                thresholdCfg.triggerLimitMs);
            Serial.printf("  [DIAG] State: %s | Stall: %lu ms (Effective: %lu ms%s) | Trigger: %lu ms | Recovery: %lu ms | Absence: %s | LastLOW: %lu ms | LastHIGH: %lu ms | LastGap: %lu ms | MaxGap: %lu ms\n",
                getSpindleStateString(runtimeState.currentState),
                (unsigned long)thresholdCfg.rotationStallMs,
                (unsigned long)effectiveStallMs,
                thresholdCfg.serialStallOverride ? ", SERIAL OVERRIDE" : "",
                (unsigned long)thresholdCfg.triggerLimitMs,
                (unsigned long)thresholdCfg.recoveryLimitMs,
                thresholdCfg.absenceDetectionEnabled ? "ON" : "OFF",
                (unsigned long)lastLowDwellMs,
                (unsigned long)lastHighGapMs,
                (unsigned long)runtimeState.lastEdgeGapMs,
                (unsigned long)runtimeState.maxObservedEdgeGapMs);
            continue;
        }

        int32_t requestedMs = -1;
        StallCmdParseResult stallParse = parseStallCommandMs(
            cmd.c_str(),
            MIN_ROTATION_STALL_MS,
            MAX_ROTATION_STALL_MS,
            &requestedMs);
        if (stallParse == STALL_CMD_VALID) {
            applyDynamicThresholds(-1, -1, -1, requestedMs, "", true);
        } else if (stallParse == STALL_CMD_INVALID) {
            Serial.printf("  [SERIAL] Invalid stall command '%s'. Use 'STALL <ms>' within %d-%d ms.\n",
                cmd.c_str(), MIN_ROTATION_STALL_MS, MAX_ROTATION_STALL_MS);
        }
    }
}

// ── Setup ───────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    delay(200);

    pinMode(SENSOR_INPUT_PIN, INPUT_PULLUP);
    pinMode(ONBOARD_LED_PIN, OUTPUT);
    digitalWrite(ONBOARD_LED_PIN, LED_NORMAL_STATE);

    prefs.begin("iot-node", false);
    bootCounter = prefs.getUInt("boot_cnt", 0) + 1;
    prefs.putUInt("boot_cnt", bootCounter);
    uint32_t storedStallMs = prefs.getUInt("stall_ms", DEFAULT_ROTATION_STALL_MS);
    uint32_t storedTriggerMs = prefs.getUInt("trig_ms", DEFAULT_TRIGGER_LIMIT_MS);
    uint32_t storedRecoveryMs = prefs.getUInt("rec_ms", DEFAULT_RECOVERY_LIMIT_MS);
    bool storedAbsenceEnabled = prefs.getBool("abs_en", true);
    prefs.end();

    thresholdCfg = sanitizeBootThresholds(
        storedStallMs,
        storedTriggerMs,
        storedRecoveryMs,
        storedAbsenceEnabled,
        MIN_ROTATION_STALL_MS,
        MAX_ROTATION_STALL_MS,
        DEFAULT_ROTATION_STALL_MS,
        DEFAULT_TRIGGER_LIMIT_MS,
        DEFAULT_RECOVERY_LIMIT_MS);
    bootId = generateUuidV4();

    Serial.println();
    Serial.println(F("=============================================================="));
    Serial.printf( "  %s — %s [%s]\n", NODE_ID, NODE_ROLE_NAME, DEVICE_ID);
    Serial.println(F("=============================================================="));
    Serial.printf( "  Backend URL    : %s\n", API_BASE_URL);
    Serial.printf( "  Sensor Pin     : GPIO %d (INPUT_PULLUP via PC817)\n", SENSOR_INPUT_PIN);
    Serial.printf( "  LED Pin        : GPIO %d (Visual Indicator)\n", ONBOARD_LED_PIN);
    Serial.printf( "  Rotation Stall : %lu ms [Dynamic]\n", (unsigned long)thresholdCfg.rotationStallMs);
    Serial.printf( "  Trigger Limit  : %lu ms (%lu s) [Dynamic]\n", (unsigned long)thresholdCfg.triggerLimitMs, (unsigned long)(thresholdCfg.triggerLimitMs / 1000));
    Serial.printf( "  Recovery Limit : %lu ms (%lu s) [Dynamic]\n", (unsigned long)thresholdCfg.recoveryLimitMs, (unsigned long)(thresholdCfg.recoveryLimitMs / 1000));
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

    fetchDeviceConfigFromBackend();

    uint32_t initialEffectiveStallMs = computeEffectiveStallMs(
        thresholdCfg.absenceDetectionEnabled,
        thresholdCfg.rotationStallMs,
        thresholdCfg.triggerLimitMs);
    uint32_t now = 0;
    int initialReading = HIGH;
    portENTER_CRITICAL(&sensorMux);
    now = (uint32_t)(esp_timer_get_time() / 1000ULL);
    initialReading = (GPIO.in >> SENSOR_INPUT_PIN) & 0x1;
    isrState = makeSpindleIsrState(initialReading, now, initialEffectiveStallMs);
    portEXIT_CRITICAL(&sensorMux);

    currentRawReading = initialReading;
    runtimeState = makeSpindleRuntimeState(initialReading, now);
    lastHeartbeatTime = now;
    lastStatusPrintTime = now;

    attachInterrupt(digitalPinToInterrupt(SENSOR_INPUT_PIN), onSensorPinChangeISR, CHANGE);

    updateVisualIndicator(runtimeState.currentState);
    sendHeartbeat();
    stateEventPending = !syncCurrentStateEvent();
}

// ── Main Loop ───────────────────────────────────────────────────────────────
void loop() {
    reconnectWifiIfNeeded();
    pollSerialCommands();

    uint32_t effectiveStallMs = computeEffectiveStallMs(
        thresholdCfg.absenceDetectionEnabled,
        thresholdCfg.rotationStallMs,
        thresholdCfg.triggerLimitMs);
    uint32_t now = 0;
    int rawReading = HIGH;
    SpindleEdgeBatch batch;

    // Sample timestamp and GPIO level atomically inside sensorMux to prevent ISR race underflow.
    portENTER_CRITICAL(&sensorMux);
    now = (uint32_t)(esp_timer_get_time() / 1000ULL);
    rawReading = (GPIO.in >> SENSOR_INPUT_PIN) & 0x1;
    isrState.effectiveStallMs = effectiveStallMs;
    batch = spindleConsumeCycleLocked(isrState, now, rawReading, SENSOR_ACTIVE_STATE, SENSOR_DEBOUNCE_MS);
    portEXIT_CRITICAL(&sensorMux);

    currentRawReading = rawReading;
    lastLowDwellMs = batch.lastLowDwellMs;
    lastHighGapMs = batch.lastHighGapMs;

    SpindleState stateBeforeStep = runtimeState.currentState;
    SpindleStepResult stepRes = spindleStep(runtimeState, batch, now, thresholdCfg);

    if (runtimeState.currentState != stateBeforeStep) {
        updateVisualIndicator(runtimeState.currentState);
        if (stepRes.stepEvent == SPINDLE_EVENT_ENTERED_RECOVERING) {
            Serial.printf("[%7lu ms] Spindle ON/OFF rotation detected during FAULT -> Holding FAULT for %lu s recovery limit (%s)\n",
                (unsigned long)now,
                (unsigned long)(thresholdCfg.recoveryLimitMs / 1000),
                getSpindleStateString(runtimeState.currentState));
        } else if (stepRes.stepEvent == SPINDLE_EVENT_RECOVERY_ABORTED_STALL) {
            Serial.printf("[%7lu ms] Spindle stopped on %s before recovery limit (%lu ms < %lu ms) -> Remaining in %s\n",
                (unsigned long)now,
                getPinLevelLabel(runtimeState.debouncedPinState),
                (unsigned long)stepRes.heldRecoveryMs,
                (unsigned long)thresholdCfg.recoveryLimitMs,
                getSpindleStateString(runtimeState.currentState));
        } else if (stepRes.stepEvent == SPINDLE_EVENT_STOPPED_TO_IDLE) {
            Serial.printf("[%7lu ms] Spindle stopped on %s -> State change: %s\n",
                (unsigned long)now,
                getPinLevelLabel(runtimeState.debouncedPinState),
                getSpindleStateString(runtimeState.currentState));
        } else if (stepRes.stepEvent == SPINDLE_EVENT_RECOVERY_MET) {
            Serial.printf("[%7lu ms] RECOVERY LIMIT MET (%lu ms) -> %s\n",
                (unsigned long)now,
                (unsigned long)thresholdCfg.recoveryLimitMs,
                getSpindleStateString(runtimeState.currentState));
        } else if (stepRes.stepEvent == SPINDLE_EVENT_TRIGGER_EXCEEDED) {
            Serial.printf("[%7lu ms] TRIGGER LIMIT EXCEEDED (%lu ms >= %lu ms on %s) -> %s\n",
                (unsigned long)now,
                (unsigned long)(now - runtimeState.absenceStartTime),
                (unsigned long)thresholdCfg.triggerLimitMs,
                getPinLevelLabel(runtimeState.debouncedPinState),
                getSpindleStateString(runtimeState.currentState));
        } else {
            Serial.printf("[%7lu ms] State change: %s\n",
                (unsigned long)now,
                getSpindleStateString(runtimeState.currentState));
        }
    }

    if (stepRes.syncAction == SPINDLE_SYNC_STATE) {
        stateEventPending = !syncCurrentStateEvent();
    }

    // Emit periodic active pulses while spindle is continuously rotating ON/OFF so the
    // backend watchdog never drops Sensor 3 into Idle (grace) while Running.
    if (runtimeState.currentState == SPINDLE_RUNNING) {
        if (!stateEventPending && elapsedAtLeast(now, runtimeState.lastPulseSentTime, MIN_PULSE_INTERVAL_MS)) {
            runtimeState.lastPulseSentTime = now;
            runtimeState.lastEventRetryTime = now;
            stateEventPending = !syncCurrentStateEvent();
        }
    }

    // ── Retry Failed State Event POST ───────────────────────────────────────
    if (stateEventPending && elapsedAtLeast(now, runtimeState.lastEventRetryTime, EVENT_RETRY_INTERVAL_MS)) {
        runtimeState.lastEventRetryTime = now;
        stateEventPending = !syncCurrentStateEvent();
    }

    // ── Background Heartbeat ────────────────────────────────────────────────
    if (elapsedAtLeast(now, lastHeartbeatTime, HEARTBEAT_INTERVAL_MS)) {
        lastHeartbeatTime = now;
        if (stateEventPending) {
            runtimeState.lastEventRetryTime = now;
            stateEventPending = !syncCurrentStateEvent();
        }
        sendHeartbeat();
    }

    // ── Periodic Serial Status Log ──────────────────────────────────────────
    if (elapsedAtLeast(now, lastStatusPrintTime, STATUS_PRINT_INTERVAL_MS)) {
        lastStatusPrintTime = now;
        if (runtimeState.currentState == SPINDLE_RUNNING) {
            Serial.printf("  [STATUS] Detection = RUNNING (Rotating ON/OFF) | Active for %lu s | Pulses: %lu\n",
                (unsigned long)((now - runtimeState.stateStartTime) / 1000),
                (unsigned long)runtimeState.pulseCount);
        } else if (runtimeState.currentState == SPINDLE_IDLE) {
            Serial.printf("  [STATUS] Detection = IDLE (%s) | Stopped %lu s / %lu s trigger limit\n",
                getPinLevelLabel(runtimeState.debouncedPinState),
                (unsigned long)((now - runtimeState.absenceStartTime) / 1000),
                (unsigned long)(thresholdCfg.triggerLimitMs / 1000));
        } else if (runtimeState.currentState == SPINDLE_FAULT) {
            Serial.printf("  [STATUS] Detection = FAULT (%s) | Stopped %lu s (Trigger limit %lu s exceeded)\n",
                getPinLevelLabel(runtimeState.debouncedPinState),
                (unsigned long)((now - runtimeState.absenceStartTime) / 1000),
                (unsigned long)(thresholdCfg.triggerLimitMs / 1000));
        } else if (runtimeState.currentState == SPINDLE_RECOVERING) {
            Serial.printf("  [STATUS] Detection = RECOVERING (Hold FAULT) | Rotating ON/OFF %lu s / %lu s recovery limit\n",
                (unsigned long)((now - runtimeState.recoveryStartTime) / 1000),
                (unsigned long)(thresholdCfg.recoveryLimitMs / 1000));
        }
    }

    delay(1);
}
