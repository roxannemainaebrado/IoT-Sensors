#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <time.h>
#include "config.h"

// =============================================================================
// ESP32 SENSOR 4 — OUTSIDE FILLER WIRE AVAILABILITY & REFILL MONITOR
// =============================================================================
// Node ID   : S-04
// Role      : Outside Filler Wire Monitor
// Logic     : - Wire detected (LOW) : Running -> emits 'pulse' / 'recovered' (active), LED OFF
//             - Wire absent  (HIGH) :
//                 * Within dynamic trigger limit (< triggerLimitMs) : Idle  -> emits 'idle' (idle), LED ON
//                 * Past dynamic trigger limit   (>= triggerLimitMs): Fault -> emits 'fault' (fault), LED ON
//             - Recovery from Fault (LOW after Fault) :
//                 * Holds Fault status (WIRE_RECOVERING) until metal is continuously
//                   detected for >= recoveryLimitMs, then transitions to Running.
//             - Dynamic Settings Sync :
//                 * Fetches triggerSeconds, recoverySeconds, and absenceDetectionEnabled
//                   from GET /api/iot/config on boot and syncs response headers on
//                   every heartbeat/event POST.
// =============================================================================

// Identity, WiFi, backend URL, pins and timing live in config.h.

// ── Runtime State ───────────────────────────────────────────────────────────
enum WireState {
    WIRE_STATE_UNKNOWN = -1,
    WIRE_PRESENT    = 0,      // Metal wire detected at sensing station (Running / Active)
    WIRE_IDLE       = 1,      // Metal wire absent within trigger limit (Idle / Grace)
    WIRE_FAULT      = 2,      // Metal wire absent past trigger limit (Fault)
    WIRE_RECOVERING = 3       // Metal wire detected after Fault; holding Fault until recoveryLimitMs elapses
};

static int       currentRawReading       = HIGH;
static int       lastRawReading          = HIGH;
static uint32_t  lastDebounceTime        = 0;

static int       debouncedPinState       = HIGH;
static WireState currentWireState        = WIRE_STATE_UNKNOWN;
static WireState previousWireState       = WIRE_STATE_UNKNOWN;
static uint32_t  stateStartTime          = 0;
static uint32_t  absenceStartTime        = 0;
static uint32_t  recoveryStartTime       = 0;
static uint32_t  lastPulseSentTime       = 0;
static uint32_t  lastEventRetryTime      = 0;
static uint32_t  lastStatusPrintTime     = 0;

static uint32_t  lastHeartbeatTime       = 0;
static uint32_t  stateChangeCount        = 0;
static bool      activitySinceHb         = false;
static bool      stateEventPending       = false;
static bool      wasFaulted              = false;

// Dynamic operational thresholds synced from Backend machine_operational_settings
static uint32_t  triggerLimitMs          = DEFAULT_TRIGGER_LIMIT_MS;
static uint32_t  recoveryLimitMs         = DEFAULT_RECOVERY_LIMIT_MS;
static bool      absenceDetectionEnabled = true;
static String    settingsVersion         = "";

static Preferences prefs;
static String      bootId;
static uint32_t    bootCounter           = 1;
static uint64_t    hbSequence            = 0;

// ── Helpers ─────────────────────────────────────────────────────────────────
const char* getWireStateString(WireState state) {
    switch (state) {
        case WIRE_PRESENT:    return "RUNNING (Wire Present / Active)";
        case WIRE_IDLE:       return "IDLE (No Wire - Within Trigger Limit)";
        case WIRE_FAULT:      return "FAULT (No Wire - Trigger Limit Exceeded)";
        case WIRE_RECOVERING: return "RECOVERING (Hold FAULT - Confirming Recovery Limit)";
        default:              return "INITIALIZING";
    }
}

WireState pinReadingToWireState(int pinValue) {
    return (pinValue == SENSOR_ACTIVE_STATE) ? WIRE_PRESENT : WIRE_IDLE;
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

void applyDynamicThresholds(int32_t triggerSec, int32_t recoverySec, int8_t absenceEnabled, const String& version) {
    bool thresholdsChanged = false;
    bool versionChanged = false;

    if (absenceEnabled == 0 || absenceEnabled == 1) {
        bool enabledBool = (absenceEnabled == 1);
        if (absenceDetectionEnabled != enabledBool) {
            bool enablingNow = (!absenceDetectionEnabled && enabledBool);
            absenceDetectionEnabled = enabledBool;
            thresholdsChanged = true;
            if (enablingNow && currentWireState == WIRE_IDLE) {
                absenceStartTime = millis();
            }
        }
    }

    if (triggerSec >= 1 && triggerSec <= 3600) {
        uint32_t newTriggerMs = (uint32_t)triggerSec * 1000UL;
        if (triggerLimitMs != newTriggerMs) {
            triggerLimitMs = newTriggerMs;
            thresholdsChanged = true;
        }
    }

    if (recoverySec >= 1 && recoverySec <= 300) {
        uint32_t newRecoveryMs = (uint32_t)recoverySec * 1000UL;
        if (recoveryLimitMs != newRecoveryMs) {
            recoveryLimitMs = newRecoveryMs;
            thresholdsChanged = true;
        }
    }

    if (version.length() > 0 && settingsVersion != version) {
        settingsVersion = version;
        versionChanged = true;
    }

    if (thresholdsChanged) {
        prefs.begin("iot-node", false);
        prefs.putUInt("trig_ms", triggerLimitMs);
        prefs.putUInt("rec_ms", recoveryLimitMs);
        prefs.putBool("abs_en", absenceDetectionEnabled);
        prefs.end();
    }

    if (thresholdsChanged || versionChanged) {
        Serial.printf("  [SETTINGS SYNC] Dynamic limits updated%s -> Trigger: %lu s (%lu ms) | Recovery: %lu s (%lu ms) | Absence Detection: %s\n",
            settingsVersion.length() > 0 ? (String(" (v") + settingsVersion + ")").c_str() : "",
            (unsigned long)(triggerLimitMs / 1000),
            (unsigned long)triggerLimitMs,
            (unsigned long)(recoveryLimitMs / 1000),
            (unsigned long)recoveryLimitMs,
            absenceDetectionEnabled ? "ENABLED" : "DISABLED");
    }
}

void syncThresholdsFromHttp(HTTPClient& http) {
    int32_t triggerSec = -1;
    int32_t recoverySec = -1;
    int8_t absenceEnabled = -1;
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
    if (http.hasHeader("x-settings-version")) {
        version = http.header("x-settings-version");
    }

    if (triggerSec > 0 || recoverySec > 0 || absenceEnabled != -1 || version.length() > 0) {
        applyDynamicThresholds(triggerSec, recoverySec, absenceEnabled, version);
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
        "x-settings-version"
    };

    HTTPClient http;
    http.setTimeout(4000);
    http.begin(String(API_BASE_URL) + "/api/iot/config");
    http.addHeader("x-device-id", DEVICE_ID);
    http.addHeader("x-device-key", DEVICE_KEY);
    http.collectHeaders(headerKeys, 4);

    int code = http.GET();
    if (code >= 200 && code < 300) {
        String resp = http.getString();
        int32_t trigSec = http.hasHeader("x-trigger-seconds")
            ? http.header("x-trigger-seconds").toInt()
            : extractJsonInt(resp, "triggerSeconds");
        int32_t recSec = http.hasHeader("x-recovery-seconds")
            ? http.header("x-recovery-seconds").toInt()
            : extractJsonInt(resp, "recoverySeconds");
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
        applyDynamicThresholds(trigSec, recSec, absEn, version);
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
        "x-settings-version"
    };

    HTTPClient http;
    http.setTimeout(4000);
    http.begin(String(API_BASE_URL) + path);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("x-device-id", DEVICE_ID);
    http.addHeader("x-device-key", DEVICE_KEY);
    http.collectHeaders(headerKeys, 4);

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
    const char* detectionLabel = (currentWireState == WIRE_PRESENT)
        ? "RUNNING"
        : (currentWireState == WIRE_RECOVERING
            ? "RECOVERING"
            : (currentWireState == WIRE_FAULT ? "FAULT" : "IDLE"));
    const bool wirePresentOnPin = (currentWireState == WIRE_PRESENT || currentWireState == WIRE_RECOVERING);
    String meta = String("{\"rssi\":") + String(WiFi.RSSI()) +
                  ",\"wireState\":\"" + (wirePresentOnPin ? "PRESENT" : "ABSENT") +
                  "\",\"detectionState\":\"" + detectionLabel + "\"" +
                  (strcmp(eventType, "recovered") == 0 ? ",\"recoveryConfirmed\":true" : "") + "}";
    String body = String("{") +
        "\"eventId\":\"" + generateUuidV4() + "\"," +
        "\"eventType\":\"" + eventType + "\"," +
        "\"signal\":\"" + signal + "\"," +
        "\"recordedAt\":\"" + isoNow() + "\"," +
        "\"metadata\":" + meta +
    "}";
    if (postToBackend("/api/iot/events", body)) {
        Serial.printf("  [EVENT SENT] %s -> %s (Detection = %s)\n", eventType, signal, detectionLabel);
        return true;
    }
    return false;
}

bool syncCurrentStateEvent() {
    if (currentWireState == WIRE_PRESENT) {
        if (wasFaulted) {
            if (sendSensorEvent("recovered", "active")) {
                wasFaulted = false;
                return true;
            }
            return false;
        }
        return sendSensorEvent("pulse", "active");
    }
    if (currentWireState == WIRE_FAULT || currentWireState == WIRE_RECOVERING) {
        return sendSensorEvent("fault", "fault");
    }
    return sendSensorEvent("idle", "idle");
}

void sendHeartbeat() {
    hbSequence++;
    bool activityObserved = (currentWireState == WIRE_PRESENT) || activitySinceHb;
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
        if (currentWireState != WIRE_PRESENT) {
            activitySinceHb = false;
        }
    }
}

void updateVisualIndicator(WireState state) {
    digitalWrite(ONBOARD_LED_PIN, (state == WIRE_PRESENT) ? LED_NORMAL_STATE : LED_ALERT_STATE);
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
    digitalWrite(ONBOARD_LED_PIN, LED_NORMAL_STATE);

    prefs.begin("iot-node", false);
    bootCounter = prefs.getUInt("boot_cnt", 0) + 1;
    prefs.putUInt("boot_cnt", bootCounter);
    uint32_t storedTriggerMs = prefs.getUInt("trig_ms", DEFAULT_TRIGGER_LIMIT_MS);
    uint32_t storedRecoveryMs = prefs.getUInt("rec_ms", DEFAULT_RECOVERY_LIMIT_MS);
    absenceDetectionEnabled = prefs.getBool("abs_en", true);
    prefs.end();
    triggerLimitMs = (storedTriggerMs >= 1000UL && storedTriggerMs <= 3600000UL) ? storedTriggerMs : DEFAULT_TRIGGER_LIMIT_MS;
    recoveryLimitMs = (storedRecoveryMs >= 1000UL && storedRecoveryMs <= 300000UL) ? storedRecoveryMs : DEFAULT_RECOVERY_LIMIT_MS;
    bootId = generateUuidV4();

    Serial.println();
    Serial.println(F("=============================================================="));
    Serial.printf( "  %s — %s [%s]\n", NODE_ID, NODE_ROLE_NAME, DEVICE_ID);
    Serial.println(F("=============================================================="));
    Serial.printf( "  Backend URL    : %s\n", API_BASE_URL);
    Serial.printf( "  Sensor Pin     : GPIO %d (INPUT_PULLUP via PC817)\n", SENSOR_INPUT_PIN);
    Serial.printf( "  LED Pin        : GPIO %d (Visual Indicator)\n", ONBOARD_LED_PIN);
    Serial.printf( "  Trigger Limit  : %lu ms (%lu s) [Dynamic]\n", (unsigned long)triggerLimitMs, (unsigned long)(triggerLimitMs / 1000));
    Serial.printf( "  Recovery Limit : %lu ms (%lu s) [Dynamic]\n", (unsigned long)recoveryLimitMs, (unsigned long)(recoveryLimitMs / 1000));
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

    int initialReading = digitalRead(SENSOR_INPUT_PIN);
    currentRawReading  = initialReading;
    lastRawReading     = initialReading;
    debouncedPinState  = initialReading;

    uint32_t now        = millis();
    currentWireState    = pinReadingToWireState(debouncedPinState);
    stateStartTime      = now;
    absenceStartTime    = now;
    lastHeartbeatTime   = now;
    lastPulseSentTime   = now;
    lastEventRetryTime  = now;
    lastStatusPrintTime = now;

    activitySinceHb     = (currentWireState == WIRE_PRESENT);

    updateVisualIndicator(currentWireState);
    sendHeartbeat();
    stateEventPending   = !syncCurrentStateEvent();
}

// ── Main Loop ───────────────────────────────────────────────────────────────
void loop() {
    reconnectWifiIfNeeded();

    uint32_t now = millis();

    if (!absenceDetectionEnabled) {
        if (currentWireState == WIRE_RECOVERING) {
            previousWireState  = currentWireState;
            currentWireState   = WIRE_PRESENT;
            recoveryStartTime  = 0;
            stateStartTime     = now;
            lastPulseSentTime  = now;
            lastEventRetryTime = now;
            stateChangeCount++;
            activitySinceHb    = true;

            updateVisualIndicator(currentWireState);
            Serial.printf("[%7lu ms] Absence detection disabled during RECOVERING -> %s\n",
                (unsigned long)now, getWireStateString(currentWireState));
            stateEventPending  = !syncCurrentStateEvent();
        } else if (currentWireState == WIRE_FAULT) {
            previousWireState  = currentWireState;
            currentWireState   = WIRE_IDLE;
            stateStartTime     = now;
            lastEventRetryTime = now;
            stateChangeCount++;
            activitySinceHb    = false;

            updateVisualIndicator(currentWireState);
            Serial.printf("[%7lu ms] Absence detection disabled during FAULT -> %s\n",
                (unsigned long)now, getWireStateString(currentWireState));
            stateEventPending  = !syncCurrentStateEvent();
        }
    }

    int rawReading = digitalRead(SENSOR_INPUT_PIN);

    if (rawReading != lastRawReading) {
        lastDebounceTime = now;
        lastRawReading   = rawReading;
    }

    if ((now - lastDebounceTime) >= SENSOR_DEBOUNCE_MS) {
        if (rawReading != debouncedPinState) {
            debouncedPinState = rawReading;

            if (debouncedPinState == SENSOR_ACTIVE_STATE) {
                if (absenceDetectionEnabled && (currentWireState == WIRE_FAULT || wasFaulted)) {
                    // Metal wire detected while in FAULT -> Hold FAULT status until
                    // wire stays continuously present for recoveryLimitMs.
                    previousWireState  = currentWireState;
                    currentWireState   = WIRE_RECOVERING;
                    stateStartTime     = now;
                    recoveryStartTime  = now;
                    stateChangeCount++;
                    activitySinceHb    = false;

                    updateVisualIndicator(currentWireState);
                    Serial.printf("[%7lu ms] Metal wire detected during FAULT -> Holding FAULT for %lu s recovery limit (%s)\n",
                        (unsigned long)now,
                        (unsigned long)(recoveryLimitMs / 1000),
                        getWireStateString(currentWireState));
                } else if (currentWireState != WIRE_PRESENT) {
                    // Returning from IDLE (within trigger limit) -> RUNNING
                    previousWireState  = currentWireState;
                    currentWireState   = WIRE_PRESENT;
                    stateStartTime     = now;
                    lastPulseSentTime  = now;
                    lastEventRetryTime = now;
                    stateChangeCount++;
                    activitySinceHb    = true;

                    updateVisualIndicator(currentWireState);
                    Serial.printf("[%7lu ms] State change: %s\n", (unsigned long)now, getWireStateString(currentWireState));
                    stateEventPending  = !syncCurrentStateEvent();
                }
            } else {
                if (currentWireState == WIRE_RECOVERING) {
                    // Metal wire lost before recovery limit completed -> stay in FAULT
                    uint32_t heldMs    = now - recoveryStartTime;
                    previousWireState  = currentWireState;
                    currentWireState   = WIRE_FAULT;
                    stateStartTime     = now;
                    recoveryStartTime  = 0;
                    stateChangeCount++;
                    activitySinceHb    = false;

                    updateVisualIndicator(currentWireState);
                    Serial.printf("[%7lu ms] Metal wire lost before recovery limit (%lu ms < %lu ms) -> Remaining in %s\n",
                        (unsigned long)now,
                        (unsigned long)heldMs,
                        (unsigned long)recoveryLimitMs,
                        getWireStateString(currentWireState));
                } else if (currentWireState == WIRE_PRESENT || currentWireState == WIRE_STATE_UNKNOWN) {
                    // Metal wire absent -> starts in IDLE (within trigger limit)
                    previousWireState  = currentWireState;
                    currentWireState   = WIRE_IDLE;
                    stateStartTime     = now;
                    absenceStartTime   = now;
                    lastEventRetryTime = now;
                    stateChangeCount++;
                    activitySinceHb    = false;

                    updateVisualIndicator(currentWireState);
                    Serial.printf("[%7lu ms] State change: %s\n", (unsigned long)now, getWireStateString(currentWireState));
                    stateEventPending  = !syncCurrentStateEvent();
                }
            }
        }
    }

    // ── Active Presence Maintenance (Detection = RUNNING) ───────────────────
    // Emit periodic active pulses while wire is continuously present so the
    // backend watchdog never drops Sensor 4 into Idle (grace) while Running.
    if (currentWireState == WIRE_PRESENT) {
        activitySinceHb = true;
        if (!stateEventPending && (now - lastPulseSentTime >= MIN_PULSE_INTERVAL_MS)) {
            lastPulseSentTime  = now;
            lastEventRetryTime = now;
            stateEventPending  = !syncCurrentStateEvent();
        }
    }

    // ── Recovery Limit Confirmation (RECOVERING -> RUNNING past limit) ──────
    // Holds FAULT status until metal wire has been continuously detected for the
    // full dynamic recoveryLimitMs, ensuring recovery is certain before Running.
    if (currentWireState == WIRE_RECOVERING && (now - recoveryStartTime >= recoveryLimitMs)) {
        previousWireState  = currentWireState;
        currentWireState   = WIRE_PRESENT;
        stateStartTime     = now;
        lastPulseSentTime  = now;
        lastEventRetryTime = now;
        stateChangeCount++;
        activitySinceHb    = true;

        updateVisualIndicator(currentWireState);
        Serial.printf("[%7lu ms] RECOVERY LIMIT MET (%lu ms >= %lu ms) -> %s\n",
            (unsigned long)now,
            (unsigned long)(now - recoveryStartTime),
            (unsigned long)recoveryLimitMs,
            getWireStateString(currentWireState));
        stateEventPending  = !syncCurrentStateEvent();
    }

    // ── Trigger Limit Escalation (IDLE -> FAULT past trigger limit) ─────────
    if (absenceDetectionEnabled && currentWireState == WIRE_IDLE && (now - absenceStartTime >= triggerLimitMs)) {
        previousWireState  = currentWireState;
        currentWireState   = WIRE_FAULT;
        stateStartTime     = now;
        lastEventRetryTime = now;
        wasFaulted         = true;
        activitySinceHb    = false;
        stateChangeCount++;

        updateVisualIndicator(currentWireState);
        Serial.printf("[%7lu ms] TRIGGER LIMIT EXCEEDED (%lu ms >= %lu ms) -> %s\n",
            (unsigned long)now,
            (unsigned long)(now - absenceStartTime),
            (unsigned long)triggerLimitMs,
            getWireStateString(currentWireState));
        stateEventPending  = !syncCurrentStateEvent();
    }

    // ── Retry Failed State Event POST ───────────────────────────────────────
    if (stateEventPending && (now - lastEventRetryTime >= EVENT_RETRY_INTERVAL_MS)) {
        lastEventRetryTime = now;
        stateEventPending  = !syncCurrentStateEvent();
    }

    // ── Background Heartbeat ────────────────────────────────────────────────
    if (now - lastHeartbeatTime >= HEARTBEAT_INTERVAL_MS) {
        lastHeartbeatTime = now;
        if (stateEventPending) {
            lastEventRetryTime = now;
            stateEventPending  = !syncCurrentStateEvent();
        }
        sendHeartbeat();
    }

    // ── Periodic Serial Status Log ──────────────────────────────────────────
    if (now - lastStatusPrintTime >= STATUS_PRINT_INTERVAL_MS) {
        lastStatusPrintTime = now;
        if (currentWireState == WIRE_PRESENT) {
            Serial.printf("  [STATUS] Detection = RUNNING | Active for %lu s\n",
                (unsigned long)((now - stateStartTime) / 1000));
        } else if (currentWireState == WIRE_IDLE) {
            Serial.printf("  [STATUS] Detection = IDLE | Absent %lu s / %lu s trigger limit\n",
                (unsigned long)((now - absenceStartTime) / 1000),
                (unsigned long)(triggerLimitMs / 1000));
        } else if (currentWireState == WIRE_FAULT) {
            Serial.printf("  [STATUS] Detection = FAULT | Absent %lu s (Trigger limit %lu s exceeded)\n",
                (unsigned long)((now - absenceStartTime) / 1000),
                (unsigned long)(triggerLimitMs / 1000));
        } else if (currentWireState == WIRE_RECOVERING) {
            Serial.printf("  [STATUS] Detection = RECOVERING (Hold FAULT) | Wire present %lu s / %lu s recovery limit\n",
                (unsigned long)((now - recoveryStartTime) / 1000),
                (unsigned long)(recoveryLimitMs / 1000));
        }
    }

    delay(1);
}
