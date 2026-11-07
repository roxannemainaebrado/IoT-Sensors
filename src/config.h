#pragma once
#include <Arduino.h>

// ==============================================================================
// 1. NODE IDENTITY & PURPOSE
// ==============================================================================
#define NODE_ID                     "S-03"
#define NODE_ROLE_NAME              "Machine Main Spindle Monitor"
#define FIRMWARE_VERSION            "1.1.0"
#define DEVICE_ID                   "esp32-m01-s03"
#define DEVICE_KEY                  "php-s-03-019b66f1698c64e3dfadb5ea3847932a087e"

// ==============================================================================
// 2. NETWORK & BACKEND CONFIGURATION
// ==============================================================================
#define WIFI_SSID                   "Realme 11"
#define WIFI_PASS                   "12345678900987654321"
#define API_BASE_URL                "http://10.72.136.68:3000"

// ==============================================================================
// 3. HARDWARE GPIO PIN CONFIGURATION (ESP32 DevKit V1 38-Pin)
// ==============================================================================
#define SENSOR_INPUT_PIN            4      // GPIO 4 (Pin D4)
#define ONBOARD_LED_PIN             2      // GPIO 2 (Visual indicator)

#define SENSOR_ACTIVE_STATE         LOW    // LOW  = Metal detected (Active / Running)
#define SENSOR_IDLE_STATE           HIGH   // HIGH = No metal detected (Absent)

#define LED_ALERT_STATE             HIGH   // Turn LED ON when alert triggered
#define LED_NORMAL_STATE            LOW    // Turn LED OFF when metal present

// ==============================================================================
// 4. SIGNAL FILTERING & TIMING THRESHOLDS
// ==============================================================================
#define SENSOR_DEBOUNCE_MS          50      // Stable duration (ms) to confirm state change
#define DEFAULT_ROTATION_STALL_MS   3000    // Fallback max time (ms) without ON/OFF edges before spindle is considered stopped (synced from backend)
#define MIN_ROTATION_STALL_MS       500     // Minimum configurable rotation stall window (ms)
#define MAX_ROTATION_STALL_MS       30000   // Maximum configurable rotation stall window (ms)
#define ROTATION_TIMEOUT_MS         DEFAULT_ROTATION_STALL_MS // Legacy alias for helper headers
#define MIN_PULSE_INTERVAL_MS       1500    // Active pulse interval (ms) while spindle is rotating (Running)
#define DEFAULT_TRIGGER_LIMIT_MS    60000   // Fallback downtime trigger limit (ms) without ON/OFF before Fault (synced from backend)
#define DEFAULT_RECOVERY_LIMIT_MS   60000   // Fallback recovery limit (ms) of continuous ON/OFF before Running (synced from backend)
#define TRIGGER_LIMIT_MS            DEFAULT_TRIGGER_LIMIT_MS
#define RECOVERY_LIMIT_MS           DEFAULT_RECOVERY_LIMIT_MS
#define INACTIVITY_TIMEOUT_MS       DEFAULT_TRIGGER_LIMIT_MS
#define EVENT_RETRY_INTERVAL_MS     5000    // Retry interval (ms) when an event POST fails
#define WIFI_RECONNECT_INTERVAL_MS  5000    // Minimum gap (ms) between WiFi reconnect attempts
#define HEARTBEAT_INTERVAL_MS       10000   // Periodic heartbeat interval (ms)
#define STATUS_PRINT_INTERVAL_MS    3000    // Periodic Serial status log interval (ms)
#define SERIAL_BAUD_RATE            115200  // Serial Monitor baud rate

// Legacy module compatibility flags
#define IS_DOWNTIME_MASTER          true
#define IS_PRODUCTION_COUNTER       false
#define ENABLE_VERBOSE_LOG          true
