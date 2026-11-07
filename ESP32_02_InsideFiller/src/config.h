#pragma once
#include <Arduino.h>

// ==============================================================================
// 1. NODE IDENTITY & PURPOSE
// ==============================================================================
#define NODE_ID                     "S-02"
#define NODE_ROLE_NAME              "Inside Filler Wire Monitor"
#define FIRMWARE_VERSION            "1.1.0"
#define DEVICE_ID                   "esp32-m01-s02"
#define DEVICE_KEY                  "php-s-02-0d837f0474edf453f820e52290abde3cbb95"
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

#define SENSOR_ACTIVE_STATE         LOW    // LOW  = Metal detected (Wire present)
#define SENSOR_IDLE_STATE           HIGH   // HIGH = No metal detected (Wire absent)

#define LED_ALERT_STATE             HIGH   // Turn LED ON when alert triggered
#define LED_NORMAL_STATE            LOW    // Turn LED OFF when material present

// ==============================================================================
// 4. SIGNAL FILTERING & TIMING THRESHOLDS
// ==============================================================================
#define SENSOR_DEBOUNCE_MS          50      // Stable duration (ms) to confirm state change
#define MIN_PULSE_INTERVAL_MS       1500    // Active pulse interval (ms) while wire present (Running)
#define DEFAULT_TRIGGER_LIMIT_MS    300000  // Fallback trigger limit (ms) before backend settings sync
#define DEFAULT_RECOVERY_LIMIT_MS   60000   // Fallback recovery limit (ms) before backend settings sync
#define TRIGGER_LIMIT_MS            DEFAULT_TRIGGER_LIMIT_MS
#define RECOVERY_LIMIT_MS           DEFAULT_RECOVERY_LIMIT_MS
#define EVENT_RETRY_INTERVAL_MS     5000    // Retry interval (ms) when an event POST fails
#define WIFI_RECONNECT_INTERVAL_MS  5000    // Minimum gap (ms) between WiFi reconnect attempts
#define HEARTBEAT_INTERVAL_MS       10000   // Periodic heartbeat interval (ms)
#define STATUS_PRINT_INTERVAL_MS    3000    // Periodic Serial status log interval (ms)
#define SERIAL_BAUD_RATE            115200  // Serial Monitor baud rate
