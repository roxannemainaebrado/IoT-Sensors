#pragma once
#include <Arduino.h>

// ==============================================================================
// 1. NODE IDENTITY & ROLE
// ==============================================================================
// Node identifiers: S-01, S-02, S-03, S-04, S-05
#define NODE_ID                     "S-03"
#define FIRMWARE_VERSION            "1.0.0"
#define DEVICE_ID                   "esp32-m01-s03"
#define DEVICE_KEY                  "php-s-03-ee1ff4f1cd060acffb5b19b40dc2a1fe7be4"

// Network and backend: the backend laptop IP changes when the WiFi network changes.
#define WIFI_SSID                   "Realme 11"
#define WIFI_PASS                   "12345678900987654321"
#define API_BASE_URL                "http://10.72.136.174:3000"

// Role selection (Set exactly one to true or combine per node requirements)
#define IS_DOWNTIME_MASTER          true   // True for Node 3 (Main welding drive inactivity timeout)
#define IS_PRODUCTION_COUNTER       false  // True for Node 5 (Falling-edge pipe output counter)

// ==============================================================================
// 2. HARDWARE GPIO PIN CONFIGURATION (ESP32 DevKit V1 38-Pin)
// ==============================================================================
#define SENSOR_INPUT_PIN            4      // GPIO 4 (Pin D4) - Safe general input
#define ONBOARD_LED_PIN             2      // GPIO 2 (Onboard blue LED for visual debug)

// Active state of sensor input:
// Under PC817 optocoupler, transistor pulls GPIO to GND when metal is detected -> Active LOW
#define SENSOR_ACTIVE_STATE         LOW    

// ==============================================================================
// 3. SIGNAL FILTERING & TIMING THRESHOLDS
// ==============================================================================
#define SENSOR_DEBOUNCE_MS          30     // Stable duration required to confirm state change (ms)
#define MIN_PULSE_INTERVAL_MS       1500   // Max 1 HTTP pulse event per 1.5s while rotating
#define INACTIVITY_TIMEOUT_MS       30000  // No edges for this long -> report spindle stopped (idle event)
#define IDLE_RETRY_INTERVAL_MS      5000   // Gap between idle event retries when the backend is unreachable
#define HEARTBEAT_INTERVAL_MS       10000  // Periodic status report interval (ms)
#define STATUS_PRINT_INTERVAL_MS    3000   // Serial status update interval (ms)
#define WIFI_RECONNECT_INTERVAL_MS  5000   // Minimum gap (ms) between WiFi reconnect attempts

// ==============================================================================
// 4. DIAGNOSTICS & SERIAL CONFIGURATION
// ==============================================================================
#define SERIAL_BAUD_RATE            115200 // Serial Monitor baud rate
#define ENABLE_VERBOSE_LOG          true   // Set to true for development logging
