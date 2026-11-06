#pragma once
#include <Arduino.h>

// ==============================================================================
// 1. NODE IDENTITY & PURPOSE
// ==============================================================================
#define NODE_ID                     "S-01"
#define NODE_ROLE_NAME              "Raw Material Coil Monitor"
#define FIRMWARE_VERSION            "1.1.0"
#define DEVICE_ID                   "esp32-m01-s01"
#define DEVICE_KEY                  "php-s-01-69aa381a2460fae81724491b616ebca9e257"

// ==============================================================================
// 2. NETWORK & BACKEND CONFIGURATION
// ==============================================================================
#define WIFI_SSID                   "Realme 11"
#define WIFI_PASS                   "12345678900987654321"
#define API_BASE_URL                "http://10.72.136.174:3000"

// ==============================================================================
// 3. HARDWARE GPIO PIN CONFIGURATION (ESP32 DevKit V1 38-Pin)
// ==============================================================================
#define SENSOR_INPUT_PIN            4      // GPIO 4 (Pin D4)
#define ONBOARD_LED_PIN             2      // GPIO 2 (Visual indicator)

// Optocoupler Inverted Logic Polarity:
// - Metal present at proximity sensor -> Sensor 12V output ON -> PC817 LED ON
// - PC817 phototransistor conducts -> GPIO 4 pulled to GND (LOW)
// - No metal at proximity sensor      -> Sensor output OFF     -> PC817 LED OFF
// - Internal pullup holds GPIO 4 at 3.3V (HIGH)
#define SENSOR_ACTIVE_STATE         LOW    // LOW = Metal detected (Coil present)

#define LED_ALERT_STATE             HIGH   // Turn LED ON when coil missing (alert)
#define LED_NORMAL_STATE            LOW    // Turn LED OFF when coil present (normal)

// ==============================================================================
// 4. SIGNAL FILTERING & TIMING THRESHOLDS
// ==============================================================================
#define SENSOR_DEBOUNCE_MS          50     // Stable duration (ms) to confirm state change
#define WIFI_RECONNECT_INTERVAL_MS  5000   // Minimum gap (ms) between WiFi reconnect attempts
#define HEARTBEAT_INTERVAL_MS       10000  // Background heartbeat interval (ms)
#define SERIAL_BAUD_RATE            115200 // Serial Monitor baud rate
