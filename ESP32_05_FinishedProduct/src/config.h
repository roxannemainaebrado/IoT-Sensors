#pragma once
#include <Arduino.h>

// ==============================================================================
// 1. NODE IDENTITY & PURPOSE
// ==============================================================================
#define NODE_ID                     "S-05"
#define NODE_ROLE_NAME              "Production Output Cutting Monitor"
#define FIRMWARE_VERSION            "1.1.0"
#define DEVICE_ID                   "esp32-m01-s05"
#define DEVICE_KEY                  "php-s-05-475aa2de5d03af0d2b58863bb80eab4fca3d"

// ==============================================================================
// 2. NETWORK & BACKEND CONFIGURATION
// ==============================================================================
#define WIFI_SSID                   "Realme 11"
#define WIFI_PASS                   "12345678900987654321"
#define API_BASE_URL                "http://10.72.136.174:3000"

// ==============================================================================
// 3. HARDWARE GPIO PIN CONFIGURATION (ESP32 DevKit V1 38-Pin)
// ==============================================================================
#define SENSOR_INPUT_PIN            4      // GPIO 4 (Pin D4) - PC817 collector input
#define ONBOARD_LED_PIN             2      // GPIO 2 - Built-in indicator LED

// Signal Polarity (Active-LOW with PC817 optocoupler):
// - Metal pipe present at proximity sensor -> PC817 conducts -> GPIO 4 pulled to GND (LOW)
// - Gap between pipes                      -> PC817 off       -> Internal pullup holds at 3.3V (HIGH)
#define SENSOR_DETECTED_STATE       LOW    // LOW  = Metal pipe present
#define SENSOR_CLEARED_STATE        HIGH   // HIGH = No pipe present

#define SENSOR_DEBOUNCE_MS          50     // Stable duration (ms) to confirm pipe detection
#define WIFI_RECONNECT_INTERVAL_MS  5000   // Minimum gap (ms) between WiFi reconnect attempts
#define HEARTBEAT_INTERVAL_MS       10000  // Periodic heartbeat interval (ms)
#define SERIAL_BAUD_RATE            115200 // Serial Monitor baud rate
