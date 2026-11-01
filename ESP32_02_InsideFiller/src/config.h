#pragma once
#include <Arduino.h>

// ==============================================================================
// 1. NODE IDENTITY & PURPOSE
// ==============================================================================
#define NODE_ID                     "S-02"
#define NODE_ROLE_NAME              "Inside Filler Wire Monitor"
#define FIRMWARE_VERSION            "1.1.0"
#define DEVICE_ID                   "esp32-m01-s02"
#define DEVICE_KEY                  "php-s-02-device-key-placeholder"

// ==============================================================================
// 2. NETWORK & BACKEND CONFIGURATION
// ==============================================================================
#define WIFI_SSID                   "Realme 11 280"
#define WIFI_PASS                   "12345678900987654321"
#define API_BASE_URL                "http://10.72.136.174:3000"

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
#define SENSOR_DEBOUNCE_MS          50     // Stable duration (ms) to confirm state change
#define HEARTBEAT_INTERVAL_MS       10000  // Periodic heartbeat interval (ms)
#define SERIAL_BAUD_RATE            115200 // Serial Monitor baud rate
