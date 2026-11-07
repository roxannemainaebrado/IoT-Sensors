# IoT Sensor Nodes — Firmware Architecture & Changelog

This repository contains the ESP32 firmware for the 5-node industrial pipe manufacturing monitoring system (`S-01` through `S-05`). Each node interfaces with an inductive proximity sensor via a PC817 optocoupler on `GPIO 4` (`INPUT_PULLUP`, Active-LOW) and drives an onboard visual status indicator on `GPIO 2`.

---

## 1. Sensor Node Overview

| Node ID | Directory | Device ID | Role | Detection Mode |
| :--- | :--- | :--- | :--- | :--- |
| **S-01** | `ESP32_01_RawMaterial/` | `esp32-m01-s01` | Raw Material Coil Monitor | Continuous metal presence + Dynamic Trigger/Recovery limits |
| **S-02** | `ESP32_02_InsideFiller/` | `esp32-m01-s02` | Inside Filler Wire Monitor | Continuous metal presence + Dynamic Trigger/Recovery limits |
| **S-03** | `src/` (Root Project) | `esp32-m01-s03` | Machine Main Spindle Monitor | Continuous `ON/OFF` rotation transitions + Dynamic Stall/Trigger/Recovery limits |
| **S-04** | `ESP32_04_OutsideFiller/` | `esp32-m01-s04` | Outside Filler Wire Monitor | Continuous metal presence + Dynamic Trigger/Recovery limits |
| **S-05** | `ESP32_05_FinishedProduct/` | `esp32-m01-s05` | Production Output Cutting Monitor | Complete `METAL (LOW) -> NO METAL (HIGH)` cycle counter |

---

## 2. Summary of Recent Firmware Changes

### A. Centralized Node Configuration & Dynamic Timing Thresholds (`config.h`)
* **Network & Device Credentials:** Updated `API_BASE_URL` across all 5 sensor nodes (`http://10.72.136.68:3000`) and synchronized provisioned `DEVICE_KEY` tokens and `WIFI_SSID` (`Realme 11`).
* **Dynamic Operational Thresholds (S-01, S-02, S-03, S-04):**
  * Added `MIN_PULSE_INTERVAL_MS` (`1500 ms`) to emit periodic active pulses while Running so the backend watchdog maintains active status.
  * Added `DEFAULT_TRIGGER_LIMIT_MS` (`60000 ms`) and `DEFAULT_RECOVERY_LIMIT_MS` (`60000 ms`) fallback thresholds prior to backend synchronization.
  * Added `EVENT_RETRY_INTERVAL_MS` (`5000 ms`) and `STATUS_PRINT_INTERVAL_MS` (`3000 ms`).
* **Spindle Rotation Stall Window (S-03):**
  * Added `DEFAULT_ROTATION_STALL_MS` (`3000 ms`) with configurable bounds (`MIN_ROTATION_STALL_MS = 500 ms`, `MAX_ROTATION_STALL_MS = 30000 ms`).
* **Output Counter Queue & Polarity Constants (S-05):**
  * Bumped firmware version to `1.2.0`.
  * Defined explicit `SENSOR_DETECTED_STATE` (`LOW`) and `SENSOR_CLEARED_STATE` (`HIGH`) constants and `MAX_PENDING_PULSES` (`32`) for offline pulse buffering.

### B. Material & Filler Availability State Machine (`S-01`, `S-02`, `S-04`)
* **4-State Lifecycle (`RUNNING`, `IDLE`, `FAULT`, `RECOVERING`):**
  * **Running (`PRESENT`):** Metal is detected (`LOW`). LED is `OFF`. Emits `pulse` (`active`) events every `1500 ms` and reports `activityObserved = true` on heartbeats.
  * **Idle (`IDLE`):** Metal is absent (`HIGH`) within `triggerLimitMs`. LED turns `ON` immediately as a visual alert and emits `idle` (`idle`).
  * **Fault (`FAULT`):** Metal remains absent for `>= triggerLimitMs` (when `absenceDetectionEnabled` is active). Escalates to `FAULT` and emits `fault` (`fault`).
  * **Fault Recovery Hold (`RECOVERING`):** When metal is re-detected after a `FAULT`, the node holds `FAULT` status in `RECOVERING` until metal remains continuously present for `>= recoveryLimitMs`. Once confirmed, it transitions to `RUNNING` and emits `recovered` (`active`, `recoveryConfirmed: true`). If metal is lost before `recoveryLimitMs` elapses, it returns to `FAULT`.
* **Dynamic Backend Settings Sync & NVS Persistence:**
  * Fetches `triggerSeconds`, `recoverySeconds`, and `absenceDetectionEnabled` from `GET /api/iot/config` on boot and parses `x-trigger-seconds`, `x-recovery-seconds`, `x-absence-enabled`, and `x-settings-version` response headers on every HTTP POST.
  * Persists updated thresholds and monotonic `bootCounter` in ESP32 NVS (`Preferences`) so settings survive reboots.

### C. Machine Main Spindle Rotation Monitor & ISR Engine (`S-03`)
* **Hardware ISR Edge Capture (`src/spindle_logic.h` & `src/main.cpp`):**
  * Implemented `onSensorPinChangeISR()` on `GPIO 4` (`CHANGE`) protected by a critical section spinlock (`sensorMux`) to capture fast `ON/OFF` inductive sensor transitions even while `HTTPClient` blocks `loop()`.
  * Detects spindle stalls whether the shaft stops on constant metal (`CONSTANT_METAL_DOWNTIME` / `LOW`) or constant gap (`CONSTANT_NO_METAL_DOWNTIME` / `HIGH`) for `>= effectiveStallMs`.
* **2-Edge Rotation & Fault Recovery Confirmation:**
  * Requires `>= 2` consecutive debounced edges without stalling to confirm spindle rotation.
  * Evaluates mid-batch stalls during HTTP blocking so widely spaced stray edges never falsely trigger `RUNNING` or bypass `RECOVERING`.
  * Reconciles state immediately (`FAULT -> IDLE`, `RECOVERING -> RUNNING`) if `absenceDetectionEnabled` is disabled from the web dashboard.
* **Serial Diagnostics & Runtime Override:**
  * Supports `STATUS` command to inspect live dwell/gap timings and `STALL <ms>` (`500`–`30000 ms`) for live serial stall window overrides, which persist until the backend advances `x-settings-version`.
* **Compile-Time Unit Test Suite (`test/test_spindle_logic.cpp`):**
  * Added `constexpr` / `static_assert` verification covering 32-bit timer wraparound (`49.7 days`), out-of-order timestamp race protection, HTTP-blocked multi-edge stall handling, absence detection toggling, and `STALL` command parsing.

### D. Finished Product Output Counter (`S-05`)
* **2-Step `METAL -> NO METAL` Cycle Counting:**
  * Step 1 (`LOW` confirmed): Arms `METAL_DETECTED` state and turns onboard LED `ON`.
  * Step 2 (`HIGH` confirmed): Completes the pipe pass cycle (`WAITING_FOR_METAL`), turns LED `OFF`, records `passDurationMs`, and queues 1 `pulse` (`active`) event.
* **Decoupled from Absence/Idle/Fault:**
  * Operates strictly as an output counter without local idle or fault timers.
* **Backend Output Validation & Ring-Buffer Queue:**
  * Queues up to `32` completed pipe pulses (`PendingPulse`) with original `eventId` and `recordedAt` timestamps for automatic retry across transient WiFi/HTTP errors.
  * Parses `outputAccepted` and `outputRejectionReason` from the backend response: increments `pipeProductionCount` (persisted in NVS) only when accepted, and logs rejected outputs (e.g., during `machine_downtime`) without double-counting.
