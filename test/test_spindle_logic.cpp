#include "spindle_logic.h"

constexpr int PIN_LOW_ACTIVE = 0;
constexpr int PIN_HIGH_IDLE  = 1;
constexpr uint32_t DEBOUNCE_MS = 50;
constexpr uint32_t MIN_STALL_MS = 500;
constexpr uint32_t MAX_STALL_MS = 30000;
constexpr uint32_t DEF_STALL_MS = 3000;
constexpr uint32_t DEF_TRIG_MS  = 60000;
constexpr uint32_t DEF_REC_MS   = 60000;

// ---------------------------------------------------------------------------
// BUG C1: Out-of-order timestamp race (nowMs < startMs) must NOT underflow
//         into 4294967295U and cause 0ms phantom debounce or false stall.
// ---------------------------------------------------------------------------
constexpr bool testBugC1NoUnsignedUnderflowOnOutOfOrderTimestamp() {
    SpindleThresholdConfig cfg{3000, 60000, 60000, true, false};
    SpindleIsrState isr = makeSpindleIsrState(PIN_HIGH_IDLE, 900, 3000);
    SpindleRuntimeState rt = makeSpindleRuntimeState(PIN_HIGH_IDLE, 900);

    // Put spindle into RUNNING via two normal debounced edges at 950 and 1051
    spindleIsrOnPinSample(isr, 900, PIN_LOW_ACTIVE, PIN_LOW_ACTIVE, DEBOUNCE_MS);
    spindleIsrOnPinSample(isr, 1001, PIN_HIGH_IDLE, PIN_LOW_ACTIVE, DEBOUNCE_MS); // confirms edge 1 at 950
    SpindleEdgeBatch b1 = spindleConsumeCycleLocked(isr, 1060, PIN_HIGH_IDLE, PIN_LOW_ACTIVE, DEBOUNCE_MS); // confirms edge 2 at 1051
    spindleStep(rt, b1, 1060, cfg);
    if (rt.currentState != SPINDLE_RUNNING) return false;

    // Simulate ISR updating candidateStartMs to 1101, then a stale nowMs = 1100 arriving:
    spindleIsrOnPinSample(isr, 1101, PIN_LOW_ACTIVE, PIN_LOW_ACTIVE, DEBOUNCE_MS);
    SpindleEdgeBatch bRace = spindleConsumeCycleLocked(isr, 1100, PIN_LOW_ACTIVE, PIN_LOW_ACTIVE, DEBOUNCE_MS);
    // Must NOT confirm a 0ms phantom edge!
    if (bRace.totalEdges != 0) return false;

    // Simulate rt.lastEdgeTime = 1151 while stale nowMs = 1100: must NOT false-stall RUNNING -> IDLE!
    rt.lastEdgeTime = 1151;
    spindleStep(rt, bRace, 1100, cfg);
    return rt.currentState == SPINDLE_RUNNING;
}

// Also verify legitimate 32-bit millis() wraparound at 49.7 days still works!
constexpr bool testMillisWraparoundAt49Days() {
    uint32_t beforeWrap = 0xFFFFFFE0UL; // 32 ms before wrap
    uint32_t afterWrap  = 0x00000020UL; // 32 ms after wrap -> 64 ms elapsed
    return elapsedAtLeast(afterWrap, beforeWrap, 50) && !elapsedAtLeast(afterWrap, beforeWrap, 100);
}

// ---------------------------------------------------------------------------
// BUG C2: Multi-edge gaps during HTTP blocking must NOT falsely trigger RUNNING,
//         hide a recovery stall, or inflate lastEdgeGapMs.
// ---------------------------------------------------------------------------
constexpr bool testBugC2WidelySpacedEdgesDuringHttpBlockDoNotEnterRunning() {
    SpindleThresholdConfig cfg{1000, 60000, 60000, true, false};
    SpindleIsrState isr = makeSpindleIsrState(PIN_HIGH_IDLE, 0, 1000);
    SpindleRuntimeState rt = makeSpindleRuntimeState(PIN_HIGH_IDLE, 0);

    // HTTP blocks loop() from t=0 to t=4000.
    // Edge 1: LOW starts at t=50, confirmed at t=100 when pin returns HIGH at t=3750.
    spindleIsrOnPinSample(isr, 50, PIN_LOW_ACTIVE, PIN_LOW_ACTIVE, DEBOUNCE_MS);
    spindleIsrOnPinSample(isr, 3750, PIN_HIGH_IDLE, PIN_LOW_ACTIVE, DEBOUNCE_MS); // Edge 1 @ 100ms
    // Edge 2: HIGH from t=3750 confirmed at t=3800 when loop() consumes at t=4000 (or pin changes at t=3850).
    spindleIsrOnPinSample(isr, 3850, PIN_LOW_ACTIVE, PIN_LOW_ACTIVE, DEBOUNCE_MS); // Edge 2 @ 3800ms (gap = 3700ms >= 1000ms!)

    SpindleEdgeBatch batch = spindleConsumeCycleLocked(isr, 3860, PIN_LOW_ACTIVE, PIN_LOW_ACTIVE, DEBOUNCE_MS);
    spindleStep(rt, batch, 3860, cfg);

    // Gap between Edge 1 (100ms) and Edge 2 (3800ms) was 3700ms >= 1000ms stall window!
    // Must stay SPINDLE_IDLE with consecutiveEdges == 1, and lastEdgeGapMs == 3700 (not 3800).
    return rt.currentState == SPINDLE_IDLE
        && rt.consecutiveEdges == 1
        && rt.lastEdgeGapMs == 3700;
}

constexpr bool testBugC2StallDuringHttpBlockResetsRecoveringToFault() {
    SpindleThresholdConfig cfg{1000, 10000, 5000, true, false};
    SpindleIsrState isr = makeSpindleIsrState(PIN_HIGH_IDLE, 1000, 1000);
    SpindleRuntimeState rt = makeSpindleRuntimeState(PIN_HIGH_IDLE, 1000);
    rt.currentState = SPINDLE_RECOVERING;
    rt.wasFaulted = true;
    rt.recoveryStartTime = 1000;
    rt.lastEdgeTime = 1000;
    rt.consecutiveEdges = 4;

    // HTTP blocks from t=1000 to t=5000. Spindle stalls for 3800ms (1000..4800),
    // then produces a single edge at t=4850 (LOW at 4800, confirmed at 4850).
    spindleIsrOnPinSample(isr, 4800, PIN_LOW_ACTIVE, PIN_LOW_ACTIVE, DEBOUNCE_MS);
    SpindleEdgeBatch batch = spindleConsumeCycleLocked(isr, 4900, PIN_LOW_ACTIVE, PIN_LOW_ACTIVE, DEBOUNCE_MS);
    spindleStep(rt, batch, 4900, cfg);

    // Must abort SPINDLE_RECOVERING -> SPINDLE_FAULT!
    return rt.currentState == SPINDLE_FAULT && rt.recoveryStartTime == 0 && rt.consecutiveEdges == 1;
}

constexpr bool testBugC2LastEdgeGapTelemetryUsesSinglePhaseDuration() {
    SpindleThresholdConfig cfg{2000, 60000, 60000, true, false};
    SpindleIsrState isr = makeSpindleIsrState(PIN_HIGH_IDLE, 1000, 2000);
    SpindleRuntimeState rt = makeSpindleRuntimeState(PIN_HIGH_IDLE, 1000);
    rt.currentState = SPINDLE_RUNNING;
    rt.lastEdgeTime = 1000;
    rt.consecutiveEdges = 4;

    // Three normal rotation edges during HTTP call:
    // Edge 1 @ 1400 (gap 400), Edge 2 @ 2020 (gap 620), Edge 3 @ 2470 (gap 450)
    spindleIsrOnPinSample(isr, 1350, PIN_LOW_ACTIVE, PIN_LOW_ACTIVE, DEBOUNCE_MS);
    spindleIsrOnPinSample(isr, 1970, PIN_HIGH_IDLE, PIN_LOW_ACTIVE, DEBOUNCE_MS); // confirms @ 1400 (gap 400)
    spindleIsrOnPinSample(isr, 2420, PIN_LOW_ACTIVE, PIN_LOW_ACTIVE, DEBOUNCE_MS); // confirms @ 2020 (gap 620)
    SpindleEdgeBatch batch = spindleConsumeCycleLocked(isr, 2500, PIN_LOW_ACTIVE, PIN_LOW_ACTIVE, DEBOUNCE_MS); // confirms @ 2470 (gap 450)
    spindleStep(rt, batch, 2500, cfg);

    return rt.currentState == SPINDLE_RUNNING
        && rt.lastEdgeGapMs == 450
        && rt.maxObservedEdgeGapMs == 620;
}

// ---------------------------------------------------------------------------
// BUG C3: Disabling absenceDetectionEnabled must exit SPINDLE_FAULT -> SPINDLE_IDLE
//         and SPINDLE_RECOVERING -> SPINDLE_RUNNING immediately, and skip RECOVERING.
// ---------------------------------------------------------------------------
constexpr bool testBugC3DisablingAbsenceDetectionExitsFaultAndRecovering() {
    SpindleThresholdConfig cfg{3000, 60000, 60000, false, false};
    SpindleEdgeBatch emptyBatch{0, 1000, PIN_HIGH_IDLE, 0, 0, 0, 0, 0, 1000, 1000, false};

    // Case A: Stopped in SPINDLE_FAULT -> must immediately reconcile to SPINDLE_IDLE and sync state
    SpindleRuntimeState rtFault = makeSpindleRuntimeState(PIN_HIGH_IDLE, 0);
    rtFault.currentState = SPINDLE_FAULT;
    rtFault.wasFaulted = true;
    rtFault.lastEdgeTime = 0;
    SpindleStepResult actA = spindleStep(rtFault, emptyBatch, 5000, cfg);
    if (rtFault.currentState != SPINDLE_IDLE || actA.syncAction != SPINDLE_SYNC_STATE || !rtFault.wasFaulted) {
        return false;
    }

    // Next 2 rotation edges while absenceDetectionEnabled == false must go straight to SPINDLE_RUNNING (not RECOVERING)
    SpindleIsrState isr = makeSpindleIsrState(PIN_HIGH_IDLE, 5000, 3000);
    spindleIsrOnPinSample(isr, 5100, PIN_LOW_ACTIVE, PIN_LOW_ACTIVE, DEBOUNCE_MS);
    spindleIsrOnPinSample(isr, 5300, PIN_HIGH_IDLE, PIN_LOW_ACTIVE, DEBOUNCE_MS); // edge 1 @ 5150
    SpindleEdgeBatch twoEdges = spindleConsumeCycleLocked(isr, 5400, PIN_HIGH_IDLE, PIN_LOW_ACTIVE, DEBOUNCE_MS); // edge 2 @ 5350
    SpindleStepResult actRotate = spindleStep(rtFault, twoEdges, 5400, cfg);
    if (rtFault.currentState != SPINDLE_RUNNING || actRotate.syncAction != SPINDLE_SYNC_STATE) {
        return false;
    }

    // Case B: Rotating in SPINDLE_RECOVERING -> must immediately reconcile to SPINDLE_RUNNING
    SpindleRuntimeState rtRec = makeSpindleRuntimeState(PIN_HIGH_IDLE, 0);
    rtRec.currentState = SPINDLE_RECOVERING;
    rtRec.wasFaulted = true;
    rtRec.recoveryStartTime = 4000;
    rtRec.lastEdgeTime = 4900;
    SpindleStepResult actB = spindleStep(rtRec, emptyBatch, 5000, cfg);
    return rtRec.currentState == SPINDLE_RUNNING && rtRec.recoveryStartTime == 0 && actB.syncAction == SPINDLE_SYNC_STATE;
}

// ---------------------------------------------------------------------------
// BUG C4: effectiveStallMs must NOT be clamped by stale triggerLimitMs when
//         absenceDetectionEnabled == false.
// ---------------------------------------------------------------------------
constexpr bool testBugC4EffectiveStallIgnoresTriggerWhenAbsenceDisabled() {
    uint32_t whenDisabled = computeEffectiveStallMs(false, 25000, 10000);
    uint32_t whenEnabled  = computeEffectiveStallMs(true, 25000, 10000);
    return whenDisabled == 25000 && whenEnabled == 10000;
}

// ---------------------------------------------------------------------------
// BUG C5: Strict STALL command parsing, NVS boot bounds, NVS wear avoidance,
//         and serialStallOverride retention across same-version heartbeats.
// ---------------------------------------------------------------------------
constexpr bool testBugC5SerialParserAndThresholdSync() {
    int32_t parsed = -1;
    if (parseStallCommandMs("STATUS", MIN_STALL_MS, MAX_STALL_MS, &parsed) != STALL_CMD_NOT_MATCHED) return false;
    if (parseStallCommandMs("STALLED 1000", MIN_STALL_MS, MAX_STALL_MS, &parsed) != STALL_CMD_NOT_MATCHED) return false;
    if (parseStallCommandMs("STALL 4500", MIN_STALL_MS, MAX_STALL_MS, &parsed) != STALL_CMD_VALID || parsed != 4500) return false;
    if (parseStallCommandMs("stall=500", MIN_STALL_MS, MAX_STALL_MS, &parsed) != STALL_CMD_VALID || parsed != 500) return false;
    if (parseStallCommandMs("STALL = 30000", MIN_STALL_MS, MAX_STALL_MS, &parsed) != STALL_CMD_VALID || parsed != 30000) return false;
    if (parseStallCommandMs("STALL 500s", MIN_STALL_MS, MAX_STALL_MS, &parsed) != STALL_CMD_INVALID) return false;
    if (parseStallCommandMs("STALL abc", MIN_STALL_MS, MAX_STALL_MS, &parsed) != STALL_CMD_INVALID) return false;
    if (parseStallCommandMs("STALL 499", MIN_STALL_MS, MAX_STALL_MS, &parsed) != STALL_CMD_INVALID || parsed != 499) return false;
    if (parseStallCommandMs("STALL 30001", MIN_STALL_MS, MAX_STALL_MS, &parsed) != STALL_CMD_INVALID || parsed != 30001) return false;

    // NVS boot sanitization clamps corrupt trig_ms=0 / rec_ms=9999999 back to defaults
    SpindleThresholdConfig bootCfg = sanitizeBootThresholds(
        100, 0, 9999999, true,
        MIN_STALL_MS, MAX_STALL_MS, DEF_STALL_MS, DEF_TRIG_MS, DEF_REC_MS);
    if (bootCfg.rotationStallMs != DEF_STALL_MS
        || bootCfg.triggerLimitMs != DEF_TRIG_MS
        || bootCfg.recoveryLimitMs != DEF_REC_MS) {
        return false;
    }

    // Boot sync where backend values equal NVS values (only versionChanged=true) must NOT write NVS!
    ThresholdUpdateOutcome bootSync = applyThresholdUpdateLogic(
        bootCfg, nullptr, 1000,
        60, 60, 1, 3000,
        false, true, false,
        MIN_STALL_MS, MAX_STALL_MS);
    if (bootSync.shouldPersistNvs) return false;

    // Serial override sets 5500ms; subsequent same-version heartbeat sending 3000ms must NOT clobber 5500ms!
    applyThresholdUpdateLogic(
        bootCfg, nullptr, 2000,
        -1, -1, -1, 5500,
        true, false, false,
        MIN_STALL_MS, MAX_STALL_MS);
    if (bootCfg.rotationStallMs != 5500 || !bootCfg.serialStallOverride) return false;

    applyThresholdUpdateLogic(
        bootCfg, nullptr, 12000,
        60, 60, 1, 3000,
        false, false, false,
        MIN_STALL_MS, MAX_STALL_MS);
    if (bootCfg.rotationStallMs != 5500) return false;

    // When Admin saves a new settings version in Web UI (backendVersionAdvanced=true), backend takes over!
    applyThresholdUpdateLogic(
        bootCfg, nullptr, 22000,
        60, 60, 1, 4200,
        false, true, true,
        MIN_STALL_MS, MAX_STALL_MS);
    return bootCfg.rotationStallMs == 4200 && !bootCfg.serialStallOverride;
}

constexpr bool testFullLifecycleAndReEnableBaselineReset() {
    SpindleThresholdConfig cfg{1000, 5000, 3000, false, false};
    SpindleIsrState isr = makeSpindleIsrState(PIN_HIGH_IDLE, 0, 1000);
    SpindleRuntimeState rt = makeSpindleRuntimeState(PIN_HIGH_IDLE, 0);

    // Sitting IDLE for 60,000ms while absenceDetectionEnabled == false does NOT fault
    SpindleEdgeBatch empty = spindleConsumeCycleLocked(isr, 60000, PIN_HIGH_IDLE, PIN_LOW_ACTIVE, DEBOUNCE_MS);
    spindleStep(rt, empty, 60000, cfg);
    if (rt.currentState != SPINDLE_IDLE) return false;

    // Enabling absenceDetectionEnabled at t=60000 resets absenceStartTime to 60000 (no instant fault at 60100!)
    applyThresholdUpdateLogic(cfg, &rt, 60000, 5, 3, 1, 1000, false, true, true, MIN_STALL_MS, MAX_STALL_MS);
    spindleStep(rt, empty, 60100, cfg);
    if (rt.currentState != SPINDLE_IDLE) return false;

    // At t=65000 (5000ms after enable), escalates IDLE -> FAULT
    spindleStep(rt, empty, 65000, cfg);
    if (rt.currentState != SPINDLE_FAULT || !rt.wasFaulted) return false;

    // Two edges at 65150 & 65350 enter SPINDLE_RECOVERING
    spindleIsrOnPinSample(isr, 65100, PIN_LOW_ACTIVE, PIN_LOW_ACTIVE, DEBOUNCE_MS);
    spindleIsrOnPinSample(isr, 65300, PIN_HIGH_IDLE, PIN_LOW_ACTIVE, DEBOUNCE_MS);
    SpindleEdgeBatch recStart = spindleConsumeCycleLocked(isr, 65400, PIN_HIGH_IDLE, PIN_LOW_ACTIVE, DEBOUNCE_MS);
    spindleStep(rt, recStart, 65400, cfg);
    if (rt.currentState != SPINDLE_RECOVERING) return false;

    // Continuous edges every 500ms (< 1000ms stall) through recoveryLimitMs (3000ms -> t=68400) -> RUNNING!
    uint32_t t = 65400;
    int level = PIN_LOW_ACTIVE;
    for (int i = 0; i < 7; i++) {
        t += 450;
        spindleIsrOnPinSample(isr, t, level, PIN_LOW_ACTIVE, DEBOUNCE_MS);
        t += 60;
        SpindleEdgeBatch b = spindleConsumeCycleLocked(isr, t, level, PIN_LOW_ACTIVE, DEBOUNCE_MS);
        spindleStep(rt, b, t, cfg);
        level = (level == PIN_LOW_ACTIVE) ? PIN_HIGH_IDLE : PIN_LOW_ACTIVE;
    }
    return rt.currentState == SPINDLE_RUNNING;
}

static_assert(testBugC1NoUnsignedUnderflowOnOutOfOrderTimestamp(), "C1 FAILED: Out-of-order timestamp underflowed");
static_assert(testMillisWraparoundAt49Days(), "C1b FAILED: 49.7-day wraparound broken");
static_assert(testBugC2WidelySpacedEdgesDuringHttpBlockDoNotEnterRunning(), "C2a FAILED: Widely spaced edges falsely entered RUNNING");
static_assert(testBugC2StallDuringHttpBlockResetsRecoveringToFault(), "C2b FAILED: Stall during HTTP block failed to reset RECOVERING");
static_assert(testBugC2LastEdgeGapTelemetryUsesSinglePhaseDuration(), "C2c FAILED: Multi-edge batch inflated lastEdgeGapMs");
static_assert(testBugC3DisablingAbsenceDetectionExitsFaultAndRecovering(), "C3 FAILED: Disabling absence detection trapped FAULT/RECOVERING");
static_assert(testBugC4EffectiveStallIgnoresTriggerWhenAbsenceDisabled(), "C4 FAILED: Stale triggerLimitMs clamped stall when disabled");
static_assert(testBugC5SerialParserAndThresholdSync(), "C5 FAILED: Serial STALL parser / NVS wear / override retention");
static_assert(testFullLifecycleAndReEnableBaselineReset(), "Lifecycle FAILED");
