#pragma once
#include <stdint.h>
#include <stddef.h>

#ifdef __GNUC__
#define SPINDLE_ALWAYS_INLINE __attribute__((always_inline))
#else
#define SPINDLE_ALWAYS_INLINE
#endif

#if __cplusplus >= 201402L
#define SPINDLE_CONSTEXPR constexpr inline
#else
#define SPINDLE_CONSTEXPR inline
#endif

enum SpindleState {
    SPINDLE_STATE_UNKNOWN = -1,
    SPINDLE_RUNNING    = 0,
    SPINDLE_IDLE       = 1,
    SPINDLE_FAULT      = 2,
    SPINDLE_RECOVERING = 3
};

enum StallCmdParseResult {
    STALL_CMD_NOT_MATCHED = 0,
    STALL_CMD_INVALID     = 1,
    STALL_CMD_VALID       = 2
};

enum SpindleSyncAction {
    SPINDLE_SYNC_NONE  = 0,
    SPINDLE_SYNC_STATE = 1
};

enum SpindleStepEvent {
    SPINDLE_EVENT_NONE                     = 0,
    SPINDLE_EVENT_ENTERED_RECOVERING       = 1,
    SPINDLE_EVENT_ENTERED_RUNNING          = 2,
    SPINDLE_EVENT_RECOVERY_ABORTED_STALL   = 3,
    SPINDLE_EVENT_STOPPED_TO_IDLE          = 4,
    SPINDLE_EVENT_RECOVERY_MET             = 5,
    SPINDLE_EVENT_TRIGGER_EXCEEDED         = 6,
    SPINDLE_EVENT_ABSENCE_DISABLED_CLEARED = 7
};

struct SpindleThresholdConfig {
    uint32_t rotationStallMs;
    uint32_t triggerLimitMs;
    uint32_t recoveryLimitMs;
    bool     absenceDetectionEnabled;
    bool     serialStallOverride;
};

struct ThresholdUpdateOutcome {
    bool anyChanged;
    bool shouldPersistNvs;
};

struct SpindleIsrState {
    int      confirmedState;
    int      candidateState;
    uint32_t candidateStartMs;
    uint32_t confirmedStateStartMs;
    uint32_t effectiveStallMs;
    uint32_t queuedEdges;
    uint32_t queuedLastEdgeMs;
    uint32_t lastLowDwellMs;
    uint32_t lastHighGapMs;
    uint32_t batchLastPhaseMs;
    uint32_t batchMaxRotatingPhaseMs;
    uint32_t batchPostStallEdges;
    uint32_t batchPostStallFirstEdgeMs;
    uint32_t batchStallAnchorMs;
    bool     batchStallOccurred;
};

struct SpindleEdgeBatch {
    uint32_t totalEdges;
    uint32_t lastEdgeMs;
    int      confirmedPin;
    uint32_t lastLowDwellMs;
    uint32_t lastHighGapMs;
    uint32_t lastPhaseMs;
    uint32_t maxRotatingPhaseMs;
    uint32_t postStallEdges;
    uint32_t postStallFirstEdgeMs;
    uint32_t stallAnchorMs;
    bool     stallOccurred;
};

struct SpindleRuntimeState {
    SpindleState currentState;
    SpindleState previousState;
    int          debouncedPinState;
    uint32_t     stateStartTime;
    uint32_t     absenceStartTime;
    uint32_t     recoveryStartTime;
    uint32_t     lastEdgeTime;
    uint32_t     consecutiveEdges;
    uint32_t     pulseCount;
    uint32_t     lastPulseSentTime;
    uint32_t     lastEventRetryTime;
    uint32_t     stateChangeCount;
    uint32_t     lastEdgeGapMs;
    uint32_t     maxObservedEdgeGapMs;
    bool         activitySinceHb;
    bool         wasFaulted;
};

struct SpindleStepResult {
    SpindleSyncAction syncAction;
    SpindleStepEvent  stepEvent;
    uint32_t          heldRecoveryMs;
};

// Handles 49.7-day uint32_t wraparound while rejecting out-of-order (nowMs < startMs) underflow.
SPINDLE_ALWAYS_INLINE SPINDLE_CONSTEXPR bool elapsedAtLeast(uint32_t nowMs, uint32_t startMs, uint32_t thresholdMs) {
    uint32_t diff = nowMs - startMs;
    return (diff < 0x80000000UL) && (diff >= thresholdMs);
}

SPINDLE_CONSTEXPR uint32_t computeEffectiveStallMs(
    bool absenceDetectionEnabled,
    uint32_t rotationStallMs,
    uint32_t triggerLimitMs) {
    return (absenceDetectionEnabled && triggerLimitMs < rotationStallMs)
        ? triggerLimitMs
        : rotationStallMs;
}

SPINDLE_CONSTEXPR SpindleIsrState makeSpindleIsrState(int initialReading, uint32_t nowMs, uint32_t effectiveStallMs) {
    return SpindleIsrState{
        initialReading,
        initialReading,
        nowMs,
        nowMs,
        effectiveStallMs,
        0,
        nowMs,
        0,
        0,
        0,
        0,
        0,
        nowMs,
        nowMs,
        false
    };
}

SPINDLE_CONSTEXPR SpindleRuntimeState makeSpindleRuntimeState(int initialReading, uint32_t nowMs) {
    return SpindleRuntimeState{
        SPINDLE_IDLE,
        SPINDLE_STATE_UNKNOWN,
        initialReading,
        nowMs,
        nowMs,
        0,
        nowMs,
        0,
        0,
        nowMs,
        nowMs,
        0,
        0,
        0,
        false,
        false
    };
}

SPINDLE_ALWAYS_INLINE SPINDLE_CONSTEXPR void recordConfirmedEdgeLocked(
    SpindleIsrState& s,
    uint32_t confirmMs,
    int activeState) {
    uint32_t phaseDuration = confirmMs - s.confirmedStateStartMs;
    if (s.confirmedState == activeState) {
        s.lastLowDwellMs = phaseDuration;
    } else {
        s.lastHighGapMs = phaseDuration;
    }
    s.batchLastPhaseMs = phaseDuration;
    if (phaseDuration >= s.effectiveStallMs) {
        if (!s.batchStallOccurred || s.batchPostStallEdges >= 2) {
            s.batchStallAnchorMs = s.confirmedStateStartMs;
        }
        s.batchStallOccurred = true;
        s.batchPostStallEdges = 1;
        s.batchPostStallFirstEdgeMs = confirmMs;
    } else {
        if (s.batchPostStallEdges == 0) {
            s.batchPostStallFirstEdgeMs = confirmMs;
        }
        s.batchPostStallEdges++;
        if (phaseDuration > s.batchMaxRotatingPhaseMs) {
            s.batchMaxRotatingPhaseMs = phaseDuration;
        }
    }
    s.confirmedState = s.candidateState;
    s.confirmedStateStartMs = confirmMs;
    s.queuedEdges++;
    s.queuedLastEdgeMs = confirmMs;
}

SPINDLE_ALWAYS_INLINE SPINDLE_CONSTEXPR void spindleIsrOnPinSample(
    SpindleIsrState& s,
    uint32_t nowMs,
    int rawLevel,
    int activeState,
    uint32_t debounceMs) {
    if (rawLevel != s.candidateState) {
        if (s.candidateState != s.confirmedState && elapsedAtLeast(nowMs, s.candidateStartMs, debounceMs)) {
            uint32_t confirmMs = s.candidateStartMs + debounceMs;
            recordConfirmedEdgeLocked(s, confirmMs, activeState);
        }
        s.candidateState = rawLevel;
        s.candidateStartMs = nowMs;
    }
}

SPINDLE_CONSTEXPR SpindleEdgeBatch spindleConsumeCycleLocked(
    SpindleIsrState& s,
    uint32_t nowMs,
    int rawReading,
    int activeState,
    uint32_t debounceMs) {
    if (rawReading != s.candidateState) {
        if ((nowMs - s.candidateStartMs) < 0x80000000UL) {
            s.candidateState = rawReading;
            s.candidateStartMs = nowMs;
        }
    }
    if (s.candidateState != s.confirmedState && elapsedAtLeast(nowMs, s.candidateStartMs, debounceMs)) {
        uint32_t confirmMs = s.candidateStartMs + debounceMs;
        recordConfirmedEdgeLocked(s, confirmMs, activeState);
    }
    SpindleEdgeBatch batch{
        s.queuedEdges,
        s.queuedLastEdgeMs,
        s.confirmedState,
        s.lastLowDwellMs,
        s.lastHighGapMs,
        s.batchLastPhaseMs,
        s.batchMaxRotatingPhaseMs,
        s.batchPostStallEdges,
        s.batchPostStallFirstEdgeMs,
        s.batchStallAnchorMs,
        s.batchStallOccurred
    };
    s.queuedEdges = 0;
    s.batchLastPhaseMs = 0;
    s.batchMaxRotatingPhaseMs = 0;
    s.batchPostStallEdges = 0;
    s.batchStallOccurred = false;
    return batch;
}

SPINDLE_CONSTEXPR SpindleStepResult spindleStep(
    SpindleRuntimeState& rt,
    const SpindleEdgeBatch& batch,
    uint32_t nowMs,
    const SpindleThresholdConfig& cfg) {
    SpindleStepResult res{SPINDLE_SYNC_NONE, SPINDLE_EVENT_NONE, 0};
    uint32_t effectiveStallMs = computeEffectiveStallMs(
        cfg.absenceDetectionEnabled,
        cfg.rotationStallMs,
        cfg.triggerLimitMs);

    // 1. Reconcile state immediately if absence detection was disabled while FAULT or RECOVERING
    if (!cfg.absenceDetectionEnabled) {
        if (rt.currentState == SPINDLE_RECOVERING) {
            rt.previousState = rt.currentState;
            rt.currentState = SPINDLE_RUNNING;
            rt.recoveryStartTime = 0;
            rt.stateStartTime = nowMs;
            rt.lastPulseSentTime = nowMs;
            rt.lastEventRetryTime = nowMs;
            rt.stateChangeCount++;
            rt.activitySinceHb = true;
            res.syncAction = SPINDLE_SYNC_STATE;
            res.stepEvent = SPINDLE_EVENT_ABSENCE_DISABLED_CLEARED;
        } else if (rt.currentState == SPINDLE_FAULT) {
            rt.previousState = rt.currentState;
            rt.currentState = SPINDLE_IDLE;
            rt.wasFaulted = false;
            rt.stateStartTime = nowMs;
            rt.lastEventRetryTime = nowMs;
            rt.stateChangeCount++;
            rt.activitySinceHb = false;
            res.syncAction = SPINDLE_SYNC_STATE;
            res.stepEvent = SPINDLE_EVENT_ABSENCE_DISABLED_CLEARED;
        }
    }

    // 2. Process hardware-captured & steady-state debounced ON/OFF transitions
    if (batch.totalEdges > 0) {
        rt.debouncedPinState = batch.confirmedPin;
        rt.pulseCount += batch.totalEdges;
        rt.lastEdgeGapMs = batch.lastPhaseMs;

        // If a stall (>= effectiveStallMs) occurred before or between queued edges during HTTP blocking,
        // apply that stall transition FIRST so recovery resets and single stray edges never sum to Running.
        if (batch.stallOccurred) {
            rt.consecutiveEdges = 0;
            if (rt.currentState == SPINDLE_RECOVERING) {
                res.heldRecoveryMs = (batch.lastEdgeMs - rt.recoveryStartTime) < 0x80000000UL
                    ? (batch.lastEdgeMs - rt.recoveryStartTime)
                    : 0;
                rt.previousState = rt.currentState;
                rt.currentState = SPINDLE_FAULT;
                rt.stateStartTime = nowMs;
                rt.recoveryStartTime = 0;
                rt.stateChangeCount++;
                rt.activitySinceHb = false;
                res.stepEvent = SPINDLE_EVENT_RECOVERY_ABORTED_STALL;
            } else if (rt.currentState == SPINDLE_RUNNING || rt.currentState == SPINDLE_STATE_UNKNOWN) {
                rt.previousState = rt.currentState;
                rt.currentState = SPINDLE_IDLE;
                rt.stateStartTime = nowMs;
                rt.absenceStartTime = batch.stallAnchorMs;
                rt.lastEventRetryTime = nowMs;
                rt.stateChangeCount++;
                rt.activitySinceHb = false;
                res.syncAction = SPINDLE_SYNC_STATE;
                res.stepEvent = SPINDLE_EVENT_STOPPED_TO_IDLE;
            }
            if (cfg.absenceDetectionEnabled
                && rt.currentState == SPINDLE_IDLE
                && elapsedAtLeast(batch.postStallFirstEdgeMs, rt.absenceStartTime, cfg.triggerLimitMs)) {
                rt.previousState = rt.currentState;
                rt.currentState = SPINDLE_FAULT;
                rt.stateStartTime = nowMs;
                rt.lastEventRetryTime = nowMs;
                rt.wasFaulted = true;
                rt.activitySinceHb = false;
                rt.stateChangeCount++;
                res.syncAction = SPINDLE_SYNC_STATE;
                res.stepEvent = SPINDLE_EVENT_TRIGGER_EXCEEDED;
            }
            rt.consecutiveEdges = batch.postStallEdges;
        } else {
            rt.consecutiveEdges += batch.totalEdges;
        }

        if (rt.currentState == SPINDLE_RUNNING && batch.maxRotatingPhaseMs > rt.maxObservedEdgeGapMs) {
            rt.maxObservedEdgeGapMs = batch.maxRotatingPhaseMs;
        }

        rt.lastEdgeTime = batch.lastEdgeMs;

        if (rt.consecutiveEdges >= 2) {
            bool shouldHoldRecovery = cfg.absenceDetectionEnabled
                && (rt.currentState == SPINDLE_FAULT
                    || (rt.wasFaulted && rt.currentState != SPINDLE_RECOVERING && rt.currentState != SPINDLE_RUNNING));
            if (shouldHoldRecovery) {
                rt.previousState = rt.currentState;
                rt.currentState = SPINDLE_RECOVERING;
                rt.stateStartTime = nowMs;
                rt.recoveryStartTime = nowMs;
                rt.stateChangeCount++;
                rt.activitySinceHb = false;
                res.stepEvent = SPINDLE_EVENT_ENTERED_RECOVERING;
            } else if (rt.currentState == SPINDLE_IDLE || rt.currentState == SPINDLE_STATE_UNKNOWN) {
                rt.previousState = rt.currentState;
                rt.currentState = SPINDLE_RUNNING;
                rt.stateStartTime = nowMs;
                rt.lastPulseSentTime = nowMs;
                rt.lastEventRetryTime = nowMs;
                rt.stateChangeCount++;
                rt.activitySinceHb = true;
                if (batch.maxRotatingPhaseMs > rt.maxObservedEdgeGapMs) {
                    rt.maxObservedEdgeGapMs = batch.maxRotatingPhaseMs;
                }
                res.syncAction = SPINDLE_SYNC_STATE;
                res.stepEvent = SPINDLE_EVENT_ENTERED_RUNNING;
            }
        }
    }

    // 3. Detect Spindle Stall since last confirmed edge
    if (elapsedAtLeast(nowMs, rt.lastEdgeTime, effectiveStallMs)) {
        rt.consecutiveEdges = 0;

        if (rt.currentState == SPINDLE_RECOVERING) {
            res.heldRecoveryMs = (nowMs - rt.recoveryStartTime) < 0x80000000UL
                ? (nowMs - rt.recoveryStartTime)
                : 0;
            rt.previousState = rt.currentState;
            rt.currentState = SPINDLE_FAULT;
            rt.stateStartTime = nowMs;
            rt.recoveryStartTime = 0;
            rt.stateChangeCount++;
            rt.activitySinceHb = false;
            res.stepEvent = SPINDLE_EVENT_RECOVERY_ABORTED_STALL;
        } else if (rt.currentState == SPINDLE_RUNNING || rt.currentState == SPINDLE_STATE_UNKNOWN) {
            rt.previousState = rt.currentState;
            rt.currentState = SPINDLE_IDLE;
            rt.stateStartTime = nowMs;
            rt.absenceStartTime = rt.lastEdgeTime;
            rt.lastEventRetryTime = nowMs;
            rt.stateChangeCount++;
            rt.activitySinceHb = false;
            res.syncAction = SPINDLE_SYNC_STATE;
            res.stepEvent = SPINDLE_EVENT_STOPPED_TO_IDLE;
        }
    }

    if (rt.currentState == SPINDLE_RUNNING) {
        rt.activitySinceHb = true;
    }

    // 4. Recovery Limit Confirmation (RECOVERING -> RUNNING past recoveryLimitMs)
    if (rt.currentState == SPINDLE_RECOVERING && elapsedAtLeast(nowMs, rt.recoveryStartTime, cfg.recoveryLimitMs)) {
        rt.previousState = rt.currentState;
        rt.currentState = SPINDLE_RUNNING;
        rt.recoveryStartTime = 0;
        rt.stateStartTime = nowMs;
        rt.lastPulseSentTime = nowMs;
        rt.lastEventRetryTime = nowMs;
        rt.stateChangeCount++;
        rt.activitySinceHb = true;
        res.syncAction = SPINDLE_SYNC_STATE;
        res.stepEvent = SPINDLE_EVENT_RECOVERY_MET;
    }

    // 5. Trigger Limit Escalation (IDLE -> FAULT past triggerLimitMs)
    if (cfg.absenceDetectionEnabled && rt.currentState == SPINDLE_IDLE && elapsedAtLeast(nowMs, rt.absenceStartTime, cfg.triggerLimitMs)) {
        rt.previousState = rt.currentState;
        rt.currentState = SPINDLE_FAULT;
        rt.stateStartTime = nowMs;
        rt.lastEventRetryTime = nowMs;
        rt.wasFaulted = true;
        rt.activitySinceHb = false;
        rt.stateChangeCount++;
        res.syncAction = SPINDLE_SYNC_STATE;
        res.stepEvent = SPINDLE_EVENT_TRIGGER_EXCEEDED;
    }

    return res;
}

SPINDLE_CONSTEXPR char asciiUpperChar(char ch) {
    return (ch >= 'a' && ch <= 'z') ? (char)(ch - ('a' - 'A')) : ch;
}

SPINDLE_CONSTEXPR bool isHorizontalSpace(char ch) {
    return ch == ' ' || ch == '\t';
}

SPINDLE_CONSTEXPR StallCmdParseResult parseStallCommandMs(
    const char* rawCmd,
    uint32_t minStallMs,
    uint32_t maxStallMs,
    int32_t* outMs) {
    if (outMs) *outMs = -1;
    if (!rawCmd) return STALL_CMD_NOT_MATCHED;

    size_t idx = 0;
    while (isHorizontalSpace(rawCmd[idx])) idx++;

    const char prefix[5] = {'S', 'T', 'A', 'L', 'L'};
    for (size_t i = 0; i < 5; i++) {
        if (asciiUpperChar(rawCmd[idx + i]) != prefix[i]) {
            return STALL_CMD_NOT_MATCHED;
        }
    }
    idx += 5;

    if (rawCmd[idx] != '\0' && !isHorizontalSpace(rawCmd[idx]) && rawCmd[idx] != '=') {
        return STALL_CMD_NOT_MATCHED;
    }

    while (isHorizontalSpace(rawCmd[idx])) idx++;
    if (rawCmd[idx] == '=') {
        idx++;
        while (isHorizontalSpace(rawCmd[idx])) idx++;
    }

    size_t endIdx = idx;
    while (rawCmd[endIdx] != '\0') endIdx++;
    while (endIdx > idx && isHorizontalSpace(rawCmd[endIdx - 1])) endIdx--;

    if (endIdx == idx) {
        return STALL_CMD_INVALID;
    }

    uint64_t accum = 0;
    for (size_t p = idx; p < endIdx; p++) {
        char ch = rawCmd[p];
        if (ch < '0' || ch > '9') {
            return STALL_CMD_INVALID;
        }
        accum = (accum * 10ULL) + (uint64_t)(ch - '0');
        if (accum > 2147483647ULL) {
            return STALL_CMD_INVALID;
        }
    }

    int32_t value = (int32_t)accum;
    if (outMs) *outMs = value;
    if (value < (int32_t)minStallMs || value > (int32_t)maxStallMs) {
        return STALL_CMD_INVALID;
    }
    return STALL_CMD_VALID;
}

SPINDLE_CONSTEXPR SpindleThresholdConfig sanitizeBootThresholds(
    uint32_t storedStallMs,
    uint32_t storedTriggerMs,
    uint32_t storedRecoveryMs,
    bool storedAbsenceEnabled,
    uint32_t minStallMs,
    uint32_t maxStallMs,
    uint32_t defaultStallMs,
    uint32_t defaultTriggerMs,
    uint32_t defaultRecoveryMs) {
    uint32_t stall = (storedStallMs >= minStallMs && storedStallMs <= maxStallMs)
        ? storedStallMs
        : defaultStallMs;
    uint32_t trig = (storedTriggerMs >= 1000UL && storedTriggerMs <= 3600000UL)
        ? storedTriggerMs
        : defaultTriggerMs;
    uint32_t rec = (storedRecoveryMs >= 1000UL && storedRecoveryMs <= 300000UL)
        ? storedRecoveryMs
        : defaultRecoveryMs;
    return SpindleThresholdConfig{stall, trig, rec, storedAbsenceEnabled, false};
}

SPINDLE_CONSTEXPR ThresholdUpdateOutcome applyThresholdUpdateLogic(
    SpindleThresholdConfig& cfg,
    SpindleRuntimeState* rt,
    uint32_t nowMs,
    int32_t triggerSec,
    int32_t recoverySec,
    int8_t absenceEnabled,
    int32_t stallMs,
    bool fromSerial,
    bool versionChanged,
    bool backendVersionAdvanced,
    uint32_t minStallMs,
    uint32_t maxStallMs) {
    bool thresholdsChanged = false;

    if (!fromSerial && backendVersionAdvanced) {
        cfg.serialStallOverride = false;
    }

    if (absenceEnabled == 0 || absenceEnabled == 1) {
        bool enabledBool = (absenceEnabled == 1);
        if (cfg.absenceDetectionEnabled != enabledBool) {
            bool enablingNow = (!cfg.absenceDetectionEnabled && enabledBool);
            cfg.absenceDetectionEnabled = enabledBool;
            thresholdsChanged = true;
            if (enablingNow && rt && rt->currentState == SPINDLE_IDLE) {
                rt->absenceStartTime = nowMs;
            }
        }
    }

    if (triggerSec >= 1 && triggerSec <= 3600) {
        uint32_t newTriggerMs = (uint32_t)triggerSec * 1000UL;
        if (cfg.triggerLimitMs != newTriggerMs) {
            cfg.triggerLimitMs = newTriggerMs;
            thresholdsChanged = true;
        }
    }

    if (recoverySec >= 1 && recoverySec <= 300) {
        uint32_t newRecoveryMs = (uint32_t)recoverySec * 1000UL;
        if (cfg.recoveryLimitMs != newRecoveryMs) {
            cfg.recoveryLimitMs = newRecoveryMs;
            thresholdsChanged = true;
        }
    }

    if (stallMs >= (int32_t)minStallMs && stallMs <= (int32_t)maxStallMs) {
        if (fromSerial || !cfg.serialStallOverride) {
            if (fromSerial) {
                cfg.serialStallOverride = true;
            }
            uint32_t newStallMs = (uint32_t)stallMs;
            if (cfg.rotationStallMs != newStallMs) {
                cfg.rotationStallMs = newStallMs;
                thresholdsChanged = true;
            }
        }
    }

    return ThresholdUpdateOutcome{thresholdsChanged || versionChanged, thresholdsChanged};
}
