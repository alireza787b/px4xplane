/**
 * @file TimestampProvider.cpp
 * @brief High-precision timestamp generation for PX4 HIL messages.
 *
 * Fixes EKF2 time_slip issues caused by float precision loss when converting
 * X-Plane's total_flight_time_sec (32-bit float) to microseconds.
 *
 * @see TimestampProvider.h for detailed documentation
 */

#include "TimestampProvider.h"
#include "DataRefManager.h"
#include "ConfigManager.h"
#include "XPLMUtilities.h"
#include <cstdio>
#include <cmath>

// Static member initialization
bool TimestampProvider::s_initialized = false;
TimestampProvider::TimePoint TimestampProvider::s_baseTimePoint;
double TimestampProvider::s_lastXPlaneTimeSec = 0.0;
uint64_t TimestampProvider::s_accumulatedDeltaUsec = 0;
uint64_t TimestampProvider::s_lastOutputUsec = 0;
int64_t TimestampProvider::s_driftUsec = 0;
uint64_t TimestampProvider::s_lastDeltaUsec = 0;

uint64_t TimestampProvider::getTimestampUsec() {
    // Get current X-Plane simulation time
    // NOTE: This is a float dataref, but we only use it for DELTA calculation.
    // Small deltas (frame-to-frame) preserve float precision even at large absolute values.
    double currentXPlaneTimeSec = static_cast<double>(
        DataRefManager::getFloat("sim/time/total_flight_time_sec")
    );

    // First call initialization
    if (!s_initialized) {
        s_baseTimePoint = SteadyClock::now();
        s_lastXPlaneTimeSec = currentXPlaneTimeSec;
        s_accumulatedDeltaUsec = 0;
        s_lastOutputUsec = 0;
        s_driftUsec = 0;
        s_lastDeltaUsec = 0;
        s_initialized = true;

        if (ConfigManager::debug_log_sensor_timing) {
            char buf[256];
            snprintf(buf, sizeof(buf),
                "px4xplane: [TIMESTAMP] TimestampProvider initialized at X-Plane time %.3f sec\n",
                currentXPlaneTimeSec);
            XPLMDebugString(buf);
        }

        // Return 0 for first timestamp - clean start for PX4 lockstep
        return 0;
    }

    // Calculate delta from last frame
    // KEY INSIGHT: Small delta values preserve float precision!
    // At 100 FPS, delta is ~0.01 seconds = 10,000 microseconds (5 digits, well within float precision)
    double deltaSec = currentXPlaneTimeSec - s_lastXPlaneTimeSec;

    // Handle time going backwards (simulation restart, scenario change)
    if (deltaSec < 0) {
        if (ConfigManager::debug_log_sensor_timing) {
            char buf[256];
            snprintf(buf, sizeof(buf),
                "px4xplane: [TIMESTAMP] Time went backwards (delta=%.6f sec), resetting\n",
                deltaSec);
            XPLMDebugString(buf);
        }
        // Reset and recurse - will return 0 and start fresh
        reset();
        return getTimestampUsec();
    }

    // Handle pause or large frame skip
    // Cap to MAX_DELTA_SEC to prevent timestamp jumps that confuse EKF2
    if (deltaSec > MAX_DELTA_SEC) {
        if (ConfigManager::debug_log_sensor_timing) {
            char buf[256];
            snprintf(buf, sizeof(buf),
                "px4xplane: [TIMESTAMP] Large delta capped: %.3f -> %.3f sec (pause/skip detected)\n",
                deltaSec, MAX_DELTA_SEC);
            XPLMDebugString(buf);
        }
        deltaSec = MAX_DELTA_SEC;
    }

    // Handle very small deltas (multiple calls within same frame)
    // Use system clock for sub-frame microsecond precision
    if (deltaSec < MIN_DELTA_SEC) {
        auto now = SteadyClock::now();
        auto systemElapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            now - s_baseTimePoint
        ).count();

        // Use system elapsed time, but ensure it's at least as large as accumulated
        // This handles the case where we're called multiple times per frame
        uint64_t timestamp = static_cast<uint64_t>(systemElapsed);

        // Ensure monotonicity - never return a value <= last output
        if (timestamp <= s_lastOutputUsec) {
            timestamp = s_lastOutputUsec + 1;
        }
        s_lastOutputUsec = timestamp;

        return timestamp;
    }

    // Normal delta processing - accumulate with full 64-bit precision
    uint64_t deltaUsec = static_cast<uint64_t>(deltaSec * 1e6);
    s_accumulatedDeltaUsec += deltaUsec;
    s_lastXPlaneTimeSec = currentXPlaneTimeSec;
    s_lastDeltaUsec = deltaUsec;

    // Ensure strict monotonicity
    if (s_accumulatedDeltaUsec <= s_lastOutputUsec) {
        s_accumulatedDeltaUsec = s_lastOutputUsec + 1;
    }
    s_lastOutputUsec = s_accumulatedDeltaUsec;

    // Periodic drift logging (diagnostic only - does not affect timestamps)
    static uint64_t lastDriftLog = 0;
    if (s_accumulatedDeltaUsec - lastDriftLog > DRIFT_LOG_INTERVAL_USEC) {
        auto now = SteadyClock::now();
        auto systemElapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            now - s_baseTimePoint
        ).count();
        s_driftUsec = static_cast<int64_t>(s_accumulatedDeltaUsec) - systemElapsed;
        lastDriftLog = s_accumulatedDeltaUsec;

        if (ConfigManager::debug_log_sensor_timing) {
            char buf[256];
            snprintf(buf, sizeof(buf),
                "px4xplane: [TIMESTAMP] t=%.1fs, drift=%+.3fms, avg_delta=%.2fms\n",
                s_accumulatedDeltaUsec / 1e6,
                s_driftUsec / 1000.0,
                s_lastDeltaUsec / 1000.0);
            XPLMDebugString(buf);
        }
    }

    return s_accumulatedDeltaUsec;
}

void TimestampProvider::reset() {
    s_initialized = false;
    s_accumulatedDeltaUsec = 0;
    s_lastOutputUsec = 0;
    s_lastXPlaneTimeSec = 0.0;
    s_driftUsec = 0;
    s_lastDeltaUsec = 0;

    if (ConfigManager::debug_log_sensor_timing) {
        XPLMDebugString("px4xplane: [TIMESTAMP] TimestampProvider reset - next call starts from 0\n");
    }
}

void TimestampProvider::getDiagnostics(int64_t& out_drift_usec, uint64_t& out_last_delta_usec) {
    out_drift_usec = s_driftUsec;
    out_last_delta_usec = s_lastDeltaUsec;
}
