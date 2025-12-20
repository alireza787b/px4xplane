#pragma once
#ifndef TIMESTAMPPROVIDER_H
#define TIMESTAMPPROVIDER_H

#include <cstdint>
#include <chrono>

/**
 * @brief Provides high-precision monotonic timestamps for HIL MAVLink messages.
 *
 * PROBLEM SOLVED:
 * X-Plane's total_flight_time_sec is a 32-bit float with only ~7 significant digits.
 * At 53.8 seconds (53,800,000 microseconds = 8 digits), precision is lost.
 * Beyond ~16.7 seconds (2^24 microseconds), timestamps become quantized,
 * causing EKF2 time_slip accumulation and fusion failures.
 *
 * SOLUTION:
 * Track time using X-Plane DELTA times (small values preserve float precision),
 * accumulated into a 64-bit integer base. This provides microsecond precision
 * for multi-hour flights while staying synchronized with X-Plane simulation time.
 *
 * USAGE:
 *   uint64_t timestamp = TimestampProvider::getTimestampUsec();
 *   // Use in HIL_SENSOR, HIL_GPS, HIL_STATE_QUATERNION, HIL_RC_INPUTS
 *
 * Thread Safety: NOT thread-safe. All calls must be from X-Plane flight loop thread.
 *
 * @see https://github.com/PX4/PX4-Autopilot/issues/13968 (EKF2 timestamp issue)
 */
class TimestampProvider {
public:
    /**
     * @brief Get current timestamp in microseconds for HIL messages.
     *
     * Returns a monotonically increasing timestamp that progresses at
     * simulation time rate, suitable for PX4 lockstep synchronization.
     * Starts from 0 on first call after initialization/reset.
     *
     * @return uint64_t Timestamp in microseconds since connection start
     */
    static uint64_t getTimestampUsec();

    /**
     * @brief Reset all timing state.
     *
     * Call this on disconnect/reconnect to ensure clean timestamp
     * generation on next connection. Next getTimestampUsec() will return 0.
     */
    static void reset();

    /**
     * @brief Get diagnostic information about timestamp state.
     *
     * @param out_drift_usec Estimated drift between accumulated time and system time
     * @param out_last_delta_usec Last frame delta in microseconds
     */
    static void getDiagnostics(int64_t& out_drift_usec, uint64_t& out_last_delta_usec);

private:
    // High-resolution clock for sub-frame timing and drift detection
    using SteadyClock = std::chrono::steady_clock;
    using TimePoint = std::chrono::time_point<SteadyClock>;

    // State variables (static, function-scope would require reset coordination)
    static bool s_initialized;
    static TimePoint s_baseTimePoint;           // System time at initialization
    static double s_lastXPlaneTimeSec;          // Last X-Plane time for delta calculation
    static uint64_t s_accumulatedDeltaUsec;     // Accumulated microseconds from deltas
    static uint64_t s_lastOutputUsec;           // Last returned timestamp (monotonicity)

    // Diagnostics
    static int64_t s_driftUsec;                 // Drift from system clock (diagnostic only)
    static uint64_t s_lastDeltaUsec;            // Last frame delta (diagnostic only)

    // Timing constraints
    static constexpr double MAX_DELTA_SEC = 0.1;        // Max valid delta (100ms) - cap large gaps
    static constexpr double MIN_DELTA_SEC = 0.0001;     // Min valid delta (100us) - use system clock below this
    static constexpr uint64_t DRIFT_LOG_INTERVAL_USEC = 10000000;  // Log drift every 10 seconds
};

#endif // TIMESTAMPPROVIDER_H
