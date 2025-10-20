#ifndef FPSMONITOR_H
#define FPSMONITOR_H

#include <array>
#include <cstdint>

/**
 * @brief FPS monitoring and adaptive warning system
 *
 * Phase 3: FPS-aware performance monitoring
 *
 * Tracks X-Plane frame rate and provides intelligent warnings when FPS is insufficient
 * for configured sensor rates. Helps users optimize their setup for stable EKF2 performance.
 *
 * Features:
 * - Rolling average FPS calculation over configurable window
 * - Detection of FPS insufficient for target sensor rates
 * - Optimal sensor rate suggestions based on current performance
 * - Interpolation usage statistics
 *
 * Usage:
 * - Call recordFrame() every X-Plane flight loop callback
 * - Periodically check isFPSTooLow() to warn user
 * - Use suggestOptimalSensorRate() for auto-tuning recommendations
 */
class FPSMonitor {
public:
    /**
     * @brief Constructor
     *
     * @param window_size Number of frames to average (default 100 = ~1-3 seconds)
     */
    FPSMonitor(size_t window_size = 100);

    /**
     * @brief Record a new frame timestamp
     *
     * Call this every X-Plane flight loop callback to track frame timing.
     *
     * @param timestamp_usec Frame timestamp in microseconds
     */
    void recordFrame(uint64_t timestamp_usec);

    /**
     * @brief Get average FPS over recent window
     *
     * @return Average frames per second (0 if insufficient data)
     */
    float getAverageFPS() const;

    /**
     * @brief Get minimum FPS over recent window
     *
     * Useful for detecting FPS drops and stutters.
     *
     * @return Minimum FPS in window
     */
    float getMinimumFPS() const;

    /**
     * @brief Get maximum FPS over recent window
     *
     * @return Maximum FPS in window
     */
    float getMaximumFPS() const;

    /**
     * @brief Check if FPS is too low for target sensor rate
     *
     * Rule of thumb: FPS should be at least 50% of target sensor rate for stable operation
     * (allows 2:1 interpolation ratio).
     *
     * @param target_sensor_rate_hz Desired sensor output rate
     * @return true if FPS is insufficient (below 50% of target)
     */
    bool isFPSTooLow(float target_sensor_rate_hz) const;

    /**
     * @brief Suggest optimal sensor rate for current FPS
     *
     * Returns a sensor rate that should work well with current X-Plane performance.
     * - At high FPS (>150): Suggests 200-250Hz
     * - At medium FPS (60-150): Suggests 100-150Hz
     * - At low FPS (<60): Suggests 50-100Hz
     *
     * @return Recommended sensor rate in Hz
     */
    float suggestOptimalSensorRate() const;

    /**
     * @brief Get current frame count
     *
     * @return Total number of frames recorded since startup
     */
    uint64_t getFrameCount() const {
        return frame_count_;
    }

    /**
     * @brief Reset statistics
     *
     * Clear all recorded data. Call when simulation restarts.
     */
    void reset();

private:
    size_t window_size_;                            ///< Number of frames to average
    std::vector<uint64_t> frame_times_;             ///< Circular buffer of frame timestamps
    size_t index_;                                  ///< Current write position in buffer
    uint64_t frame_count_;                          ///< Total frames recorded
    uint64_t last_frame_time_;                      ///< Previous frame timestamp

    /**
     * @brief Calculate FPS from frame time delta
     *
     * @param delta_usec Time between frames in microseconds
     * @return Frames per second
     */
    float deltaToFPS(uint64_t delta_usec) const;
};

#endif // FPSMONITOR_H
