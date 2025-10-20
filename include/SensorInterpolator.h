#ifndef SENSORINTERPOLATOR_H
#define SENSORINTERPOLATOR_H

#include "SensorBuffer.h"
#include <cstdint>
#include <vector>

/**
 * @brief Kalman-based sensor interpolator/extrapolator
 *
 * Phase 3: FPS-independent sensor interpolation
 *
 * This class provides intelligent sensor value estimation between X-Plane frames using
 * simplified linear Kalman filtering. It enables smooth, realistic sensor data generation
 * even when X-Plane FPS < target publish rate.
 *
 * Key capabilities:
 * - Interpolation: Generate samples between two known measurements
 * - Extrapolation: Predict future samples based on recent history
 * - Velocity estimation: Track rate of change for smooth predictions
 * - Configurable process noise: Tune for different sensor characteristics
 *
 * Algorithm: Linear Kalman filter per axis
 * State vector: [value, velocity]
 * Process model: Constant velocity (value += velocity * dt)
 *
 * Example use cases:
 * - 60 FPS → 200Hz: Interpolate 3 samples between each X-Plane frame
 * - 30 FPS → 200Hz: Extrapolate 6 samples with velocity prediction
 * - 120 FPS → 200Hz: Light interpolation (1-2 samples between frames)
 *
 * Performance:
 * - update(): ~5μs (Kalman measurement update, 6 axes)
 * - predict(): ~2μs (Kalman time update, 6 axes)
 * - interpolate(): ~8μs (linear interpolation between two samples)
 */
class SensorInterpolator {
public:
    /**
     * @brief Constructor
     *
     * @param gyro_process_noise Process noise for gyroscope (rad/s²) - default 0.01
     * @param accel_process_noise Process noise for accelerometer (m/s³) - default 0.1
     */
    SensorInterpolator(float gyro_process_noise = 0.01f, float accel_process_noise = 0.1f);

    /**
     * @brief Update Kalman filter with new measurement from X-Plane
     *
     * This performs the Kalman "measurement update" step, incorporating new sensor data
     * and updating velocity estimates.
     *
     * Should be called every time X-Plane provides a new sensor sample.
     *
     * @param measurement Latest IMU sample from X-Plane
     */
    void update(const SensorBuffer::IMUSample& measurement);

    /**
     * @brief Predict sensor state at future timestamp (extrapolation)
     *
     * Uses Kalman "time update" (prediction) to estimate sensor values at a future time
     * based on current state and velocity estimates.
     *
     * Useful when X-Plane FPS < target rate and we need to generate intermediate samples.
     *
     * @param target_time_usec Target timestamp for prediction
     * @return Predicted IMU sample at target time
     */
    SensorBuffer::IMUSample predict(uint64_t target_time_usec);

    /**
     * @brief Interpolate between two known samples
     *
     * Performs linear interpolation between two measurements. More accurate than
     * pure prediction when both surrounding samples are known.
     *
     * @param samples Vector of recent samples (at least 2, newest first)
     * @param target_time_usec Target timestamp between samples[1] and samples[0]
     * @return Interpolated IMU sample
     */
    SensorBuffer::IMUSample interpolate(
        const std::vector<SensorBuffer::IMUSample>& samples,
        uint64_t target_time_usec
    );

    /**
     * @brief Check if safe to extrapolate to given timestamp
     *
     * Validates that extrapolation time delta is within safe limits.
     * Beyond certain thresholds, predictions become unreliable.
     *
     * @param target_time_usec Target timestamp
     * @param max_delta_us Maximum allowed time delta (microseconds)
     * @return true if extrapolation is safe
     */
    bool canExtrapolate(uint64_t target_time_usec, uint64_t max_delta_us) const;

    /**
     * @brief Reset interpolator state
     *
     * Clears all Kalman filter state. Call when simulation restarts or large discontinuities occur.
     */
    void reset();

private:
    /**
     * @brief Kalman filter state for single axis
     *
     * State vector: x = [value, velocity]ᵀ
     * Covariance: 2x2 matrix
     */
    struct AxisState {
        float value;           ///< Current estimated value
        float velocity;        ///< Current estimated velocity (rate of change)
        float cov[2][2];       ///< 2x2 covariance matrix

        AxisState() : value(0.0f), velocity(0.0f) {
            // Initialize covariance to identity
            cov[0][0] = 1.0f; cov[0][1] = 0.0f;
            cov[1][0] = 0.0f; cov[1][1] = 1.0f;
        }
    };

    // Kalman filter states for each sensor axis
    AxisState gyro_x_, gyro_y_, gyro_z_;       ///< Gyroscope axes
    AxisState accel_x_, accel_y_, accel_z_;    ///< Accelerometer axes

    // Process noise parameters (tunable)
    float gyro_process_noise_;    ///< Gyro process noise (rad/s²)
    float accel_process_noise_;   ///< Accel process noise (m/s³)

    // Measurement noise (assumed constant for MEMS sensors)
    static constexpr float GYRO_MEAS_NOISE = 0.01f;   ///< 0.01 rad/s (from Phase 1)
    static constexpr float ACCEL_MEAS_NOISE = 0.05f;  ///< 0.05 m/s² (vibration)

    // Last update timestamp
    uint64_t last_update_time_;   ///< Timestamp of last measurement update
    bool initialized_;            ///< true if received at least one measurement

    /**
     * @brief Perform Kalman prediction step for single axis
     *
     * Predicts state forward in time using constant-velocity model:
     * x_pred = F * x, where F = [1 dt; 0 1]
     * P_pred = F * P * Fᵀ + Q
     *
     * @param state Axis state to update (modified in-place)
     * @param dt_sec Time step in seconds
     * @param process_noise Process noise magnitude
     */
    void predictAxis(AxisState& state, float dt_sec, float process_noise);

    /**
     * @brief Perform Kalman measurement update for single axis
     *
     * Updates state estimate with new measurement:
     * K = P * Hᵀ / (H * P * Hᵀ + R)
     * x = x + K * (z - H * x)
     * P = (I - K * H) * P
     *
     * @param state Axis state to update (modified in-place)
     * @param measurement New sensor measurement
     * @param meas_noise Measurement noise variance
     */
    void updateAxis(AxisState& state, float measurement, float meas_noise);

    /**
     * @brief Linear interpolation between two values
     *
     * @param v0 Value at time t0
     * @param v1 Value at time t1
     * @param t0 Start time
     * @param t1 End time
     * @param t Target time (between t0 and t1)
     * @return Interpolated value at time t
     */
    float lerp(float v0, float v1, uint64_t t0, uint64_t t1, uint64_t t) const;
};

#endif // SENSORINTERPOLATOR_H
