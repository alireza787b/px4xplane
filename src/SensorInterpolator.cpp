#include "SensorInterpolator.h"
#include <cmath>
#include <algorithm>

SensorInterpolator::SensorInterpolator(float gyro_process_noise, float accel_process_noise)
    : gyro_process_noise_(gyro_process_noise),
      accel_process_noise_(accel_process_noise),
      last_update_time_(0),
      initialized_(false) {
}

void SensorInterpolator::update(const SensorBuffer::IMUSample& measurement) {
    uint64_t current_time = measurement.timestamp_usec;

    if (!initialized_) {
        // First measurement - initialize states
        gyro_x_.value = measurement.gyro[0];
        gyro_y_.value = measurement.gyro[1];
        gyro_z_.value = measurement.gyro[2];

        accel_x_.value = measurement.accel[0];
        accel_y_.value = measurement.accel[1];
        accel_z_.value = measurement.accel[2];

        // Velocities start at zero
        gyro_x_.velocity = 0.0f;
        gyro_y_.velocity = 0.0f;
        gyro_z_.velocity = 0.0f;

        accel_x_.velocity = 0.0f;
        accel_y_.velocity = 0.0f;
        accel_z_.velocity = 0.0f;

        last_update_time_ = current_time;
        initialized_ = true;
        return;
    }

    // Calculate time delta
    float dt_sec = (current_time - last_update_time_) * 1e-6f;

    // Clamp to reasonable range (prevent issues with time jumps)
    if (dt_sec < 0.001f) dt_sec = 0.001f;    // Min 1ms
    if (dt_sec > 0.1f) dt_sec = 0.1f;        // Max 100ms

    // Prediction step for all axes
    predictAxis(gyro_x_, dt_sec, gyro_process_noise_);
    predictAxis(gyro_y_, dt_sec, gyro_process_noise_);
    predictAxis(gyro_z_, dt_sec, gyro_process_noise_);

    predictAxis(accel_x_, dt_sec, accel_process_noise_);
    predictAxis(accel_y_, dt_sec, accel_process_noise_);
    predictAxis(accel_z_, dt_sec, accel_process_noise_);

    // Measurement update step for all axes
    updateAxis(gyro_x_, measurement.gyro[0], GYRO_MEAS_NOISE);
    updateAxis(gyro_y_, measurement.gyro[1], GYRO_MEAS_NOISE);
    updateAxis(gyro_z_, measurement.gyro[2], GYRO_MEAS_NOISE);

    updateAxis(accel_x_, measurement.accel[0], ACCEL_MEAS_NOISE);
    updateAxis(accel_y_, measurement.accel[1], ACCEL_MEAS_NOISE);
    updateAxis(accel_z_, measurement.accel[2], ACCEL_MEAS_NOISE);

    last_update_time_ = current_time;
}

SensorBuffer::IMUSample SensorInterpolator::predict(uint64_t target_time_usec) {
    SensorBuffer::IMUSample result;

    if (!initialized_) {
        // No data yet, return zero sample
        return result;
    }

    // Calculate time delta from last update
    float dt_sec = (target_time_usec - last_update_time_) * 1e-6f;

    // Clamp prediction time
    if (dt_sec < 0.0f) dt_sec = 0.0f;        // Don't predict backwards
    if (dt_sec > 0.1f) dt_sec = 0.1f;        // Max 100ms extrapolation

    // Predict using current velocity estimates (constant-velocity model)
    result.timestamp_usec = target_time_usec;

    result.gyro[0] = gyro_x_.value + gyro_x_.velocity * dt_sec;
    result.gyro[1] = gyro_y_.value + gyro_y_.velocity * dt_sec;
    result.gyro[2] = gyro_z_.value + gyro_z_.velocity * dt_sec;

    result.accel[0] = accel_x_.value + accel_x_.velocity * dt_sec;
    result.accel[1] = accel_y_.value + accel_y_.velocity * dt_sec;
    result.accel[2] = accel_z_.value + accel_z_.velocity * dt_sec;

    // Note: Other fields (mag, pressure, etc.) not predicted by Kalman
    // These will be filled by simple linear interpolation or last-known-value
    // in the SensorPublisher

    return result;
}

SensorBuffer::IMUSample SensorInterpolator::interpolate(
    const std::vector<SensorBuffer::IMUSample>& samples,
    uint64_t target_time_usec
) {
    SensorBuffer::IMUSample result;

    if (samples.size() < 2) {
        // Not enough samples for interpolation, use prediction if we have one sample
        if (samples.size() == 1) {
            return samples[0];  // Return most recent sample
        }
        return result;  // Empty sample
    }

    // samples are in reverse chronological order: [newest, older, ...]
    const SensorBuffer::IMUSample& newer = samples[0];
    const SensorBuffer::IMUSample& older = samples[1];

    uint64_t t_newer = newer.timestamp_usec;
    uint64_t t_older = older.timestamp_usec;

    // Handle edge cases
    if (target_time_usec >= t_newer) {
        // Target is newer than our newest sample - extrapolate
        return predict(target_time_usec);
    }

    if (target_time_usec <= t_older) {
        // Target is older than our samples - return older sample
        return older;
    }

    // Linear interpolation between two samples
    result.timestamp_usec = target_time_usec;

    // Interpolate gyro
    result.gyro[0] = lerp(older.gyro[0], newer.gyro[0], t_older, t_newer, target_time_usec);
    result.gyro[1] = lerp(older.gyro[1], newer.gyro[1], t_older, t_newer, target_time_usec);
    result.gyro[2] = lerp(older.gyro[2], newer.gyro[2], t_older, t_newer, target_time_usec);

    // Interpolate accel
    result.accel[0] = lerp(older.accel[0], newer.accel[0], t_older, t_newer, target_time_usec);
    result.accel[1] = lerp(older.accel[1], newer.accel[1], t_older, t_newer, target_time_usec);
    result.accel[2] = lerp(older.accel[2], newer.accel[2], t_older, t_newer, target_time_usec);

    // Interpolate mag (linear is fine for slowly-varying field)
    result.mag[0] = lerp(older.mag[0], newer.mag[0], t_older, t_newer, target_time_usec);
    result.mag[1] = lerp(older.mag[1], newer.mag[1], t_older, t_newer, target_time_usec);
    result.mag[2] = lerp(older.mag[2], newer.mag[2], t_older, t_newer, target_time_usec);

    // Interpolate pressure (linear interpolation of pressure, not altitude)
    result.pressure_hPa = lerp(older.pressure_hPa, newer.pressure_hPa, t_older, t_newer, target_time_usec);

    // Interpolate temperature
    result.temperature_C = lerp(older.temperature_C, newer.temperature_C, t_older, t_newer, target_time_usec);

    // Quaternion interpolation (SLERP would be better, but linear is acceptable for small dt)
    result.attitude_q[0] = lerp(older.attitude_q[0], newer.attitude_q[0], t_older, t_newer, target_time_usec);
    result.attitude_q[1] = lerp(older.attitude_q[1], newer.attitude_q[1], t_older, t_newer, target_time_usec);
    result.attitude_q[2] = lerp(older.attitude_q[2], newer.attitude_q[2], t_older, t_newer, target_time_usec);
    result.attitude_q[3] = lerp(older.attitude_q[3], newer.attitude_q[3], t_older, t_newer, target_time_usec);

    // Normalize quaternion (important after linear interpolation)
    float quat_norm = sqrtf(
        result.attitude_q[0] * result.attitude_q[0] +
        result.attitude_q[1] * result.attitude_q[1] +
        result.attitude_q[2] * result.attitude_q[2] +
        result.attitude_q[3] * result.attitude_q[3]
    );

    if (quat_norm > 0.001f) {  // Avoid division by zero
        result.attitude_q[0] /= quat_norm;
        result.attitude_q[1] /= quat_norm;
        result.attitude_q[2] /= quat_norm;
        result.attitude_q[3] /= quat_norm;
    } else {
        // Invalid quaternion, use identity
        result.attitude_q[0] = 1.0f;
        result.attitude_q[1] = 0.0f;
        result.attitude_q[2] = 0.0f;
        result.attitude_q[3] = 0.0f;
    }

    return result;
}

bool SensorInterpolator::canExtrapolate(uint64_t target_time_usec, uint64_t max_delta_us) const {
    if (!initialized_) {
        return false;
    }

    uint64_t time_delta = (target_time_usec > last_update_time_)
                          ? (target_time_usec - last_update_time_)
                          : 0;

    return time_delta <= max_delta_us;
}

void SensorInterpolator::reset() {
    initialized_ = false;
    last_update_time_ = 0;

    // Reset all axis states
    gyro_x_ = AxisState();
    gyro_y_ = AxisState();
    gyro_z_ = AxisState();

    accel_x_ = AxisState();
    accel_y_ = AxisState();
    accel_z_ = AxisState();
}

// Private helper methods

void SensorInterpolator::predictAxis(AxisState& state, float dt_sec, float process_noise) {
    // State transition: x = [value, velocity]
    // F = [1  dt]
    //     [0   1]
    //
    // x_pred = F * x
    float value_pred = state.value + state.velocity * dt_sec;
    float velocity_pred = state.velocity;  // Constant velocity model

    // Covariance prediction: P_pred = F * P * F' + Q
    // Q = process_noise matrix (simplified: diagonal)
    float q_value = process_noise * dt_sec * dt_sec * 0.25f;  // Variance in position
    float q_velocity = process_noise * dt_sec;                 // Variance in velocity

    float p00 = state.cov[0][0] + 2.0f * state.cov[0][1] * dt_sec + state.cov[1][1] * dt_sec * dt_sec + q_value;
    float p01 = state.cov[0][1] + state.cov[1][1] * dt_sec;
    float p10 = p01;  // Symmetric
    float p11 = state.cov[1][1] + q_velocity;

    // Update state
    state.value = value_pred;
    state.velocity = velocity_pred;

    state.cov[0][0] = p00;
    state.cov[0][1] = p01;
    state.cov[1][0] = p10;
    state.cov[1][1] = p11;
}

void SensorInterpolator::updateAxis(AxisState& state, float measurement, float meas_noise) {
    // Measurement model: z = H * x, where H = [1 0] (measure position only)
    //
    // Innovation: y = z - H * x_pred
    float innovation = measurement - state.value;

    // Innovation covariance: S = H * P * H' + R
    float S = state.cov[0][0] + meas_noise;

    // Kalman gain: K = P * H' / S
    float K0 = state.cov[0][0] / S;
    float K1 = state.cov[1][0] / S;

    // State update: x = x_pred + K * y
    state.value += K0 * innovation;
    state.velocity += K1 * innovation;

    // Covariance update: P = (I - K * H) * P
    // Simplified Joseph form for numerical stability
    float p00_new = (1.0f - K0) * state.cov[0][0];
    float p01_new = (1.0f - K0) * state.cov[0][1];
    float p10_new = state.cov[1][0] - K1 * state.cov[0][0];
    float p11_new = state.cov[1][1] - K1 * state.cov[0][1];

    state.cov[0][0] = p00_new;
    state.cov[0][1] = p01_new;
    state.cov[1][0] = p10_new;
    state.cov[1][1] = p11_new;
}

float SensorInterpolator::lerp(float v0, float v1, uint64_t t0, uint64_t t1, uint64_t t) const {
    if (t1 == t0) {
        // Avoid division by zero
        return v1;
    }

    // Linear interpolation: v = v0 + (v1 - v0) * (t - t0) / (t1 - t0)
    float alpha = static_cast<float>(t - t0) / static_cast<float>(t1 - t0);

    // Clamp alpha to [0, 1]
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;

    return v0 + (v1 - v0) * alpha;
}
