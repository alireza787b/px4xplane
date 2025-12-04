// AccelCalibration.cpp
// Runtime accelerometer calibration for X-Plane SITL
//
// Fixes "High Accelerometer Bias" error caused by X-Plane aircraft models
// reporting g_nrml != 1.0 when stationary on ground.
//
// VERSION 2 (January 2025): Uses MAGNITUDE SCALING instead of offset subtraction
// This fixes the flight bias error where v1 offset approach caused 17.8% error
// during dynamic flight because the static offset doesn't rotate with attitude.

#include "AccelCalibration.h"
#include "DataRefManager.h"
#include "ConfigManager.h"
#include "XPLMUtilities.h"
#include <cstdio>
#include <cmath>

// Static member initialization
bool AccelCalibration::initialized_ = false;
bool AccelCalibration::calibrated_ = false;
bool AccelCalibration::autoCalibrate_ = true;
Eigen::Vector3f AccelCalibration::gravityOffset_ = Eigen::Vector3f::Zero();
Eigen::Vector3f AccelCalibration::manualOffset_ = Eigen::Vector3f::Zero();
float AccelCalibration::measuredGravityMagnitude_ = 0.0f;
float AccelCalibration::gravityScaleFactor_ = 1.0f;  // NEW: Default scale factor (no scaling)
std::deque<Eigen::Vector3f> AccelCalibration::calibrationSamples_;
int AccelCalibration::stationaryCount_ = 0;
int AccelCalibration::sampleCount_ = 0;

void AccelCalibration::initialize() {
    initialized_ = true;
    reset();

    // Load configuration
    autoCalibrate_ = ConfigManager::accel_auto_calibrate;
    manualOffset_ = Eigen::Vector3f(
        ConfigManager::accel_offset_x,
        ConfigManager::accel_offset_y,
        ConfigManager::accel_offset_z
    );

    logCalibrationStatus("Initialized");
}

void AccelCalibration::reset() {
    calibrated_ = false;
    gravityOffset_ = Eigen::Vector3f::Zero();
    measuredGravityMagnitude_ = 0.0f;
    gravityScaleFactor_ = 1.0f;  // Reset scale factor to 1.0 (no scaling)
    calibrationSamples_.clear();
    stationaryCount_ = 0;
    sampleCount_ = 0;

    if (initialized_) {
        logCalibrationStatus("Reset - awaiting calibration");
    }
}

Eigen::Vector3f AccelCalibration::applyCalibration(const Eigen::Vector3f& raw_accel) {
    // Debug bypass: Skip all calibration if configured
    if (ConfigManager::debug_accel_bypass_calibration) {
        return raw_accel;  // Return raw data unchanged
    }

    if (!initialized_) {
        initialize();
    }

    // If auto-calibration is enabled and not yet calibrated, collect samples
    if (autoCalibrate_ && !calibrated_) {
        if (isStationary()) {
            stationaryCount_++;

            // Wait for aircraft to settle before collecting samples
            if (stationaryCount_ >= STATIONARY_WAIT_SAMPLES) {
                collectSample(raw_accel);
            }
        } else {
            // Aircraft moved - reset calibration attempt
            if (stationaryCount_ > 0) {
                stationaryCount_ = 0;
                sampleCount_ = 0;
                calibrationSamples_.clear();
            }
        }
    }

    // Apply calibration using MAGNITUDE SCALING
    // This approach preserves directional information while correcting magnitude error
    // Unlike offset subtraction, scaling works correctly at all attitudes
    Eigen::Vector3f calibrated = raw_accel;

    if (calibrated_) {
        // Scale all axes by the same factor to preserve direction
        calibrated *= gravityScaleFactor_;
    }

    // Always apply manual offset (for fine-tuning, typically zero)
    calibrated -= manualOffset_;

    return calibrated;
}

bool AccelCalibration::isCalibrated() {
    return calibrated_;
}

void AccelCalibration::setAutoCalibrate(bool enable) {
    autoCalibrate_ = enable;
    if (!enable) {
        // Clear any in-progress calibration
        calibrationSamples_.clear();
        stationaryCount_ = 0;
        sampleCount_ = 0;
    }
}

void AccelCalibration::setManualOffset(const Eigen::Vector3f& offset) {
    manualOffset_ = offset;
}

Eigen::Vector3f AccelCalibration::getGravityOffset() {
    return gravityOffset_;
}

float AccelCalibration::getMeasuredGravityMagnitude() {
    return measuredGravityMagnitude_;
}

bool AccelCalibration::isStationary() {
    float groundspeed = DataRefManager::getFloat("sim/flightmodel/position/groundspeed");
    return groundspeed < STATIONARY_THRESHOLD;
}

void AccelCalibration::collectSample(const Eigen::Vector3f& accel) {
    calibrationSamples_.push_back(accel);
    sampleCount_++;

    // Log progress every 50 samples
    if (sampleCount_ % 50 == 0) {
        char buf[128];
        snprintf(buf, sizeof(buf), "Collecting samples: %d/%d", sampleCount_, CALIBRATION_SAMPLES);
        logCalibrationStatus(buf);
    }

    // Check if we have enough samples
    if (sampleCount_ >= CALIBRATION_SAMPLES) {
        performCalibration();
    }
}

void AccelCalibration::performCalibration() {
    if (calibrationSamples_.empty()) {
        logCalibrationStatus("ERROR: No samples collected");
        return;
    }

    // Compute mean acceleration vector from collected samples
    Eigen::Vector3f sum = Eigen::Vector3f::Zero();
    for (const auto& sample : calibrationSamples_) {
        sum += sample;
    }
    Eigen::Vector3f meanAccel = sum / static_cast<float>(calibrationSamples_.size());

    // Store measured gravity magnitude (for diagnostics)
    measuredGravityMagnitude_ = meanAccel.norm();

    //--------------------------------------------------------------------------
    // MAGNITUDE SCALING CALIBRATION (v2 - January 2025)
    //--------------------------------------------------------------------------
    //
    // WHY SCALING INSTEAD OF OFFSET?
    //
    // The v1 offset approach computed: gravityOffset = measured - expected
    // and then applied: calibrated = raw - gravityOffset
    //
    // PROBLEM: This static offset doesn't rotate with attitude!
    // - On ground (level): Works correctly
    // - During flight (pitched/banked): The offset is WRONG because it was
    //   computed at ground attitude but applied at flight attitude
    // - Result: 17.8% bias error during dynamic flight
    //
    // SOLUTION: Use MAGNITUDE SCALING instead
    // - Compute: scaleFactor = expected_magnitude / measured_magnitude
    // - Apply: calibrated = raw * scaleFactor
    // - This preserves directional information while correcting magnitude
    // - Works at ALL attitudes because direction is preserved
    //
    // PHYSICS INSIGHT:
    // X-Plane aircraft models have magnitude error (report 1.05g instead of 1.0g)
    // NOT directional offset. Scaling corrects magnitude, offset doesn't.
    //--------------------------------------------------------------------------

    // Compute scale factor: expected gravity / measured gravity
    if (measuredGravityMagnitude_ < 0.1f) {
        logCalibrationStatus("ERROR: Measured gravity too small - aircraft may be in free fall");
        return;
    }

    gravityScaleFactor_ = EXPECTED_GRAVITY / measuredGravityMagnitude_;

    // Validate scale factor - should be close to 1.0 (within 10%)
    float scaleDeviation = std::fabs(1.0f - gravityScaleFactor_);
    if (scaleDeviation > 0.1f) {
        char buf[256];
        snprintf(buf, sizeof(buf),
            "WARNING: Large scale correction (%.1f%%). "
            "Check X-Plane aircraft physics model.",
            scaleDeviation * 100.0f);
        logCalibrationStatus(buf);
    }

    // Clear legacy offset (no longer used)
    gravityOffset_ = Eigen::Vector3f::Zero();

    // Mark calibration complete
    calibrated_ = true;
    calibrationSamples_.clear();

    // Get pitch/roll for logging (in degrees)
    float pitch = DataRefManager::getFloat("sim/flightmodel/position/theta");
    float roll = DataRefManager::getFloat("sim/flightmodel/position/phi");

    // Log success with calibration details
    char buf[600];
    snprintf(buf, sizeof(buf),
        "COMPLETE (v2 SCALING) - Attitude: P=%.1f deg R=%.1f deg | "
        "Measured: %.4f m/s^2 | Expected: %.4f m/s^2 | "
        "Scale Factor: %.6f (%.2f%% correction)",
        pitch, roll,
        measuredGravityMagnitude_, EXPECTED_GRAVITY,
        gravityScaleFactor_, (gravityScaleFactor_ - 1.0f) * 100.0f);
    logCalibrationStatus(buf);
}

void AccelCalibration::logCalibrationStatus(const char* status) {
    char buf[600];
    snprintf(buf, sizeof(buf), "px4xplane: [ACCEL_CAL] %s\n", status);
    XPLMDebugString(buf);
}
