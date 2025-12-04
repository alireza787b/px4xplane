// AccelCalibration.h
// Runtime accelerometer calibration for X-Plane SITL
//
// Purpose: Corrects systematic gravity magnitude error in X-Plane's g-load sensors.
// Different X-Plane aircraft models (especially VTOLs like Alia250) may report
// g_nrml != 1.0 when stationary, causing PX4's "High Accelerometer Bias" error.
//
// Solution: MAGNITUDE SCALING approach (v2, January 2025)
// On startup when stationary, measure actual gravity magnitude and compute
// a SCALE FACTOR to normalize to expected 9.81 m/s^2.
//
// Why scaling instead of offset?
// - Offset approach (v1): Works on ground, BREAKS during dynamic flight
//   because static offset doesn't rotate with attitude
// - Scaling approach (v2): Works at ALL attitudes because it preserves
//   directional information while only correcting magnitude error

#ifndef ACCELCALIBRATION_H
#define ACCELCALIBRATION_H

#include <Eigen/Dense>
#include <deque>
#include <cstdint>

class AccelCalibration {
public:
    /**
     * Initialize the calibration system.
     * Call once at plugin startup.
     */
    static void initialize();

    /**
     * Reset calibration state.
     * Call when connection is lost or aircraft changes.
     */
    static void reset();

    /**
     * Apply calibration to raw acceleration data.
     * During calibration phase: collects samples when stationary.
     * After calibration: applies SCALE FACTOR to correct magnitude.
     *
     * The scaling approach preserves directional information while
     * correcting the magnitude error from X-Plane aircraft models.
     *
     * @param raw_accel Raw acceleration vector in m/s^2 (FRD frame)
     * @return Calibrated acceleration vector in m/s^2
     */
    static Eigen::Vector3f applyCalibration(const Eigen::Vector3f& raw_accel);

    /**
     * Check if calibration is complete.
     * @return true if calibration has been performed
     */
    static bool isCalibrated();

    /**
     * Enable or disable automatic calibration.
     * When disabled, only manual offsets are applied.
     */
    static void setAutoCalibrate(bool enable);

    /**
     * Set manual offset to apply (in addition to auto-calibration).
     * Use for aircraft-specific fine-tuning if needed.
     */
    static void setManualOffset(const Eigen::Vector3f& offset);

    /**
     * Get the computed gravity offset (for diagnostics).
     * @return Offset vector (measured_gravity - expected_gravity)
     */
    static Eigen::Vector3f getGravityOffset();

    /**
     * Get the measured gravity magnitude during calibration.
     * @return Gravity magnitude in m/s^2 (expected: ~9.81)
     */
    static float getMeasuredGravityMagnitude();

    /**
     * Get the current scale factor used for calibration.
     * @return Scale factor (1.0 = no scaling, <1.0 = measured too high, >1.0 = measured too low)
     */
    static float getScaleFactor() { return gravityScaleFactor_; }

private:
    // Physical constants
    static constexpr float EXPECTED_GRAVITY = 9.80665f;  // Standard gravity (m/s^2)
    static constexpr float GRAVITY_TOLERANCE = 2.0f;     // Max acceptable deviation (m/s^2)

    // Calibration parameters
    static constexpr int CALIBRATION_SAMPLES = 200;      // ~1 second at 200Hz
    static constexpr float STATIONARY_THRESHOLD = 0.5f;  // Max groundspeed (m/s) for calibration
    static constexpr int STATIONARY_WAIT_SAMPLES = 50;   // Wait before sampling (~0.25s)

    // State variables
    static bool initialized_;
    static bool calibrated_;
    static bool autoCalibrate_;
    static Eigen::Vector3f gravityOffset_;       // Legacy (kept for compatibility, not used)
    static Eigen::Vector3f manualOffset_;
    static float measuredGravityMagnitude_;
    static float gravityScaleFactor_;            // NEW: Scale factor for magnitude correction

    // Calibration sample collection
    static std::deque<Eigen::Vector3f> calibrationSamples_;
    static int stationaryCount_;
    static int sampleCount_;

    /**
     * Check if aircraft is stationary (groundspeed < threshold).
     */
    static bool isStationary();

    /**
     * Collect a calibration sample.
     */
    static void collectSample(const Eigen::Vector3f& accel);

    /**
     * Perform calibration using collected samples.
     */
    static void performCalibration();

    /**
     * Log calibration status to X-Plane log.
     */
    static void logCalibrationStatus(const char* status);
};

#endif // ACCELCALIBRATION_H
