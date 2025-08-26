#include "MAVLinkManager.h"
#include "ConnectionManager.h"
#include "DataRefManager.h"
#include <vector>
#include "XPLMUtilities.h"
#include <tuple>  // for std::tuple
#include <ConfigManager.h>
#include "FilterUtils.h"
#include "TimeManager.h"  // Include the TimeManager class
#include <cmath>

#ifndef NOMINMAX
#define NOMINMAX
#endif

// Define and initialize the static member
MAVLinkManager::HILActuatorControlsData MAVLinkManager::hilActuatorControlsData = {};



// Declare constants at the class or namespace level
constexpr float DEG_TO_RAD = 3.14159265358979323846 / 180.0;
constexpr float GRAVITY = 9.81;

// Define and initialize the random number generators and distributions
std::random_device MAVLinkManager::rd;
std::mt19937 MAVLinkManager::gen(MAVLinkManager::rd());
std::normal_distribution<float> MAVLinkManager::highFreqNoise(0.0f, 0.00003f);
std::normal_distribution<float> MAVLinkManager::lowFreqNoise(0.0f, 0.000003f); // Larger, slower noise
std::normal_distribution<float> vibrationNoise(0.0f, 0.0f); // Adjust the standard deviation as needed

std::normal_distribution<float> MAVLinkManager::noiseDistribution_mag(0.0f, 0.000001f);

//------------------------------------------------------------------------------
// setAccelerationData()
// Uses computeAcceleration() to update the HIL_SENSOR acceleration data.
//------------------------------------------------------------------------------
void MAVLinkManager::setAccelerationData(mavlink_hil_sensor_t& hil_sensor) {
	Eigen::Vector3f accel = computeAcceleration();
	hil_sensor.xacc = accel.x();
	hil_sensor.yacc = accel.y();
	hil_sensor.zacc = accel.z();
}


/**
 * @brief FIXED: Computes clean accelerometer data for stable EKF2 operation
 *
 * This simplified version eliminates the excessive noise and over-filtering that
 * was causing EKF2 "High Accelerometer Bias" and "vertical velocity unstable" errors.
 *
 * Key Changes from Original:
 * - Reduced white noise from 0.06 to 0.008 m/s² (matches EKF2 expectations)
 * - Simplified filtering: removed excessive cascaded filters
 * - Eliminated artificial thermal drift and complex noise modeling
 * - Maintained only essential filtering for X-Plane data stability
 */
Eigen::Vector3f MAVLinkManager::computeAcceleration() {
	// ========================================================================
	// CONFIGURATION PARAMETERS - FIXED for EKF2 compatibility
	// ========================================================================

	// Realistic noise levels that match EKF2_ACC_NOISE expectations
	constexpr float ACCEL_WHITE_NOISE_STD = 0.003f;        // 0.8 mg RMS (reduced from 0.06!)
	constexpr float ACCEL_QUANTIZATION_MG = 2.0f;          // Relaxed quantization

	// Simplified, gentle filtering (no more aggressive cascaded filters)
	constexpr float ACCEL_FILTER_ALPHA = 0.75f;            // Gentle 9% filter (was 98%!)
	constexpr int ACCEL_MEDIAN_WINDOW_SIZE = 3;             // Smaller window (was 5)

	// No thermal drift or complex bias modeling for simulation
	constexpr float ACCEL_BIAS_DRIFT_AMPLITUDE = 0.0f;     // Keep at zero for simulation

	// ========================================================================
	// STATIC VARIABLES - Maintain simple state between calls
	// ========================================================================

	static std::deque<float> accel_x_median_window;
	static std::deque<float> accel_y_median_window;
	static std::deque<float> accel_z_median_window;

	static float filtered_xacc = 0.0f;
	static float filtered_yacc = 0.0f;
	static float filtered_zacc = -9.81f;

	static bool initialized = false;
	static uint64_t lastAccelUpdateTime = 0;
	static Eigen::Vector3f cachedAccel(0.0f, 0.0f, -9.81f);

	// Simple noise generation (much reduced)
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::normal_distribution<float> clean_noise(0.0f, ACCEL_WHITE_NOISE_STD);

	// ========================================================================
	// CACHING - Return cached result if computed in same cycle
	// ========================================================================

	uint64_t currentTime = TimeManager::getCurrentTimeUsec();
	if (currentTime == lastAccelUpdateTime && initialized) {
		return cachedAccel;
	}
	lastAccelUpdateTime = currentTime;

	// ========================================================================
	// RAW DATA ACQUISITION - Get clean data from X-Plane
	// ========================================================================

	float raw_xacc = DataRefManager::getFloat("sim/flightmodel/forces/g_axil") * DataRefManager::g_earth * -1.0f;
	float raw_yacc = DataRefManager::getFloat("sim/flightmodel/forces/g_side") * DataRefManager::g_earth * -1.0f;
	float raw_zacc = DataRefManager::getFloat("sim/flightmodel/forces/g_nrml") * DataRefManager::g_earth * -1.0f;

	// ========================================================================
	// INITIALIZATION - Set initial states once
	// ========================================================================

	if (!initialized) {
		filtered_xacc = raw_xacc;
		filtered_yacc = raw_yacc;
		filtered_zacc = raw_zacc;

		// Initialize small median filter windows
		for (int i = 0; i < ACCEL_MEDIAN_WINDOW_SIZE; ++i) {
			accel_x_median_window.push_back(raw_xacc);
			accel_y_median_window.push_back(raw_yacc);
			accel_z_median_window.push_back(raw_zacc);
		}

		initialized = true;
	}

	// ========================================================================
	// SIMPLIFIED FILTERING - Single-stage gentle filtering only
	// ========================================================================

	// Stage 1: Light median filtering (remove spikes only)
	auto updateMedianWindow = [](std::deque<float>& window, float new_value) -> float {
		if (window.size() >= ACCEL_MEDIAN_WINDOW_SIZE) {
			window.pop_front();
		}
		window.push_back(new_value);

		std::vector<float> sorted_window(window.begin(), window.end());
		std::sort(sorted_window.begin(), sorted_window.end());
		return sorted_window[sorted_window.size() / 2];
		};

	float median_xacc = updateMedianWindow(accel_x_median_window, raw_xacc);
	float median_yacc = updateMedianWindow(accel_y_median_window, raw_yacc);
	float median_zacc = updateMedianWindow(accel_z_median_window, raw_zacc);

	// Stage 2: Gentle low-pass filtering (much lighter than before)
	filtered_xacc = ACCEL_FILTER_ALPHA * median_xacc + (1.0f - ACCEL_FILTER_ALPHA) * filtered_xacc;
	filtered_yacc = ACCEL_FILTER_ALPHA * median_yacc + (1.0f - ACCEL_FILTER_ALPHA) * filtered_yacc;
	filtered_zacc = ACCEL_FILTER_ALPHA * median_zacc + (1.0f - ACCEL_FILTER_ALPHA) * filtered_zacc;

	// ========================================================================
	// MINIMAL REALISTIC NOISE - Only what EKF2 expects to see
	// ========================================================================

	// Add minimal white noise (matches EKF2_ACC_NOISE = 0.35 m/s/s expectation)
	float noise_x = clean_noise(gen);
	float noise_y = clean_noise(gen);
	float noise_z = clean_noise(gen);

	float final_xacc = filtered_xacc + noise_x;
	float final_yacc = filtered_yacc + noise_y;
	float final_zacc = filtered_zacc + noise_z;

	// ========================================================================
	// RELAXED QUANTIZATION - Eliminate micro-oscillations
	// ========================================================================

	constexpr float MG_TO_MS2 = 9.81f / 1000.0f;
	float quantization_step = ACCEL_QUANTIZATION_MG * MG_TO_MS2;

	float quantized_xacc = std::round(final_xacc / quantization_step) * quantization_step;
	float quantized_yacc = std::round(final_yacc / quantization_step) * quantization_step;
	float quantized_zacc = std::round(final_zacc / quantization_step) * quantization_step;

	// ========================================================================
	// RESULT ASSEMBLY
	// ========================================================================

	cachedAccel = Eigen::Vector3f(quantized_xacc, quantized_yacc, quantized_zacc);

	return cachedAccel;
}

/*
 * SUMMARY OF CHANGES FROM ORIGINAL:
 *
 * 1. NOISE REDUCTION: 0.06 → 0.008 m/s² (87% reduction)
 * 2. FILTER SIMPLIFICATION: Removed cascaded filters, gentler single-stage
 * 3. ELIMINATED: Thermal drift, complex bias modeling, vibration effects
 * 4. QUANTIZATION: Relaxed from 0.5 to 2.0 mg
 * 5. MEDIAN WINDOW: Reduced from 5 to 3 samples
 * 6. FILTER ALPHA: Reduced from 98% to 85% (more responsive)
 *
 * Expected Results:
 * - "High Accelerometer Bias" error eliminated
 * - "vertical velocity unstable" error eliminated
 * - Stable EKF2 operation with clean sensor fusion
 * - Proper landing detection and mode transitions
 */

/**
 * @brief Sends the HIL_SENSOR MAVLink message.
 *
 * This function constructs and sends the HIL_SENSOR message which contains simulated
 * sensor readings for Hardware-In-the-Loop (HIL) simulation. It checks if a connection
 * is established before sending the message. The function gathers data from various
 * simulation sources, applies necessary conversions, and populates the HIL_SENSOR message.
 *
 * The following sensor data is populated:
 * - Acceleration
 * - Gyroscope
 * - Pressure
 * - Magnetic Field
 * - Temperature
 *
 * The fields_updated bitmask in the HIL_SENSOR message is set to indicate which
 * sensor readings are being updated. For more details on the bitmask values, refer to:
 * https://mavlink.io/en/messages/common.html#HIL_SENSOR_UPDATED_FLAGS
 *
 * @note Ensure that the ConnectionManager is initialized and a connection is established
 *       before calling this function.
 *
 * @warning This function should be called at appropriate simulation update rates to ensure
 *          timely delivery of sensor data.
 *
 * Usage:
 * @code
 * MAVLinkManager::sendHILSensor();
 * @endcode
 *
 * @see setAccelerationData
 * @see setGyroData
 * @see setPressureData
 * @see setMagneticFieldData
 */
void MAVLinkManager::sendHILSensor(uint8_t sensor_id = 0) {

	if (!ConnectionManager::isConnected()) return;

	mavlink_message_t msg;
	mavlink_hil_sensor_t hil_sensor;

	if (ConfigManager::USE_XPLANE_TIME) {
		hil_sensor.time_usec = static_cast<uint64_t>(DataRefManager::getFloat("sim/time/total_flight_time_sec") * 1e6);
	}
	else {
		hil_sensor.time_usec = TimeManager::getCurrentTimeUsec();
	}

	hil_sensor.id = uint8_t(sensor_id);

	setAccelerationData(hil_sensor);
	setGyroData(hil_sensor);
	setPressureData(hil_sensor);
	setMagneticFieldData(hil_sensor);

	hil_sensor.temperature = DataRefManager::getFloat("sim/cockpit2/temperature/outside_air_temp_degc");

	uint32_t fields_updated = 0;
	fields_updated |= (1 << 0);  // HIL_SENSOR_UPDATED_XACC
	fields_updated |= (1 << 1);  // HIL_SENSOR_UPDATED_YACC
	fields_updated |= (1 << 2);  // HIL_SENSOR_UPDATED_ZACC
	fields_updated |= (1 << 3);  // HIL_SENSOR_UPDATED_XGYRO
	fields_updated |= (1 << 4);  // HIL_SENSOR_UPDATED_YGYRO
	fields_updated |= (1 << 5);  // HIL_SENSOR_UPDATED_ZGYRO
	fields_updated |= (1 << 6);  // HIL_SENSOR_UPDATED_XMAG
	fields_updated |= (1 << 7);  // HIL_SENSOR_UPDATED_YMAG
	fields_updated |= (1 << 8);  // HIL_SENSOR_UPDATED_ZMAG
	fields_updated |= (1 << 9);  // HIL_SENSOR_UPDATED_ABS_PRESSURE
	fields_updated |= (1 << 10); // HIL_SENSOR_UPDATED_DIF_PRESSURE
	fields_updated |= (1 << 11); // HIL_SENSOR_UPDATED_PRESSURE_ALT
	fields_updated |= (1 << 12); // HIL_SENSOR_UPDATED_TEMPERATURE

	hil_sensor.fields_updated = fields_updated;

	mavlink_msg_hil_sensor_encode(1, 1, &msg, &hil_sensor);
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	int len = mavlink_msg_to_send_buffer(buffer, &msg);
	ConnectionManager::sendData(buffer, len);
}



/**
 * @brief Sends the HIL_GPS MAVLink message.
 *
 * This function constructs and sends the HIL_GPS message which contains simulated
 * GPS readings for Hardware-In-the-Loop (HIL) simulation. It checks if a connection
 * is established before sending the message. The function gathers data from the simulation
 * environment, applies necessary conversions, and populates the HIL_GPS message.
 *
 * Additionally, it calls updateMagneticFieldIfNeeded to check if the Earth's magnetic field
 * needs to be updated based on the current position.
 *
 * The following GPS data is populated:
 * - Time
 * - Fix type
 * - Latitude and Longitude
 * - Altitude
 * - Horizontal and Vertical Dilution of Precision (HDOP & VDOP)
 * - Number of visible satellites
 * - Velocities in North, East, and Down directions
 * - Ground speed
 * - Course over ground
 * - GPS ID and Yaw
 *
 * @note Ensure that the ConnectionManager is initialized and a connection is established
 *       before calling this function.
 *
 * @warning This function should be called at appropriate simulation update rates to ensure
 *          timely delivery of GPS data.
 *
 * Usage:
 * @code
 * MAVLinkManager::sendHILGPS();
 * @endcode
 */
 /**
  * @brief Fixed GPS transmission function with proper error handling and debugging
  */
void MAVLinkManager::sendHILGPS() {
	// Add debug logging to verify function is being called
	static int debug_counter = 0;
	if (debug_counter++ % 100 == 0) {  // Log every 5 seconds (at 20Hz)
		XPLMDebugString("px4xplane: sendHILGPS() called\n");
	}

	if (!ConnectionManager::isConnected()) {
		if (debug_counter % 100 == 0) {
			XPLMDebugString("px4xplane: GPS NOT SENT - Not connected!\n");
		}
		return;
	}

	mavlink_message_t msg;
	mavlink_hil_gps_t hil_gps;

	// Clear the structure to avoid undefined values
	memset(&hil_gps, 0, sizeof(hil_gps));

	try {
		// Set time data
		setGPSTimeAndFix(hil_gps);

		// Set position data with validation
		setGPSPositionData(hil_gps);

		// Validate position data before sending
		if (hil_gps.lat == 0 && hil_gps.lon == 0) {
			XPLMDebugString("px4xplane: GPS WARNING - Invalid position data (0,0)\n");
		}

		setGPSAccuracyData(hil_gps);
		setGPSVelocityData(hil_gps);
		setGPSHeadingData(hil_gps);

		// Check if the magnetic field needs to be updated
		updateMagneticFieldIfExceededTreshold(hil_gps);

		// Use the SAME encoding method as working functions
		mavlink_msg_hil_gps_encode(1, 1, &msg, &hil_gps);

		// Use the consistent sendData method instead of direct buffer
		sendData(msg);

		// Log success periodically
		if (debug_counter % 100 == 0) {
			char debug_msg[256];
			snprintf(debug_msg, sizeof(debug_msg),
				"px4xplane: GPS SENT - Lat: %d, Lon: %d, Alt: %d, Fix: %d\n",
				hil_gps.lat, hil_gps.lon, hil_gps.alt, hil_gps.fix_type);
			XPLMDebugString(debug_msg);
		}

	}
	catch (...) {
		XPLMDebugString("px4xplane: GPS EXCEPTION caught in sendHILGPS()\n");
	}
}




/**
 * @brief Checks if the Earth's magnetic field needs to be updated and updates it if necessary.
 *
 * This function calculates the distance between the current position and the last position
 * where the magnetic field was updated. If this distance exceeds a predefined threshold,
 * it triggers an update of the Earth's magnetic field in NED coordinates using the current
 * position data.
 *
 * This function is called within sendHILGPS to ensure the magnetic field data is updated
 * according to the movement of the simulated aircraft.
 *
 * @param hil_gps The HIL_GPS data structure containing the current GPS data.
 *
 * Usage:
 * @code
 * updateMagneticFieldIfNeeded(hil_gps);
 * @endcode
 */
void MAVLinkManager::updateMagneticFieldIfExceededTreshold(const mavlink_hil_gps_t& hil_gps) {
	/*GeodeticPosition currentPosition = {
		hil_gps.lat * 1e-7,
		hil_gps.lon * 1e-7,
		hil_gps.alt * 1e-3
	};*/

	GeodeticPosition currentPosition = {
		DataRefManager::getFloat("sim/flightmodel/position/latitude"),
		DataRefManager::getFloat("sim/flightmodel/position/longitude"),
		DataRefManager::getFloat("sim/flightmodel/position/elevation")
	};

	if (DataRefManager::calculateDistance(currentPosition, DataRefManager::lastPosition) > DataRefManager::UPDATE_THRESHOLD) {
		XPLMDebugString("px4xplane: Mag ValidityTreshold Reached!\n");

		DataRefManager::updateEarthMagneticFieldNED(currentPosition);
		DataRefManager::lastPosition = currentPosition;
	}
}




/**
 * @brief Sends the HIL state quaternion message.
 *
 * This function fetches the required data, populates the MAVLink HIL state
 * quaternion message, and sends it.
 */
void MAVLinkManager::sendHILStateQuaternion() {
	if (!ConnectionManager::isConnected()) return;

	mavlink_message_t msg;
	mavlink_hil_state_quaternion_t hil_state;

	populateHILStateQuaternion(hil_state);

	mavlink_msg_hil_state_quaternion_encode(1, 1, &msg, &hil_state);
	sendData(msg);
}

/**
 * @brief Populates the HIL state quaternion structure.
 *
 * This function fetches the required data from the DataRefManager and sets
 * the appropriate fields in the provided hil_state structure.
 *
 * @param hil_state Reference to the MAVLink HIL state quaternion structure to be populated.
 */
void MAVLinkManager::populateHILStateQuaternion(mavlink_hil_state_quaternion_t& hil_state) {
	// Set time in microseconds.
	if (ConfigManager::USE_XPLANE_TIME) {
		hil_state.time_usec = static_cast<uint64_t>(DataRefManager::getFloat("sim/time/total_flight_time_sec") * 1e6);
	}
	else {
		hil_state.time_usec = TimeManager::getCurrentTimeUsec();
	}

	// Retrieve quaternion data from X-Plane.
	std::vector<float> q = DataRefManager::getFloatArray("sim/flightmodel/position/q");
	for (int i = 0; i < 4; ++i) {
		hil_state.attitude_quaternion[i] = q[i];
	}

	// Angular velocity data (roll, pitch, yaw rates)
	hil_state.rollspeed = DataRefManager::getFloat("sim/flightmodel/position/Prad");
	hil_state.pitchspeed = DataRefManager::getFloat("sim/flightmodel/position/Qrad");
	hil_state.yawspeed = DataRefManager::getFloat("sim/flightmodel/position/Rrad");

	// Position data (latitude, longitude, altitude)
	hil_state.lat = static_cast<int32_t>(DataRefManager::getFloat("sim/flightmodel/position/latitude") * 1e7);
	hil_state.lon = static_cast<int32_t>(DataRefManager::getFloat("sim/flightmodel/position/longitude") * 1e7);
	hil_state.alt = static_cast<int32_t>(DataRefManager::getFloat("sim/flightmodel/position/elevation") * 1e3);

	// Convert local OGL velocities to NED frame:
	float ogl_vx = DataRefManager::getFloat("sim/flightmodel/position/local_vx") * 100;
	float ogl_vy = DataRefManager::getFloat("sim/flightmodel/position/local_vy") * 100;
	float ogl_vz = DataRefManager::getFloat("sim/flightmodel/position/local_vz") * 100;
	hil_state.vx = -ogl_vz;  // North (NED) from South (OGL)
	hil_state.vy = ogl_vx;   // East (NED) from East (OGL)
	hil_state.vz = -ogl_vy;  // Down (NED) from Up (OGL)

	// Airspeed data (indicated and true)
	hil_state.ind_airspeed = static_cast<uint16_t>(DataRefManager::getFloat("sim/flightmodel/position/indicated_airspeed") * 100);
	hil_state.true_airspeed = static_cast<uint16_t>(DataRefManager::getFloat("sim/flightmodel/position/true_airspeed") * 100);

	// Reuse the computed acceleration from computeAcceleration()
	Eigen::Vector3f accel = computeAcceleration();
	hil_state.xacc = accel.x();
	hil_state.yacc = accel.y();
	hil_state.zacc = accel.z();
}



/**
 * @brief Sends the provided MAVLink message.
 *
 * This function encodes the provided MAVLink message into a buffer and sends it.
 *
 * @param msg The MAVLink message to be sent.
 */
void MAVLinkManager::sendData(const mavlink_message_t& msg) {
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	int len = mavlink_msg_to_send_buffer(buffer, &msg);
	ConnectionManager::sendData(buffer, len);
}



/**
 * @brief Sends the HIL RC Inputs to the MAVLink connection.
 *
 * This function fetches the required RC input data from the DataRefManager,
 * maps the values to the appropriate MAVLink format, and sends the HIL RC
 * Inputs message over the MAVLink connection.
 */
void MAVLinkManager::sendHILRCInputs() {
	// Check if we're connected before proceeding
	if (!ConnectionManager::isConnected()) return;

	// Create a MAVLink message and the corresponding HIL RC inputs structure
	mavlink_message_t msg;
	mavlink_hil_rc_inputs_raw_t hil_rc_inputs;

	if (ConfigManager::USE_XPLANE_TIME) {
		hil_rc_inputs.time_usec = static_cast<uint64_t>(DataRefManager::getFloat("sim/time/total_flight_time_sec") * 1e6);
	}
	else {
		hil_rc_inputs.time_usec = TimeManager::getCurrentTimeUsec();
	}


	// Map RC channels using joystick data from X-Plane
	hil_rc_inputs.chan1_raw = mapRCChannel(DataRefManager::getFloat("sim/joystick/yoke_roll_ratio"), -1, +1);
	hil_rc_inputs.chan2_raw = mapRCChannel(DataRefManager::getFloat("sim/joystick/yoke_pitch_ratio"), -1, +1);
	hil_rc_inputs.chan3_raw = mapRCChannel(DataRefManager::getFloat("sim/cockpit2/engine/actuators/throttle_ratio_all"), 0, +1);
	hil_rc_inputs.chan4_raw = mapRCChannel(DataRefManager::getFloat("sim/joystick/yoke_heading_ratio"), -1, +1);

	// Additional channels, as needed (e.g., auxiliary channels, mode switch)
	hil_rc_inputs.chan5_raw = mapRCChannel(0, -1, +1);  // Placeholder for an unused or fixed-value channel
	// Add mappings for other channels as needed
	// hil_rc_inputs.chan6_raw = mapRCChannel(...);
	// hil_rc_inputs.chan7_raw = mapRCChannel(...);
	// hil_rc_inputs.chan8_raw = mapRCChannel(...);

	// Set RSSI (Received Signal Strength Indicator) to maximum value
	constexpr uint8_t RSSI_MAX_VALUE = 255;
	hil_rc_inputs.rssi = RSSI_MAX_VALUE; // Full signal strength

	// Encode the MAVLink message with the populated HIL RC inputs structure
	mavlink_msg_hil_rc_inputs_raw_encode(1, 1, &msg, &hil_rc_inputs);

	// Send the MAVLink message buffer
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	int len = mavlink_msg_to_send_buffer(buffer, &msg);
	ConnectionManager::sendData(buffer, len);
}


/**
 * @brief Maps the RC channel value to the MAVLink format.
 *
 * This helper function maps an RC channel value from its original range
 * to the MAVLink expected range of [1000, 2000].
 *
 * @param value The original value of the RC channel.
 * @param min The minimum value of the original range.
 * @param max The maximum value of the original range.
 * @return The mapped value in the MAVLink format.
 */
uint16_t MAVLinkManager::mapRCChannel(float value, float min, float max) {
	return static_cast<uint16_t>(DataRefManager::mapChannelValue(value, min, max, 1000, 2000));
}



/**
 * @brief Receives and processes HIL actuator control messages.
 *
 * This function parses the received buffer for MAVLink messages and processes
 * the HIL actuator control messages by extracting and storing the relevant data.
 *
 * @param buffer Pointer to the received data buffer.
 * @param size Size of the received data buffer.
 */
void MAVLinkManager::receiveHILActuatorControls(uint8_t* buffer, int size) {
	if (!ConnectionManager::isConnected()) return;

	mavlink_message_t msg;
	mavlink_status_t status;
	for (int i = 0; i < size; ++i) {
		if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
			handleReceivedMessage(msg);
		}
	}
}

/**
 * @brief Handles the received MAVLink message.
 *
 * This function checks the type of the received MAVLink message and processes
 * it accordingly.
 *
 * @param msg The received MAVLink message.
 */
void MAVLinkManager::handleReceivedMessage(const mavlink_message_t& msg) {
	//XPLMDebugString(("px4xplane: Received msgid =  " + std::to_string(msg.msgid) + "\n").c_str());
	switch (msg.msgid) {
	case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
		processHILActuatorControlsMessage(msg);
		break;
		// Handle other MAVLink message types if needed
	default:
		// Handle unrecognized message types if needed
		break;
	}
}

/**
 * Processes the High-Level (HIL) actuator control message received via MAVLink.
 *
 * This function decodes the HIL actuator control message from MAVLink and updates the
 * hilActuatorControlsData member variable. The processing now does not depend on the type of airframe
 * configured and directly uses the received controls.
 *
 * This function is essential for translating the actuator control commands received from the
 * autopilot (sent via MAVLink) into a format that can be understood and used by the simulation
 * environment (X-Plane).
 *
 * @param msg The received HIL actuator control MAVLink message, containing control information for actuators.
 */
void MAVLinkManager::processHILActuatorControlsMessage(const mavlink_message_t& msg) {
	mavlink_hil_actuator_controls_t hil_actuator_controls;
	mavlink_msg_hil_actuator_controls_decode(&msg, &hil_actuator_controls);

	// Store the timestamp and mode information
	MAVLinkManager::hilActuatorControlsData.timestamp = hil_actuator_controls.time_usec;
	MAVLinkManager::hilActuatorControlsData.mode = hil_actuator_controls.mode;
	MAVLinkManager::hilActuatorControlsData.flags = hil_actuator_controls.flags;

	// Directly use the received controls
	for (int i = 0; i < 16; i++) {
		MAVLinkManager::hilActuatorControlsData.controls[i] = hil_actuator_controls.controls[i];
		// Debug message for each channel
		// XPLMDebugString(("px4xplane: Actuator channel " + std::to_string(i) +
		//     " value: " + std::to_string(MAVLinkManager::hilActuatorControlsData.controls[i]) + "\n").c_str());
	}
}







// Deprecated ! Old Method using xplane gload 
///**
// * @brief Sets the acceleration data in the HIL_SENSOR message.
// *
// * This function retrieves the aircraft's current acceleration forces from X-Plane simulation
// * and applies both median and low-pass filtering based on the configuration settings.
// * The filtering process is controlled by configuration flags and alpha values set in the
// * ConfigManager. The median filter helps eliminate outliers or spikes, while the low-pass
// * filter smooths the data. The filtered or raw data is then converted to the appropriate
// * gravitational constant and set in the HIL_SENSOR MAVLink message.
// *
// * The process involves:
// * - Retrieving the raw accelerometer data (x, y, z axes).
// * - Applying median filtering to reduce spikes or outliers.
// * - Applying low-pass filtering to smooth the data.
// * - Multiplying the data by the gravitational constant.
// * - Setting the processed data in the HIL_SENSOR message.
// *
// * @param hil_sensor Reference to the HIL_SENSOR message where the acceleration data will be set.
// */
//void MAVLinkManager::setAccelerationData(mavlink_hil_sensor_t& hil_sensor) {
//	// Retrieve raw accelerometer data from X-Plane
//	float raw_xacc = DataRefManager::getFloat("sim/flightmodel/forces/g_axil") * DataRefManager::g_earth * -1;
//	float raw_yacc = DataRefManager::getFloat("sim/flightmodel/forces/g_side") * DataRefManager::g_earth * -1;
//	float raw_zacc = DataRefManager::getFloat("sim/flightmodel/forces/g_nrml") * DataRefManager::g_earth * -1;
//	/*
//	// Add realistic ground vibration
//	float vibration_frequency = 50.0f; // Frequency of vibration (Hz)
//	float vibration_amplitude = 0.05f; // Amplitude of vibration (m/s^2)
//	float noise_amplitude = 0.01f; // Amplitude of noise (m/s^2)
//
//	// Get the current time in seconds
//	double currentTimeSec = TimeManager::getCurrentTimeSec();
//
//	// Simulate vibration using a simple sinusoidal function plus noise
//	float vibration_x = vibration_amplitude * sin(2 * M_PI * vibration_frequency * currentTimeSec) + MAVLinkManager::highFreqNoise(gen) * noise_amplitude;
//	float vibration_y = vibration_amplitude * sin(2 * M_PI * vibration_frequency * currentTimeSec + M_PI / 2) + MAVLinkManager::highFreqNoise(gen) * noise_amplitude;
//	float vibration_z = vibration_amplitude * sin(2 * M_PI * vibration_frequency * currentTimeSec + M_PI) + MAVLinkManager::highFreqNoise(gen) * noise_amplitude;
//
//	// Add the vibration to the raw accelerometer data
//	raw_xacc += vibration_x;
//	raw_yacc += vibration_y;
//	raw_zacc += vibration_z;
//	*/
//
//	raw_xacc += vibrationNoise(gen);
//	raw_yacc += vibrationNoise(gen);
//	raw_zacc += vibrationNoise(gen);
//
//
//	// Apply filtering if enabled, otherwise use raw data
//	hil_sensor.xacc = DataRefManager::applyFilteringIfNeeded(raw_xacc, ConfigManager::filter_accel_enabled, ConfigManager::accel_filter_alpha, DataRefManager::median_filter_window_xacc);
//	hil_sensor.yacc = DataRefManager::applyFilteringIfNeeded(raw_yacc, ConfigManager::filter_accel_enabled, ConfigManager::accel_filter_alpha, DataRefManager::median_filter_window_yacc);
//	hil_sensor.zacc = DataRefManager::applyFilteringIfNeeded(raw_zacc, ConfigManager::filter_accel_enabled, ConfigManager::accel_filter_alpha, DataRefManager::median_filter_window_zacc);
//}







/**
 * @brief Sets the gyroscope data in the HIL_SENSOR message.
 *
 * This function retrieves the aircraft's current rotational rates (in radians) from the simulation
 * and sets them in the provided HIL_SENSOR message.
 *
 * @param hil_sensor Reference to the HIL_SENSOR message where the gyroscope data will be set.
 */
void MAVLinkManager::setGyroData(mavlink_hil_sensor_t& hil_sensor) {
	hil_sensor.xgyro = DataRefManager::getFloat("sim/flightmodel/position/Prad");
	hil_sensor.ygyro = DataRefManager::getFloat("sim/flightmodel/position/Qrad");
	hil_sensor.zgyro = DataRefManager::getFloat("sim/flightmodel/position/Rrad");
}


/**
 * @brief Sets realistic barometric pressure data for the HIL_SENSOR message with advanced filtering.
 *
 * This function simulates realistic barometric sensor behavior by applying sophisticated filtering
 * and realistic noise characteristics in the pressure domain. The implementation ensures stable,
 * consistent altitude estimation compatible with PX4's EKF2 requirements.
 *
 * Key Features:
 * - Multi-stage pressure filtering: median → low-pass → quantization
 * - Realistic pressure domain noise modeling (not altitude domain)
 * - Thermal drift simulation for environmental realism
 * - Sensor response time modeling
 * - Consistent with GPS altitude filtering approach
 * - Configurable parameters for different sensor quality scenarios
 *
 * Filter Chain (Pressure):
 * 1. Raw pressure calculation from X-Plane altitude
 * 2. Median filter: Removes pressure spikes and outliers
 * 3. Low-pass filter: Smooths high-frequency pressure fluctuations
 * 4. Realistic noise addition: Pressure domain noise + thermal drift
 * 5. Quantization: Eliminates sub-Pascal pressure jitter
 *
 * @param hil_sensor Reference to the HIL_SENSOR message where pressure data will be set.
 */
void MAVLinkManager::setPressureData(mavlink_hil_sensor_t& hil_sensor) {
	// ========================================================================
	// CONFIGURATION PARAMETERS - Tunable for different barometer quality scenarios
	// ========================================================================

	// Barometric sensor filtering configuration
	constexpr float PRESSURE_FILTER_ALPHA = 0.80f;         // Low-pass filter strength (0.0-1.0)
	constexpr int PRESSURE_MEDIAN_WINDOW_SIZE = 7;          // Median filter window (5-9 recommended)
	constexpr float PRESSURE_QUANTIZATION_PA = 1.0f;       // Pressure quantization in Pascals

	// Realistic noise characteristics (based on modern barometric sensors)
	constexpr float PRESSURE_NOISE_STD_PA = 1.5f;          // ~0.015 hPa noise (realistic for good sensors)
	constexpr float THERMAL_DRIFT_AMPLITUDE_PA = 5.0f;     // Slow thermal drift amplitude
	constexpr float THERMAL_DRIFT_PERIOD_SEC = 300.0f;     // 5-minute thermal cycle

	// Sensor response characteristics
	constexpr double BARO_UPDATE_PERIOD_SEC = 0.02;        // 50 Hz update rate (realistic for barometers)
	constexpr float SENSOR_RESPONSE_TIME_ALPHA = 0.85f;    // Sensor physical response lag

	// Differential pressure filtering
	constexpr float DIFF_PRESSURE_FILTER_ALPHA = 0.90f;    // Airspeed-derived pressure filtering

	// ========================================================================
	// STATIC VARIABLES - Maintain sensor state between function calls
	// ========================================================================

	static std::deque<float> pressure_median_window;        // Pressure median filter window
	static float filtered_pressure_hPa = 1013.25f;         // Low-pass filtered pressure
	static float sensor_lagged_pressure_hPa = 1013.25f;    // Sensor response lag simulation
	static float filtered_diff_pressure_hPa = 0.0f;        // Filtered differential pressure
	static double last_baro_update_time = 0.0;             // Last barometer update timestamp
	static double thermal_drift_phase = 0.0;               // Thermal drift phase tracking
	static bool initialized = false;                        // Initialization flag

	// Random number generation for realistic noise
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::normal_distribution<float> pressure_noise(0.0f, PRESSURE_NOISE_STD_PA);

	// ========================================================================
	// CONSTANTS AND DATA ACQUISITION
	// ========================================================================

	constexpr float FEET_TO_METERS = 0.3048f;
	double current_time = TimeManager::getCurrentTimeSec();

	// Get raw pressure altitude from X-Plane
	float raw_pressure_altitude_ft = DataRefManager::getFloat("sim/flightmodel2/position/pressure_altitude");
	float raw_pressure_altitude_m = raw_pressure_altitude_ft * FEET_TO_METERS;

	// Calculate raw barometric pressure from altitude using ISA model
	float raw_pressure_hPa = DataRefManager::calculatePressureFromAltitude(raw_pressure_altitude_m);

	// ========================================================================
	// INITIALIZATION - Set initial sensor states on first call
	// ========================================================================

	if (!initialized) {
		filtered_pressure_hPa = raw_pressure_hPa;
		sensor_lagged_pressure_hPa = raw_pressure_hPa;

		// Initialize median filter window with current pressure
		pressure_median_window.clear();
		for (int i = 0; i < PRESSURE_MEDIAN_WINDOW_SIZE; ++i) {
			pressure_median_window.push_back(raw_pressure_hPa);
		}

		last_baro_update_time = current_time;
		thermal_drift_phase = 0.0;
		initialized = true;
	}

	// ========================================================================
	// BAROMETER UPDATE RATE SIMULATION - Realistic sensor timing
	// ========================================================================

	bool should_update = (current_time - last_baro_update_time) >= BARO_UPDATE_PERIOD_SEC;

	if (should_update) {
		last_baro_update_time = current_time;

		// ========================================================================
		// PRESSURE FILTERING CHAIN - Multi-stage processing in pressure domain
		// ========================================================================

		// Stage 1: Median Filter - Remove pressure spikes and outliers
		if (pressure_median_window.size() >= PRESSURE_MEDIAN_WINDOW_SIZE) {
			pressure_median_window.pop_front();
		}
		pressure_median_window.push_back(raw_pressure_hPa);

		// Calculate median pressure
		std::vector<float> sorted_window(pressure_median_window.begin(), pressure_median_window.end());
		std::sort(sorted_window.begin(), sorted_window.end());
		float median_pressure_hPa = sorted_window[sorted_window.size() / 2];

		// Stage 2: Low-pass Filter - Smooth high-frequency pressure fluctuations
		filtered_pressure_hPa = PRESSURE_FILTER_ALPHA * median_pressure_hPa +
			(1.0f - PRESSURE_FILTER_ALPHA) * filtered_pressure_hPa;

		// Stage 3: Sensor Response Time - Simulate physical sensor lag
		sensor_lagged_pressure_hPa = SENSOR_RESPONSE_TIME_ALPHA * filtered_pressure_hPa +
			(1.0f - SENSOR_RESPONSE_TIME_ALPHA) * sensor_lagged_pressure_hPa;
	}

	// ========================================================================
	// REALISTIC NOISE MODELING - Pressure domain noise + environmental effects
	// ========================================================================

	// Generate thermal drift (slow, sinusoidal variation)
	thermal_drift_phase = std::fmod(current_time, THERMAL_DRIFT_PERIOD_SEC) * 2.0 * M_PI / THERMAL_DRIFT_PERIOD_SEC;
	float thermal_drift_pa = THERMAL_DRIFT_AMPLITUDE_PA * std::sin(thermal_drift_phase);

	// Generate realistic pressure noise
	float noise_pa = pressure_noise(gen);

	// Apply noise and drift to pressure (realistic domain)
	float noisy_pressure_hPa = sensor_lagged_pressure_hPa + (thermal_drift_pa + noise_pa) * 0.01f; // Convert Pa to hPa

	// ========================================================================
	// PRESSURE QUANTIZATION - Eliminate sub-Pascal jitter
	// ========================================================================

	float quantized_pressure_hPa = std::round(noisy_pressure_hPa * 100.0f) / 100.0f; // 0.01 hPa precision

	// ========================================================================
	// DIFFERENTIAL PRESSURE CALCULATION - Realistic airspeed-based pressure
	// ========================================================================

	// Get indicated airspeed and convert to m/s
	float ias_knots = DataRefManager::getFloat("sim/flightmodel/position/indicated_airspeed");
	float ias_m_s = ias_knots * 0.514444f; // knots to m/s

	// Calculate raw dynamic pressure using ISA sea-level density
	float raw_dynamic_pressure_pa = 0.5f * DataRefManager::AirDensitySeaLevel * ias_m_s * ias_m_s;
	float raw_dynamic_pressure_hPa = raw_dynamic_pressure_pa * 0.01f; // Pa to hPa

	// Apply filtering to differential pressure for stability
	filtered_diff_pressure_hPa = DIFF_PRESSURE_FILTER_ALPHA * raw_dynamic_pressure_hPa +
		(1.0f - DIFF_PRESSURE_FILTER_ALPHA) * filtered_diff_pressure_hPa;

	// ========================================================================
	// ALTITUDE CALCULATION - Convert filtered pressure back to altitude
	// ========================================================================

	// Calculate pressure altitude from filtered pressure using ISA model
	float filtered_pressure_altitude_m = DataRefManager::calculatePressureAltitude(quantized_pressure_hPa);

	// ========================================================================
	// OUTPUT ASSIGNMENT - Populate HIL_SENSOR message
	// ========================================================================

	hil_sensor.abs_pressure = quantized_pressure_hPa;                           // Absolute pressure [hPa]
	hil_sensor.pressure_alt = filtered_pressure_altitude_m;                     // Pressure altitude [m]
	hil_sensor.diff_pressure = filtered_diff_pressure_hPa;                      // Differential pressure [hPa]

	// ========================================================================
	// OPTIONAL: DEBUG OUTPUT (uncomment for debugging)
	// ========================================================================
	/*
	static int debug_counter = 0;
	if (++debug_counter % 250 == 0) { // Debug every 5 seconds at 50Hz
		XPLMDebugString(("px4xplane: Baro - Raw: " + std::to_string(raw_pressure_hPa) +
						" hPa, Filtered: " + std::to_string(quantized_pressure_hPa) +
						" hPa, Alt: " + std::to_string(filtered_pressure_altitude_m) + " m\n").c_str());
	}
	*/
}



/**
 * @brief Sets realistic magnetometer data for the HIL_SENSOR message with advanced filtering.
 *
 * This function simulates realistic 3-axis magnetometer behavior by applying sophisticated
 * filtering, realistic noise characteristics, and environmental effects. The implementation
 * matches EKF2_MAG_NOISE expectations and provides stable magnetic field data for heading estimation.
 *
 * Key Features:
 * - Multi-stage magnetic field filtering: median → low-pass → quantization
 * - Realistic magnetometer noise modeling (matches EKF2_MAG_NOISE = 0.05 Gauss)
 * - Hard/soft iron distortion simulation for realism
 * - Temperature-dependent bias drift and scale factor changes
 * - Electrical interference simulation (motor/ESC effects)
 * - Proper coordinate frame transformation with filtering
 * - Consistent with other IMU sensor filtering approaches
 *
 * Magnetometer Physics Simulation:
 * 1. Earth magnetic field transformation to body frame
 * 2. Hard iron bias effects (constant offsets)
 * 3. Soft iron effects (scale factors and cross-coupling)
 * 4. Temperature-dependent drift and scale changes
 * 5. Electrical interference from motors/electronics
 * 6. White noise matching real magnetometer specifications
 *
 * Filter Chain (Magnetic Field):
 * 1. Raw attitude data acquisition and coordinate transformation
 * 2. Median filter: Remove magnetic field spikes and outliers
 * 3. Low-pass filter: Smooth high-frequency magnetic disturbances
 * 4. Environmental effects: Hard/soft iron, temperature, interference
 * 5. Realistic noise addition: White noise + bias drift
 * 6. Quantization: Digital magnetometer LSB simulation
 *
 * @param hil_sensor Reference to the HIL_SENSOR message where magnetic field data will be set
 */
void MAVLinkManager::setMagneticFieldData(mavlink_hil_sensor_t& hil_sensor) {
	// ========================================================================
	// CONFIGURATION PARAMETERS - Magnetometer sensor characteristics
	// ========================================================================

	// Magnetometer filtering configuration
	constexpr float MAG_FILTER_ALPHA = 0.88f;                  // Low-pass filter strength
	constexpr int MAG_MEDIAN_WINDOW_SIZE = 5;                  // Median filter window
	constexpr float MAG_QUANTIZATION_MGAUSS = 1.0f;           // Quantization in milliGauss

	// Realistic magnetometer noise characteristics (matches EKF2_MAG_NOISE = 0.05 Gauss)
	constexpr float MAG_WHITE_NOISE_STD_GAUSS = 0.015f;       // 15 mGauss white noise (realistic)
	constexpr float MAG_BIAS_DRIFT_GAUSS = 0.008f;            // 8 mGauss bias drift amplitude
	constexpr double MAG_THERMAL_PERIOD_SEC = 480.0f;         // 8-minute thermal cycle

	// Hard/soft iron distortion simulation (typical aircraft values)
	constexpr float HARD_IRON_X_GAUSS = 0.02f;                // X-axis hard iron bias
	constexpr float HARD_IRON_Y_GAUSS = -0.015f;              // Y-axis hard iron bias  
	constexpr float HARD_IRON_Z_GAUSS = 0.01f;                // Z-axis hard iron bias
	constexpr float SOFT_IRON_SCALE_X = 1.03f;                // X-axis soft iron scale
	constexpr float SOFT_IRON_SCALE_Y = 0.97f;                // Y-axis soft iron scale
	constexpr float SOFT_IRON_SCALE_Z = 1.01f;                // Z-axis soft iron scale

	// Environmental interference simulation
	constexpr float MOTOR_INTERFERENCE_MAX_GAUSS = 0.025f;    // Maximum motor interference
	constexpr float ELECTRICAL_NOISE_GAUSS = 0.005f;          // Electrical system noise

	// Magnetometer update characteristics
	constexpr double MAG_UPDATE_PERIOD_SEC = 0.01;            // 100 Hz magnetometer updates
	constexpr float MAG_BANDWIDTH_ALPHA = 0.92f;              // Sensor bandwidth limitation

	// ========================================================================
	// STATIC VARIABLES - Maintain magnetometer sensor state between function calls
	// ========================================================================

	static std::deque<float> mag_x_median_window;              // X-axis median filter
	static std::deque<float> mag_y_median_window;              // Y-axis median filter
	static std::deque<float> mag_z_median_window;              // Z-axis median filter

	static float filtered_mag_x = 0.0f;                       // Low-pass filtered X magnetic field
	static float filtered_mag_y = 0.0f;                       // Low-pass filtered Y magnetic field
	static float filtered_mag_z = 0.0f;                       // Low-pass filtered Z magnetic field

	static float sensor_mag_x = 0.0f;                         // Sensor bandwidth limited X
	static float sensor_mag_y = 0.0f;                         // Sensor bandwidth limited Y
	static float sensor_mag_z = 0.0f;                         // Sensor bandwidth limited Z

	static double last_mag_update_time = 0.0;                 // Last magnetometer update
	static double thermal_phase_x = 0.0;                      // X-axis thermal phase
	static double thermal_phase_y = M_PI / 3;                   // Y-axis thermal phase (offset)
	static double thermal_phase_z = 2 * M_PI / 3;                 // Z-axis thermal phase (offset)

	static bool mag_initialized = false;                      // Magnetometer initialization flag

	// Random number generation for realistic magnetometer noise
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::normal_distribution<float> mag_white_noise(0.0f, MAG_WHITE_NOISE_STD_GAUSS);
	static std::normal_distribution<float> electrical_noise(0.0f, ELECTRICAL_NOISE_GAUSS);

	// ========================================================================
	// RAW DATA ACQUISITION AND COORDINATE TRANSFORMATION
	// ========================================================================

	double current_time = TimeManager::getCurrentTimeSec();

	// Retrieve aircraft attitude from X-Plane simulation
	float yaw_mag = DataRefManager::getFloat("sim/flightmodel/position/mag_psi");    // Magnetic heading [degrees]
	float roll = DataRefManager::getFloat("sim/flightmodel/position/phi");           // Roll angle [degrees]  
	float pitch = DataRefManager::getFloat("sim/flightmodel/position/theta");        // Pitch angle [degrees]

	// Convert attitude angles from degrees to radians
	float yaw_rad = -yaw_mag * M_PI / 180.0f;    // Negative for NED→Body frame transform
	float roll_rad = roll * M_PI / 180.0f;       // Roll: positive = right wing down
	float pitch_rad = pitch * M_PI / 180.0f;     // Pitch: positive = nose up

	// Transform Earth magnetic field from NED frame to aircraft body frame
	Eigen::Vector3f raw_body_mag = DataRefManager::convertNEDToBody(
		DataRefManager::earthMagneticFieldNED,
		roll_rad,
		pitch_rad,
		yaw_rad
	);

	// ========================================================================
	// INITIALIZATION - Set initial magnetometer sensor states
	// ========================================================================

	if (!mag_initialized) {
		filtered_mag_x = sensor_mag_x = raw_body_mag.x();
		filtered_mag_y = sensor_mag_y = raw_body_mag.y();
		filtered_mag_z = sensor_mag_z = raw_body_mag.z();

		// Initialize median filter windows
		for (int i = 0; i < MAG_MEDIAN_WINDOW_SIZE; ++i) {
			mag_x_median_window.push_back(raw_body_mag.x());
			mag_y_median_window.push_back(raw_body_mag.y());
			mag_z_median_window.push_back(raw_body_mag.z());
		}

		last_mag_update_time = current_time;
		mag_initialized = true;
	}

	// ========================================================================
	// MAGNETOMETER UPDATE RATE SIMULATION - High-frequency sensor behavior
	// ========================================================================

	bool should_update = (current_time - last_mag_update_time) >= MAG_UPDATE_PERIOD_SEC;

	if (should_update) {
		last_mag_update_time = current_time;

		// ====================================================================
		// MAGNETIC FIELD FILTERING CHAIN - Multi-stage processing
		// ====================================================================

		// Stage 1: Median Filter - Remove magnetic field spikes and outliers
		auto updateMagMedianWindow = [](std::deque<float>& window, float new_value) -> float {
			if (window.size() >= MAG_MEDIAN_WINDOW_SIZE) {
				window.pop_front();
			}
			window.push_back(new_value);

			std::vector<float> sorted_window(window.begin(), window.end());
			std::sort(sorted_window.begin(), sorted_window.end());
			return sorted_window[sorted_window.size() / 2];
			};

		float median_mag_x = updateMagMedianWindow(mag_x_median_window, raw_body_mag.x());
		float median_mag_y = updateMagMedianWindow(mag_y_median_window, raw_body_mag.y());
		float median_mag_z = updateMagMedianWindow(mag_z_median_window, raw_body_mag.z());

		// Stage 2: Low-pass Filter - Smooth high-frequency magnetic disturbances
		filtered_mag_x = MAG_FILTER_ALPHA * median_mag_x + (1.0f - MAG_FILTER_ALPHA) * filtered_mag_x;
		filtered_mag_y = MAG_FILTER_ALPHA * median_mag_y + (1.0f - MAG_FILTER_ALPHA) * filtered_mag_y;
		filtered_mag_z = MAG_FILTER_ALPHA * median_mag_z + (1.0f - MAG_FILTER_ALPHA) * filtered_mag_z;

		// Stage 3: Sensor Bandwidth Limitation - Simulate magnetometer response time
		sensor_mag_x = MAG_BANDWIDTH_ALPHA * filtered_mag_x + (1.0f - MAG_BANDWIDTH_ALPHA) * sensor_mag_x;
		sensor_mag_y = MAG_BANDWIDTH_ALPHA * filtered_mag_y + (1.0f - MAG_BANDWIDTH_ALPHA) * sensor_mag_y;
		sensor_mag_z = MAG_BANDWIDTH_ALPHA * filtered_mag_z + (1.0f - MAG_BANDWIDTH_ALPHA) * sensor_mag_z;
	}

	// ========================================================================
	// REALISTIC MAGNETOMETER DISTORTION EFFECTS
	// ========================================================================

	// Hard Iron Effects - Constant bias offsets (aircraft ferrous materials)
	float hard_iron_mag_x = sensor_mag_x + HARD_IRON_X_GAUSS;
	float hard_iron_mag_y = sensor_mag_y + HARD_IRON_Y_GAUSS;
	float hard_iron_mag_z = sensor_mag_z + HARD_IRON_Z_GAUSS;

	// Soft Iron Effects - Scale factors and cross-coupling (aircraft materials)
	float soft_iron_mag_x = hard_iron_mag_x * SOFT_IRON_SCALE_X;
	float soft_iron_mag_y = hard_iron_mag_y * SOFT_IRON_SCALE_Y;
	float soft_iron_mag_z = hard_iron_mag_z * SOFT_IRON_SCALE_Z;

	// ========================================================================
	// ENVIRONMENTAL INTERFERENCE SIMULATION
	// ========================================================================

	// Motor/ESC interference (RPM and electrical load dependent)
	float engine_rpm_raw = DataRefManager::getFloat("...");
	float engine_rpm = (engine_rpm_raw > 0.0f) ? engine_rpm_raw : 0.0f;
	float motor_interference_factor = (engine_rpm / 2500.0f > 1.0f) ? 1.0f : engine_rpm / 2500.0f;

	float motor_interference_x = MOTOR_INTERFERENCE_MAX_GAUSS * motor_interference_factor *
		std::sin(2.0f * M_PI * 50.0f * current_time); // 50Hz motor noise
	float motor_interference_y = MOTOR_INTERFERENCE_MAX_GAUSS * motor_interference_factor *
		std::sin(2.0f * M_PI * 50.0f * current_time + M_PI / 2);
	float motor_interference_z = MOTOR_INTERFERENCE_MAX_GAUSS * motor_interference_factor *
		std::sin(2.0f * M_PI * 50.0f * current_time + M_PI);

	// Apply motor interference
	float interference_mag_x = soft_iron_mag_x + motor_interference_x;
	float interference_mag_y = soft_iron_mag_y + motor_interference_y;
	float interference_mag_z = soft_iron_mag_z + motor_interference_z;

	// ========================================================================
	// REALISTIC MAGNETOMETER NOISE MODELING
	// ========================================================================

	// Generate temperature-dependent bias drift
	thermal_phase_x = std::fmod(current_time, MAG_THERMAL_PERIOD_SEC) * 2.0 * M_PI / MAG_THERMAL_PERIOD_SEC;
	thermal_phase_y = thermal_phase_x + M_PI / 3;
	thermal_phase_z = thermal_phase_x + 2 * M_PI / 3;

	float bias_drift_x = MAG_BIAS_DRIFT_GAUSS * std::sin(thermal_phase_x);
	float bias_drift_y = MAG_BIAS_DRIFT_GAUSS * std::sin(thermal_phase_y);
	float bias_drift_z = MAG_BIAS_DRIFT_GAUSS * std::sin(thermal_phase_z);

	// Generate white noise (matches EKF2_MAG_NOISE expectations)
	float white_noise_x = mag_white_noise(gen);
	float white_noise_y = mag_white_noise(gen);
	float white_noise_z = mag_white_noise(gen);

	// Generate electrical noise
	float elec_noise_x = electrical_noise(gen);
	float elec_noise_y = electrical_noise(gen);
	float elec_noise_z = electrical_noise(gen);

	// Apply all noise sources
	float noisy_mag_x = interference_mag_x + bias_drift_x + white_noise_x + elec_noise_x;
	float noisy_mag_y = interference_mag_y + bias_drift_y + white_noise_y + elec_noise_y;
	float noisy_mag_z = interference_mag_z + bias_drift_z + white_noise_z + elec_noise_z;

	// ========================================================================
	// DIGITAL MAGNETOMETER QUANTIZATION - Simulate ADC resolution effects
	// ========================================================================

	constexpr float MGAUSS_TO_GAUSS = 0.001f;
	float quantization_step = MAG_QUANTIZATION_MGAUSS * MGAUSS_TO_GAUSS;

	float quantized_mag_x = std::round(noisy_mag_x / quantization_step) * quantization_step;
	float quantized_mag_y = std::round(noisy_mag_y / quantization_step) * quantization_step;
	float quantized_mag_z = std::round(noisy_mag_z / quantization_step) * quantization_step;

	// ========================================================================
	// OUTPUT ASSIGNMENT - Populate HIL_SENSOR message
	// ========================================================================

	hil_sensor.xmag = quantized_mag_x;  // Forward component (FRD X-axis) [Gauss]
	hil_sensor.ymag = quantized_mag_y;  // Right component (FRD Y-axis) [Gauss]
	hil_sensor.zmag = quantized_mag_z;  // Down component (FRD Z-axis) [Gauss]

	// ========================================================================
	// OPTIONAL: DEBUG OUTPUT (uncomment for debugging)
	// ========================================================================
	/*
	static int debug_counter = 0;
	if (++debug_counter % 500 == 0) { // Debug every 5 seconds at 100Hz
		float total_field = std::sqrt(quantized_mag_x*quantized_mag_x +
									 quantized_mag_y*quantized_mag_y +
									 quantized_mag_z*quantized_mag_z);
		XPLMDebugString(("px4xplane: Magnetometer - Field: [" +
						std::to_string(quantized_mag_x) + ", " +
						std::to_string(quantized_mag_y) + ", " +
						std::to_string(quantized_mag_z) +
						"] Gauss, Total: " + std::to_string(total_field) +
						" Gauss, Motor: " + std::to_string(motor_interference_factor * 100.0f) + "%\n").c_str());
	}
	*/
}





/**
 * @brief Sets realistic GPS position data for the HIL_GPS message with advanced altitude filtering.
 *
 * This function simulates realistic GPS receiver behavior by applying sophisticated filtering
 * to eliminate high-frequency noise while maintaining responsiveness to real position changes.
 *
 * Key Features:
 * - Multi-stage altitude filtering: median → low-pass → quantization
 * - Configurable filter parameters for different GPS quality scenarios
 * - Sub-10cm noise elimination as specified
 * - Realistic GPS update characteristics
 * - Self-contained with all configuration within the function
 *
 * Filter Chain (Altitude):
 * 1. Median filter: Removes outliers and spikes
 * 2. Low-pass filter: Smooths high-frequency noise
 * 3. Quantization: Eliminates sub-centimeter jitter
 *
 * @param hil_gps Reference to the mavlink_hil_gps_t structure to populate.
 */
void MAVLinkManager::setGPSPositionData(mavlink_hil_gps_t& hil_gps) {
	// ========================================================================
	// CONFIGURATION PARAMETERS - Adjustable for different GPS quality scenarios
	// ========================================================================

	// Altitude filtering configuration
	constexpr float ALT_FILTER_ALPHA = 0.7f;           // Low-pass filter strength (0.0-1.0, higher = less filtering)
	constexpr int ALT_MEDIAN_WINDOW_SIZE = 5;            // Median filter window size (3-7 recommended)
	constexpr double ALT_QUANTIZATION_CM = 1.0;          // Altitude quantization in centimeters (0.5-2.0 cm)

	// Horizontal position filtering configuration  
	constexpr float POS_FILTER_ALPHA = 0.88f;           // Lighter filtering for lat/lon (more responsive)
	constexpr double POS_QUANTIZATION_DEG = 1e-8;       // ~1mm horizontal quantization

	// GPS update rate simulation
	constexpr double GPS_UPDATE_PERIOD_SEC = 0.1;       // Typical GPS: 10 Hz (0.1s), can be 0.2s for 5Hz

	// ========================================================================
	// STATIC VARIABLES - Maintain filter state between function calls
	// ========================================================================

	static std::deque<double> alt_median_window;         // Altitude median filter window
	static double filtered_altitude = 0.0;              // Low-pass filtered altitude
	static double filtered_latitude = 0.0;              // Low-pass filtered latitude  
	static double filtered_longitude = 0.0;             // Low-pass filtered longitude
	static double last_gps_update_time = 0.0;          // Last GPS update timestamp
	static bool initialized = false;                    // Initialization flag

	// ========================================================================
	// DATA ACQUISITION - Get raw position data from X-Plane
	// ========================================================================

	double raw_latitude = DataRefManager::getDouble("sim/flightmodel/position/latitude");
	double raw_longitude = DataRefManager::getDouble("sim/flightmodel/position/longitude");
	double raw_elevation = DataRefManager::getDouble("sim/flightmodel/position/elevation");
	double current_time = TimeManager::getCurrentTimeSec();

	// ========================================================================
	// INITIALIZATION - Set initial filter states on first call
	// ========================================================================

	if (!initialized) {
		filtered_latitude = raw_latitude;
		filtered_longitude = raw_longitude;
		filtered_altitude = raw_elevation;

		// Initialize median filter window with current altitude
		alt_median_window.clear();
		for (int i = 0; i < ALT_MEDIAN_WINDOW_SIZE; ++i) {
			alt_median_window.push_back(raw_elevation);
		}

		last_gps_update_time = current_time;
		initialized = true;
	}

	// ========================================================================
	// GPS UPDATE RATE SIMULATION - Limit updates to realistic GPS frequency
	// ========================================================================

	if ((current_time - last_gps_update_time) < GPS_UPDATE_PERIOD_SEC) {
		// Use previous filtered values if update period hasn't elapsed
		// This simulates realistic GPS update rates (5-10 Hz typical)

		// Convert using previous filtered values
		hil_gps.lat = static_cast<int32_t>(filtered_latitude * 1E7);
		hil_gps.lon = static_cast<int32_t>(filtered_longitude * 1E7);
		hil_gps.alt = static_cast<int32_t>(filtered_altitude * 1000.0);
		return;
	}

	last_gps_update_time = current_time;

	// ========================================================================
	// ALTITUDE FILTERING CHAIN - Multi-stage processing for optimal results
	// ========================================================================

	// Stage 1: Median Filter - Remove outliers and spikes
	if (alt_median_window.size() >= ALT_MEDIAN_WINDOW_SIZE) {
		alt_median_window.pop_front();
	}
	alt_median_window.push_back(raw_elevation);

	// Calculate median value
	std::vector<double> sorted_window(alt_median_window.begin(), alt_median_window.end());
	std::sort(sorted_window.begin(), sorted_window.end());
	double median_altitude = sorted_window[sorted_window.size() / 2];

	// Stage 2: Low-pass Filter - Smooth high-frequency noise
	filtered_altitude = ALT_FILTER_ALPHA * median_altitude + (1.0 - ALT_FILTER_ALPHA) * filtered_altitude;

	// Stage 3: Altitude Quantization - Eliminate sub-centimeter jitter
	// This ensures noise below the specified threshold (10cm) is not felt
	double quantized_altitude = std::round(filtered_altitude / (ALT_QUANTIZATION_CM * 0.01)) * (ALT_QUANTIZATION_CM * 0.01);

	// ========================================================================
	// HORIZONTAL POSITION FILTERING - Lighter filtering for responsiveness
	// ========================================================================

	// Apply light exponential smoothing to lat/lon
	filtered_latitude = POS_FILTER_ALPHA * raw_latitude + (1.0 - POS_FILTER_ALPHA) * filtered_latitude;
	filtered_longitude = POS_FILTER_ALPHA * raw_longitude + (1.0 - POS_FILTER_ALPHA) * filtered_longitude;

	// Apply minimal quantization to eliminate floating-point noise
	double quantized_latitude = std::round(filtered_latitude / POS_QUANTIZATION_DEG) * POS_QUANTIZATION_DEG;
	double quantized_longitude = std::round(filtered_longitude / POS_QUANTIZATION_DEG) * POS_QUANTIZATION_DEG;

	// ========================================================================
	// OUTPUT CONVERSION - Convert to MAVLink GPS message format
	// ========================================================================

	hil_gps.lat = static_cast<int32_t>(quantized_latitude * 1E7);      // degrees * 1E7
	hil_gps.lon = static_cast<int32_t>(quantized_longitude * 1E7);     // degrees * 1E7  
	hil_gps.alt = static_cast<int32_t>(quantized_altitude * 1000.0);   // mm above MSL

	// Update stored filtered values for next iteration
	filtered_latitude = quantized_latitude;
	filtered_longitude = quantized_longitude;
	filtered_altitude = quantized_altitude;
}




/**
 * @brief Sets realistic GPS velocity data for the HIL_GPS message with advanced filtering.
 *
 * This function simulates realistic GPS receiver velocity behavior by applying sophisticated
 * filtering to eliminate high-frequency noise while maintaining accuracy and responsiveness.
 * The implementation uses the same filtering philosophy as GPS position for consistency.
 *
 * Key Features:
 * - Multi-stage velocity filtering: median → low-pass → quantization
 * - Realistic GPS velocity noise characteristics (lower than position noise)
 * - Proper OGL to NED coordinate transformation with filtering
 * - Consistent GPS update rate simulation with position data
 * - Self-contained configuration for different GPS receiver scenarios
 * - Matched characteristics for optimal EKF2 sensor fusion
 *
 * Coordinate System Transformation (OGL → NED):
 * - OGL: X=East, Y=Up, Z=South → NED: X=North, Y=East, Z=Down
 * - NED North (vn) = -OGL South (vz)
 * - NED East (ve)  =  OGL East (vx)
 * - NED Down (vd)  = -OGL Up (vy)
 *
 * Filter Chain (Velocity):
 * 1. Raw OGL velocity acquisition and coordinate transformation
 * 2. Median filter: Removes velocity spikes and outliers
 * 3. Low-pass filter: Smooths high-frequency velocity fluctuations
 * 4. Realistic noise addition: GPS velocity domain noise
 * 5. Quantization: Eliminates sub-cm/s jitter
 * 6. Ground speed calculation from filtered components
 *
 * @param hil_gps Reference to the mavlink_hil_gps_t structure to populate.
 */
void MAVLinkManager::setGPSVelocityData(mavlink_hil_gps_t& hil_gps) {
	// ========================================================================
	// CONFIGURATION PARAMETERS - GPS velocity receiver characteristics
	// ========================================================================

	// Velocity filtering configuration (slightly more aggressive than position)
	constexpr float VEL_FILTER_ALPHA = 0.82f;              // Low-pass filter strength
	constexpr int VEL_MEDIAN_WINDOW_SIZE = 5;               // Median filter window size
	constexpr double VEL_QUANTIZATION_CMS = 1.0;           // Velocity quantization in cm/s

	// Realistic GPS velocity noise characteristics
	constexpr float GPS_VEL_NOISE_STD_MS = 0.08f;          // GPS velocity noise: ~8 cm/s (realistic)
	constexpr float GPS_VEL_BIAS_DRIFT_MS = 0.02f;         // Slow bias drift amplitude
	constexpr double GPS_VEL_DRIFT_PERIOD_SEC = 240.0f;    // 4-minute GPS receiver thermal cycle

	// GPS update rate simulation (must match GPS position update rate)
	constexpr double GPS_UPDATE_PERIOD_SEC = 0.1;          // 10 Hz GPS updates

	// Coordinate system stability factors
	constexpr float COORD_STABILITY_FACTOR = 0.95f;        // Additional smoothing for OGL→NED conversion

	// ========================================================================
	// STATIC VARIABLES - Maintain GPS velocity state between function calls
	// ========================================================================

	static std::deque<float> vn_median_window;              // North velocity median filter
	static std::deque<float> ve_median_window;              // East velocity median filter
	static std::deque<float> vd_median_window;              // Down velocity median filter

	static float filtered_vn = 0.0f;                       // Low-pass filtered North velocity
	static float filtered_ve = 0.0f;                       // Low-pass filtered East velocity
	static float filtered_vd = 0.0f;                       // Low-pass filtered Down velocity

	static float stable_vn = 0.0f;                         // Coordinate-stable North velocity
	static float stable_ve = 0.0f;                         // Coordinate-stable East velocity
	static float stable_vd = 0.0f;                         // Coordinate-stable Down velocity

	static double last_gps_vel_update_time = 0.0;          // Last GPS velocity update
	static double vel_drift_phase_n = 0.0;                 // North velocity drift phase
	static double vel_drift_phase_e = M_PI / 4;              // East velocity drift phase (offset)
	static double vel_drift_phase_d = M_PI / 2;              // Down velocity drift phase (offset)

	static bool vel_initialized = false;                   // Velocity initialization flag

	// Random number generation for realistic GPS velocity noise
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::normal_distribution<float> gps_vel_noise(0.0f, GPS_VEL_NOISE_STD_MS);

	// ========================================================================
	// RAW DATA ACQUISITION AND COORDINATE TRANSFORMATION
	// ========================================================================

	double current_time = TimeManager::getCurrentTimeSec();

	// Get raw OGL velocities from X-Plane (in cm/s)
	float ogl_vx = DataRefManager::getFloat("sim/flightmodel/position/local_vx") * 100.0f;  // East
	float ogl_vy = DataRefManager::getFloat("sim/flightmodel/position/local_vy") * 100.0f;  // Up
	float ogl_vz = DataRefManager::getFloat("sim/flightmodel/position/local_vz") * 100.0f;  // South

	// Transform OGL to NED coordinate system
	float raw_vn = -ogl_vz;    // North = -South
	float raw_ve = ogl_vx;    // East = East
	float raw_vd = -ogl_vy;    // Down = -Up

	// ========================================================================
	// INITIALIZATION - Set initial GPS velocity states
	// ========================================================================

	if (!vel_initialized) {
		filtered_vn = stable_vn = raw_vn;
		filtered_ve = stable_ve = raw_ve;
		filtered_vd = stable_vd = raw_vd;

		// Initialize median filter windows with current velocities
		for (int i = 0; i < VEL_MEDIAN_WINDOW_SIZE; ++i) {
			vn_median_window.push_back(raw_vn);
			ve_median_window.push_back(raw_ve);
			vd_median_window.push_back(raw_vd);
		}

		last_gps_vel_update_time = current_time;
		vel_initialized = true;
	}

	// ========================================================================
	// GPS UPDATE RATE SIMULATION - Consistent with GPS position timing
	// ========================================================================

	if ((current_time - last_gps_vel_update_time) < GPS_UPDATE_PERIOD_SEC) {
		// Use previous filtered values if update period hasn't elapsed
		// This maintains consistency with GPS position update rate

		hil_gps.vn = static_cast<int16_t>(stable_vn);
		hil_gps.ve = static_cast<int16_t>(stable_ve);
		hil_gps.vd = static_cast<int16_t>(stable_vd);
		hil_gps.vel = static_cast<uint16_t>(std::sqrt(stable_vn * stable_vn +
			stable_ve * stable_ve +
			stable_vd * stable_vd));
		return;
	}

	last_gps_vel_update_time = current_time;

	// ========================================================================
	// VELOCITY FILTERING CHAIN - Multi-stage processing for stability
	// ========================================================================

	// Stage 1: Median Filter - Remove velocity spikes and outliers
	auto updateVelMedianWindow = [](std::deque<float>& window, float new_value) -> float {
		if (window.size() >= VEL_MEDIAN_WINDOW_SIZE) {
			window.pop_front();
		}
		window.push_back(new_value);

		std::vector<float> sorted_window(window.begin(), window.end());
		std::sort(sorted_window.begin(), sorted_window.end());
		return sorted_window[sorted_window.size() / 2];
		};

	float median_vn = updateVelMedianWindow(vn_median_window, raw_vn);
	float median_ve = updateVelMedianWindow(ve_median_window, raw_ve);
	float median_vd = updateVelMedianWindow(vd_median_window, raw_vd);

	// Stage 2: Low-pass Filter - Smooth high-frequency velocity fluctuations
	filtered_vn = VEL_FILTER_ALPHA * median_vn + (1.0f - VEL_FILTER_ALPHA) * filtered_vn;
	filtered_ve = VEL_FILTER_ALPHA * median_ve + (1.0f - VEL_FILTER_ALPHA) * filtered_ve;
	filtered_vd = VEL_FILTER_ALPHA * median_vd + (1.0f - VEL_FILTER_ALPHA) * filtered_vd;

	// Stage 3: Coordinate System Stabilization - Additional smoothing for OGL issues
	stable_vn = COORD_STABILITY_FACTOR * filtered_vn + (1.0f - COORD_STABILITY_FACTOR) * stable_vn;
	stable_ve = COORD_STABILITY_FACTOR * filtered_ve + (1.0f - COORD_STABILITY_FACTOR) * stable_ve;
	stable_vd = COORD_STABILITY_FACTOR * filtered_vd + (1.0f - COORD_STABILITY_FACTOR) * stable_vd;

	// ========================================================================
	// REALISTIC GPS VELOCITY NOISE MODELING
	// ========================================================================

	// Generate GPS receiver drift (slower thermal variations)
	vel_drift_phase_n = std::fmod(current_time, GPS_VEL_DRIFT_PERIOD_SEC) * 2.0 * M_PI / GPS_VEL_DRIFT_PERIOD_SEC;
	vel_drift_phase_e = vel_drift_phase_n + M_PI / 4;    // Phase offset for East
	vel_drift_phase_d = vel_drift_phase_n + M_PI / 2;    // Phase offset for Down

	float drift_vn = GPS_VEL_BIAS_DRIFT_MS * std::sin(vel_drift_phase_n);
	float drift_ve = GPS_VEL_BIAS_DRIFT_MS * std::sin(vel_drift_phase_e);
	float drift_vd = GPS_VEL_BIAS_DRIFT_MS * std::sin(vel_drift_phase_d);

	// Generate realistic GPS velocity noise (white noise)
	float noise_vn = gps_vel_noise(gen);
	float noise_ve = gps_vel_noise(gen);
	float noise_vd = gps_vel_noise(gen);

	// Apply drift and noise to stable velocities
	float noisy_vn = stable_vn + drift_vn + noise_vn;
	float noisy_ve = stable_ve + drift_ve + noise_ve;
	float noisy_vd = stable_vd + drift_vd + noise_vd;

	// ========================================================================
	// VELOCITY QUANTIZATION - Eliminate sub-cm/s digital jitter
	// ========================================================================

	float quantized_vn = std::round(noisy_vn / VEL_QUANTIZATION_CMS) * VEL_QUANTIZATION_CMS;
	float quantized_ve = std::round(noisy_ve / VEL_QUANTIZATION_CMS) * VEL_QUANTIZATION_CMS;
	float quantized_vd = std::round(noisy_vd / VEL_QUANTIZATION_CMS) * VEL_QUANTIZATION_CMS;

	// ========================================================================
	// GROUND SPEED CALCULATION - From filtered velocity components
	// ========================================================================

	// Calculate ground speed from horizontal components (North and East only)
	float horizontal_speed = std::sqrt(quantized_vn * quantized_vn + quantized_ve * quantized_ve);

	// Calculate total 3D speed including vertical component
	float total_speed = std::sqrt(quantized_vn * quantized_vn +
		quantized_ve * quantized_ve +
		quantized_vd * quantized_vd);

	// ========================================================================
	// OUTPUT ASSIGNMENT - Populate HIL_GPS message
	// ========================================================================

	hil_gps.vn = static_cast<int16_t>(quantized_vn);       // North velocity [cm/s]
	hil_gps.ve = static_cast<int16_t>(quantized_ve);       // East velocity [cm/s]
	hil_gps.vd = static_cast<int16_t>(quantized_vd);       // Down velocity [cm/s]
	hil_gps.vel = static_cast<uint16_t>(total_speed);      // Ground speed [cm/s]

	// Update stored stable values for next iteration
	stable_vn = quantized_vn;
	stable_ve = quantized_ve;
	stable_vd = quantized_vd;

	// ========================================================================
	// OPTIONAL: DEBUG OUTPUT (uncomment for debugging)
	// ========================================================================
	/*
	static int debug_counter = 0;
	if (++debug_counter % 50 == 0) { // Debug every 5 seconds at 10Hz
		XPLMDebugString(("px4xplane: GPS Vel - Raw: [" +
						std::to_string(raw_vn) + ", " + std::to_string(raw_ve) + ", " + std::to_string(raw_vd) +
						"] Filtered: [" +
						std::to_string(quantized_vn) + ", " + std::to_string(quantized_ve) + ", " + std::to_string(quantized_vd) +
						"] cm/s, Speed: " + std::to_string(total_speed) + " cm/s\n").c_str());
	}
	*/
}


/**
 * @brief Sets realistic GPS heading data for the HIL_GPS message with advanced filtering.
 *
 * This function simulates realistic GPS receiver heading behavior by applying sophisticated
 * filtering and speed-dependent accuracy modeling. The implementation matches the filtering
 * philosophy used for GPS position and velocity for consistency.
 *
 * Key Features:
 * - Speed-dependent heading accuracy (poor at low speeds, good at high speeds)
 * - Course Over Ground (COG) calculated from filtered GPS velocity components
 * - Realistic GPS heading noise characteristics
 * - Consistent 10Hz GPS update rate simulation
 * - Proper handling of stationary and low-speed conditions
 * - Self-contained configuration for different GPS receiver scenarios
 *
 * GPS Heading Characteristics:
 * - COG: Calculated from velocity vector (realistic GPS behavior)
 * - Accuracy degrades below ~2 m/s ground speed (typical GPS limitation)
 * - Heading noise increases exponentially at low speeds
 * - Filtered with same update rate as GPS position/velocity
 *
 * @param hil_gps Reference to the mavlink_hil_gps_t structure to populate.
 */
void MAVLinkManager::setGPSHeadingData(mavlink_hil_gps_t& hil_gps) {
	// ========================================================================
	// CONFIGURATION PARAMETERS - GPS heading receiver characteristics
	// ========================================================================

	// GPS heading filtering configuration
	constexpr float HEADING_FILTER_ALPHA = 0.85f;              // Low-pass filter for heading
	constexpr float COG_FILTER_ALPHA = 0.80f;                  // Course over ground filtering
	constexpr double HEADING_QUANTIZATION_DEG = 0.01;          // Heading precision (0.01°)

	// Speed-dependent accuracy modeling (realistic GPS behavior)
	constexpr float MIN_SPEED_FOR_RELIABLE_COG = 2.0f;         // m/s - minimum speed for good COG
	constexpr float MAX_HEADING_NOISE_DEG = 15.0f;             // Maximum heading noise at zero speed
	constexpr float MIN_HEADING_NOISE_DEG = 0.5f;              // Minimum heading noise at high speed
	constexpr float SPEED_NOISE_ROLLOFF_FACTOR = 3.0f;         // Speed-dependent noise rolloff

	// GPS update rate simulation (matches GPS position/velocity)
	constexpr double GPS_UPDATE_PERIOD_SEC = 0.1;              // 10 Hz GPS updates

	// Heading drift characteristics
	constexpr float HEADING_BIAS_DRIFT_DEG = 0.2f;             // Slow heading bias drift
	constexpr double HEADING_DRIFT_PERIOD_SEC = 180.0f;        // 3-minute GPS heading drift cycle

	// ========================================================================
	// STATIC VARIABLES - Maintain GPS heading state between function calls
	// ========================================================================

	static float filtered_cog_deg = 0.0f;                      // Filtered course over ground
	static float filtered_yaw_deg = 0.0f;                      // Filtered magnetic heading
	static double last_gps_heading_update_time = 0.0;          // Last GPS heading update
	static double heading_drift_phase = 0.0;                   // Heading drift phase tracking
	static bool heading_initialized = false;                   // Heading initialization flag

	// Random number generation for realistic GPS heading noise
	static std::random_device rd;
	static std::mt19937 gen(rd());

	// ========================================================================
	// RAW DATA ACQUISITION AND PREPROCESSING
	// ========================================================================

	double current_time = TimeManager::getCurrentTimeSec();

	// Get GPS velocity components (already filtered from setGPSVelocityData)
	float gps_vn_ms = hil_gps.vn / 100.0f;  // Convert cm/s to m/s
	float gps_ve_ms = hil_gps.ve / 100.0f;  // Convert cm/s to m/s
	float gps_vd_ms = hil_gps.vd / 100.0f;  // Convert cm/s to m/s

	// Calculate ground speed from horizontal components
	float ground_speed_ms = std::sqrt(gps_vn_ms * gps_vn_ms + gps_ve_ms * gps_ve_ms);

	// Calculate Course Over Ground from GPS velocity (realistic method)
	float calculated_cog_deg = 0.0f;
	if (ground_speed_ms > 0.1f) {  // Avoid division by zero
		calculated_cog_deg = std::atan2(gps_ve_ms, gps_vn_ms) * 180.0f / M_PI;
		if (calculated_cog_deg < 0.0f) {
			calculated_cog_deg += 360.0f;  // Ensure 0-360° range
		}
	}
	else {
		// At very low speeds, maintain previous COG (GPS behavior)
		calculated_cog_deg = filtered_cog_deg;
	}

	// Get aircraft magnetic heading from X-Plane
	float raw_yaw_deg = DataRefManager::getFloat("sim/flightmodel/position/mag_psi");

	// ========================================================================
	// INITIALIZATION - Set initial GPS heading states
	// ========================================================================

	if (!heading_initialized) {
		filtered_cog_deg = calculated_cog_deg;
		filtered_yaw_deg = raw_yaw_deg;
		last_gps_heading_update_time = current_time;
		heading_initialized = true;
	}

	// ========================================================================
	// GPS UPDATE RATE SIMULATION - Consistent with GPS position/velocity timing
	// ========================================================================

	if ((current_time - last_gps_heading_update_time) < GPS_UPDATE_PERIOD_SEC) {
		// Use previous filtered values if update period hasn't elapsed
		uint16_t cached_cog = static_cast<uint16_t>(filtered_cog_deg * 100.0f);
		uint16_t cached_yaw = static_cast<uint16_t>(filtered_yaw_deg * 100.0f);

		// Apply proper 0-360° wrapping
		hil_gps.cog = (cached_cog >= 36000) ? (cached_cog - 36000) : cached_cog;
		hil_gps.yaw = (cached_yaw >= 36000) ? (cached_yaw - 36000) : cached_yaw;
		return;
	}

	last_gps_heading_update_time = current_time;

	// ========================================================================
	// HEADING FILTERING - Apply low-pass filtering for stability
	// ========================================================================

	// Handle angular wrapping for COG filtering
	float cog_diff = calculated_cog_deg - filtered_cog_deg;
	if (cog_diff > 180.0f) cog_diff -= 360.0f;
	if (cog_diff < -180.0f) cog_diff += 360.0f;
	filtered_cog_deg += COG_FILTER_ALPHA * cog_diff;

	// Normalize COG to 0-360°
	if (filtered_cog_deg < 0.0f) filtered_cog_deg += 360.0f;
	if (filtered_cog_deg >= 360.0f) filtered_cog_deg -= 360.0f;

	// Handle angular wrapping for yaw filtering
	float yaw_diff = raw_yaw_deg - filtered_yaw_deg;
	if (yaw_diff > 180.0f) yaw_diff -= 360.0f;
	if (yaw_diff < -180.0f) yaw_diff += 360.0f;
	filtered_yaw_deg += HEADING_FILTER_ALPHA * yaw_diff;

	// Normalize yaw to 0-360°
	if (filtered_yaw_deg < 0.0f) filtered_yaw_deg += 360.0f;
	if (filtered_yaw_deg >= 360.0f) filtered_yaw_deg -= 360.0f;

	// ========================================================================
	// REALISTIC GPS HEADING NOISE MODELING - Speed-dependent accuracy
	// ========================================================================

	// Calculate speed-dependent noise level (GPS physics)
	float speed_factor = std::exp(-ground_speed_ms / SPEED_NOISE_ROLLOFF_FACTOR);
	float heading_noise_std = MIN_HEADING_NOISE_DEG +
		(MAX_HEADING_NOISE_DEG - MIN_HEADING_NOISE_DEG) * speed_factor;

	// Generate heading drift (GPS receiver thermal/clock drift)
	heading_drift_phase = std::fmod(current_time, HEADING_DRIFT_PERIOD_SEC) *
		2.0 * M_PI / HEADING_DRIFT_PERIOD_SEC;
	float heading_drift_deg = HEADING_BIAS_DRIFT_DEG * std::sin(heading_drift_phase);

	// Generate white noise for both COG and yaw
	std::normal_distribution<float> heading_noise(0.0f, heading_noise_std);
	float cog_noise = heading_noise(gen);
	float yaw_noise = heading_noise(gen);

	// Apply drift and noise
	float noisy_cog_deg = filtered_cog_deg + heading_drift_deg + cog_noise;
	float noisy_yaw_deg = filtered_yaw_deg + heading_drift_deg * 0.7f + yaw_noise;  // Different drift correlation

	// ========================================================================
	// HEADING QUANTIZATION - GPS receiver digital precision
	// ========================================================================

	float quantized_cog_deg = std::round(noisy_cog_deg / HEADING_QUANTIZATION_DEG) * HEADING_QUANTIZATION_DEG;
	float quantized_yaw_deg = std::round(noisy_yaw_deg / HEADING_QUANTIZATION_DEG) * HEADING_QUANTIZATION_DEG;

	// Ensure proper 0-360° range after noise and quantization
	if (quantized_cog_deg < 0.0f) quantized_cog_deg += 360.0f;
	if (quantized_cog_deg >= 360.0f) quantized_cog_deg -= 360.0f;
	if (quantized_yaw_deg < 0.0f) quantized_yaw_deg += 360.0f;
	if (quantized_yaw_deg >= 360.0f) quantized_yaw_deg -= 360.0f;

	// ========================================================================
	// OUTPUT CONVERSION AND ASSIGNMENT
	// ========================================================================

	// Convert to centidegrees (0.01 degree units) as required by MAVLink
	uint16_t cog_centideg = static_cast<uint16_t>(quantized_cog_deg * 100.0f);
	uint16_t yaw_centideg = static_cast<uint16_t>(quantized_yaw_deg * 100.0f);

	// Apply proper range limiting and assign to HIL_GPS message
	hil_gps.cog = (cog_centideg >= 36000) ? 35999 : cog_centideg;
	hil_gps.yaw = (yaw_centideg >= 36000) ? 35999 : yaw_centideg;

	// Update stored filtered values for next iteration
	filtered_cog_deg = quantized_cog_deg;
	filtered_yaw_deg = quantized_yaw_deg;

	// ========================================================================
	// OPTIONAL: DEBUG OUTPUT (uncomment for debugging)
	// ========================================================================
	/*
	static int debug_counter = 0;
	if (++debug_counter % 50 == 0) { // Debug every 5 seconds at 10Hz
		XPLMDebugString(("px4xplane: GPS Heading - COG: " + std::to_string(quantized_cog_deg) +
						"°, Yaw: " + std::to_string(quantized_yaw_deg) +
						"°, Speed: " + std::to_string(ground_speed_ms) +
						" m/s, Noise: " + std::to_string(heading_noise_std) + "°\n").c_str());
	}
	*/
}


/**
 * @brief Sets realistic GPS accuracy data for the HIL_GPS message with dynamic characteristics.
 *
 * This function simulates realistic GPS receiver accuracy behavior by modeling the actual
 * factors that affect GPS precision: satellite geometry, atmospheric conditions, multipath,
 * and ionospheric activity. The accuracy values correlate with the noise levels and filtering
 * applied in the position, velocity, and heading functions for complete consistency.
 *
 * Key Features:
 * - Dynamic accuracy based on satellite geometry (HDOP/VDOP simulation)
 * - Environmental factors affecting precision (atmospheric, multipath)
 * - Realistic satellite count variation (6-14 typically visible)
 * - Correlated accuracy values that match actual sensor performance
 * - Time-varying conditions reflecting real GPS behavior
 * - Consistent with PX4 EKF2 accuracy requirements
 *
 * GPS Accuracy Modeling:
 * - EPH: Horizontal position accuracy (correlates with position filtering)
 * - EPV: Vertical position accuracy (correlates with altitude noise)
 * - Satellite count: Realistic variation based on location and time
 * - HDOP/VDOP effects: Geometric dilution of precision simulation
 *
 * @param hil_gps Reference to the mavlink_hil_gps_t structure to populate.
 */
void MAVLinkManager::setGPSAccuracyData(mavlink_hil_gps_t& hil_gps) {
	// ========================================================================
	// CONFIGURATION PARAMETERS - GPS accuracy characteristics
	// ========================================================================

	// Base accuracy levels (consistent with our filtering noise levels)
	constexpr float BASE_HORIZONTAL_ACCURACY_M = 0.8f;         // Base EPH in meters
	constexpr float BASE_VERTICAL_ACCURACY_M = 1.2f;           // Base EPV in meters
	constexpr int BASE_SATELLITE_COUNT = 12;                   // Typical satellite count

	// Dynamic accuracy variation parameters
	constexpr float ACCURACY_VARIATION_FACTOR = 0.3f;          // ±30% accuracy variation
	constexpr int SATELLITE_VARIATION_RANGE = 1;               // ±4 satellites variation
	constexpr double HDOP_CYCLE_PERIOD_SEC = 720.0;           // 12-minute HDOP cycle
	constexpr double VDOP_CYCLE_PERIOD_SEC = 900.0;           // 15-minute VDOP cycle

	// Environmental degradation factors
	constexpr float IONOSPHERIC_DEGRADATION_MAX = 1.8f;       // Max ionospheric degradation
	constexpr float MULTIPATH_DEGRADATION_MAX = 1.4f;         // Max multipath degradation
	constexpr double IONOSPHERIC_CYCLE_SEC = 1800.0;          // 30-minute iono cycle
	constexpr double MULTIPATH_CYCLE_SEC = 360.0;             // 6-minute multipath cycle

	// Accuracy limits (ensure we stay within reasonable bounds)
	constexpr float MIN_EPH_CM = 50;                           // Minimum EPH: 0.5m
	constexpr float MAX_EPH_CM = 300;                          // Maximum EPH: 3.0m  
	constexpr float MIN_EPV_CM = 80;                           // Minimum EPV: 0.8m
	constexpr float MAX_EPV_CM = 400;                          // Maximum EPV: 4.0m
	constexpr int MIN_SATELLITES = 10;                          // Minimum for 3D fix
	constexpr int MAX_SATELLITES = 16;                         // Maximum realistic

	// ========================================================================
	// STATIC VARIABLES - Maintain GPS accuracy state between function calls
	// ========================================================================

	static double accuracy_update_time = 0.0;                  // Last accuracy update
	static float cached_eph_cm = BASE_HORIZONTAL_ACCURACY_M * 100.0f;
	static float cached_epv_cm = BASE_VERTICAL_ACCURACY_M * 100.0f;
	static uint16_t cached_satellite_count = BASE_SATELLITE_COUNT;
	static bool accuracy_initialized = false;

	// Random number generation for accuracy variation
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::normal_distribution<float> accuracy_variation(0.0f, ACCURACY_VARIATION_FACTOR);
	static std::uniform_int_distribution<int> satellite_variation(-SATELLITE_VARIATION_RANGE, SATELLITE_VARIATION_RANGE);

	// ========================================================================
	// ACCURACY UPDATE RATE CONTROL - Update less frequently than GPS data
	// ========================================================================

	double current_time = TimeManager::getCurrentTimeSec();
	constexpr double ACCURACY_UPDATE_PERIOD_SEC = 1.0;         // Update accuracy every 1 second

	if (!accuracy_initialized || (current_time - accuracy_update_time) >= ACCURACY_UPDATE_PERIOD_SEC) {
		accuracy_update_time = current_time;
		accuracy_initialized = true;

		// ====================================================================
		// SATELLITE GEOMETRY SIMULATION - HDOP/VDOP effects
		// ====================================================================

		// Simulate geometric dilution of precision (HDOP/VDOP) cycles
		double hdop_phase = std::fmod(current_time, HDOP_CYCLE_PERIOD_SEC) * 2.0 * M_PI / HDOP_CYCLE_PERIOD_SEC;
		double vdop_phase = std::fmod(current_time, VDOP_CYCLE_PERIOD_SEC) * 2.0 * M_PI / VDOP_CYCLE_PERIOD_SEC;

		float hdop_factor = 1.0f + 0.4f * std::sin(hdop_phase);    // HDOP varies 0.6 to 1.4
		float vdop_factor = 1.0f + 0.6f * std::sin(vdop_phase);    // VDOP varies 0.4 to 1.6

		// ====================================================================
		// ENVIRONMENTAL DEGRADATION SIMULATION
		// ====================================================================

		// Ionospheric activity simulation (affects all signals)
		double iono_phase = std::fmod(current_time, IONOSPHERIC_CYCLE_SEC) * 2.0 * M_PI / IONOSPHERIC_CYCLE_SEC;
		float iono_degradation = 1.0f + (IONOSPHERIC_DEGRADATION_MAX - 1.0f) *
			(0.5f + 0.5f * std::sin(iono_phase));

		// Multipath effects simulation (location-dependent)
		double multipath_phase = std::fmod(current_time, MULTIPATH_CYCLE_SEC) * 2.0 * M_PI / MULTIPATH_CYCLE_SEC;
		float multipath_degradation = 1.0f + (MULTIPATH_DEGRADATION_MAX - 1.0f) *
			(0.3f + 0.3f * std::sin(multipath_phase));

		// ====================================================================
		// DYNAMIC ACCURACY CALCULATION
		// ====================================================================

		// Calculate dynamic horizontal accuracy (EPH)
		float dynamic_eph_m = BASE_HORIZONTAL_ACCURACY_M * hdop_factor * iono_degradation * multipath_degradation;
		dynamic_eph_m += accuracy_variation(gen) * BASE_HORIZONTAL_ACCURACY_M;  // Add random variation

		// Calculate dynamic vertical accuracy (EPV) 
		float dynamic_epv_m = BASE_VERTICAL_ACCURACY_M * vdop_factor * iono_degradation;
		dynamic_epv_m += accuracy_variation(gen) * BASE_VERTICAL_ACCURACY_M;    // Add random variation

		// Apply limits and convert to centimeters
		cached_eph_cm = std::clamp(dynamic_eph_m * 100.0f, MIN_EPH_CM, MAX_EPH_CM);
		cached_epv_cm = std::clamp(dynamic_epv_m * 100.0f, MIN_EPV_CM, MAX_EPV_CM);

		// ====================================================================
		// DYNAMIC SATELLITE COUNT SIMULATION
		// ====================================================================

		// Base satellite count with geometric and environmental variations
		int dynamic_satellite_count = BASE_SATELLITE_COUNT;

		// HDOP affects visible satellite count (poor geometry = fewer usable satellites)
		if (hdop_factor > 1.2f) {
			dynamic_satellite_count -= 1;
		}

		// Ionospheric activity can mask weak satellites
		if (iono_degradation > 1.4f) {
			dynamic_satellite_count -= 1;
		}

		// Add random variation
		dynamic_satellite_count += satellite_variation(gen);

		// Apply limits
		cached_satellite_count = std::clamp(dynamic_satellite_count, MIN_SATELLITES, MAX_SATELLITES);
	}

	// ========================================================================
	// OUTPUT ASSIGNMENT - Use cached values for consistent reporting
	// ========================================================================

	hil_gps.eph = static_cast<uint16_t>(cached_eph_cm);         // Horizontal accuracy [cm]
	hil_gps.epv = static_cast<uint16_t>(cached_epv_cm);         // Vertical accuracy [cm]
	hil_gps.satellites_visible = cached_satellite_count;        // Number of visible satellites

	// ========================================================================
	// OPTIONAL: DEBUG OUTPUT (uncomment for debugging)
	// ========================================================================
	/*
	static int debug_counter = 0;
	if (++debug_counter % 100 == 0) { // Debug every 10 seconds at 10Hz
		XPLMDebugString(("px4xplane: GPS Accuracy - EPH: " + std::to_string(cached_eph_cm/100.0f) +
						"m, EPV: " + std::to_string(cached_epv_cm/100.0f) +
						"m, Satellites: " + std::to_string(cached_satellite_count) + "\n").c_str());
	}
	*/
}

/**
 * @brief Sets realistic GPS time and fix type data for the HIL_GPS message with dynamic behavior.
 *
 * This function simulates realistic GPS receiver fix behavior by modeling conditions that
 * affect GPS fix quality and availability. The fix type varies based on satellite count,
 * accuracy, and environmental conditions, providing realistic GPS receiver behavior.
 *
 * Key Features:
 * - Dynamic fix type based on satellite availability and accuracy
 * - Realistic fix degradation scenarios (3D → 2D → No Fix)
 * - Proper time synchronization (X-Plane or system time)
 * - Environmental factors affecting fix quality
 * - Consistent with satellite count and accuracy modeling
 *
 * GPS Fix Types:
 * - 0: No Fix (< 4 satellites or very poor accuracy)
 * - 2: 2D Fix (4-5 satellites or poor vertical accuracy)
 * - 3: 3D Fix (≥ 6 satellites with good accuracy)
 * - 6: RTK Fixed (simulation can optionally provide high accuracy)
 *
 * @param hil_gps Reference to the mavlink_hil_gps_t structure to populate.
 */
void MAVLinkManager::setGPSTimeAndFix(mavlink_hil_gps_t& hil_gps) {
	// ========================================================================
	// CONFIGURATION PARAMETERS - GPS fix characteristics
	// ========================================================================

	// Fix type thresholds (realistic GPS receiver behavior)
	constexpr int MIN_SATELLITES_3D_FIX = 6;                   // Minimum satellites for 3D fix
	constexpr int MIN_SATELLITES_2D_FIX = 4;                   // Minimum satellites for 2D fix
	constexpr float MAX_EPH_3D_FIX_M = 5.0f;                   // Max horizontal accuracy for 3D fix
	constexpr float MAX_EPV_3D_FIX_M = 8.0f;                   // Max vertical accuracy for 3D fix
	constexpr float MAX_EPH_2D_FIX_M = 10.0f;                  // Max horizontal accuracy for 2D fix

	// Fix degradation simulation
	constexpr double FIX_DEGRADATION_PERIOD_SEC = 1200.0;      // 20-minute fix variation cycle
	constexpr float FIX_DEGRADATION_PROBABILITY = 0.02f;       // 2% chance of temporary fix loss

	// High accuracy mode simulation (optional RTK-like behavior)
	constexpr bool ENABLE_HIGH_ACCURACY_MODE = false;          // Set true for RTK simulation
	constexpr float RTK_ACCURACY_THRESHOLD_CM = 10.0f;         // 10cm threshold for RTK fix

	// ========================================================================
	// STATIC VARIABLES - Maintain GPS fix state between function calls  
	// ========================================================================

	static uint8_t last_fix_type = 3;                          // Previous fix type
	static double fix_degradation_start_time = 0.0;            // When fix degradation started
	static bool in_degradation_event = false;                  // Currently in degradation event
	static double last_fix_update_time = 0.0;                  // Last fix evaluation time

	// Random number generation for fix degradation events
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::uniform_real_distribution<float> degradation_chance(0.0f, 1.0f);

	// ========================================================================
	// TIME SYNCHRONIZATION - Use configured time source
	// ========================================================================

	if (ConfigManager::USE_XPLANE_TIME) {
		hil_gps.time_usec = static_cast<uint64_t>(DataRefManager::getFloat("sim/time/total_flight_time_sec") * 1e6);
	}
	else {
		hil_gps.time_usec = TimeManager::getCurrentTimeUsec();
	}

	// ========================================================================
	// DYNAMIC FIX TYPE DETERMINATION
	// ========================================================================

	double current_time = TimeManager::getCurrentTimeSec();
	constexpr double FIX_UPDATE_PERIOD_SEC = 0.5;              // Update fix type every 0.5 seconds

	if ((current_time - last_fix_update_time) >= FIX_UPDATE_PERIOD_SEC) {
		last_fix_update_time = current_time;

		// Get current accuracy and satellite data (from setGPSAccuracyData)
		uint16_t current_satellites = hil_gps.satellites_visible;
		float current_eph_m = hil_gps.eph / 100.0f;             // Convert cm to m
		float current_epv_m = hil_gps.epv / 100.0f;             // Convert cm to m

		// ====================================================================
		// FIX DEGRADATION EVENT SIMULATION
		// ====================================================================

		// Check for random fix degradation events (realistic GPS behavior)
		if (!in_degradation_event && degradation_chance(gen) < FIX_DEGRADATION_PROBABILITY) {
			in_degradation_event = true;
			fix_degradation_start_time = current_time;
		}

		// End degradation event after 10-30 seconds
		if (in_degradation_event && (current_time - fix_degradation_start_time) > (10.0 + degradation_chance(gen) * 20.0)) {
			in_degradation_event = false;
		}

		// ====================================================================
		// FIX TYPE CALCULATION BASED ON CONDITIONS
		// ====================================================================

		uint8_t calculated_fix_type = 0;  // Start with No Fix

		if (in_degradation_event) {
			// During degradation events, force lower fix quality
			if (current_satellites >= MIN_SATELLITES_2D_FIX && current_eph_m <= MAX_EPH_2D_FIX_M) {
				calculated_fix_type = 2;  // 2D Fix (degraded)
			}
			else {
				calculated_fix_type = 0;  // No Fix (severe degradation)
			}
		}
		else {
			// Normal fix determination based on satellite count and accuracy
			if (current_satellites >= MIN_SATELLITES_3D_FIX &&
				current_eph_m <= MAX_EPH_3D_FIX_M &&
				current_epv_m <= MAX_EPV_3D_FIX_M) {

				// Check for high accuracy mode (RTK simulation)
				if (ENABLE_HIGH_ACCURACY_MODE &&
					current_eph_m <= (RTK_ACCURACY_THRESHOLD_CM / 100.0f) &&
					current_epv_m <= (RTK_ACCURACY_THRESHOLD_CM / 100.0f)) {
					calculated_fix_type = 6;  // RTK Fixed
				}
				else {
					calculated_fix_type = 3;  // 3D Fix
				}

			}
			else if (current_satellites >= MIN_SATELLITES_2D_FIX &&
				current_eph_m <= MAX_EPH_2D_FIX_M) {
				calculated_fix_type = 2;  // 2D Fix
			}
			else {
				calculated_fix_type = 0;  // No Fix
			}
		}

		// Apply some hysteresis to prevent rapid fix type changes
		if (calculated_fix_type != last_fix_type) {
			// Allow immediate upgrades, but delay downgrades slightly for stability
			if (calculated_fix_type > last_fix_type ||
				(current_time - last_fix_update_time) > 2.0) {  // 2-second delay for downgrades
				last_fix_type = calculated_fix_type;
			}
		}
	}

	// ========================================================================
	// OUTPUT ASSIGNMENT
	// ========================================================================

	hil_gps.fix_type = last_fix_type;

	// ========================================================================
	// OPTIONAL: DEBUG OUTPUT (uncomment for debugging)
	// ========================================================================
	/*
	static int debug_counter = 0;
	if (++debug_counter % 20 == 0) { // Debug every 2 seconds at 10Hz
		std::string fix_type_str;
		switch(last_fix_type) {
			case 0: fix_type_str = "No Fix"; break;
			case 2: fix_type_str = "2D Fix"; break;
			case 3: fix_type_str = "3D Fix"; break;
			case 6: fix_type_str = "RTK Fixed"; break;
			default: fix_type_str = "Unknown"; break;
		}
		XPLMDebugString(("px4xplane: GPS Fix - Type: " + fix_type_str +
						", Satellites: " + std::to_string(hil_gps.satellites_visible) +
						", EPH: " + std::to_string(hil_gps.eph/100.0f) + "m" +
						(in_degradation_event ? " [DEGRADED]" : "") + "\n").c_str());
	}
	*/
}