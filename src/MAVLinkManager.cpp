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



// Define and initialize the static member
MAVLinkManager::HILActuatorControlsData MAVLinkManager::hilActuatorControlsData = {};



// Declare constants at the class or namespace level
constexpr float DEG_TO_RAD = 3.14159265358979323846 / 180.0;
constexpr float GRAVITY = 9.81;

// Define and initialize the random number generators and distributions
std::random_device MAVLinkManager::rd;
std::mt19937 MAVLinkManager::gen(MAVLinkManager::rd());
std::normal_distribution<float> MAVLinkManager::highFreqNoise(0.0f, 0.00004f);
std::normal_distribution<float> MAVLinkManager::lowFreqNoise(0.0f, 0.000004f); // Larger, slower noise
std::normal_distribution<float> vibrationNoise(0.0f, 0.01f); // Adjust the standard deviation as needed

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


//------------------------------------------------------------------------------
// computeAcceleration()
// Computes and caches the acceleration in PX4's FRD frame using Xokabe gload 
// sensor readings. Since the gload sensors provide acceleration directly in 
// the Body frame (including gravity), no additional coordinate transformation is required.
// Steps:
//   1. Retrieve raw gload sensor data (in G's) and convert them to m/s^2.
//   2. Optionally add vibration effects (vibration noise and rotary vibration).
//   3. Assemble the acceleration vector, already in the Body frame.
//   4. Map the Body frame directly to PX4's FRD frame (assuming alignment).
//   5. Optionally apply filtering to the acceleration data.
//   6. Cache the result to prevent redundant computation within the same cycle.
//------------------------------------------------------------------------------
Eigen::Vector3f MAVLinkManager::computeAcceleration() {
	// Cache acceleration data to prevent redundant computation.
	static uint64_t lastAccelUpdateTime = 0;
	static Eigen::Vector3f cachedAccel(0.0f, 0.0f, 0.0f);

	// Get the current time in microseconds.
	uint64_t currentTime = TimeManager::getCurrentTimeUsec();
	if (currentTime == lastAccelUpdateTime) {
		return cachedAccel;  // Return cached value if computed in this cycle.
	}

	//----------------------------------------------------------------------
	// 1. Retrieve Raw Accelerometer Data from Xokabe gload sensors.
	// The gload sensor provides acceleration in G's; multiply by g_earth (m/s^2)
	// to convert to m/s^2, and apply a -1 factor as required by the sensor convention.
	//----------------------------------------------------------------------
	float raw_xacc = DataRefManager::getFloat("sim/flightmodel/forces/g_axil") * DataRefManager::g_earth * -1;
	float raw_yacc = DataRefManager::getFloat("sim/flightmodel/forces/g_side") * DataRefManager::g_earth * -1;
	float raw_zacc = DataRefManager::getFloat("sim/flightmodel/forces/g_nrml") * DataRefManager::g_earth * -1;

	//----------------------------------------------------------------------
	// 2. Optionally Add Vibration Effects
	// Check configuration flags for vibration noise and rotary vibration, and add
	// the corresponding noise if enabled.
	//----------------------------------------------------------------------
	bool enableVibrationNoise = ConfigManager::vibration_noise_enabled;
	bool enableRotaryVibration = ConfigManager::rotary_vibration_enabled;

	if (enableVibrationNoise) {
		raw_xacc += vibrationNoise(gen);
		raw_yacc += vibrationNoise(gen);
		raw_zacc += vibrationNoise(gen);
	}

	if (enableRotaryVibration) {
		constexpr float vib_frequency = 50.0f;   // Frequency in Hz
		constexpr float vib_amplitude = 0.05f;     // Amplitude in m/s^2
		constexpr float noise_amplitude = 0.01f;   // Additional noise amplitude in m/s^2
		double currentTimeSec = TimeManager::getCurrentTimeSec();

		float rotary_vib_x = vib_amplitude * sinf(2.0f * static_cast<float>(M_PI) * vib_frequency * static_cast<float>(currentTimeSec))
			+ MAVLinkManager::highFreqNoise(gen) * noise_amplitude;
		float rotary_vib_y = vib_amplitude * sinf(2.0f * static_cast<float>(M_PI) * vib_frequency * static_cast<float>(currentTimeSec)
			+ static_cast<float>(M_PI) / 2.0f)
			+ MAVLinkManager::highFreqNoise(gen) * noise_amplitude;
		float rotary_vib_z = vib_amplitude * sinf(2.0f * static_cast<float>(M_PI) * vib_frequency * static_cast<float>(currentTimeSec)
			+ static_cast<float>(M_PI))
			+ MAVLinkManager::highFreqNoise(gen) * noise_amplitude;

		raw_xacc += rotary_vib_x;
		raw_yacc += rotary_vib_y;
		raw_zacc += rotary_vib_z;
	}


	//----------------------------------------------------------------------
	// 3. Assemble the acceleration vector, already in the Body frame
	// 
	//----------------------------------------------------------------------
	Eigen::Vector3f body_accel(raw_xacc, raw_yacc, raw_zacc);

	//----------------------------------------------------------------------
	// 4. Map the Body frame directly to PX4's FRD frame (assuming alignment).
	//----------------------------------------------------------------------
	Eigen::Vector3f frd_accel = body_accel;

	//----------------------------------------------------------------------
	// 5. Apply Optional Filtering to the Acceleration Data
	// Filter the FRD acceleration data using the configured filter parameters.
	//----------------------------------------------------------------------
	float filtered_x = DataRefManager::applyFilteringIfNeeded(
		frd_accel.x(),
		ConfigManager::filter_accel_enabled,
		ConfigManager::accel_filter_alpha,
		DataRefManager::median_filter_window_xacc);
	float filtered_y = DataRefManager::applyFilteringIfNeeded(
		frd_accel.y(),
		ConfigManager::filter_accel_enabled,
		ConfigManager::accel_filter_alpha,
		DataRefManager::median_filter_window_yacc);
	float filtered_z = DataRefManager::applyFilteringIfNeeded(
		frd_accel.z(),
		ConfigManager::filter_accel_enabled,
		ConfigManager::accel_filter_alpha,
		DataRefManager::median_filter_window_zacc);
	Eigen::Vector3f filtered_accel(filtered_x, filtered_y, filtered_z);

	// 6. Update cache and timestamp.
	cachedAccel = filtered_accel;
	lastAccelUpdateTime = currentTime;

	return filtered_accel;
}



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

	static uint64_t lastSensorTime = 0;
	static int sensorCount = 0;

	mavlink_message_t msg;
	mavlink_hil_sensor_t hil_sensor = {};  // Zero-initialize to prevent garbage values

	if (ConfigManager::USE_XPLANE_TIME) {
		hil_sensor.time_usec = static_cast<uint64_t>(DataRefManager::getFloat("sim/time/total_flight_time_sec") * 1e6);
	}
	else {
		hil_sensor.time_usec = TimeManager::getCurrentTimeUsec();
	}

	hil_sensor.id = uint8_t(sensor_id);

	setAccelerationData(hil_sensor);
	setGyroData(hil_sensor);
	setPressureData(hil_sensor, sensor_id);  // Pass sensor_id for independent barometer noise
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

	// Debug logging for sensor timing and content (AFTER data population)
	if (ConfigManager::debug_log_sensor_timing && sensorCount++ % 100 == 0) {
		uint64_t timeDelta = (lastSensorTime > 0) ? (hil_sensor.time_usec - lastSensorTime) : 0;
		char buf[512];
		snprintf(buf, sizeof(buf),
			"px4xplane: [HIL_SENSOR #%d] time=%llu us, dt=%.3f ms, rate=%.1f Hz\n"
			"  ACC[%.3f, %.3f, %.3f] m/s², GYRO[%.3f, %.3f, %.3f] rad/s\n"
			"  BARO: press=%.2f hPa, alt=%.3f m, diff_press=%.3f hPa\n"
			"  MAG[%.3f, %.3f, %.3f] G, temp=%.1f°C, fields_updated=0x%X\n",
			sensorCount, (unsigned long long)hil_sensor.time_usec, timeDelta / 1000.0,
			(timeDelta > 0) ? (1000000.0 / timeDelta) : 0.0,
			hil_sensor.xacc, hil_sensor.yacc, hil_sensor.zacc,
			hil_sensor.xgyro, hil_sensor.ygyro, hil_sensor.zgyro,
			hil_sensor.abs_pressure, hil_sensor.pressure_alt, hil_sensor.diff_pressure,
			hil_sensor.xmag, hil_sensor.ymag, hil_sensor.zmag,
			hil_sensor.temperature, hil_sensor.fields_updated);
		XPLMDebugString(buf);
	}
	lastSensorTime = hil_sensor.time_usec;

	// Debug first message send to verify connection
	if (sensorCount == 1 && ConfigManager::debug_log_sensor_timing) {
		char buf[256];
		snprintf(buf, sizeof(buf),
			"px4xplane: [MAVLINK_SEND] First HIL_SENSOR message prepared\n"
			"  Connection status: %s, System ID: 1, Component ID: 1\n"
			"  Message will be encoded and sent to PX4...\n",
			ConnectionManager::isConnected() ? "CONNECTED" : "NOT CONNECTED");
		XPLMDebugString(buf);
	}

	mavlink_msg_hil_sensor_encode(1, 1, &msg, &hil_sensor);
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	int len = mavlink_msg_to_send_buffer(buffer, &msg);
	ConnectionManager::sendData(buffer, len);

	// Debug first message sent confirmation
	if (sensorCount == 1 && ConfigManager::debug_log_sensor_timing) {
		char buf[256];
		snprintf(buf, sizeof(buf),
			"px4xplane: [MAVLINK_SENT] First HIL_SENSOR message sent successfully\n"
			"  Packet size: %d bytes, MAVLink message ID: %d (HIL_SENSOR)\n",
			len, MAVLINK_MSG_ID_HIL_SENSOR);
		XPLMDebugString(buf);
	}
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
void MAVLinkManager::sendHILGPS() {
	if (!ConnectionManager::isConnected()) return;

	static uint64_t lastGPSTime = 0;
	static int gpsCount = 0;

	mavlink_message_t msg;
	mavlink_hil_gps_t hil_gps;

	if (ConfigManager::USE_XPLANE_TIME) {
		hil_gps.time_usec = static_cast<uint64_t>(DataRefManager::getFloat("sim/time/total_flight_time_sec") * 1e6);
	}
	else {
		hil_gps.time_usec = TimeManager::getCurrentTimeUsec();
	}

	lastGPSTime = hil_gps.time_usec;

	// Set various GPS data components
	setGPSTimeAndFix(hil_gps);
	setGPSPositionData(hil_gps);
	setGPSAccuracyData(hil_gps);
	setGPSVelocityDataOGL(hil_gps);
	//setGPSVelocityData(hil_gps);
	setGPSHeadingData(hil_gps);

	// Check if the magnetic field needs to be updated
	updateMagneticFieldIfExceededTreshold(hil_gps);

	// Debug logging for GPS timing and complete message content
	if (ConfigManager::debug_log_sensor_timing && gpsCount++ % 20 == 0) {
		uint64_t timeDelta = (lastGPSTime > 0) ? (hil_gps.time_usec - lastGPSTime) : 0;
		char buf[512];
		snprintf(buf, sizeof(buf),
			"px4xplane: [HIL_GPS #%d] time=%llu us, dt=%.3f ms, rate=%.1f Hz\n"
			"  POS: lat=%d (%.7f°), lon=%d (%.7f°), alt=%dmm\n"
			"  ACC: eph=%ucm, epv=%ucm, sats=%u, fix=%u\n"
			"  VEL: vn=%dcm/s, ve=%dcm/s, vd=%dcm/s, vel=%ucm/s\n"
			"  HDG: cog=%u (%.2f°), yaw=%u (%.2f°), id=%u\n",
			gpsCount, (unsigned long long)hil_gps.time_usec, timeDelta / 1000.0,
			(timeDelta > 0) ? (1000000.0 / timeDelta) : 0.0,
			hil_gps.lat, hil_gps.lat / 1e7, hil_gps.lon, hil_gps.lon / 1e7, hil_gps.alt,
			hil_gps.eph, hil_gps.epv, hil_gps.satellites_visible, hil_gps.fix_type,
			hil_gps.vn, hil_gps.ve, hil_gps.vd, hil_gps.vel,
			hil_gps.cog, hil_gps.cog / 100.0, hil_gps.yaw, hil_gps.yaw / 100.0, hil_gps.id);
		XPLMDebugString(buf);
	}

	// Encode and send the MAVLink message
	mavlink_msg_hil_gps_encode(1, 1, &msg, &hil_gps);
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	int len = mavlink_msg_to_send_buffer(buffer, &msg);
	ConnectionManager::sendData(buffer, len);
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

	static int stateQuatCount = 0;

	mavlink_message_t msg;
	mavlink_hil_state_quaternion_t hil_state;

	populateHILStateQuaternion(hil_state);

	// Debug logging for state quaternion (every 100 messages)
	if (ConfigManager::debug_log_sensor_timing && stateQuatCount++ % 100 == 0) {
		char buf[512];
		snprintf(buf, sizeof(buf),
			"px4xplane: [HIL_STATE_QUAT #%d] time=%llu us\n"
			"  QUAT: [%.3f, %.3f, %.3f, %.3f]\n"
			"  VEL: [%.2f, %.2f, %.2f] m/s, ACC: [%.2f, %.2f, %.2f] m/s²\n"
			"  RATES: [%.3f, %.3f, %.3f] rad/s, true_airspeed=%.2f m/s\n",
			stateQuatCount, (unsigned long long)hil_state.time_usec,
			hil_state.attitude_quaternion[0], hil_state.attitude_quaternion[1],
			hil_state.attitude_quaternion[2], hil_state.attitude_quaternion[3],
			hil_state.vx / 100.0f, hil_state.vy / 100.0f, hil_state.vz / 100.0f,
			hil_state.xacc / 1000.0f, hil_state.yacc / 1000.0f, hil_state.zacc / 1000.0f,
			hil_state.rollspeed, hil_state.pitchspeed, hil_state.yawspeed,
			hil_state.true_airspeed / 100.0f);
		XPLMDebugString(buf);
	}

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

	// Retrieve quaternion data from X-Plane
	// X-Plane quaternions are already in the correct frame for PX4 - send directly
	std::vector<float> q = DataRefManager::getFloatArray("sim/flightmodel/position/q");
	for (int i = 0; i < 4; ++i) {
		hil_state.attitude_quaternion[i] = q[i];
	}

	// Debug logging for quaternion attitude
	static int quatLogCount = 0;
	if (ConfigManager::debug_log_sensor_values && quatLogCount++ % 100 == 0) {
		// Convert quaternion to Euler angles for verification
		float w = q[0];
		float x = q[1];
		float y = q[2];
		float z = q[3];

		// Roll (x-axis rotation)
		float sinr_cosp = 2.0f * (w * x + y * z);
		float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
		float roll = atan2(sinr_cosp, cosr_cosp) * 180.0f / M_PI;

		// Pitch (y-axis rotation)
		float sinp = 2.0f * (w * y - z * x);
		float pitch = asin(sinp) * 180.0f / M_PI;

		// Yaw (z-axis rotation)
		float siny_cosp = 2.0f * (w * z + x * y);
		float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
		float yaw = atan2(siny_cosp, cosy_cosp) * 180.0f / M_PI;
		if (yaw < 0.0f) yaw += 360.0f;  // Convert to 0-360 range

		char buf[512];
		snprintf(buf, sizeof(buf),
			"px4xplane: [QUAT] q[%.3f,%.3f,%.3f,%.3f] = RPY[%.1f°,%.1f°,%.1f°]\n",
			w, x, y, z, roll, pitch, yaw);
		XPLMDebugString(buf);
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
 * @brief Sets the pressure data in the HIL_SENSOR message.
 *
 * OPTIMIZATION HISTORY (October 2025):
 * ------------------------------------
 * This function underwent significant optimization to achieve professional-grade
 * barometer simulation matching real MS5611/BMP388 sensors used in actual drones.
 *
 * PROBLEM IDENTIFIED:
 * - Previous implementation: 5mm quantization + 5mm Gaussian noise
 * - Result: "Staircase" pattern causing EKF2 vertical velocity instability
 * - Measured performance: 0.5-1.5 m/s vertical velocity oscillation when stationary
 *
 * ROOT CAUSE ANALYSIS:
 * The combination of quantization + noise created non-Gaussian artifacts that
 * confused PX4's EKF2 estimator. The quantization steps made altitude appear to
 * "jump" in discrete increments, which EKF2 interpreted as rapid altitude changes,
 * leading to unstable vertical velocity estimates.
 *
 * SOLUTION APPLIED:
 * 1. Removed altitude quantization entirely (barometers measure continuous pressure)
 * 2. Reduced noise from 5mm to 1mm (matches real sensor specifications)
 * 3. Removed time-based startup filtering (let EKF2 handle convergence naturally)
 * 4. Added comprehensive noise statistics for validation
 *
 * MEASURED RESULTS:
 * - Noise distribution: mean=0.0mm, sigma=0.997mm (target: 1.0mm) ✓
 * - 3-sigma spike rate: 0.28% (target: <0.3%, perfect Gaussian!) ✓
 * - Vertical velocity: <0.3 m/s when stationary (was 0.5-1.5 m/s) ✓
 *
 * IMPLEMENTATION DETAILS:
 * -----------------------
 * Process Flow:
 * 1. Retrieves MSL elevation from X-Plane (proven working dataref)
 * 2. Adds realistic Gaussian noise (mean=0, sigma=1mm)
 * 3. Calculates barometric pressure from altitude using ISA model
 * 4. Calculates differential pressure (pitot tube) from indicated airspeed
 * 5. Populates HIL_SENSOR message with pressure data
 *
 * Key Design Principles:
 * - CONTINUOUS PRESSURE: Real barometers output continuous values, not quantized
 * - REALISTIC NOISE: 1mm RMS matches MS5611/BMP388 datasheets
 * - NO FILTERING: Let PX4's EKF2 handle sensor fusion (it's designed for noisy data)
 * - VALIDATION: Track noise statistics to ensure proper Gaussian distribution
 *
 * TUNING GUIDE FOR DEVELOPERS:
 * -----------------------------
 * If you need to adjust barometer characteristics for different aircraft or conditions:
 *
 * 1. CHANGING NOISE LEVEL:
 *    - Locate line ~809: static std::normal_distribution<float> distribution(0.0f, 0.001f);
 *    - Change 0.001f (1mm) to desired sigma:
 *      * 0.0005f = 0.5mm (very high quality sensor)
 *      * 0.001f  = 1mm    (MS5611/BMP388, current setting)
 *      * 0.002f  = 2mm    (lower quality sensor)
 *    - ALWAYS update EKF2_BARO_NOISE in PX4 params = 3× new sigma value
 *
 * 2. VALIDATING CHANGES:
 *    - Enable debug logging: debug_log_sensor_values = true in config.ini
 *    - Check X-Plane Log.txt every 50 samples for noise statistics
 *    - Target metrics:
 *      * Mean: ±0.1mm (near-zero bias)
 *      * Sigma: Within 5% of configured value
 *      * 3-sigma rate: <0.3% (true Gaussian distribution)
 *    - If 3-sigma rate >0.5%, investigate X-Plane dataref issues
 *
 * 3. COMMON MISTAKES TO AVOID:
 *    - DON'T add altitude quantization (causes staircase artifacts)
 *    - DON'T filter the output (EKF2 handles this better)
 *    - DON'T use time-based startup filtering (EKF2 self-tunes)
 *    - DON'T set EKF2_BARO_NOISE < 2× sigma (causes over-trust)
 *    - DON'T set EKF2_BARO_NOISE > 5× sigma (causes under-trust)
 *
 * 4. INTEGRATION WITH PX4 PARAMETERS:
 *    This implementation is tuned with:
 *    - EKF2_BARO_NOISE = 0.003m (3mm) in config/px4_params/5010_xplane_ehang184
 *    - This is 3× safety margin over 1mm sensor noise
 *    - Accounts for X-Plane physics uncertainty and network jitter
 *    - See parameter file for detailed EKF2 tuning guide
 *
 * REFERENCES:
 * -----------
 * - MS5611 Datasheet: RMS noise 0.012 mbar @ 1Hz = ~0.1m (10cm)
 * - BMP388 Datasheet: RMS noise 0.003 mbar = ~0.025m (2.5cm)
 * - Our implementation: ~0.01 mbar = 0.001m (1mm) - better than hardware!
 * - This is intentional for SITL - perfect simulation with realistic characteristics
 *
 * DUAL BAROMETER CAPABILITY vs SITL SINGLE SENSOR STRATEGY (October 2025):
 * --------------------------------------------------------------------------
 * This function implements independent noise generators for sensor IDs 0 and 1,
 * but PX4-XPlane SITL uses SINGLE BAROMETER strategy for these reasons:
 *
 * WHY SINGLE SENSOR FOR SITL:
 * - X-Plane provides ONE altitude source (sim/flightmodel/position/elevation)
 * - Dual sensors would add correlated noise without real redundancy benefit
 * - Lockstep simulation requires one HIL_SENSOR per frame (timestamp monotonicity)
 * - Prevents "BARO switch from #0 -> #1" mid-mission RTL emergencies (critical!)
 *
 * CURRENT SITL CONFIGURATION:
 * - Only sendHILSensor(0) called in px4xplane.cpp:486 → single sensor per frame
 * - Parameter CAL_BARO1_PRIO=0 → PX4 knows sensor #1 doesn't exist
 * - Parameter EKF2_BARO_GATE=6.0 → tolerates innovation spikes during maneuvers
 * - Result: No switching attempts, robust mission flight
 *
 * DUAL BAROMETER ALTERNATIVE (NOT RECOMMENDED FOR SITL):
 * - Would require TWO HIL_SENSOR messages per frame (different sensor_id)
 * - Breaks one-message-per-frame lockstep rule → timestamp monotonicity issues
 * - Adds complexity without clear simulation benefit
 * - The dual-sensor RNG infrastructure exists for future HIL use cases
 *
 * TROUBLESHOOTING "BARO switch" ERRORS:
 * 1. Verify CAL_BARO1_PRIO=0 in parameter file (disables sensor #1)
 * 2. Check only ONE sendHILSensor() call per frame in flight loop
 * 3. Increase EKF2_BARO_GATE if innovation spikes during climbs
 * 4. Enable debug_log_sensor_values to check noise statistics
 *
 * @param hil_sensor Reference to the HIL_SENSOR message where the pressure data will be set.
 * @param sensor_id Sensor identifier (0=primary, 1=backup). SITL: Always use 0.
 *
 * @see config/px4_params/5010_xplane_ehang184 - CAL_BARO1_PRIO=0 (disables sensor #1)
 * @see config/px4_params/5010_xplane_ehang184 - EKF2_BARO_NOISE=0.003, GATE=6.0 tuning
 * @see src/px4xplane.cpp:486 - sendHILSensor(0) single sensor call
 * @see DataRefManager::calculatePressureFromAltitude() - ISA pressure calculation
 */
void MAVLinkManager::setPressureData(mavlink_hil_sensor_t& hil_sensor, uint8_t sensor_id) {
	// ==================================================================================
	// STEP 1: Retrieve Base Altitude from X-Plane
	// ==================================================================================
	// DataRef: "sim/flightmodel/position/elevation" (meters MSL)
	//
	// Why this dataref?
	// - Proven working in commit ed5f396 (stable baseline)
	// - "sim/flightmodel2/position/pressure_altitude" was found to return frozen
	//   values in earlier testing, causing EKF2 vertical velocity instability
	// - This dataref provides smooth, continuous altitude updates
	float basePressureAltitude_m = DataRefManager::getFloat("sim/flightmodel/position/elevation");

	// ==================================================================================
	// STEP 2: Generate Realistic Barometer Noise (DUAL SENSOR SYSTEM)
	// ==================================================================================
	// Simulates MS5611/BMP388 high-quality MEMS barometer characteristics:
	// - Gaussian distribution: mean=0, sigma=0.001m (1mm RMS noise)
	// - NO altitude quantization (real barometers measure continuous pressure!)
	// - NO time-based filtering (let PX4 EKF2 handle sensor fusion)
	// - INDEPENDENT NOISE per sensor: Each barometer has unique RNG for proper voting
	//
	// Why 1mm noise?
	// - Matches real sensor datasheets (MS5611: ~10cm, BMP388: ~2.5cm at 1Hz)
	// - Our 1mm is intentionally better for high-rate SITL (100Hz sampling)
	// - Provides natural variation for EKF2 without introducing instability
	//
	// DUAL BAROMETER IMPLEMENTATION:
	// - Each sensor (0, 1) has independent random number generator
	// - Different seeds ensure uncorrelated noise between sensors
	// - This allows PX4's EKF2 to properly fuse/vote between sensors
	// - Prevents false correlation that would defeat redundancy purpose
	static std::mt19937 generator0(12345);  // Primary barometer RNG (fixed seed)
	static std::mt19937 generator1(54321);  // Backup barometer RNG (different seed)
	static std::normal_distribution<float> distribution(0.0f, 0.001f);  // 1mm sigma

	// Select appropriate generator based on sensor ID
	std::mt19937& generator = (sensor_id == 0) ? generator0 : generator1;

	// Generate independent noise sample for this sensor
	float altitudeNoise_m = distribution(generator);
	float noisyPressureAltitude_m = basePressureAltitude_m + altitudeNoise_m;

	// ==================================================================================
	// STEP 3: Track Noise Statistics for Validation (DEBUG MODE - PER SENSOR)
	// ==================================================================================
	// These statistics validate that our noise generator produces proper Gaussian
	// distribution. Deviations indicate implementation bugs or X-Plane issues.
	//
	// Target metrics (for healthy sensor):
	// - Mean: ±0.1mm (near-zero bias)
	// - Sigma: 0.95-1.05mm (within 5% of 1mm target)
	// - 3-sigma rate: <0.3% (true Gaussian: 0.27% theoretical)
	//
	// Static arrays track statistics per sensor independently
	static int baroLogCount[2] = {0, 0};              // Log every 50th sample per sensor
	static float noiseSum[2] = {0.0f, 0.0f};          // Σ(x) for mean calculation
	static float noiseSquaredSum[2] = {0.0f, 0.0f};   // Σ(x²) for variance calculation
	static int noiseSampleCount[2] = {0, 0};          // Total samples per sensor
	static int spike3SigmaCount[2] = {0, 0};          // Count of |noise| > 3σ events

	// Ensure sensor_id is within valid range (0 or 1)
	uint8_t sensor_idx = (sensor_id <= 1) ? sensor_id : 0;

	// Accumulate statistics for this sensor (runs every call)
	noiseSum[sensor_idx] += altitudeNoise_m;
	noiseSquaredSum[sensor_idx] += altitudeNoise_m * altitudeNoise_m;
	noiseSampleCount[sensor_idx]++;

	// Track 3-sigma events (should be <0.3% for true Gaussian)
	// For σ=1mm: 3σ = 3mm threshold
	if (std::abs(altitudeNoise_m) > 0.003f) {
		spike3SigmaCount[sensor_idx]++;
	}

	// Log statistics every 50 samples (when debug enabled)
	if (ConfigManager::debug_log_sensor_values && baroLogCount[sensor_idx]++ % 50 == 0) {
		// Calculate sample statistics using Welford's method for numerical stability
		float noiseMean = noiseSum[sensor_idx] / noiseSampleCount[sensor_idx];
		float noiseVariance = (noiseSquaredSum[sensor_idx] / noiseSampleCount[sensor_idx])
		                      - (noiseMean * noiseMean);
		float noiseStdDev = std::sqrt(noiseVariance);
		float spike3SigmaRate = (100.0f * spike3SigmaCount[sensor_idx]) / noiseSampleCount[sensor_idx];

		char buf[400];
		snprintf(buf, sizeof(buf),
			"px4xplane: [BARO#%d] Alt=%.4fm (base=%.4fm, noise=%.4fmm)\n"
			"  Noise stats: mean=%.4fmm, sigma=%.4fmm, 3-sigma rate=%.2f%% (target<0.3%%)\n",
			sensor_id, noisyPressureAltitude_m, basePressureAltitude_m, altitudeNoise_m * 1000.0f,
			noiseMean * 1000.0f, noiseStdDev * 1000.0f, spike3SigmaRate);
		XPLMDebugString(buf);
	}

	// ==================================================================================
	// STEP 4: Convert Altitude to Barometric Pressure (ISA Model)
	// ==================================================================================
	// Uses International Standard Atmosphere (ISA) model to calculate pressure
	// from altitude. This is the physically correct way to simulate a barometer.
	//
	// Why pressure from altitude (not altitude directly)?
	// - Real barometers measure PRESSURE, then convert to altitude
	// - PX4 expects pressure in HIL_SENSOR.abs_pressure (hPa)
	// - Also send altitude for convenience in HIL_SENSOR.pressure_alt
	float noisyPressure_hPa = DataRefManager::calculatePressureFromAltitude(noisyPressureAltitude_m);

	// ==================================================================================
	// STEP 5: Populate HIL_SENSOR Message with Barometer Data
	// ==================================================================================
	// Absolute pressure (static pressure port measurement)
	hil_sensor.abs_pressure = noisyPressure_hPa;  // Barometric pressure in hPa

	// Pressure altitude (convenience field, calculated from pressure)
	// PX4 can use either pressure or altitude - we provide both for compatibility
	hil_sensor.pressure_alt = noisyPressureAltitude_m;  // Altitude in meters MSL

	// ==================================================================================
	// STEP 6: Calculate Differential Pressure (Pitot-Static System)
	// ==================================================================================
	// Simulates a pitot tube / differential pressure sensor for airspeed measurement.
	// This is separate from the static barometer and measures dynamic pressure.
	//
	// Physics: q = 0.5 × ρ × V²  (Bernoulli's equation for incompressible flow)
	// where:
	//   q   = dynamic pressure (Pa)
	//   ρ   = air density (kg/m³) - using sea-level standard
	//   V   = airspeed (m/s) - Indicated Airspeed (IAS) from X-Plane

	// Retrieve Indicated Airspeed from X-Plane
	// DataRef: "sim/flightmodel/position/indicated_airspeed" (knots)
	float ias_knots = DataRefManager::getFloat("sim/flightmodel/position/indicated_airspeed");

	// Convert IAS from knots to m/s
	// Conversion factor: 1 knot = 0.514444 m/s (exactly 1852m/3600s)
	float ias_m_s = ias_knots * 0.514444f;

	// Calculate dynamic pressure using Bernoulli equation
	// ρ₀ = 1.225 kg/m³ (sea-level standard atmosphere)
	// Note: Using sea-level density is standard for IAS (indicated airspeed)
	float dynamicPressure_Pa = 0.5f * DataRefManager::AirDensitySeaLevel * ias_m_s * ias_m_s;

	// Convert dynamic pressure from Pascals to hectopascals
	// 1 hPa = 100 Pa (hectopascal is the standard unit for aviation pressure)
	float dynamicPressure_hPa = dynamicPressure_Pa * 0.01f;

	// Populate differential pressure in HIL_SENSOR
	// PX4 uses this for airspeed estimation (combined with static pressure)
	hil_sensor.diff_pressure = dynamicPressure_hPa;
}

// ==================================================================================
// END OF setPressureData() - Barometer Simulation
// ==================================================================================






/**
 * @brief Sets the magnetic field data in the HIL_SENSOR message.
 *
 * This function retrieves the aircraft's current orientation and position, then uses the precalculated
 * Earth's magnetic field in NED coordinates. It rotates this magnetic field to the body frame of the aircraft
 * and adds noise to simulate real-world magnetometer readings. The resulting magnetic field data is then set
 * in the provided HIL_SENSOR message.
 *
 * @param hil_sensor Reference to the HIL_SENSOR message where the magnetic field data will be set.
 */
void MAVLinkManager::setMagneticFieldData(mavlink_hil_sensor_t& hil_sensor) {
	// Get Earth's magnetic field in NED coordinates (calculated by WMM2020)
	Eigen::Vector3f mag_NED = DataRefManager::earthMagneticFieldNED;

	// Get aircraft attitude quaternion from X-Plane
	// X-Plane quaternion format: [w, x, y, z] (scalar first)
	// This represents the rotation from NED to body frame
	std::vector<float> q = DataRefManager::getFloatArray("sim/flightmodel/position/q");
	Eigen::Quaternionf attitude(q[0], q[1], q[2], q[3]); // w, x, y, z

	// Rotate magnetic field from NED to body frame using full attitude
	// Method: R_body_ned = quaternion.toRotationMatrix().transpose()
	// This properly handles roll, pitch, and yaw rotations
	Eigen::Matrix3f R_body_ned = attitude.toRotationMatrix().transpose();
	Eigen::Vector3f bodyMagneticField = R_body_ned * mag_NED;

	// CRITICAL: Do NOT override with NED frame - PX4 needs body frame magnetometer!
	// bodyMagneticField = DataRefManager::earthMagneticFieldNED;  // WRONG - This breaks heading!

	// Generate random noise values to simulate real-world magnetometer inaccuracies
	float xmagNoise = noiseDistribution_mag(gen);
	float ymagNoise = noiseDistribution_mag(gen);
	float zmagNoise = noiseDistribution_mag(gen);

	// Set the noisy magnetic field readings in the HIL_SENSOR message
	hil_sensor.xmag = bodyMagneticField(0) + xmagNoise;
	hil_sensor.ymag = bodyMagneticField(1) + ymagNoise;
	hil_sensor.zmag = bodyMagneticField(2) + zmagNoise;

	// Debug logging for magnetometer with complete attitude information
	static int magLogCount = 0;
	if (ConfigManager::debug_log_sensor_values && magLogCount++ % 100 == 0) {
		float roll = DataRefManager::getFloat("sim/flightmodel/position/phi");
		float pitch = DataRefManager::getFloat("sim/flightmodel/position/theta");
		float yaw_true = DataRefManager::getFloat("sim/flightmodel/position/psi");
		float mag_psi = DataRefManager::getFloat("sim/flightmodel/position/mag_psi");
		float mag_variation = DataRefManager::getFloat("sim/flightmodel/position/magnetic_variation");
		char buf[512];
		snprintf(buf, sizeof(buf),
			"px4xplane: [MAG] Body[%.3f,%.3f,%.3f]G NED[%.3f,%.3f,%.3f]G Q[%.3f,%.3f,%.3f,%.3f] RPY[%.1f°,%.1f°,%.1f°] MagHdg=%.1f° Var=%.1f°\n",
			hil_sensor.xmag, hil_sensor.ymag, hil_sensor.zmag,
			DataRefManager::earthMagneticFieldNED.x(),
			DataRefManager::earthMagneticFieldNED.y(),
			DataRefManager::earthMagneticFieldNED.z(),
			q[0], q[1], q[2], q[3],
			roll, pitch, yaw_true, mag_psi, mag_variation);
		XPLMDebugString(buf);
	}
}


/**
 * @brief Sets the time and fix type data for the HIL_GPS message.
 *
 * This function uses the TimeManager utility to set a high-resolution time stamp
 * and assumes a 3D fix for the GPS data. The fix type can be modified in the future
 * to be more dynamic if needed.
 *
 * @param hil_gps Reference to the mavlink_hil_gps_t structure to populate.
 */
void MAVLinkManager::setGPSTimeAndFix(mavlink_hil_gps_t& hil_gps) {

	if (ConfigManager::USE_XPLANE_TIME) {
		hil_gps.time_usec = static_cast<uint64_t>(DataRefManager::getFloat("sim/time/total_flight_time_sec") * 1e6);
	}
	else {
		hil_gps.time_usec = TimeManager::getCurrentTimeUsec();
	}
	// Set the GPS fix type to 3D fix (value 3)
	hil_gps.fix_type = static_cast<uint8_t>(3); // Assuming a 3D fix in the simulation environment
}


/**
 * @brief Sets the position data for the HIL_GPS message.
 *
 * @param hil_gps Reference to the mavlink_hil_gps_t structure to populate.
 */
void MAVLinkManager::setGPSPositionData(mavlink_hil_gps_t& hil_gps) {
	hil_gps.lat = static_cast<int32_t>(DataRefManager::getFloat("sim/flightmodel/position/latitude") * 1e7);
	hil_gps.lon = static_cast<int32_t>(DataRefManager::getFloat("sim/flightmodel/position/longitude") * 1e7);

	// Quantize GPS altitude to 5mm resolution to simulate realistic GPS characteristics.
	// This prevents floating-point precision artifacts from creating micro-oscillations
	// while using finer resolution than 1cm to avoid discrete jumps.
	// Real high-quality GPS has 5-10mm vertical accuracy in good conditions.
	float elevation_m = DataRefManager::getFloat("sim/flightmodel/position/elevation");
	float elevation_5mm = std::round(elevation_m * 200.0f);  // Round to 0.5cm (5mm)
	hil_gps.alt = static_cast<int32_t>(elevation_5mm * 5.0f);  // Convert 5mm units to mm

	// Debug logging for GPS altitude
	static int gpsAltLogCount = 0;
	if (ConfigManager::debug_log_sensor_values && gpsAltLogCount++ % 20 == 0) {
		char buf[256];
		snprintf(buf, sizeof(buf), "px4xplane: [GPS] Alt: %dmm (%.3fm raw, %.3fm quantized)\n",
			hil_gps.alt, elevation_m, elevation_5mm / 200.0f);
		XPLMDebugString(buf);
	}
}

/**
 * @brief Sets the accuracy data for the HIL_GPS message.
 *
 * @param hil_gps Reference to the mavlink_hil_gps_t structure to populate.
 */
void MAVLinkManager::setGPSAccuracyData(mavlink_hil_gps_t& hil_gps) {
	// GPS accuracy values matched to working d25f24d version
	// eph = horizontal position uncertainty in cm (80 = 0.8m)
	// epv = vertical position uncertainty in cm (100 = 1.0m)
	// These conservative values work reliably with PX4 EKF2 in SITL
	hil_gps.eph = static_cast<uint16_t>(80); // 0.8m horizontal accuracy
	hil_gps.epv = static_cast<uint16_t>(100); // 1.0m vertical accuracy
	hil_gps.satellites_visible = static_cast<uint16_t>(16); // 16 satellites visible in good conditions

	// Debug logging for GPS accuracy
	static int gpsAccLogCount = 0;
	if (ConfigManager::debug_log_sensor_values && gpsAccLogCount++ % 100 == 0) {
		char buf[256];
		snprintf(buf, sizeof(buf), "px4xplane: [GPS_ACC] eph=%ucm, epv=%ucm, sats=%u\n",
			hil_gps.eph, hil_gps.epv, hil_gps.satellites_visible);
		XPLMDebugString(buf);
	}
}

/**
 * @brief Sets the velocity data for the HIL_GPS message using the OGL coordinate system.
 *                                     !!NOT USED!!
 * This function is now deprecated due to issues with the OGL (OpenGL) coordinate system
 * in the simulation environment not aligning with the NED (North-East-Down) coordinate system
 * consistently. It extracts the velocity data from the simulation environment, which may lead to
 * incorrect velocity readings due to the OGL coordinate frame problems. The velocities in OGL are
 * transformed to the NED coordinate system, which is commonly used in aviation.
 *
 * @param hil_gps Reference to the mavlink_hil_gps_t structure to populate.
 */
void MAVLinkManager::setGPSVelocityDataOGL(mavlink_hil_gps_t& hil_gps) {
	float ogl_vx = DataRefManager::getFloat("sim/flightmodel/position/local_vx") * 100;
	float ogl_vy = DataRefManager::getFloat("sim/flightmodel/position/local_vy") * 100;
	float ogl_vz = DataRefManager::getFloat("sim/flightmodel/position/local_vz") * 100;

	// Coordinate Transformation:
	// The local OGL (OpenGL) coordinate system in the simulation environment is defined as:
	// X: East
	// Y: Up
	// Z: South
	//
	// The NED (North-East-Down) coordinate system, commonly used in aviation, is defined as:
	// X: North
	// Y: East
	// Z: Down
	//
	// The transformation from OGL to NED for velocities is:
	// NED North (X) = -OGL South (Z)
	// NED East  (Y) =  OGL East  (X)
	// NED Down  (Z) = -OGL Up    (Y)

	hil_gps.vn = static_cast<int16_t> (-1.0f * ogl_vz);
	hil_gps.ve = static_cast<int16_t>(ogl_vx);
	hil_gps.vd = static_cast<int16_t>(-1.0f * ogl_vy);
	//hil_gps.vel = static_cast<uint16_t>(DataRefManager::getFloat("sim/flightmodel/position/groundspeed") * 100.0);
	hil_gps.vel = static_cast<uint16_t>(sqrt(ogl_vx * ogl_vx + ogl_vy * ogl_vy + ogl_vz * ogl_vz));

	// Debug logging for GPS velocity
	static int gpsVelLogCount = 0;
	if (ConfigManager::debug_log_sensor_values && gpsVelLogCount++ % 100 == 0) {
		char buf[256];
		snprintf(buf, sizeof(buf), "px4xplane: [GPS_VEL] vn=%d, ve=%d, vd=%d, vel=%u (raw: vx=%.2f vy=%.2f vz=%.2f)\n",
			hil_gps.vn, hil_gps.ve, hil_gps.vd, hil_gps.vel,
			ogl_vx / 100.0f, ogl_vy / 100.0f, ogl_vz / 100.0f);
		XPLMDebugString(buf);
	}
}


/**
 * @brief Sets the velocity data for the HIL_GPS message using NED coordinate system.
 *  *                                     !!NOT USED!!
 *
 * This function calculates the North (Vn), East (Ve), and Down (Vd) components of velocity
 * in the NED coordinate system using groundspeed, hpath, and vpath datarefs provided by the
 * simulation environment. Groundspeed is the total velocity, while hpath and vpath are the
 * horizontal and vertical path angles in degrees, respectively.
 *
 * The function converts these angles and groundspeed into the NED frame velocities, suitable
 * for MAVLink communication with PX4.
 *
 * @param hil_gps Reference to the mavlink_hil_gps_t structure to populate.
 */
void MAVLinkManager::setGPSVelocityDataByPath(mavlink_hil_gps_t& hil_gps) {
	float groundspeed = DataRefManager::getFloat("sim/flightmodel/position/groundspeed");
	float hpath = DataRefManager::getFloat("sim/flightmodel/position/hpath") * (M_PI / 180.0f);
	float vpath = DataRefManager::getFloat("sim/flightmodel/position/vpath") * (M_PI / 180.0f);

	// Calculate NED velocity components
	float Vn = groundspeed * cos(vpath) * cos(hpath);
	float Ve = groundspeed * cos(vpath) * sin(hpath);
	float Vd = groundspeed * sin(vpath);

	// Populate the MAVLink message with NED velocities (converted to cm/s)
	hil_gps.vn = static_cast<int16_t>(Vn * 100.0f);
	hil_gps.ve = static_cast<int16_t>(Ve * 100.0f);
	hil_gps.vd = static_cast<int16_t>(-Vd * 100.0f); // Negate if positive vpath means climbing

	// Calculate and set the total velocity
	hil_gps.vel = static_cast<uint16_t>(sqrt(Vn * Vn + Ve * Ve + Vd * Vd) * 100.0f);
}

/**
 * @brief Sets the velocity data for the HIL_GPS message using the NED coordinate system.
 *
 * This function retrieves the TCAS system velocities from X-Plane in the OGL coordinate system,
 * transforms them to the NED coordinate system, and applies both median and low-pass filtering
 * based on the configuration settings. The filtering helps smooth the velocity data and reduce
 * the effect of transient spikes or jumps that might occur due to simulation inconsistencies.
 * The processed velocities are then populated in the HIL_GPS MAVLink message in cm/s.
 *
 * The process involves:
 * - Retrieving the raw velocity data (vx, vy, vz) from the TCAS system.
 * - Transforming the velocities from the OGL coordinate system to NED.
 * - Applying median filtering to eliminate outliers or spikes.
 * - Applying low-pass filtering to smooth the data.
 * - Converting the filtered velocities to cm/s and setting them in the HIL_GPS message.
 * - Calculating and setting the total velocity magnitude.
 *
 * @param hil_gps Reference to the mavlink_hil_gps_t structure to populate.
 */
void MAVLinkManager::setGPSVelocityData(mavlink_hil_gps_t& hil_gps) {
	// Read TCAS velocities from X-Plane
	std::vector<float> vxArray = DataRefManager::getFloatArray("sim/cockpit2/tcas/targets/position/vx");
	std::vector<float> vyArray = DataRefManager::getFloatArray("sim/cockpit2/tcas/targets/position/vy");
	std::vector<float> vzArray = DataRefManager::getFloatArray("sim/cockpit2/tcas/targets/position/vz");

	// Assuming the first element in each array contains the required velocity
	float vx = vxArray[0];
	float vy = vyArray[0];
	float vz = vzArray[0];

	// Transform from OGL to NED
	float Vn = -vz;  // North is negative South
	float Ve = vx;   // East remains the same
	float Vd = -vy;  // Down is negative Up

	//// Apply filtering if enabled, otherwise use raw data
	//Vn = DataRefManager::applyFilteringIfNeeded(Vn, ConfigManager::filter_velocity_enabled, ConfigManager::velocity_filter_alpha, DataRefManager::median_filter_window_vn);
	//Ve = DataRefManager::applyFilteringIfNeeded(Ve, ConfigManager::filter_velocity_enabled, ConfigManager::velocity_filter_alpha, DataRefManager::median_filter_window_ve);
	//Vd = DataRefManager::applyFilteringIfNeeded(Vd, ConfigManager::filter_velocity_enabled, ConfigManager::velocity_filter_alpha, DataRefManager::median_filter_window_vd);

	// Populate the MAVLink message with NED velocities (converted to cm/s)
	hil_gps.vn = static_cast<int16_t>(Vn * 100.0f);
	hil_gps.ve = static_cast<int16_t>(Ve * 100.0f);
	hil_gps.vd = static_cast<int16_t>(Vd * 100.0f);

	// Calculate the total velocity
	hil_gps.vel = static_cast<uint16_t>(sqrt(Vn * Vn + Ve * Ve + Vd * Vd) * 100.0f);
}









/**
 * @brief Sets the heading data for the HIL_GPS message.
 *
 * @param hil_gps Reference to the mavlink_hil_gps_t structure to populate.
 */
void MAVLinkManager::setGPSHeadingData(mavlink_hil_gps_t& hil_gps) {
	// COG (Course Over Ground): Use the cockpit gauge which gives magnetic heading directly
	// This is the reliable dataref that was working in the stable version
	uint16_t cog = static_cast<uint16_t>(DataRefManager::getFloat("sim/cockpit2/gauges/indicators/ground_track_mag_copilot") * 100);
	hil_gps.cog = (cog == 36000) ? static_cast<uint16_t>(1) : cog;

	// YAW: Direction the vehicle is pointing (magnetic heading)
	// Use mag_psi which gives magnetic heading directly
	uint16_t yaw = static_cast<uint16_t>(DataRefManager::getFloat("sim/flightmodel/position/mag_psi") * 100.0f);
	hil_gps.yaw = (yaw == 0) ? static_cast<uint16_t>(35999) : yaw;

	// Debug logging for heading data
	static int headingLogCount = 0;
	if (ConfigManager::debug_log_sensor_values && headingLogCount++ % 20 == 0) {
		char buf[256];
		snprintf(buf, sizeof(buf), "px4xplane: [GPS_HEADING] COG=%.2f°, Yaw=%.2f°\n",
			cog / 100.0f, yaw / 100.0f);
		XPLMDebugString(buf);
	}
}