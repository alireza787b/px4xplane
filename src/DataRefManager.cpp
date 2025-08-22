// DataRefManager.cpp

#include "DataRefManager.h"
#include "ConnectionManager.h"
#include <cstring>
#include <string>
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include <vector>
#include<cmath>
#include <bitset>
#include "MAVLinkManager.h"
#include "XPLMUtilities.h"
#include <ConfigManager.h>
#define M_PI 3.14
#include <chrono>
#include <FilterUtils.h>

std::vector<DataRefItem> DataRefManager::dataRefs = {
	// Position Information
	{"Position", "Latitude", "sim/flightmodel/position/latitude", "deg", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"Position", "Longitude", "sim/flightmodel/position/longitude", "deg", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"Position", "Longitude", "sim/flightmodel/position/longitude", "deg", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"Position", "Altitude", "sim/flightmodel/position/elevation", "m", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"Position", "Ground Speed", "sim/flightmodel/position/groundspeed", "m/s", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"Position", "Course (Track)", "sim/cockpit2/gauges/indicators/ground_track_mag_copilot", "deg", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},

	// Attitude Information
	{"Attitude", "Pitch", "sim/flightmodel/position/theta", "deg", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"Attitude", "Roll", "sim/flightmodel/position/phi", "deg", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"Attitude", "Heading", "sim/flightmodel/position/psi", "deg", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"Attitude", "Quaternion", "sim/flightmodel/position/q", "", 1, DRT_FLOAT_ARRAY, std::function<std::vector<float>(const char*)>(getFloatArray)},

	//// Acceleration Information
	//{"Acceleration (NED)", "X-Acc", "sim/flightmodel/position/local_ax", "m/s^2", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	//{"Acceleration (NED)", "Y-Acc", "sim/flightmodel/position/local_ay", "m/s^2", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	//{"Acceleration (NED)", "Z-Acc", "sim/flightmodel/position/local_az", "m/s^2", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},

	// Acceleration Information
	{"Acceleration (BODY)", "X-Acc", "sim/flightmodel/forces/g_axil", "m/s^2", g_earth, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"Acceleration (BODY)", "Y-Acc", "sim/flightmodel/forces/g_side", "m/s^2", g_earth, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"Acceleration (BODY)", "Z-Acc", "sim/flightmodel/forces/g_nrml", "m/s^2", g_earth, DRT_FLOAT, std::function<float(const char*)>(getFloat)},

	// Ground Speed Information
	/*{"Ground Speed", "Vx", "sim/flightmodel/position/local_vx", "m/s", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"Ground Speed", "Vy", "sim/flightmodel/position/local_vy", "m/s", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"Ground Speed", "Vz", "sim/flightmodel/position/local_vz", "m/s", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},*/

	// Airspeed Information
	{"Airspeed", "IAS", "sim/flightmodel/position/indicated_airspeed", "m/s", 0.514444, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"Airspeed", "TAS", "sim/flightmodel/position/true_airspeed", "m/s", 0.514444, DRT_FLOAT, std::function<float(const char*)>(getFloat)},

	// Pressure Information
	//{"Pressure", "Pressure Altitude", "sim/flightmodel2/position/pressure_altitude", "m", 0.3048, DRT_DOUBLE, std::function<double(const char*)>(getDouble)},

	// Angular Velocities
	{"Angular Velocity", "P", "sim/flightmodel/position/Prad", "rad/s", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"Angular Velocity", "Q", "sim/flightmodel/position/Qrad", "rad/s", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"Angular Velocity", "R", "sim/flightmodel/position/Rrad", "rad/s", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},

	// Temperature
	{"Sensors", "Outside Air Temp", "sim/cockpit2/temperature/outside_air_temp_degc", "degC", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"Sensors", "Absolute Pressure", "sim/weather/barometer_current_inhg", "hPa", 33.8639, DRT_FLOAT, std::function<float(const char*)>(getFloat)},

	// Time Information
	{"Time", "Total Flight Time", "sim/time/total_flight_time_sec", "s", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"Time", "Zulu Time", "sim/time/zulu_time_sec", "s", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},

	// RC Information
	{"RC Information", "Aileron", "sim/joystick/yoke_roll_ratio", "", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"RC Information", "Elevator", "sim/joystick/yoke_pitch_ratio", "", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"RC Information", "Throttle", "sim/cockpit2/engine/actuators/throttle_ratio_all", "", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"RC Information", "Rudder", "sim/joystick/yoke_heading_ratio", "", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
};

/**
 * @brief Static variable representing the Earth's magnetic field in NED (North-East-Down) coordinates.
 * Initialized to zero and updated based on the aircraft's location.
 */
Eigen::Vector3f DataRefManager::earthMagneticFieldNED = Eigen::Vector3f::Zero();




// Initialize static variables for median filter windows
std::deque<float> DataRefManager::median_filter_window_xacc;
std::deque<float> DataRefManager::median_filter_window_yacc;
std::deque<float> DataRefManager::median_filter_window_zacc;

std::deque<float> DataRefManager::median_filter_window_vn;
std::deque<float> DataRefManager::median_filter_window_ve;
std::deque<float> DataRefManager::median_filter_window_vd;

/**
 * @brief Size of the median filter window for barometer data.
 *
 * This constant defines the size of the window used for the median filter.
 * The median filter is applied to the barometric pressure data to reduce the impact of outliers or noise spikes.
 */
std::deque<float> DataRefManager::median_filter_window_pressure;


/**
 * @brief Tracks the current brake state of each motor.
 *
 * This bitset holds the brake state for up to 8 motors, where each bit represents a motor.
 * A set bit indicates that the brake is currently applied to the corresponding motor. This
 * tracking is crucial for ensuring brakes are only applied or released when necessary, avoiding
 * redundant operations and ensuring synchronicity with the actual state of the motors in X-Plane.
 */
std::bitset<8> DataRefManager::motorBrakeStates;



float DataRefManager::SIM_Timestep = 0;


/**
 * @brief Static member variable to store the last known geodetic position of the aircraft.
 *
 * This variable holds the last recorded geodetic position (latitude, longitude, and altitude)
 * of the aircraft. It is used to determine when to update the Earth's magnetic field in NED
 * coordinates based on the movement of the aircraft. The position is updated whenever the aircraft
 * moves beyond a certain threshold distance from this last known position.
 *
 * The variable is initialized with default values (0, 0, 0) representing a neutral geodetic position.
 *
 * Usage:
 * - Updated in `initializeMagneticField` and `updateEarthMagneticFieldNED` functions.
 * - Used in `sendHILGPS` to check if the magnetic field needs updating based on position change.
 */
GeodeticPosition DataRefManager::lastPosition = {}; // Initialize with default values



/**
 * @brief Converts a vector from NED (North-East-Down) frame to FRD (Forward-Right-Down) body frame.
 *
 * This function performs coordinate frame transformation using aerospace-standard rotation sequence.
 * The transformation rotates the coordinate system, not the vector itself.
 *
 * Rotation Sequence: ZYX Euler angles (Yaw-Pitch-Roll)
 * - First: Rotate about Z-axis by yaw angle (heading)
 * - Second: Rotate about new Y-axis by pitch angle
 * - Third: Rotate about final X-axis by roll angle
 *
 * Frame Definitions:
 * - NED Frame: X=North, Y=East, Z=Down (Earth-fixed reference frame)
 * - FRD Body Frame: X=Forward, Y=Right, Z=Down (Aircraft-fixed body frame)
 *
 * Sign Conventions:
 * - Roll: positive = right wing down
 * - Pitch: positive = nose up
 * - Yaw: positive = clockwise rotation when viewed from above
 *
 * @param nedVector Input vector in NED coordinate frame
 * @param roll Roll angle in radians (rotation about X-axis)
 * @param pitch Pitch angle in radians (rotation about Y-axis)
 * @param yaw Yaw angle in radians (rotation about Z-axis)
 * @return Vector transformed to FRD body coordinate frame
 */
Eigen::Vector3f DataRefManager::convertNEDToBody(const Eigen::Vector3f& nedVector, float roll, float pitch, float yaw) {
	// -------------------------------------------------------------------------
	// Create individual rotation quaternions for each Euler angle
	// -------------------------------------------------------------------------
	Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());     // Rotation about X-axis
	Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());   // Rotation about Y-axis
	Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());       // Rotation about Z-axis

	// -------------------------------------------------------------------------
	// Combine rotations in ZYX sequence: Yaw * Pitch * Roll
	// This is the aerospace-standard rotation sequence
	// -------------------------------------------------------------------------
	Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;

	// -------------------------------------------------------------------------
	// Convert quaternion to rotation matrix for vector transformation
	// -------------------------------------------------------------------------
	Eigen::Matrix3f rotationMatrix = q.toRotationMatrix();

	// -------------------------------------------------------------------------
	// Apply rotation to transform vector from NED to body frame
	// -------------------------------------------------------------------------
	Eigen::Vector3f bodyVector = rotationMatrix * nedVector;

	return bodyVector;
}


/**
 * @brief Initializes the Earth's magnetic field in NED based on the aircraft's initial location.
 *
 * This function sets the initial value of the Earth's magnetic field in NED coordinates
 * at the start of the simulation. It retrieves the initial geodetic position (latitude,
 * longitude, and altitude) of the aircraft and uses it to update the magnetic field.
 *
 * The function also initializes the last known position to the initial position.
 *
 * Usage:
 * @code
 * DataRefManager::initializeMagneticField();
 * @endcode
 */
void DataRefManager::initializeMagneticField() {
	DataRefManager::lastPosition = {
		getFloat("sim/flightmodel/position/latitude"),
		getFloat("sim/flightmodel/position/longitude"),
		getFloat("sim/flightmodel/position/elevation")
	};

	updateEarthMagneticFieldNED(DataRefManager::lastPosition);
}


/**
 * @brief Calculates the pressure altitude from a given barometric pressure.
 *
 * This function uses the International Standard Atmosphere (ISA) barometric formula
 * to calculate the altitude from the given barometric pressure. The calculation assumes
 * that the pressure is given in hPa and returns the altitude in meters.
 *
 * @param pressure_hPa The barometric pressure in hPa.
 * @return The calculated pressure altitude in meters.
 */
float DataRefManager::calculatePressureAltitude(float pressure_hPa) {
	// Constants based on ISA
	constexpr float P0 = 1013.25f;  // Standard sea level pressure in hPa
	constexpr float T0 = 288.15f;   // Standard temperature at sea level in Kelvin
	constexpr float L = 0.0065f;    // Temperature lapse rate in K/m
	constexpr float R = 8.3144598f; // Universal gas constant in J/(mol�K)
	constexpr float g = 9.80665f;   // Gravitational acceleration in m/s�
	constexpr float M = 0.0289644f; // Molar mass of Earth's air in kg/mol

	// Simplified constant for the altitude formula
	constexpr float exp_factor = (g * M) / (R * L);

	// Calculate altitude using the barometric formula
	float altitude = (T0 / L) * (1.0f - std::pow(pressure_hPa / P0, 1.0f / exp_factor));

	return altitude;
}

/**
 * @brief Calculates the barometric pressure from a given pressure altitude.
 *
 * This function uses the International Standard Atmosphere (ISA) barometric formula
 * to calculate the barometric pressure based on the given altitude. The calculation assumes
 * a standard temperature lapse rate and returns the pressure in hPa.
 *
 * @param altitude_m The pressure altitude in meters.
 * @return The calculated barometric pressure in hPa.
 */
float DataRefManager::calculatePressureFromAltitude(float altitude_m) {
	// Constants based on ISA
	constexpr float P0 = 1013.25f;  // Standard sea level pressure in hPa
	constexpr float T0 = 288.15f;   // Standard temperature at sea level in Kelvin
	constexpr float L = 0.0065f;    // Temperature lapse rate in K/m
	constexpr float R = 8.3144598f; // Universal gas constant in J/(mol�K)
	constexpr float g = 9.80665f;   // Gravitational acceleration in m/s�
	constexpr float M = 0.0289644f; // Molar mass of Earth's air in kg/mol

	// Calculate temperature at the given altitude
	float temperature = T0 - L * altitude_m;

	// Calculate pressure using the barometric formula
	float pressure = P0 * pow(1.0f - (L * altitude_m) / T0, (g * M) / (R * L));

	return pressure;
}

/**
 * @brief Calculates the distance between two geodetic positions.
 *
 * This function computes the distance in meters between two points on the Earth's surface,
 * given their latitude, longitude, and altitude. It uses the Haversine formula for calculating
 * the great-circle distance between two points on a sphere from their longitudes and latitudes.
 *
 * The altitude difference is also considered in the distance calculation.
 *
 * @param pos1 The first geodetic position.
 * @param pos2 The second geodetic position.
 * @return float The distance between pos1 and pos2 in meters.
 *
 * Usage:
 * @code
 * float distance = DataRefManager::calculateDistance(pos1, pos2);
 * @endcode
 */
float DataRefManager::calculateDistance(const GeodeticPosition& pos1, const GeodeticPosition& pos2) {
	const float R = 6371000; // Earth's radius in meters
	float lat1Rad = pos1.latitude * M_PI / 180.0f;
	float lat2Rad = pos2.latitude * M_PI / 180.0f;
	float deltaLat = (pos2.latitude - pos1.latitude) * M_PI / 180.0f;
	float deltaLon = (pos2.longitude - pos1.longitude) * M_PI / 180.0f;

	float a = sin(deltaLat / 2) * sin(deltaLat / 2) +
		cos(lat1Rad) * cos(lat2Rad) *
		sin(deltaLon / 2) * sin(deltaLon / 2);
	float c = 2 * atan2(sqrt(a), sqrt(1 - a));

	float distance = R * c;

	// Considering altitude difference
	float altDiff = pos2.altitude - pos1.altitude;
	distance = sqrt(distance * distance + altDiff * altDiff);

	return distance;
}




/**
 * @brief Updates the Earth's magnetic field in NED based on the provided geodetic coordinates.
 *
 * This function calculates the Earth's magnetic field in NED using the World Magnetic Model (WMM2020)
 * and the provided geodetic position (latitude, longitude, altitude). The magnetic field is calculated
 * in the North-East-Down (NED) coordinate system and scaled from nanoteslas to Gauss.
 *
 * The function also updates the last known position and logs the updated magnetic field values
 * for debugging purposes.
 *
 * @param position The geodetic position (latitude, longitude, altitude) for the magnetic field calculation.
 * @return Eigen::Vector3f The updated magnetic field vector in NED coordinates.
 *
 * Usage:
 * @code
 * Eigen::Vector3f magneticField = DataRefManager::updateEarthMagneticFieldNED(position);
 * @endcode
 */
Eigen::Vector3f DataRefManager::updateEarthMagneticFieldNED(const GeodeticPosition& position) {
	float latRad = position.latitude * M_PI / 180.0f;
	float lonRad = position.longitude * M_PI / 180.0f;

	// Convert geodetic coordinates to ECEF (Earth-Centered, Earth-Fixed)
	geomag::Vector ecefPosition = geomag::geodetic2ecef(position.latitude, position.longitude, position.altitude);

	// Calculate the magnetic field using the World Magnetic Model
	geomag::Vector magField = geomag::GeoMag(2025.8, ecefPosition, geomag::WMM2020);
	geomag::Elements nedElements = geomag::magField2Elements(magField, position.latitude, position.longitude);

	// Convert the magnetic field to NED (North-East-Down) coordinates and scale it
	DataRefManager::earthMagneticFieldNED = Eigen::Vector3f(nedElements.north, nedElements.east, nedElements.down) * 0.00001;  // Convert from nT to Gauss

	// Update the last known position
	DataRefManager::lastPosition = position;


	/*char log[256];
	sprintf(log, "px4xplane: Magnetic field updated - North: %f, East: %f, Down: %f\n", earthMagneticFieldNED.x(), earthMagneticFieldNED.y(), earthMagneticFieldNED.z());
	XPLMDebugString(log);*/


	return earthMagneticFieldNED;
}

/**
 * Draws the data references and their current values on an X-Plane window.
 * This function iterates through a collection of data references, categorized and formatted,
 * and displays their values in the simulation window. It supports various data types, including
 * floats, doubles, integers, and float arrays.
 *
 * @param in_window_id The identifier for the X-Plane window where the data will be drawn.
 * @param l Left coordinate for the starting point of the drawing.
 * @param t Top coordinate for the starting point of the drawing.
 * @param col_white Color array for the text.
 * @param lineOffset Vertical offset for each line of text.
 * @return The new line offset after drawing all data references.
 */
int DataRefManager::drawDataRefs(XPLMWindowID in_window_id, int l, int t, float col_white[],int lineOffset) {
	

	const char* currentCategory = nullptr;
	for (const auto& item : dataRefs) {
		if (currentCategory == nullptr || strcmp(currentCategory, item.category) != 0) {
			currentCategory = item.category;
			XPLMDrawString(col_white, l + 10, t - lineOffset, (char*)currentCategory, NULL, xplmFont_Proportional);
			lineOffset += 20; // Increment line offset for next items
		}

		std::string valueStr; // string to store the formatted value
		if (item.type == DRT_FLOAT) {
			float value = getFloat(item.dataRef) * item.multiplier;
			valueStr = std::to_string(value);
		}
		else if (item.type == DRT_DOUBLE) {
			double value = getDouble(item.dataRef) * item.multiplier;
			valueStr = std::to_string(value);
		}
		else if (item.type == DRT_INT) {
			int value = getInt(item.dataRef) * static_cast<int>(item.multiplier);
			valueStr = std::to_string(value);
		}
		else if (item.type == DRT_FLOAT_ARRAY) {
			std::vector<float> arrayValues = getFloatArray(item.dataRef);
			valueStr = arrayToString(arrayValues);
		}

		char buf[512];  // Ensure this buffer is large enough to hold the entire string.
		snprintf(buf, sizeof(buf), "  %s: %s %s", item.title, valueStr.c_str(), item.unit);
		XPLMDrawString(col_white, l + 10, t - lineOffset, buf, NULL, xplmFont_Proportional);
		lineOffset += 20; // Increment line offset for next items
	}
	return lineOffset;
}

/**
 * Draws the actuator control data on an X-Plane window.
 * This function displays various control parameters such as the control values,
 * timestamp, mode, and flags obtained from the High-Level Actuator Control Data (HILActuatorControlsData form PX4).
 * It's used for visualization and debugging of the actuator control data within the simulation.
 *
 * @param in_window_id The identifier for the X-Plane window where the data will be drawn.
 * @param l Left coordinate for the starting point of the drawing.
 * @param t Top coordinate for the starting point of the drawing.
 * @param col_white Color array for the text.
 * @param lineOffset Vertical offset for each line of text.
 * @return The new line offset after drawing the actuator controls.
 */
int DataRefManager::drawActuatorControls(XPLMWindowID in_window_id, int l, int t, float col_white[], int lineOffset) {
	// Get the actuator controls data
	auto& hilActuatorControlsData = MAVLinkManager::hilActuatorControlsData;

	// Draw the timestamp
	char buf[512];
	snprintf(buf, sizeof(buf), "Timestamp: %llu", hilActuatorControlsData.timestamp);
	XPLMDrawString(col_white, l + 10, t - lineOffset, buf, NULL, xplmFont_Proportional);
	lineOffset += 20;

	// Draw the controls
	for (int i = 0; i < 16; ++i) {
		snprintf(buf, sizeof(buf), "Control %d: %f", i, hilActuatorControlsData.controls[i]);
		XPLMDrawString(col_white, l + 10, t - lineOffset, buf, NULL, xplmFont_Proportional);
		lineOffset += 20;
	}

	// Draw the mode
	snprintf(buf, sizeof(buf), "Mode: %u", hilActuatorControlsData.mode);
	XPLMDrawString(col_white, l + 10, t - lineOffset, buf, NULL, xplmFont_Proportional);
	lineOffset += 20;

	// Draw the flags
	snprintf(buf, sizeof(buf), "Flags: %llu", hilActuatorControlsData.flags);
	XPLMDrawString(col_white, l + 10, t - lineOffset, buf, NULL, xplmFont_Proportional);
	return lineOffset;
}



/**
 * Retrieves a float value from the specified data reference.
 * If the data reference is valid, the function returns its current float value.
 * Otherwise, a default value of 0 is returned.
 *
 * @param dataRef The data reference from which to retrieve the value.
 * @return The float value of the data reference, or 0 if not found.
 */
float DataRefManager::getFloat(const char* dataRef) {
	XPLMDataRef ref = XPLMFindDataRef(dataRef);
	if (ref) return XPLMGetDataf(ref);
	return 0; // or some other suitable default or error value
}

/**
 * Retrieves a double value from the specified data reference.
 * If the data reference is valid, the function returns its current double value.
 * Otherwise, a default value of 0 is returned.
 *
 * @param dataRef The data reference from which to retrieve the value.
 * @return The double value of the data reference, or 0 if not found.
 */
double DataRefManager::getDouble(const char* dataRef) {
	XPLMDataRef ref = XPLMFindDataRef(dataRef);
	if (ref) return XPLMGetDatad(ref);
	return 0; // or some other suitable default or error value
}

/**
 * Retrieves an integer value from the specified data reference.
 * If the data reference is valid, the function returns its current integer value.
 * Otherwise, a default value of 0 is returned.
 *
 * @param dataRef The data reference from which to retrieve the value.
 * @return The integer value of the data reference, or 0 if not found.
 */
int DataRefManager::getInt(const char* dataRef) {
	XPLMDataRef ref = XPLMFindDataRef(dataRef);
	if (ref) return XPLMGetDatai(ref);
	return 0; // or some other suitable default or error value
}

/**
 * Retrieves an array of float values from the specified data reference.
 * The function determines the size of the array and retrieves all values,
 * returning them in a vector.
 *
 * @param dataRefName The data reference from which to retrieve the array.
 * @return A vector containing the float values from the data reference.
 */
std::vector<float> DataRefManager::getFloatArray(const char* dataRefName) {
	// This is just a placeholder; replace with the actual SDK call to get float array dataRef.
	XPLMDataRef dataRef = XPLMFindDataRef(dataRefName);
	int arraySize = XPLMGetDatavf(dataRef, NULL, 0, 0); // To get the array size
	std::vector<float> values(arraySize);
	XPLMGetDatavf(dataRef, values.data(), 0, arraySize); // To get the array data
	return values;
}

/**
 * Converts an array of float values to a string representation.
 * This function creates a comma-separated string enclosed in square brackets,
 * representing the contents of the float array.
 *
 * @param array The vector of float values to be converted.
 * @return A string representation of the array.
 */
std::string DataRefManager::arrayToString(const std::vector<float>& array) {
	std::string result = "[";
	for (size_t i = 0; i < array.size(); ++i) {
		result += std::to_string(array[i]);
		if (i < array.size() - 1)
			result += ",";
	}
	result += "]";
	return result;
}

/**
 * Maps a value from one range to another.
 * This utility function scales an input value from its original range to a new specified range.
 *
 * @param value The input value to be mapped.
 * @param minInput The minimum value of the input range.
 * @param maxInput The maximum value of the input range.
 * @param minOutput The minimum value of the output range.
 * @param maxOutput The maximum value of the output range.
 * @return The mapped value in the output range.
 */
float DataRefManager::mapChannelValue(float value, float minInput, float maxInput, float minOutput, float maxOutput) {
	// Map the input value from the input range to the output range
	return ((value - minInput) / (maxInput - minInput)) * (maxOutput - minOutput) + minOutput;
}

/**
 * Enables override for various control aspects in the simulation environment.
 * This function activates the override for throttle, control surfaces, and wheel steering
 * in the X-Plane simulation. This is necessary to allow external control inputs (e.g., from PX4)
 * to take effect, bypassing the default X-Plane control mechanisms.
 */
void DataRefManager::enableOverride() {
	XPLMSetDatai(XPLMFindDataRef("sim/operation/override/override_throttles"), 1);
	XPLMSetDatai(XPLMFindDataRef("sim/operation/override/override_control_surfaces"), 1);
	XPLMSetDatai(XPLMFindDataRef("sim/operation/override/override_wheel_steer"), 1);
	
	}

/**
 * Disables override for various control aspects in the simulation environment.
 * This function deactivates the override for throttle, control surfaces, and wheel steering
 * in the X-Plane simulation, returning control to the default X-Plane mechanisms.
 * This is used when external control inputs are no longer needed or when returning to manual control.
 */
void DataRefManager::disableOverride() {
		XPLMSetDatai(XPLMFindDataRef("sim/operation/override/override_throttles"), 0);
		XPLMSetDatai(XPLMFindDataRef("sim/operation/override/override_control_surfaces"), 0);
		XPLMSetDatai(XPLMFindDataRef("sim/operation/override/override_wheel_steer"), 0);


	}



/**
 * @brief Overrides actuators based on the current aircraft configuration and applies brakes as necessary.
 *
 * This function applies control values from the MAVLink HIL (Hardware In the Loop)
 * actuator controls data to the corresponding X-Plane datarefs. It iterates through
 * each actuator configuration defined in the ConfigManager and scales the control
 * values to the appropriate range for each dataref. After setting these values,
 * it also checks each motor to determine if a brake should be applied or released,
 * based on the motor's current throttle value and its brake configuration.
 *
 * The function handles different types of datarefs (single float or float array) and
 * applies the scaled value to the correct index in the case of float arrays. The scaling
 * is based on the range specified in each actuator's configuration.
 *
 * For each actuator, the function:
 * 1. Retrieves the original control value from the HIL data.
 * 2. Scales this value to the range defined in the actuator configuration.
 * 3. Sets the scaled value to the appropriate X-Plane dataref.
 * 4. After processing all actuators, it checks and applies brakes on motors as necessary.
 *
 * If the data type of the actuator is not recognized, an error message is logged.
 */
void DataRefManager::overrideActuators() {
	MAVLinkManager::HILActuatorControlsData hilControls = MAVLinkManager::hilActuatorControlsData;

	for (const auto& [channel, actuatorConfig] : ConfigManager::actuatorConfigs) {
		float originalValue = hilControls.controls[channel];

		for (const auto& datarefConfig : actuatorConfig.getDatarefConfigs()) {
			// Scale the value to the datarefConfig's range
			float scaledValue = scaleActuatorCommand(originalValue, -1.0, +1.0, datarefConfig.range.first, datarefConfig.range.second);

			// Set the dataref based on the data type
			switch (datarefConfig.dataType) {
			case FLOAT_SINGLE: {
				XPLMDataRef floatDataRef = XPLMFindDataRef(datarefConfig.datarefName.c_str());
				if (floatDataRef != nullptr) {
					XPLMSetDataf(floatDataRef, scaledValue);
				}
				break;
			}
			case FLOAT_ARRAY: {
				if (!datarefConfig.arrayIndices.empty()) {
					XPLMDataRef arrayDataRef = XPLMFindDataRef(datarefConfig.datarefName.c_str());
					if (arrayDataRef != nullptr) {
						for (int index : datarefConfig.arrayIndices) {
							XPLMSetDatavf(arrayDataRef, &scaledValue, index, 1);
						}
					}
				}
				break;
			}
			default:
				XPLMDebugString(("px4xplane: Unknown data type for actuator " + std::to_string(channel) + "\n").c_str());
				break;
			}
		}

		// Check and apply brakes on motors as necessary
		checkAndApplyPropBrakes();
	}

}



/**
 * Scales an actuator command value from its input range to a specified output range.
 * This utility function is used to map a control input value (e.g., from PX4) that is in a
 * certain range (e.g., -1 to +1) to a new range that is expected by the simulation environment
 * (e.g., 0 to 100).
 *
 * @param input The original input value to be scaled.
 * @param inputMin The minimum value of the input range.
 * @param inputMax The maximum value of the input range.
 * @param outputMin The minimum value of the output range.
 * @param outputMax The maximum value of the output range.
 * @return The scaled value.
 */
float DataRefManager::scaleActuatorCommand(float input, float inputMin, float inputMax, float outputMin, float outputMax) {
	float scale = (outputMax - outputMin) / (inputMax - inputMin);
	return outputMin + scale * (input - inputMin);
}


/**
 * Draws the actual throttle data on an X-Plane window.
 * This function fetches and displays the actual throttle data used in X-Plane for each engine.
 * It is used for visualizing the current throttle settings in the simulation, assisting in debugging
 * and monitoring purposes.
 *
 * @param in_window_id The identifier for the X-Plane window where the data will be drawn.
 * @param l Left coordinate for the starting point of the drawing.
 * @param t Top coordinate for the starting point of the drawing.
 * @param col_white Color array for the text.
 * @param lineOffset Vertical offset for each line of text.
 * @return The new line offset after drawing the text.
 */
int DataRefManager::drawActualThrottle(XPLMWindowID in_window_id, int l, int t, float col_white[], int lineOffset) {
	// Fetch the actual throttle data from X-Plane
	std::string throttleDataRef = "sim/flightmodel2/engines/throttle_used_ratio";
	float throttleData[16];
	XPLMGetDatavf(XPLMFindDataRef(throttleDataRef.c_str()), throttleData, 0, 16);

	// Draw the actual throttle data
	char buf[512];
	for (int i = 0; i < 16; ++i) {
		snprintf(buf, sizeof(buf), "Motor %d Throttle: %f", i, throttleData[i]);
		XPLMDrawString(col_white, l + 10, t - lineOffset, buf, NULL, xplmFont_Proportional);
		lineOffset += 20;
	}
	return lineOffset;
}

/**
 * Retrieves and formats the drone configuration information for display.
 * This function fetches the current configuration name and type from ConfigManager and
 * formats them into a human-readable string. If either the name or type is not set,
 * they are replaced with "N/A" to indicate their absence.
 *
 * @return A formatted string containing the drone configuration details.
 */
std::string DataRefManager::GetFormattedDroneConfig() {
	// Fetch the configuration name and type from ConfigManager
	std::string configName = ConfigManager::getConfigName();

	// Check for empty values and replace them with "N/A"
	if (configName.empty()) configName = "N/A";

	// Prepare the formatted string
	char formattedConfig[512]; // Adjust the size as needed
	snprintf(formattedConfig, sizeof(formattedConfig), "Drone Config: %s", configName.c_str());

	return std::string(formattedConfig);
}


/**
 * @brief Applies or releases a brake on a specific motor by manipulating X-Plane's failure DataRefs.
 * This function simulates applying a brake to a motor by injecting a seizure failure and a prop separation failure
 * into X-Plane's simulation for the specified motor. It constructs the DataRef names for the specific motor's
 * failures and then sets the failure modes to simulate the brake being applied or released. The function also
 * tracks the state of each motor's brake to avoid unnecessary operations.
 *
 * @param motorIndex The index of the motor to which the brake will be applied or released.
 *                   This should correspond to the actual motor numbers in X-Plane (typically 0-7).
 * @param enable A boolean flag indicating whether to apply (true) or release (false) the brake.
 */
void DataRefManager::applyBrake(int motorIndex, bool enable) {
	// Construct the DataRef names for the specific motor's seizure and prop separation failures
	std::string seizureFailureDataRefName = "sim/operation/failures/rel_seize_" + std::to_string(motorIndex);
	std::string propSepFailureDataRefName = "sim/operation/failures/rel_prpsep" + std::to_string(motorIndex);

	// Find the DataRefs
	XPLMDataRef seizureFailureDataRef = XPLMFindDataRef(seizureFailureDataRefName.c_str());
	XPLMDataRef propSepFailureDataRef = XPLMFindDataRef(propSepFailureDataRefName.c_str());

	if (seizureFailureDataRef != nullptr && propSepFailureDataRef != nullptr) {
		if (enable) {
			// Inject failures to simulate brake - Set failure modes for both seizure and prop separation
			XPLMSetDatai(seizureFailureDataRef, 6); // Engine Seize
			XPLMSetDatai(propSepFailureDataRef, 6); // Prop Separation
			XPLMDebugString(("Applying brake to motor " + std::to_string(motorIndex) + "\n").c_str());
		}
		else {
			// Remove failures to release brake - Reset failure modes to 0 (No failure)
			XPLMSetDatai(seizureFailureDataRef, 0);
			XPLMSetDatai(propSepFailureDataRef, 0);
			XPLMDebugString(("Releasing brake from motor " + std::to_string(motorIndex) + "\n").c_str());
		}
		motorBrakeStates.set(motorIndex, enable); // Update the brake state tracking
	}
	else {
		// Log an error if the DataRefs were not found
		XPLMDebugString(("px4xplane: Failed to find failure DataRefs for motor " + std::to_string(motorIndex) + "\n").c_str());
	}
}



/**
 * @brief Applies low-pass and median filtering to data if filtering is enabled.
 *
 * This function checks if filtering is enabled for a specific data type. It applies
 * a low-pass filter to the data, followed by a median filter if enabled.
 * The previous filtered value is fetched from the last value in the window.
 *
 * @param raw_value The raw sensor value (e.g., accelerometer, velocity).
 * @param filter_enabled Flag indicating whether filtering is enabled.
 * @param filter_alpha The alpha value for the low-pass filter.
 * @param median_filter_window The deque storing the window for the median filter.
 * @return The filtered or raw value, depending on the filtering settings.
 */
float DataRefManager::applyFilteringIfNeeded(float raw_value,
	bool filter_enabled,
	float filter_alpha,
	std::deque<float>& median_filter_window)
{
	if (filter_enabled) {
		// Initialize the window with the first raw value if empty
		if (median_filter_window.empty()) {
			median_filter_window.push_back(raw_value);
		}

		// Apply low-pass filter using the last value in the window as the previous filtered value
		float filtered_value = lowPassFilter(raw_value, median_filter_window.back(), filter_alpha);

		// Apply median filter
		filtered_value = medianFilter(filtered_value, median_filter_window);

		return filtered_value;
	}
	else {
		// If filtering is disabled, return the raw value
		return raw_value;
	}
}


/**
 * @brief Checks the throttle for each motor and applies or releases the brake as necessary.
 *
 * This function retrieves the current throttle values for each motor and determines
 * if the brake should be applied or released based on the motor's brake configuration
 * and its current throttle value. It uses the 'applyBrake' function to manage the
 * actual application or release of the brake.
 */
void DataRefManager::checkAndApplyPropBrakes() {
	XPLMDataRef throttleDataRef = XPLMFindDataRef("sim/flightmodel/engine/ENGN_thro_use");
	if (throttleDataRef != nullptr) {
		float throttleValues[8]; // Assuming a maximum of 8 motors
		XPLMGetDatavf(throttleDataRef, throttleValues, 0, 8);

		for (int i = 0; i < 8; ++i) {
			// Check if the motor has a brake feature and needs a brake applied or removed
			if (ConfigManager::hasPropBrake(i)) {
				bool shouldBrake = throttleValues[i] == 0.0f;
				bool isBraking = motorBrakeStates.test(i);
				if (shouldBrake != isBraking) {
					applyBrake(i, shouldBrake);
				}
			}
		}
	}
	else {
		XPLMDebugString("px4xplane: Unable to find throttle dataref\n");
	}
}