// DataRefManager.cpp

#include "DataRefManager.h"
#include "ConnectionManager.h"
#include <cstring>
#include <string>
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include <vector>
#include<cmath>
#include "MAVLinkManager.h"
#include "XPLMUtilities.h"
#include <ConfigManager.h>
#define M_PI 3.14
#include <chrono>

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
 * @brief Converts a magnetic field vector from NED to body frame.
 *
 * This function uses the provided roll, pitch, and yaw angles to rotate the magnetic field vector
 * from the NED frame to the body frame of the aircraft.
 *
 * @param nedVector Magnetic field vector in NED coordinates.
 * @param roll Roll angle in radians (rotation about the X-axis).
 * @param pitch Pitch angle in radians (rotation about the Y-axis).
 * @param yaw Yaw angle in radians with respect to Magnetic North (rotation about the Z-axis).
 * @return Eigen::Vector3f The magnetic field vector in the body frame, in Gauss.
 */
Eigen::Vector3f DataRefManager::convertNEDToBody(const Eigen::Vector3f& nedVector, float roll, float pitch, float yaw) {
	//temporarly remove the effect of roll and pitch
	Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
	Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf yawAngle(-yaw, Eigen::Vector3f::UnitZ());
	Eigen::Matrix3f rotationMatrix = (rollAngle * pitchAngle * yawAngle).matrix();

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
	geomag::Vector magField = geomag::GeoMag(2022.5, ecefPosition, geomag::WMM2020);
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
 * Overrides the actuators for a multirotor configuration in the simulation.
 * This function fetches the actuator control data from MAVLinkManager and corrects it based
 * on the motor mappings configured in ConfigManager. It then overrides the throttle dataref
 * in X-Plane for all engines to reflect these corrected controls.
 * This is specifically tailored for multirotor vehicles and handles up to 8 motors.
 */
void DataRefManager::overrideActuators_multirotor() {
	// Get the HILActuatorControlsData from MAVLinkManager
	MAVLinkManager::HILActuatorControlsData hilControls = MAVLinkManager::hilActuatorControlsData;

	// Create a new HILActuatorControlsData structure for the corrected controls
	MAVLinkManager::HILActuatorControlsData correctedControls;
	correctedControls.timestamp = hilControls.timestamp;
	correctedControls.mode = hilControls.mode;
	correctedControls.flags = hilControls.flags;

	// Correct the controls based on the motor mappings from ConfigManager
	for (int i = 0; i < 8 ; i++) {
		int xplaneMotor = ConfigManager::getXPlaneMotorFromPX4(i + 1);
		if (xplaneMotor != -1) { // Check if mapping exists
			correctedControls.controls[xplaneMotor - 1] = hilControls.controls[i];

			
		}
	}

	// Override the throttle dataref in X-Plane for all engines at once
	std::string dataRef = "sim/flightmodel/engine/ENGN_thro_use";
	XPLMSetDatavf(XPLMFindDataRef(dataRef.c_str()), hilControls.controls, 0, 8);
}



void DataRefManager::overrideActuators_fixedwing() {
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
	std::string configType = ConfigManager::getConfigType();

	// Check for empty values and replace them with "N/A"
	if (configName.empty()) configName = "N/A";
	if (configType.empty()) configType = "N/A";

	// Prepare the formatted string
	char formattedConfig[512]; // Adjust the size as needed
	snprintf(formattedConfig, sizeof(formattedConfig), "Drone Config: %s\nConfig Type: %s", configName.c_str(), configType.c_str());

	return std::string(formattedConfig);
}