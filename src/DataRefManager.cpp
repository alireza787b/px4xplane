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
 * This function should be called when the plugin or simulation starts to set the initial value
 * of the Earth's magnetic field in NED.
 */
void DataRefManager::initializeMagneticField() {
	float initialLatitude = getFloat("sim/flightmodel/position/latitude");
	float initialLongitude = getFloat("sim/flightmodel/position/longitude");
	float initialAltitude = getFloat("sim/flightmodel/position/elevation");

	updateEarthMagneticFieldNED(initialLatitude, initialLongitude, initialAltitude);
}


/**
 * @brief Updates the Earth's magnetic field in NED based on the provided geodetic coordinates.
 *
 * This function calculates the Earth's magnetic field in NED using the World Magnetic Model (WMM2020)
 * and the provided latitude, longitude, and altitude.
 *
 * @param lat Latitude in degrees.
 * @param lon Longitude in degrees.
 * @param alt Altitude in meters above sea level.
 * @return Eigen::Vector3f The updated magnetic field vector in NED coordinates.
 */
Eigen::Vector3f DataRefManager::updateEarthMagneticFieldNED(float lat, float lon, float alt) {
	float latRad = lat * M_PI / 180.0;
	float lonRad = lon * M_PI / 180.0;

	// Log the input values
	char log[256];
	sprintf(log, "updateEarthMagneticFieldNED called with lat: %f, lon: %f, alt: %f\n", lat, lon, alt);
	XPLMDebugString(log);

	geomag::Vector position = geomag::geodetic2ecef(lat, lon, alt);
	geomag::Vector mag_field = geomag::GeoMag(2022.5, position, geomag::WMM2020);
	geomag::Elements nedElements = geomag::magField2Elements(mag_field, lat, lon);

	earthMagneticFieldNED = Eigen::Vector3f(nedElements.north, nedElements.east, nedElements.down);

	// Log the calculated magnetic field
	sprintf(log, "Calculated magnetic field NED: north: %f, east: %f, down: %f\n", nedElements.north, nedElements.east, nedElements.down);
	XPLMDebugString(log);

	earthMagneticFieldNED *= 0.00001;  // Convert from nT to Gauss

	return earthMagneticFieldNED;
}


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




float DataRefManager::getFloat(const char* dataRef) {
	XPLMDataRef ref = XPLMFindDataRef(dataRef);
	if (ref) return XPLMGetDataf(ref);
	return 0; // or some other suitable default or error value
}

double DataRefManager::getDouble(const char* dataRef) {
	XPLMDataRef ref = XPLMFindDataRef(dataRef);
	if (ref) return XPLMGetDatad(ref);
	return 0; // or some other suitable default or error value
}

int DataRefManager::getInt(const char* dataRef) {
	XPLMDataRef ref = XPLMFindDataRef(dataRef);
	if (ref) return XPLMGetDatai(ref);
	return 0; // or some other suitable default or error value
}

std::vector<float> DataRefManager::getFloatArray(const char* dataRefName) {
	// This is just a placeholder; replace with the actual SDK call to get float array dataRef.
	XPLMDataRef dataRef = XPLMFindDataRef(dataRefName);
	int arraySize = XPLMGetDatavf(dataRef, NULL, 0, 0); // To get the array size
	std::vector<float> values(arraySize);
	XPLMGetDatavf(dataRef, values.data(), 0, arraySize); // To get the array data
	return values;
}

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
float DataRefManager::mapChannelValue(float value, float minInput, float maxInput, float minOutput, float maxOutput) {
	// Map the input value from the input range to the output range
	return ((value - minInput) / (maxInput - minInput)) * (maxOutput - minOutput) + minOutput;
}


void DataRefManager::enableOverride() {
		XPLMSetDatai(XPLMFindDataRef("sim/operation/override/override_throttles"), 1);
	}

void DataRefManager::disableOverride() {
		XPLMSetDatai(XPLMFindDataRef("sim/operation/override/override_throttles"), 0);
	}

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

			// Debug print
			char debugMsg[100];
			snprintf(debugMsg, sizeof(debugMsg), "PX4 Motor %d -> X-Plane Motor %d\n", i + 1, xplaneMotor);
			XPLMDebugString(debugMsg);
		}
	}

	// Override the throttle dataref in X-Plane for all engines at once
	std::string dataRef = "sim/flightmodel/engine/ENGN_thro_use";
	XPLMSetDatavf(XPLMFindDataRef(dataRef.c_str()), hilControls.controls, 0, 8);
}



void DataRefManager::overrideActuators_fixedwing() {
    // Retrieve the current HIL actuator controls data from the MAVLink manager
    MAVLinkManager::HILActuatorControlsData hilControls = MAVLinkManager::hilActuatorControlsData;

    // Iterate over each configured actuator
	for (const auto& [channel, config] : ConfigManager::actuatorConfigs) {
		// Retrieve the control value for the current channel and clamp it within the configured range
        float value = hilControls.controls[channel];
		value = (std::max)(config.range.first, (std::min)(config.range.second, value));

        // Set the dataref in X-Plane based on the actuator's data type
        switch (config.dataType) {
            case FLOAT_SINGLE:
                // For float type, directly set the dataref with the clamped value
                XPLMSetDataf(XPLMFindDataRef(config.datarefName.c_str()), value);
                break;
            case FLOAT_ARRAY:
                // For float array type, set the specified index in the dataref array
                if (!config.arrayIndices.empty()) {
                    XPLMSetDatavf(XPLMFindDataRef(config.datarefName.c_str()), &value, config.arrayIndices[0], 1);
                }
                break;
            default:
                // Optionally handle unknown data types, if necessary
                break;
        }
    }
}



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