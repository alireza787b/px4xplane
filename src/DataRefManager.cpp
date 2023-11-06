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
#define M_PI 3.14

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
	bodyVector *= 0.00001;  // Convert from nT to Gauss

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

	geomag::Vector position = geomag::geodetic2ecef(latRad, lonRad, alt);
	geomag::Vector mag_field = geomag::GeoMag(2022.5, position, geomag::WMM2020);
	geomag::Elements nedElements = geomag::magField2Elements(mag_field, latRad, lonRad);

	earthMagneticFieldNED = Eigen::Vector3f(nedElements.north, nedElements.east, nedElements.down);
	return earthMagneticFieldNED;
}


void DataRefManager::drawDataRefs(XPLMWindowID in_window_id, int l, int t, float col_white[],int lineOffset) {
	

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

std::tuple<float, float, float> DataRefManager::convertOGLtoNED(float ogl_vx, float ogl_vy, float ogl_vz, float roll_rad, float pitch_rad, float yaw_rad) {
	// Calculate the rotation matrix from OGL to NED
	// This is a simplified example and might not exactly match your requirements
	float R[3][3];
	R[0][0] = cos(yaw_rad) * cos(pitch_rad);
	R[0][1] = cos(yaw_rad) * sin(pitch_rad) * sin(roll_rad) - sin(yaw_rad) * cos(roll_rad);
	R[0][2] = cos(yaw_rad) * sin(pitch_rad) * cos(roll_rad) + sin(yaw_rad) * sin(roll_rad);

	R[1][0] = sin(yaw_rad) * cos(pitch_rad);    
	R[1][1] = sin(yaw_rad) * sin(pitch_rad) * sin(roll_rad) + cos(yaw_rad) * cos(roll_rad);
	R[1][2] = sin(yaw_rad) * sin(pitch_rad) * cos(roll_rad) - cos(yaw_rad) * sin(roll_rad);

	R[2][0] = -sin(pitch_rad);
	R[2][1] = cos(pitch_rad) * sin(roll_rad);
	R[2][2] = cos(pitch_rad) * cos(roll_rad);

	// Convert OGL velocities to NED velocities using the rotation matrix
	float ned_vn = R[0][0] * ogl_vx + R[0][1] * ogl_vy + R[0][2] * ogl_vz;
	float ned_ve = R[1][0] * ogl_vx + R[1][1] * ogl_vy + R[1][2] * ogl_vz;
	float ned_vd = R[2][0] * ogl_vx + R[2][1] * ogl_vy + R[2][2] * ogl_vz;

	return std::make_tuple(ned_vn, ned_ve, ned_vd);
}

void DataRefManager::enableOverride() {
		XPLMSetDatai(XPLMFindDataRef("sim/operation/override/override_throttles"), 1);
	}

void DataRefManager::disableOverride() {
		XPLMSetDatai(XPLMFindDataRef("sim/operation/override/override_throttles"), 0);
	}


void DataRefManager::overrideActuators() {
	// Get the HILActuatorControlsData from MAVLinkManager
	MAVLinkManager::HILActuatorControlsData hilControls = MAVLinkManager::hilActuatorControlsData;

	// Create a new HILActuatorControlsData structure for the corrected controls
	MAVLinkManager::HILActuatorControlsData correctedControls;
	correctedControls.timestamp = hilControls.timestamp;
	correctedControls.mode = hilControls.mode;
	correctedControls.flags = hilControls.flags;

	// Correct the controls based on the motor mappings
	for (int i = 0; i < 16; i++) {
		int xplaneMotor = ConnectionManager::motorMappings[i + 1];
		correctedControls.controls[xplaneMotor - 1] = hilControls.controls[i];
	}

	// Enable the override (already enabled)
	//DataRefManager::enableOverride();

	// Override the throttle dataref in X-Plane for all engines at once
	std::string dataRef = "sim/flightmodel/engine/ENGN_thro_use";
	XPLMSetDatavf(XPLMFindDataRef(dataRef.c_str()), correctedControls.controls, 0, 16);
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

