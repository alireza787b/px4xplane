// ConfigManager.h

#ifndef CONFIGMANAGER_H
#define CONFIGMANAGER_H

#include <string>
#include <map>
#include <vector>
#include <bitset>
#include "SimpleIni.h"

/**
 * Enumeration of actuator data types.
 * This enumeration defines the types of data that can be associated with an actuator.
 * - UNKNOWN: Represents an undefined or unrecognized data type.
 * - FLOAT_SINGLE: Represents a single floating-point value.
 * - FLOAT_ARRAY: Represents an array of floating-point values.
 * This is used to specify how the actuator data should be interpreted and processed.
 */
enum ActuatorDataType { UNKNOWN, FLOAT_SINGLE, FLOAT_ARRAY };

/**
 * Structure representing the configuration of a single data reference (dataref).
 * This structure encapsulates all the necessary information for configuring a dataref associated with an actuator.
 * - datarefName: The name of the dataref.
 * - dataType: The type of data (e.g., single float, array of floats).
 * - arrayIndices: Indices in the case of an array data type.
 * - range: The range of values that the actuator can accept.
 *
 * The constructor initializes the structure with the provided values.
 */
struct DatarefConfig {
    std::string datarefName;
    ActuatorDataType dataType;
    std::vector<int> arrayIndices;
    std::pair<float, float> range;

    DatarefConfig(const std::string& name, ActuatorDataType type, const std::vector<int>& indices, const std::pair<float, float>& rng)
        : datarefName(name), dataType(type), arrayIndices(indices), range(rng) {}
}

;

/**
 * Structure representing the configuration of an actuator.
 * This structure holds a collection of DatarefConfig objects, each representing a specific data reference configuration.
 * Actuators in a simulation environment can be associated with multiple data references, each potentially having different
 * data types, indices, and value ranges.
 *
 * - datarefs: A vector of DatarefConfig objects.
 *
 * The addDatarefConfig method allows adding a new DatarefConfig to the actuator configuration.
 * The getDatarefConfigs method provides access to the stored DatarefConfig objects.
 */
struct ActuatorConfig {
    std::vector<DatarefConfig> datarefs;

    void addDatarefConfig(const DatarefConfig& config) {
        datarefs.push_back(config);
    }

    const std::vector<DatarefConfig>& getDatarefConfigs() const {
        return datarefs;
    }
};



class ConfigManager {
public:
    static void loadConfiguration();
    static int getPX4MotorFromXPlane(int xPlaneMotorNumber);
    static int getXPlaneMotorFromPX4(int px4MotorNumber);
    static std::string getConfigName();
    static std::map<int, ActuatorConfig> actuatorConfigs; // Map channel number to ActuatorConfig
    static std::string getSelectedAirframeName(CSimpleIniA& ini);
    static std::string selectedConfig;
    static bool hasPropBrake(int motorIndex);
    static const int MAX_MOTORS = 8;  // Maximum possible motors in X-Plane
    static void configureMotorBrakes(const CSimpleIniA& ini);
    static std::vector<std::string> getAirframeLists();
    static std::string getActiveAirframeName();
    static void setActiveAirframeName(const std::string& airframeName);
    static std::string getAirframeByIndex(int index);


    static bool filter_velocity_enabled;  // Flag for enabling/disabling velocity filtering
    static float velocity_filter_alpha;   // Alpha value for velocity filtering

    static bool filter_accel_enabled;  // Flag for enabling/disabling accelerometer filtering
    static float accel_filter_alpha;   // Alpha value for accelerometer filtering

    // Filtering configuration for barometric pressure data
    static bool filter_barometer_enabled;  // Flag for enabling/disabling barometric pressure filtering
    static float barometer_filter_alpha;   // Alpha value for barometric pressure filtering

    static bool USE_XPLANE_TIME;

    static bool vibration_noise_enabled;
    static bool rotary_vibration_enabled;
    

    static int MEDIAN_FILTER_WINDOW_SIZE;

    // Debug logging configuration
    static bool debug_verbose_logging;
    static bool debug_log_sensor_timing;
    static bool debug_log_sensor_values;
    static bool debug_log_ekf_innovations;  // NEW: Log EKF innovation data for diagnostics

    // UX configuration
    static bool show_connection_status_hud;  // NEW: Show HUD connection status overlay


private:
    static std::vector<int> parseMotorIndices(const std::string& indicesStr);

    static std::map<int, int> motorMappingsPX4toXPlane;
    static std::map<int, int> motorMappingsXPlanetoPX4;
    static std::bitset<MAX_MOTORS> motorsWithBrakes; // Bitset to hold the brake configuration
    static std::string configName;
    static std::string getConfigFilePath();
    static void parseConfig(CSimpleIniA& ini) ;
    static ActuatorDataType stringToDataType(const std::string& typeStr);
    static std::vector<int> parseArrayIndices(const std::string& token);
    static std::pair<float, float> parseRange(const std::string& token);
    static void parseChannelValue(const std::string& value, ActuatorConfig& config);
    static void trimWhitespace(std::string& str);



};

#endif // CONFIGMANAGER_H
