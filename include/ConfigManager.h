// ConfigManager.h

#ifndef CONFIGMANAGER_H
#define CONFIGMANAGER_H

#include <string>
#include <map>
#include <vector>
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
};

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
    static std::string getConfigVersion();
    static std::string getConfigType();
    static uint8_t getConfigTypeCode();
    static std::map<int, ActuatorConfig> actuatorConfigs; // Map channel number to ActuatorConfig


private:
    static std::map<int, int> motorMappingsPX4toXPlane;
    static std::map<int, int> motorMappingsXPlanetoPX4;
    static std::string configName;
    static std::string configVersion;
    static std::string configType; // "Multirotor" or "FixedWing" or ...
    static uint8_t ConfigManager::configTypeCode;
    static std::string getConfigFilePath();
    static void parseFixedWingConfig(CSimpleIniA& ini) ;
    static void parseMultirotorConfig(CSimpleIniA& ini) ;
    static ActuatorDataType stringToDataType(const std::string& typeStr);
    static std::vector<int> ConfigManager::parseArrayIndices(const std::string& token);
    static std::pair<float, float> ConfigManager::parseRange(const std::string& token);
    static void ConfigManager::parseChannelValue(const std::string& value, ActuatorConfig& config);
     static void ConfigManager::trimWhitespace(std::string& str);



};

#endif // CONFIGMANAGER_H
