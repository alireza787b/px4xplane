// ConfigManager.cpp

#include "ConfigManager.h"
#include "XPLMUtilities.h"
#include "XPLMPlugin.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <regex>


// Maps PX4 motor numbers to X-Plane motor numbers. Used in multirotor configurations.
std::map<int, int> ConfigManager::motorMappingsPX4toXPlane;

// Maps X-Plane motor numbers to PX4 motor numbers. Facilitates bidirectional lookup.
std::map<int, int> ConfigManager::motorMappingsXPlanetoPX4;

// Name of the configuration, typically used for display purposes.
std::string ConfigManager::configName;

// Version of the configuration, indicating the version of setup or file format.
std::string ConfigManager::configVersion;

// Type of the configuration (e.g., Multirotor, FixedWing) as specified in the config file.
std::string ConfigManager::configType;

// Numeric code representing the type of configuration, useful for conditional logic. It willl assigned in code . not by user!
uint8_t ConfigManager::configTypeCode;

// Stores configurations for actuators, indexed by their channel number.
std::map<int, ActuatorConfig> ConfigManager::actuatorConfigs;




/**
 * Loads and parses the configuration from the 'config.ini' file.
 * This function initializes the configuration by loading global settings and then
 * determines which specific configuration (Multirotor or FixedWing) to parse.
 * Based on the type of configuration, it delegates the parsing to the respective
 * parse function. It also resolves and logs the path to the configuration file.
 */
void ConfigManager::loadConfiguration() {
    // Initialize the SimpleIni object and set it to handle Unicode
    CSimpleIniA ini;
    ini.SetUnicode();

    std::string configFilePath = getConfigFilePath();
    XPLMDebugString(("px4xplane: Loading config file from: " + configFilePath + "\n").c_str());

    // Load the INI file
    SI_Error rc = ini.LoadFile(configFilePath.c_str());
    if (rc < 0) {
        XPLMDebugString(("px4xplane: Unable to open config file: " + configFilePath + "\n").c_str());
        return;
    }

    // Load general configurations
    // These are global settings not within any specific section
    configName = ini.GetValue("", "config_name", "");
    configVersion = ini.GetValue("", "config_version", "");
    configType = ini.GetValue("", "active_config_type", "");
    

    if (configType == "Multirotor") {
    configTypeCode = 1;
    parseMultirotorConfig(ini);
} else if (configType == "FixedWing") {
    configTypeCode = 2;
    parseFixedWingConfig(ini);
}

    XPLMDebugString("px4xplane: Config file loaded successfully.\n");
}


/**
 * Resolves and returns the full file path of the 'config.ini' file.
 * This function is used to determine the location of the configuration file
 * relative to the plugin's directory. It is essential for loading the correct
 * configuration file.
 *
 * @return The full file path to the 'config.ini' file.
 */
std::string ConfigManager::getConfigFilePath() {
    char filePath[512];
    XPLMGetPluginInfo(XPLMGetMyID(), nullptr, filePath, nullptr, nullptr);
    std::string path(filePath);
    size_t lastSlash = path.find_last_of("/\\");
    std::string finalPath = path.substr(0, lastSlash + 1) + "config.ini";
    XPLMDebugString(("px4xplane: Resolved config file path: " + finalPath + "\n").c_str());
    return finalPath;
}

/**
 * Retrieves the corresponding PX4 motor number for a given X-Plane motor number.
 * This function is used in the mapping between X-Plane and PX4 motor numbers,
 * particularly useful when translating motor commands from one system to another.
 *
 * @param xPlaneMotorNumber The motor number in X-Plane.
 * @return Corresponding PX4 motor number, or -1 if not found.
 */
int ConfigManager::getPX4MotorFromXPlane(int xPlaneMotorNumber) {
    auto it = motorMappingsXPlanetoPX4.find(xPlaneMotorNumber);
    return it != motorMappingsXPlanetoPX4.end() ? it->second : -1;
}

/**
 * Retrieves the corresponding X-Plane motor number for a given PX4 motor number.
 * This function is used in the mapping between PX4 and X-Plane motor numbers,
 * aiding in translating motor commands from PX4 to the X-Plane environment.
 *
 * @param px4MotorNumber The motor number in PX4.
 * @return Corresponding X-Plane motor number, or -1 if not found.
 */
int ConfigManager::getXPlaneMotorFromPX4(int px4MotorNumber) {
    auto it = motorMappingsPX4toXPlane.find(px4MotorNumber);
    return it != motorMappingsPX4toXPlane.end() ? it->second : -1;
}


// Simple getters for configuration properties like name, version, type, and type code.
// These functions provide a clean interface for accessing configuration details.
std::string ConfigManager::getConfigName() {
    return configName;
}

std::string ConfigManager::getConfigVersion() {
    return configVersion;
}

std::string ConfigManager::getConfigType() {
    return configType;
}

uint8_t ConfigManager::getConfigTypeCode(){
    return configTypeCode;
}



/**
 * Parses the Multirotor configuration section from a configuration file.
 * This function first reads the number of motors configured for the multirotor setup.
 * It then iterates through each motor, mapping PX4 motor numbers to X-Plane motor numbers.
 * This mapping is essential for ensuring that the motors are controlled correctly in
 * the simulation environment.
 *
 * The function retrieves the number of motors from the 'num_motors' key in the 'Multirotor'
 * section. Then, for each motor, it reads its corresponding X-Plane motor number using a
 * 'motorX' key, where X is the motor number. These mappings are stored in two dictionaries:
 * motorMappingsPX4toXPlane and motorMappingsXPlanetoPX4, facilitating bidirectional lookup.
 *
 * @param ini Reference to CSimpleIniA object representing the loaded ini file.
 */
void ConfigManager::parseMultirotorConfig(CSimpleIniA& ini) {
    std::string numMotorsStr = ini.GetValue("Multirotor", "num_motors", "0");
    int numMotors = std::stoi(numMotorsStr);

    for (int i = 1; i <= numMotors; ++i) {
        std::string motorKey = "motor" + std::to_string(i);
        std::string motorValue = ini.GetValue("Multirotor", motorKey.c_str(), "0");
        int xPlaneMotor = std::stoi(motorValue);
        motorMappingsPX4toXPlane[i] = xPlaneMotor;
        motorMappingsXPlanetoPX4[xPlaneMotor] = i;
    }
}


/**
 * Parses the FixedWing configuration section from a configuration file.
 * This function iterates through a predefined number of channels (in this case, 16),
 * attempting to read and parse the configuration for each channel from the ini file.
 * The parsing for each channel's configuration string is handled by the
 * parseChannelValue function. The parsed configuration is stored in the actuatorConfigs
 * array, with each index corresponding to a channel.
 *
 * The function reads the configuration values using a key format "channelX", where X
 * is the channel number. It then checks if the value is non-empty before parsing.
 * Each parsed configuration includes dataref name, data type, array indices (if applicable),
 * and range values, which are extracted and stored in an ActuatorConfig object.
 *
 * @param ini Reference to CSimpleIniA object representing the loaded ini file.
 */
void ConfigManager::parseFixedWingConfig(CSimpleIniA& ini) {
    XPLMDebugString("px4xplane: Starting to parse FixedWing configuration.\n");

    for (int channel = 0; channel < 16; channel++) {
        std::string key = "channel" + std::to_string(channel);
        std::string value = ini.GetValue("FixedWing", key.c_str(), "");

        XPLMDebugString(("px4xplane: Parsing channel " + std::to_string(channel) + " with value: " + value + "\n").c_str());

        if (!value.empty()) {
            ActuatorConfig config;
            parseChannelValue(value, config);
            actuatorConfigs[channel] = config;
        }
    }

    XPLMDebugString("px4xplane: Finished parsing FixedWing configuration.\n");
}

/**
 * Parses an individual channel value string into the ActuatorConfig structure.
 * This function is enhanced to handle multiple datarefs per channel, allowing for more
 * complex actuator configurations. Each dataref configuration within a channel is separated
 * by the '|' delimiter. The function iterates over these configurations, parsing and storing
 * each one as a DatarefConfig object within the ActuatorConfig.
 *
 * The parsing process involves splitting the channel value string into individual dataref
 * configurations and then further splitting each configuration into its components (dataref name,
 * data type, array indices, and range). These components are then used to construct a DatarefConfig
 * object, which is added to the ActuatorConfig.
 *
 * @param value The string containing the channel's configuration data.
 * @param config Reference to ActuatorConfig where the parsed data will be stored.
 */
void ConfigManager::parseChannelValue(const std::string& value, ActuatorConfig& config) {
    std::istringstream channelStream(value);
    std::string datarefConfigStr;

    // Iterate over each dataref configuration separated by '|'
    while (std::getline(channelStream, datarefConfigStr, '|')) {
        trimWhitespace(datarefConfigStr);
        std::istringstream datarefStream(datarefConfigStr);
        std::string token;
        std::vector<std::string> tokens;

        // Parse each component of the dataref configuration
        while (std::getline(datarefStream, token, ',')) {
            trimWhitespace(token);
            tokens.push_back(token);
        }

        // Construct DatarefConfig from parsed tokens
        if (tokens.size() >= 4) {
            std::string datarefName = tokens[0];
            ActuatorDataType dataType = stringToDataType(tokens[1]);
            std::vector<int> arrayIndices = parseArrayIndices(tokens[2]);
            std::pair<float, float> range = parseRange(tokens[3]);

            DatarefConfig datarefConfig(datarefName, dataType, arrayIndices, range);
            config.addDatarefConfig(datarefConfig);
        }
        else {
            XPLMDebugString(("px4xplane: Incomplete dataref configuration: " + datarefConfigStr + "\n").c_str());
        }
    }
}




/**
 * Trims leading and trailing whitespace from a string.
 * This utility function removes any whitespace characters from the beginning
 * and end of a given string, which is useful for cleaning up tokenized parts
 * of a configuration line.
 *
 * @param str Reference to the string that needs whitespace trimming.
 */
void ConfigManager::trimWhitespace(std::string& str) {
    str.erase(0, str.find_first_not_of(" \t\n\r\f\v")); // Trim leading whitespace
    str.erase(str.find_last_not_of(" \t\n\r\f\v") + 1); // Trim trailing whitespace
}

/**
 * Parses the array indices part of a dataref configuration.
 * This function is designed to extract array indices from a string token, typically representing
 * part of a dataref configuration. It handles the parsing of indices specified in a bracketed,
 * space-separated format (e.g., "[0 1 2]"). The function returns a vector of integers, each
 * representing an array index. This is particularly useful for configurations where a dataref
 * is an array, and specific indices within that array need to be accessed or modified.
 *
 * The function checks for the presence of brackets to identify the indices string and then
 * iterates over each index, converting it from a string to an integer. If no indices are specified
 * (indicated by a "0"), an empty vector is returned.
 *
 * @param token The string containing the array indices in bracketed format.
 * @return A vector of integers representing the parsed array indices.
 */
std::vector<int> ConfigManager::parseArrayIndices(const std::string& token) {
    std::vector<int> indices;

    // Check if the token is not just a "0" (indicating no indices)
    if (token != "0") {
        size_t start_bracket = token.find('[');
        size_t end_bracket = token.find(']');

        // Ensure the presence of both opening and closing brackets
        if (start_bracket != std::string::npos && end_bracket != std::string::npos) {
            std::string indicesStr = token.substr(start_bracket + 1, end_bracket - start_bracket - 1);
            std::istringstream arrayStream(indicesStr);
            std::string arrayIndex;

            // Iterate over each index within the brackets
            while (std::getline(arrayStream, arrayIndex, ' ')) {
                trimWhitespace(arrayIndex);
                if (!arrayIndex.empty()) {
                    // Convert the index from string to integer and add to the vector
                    indices.push_back(std::stoi(arrayIndex));
                }
            }
        }
        else {
            XPLMDebugString(("px4xplane: Invalid array indices format: " + token + "\n").c_str());
        }
    }

    return indices;
}



/**
 * Parses the range part of a dataref configuration.
 * This function is responsible for extracting the minimum and maximum values of a range from a string token.
 * The range is expected to be in a specific format, enclosed in brackets and separated by a space (e.g., "[0.0 1.0]").
 * It returns a pair of floats representing these min and max values. This is crucial for scaling actuator commands
 * to the appropriate range as required by the simulation environment or the specific actuator being controlled.
 *
 * The function uses regular expressions to parse the range string, ensuring that the format is correctly followed.
 * If the token matches the expected format, the min and max values are extracted and converted from strings to floats.
 * In case of an invalid format, the function returns a default range (0.0, 0.0), and a debug message is logged.
 *
 * @param token The string containing the range in the expected format.
 * @return A pair of floats representing the parsed min and max values of the range.
 */
std::pair<float, float> ConfigManager::parseRange(const std::string& token) {
    std::pair<float, float> range(0.0f, 0.0f); // Default range
    std::regex rangeRegex(R"(\[(\S+)\s+(\S+)\])");
    std::smatch matches;

    // Check if the token matches the range format
    if (std::regex_search(token, matches, rangeRegex) && matches.size() == 3) {
        try {
            range.first = std::stof(matches[1].str());
            range.second = std::stof(matches[2].str());
        }
        catch (const std::exception& e) {
            XPLMDebugString(("px4xplane: Error parsing range: " + token + ", Exception: " + e.what() + "\n").c_str());
        }
    }
    else {
        XPLMDebugString(("px4xplane: Invalid range format: " + token + "\n").c_str());
    }

    return range;
}




/**
 * Converts a string representation of a data type to its corresponding ActuatorDataType enum.
 * This function maps string descriptions of data types (like "float" or "floatArray")
 * to their respective enumerated values. This is useful for interpreting the data type
 * part of a channel configuration line.
 *
 * @param typeStr The string representing the data type.
 * @return The corresponding ActuatorDataType enum value.
 */
ActuatorDataType ConfigManager::stringToDataType(const std::string& typeStr) {
    if (typeStr == "float") return FLOAT_SINGLE;
    else if (typeStr == "floatArray") return FLOAT_ARRAY;
    return UNKNOWN;
}
