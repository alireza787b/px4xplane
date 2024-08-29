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

/**
 * @brief Size of the median filter window.
 *
 * This constant defines the size of the window used for the median filter.
 * The median filter is applied to the sensor data (e.g., accelerometer, velocity)
 * to reduce the impact of outliers or noise spikes. A larger window size provides
 * more smoothing but can introduce lag, while a smaller window reacts more quickly
 * to changes but may be less effective at eliminating noise. The optimal window
 * size should be chosen based on the specific application and expected noise characteristics.
 */
int ConfigManager::MEDIAN_FILTER_WINDOW_SIZE = 3;

/**
 * @brief Flag for enabling/disabling velocity filtering.
 *
 * This flag controls whether the velocity data from the simulation is filtered
 * before being sent to the EKF in the PX4 firmware. When enabled, the velocity data
 * undergoes both median and low-pass filtering to smooth the input and reduce noise.
 * This is particularly important in scenarios where the raw simulation data may
 * experience spikes or small jumps that could negatively affect the EKF's state estimation.
 */
bool ConfigManager::filter_velocity_enabled = false;

/**
 * @brief Alpha value for velocity filtering.
 *
 * This variable controls the strength of the low-pass filter applied to the velocity data.
 * The alpha value should be between 0 and 1, where a higher value retains more of the current
 * data point (less smoothing), and a lower value provides more smoothing (more reliance on
 * previous data points). Tuning this parameter helps balance the responsiveness of the velocity
 * data with the need for noise reduction, depending on the characteristics of the simulation.
 */
float ConfigManager::velocity_filter_alpha = 0.99f;


/**
 * @brief Flag for enabling/disabling accelerometer filtering.
 *
 * This flag controls whether the accelerometer data from the simulation is filtered
 * before being sent to the EKF in the PX4 firmware. Similar to velocity filtering,
 * this helps to smooth the input data and reduce the impact of noise or spikes, ensuring
 * more reliable state estimation in the EKF. When disabled, the raw accelerometer data
 * is used without filtering.
 */
bool ConfigManager::filter_accel_enabled = false;

/**
 * @brief Alpha value for accelerometer filtering.
 *
 * This variable controls the strength of the low-pass filter applied to the accelerometer data.
 * Similar to the velocity filter, the alpha value should be between 0 and 1. A higher alpha value
 * makes the filter more responsive (less smoothing), while a lower alpha value smooths the data more
 * (more reliance on previous data points). This parameter should be tuned based on the noise
 * characteristics of the simulation and the desired filter behavior.
 */
float ConfigManager::accel_filter_alpha = 0.99f;


/**
 * @brief Flag for enabling/disabling barometer filtering.
 *
 * This flag controls whether the barometric pressure data from the simulation is filtered
 * before being sent to the EKF in the PX4 firmware. When enabled, both low-pass and median filtering
 * are applied to the barometric pressure data to smooth the input and reduce noise. This ensures
 * more stable pressure readings and reduces the impact of noisy sensor data on state estimation.
 */
bool ConfigManager::filter_barometer_enabled = false;

/**
 * @brief Alpha value for barometer filtering.
 *
 * This variable controls the strength of the low-pass filter applied to the barometric pressure data.
 * The alpha value should be between 0 and 1, where a higher value retains more of the current data point
 * (less smoothing), and a lower value provides more smoothing (more reliance on previous data points).
 * Tuning this parameter helps balance the responsiveness of the pressure data with the need for noise reduction.
 */
float ConfigManager::barometer_filter_alpha = 0.99f;


// Stores configurations for actuators, indexed by their channel number.
std::map<int, ActuatorConfig> ConfigManager::actuatorConfigs;

/**
 * @brief Static member to track motor equipped with prop brakes.
 *
 * This bitset holds the brake status for up to 8 motor, where each bit represents an motor.
 * A set bit indicates that the corresponding motor has a prop brake enabled.
 */
std::bitset<ConfigManager::MAX_MOTORS> ConfigManager::motorsWithBrakes; // Definition of the static member





/**
 * @brief Loads and parses the configuration from the 'config.ini' file.
 *
 * This function initializes the configuration by loading global settings and then
 * determines which specific configuration (e.g., Multirotor or FixedWing) to parse.
 * It delegates the parsing to the respective parse function based on the type of configuration.
 * Additionally, it configures actuator brakes for specific actuators based on the loaded configuration.
 * It also resolves and logs the path to the configuration file.
 *
 * The process involves:
 * 1. Initializing the SimpleIni object and setting it to handle Unicode.
 * 2. Loading the INI file and handling any errors encountered.
 * 3. Retrieving and storing the general configuration settings.
 * 4. Configuring motor brakes based on the specific requirements of the loaded configuration.
 * 5. Parsing additional configuration details specific to the selected airframe or setup.
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

    // Configure motor brakes based on the loaded configuration
    configureMotorBrakes(ini);
    
    parseConfig(ini);


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


// Simple getters for configuration properties like name
// These functions provide a clean interface for accessing configuration details.
std::string ConfigManager::getConfigName() {
    return configName;
}



/**
 * Retrieves the name of the selected airframe configuration from the config.ini file.
 * This function is responsible for determining which airframe configuration is currently
 * selected by the user. It reads the 'config_name' value from the config.ini file, which
 * should correspond to one of the defined airframe sections in the file.
 *
 * If the 'config_name' is not specified or is empty, an error message is logged, and an
 * empty string is returned. This function is crucial for ensuring that the correct airframe
 * configuration is loaded and parsed.
 *
 * @param ini Reference to CSimpleIniA object representing the loaded ini file.
 * @return The name of the selected airframe configuration, or an empty string if not specified.
 */
std::string ConfigManager::getSelectedAirframeName(CSimpleIniA& ini) {
    // Retrieve the selected configuration name
    std::string selectedConfig = ini.GetValue("", "config_name", "");
    if (selectedConfig.empty()) {
        XPLMDebugString("px4xplane: No configuration name specified in config.ini.\n");
    }
    return selectedConfig;
}


/**
 * Parses the configuration for the selected airframe from the config.ini file.
 * This function dynamically parses the configuration section corresponding to the
 * airframe selected by the user. It iterates through a predefined number of channels
 * (16 in this implementation) and attempts to read and parse the configuration for
 * each channel. The parsed configuration is stored in the actuatorConfigs map, indexed
 * by the channel number.
 *
 * The function first retrieves the selected airframe name using getSelectedAirframeName.
 * It then reads the configuration values for each channel using a key format "channelX",
 * where X is the channel number. If a channel configuration is found, it is parsed and
 * stored; otherwise, a log message is generated.
 *
 * This approach allows for flexible and dynamic configuration parsing, supporting various
 * airframe types with different channel setups.
 *
 * @param ini Reference to CSimpleIniA object representing the loaded ini file.
 */
void ConfigManager::parseConfig(CSimpleIniA& ini) {
    std::string selectedConfig = getSelectedAirframeName(ini);
    if (selectedConfig.empty()) {
        return; // No configuration name specified, or an error occurred.
    }

    XPLMDebugString(("px4xplane: Starting to parse configuration for: " + selectedConfig + "\n").c_str());

    for (int channel = 0; channel < 16; channel++) {
        std::string key = "channel" + std::to_string(channel);
        std::string value = ini.GetValue(selectedConfig.c_str(), key.c_str(), "");

        if (!value.empty()) {
            ActuatorConfig config;
            parseChannelValue(value, config);
            actuatorConfigs[channel] = config;
            XPLMDebugString(("px4xplane: Parsed channel " + std::to_string(channel) + "\n").c_str());
        }
        else {
            XPLMDebugString(("px4xplane: No configuration for channel " + std::to_string(channel) + " in " + selectedConfig + "\n").c_str());
        }
    }

    XPLMDebugString(("px4xplane: Finished parsing configuration for: " + selectedConfig + "\n").c_str());
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

/**
 * @brief Checks if a specific motor has a prop brake enabled.
 *
 * This function checks the motorsWithBrakes bitset to determine if the specified motor
 * is configured to have a prop brake.
 *
 * @param motorIndex The index of the motor to check. Valid indices range from 0 to 7.
 * @return True if the motor has a prop brake enabled, false otherwise or if the index is out of range.
 */
bool ConfigManager::hasPropBrake(int motorIndex) {
    if (motorIndex >= 0 && motorIndex < 8) {
        return motorsWithBrakes.test(motorIndex);
    }
    return false; // Index out of range, no brake
}


/**
 * @brief Configures motors with prop brakes based on the configuration loaded from 'config.ini'.
 *
 * This function reads the 'autoPropBrakes' parameter from the configuration file for the
 * current aircraft configuration. It then parses the list of motor indices specified and sets
 * the corresponding bits in the motorsWithBrakes bitset. Each bit in this bitset corresponds to
 * a motor in X-Plane, and a set bit indicates that the motor is equipped with an auto-prop brake.
 *
 * The function performs the following operations:
 * 1. Resets the motorsWithBrakes bitset to ensure a clean configuration.
 * 2. Retrieves the 'autoPropBrakes' configuration from the 'config.ini' file.
 * 3. Parses the comma-separated list of motor indices and sets the corresponding bits.
 * 4. Logs the final brake configuration for verification.
 *
 * If no 'autoPropBrakes' configuration is found or it's left empty, the function assumes no
 * motors are equipped with brakes and logs an appropriate message.
 *
 * @param ini Reference to the loaded CSimpleIniA object containing the configuration settings.
 */
void ConfigManager::configureMotorBrakes(const CSimpleIniA& ini) {
    motorsWithBrakes.reset();
    std::string brakesConfig = ini.GetValue(configName.c_str(), "autoPropBrakes", "");

    if (!brakesConfig.empty()) {
        std::vector<int> motorIndices = parseMotorIndices(brakesConfig);
        for (int index : motorIndices) {
            motorsWithBrakes.set(index);
        }
    }
    else {
        XPLMDebugString(("px4xplane: No autoPropBrakes specified or parameter not found for configuration: " + configName).c_str());
    }

    // Log the final configuration for verification
    XPLMDebugString(("px4xplane: Motor brakes configured for motors: " + motorsWithBrakes.to_string() + "\n").c_str());
}

/**
 * @brief Parses a comma-separated string of motor indices and returns a vector of integers.
 *
 * This function is a utility designed to convert a string containing a list of comma-separated
 * motor indices into a vector of integers. Each integer represents the index of a motor in
 * X-Plane. The function is robust against invalid input and logs an error for any non-numeric
 * or out-of-range indices encountered in the string.
 *
 * The function performs the following operations:
 * 1. Splits the input string by commas to extract individual motor index strings.
 * 2. Attempts to convert each string to an integer representing the motor index.
 * 3. Validates that the parsed index is within the acceptable range (0 to MAX_MOTORS - 1).
 * 4. Adds valid indices to the vector to be returned and logs errors for invalid entries.
 *
 * This function is typically used in conjunction with configureMotorBrakes to parse the
 * 'autoPropBrakes' configuration parameter.
 *
 * @param indicesStr The string containing the comma-separated list of motor indices.
 * @return A vector of integers representing valid motor indices parsed from the input string.
 */
std::vector<int> ConfigManager::parseMotorIndices(const std::string& indicesStr) {
    std::vector<int> indices;
    std::istringstream stream(indicesStr);
    std::string index;
    while (getline(stream, index, ',')) {
        try {
            int motorIndex = std::stoi(index);
            if (motorIndex >= 0 && motorIndex < MAX_MOTORS) {
                indices.push_back(motorIndex);
            }
            else {
                XPLMDebugString(("px4xplane: Motor index out of range: " + index).c_str());
            }
        }
        catch (const std::invalid_argument& e) {
            XPLMDebugString(("px4xplane: Invalid motor index encountered: " + index).c_str());
        }
    }
    return indices;
}


/**
 * @brief Retrieves a list of airframe configuration names from the config.ini file.
 *
 * This function reads the config.ini file and extracts the names of all sections,
 * which represent different airframe configurations. It uses the SimpleIni library
 * to parse the INI file and collect the section names. Each section name is assumed
 * to correspond to a unique airframe configuration.
 *
 * The function is static and can be called without an instance of ConfigManager.
 * It utilizes the getConfigFilePath method to resolve the path to the config.ini file.
 *
 * Usage example:
 *     std::vector<std::string> airframes = ConfigManager::getAirframeLists();
 *     for (const auto& airframe : airframes) {
 *         // Process each airframe name
 *     }
 *
 * @return A vector of strings, each representing the name of an airframe configuration.
 *         Returns an empty vector if the config.ini file cannot be read or if there are no sections.
 */
std::vector<std::string> ConfigManager::getAirframeLists() {
    CSimpleIniA ini;
    ini.SetUnicode();
    ini.LoadFile(getConfigFilePath().c_str());

    CSimpleIniA::TNamesDepend sections;
    ini.GetAllSections(sections);

    std::vector<std::string> airframeNames;
    for (const auto& section : sections) {
        // Check and ignore the default section which does not represent an airframe.
        if (std::string(section.pItem) != "") {
            airframeNames.push_back(section.pItem);
        }
    }

    return airframeNames;
}



/**
 * @brief Retrieves the name of the currently active airframe configuration.
 *
 * This function reads the 'config.ini' file to obtain the name of the active airframe configuration.
 * It returns the value of the 'config_name' parameter, which indicates the currently selected airframe.
 *
 * @return The name of the active airframe configuration. Returns an empty string if not specified.
 */
std::string ConfigManager::getActiveAirframeName() {
    CSimpleIniA ini;
    ini.SetUnicode();
    ini.LoadFile(getConfigFilePath().c_str());
    return ini.GetValue("", "config_name", "");
}

/**
 * @brief Sets the active airframe configuration to the specified name.
 *
 * This function updates the 'config.ini' file to set the active airframe configuration.
 * It first checks if the specified airframe name exists in the config file. If it does,
 * the 'config_name' parameter is updated with the new airframe name. The updated configuration
 * is then saved back to the 'config.ini' file.
 *
 * @param airframeName The name of the airframe configuration to be set as active.
 */
void ConfigManager::setActiveAirframeName(const std::string& airframeName) {
    CSimpleIniA ini;
    ini.SetUnicode();
    ini.LoadFile(getConfigFilePath().c_str());

    // Check if the airframe name exists in the config
    CSimpleIniA::TNamesDepend sections;
    ini.GetAllSections(sections);
    bool exists = std::any_of(sections.begin(), sections.end(), [&](const CSimpleIniA::Entry& entry) {
        return entry.pItem == airframeName;
        });

    if (exists) {
        ini.SetValue("", "config_name", airframeName.c_str());
        ini.SaveFile(getConfigFilePath().c_str());
        XPLMDebugString("px4xplane: Config File Updated.");

    }
    else {
        XPLMDebugString(("px4xplane: Airframe name not found in config: " + airframeName + "\n").c_str());
    }
}


std::string ConfigManager::getAirframeByIndex(int index) {
    // Retrieve the list of airframes and return the one at the given index
    std::vector<std::string> airframes = getAirframeLists();
    if (index >= 0 && index < airframes.size()) {
        return airframes[index];
    }
    return ""; // Return an empty string if the index is out of range
}

