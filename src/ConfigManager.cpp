// ConfigManager.cpp

#include "ConfigManager.h"
#include "XPLMUtilities.h"
#include "XPLMPlugin.h"
#include <fstream>
#include <sstream>
#include <iostream>

std::map<int, int> ConfigManager::motorMappingsPX4toXPlane;
std::map<int, int> ConfigManager::motorMappingsXPlanetoPX4;
std::string ConfigManager::configName;
std::string ConfigManager::configVersion;
std::string ConfigManager::configType;
uint8_t ConfigManager::configTypeCode;

std::map<int, ActuatorConfig> ConfigManager::actuatorConfigs;


void ConfigManager::loadConfiguration() {
    // Initialize the SimpleIni object and set it to handle Unicode
    CSimpleIniA ini;
    ini.SetUnicode();

    // Resolve the full path to the config.ini file
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



std::string ConfigManager::getConfigFilePath() {
    char filePath[512];
    XPLMGetPluginInfo(XPLMGetMyID(), nullptr, filePath, nullptr, nullptr);
    std::string path(filePath);
    size_t lastSlash = path.find_last_of("/\\");
    std::string finalPath = path.substr(0, lastSlash + 1) + "config.ini";
    XPLMDebugString(("px4xplane: Resolved config file path: " + finalPath + "\n").c_str());
    return finalPath;
}


int ConfigManager::getPX4MotorFromXPlane(int xPlaneMotorNumber) {
    auto it = motorMappingsXPlanetoPX4.find(xPlaneMotorNumber);
    return it != motorMappingsXPlanetoPX4.end() ? it->second : -1;
}

int ConfigManager::getXPlaneMotorFromPX4(int px4MotorNumber) {
    auto it = motorMappingsPX4toXPlane.find(px4MotorNumber);
    return it != motorMappingsPX4toXPlane.end() ? it->second : -1;
}

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



void ConfigManager::parseFixedWingConfig(CSimpleIniA& ini) {
    for (int channel = 0; channel < 16; ++channel) {
        std::string key = "channel" + std::to_string(channel);
        std::string value = ini.GetValue("FixedWing", key.c_str(), "");

        if (!value.empty()) {
            ActuatorConfig config;
            std::istringstream iss(value);
            std::string token;
            int idx = 0;

            while (std::getline(iss, token, ',')) {
                token.erase(0, token.find_first_not_of(" \t\n\r\f\v")); // Trim leading whitespace
                switch (idx) {
                    case 0: // Dataref name
                        config.datarefName = token;
                        break;
                    case 1: // Data type
                        config.dataType = stringToDataType(token);
                        break;
                    case 2: // Array indices
                        if (token != "0") {
                            std::string indices = token.substr(1, token.length() - 2); // Remove brackets
                            std::istringstream arrayStream(indices);
                            std::string arrayIndex;
                            while (std::getline(arrayStream, arrayIndex, ' ')) {
                                config.arrayIndices.push_back(std::stoi(arrayIndex));
                            }
                        }
                        break;
                    case 3: // Range
                        std::string range = token.substr(1, token.length() - 2); // Remove brackets
                        std::istringstream rangeStream(range);
                        std::string rangePart;
                        if (std::getline(rangeStream, rangePart, ',')) {
                            config.range.first = std::stof(rangePart);
                        }
                        if (std::getline(rangeStream, rangePart)) {
                            config.range.second = std::stof(rangePart);
                        }
                        break;
                }
                ++idx;
            }

            actuatorConfigs[channel] = config;
        }
    }
}



ActuatorDataType ConfigManager::stringToDataType(const std::string& typeStr) {
    if (typeStr == "float") return FLOAT_SINGLE;
    else if (typeStr == "floatArray") return FLOAT_ARRAY;
    return UNKNOWN;
}
