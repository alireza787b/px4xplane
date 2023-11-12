// ConfigManager.cpp

#include "ConfigManager.h"
#include "XPLMUtilities.h"
#include "XPLMPlugin.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include "SimpleIni.h"

std::map<int, int> ConfigManager::motorMappingsPX4toXPlane;
std::map<int, int> ConfigManager::motorMappingsXPlanetoPX4;
std::string ConfigManager::configName;
std::string ConfigManager::configVersion;
std::string ConfigManager::configType;

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

    // Load motor mappings if the active config type is Multirotor
    if (configType == "Multirotor") {
        // Get the number of motors from the Multirotor section
        std::string numMotorsStr = ini.GetValue("Multirotor", "num_motors", "0");
        int numMotors = std::stoi(numMotorsStr);

        // Loop through each motor and load its mapping
        for (int i = 1; i <= numMotors; ++i) {
            std::string motorKey = "motor" + std::to_string(i);
            std::string motorValue = ini.GetValue("Multirotor", motorKey.c_str(), "0");
            int xPlaneMotor = std::stoi(motorValue);
            motorMappingsPX4toXPlane[i] = xPlaneMotor;
            motorMappingsXPlanetoPX4[xPlaneMotor] = i;
        }
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


