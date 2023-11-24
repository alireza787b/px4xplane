// ConfigManager.h

#ifndef CONFIGMANAGER_H
#define CONFIGMANAGER_H

#include <string>
#include <map>
#include <vector>
#include "SimpleIni.h"

enum ActuatorDataType { UNKNOWN, FLOAT_SINGLE, FLOAT_ARRAY };

struct ActuatorConfig {
    std::string datarefName;
    ActuatorDataType dataType;
    std::vector<int> arrayIndices;
    std::pair<float, float> range;
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



};

#endif // CONFIGMANAGER_H
