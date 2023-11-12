// ConfigManager.h

#ifndef CONFIGMANAGER_H
#define CONFIGMANAGER_H

#include <string>
#include <map>

class ConfigManager {
public:
    static void loadConfiguration();
    static int getPX4MotorFromXPlane(int xPlaneMotorNumber);
    static int getXPlaneMotorFromPX4(int px4MotorNumber);

    static std::string getConfigName();
    static std::string getConfigVersion();
    static std::string getConfigType();

private:
    static std::map<int, int> motorMappingsPX4toXPlane;
    static std::map<int, int> motorMappingsXPlanetoPX4;
    static std::string configName;
    static std::string configVersion;
    static std::string configType; // "Multirotor" or "FixedWing"

    static std::string getConfigFilePath();
};

#endif // CONFIGMANAGER_H
