#include "ConfigReader.h"
#include <fstream>
#include <sstream>

std::map<std::string, std::string> ConfigReader::readConfigFile(const std::string& fileName) {
    std::ifstream inFile(fileName);
    std::string line;
    std::map<std::string, std::string> settings;

    while (std::getline(inFile, line)) {
        std::istringstream iss(line);
        std::string key;
        if (std::getline(iss, key, '=')) {
            std::string value;
            if (std::getline(iss, value)) settings[key] = value;
        }
    }
    return settings;
}
