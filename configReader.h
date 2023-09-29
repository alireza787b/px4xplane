#pragma once
#include <map>
#include <string>

class ConfigReader {
public:
    static std::map<std::string, std::string> readConfigFile(const std::string& fileName);
};
