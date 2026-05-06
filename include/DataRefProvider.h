#pragma once

#include <vector>

class DataRefProvider {
public:
    virtual ~DataRefProvider() = default;

    virtual float getFloat(const char* dataRefName) const = 0;
    virtual double getDouble(const char* dataRefName) const = 0;
    virtual int getInt(const char* dataRefName) const = 0;
    virtual std::vector<float> getFloatArray(const char* dataRefName) const = 0;
};
