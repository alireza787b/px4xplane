// DataRefManager.h
#pragma once
#include <vector>
#include <variant>
#include <functional>
#include "XPLMDisplay.h"
#include "XPLMDataAccess.h"
#include "XPLMDisplay.h"
#include <string>
#include<cmath>
#include<vector>


enum DataRefType {
    DRT_FLOAT,
    DRT_DOUBLE,
    DRT_INT,
    DRT_FLOAT_ARRAY
};

struct DataRefItem {
    const char* category;
    const char* title;
    const char* dataRef;
    const char* unit;
    float multiplier;
    DataRefType type;
    std::variant<std::function<float(const char*)>,
        std::function<double(const char*)>,
        std::function<int(const char*)>,
        std::function<std::vector<float>(const char*)>> getter;
};

class DataRefManager {
public:
    static constexpr float g_earth = 9.81f;
    static std::vector<DataRefItem> dataRefs;

    static void drawDataRefs(XPLMWindowID in_window_id, int l, int t, float col_white[], int lineOffset);
    static std::vector<float> getFloatArray(const char* dataRefName);
    static float getFloat(const char* dataRef);
    static int getInt(const char* dataRef);
    static double getDouble(const char* dataRef);
    static std::string arrayToString(const std::vector<float>& array); // add this line
    static float mapChannelValue(float value, float minInput, float maxInput, float minOutput, float maxOutput);
    static std::tuple<float, float, float> convertOGLtoNED(float ogl_vx, float ogl_vy, float ogl_vz, float roll_rad, float pitch_rad, float yaw_rad);
    static void drawActuatorControls(XPLMWindowID in_window_id, int l, int t, float col_white[], int lineOffset);
    static void overrideActuators();
    static void enableOverride();
    static void disableOverride();

};

