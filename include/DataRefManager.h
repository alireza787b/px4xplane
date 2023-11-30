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
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

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

struct GeodeticPosition {
    float latitude;
    float longitude;
    float altitude;
};



class DataRefManager {
public:
    static constexpr float g_earth = 9.81f;
    static std::vector<DataRefItem> dataRefs;
    static Eigen::Vector3f earthMagneticFieldNED;

    static constexpr float AirDensitySeaLevel = 1.225; // kg/m^3
    static int drawDataRefs(XPLMWindowID in_window_id, int l, int t, float col_white[], int lineOffset);
    static std::vector<float> getFloatArray(const char* dataRefName);
    static float getFloat(const char* dataRef);
    static int getInt(const char* dataRef);
    static double getDouble(const char* dataRef);
    static std::string arrayToString(const std::vector<float>& array); // add this line
    static float mapChannelValue(float value, float minInput, float maxInput, float minOutput, float maxOutput);
    static int drawActuatorControls(XPLMWindowID in_window_id, int l, int t, float col_white[], int lineOffset);
    static void overrideActuators_multirotor();
    static void overrideActuators_fixedwing();
    static void enableOverride();
    static void disableOverride();
    static int drawActualThrottle(XPLMWindowID in_window_id, int l, int t, float col_white[], int lineOffset);
    static Eigen::Vector3f updateEarthMagneticFieldNED(const GeodeticPosition& position);
    static float calculateDistance(const GeodeticPosition& pos1, const GeodeticPosition& pos2);
    static Eigen::Vector3f convertNEDToBody(const Eigen::Vector3f& nedVector, float roll, float pitch, float yaw);
    static void initializeMagneticField();

    static std::string GetFormattedDroneConfig();
    static float scaleActuatorCommand(float input, float inputMin, float inputMax, float outputMin, float outputMax);

    static float lastLatitude;
    static float lastLongitude;
    static float lastAltitude;
    static GeodeticPosition lastPosition;
    static constexpr float UPDATE_THRESHOLD=1000;  // Define a threshold for position change in meters
};

