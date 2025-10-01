/**
 * @file UIConstants.cpp
 * @brief UI Constants Implementation for X-Plane Plugin
 *
 * @author Alireza Ghaderi
 * @copyright Copyright (c) 2025 Alireza Ghaderi. All rights reserved.
 * @license MIT License
 * @version 2.5.0
 */

#include "UIConstants.h"
#include "XPLMDataAccess.h"
#include "XPLMGraphics.h"
#include "XPLMUtilities.h"
#include <cstring>

using namespace UIConstants;

// Static member initialization
XPLMDataRef XPlaneColors::s_uiScaleRef = nullptr;
bool XPlaneColors::s_initialized = false;

// Font constants
int Fonts::PRIMARY = 0;
int Fonts::HEADER = 0;
int Fonts::CONTENT = 0;
int Fonts::FOOTER = 0;

void Fonts::initialize() {
    PRIMARY = xplmFont_Basic;
    FOOTER = xplmFont_Basic;
#if defined(XPLM200)
    HEADER = xplmFont_Proportional;
    CONTENT = xplmFont_Proportional;
#else
    HEADER = xplmFont_Basic;
    CONTENT = xplmFont_Basic;
#endif
    XPLMDebugString("px4xplane: Font system initialized successfully\n");
}

const char* Tabs::getTabName(int index) {
    static const char* TAB_NAMES[TAB_COUNT] = {
        "Connection",
        "Position",
        "Sensors",
        "Controls",
        "Mixing"
    };
    if (index >= 0 && index < TAB_COUNT) {
        return TAB_NAMES[index];
    }
    return "Unknown";
}

const char* Tabs::getTabDescription(int index) {
    static const char* TAB_DESCRIPTIONS[TAB_COUNT] = {
        "System Status & Connection Info",
        "GPS, Attitude & Navigation Data",
        "IMU, Environment & Sensor Data",
        "RC Inputs, HIL Actuators & Aircraft Config",
        "Airframe Configuration & Live HIL Mapping"
    };
    if (index >= 0 && index < TAB_COUNT) {
        return TAB_DESCRIPTIONS[index];
    }
    return "Unknown tab";
}

void XPlaneColors::initialize() {
    if (s_initialized) {
        return;
    }

    Fonts::initialize();
    s_uiScaleRef = XPLMFindDataRef("sim/graphics/misc/user_interface_scale");

    if (!s_uiScaleRef) {
        XPLMDebugString("px4xplane: Warning - UI scale dataref not available, using scale factor 1.0\n");
    }

    s_initialized = true;
    XPLMDebugString("px4xplane: X-Plane integration initialized successfully\n");
}

float XPlaneColors::getUIScale() {
    if (!s_initialized || !s_uiScaleRef) {
        return 1.0f;
    }

    float scale = XPLMGetDataf(s_uiScaleRef);

    if (scale < 0.5f || scale > 3.0f) {
        return 1.0f;
    }

    return scale;
}
