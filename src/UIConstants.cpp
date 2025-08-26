/**
 * @file UIConstants.cpp
 * @brief Production-Ready UI Constants Implementation for X-Plane Plugin
 *
 * Implementation of high-contrast, readable UI system optimized for X-Plane's
 * dark theme. Features proper font initialization, runtime constants, and
 * enhanced support for HIL data display and scrollable dialogs.
 *
 * @author Alireza Ghaderi
 * @copyright Copyright (c) 2025 Alireza Ghaderi. All rights reserved.
 * @license MIT License
 * @version 2.5.0 (Production Ready)
 * @url https://github.com/alireza787b/px4xplane
 */

#include "UIConstants.h"
#include "XPLMDataAccess.h"
#include "XPLMGraphics.h"  // For font constants
#include "XPLMUtilities.h"
#include <cstring>
#include <cmath>

using namespace UIConstants;

// =================================================================
// STATIC MEMBER INITIALIZATION  
// =================================================================

XPLMDataRef XPlaneColors::s_uiScaleRef = nullptr;
bool XPlaneColors::s_initialized = false;

// Font constants - initialized at runtime
int Fonts::PRIMARY = 0;
int Fonts::HEADER = 0;
int Fonts::CONTENT = 0;
int Fonts::FOOTER = 0;

// =================================================================
// FONT SYSTEM IMPLEMENTATION
// =================================================================

void Fonts::initialize() {
    // Use xplmFont_Basic for maximum compatibility and reliability
    PRIMARY = xplmFont_Basic;
    FOOTER = xplmFont_Basic;

    // Try to use proportional font for headers and content if available
#if defined(XPLM200)
    HEADER = xplmFont_Proportional;   // Better for headers
    CONTENT = xplmFont_Proportional;  // Better for content
#else
    // Fallback to basic font for older SDK versions
    HEADER = xplmFont_Basic;
    CONTENT = xplmFont_Basic;
#endif

    XPLMDebugString("px4xplane: Font system initialized successfully\n");
}

// =================================================================
// TAB SYSTEM IMPLEMENTATION
// =================================================================

const char* Tabs::getTabName(int index) {
    static const char* TAB_NAMES[TAB_COUNT] = {
        "Connection",    // Tab 0: Connection status, system info
        "Position",      // Tab 1: GPS, attitude, navigation  
        "Sensors",       // Tab 2: IMU, pressure, temperature
        "Controls",      // Tab 3: RC inputs, actuators, HIL data
        "Mixing"         // Tab 4: Airframe config visualization with live HIL
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

// =================================================================
// X-PLANE INTEGRATION IMPLEMENTATION
// =================================================================

void XPlaneColors::initialize() {
    if (s_initialized) {
        return; // Already initialized
    }

    // Initialize fonts first
    Fonts::initialize();

    // Initialize X-Plane UI scale dataref
    s_uiScaleRef = XPLMFindDataRef("sim/graphics/misc/user_interface_scale");

    if (!s_uiScaleRef) {
        XPLMDebugString("px4xplane: Warning - UI scale dataref not available, using scale factor 1.0\n");
    }

    s_initialized = true;
    XPLMDebugString("px4xplane: X-Plane integration initialized successfully with enhanced HIL support\n");
}

float XPlaneColors::getUIScale() {
    if (!s_initialized || !s_uiScaleRef) {
        return 1.0f; // Default scale if not available
    }

    float scale = XPLMGetDataf(s_uiScaleRef);

    // Sanity check - clamp to reasonable values
    if (scale < 0.5f || scale > 3.0f) {
        return 1.0f;
    }

    return scale;
}