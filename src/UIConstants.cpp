/**
 * @file UIConstants.cpp
 * @brief Implementation of X-Plane Native Color System for PX4-XPlane Plugin
 *
 * Provides integration with X-Plane's native color datarefs for consistent
 * UI appearance that matches the simulator's theme and user preferences.
 *
 * @author Alireza Ghaderi
 * @copyright Copyright (c) 2025 Alireza Ghaderi. All rights reserved.
 * @license MIT License
 * @url https://github.com/alireza787b/px4xplane
 */

#include "UIConstants.h"
#include "XPLMDataAccess.h"
#include "XPLMGraphics.h"  // For font constants
#include "XPLMUtilities.h"
#include <cstring>

using namespace UIConstants;

// =================================================================
// FONT SYSTEM IMPLEMENTATION
// =================================================================

int Fonts::PRIMARY = 0;
int Fonts::HEADER = 0;
int Fonts::CONTENT = 0;
int Fonts::FOOTER = 0;

void Fonts::initialize() {
    PRIMARY = xplmFont_Proportional;
    HEADER = xplmFont_Proportional;
    CONTENT = xplmFont_Proportional;
    FOOTER = xplmFont_Proportional;
}

// =================================================================
// TAB SYSTEM IMPLEMENTATION
// =================================================================

const char* Tabs::getTabName(int index) {
    static const char* TAB_NAMES[TAB_COUNT] = {
        "Connection",    // Tab 0: Connection status, system info
        "Position",      // Tab 1: GPS, attitude, navigation  
        "Sensors",       // Tab 2: IMU, pressure, temperature
        "Controls",      // Tab 3: RC inputs, actuators
        "Mixing"         // Tab 4: Airframe config visualization
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
        "RC Inputs, Actuators & Aircraft",
        "Airframe Configuration & DataRef Mapping"
    };

    if (index >= 0 && index < TAB_COUNT) {
        return TAB_DESCRIPTIONS[index];
    }
    return "Unknown tab";
}

// =================================================================
// STATIC MEMBER INITIALIZATION  
// =================================================================

XPLMDataRef XPlaneColors::s_menuTextRef = nullptr;
XPLMDataRef XPlaneColors::s_menuTextDisabledRef = nullptr;
XPLMDataRef XPlaneColors::s_tabFrontRef = nullptr;
XPLMDataRef XPlaneColors::s_tabBackRef = nullptr;
XPLMDataRef XPlaneColors::s_captionTextRef = nullptr;
XPLMDataRef XPlaneColors::s_listTextRef = nullptr;
XPLMDataRef XPlaneColors::s_menuHiliteRef = nullptr;
XPLMDataRef XPlaneColors::s_uiScaleRef = nullptr;
bool XPlaneColors::s_initialized = false;

// =================================================================
// X-PLANE COLOR SYSTEM IMPLEMENTATION
// =================================================================

void XPlaneColors::initialize() {
    if (s_initialized) {
        return; // Already initialized
    }

    // Initialize fonts first
    Fonts::initialize();

    // Initialize X-Plane color datarefs
    s_menuTextRef = XPLMFindDataRef("sim/graphics/colors/menu_text_rgb");
    s_menuTextDisabledRef = XPLMFindDataRef("sim/graphics/colors/menu_text_disabled_rgb");
    s_tabFrontRef = XPLMFindDataRef("sim/graphics/colors/tab_front_rgb");
    s_tabBackRef = XPLMFindDataRef("sim/graphics/colors/tab_back_rgb");
    s_captionTextRef = XPLMFindDataRef("sim/graphics/colors/caption_text_rgb");
    s_listTextRef = XPLMFindDataRef("sim/graphics/colors/list_text_rgb");
    s_menuHiliteRef = XPLMFindDataRef("sim/graphics/colors/menu_hilite_rgb");
    s_uiScaleRef = XPLMFindDataRef("sim/graphics/misc/user_interface_scale");

    // Verify critical datarefs are available
    if (!s_menuTextRef || !s_tabFrontRef || !s_tabBackRef) {
        XPLMDebugString("px4xplane: Warning - Some X-Plane color datarefs not available, using fallbacks\n");
    }

    if (!s_uiScaleRef) {
        XPLMDebugString("px4xplane: Warning - UI scale dataref not available, using scale factor 1.0\n");
    }

    s_initialized = true;
    XPLMDebugString("px4xplane: X-Plane native color system initialized successfully\n");
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

void XPlaneColors::getMenuTextColor(float color[3]) {
    if (!s_initialized || !s_menuTextRef) {
        // Fallback to white if dataref not available
        color[0] = 1.0f; color[1] = 1.0f; color[2] = 1.0f;
        return;
    }

    XPLMGetDatavf(s_menuTextRef, color, 0, 3);

    // Ensure color values are valid
    for (int i = 0; i < 3; i++) {
        if (color[i] < 0.0f) color[i] = 0.0f;
        if (color[i] > 1.0f) color[i] = 1.0f;
    }
}

void XPlaneColors::getMenuTextDisabledColor(float color[3]) {
    if (!s_initialized || !s_menuTextDisabledRef) {
        // Fallback to gray if dataref not available
        color[0] = 0.6f; color[1] = 0.6f; color[2] = 0.6f;
        return;
    }

    XPLMGetDatavf(s_menuTextDisabledRef, color, 0, 3);

    // Ensure color values are valid
    for (int i = 0; i < 3; i++) {
        if (color[i] < 0.0f) color[i] = 0.0f;
        if (color[i] > 1.0f) color[i] = 1.0f;
    }
}

void XPlaneColors::getTabActiveColor(float color[3]) {
    if (!s_initialized || !s_tabFrontRef) {
        // Fallback to light blue if dataref not available
        color[0] = 0.9f; color[1] = 0.95f; color[2] = 1.0f;
        return;
    }

    XPLMGetDatavf(s_tabFrontRef, color, 0, 3);

    // Ensure color values are valid
    for (int i = 0; i < 3; i++) {
        if (color[i] < 0.0f) color[i] = 0.0f;
        if (color[i] > 1.0f) color[i] = 1.0f;
    }
}

void XPlaneColors::getTabInactiveColor(float color[3]) {
    if (!s_initialized || !s_tabBackRef) {
        // Fallback to gray if dataref not available  
        color[0] = 0.6f; color[1] = 0.6f; color[2] = 0.7f;
        return;
    }

    XPLMGetDatavf(s_tabBackRef, color, 0, 3);

    // Ensure color values are valid
    for (int i = 0; i < 3; i++) {
        if (color[i] < 0.0f) color[i] = 0.0f;
        if (color[i] > 1.0f) color[i] = 1.0f;
    }
}

void XPlaneColors::getCaptionTextColor(float color[3]) {
    if (!s_initialized || !s_captionTextRef) {
        // Fallback to white if dataref not available
        color[0] = 1.0f; color[1] = 1.0f; color[2] = 1.0f;
        return;
    }

    XPLMGetDatavf(s_captionTextRef, color, 0, 3);

    // Ensure color values are valid
    for (int i = 0; i < 3; i++) {
        if (color[i] < 0.0f) color[i] = 0.0f;
        if (color[i] > 1.0f) color[i] = 1.0f;
    }
}

void XPlaneColors::getListTextColor(float color[3]) {
    if (!s_initialized || !s_listTextRef) {
        // Fallback to light gray if dataref not available
        color[0] = 0.9f; color[1] = 0.9f; color[2] = 0.9f;
        return;
    }

    XPLMGetDatavf(s_listTextRef, color, 0, 3);

    // Ensure color values are valid
    for (int i = 0; i < 3; i++) {
        if (color[i] < 0.0f) color[i] = 0.0f;
        if (color[i] > 1.0f) color[i] = 1.0f;
    }
}

void XPlaneColors::getHighlightColor(float color[3]) {
    if (!s_initialized || !s_menuHiliteRef) {
        // Fallback to bright blue if dataref not available
        color[0] = 0.3f; color[1] = 0.7f; color[2] = 1.0f;
        return;
    }

    XPLMGetDatavf(s_menuHiliteRef, color, 0, 3);

    // Ensure color values are valid
    for (int i = 0; i < 3; i++) {
        if (color[i] < 0.0f) color[i] = 0.0f;
        if (color[i] > 1.0f) color[i] = 1.0f;
    }
}