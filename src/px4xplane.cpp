/**
 * @file px4xplane_main.cpp
 * @brief PX4-XPlane Interface Plugin with Fixed Window Management
 *
 * @author Alireza Ghaderi
 * @copyright Copyright (c) 2025 Alireza Ghaderi. All rights reserved.
 * @license MIT License
 * @version 2.5.0
 * @url https://github.com/alireza787b/px4xplane
 */

#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMDataAccess.h"
#include "XPLMMenus.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"
#include <string>
#include <stdio.h>
#include <vector>
#include <functional>
#include <variant>
#include <fstream>
#include <sstream>
#include <map>
#include "ConfigReader.h"
#include "DataRefManager.h"
#include "ConnectionManager.h"
#include "ConfigManager.h"
#include <algorithm>
#include <cstring>

#if IBM
#include <windows.h>
#endif
#if LIN || APL
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#endif
#include "MAVLinkManager.h"

#ifndef XPLM300
#error This is made to be compiled against the XPLM300 SDK
#endif
#include <XPLMPlugin.h>

 // ============================================================================
 // GLOBAL VARIABLES
 // ============================================================================

static XPLMWindowID g_window = NULL;
static XPLMWindowID g_about_window = NULL;
static XPLMMenuID g_menu_id;
static XPLMMenuID g_airframes_menu;
static int g_menu_container_idx;
static XPLMCommandRef toggleEnableCmd;

// Menu item identifiers (string-based like the demo)
#define MENU_ITEM_DATA_INSPECTOR "Data Inspector"
#define MENU_ITEM_ABOUT "About"
#define MENU_ITEM_TOGGLE_CONNECTION "Toggle Connection"
#define MENU_ITEM_AIRFRAMES "Airframes"

// Update frequencies
const float BASE_SENSOR_UPDATE_PERIOD = 0.001f;  // 1000 Hz for IMU
const float BASE_GPS_UPDATE_PERIOD = 0.1f;       // 10 Hz for GPS
const float BASE_STATE_QUAT_UPDATE_PERIOD = 0.05f; // 20 Hz
const float BASE_RC_UPDATE_PERIOD = 0.05f;       // 20 Hz

// Timing variables
float timeSinceLastSensorUpdate = 0.0f;
float timeSinceLastGpsUpdate = 0.0f;
float timeSinceLastStateQuaternionUpdate = 0.0f;
float timeSinceLastRcUpdate = 0.0f;
float lastFlightTime = 0.0f;

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

void draw_px4xplane(XPLMWindowID in_window_id, void* in_refcon);
void draw_about_window(XPLMWindowID in_window_id, void* in_refcon);
int handle_main_mouse(XPLMWindowID in_window_id, int x, int y, int is_down, void* in_refcon);
int handle_about_mouse(XPLMWindowID in_window_id, int x, int y, int is_down, void* in_refcon);
void menu_handler(void* in_menu_ref, void* in_item_ref);
void airframes_menu_handler(void* in_menu_ref, void* in_item_ref);
int toggleEnableHandler(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon);
void create_menu();
void create_airframes_menu();
void toggleEnable();
void refresh_airframes_menu();
float MyFlightLoopCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void* inRefcon);

// Window management functions
void show_main_window();
void show_about_window();
void hide_main_window();
void hide_about_window();
void destroy_windows();

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

void debugLog(const char* message) {
    XPLMDebugString("px4xplane: ");
    XPLMDebugString(message);
    XPLMDebugString("\n");
}

// ============================================================================
// WINDOW MANAGEMENT FUNCTIONS
// ============================================================================

void create_main_window() {
    if (g_window != NULL) {
        debugLog("Main window already exists");
        return;
    }

    debugLog("Creating main window...");

    // Get screen dimensions
    int left, bottom, right, top;
    XPLMGetScreenBoundsGlobal(&left, &top, &right, &bottom);

    int screenWidth = right - left;
    int screenHeight = top - bottom;

    // Calculate window size
    int windowWidth = std::max<int>(800, (int)(screenWidth * 0.4));
    int windowHeight = (int)(screenHeight * 0.7);

    // Position window towards left side of screen
    int windowLeft = left + (int)(screenWidth * 0.05);
    int windowBottom = bottom + (int)(screenHeight * 0.15);
    int windowRight = windowLeft + windowWidth;
    int windowTop = windowBottom + windowHeight;

    XPLMCreateWindow_t params;
    memset(&params, 0, sizeof(params));
    params.structSize = sizeof(params);
    params.visible = 1;  // Start visible
    params.drawWindowFunc = draw_px4xplane;
    params.handleMouseClickFunc = handle_main_mouse;
    params.handleRightClickFunc = NULL;
    params.handleMouseWheelFunc = NULL;
    params.handleKeyFunc = NULL;
    params.handleCursorFunc = NULL;
    params.refcon = NULL;
    params.layer = xplm_WindowLayerFloatingWindows;
    params.decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle;
    params.left = windowLeft;
    params.bottom = windowBottom;
    params.right = windowRight;
    params.top = windowTop;

    g_window = XPLMCreateWindowEx(&params);

    if (g_window == NULL) {
        debugLog("ERROR: Failed to create main window!");
        return;
    }

    // Configure window properties
    XPLMSetWindowPositioningMode(g_window, xplm_WindowPositionFree, -1);
    XPLMSetWindowTitle(g_window, "PX4-XPlane Data Inspector v2.5.0");
    XPLMSetWindowResizingLimits(g_window, 800, 500, windowWidth + 300, windowHeight + 200);

    debugLog("Main window created successfully");
}

void create_about_window() {
    if (g_about_window != NULL) {
        debugLog("About window already exists");
        return;
    }

    debugLog("Creating about window...");

    // Get screen dimensions
    int left, bottom, right, top;
    XPLMGetScreenBoundsGlobal(&left, &top, &right, &bottom);

    int screenWidth = right - left;
    int screenHeight = top - bottom;

    // Calculate centered about dialog dimensions
    int dialogWidth = 550;
    int dialogHeight = 400;

    int dialogLeft = left + (screenWidth - dialogWidth) / 2;
    int dialogBottom = bottom + (screenHeight - dialogHeight) / 2;
    int dialogRight = dialogLeft + dialogWidth;
    int dialogTop = dialogBottom + dialogHeight;

    XPLMCreateWindow_t params;
    memset(&params, 0, sizeof(params));
    params.structSize = sizeof(params);
    params.visible = 1;  // Start visible
    params.drawWindowFunc = draw_about_window;
    params.handleMouseClickFunc = handle_about_mouse;
    params.handleRightClickFunc = NULL;
    params.handleMouseWheelFunc = NULL;
    params.handleKeyFunc = NULL;
    params.handleCursorFunc = NULL;
    params.refcon = NULL;
    params.layer = xplm_WindowLayerFloatingWindows;
    params.decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle;
    params.left = dialogLeft;
    params.bottom = dialogBottom;
    params.right = dialogRight;
    params.top = dialogTop;

    g_about_window = XPLMCreateWindowEx(&params);

    if (g_about_window == NULL) {
        debugLog("ERROR: Failed to create about window!");
        return;
    }

    // Configure window properties
    XPLMSetWindowPositioningMode(g_about_window, xplm_WindowPositionFree, -1);
    XPLMSetWindowTitle(g_about_window, "About PX4-XPlane Interface v2.5.0");

    debugLog("About window created successfully");
}

void show_main_window() {
    debugLog("Showing main window...");

    if (g_window == NULL) {
        create_main_window();
    }

    if (g_window != NULL) {
        XPLMSetWindowIsVisible(g_window, 1);
        XPLMBringWindowToFront(g_window);
        debugLog("Main window should now be visible");
    }
    else {
        debugLog("ERROR: Failed to show main window - window is NULL");
    }
}

void show_about_window() {
    debugLog("Showing about window...");

    if (g_about_window == NULL) {
        create_about_window();
    }

    if (g_about_window != NULL) {
        XPLMSetWindowIsVisible(g_about_window, 1);
        XPLMBringWindowToFront(g_about_window);
        debugLog("About window should now be visible");
    }
    else {
        debugLog("ERROR: Failed to show about window - window is NULL");
    }
}

void hide_main_window() {
    if (g_window != NULL) {
        XPLMSetWindowIsVisible(g_window, 0);
        debugLog("Main window hidden");
    }
}

void hide_about_window() {
    if (g_about_window != NULL) {
        XPLMSetWindowIsVisible(g_about_window, 0);
        debugLog("About window hidden");
    }
}

void destroy_windows() {
    if (g_about_window != NULL) {
        XPLMDestroyWindow(g_about_window);
        g_about_window = NULL;
        debugLog("About window destroyed");
    }

    if (g_window != NULL) {
        XPLMDestroyWindow(g_window);
        g_window = NULL;
        debugLog("Main window destroyed");
    }
}

// ============================================================================
// WINDOW DRAWING FUNCTIONS
// ============================================================================

int drawHeader(int l, int t, float col_white[]) {
    char header[512];
    snprintf(header, sizeof(header), "PX4-XPlane Data Inspector v2.5.0");
    XPLMDrawString(col_white, l + 10, t - 20, header, NULL, xplmFont_Proportional);
    return 45;
}

int drawConnectionStatus(int l, int t, float col_white[], int lineOffset) {
    char buf[512];

    // Connection status with visual indicator
    const std::string& status = ConnectionManager::getStatus();
    const char* statusIcon = ConnectionManager::isConnected() ? "[CONNECTED]" : "[DISCONNECTED]";
    snprintf(buf, sizeof(buf), "Status: %s %s", statusIcon, status.c_str());
    XPLMDrawString(col_white, l + 10, t - lineOffset, buf, NULL, xplmFont_Proportional);
    lineOffset += 25;

    // SITL timestep info
    snprintf(buf, sizeof(buf), "SITL Timestep: %.3f ms (%.1f Hz)",
        DataRefManager::SIM_Timestep * 1000.0f, 1.0f / DataRefManager::SIM_Timestep);
    XPLMDrawString(col_white, l + 10, t - lineOffset, buf, NULL, xplmFont_Proportional);
    lineOffset += 30;

    return lineOffset;
}

int drawAircraftConfig(int l, int t, float col_white[], int lineOffset) {
    char buf[512];

    // Section header
    snprintf(buf, sizeof(buf), "--- AIRCRAFT CONFIGURATION ---");
    XPLMDrawString(col_white, l + 10, t - lineOffset, buf, NULL, xplmFont_Proportional);
    lineOffset += 25;

    // Get and display formatted config
    std::string droneConfigStr = DataRefManager::GetFormattedDroneConfig();
    std::istringstream iss(droneConfigStr);
    std::string line;

    while (std::getline(iss, line)) {
        char lineBuffer[512];
        strncpy(lineBuffer, line.c_str(), sizeof(lineBuffer) - 1);
        lineBuffer[sizeof(lineBuffer) - 1] = '\0';
        XPLMDrawString(col_white, l + 20, t - lineOffset, lineBuffer, NULL, xplmFont_Proportional);
        lineOffset += 20;
    }

    return lineOffset + 15;
}

int drawSensorData(int l, int t, float col_white[], int lineOffset) {
    char buf[512];

    // Section header
    snprintf(buf, sizeof(buf), "--- SENSOR DATA ---");
    XPLMDrawString(col_white, l + 10, t - lineOffset, buf, NULL, xplmFont_Proportional);
    lineOffset += 25;

    // Use existing DataRefManager function
    lineOffset = DataRefManager::drawDataRefs(nullptr, l, t, col_white, lineOffset);

    return lineOffset;
}

void drawActuatorData(int l, int t, int r, float col_white[]) {
    char buf[512];

    // Calculate right column position
    int windowWidth = r - l;
    int rightColumnPos = l + (windowWidth / 2) + 20;

    // Section header
    snprintf(buf, sizeof(buf), "--- ACTUATOR CONTROLS ---");
    XPLMDrawString(col_white, rightColumnPos, t - 45, buf, NULL, xplmFont_Proportional);

    // Draw actuator controls
    int rightLineOffset = 70;
    rightLineOffset = DataRefManager::drawActuatorControls(nullptr, rightColumnPos, t, col_white, rightLineOffset);

    // Draw actual throttle
    DataRefManager::drawActualThrottle(nullptr, rightColumnPos, t, col_white, rightLineOffset + 20);
}

void drawFooter(int l, int b, int r, float col_white[]) {
    char footer[512];
    snprintf(footer, sizeof(footer), "Copyright (c) 2025 Alireza Ghaderi | github.com/alireza787b/px4xplane");

    // Center the footer
    int windowWidth = r - l;
    int textWidth = strlen(footer) * 6; // Approximate character width
    int centerX = l + (windowWidth - textWidth) / 2;
    if (centerX < l + 10) centerX = l + 10; // Ensure minimum margin

    XPLMDrawString(col_white, centerX, b + 10, footer, NULL, xplmFont_Proportional);
}

void draw_px4xplane(XPLMWindowID in_window_id, void* in_refcon) {
    if (in_window_id == NULL) return;

    int l, t, r, b;
    XPLMGetWindowGeometry(in_window_id, &l, &t, &r, &b);
    float col_white[] = { 1.0, 1.0, 1.0 };

    // Validate window dimensions
    if (r <= l || t <= b) {
        debugLog("Invalid window dimensions detected");
        return;
    }

    int lineOffset = drawHeader(l, t, col_white);
    lineOffset = drawConnectionStatus(l, t, col_white, lineOffset);
    lineOffset = drawAircraftConfig(l, t, col_white, lineOffset);
    lineOffset = drawSensorData(l, t, col_white, lineOffset);

    // Draw actuator data in right column
    drawActuatorData(l, t, r, col_white);

    // Draw footer
    drawFooter(l, b, r, col_white);
}

void draw_about_window(XPLMWindowID in_window_id, void* in_refcon) {
    if (in_window_id == NULL) return;

    int l, t, r, b;
    XPLMGetWindowGeometry(in_window_id, &l, &t, &r, &b);
    float col_white[] = { 1.0, 1.0, 1.0 };

    // Validate window dimensions
    if (r <= l || t <= b) {
        debugLog("Invalid about window dimensions detected");
        return;
    }

    int lineOffset = 30;
    char buf[512];

    // Title
    snprintf(buf, sizeof(buf), "PX4-XPlane Interface v2.5.0");
    XPLMDrawString(col_white, l + 20, t - lineOffset, buf, NULL, xplmFont_Proportional);
    lineOffset += 40;

    // Description
    snprintf(buf, sizeof(buf), "Hardware-in-the-Loop interface for PX4 SITL and X-Plane 12");
    XPLMDrawString(col_white, l + 20, t - lineOffset, buf, NULL, xplmFont_Proportional);
    lineOffset += 30;

    // Features
    snprintf(buf, sizeof(buf), "Features:");
    XPLMDrawString(col_white, l + 20, t - lineOffset, buf, NULL, xplmFont_Proportional);
    lineOffset += 25;

    snprintf(buf, sizeof(buf), "* Real-time sensor data streaming with realistic noise");
    XPLMDrawString(col_white, l + 30, t - lineOffset, buf, NULL, xplmFont_Proportional);
    lineOffset += 20;

    snprintf(buf, sizeof(buf), "* Professional sensor filtering and physics simulation");
    XPLMDrawString(col_white, l + 30, t - lineOffset, buf, NULL, xplmFont_Proportional);
    lineOffset += 20;

    snprintf(buf, sizeof(buf), "* Multiple airframe configuration support");
    XPLMDrawString(col_white, l + 30, t - lineOffset, buf, NULL, xplmFont_Proportional);
    lineOffset += 20;

    snprintf(buf, sizeof(buf), "* Robust connection management with auto-reconnection");
    XPLMDrawString(col_white, l + 30, t - lineOffset, buf, NULL, xplmFont_Proportional);
    lineOffset += 30;

    // Copyright
    snprintf(buf, sizeof(buf), "Copyright (c) 2025 Alireza Ghaderi");
    XPLMDrawString(col_white, l + 20, t - lineOffset, buf, NULL, xplmFont_Proportional);
    lineOffset += 20;

    snprintf(buf, sizeof(buf), "MIT License");
    XPLMDrawString(col_white, l + 20, t - lineOffset, buf, NULL, xplmFont_Proportional);
    lineOffset += 20;

    snprintf(buf, sizeof(buf), "https://github.com/alireza787b/px4xplane");
    XPLMDrawString(col_white, l + 20, t - lineOffset, buf, NULL, xplmFont_Proportional);

    // Instructions
    lineOffset += 40;
    snprintf(buf, sizeof(buf), "Click anywhere to close this dialog");
    XPLMDrawString(col_white, l + 20, t - lineOffset, buf, NULL, xplmFont_Proportional);
}

// ============================================================================
// MOUSE HANDLERS
// ============================================================================

int handle_main_mouse(XPLMWindowID in_window_id, int x, int y, int is_down, void* in_refcon) {
    // Basic mouse handling for main window if needed
    return 1;
}

int handle_about_mouse(XPLMWindowID in_window_id, int x, int y, int is_down, void* in_refcon) {
    if (is_down) {
        debugLog("About dialog clicked - closing");
        hide_about_window();
    }
    return 1;
}

// ============================================================================
// MENU SYSTEM (Following X-Plane Demo Pattern)
// ============================================================================

void menu_handler(void* in_menu_ref, void* in_item_ref) {
    debugLog("Main menu handler called");

    if (in_item_ref == NULL) {
        debugLog("ERROR: Menu item reference is NULL");
        return;
    }

    const char* item_name = (const char*)in_item_ref;
    debugLog(("Menu item selected: " + std::string(item_name)).c_str());

    if (strcmp(item_name, MENU_ITEM_DATA_INSPECTOR) == 0) {
        debugLog("Data Inspector menu item selected - showing main window");
        show_main_window();
    }
    else if (strcmp(item_name, MENU_ITEM_ABOUT) == 0) {
        debugLog("About menu item selected - showing about window");
        show_about_window();
    }
    else if (strcmp(item_name, MENU_ITEM_TOGGLE_CONNECTION) == 0) {
        debugLog("Toggle connection menu item selected");
        toggleEnable();
    }
}

void airframes_menu_handler(void* in_menu_ref, void* in_item_ref) {
    debugLog("Airframes menu handler called");

    if (in_item_ref == NULL) {
        debugLog("ERROR: Airframe menu item reference is NULL");
        return;
    }

    const char* airframe_name = (const char*)in_item_ref;
    debugLog(("Airframe selected: " + std::string(airframe_name)).c_str());

    ConfigManager::setActiveAirframeName(std::string(airframe_name));
    refresh_airframes_menu();
}

void create_airframes_menu() {
    if (g_airframes_menu != NULL) {
        XPLMDestroyMenu(g_airframes_menu);
    }

    int airframes_idx = XPLMAppendMenuItem(g_menu_id, MENU_ITEM_AIRFRAMES, NULL, 1);
    g_airframes_menu = XPLMCreateMenu("Airframes", g_menu_id, airframes_idx, airframes_menu_handler, NULL);

    if (g_airframes_menu == NULL) {
        debugLog("ERROR: Failed to create airframes submenu!");
        return;
    }

    refresh_airframes_menu();
}

void refresh_airframes_menu() {
    if (g_airframes_menu == NULL) {
        debugLog("WARNING: airframes_menu is NULL in refresh_airframes_menu");
        return;
    }

    // Clear existing items
    XPLMClearAllMenuItems(g_airframes_menu);

    std::vector<std::string> airframeNames = ConfigManager::getAirframeLists();
    std::string activeAirframe = ConfigManager::getActiveAirframeName();

    debugLog(("Refreshing airframes menu with " + std::to_string(airframeNames.size()) + " items").c_str());

    for (const std::string& name : airframeNames) {
        std::string menuItemName = name + (name == activeAirframe ? " *" : "");
        // Use the airframe name itself as the reference (like the demo)
        char* itemRef = new char[name.length() + 1];
        strcpy(itemRef, name.c_str());
        XPLMAppendMenuItem(g_airframes_menu, menuItemName.c_str(), itemRef, 1);
    }

    debugLog("Airframes menu refreshed successfully");
}

int toggleEnableHandler(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon) {
    if (inPhase == xplm_CommandBegin) {
        debugLog("Toggle enable command executed");
        toggleEnable();
    }
    return 0;
}

void toggleEnable() {
    debugLog("toggleEnable() called");
    if (ConnectionManager::isConnected()) {
        debugLog("Currently connected, attempting to disconnect");
        ConnectionManager::disconnect();
    }
    else {
        debugLog("Currently disconnected, attempting to set up server socket");
        ConnectionManager::setupServerSocket();
    }
}

void create_menu() {
    debugLog("Creating plugin menu...");

    // Create main menu container (following demo pattern)
    g_menu_container_idx = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "PX4 X-Plane", NULL, 1);
    g_menu_id = XPLMCreateMenu("PX4 X-Plane", XPLMFindPluginsMenu(), g_menu_container_idx, menu_handler, NULL);

    if (g_menu_id == NULL) {
        debugLog("ERROR: Failed to create main menu!");
        return;
    }

    // Add menu items using string references (like the demo)
    create_airframes_menu();
    XPLMAppendMenuItem(g_menu_id, MENU_ITEM_DATA_INSPECTOR, (void*)MENU_ITEM_DATA_INSPECTOR, 1);
    XPLMAppendMenuItem(g_menu_id, MENU_ITEM_ABOUT, (void*)MENU_ITEM_ABOUT, 1);
    XPLMAppendMenuSeparator(g_menu_id);

    // Create toggle command
    toggleEnableCmd = XPLMCreateCommand("px4xplane/toggleEnable", "Toggle PX4-XPlane connection");
    if (toggleEnableCmd != NULL) {
        XPLMRegisterCommandHandler(toggleEnableCmd, toggleEnableHandler, 1, (void*)0);
    }

    XPLMAppendMenuItemWithCommand(g_menu_id, "Connect/Disconnect SITL", toggleEnableCmd);

    debugLog("Menu created successfully");
}

// ============================================================================
// FLIGHT LOOP
// ============================================================================

float MyFlightLoopCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void* inRefcon) {
    if (!ConnectionManager::isConnected()) return 0.005f;

    float currentFlightTime = XPLMGetDataf(XPLMFindDataRef("sim/time/total_flight_time_sec"));

    if (currentFlightTime < lastFlightTime) {
        if (ConnectionManager::isConnected()) {
            toggleEnable();
        }
    }

    timeSinceLastSensorUpdate += inElapsedSinceLastCall;
    timeSinceLastGpsUpdate += inElapsedSinceLastCall;
    timeSinceLastStateQuaternionUpdate += inElapsedSinceLastCall;
    timeSinceLastRcUpdate += inElapsedSinceLastCall;

    if (timeSinceLastSensorUpdate >= BASE_SENSOR_UPDATE_PERIOD) {
        MAVLinkManager::sendHILSensor(uint8_t(0));
        timeSinceLastSensorUpdate = 0.0f;
    }

    if (timeSinceLastGpsUpdate >= BASE_GPS_UPDATE_PERIOD) {
        MAVLinkManager::sendHILGPS();
        timeSinceLastGpsUpdate = 0.0f;
    }

    if (timeSinceLastStateQuaternionUpdate >= BASE_STATE_QUAT_UPDATE_PERIOD) {
        MAVLinkManager::sendHILStateQuaternion();
        timeSinceLastStateQuaternionUpdate = 0.0f;
    }

    if (timeSinceLastRcUpdate >= BASE_RC_UPDATE_PERIOD) {
        MAVLinkManager::sendHILRCInputs();
        timeSinceLastRcUpdate = 0.0f;
    }

    ConnectionManager::receiveData();
    DataRefManager::overrideActuators();
    DataRefManager::SIM_Timestep = inElapsedSinceLastCall;
    lastFlightTime = currentFlightTime;

    return 0.005f;
}

// ============================================================================
// X-PLANE PLUGIN API
// ============================================================================

PLUGIN_API int XPluginStart(char* outName, char* outSig, char* outDesc) {
    strcpy(outName, "PX4 to X-Plane");
    strcpy(outSig, "alireza787.px4xplane");
    strcpy(outDesc, "PX4 SITL Interface for X-Plane with Fixed Window Management.");

    debugLog("Plugin starting...");

    // Enable modern window features for X-Plane 12 compatibility
    if (XPLMHasFeature("XPLM_USE_NATIVE_WIDGET_WINDOWS")) {
        XPLMEnableFeature("XPLM_USE_NATIVE_WIDGET_WINDOWS", 1);
        debugLog("Enabled modern native widget windows");
    }

    // Create menu first (windows will be created on demand)
    create_menu();

#if IBM
    if (!ConnectionManager::initializeWinSock()) {
        debugLog("ERROR: initializeWinSock failed");
        return 0;
    }
#endif

    XPLMRegisterFlightLoopCallback(MyFlightLoopCallback, -1.0f, NULL);
    debugLog("Plugin started successfully");

    return 1;
}

PLUGIN_API void XPluginStop(void) {
    debugLog("Plugin stopping...");

    if (ConnectionManager::isConnected()) {
        toggleEnable();
    }

    XPLMUnregisterFlightLoopCallback(MyFlightLoopCallback, NULL);

    // Clean up menu
    if (g_airframes_menu != NULL) {
        XPLMDestroyMenu(g_airframes_menu);
        g_airframes_menu = NULL;
    }

    if (g_menu_id != NULL) {
        XPLMDestroyMenu(g_menu_id);
        g_menu_id = NULL;
    }

    destroy_windows();

#if IBM
    ConnectionManager::cleanupWinSock();
    debugLog("WinSock cleaned up");
#endif

    debugLog("Plugin stopped");
}

PLUGIN_API int XPluginEnable(void) {
    debugLog("Plugin enabled");
    return 1;
}

PLUGIN_API void XPluginDisable(void) {
    debugLog("Plugin disabled");
    if (ConnectionManager::isConnected()) {
        ConnectionManager::disconnect();
    }
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMessage, void* inParam) {
    // Handle X-Plane system messages
    switch (inMessage) {
    case XPLM_MSG_PLANE_CRASHED:
        debugLog("Aircraft crashed - maintaining connection");
        break;
    case XPLM_MSG_PLANE_LOADED:
        debugLog("New aircraft loaded");
        break;
    default:
        break;
    }
}