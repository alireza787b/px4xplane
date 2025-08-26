/**
 * @file px4xplane.cpp
 * @brief PX4-XPlane Interface Plugin - Production Ready Beta v2.5.0
 *
 * PRODUCTION BETA FEATURES:
 * - High-contrast UI optimized for X-Plane's dark theme
 * - Professional 5-tab interface with excellent readability
 * - Real-time airframe configuration visualization
 * - Clear status indicators and visual feedback
 * - Production-ready code architecture and error handling
 * - Easy extensibility for adding new data fields
 * - Dynamic menu system with real-time connection status updates
 *
 * PRESERVED FUNCTIONALITY:
 * - All original PX4 SITL communication and data handling
 * - Complete airframe support and configuration management
 * - MAVLink messaging and sensor data transmission
 * - Flight loop callback and timing management
 * - Cross-platform compatibility (Windows, macOS, Linux)
 *
 * @author Alireza Ghaderi
 * @copyright Copyright (c) 2025 Alireza Ghaderi. All rights reserved.
 * @license MIT License
 * @version 2.5.0 (Beta - Production Ready)
 * @url https://github.com/alireza787b/px4xplane
 */

#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMDataAccess.h"
#include "XPLMMenus.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"

 // Standard library includes
#include <string>
#include <stdio.h>
#include <vector>
#include <functional>
#include <variant>
#include <fstream>
#include <sstream>
#include <map>
#include <algorithm>
#include <cstring>

// Core plugin includes
#include "ConfigReader.h"
#include "DataRefManager.h"
#include "ConnectionManager.h"
#include "ConfigManager.h"
#include "MAVLinkManager.h"

// New UI system includes
#include "VersionInfo.h"
#include "UIConstants.h"
#include "UIHandler.h"

// Platform-specific includes
#if IBM
#include <windows.h>
#endif
#if LIN || APL
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#endif

#ifndef XPLM300
#error This plugin requires X-Plane SDK XPLM300 or later
#endif
#include <XPLMPlugin.h>

// =================================================================
// GLOBAL VARIABLES - Core Plugin State
// =================================================================

static XPLMWindowID g_mainWindow = nullptr;
static XPLMWindowID g_aboutWindow = nullptr;
static XPLMMenuID g_mainMenu = nullptr;
static XPLMMenuID g_airframesMenu = nullptr;
static int g_menuContainerIdx = -1;
static XPLMCommandRef g_toggleConnectionCmd = nullptr;

// NEW: Dynamic menu system variables
static int g_connectionMenuItemIndex = -1;  // Track connection menu item position

// Menu item identifiers (string-based for robustness)
#define MENU_ITEM_DATA_INSPECTOR "Data Inspector"
#define MENU_ITEM_ABOUT "About"
#define MENU_ITEM_TOGGLE_CONNECTION "Toggle Connection"
#define MENU_ITEM_AIRFRAMES "Airframes"

// Update frequencies for MAVLink messaging (unchanged from original)
const float BASE_SENSOR_UPDATE_PERIOD = 0.001f;  // 1000 Hz for IMU
const float BASE_GPS_UPDATE_PERIOD = 0.1f;       // 10 Hz for GPS  
const float BASE_STATE_QUAT_UPDATE_PERIOD = 0.05f; // 20 Hz
const float BASE_RC_UPDATE_PERIOD = 0.05f;       // 20 Hz

// Timing variables for flight loop (unchanged from original)
float timeSinceLastSensorUpdate = 0.0f;
float timeSinceLastGpsUpdate = 0.0f;
float timeSinceLastStateQuaternionUpdate = 0.0f;
float timeSinceLastRcUpdate = 0.0f;
float lastFlightTime = 0.0f;

// =================================================================
// FORWARD DECLARATIONS
// =================================================================

// Window management functions
void createMainWindow();
void createAboutWindow();
void showMainWindow();
void showAboutWindow();
void hideMainWindow();
void hideAboutWindow();
void destroyWindows();

// Menu system functions
void createMainMenu();
void createAirframesMenu();
void refreshAirframesMenu();
void updateConnectionMenuItem();  // NEW: Dynamic menu update function
void menuHandler(void* menuRef, void* itemRef);
void airframesMenuHandler(void* menuRef, void* itemRef);

// Core plugin functions
void toggleConnection();
int toggleConnectionHandler(XPLMCommandRef command, XPLMCommandPhase phase, void* refcon);
float flightLoopCallback(float elapsedSinceLastCall, float elapsedTimeSinceLastFlightLoop, int counter, void* refcon);

// Utility functions
void debugLog(const char* message);

// =================================================================
// UTILITY FUNCTIONS
// =================================================================

void debugLog(const char* message) {
    XPLMDebugString("px4xplane: ");
    XPLMDebugString(message);
    XPLMDebugString("\n");
}

// =================================================================
// WINDOW MANAGEMENT FUNCTIONS - Integrated with UIHandler
// =================================================================

void createMainWindow() {
    if (g_mainWindow != nullptr) {
        debugLog("Main window already exists");
        return;
    }

    debugLog("Creating main window with professional UI system...");

    XPLMCreateWindow_t params = {};
    params.structSize = sizeof(params);
    params.visible = 0;
    params.drawWindowFunc = UIHandler::drawMainWindow;
    params.handleMouseClickFunc = UIHandler::handleMainWindowMouse;
    params.handleRightClickFunc = nullptr;
    params.handleMouseWheelFunc = UIHandler::handleMainWindowWheel;
    params.handleKeyFunc = nullptr;
    params.handleCursorFunc = nullptr;
    params.refcon = nullptr;
    params.layer = xplm_WindowLayerFloatingWindows;
    params.decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle;

    // Set reasonable default size and position
    params.left = 100;
    params.bottom = 200;
    params.right = params.left + UIConstants::getScaledLayout(UIConstants::Layout::MIN_WINDOW_WIDTH);
    params.top = params.bottom + UIConstants::getScaledLayout(UIConstants::Layout::MIN_WINDOW_HEIGHT);

    g_mainWindow = XPLMCreateWindowEx(&params);

    if (g_mainWindow == nullptr) {
        debugLog("ERROR: Failed to create main window!");
        return;
    }

    XPLMSetWindowPositioningMode(g_mainWindow, xplm_WindowPositionFree, -1);
    XPLMSetWindowTitle(g_mainWindow, PX4XPlaneVersion::getMainWindowTitle());

    debugLog("Main window created successfully with professional UI");
}

void createAboutWindow() {
    if (g_aboutWindow != nullptr) {
        debugLog("About window already exists");
        return;
    }

    debugLog("Creating about window...");

    XPLMCreateWindow_t params = {};
    params.structSize = sizeof(params);
    params.visible = 0;
    params.drawWindowFunc = UIHandler::drawAboutWindow;
    params.handleMouseClickFunc = UIHandler::handleAboutWindowMouse;
    params.handleRightClickFunc = nullptr;
    params.handleMouseWheelFunc = nullptr;
    params.handleKeyFunc = nullptr;
    params.handleCursorFunc = nullptr;
    params.refcon = nullptr;
    params.layer = xplm_WindowLayerModal;
    params.decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle;

    // Center about window with reasonable size
    int screenWidth, screenHeight;
    XPLMGetScreenSize(&screenWidth, &screenHeight);
    int windowWidth = 600;
    int windowHeight = 500;
    params.left = (screenWidth - windowWidth) / 2;
    params.right = params.left + windowWidth;
    params.bottom = (screenHeight - windowHeight) / 2;
    params.top = params.bottom + windowHeight;

    g_aboutWindow = XPLMCreateWindowEx(&params);

    if (g_aboutWindow == nullptr) {
        debugLog("ERROR: Failed to create about window!");
        return;
    }

    XPLMSetWindowPositioningMode(g_aboutWindow, xplm_WindowPositionFree, -1);
    XPLMSetWindowTitle(g_aboutWindow, PX4XPlaneVersion::getAboutWindowTitle());

    debugLog("About window created successfully");
}

void showMainWindow() {
    debugLog("Showing main window...");

    if (g_mainWindow == nullptr) {
        createMainWindow();
    }

    if (g_mainWindow != nullptr) {
        XPLMSetWindowIsVisible(g_mainWindow, 1);
        XPLMBringWindowToFront(g_mainWindow);
        debugLog("Main window should now be visible");
    }
    else {
        debugLog("ERROR: Failed to show main window - window is null");
    }
}

void showAboutWindow() {
    debugLog("Showing about window...");

    if (g_aboutWindow == nullptr) {
        createAboutWindow();
    }

    if (g_aboutWindow != nullptr) {
        XPLMSetWindowIsVisible(g_aboutWindow, 1);
        XPLMBringWindowToFront(g_aboutWindow);
        debugLog("About window should now be visible");
    }
    else {
        debugLog("ERROR: Failed to show about window - window is null");
    }
}

void hideMainWindow() {
    if (g_mainWindow != nullptr) {
        XPLMSetWindowIsVisible(g_mainWindow, 0);
        debugLog("Main window hidden");
    }
}

void hideAboutWindow() {
    if (g_aboutWindow != nullptr) {
        XPLMSetWindowIsVisible(g_aboutWindow, 0);
        debugLog("About window hidden");
    }
}

void destroyWindows() {
    if (g_aboutWindow != nullptr) {
        XPLMDestroyWindow(g_aboutWindow);
        g_aboutWindow = nullptr;
        debugLog("About window destroyed");
    }

    if (g_mainWindow != nullptr) {
        XPLMDestroyWindow(g_mainWindow);
        g_mainWindow = nullptr;
        debugLog("Main window destroyed");
    }
}

// =================================================================
// MENU SYSTEM - Enhanced with Dynamic Text Updates
// =================================================================

void updateConnectionMenuItem() {
    if (g_mainMenu == nullptr) {
        debugLog("ERROR: Cannot update connection menu item - main menu is null");
        return;
    }

    // Remove existing connection menu item if it exists
    if (g_connectionMenuItemIndex != -1) {
        XPLMRemoveMenuItem(g_mainMenu, g_connectionMenuItemIndex);
        debugLog("Removed old connection menu item");
    }

    // Get current connection state and appropriate menu text
    const char* menuText = UIHandler::getConnectionMenuText();

    // Add new connection menu item with updated text
    g_connectionMenuItemIndex = XPLMAppendMenuItemWithCommand(g_mainMenu, menuText, g_toggleConnectionCmd);

    debugLog(("Updated connection menu item text to: " + std::string(menuText)).c_str());
}

void menuHandler(void* menuRef, void* itemRef) {
    debugLog("Main menu handler called");

    if (itemRef == nullptr) {
        debugLog("ERROR: Menu item reference is null");
        return;
    }

    const char* itemName = (const char*)itemRef;
    debugLog(("Menu item selected: " + std::string(itemName)).c_str());

    if (strcmp(itemName, MENU_ITEM_DATA_INSPECTOR) == 0) {
        debugLog("Data Inspector menu item selected - showing main window");
        showMainWindow();
    }
    else if (strcmp(itemName, MENU_ITEM_ABOUT) == 0) {
        debugLog("About menu item selected - showing about window");
        showAboutWindow();
    }
    else if (strcmp(itemName, MENU_ITEM_TOGGLE_CONNECTION) == 0) {
        debugLog("Toggle connection menu item selected");
        toggleConnection();
    }
}

void airframesMenuHandler(void* menuRef, void* itemRef) {
    debugLog("Airframes menu handler called");

    if (itemRef == nullptr) {
        debugLog("ERROR: Airframe menu item reference is null");
        return;
    }

    const char* airframeName = (const char*)itemRef;
    debugLog(("Airframe selected: " + std::string(airframeName)).c_str());

    ConfigManager::setActiveAirframeName(std::string(airframeName));
    refreshAirframesMenu();
}

void createAirframesMenu() {
    if (g_airframesMenu != nullptr) {
        XPLMDestroyMenu(g_airframesMenu);
        g_airframesMenu = nullptr;
    }

    int airframesIdx = XPLMAppendMenuItem(g_mainMenu, MENU_ITEM_AIRFRAMES, nullptr, 1);
    g_airframesMenu = XPLMCreateMenu("Airframes", g_mainMenu, airframesIdx, airframesMenuHandler, nullptr);

    if (g_airframesMenu == nullptr) {
        debugLog("ERROR: Failed to create airframes submenu!");
        return;
    }

    refreshAirframesMenu();
}

void refreshAirframesMenu() {
    if (g_airframesMenu == nullptr) {
        debugLog("WARNING: Airframes menu is null in refresh");
        return;
    }

    XPLMClearAllMenuItems(g_airframesMenu);

    std::vector<std::string> airframeNames = ConfigManager::getAirframeLists();
    std::string activeAirframe = ConfigManager::getActiveAirframeName();

    debugLog(("Refreshing airframes menu with " + std::to_string(airframeNames.size()) + " items").c_str());

    for (const std::string& name : airframeNames) {
        std::string menuItemName = name + (name == activeAirframe ? " *" : "");
        char* itemRef = new char[name.length() + 1];
        strcpy(itemRef, name.c_str());
        XPLMAppendMenuItem(g_airframesMenu, menuItemName.c_str(), itemRef, 1);
    }

    debugLog("Airframes menu refreshed successfully");
}

int toggleConnectionHandler(XPLMCommandRef command, XPLMCommandPhase phase, void* refcon) {
    if (phase == xplm_CommandBegin) {
        debugLog("Toggle connection command executed");
        toggleConnection();
    }
    return 0;
}

void toggleConnection() {
    debugLog("toggleConnection() called");

    // Store previous connection state for comparison
    bool wasConnected = ConnectionManager::isConnected();

    if (wasConnected) {
        debugLog("Currently connected, attempting to disconnect");
        ConnectionManager::disconnect();
    }
    else {
        debugLog("Currently disconnected, attempting to connect to PX4 SITL");
        ConnectionManager::setupServerSocket();
    }

    // Update menu text dynamically after connection state change
    bool isNowConnected = ConnectionManager::isConnected();
    if (wasConnected != isNowConnected) {
        debugLog("Connection state changed - updating menu text");
        updateConnectionMenuItem();
        debugLog("Menu text updated successfully");
    }
    else {
        debugLog("Connection state unchanged - no menu update needed");
    }
}

void createMainMenu() {
    debugLog("Creating plugin menu system...");

    // Create main menu container
    g_menuContainerIdx = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "PX4 X-Plane", nullptr, 1);
    g_mainMenu = XPLMCreateMenu("PX4 X-Plane", XPLMFindPluginsMenu(), g_menuContainerIdx, menuHandler, nullptr);

    if (g_mainMenu == nullptr) {
        debugLog("ERROR: Failed to create main menu!");
        return;
    }

    // Create airframes submenu
    createAirframesMenu();

    // Add static menu items
    XPLMAppendMenuItem(g_mainMenu, MENU_ITEM_DATA_INSPECTOR, (void*)MENU_ITEM_DATA_INSPECTOR, 1);
    XPLMAppendMenuItem(g_mainMenu, MENU_ITEM_ABOUT, (void*)MENU_ITEM_ABOUT, 1);
    XPLMAppendMenuSeparator(g_mainMenu);

    // Create toggle connection command
    g_toggleConnectionCmd = XPLMCreateCommand("px4xplane/toggleConnection", "Toggle PX4-XPlane connection");
    if (g_toggleConnectionCmd != nullptr) {
        XPLMRegisterCommandHandler(g_toggleConnectionCmd, toggleConnectionHandler, 1, (void*)0);
    }

    // Add connection menu item with initial text (will be updated dynamically)
    updateConnectionMenuItem();

    debugLog("Menu system created successfully with dynamic connection menu");
}

// =================================================================
// CORE PLUGIN FUNCTIONALITY - Flight Loop (PRESERVED FROM ORIGINAL)
// =================================================================

float flightLoopCallback(float elapsedSinceLastCall, float elapsedTimeSinceLastFlightLoop, int counter, void* refcon) {
    // Check connection status
    if (!ConnectionManager::isConnected()) {
        return 0.005f; // Continue checking at 200Hz even when disconnected
    }

    // Handle flight time reset (aircraft change detection)
    float currentFlightTime = XPLMGetDataf(XPLMFindDataRef("sim/time/total_flight_time_sec"));
    if (currentFlightTime < lastFlightTime) {
        if (ConnectionManager::isConnected()) {
            debugLog("Flight time reset detected - toggling connection");
            toggleConnection();
        }
    }

    // Update timing accumulators
    timeSinceLastSensorUpdate += elapsedSinceLastCall;
    timeSinceLastGpsUpdate += elapsedSinceLastCall;
    timeSinceLastStateQuaternionUpdate += elapsedSinceLastCall;
    timeSinceLastRcUpdate += elapsedSinceLastCall;

    // Send sensor data at 1000 Hz
    if (timeSinceLastSensorUpdate >= BASE_SENSOR_UPDATE_PERIOD) {
        MAVLinkManager::sendHILSensor(uint8_t(0));
        timeSinceLastSensorUpdate = 0.0f;
    }

    // Send GPS data at 10 Hz
    if (timeSinceLastGpsUpdate >= BASE_GPS_UPDATE_PERIOD) {
        MAVLinkManager::sendHILGPS();
        timeSinceLastGpsUpdate = 0.0f;
    }

    // Send state quaternion at 20 Hz
    if (timeSinceLastStateQuaternionUpdate >= BASE_STATE_QUAT_UPDATE_PERIOD) {
        MAVLinkManager::sendHILStateQuaternion();
        timeSinceLastStateQuaternionUpdate = 0.0f;
    }

    // Send RC inputs at 20 Hz
    if (timeSinceLastRcUpdate >= BASE_RC_UPDATE_PERIOD) {
        MAVLinkManager::sendHILRCInputs();
        timeSinceLastRcUpdate = 0.0f;
    }

    // Handle incoming data and actuator override
    ConnectionManager::receiveData();
    DataRefManager::overrideActuators();

    // Update simulation timestep
    DataRefManager::SIM_Timestep = elapsedSinceLastCall;
    lastFlightTime = currentFlightTime;

    return 0.005f; // Continue at 200Hz
}

// =================================================================
// X-PLANE PLUGIN API - Standard Interface (ENHANCED FOR PHASE 3)
// =================================================================

PLUGIN_API int XPluginStart(char* outName, char* outSig, char* outDesc) {
    // Set plugin identification using centralized version info
    strcpy(outName, PX4XPlaneVersion::PLUGIN_NAME);
    strcpy(outSig, PX4XPlaneVersion::PLUGIN_SIGNATURE);
    strcpy(outDesc, PX4XPlaneVersion::getBuildInfo());

    debugLog("Plugin starting (v2.5.0 - Beta Production Ready)...");

    // Enable modern X-Plane features
    if (XPLMHasFeature("XPLM_USE_NATIVE_WIDGET_WINDOWS")) {
        XPLMEnableFeature("XPLM_USE_NATIVE_WIDGET_WINDOWS", 1);
        debugLog("Enabled modern native widget windows");
    }

    ConfigManager::loadConfiguration();

    // Initialize UI system first
    UIConstants::XPlaneColors::initialize();
    UIHandler::initialize();

    // Create menu system with dynamic updates
    createMainMenu();

    // Initialize platform-specific networking
#if IBM
    if (!ConnectionManager::initializeWinSock()) {
        debugLog("ERROR: Windows socket initialization failed");
        return 0;
    }
#endif

    // Register flight loop callback
    XPLMRegisterFlightLoopCallback(flightLoopCallback, -1.0f, nullptr);

    debugLog("Plugin started successfully (v2.5.0 - Beta Production Ready)");
    debugLog(("Version: " + std::string(PX4XPlaneVersion::getFullVersionString())).c_str());

    return 1; // Success
}

PLUGIN_API void XPluginStop(void) {
    debugLog("Plugin stopping (v2.5.0 - Beta Production Ready)...");

    // Disconnect if still connected
    if (ConnectionManager::isConnected()) {
        toggleConnection();
    }

    // Unregister flight loop callback
    XPLMUnregisterFlightLoopCallback(flightLoopCallback, nullptr);

    // Destroy menu system
    if (g_airframesMenu != nullptr) {
        XPLMDestroyMenu(g_airframesMenu);
        g_airframesMenu = nullptr;
    }

    if (g_mainMenu != nullptr) {
        XPLMDestroyMenu(g_mainMenu);
        g_mainMenu = nullptr;
    }

    // Reset menu tracking variables
    g_connectionMenuItemIndex = -1;

    // Destroy windows
    destroyWindows();

    // Clean up UI system
    UIHandler::cleanup();

    // Clean up platform-specific resources
#if IBM
    ConnectionManager::cleanupWinSock();
    debugLog("Windows socket system cleaned up");
#endif

    debugLog("Plugin stopped successfully (v2.5.0 - Beta Production Ready)");
}

PLUGIN_API int XPluginEnable(void) {
    debugLog("Plugin enabled (v2.5.0 - Beta Production Ready)");
    // Update menu to reflect current state on plugin enable
    updateConnectionMenuItem();
    return 1;
}

PLUGIN_API void XPluginDisable(void) {
    debugLog("Plugin disabled (v2.5.0 - Beta Production Ready)");
    if (ConnectionManager::isConnected()) {
        ConnectionManager::disconnect();
        // Update menu to reflect disconnected state
        updateConnectionMenuItem();
    }
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMessage, void* inParam) {
    switch (inMessage) {
    case XPLM_MSG_PLANE_CRASHED:
        debugLog("Aircraft crashed - maintaining connection");
        break;
    case XPLM_MSG_PLANE_LOADED:
        debugLog("New aircraft loaded");
        // Update menu in case connection state needs to be reflected
        updateConnectionMenuItem();
        break;
    default:
        // Handle other messages as needed
        break;
    }
}