#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMDataAccess.h"
#include "XPLMMenus.h"
#include "XPLMProcessing.h"
#include <string>
#include <stdio.h>
#include <vector>
#include <functional> // Include this
#include <variant>
#include <fstream>
#include <sstream>
#include <map>
#include "configReader.h"
#include "DataRefManager.h"
#include "ConnectionManager.h"
#include "ConfigManager.h"
#include <algorithm>



#if IBM
#include <windows.h>
#endif
#if LIN || APL
#include <unistd.h>  // For TCP/IP
#include <sys/socket.h> // For TCP/IP
#include <netinet/in.h> // For TCP/IP
#endif
#include "MAVLinkManager.h"
#include "ConnectionStatusHUD.h"
#include "VersionInfo.h"
#include "UIConstants.h"
#include "UIHandler.h"
// SensorPublisher.h removed - feature disabled in v3.3.1

#ifndef XPLM300
#error This is made to be compiled against the XPLM300 SDK
#endif

static XPLMWindowID g_window;
static XPLMMenuID g_menu_id;
static XPLMMenuID airframesMenu; // Global variable for the airframes submenu
static int g_airframesMenuItemIndex; // Global variable to store the index of the airframes submenu

// Menu reference constants for menu_handler callback
// Using distinct non-zero values to avoid confusion with NULL
#define MENU_REF_MAIN ((void*)100)
#define MENU_REF_AIRFRAMES ((void*)200)


// Global variable to hold our command reference
static XPLMCommandRef toggleEnableCmd;

static int g_enabled = 0; // Used to toggle connection state

// Phase 3: FPS-independent sensor publisher (DISABLED in v3.3.1)
// Was causing "High Accelerometer Bias" regression - reverted to direct sendHILSensor()
// static std::unique_ptr<SensorPublisher> g_sensorPublisher;





// OLD UI FUNCTIONS - No longer used, replaced by UIHandler
// void draw_px4xplane(XPLMWindowID in_window_id, void* in_refcon);

void menu_handler(void* in_menu_ref, void* in_item_ref);
int toggleEnableHandler(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon);
void create_menu();
void toggleEnable();
void updateMenuItems();
void initializeMessagePeriods();  // Initialize MAVLink message periods from config
float MyFlightLoopCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void* inRefcon);


// Debugging function - Logs a message to the X-Plane log
void debugLog(const char* message) {
	XPLMDebugString("px4xplane: ");
	XPLMDebugString(message);
	XPLMDebugString("\n");
}



// ============================================================================
// OLD UI DRAWING FUNCTIONS - DEPRECATED - Replaced by UIHandler system
// Kept for reference only - these are no longer called
// ============================================================================

/*
int drawHeader(int l, int t, float col_white[]) {
	char header[512];
	snprintf(header, sizeof(header), "PX4-XPlane Interface %s", PX4XPlaneVersion::getFullVersionString());
	XPLMDrawString(col_white, l + 10, t - 20, header, NULL, xplmFont_Proportional);
	return l + 20;
}

int drawStatusAndConfig(int l, int t, float col_white[], int& lineOffset, int columnWidth) {
	char buf[512];
	int droneConfigOffset = 20;

	std::string droneConfigStr = DataRefManager::GetFormattedDroneConfig();
	std::istringstream iss(droneConfigStr);
	std::string line;
	while (std::getline(iss, line)) {
		char lineBuffer[512];
		strncpy(lineBuffer, line.c_str(), sizeof(lineBuffer));
		lineBuffer[sizeof(lineBuffer) - 1] = '\0';
		XPLMDrawString(col_white, l + droneConfigOffset, t - lineOffset, lineBuffer, NULL, xplmFont_Proportional);
		lineOffset += 20;
	}

	const std::string& status = ConnectionManager::getStatus();
	bool isConnected = ConnectionManager::isConnected();

	float statusColor[3];
	if (isConnected) {
		memcpy(statusColor, UIConstants::Colors::CONNECTED, sizeof(statusColor));
		snprintf(buf, sizeof(buf), "%s Status: %s", UIConstants::Status::CONNECTED_ICON, UIConstants::Status::CONNECTED_TEXT);
	} else {
		memcpy(statusColor, UIConstants::Colors::DISCONNECTED, sizeof(statusColor));
		snprintf(buf, sizeof(buf), "%s Status: %s", UIConstants::Status::DISCONNECTED_ICON, UIConstants::Status::DISCONNECTED_TEXT);
	}
	XPLMDrawString(statusColor, l + 10, t - lineOffset, buf, NULL, xplmFont_Proportional);
	lineOffset += 20;

	snprintf(buf, sizeof(buf), "Details: %s", status.c_str());
	XPLMDrawString(col_white, l + 10, t - lineOffset, buf, NULL, xplmFont_Proportional);
	lineOffset += 20;

	snprintf(buf, sizeof(buf), "SITL Time Step: %.3f", DataRefManager::SIM_Timestep);
	XPLMDrawString(col_white, l + 10, t - lineOffset, buf, NULL, xplmFont_Proportional);
	lineOffset += 20;

	return lineOffset;
}

void drawFooter(int l, int b, float col_white[]) {
	char footer[512];
	snprintf(footer, sizeof(footer), "Copyright (c) Alireza Ghaderi - https://github.com/alireza787b/px4xplane");
	XPLMDrawString(col_white, l + 10, b + 10, footer, NULL, xplmFont_Proportional);
}
*/


PLUGIN_API void XPluginStop(void);


PLUGIN_API int XPluginStart(
	char* outName,
	char* outSig,
	char* outDesc)
{
	// Use centralized version information
	strcpy(outName, PX4XPlaneVersion::PLUGIN_NAME);
	strcpy(outSig, PX4XPlaneVersion::PLUGIN_SIGNATURE);
	strcpy(outDesc, PX4XPlaneVersion::getBuildInfo());

	// Initialize professional UI system
	UIConstants::XPlaneColors::initialize();
	UIHandler::initialize();

	debugLog("Plugin starting with enhanced UI system...");
	debugLog(("Version: " + std::string(PX4XPlaneVersion::getFullVersionString())).c_str());

	create_menu();

	XPLMCreateWindow_t params;
	params.structSize = sizeof(params);
	params.visible = 0; // Window is initially invisible
	params.drawWindowFunc = UIHandler::drawMainWindow;
	// Register professional UI handlers
	params.handleMouseClickFunc = UIHandler::handleMainWindowMouse;
	params.handleRightClickFunc = NULL;
	params.handleMouseWheelFunc = UIHandler::handleMainWindowWheel;
	params.handleKeyFunc = NULL;
	params.handleCursorFunc = NULL;
	params.refcon = NULL;
	params.layer = xplm_WindowLayerFloatingWindows;
	params.decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle;

	int left, bottom, right, top;
	XPLMGetScreenBoundsGlobal(&left, &top, &right, &bottom);

	int screenWidth = right - left;
	int screenHeight = top - bottom;
	int windowWidth = (screenWidth * 0.3 > 550) ? static_cast<int>(screenWidth * 0.3) : 550;

	params.left = left + screenWidth * 0.1; // 10% from the left edge of the screen
	params.bottom = bottom + screenHeight * 0.1; // 10% from the bottom edge of the screen
	params.right = params.left + windowWidth;
	params.top = params.bottom + screenHeight * 0.5; // Initial height, adjust as needed



	g_window = XPLMCreateWindowEx(&params);

	XPLMSetWindowPositioningMode(g_window, xplm_WindowPositionFree, -1);
	XPLMSetWindowTitle(g_window, "PX4 to X-Plane");
	XPLMSetWindowResizingLimits(g_window, 600, 200, windowWidth, 1000); // Set minimum width to 550px



#if IBM
	if (!ConnectionManager::initializeWinSock()) {
		XPLMDebugString("initializeWinSock failed \n");
	}
#endif

	// Register the flight loop callback to be called at the next cycle
	XPLMRegisterFlightLoopCallback(MyFlightLoopCallback, -1.0f, NULL);

	// Initialize connection status HUD overlay
	ConnectionStatusHUD::initialize();
	ConnectionStatusHUD::setEnabled(ConfigManager::show_connection_status_hud);

	// Initialize MAVLink message periods from config
	initializeMessagePeriods();

	// Phase 3: FPS-independent sensor publisher (DISABLED in v3.3.1)
	// Was causing "High Accelerometer Bias" regression due to Kalman interpolation
	// creating artificially smooth data that PX4 EKF2 doesn't expect.
	// Reverted to direct sendHILSensor() in flight loop.
	// g_sensorPublisher = std::make_unique<SensorPublisher>(ConfigManager::mavlink_sensor_rate_hz);
	// debugLog("SensorPublisher created for FPS-independent sensor output");
	debugLog("v3.3.1: Using direct sendHILSensor() (SensorPublisher disabled)");

	debugLog("Plugin started successfully");


	return 1;
}


// Define a handler for the command like this
int toggleEnableHandler(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon)
{
	if (inPhase == xplm_CommandBegin)
		toggleEnable(); // Call your function to toggle enable/disable
	return 0;
}

/*
// OLD DRAW FUNCTION - DEPRECATED - Replaced by UIHandler::drawMainWindow
void draw_px4xplane(XPLMWindowID in_window_id, void* in_refcon) {
	// This function is no longer used - UIHandler provides professional tabbed interface
	// Kept for reference only
}
*/

// Function to refresh the airframes submenu to indicate the active airframe.
void refreshAirframesMenu() {
	// Clear the current submenu items.
	XPLMClearAllMenuItems(airframesMenu);

	// Repopulate the submenu with updated airframe names and active status.
	std::vector<std::string> airframeNames = ConfigManager::getAirframeLists();
	std::string activeAirframe = ConfigManager::getActiveAirframeName();

	for (size_t i = 0; i < airframeNames.size(); ++i) {
		const std::string& name = airframeNames[i];
		std::string menuItemName = name;
		if (name == activeAirframe) {
			menuItemName += " *";
		}
		// Pass integer index as item refcon (not a pointer!)
		XPLMAppendMenuItem(airframesMenu, menuItemName.c_str(), (void*)(intptr_t)i, 1);
	}
}


void menu_handler(void* in_menu_ref, void* in_item_ref) {
	char debugBuf[256];
	snprintf(debugBuf, sizeof(debugBuf), "Menu handler called - menu_ref: %p, item_ref: %p", in_menu_ref, in_item_ref);
	debugLog(debugBuf);

	// Check which menu triggered this callback using explicit menu ref comparison
	if (in_menu_ref == MENU_REF_MAIN) {
		// Main menu item clicked
		debugLog("Main menu item clicked");

		// "Show Data" has item_ref (void*)0
		if (in_item_ref == (void*)0) {
			XPLMSetWindowIsVisible(g_window, 1);
			debugLog("Show Data menu item selected - window should now be visible");
		}
		else {
			snprintf(debugBuf, sizeof(debugBuf), "Unknown main menu item: %p", in_item_ref);
			debugLog(debugBuf);
		}
	}
	else if (in_menu_ref == MENU_REF_AIRFRAMES) {
		// Airframe submenu item clicked
		debugLog("Airframe submenu item clicked");

		int index = (int)(intptr_t)in_item_ref;
		std::vector<std::string> airframeNames = ConfigManager::getAirframeLists();

		if (index >= 0 && index < (int)airframeNames.size()) {
			const std::string& selectedAirframe = airframeNames[index];
			debugLog(("Airframe selected: " + selectedAirframe).c_str());

			ConfigManager::setActiveAirframeName(selectedAirframe);
			ConfigManager::loadConfiguration();

			// Reinitialize message periods after config reload
			initializeMessagePeriods();

			refreshAirframesMenu();
			XPLMDebugString(("px4xplane: Configuration reloaded for: " + selectedAirframe + "\n").c_str());
		}
		else {
			snprintf(debugBuf, sizeof(debugBuf), "Invalid airframe index: %d (total: %d)", index, (int)airframeNames.size());
			debugLog(debugBuf);
		}
	}
	else {
		// Unknown menu ref - should never happen
		snprintf(debugBuf, sizeof(debugBuf), "Unknown menu ref: %p", in_menu_ref);
		debugLog(debugBuf);
	}

}



/**
 * @brief Resets all flight loop static state variables.
 *
 * CRITICAL BUG FIX (January 2025): Clear timing/statistics on disconnect
 *
 * PROBLEM: Static variables persist across disconnect/reconnect cycles
 * → Timing variables (lastSensorSendTime, etc.) not reset
 * → Logging statistics (sensorMessageCount, sumRate, etc.) accumulate
 * → Can cause timing issues or corrupted statistics on reconnect
 *
 * SOLUTION: Reset all static state when disconnecting
 * → Next connection starts with clean timing
 * → Statistics start from zero
 * → No accumulated state from previous sessions
 *
 * NOTE: This function is defined here (before MyFlightLoopCallback) so it
 * can access the static timing variables declared later in this file.
 * The actual reset happens in toggleEnable() before disconnect.
 */
void resetFlightLoopTimers();  // Forward declaration

void toggleEnable() {
	XPLMDebugString("px4xplane: toggleEnable() called.\n");
	if (ConnectionManager::isConnected()) {
		XPLMDebugString("px4xplane: Currently connected, attempting to disconnect.\n");

		// Phase 3: SensorPublisher (DISABLED in v3.3.1)
		// if (g_sensorPublisher && g_sensorPublisher->isRunning()) {
		// 	g_sensorPublisher->stop();
		// 	g_sensorPublisher->reset();
		// 	XPLMDebugString("px4xplane: SensorPublisher stopped\n");
		// }

		resetFlightLoopTimers();  // Reset timing state BEFORE disconnect
		ConnectionManager::disconnect();
	}
	else {
		XPLMDebugString("px4xplane: Currently disconnected, attempting to set up server socket.\n");
		ConnectionManager::setupServerSocket();
	}
	updateMenuItems(); // Update menu items after toggling connection
}


int getDataRefInt(const char* dataRefName) {
	XPLMDataRef dataRef = XPLMFindDataRef(dataRefName);
	if (!dataRef) return -1; // or some error value
	return XPLMGetDatai(dataRef);
}

float getDataRefFloat(const char* dataRefName) {
	XPLMDataRef dataRef = XPLMFindDataRef(dataRefName);
	if (!dataRef) return -1.0f; // or some error value
	return XPLMGetDataf(dataRef);
}

double getDataRefDouble(const char* dataRefName) {
	XPLMDataRef dataRef = XPLMFindDataRef(dataRefName);
	if (!dataRef) return -1.0; // or some error value
	return XPLMGetDatad(dataRef);
}

std::vector<float> getDataRefFloatArray(const char* dataRefName) {
	XPLMDataRef dataRef = XPLMFindDataRef(dataRefName);
	std::vector<float> result;

	if (!dataRef) return result; // Return an empty vector if the data ref is not found

	int arraySize = XPLMGetDatavf(dataRef, NULL, 0, 0);
	if (arraySize > 0) {
		result.resize(arraySize);
		XPLMGetDatavf(dataRef, result.data(), 0, arraySize);
	}

	return result;
}



void create_menu() {
	debugLog("Creating plugin menu");

	int menu_container_idx = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "PX4 X-Plane", NULL, 1);
	g_menu_id = XPLMCreateMenu("px4xplane", XPLMFindPluginsMenu(), menu_container_idx, menu_handler, MENU_REF_MAIN);

	// Save the index of the airframes submenu in the main menu
	g_airframesMenuItemIndex = XPLMAppendMenuItem(g_menu_id, "Airframes", NULL, 1);
	// Create airframes submenu with distinct menu ref constant
	airframesMenu = XPLMCreateMenu("Airframes", g_menu_id, g_airframesMenuItemIndex, menu_handler, MENU_REF_AIRFRAMES);


	debugLog("Airframes submenu created");

	std::vector<std::string> airframeNames = ConfigManager::getAirframeLists();
	std::string activeAirframe = ConfigManager::getActiveAirframeName();
	for (size_t i = 0; i < airframeNames.size(); ++i) {
		const std::string& name = airframeNames[i];
		std::string menuItemName = name;
		if (name == activeAirframe) {
			menuItemName += " *";
		}
		XPLMAppendMenuItem(airframesMenu, menuItemName.c_str(), (void*)(intptr_t)i, 1);
	}

	XPLMAppendMenuItem(g_menu_id, "Show Data", (void*)0, 1);
	toggleEnableCmd = XPLMCreateCommand("px4xplane/toggleEnable", "Toggle enable/disable state");
	XPLMRegisterCommandHandler(toggleEnableCmd, toggleEnableHandler, 1, (void*)0);
	XPLMAppendMenuSeparator(g_menu_id);
	XPLMAppendMenuItemWithCommand(g_menu_id, "Connect to SITL", toggleEnableCmd);
	XPLMAppendMenuItemWithCommand(g_menu_id, "Disconnect from SITL", toggleEnableCmd);

	updateMenuItems();

	debugLog("Menu created successfully");
}


void updateMenuItems() {
	XPLMClearAllMenuItems(g_menu_id);

	// Recreate the main menu items
	// Important: Update the airframesMenuItemIndex to reflect the new index after clearing
	g_airframesMenuItemIndex = XPLMAppendMenuItem(g_menu_id, "Airframes", NULL, 1);
	// Recreate airframes submenu with distinct menu ref constant
	airframesMenu = XPLMCreateMenu("Airframes", g_menu_id, g_airframesMenuItemIndex, menu_handler, MENU_REF_AIRFRAMES);

	std::vector<std::string> airframeNames = ConfigManager::getAirframeLists();
	std::string activeAirframe = ConfigManager::getActiveAirframeName();
	for (size_t i = 0; i < airframeNames.size(); ++i) {
		const std::string& name = airframeNames[i];
		std::string menuItemName = name;
		if (name == activeAirframe) {
			menuItemName += " *";
		}
		XPLMAppendMenuItem(airframesMenu, menuItemName.c_str(), (void*)(intptr_t)i, 1);
	}

	// Recreate the remaining main menu items
	XPLMAppendMenuItem(g_menu_id, "Show Data", (void*)0, 1);
	XPLMAppendMenuSeparator(g_menu_id);
	if (ConnectionManager::isConnected()) {
		XPLMAppendMenuItemWithCommand(g_menu_id, "Disconnect from SITL", toggleEnableCmd);
	}
	else {
		XPLMAppendMenuItemWithCommand(g_menu_id, "Connect to SITL", toggleEnableCmd);
	}
}


// Placeholder function for handling airframe selection commands.
void handleAirframeSelection(const std::string& airframeName) {
	// TODO: Implement the logic for when a user selects an airframe from the "Airframes" submenu.
	debugLog(("Airframe selected: " + airframeName).c_str());
	// Perform necessary actions to activate the selected airframe.
}




// TARGET update periods (in seconds) - CONFIGURABLE via config.ini
// These are loaded from ConfigManager and converted to periods
// Using simulation time ensures consistent timing regardless of rendering FPS
//
// IMPORTANT: Higher rates require higher X-Plane FPS
// - 200 Hz sensor rate needs ~150+ FPS for stability
// - 75 Hz sensor rate works well with 60 FPS
// - Formula: Recommended FPS = target_hz × 1.5 (for margin)
//
// PX4 SITL BEST PRACTICES (Gazebo/jMAVSim standards):
// - HIL_SENSOR: 200-250 Hz (IMU/barometer - highest priority)
// - HIL_GPS: 5-10 Hz (GPS updates - realistic rate)
// - HIL_STATE_QUATERNION: 50 Hz (ground truth logging)
// - HIL_RC_INPUTS: 50 Hz (RC receiver standard rate)
//
// NOTE: Periods are calculated from Hz rates in initializeMessagePeriods()
static float TARGET_SENSOR_PERIOD = 0.005f;      // Default 200 Hz (updated from config)
static float TARGET_GPS_PERIOD = 0.1f;           // Default 10 Hz (updated from config)
static float TARGET_STATE_QUAT_PERIOD = 0.02f;   // Default 50 Hz (updated from config)
static float TARGET_RC_PERIOD = 0.02f;           // Default 50 Hz (updated from config)

// Initialize periods from config (called once during plugin startup)
void initializeMessagePeriods() {
    TARGET_SENSOR_PERIOD = 1.0f / (float)ConfigManager::mavlink_sensor_rate_hz;
    TARGET_GPS_PERIOD = 1.0f / (float)ConfigManager::mavlink_gps_rate_hz;
    TARGET_STATE_QUAT_PERIOD = 1.0f / (float)ConfigManager::mavlink_state_rate_hz;
    TARGET_RC_PERIOD = 1.0f / (float)ConfigManager::mavlink_rc_rate_hz;

    char buf[300];
    snprintf(buf, sizeof(buf),
        "px4xplane: Message periods initialized - SENSOR:%.4fs (%dHz) GPS:%.4fs (%dHz) STATE:%.4fs (%dHz) RC:%.4fs (%dHz)\n",
        TARGET_SENSOR_PERIOD, ConfigManager::mavlink_sensor_rate_hz,
        TARGET_GPS_PERIOD, ConfigManager::mavlink_gps_rate_hz,
        TARGET_STATE_QUAT_PERIOD, ConfigManager::mavlink_state_rate_hz,
        TARGET_RC_PERIOD, ConfigManager::mavlink_rc_rate_hz);
    XPLMDebugString(buf);
}

// Track LAST SEND TIME (not accumulated time!)
// This approach is frame-rate independent and works at any FPS (30-150+)
static float lastSensorSendTime = 0.0f;
static float lastGpsSendTime = 0.0f;
static float lastStateQuatSendTime = 0.0f;
static float lastRcSendTime = 0.0f;

float lastFlightTime = 0.0f;

// Implementation of resetFlightLoopTimers (declared earlier, defined here after statics)
void resetFlightLoopTimers() {
	lastSensorSendTime = 0.0f;
	lastGpsSendTime = 0.0f;
	lastStateQuatSendTime = 0.0f;
	lastRcSendTime = 0.0f;
	XPLMDebugString("px4xplane: Flight loop timers reset\n");
}

float MyFlightLoopCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void* inRefcon) {
	// CRITICAL FIX (January 2025): Non-blocking connection polling with timeout
	// Poll for incoming PX4 connection if socket is waiting
	static float waitStartTime = 0.0f;
	static const float CONNECTION_TIMEOUT = 30.0f;  // 30 second timeout

	if (ConnectionManager::isWaitingForConnection()) {
		// Get current simulation time for timeout tracking
		float currentSimTime = XPLMGetDataf(XPLMFindDataRef("sim/time/total_running_time_sec"));

		if (waitStartTime == 0.0f) {
			waitStartTime = currentSimTime;
		}

		float elapsed = currentSimTime - waitStartTime;

		// Update HUD with current wait time
		ConnectionStatusHUD::updateStatus(
			ConnectionStatusHUD::Status::WAITING,
			"Start PX4 SITL to connect",
			elapsed
		);

		// Timeout after 30 seconds
		if (elapsed > CONNECTION_TIMEOUT) {
			XPLMDebugString("px4xplane: Connection timeout after 30s\n");

			ConnectionManager::setLastMessage(
				"Connection timeout. Is PX4 running? Click Connect to retry.");
			XPLMSpeakString("Connection timeout");

			// Update HUD to show timeout
			ConnectionStatusHUD::updateStatus(
				ConnectionStatusHUD::Status::TIMEOUT,
				"Is PX4 SITL running?"
			);

			// Auto-disconnect to allow retry
			ConnectionManager::disconnect();
			waitStartTime = 0.0f;
			return -1.0f;
		}

		// Try to accept connection (non-blocking)
		ConnectionManager::tryAcceptConnection();

		// If still waiting, return and try next frame
		if (!ConnectionManager::isConnected()) {
			return -1.0f;
		}

		// Connection succeeded - reset wait timer and update HUD
		ConnectionStatusHUD::updateStatus(
			ConnectionStatusHUD::Status::CONNECTED,
			"Ready to fly!"
		);
		waitStartTime = 0.0f;

		// Phase 3: SensorPublisher (DISABLED in v3.3.1)
		// if (g_sensorPublisher && !g_sensorPublisher->isRunning()) {
		// 	g_sensorPublisher->start();
		// 	XPLMDebugString("px4xplane: SensorPublisher started (200Hz fixed rate)\n");
		// }
		XPLMDebugString("px4xplane: v3.3.1 - Using direct sendHILSensor() in flight loop\n");
	} else {
		waitStartTime = 0.0f;  // Reset when not waiting

		// Hide HUD when not connecting
		if (!ConnectionManager::isConnected()) {
			ConnectionStatusHUD::updateStatus(ConnectionStatusHUD::Status::DISCONNECTED);
		}
	}

	// Ensure we're connected before proceeding with sensor data
	if (!ConnectionManager::isConnected()) return -1.0f;

	// Get current simulation time (frame-rate independent!)
	float currentSimTime = XPLMGetDataf(XPLMFindDataRef("sim/time/total_flight_time_sec"));

	// Check for simulation restart
	if (currentSimTime < lastFlightTime) {
		if (ConnectionManager::isConnected()) {
			toggleEnable();
		}
		// Reset all timing trackers on simulation restart
		lastSensorSendTime = 0.0f;
		lastGpsSendTime = 0.0f;
		lastStateQuatSendTime = 0.0f;
		lastRcSendTime = 0.0f;
	}

	// ==================================================================================
	// SENSOR DATA - Direct sendHILSensor() at target rate
	// ==================================================================================
	// v3.3.1 HOTFIX: Reverted from SensorPublisher thread approach
	//
	// The SensorPublisher with Kalman interpolation (v3.3.0) caused WORSE "High
	// Accelerometer Bias" errors because:
	// 1. Kalman interpolation creates artificially smooth data PX4 doesn't expect
	// 2. Perfectly regular 200Hz timestamps are unrealistic
	// 3. Bias drift was applied before interpolation, getting compounded
	//
	// Back to proven direct sendHILSensor() approach - simple and reliable.
	// FPS-dependent but works correctly when X-Plane runs at reasonable FPS.
	// ==================================================================================

	// Send sensor data at target rate using direct sendHILSensor()
	if ((currentSimTime - lastSensorSendTime) >= TARGET_SENSOR_PERIOD) {
		MAVLinkManager::sendHILSensor(0);

		// Statistics tracking for debugging
		static int sensorMessageCount = 0;
		static float sumRate = 0.0f;
		sensorMessageCount++;

		float actualDt_sec = currentSimTime - lastSensorSendTime;
		if (actualDt_sec > 0.0f) {
			float actualRate = 1.0f / actualDt_sec;
			sumRate += actualRate;
		}

		// Log statistics every 1000 messages (~5 seconds at 200Hz)
		if (ConfigManager::debug_log_sensor_timing && sensorMessageCount % 1000 == 0) {
			float avgRate = sumRate / 1000.0f;
			char buf[256];
			snprintf(buf, sizeof(buf),
				"px4xplane: [v3.3.1 @ %.1fs] HIL_SENSOR: %d msgs, avg %.1f Hz (target %d Hz)\n",
				currentSimTime, sensorMessageCount, avgRate, ConfigManager::mavlink_sensor_rate_hz);
			XPLMDebugString(buf);
			sumRate = 0.0f;
		}

		lastSensorSendTime = currentSimTime;
	}

	// Send GPS data at target rate (20 Hz) - one message per eligible frame
	if ((currentSimTime - lastGpsSendTime) >= TARGET_GPS_PERIOD) {
		MAVLinkManager::sendHILGPS();
		lastGpsSendTime = currentSimTime;
	}

	// Optional: Send state quaternion data (10 Hz)
	if ((currentSimTime - lastStateQuatSendTime) >= TARGET_STATE_QUAT_PERIOD) {
		MAVLinkManager::sendHILStateQuaternion();
		lastStateQuatSendTime = currentSimTime;
	}

	// Optional: Send RC data (10 Hz)
	if ((currentSimTime - lastRcSendTime) >= TARGET_RC_PERIOD) {
		MAVLinkManager::sendHILRCInputs();
		lastRcSendTime = currentSimTime;
	}


	// Continuously receive and process actuator data
	ConnectionManager::receiveData();

	// Actuator overrides
	DataRefManager::overrideActuators();

	// Update the SITL timestep with the loop callback rate
	DataRefManager::SIM_Timestep = inElapsedSinceLastCall;

	lastFlightTime = currentSimTime;

	// Return -1.0f to be called EVERY FRAME (most robust for frame-rate independence)
	// This ensures the plugin works correctly at any X-Plane FPS (30-150+)
	// Message rates are controlled by simulation time, not callback frequency
	return -1.0f;
}









PLUGIN_API void XPluginStop(void) {
	// Unregister the flight loop callback
	if (ConnectionManager::isConnected()) {
		toggleEnable();
	}

	// Phase 3: SensorPublisher cleanup (DISABLED in v3.3.1)
	// if (g_sensorPublisher) {
	// 	g_sensorPublisher->stop();
	// 	g_sensorPublisher.reset();
	// 	XPLMDebugString("px4xplane: SensorPublisher destroyed\n");
	// }

	XPLMUnregisterFlightLoopCallback(MyFlightLoopCallback, NULL);

	// Cleanup UI systems
	UIHandler::cleanup();
	ConnectionStatusHUD::cleanup();

#if IBM
	ConnectionManager::cleanupWinSock();
	XPLMDebugString("px4xplane: WinSock cleaned up\n");
#endif
	// ...
	XPLMDebugString("px4xplane: Plugin stopped\n");
}