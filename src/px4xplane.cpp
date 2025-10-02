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
#include "ConfigReader.h"
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
#include "VersionInfo.h"
#include "UIConstants.h"

#ifndef XPLM300
#error This is made to be compiled against the XPLM300 SDK
#endif

static XPLMWindowID g_window;
static XPLMMenuID g_menu_id;
static XPLMMenuID airframesMenu; // Global variable for the airframes submenu
static int g_airframesMenuItemIndex; // Global variable to store the index of the airframes submenu


// Global variable to hold our command reference
static XPLMCommandRef toggleEnableCmd;

static int g_enabled = 0; // Used to toggle connection state





void draw_px4xplane(XPLMWindowID in_window_id, void* in_refcon);
void menu_handler(void* in_menu_ref, void* in_item_ref);
int toggleEnableHandler(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon);
void create_menu();
void toggleEnable();
void updateMenuItems();
float MyFlightLoopCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void* inRefcon);


// Debugging function - Logs a message to the X-Plane log
void debugLog(const char* message) {
	XPLMDebugString("px4xplane: ");
	XPLMDebugString(message);
	XPLMDebugString("\n");
}



int drawHeader(int l, int t, float col_white[]) {
	char header[512];
	// Use centralized version info for consistent branding
	snprintf(header, sizeof(header), "PX4-XPlane Interface %s", PX4XPlaneVersion::getFullVersionString());
	XPLMDrawString(col_white, l + 10, t - 20, header, NULL, xplmFont_Proportional);
	return l + 20;
}

int drawStatusAndConfig(int l, int t, float col_white[], int& lineOffset, int columnWidth) {
	char buf[512];
	int droneConfigOffset = 20; // Adjust this offset as needed



	// Get the formatted drone configuration string
	std::string droneConfigStr = DataRefManager::GetFormattedDroneConfig();
	// Split the string into two lines for display
	std::istringstream iss(droneConfigStr);
	std::string line;
	while (std::getline(iss, line)) {
		char lineBuffer[512]; // Ensure this buffer is large enough
		strncpy(lineBuffer, line.c_str(), sizeof(lineBuffer));
		lineBuffer[sizeof(lineBuffer) - 1] = '\0'; // Ensure null termination

		XPLMDrawString(col_white, l + droneConfigOffset, t - lineOffset, lineBuffer, NULL, xplmFont_Proportional);
		lineOffset += 20;
	}

	// Draw Connection Status with high-contrast color coding
	const std::string& status = ConnectionManager::getStatus();
	bool isConnected = ConnectionManager::isConnected();

	// Use professional color indicators
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

	// Show detailed status message
	snprintf(buf, sizeof(buf), "Details: %s", status.c_str());
	XPLMDrawString(col_white, l + 10, t - lineOffset, buf, NULL, xplmFont_Proportional);
	lineOffset += 20;

	// Draw SITL Frequency
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

	debugLog("Plugin starting with enhanced UI system...");
	debugLog(("Version: " + std::string(PX4XPlaneVersion::getFullVersionString())).c_str());

	create_menu();

	XPLMCreateWindow_t params;
	params.structSize = sizeof(params);
	params.visible = 0; // Window is initially invisible
	params.drawWindowFunc = draw_px4xplane;
	// Note on handlers:
	// Note on handlers:
	// Register real handlers here as needed
	params.handleMouseClickFunc = NULL; // Example: mouse_handler;
	params.handleRightClickFunc = NULL;
	params.handleMouseWheelFunc = NULL;
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

void draw_px4xplane(XPLMWindowID in_window_id, void* in_refcon) {
	int l, t, r, b;
	XPLMGetWindowGeometry(in_window_id, &l, &t, &r, &b);
	float col_white[] = { 1.0, 1.0, 1.0 };
	int lineOffset = 20;
	int columnWidth = 350; // Adjust as needed

	// Draw Header
	lineOffset = drawHeader(l, t, col_white);

	// Draw Status and Configuration
	lineOffset = drawStatusAndConfig(l, t, col_white, lineOffset, columnWidth);

	// Draw DataRefs
	lineOffset = DataRefManager::drawDataRefs(in_window_id, l, t, col_white, lineOffset);

	// Draw Actuator Controls
	int rightColumnPosition = l + columnWidth;
	lineOffset = 20;
	lineOffset = DataRefManager::drawActuatorControls(in_window_id, rightColumnPosition, t, col_white, lineOffset);

	// Draw Actual Throttle
	lineOffset = DataRefManager::drawActualThrottle(in_window_id, rightColumnPosition, t, col_white, lineOffset + 20);

	// Draw Footer
	drawFooter(l, b, col_white);
}

// Function to refresh the airframes submenu to indicate the active airframe.
void refreshAirframesMenu() {
	// Clear the current submenu items.
	XPLMClearAllMenuItems(airframesMenu);

	// Repopulate the submenu with updated airframe names and active status.
	std::vector<std::string> airframeNames = ConfigManager::getAirframeLists();
	for (const std::string& name : airframeNames) {
		// Append each airframe to the submenu and mark the active one.
		XPLMAppendMenuItem(airframesMenu, (name + (name == ConfigManager::getActiveAirframeName() ? " *" : "")).c_str(), (void*)new std::string(name), 1);
	}
}


void menu_handler(void* in_menu_ref, void* in_item_ref) {
	//TODO: Remember here there is a problem. handler cant find when we click on airframes. now I did a hack and put that in else when it is not part of main list menu. we should fix this later.
	debugLog("Menu handler called");

	if (in_menu_ref == (void*)g_menu_id) {
		if ((int)(intptr_t)in_item_ref == g_airframesMenuItemIndex) {
			debugLog("Airframes submenu selected");
			// The submenu itself was clicked; specific handling if needed
		}
		else {
			// Handling other main menu items
			if (in_item_ref == (void*)0) {
				XPLMSetWindowIsVisible(g_window, 1);
				debugLog("Show Data menu item selected");
			}
			else if (in_item_ref == (void*)1) {
				toggleEnable();
				debugLog("Toggle enable menu item selected");
			}
		}
	}
	else {
		// Handling airframes submenu interactions
		debugLog("Entered airframe submenu handler");

		int index = (int)(intptr_t)in_item_ref;
		std::vector<std::string> airframeNames = ConfigManager::getAirframeLists();

		if (index >= 0 && index < airframeNames.size()) {
			const std::string& selectedAirframe = airframeNames[index];
			debugLog(("Airframe selected: " + selectedAirframe).c_str());

			ConfigManager::setActiveAirframeName(selectedAirframe);
			refreshAirframesMenu();
		}
	}

}



void toggleEnable() {
	XPLMDebugString("px4xplane: toggleEnable() called.\n");
	if (ConnectionManager::isConnected()) {
		XPLMDebugString("px4xplane: Currently connected, attempting to disconnect.\n");
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
	g_menu_id = XPLMCreateMenu("px4xplane", XPLMFindPluginsMenu(), menu_container_idx, menu_handler, NULL);

	// Save the index of the airframes submenu in the main menu
	g_airframesMenuItemIndex = XPLMAppendMenuItem(g_menu_id, "Airframes", NULL, 1);
	airframesMenu = XPLMCreateMenu("Airframes", g_menu_id, g_airframesMenuItemIndex, menu_handler, NULL);


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
	airframesMenu = XPLMCreateMenu("Airframes", g_menu_id, g_airframesMenuItemIndex, menu_handler, NULL);

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




// Constants for update frequencies (in seconds)
// 250Hz for HIL_SENSOR improves EKF2 timing and prevents BARO STALE errors
const float BASE_SENSOR_UPDATE_PERIOD = 0.004f; // 250 Hz
const float BASE_GPS_UPDATE_PERIOD = 0.05f;     // 20 Hz
const float BASE_STATE_QUAT_UPDATE_PERIOD = 0.1f; // 10 Hz
const float BASE_RC_UPDATE_PERIOD = 0.1f;     // 10 Hz

// Global timing variables
float timeSinceLastSensorUpdate = 0.0f;
float timeSinceLastGpsUpdate = 0.0f;
float timeSinceLastStateQuaternionUpdate = 0.0f; // Optional
float timeSinceLastRcUpdate = 0.0f; // Optional

float lastFlightTime = 0.0f;

float MyFlightLoopCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void* inRefcon) {
	// Ensure we're connected before proceeding
	if (!ConnectionManager::isConnected()) return -1.0f;

	float currentFlightTime = XPLMGetDataf(XPLMFindDataRef("sim/time/total_flight_time_sec"));

	// Check for new flight start
	if (currentFlightTime < lastFlightTime) {
		if (ConnectionManager::isConnected()) {
			toggleEnable();
		}
	}

	// Update timing variables
	timeSinceLastSensorUpdate += inElapsedSinceLastCall;
	timeSinceLastGpsUpdate += inElapsedSinceLastCall;
	timeSinceLastStateQuaternionUpdate += inElapsedSinceLastCall;
	timeSinceLastRcUpdate += inElapsedSinceLastCall;

	// Send sensor data at high frequency
	if (timeSinceLastSensorUpdate >= BASE_SENSOR_UPDATE_PERIOD) {
		MAVLinkManager::sendHILSensor(uint8_t(0));
		timeSinceLastSensorUpdate = 0.0f;
	}

	// Send GPS data at a lower frequency
	if (timeSinceLastGpsUpdate >= BASE_GPS_UPDATE_PERIOD) {
		MAVLinkManager::sendHILGPS();
		timeSinceLastGpsUpdate = 0.0f;
	}

	// Optional: Send state quaternion data
	if (timeSinceLastStateQuaternionUpdate >= BASE_STATE_QUAT_UPDATE_PERIOD) {
		MAVLinkManager::sendHILStateQuaternion();
		timeSinceLastStateQuaternionUpdate = 0.0f;
	}

	// Optional: Send RC data
	if (timeSinceLastRcUpdate >= BASE_RC_UPDATE_PERIOD) {
		MAVLinkManager::sendHILRCInputs();
		timeSinceLastRcUpdate = 0.0f;
	}


	// Continuously receive and process actuator data
	ConnectionManager::receiveData();

	// Actuator overrides
	DataRefManager::overrideActuators();

	// Update the SITL timestep with the loop callback rate
	DataRefManager::SIM_Timestep = inElapsedSinceLastCall;

	lastFlightTime = currentFlightTime;

	// Always return a negative value to continue calling this function every frame
	return 0.005f; // Continue calling at the next cycle
	//return -1.0f;
}









PLUGIN_API void XPluginStop(void) {
	// Unregister the flight loop callback
	if (ConnectionManager::isConnected()) {
		toggleEnable();
	}

	XPLMUnregisterFlightLoopCallback(MyFlightLoopCallback, NULL);
#if IBM
	ConnectionManager::cleanupWinSock();
	XPLMDebugString("px4xplane: WinSock cleaned up\n");
#endif
	// ...
	XPLMDebugString("px4xplane: Plugin stopped\n");
}