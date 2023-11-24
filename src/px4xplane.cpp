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


#if IBM
#include <windows.h>
#endif
#if LIN || APL
#include <unistd.h>  // For TCP/IP
#include <sys/socket
.h> // For TCP/IP
#include <netinet/in.h> // For TCP/IP
#endif
#include "MAVLinkManager.h"

#ifndef XPLM300
#error This is made to be compiled against the XPLM300 SDK
#endif

static XPLMWindowID g_window;
static XPLMMenuID g_menu_id;
// Global variable to hold our command reference
static XPLMCommandRef toggleEnableCmd;

static int g_enabled = 0; // Used to toggle connection state

// Global variable to store the SITL frequency
double SIM_Timestep = 0.0;



void draw_px4xplane(XPLMWindowID in_window_id, void* in_refcon);
void menu_handler(void* in_menu_ref, void* in_item_ref);
int toggleEnableHandler(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon);
void create_menu();
void toggleEnable();
void updateMenuItems();
float MyFlightLoopCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void* inRefcon);



int drawHeader(int l, int t, float col_white[]) {
	char header[512];
	snprintf(header, sizeof(header), "PX4-XPlane Interface v1.1.0");
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

	// Draw Connection Status
	const std::string& status = ConnectionManager::getStatus();
	snprintf(buf, sizeof(buf), "Status: %s", status.c_str());
	XPLMDrawString(col_white, l + 10, t - lineOffset, buf, NULL, xplmFont_Proportional);
	lineOffset += 20;

	// Draw SITL Frequency
	snprintf(buf, sizeof(buf), "SITL Time Step: %.3f", SIM_Timestep);
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
	strcpy(outName, "px4xplane");
	strcpy(outSig, "alireza787.px4xplane");
	strcpy(outDesc, "PX4 SITL Interface for X-Plane.");

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
	params.left = left + 50;
	params.bottom = bottom + 150;
	params.right = params.left + 600;
	params.top = params.bottom + 850;

	g_window = XPLMCreateWindowEx(&params);

	XPLMSetWindowPositioningMode(g_window, xplm_WindowPositionFree, -1);
	XPLMSetWindowResizingLimits(g_window, 200, 200, 600, 1000);
	XPLMSetWindowTitle(g_window, "px4xplane");


#if IBM
	if (!ConnectionManager::initializeWinSock()) {
		XPLMDebugString("initializeWinSock failed \n");
	}
#endif

	// Register the flight loop callback to be called at the next cycle
	XPLMRegisterFlightLoopCallback(MyFlightLoopCallback, -1.0f, NULL);




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
	lineOffset = DataRefManager::drawActualThrottle(in_window_id, rightColumnPosition, t, col_white, lineOffset+20);

	// Draw Footer
	drawFooter(l, b, col_white);
}




void menu_handler(XPLMMenuID in_menu, void* in_item_ref) {
    if (in_item_ref == (void*)0) { // Show Window item
        XPLMSetWindowIsVisible(g_window, 1);
    }
    else if (in_item_ref == (void*)1) { // Toggle enable
        toggleEnable();
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
	int menu_container_idx = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "px4xplane", NULL, 1);
	g_menu_id = XPLMCreateMenu("px4xplane", XPLMFindPluginsMenu(), menu_container_idx, menu_handler, NULL);

	XPLMAppendMenuItem(g_menu_id, "Show Settings", (void*)0, 1);

	toggleEnableCmd = XPLMCreateCommand("px4xplane/toggleEnable", "Toggle enable/disable state");
	XPLMRegisterCommandHandler(toggleEnableCmd, toggleEnableHandler, 1, (void*)0);
	XPLMAppendMenuSeparator(g_menu_id);

	XPLMAppendMenuItemWithCommand(g_menu_id, "Connect to SITL", toggleEnableCmd);  // Update this dynamically based on connection state
	XPLMAppendMenuItemWithCommand(g_menu_id, "Disconnect from SITL", toggleEnableCmd);  // Update this dynamically based on connection state
	updateMenuItems(); // Update menu items after toggling connection

}

void updateMenuItems() {
	XPLMClearAllMenuItems(g_menu_id); // Clear all existing items
	XPLMAppendMenuItem(g_menu_id, "Show Settings", (void*)0, 1);
	XPLMAppendMenuSeparator(g_menu_id);
	if (ConnectionManager::isConnected()) {
		XPLMAppendMenuItemWithCommand(g_menu_id, "Disconnect from SITL", toggleEnableCmd);
	}
	else {
		XPLMAppendMenuItemWithCommand(g_menu_id, "Connect to SITL", toggleEnableCmd);
	}
}



float MyFlightLoopCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void* inRefcon) {
    
    // Check if the plugin is connected to PX4 SITL
    if (!ConnectionManager::isConnected()) return -1.0f; // Return -1 to be called at the next cycle

    // Call MAVLinkManager::sendHILSensor() to send HIL_SENSOR data
    MAVLinkManager::sendHILSensor(uint8_t(0));
    MAVLinkManager::sendHILGPS();
    /*MAVLinkManager::sendHILStateQuaternion();
    MAVLinkManager::sendHILRCInputs();*/

    ConnectionManager::receiveData();

    // Check the configuration type and call the appropriate actuator override function
    if (ConfigManager::getConfigTypeCode() == 1) {
        // Multirotor configuration
        DataRefManager::overrideActuators_multirotor();
    } else if (ConfigManager::getConfigTypeCode() == 2) {
        // Fixed-wing configuration
        // TODO: Implement and call the overrideActuators_fixedwing function
        // DataRefManager::overrideActuators_fixedwing();
    }

    // Calculate the frequency 
    SIM_Timestep = inElapsedSinceLastCall;

    return -1.0f; // Return -1 to be called at the next cycle
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

