#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMDataAccess.h"
#include "XPLMMenus.h"
#include <string>
#include <stdio.h>
#include <vector>
#include <functional> // Include this
#include <variant>

#if IBM
#include <windows.h>
#endif
#if LIN || APL
#include <unistd.h>  // For TCP/IP
#include <sys/socket
.h> // For TCP/IP
#include <netinet/in.h> // For TCP/IP
#endif

#ifndef XPLM300
#error This is made to be compiled against the XPLM300 SDK
#endif

static XPLMWindowID g_window;
static XPLMMenuID g_menu_id;
// Global variable to hold our command reference
static XPLMCommandRef toggleEnableCmd;

static int g_enabled = 0; // Used to toggle connection state

void draw_px4xplane(XPLMWindowID in_window_id, void* in_refcon);
void menu_handler(void* in_menu_ref, void* in_item_ref);
int toggleEnableHandler(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon);
void create_menu();
void toggleEnable();
int getDataRefInt(const char* dataRefName);

float getDataRefFloat(const char* dataRefName);

double getDataRefDouble(const char* dataRefName);

std::vector<float> getDataRefFloatArray(const char* dataRefName);


enum DataRefType {
	DRT_FLOAT,
	DRT_DOUBLE,
	DRT_INT,
	DRT_FLOAT_ARRAY
};

float getFloat(const char* dataRef) {
	XPLMDataRef ref = XPLMFindDataRef(dataRef);
	if (ref) return XPLMGetDataf(ref);
	return 0; // or some other suitable default or error value
}

double getDouble(const char* dataRef) {
	XPLMDataRef ref = XPLMFindDataRef(dataRef);
	if (ref) return XPLMGetDatad(ref);
	return 0; // or some other suitable default or error value
}

std::vector<float> getFloatArray(const char* dataRefName) {
	// This is just a placeholder; replace with the actual SDK call to get float array dataRef.
	XPLMDataRef dataRef = XPLMFindDataRef(dataRefName);
	int arraySize = XPLMGetDatavf(dataRef, NULL, 0, 0); // To get the array size
	std::vector<float> values(arraySize);
	XPLMGetDatavf(dataRef, values.data(), 0, arraySize); // To get the array data
	return values;
}
std::string arrayToString(const std::vector<float>& array) {
	std::string result = "[";
	for (size_t i = 0; i < array.size(); ++i) {
		result += std::to_string(array[i]);
		if (i < array.size() - 1)
			result += ",";
	}
	result += "]";
	return result;
}


struct DataRefItem {
	const char* category;
	const char* title;
	const char* dataRef;
	const char* unit;
	float multiplier;
	DataRefType type;
	std::variant<std::function<float(const char*)>, std::function<double(const char*)>, std::function<int(const char*)>, std::function<std::vector<float>(const char*)>> getter;
};

std::vector<DataRefItem> dataRefs = {
	// Position Information
	{"Position", "Latitude", "sim/flightmodel/position/latitude", "deg", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"Position", "Longitude", "sim/flightmodel/position/longitude", "deg", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"Position", "Altitude", "sim/flightmodel/position/elevation", "m", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},

	// Attitude Information
	{"Attitude", "Pitch", "sim/flightmodel/position/theta", "deg", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"Attitude", "Roll", "sim/flightmodel/position/phi", "deg", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"Attitude", "Heading", "sim/flightmodel/position/psi", "deg", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"Attitude", "Quaternion", "sim/flightmodel/position/q", "", 1, DRT_FLOAT_ARRAY, std::function<std::vector<float>(const char*)>(getFloatArray)},

	// Acceleration Information
	{"Acceleration (NED)", "X-Acc", "sim/flightmodel/position/local_ax", "m/s^2", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"Acceleration (NED)", "Y-Acc", "sim/flightmodel/position/local_ay", "m/s^2", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"Acceleration (NED)", "Z-Acc", "sim/flightmodel/position/local_az", "m/s^2", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},

	// Ground Speed Information
	{"Ground Speed", "Vx", "sim/flightmodel/position/local_vx", "m/s", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"Ground Speed", "Vy", "sim/flightmodel/position/local_vy", "m/s", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"Ground Speed", "Vz", "sim/flightmodel/position/local_vz", "m/s", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},

	// Airspeed Information
	{"Airspeed", "IAS", "sim/flightmodel/position/indicated_airspeed", "knots", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},

	// Pressure Information
	{"Pressure", "Pressure Altitude", "sim/flightmodel2/position/pressure_altitude", "m", 0.3048, DRT_DOUBLE, std::function<double(const char*)>(getDouble)},
	{"Pressure", "Absolute Pressure", "sim/weather/barometer_current_inhg", "hPa", 33.8639, DRT_FLOAT, std::function<float(const char*)>(getFloat)},

	// Angular Velocities
	{"Angular Velocity", "P", "sim/flightmodel/position/P", "rad/s", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"Angular Velocity", "Q", "sim/flightmodel/position/Q", "rad/s", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"Angular Velocity", "R", "sim/flightmodel/position/R", "rad/s", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},

	// Temperature
	{"Temperature", "Outside Air Temp", "sim/cockpit2/temperature/outside_air_temp_degc", "degC", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},

	// Time Information
	{"Time", "Total Flight Time", "sim/time/total_flight_time_sec", "s", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
	{"Time", "Zulu Time", "sim/time/zulu_time_sec", "s", 1, DRT_FLOAT, std::function<float(const char*)>(getFloat)},
};





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
	params.right = params.left + 500;
	params.top = params.bottom + 800;

	g_window = XPLMCreateWindowEx(&params);

	XPLMSetWindowPositioningMode(g_window, xplm_WindowPositionFree, -1);
	XPLMSetWindowResizingLimits(g_window, 200, 200, 600, 1000);
	XPLMSetWindowTitle(g_window, "px4xplane");

	// TODO: Setup the TCP connection here
	// int sock = socket(AF_INET, SOCK_STREAM, 0);
	// ...

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
	int lineOffset = 40; // Incremented to leave space for the heading

	char* heading = "Simulator Data";
	XPLMDrawString(col_white, l + 10, t - lineOffset, heading, NULL, xplmFont_Proportional);

	lineOffset += 40; // Incremented to leave space below the heading

	char buf[512];

	const char* currentCategory = nullptr;

	for (const auto& item : dataRefs) {
		// If currentCategory is either null or different from item.category
		if (currentCategory == nullptr || strcmp(currentCategory, item.category) != 0) {
			currentCategory = item.category;
			// Draw category header
			XPLMDrawString(col_white, l + 10, t - lineOffset, (char*)currentCategory, NULL, xplmFont_Proportional);
			lineOffset += 20; // Increment line offset for next items
		}

		std::string valueStr; // string to store the formatted value

		// Check if the getter holds a float function
		if (std::holds_alternative<std::function<float(const char*)>>(item.getter)) {
			auto getter = std::get<std::function<float(const char*)>>(item.getter);
			float value = getter(item.dataRef) * item.multiplier;
			valueStr = std::to_string(value);
		}
		// Check if the getter holds a double function
		else if (std::holds_alternative<std::function<double(const char*)>>(item.getter)) {
			auto getter = std::get<std::function<double(const char*)>>(item.getter);
			double value = getter(item.dataRef) * item.multiplier;
			valueStr = std::to_string(value);
		}
		// Check if the getter holds an int function
		else if (std::holds_alternative<std::function<int(const char*)>>(item.getter)) {
			auto getter = std::get<std::function<int(const char*)>>(item.getter);
			int value = getter(item.dataRef) * static_cast<int>(item.multiplier);
			valueStr = std::to_string(value);
		}
		// Check if the getter holds a float vector function
		else if (std::holds_alternative<std::function<std::vector<float>(const char*)>>(item.getter)) {
			auto getter = std::get<std::function<std::vector<float>(const char*)>>(item.getter);
			std::vector<float> arrayValues = getter(item.dataRef);
			valueStr = arrayToString(arrayValues);
		}

		// Draw item details like title, value, and unit
		snprintf(buf, sizeof(buf), "  %s: %s %s", item.title, valueStr.c_str(), item.unit);
		XPLMDrawString(col_white, l + 10, t - lineOffset, buf, NULL, xplmFont_Proportional);
		lineOffset += 20; // Increment line offset for next items
	}

	// Draw other elements like copyright footer at the end
	char* footer = "Copyright (c) Alireza Ghaderi - https://github.com/alireza787b/px4xplane";
	XPLMDrawString(col_white, l + 10, b + 10, footer, NULL, xplmFont_Proportional);
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
    g_enabled = !g_enabled;
    // TODO: Implement actual enabling and disabling logic here
    // For instance, initiating or terminating the connection with PX4
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

	XPLMAppendMenuItem(g_menu_id, "Show Settings", (void*)0, 1); // This should be 0 as per your menu handler function.

	toggleEnableCmd = XPLMCreateCommand("px4xplane/toggleEnable", "Toggle enable/disable state");
	XPLMRegisterCommandHandler(toggleEnableCmd, toggleEnableHandler, 1, (void*)0);
	XPLMAppendMenuSeparator(g_menu_id); // Just to separate toggle from other menu items for better user experience
	XPLMAppendMenuItemWithCommand(g_menu_id, "Toggle Enable", toggleEnableCmd);
}

