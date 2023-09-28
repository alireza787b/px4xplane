#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMMenus.h"
#include <string.h>

#if IBM
#include <windows.h>
#endif
#if LIN || APL
#include <unistd.h>  // For TCP/IP
#include <sys/socket.h> // For TCP/IP
#include <netinet/in.h> // For TCP/IP
#endif

#ifndef XPLM300
#error This is made to be compiled against the XPLM300 SDK
#endif

static XPLMWindowID	g_window;
static XPLMMenuID     g_menu_id;

void draw_px4xplane(XPLMWindowID in_window_id, void* in_refcon);
void menu_handler(void* in_menu_ref, void* in_item_ref);
void create_menu();

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
	params.right = params.left + 400;
	params.top = params.bottom + 400;

	g_window = XPLMCreateWindowEx(&params);

	XPLMSetWindowPositioningMode(g_window, xplm_WindowPositionFree, -1);
	XPLMSetWindowResizingLimits(g_window, 200, 200, 500, 500);
	XPLMSetWindowTitle(g_window, "px4xplane");

	// TODO: Setup the TCP connection here
	// int sock = socket(AF_INET, SOCK_STREAM, 0);
	// ...

	return 1;
}

PLUGIN_API void	XPluginStop(void)
{
	XPLMDestroyWindow(g_window);
	g_window = NULL;
	XPLMDestroyMenu(g_menu_id);
	// TODO: Close the TCP connection here
	// close(sock);
}

PLUGIN_API void XPluginDisable(void) { }
PLUGIN_API int  XPluginEnable(void) { return 1; }
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMsg, void* inParam) { }

void draw_px4xplane(XPLMWindowID in_window_id, void* in_refcon)
{
	XPLMSetGraphicsState(0, 0, 0, 0, 1, 1, 0);

	int l, t, r, b;
	XPLMGetWindowGeometry(in_window_id, &l, &t, &r, &b);

	float col_white[] = { 1.0, 1.0, 1.0 };

	char* simulatorData = "Simulator Data: \n DataRef1: Placeholder\n DataRef2: Placeholder\n ...";
	char* autopilotData = "Autopilot Data: \n Motor1: Placeholder\n Motor2: Placeholder\n ...";

	XPLMDrawString(col_white, l + 10, t - 20, simulatorData, NULL, xplmFont_Proportional);
	XPLMDrawString(col_white, r - 200, t - 20, autopilotData, NULL, xplmFont_Proportional);
	// Replace with real data as you progress
}

void menu_handler(XPLMMenuID in_menu, void* in_item_ref) {
	if (in_item_ref == (void*)0) { // Show Window item
		XPLMSetWindowIsVisible(g_window, 1);
	}
}


void create_menu() {
	int menu_container_idx = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "px4xplane", NULL, 1);
	g_menu_id = XPLMCreateMenu("px4xplane", XPLMFindPluginsMenu(), menu_container_idx, menu_handler, NULL);
	XPLMAppendMenuItem(g_menu_id, "Show Window", (void*)0, 1);
}
