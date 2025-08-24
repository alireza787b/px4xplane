/**
 * @file px4xplane_main.cpp
 * @brief PX4-XPlane Interface Plugin with Professional Tabbed Data Inspector (Phase 2)
 *
 * PHASE 2 ENHANCEMENTS:
 * - Professional tabbed interface with 4 organized categories
 * - Fixed text overflow with proper content clipping and bounds checking
 * - Per-tab scrolling support with enhanced scroll management
 * - Search framework ready for future text input enhancements
 * - Responsive layout that adapts to window resizing
 * - Smart content filtering and organization
 * - Clean, modern UI following X-Plane SDK best practices
 *
 * PHASE 1 FOUNDATION:
 * - Vertical scrolling capability maintained and enhanced
 * - Mouse wheel support with smooth scrolling
 * - Professional window management
 *
 * @author Alireza Ghaderi
 * @copyright Copyright (c) 2025 Alireza Ghaderi. All rights reserved.
 * @license MIT License
 * @version 2.5.0 (Phase 2)
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
#include <algorithm>
#include <cstring>
#include "ConfigReader.h"
#include "DataRefManager.h"
#include "ConnectionManager.h"
#include "ConfigManager.h"

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
 // GLOBAL VARIABLES (UNCHANGED FROM PHASE 1)
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
// PHASE 2: PROFESSIONAL TABBED INTERFACE SYSTEM
// ============================================================================

/**
 * @brief Enumeration for different tabs in the data inspector
 */
enum class ActiveTab {
    CONNECTION = 0,     // Connection status, system info, timestamps
    POSITION = 1,       // GPS, attitude, navigation, velocities  
    SENSORS = 2,        // IMU, pressure, temperature, airspeed
    CONTROLS = 3        // RC inputs, actuators, aircraft config
};

/**
 * @brief Tab configuration and metadata
 */
struct TabInfo {
    const char* name;
    const char* description;
    ActiveTab id;
};

static const TabInfo TAB_CONFIG[4] = {
    {"Connection", "System Status & Connection Info", ActiveTab::CONNECTION},
    {"Position", "GPS, Attitude & Navigation Data", ActiveTab::POSITION},
    {"Sensors", "IMU, Environment & Sensor Data", ActiveTab::SENSORS},
    {"Controls", "RC Inputs, Actuators & Aircraft", ActiveTab::CONTROLS}
};

/**
 * @brief Professional tab state management system
 */
struct TabState {
    ActiveTab currentTab = ActiveTab::CONNECTION;
    int scrollOffset[4] = { 0, 0, 0, 0 };        // Per-tab independent scrolling
    int contentHeight[4] = { 0, 0, 0, 0 };       // Per-tab content height tracking
    int maxScrollOffset[4] = { 0, 0, 0, 0 };     // Per-tab scroll limits
    bool needsScrollbar[4] = { false, false, false, false }; // Per-tab scroll indicators
    std::string searchFilter[4] = { "", "", "", "" }; // Per-tab search (framework for future)

    // Visual layout constants
    static constexpr int TAB_HEIGHT = 35;
    static constexpr int SEARCH_BOX_HEIGHT = 30;
    static constexpr int CONTENT_MARGIN = 10;
    static constexpr int SCROLL_SPEED = 25;
    static constexpr int MIN_VISIBLE_LINES = 5;

    // Tab positioning
    int tabAreaHeight = 0;
    int contentAreaTop = 0;
    int contentAreaBottom = 0;
    int visibleContentHeight = 0;
};

static TabState g_tabState;

/**
 * @brief Enhanced scroll state from Phase 1 - now tab-aware
 */
struct ScrollState {
    static constexpr int SCROLL_SPEED = 25;        // Increased for better UX
    static constexpr int MIN_CONTENT_MARGIN = 15;  // Better margins
    static constexpr int HEADER_HEIGHT = 80;       // Space for tabs + search
    static constexpr int FOOTER_HEIGHT = 40;       // Space for footer
};

static ScrollState g_scrollState;

// ============================================================================
// FORWARD DECLARATIONS (ENHANCED FOR PHASE 2)
// ============================================================================

// Core drawing functions
void draw_px4xplane(XPLMWindowID in_window_id, void* in_refcon);
void draw_about_window(XPLMWindowID in_window_id, void* in_refcon);

// Enhanced mouse handling
int handle_main_mouse(XPLMWindowID in_window_id, int x, int y, int is_down, void* in_refcon);
int handle_about_mouse(XPLMWindowID in_window_id, int x, int y, int is_down, void* in_refcon);
int handle_main_wheel(XPLMWindowID in_window_id, int x, int y, int wheel, int clicks, void* in_refcon);

// Tab system functions
void drawTabHeaders(int windowLeft, int windowTop, int windowRight, int windowBottom, float col_white[]);
void drawTabContent(int windowLeft, int windowTop, int windowRight, int windowBottom, float col_white[]);
void drawSearchBox(int windowLeft, int windowTop, int windowRight, int windowBottom, float col_white[]);
bool handleTabClick(int mouseX, int mouseY, int windowLeft, int windowTop, int windowRight, int windowBottom);
void switchToTab(ActiveTab newTab);
void updateTabScrollLimits(ActiveTab tab);

// Enhanced content drawing (tab-aware)
int drawConnectionTabContent(int l, int t, int r, int b, float col_white[], int startOffset);
int drawPositionTabContent(int l, int t, int r, int b, float col_white[], int startOffset);
int drawSensorsTabContent(int l, int t, int r, int b, float col_white[], int startOffset);
int drawControlsTabContent(int l, int t, int r, int b, float col_white[], int startOffset);

// Smart content filtering and clipping
bool isLineVisible(int yPosition, int contentTop, int contentBottom);
int drawDataRefCategory(int l, int t, float col_white[], int lineOffset, const char* categoryFilter, int contentTop, int contentBottom, int& drawnLines);
int drawSmartText(int x, int y, const char* text, float color[], int contentTop, int contentBottom);

// Utility functions
void debugLog(const char* message);
void calculateTabLayout(int windowLeft, int windowTop, int windowRight, int windowBottom);
void updateCurrentTabScroll();

// Original system functions (unchanged)
void menu_handler(void* in_menu_ref, void* in_item_ref);
void airframes_menu_handler(void* in_menu_ref, void* in_item_ref);
int toggleEnableHandler(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon);
void create_menu();
void create_airframes_menu();
void toggleEnable();
void refresh_airframes_menu();
float MyFlightLoopCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void* inRefcon);

// Window management functions (enhanced)
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

/**
 * @brief Calculate tab layout dimensions based on current window size
 */
void calculateTabLayout(int windowLeft, int windowTop, int windowRight, int windowBottom) {
    int windowHeight = windowTop - windowBottom;

    // Calculate tab area
    g_tabState.tabAreaHeight = TabState::TAB_HEIGHT + TabState::SEARCH_BOX_HEIGHT;
    g_tabState.contentAreaTop = windowTop - g_tabState.tabAreaHeight - TabState::CONTENT_MARGIN;
    g_tabState.contentAreaBottom = windowBottom + ScrollState::FOOTER_HEIGHT + TabState::CONTENT_MARGIN;
    g_tabState.visibleContentHeight = g_tabState.contentAreaTop - g_tabState.contentAreaBottom;

    // Ensure minimum content height
    if (g_tabState.visibleContentHeight < TabState::MIN_VISIBLE_LINES * 20) {
        g_tabState.visibleContentHeight = TabState::MIN_VISIBLE_LINES * 20;
        g_tabState.contentAreaBottom = g_tabState.contentAreaTop - g_tabState.visibleContentHeight;
    }
}

/**
 * @brief Update scroll limits for current tab
 */
void updateCurrentTabScroll() {
    int tabIndex = static_cast<int>(g_tabState.currentTab);

    g_tabState.needsScrollbar[tabIndex] = g_tabState.contentHeight[tabIndex] > g_tabState.visibleContentHeight;

    if (g_tabState.needsScrollbar[tabIndex]) {
        g_tabState.maxScrollOffset[tabIndex] = g_tabState.contentHeight[tabIndex] - g_tabState.visibleContentHeight;
        if (g_tabState.scrollOffset[tabIndex] > g_tabState.maxScrollOffset[tabIndex]) {
            g_tabState.scrollOffset[tabIndex] = g_tabState.maxScrollOffset[tabIndex];
        }
    }
    else {
        g_tabState.maxScrollOffset[tabIndex] = 0;
        g_tabState.scrollOffset[tabIndex] = 0;
    }

    // Prevent negative scrolling
    if (g_tabState.scrollOffset[tabIndex] < 0) {
        g_tabState.scrollOffset[tabIndex] = 0;
    }
}

/**
 * @brief Check if a line at given Y position is visible within content bounds
 */
bool isLineVisible(int yPosition, int contentTop, int contentBottom) {
    return (yPosition <= contentTop && yPosition >= contentBottom);
}

/**
 * @brief Smart text drawing with bounds checking - only draws if visible
 */
int drawSmartText(int x, int y, const char* text, float color[], int contentTop, int contentBottom) {
    if (isLineVisible(y, contentTop, contentBottom)) {
        XPLMDrawString(color, x, y, (char*)text, NULL, xplmFont_Proportional);
        return 1; // Line was drawn
    }
    return 0; // Line was clipped
}

// ============================================================================
// PHASE 2: PROFESSIONAL TAB SYSTEM IMPLEMENTATION
// ============================================================================

/**
 * @brief Switch to a different tab with proper state management
 */
void switchToTab(ActiveTab newTab) {
    if (g_tabState.currentTab != newTab) {
        g_tabState.currentTab = newTab;
        updateCurrentTabScroll();

        char debugMsg[128];
        snprintf(debugMsg, sizeof(debugMsg), "Switched to tab: %s", TAB_CONFIG[static_cast<int>(newTab)].name);
        debugLog(debugMsg);
    }
}

/**
 * @brief Handle mouse clicks on tab headers
 */
bool handleTabClick(int mouseX, int mouseY, int windowLeft, int windowTop, int windowRight, int windowBottom) {
    // Check if click is in tab header area
    int tabTop = windowTop - 5;
    int tabBottom = windowTop - TabState::TAB_HEIGHT - 5;

    if (mouseY < tabBottom || mouseY > tabTop) {
        return false; // Click not in tab area
    }

    // Calculate tab widths
    int windowWidth = windowRight - windowLeft;
    int tabWidth = windowWidth / 4;

    // Determine which tab was clicked
    for (int i = 0; i < 4; i++) {
        int tabLeft = windowLeft + (i * tabWidth);
        int tabRight = tabLeft + tabWidth;

        if (mouseX >= tabLeft && mouseX <= tabRight) {
            switchToTab(static_cast<ActiveTab>(i));
            return true; // Tab switch handled
        }
    }

    return false; // No tab clicked
}

/**
 * @brief Draw professional tab headers
 */
void drawTabHeaders(int windowLeft, int windowTop, int windowRight, int windowBottom, float col_white[]) {
    int windowWidth = windowRight - windowLeft;
    int tabWidth = windowWidth / 4;
    int tabTop = windowTop - 5;
    int tabBottom = windowTop - TabState::TAB_HEIGHT - 5;
    int currentTabIndex = static_cast<int>(g_tabState.currentTab);

    // Colors for active/inactive tabs
    float col_active[] = { 0.9f, 0.9f, 1.0f };      // Light blue for active
    float col_inactive[] = { 0.7f, 0.7f, 0.7f };    // Gray for inactive
    float col_border[] = { 0.5f, 0.5f, 0.5f };      // Border color

    // Draw tab backgrounds and borders
    for (int i = 0; i < 4; i++) {
        int tabLeft = windowLeft + (i * tabWidth);
        int tabRight = tabLeft + tabWidth;
        bool isActive = (i == currentTabIndex);

        // Draw tab border (simple line-based approach for X-Plane compatibility)
        float* borderColor = isActive ? col_active : col_border;
        char borderChar[] = "-";

        // Top border
        for (int x = tabLeft; x < tabRight; x += 8) {
            XPLMDrawString(borderColor, x, tabTop, borderChar, NULL, xplmFont_Proportional);
        }

        // Bottom border  
        for (int x = tabLeft; x < tabRight; x += 8) {
            XPLMDrawString(borderColor, x, tabBottom, borderChar, NULL, xplmFont_Proportional);
        }

        // Side borders
        char sideChar[] = "|";
        for (int y = tabBottom; y < tabTop; y += 4) {
            XPLMDrawString(borderColor, tabLeft, y, sideChar, NULL, xplmFont_Proportional);
            XPLMDrawString(borderColor, tabRight - 8, y, sideChar, NULL, xplmFont_Proportional);
        }

        // Draw tab label
        float* textColor = isActive ? col_white : col_inactive;
        int textX = tabLeft + (tabWidth - (strlen(TAB_CONFIG[i].name) * 6)) / 2; // Center text
        int textY = tabTop - 20;

        XPLMDrawString(textColor, textX, textY, (char*)TAB_CONFIG[i].name, NULL, xplmFont_Proportional);

        // Draw scroll indicator if needed
        if (g_tabState.needsScrollbar[i]) {
            char scrollIndicator[] = "↕";
            XPLMDrawString(textColor, tabRight - 15, textY, scrollIndicator, NULL, xplmFont_Proportional);
        }
    }
}

/**
 * @brief Draw search box framework (visual placeholder for future enhancement)
 */
void drawSearchBox(int windowLeft, int windowTop, int windowRight, int windowBottom, float col_white[]) {
    int searchTop = windowTop - TabState::TAB_HEIGHT - 10;
    int searchBottom = searchTop - TabState::SEARCH_BOX_HEIGHT;
    int searchLeft = windowLeft + 10;
    int searchRight = windowRight - 10;

    // Draw search box border
    float col_search[] = { 0.6f, 0.6f, 0.8f };
    char borderChar[] = "-";
    char sideChar[] = "|";

    // Top and bottom borders
    for (int x = searchLeft; x < searchRight; x += 8) {
        XPLMDrawString(col_search, x, searchTop, borderChar, NULL, xplmFont_Proportional);
        XPLMDrawString(col_search, x, searchBottom, borderChar, NULL, xplmFont_Proportional);
    }

    // Side borders
    for (int y = searchBottom; y < searchTop; y += 4) {
        XPLMDrawString(col_search, searchLeft, y, sideChar, NULL, xplmFont_Proportional);
        XPLMDrawString(col_search, searchRight - 8, y, sideChar, NULL, xplmFont_Proportional);
    }

    // Search placeholder text
    char searchText[256];
    const char* currentTabName = TAB_CONFIG[static_cast<int>(g_tabState.currentTab)].name;
    snprintf(searchText, sizeof(searchText), "🔍 Search %s data... (Feature ready for Phase 3)", currentTabName);

    XPLMDrawString(col_search, searchLeft + 10, searchTop - 18, searchText, NULL, xplmFont_Proportional);
}

/**
 * @brief Enhanced scroll indicator with per-tab information
 */
void drawEnhancedScrollIndicator(int windowLeft, int windowTop, int windowRight, int windowBottom, float col_white[]) {
    int currentTabIndex = static_cast<int>(g_tabState.currentTab);

    if (!g_tabState.needsScrollbar[currentTabIndex]) return;

    // Calculate scrollbar dimensions
    int scrollbarWidth = 12;
    int scrollbarLeft = windowRight - scrollbarWidth - 8;
    int scrollbarTop = g_tabState.contentAreaTop - 5;
    int scrollbarBottom = g_tabState.contentAreaBottom + 5;
    int scrollbarHeight = scrollbarTop - scrollbarBottom;

    if (scrollbarHeight <= 20) return;

    // Calculate thumb position and size
    float scrollRatio = (g_tabState.maxScrollOffset[currentTabIndex] > 0) ?
        (float)g_tabState.scrollOffset[currentTabIndex] / (float)g_tabState.maxScrollOffset[currentTabIndex] : 0.0f;
    float thumbRatio = (g_tabState.contentHeight[currentTabIndex] > 0) ?
        (float)g_tabState.visibleContentHeight / (float)g_tabState.contentHeight[currentTabIndex] : 1.0f;

    int thumbHeight = (int)(scrollbarHeight * thumbRatio);
    if (thumbHeight < 15) thumbHeight = 15; // Minimum thumb size

    int thumbTop = scrollbarTop - (int)(scrollRatio * (scrollbarHeight - thumbHeight));
    int thumbBottom = thumbTop - thumbHeight;

    // Draw enhanced scrollbar
    float col_track[] = { 0.4f, 0.4f, 0.4f };
    float col_thumb[] = { 0.8f, 0.8f, 0.9f };

    // Draw track
    char trackChar[] = "│";
    for (int i = scrollbarBottom; i < scrollbarTop; i += 3) {
        XPLMDrawString(col_track, scrollbarLeft + 2, i, trackChar, NULL, xplmFont_Proportional);
    }

    // Draw thumb
    char thumbChar[] = "█";
    for (int i = thumbBottom; i < thumbTop; i += 2) {
        XPLMDrawString(col_thumb, scrollbarLeft, i, thumbChar, NULL, xplmFont_Proportional);
    }

    // Draw scroll position info
    char scrollInfo[128];
    snprintf(scrollInfo, sizeof(scrollInfo), "Tab: %s | Content: %dpx | Scroll: %d/%d",
        TAB_CONFIG[currentTabIndex].name,
        g_tabState.contentHeight[currentTabIndex],
        g_tabState.scrollOffset[currentTabIndex],
        g_tabState.maxScrollOffset[currentTabIndex]);

    XPLMDrawString(col_white, windowLeft + 10, windowBottom + 5, scrollInfo, NULL, xplmFont_Proportional);
}

// ============================================================================
// PHASE 2: TAB-SPECIFIC CONTENT DRAWING
// ============================================================================

/**
 * @brief Draw Connection & Status tab content
 */
int drawConnectionTabContent(int l, int t, int r, int b, float col_white[], int startOffset) {
    int lineOffset = startOffset;
    char buf[512];

    // Connection Status Section
    int scrolledY = t - lineOffset + g_tabState.scrollOffset[0];
    if (drawSmartText(l + 10, scrolledY, "=== CONNECTION STATUS ===", col_white, g_tabState.contentAreaTop, g_tabState.contentAreaBottom)) {
        // Only increment if line was actually drawn
    }
    lineOffset += 25;

    // Connection status with visual indicator
    const std::string& status = ConnectionManager::getStatus();
    const char* statusIcon = ConnectionManager::isConnected() ? "🟢 [CONNECTED]" : "🔴 [DISCONNECTED]";
    snprintf(buf, sizeof(buf), "Status: %s %s", statusIcon, status.c_str());
    scrolledY = t - lineOffset + g_tabState.scrollOffset[0];
    drawSmartText(l + 20, scrolledY, buf, col_white, g_tabState.contentAreaTop, g_tabState.contentAreaBottom);
    lineOffset += 25;

    // SITL timestep info
    snprintf(buf, sizeof(buf), "SITL Timestep: %.3f ms (%.1f Hz)",
        DataRefManager::SIM_Timestep * 1000.0f, 1.0f / DataRefManager::SIM_Timestep);
    scrolledY = t - lineOffset + g_tabState.scrollOffset[0];
    drawSmartText(l + 20, scrolledY, buf, col_white, g_tabState.contentAreaTop, g_tabState.contentAreaBottom);
    lineOffset += 30;

    // Time Information Section
    scrolledY = t - lineOffset + g_tabState.scrollOffset[0];
    drawSmartText(l + 10, scrolledY, "=== TIME INFORMATION ===", col_white, g_tabState.contentAreaTop, g_tabState.contentAreaBottom);
    lineOffset += 25;

    // Draw Time category datarefs
    int drawnLines = 0;
    lineOffset = drawDataRefCategory(l, t, col_white, lineOffset, "Time", g_tabState.contentAreaTop, g_tabState.contentAreaBottom, drawnLines);

    return lineOffset;
}

/**
 * @brief Draw Position & Navigation tab content
 */
int drawPositionTabContent(int l, int t, int r, int b, float col_white[], int startOffset) {
    int lineOffset = startOffset;
    int drawnLines = 0;

    // Position Section
    int scrolledY = t - lineOffset + g_tabState.scrollOffset[1];
    drawSmartText(l + 10, scrolledY, "=== POSITION DATA ===", col_white, g_tabState.contentAreaTop, g_tabState.contentAreaBottom);
    lineOffset += 25;

    lineOffset = drawDataRefCategory(l, t, col_white, lineOffset, "Position", g_tabState.contentAreaTop, g_tabState.contentAreaBottom, drawnLines);
    lineOffset += 15;

    // Attitude Section
    scrolledY = t - lineOffset + g_tabState.scrollOffset[1];
    drawSmartText(l + 10, scrolledY, "=== ATTITUDE DATA ===", col_white, g_tabState.contentAreaTop, g_tabState.contentAreaBottom);
    lineOffset += 25;

    lineOffset = drawDataRefCategory(l, t, col_white, lineOffset, "Attitude", g_tabState.contentAreaTop, g_tabState.contentAreaBottom, drawnLines);
    lineOffset += 15;

    // Angular Velocity Section
    scrolledY = t - lineOffset + g_tabState.scrollOffset[1];
    drawSmartText(l + 10, scrolledY, "=== ANGULAR VELOCITIES ===", col_white, g_tabState.contentAreaTop, g_tabState.contentAreaBottom);
    lineOffset += 25;

    lineOffset = drawDataRefCategory(l, t, col_white, lineOffset, "Angular Velocity", g_tabState.contentAreaTop, g_tabState.contentAreaBottom, drawnLines);

    return lineOffset;
}

/**
 * @brief Draw Sensors & Environment tab content
 */
int drawSensorsTabContent(int l, int t, int r, int b, float col_white[], int startOffset) {
    int lineOffset = startOffset;
    int drawnLines = 0;

    // Acceleration Section
    int scrolledY = t - lineOffset + g_tabState.scrollOffset[2];
    drawSmartText(l + 10, scrolledY, "=== ACCELERATION (BODY FRAME) ===", col_white, g_tabState.contentAreaTop, g_tabState.contentAreaBottom);
    lineOffset += 25;

    lineOffset = drawDataRefCategory(l, t, col_white, lineOffset, "Acceleration (BODY)", g_tabState.contentAreaTop, g_tabState.contentAreaBottom, drawnLines);
    lineOffset += 15;

    // Airspeed Section
    scrolledY = t - lineOffset + g_tabState.scrollOffset[2];
    drawSmartText(l + 10, scrolledY, "=== AIRSPEED DATA ===", col_white, g_tabState.contentAreaTop, g_tabState.contentAreaBottom);
    lineOffset += 25;

    lineOffset = drawDataRefCategory(l, t, col_white, lineOffset, "Airspeed", g_tabState.contentAreaTop, g_tabState.contentAreaBottom, drawnLines);
    lineOffset += 15;

    // Sensors Section
    scrolledY = t - lineOffset + g_tabState.scrollOffset[2];
    drawSmartText(l + 10, scrolledY, "=== ENVIRONMENTAL SENSORS ===", col_white, g_tabState.contentAreaTop, g_tabState.contentAreaBottom);
    lineOffset += 25;

    lineOffset = drawDataRefCategory(l, t, col_white, lineOffset, "Sensors", g_tabState.contentAreaTop, g_tabState.contentAreaBottom, drawnLines);

    return lineOffset;
}

/**
 * @brief Draw Controls & Aircraft tab content
 */
int drawControlsTabContent(int l, int t, int r, int b, float col_white[], int startOffset) {
    int lineOffset = startOffset;
    int drawnLines = 0;

    // Aircraft Configuration Section
    int scrolledY = t - lineOffset + g_tabState.scrollOffset[3];
    drawSmartText(l + 10, scrolledY, "=== AIRCRAFT CONFIGURATION ===", col_white, g_tabState.contentAreaTop, g_tabState.contentAreaBottom);
    lineOffset += 25;

    // Get and display formatted config
    std::string droneConfigStr = DataRefManager::GetFormattedDroneConfig();
    std::istringstream iss(droneConfigStr);
    std::string line;

    while (std::getline(iss, line)) {
        scrolledY = t - lineOffset + g_tabState.scrollOffset[3];
        char lineBuffer[512];
        strncpy(lineBuffer, line.c_str(), sizeof(lineBuffer) - 1);
        lineBuffer[sizeof(lineBuffer) - 1] = '\0';
        drawSmartText(l + 20, scrolledY, lineBuffer, col_white, g_tabState.contentAreaTop, g_tabState.contentAreaBottom);
        lineOffset += 20;
    }
    lineOffset += 15;

    // RC Information Section
    scrolledY = t - lineOffset + g_tabState.scrollOffset[3];
    drawSmartText(l + 10, scrolledY, "=== RC INPUT DATA ===", col_white, g_tabState.contentAreaTop, g_tabState.contentAreaBottom);
    lineOffset += 25;

    lineOffset = drawDataRefCategory(l, t, col_white, lineOffset, "RC Information", g_tabState.contentAreaTop, g_tabState.contentAreaBottom, drawnLines);
    lineOffset += 15;

    // Actuator Controls Section (Right column in original, now integrated)
    scrolledY = t - lineOffset + g_tabState.scrollOffset[3];
    drawSmartText(l + 10, scrolledY, "=== ACTUATOR CONTROLS ===", col_white, g_tabState.contentAreaTop, g_tabState.contentAreaBottom);
    lineOffset += 25;

    // Draw actuator controls - need to adapt this to work with clipping
    int adjustedT = t + g_tabState.scrollOffset[3];
    auto& hilActuatorControlsData = MAVLinkManager::hilActuatorControlsData;

    // Timestamp
    char buf[512];
    snprintf(buf, sizeof(buf), "Timestamp: %llu", hilActuatorControlsData.timestamp);
    scrolledY = t - lineOffset + g_tabState.scrollOffset[3];
    drawSmartText(l + 20, scrolledY, buf, col_white, g_tabState.contentAreaTop, g_tabState.contentAreaBottom);
    lineOffset += 20;

    // Controls (show first 8 to keep it manageable)
    for (int i = 0; i < 8; ++i) {
        snprintf(buf, sizeof(buf), "Control %d: %.3f", i, hilActuatorControlsData.controls[i]);
        scrolledY = t - lineOffset + g_tabState.scrollOffset[3];
        drawSmartText(l + 20, scrolledY, buf, col_white, g_tabState.contentAreaTop, g_tabState.contentAreaBottom);
        lineOffset += 20;
    }

    // Mode and Flags
    snprintf(buf, sizeof(buf), "Mode: %u", hilActuatorControlsData.mode);
    scrolledY = t - lineOffset + g_tabState.scrollOffset[3];
    drawSmartText(l + 20, scrolledY, buf, col_white, g_tabState.contentAreaTop, g_tabState.contentAreaBottom);
    lineOffset += 20;

    snprintf(buf, sizeof(buf), "Flags: %llu", hilActuatorControlsData.flags);
    scrolledY = t - lineOffset + g_tabState.scrollOffset[3];
    drawSmartText(l + 20, scrolledY, buf, col_white, g_tabState.contentAreaTop, g_tabState.contentAreaBottom);
    lineOffset += 20;

    return lineOffset;
}

/**
 * @brief Smart DataRef category drawing with content clipping
 */
int drawDataRefCategory(int l, int t, float col_white[], int lineOffset, const char* categoryFilter, int contentTop, int contentBottom, int& drawnLines) {
    for (const auto& item : DataRefManager::dataRefs) {
        // Filter by category
        if (strcmp(item.category, categoryFilter) != 0) {
            continue;
        }

        // Get value string
        std::string valueStr;
        if (item.type == DRT_FLOAT) {
            float value = DataRefManager::getFloat(item.dataRef) * item.multiplier;
            valueStr = std::to_string(value);
        }
        else if (item.type == DRT_DOUBLE) {
            double value = DataRefManager::getDouble(item.dataRef) * item.multiplier;
            valueStr = std::to_string(value);
        }
        else if (item.type == DRT_INT) {
            int value = DataRefManager::getInt(item.dataRef) * static_cast<int>(item.multiplier);
            valueStr = std::to_string(value);
        }
        else if (item.type == DRT_FLOAT_ARRAY) {
            std::vector<float> arrayValues = DataRefManager::getFloatArray(item.dataRef);
            valueStr = DataRefManager::arrayToString(arrayValues);
        }

        // Format and draw line
        char buf[512];
        snprintf(buf, sizeof(buf), "  %s: %s %s", item.title, valueStr.c_str(), item.unit);

        int currentTabIndex = static_cast<int>(g_tabState.currentTab);
        int scrolledY = t - lineOffset + g_tabState.scrollOffset[currentTabIndex];

        if (drawSmartText(l + 20, scrolledY, buf, col_white, contentTop, contentBottom)) {
            drawnLines++;
        }

        lineOffset += 20;
    }

    return lineOffset;
}

/**
 * @brief Main tab content dispatcher
 */
void drawTabContent(int windowLeft, int windowTop, int windowRight, int windowBottom, float col_white[]) {
    calculateTabLayout(windowLeft, windowTop, windowRight, windowBottom);

    int startOffset = 20; // Start offset for content
    int contentHeight = 0;

    // Draw content based on active tab
    switch (g_tabState.currentTab) {
    case ActiveTab::CONNECTION:
        contentHeight = drawConnectionTabContent(windowLeft, g_tabState.contentAreaTop,
            windowRight, g_tabState.contentAreaBottom,
            col_white, startOffset);
        break;

    case ActiveTab::POSITION:
        contentHeight = drawPositionTabContent(windowLeft, g_tabState.contentAreaTop,
            windowRight, g_tabState.contentAreaBottom,
            col_white, startOffset);
        break;

    case ActiveTab::SENSORS:
        contentHeight = drawSensorsTabContent(windowLeft, g_tabState.contentAreaTop,
            windowRight, g_tabState.contentAreaBottom,
            col_white, startOffset);
        break;

    case ActiveTab::CONTROLS:
        contentHeight = drawControlsTabContent(windowLeft, g_tabState.contentAreaTop,
            windowRight, g_tabState.contentAreaBottom,
            col_white, startOffset);
        break;
    }

    // Update content height for current tab
    int currentTabIndex = static_cast<int>(g_tabState.currentTab);
    g_tabState.contentHeight[currentTabIndex] = contentHeight - startOffset;
    updateCurrentTabScroll();
}

// ============================================================================
// ENHANCED MOUSE HANDLING FOR PHASE 2
// ============================================================================

/**
 * @brief Enhanced mouse wheel handler with per-tab scrolling
 */
int handle_main_wheel(XPLMWindowID in_window_id, int x, int y, int wheel, int clicks, void* in_refcon) {
    int currentTabIndex = static_cast<int>(g_tabState.currentTab);

    if (!g_tabState.needsScrollbar[currentTabIndex]) {
        return 0; // No scrolling needed for current tab
    }

    // Calculate scroll delta
    int scrollDelta = -clicks * TabState::SCROLL_SPEED;

    // Apply scroll delta with bounds checking
    int newScrollOffset = g_tabState.scrollOffset[currentTabIndex] + scrollDelta;

    // Clamp to valid range
    if (newScrollOffset < 0) {
        newScrollOffset = 0;
    }
    if (newScrollOffset > g_tabState.maxScrollOffset[currentTabIndex]) {
        newScrollOffset = g_tabState.maxScrollOffset[currentTabIndex];
    }

    // Update scroll position if it changed
    if (newScrollOffset != g_tabState.scrollOffset[currentTabIndex]) {
        g_tabState.scrollOffset[currentTabIndex] = newScrollOffset;

        char debugMsg[128];
        snprintf(debugMsg, sizeof(debugMsg), "Tab %s scroll: %d/%d",
            TAB_CONFIG[currentTabIndex].name,
            g_tabState.scrollOffset[currentTabIndex],
            g_tabState.maxScrollOffset[currentTabIndex]);
        debugLog(debugMsg);

        return 1; // Event handled
    }

    return 0; // No change
}

/**
 * @brief Enhanced mouse click handler with tab switching
 */
int handle_main_mouse(XPLMWindowID in_window_id, int x, int y, int is_down, void* in_refcon) {
    if (!is_down) {
        return 1; // Only handle mouse down events
    }

    // Get window geometry for click detection
    int l, t, r, b;
    XPLMGetWindowGeometry(in_window_id, &l, &t, &r, &b);

    // Check for tab click
    if (handleTabClick(x, y, l, t, r, b)) {
        return 1; // Tab click handled
    }

    // Handle other clicks if needed
    return 1;
}

// ============================================================================
// ENHANCED WINDOW MANAGEMENT (PHASE 2)
// ============================================================================

void create_main_window() {
    if (g_window != NULL) {
        debugLog("Main window already exists");
        return;
    }

    debugLog("Creating Phase 2 enhanced main window with professional tabbed interface...");

    // Get screen dimensions
    int left, bottom, right, top;
    XPLMGetScreenBoundsGlobal(&left, &top, &right, &bottom);

    int screenWidth = right - left;
    int screenHeight = top - bottom;

    // Calculate window size (slightly larger for tabs)
    int windowWidth = std::max<int>(900, (int)(screenWidth * 0.45));  // Wider for tabs
    int windowHeight = (int)(screenHeight * 0.75); // Taller for better content visibility

    // Position window towards left side of screen
    int windowLeft = left + (int)(screenWidth * 0.05);
    int windowBottom = bottom + (int)(screenHeight * 0.1);
    int windowRight = windowLeft + windowWidth;
    int windowTop = windowBottom + windowHeight;

    // Initialize tab state
    g_tabState = TabState();
    calculateTabLayout(windowLeft, windowTop, windowRight, windowBottom);

    XPLMCreateWindow_t params;
    memset(&params, 0, sizeof(params));
    params.structSize = sizeof(params);
    params.visible = 1;
    params.drawWindowFunc = draw_px4xplane;
    params.handleMouseClickFunc = handle_main_mouse;
    params.handleRightClickFunc = NULL;
    params.handleMouseWheelFunc = handle_main_wheel;
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
        debugLog("ERROR: Failed to create Phase 2 enhanced main window!");
        return;
    }

    // Configure window properties
    XPLMSetWindowPositioningMode(g_window, xplm_WindowPositionFree, -1);
    XPLMSetWindowTitle(g_window, "PX4-XPlane Data Inspector v2.5.0 (Phase 2 - Professional)");
    XPLMSetWindowResizingLimits(g_window, 800, 600, windowWidth + 400, windowHeight + 300);

    debugLog("Phase 2 enhanced main window created successfully with professional tabbed interface");
}

// ============================================================================
// MAIN DRAWING FUNCTION (PHASE 2)
// ============================================================================

/**
 * @brief Main drawing function with Phase 2 professional tabbed interface
 */
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

    // Calculate tab layout
    calculateTabLayout(l, t, r, b);

    // Draw professional tabbed interface
    drawTabHeaders(l, t, r, b, col_white);
    drawSearchBox(l, t, r, b, col_white);
    drawTabContent(l, t, r, b, col_white);
    drawEnhancedScrollIndicator(l, t, r, b, col_white);

    // Draw enhanced footer
    char footer[512];
    snprintf(footer, sizeof(footer), "PX4-XPlane v2.5.0 Phase 2 | Professional Tabbed Interface | Tab: %s | github.com/alireza787b/px4xplane",
        TAB_CONFIG[static_cast<int>(g_tabState.currentTab)].name);

    int windowWidth = r - l;
    int textWidth = strlen(footer) * 5;
    int centerX = l + (windowWidth - textWidth) / 2;
    if (centerX < l + 10) centerX = l + 10;

    XPLMDrawString(col_white, centerX, b + 10, footer, NULL, xplmFont_Proportional);
}

// ============================================================================
// ABOUT WINDOW (ENHANCED FOR PHASE 2)
// ============================================================================

void draw_about_window(XPLMWindowID in_window_id, void* in_refcon) {
    if (in_window_id == NULL) return;

    int l, t, r, b;
    XPLMGetWindowGeometry(in_window_id, &l, &t, &r, &b);
    float col_white[] = { 1.0, 1.0, 1.0 };

    if (r <= l || t <= b) {
        debugLog("Invalid about window dimensions detected");
        return;
    }

    int lineOffset = 30;
    char buf[512];

    // Title
    snprintf(buf, sizeof(buf), "PX4-XPlane Interface v2.5.0 - Phase 2 Professional");
    XPLMDrawString(col_white, l + 20, t - lineOffset, buf, NULL, xplmFont_Proportional);
    lineOffset += 40;

    // Description
    snprintf(buf, sizeof(buf), "Hardware-in-the-Loop interface for PX4 SITL and X-Plane 12");
    XPLMDrawString(col_white, l + 20, t - lineOffset, buf, NULL, xplmFont_Proportional);
    lineOffset += 30;

    // Phase 2 Enhancements
    snprintf(buf, sizeof(buf), "Phase 2 Professional Enhancements:");
    XPLMDrawString(col_white, l + 20, t - lineOffset, buf, NULL, xplmFont_Proportional);
    lineOffset += 25;

    snprintf(buf, sizeof(buf), "✓ Professional 4-tab interface with organized data categories");
    XPLMDrawString(col_white, l + 30, t - lineOffset, buf, NULL, xplmFont_Proportional);
    lineOffset += 20;

    snprintf(buf, sizeof(buf), "✓ Fixed text overflow with smart content clipping");
    XPLMDrawString(col_white, l + 30, t - lineOffset, buf, NULL, xplmFont_Proportional);
    lineOffset += 20;

    snprintf(buf, sizeof(buf), "✓ Per-tab independent scrolling with enhanced indicators");
    XPLMDrawString(col_white, l + 30, t - lineOffset, buf, NULL, xplmFont_Proportional);
    lineOffset += 20;

    snprintf(buf, sizeof(buf), "✓ Search framework ready for future text input");
    XPLMDrawString(col_white, l + 30, t - lineOffset, buf, NULL, xplmFont_Proportional);
    lineOffset += 20;

    snprintf(buf, sizeof(buf), "✓ Responsive layout with professional visual design");
    XPLMDrawString(col_white, l + 30, t - lineOffset, buf, NULL, xplmFont_Proportional);
    lineOffset += 30;

    // Tab information
    snprintf(buf, sizeof(buf), "Data Organization Tabs:");
    XPLMDrawString(col_white, l + 20, t - lineOffset, buf, NULL, xplmFont_Proportional);
    lineOffset += 25;

    for (int i = 0; i < 4; i++) {
        snprintf(buf, sizeof(buf), "• %s: %s", TAB_CONFIG[i].name, TAB_CONFIG[i].description);
        XPLMDrawString(col_white, l + 30, t - lineOffset, buf, NULL, xplmFont_Proportional);
        lineOffset += 20;
    }
    lineOffset += 10;

    // Usage instructions
    snprintf(buf, sizeof(buf), "Usage Instructions:");
    XPLMDrawString(col_white, l + 20, t - lineOffset, buf, NULL, xplmFont_Proportional);
    lineOffset += 25;

    snprintf(buf, sizeof(buf), "• Click tab headers to switch between data categories");
    XPLMDrawString(col_white, l + 30, t - lineOffset, buf, NULL, xplmFont_Proportional);
    lineOffset += 20;

    snprintf(buf, sizeof(buf), "• Use mouse wheel to scroll within each tab independently");
    XPLMDrawString(col_white, l + 30, t - lineOffset, buf, NULL, xplmFont_Proportional);
    lineOffset += 20;

    snprintf(buf, sizeof(buf), "• Look for ↕ symbol on tabs that have scrollable content");
    XPLMDrawString(col_white, l + 30, t - lineOffset, buf, NULL, xplmFont_Proportional);
    lineOffset += 30;

    // Copyright
    snprintf(buf, sizeof(buf), "Copyright (c) 2025 Alireza Ghaderi | MIT License");
    XPLMDrawString(col_white, l + 20, t - lineOffset, buf, NULL, xplmFont_Proportional);
    lineOffset += 20;

    snprintf(buf, sizeof(buf), "https://github.com/alireza787b/px4xplane");
    XPLMDrawString(col_white, l + 20, t - lineOffset, buf, NULL, xplmFont_Proportional);
    lineOffset += 30;

    snprintf(buf, sizeof(buf), "Click anywhere to close this dialog");
    XPLMDrawString(col_white, l + 20, t - lineOffset, buf, NULL, xplmFont_Proportional);
}

void create_about_window() {
    if (g_about_window != NULL) {
        debugLog("About window already exists");
        return;
    }

    debugLog("Creating Phase 2 about window...");

    // Get screen dimensions
    int left, bottom, right, top;
    XPLMGetScreenBoundsGlobal(&left, &top, &right, &bottom);

    int screenWidth = right - left;
    int screenHeight = top - bottom;

    // Calculate centered about dialog dimensions (larger for Phase 2 info)
    int dialogWidth = 650;
    int dialogHeight = 550;

    int dialogLeft = left + (screenWidth - dialogWidth) / 2;
    int dialogBottom = bottom + (screenHeight - dialogHeight) / 2;
    int dialogRight = dialogLeft + dialogWidth;
    int dialogTop = dialogBottom + dialogHeight;

    XPLMCreateWindow_t params;
    memset(&params, 0, sizeof(params));
    params.structSize = sizeof(params);
    params.visible = 1;
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
        debugLog("ERROR: Failed to create Phase 2 about window!");
        return;
    }

    XPLMSetWindowPositioningMode(g_about_window, xplm_WindowPositionFree, -1);
    XPLMSetWindowTitle(g_about_window, "About PX4-XPlane Interface v2.5.0 (Phase 2)");

    debugLog("Phase 2 about window created successfully");
}

// Mouse handler for about window (unchanged)
int handle_about_mouse(XPLMWindowID in_window_id, int x, int y, int is_down, void* in_refcon) {
    if (is_down) {
        debugLog("About dialog clicked - closing");
        hide_about_window();
    }
    return 1;
}

// ============================================================================
// UNCHANGED SYSTEM FUNCTIONS (MAINTAINING 100% COMPATIBILITY)
// ============================================================================

// Window management functions
void show_main_window() {
    debugLog("Showing Phase 2 enhanced main window...");

    if (g_window == NULL) {
        create_main_window();
    }

    if (g_window != NULL) {
        XPLMSetWindowIsVisible(g_window, 1);
        XPLMBringWindowToFront(g_window);
        debugLog("Phase 2 enhanced main window should now be visible");
    }
    else {
        debugLog("ERROR: Failed to show Phase 2 enhanced main window - window is NULL");
    }
}

void show_about_window() {
    debugLog("Showing Phase 2 about window...");

    if (g_about_window == NULL) {
        create_about_window();
    }

    if (g_about_window != NULL) {
        XPLMSetWindowIsVisible(g_about_window, 1);
        XPLMBringWindowToFront(g_about_window);
        debugLog("Phase 2 about window should now be visible");
    }
    else {
        debugLog("ERROR: Failed to show Phase 2 about window - window is NULL");
    }
}

void hide_main_window() {
    if (g_window != NULL) {
        XPLMSetWindowIsVisible(g_window, 0);
        debugLog("Phase 2 main window hidden");
    }
}

void hide_about_window() {
    if (g_about_window != NULL) {
        XPLMSetWindowIsVisible(g_about_window, 0);
        debugLog("Phase 2 about window hidden");
    }
}

void destroy_windows() {
    if (g_about_window != NULL) {
        XPLMDestroyWindow(g_about_window);
        g_about_window = NULL;
        debugLog("Phase 2 about window destroyed");
    }

    if (g_window != NULL) {
        XPLMDestroyWindow(g_window);
        g_window = NULL;
        debugLog("Phase 2 main window destroyed");
    }
}

// ============================================================================
// MENU SYSTEM (UNCHANGED - MAINTAINING 100% COMPATIBILITY)
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
        debugLog("Data Inspector menu item selected - showing Phase 2 enhanced main window");
        show_main_window();
    }
    else if (strcmp(item_name, MENU_ITEM_ABOUT) == 0) {
        debugLog("About menu item selected - showing Phase 2 about window");
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

    XPLMClearAllMenuItems(g_airframes_menu);

    std::vector<std::string> airframeNames = ConfigManager::getAirframeLists();
    std::string activeAirframe = ConfigManager::getActiveAirframeName();

    debugLog(("Refreshing airframes menu with " + std::to_string(airframeNames.size()) + " items").c_str());

    for (const std::string& name : airframeNames) {
        std::string menuItemName = name + (name == activeAirframe ? " *" : "");
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

    g_menu_container_idx = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "PX4 X-Plane", NULL, 1);
    g_menu_id = XPLMCreateMenu("PX4 X-Plane", XPLMFindPluginsMenu(), g_menu_container_idx, menu_handler, NULL);

    if (g_menu_id == NULL) {
        debugLog("ERROR: Failed to create main menu!");
        return;
    }

    create_airframes_menu();
    XPLMAppendMenuItem(g_menu_id, MENU_ITEM_DATA_INSPECTOR, (void*)MENU_ITEM_DATA_INSPECTOR, 1);
    XPLMAppendMenuItem(g_menu_id, MENU_ITEM_ABOUT, (void*)MENU_ITEM_ABOUT, 1);
    XPLMAppendMenuSeparator(g_menu_id);

    toggleEnableCmd = XPLMCreateCommand("px4xplane/toggleEnable", "Toggle PX4-XPlane connection");
    if (toggleEnableCmd != NULL) {
        XPLMRegisterCommandHandler(toggleEnableCmd, toggleEnableHandler, 1, (void*)0);
    }

    XPLMAppendMenuItemWithCommand(g_menu_id, "Connect/Disconnect SITL", toggleEnableCmd);

    debugLog("Menu created successfully");
}

// ============================================================================
// FLIGHT LOOP (UNCHANGED - PRESERVING ALL ORIGINAL FUNCTIONALITY)
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
// X-PLANE PLUGIN API (UNCHANGED - PRESERVING ALL ORIGINAL FUNCTIONALITY)
// ============================================================================

PLUGIN_API int XPluginStart(char* outName, char* outSig, char* outDesc) {
    strcpy(outName, "PX4 to X-Plane");
    strcpy(outSig, "alireza787.px4xplane");
    strcpy(outDesc, "PX4 SITL Interface with Professional Tabbed Data Inspector (Phase 2).");

    debugLog("Plugin starting (Phase 2 Professional)...");

    if (XPLMHasFeature("XPLM_USE_NATIVE_WIDGET_WINDOWS")) {
        XPLMEnableFeature("XPLM_USE_NATIVE_WIDGET_WINDOWS", 1);
        debugLog("Enabled modern native widget windows");
    }

    create_menu();

#if IBM
    if (!ConnectionManager::initializeWinSock()) {
        debugLog("ERROR: initializeWinSock failed");
        return 0;
    }
#endif

    XPLMRegisterFlightLoopCallback(MyFlightLoopCallback, -1.0f, NULL);
    debugLog("Plugin started successfully (Phase 2 Professional)");

    return 1;
}

PLUGIN_API void XPluginStop(void) {
    debugLog("Plugin stopping (Phase 2 Professional)...");

    if (ConnectionManager::isConnected()) {
        toggleEnable();
    }

    XPLMUnregisterFlightLoopCallback(MyFlightLoopCallback, NULL);

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

    debugLog("Plugin stopped (Phase 2 Professional)");
}

PLUGIN_API int XPluginEnable(void) {
    debugLog("Plugin enabled (Phase 2 Professional)");
    return 1;
}

PLUGIN_API void XPluginDisable(void) {
    debugLog("Plugin disabled (Phase 2 Professional)");
    if (ConnectionManager::isConnected()) {
        ConnectionManager::disconnect();
    }
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMessage, void* inParam) {
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