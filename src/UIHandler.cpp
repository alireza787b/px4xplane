/**
 * @file UIHandler.cpp
 * @brief Production-Ready Professional UI Implementation for PX4-XPlane Plugin
 *
 * This file implements a highly readable, professional UI system optimized for
 * X-Plane's dark theme. Features:
 * - High-contrast colors for excellent readability
 * - Professional spacing and typography
 * - Clear status indicators and visual feedback
 * - Clickable links and interactive elements with URL opening
 * - Scrollable about dialog with better organization
 * - Enhanced HIL actuator control values display
 * - Real-time sensor data display with actual values
 * - Enhanced mixing tab with detailed configuration mapping and live HIL data
 * - Production-ready error handling and robustness
 *
 * @author Alireza Ghaderi
 * @copyright Copyright (c) 2025 Alireza Ghaderi. All rights reserved.
 * @license MIT License
 * @version 2.5.0 (Production Ready)
 * @url https://github.com/alireza787b/px4xplane
 */

#include "UIHandler.h"
#include "UIConstants.h"
#include "VersionInfo.h"
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMUtilities.h"
#include "ConnectionManager.h"
#include "DataRefManager.h"
#include "ConfigManager.h"
#include "MAVLinkManager.h"
#include <sstream>
#include <algorithm>
#include <cstring>
#include <iomanip>

 // Platform-specific includes for URL opening
#ifdef _WIN32
#include <windows.h>
#include <shellapi.h>
#elif defined(__APPLE__)
#include <CoreFoundation/CFBundle.h>
#include <ApplicationServices/ApplicationServices.h>
#else
#include <cstdlib>
#endif

using namespace UIConstants;
using namespace UIHandler;

// =================================================================
// STATIC MEMBER VARIABLES
// =================================================================

static UIState g_uiState;
static bool g_initialized = false;

// =================================================================
// MAIN UI MANAGEMENT FUNCTIONS
// =================================================================

void UIHandler::initialize() {
    if (g_initialized) {
        return;
    }

    debugLog("Initializing production-ready UI handler system...");

    // Initialize X-Plane integration
    XPlaneColors::initialize();

    // Initialize UI state with production defaults
    g_uiState = UIState();
    g_uiState.uiScale = XPlaneColors::getUIScale();

    g_initialized = true;
    debugLog("Production-ready UI handler system initialized successfully");
}

void UIHandler::cleanup() {
    if (!g_initialized) {
        return;
    }

    debugLog("Cleaning up UI handler system");
    g_initialized = false;
}

void UIHandler::drawMainWindow(XPLMWindowID windowID, void* refcon) {
    if (!g_initialized || windowID == nullptr) {
        return;
    }

    int l, t, r, b;
    XPLMGetWindowGeometry(windowID, &l, &t, &r, &b);

    // Validate window dimensions
    if (r <= l || t <= b) {
        debugLog("Invalid main window dimensions detected");
        return;
    }

    // Update UI scale for high-DPI displays
    g_uiState.uiScale = XPlaneColors::getUIScale();

    // Calculate layout for current window size
    calculateLayout(l, t, r, b);

    // Draw complete professional UI
    Internal::drawBackground(l, t, r, b);
    Internal::drawTabHeaders(l, t, r, b);
    Internal::drawTabContent(l, t, r, b);
    Internal::drawScrollbar(l, t, r, b);
    Internal::drawFooter(l, t, r, b);
}



// =================================================================
// INPUT HANDLING FUNCTIONS
// =================================================================

int UIHandler::handleMainWindowMouse(XPLMWindowID windowID, int x, int y, int isDown, void* refcon) {
    if (!g_initialized || !isDown) {
        return 1; // Only handle mouse down events
    }

    int l, t, r, b;
    XPLMGetWindowGeometry(windowID, &l, &t, &r, &b);

    // Handle tab clicks
    if (Internal::handleTabClick(x, y, l, t, r, b)) {
        return 1; // Tab click handled
    }

    return 1; // Event handled
}

int UIHandler::handleMainWindowWheel(XPLMWindowID windowID, int x, int y, int wheel, int clicks, void* refcon) {
    if (!g_initialized) {
        return 0;
    }

    int currentTabIndex = static_cast<int>(g_uiState.currentTab);

    if (!g_uiState.needsScrollbar[currentTabIndex]) {
        return 0; // No scrolling needed for current tab
    }

    // Calculate scroll delta with improved responsiveness
    int scrollDelta = -clicks * getScaledLayout(Scrolling::SCROLL_SPEED);
    int newScrollOffset = g_uiState.scrollOffset[currentTabIndex] + scrollDelta;

    // Clamp to valid range
    newScrollOffset = clamp(newScrollOffset, 0, g_uiState.maxScrollOffset[currentTabIndex]);

    // Update scroll position if changed
    if (newScrollOffset != g_uiState.scrollOffset[currentTabIndex]) {
        g_uiState.scrollOffset[currentTabIndex] = newScrollOffset;

        char debugMsg[128];
        snprintf(debugMsg, sizeof(debugMsg), "Tab %s scroll: %d/%d",
            Tabs::getTabName(currentTabIndex),
            g_uiState.scrollOffset[currentTabIndex],
            g_uiState.maxScrollOffset[currentTabIndex]);
        debugLog(debugMsg);

        return 1; // Event handled
    }

    return 0; // No change
}

int UIHandler::handleAboutWindowMouse(XPLMWindowID windowID, int x, int y, int isDown, void* refcon) {
    if (!g_initialized) {
        return 0; // Let X-Plane handle it
    }

    // Only handle mouse DOWN events, ignore mouse UP to prevent loops
    if (!isDown) {
        return 0; // Let X-Plane handle mouse up
    }

    // Simple click handling - just close dialog, no infinite loops
    debugLog("About dialog close requested");

    // Return 0 to let main plugin handle the actual window closing
    return 0;
}



// =================================================================
// TAB MANAGEMENT FUNCTIONS
// =================================================================

void UIHandler::switchToTab(Tab newTab) {
    if (g_uiState.currentTab != newTab) {
        g_uiState.currentTab = newTab;
        updateCurrentTabScroll();

        char debugMsg[128];
        snprintf(debugMsg, sizeof(debugMsg), "Switched to tab: %s",
            Tabs::getTabName(static_cast<int>(newTab)));
        debugLog(debugMsg);
    }
}

Tab UIHandler::getCurrentTab() {
    return g_uiState.currentTab;
}

void UIHandler::updateCurrentTabScroll() {
    int tabIndex = static_cast<int>(g_uiState.currentTab);

    g_uiState.needsScrollbar[tabIndex] = g_uiState.contentHeight[tabIndex] > g_uiState.visibleContentHeight;

    if (g_uiState.needsScrollbar[tabIndex]) {
        g_uiState.maxScrollOffset[tabIndex] = g_uiState.contentHeight[tabIndex] - g_uiState.visibleContentHeight;
        g_uiState.scrollOffset[tabIndex] = clamp(g_uiState.scrollOffset[tabIndex], 0, g_uiState.maxScrollOffset[tabIndex]);
    }
    else {
        g_uiState.maxScrollOffset[tabIndex] = 0;
        g_uiState.scrollOffset[tabIndex] = 0;
    }
}

// =================================================================
// MENU INTEGRATION FUNCTIONS
// =================================================================

const char* UIHandler::getConnectionMenuText() {
    return ConnectionManager::isConnected() ?
        "Disconnect from PX4 SITL" : "Connect to PX4 SITL";
}

void UIHandler::updateAirframesMenu() {
    debugLog("Airframes menu update requested - will be handled by main plugin");
}

// =================================================================
// UTILITY AND HELPER FUNCTIONS  
// =================================================================

void UIHandler::calculateLayout(int windowLeft, int windowTop, int windowRight, int windowBottom) {
    // Calculate tab area with better spacing
    g_uiState.tabAreaHeight = getScaledLayout(Layout::TAB_HEIGHT + Layout::SEARCH_BOX_HEIGHT);
    g_uiState.contentAreaTop = windowTop - g_uiState.tabAreaHeight - getScaledLayout(Layout::CONTENT_MARGIN);
    g_uiState.contentAreaBottom = windowBottom + getScaledLayout(Layout::FOOTER_HEIGHT + Layout::CONTENT_MARGIN);
    g_uiState.visibleContentHeight = g_uiState.contentAreaTop - g_uiState.contentAreaBottom;

    // Ensure minimum content height for readability
    int minHeight = getScaledLayout(Layout::MIN_VISIBLE_LINES * Layout::LINE_HEIGHT);
    if (g_uiState.visibleContentHeight < minHeight) {
        g_uiState.visibleContentHeight = minHeight;
        g_uiState.contentAreaBottom = g_uiState.contentAreaTop - g_uiState.visibleContentHeight;
    }
}

bool UIHandler::isLineVisible(int yPosition, int contentTop, int contentBottom) {
    return (yPosition <= contentTop && yPosition >= contentBottom);
}

int UIHandler::drawSmartText(int x, int y, const char* text, float color[3], int contentTop, int contentBottom) {
    if (isLineVisible(y, contentTop, contentBottom)) {
        XPLMDrawString(color, x, y, (char*)text, nullptr, Fonts::CONTENT);
        return 1; // Line was drawn
    }
    return 0; // Line was clipped
}

bool UIHandler::openURL(const char* url) {
    // DISABLED: URL opening functionality to prevent X-Plane stability issues
    debugLog("URL opening is disabled for stability - copy URL manually from about dialog");
    return false;
}
void UIHandler::debugLog(const char* message) {
    XPLMDebugString("px4xplane UI: ");
    XPLMDebugString(message);
    XPLMDebugString("\n");
}

// =================================================================
// INTERNAL DRAWING FUNCTIONS IMPLEMENTATION
// =================================================================

void UIHandler::Internal::drawBackground(int windowLeft, int windowTop, int windowRight, int windowBottom) {
    // Draw subtle background using X-Plane's translucent dark box
    XPLMDrawTranslucentDarkBox(windowLeft, windowTop, windowRight, windowBottom);
}

void UIHandler::Internal::drawTabHeaders(int windowLeft, int windowTop, int windowRight, int windowBottom) {
    int windowWidth = windowRight - windowLeft;
    int tabWidth = windowWidth / Tabs::TAB_COUNT;
    int tabTop = windowTop - getScaledSize(8);
    int tabBottom = windowTop - getScaledLayout(Layout::TAB_HEIGHT) - getScaledSize(8);
    int currentTabIndex = static_cast<int>(g_uiState.currentTab);

    // Get high-contrast colors
    float activeColor[3], inactiveColor[3], borderColor[3];
    memcpy(activeColor, Colors::TAB_ACTIVE, sizeof(activeColor));
    memcpy(inactiveColor, Colors::TAB_INACTIVE, sizeof(inactiveColor));
    memcpy(borderColor, Colors::TAB_BORDER, sizeof(borderColor));

    // Draw professional tab headers with better spacing
    for (int i = 0; i < Tabs::TAB_COUNT; i++) {
        int tabLeft = windowLeft + (i * tabWidth);
        int tabRight = tabLeft + tabWidth;
        bool isActive = (i == currentTabIndex);

        // Draw tab border with improved visibility
        float* currentBorderColor = isActive ? borderColor : inactiveColor;

        // Draw solid line borders - using ASCII characters only
        for (int x = tabLeft + 2; x < tabRight - 2; x += 6) {
            XPLMDrawString(currentBorderColor, x, tabTop, (char*)"_", nullptr, Fonts::PRIMARY);
            XPLMDrawString(currentBorderColor, x, tabBottom, (char*)"_", nullptr, Fonts::PRIMARY);
        }

        // Side borders - using ASCII pipe character
        for (int y = tabBottom + 4; y < tabTop - 4; y += 6) {
            XPLMDrawString(currentBorderColor, tabLeft + 2, y, (char*)"|", nullptr, Fonts::PRIMARY);
            XPLMDrawString(currentBorderColor, tabRight - 8, y, (char*)"|", nullptr, Fonts::PRIMARY);
        }

        // Draw tab label with better positioning
        float* textColor = isActive ? activeColor : inactiveColor;
        const char* tabName = Tabs::getTabName(i);
        int textX = centerTextX(tabLeft, tabRight, tabName, 7);
        int textY = (tabTop + tabBottom) / 2 - getScaledSize(6);
        XPLMDrawString(textColor, textX, textY, (char*)tabName, nullptr, Fonts::PRIMARY);
    }
}

void UIHandler::Internal::drawSearchBox(int windowLeft, int windowTop, int windowRight, int windowBottom) {
    // Calculate search box position
    int searchTop = windowTop - getScaledLayout(Layout::TAB_HEIGHT) - getScaledSize(15);
    int searchBottom = searchTop - getScaledLayout(Layout::SEARCH_BOX_HEIGHT);
    int searchLeft = windowLeft + getScaledSize(20);
    int searchRight = windowRight - getScaledSize(20);

    float searchColor[3];
    memcpy(searchColor, Colors::SEARCH_BOX, sizeof(searchColor));

    // Draw search box outline using ASCII characters
    char searchPrompt[64];
    snprintf(searchPrompt, sizeof(searchPrompt), "%s Search: [Future Feature]", Status::SEARCH_ICON);
    int textY = (searchTop + searchBottom) / 2 - getScaledSize(6);
    XPLMDrawString(searchColor, searchLeft + 10, textY, searchPrompt, nullptr, Fonts::CONTENT);
}

void UIHandler::Internal::drawTabContent(int windowLeft, int windowTop, int windowRight, int windowBottom) {
    // Calculate content start offset
    int startOffset = getScaledLayout(Layout::CONTENT_MARGIN);

    // Draw tab-specific content
    int contentHeight = startOffset;
    switch (g_uiState.currentTab) {
    case Tab::CONNECTION:
        contentHeight = drawConnectionTabContent(windowLeft, g_uiState.contentAreaTop, windowRight, g_uiState.contentAreaBottom, startOffset);
        break;
    case Tab::POSITION:
        contentHeight = drawPositionTabContent(windowLeft, g_uiState.contentAreaTop, windowRight, g_uiState.contentAreaBottom, startOffset);
        break;
    case Tab::SENSORS:
        contentHeight = drawSensorsTabContent(windowLeft, g_uiState.contentAreaTop, windowRight, g_uiState.contentAreaBottom, startOffset);
        break;
    case Tab::CONTROLS:
        contentHeight = drawControlsTabContent(windowLeft, g_uiState.contentAreaTop, windowRight, g_uiState.contentAreaBottom, startOffset);
        break;
    case Tab::MIXING:
        contentHeight = drawMixingTabContent(windowLeft, g_uiState.contentAreaTop, windowRight, g_uiState.contentAreaBottom, startOffset);
        break;
    }

    // Update content height for current tab
    int currentTabIndex = static_cast<int>(g_uiState.currentTab);
    g_uiState.contentHeight[currentTabIndex] = contentHeight - startOffset;
    updateCurrentTabScroll();
}

void UIHandler::Internal::drawScrollbar(int windowLeft, int windowTop, int windowRight, int windowBottom) {
    int currentTabIndex = static_cast<int>(g_uiState.currentTab);

    if (!g_uiState.needsScrollbar[currentTabIndex]) return;

    // Calculate scrollbar dimensions with better sizing
    int scrollbarWidth = getScaledLayout(Layout::SCROLLBAR_WIDTH);
    int scrollbarLeft = windowRight - scrollbarWidth - getScaledSize(10);
    int scrollbarTop = g_uiState.contentAreaTop - getScaledSize(8);
    int scrollbarBottom = g_uiState.contentAreaBottom + getScaledSize(8);
    int scrollbarHeight = scrollbarTop - scrollbarBottom;

    if (scrollbarHeight <= 30) return;

    // Calculate thumb position and size
    float scrollRatio = (g_uiState.maxScrollOffset[currentTabIndex] > 0) ?
        (float)g_uiState.scrollOffset[currentTabIndex] / (float)g_uiState.maxScrollOffset[currentTabIndex] : 0.0f;
    float thumbRatio = (g_uiState.contentHeight[currentTabIndex] > 0) ?
        (float)g_uiState.visibleContentHeight / (float)g_uiState.contentHeight[currentTabIndex] : 1.0f;

    int thumbHeight = static_cast<int>(scrollbarHeight * thumbRatio);
    thumbHeight = std::max<int>(thumbHeight, getScaledLayout(Scrolling::MIN_THUMB_SIZE));

    int thumbTop = scrollbarTop - static_cast<int>(scrollRatio * (scrollbarHeight - thumbHeight));
    int thumbBottom = thumbTop - thumbHeight;

    // Draw enhanced scrollbar with better colors
    float trackColor[3], thumbColor[3];
    memcpy(trackColor, Colors::SCROLLBAR_TRACK, sizeof(trackColor));
    memcpy(thumbColor, Colors::SCROLLBAR_THUMB, sizeof(thumbColor));

    // Draw track using ASCII characters
    for (int i = scrollbarBottom; i < scrollbarTop; i += 4) {
        XPLMDrawString(trackColor, scrollbarLeft + 4, i, (char*)"|", nullptr, Fonts::PRIMARY);
    }

    // Draw thumb with better visibility using ASCII characters
    for (int i = thumbBottom; i < thumbTop; i += 3) {
        XPLMDrawString(thumbColor, scrollbarLeft + 2, i, (char*)"##", nullptr, Fonts::PRIMARY);
    }
}

void UIHandler::Internal::drawFooter(int windowLeft, int windowTop, int windowRight, int windowBottom) {
    // Professional footer with clickable links
    float footerColor[3], linkColor[3];
    memcpy(footerColor, Colors::FOOTER_TEXT, sizeof(footerColor));
    memcpy(linkColor, Colors::LINK_TEXT, sizeof(linkColor));

    // Create comprehensive footer with links
    char footer[512];
    snprintf(footer, sizeof(footer), "PX4-XPlane v%s | Tab: %s | Ready for Production",
        PX4XPlaneVersion::VERSION,
        Tabs::getTabName(static_cast<int>(g_uiState.currentTab)));

    int windowWidth = windowRight - windowLeft;
    int textX = centerTextX(windowLeft, windowRight, footer, 6);
    textX = std::max<int>(textX, windowLeft + 15); // Prevent overflow

    // Main footer text
    XPLMDrawString(footerColor, textX, windowBottom + 30, footer, nullptr, Fonts::FOOTER);

    // GitHub link
    char linkText[256];
    snprintf(linkText, sizeof(linkText), "%s %s %s Documentation & Updates",
        Status::LINK_ICON, PX4XPlaneVersion::REPOSITORY_URL, Status::LINK_ICON);
    int linkX = centerTextX(windowLeft, windowRight, linkText, 5);
    linkX = std::max<int>(linkX, windowLeft + 15);
    XPLMDrawString(linkColor, linkX, windowBottom + 12, linkText, nullptr, Fonts::FOOTER);

    // Copyright
    char copyright[256];
    snprintf(copyright, sizeof(copyright), "%s", PX4XPlaneVersion::getCopyrightString());
    int copyrightX = centerTextX(windowLeft, windowRight, copyright, 5);
    copyrightX = std::max<int>(copyrightX, windowLeft + 15);
    XPLMDrawString(footerColor, copyrightX, windowBottom + 3, copyright, nullptr, Fonts::FOOTER);
}

void UIHandler::drawAboutWindow(XPLMWindowID windowID, void* refcon) {
    if (!g_initialized || windowID == nullptr) {
        return;
    }

    int l, t, r, b;
    XPLMGetWindowGeometry(windowID, &l, &t, &r, &b);

    if (r <= l || t <= b) {
        debugLog("Invalid about window dimensions detected");
        return;
    }

    // Draw simple about dialog - no scrolling needed
    XPLMDrawTranslucentDarkBox(l, t, r, b);
    Internal::drawAboutContent(l, t, r, b);
}




void UIHandler::Internal::drawAboutContent(int windowLeft, int windowTop, int windowRight, int windowBottom) {
    // Simple, concise about dialog - no scrolling needed
    float titleColor[3], contentColor[3], linkColor[3];
    memcpy(titleColor, Colors::TAB_ACTIVE, sizeof(titleColor));
    memcpy(contentColor, Colors::CONTENT_TEXT, sizeof(contentColor));
    memcpy(linkColor, Colors::LINK_TEXT, sizeof(linkColor));

    int lineOffset = getScaledSize(40);
    char buf[512];

    // Title
    snprintf(buf, sizeof(buf), "PX4-XPlane Plugin v%s", PX4XPlaneVersion::VERSION);
    XPLMDrawString(titleColor, windowLeft + 25, windowTop - lineOffset, buf, nullptr, Fonts::HEADER);
    lineOffset += getScaledSize(45);

    // Brief description
    XPLMDrawString(contentColor, windowLeft + 25, windowTop - lineOffset, "Professional SITL Bridge for X-Plane & PX4 Autopilot", nullptr, Fonts::CONTENT);
    lineOffset += getScaledSize(35);

    // Key features - condensed
    const char* features[] = {
        "• Real-time MAVLink communication with PX4 SITL",
        "• Live HIL actuator control values display",
        "• Configurable airframe mixing & channel mapping",
        "• Professional 5-tab data monitoring interface"
    };

    for (const auto& feature : features) {
        XPLMDrawString(contentColor, windowLeft + 35, windowTop - lineOffset, (char*)feature, nullptr, Fonts::CONTENT);
        lineOffset += getScaledSize(22);
    }
    lineOffset += getScaledSize(25);

    // Quick usage
    XPLMDrawString(contentColor, windowLeft + 25, windowTop - lineOffset, "Quick Start: Connect to PX4 SITL via menu, select airframe, monitor data", nullptr, Fonts::CONTENT);
    lineOffset += getScaledSize(30);

    // Links - display only (not clickable)
    snprintf(buf, sizeof(buf), "GitHub: %s", PX4XPlaneVersion::REPOSITORY_URL);
    XPLMDrawString(linkColor, windowLeft + 25, windowTop - lineOffset, buf, nullptr, Fonts::CONTENT);
    lineOffset += getScaledSize(25);

    // Copyright
    snprintf(buf, sizeof(buf), "%s", PX4XPlaneVersion::getCopyrightString());
    XPLMDrawString(contentColor, windowLeft + 25, windowTop - lineOffset, buf, nullptr, Fonts::FOOTER);
    lineOffset += getScaledSize(35);

}

// =================================================================
// TAB-SPECIFIC CONTENT DRAWING FUNCTIONS - WITH ENHANCED HIL DATA
// =================================================================

int UIHandler::Internal::drawConnectionTabContent(int left, int top, int right, int bottom, int startOffset) {
    int lineOffset = startOffset;
    char buf[512];
    float headerColor[3], contentColor[3], statusColor[3];

    memcpy(headerColor, Colors::HEADER_TEXT, sizeof(headerColor));
    memcpy(contentColor, Colors::CONTENT_TEXT, sizeof(contentColor));

    int currentTabIndex = static_cast<int>(g_uiState.currentTab);

    // Enhanced Connection Status Section
    int scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 20, scrolledY, "CONNECTION STATUS", headerColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::HEADER_SPACING);

    // Clear connection status with improved visibility
    bool isConnected = ConnectionManager::isConnected();
    const std::string& status = ConnectionManager::getStatus();

    if (isConnected) {
        memcpy(statusColor, Colors::CONNECTED, sizeof(statusColor));
        snprintf(buf, sizeof(buf), "%s %s", Status::CONNECTED_ICON, Status::CONNECTED_TEXT);
    }
    else {
        memcpy(statusColor, Colors::DISCONNECTED, sizeof(statusColor));
        snprintf(buf, sizeof(buf), "%s %s", Status::DISCONNECTED_ICON, Status::DISCONNECTED_TEXT);
    }

    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 30, scrolledY, buf, statusColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

    // Additional status details
    snprintf(buf, sizeof(buf), "Details: %s", status.c_str());
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 30, scrolledY, buf, contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

    // Enhanced system information
    snprintf(buf, sizeof(buf), "SITL Timestep: %.3f ms (%.1f Hz)",
        DataRefManager::SIM_Timestep * 1000.0f, 1.0f / DataRefManager::SIM_Timestep);
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 30, scrolledY, buf, contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::SECTION_SPACING);

    // Time Information Section with REAL DATA
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 20, scrolledY, "TIME INFORMATION", headerColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::HEADER_SPACING);

    // Flight time
    float flightTime = DataRefManager::getFloat("sim/time/total_flight_time_sec");
    snprintf(buf, sizeof(buf), "  Flight Time         : %8.1f s", flightTime);
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 35, scrolledY, buf, contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

    // Real time
    float realTime = DataRefManager::getFloat("sim/time/total_running_time_sec");
    snprintf(buf, sizeof(buf), "  Real Time           : %8.1f s", realTime);
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 35, scrolledY, buf, contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

    // Frame rate
    float frameRate = DataRefManager::getFloat("sim/operation/misc/frame_rate_period");
    if (frameRate > 0) frameRate = 1.0f / frameRate;
    snprintf(buf, sizeof(buf), "  Frame Rate          : %8.1f fps", frameRate);
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 35, scrolledY, buf, contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

    return lineOffset;
}

int UIHandler::Internal::drawPositionTabContent(int left, int top, int right, int bottom, int startOffset) {
    int lineOffset = startOffset;
    char buf[512];
    float headerColor[3], contentColor[3];

    memcpy(headerColor, Colors::HEADER_TEXT, sizeof(headerColor));
    memcpy(contentColor, Colors::VALUE_TEXT, sizeof(contentColor));

    int currentTabIndex = static_cast<int>(g_uiState.currentTab);

    // Position Section with REAL DATA
    int scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 20, scrolledY, "POSITION DATA", headerColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::HEADER_SPACING);

    // Latitude
    double latitude = DataRefManager::getDouble("sim/flightmodel/position/latitude");
    snprintf(buf, sizeof(buf), "  Latitude            : %12.7f deg", latitude);
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 35, scrolledY, buf, contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

    // Longitude
    double longitude = DataRefManager::getDouble("sim/flightmodel/position/longitude");
    snprintf(buf, sizeof(buf), "  Longitude           : %12.7f deg", longitude);
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 35, scrolledY, buf, contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

    // Elevation
    float elevation = DataRefManager::getFloat("sim/flightmodel/position/elevation");
    snprintf(buf, sizeof(buf), "  Elevation           : %10.1f m", elevation);
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 35, scrolledY, buf, contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

    lineOffset += getScaledLayout(Layout::SECTION_SPACING);

    // Attitude Section with REAL DATA
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 20, scrolledY, "ATTITUDE DATA", headerColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::HEADER_SPACING);

    // Roll (phi)
    float roll = DataRefManager::getFloat("sim/flightmodel/position/phi");
    snprintf(buf, sizeof(buf), "  Roll (phi)          : %10.2f deg", roll);
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 35, scrolledY, buf, contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

    // Pitch (theta)
    float pitch = DataRefManager::getFloat("sim/flightmodel/position/theta");
    snprintf(buf, sizeof(buf), "  Pitch (theta)       : %10.2f deg", pitch);
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 35, scrolledY, buf, contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

    // Yaw (psi)
    float yaw = DataRefManager::getFloat("sim/flightmodel/position/psi");
    snprintf(buf, sizeof(buf), "  Yaw (psi)           : %10.2f deg", yaw);
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 35, scrolledY, buf, contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

    lineOffset += getScaledLayout(Layout::SECTION_SPACING);

    // Angular Velocity Section with REAL DATA
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 20, scrolledY, "ANGULAR VELOCITIES", headerColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::HEADER_SPACING);

    // Roll rate
    float rollRate = DataRefManager::getFloat("sim/flightmodel/position/Prad");
    snprintf(buf, sizeof(buf), "  Roll Rate (P)       : %10.3f rad/s", rollRate);
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 35, scrolledY, buf, contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

    // Pitch rate
    float pitchRate = DataRefManager::getFloat("sim/flightmodel/position/Qrad");
    snprintf(buf, sizeof(buf), "  Pitch Rate (Q)      : %10.3f rad/s", pitchRate);
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 35, scrolledY, buf, contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

    // Yaw rate
    float yawRate = DataRefManager::getFloat("sim/flightmodel/position/Rrad");
    snprintf(buf, sizeof(buf), "  Yaw Rate (R)        : %10.3f rad/s", yawRate);
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 35, scrolledY, buf, contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

    return lineOffset;
}

int UIHandler::Internal::drawSensorsTabContent(int left, int top, int right, int bottom, int startOffset) {
    int lineOffset = startOffset;
    char buf[512];
    float headerColor[3], contentColor[3];

    memcpy(headerColor, Colors::HEADER_TEXT, sizeof(headerColor));
    memcpy(contentColor, Colors::VALUE_TEXT, sizeof(contentColor));

    int currentTabIndex = static_cast<int>(g_uiState.currentTab);

    // Acceleration Section with REAL DATA
    int scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 20, scrolledY, "ACCELERATION (BODY FRAME)", headerColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::HEADER_SPACING);

    // X acceleration
    float xacc = DataRefManager::getFloat("sim/flightmodel/forces/g_axil") * DataRefManager::g_earth;
    snprintf(buf, sizeof(buf), "  X Acceleration      : %10.2f m/s²", xacc);
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 35, scrolledY, buf, contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

    // Y acceleration
    float yacc = DataRefManager::getFloat("sim/flightmodel/forces/g_side") * DataRefManager::g_earth;
    snprintf(buf, sizeof(buf), "  Y Acceleration      : %10.2f m/s²", yacc);
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 35, scrolledY, buf, contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

    // Z acceleration
    float zacc = DataRefManager::getFloat("sim/flightmodel/forces/g_nrml") * DataRefManager::g_earth;
    snprintf(buf, sizeof(buf), "  Z Acceleration      : %10.2f m/s²", zacc);
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 35, scrolledY, buf, contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

    lineOffset += getScaledLayout(Layout::SECTION_SPACING);

    // Airspeed Section with REAL DATA
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 20, scrolledY, "AIRSPEED DATA", headerColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::HEADER_SPACING);

    // Indicated airspeed
    float ias = DataRefManager::getFloat("sim/flightmodel/position/indicated_airspeed");
    snprintf(buf, sizeof(buf), "  Indicated Airspeed  : %10.1f kts", ias);
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 35, scrolledY, buf, contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

    // True airspeed
    float tas = DataRefManager::getFloat("sim/flightmodel/position/true_airspeed");
    snprintf(buf, sizeof(buf), "  True Airspeed       : %10.1f kts", tas);
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 35, scrolledY, buf, contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

    // Ground speed
    float groundSpeed = DataRefManager::getFloat("sim/flightmodel/position/groundspeed");
    snprintf(buf, sizeof(buf), "  Ground Speed        : %10.1f kts", groundSpeed);
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 35, scrolledY, buf, contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

    lineOffset += getScaledLayout(Layout::SECTION_SPACING);

    // Environmental Sensors Section with REAL DATA
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 20, scrolledY, "ENVIRONMENTAL SENSORS", headerColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::HEADER_SPACING);

    // Barometric pressure
    constexpr float FEET_TO_METERS = 0.3048f;
    float raw_pressure_altitude_ft = DataRefManager::getFloat("sim/flightmodel2/position/pressure_altitude");
    float raw_pressure_altitude_m = raw_pressure_altitude_ft * FEET_TO_METERS;
    // Calculate raw barometric pressure from altitude using ISA model
    float raw_pressure_hPa = DataRefManager::calculatePressureFromAltitude(raw_pressure_altitude_m);
    snprintf(buf, sizeof(buf), "  Barometric Pressure : %.1f hPa", raw_pressure_hPa);
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 35, scrolledY, buf, contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

    // Outside temperature
    float oat = DataRefManager::getFloat("sim/cockpit2/temperature/outside_air_temp_degc");
    snprintf(buf, sizeof(buf), "  Outside Air Temp    : %10.1f °C", oat);
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 35, scrolledY, buf, contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

    return lineOffset;
}

int UIHandler::Internal::drawControlsTabContent(int left, int top, int right, int bottom, int startOffset) {
    int lineOffset = startOffset;
    char buf[512];
    float headerColor[3], contentColor[3], airframeColor[3], valueColor[3];

    memcpy(headerColor, Colors::HEADER_TEXT, sizeof(headerColor));
    memcpy(contentColor, Colors::CONTENT_TEXT, sizeof(contentColor));
    memcpy(airframeColor, Colors::AIRFRAME_TEXT, sizeof(airframeColor));
    memcpy(valueColor, Colors::VALUE_TEXT, sizeof(valueColor));

    int currentTabIndex = static_cast<int>(g_uiState.currentTab);

    // Aircraft Configuration Section with REAL DATA
    int scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 20, scrolledY, "AIRCRAFT CONFIGURATION", headerColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::HEADER_SPACING);

    // Get and display formatted config with better formatting
    std::string droneConfigStr = DataRefManager::GetFormattedDroneConfig();
    std::istringstream iss(droneConfigStr);
    std::string line;

    while (std::getline(iss, line)) {
        scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
        char lineBuffer[512];
        strncpy(lineBuffer, line.c_str(), sizeof(lineBuffer) - 1);
        lineBuffer[sizeof(lineBuffer) - 1] = '\0';
        drawSmartText(left + 35, scrolledY, lineBuffer, airframeColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
        lineOffset += getScaledLayout(Layout::LINE_HEIGHT);
    }

    lineOffset += getScaledLayout(Layout::SECTION_SPACING);

    // ENHANCED HIL Actuator Controls Section with ALL VALUES
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 20, scrolledY, "HIL ACTUATOR CONTROLS (LIVE DATA)", headerColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::HEADER_SPACING);

    // HIL message metadata
    snprintf(buf, sizeof(buf), "Timestamp: %llu us", (unsigned long long)MAVLinkManager::hilActuatorControlsData.timestamp);
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 35, scrolledY, buf, contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

    snprintf(buf, sizeof(buf), "Mode: %d | Flags: %llu", MAVLinkManager::hilActuatorControlsData.mode,
        (unsigned long long)MAVLinkManager::hilActuatorControlsData.flags);
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 35, scrolledY, buf, contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

    lineOffset += getScaledLayout(Layout::LINE_HEIGHT / 2);

    // Display all 16 HIL actuator control values in organized columns
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 35, scrolledY, "Actuator Channel Values (Range: -1.0 to 1.0):", contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

    // Display in 2 columns for better space utilization
    for (int i = 0; i < 8; i++) {
        // Left column (channels 0-7)
        float leftValue = MAVLinkManager::hilActuatorControlsData.controls[i];
        float rightValue = MAVLinkManager::hilActuatorControlsData.controls[i + 8];

        snprintf(buf, sizeof(buf), "  Ch%02d: %+6.3f     Ch%02d: %+6.3f",
            i, leftValue, i + 8, rightValue);

        scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
        drawSmartText(left + 45, scrolledY, buf, valueColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
        lineOffset += getScaledLayout(Layout::LINE_HEIGHT);
    }

    lineOffset += getScaledLayout(Layout::SECTION_SPACING);

    // RC Input Status Section (if applicable)
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 20, scrolledY, "RC INPUT STATUS", headerColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::HEADER_SPACING);

    // Show connection status and basic RC info
    if (ConnectionManager::isConnected()) {
        snprintf(buf, sizeof(buf), "Status: %s HIL Mode Active - RC via MAVLink", Status::CONNECTED_ICON);
        scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
        drawSmartText(left + 35, scrolledY, buf, (float*)Colors::CONNECTED, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    }
    else {
        snprintf(buf, sizeof(buf), "Status: %s No SITL Connection", Status::DISCONNECTED_ICON);
        scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
        drawSmartText(left + 35, scrolledY, buf, (float*)Colors::DISCONNECTED, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    }
    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

    return lineOffset;
}


int UIHandler::Internal::drawMixingTabContent(int left, int top, int right, int bottom, int startOffset) {
    int lineOffset = startOffset;
    char buf[512];
    float headerColor[3], contentColor[3], mixingColor[3], linkColor[3], channelColor[3], valueColor[3];

    memcpy(headerColor, Colors::HEADER_TEXT, sizeof(headerColor));
    memcpy(contentColor, Colors::CONTENT_TEXT, sizeof(contentColor));
    memcpy(mixingColor, Colors::MIXING_CONTENT, sizeof(mixingColor));
    memcpy(linkColor, Colors::LINK_TEXT, sizeof(linkColor));
    memcpy(channelColor, Colors::AIRFRAME_TEXT, sizeof(channelColor));
    memcpy(valueColor, Colors::VALUE_TEXT, sizeof(valueColor));

    int currentTabIndex = static_cast<int>(g_uiState.currentTab);
    bool isConnected = ConnectionManager::isConnected();

    // ALWAYS SHOW: Enhanced Airframe Configuration Header
    int scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    const char* headerText = isConnected ? "AIRFRAME CONFIG MAPPING WITH LIVE HIL DATA" : "AIRFRAME CONFIGURATION MAPPING";
    drawSmartText(left + 20, scrolledY, headerText, headerColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::HEADER_SPACING);

    // ALWAYS SHOW: Active airframe display
    std::string activeAirframe = ConfigManager::getActiveAirframeName();
    if (activeAirframe.empty()) {
        snprintf(buf, sizeof(buf), "Active Airframe: %s NOT CONFIGURED %s", Status::WARNING_ICON, Status::WARNING_ICON);
        scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
        drawSmartText(left + 35, scrolledY, buf, (float*)Colors::WARNING, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    }
    else {
        snprintf(buf, sizeof(buf), "Active Airframe: %s", activeAirframe.c_str());
        scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
        drawSmartText(left + 35, scrolledY, buf, mixingColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    }
    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

    // ALWAYS SHOW: Configuration file location
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 35, scrolledY, "Config File: X-Plane/Resources/plugins/px4xplane/config.ini", linkColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

    // ALWAYS SHOW: Connection status for HIL data
    if (isConnected) {
        snprintf(buf, sizeof(buf), "SITL Status: %s CONNECTED - Live HIL Values Available", Status::CONNECTED_ICON);
        scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
        drawSmartText(left + 35, scrolledY, buf, (float*)Colors::CONNECTED, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    }
    else {
        snprintf(buf, sizeof(buf), "SITL Status: %s NOT CONNECTED - Static Configuration Only", Status::DISCONNECTED_ICON);
        scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
        drawSmartText(left + 35, scrolledY, buf, (float*)Colors::DISCONNECTED, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    }
    lineOffset += getScaledLayout(Layout::SECTION_SPACING);

    // ALWAYS SHOW: Channel Configuration Display (regardless of connection)
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 20, scrolledY, "CHANNEL CONFIGURATIONS:", headerColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::HEADER_SPACING);

    if (!ConfigManager::actuatorConfigs.empty()) {
        // ALWAYS display channel configurations from ConfigManager
        for (const auto& [channelNum, config] : ConfigManager::actuatorConfigs) {
            // Channel header - show HIL value only if connected
            if (isConnected && channelNum < 16) {
                float hilValue = MAVLinkManager::hilActuatorControlsData.controls[channelNum];
                snprintf(buf, sizeof(buf), "Channel %d: [HIL: %+6.3f]", channelNum, hilValue);
                scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
                drawSmartText(left + 35, scrolledY, buf, valueColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
            }
            else {
                snprintf(buf, sizeof(buf), "Channel %d: [Configuration]", channelNum);
                scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
                drawSmartText(left + 35, scrolledY, buf, channelColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
            }
            lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

            // ALWAYS display each dataref in this channel
            for (const auto& datarefConfig : config.getDatarefConfigs()) {
                // Dataref name - ALWAYS show
                snprintf(buf, sizeof(buf), "  -> %s", datarefConfig.datarefName.c_str());
                scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
                drawSmartText(left + 50, scrolledY, buf, mixingColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
                lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

                // Show current dataref value if possible (regardless of SITL connection)
                try {
                    float currentValue = DataRefManager::getFloat(datarefConfig.datarefName.c_str());
                    snprintf(buf, sizeof(buf), "     Current: %.3f | Range: [%.2f, %.2f]",
                        currentValue, datarefConfig.range.first, datarefConfig.range.second);
                    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
                    drawSmartText(left + 55, scrolledY, buf, contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
                    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);
                }
                catch (...) {
                    // Show range and type even if value read fails
                    const char* typeStr = (datarefConfig.dataType == FLOAT_ARRAY) ? "Float Array" : "Float";
                    snprintf(buf, sizeof(buf), "     Range: [%.2f, %.2f] | Type: %s",
                        datarefConfig.range.first, datarefConfig.range.second, typeStr);
                    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
                    drawSmartText(left + 55, scrolledY, buf, contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
                    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);
                }

                // Array indices if applicable - ALWAYS show
                if (datarefConfig.dataType == FLOAT_ARRAY && !datarefConfig.arrayIndices.empty()) {
                    std::string indicesStr = "     Indices: [";
                    for (size_t i = 0; i < datarefConfig.arrayIndices.size(); ++i) {
                        if (i > 0) indicesStr += ", ";
                        indicesStr += std::to_string(datarefConfig.arrayIndices[i]);
                    }
                    indicesStr += "]";

                    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
                    drawSmartText(left + 55, scrolledY, indicesStr.c_str(), contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
                    lineOffset += getScaledLayout(Layout::LINE_HEIGHT);
                }

                lineOffset += getScaledLayout(Layout::LINE_HEIGHT / 2); // Small gap between datarefs
            }

            lineOffset += getScaledLayout(Layout::LINE_HEIGHT); // Gap between channels
        }
    }
    else {
        // ALWAYS show this if no configurations found
        scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
        snprintf(buf, sizeof(buf), "%s No channel configurations found.", Status::WARNING_ICON);
        drawSmartText(left + 35, scrolledY, buf, (float*)Colors::WARNING, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
        lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

        scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
        drawSmartText(left + 35, scrolledY, "Check config.ini file and restart X-Plane.", contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
        lineOffset += getScaledLayout(Layout::LINE_HEIGHT);
    }

    lineOffset += getScaledLayout(Layout::SECTION_SPACING);

    // Show HIL Summary only if connected
    if (isConnected) {
        scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
        drawSmartText(left + 20, scrolledY, "LIVE HIL ACTUATOR SUMMARY:", headerColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
        lineOffset += getScaledLayout(Layout::HEADER_SPACING);

        // Show active channels (non-zero HIL values)
        int activeChannels = 0;
        for (int i = 0; i < 16; i++) {
            if (fabs(MAVLinkManager::hilActuatorControlsData.controls[i]) > 0.001f) {
                activeChannels++;
            }
        }

        snprintf(buf, sizeof(buf), "Active Channels: %d/16 | Timestamp: %llu us",
            activeChannels, (unsigned long long)MAVLinkManager::hilActuatorControlsData.timestamp);
        scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
        drawSmartText(left + 35, scrolledY, buf, valueColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
        lineOffset += getScaledLayout(Layout::LINE_HEIGHT);

        lineOffset += getScaledLayout(Layout::SECTION_SPACING);
    }

    // ALWAYS SHOW: Configuration help
    scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
    drawSmartText(left + 20, scrolledY, "CONFIGURATION HELP:", headerColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
    lineOffset += getScaledLayout(Layout::HEADER_SPACING);

    const char* quickHelp[] = {
        "• Edit config.ini to modify channel-to-dataref mappings",
        "• Configuration is loaded at X-Plane startup",
        "• Use 'Controls' tab to see raw HIL actuator values when connected",
        "• Restart X-Plane after configuration changes",
        "• Connect to PX4 SITL to see live HIL values in this tab"
    };

    for (const auto& help : quickHelp) {
        scrolledY = top - lineOffset + g_uiState.scrollOffset[currentTabIndex];
        drawSmartText(left + 35, scrolledY, (char*)help, contentColor, g_uiState.contentAreaTop, g_uiState.contentAreaBottom);
        lineOffset += getScaledLayout(Layout::LINE_HEIGHT);
    }

    return lineOffset;
}

// =================================================================
// HELPER FUNCTIONS IMPLEMENTATION
// =================================================================

bool UIHandler::Internal::handleTabClick(int mouseX, int mouseY, int windowLeft, int windowTop, int windowRight, int windowBottom) {
    int windowWidth = windowRight - windowLeft;
    int tabWidth = windowWidth / Tabs::TAB_COUNT;
    int tabTop = windowTop - getScaledSize(8);
    int tabBottom = windowTop - getScaledLayout(Layout::TAB_HEIGHT) - getScaledSize(8);

    // Check if click is within tab area
    if (mouseY >= tabBottom && mouseY <= tabTop) {
        int clickedTab = (mouseX - windowLeft) / tabWidth;
        if (clickedTab >= 0 && clickedTab < Tabs::TAB_COUNT) {
            switchToTab(static_cast<Tab>(clickedTab));
            return true;
        }
    }
    return false;
}

bool UIHandler::Internal::handleAboutLinkClick(int mouseX, int mouseY, int windowLeft, int windowTop, int windowRight, int windowBottom) {
    // DISABLED: URL opening to prevent browser spam and X-Plane freezing
    // Links are visible but not clickable for now - this prevents the infinite loop issue

    debugLog("Link click detected but URL opening is disabled for stability");
    return false; // Always return false so click goes to close dialog instead
}