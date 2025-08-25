/**
 * @file UIHandler.h
 * @brief Professional UI Management System for PX4-XPlane Plugin
 *
 * This header defines the UIHandler static class that manages all UI functionality
 * for the PX4-XPlane plugin. Separates UI concerns from core plugin functionality
 * for better code organization and maintainability.
 *
 * Features:
 * - Professional 5-tab interface (Connection, Position, Sensors, Controls, Mixing)
 * - X-Plane native color integration
 * - High-DPI display support
 * - Per-tab independent scrolling
 * - Dynamic menu text based on connection state
 * - Enhanced airframe configuration visualization with live updates
 * - Real-time sensor data display across all tabs
 *
 * @author Alireza Ghaderi
 * @copyright Copyright (c) 2025 Alireza Ghaderi. All rights reserved.
 * @license MIT License
 * @url https://github.com/alireza787b/px4xplane
 */

#ifndef UI_HANDLER_H
#define UI_HANDLER_H

#include "XPLMDisplay.h"
#include <string>

 // Forward declarations
class ConnectionManager;
class DataRefManager;
class ConfigManager;

namespace UIHandler {

    // =================================================================
    // TAB SYSTEM ENUMERATIONS
    // =================================================================

    /** Available tabs in the data inspector interface */
    enum class Tab {
        CONNECTION = 0,   ///< Connection status, system info, timestamps
        POSITION = 1,     ///< GPS, attitude, navigation, velocities  
        SENSORS = 2,      ///< IMU, pressure, temperature, airspeed
        CONTROLS = 3,     ///< RC inputs, actuators, aircraft config
        MIXING = 4        ///< Airframe configuration and DataRef mapping
    };

    // =================================================================
    // UI STATE MANAGEMENT
    // =================================================================

    /** Complete UI state for the tabbed interface */
    struct UIState {
        // Tab management
        Tab currentTab = Tab::CONNECTION;
        int scrollOffset[5] = { 0, 0, 0, 0, 0 };        ///< Per-tab scroll positions
        int contentHeight[5] = { 0, 0, 0, 0, 0 };       ///< Per-tab content heights
        int maxScrollOffset[5] = { 0, 0, 0, 0, 0 };     ///< Per-tab scroll limits
        bool needsScrollbar[5] = { false, false, false, false, false }; ///< Per-tab scroll indicators

        // Layout calculations
        int tabAreaHeight = 0;           ///< Height of tab headers + search box
        int contentAreaTop = 0;          ///< Top boundary of content area
        int contentAreaBottom = 0;       ///< Bottom boundary of content area  
        int visibleContentHeight = 0;    ///< Visible content height

        // Search functionality (framework for future)
        std::string searchFilter[5] = { "", "", "", "", "" }; ///< Per-tab search terms

        // UI scaling
        float uiScale = 1.0f;           ///< Current UI scale factor
    };

    // =================================================================
    // MAIN UI MANAGEMENT FUNCTIONS
    // =================================================================

    /**
     * @brief Initialize the UI handler system
     *
     * Sets up X-Plane color datarefs, initializes UI state, and prepares
     * the interface for display. Call once during plugin startup.
     */
    void initialize();

    /**
     * @brief Clean up UI resources
     *
     * Releases any allocated resources and resets UI state.
     * Call during plugin shutdown.
     */
    void cleanup();

    /**
     * @brief Main window drawing function
     *
     * Renders the complete tabbed interface including headers, content,
     * and footer. This is the primary entry point from X-Plane's draw callback.
     *
     * @param windowID X-Plane window identifier
     * @param refcon Reference constant (unused)
     */
    void drawMainWindow(XPLMWindowID windowID, void* refcon);

    /**
     * @brief About dialog drawing function
     *
     * Renders the about dialog with version information, features,
     * and usage instructions.
     *
     * @param windowID X-Plane window identifier
     * @param refcon Reference constant (unused)
     */
    void drawAboutWindow(XPLMWindowID windowID, void* refcon);

    // =================================================================
    // INPUT HANDLING FUNCTIONS
    // =================================================================

    /**
     * @brief Handle mouse clicks on main window
     *
     * Processes tab switching, scrollbar interaction, and other
     * clickable elements in the main interface.
     *
     * @param windowID X-Plane window identifier
     * @param x Mouse X coordinate
     * @param y Mouse Y coordinate
     * @param isDown True if mouse button is pressed
     * @param refcon Reference constant (unused)
     * @return 1 if event was handled, 0 otherwise
     */
    int handleMainWindowMouse(XPLMWindowID windowID, int x, int y, int isDown, void* refcon);

    /**
     * @brief Handle mouse wheel scrolling
     *
     * Manages per-tab independent scrolling with bounds checking
     * and smooth scroll behavior.
     *
     * @param windowID X-Plane window identifier
     * @param x Mouse X coordinate
     * @param y Mouse Y coordinate
     * @param wheel Wheel axis (unused)
     * @param clicks Number of scroll clicks (negative = up, positive = down)
     * @param refcon Reference constant (unused)
     * @return 1 if event was handled, 0 otherwise
     */
    int handleMainWindowWheel(XPLMWindowID windowID, int x, int y, int wheel, int clicks, void* refcon);

    /**
     * @brief Handle mouse clicks on about window
     *
     * Simple click handler that closes the about dialog.
     *
     * @param windowID X-Plane window identifier
     * @param x Mouse X coordinate (unused)
     * @param y Mouse Y coordinate (unused)
     * @param isDown True if mouse button is pressed
     * @param refcon Reference constant (unused)
     * @return 1 if event was handled, 0 otherwise
     */
    int handleAboutWindowMouse(XPLMWindowID windowID, int x, int y, int isDown, void* refcon);

    // =================================================================
    // TAB MANAGEMENT FUNCTIONS
    // =================================================================

    /**
     * @brief Switch to a different tab
     *
     * Changes the active tab and updates scroll state accordingly.
     * Provides smooth transitions and state management.
     *
     * @param newTab Target tab to switch to
     */
    void switchToTab(Tab newTab);

    /**
     * @brief Get current active tab
     *
     * @return Currently active tab
     */
    Tab getCurrentTab();

    /**
     * @brief Update scroll limits for current tab
     *
     * Recalculates scroll boundaries and scrollbar visibility
     * based on content height and visible area.
     */
    void updateCurrentTabScroll();

    // =================================================================
    // MENU INTEGRATION FUNCTIONS  
    // =================================================================

    /**
     * @brief Get dynamic menu text for connection toggle
     *
     * Returns appropriate menu text based on current connection state.
     * Updates automatically as connection state changes.
     *
     * @return "Connect to PX4 SITL" or "Disconnect from PX4 SITL"
     */
    const char* getConnectionMenuText();

    /**
     * @brief Update airframes menu with current configuration
     *
     * Refreshes the airframes submenu to show available configurations
     * and mark the currently active one.
     */
    void updateAirframesMenu();

    /**
     * @brief Notify UI that airframe configuration has changed
     *
     * This function should be called whenever the active airframe changes
     * to ensure the mixing tab displays current configuration information.
     *
     * @param newAirframeName Name of the newly selected airframe
     */
    void notifyAirframeChanged(const std::string& newAirframeName);

    // =================================================================
    // UTILITY AND HELPER FUNCTIONS
    // =================================================================

    /**
     * @brief Calculate layout dimensions for current window size
     *
     * Computes tab areas, content boundaries, and scroll regions
     * based on current window geometry and UI scale.
     *
     * @param windowLeft Left edge of window
     * @param windowTop Top edge of window
     * @param windowRight Right edge of window
     * @param windowBottom Bottom edge of window
     */
    void calculateLayout(int windowLeft, int windowTop, int windowRight, int windowBottom);

    /**
     * @brief Check if content line is visible within bounds
     *
     * Optimizes rendering by only drawing visible content lines.
     *
     * @param yPosition Y coordinate of line
     * @param contentTop Top boundary of visible content
     * @param contentBottom Bottom boundary of visible content
     * @return True if line should be drawn
     */
    bool isLineVisible(int yPosition, int contentTop, int contentBottom);

    /**
     * @brief Smart text drawing with visibility checking
     *
     * Only draws text if it's within visible bounds, improving performance
     * for large content areas.
     *
     * @param x X coordinate for text
     * @param y Y coordinate for text
     * @param text Text string to draw
     * @param color Text color array [R, G, B]
     * @param contentTop Top boundary of visible content
     * @param contentBottom Bottom boundary of visible content
     * @return 1 if text was drawn, 0 if clipped
     */
    int drawSmartText(int x, int y, const char* text, float color[3], int contentTop, int contentBottom);

    /**
     * @brief Log debug messages with UI prefix
     *
     * Standardized debug logging for UI-related messages.
     *
     * @param message Debug message to log
     */
    void debugLog(const char* message);

    // =================================================================
    // INTERNAL DRAWING FUNCTIONS (Private interface)
    // =================================================================

    namespace Internal {
        /** Draw subtle background */
        void drawBackground(int windowLeft, int windowTop, int windowRight, int windowBottom);

        /** Draw professional tab headers */
        void drawTabHeaders(int windowLeft, int windowTop, int windowRight, int windowBottom);

        /** Draw search box framework */
        void drawSearchBox(int windowLeft, int windowTop, int windowRight, int windowBottom);

        /** Draw tab-specific content */
        void drawTabContent(int windowLeft, int windowTop, int windowRight, int windowBottom);

        /** Draw enhanced scrollbar with indicators */
        void drawScrollbar(int windowLeft, int windowTop, int windowRight, int windowBottom);

        /** Draw professional footer with links */
        void drawFooter(int windowLeft, int windowTop, int windowRight, int windowBottom);

        /** Draw professional about dialog content */
        void drawAboutContent(int windowLeft, int windowTop, int windowRight, int windowBottom);

        // Tab-specific content drawing functions
        int drawConnectionTabContent(int left, int top, int right, int bottom, int startOffset);
        int drawPositionTabContent(int left, int top, int right, int bottom, int startOffset);
        int drawSensorsTabContent(int left, int top, int right, int bottom, int startOffset);
        int drawControlsTabContent(int left, int top, int right, int bottom, int startOffset);
        int drawMixingTabContent(int left, int top, int right, int bottom, int startOffset);

        /** Handle tab header click detection */
        bool handleTabClick(int mouseX, int mouseY, int windowLeft, int windowTop, int windowRight, int windowBottom);

        /** Draw DataRef category with filtering and live values */
        int drawDataRefCategory(int left, int top, float color[3], int lineOffset, const char* categoryFilter, int contentTop, int contentBottom, int& drawnLines);
    }

} // namespace UIHandler

#endif // UI_HANDLER_H