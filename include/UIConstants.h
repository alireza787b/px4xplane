/**
 * @file UIConstants.h
 * @brief Professional UI Constants and Color Management for PX4-XPlane Plugin
 *
 * This header provides a comprehensive UI constants system using X-Plane's native
 * color datarefs for consistency with the simulator's theme. Automatically adapts
 * to high-DPI displays and user color preferences.
 *
 * @author Alireza Ghaderi
 * @copyright Copyright (c) 2025 Alireza Ghaderi. All rights reserved.
 * @license MIT License
 * @url https://github.com/alireza787b/px4xplane
 */

#ifndef UI_CONSTANTS_H
#define UI_CONSTANTS_H

#include "XPLMDataAccess.h"
#include "XPLMGraphics.h"  // For font constants

namespace UIConstants {

    // =================================================================
    // LAYOUT DIMENSIONS - Scalable with X-Plane UI scale
    // =================================================================

    namespace Layout {
        /** Tab header height in pixels */
        constexpr int TAB_HEIGHT = 35;

        /** Search box height in pixels */
        constexpr int SEARCH_BOX_HEIGHT = 30;

        /** Footer height in pixels */
        constexpr int FOOTER_HEIGHT = 40;

        /** Content margin around tab content */
        constexpr int CONTENT_MARGIN = 10;

        /** Minimum window width for proper display */
        constexpr int MIN_WINDOW_WIDTH = 800;

        /** Minimum window height for proper display */
        constexpr int MIN_WINDOW_HEIGHT = 600;

        /** Scrollbar width in pixels */
        constexpr int SCROLLBAR_WIDTH = 12;

        /** Minimum visible lines before scrolling */
        constexpr int MIN_VISIBLE_LINES = 5;

        /** Line height for text content */
        constexpr int LINE_HEIGHT = 20;

        /** Tab border thickness */
        constexpr int TAB_BORDER_WIDTH = 2;
    }

    namespace Scrolling {
        /** Pixels per scroll wheel click */
        constexpr int SCROLL_SPEED = 25;

        /** Minimum thumb size for scrollbar */
        constexpr int MIN_THUMB_SIZE = 15;

        /** Scrollbar track margin */
        constexpr int TRACK_MARGIN = 5;
    }

    // =================================================================
    // X-PLANE NATIVE COLOR SYSTEM - Uses X-Plane's built-in color datarefs
    // =================================================================

    class XPlaneColors {
    private:
        // X-Plane color dataref references
        static XPLMDataRef s_menuTextRef;
        static XPLMDataRef s_menuTextDisabledRef;
        static XPLMDataRef s_tabFrontRef;
        static XPLMDataRef s_tabBackRef;
        static XPLMDataRef s_captionTextRef;
        static XPLMDataRef s_listTextRef;
        static XPLMDataRef s_menuHiliteRef;
        static XPLMDataRef s_uiScaleRef;

        static bool s_initialized;

    public:
        /** Initialize X-Plane color dataref system */
        static void initialize();

        /** Get current UI scale factor for high-DPI support */
        static float getUIScale();

        /** Get X-Plane's native menu text color */
        static void getMenuTextColor(float color[3]);

        /** Get X-Plane's native disabled text color */
        static void getMenuTextDisabledColor(float color[3]);

        /** Get X-Plane's native active tab color */
        static void getTabActiveColor(float color[3]);

        /** Get X-Plane's native inactive tab color */
        static void getTabInactiveColor(float color[3]);

        /** Get X-Plane's native caption text color */
        static void getCaptionTextColor(float color[3]);

        /** Get X-Plane's native list text color */
        static void getListTextColor(float color[3]);

        /** Get X-Plane's native highlight color */
        static void getHighlightColor(float color[3]);
    };

    // =================================================================
    // CUSTOM COLORS - Professional additions to X-Plane's palette
    // =================================================================

    namespace CustomColors {
        /** Bright green for connected status */
        constexpr float CONNECTED[3] = { 0.2f, 0.8f, 0.2f };

        /** Bright red for disconnected status */
        constexpr float DISCONNECTED[3] = { 0.9f, 0.3f, 0.3f };

        /** Orange for warning states */
        constexpr float WARNING[3] = { 1.0f, 0.7f, 0.0f };

        /** Light yellow for header text */
        constexpr float HEADER_TEXT[3] = { 1.0f, 1.0f, 0.8f };

        /** Light green for airframe configuration text */
        constexpr float AIRFRAME_TEXT[3] = { 0.8f, 1.0f, 0.8f };

        /** Semi-transparent gray for search box */
        constexpr float SEARCH_BOX[3] = { 0.6f, 0.6f, 0.8f };

        /** Dark gray for scrollbar track */
        constexpr float SCROLLBAR_TRACK[3] = { 0.4f, 0.4f, 0.4f };

        /** Light gray for scrollbar thumb */
        constexpr float SCROLLBAR_THUMB[3] = { 0.8f, 0.8f, 0.9f };

        /** Pure white for standard text (fallback) */
        constexpr float WHITE[3] = { 1.0f, 1.0f, 1.0f };

        /** Light blue for mixing tab content */
        constexpr float MIXING_CONTENT[3] = { 0.8f, 0.9f, 1.0f };
    }

    // =================================================================
    // FONT CONFIGURATION - X-Plane SDK font types
    // =================================================================

    namespace Fonts {
        // Runtime font constants - initialized at startup
        extern int PRIMARY;
        extern int HEADER;
        extern int CONTENT;
        extern int FOOTER;

        /** Initialize font constants at runtime */
        void initialize();
    }

    // =================================================================
    // TAB SYSTEM CONFIGURATION
    // =================================================================

    namespace Tabs {
        /** Total number of tabs in the interface */
        constexpr int TAB_COUNT = 5;  // Added mixing tab

        /** Get tab name by index */
        const char* getTabName(int index);

        /** Get tab description by index */
        const char* getTabDescription(int index);

        /** Tab content spacing */
        constexpr int TAB_CONTENT_SPACING = 15;

        /** Tab label padding */
        constexpr int TAB_LABEL_PADDING = 10;
    }

    // =================================================================
    // STATUS INDICATORS 
    // =================================================================

    namespace Status {
        /** Connection status indicators */
        constexpr const char* CONNECTED_ICON = "🟢";
        constexpr const char* DISCONNECTED_ICON = "🔴";
        constexpr const char* WARNING_ICON = "🟡";

        /** Status text */
        constexpr const char* CONNECTED_TEXT = "CONNECTED";
        constexpr const char* DISCONNECTED_TEXT = "DISCONNECTED";
        constexpr const char* CONNECTING_TEXT = "CONNECTING...";

        /** Scroll indicators */
        constexpr const char* SCROLL_INDICATOR = "↕";
        constexpr const char* SEARCH_ICON = "🔍";
    }

    // =================================================================
    // UTILITY FUNCTIONS - Scale-aware helpers
    // =================================================================

    /** Get scaled dimension based on current UI scale */
    inline int getScaledSize(int baseSize) {
        return static_cast<int>(baseSize * XPlaneColors::getUIScale());
    }

    /** Get scaled layout dimension */
    inline int getScaledLayout(int baseSize) {
        float scale = XPlaneColors::getUIScale();
        // Clamp scaling for layout elements to prevent UI breakage
        scale = (scale < 1.0f) ? 1.0f : ((scale > 2.0f) ? 2.0f : scale);
        return static_cast<int>(baseSize * scale);
    }

    /** Center text within given bounds */
    inline int centerTextX(int leftBound, int rightBound, const char* text, int charWidth = 6) {
        int textWidth = strlen(text) * charWidth;
        int available = rightBound - leftBound;
        return leftBound + ((available - textWidth) / 2);
    }

} // namespace UIConstants

#endif // UI_CONSTANTS_H