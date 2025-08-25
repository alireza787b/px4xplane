/**
 * @file UIConstants.h
 * @brief Production-Ready UI Constants with High Contrast Colors for X-Plane Plugin
 *
 * This header provides a professional UI constants system optimized for readability
 * against X-Plane's dark theme. Features high-contrast colors, proper font handling,
 * and scalable layout dimensions for production use.
 *
 * @author Alireza Ghaderi
 * @copyright Copyright (c) 2025 Alireza Ghaderi. All rights reserved.
 * @license MIT License
 * @version 2.5.0 (Production Ready)
 * @url https://github.com/alireza787b/px4xplane
 */

#ifndef UI_CONSTANTS_H
#define UI_CONSTANTS_H

#include "XPLMDataAccess.h"
#include "XPLMGraphics.h"  // For font constants

namespace UIConstants {

    // =================================================================
    // LAYOUT DIMENSIONS - Professional spacing and sizing
    // =================================================================

    namespace Layout {
        /** Tab header height in pixels */
        constexpr int TAB_HEIGHT = 40;

        /** Search box height in pixels */
        constexpr int SEARCH_BOX_HEIGHT = 35;

        /** Footer height in pixels */
        constexpr int FOOTER_HEIGHT = 50;

        /** Content margin around tab content */
        constexpr int CONTENT_MARGIN = 15;

        /** Minimum window width for proper display */
        constexpr int MIN_WINDOW_WIDTH = 900;

        /** Minimum window height for proper display */
        constexpr int MIN_WINDOW_HEIGHT = 700;

        /** Scrollbar width in pixels */
        constexpr int SCROLLBAR_WIDTH = 16;

        /** Minimum visible lines before scrolling */
        constexpr int MIN_VISIBLE_LINES = 8;

        /** Line height for text content - increased for readability */
        constexpr int LINE_HEIGHT = 24;

        /** Section spacing between content groups */
        constexpr int SECTION_SPACING = 35;

        /** Header text spacing */
        constexpr int HEADER_SPACING = 30;

        /** Tab label padding */
        constexpr int TAB_PADDING = 15;
    }

    namespace Scrolling {
        /** Pixels per scroll wheel click */
        constexpr int SCROLL_SPEED = 30;

        /** Minimum thumb size for scrollbar */
        constexpr int MIN_THUMB_SIZE = 20;

        /** Scrollbar track margin */
        constexpr int TRACK_MARGIN = 8;
    }

    // =================================================================
    // HIGH CONTRAST COLORS - Optimized for X-Plane's Dark Theme
    // =================================================================

    namespace Colors {
        // Status indicators - bright and clear
        constexpr float CONNECTED[3] = { 0.1f, 1.0f, 0.1f };     // Bright green
        constexpr float DISCONNECTED[3] = { 1.0f, 0.2f, 0.2f };  // Bright red
        constexpr float WARNING[3] = { 1.0f, 0.8f, 0.0f };       // Bright amber

        // Text colors - high contrast against dark backgrounds
        constexpr float HEADER_TEXT[3] = { 1.0f, 1.0f, 0.4f };   // Bright yellow
        constexpr float CONTENT_TEXT[3] = { 1.0f, 1.0f, 1.0f };  // Pure white
        constexpr float VALUE_TEXT[3] = { 0.8f, 1.0f, 0.8f };    // Light green
        constexpr float LABEL_TEXT[3] = { 0.9f, 0.9f, 1.0f };    // Light blue-white

        // Tab colors - clear distinction
        constexpr float TAB_ACTIVE[3] = { 1.0f, 1.0f, 1.0f };    // Pure white
        constexpr float TAB_INACTIVE[3] = { 0.7f, 0.7f, 0.7f };  // Medium gray
        constexpr float TAB_BORDER[3] = { 0.5f, 0.8f, 1.0f };    // Light blue

        // Background elements - subtle but visible
        constexpr float SEARCH_BOX[3] = { 0.4f, 0.6f, 1.0f };    // Light blue
        constexpr float SCROLLBAR_TRACK[3] = { 0.3f, 0.3f, 0.3f }; // Dark gray
        constexpr float SCROLLBAR_THUMB[3] = { 0.8f, 0.8f, 1.0f }; // Light gray

        // Special content colors
        constexpr float MIXING_CONTENT[3] = { 0.6f, 1.0f, 1.0f }; // Cyan
        constexpr float AIRFRAME_TEXT[3] = { 0.4f, 1.0f, 0.4f };  // Bright green
        constexpr float FOOTER_TEXT[3] = { 0.8f, 0.8f, 0.9f };    // Light gray
        constexpr float LINK_TEXT[3] = { 0.5f, 0.8f, 1.0f };      // Light blue for links

        // Border and separator colors
        constexpr float BORDER[3] = { 0.6f, 0.6f, 0.8f };        // Light gray-blue
        constexpr float SEPARATOR[3] = { 0.5f, 0.5f, 0.7f };     // Medium gray-blue
    }

    // =================================================================
    // FONT MANAGEMENT - Runtime initialization required
    // =================================================================

    namespace Fonts {
        // Font constants - initialized at runtime (NOT constexpr)
        extern int PRIMARY;    // xplmFont_Basic for compatibility
        extern int HEADER;     // xplmFont_Proportional for headers  
        extern int CONTENT;    // xplmFont_Proportional for content
        extern int FOOTER;     // xplmFont_Basic for footer

        /** Initialize font constants at startup */
        void initialize();
    }

    // =================================================================
    // TAB SYSTEM CONFIGURATION
    // =================================================================

    namespace Tabs {
        /** Total number of tabs in the interface */
        static constexpr int TAB_COUNT = 5;

        /** Tab content spacing */
        constexpr int CONTENT_SPACING = 20;

        /** Tab label padding */
        constexpr int LABEL_PADDING = 12;

        /** Get tab name by index (runtime function) */
        const char* getTabName(int index);

        /** Get tab description by index (runtime function) */
        const char* getTabDescription(int index);
    }

    // =================================================================
    // STATUS INDICATORS AND ICONS - ASCII COMPATIBLE
    // =================================================================

    namespace Status {
        // Connection status indicators - ASCII only for compatibility
        constexpr const char* CONNECTED_ICON = "*";        // Asterisk (connected)
        constexpr const char* DISCONNECTED_ICON = "o";     // Lowercase o (disconnected)
        constexpr const char* WARNING_ICON = "!";          // Exclamation mark (warning)

        // Status text
        constexpr const char* CONNECTED_TEXT = "CONNECTED";
        constexpr const char* DISCONNECTED_TEXT = "DISCONNECTED";
        constexpr const char* CONNECTING_TEXT = "CONNECTING...";

        // UI indicators - ASCII only
        constexpr const char* SCROLL_INDICATOR = "|";      // Pipe for scroll
        constexpr const char* SEARCH_ICON = "?";           // Question mark for search
        constexpr const char* LINK_ICON = ">";             // Greater than for links
    }

    // =================================================================
    // X-PLANE NATIVE COLOR INTEGRATION
    // =================================================================

    class XPlaneColors {
    private:
        static XPLMDataRef s_uiScaleRef;
        static bool s_initialized;

    public:
        /** Initialize X-Plane integration */
        static void initialize();

        /** Get current UI scale factor for high-DPI support */
        static float getUIScale();

        /** Check if we're in a dark theme (always assume yes for readability) */
        static bool isDarkTheme() { return true; }
    };

    // =================================================================
    // UTILITY FUNCTIONS - Scale-aware helpers
    // =================================================================

    /** Get scaled dimension based on current UI scale */
    inline int getScaledSize(int baseSize) {
        return static_cast<int>(baseSize * XPlaneColors::getUIScale());
    }

    /** Get scaled layout dimension with reasonable limits */
    inline int getScaledLayout(int baseSize) {
        float scale = XPlaneColors::getUIScale();
        // Clamp scaling for layout elements to prevent UI breakage
        scale = (scale < 0.8f) ? 0.8f : ((scale > 2.5f) ? 2.5f : scale);
        return static_cast<int>(baseSize * scale);
    }

    /** Center text within given bounds */
    inline int centerTextX(int leftBound, int rightBound, const char* text, int charWidth = 7) {
        int textWidth = static_cast<int>(strlen(text)) * charWidth;
        int available = rightBound - leftBound;
        return leftBound + ((available - textWidth) / 2);
    }

    /** Clamp value between min and max */
    template<typename T>
    inline T clamp(T value, T minVal, T maxVal) {
        return (value < minVal) ? minVal : ((value > maxVal) ? maxVal : value);
    }

    /** Calculate contrast-adjusted color based on background */
    inline void adjustColorForReadability(float color[3], bool isDarkBackground = true) {
        if (isDarkBackground) {
            // Ensure minimum brightness for dark backgrounds
            for (int i = 0; i < 3; i++) {
                if (color[i] < 0.3f) color[i] = 0.3f + (color[i] * 0.7f);
            }
        }
    }

} // namespace UIConstants

#endif // UI_CONSTANTS_H