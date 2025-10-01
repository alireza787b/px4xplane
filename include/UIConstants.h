/**
 * @file UIConstants.h
 * @brief UI Constants with High Contrast Colors for X-Plane Plugin
 *
 * Provides professional UI constants system optimized for readability
 * against X-Plane's dark theme.
 *
 * @author Alireza Ghaderi
 * @copyright Copyright (c) 2025 Alireza Ghaderi. All rights reserved.
 * @license MIT License
 * @version 2.5.0
 */

#ifndef UI_CONSTANTS_H
#define UI_CONSTANTS_H

#include "XPLMDataAccess.h"
#include "XPLMGraphics.h"
#include <stdio.h>
#include <math.h>

namespace UIConstants {

    // Layout dimensions
    namespace Layout {
        constexpr int TAB_HEIGHT = 40;
        constexpr int SEARCH_BOX_HEIGHT = 35;
        constexpr int FOOTER_HEIGHT = 50;
        constexpr int CONTENT_MARGIN = 15;
        constexpr int MIN_WINDOW_WIDTH = 900;
        constexpr int MIN_WINDOW_HEIGHT = 700;
        constexpr int SCROLLBAR_WIDTH = 16;
        constexpr int LINE_HEIGHT = 24;
        constexpr int SECTION_SPACING = 35;
        constexpr int HEADER_SPACING = 30;
    }

    // High contrast colors
    namespace Colors {
        constexpr float CONNECTED[3] = { 0.1f, 1.0f, 0.1f };
        constexpr float DISCONNECTED[3] = { 1.0f, 0.2f, 0.2f };
        constexpr float WARNING[3] = { 1.0f, 0.8f, 0.0f };
        constexpr float HEADER_TEXT[3] = { 1.0f, 1.0f, 0.4f };
        constexpr float CONTENT_TEXT[3] = { 1.0f, 1.0f, 1.0f };
        constexpr float VALUE_TEXT[3] = { 0.8f, 1.0f, 0.8f };
        constexpr float TAB_ACTIVE[3] = { 1.0f, 1.0f, 1.0f };
        constexpr float TAB_INACTIVE[3] = { 0.7f, 0.7f, 0.7f };
        constexpr float TAB_BORDER[3] = { 0.5f, 0.8f, 1.0f };
        constexpr float SCROLLBAR_TRACK[3] = { 0.3f, 0.3f, 0.3f };
        constexpr float SCROLLBAR_THUMB[3] = { 0.8f, 0.8f, 1.0f };
        constexpr float FOOTER_TEXT[3] = { 0.8f, 0.8f, 0.9f };
        constexpr float LINK_TEXT[3] = { 0.5f, 0.8f, 1.0f };
        constexpr float HIL_VALUE[3] = { 0.9f, 1.0f, 0.6f };
    }

    // Font management
    namespace Fonts {
        extern int PRIMARY;
        extern int HEADER;
        extern int CONTENT;
        extern int FOOTER;
        void initialize();
    }

    // Tab system
    namespace Tabs {
        static constexpr int TAB_COUNT = 5;
        const char* getTabName(int index);
        const char* getTabDescription(int index);
    }

    // Status indicators
    namespace Status {
        constexpr const char* CONNECTED_ICON = "*";
        constexpr const char* DISCONNECTED_ICON = "o";
        constexpr const char* WARNING_ICON = "!";
        constexpr const char* CONNECTED_TEXT = "CONNECTED";
        constexpr const char* DISCONNECTED_TEXT = "DISCONNECTED";
    }

    // Scrolling
    namespace Scrolling {
        constexpr int SCROLL_SPEED = 30;
        constexpr int MIN_THUMB_SIZE = 20;
    }

    // X-Plane integration
    class XPlaneColors {
    private:
        static XPLMDataRef s_uiScaleRef;
        static bool s_initialized;
    public:
        static void initialize();
        static float getUIScale();
        static bool isDarkTheme() { return true; }
    };

    // Utility functions
    inline int getScaledSize(int baseSize) {
        return static_cast<int>(baseSize * XPlaneColors::getUIScale());
    }

    inline int getScaledLayout(int baseSize) {
        float scale = XPlaneColors::getUIScale();
        scale = (scale < 0.8f) ? 0.8f : ((scale > 2.5f) ? 2.5f : scale);
        return static_cast<int>(baseSize * scale);
    }

    inline int centerTextX(int leftBound, int rightBound, const char* text, int charWidth = 7) {
        int textWidth = static_cast<int>(strlen(text)) * charWidth;
        int available = rightBound - leftBound;
        return leftBound + ((available - textWidth) / 2);
    }

    template<typename T>
    inline T clamp(T value, T minVal, T maxVal) {
        return (value < minVal) ? minVal : ((value > maxVal) ? maxVal : value);
    }

} // namespace UIConstants

#endif // UI_CONSTANTS_H
