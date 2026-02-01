#include "ConnectionStatusHUD.h"
#include "ConfigManager.h"
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMUtilities.h"
#include "XPLMDataAccess.h"
#if IBM
#include <windows.h>
#include <GL/gl.h>
#elif APL
#include <OpenGL/gl.h>
#elif LIN
#include <GL/gl.h>
#endif
#include <cstdio>
#include <cstring>  // For strlen()

// Initialize static members - Connection status
ConnectionStatusHUD::Status ConnectionStatusHUD::currentStatus = ConnectionStatusHUD::Status::DISCONNECTED;
std::string ConnectionStatusHUD::currentMessage;
float ConnectionStatusHUD::elapsedTime = 0.0f;
float ConnectionStatusHUD::fadeTimer = 0.0f;
bool ConnectionStatusHUD::enabled = false;
bool ConnectionStatusHUD::registered = false;

// Initialize static members - FPS monitoring
bool ConnectionStatusHUD::isActivelyConnected = false;
float ConnectionStatusHUD::fpsRollingAvg = 60.0f;  // Start with reasonable assumption
float ConnectionStatusHUD::fpsWarningTimer = 0.0f;
float ConnectionStatusHUD::lastFPSCheckTime = 0.0f;
int ConnectionStatusHUD::frameCount = 0;

void ConnectionStatusHUD::initialize() {
    if (!registered) {
        // Register draw callback for HUD overlay
        // xplm_Phase_Window draws after 3D scene but before X-Plane UI
        XPLMRegisterDrawCallback(
            drawCallback,
            xplm_Phase_Window,
            0,  // Draw after 3D (before = 0)
            nullptr
        );
        registered = true;
        XPLMDebugString("px4xplane: Connection status HUD initialized\n");
    }
}

void ConnectionStatusHUD::cleanup() {
    if (registered) {
        XPLMUnregisterDrawCallback(
            drawCallback,
            xplm_Phase_Window,
            0,
            nullptr
        );
        registered = false;
        XPLMDebugString("px4xplane: Connection status HUD cleaned up\n");
    }
}

void ConnectionStatusHUD::updateStatus(Status status, const std::string& message, float elapsedSeconds) {
    currentStatus = status;
    currentMessage = message;
    elapsedTime = elapsedSeconds;

    // Reset fade timer when status changes
    if (status == ConnectionStatusHUD::Status::CONNECTED) {
        fadeTimer = 3.0f;  // Show success message for 3 seconds
    } else {
        fadeTimer = 0.0f;
    }
}

bool ConnectionStatusHUD::isEnabled() {
    return enabled;
}

void ConnectionStatusHUD::setEnabled(bool isEnabled) {
    enabled = isEnabled;
}

void ConnectionStatusHUD::notifyConnected() {
    isActivelyConnected = true;
}

int ConnectionStatusHUD::drawCallback(XPLMDrawingPhase inPhase, int inIsBefore, void* inRefcon) {
    // Skip if HUD is disabled entirely
    if (!enabled) {
        return 1;
    }

    // Get timing info (needed for both connection status and FPS monitoring)
    static float lastTime = 0.0f;
    float currentTime = XPLMGetDataf(XPLMFindDataRef("sim/time/total_running_time_sec"));
    float deltaTime = (lastTime > 0.0f) ? (currentTime - lastTime) : 0.0f;
    lastTime = currentTime;

    // Get screen dimensions
    int screenWidth, screenHeight;
    XPLMGetScreenSize(&screenWidth, &screenHeight);

    // =========================================================================
    // FPS MONITORING (runs whenever connected, independent of connection HUD)
    // =========================================================================
    if (isActivelyConnected && ConfigManager::fps_warning_enabled) {
        frameCount++;

        // Calculate FPS periodically (not every frame - reduces overhead)
        float timeSinceLastCheck = currentTime - lastFPSCheckTime;
        if (timeSinceLastCheck >= FPS_CHECK_INTERVAL) {
            float instantFPS = (float)frameCount / timeSinceLastCheck;
            frameCount = 0;
            lastFPSCheckTime = currentTime;

            // Apply exponential moving average for smooth, stable reading
            fpsRollingAvg = (FPS_SMOOTHING_FACTOR * instantFPS) + ((1.0f - FPS_SMOOTHING_FACTOR) * fpsRollingAvg);
        }

        // Check if we should show/extend warning
        if (fpsRollingAvg < (float)ConfigManager::fps_warning_threshold) {
            fpsWarningTimer = FPS_WARNING_PERSIST_TIME;  // Reset/extend timer
        }

        // Draw FPS warning if timer is active
        if (fpsWarningTimer > 0.0f) {
            drawFPSWarning(screenWidth, screenHeight);
            fpsWarningTimer -= deltaTime;
        }
    } else {
        // Not connected - reset FPS state
        isActivelyConnected = false;
        fpsWarningTimer = 0.0f;
        frameCount = 0;
    }

    // =========================================================================
    // CONNECTION STATUS HUD (original functionality)
    // =========================================================================
    if (currentStatus == ConnectionStatusHUD::Status::DISCONNECTED) {
        return 1;  // Nothing more to draw
    }

    // Handle fade timer for success message
    if (fadeTimer > 0.0f) {
        fadeTimer -= deltaTime;
        if (fadeTimer <= 0.0f) {
            // Fade complete - hide success message
            currentStatus = ConnectionStatusHUD::Status::DISCONNECTED;
            return 1;
        }
    }

    // HUD position: Top-center (X-Plane standard position)
    const int HUD_WIDTH = 400;
    const int HUD_HEIGHT = 60;
    const int TOP_PADDING = 40;  // More space from top for better visibility

    // Center horizontally
    int hudLeft = (screenWidth - HUD_WIDTH) / 2;
    int hudRight = hudLeft + HUD_WIDTH;
    int hudTop = screenHeight - TOP_PADDING;
    int hudBottom = hudTop - HUD_HEIGHT;

    // Background box (darker, more opaque for better contrast - X-Plane standard)
    float bgColor[4] = { 0.0f, 0.0f, 0.0f, 0.85f };
    XPLMSetGraphicsState(
        0,  // No fog
        0,  // No texture
        0,  // No lighting
        0,  // No alpha testing
        1,  // Blend
        0,  // No depth test
        0   // No depth write
    );

    // Draw background
    glColor4fv(bgColor);
    glBegin(GL_QUADS);
    glVertex2i(hudLeft, hudBottom);
    glVertex2i(hudRight, hudBottom);
    glVertex2i(hudRight, hudTop);
    glVertex2i(hudLeft, hudTop);
    glEnd();

    // Border (subtle)
    float borderColor[4];
    switch (currentStatus) {
        case ConnectionStatusHUD::Status::WAITING:
            borderColor[0] = 1.0f; borderColor[1] = 1.0f; borderColor[2] = 0.0f; borderColor[3] = 0.8f; // Yellow
            break;
        case ConnectionStatusHUD::Status::CONNECTED:
            borderColor[0] = 0.0f; borderColor[1] = 1.0f; borderColor[2] = 0.0f; borderColor[3] = 0.8f; // Green
            break;
        case ConnectionStatusHUD::Status::TIMEOUT:
        case ConnectionStatusHUD::Status::CONN_ERROR:
            borderColor[0] = 1.0f; borderColor[1] = 0.3f; borderColor[2] = 0.0f; borderColor[3] = 0.8f; // Orange/Red
            break;
        default:
            borderColor[0] = 0.5f; borderColor[1] = 0.5f; borderColor[2] = 0.5f; borderColor[3] = 0.8f; // Gray
            break;
    }

    glColor4fv(borderColor);
    glLineWidth(2.0f);
    glBegin(GL_LINE_LOOP);
    glVertex2i(hudLeft, hudBottom);
    glVertex2i(hudRight, hudBottom);
    glVertex2i(hudRight, hudTop);
    glVertex2i(hudLeft, hudTop);
    glEnd();

    // Prepare status text - clean, minimal, informative (X-Plane standard)
    char mainText[256];
    char subText[256];
    float mainColor[3];
    float subColor[3] = { 0.9f, 0.9f, 0.9f }; // Light gray for subtitle

    subText[0] = '\0'; // Default: no subtitle

    switch (currentStatus) {
        case ConnectionStatusHUD::Status::WAITING: {
            // Main: Clean connection message with timer
            int dots = ((int)elapsedTime) % 4;
            snprintf(mainText, sizeof(mainText),
                     "PX4 SITL CONNECTION%.*s",
                     dots + 1, "...");
            mainColor[0] = 1.0f; mainColor[1] = 0.95f; mainColor[2] = 0.0f; // Bright yellow

            // Subtitle: Show elapsed time and hint
            if (elapsedTime > 20.0f) {
                // Warning after 20s
                snprintf(subText, sizeof(subText), "Waiting %ds - Check PX4 SITL is running", (int)elapsedTime);
                subColor[0] = 1.0f; subColor[1] = 0.6f; subColor[2] = 0.2f; // Orange warning
            } else {
                snprintf(subText, sizeof(subText), "Waiting for PX4 (%ds)", (int)elapsedTime);
            }
            break;
        }
        case ConnectionStatusHUD::Status::CONNECTED:
            snprintf(mainText, sizeof(mainText), "PX4 SITL CONNECTED");
            mainColor[0] = 0.2f; mainColor[1] = 1.0f; mainColor[2] = 0.2f; // Bright green
            snprintf(subText, sizeof(subText), "Ready for flight");
            break;
        case ConnectionStatusHUD::Status::TIMEOUT:
            snprintf(mainText, sizeof(mainText), "PX4 CONNECTION TIMEOUT");
            mainColor[0] = 1.0f; mainColor[1] = 0.3f; mainColor[2] = 0.1f; // Red-orange
            snprintf(subText, sizeof(subText), "Click 'Connect to SITL' to retry");
            break;
        case ConnectionStatusHUD::Status::CONN_ERROR:
            snprintf(mainText, sizeof(mainText), "PX4 CONNECTION ERROR");
            mainColor[0] = 1.0f; mainColor[1] = 0.4f; mainColor[2] = 0.0f; // Orange
            if (!currentMessage.empty()) {
                snprintf(subText, sizeof(subText), "%.80s", currentMessage.c_str());
            }
            break;
        default:
            mainText[0] = '\0';
            break;
    }

    // Calculate text centering
    int textCenterX = hudLeft + (HUD_WIDTH / 2);

    // Draw main status text (centered, larger)
    if (mainText[0] != '\0') {
        // Center the text by calculating its approximate width
        int textLen = strlen(mainText);
        int approxTextWidth = textLen * 8; // Approximate 8 pixels per char for proportional font

        XPLMDrawString(mainColor,
                       textCenterX - (approxTextWidth / 2),
                       hudTop - 25,
                       mainText,
                       nullptr,
                       xplmFont_Proportional);
    }

    // Draw subtitle (centered, smaller)
    if (subText[0] != '\0') {
        int textLen = strlen(subText);
        int approxTextWidth = textLen * 6; // Smaller font

        XPLMDrawString(subColor,
                       textCenterX - (approxTextWidth / 2),
                       hudTop - 45,
                       subText,
                       nullptr,
                       xplmFont_Basic);
    }

    return 1;  // Return 1 to continue calling
}

/**
 * @brief Draw the FPS warning indicator
 *
 * Design principles (consistent with main HUD):
 * - Minimal and non-intrusive
 * - Positioned to avoid obscuring critical flight info
 * - Color-coded severity (yellow for warning, orange for critical)
 * - Auto-hides when FPS recovers
 */
void ConnectionStatusHUD::drawFPSWarning(int screenWidth, int screenHeight) {
    // Position: Bottom-left corner (out of the way, but visible)
    const int WARNING_WIDTH = 200;
    const int WARNING_HEIGHT = 28;
    const int LEFT_MARGIN = 15;
    const int BOTTOM_MARGIN = 15;

    int warnLeft = LEFT_MARGIN;
    int warnRight = warnLeft + WARNING_WIDTH;
    int warnBottom = BOTTOM_MARGIN;
    int warnTop = warnBottom + WARNING_HEIGHT;

    // Determine severity level
    float threshold = (float)ConfigManager::fps_warning_threshold;
    bool isCritical = (fpsRollingAvg < threshold * 0.6f);  // Below 60% of threshold = critical

    // Background with transparency
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);

    float bgAlpha = 0.75f;
    glColor4f(0.1f, 0.1f, 0.1f, bgAlpha);
    glBegin(GL_QUADS);
    glVertex2i(warnLeft, warnBottom);
    glVertex2i(warnRight, warnBottom);
    glVertex2i(warnRight, warnTop);
    glVertex2i(warnLeft, warnTop);
    glEnd();

    // Border color based on severity
    float borderColor[4];
    if (isCritical) {
        // Orange-red for critical
        borderColor[0] = 1.0f; borderColor[1] = 0.4f; borderColor[2] = 0.1f; borderColor[3] = 0.9f;
    } else {
        // Yellow for warning
        borderColor[0] = 1.0f; borderColor[1] = 0.9f; borderColor[2] = 0.2f; borderColor[3] = 0.9f;
    }

    glColor4fv(borderColor);
    glLineWidth(1.5f);
    glBegin(GL_LINE_LOOP);
    glVertex2i(warnLeft, warnBottom);
    glVertex2i(warnRight, warnBottom);
    glVertex2i(warnRight, warnTop);
    glVertex2i(warnLeft, warnTop);
    glEnd();

    // Warning text
    char fpsText[64];
    snprintf(fpsText, sizeof(fpsText), "LOW FPS: %.0f (min %d)", fpsRollingAvg, ConfigManager::fps_warning_threshold);

    // Text color matches border
    float textColor[3] = { borderColor[0], borderColor[1], borderColor[2] };

    // Center text in the warning box
    int textLen = strlen(fpsText);
    int approxTextWidth = textLen * 6;
    int textX = warnLeft + (WARNING_WIDTH - approxTextWidth) / 2;
    int textY = warnBottom + 8;

    XPLMDrawString(textColor, textX, textY, fpsText, nullptr, xplmFont_Basic);
}
