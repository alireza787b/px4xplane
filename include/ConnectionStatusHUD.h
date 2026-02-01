#pragma once

/**
 * @file ConnectionStatusHUD.h
 * @brief Professional HUD-style connection status overlay for X-Plane
 *
 * CRITICAL UX FIX (January 2025):
 * Previously, connection status was only visible in the data window (hidden by default).
 * Users couldn't see progress when clicking "Connect to SITL" - appeared broken.
 *
 * NEW APPROACH: HUD-style overlay on main X-Plane screen
 * - Always visible during connection attempts (no window needed)
 * - Minimal, clean, professional design
 * - Auto-shows/hides based on connection state
 * - Configurable via config.ini
 *
 * FPS WARNING (February 2025):
 * - Shows subtle warning when X-Plane FPS drops below threshold
 * - Low FPS directly impacts sensor data rate sent to PX4
 * - Non-intrusive indicator appears only when needed
 *
 * Uses X-Plane SDK XPLMRegisterDrawCallback() for proper HUD rendering.
 */

#include "XPLMDisplay.h"
#include <string>

class ConnectionStatusHUD {
public:
    /**
     * @brief Connection status enum
     */
    enum class Status {
        DISCONNECTED,     // Not shown (HUD hidden)
        WAITING,          // Yellow "Connecting..." with progress
        CONNECTED,        // Green "Connected" (shows 3s then fades)
        TIMEOUT,          // Red "Connection timeout"
        CONN_ERROR        // Orange error message (renamed from ERROR due to Windows macro conflict)
    };

    /**
     * @brief Initialize the HUD and register draw callback
     * Called once during plugin startup
     */
    static void initialize();

    /**
     * @brief Cleanup and unregister draw callback
     * Called during plugin shutdown
     */
    static void cleanup();

    /**
     * @brief Update connection status (triggers HUD display)
     * @param status Current connection state
     * @param message Optional detail message
     * @param elapsedSeconds Time spent waiting (for progress indicator)
     */
    static void updateStatus(Status status, const std::string& message = "", float elapsedSeconds = 0.0f);

    /**
     * @brief Notify HUD that we're actively connected (for FPS monitoring)
     * Called from flight loop when connected
     */
    static void notifyConnected();

    /**
     * @brief Check if HUD is enabled via config
     * @return true if HUD should be shown
     */
    static bool isEnabled();

    /**
     * @brief Enable/disable HUD display
     * @param enabled true to show HUD, false to hide
     */
    static void setEnabled(bool enabled);

private:
    /**
     * @brief Draw callback registered with X-Plane
     * Called every frame to render HUD overlay
     */
    static int drawCallback(XPLMDrawingPhase inPhase, int inIsBefore, void* inRefcon);

    /**
     * @brief Draw the FPS warning indicator (if needed)
     * @param screenWidth Screen width in pixels
     * @param screenHeight Screen height in pixels
     */
    static void drawFPSWarning(int screenWidth, int screenHeight);

    /**
     * @brief Internal state - Connection status
     */
    static Status currentStatus;
    static std::string currentMessage;
    static float elapsedTime;
    static float fadeTimer;          // For auto-hide after success
    static bool enabled;             // Controlled by config.ini
    static bool registered;          // Track if callback is registered

    /**
     * @brief Internal state - FPS monitoring
     */
    static bool isActivelyConnected; // True when PX4 connection is active
    static float fpsRollingAvg;      // Rolling average FPS (smoothed)
    static float fpsWarningTimer;    // Timer for warning display persistence
    static float lastFPSCheckTime;   // Last time we calculated FPS
    static int frameCount;           // Frame counter for FPS calculation

    // Constants for FPS warning behavior
    static constexpr float FPS_SMOOTHING_FACTOR = 0.1f;    // Lower = smoother (less flickering)
    static constexpr float FPS_WARNING_PERSIST_TIME = 5.0f; // Show warning for at least 5s
    static constexpr float FPS_CHECK_INTERVAL = 0.5f;       // Calculate FPS every 0.5s
};
