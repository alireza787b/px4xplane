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
     * @brief Internal state
     */
    static Status currentStatus;
    static std::string currentMessage;
    static float elapsedTime;
    static float fadeTimer;          // For auto-hide after success
    static bool enabled;             // Controlled by config.ini
    static bool registered;          // Track if callback is registered
};
