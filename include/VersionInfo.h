/**
 * @file VersionInfo.h
 * @brief Centralized Version and Copyright Information for PX4-XPlane Plugin
 *
 * This header provides easy-to-update version constants for the entire plugin.
 * Update these values for new releases - they automatically propagate throughout the UI.
 *
 * @author Alireza Ghaderi
 * @copyright Copyright (c) 2025 Alireza Ghaderi. All rights reserved.
 * @license MIT License
 * @version 3.0.0 (Phase 3 - Production Ready)
 * @url https://github.com/alireza787b/px4xplane
 */

#ifndef VERSION_INFO_H
#define VERSION_INFO_H

namespace PX4XPlaneVersion {

    // =================================================================
    // VERSION INFORMATION - Update these for new releases
    // =================================================================

    /** Plugin version string - displayed in UI and About dialog */
    constexpr const char* VERSION = "3.4.1";

    /** Version phase description */
    constexpr const char* PHASE = "Stable";

    /** Copyright year - update annually */
    constexpr const char* YEAR = "2025";

    /** Plugin build number - increment for each build */
    constexpr const char* BUILD = "008";

    // =================================================================
    // AUTHOR AND PROJECT INFORMATION  
    // =================================================================

    /** Plugin author */
    constexpr const char* AUTHOR = "Alireza Ghaderi";

    /** Project license */
    constexpr const char* LICENSE = "MIT License";

    /** GitHub repository URL */
    constexpr const char* REPOSITORY_URL = "https://github.com/alireza787b/px4xplane";

    /** Project description */
    constexpr const char* DESCRIPTION = "Software-in-the-Loop interface for PX4 and X-Plane";

    // =================================================================
    // PLUGIN IDENTIFIERS - Used by X-Plane
    // =================================================================

    /** Plugin signature for X-Plane */
    constexpr const char* PLUGIN_SIGNATURE = "alireza787b.px4xplane";

    /** Plugin name shown in X-Plane */
    constexpr const char* PLUGIN_NAME = "PX4 to X-Plane";

    // =================================================================
    // FORMATTED STRINGS - Auto-generated from above constants
    // =================================================================

    /** Full version string with phase information */
    inline const char* getFullVersionString() {
        static char versionBuffer[256];
        snprintf(versionBuffer, sizeof(versionBuffer),
            "v%s (%s)", VERSION, PHASE);
        return versionBuffer;
    }

    /** Complete copyright string */
    inline const char* getCopyrightString() {
        static char copyrightBuffer[256];
        snprintf(copyrightBuffer, sizeof(copyrightBuffer),
            "Copyright (c) %s %s. All rights reserved.", YEAR, AUTHOR);
        return copyrightBuffer;
    }

    /** Window title for main data inspector */
    inline const char* getMainWindowTitle() {
        static char titleBuffer[256];
        snprintf(titleBuffer, sizeof(titleBuffer),
            "PX4-XPlane Data Inspector %s", getFullVersionString());
        return titleBuffer;
    }

    /** About dialog title */
    inline const char* getAboutWindowTitle() {
        static char titleBuffer[256];
        snprintf(titleBuffer, sizeof(titleBuffer),
            "About PX4-XPlane Interface %s", getFullVersionString());
        return titleBuffer;
    }

    /** Footer string for main window */
    inline const char* getFooterString() {
        static char footerBuffer[256];
        snprintf(footerBuffer, sizeof(footerBuffer),
            "PX4-XPlane %s | %s | %s",
            VERSION, REPOSITORY_URL, getCopyrightString());
        return footerBuffer;
    }

    // =================================================================
    // BUILD INFORMATION - For debugging and support
    // =================================================================

    /** Complete build information string */
    inline const char* getBuildInfo() {
        static char buildBuffer[512];
        snprintf(buildBuffer, sizeof(buildBuffer),
            "PX4-XPlane v%s Build %s\n%s\n%s\nBuilt with X-Plane SDK XPLM300+\n%s",
            VERSION, BUILD, getCopyrightString(), LICENSE, REPOSITORY_URL);
        return buildBuffer;
    }

} // namespace PX4XPlaneVersion

#endif // VERSION_INFO_H