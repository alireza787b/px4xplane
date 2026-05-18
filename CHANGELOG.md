# Changelog

All notable changes to the PX4-XPlane plugin will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [3.4.11] - 2026-05-18

### Fixed

- Disabled Alia lift-prop auto braking by default after `alia-test5` proved the
  v3.4.10 brake policy could leave lift props feathered/stopped during
  transition recovery.
- Reworked optional prop-brake policy with all-motor apply gating, immediate
  all-motor release, dwell time, optional true-airspeed gate, and experimental
  failure-dataref use disabled by default.
- Released active prop brakes during actuator stale/reset/disconnect cleanup.

### Changed

- Narrowly adjusted Alia front-transition margin: `VT_ARSP_TRANS=46.0`,
  `VT_F_TRANS_DUR=45.0`, and `VT_F_TR_OL_TM=55.0`.
- Updated Alia simulated accel offset defaults to the latest no-warning
  test5-start calibration values.

---

## [3.4.10] - 2026-05-18

### Changed

- Reverted the unproven Alia v3.4.9 TECS, throttle, RTL descent, and
  back-transition tuning to the last better-tracking baseline.
- Raised X-Plane SITL barometer height noise to `1.0 m` across the X-Plane PX4
  airframes after ULog evidence showed `0.05 m` was still overconfident for
  multi-meter VTOL/FW baro innovations.
- Seeded Alia simulated accelerometer offsets so fresh `distclean` runs do not
  start from the zero-offset state that produced the high-accelerometer-bias
  warning in `alia-test4`.

### Fixed

- Strengthened optional prop brakes: configured lift props are now feathered,
  commanded to zero prop speed, and kept seized while braked instead of relying
  on the prop-separation failure trick alone.

---

## [3.4.9] - 2026-05-18

### Changed

- Added a ULog-guided Alia-250 tuning slice to reduce fixed-wing altitude
  phugoid behavior and back-transition climb/overshoot.
- Seeded PX4 simulated barometer device IDs in X-Plane airframe files so the
  intended disabled backup barometer priority applies on a fresh SITL rootfs.
- Relaxed X-Plane SITL barometer fusion noise from an overconfident 0.003 m to
  0.05 m.

### Fixed

- Added hysteresis to optional prop-brake activation so X-Plane failure datarefs
  do not chatter around exactly zero throttle.
- Clamped negative indicated airspeed before calculating HIL differential
  pressure.

---

## [3.4.8] - 2026-05-07

### Fixed

- Fixed Windows MSVC release-build portability by defining `NOMINMAX` and
  guarding a `std::numeric_limits<uint64_t>::max()` call from Windows `max`
  macro expansion.

### Unchanged

- Same config safety/editor/runtime behavior as v3.4.7.
- Alia PX4 parameters remain frozen at the v3.4.6 successful baseline.
- No bridge sensor sign/unit, GPS, HIL state, quaternion, actuator mapping, or
  prop-brake policy change is included in this release.

---

## [3.4.7] - 2026-05-07

### Added

- Added config safety validation foundation:
  - stale actuator mappings are cleared on reload
  - actuator outputs are finite-checked and clamped
  - invalid output ranges are skipped and logged
  - stale PX4 actuator input zeros configured actuator datarefs
  - inbound MAVLink receive drains a bounded multi-packet budget per frame
- Added `tools/validate_config.py` and tests for `config.ini` channel mappings,
  global field types/ranges, and prop-brake motor indices.
- Added runtime config validation status in the X-Plane UI.
- Added `Advanced` menu items for config validation, reload+validate, bridge
  diagnostics, docs location, and config editor location.
- Added `config/config_schema.json` as schema metadata for config fields,
  reload policy, supported channel types, and editor tooling.
- Added a static schema-backed config editor:
  - `docs/config-editor.html`
  - `docs/assets/config-editor.js`
  - packaged under `px4xplane/docs/`
- Added docs and reports for the config safety, validation UI, schema, and
  editor slices.

### Changed

- Moved bridge diagnostics from the normal menu into the Advanced menu.
- Packaged `config_schema.json` next to `config.ini`.
- Documented that `config.ini` remains the runtime format; JSON schema is
  metadata for validation and editor UX.

### Fixed

- Fixed several config/mapping hazards that could masquerade as tuning problems:
  stale channel mappings, non-finite actuator values, invalid ranges, stale PX4
  actuator streams, and missing float-array datarefs.

### Unchanged

- Alia PX4 parameters remain frozen at the v3.4.6 successful baseline.
- No bridge sensor sign/unit, GPS, HIL state, quaternion, or prop-brake policy
  change is included in this release.

---

## [3.4.6] - 2026-05-07

### Changed

- Restored the Alia fixed-wing/transition guidance defaults to the values
  observed in the full successful `alia-sitl1` ULog: TECS damping/time constants,
  NPFG period/damping, roll limit, and RTL/loiter radii.
- Updated the Alia test workflow to require a parameter-reset/distclean step
  when changing airframe defaults, because PX4 SITL can otherwise keep saved
  parameters across runs.

### Fixed

- Corrected the previous release assumption that the plugin-repo Alia file was
  the tested source of truth. The successful ULog and the PX4 fork state are now
  treated as the Alia baseline until controlled tuning proves otherwise.

---

## [3.4.5] - 2026-05-06

### Changed

- Cleaned the Alia airframe troubleshooting comments so the guidance matches the
  current TECS, NPFG, transition, and RTL/circle baseline values used for the
  next X-Plane 12 retest.

---

## [3.4.4] - 2026-05-06

### Fixed

- Fixed MSVC Windows release builds by protecting new diagnostics
  `std::min`/`std::max` calls from Windows `min`/`max` macros. This is a build
  portability hotfix; the validated v3.4.3 Alia sensor contract and tuning
  remain unchanged.

---

## [3.4.3] - 2026-05-06

### Added

#### Alia X-Plane 12 Validation Follow-Up
- Added bridge health diagnostics for effective sensor/GPS/state/RC rates,
  frame-period min/p50/p95/max, late/missed sends, timestamp monotonicity, and
  target-rate-vs-frame-rate warnings.
- Added `tools/replay_truth_capture.py` for deterministic core-contract replay
  from XPlaneTruthCapture folders or zip files.
- Added `tools/analyze_ulog_estimator.py` for ULog estimator-topic summaries and
  event-window analysis around RTL, barometer switches, landing, and disarm.
- Added replay unit tests for the current v3.4.3 sensor contract.
- Added `docs/ALIA_XPLANE12_TEST.md` as the next X-Plane 12 Alia run card.

### Changed

- Preserved the validated v3.4.3 sensor contract:
  `xacc=-g_axil`, `yacc=+g_side`, `zacc=-g_nrml`, GPS velocity from local
  velocity, true course-over-ground from local velocity, and HIL_STATE
  acceleration in milli-g.
- Removed outgoing HIL_SENSOR timestamp jitter so emitted MAVLink timestamps stay
  globally monotonic across messages in a frame.
- Reduced default debug log noise while keeping compact bridge diagnostics on.
- Cleaned `5020_xplane_alia250` comments and separated bridge-contract settings
  from aircraft-performance tuning settings.
- Made a conservative first Alia path/altitude tuning pass for TECS, NPFG, roll
  limit, loiter radius, and RTL loiter radius based on the successful May 6
  X-Plane 12 run.

### Fixed

- Fixed disconnect cleanup while waiting for PX4 SITL so the listening socket is
  closed when connection wait is cancelled or the plugin is disabled.

---

## [3.4.2] - 2025-02-01

### Added

#### FPS Warning HUD Indicator
- **Low FPS detection**: Shows subtle warning when X-Plane FPS drops below threshold
  - X-Plane frame rate directly affects sensor data quality sent to PX4
  - Low FPS causes reduced sensor update rate (EKF2 expects ~200Hz, gets X-Plane FPS)
  - Can lead to increased estimation noise and potential EKF2 warnings

- **Non-intrusive design**:
  - Small indicator in bottom-left corner (out of the way)
  - Color-coded severity: yellow for warning, orange-red for critical
  - Auto-hides when FPS recovers (5-second persistence prevents flickering)

- **Configurable via config.ini**:
  - `fps_warning_enabled`: Enable/disable the warning (default: true)
  - `fps_warning_threshold`: FPS below this triggers warning (default: 50)

- **Technical implementation**:
  - Rolling average FPS calculation (exponential moving average, α=0.1)
  - FPS sampled every 0.5 seconds for efficiency
  - Critical threshold at 60% of warning threshold

### Technical Details

#### Files Modified
- `include/ConnectionStatusHUD.h`: Added FPS monitoring state and methods
- `src/ConnectionStatusHUD.cpp`: FPS calculation and warning drawing
- `include/ConfigManager.h`: Added fps_warning_enabled, fps_warning_threshold
- `src/ConfigManager.cpp`: Load and validate FPS warning config
- `config/config.ini`: Added FPS warning configuration section
- `src/px4xplane.cpp`: Notify HUD when actively connected
- `include/VersionInfo.h`: VERSION 3.4.2, BUILD 009

---

## [3.1.1] - 2025-01-03

### Fixed

#### Critical Menu Handler Bug
- **Airframe selection not working**: Fixed airframe menu not switching configurations
  - Root cause: `refreshAirframesMenu()` was passing heap pointers instead of integer indices
  - Memory leak: `new std::string()` allocations never freed
  - Fixed: Changed to indexed for-loop passing `(void*)(intptr_t)i`
  - Also clicking airframes was incorrectly triggering "Show Data" window

#### Menu System Improvements
- **Added defensive constants**: `MENU_REF_MAIN` and `MENU_REF_AIRFRAMES` for clarity
- **Improved menu_handler()**: Explicit comparison instead of NULL check
- **Better error handling**: Added bounds checking and debug logging

#### PX4 SITL Reconnection Hang
- **Lockstep scheduler stuck after aircraft change**: Fixed reconnection hanging at "setting initial absolute time"
  - Root cause: Static timing variables (`lastSensorTime`, `lastGPSTime`, etc.) persisted across disconnect cycles
  - Timestamp jumps: Old timestamps (e.g., 1655s) mixed with new session (e.g., 5ms) confused lockstep scheduler
  - Solution: Added `g_needsTimingReset` flag to coordinate state cleanup
  - Reset logic in: `sendHILSensor()`, `sendHILGPS()`, `sendHILStateQuaternion()`, `computeAcceleration()`
  - Enhanced `MAVLinkManager::reset()` with comprehensive timing state cleanup

### Technical Details

#### Files Modified
- `src/px4xplane.cpp`: Menu handler fixes, defensive constants
- `src/MAVLinkManager.cpp`: Timing state reset logic, comprehensive documentation
- `include/VersionInfo.h`: VERSION 3.1.1, BUILD 002

---

## [3.1.0] - 2025-01-26

### Added

#### Professional 5-Tab UI System
- **Complete UI overhaul**: Replaced old overflow-prone single window with professional tabbed interface
- **5 organized tabs**:
  - **CONNECTION**: Status, timestamps, frame rate, connection details
  - **POSITION**: GPS coordinates, attitude (roll/pitch/yaw), angular velocities
  - **SENSORS**: IMU data, airspeed, environmental sensors
  - **CONTROLS**: Aircraft configuration, all 16 HIL actuator channels with live values
  - **MIXING**: Airframe configuration with real-time HIL data visualization
- **Per-tab scrolling**: Each tab scrolls independently - no more overflow
- **High-DPI support**: Professional rendering on all displays
- **Mouse wheel scrolling**: Smooth navigation through data

#### Performance Improvements
- **SITL rate increased to 400Hz**: HIL_SENSOR messages now sent at 400Hz
  - Better sensor fusion and state estimation
  - Reduced latency in control loop
  - Matches high-performance SITL standards

#### Documentation Overhaul
- **docs/DEVELOPER.md**: New streamlined developer guide
  - Quick reference workflow
  - Concise versioning guide (MAJOR.MINOR.PATCH)
  - CI/CD trigger reference
  - Common tasks and best practices
- **Consolidated documentation**: All docs moved to `docs/` folder
- **Removed redundancy**: Eliminated duplicate documentation

#### Configuration
- **Cleaned parameter files**: All 5 airframe parameter files reorganized
  - Removed obsolete parameters
  - Added clear documentation comments
  - Consistent formatting across all aircraft

### Fixed

#### Critical UI Bug
- **"Show Data" menu button**: Fixed handler not responding to clicks
  - Root cause: Incorrect `in_menu_ref` check
  - Now correctly identifies main menu vs submenu items
  - Added detailed debug logging

#### UI Integration
- **UIHandler not initialized**: Old drawing function was being used instead of new tabbed UI
  - Integrated `UIHandler::initialize()` in plugin startup
  - Changed to `UIHandler::drawMainWindow()`
  - Added mouse handlers for tab interaction and scrolling
  - Removed old drawing functions

### Changed

#### Menu Handler Improvements
- Better error handling with detailed debug output
- Clearer logic for menu item detection
- Improved airframe selection validation

### Technical Details

#### Files Modified
- `src/px4xplane.cpp`: Menu handler fix, UIHandler integration
- `config/config.ini`: HIL_SENSOR rate 250Hz → 400Hz
- `config/px4_params/*`: All 5 airframe files cleaned
- `include/VersionInfo.h`: VERSION 3.1.0, BUILD 001
- `CMakeLists.txt`: VERSION 3.1.0
- `docs/DEVELOPER.md`: New streamlined guide
- `README.md`: Updated workflow documentation link
- Moved: `BUILD_SYSTEM.md` → `docs/BUILD_SYSTEM.md`
- Removed: `VERSIONING.md`, `docs/GITHUB_ACTIONS.md`

---

## [3.0.1] - 2025-01-26

### Added

#### Documentation
- **VERSIONING.md**: Comprehensive developer guide for versioning and releases
  - Step-by-step release process with semantic versioning
  - Branch strategy (develop/master/tags)
  - Common scenarios (patch/minor/major releases)
  - GitHub Actions CI/CD reference with workflow triggers
  - Best practices and troubleshooting guide

#### README Improvements
- Added v3.0 video tutorial placeholder
- Highlighted major v3.0 improvements:
  - EKF2 stability fixes (altitude drift resolution)
  - Multi-threaded communication architecture
  - Improved state estimators (GPS/Baro/IMU fusion)
- Added PX4 official integration status section
- Reference to automated setup script for easy installation

### Fixed

#### Release Workflow
- **Windows Release Upload**: Enhanced build output verification
  - Added verification step to check build structure
  - Explicit plugin binary existence validation
  - Detailed error messages for debugging
  - Better error handling for missing directories
- Improved diagnostics for troubleshooting failed releases

### Technical Details

#### Files Modified
- `VERSIONING.md`: New comprehensive versioning guide
- `README.md`: Enhanced v3.0 feature highlights, PX4 integration status
- `.github/workflows/release.yml`: Build verification and error handling
- `include/VersionInfo.h`: VERSION 3.0.1, BUILD 002
- `CMakeLists.txt`: VERSION 3.0.1

---

## [3.0.0] - 2025-01-26

### 🎉 Major Release - Breaking Changes

This is a major version release with significant structural changes to comply with X-Plane SDK standards and implement professional CI/CD workflows.

### ⚠️ BREAKING CHANGES

#### Plugin Structure Reorganization
- **config.ini moved to 64/ folder**: Now located WITH the binary instead of parent folder
- **PX4 parameters organized**: Moved to dedicated `px4_airframes/` subdirectory
- **Installation method changed**: Must copy entire `px4xplane/` folder (not individual files)

**Migration Required**: Remove old v2.x installation and reinstall with new structure.

**Old Structure (v2.x)**:
```
px4xplane/
├── config.ini                    ← Root folder
├── 64/win.xpl
└── 5001_xplane_cessna172         ← Scattered params
```

**New Structure (v3.0.0)**:
```
px4xplane/
├── 64/
│   ├── win.xpl
│   └── config.ini                ← WITH binary
├── px4_airframes/                ← Organized
│   ├── 5001_xplane_cessna172
│   ├── 5002_xplane_tb2
│   ├── 5010_xplane_ehang184
│   ├── 5020_xplane_alia250
│   └── 5021_xplane_qtailsitter
└── README.md
```

### Added

#### Automated CI/CD System
- **GitHub Actions workflows**: Automatic builds on every push to master
- **Multi-platform builds**: Windows, Linux, macOS built in parallel (5-15 minutes)
- **Automated releases**: Push version tag to create release with pre-built binaries
- **Artifact retention**: 90-day storage for testing builds
- **Status badges**: Real-time build status on README
- **develop branch strategy**: No builds on develop (saves CI minutes), only master

#### Cross-Platform Build System
- **Unified CMake**: Single build system for all platforms (Windows/Linux/macOS)
- **Native Makefiles**: `Makefile.linux` and `Makefile.macos` for direct builds
- **Build consistency**: All methods (CMake, Visual Studio, Make) produce identical structure
- **Universal Binary (macOS)**: Single binary supports Intel x86_64 + Apple Silicon ARM64

#### Documentation
- **CHANGELOG.md**: Comprehensive version history (this file)
- **docs/GITHUB_ACTIONS.md**: Complete CI/CD workflow guide for developers
- **docs/BUILD_SYSTEM.md**: Technical build system documentation
- **Enhanced README.md**: Status badges, simplified installation, release workflow

### Fixed

#### Critical Build Failures
- **Linux case-sensitive includes**: Fixed `ConfigReader.h` → `configReader.h`
- **macOS missing headers**: Added `#include <cstring>` for `strlen()` in UIConstants.h
- **macOS linker errors**: Removed incorrect XPLM/XPWidgets framework linking
- **macOS deprecated warnings**: Updated `-undefined suppress` → `-undefined dynamic_lookup`
- **Windows output path**: Fixed MSVC multi-config directory nesting issue

#### Code Quality
- **Uninitialized variables**: Zero-initialized RC input struct (`hil_rc_inputs = {}`)
- **Compiler warnings**: Fixed unused variable warnings across all platforms

### Changed

#### macOS Plugin Linking
- **Standard X-Plane approach**: Using flat namespace with dynamic lookup
- **Framework cleanup**: Removed non-existent XPLM/XPWidgets frameworks
- **Runtime symbol resolution**: X-Plane provides XPLM symbols at load time
- **OpenGL only**: Only link OpenGL framework (required for rendering)

#### Visual Studio Project
- **Debug build fixed**: config.ini now copies to 64/ folder (was parent folder)
- **px4_airframes/ added**: Both Debug and Release now copy organized param files
- **Consistency achieved**: Debug and Release produce identical structures
- **Post-build messages**: Enhanced progress output with clear file locations

#### CMakeLists.txt
- **Version**: Updated project(px4xplane VERSION 3.0.0)
- **File organization**: config.ini → 64/, params → px4_airframes/
- **Post-build automation**: Directory creation and file copying
- **Windows MSVC fix**: Per-configuration output directories

### Removed

#### Dependencies
- **GeographicLib submodule**: Removed (not used in project code)

### Technical Details

#### Git Submodules
- **Eigen**: Added as proper submodule (linear algebra library)
- **Workflow configuration**: `actions/checkout@v4` with `submodules: recursive`

#### Files Modified
- `CMakeLists.txt`: Version 3.0.0, file organization, Windows MSVC fix
- `px4-xplane.vcxproj`: Debug/Release consistency, px4_airframes/ support
- `.github/workflows/build.yml`: Build only on master branch
- `.github/workflows/release.yml`: Automated release creation
- `include/VersionInfo.h`: VERSION 3.0.0, BUILD 001
- `src/ConnectionStatusHUD.cpp`: Added `<cstring>` header
- `src/px4xplane.cpp`: Fixed case-sensitive include
- `src/MAVLinkManager.cpp`: Zero-initialized RC inputs

### Migration Guide

#### Upgrading from v2.x to v3.0.0

1. **Remove old installation**:
   ```bash
   rm -rf "X-Plane 12/Resources/plugins/px4xplane"
   ```

2. **Download v3.0.0**:
   - Visit: https://github.com/alireza787b/px4xplane/releases/tag/v3.0.0
   - Download ZIP for your platform

3. **Install new version**:
   ```bash
   unzip px4xplane-{platform}-v3.0.0.zip
   cp -r px4xplane/ "X-Plane 12/Resources/plugins/"
   ```

4. **Verify**:
   - Launch X-Plane
   - Plugins → Plugin Admin → px4xplane should appear
   - Check PX4-SITL menu is available

5. **Config location changed**:
   - Old: `px4xplane/config.ini`
   - New: `px4xplane/64/config.ini`
   - Edit new location if custom configuration needed

---

## [2.5.1] - 2025-01-19

### Fixed
- **Height Estimate Stability**: Resolved altitude drift and EKF2 reset issues affecting flight stability
  - Reduced GPS altitude noise from σ=0.5m to σ=0.15m (matches high-quality GPS vertical accuracy)
  - GPS altitude now varies smoothly ±30cm instead of jumping ±1m
  - EKF2 properly prioritizes barometer (±3mm) over GPS for altitude estimation
  - Eliminated false climb/descent detections caused by GPS noise
  - Height estimate resets reduced from ~10/hour to rare occurrences

- **EKF2 Parameter Consistency**: Updated EKF2_GPS_P_NOISE across all aircraft configurations
  - Changed from 0.01 to 0.15 in: Alia250, eHang184, Cessna172, TB2, QTailsitter
  - Added comprehensive documentation explaining GPS-barometer sensor fusion
  - Parameter now accurately reflects GPS vertical accuracy (15cm)

- **PX4 Parameter Compatibility**: Removed non-existent parameters causing startup errors
  - Removed VT_F_TRANS_RAMP (VTOL transition ramp - version-specific)
  - Removed VT_DWN_PITCH_MAX (VTOL pitch limit - version-specific)
  - Removed FW_T_SPD_OMEGA (TECS speed filter - version-specific)
  - Removed FW_T_I_GAIN_THR (TECS throttle integrator - version-specific)
  - Added explanatory comments for each removed parameter

### Added
- **Connection Status HUD**: Professional on-screen feedback for SITL connection attempts
  - Real-time progress indicator with elapsed time (0-30 seconds)
  - Visual connection states: WAITING (yellow), CONNECTED (green), TIMEOUT (red), ERROR (orange)
  - Auto-fade success notification after 3 seconds
  - 30-second timeout with actionable error messages
  - Top-center positioning following X-Plane UI standards
  - Minimal, informative design with main text + subtitle format
  - Platform-specific OpenGL rendering (Windows/Mac/Linux)
  - Configurable via `show_connection_status_hud` in config.ini (enabled by default)

- **Configuration Options**:
  - `show_connection_status_hud`: Enable/disable connection HUD overlay
  - `debug_log_ekf_innovations`: Log EKF2 innovation data for diagnostics

### Changed
- **Performance Optimization**: Debug logging disabled by default for production use
  - `debug_verbose_logging`: false (was true)
  - `debug_log_sensor_timing`: false (was true)
  - `debug_log_sensor_values`: false (was true)
  - Reduces X-Plane Log.txt file size and improves performance
  - Can be re-enabled for development/tuning via config.ini

- **GPS Sensor Model**: Improved realism and EKF2 compatibility
  - GPS altitude noise: σ=0.5m → σ=0.15m (3.3× improvement)
  - Matches u-blox F9P high-quality GPS vertical accuracy specifications
  - 95% of GPS readings now within ±30cm (was ±1m)
  - Better alignment between sensor noise and EKF2 parameters

### Technical Details

#### Height Stability Root Cause Analysis
The height estimate drift issue was caused by a mismatch between GPS sensor noise and EKF2 parameters:

**Problem:**
- GPS altitude noise: σ=0.5m → random jumps of ±1m
- EKF2_GPS_P_NOISE: 0.01 → EKF2 trusted GPS as having 1cm accuracy
- EKF2 interpreted ±1m GPS jumps as actual altitude changes
- Result: False climb/descent detection → height estimate reset

**Solution:**
- GPS altitude noise: σ=0.15m → smooth variation of ±30cm
- EKF2_GPS_P_NOISE: 0.15 → EKF2 expects 15cm GPS uncertainty
- EKF2 now correctly weights barometer (±3mm) > GPS (±30cm)
- Result: Stable height estimate with rare resets

#### Connection HUD Implementation
- Uses X-Plane SDK `XPLMRegisterDrawCallback()` for HUD rendering
- Non-blocking draw callback executed every frame during `xplm_Phase_Window`
- Platform-specific OpenGL includes for Windows/Mac/Linux compatibility
- Static class design with no dynamic memory allocation
- Enum class `Status` with scoped values to avoid Windows `ERROR` macro conflict
- Text centering using approximate character width calculation
- Background opacity: 85% for optimal contrast while maintaining transparency

#### Files Modified
- `src/MAVLinkManager.cpp`: GPS altitude noise model (line 43)
- `config/px4_params/5010_xplane_ehang184`: EKF2_GPS_P_NOISE (line 350)
- `config/px4_params/5001_xplane_cessna172`: EKF2_GPS_P_NOISE (line 230)
- `config/px4_params/5002_xplane_tb2`: EKF2_GPS_P_NOISE (line 286)
- `config/px4_params/5021_xplane_qtailsitter`: EKF2_GPS_P_NOISE (line 174)
- `config/px4_params/5020_xplane_alia250`: EKF2_GPS_P_NOISE (line 297), removed invalid params
- `src/ConnectionStatusHUD.cpp`: New HUD implementation
- `include/ConnectionStatusHUD.h`: New HUD header
- `src/px4xplane.cpp`: HUD integration (lines 31, 205-206, 553-592, 722)
- `include/ConfigManager.h`: New config flags (lines 109, 112)
- `src/ConfigManager.cpp`: Config loading (lines 123-182)
- `config/config.ini`: Updated configuration (lines 39-48)
- `px4-xplane.vcxproj`: Added ConnectionStatusHUD to build

---

## [2.5.0] - 2025-01-15

### Fixed
- **Critical Sensor Stability**: Resolved BARO STALE errors and vertical velocity oscillation
  - Increased HIL_SENSOR update rate from 100Hz to 250Hz
  - Enhanced barometer noise modeling (3cm std dev)
  - Synchronized GPS-Barometer altitude quantization (5mm resolution)
  - Relaxed EKF2 innovation gates (BARO_GATE, GPS_V_GATE: 8.0)
  - Tuned COM_ARM_EKF_VEL parameter (0.8)

### Added
- Enhanced User Interface with professional high-contrast design
- Cross-platform support for Linux and macOS (native makefiles)
- Portability improvements with relative paths

### Changed
- Project structure improvements for better maintainability
- External dependencies properly excluded from version control

---

## [2.0.0] - 2024-12-26

### Added
- Multi-airframe support (Ehang 184, Alia 250, Cessna 172, Bayraktar TB2)
- In-menu airframe selection
- Automated setup script for WSL/Linux/macOS
- Detailed configuration instructions in config.ini
- Pre-loaded PX4 parameters

### Fixed
- Multiple sensor problems and inconsistencies

---

## [1.0.0] - 2024-11-15

### Added
- Initial release of PX4-XPlane plugin
- Basic MAVLink HIL integration
- Support for quadcopter airframes
- Configuration file system
