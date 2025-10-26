# Changelog

All notable changes to the PX4-XPlane plugin will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
