# Changelog

All notable changes to the PX4-XPlane plugin will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
