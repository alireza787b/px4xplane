# PX4-XPlane - PX4 X-Plane Integration Plugin

[![Build Status](https://github.com/alireza787b/px4xplane/actions/workflows/build.yml/badge.svg)](https://github.com/alireza787b/px4xplane/actions/workflows/build.yml)
[![Release](https://img.shields.io/github/v/release/alireza787b/px4xplane)](https://github.com/alireza787b/px4xplane/releases)
[![License](https://img.shields.io/github/license/alireza787b/px4xplane)](LICENSE)
[![Platform](https://img.shields.io/badge/platform-Windows%20%7C%20Linux%20%7C%20macOS-blue)](https://github.com/alireza787b/px4xplane)

This project establishes a robust connection between X-Plane and PX4 SITL (Software In The Loop) to simulate drone flight in the X-Plane environment. Our goal is to deliver a realistic simulation experience where PX4 can control various drones within X-Plane, and with our continuous improvements, it's getting better all the time.

## Latest Release - Version 3.0.0 (January 2025)

### 🎥 Video Tutorial

**🆕 NEW VIDEO COMING SOON:** A comprehensive PX4-XPlane 3.0 tutorial showcasing the major improvements, enhanced EKF stability, multi-threaded communication architecture, and automated setup workflow will be available soon! Stay tuned for the updated demonstration.

**Current Video:** Check out our [v2.0.0 demo video](https://youtu.be/oQTlBVXqR04) for installation guide and basic features.

### 🎉 Major Release - Breaking Changes

**⚠️ BREAKING CHANGES**: Version 3.0.0 introduces significant structural changes for X-Plane SDK compliance and professional CI/CD workflows. **Migration required from v2.x.**

### What's New in Version 3.0.0

#### 🚁 Core Simulation Improvements

- **🎯 EKF2 Stability Fixes** (Critical):
  - Resolved altitude drift and height estimate reset issues
  - GPS altitude noise reduced from σ=0.5m to σ=0.15m (3.3× improvement)
  - EKF2_GPS_P_NOISE properly tuned (0.01 → 0.15) across all aircraft
  - Barometer (±3mm) now correctly prioritized over GPS (±30cm)
  - Height estimate resets reduced from ~10/hour to rare occurrences
  - Eliminated false climb/descent detections caused by GPS noise

- **⚡ Multi-Threaded Communication Architecture**:
  - Robust, stable PX4 ↔ X-Plane MAVLink communication
  - Asynchronous sensor data processing with thread-safe queues
  - HIL_SENSOR update rate increased to 250Hz (matches PX4 Gazebo standard)
  - Enhanced barometer noise modeling (3cm std dev) for proper liveness detection
  - Synchronized GPS-Barometer altitude quantization (5mm resolution)
  - Prevents sensor timeout/STALE errors

- **📡 Improved State Estimators**:
  - EKF2 innovation gates relaxed for SITL environment (BARO_GATE: 8.0, GPS_V_GATE: 8.0)
  - COM_ARM_EKF_VEL tuned (0.8) to prevent false preflight velocity errors
  - Better sensor fusion between GPS, barometer, IMU
  - Stable attitude, position, and velocity estimation

#### 🤖 Development & CI/CD

- **🤖 Automated CI/CD System**: GitHub Actions workflows for cross-platform builds:
  - Automatic builds on every push to master (Windows, Linux, macOS in parallel)
  - Automated releases with pre-built binaries via version tags
  - 90-day artifact retention for testing builds
  - Real-time build status badges on README

- **📁 Plugin Structure Reorganization** (X-Plane SDK Standards):
  - **config.ini moved to 64/ folder** (WITH the binary, not parent folder)
  - **PX4 parameters organized in px4_airframes/ subdirectory**
  - Consistent structure across all build methods (CMake, Visual Studio, Make)
  - Standard X-Plane fat plugin format (SDK 2.0)

- **🔧 Critical Build Fixes**:
  - Fixed Linux case-sensitive include errors
  - Fixed macOS missing headers and linker errors
  - Fixed Windows MSVC output path issues
  - Proper Eigen library submodule integration

- **🍎 macOS Improvements**:
  - Fixed framework linking (using flat namespace with dynamic lookup)
  - Universal Binary support (Intel x86_64 + Apple Silicon ARM64)
  - Proper X-Plane plugin runtime symbol resolution

- **📦 Build System Consistency**:
  - CMakeLists.txt updated to v3.0.0 with file organization
  - Visual Studio Debug/Release now produce identical structures
  - All platforms build successfully via GitHub Actions

#### 🖥️ User Interface

- **Connection Status HUD**: Professional on-screen feedback for SITL connection
  - Real-time progress indicator with elapsed time (0-30 seconds)
  - Visual states: WAITING (yellow), CONNECTED (green), TIMEOUT (red), ERROR (orange)
  - Auto-fade success notification after 3 seconds
  - Configurable via `show_connection_status_hud` in config.ini

### ⚠️ Migration from v2.x to v3.0.0

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

**To upgrade**: Remove old v2.x installation and download v3.0.0 from [Releases](https://github.com/alireza787b/px4xplane/releases/tag/v3.0.0). See [CHANGELOG.md](CHANGELOG.md) for complete migration guide.

### 🚀 PX4 Official Integration Status

**Coming Soon**: px4xplane is being prepared for official integration into the main PX4-Autopilot repository. Until the pull request is merged:

- **Use the automated setup script** (recommended): Installs PX4 SITL with px4xplane support automatically
  ```bash
  cd ~ && curl -O https://raw.githubusercontent.com/alireza787b/px4xplane/master/setup/setup_px4_sitl.sh && bash setup_px4_sitl.sh
  ```
- **Or use the forked PX4 repo**: [PX4-Autopilot-Me](https://github.com/alireza787b/PX4-Autopilot-Me) (branch: `px4xplane-sitl`)
- **Full installation guides**: See [Usage Guide](#usage-guide) below for detailed instructions

Once merged with official PX4, you'll be able to use the standard PX4-Autopilot repository directly.

---

## Previous Release - Version 2.5.2 (January 2025)

### 🎥 Video Tutorials

[![PX4 X-Plane Plugin v2.0.0 Demo](https://github.com/user-attachments/assets/3fb6d042-20d9-4589-8a79-44f434d5870d)](https://youtu.be/oQTlBVXqR04)

**Current Video:** Check out our v2.0.0 demo video where we walk you through the simplified setup process, showcase the multi-airframe support, and test various drones like the Ehang 184, Alia 250, Cessna 172, and Bayraktar TB2.

👉 [Watch the current demo and installation guide here!](https://youtu.be/oQTlBVXqR04)

**🆕 NEW VIDEO COMING SOON:** A comprehensive PX4-XPlane 2.5 tutorial showcasing the latest improvements, enhanced UI, and cross-platform support will be available soon! Stay tuned for the updated demonstration.

### What's New in Version 2.5.2

- **🔨 Unified Cross-Platform Build System**: Complete build system overhaul with CMake support:
  - **CMakeLists.txt**: Modern cross-platform build configuration (Windows, Linux, macOS)
  - **Makefile.linux**: Native Linux builds with GCC/Clang
  - **Makefile.macos**: Native macOS builds with Universal Binary support (Intel + Apple Silicon)
  - Single unified build process across all platforms
  - Easy for contributors and developers
- **📖 Comprehensive Build Documentation**: New detailed [Build Guide](docs/BUILD.md) with:
  - Step-by-step build instructions for all platforms
  - Prerequisites and troubleshooting
  - IDE integration guides (VS Code, CLion, Xcode)
  - Advanced build options
- **🧹 Project Organization**: Clean separation of build artifacts with proper .gitignore patterns

### Previous Release - Version 2.5.1 (January 2025)

- **🎯 Height Estimate Stability Fix**: Resolved altitude drift and EKF2 reset issues:
  - Reduced GPS altitude noise from 0.5m to 0.15m (matches high-quality GPS vertical accuracy)
  - Updated EKF2_GPS_P_NOISE parameter from 0.01 to 0.15 across all aircraft configurations
  - Eliminated ±1m GPS altitude jumps that caused EKF2 to incorrectly detect climbing/descending
  - Height estimate now stable with barometer (±3mm) properly prioritized over GPS (±30cm)
- **🖥️ Connection Status HUD**: Professional HUD-style overlay for connection feedback:
  - Real-time connection progress indicator with elapsed time display
  - 30-second timeout with clear warning messages
  - Auto-fade success notification after 3 seconds
  - Clean, minimal design following X-Plane standards (top-center positioning)
  - Configurable via `show_connection_status_hud` in config.ini
- **🔧 Parameter Cleanup**: Removed non-existent PX4 parameters causing errors:
  - Cleaned up VT_F_TRANS_RAMP, VT_DWN_PITCH_MAX, FW_T_SPD_OMEGA, FW_T_I_GAIN_THR from Alia250 config
  - Added explanatory comments for all removed parameters
- **⚡ Performance Optimization**: Debug logging disabled by default for production use

### Previous Release - Version 2.5.0 (January 2025)

- **🎯 Critical Sensor Stability Fix**: Resolved BARO STALE errors and vertical velocity oscillation issues:
  - Increased HIL_SENSOR update rate from 100Hz to 250Hz (matches PX4 Gazebo SITL standard)
  - Enhanced barometer noise modeling (3cm std dev) for proper liveness detection
  - Synchronized GPS-Barometer altitude quantization (5mm resolution) to eliminate EKF2 innovation conflicts
  - Relaxed EKF2 innovation gates (BARO_GATE, GPS_V_GATE: 8.0) for SITL environment
  - Tuned COM_ARM_EKF_VEL parameter (0.8) to prevent false preflight velocity errors
- **🖥️ Enhanced User Interface**: Professional high-contrast UI system with centralized version management and color-coded status indicators
- **🐧🍎 Cross-Platform Support**: Native Linux and macOS makefile support for seamless compilation across all major platforms
- **📦 Portability Improvements**: Project uses relative paths and external dependencies properly excluded from version control
- **🔧 Stability Enhancements**: Multiple under-the-hood improvements for reliable SITL simulation performance

### Previous Features (v2.0.0)
- **Sensor Improvements**: Fixed many sensor problems and inconsistencies.
- **Multi-Airframe Support**: Native support for multiple airframes including:
  - **Ehang 184**
  - **Alia 250**
  - **Cessna 172**
  - **Bayraktar TB2**
- **In-Menu Airframe Selection**: Change airframes directly from the X-Plane menu.
- **Automated Setup Script**: A full automation script that sets everything up for you in WSL or Linux environments.
- **Detailed Configuration Instructions**: The `config.ini` file includes instructions for building custom airframes.
- **Pre-Loaded Parameters**: Parameters are automatically loaded in PX4 build commands.

Check out the latest features in the [v2.5.0 release](https://github.com/alireza787b/px4xplane/releases/tag/v2.5.0).

## Video Tutorials

For setup guidance and demonstrations, visit our [YouTube playlist](https://www.youtube.com/watch?v=eZJpRHFgx6g&list=PLVZvZdBQdm_4RepbwUZaccwH0iQvHtMBh&pp=gAQBiAQB), which includes detailed tutorials and showcases the plugin in action!

[![PX4 X-Plane Plugin Video Thumbnail](https://img.youtube.com/vi/eZJpRHFgx6g/0.jpg)](https://www.youtube.com/watch?v=eZJpRHFgx6g&list=PLVZvZdBQdm_4RepbwUZaccwH0iQvHtMBh&pp=gAQBiAQB "Click to Watch!")

## Introduction

We bridge X-Plane and PX4 using the Simulator MAVLink API, which facilitates the exchange of sensor and actuator data, providing a rich and accurate simulation environment.

The following table illustrates the message flow:

| Message | Direction | Description |
|---------|-----------|-------------|
| [MAV_MODE:MAV_MODE_FLAG_HIL_ENABLED](https://mavlink.io/en/messages/common.html#MAV_MODE_FLAG_HIL_ENABLED) | NA | Mode flag when using simulation. All motors/actuators are blocked, but internal software is fully operational. |
| [HIL_ACTUATOR_CONTROLS](https://mavlink.io/en/messages/common.html#HIL_ACTUATOR_CONTROLS) | PX4 to Sim | PX4 control outputs (to motors, actuators). |
| [HIL_SENSOR](https://mavlink.io/en/messages/common.html#HIL_SENSOR) | Sim to PX4 | Simulated IMU readings in SI units in NED body frame. |
| [HIL_GPS](https://mavlink.io/en/messages/common.html#HIL_GPS) | Sim to PX4 | The simulated GPS RAW sensor value. |
| [HIL_OPTICAL_FLOW](https://mavlink.io/en/messages/common.html#HIL_OPTICAL_FLOW) | Sim to PX4 | Simulated optical flow from a flow sensor (e.g., PX4FLOW or optical mouse sensor). |
| [HIL_STATE_QUATERNION](https://mavlink.io/en/messages/common.html#HIL_STATE_QUATERNION) | Sim to PX4 | Contains the actual "simulated" vehicle position, attitude, speed, etc. This can be logged and compared to PX4's estimates for analysis and debugging. |
| [HIL_RC_INPUTS_RAW](https://mavlink.io/en/messages/common.html#HIL_RC_INPUTS_RAW) | Sim to PX4 | The RAW values of the RC channels received. |

## Building from Source

The px4xplane plugin supports **true cross-platform development** on Windows, Linux, and macOS.

### Quick Build Guide

**CMake (Recommended - All Platforms):**
```bash
git clone --recursive https://github.com/alireza787b/px4xplane.git
cd px4xplane
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
```

**Windows (Visual Studio):**
1. Open `px4-xplane.sln` in Visual Studio 2019/2022
2. Select **Release | x64**
3. Build → Build Solution

**Linux:**
```bash
make -f Makefile.linux
```

**macOS:**
```bash
make -f Makefile.macos
```

📖 **For detailed build instructions, prerequisites, and troubleshooting**, see the comprehensive **[Build Documentation](docs/BUILD.md)**.

### Pre-Built Binaries (Recommended for Users)

Don't want to build from source? **Download pre-built binaries** from the [Releases page](https://github.com/alireza787b/px4xplane/releases):

- 🪟 **Windows** (x64)
- 🐧 **Linux** (x64)
- 🍎 **macOS** (Universal: Intel + Apple Silicon)

All platforms are automatically built and tested via GitHub Actions CI/CD on every release.

### Development Workflow

**For Contributors:**
- Work on `master` branch for regular development
- Push to `master` triggers builds (artifacts kept 90 days)
- Create version tag (`v3.1.0`) to trigger automatic release with binaries

📖 **For complete developer workflow, versioning, and CI/CD**, see **[Developer Guide](docs/DEVELOPER.md)**.

---

## Known Issues

- **Performance**: Ensure X-Plane runs at high FPS for optimal EKF performance.
- **Temporary Fix**: This plugin is a temporary fix until merged with the official PX4 repository. For now, we use a forked version of PX4 with custom airframes and modifications.
- **Bug Reports**: If you find any bug, please report it on the GitHub issues page.

## Usage Guide

For full step-by-step instructions, follow the [Version 2 Documentation](https://github.com/alireza787b/px4xplane/blob/master/docs/v2.md).

### Automated Setup Script

This automated script is optimized for **WSL** (Windows Subsystem for Linux), where X-Plane runs on Windows and PX4 SITL runs on Linux (WSL). It also works natively on Ubuntu, Linux, and macOS distributions.

To quickly set up PX4 with X-Plane using WSL (Ubuntu 22.04 or newer), follow this simple step:

1. **Run the setup script directly from your terminal:**

   First, make sure you are in your WSL Ubuntu terminal (launch it from PowerShell), then enter the following command:

   ```bash
   cd ~ && curl -O https://raw.githubusercontent.com/alireza787b/px4xplane/master/setup/setup_px4_sitl.sh && bash setup_px4_sitl.sh
   ```

### Script Features

- **Auto-Update Detection**: Automatically checks for updates on subsequent runs
- **Self-Updating**: Script updates itself when new versions are available
- **Quick Skip**: Press 's' during update check for instant start (perfect for quick iterations)
- **Smart Defaults**: Auto-proceeds with sensible defaults if no input provided
- **Robust Error Handling**: Graceful fallbacks if network is unavailable
- **Repair Mode**: Use `--repair` flag to force full re-setup if needed

### Important Notes

- **First-Time Setup**: The installation may take 10-20 minutes, depending on your system, especially the first time
- **Subsequent Runs**: Script automatically detects existing installation and offers to check for updates
- **Interactive Prompts**: You can customize the setup or use defaults (auto-proceeds after timeout)
- **Cross-Platform**: Supports WSL, native Linux, and macOS
- **Update Check**: Press 's' to skip update check for quick start, or wait 5 seconds to check for updates

### Typical Usage after first setup


**Subsequent runs (with update check):**
```bash
px4xplane
# Wait 5 seconds or press any key to check for updates
# Select airframe and run
```

**Quick start (skip updates):**
```bash
px4xplane
# Press 's' immediately to skip update check
# Instant start
```

**Force update/repair:**
```bash
px4xplane --repair
# Forces full setup and update
```

### Available Airframes

The script supports 5 pre-configured X-Plane airframes:

- **xplane_cessna172** - Cessna 172 (fixed-wing trainer)
- **xplane_tb2** - TB2 UAV (fixed-wing)
- **xplane_ehang184** - Ehang 184 (quadcopter airtaxi)
- **xplane_alia250** - Alia-250 (eVTOL quadplane)
- **xplane_qtailsitter** - Quantix (quad tailsitter VTOL)

### Uninstall

To remove the global paths and the `px4xplane` command:

```bash
px4xplane --uninstall
```

This will remove:
- The global `px4xplane` command
- The PX4-Autopilot-Me repository
- Configuration files

### Troubleshooting

**Update check fails:**
- Script continues with local version
- Or use repair mode: `px4xplane --repair`

**Build errors:**
- Try clean build: `px4xplane`
- When prompted, type 'c' for clean or 'd' for distclean

**Script outdated:**
- Script auto-updates itself
- Or manually download latest:
```bash
cd ~ && curl -O https://raw.githubusercontent.com/alireza787b/px4xplane/master/setup/setup_px4_sitl.sh && bash setup_px4_sitl.sh
```

### Manual Advanced Setup

For advanced users who prefer manual control, you can clone and set up the environment using the forked PX4 repository:

1. **Clone the repository:**
   ```bash
   git clone https://github.com/alireza787b/PX4-Autopilot-Me.git --recursive
   cd PX4-Autopilot-Me
   git checkout px4xplane-sitl
   ```

2. **Set up the environment:**
   ```bash
   bash ./Tools/setup/ubuntu.sh
   ```

3. **Configure PX4 SITL (WSL users only):**
   - In Windows Terminal, check your IP address (e.g., `172.21.144.1`):
     ```bash
     ipconfig
     ```
   - In the WSL Terminal, inside the PX4-Autopilot-Me directory, set the PX4 simulation hostname with your IP:
     ```bash
     export PX4_SIM_HOSTNAME=172.21.144.1
     ```

4. **Build the desired airframe:**
   ```bash
   make px4_sitl_default xplane_ehang184
   ```

   Replace `xplane_ehang184` with your desired airframe from the list above.

**Note for Native Linux Users:** If both X-Plane and PX4 run on the same Linux machine, the IP configuration step (step 3) may not be needed. You can use localhost or `127.0.0.1`.

Refer to the [PX4 official documentation](https://docs.px4.io/main/en/simulation/) for more details, but use the forked repository (PX4-Autopilot-Me branch `px4xplane-sitl`) instead of the official PX4 repo.

### Temporary Notice

⚠️ **Testing Phase**: This setup currently uses a forked PX4 repository while X-Plane SITL support is being prepared for submission to the official PX4 repository. Once the pull request is merged into [https://github.com/PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot), you will be able to use the official PX4 repository directly, and this fork will no longer be needed. The setup script will be updated accordingly.

**Track progress:** [https://github.com/alireza787b/PX4-Autopilot-Me/tree/feature/xplane-sitl-integration](https://github.com/alireza787b/PX4-Autopilot-Me/tree/feature/xplane-sitl-integration)

## Defining Custom Airframes

If you need to define new airframes for your simulation, follow the detailed guide here:

👉 [How to Define Custom Airframes in PX4-XPlane](https://github.com/alireza787b/px4xplane/blob/master/docs/custom-airframe-config.md)

This guide will walk you through the process of setting up custom configurations, mapping PX4 channels to X-Plane datarefs, and integrating new airframes into your simulations. Whether you're using existing X-Plane models or creating new ones in Plane Maker, this guide has you covered.

---

## Technical Improvements in Version 2.5.0

### Sensor Stability and EKF2 Performance
Version 2.5.0 addresses critical sensor timing and fusion issues that previously caused BARO STALE errors and vertical velocity instability:

**HIL_SENSOR Update Rate Optimization**
- Increased from 100Hz to 250Hz to match PX4's lockstep timing expectations
- Prevents sensor timeout/STALE errors by ensuring consistent high-frequency updates
- Aligns with PX4 Gazebo SITL standard for optimal sensor fusion

**Barometer Modeling**
- Enhanced noise model (3cm standard deviation) ensures PX4 properly detects sensor liveness
- Synchronized altitude quantization (5mm) with GPS to eliminate innovation conflicts
- Prevents temporal misalignment between different height sources in EKF2

**EKF2 Parameter Tuning**
- Relaxed innovation gates (EKF2_BARO_GATE: 8.0, EKF2_GPS_V_GATE: 8.0) for SITL environment
- Adjusted COM_ARM_EKF_VEL (0.8) to prevent false velocity limit exceeded errors
- Optimized for simulation timing characteristics while maintaining accuracy

### User Interface Enhancements
Professional high-contrast UI system with:
- Centralized version management (VersionInfo.h)
- Color-coded connection status indicators
- Improved readability and real-time debugging information

### Cross-Platform Compatibility
With native Linux and macOS makefile support, PX4-XPlane 2.5.0 ensures seamless compilation and operation across all major platforms, making it accessible to a broader range of developers and researchers.

---

## Contribution and Support

Feel free to reach out to me in the issue section if you need help. Having experienced contributors is greatly appreciated!

## License

This project is licensed under the MIT License. See the `LICENSE` file for more details.