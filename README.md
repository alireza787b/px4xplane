# PX4-XPlane - PX4 X-Plane Integration Plugin

This project establishes a robust connection between X-Plane and PX4 SITL (Software In The Loop) to simulate drone flight in the X-Plane environment. Our goal is to deliver a realistic simulation experience where PX4 can control various drones within X-Plane, and with our continuous improvements, it's getting better all the time.

## Latest Release - Version 2.5.2 (January 2025)

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

---

## Known Issues

- **Performance**: Ensure X-Plane runs at high FPS for optimal EKF performance.
- **Temporary Fix**: This plugin is a temporary fix until merged with the official PX4 repository. For now, we use a forked version of PX4 with custom airframes and modifications.
- **Bug Reports**: If you find any bug, please report it on the GitHub issues page.

## Usage Guide

For full step-by-step instructions, follow the [Version 2 Documentation](https://github.com/alireza787b/px4xplane/blob/master/docs/v2.md).

### Automated Setup Script

This automated script is optimized for using with **WSL** (Windows Subsystem for Linux), where X-Plane runs on Windows, and PX4 SITL runs on Linux (WSL). It also works natively on Ubuntu, Linux, and macOS distributions.

To quickly set up PX4 with X-Plane using WSL (Ubuntu 22.04), follow this simple step:

1. **Run the setup script directly from your terminal:**

   First, make sure you are in your WSL Ubuntu terminal (launch it from PowerShell), then enter the following command in your WSL terminal:

   ```bash
   cd ~ && curl -O https://raw.githubusercontent.com/alireza787b/px4xplane/master/setup/setup_px4_sitl.sh && bash setup_px4_sitl.sh
   ```

### Important Notes:

- **Time Required**: The installation process may take some time, depending on your system and configuration, especially the first time.
- **Prompts**: During installation, you will encounter several prompts asking you to customize the setup. You can either read and decide on each prompt or just let the defaults take effect (the script will automatically proceed with default settings if you don't respond).
- **Cross-Platform**: Now supports Linux and macOS natively with dedicated makefiles.

2. Once setup is complete, use the `px4xplane` command to run the simulation from anywhere in your terminal.

### Uninstall

To remove the global paths and the `px4xplane` command:

```bash
px4xplane --uninstall
```

### Manual Advanced Setup

For advanced users, manually clone and set up the environment using the forked PX4 repository:

1. Clone the repository:
   ```bash
   git clone https://github.com/alireza787b/PX4-Autopilot-Me.git --recursive
   cd PX4-Autopilot-Me
   ```

2. Set up the environment:
   ```bash
   bash ./Tools/setup/ubuntu.sh
   ```

3. Configure PX4 SITL:
   - In Windows Terminal, check your IP address (e.g., `172.21.144.1`):
     ```bash
     ipconfig
     ```
   - In the WSL Terminal, inside the PX4-Autopilot directory, set the PX4 simulation hostname with your IP:
     ```bash
     export PX4_SIM_HOSTNAME=172.21.144.1
     ```

4. Build the desired airframes:
   ```bash
   make px4_sitl xplane_ehang184
   ```

   Other airframes include:
   - `xplane_alia250`
   - `xplane_cessna172`
   - `xplane_tb2`

Refer to the [official PX4 WSL setup documentation](https://docs.px4.io/main/en/simulation/) for more details, but use the forked repository instead of the official PX4 repo until the changes are merged.

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
