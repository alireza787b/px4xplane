# PX4 X-Plane Plugin

This project establishes a robust connection between X-Plane and PX4 SITL (Software In The Loop) to simulate drone flight in the X-Plane environment. Our goal is to deliver a realistic simulation experience where PX4 can control various drones within X-Plane, and with our continuous improvements, it's getting better all the time.

## Latest Release - Version 2.0.0 (August 2024)

### What's New
- **Sensor Improvements**: Fixed many sensor problems and inconsistencies.
- **Multi-Airframe Support**: Native support for multiple airframes including:
  - **Ehang 184**
  - **Alia 250**
  - **Cessna 172**
  - **Bayraktar TB2**
- **In-Menu Airframe Selection**: Change airframes directly from the X-Plane menu.
- **Automated Setup Script**: A full automation script that sets everything up for you in WSL or Linux environments. This serves as a temporary fix until the changes are merged into the official PX4 repository.
- **Detailed Configuration Instructions**: The `config.ini` file includes instructions for building custom airframes.
- **Pre-Loaded Parameters**: Parameters are automatically loaded in PX4 build commands. However, if you need to load them manually, they are still available in the config folder.

Check out the new features in the [v2.0.0 release](https://github.com/alireza787b/px4xplane/releases/tag/v2.0.0). **YouTube video tutorial for version 2.0 will be released soon!**

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

## Known Issues

- **Performance**: Ensure X-Plane runs at high FPS for optimal EKF performance.
- **Temporary Fix**: This plugin is a temporary fix until merged with the official PX4 repository. For now, we use a forked version of PX4 with custom airframes and modifications.
- **Bug Reports**: If you find any bug, please report it on the GitHub issues page.

## Usage Guide

For full step-by-step instructions, follow the [Version 2 Documentation](https://github.com/alireza787b/px4xplane/blob/master/docs/v2.md).

### Automated Setup Script

This automated script is optimized for using with **WSL** (Windows Subsystem for Linux), where X-Plane runs on Windows, and PX4 SITL runs on Linux (WSL). It should also work natively on Ubuntu and other Linux distributions.

1. Open your terminal and enter your WSL terminal. Then change the directory to your home directory:

   ```bash
   cd ~
   ```

2. Download the setup script using `curl`:

   ```bash
   curl -O https://raw.githubusercontent.com/alireza787b/px4xplane/master/setup/setup_px4_sitl.sh
   ```

3. Run the setup script:

   ```bash
   bash ./setup_px4_sitl.sh
   ```
   Follow Instruction and select the platform you want to run.

Once default setup is complete, use the `px4xplane` command to run the simulation from anywhere in your terminal.

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

3. Build the desired airframes:
   ```bash
   make px4_sitl xplane_ehang184
   ```

   Other airframes include:
   - `xplane_alia250`
   - `xplane_cessna172`
   - `xplane_tb2`

Refer to the [official PX4 WSL setup documentation](https://docs.px4.io/main/en/simulation/) for more details, but use the forked repository instead of the official PX4 repo until the changes are merged.



## Contribution and Support

Feel free to reach out to me in the issue section if you need help. Having experienced contributors is greatly appreciated!

## License

This project is licensed under the MIT License. See the `LICENSE` file for more details.

