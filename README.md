# PX4 X-Plane Plugin

This project establishes a robust connection between X-Plane and PX4 SITL (Software In The Loop) to simulate drone flight in the X-Plane environment. Our goal is to deliver a realistic simulation experience where PX4 can control various drones within X-Plane, and with our continuous improvements, it's getting better all the time.

## Latest Release

Check out the new features in the [v1.1.0 release](https://github.com/alireza787b/px4xplane/releases/tag/v1.1.0) of the plugin, including support for additional airframes like the Alia-250 eVTOL. Don't miss the upcoming step-by-step tutorials on our GitHub!

## Video Tutorials

For setup guidance and demonstrations, visit our [YouTube playlist](https://www.youtube.com/watch?v=eZJpRHFgx6g&list=PLVZvZdBQdm_4RepbwUZaccwH0iQvHtMBh&pp=gAQBiAQB), which includes detailed tutorials and showcases the plugin in action!

[![PX4 X-Plane Plugin Video Thumbnail](https://img.youtube.com/vi/eZJpRHFgx6g/0.jpg)](https://www.youtube.com/watch?v=eZJpRHFgx6g&list=PLVZvZdBQdm_4RepbwUZaccwH0iQvHtMBh&pp=gAQBiAQB "Click to Watch!")

## Introduction

We bridge X-Plane and PX4 using the Simulator MAVLink API, which facilitates the exchange of sensor and actuator data, providing a rich and accurate simulation environment.

![PX4 Simulator Messages](https://github.com/alireza787b/px4xplane/assets/simulator_messages.png)



The following table illustrates the message flow:

| Message | Direction | Description |
|---------|-----------|-------------|
| [MAV_MODE:MAV_MODE_FLAG_HIL_ENABLED](https://mavlink.io/en/messages/common.html#MAV_MODE_FLAG_HIL_ENABLED) | NA | Mode flag when using simulation. All motors/actuators are blocked, but internal software is fully operational. |
| [HIL_ACTUATOR_CONTROLS](https://mavlink.io/en/messages/common.html#HIL_ACTUATOR_CONTROLS) | PX4 to Sim | PX4 control outputs (to motors, actuators). |
| [HIL_SENSOR](https://mavlink.io/en/messages/common.html#HIL_SENSOR) | Sim to PX4 | Simulated IMU readings in SI units in NED body frame. |
| [HIL_GPS](https://mavlink.io/en/messages/common.html#HIL_GPS) | Sim to PX4 | The simulated GPS RAW sensor value. |
| [HIL_OPTICAL_FLOW](https://mavlink.io/en/messages/common.html#HIL_OPTICAL_FLOW) | Sim to PX4 | Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor). |
| [HIL_STATE_QUATERNION](https://mavlink.io/en/messages/common.html#HIL_STATE_QUATERNION) | Sim to PX4 | Contains the actual "simulated" vehicle position, attitude, speed, etc. This can be logged and compared to PX4's estimates for analysis and debugging. |
| [HIL_RC_INPUTS_RAW](https://mavlink.io/en/messages/common.html#HIL_RC_INPUTS_RAW) | Sim to PX4 | The RAW values of the RC channels received. |

## Known Issues

- As of January 2024, I've fixed previous issues with attitude estimation. Performance remains key; ensure X-Plane runs at high FPS for optimal EKF performance. Stay tuned for more updates as we progress.
- If you find any bug, please report it...



### New Configurations

The `config.ini` file has been significantly improved to facilitate the easy definition of custom airframes. The plugin now comes with support for the Cessna 172, Alia-250, and Ehang 184, complete with preset parameters for immediate use. To utilize these new configurations, you can wait for the [pull request](https://github.com/PX4/PX4-Autopilot/pull/22493) to be merged into the official PX4 codebase, or use my forked repository and branch temporarily.

For those who need to use the default `make px4_sitl none_iris` command, be aware that manual updating of parameter files is required, and this approach may not support fixed-wing and VTOL models effectively. Once the pull request is merged, you'll be able to use dedicated commands for each of the supported models — ehang184, alia250, and cessna172 — streamlining your simulation setup process.



## Usage Guide

follow the <a href="https://alireza787b.github.io/px4xplane/v1.0.0.html" target="_blank"> step-by-step instruction version 1.0.0</a>.


### For Developers: Building from Source

1. **Repository Cloning**:
   - Clone this repository to your local machine:
     ```bash
     git clone --recurse-submodules https://github.com/alireza787b/px4xplane.git
     ```
   - If you've already cloned the repository without the `--recurse-submodules` option, you can fetch the contents of the submodules with:
     ```bash
     git submodule init
     git submodule update
     ```

2. **Building the Plugin**:
   - Follow the instructions in the `BUILDING.md` file to build the plugin from the source. (will be added)



## Configuration and Motor Arrangement

The plugin now supports flexible motor arrangements and configuration changes through a `config.ini` file located alongside the `.xpl` file. This enhancement allows users to easily switch between different airframe types (note: fixed-wing configurations are currently not supported) and customize motor mappings without modifying the source code.


### X-Plane Aircraft Setup

For the best experience, we recommend using the following aircraft models with our plugin:

- **Quadcopter (Ehang 184)**: Utilize the quadricopter air taxi model, which graphically displays a coaxial X8 configuration. Download the model that aligns with our plugin from [X-Plane.org](https://forums.x-plane.org/index.php?/files/file/76635-quadricopter-piloted/). A parameter file (`quadricopter_px4.params`) is included in the `config` folder to pre-configure all necessary settings for this airframe.

- **Quad Plane VTOL (Alia-250)** and **Fixed-Wing (Cessna 172)**: These are default aircraft in X-Plane 12, and our plugin supports them out of the box. Simply select them within X-Plane for immediate use with PX4 SITL.

Remember, for an optimized simulation experience, ensure that you are using the correct aircraft model with the corresponding parameters provided.



## Status

As of January 2024, This project is currently under development.

## Contribution and Support

Feel free to reach out to me in the issue section if you need help. Having experienced contributors is greatly appreciated!


## License

This project is licensed under the MIT License. See the `LICENSE` file for more details.

