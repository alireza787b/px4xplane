# PX4 X-Plane Plugin

This project establishes a connection between X-Plane and PX4 SITL (Software In The Loop) to simulate drone flight in the X-Plane environment. The goal is to provide a realistic simulation environment where PX4 can control a drone in the X-Plane simulator.

## Introduction

All simulators, with the exception of Gazebo, communicate with PX4 using the Simulator MAVLink API. This API defines a set of MAVLink messages that provide sensor data from the simulated environment to PX4 and return motor and actuator values from the flight code that will be applied to the simulated vehicle.

![PX4 Simulator Messages](https://github.com/alireza787b/px4xplane/assets/30341941/0f7d0129-a780-4952-abef-3a858aaf6f92)


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

## Features

- **Xplane-PX4 SITL TCP Connection**: Establishes a TCP connection between X-Plane and PX4 SITL.
- **MAVLink Message Integration**: MAVLink messages such as HIL_SENSOR, HIL_GPS, and HIL_STATE_QUATERNION are used to communicate between the simulator and PX4.
- **Flight Modes**: Currently, you can fly a drone in Acro mode as angular rates are functional.

## Known Issues

- **Attitude Estimation**: There seems to be a problem is abnormal attitudes and high-speed rapid rotation.
- **Performance**: It's crucial to run X-Plane at a high FPS since the PX4 EKF requires fast updating of sensor data.
- **Yaw PID Tuning**: The Quadricopter drone seems to be very hard to tune. now it almost flies but now very good in yaw. If you could tune it better please share your gains. 

## Setup and Testing


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

2. **Plugin Installation**:
   - You have two options to get the plugin: 
     - Download the precompiled binary from the [Releases](https://github.com/alireza787b/px4xplane/releases) page.
     - Build the plugin yourself from the source code.
   - After obtaining the plugin, you should see a `px4xplane` folder containing a `64` folder that contains a  `.xpl` file. This is the plugin file.
   - To install the plugin for a specific drone, copy the `px4xplane` folder to the plugins folder of the drone in your X-Plane installation. The path should look like this: `X-Plane Installation Folder/Aircraft/Your Drone/Plugins`.
   - If you want to install the plugin globally for all aircraft in X-Plane, copy the `px4xplane` folder to the global plugins folder. The path should look like this: `X-Plane Installation Folder/Resources/Plugins`.
   - Please note that installing the plugin globally will make it available for all aircraft, but it may not work correctly with all of them.

3. **PX4 SITL Setup**:
   - Clone and build `PX4-Autopilot` and the SITL Engine. Follow the instructions provided by [PX4](https://docs.px4.io/main/en/simulation/).
   - If cloning on WSL (or another system), change the hostname IP from `localhost` to where X-Plane is running:
     ```bash
     export PX4_SIM_HOSTNAME=XPLANE_SYSTEM_IP
     ```

4. **Running PX4 SITL**:
   - Run the default `iris` (or your custom airframe) with `none_` to indicate that you'll connect your own simulator:
     ```bash
     make px4_sitl none_iris
     ```

5. **X-Plane Connection**:
   - Launch X-Plane and load your drone.
   - Navigate to `plugin -> px4xplane -> Connect to SITL`. The simulator should now be connected.
   - Under `plugin -> px4xplane -> Show Settings`, you can view live data and status.

6. **Firewall Configuration**:
   - Ensure that port `4560 TCP` is not blocked by your firewall on both systems. If running on the same system, this is typically not an issue.

7. **QGroundControl (QGC) Connection**:
   - If running on the same system, QGC should auto-connect to the drone on X-Plane.
   - For WSL or another system, open QGroundControl, navigate to `Application Settings -> Comm Links`, and add a new connection to UDP port `18570`. Click `connect`, and it should work.

## Status

As of November 2023, This project is currently under development. Once a fully functional initial version is ready, the first beta release will be made available.

## Contribution and Support

Feel free to reach out to me in the issue section if you need help. Having experienced contributors is greatly appreciated!


## License

This project is licensed under the MIT License. See the `LICENSE` file for more details.


## X-Plane Aircraft

Since motor mapping is still hard-coded in the code, I recommend you use quadricopter air taxi model. It is basically a quad config but the graphics just show coaxial X8 Configuration.

https://forums.x-plane.org/index.php?/files/file/76635-quadricopter-piloted/
