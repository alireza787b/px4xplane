# PX4 X-Plane Plugin

This project establishes a connection between X-Plane and PX4 SITL (Software In The Loop) to simulate drone flight in the X-Plane environment. The goal is to provide a realistic simulation environment where PX4 can control a drone in the X-Plane simulator.

## Introduction

All simulators, with the exception of Gazebo, communicate with PX4 using the Simulator MAVLink API. This API defines a set of MAVLink messages that provide sensor data from the simulated environment to PX4 and return motor and actuator values from the flight code that will be applied to the simulated vehicle.

![PX4 Simulator Messages](https://docs.px4.io/main/assets/img/px4_simulator_messages.f1161233.svg)

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

- **Magnetic Field Modeling**: X-Plane doesn't model the magnetic field. Currently, it's hardcoded for Tehran City, but this needs a more generic solution.
- **Attitude Estimation**: There seems to be a discrepancy in the acceleration and magnetic field coordinate system of X-Plane compared to expectations (Body coordinate system). The potential problem lies in the coordinate transformation and definition mismatch between X-Plane and PX4.
- **Plugin Responsiveness**: Occasionally, the plugin may cause the plane to become unresponsive when attempting to exit.
- **Performance**: It's crucial to run X-Plane at a high FPS since the PX4 EKF requires fast updating of sensor data.

## Setup and Testing

1. **Plugin Installation**:
   - Install the plugin to your plugins folder for the specific drone.

2. **PX4 SITL Setup**:
   - Clone and build `px4_sitl`. Follow the instructions provided by [PX4](https://docs.px4.io/main/en/simulation/).
   - If cloning on WSL (or another system), change the hostname IP from `localhost` to where X-Plane is running:
     ```bash
     export PX4_SIM_HOSTNAME=XPLANE_SYSTEM_IP
     ```
     For example, if using WSL on Windows:
     ```bash
     export PX4_SIM_HOSTNAME=172.21.160.1
     ```

3. **Running PX4 SITL**:
   - Run the default `iris` (or your custom airframe) with `none_` to indicate that you'll connect your own simulator:
     ```bash
     make px4_sitl none_iris
     ```

4. **X-Plane Connection**:
   - Launch X-Plane and load your drone.
   - Navigate to `plugin -> px4xplane -> click on connect to SITL`. The simulator should now be connected.
   - Under `plugin -> px4xplane -> Show Settings`, you can view live data and status.

5. **Firewall Configuration**:
   - Ensure that port `4560 TCP` is not blocked by your firewall on both systems. If running on the same system, this is typically not an issue.

6. **QGroundControl (QGC) Connection**:
   - If running on the same system, QGC should auto-connect to the drone on X-Plane.
   - For WSL or another system, open QGroundControl, navigate to `Application Settings -> Comm Links`, and add a new connection to UDP port `18570`. Click `connect`, and it should work.

## Status

This project is currently under development. Once a fully functional initial version is ready, the first beta release will be made available.

## Contribution and Support

Feel free to contact in the issue section if you need help. Having experienced contributors is greatly appreciated!


## License

This project is licensed under the MIT License. See the `LICENSE` file for more details.

## Project Status

As of September 2023, this project is under progress with minimal functionality.
