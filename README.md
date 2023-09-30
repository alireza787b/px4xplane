# PX4 X-Plane Plugin

This repository contains the source code for a robust, modular, and user-friendly X-Plane plugin designed to enable seamless interaction with PX4 Software In The Loop (SITL). The plugin facilitates Software-In-The-Loop (SITL) simulation for drone software development and testing.

## Introduction

The PX4 X-Plane plugin communicates with PX4 using the Simulator MAVLink API. This API defines a set of MAVLink messages that supply sensor data from the simulated world to PX4 and return motor and actuator values from the flight code that will be applied to the simulated vehicle. The plugin communicates with PX4 over TCP connections. The simulator's local TCP Port, 4560, is used for communication with PX4. The simulator listens to this port, and PX4 initiates a TCP connection to it.

The following MAVLink messages are used in the communication between the plugin and PX4:

- **HIL_ACTUATOR_CONTROLS:** Sent from PX4 to the simulator. It contains PX4 control outputs (to motors, actuators).
- **HIL_SENSOR:** Sent from the simulator to PX4. It contains simulated IMU readings in SI units in NED body frame.
- **HIL_GPS:** Sent from the simulator to PX4. It contains the simulated GPS RAW sensor value.
- **HIL_OPTICAL_FLOW:** Sent from the simulator to PX4. It contains simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor).
- **HIL_STATE_QUATERNION:** Sent from the simulator to PX4. It contains the actual "simulated" vehicle position, attitude, speed etc. This can be logged and compared to PX4's estimates for analysis and debugging (for example, checking how well an estimator works for noisy (simulated) sensor inputs).
- **HIL_RC_INPUTS_RAW:** Sent from the simulator to PX4. It contains the RAW values of the RC channels received.

For more information about the Simulator MAVLink API, refer to the [PX4 Developer Guide](https://docs.px4.io/main/en/simulation/).

## Current Progress

The project is currently in the development phase with the following components implemented:

- **Plugin Structure:** Basic scaffolding has been established, inclusive of menu creation, a settings window, and fundamental rendering capabilities.
- **Data Management:** A `DataRefManager` class has been developed to manage X-Plane sensor data references.
- **Plugin Display:** The plugin can display sensor data within a dedicated window on startup, leveraging data obtained via the `DataRefManager`.
- **Configuration Management:** A configuration reader is implemented to read settings such as the SITL IP from a configuration file.

## Future Work

The following components are yet to be implemented:

- **Connection Management:** The `ConnectionManager` class will handle the establishment, maintenance, and teardown of TCP connections with PX4 SITL.
- **MAVLink Integration:** A component will be developed to package and parse MAVLink messages, ensuring accurate and efficient communication between the plugin and PX4 SITL.
- **X-Plane Data Handling:** Functionality will be implemented to obtain, process, and convert X-Plane data references into appropriate MAVLink message formats.
- **Control Overrides:** Mechanisms will be developed to override X-Plane's autopilot settings and control surfaces based on received MAVLink messages while maintaining the ability to revert to default settings upon disconnection.
- **Enhanced Sensor Integration:** The integration will be extended to include additional sensors like barometer, airspeed, and RPM, among others.

## Approach

The project follows a modular approach with each component (e.g., Connection Manager, DataRef Manager) developed as a separate, self-contained entity ensuring high cohesion and low coupling. The codebase maintains a high standard of readability, with clear naming conventions, structured formatting, and comprehensive documentation. Each component and functionality is developed and tested in isolation before integration, facilitating easier debugging and validation. The architecture is designed to accommodate future enhancements and integrations effortlessly, such as support for additional sensors and communication protocols.

## Immediate Next Steps

1. Implement and test the `ConnectionManager` class.
2. Develop MAVLink message handling functionalities.
3. Implement and validate data conversion and processing functionalities for transforming X-Plane data into MAVLink messages.
4. Establish the end-to-end communication flow between the plugin and PX4 SITL.

## Long-Term Goals

1. Explore the integration of UDP for enhanced performance and reliability.
2. Implement radio control overrides and investigate the integration of vision sensors.
3. Enable multi-drone simulation capabilities within the plugin.
4. Establish thorough validation mechanisms for sensor data and connectivity and ensure graceful connection handling during runtime.

## Repository Structure

The repository contains the following key files and directories:

- `ConnectionManager.cpp`, `ConnectionManager.h`: These files are for the ConnectionManager class, which will handle TCP connections with PX4 SITL.
- `DataRefManager.cpp`, `DataRefManager.h`: These files contain the DataRefManager class, which manages X-Plane sensor data references.
- `configReader.cpp`, `configReader.h`: These files contain the ConfigReader class, which reads settings such as the SITL IP from a configuration file.
- `px4xplane.cpp`: This is the main file for the plugin.
- `px4-xplane.sln`, `Hello-World-SDK-3.vcxproj`: These are the project files for Visual Studio.
- `SDK`: This directory contains additional code and resources related to the PX4 X-Plane plugin.

## Getting Started

To get started with the project, clone the repository and open the solution file (`px4-xplane.sln`) in Visual Studio. Make sure you have the X-Plane SDK installed and configured correctly. For more information on how to do this, refer to the [X-Plane SDK Documentation](https://developer.x-plane.com/sdk/plugin-sdk-documents/).

To build the project, use the command `make px4_sitl none_iris`.

## Contributing

Contributions are welcome! Please read our contributing guidelines and code of conduct before making a pull request.

## License

This project is licensed under the MIT License. See the `LICENSE` file for more details.

## Project Status

As of September 2023, this project is under progress with minimal functionality.
