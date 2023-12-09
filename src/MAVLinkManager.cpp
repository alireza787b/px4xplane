#include "MAVLinkManager.h"
#include "ConnectionManager.h"
#include "DataRefManager.h"
#include <vector>
#include "XPLMUtilities.h"
#include <tuple>  // for std::tuple
#include <ConfigManager.h>



// Define and initialize the static member
MAVLinkManager::HILActuatorControlsData MAVLinkManager::hilActuatorControlsData = {};



// Declare constants at the class or namespace level
constexpr float DEG_TO_RAD = 3.14159265358979323846 / 180.0;
constexpr float GRAVITY = 9.81;

// Define and initialize the random number generators and distributions
std::random_device MAVLinkManager::rd;
std::mt19937 MAVLinkManager::gen(MAVLinkManager::rd());
std::normal_distribution<float> MAVLinkManager::noiseDistribution(0.0f, 0.001f);
std::normal_distribution<float> MAVLinkManager::noiseDistribution_mag(0.0f, 0.00001f);


/**
 * @brief Sends the HIL_SENSOR MAVLink message.
 *
 * This function constructs and sends the HIL_SENSOR message which contains simulated
 * sensor readings for Hardware-In-the-Loop (HIL) simulation. It checks if a connection
 * is established before sending the message. The function gathers data from various
 * simulation sources, applies necessary conversions, and populates the HIL_SENSOR message.
 *
 * The following sensor data is populated:
 * - Acceleration
 * - Gyroscope
 * - Pressure
 * - Magnetic Field
 * - Temperature
 *
 * The fields_updated bitmask in the HIL_SENSOR message is set to indicate which
 * sensor readings are being updated. For more details on the bitmask values, refer to:
 * https://mavlink.io/en/messages/common.html#HIL_SENSOR_UPDATED_FLAGS
 *
 * @note Ensure that the ConnectionManager is initialized and a connection is established
 *       before calling this function.
 *
 * @warning This function should be called at appropriate simulation update rates to ensure
 *          timely delivery of sensor data.
 *
 * Usage:
 * @code
 * MAVLinkManager::sendHILSensor();
 * @endcode
 *
 * @see setAccelerationData
 * @see setGyroData
 * @see setPressureData
 * @see setMagneticFieldData
 */
void MAVLinkManager::sendHILSensor(uint8_t sensor_id=0) {

    if (!ConnectionManager::isConnected()) return;

    mavlink_message_t msg;
    mavlink_hil_sensor_t hil_sensor;

    hil_sensor.time_usec = static_cast<uint64_t>(DataRefManager::getFloat("sim/time/total_flight_time_sec") * 1e6);
    hil_sensor.id = uint8_t(sensor_id);

    setAccelerationData(hil_sensor);
    setGyroData(hil_sensor);
    setPressureData(hil_sensor);
    setMagneticFieldData(hil_sensor);

    hil_sensor.temperature = DataRefManager::getFloat("sim/cockpit2/temperature/outside_air_temp_degc");

    // Now set the bits for the fields you are updating.
    // Refer to the provided enum values for the fields_updated flags.
    //https://mavlink.io/en/messages/common.html#HIL_SENSOR_UPDATED_FLAGS
    uint32_t fields_updated = 0; // Start with a bitmask of all zeros
    fields_updated |= (1 << 0); // HIL_SENSOR_UPDATED_XACC
    fields_updated |= (1 << 1); // HIL_SENSOR_UPDATED_YACC
    fields_updated |= (1 << 2); // HIL_SENSOR_UPDATED_ZACC
    fields_updated |= (1 << 3); // HIL_SENSOR_UPDATED_XGYRO
    fields_updated |= (1 << 4); // HIL_SENSOR_UPDATED_YGYRO
    fields_updated |= (1 << 5); // HIL_SENSOR_UPDATED_ZGYRO
    fields_updated |= (1 << 6); // HIL_SENSOR_UPDATED_XMAG
    fields_updated |= (1 << 7); // HIL_SENSOR_UPDATED_YMAG
    fields_updated |= (1 << 8); // HIL_SENSOR_UPDATED_ZMAG
    fields_updated |= (1 << 9); // HIL_SENSOR_UPDATED_ABS_PRESSURE
    fields_updated |= (1 << 10); // HIL_SENSOR_UPDATED_DIF_PRESSURE
    fields_updated |= (1 << 11); // HIL_SENSOR_UPDATED_PRESSURE_ALT
    fields_updated |= (1 << 12); // HIL_SENSOR_UPDATED_TEMPERATURE

    hil_sensor.fields_updated = fields_updated;

    mavlink_msg_hil_sensor_encode(1, 1, &msg, &hil_sensor);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buffer, &msg);
    ConnectionManager::sendData(buffer, len);
}


/**
 * @brief Sends the HIL_GPS MAVLink message.
 *
 * This function constructs and sends the HIL_GPS message which contains simulated
 * GPS readings for Hardware-In-the-Loop (HIL) simulation. It checks if a connection
 * is established before sending the message. The function gathers data from the simulation
 * environment, applies necessary conversions, and populates the HIL_GPS message.
 *
 * Additionally, it calls updateMagneticFieldIfNeeded to check if the Earth's magnetic field
 * needs to be updated based on the current position.
 *
 * The following GPS data is populated:
 * - Time
 * - Fix type
 * - Latitude and Longitude
 * - Altitude
 * - Horizontal and Vertical Dilution of Precision (HDOP & VDOP)
 * - Number of visible satellites
 * - Velocities in North, East, and Down directions
 * - Ground speed
 * - Course over ground
 * - GPS ID and Yaw
 *
 * @note Ensure that the ConnectionManager is initialized and a connection is established
 *       before calling this function.
 *
 * @warning This function should be called at appropriate simulation update rates to ensure
 *          timely delivery of GPS data.
 *
 * Usage:
 * @code
 * MAVLinkManager::sendHILGPS();
 * @endcode
 */
void MAVLinkManager::sendHILGPS() {
    if (!ConnectionManager::isConnected()) return;

    mavlink_message_t msg;
    mavlink_hil_gps_t hil_gps;

    // Set various GPS data components
    setGPSTimeAndFix(hil_gps);
    setGPSPositionData(hil_gps);
    setGPSAccuracyData(hil_gps);
    setGPSVelocityData(hil_gps);
    setGPSHeadingData(hil_gps);

    // Check if the magnetic field needs to be updated
    updateMagneticFieldIfExceededTreshold(hil_gps);

    // Encode and send the MAVLink message
    mavlink_msg_hil_gps_encode(1, 1, &msg, &hil_gps);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buffer, &msg);
    ConnectionManager::sendData(buffer, len);
}



/**
 * @brief Checks if the Earth's magnetic field needs to be updated and updates it if necessary.
 *
 * This function calculates the distance between the current position and the last position
 * where the magnetic field was updated. If this distance exceeds a predefined threshold,
 * it triggers an update of the Earth's magnetic field in NED coordinates using the current
 * position data.
 *
 * This function is called within sendHILGPS to ensure the magnetic field data is updated
 * according to the movement of the simulated aircraft.
 *
 * @param hil_gps The HIL_GPS data structure containing the current GPS data.
 *
 * Usage:
 * @code
 * updateMagneticFieldIfNeeded(hil_gps);
 * @endcode
 */
void MAVLinkManager::updateMagneticFieldIfExceededTreshold(const mavlink_hil_gps_t& hil_gps) {
    /*GeodeticPosition currentPosition = {
        hil_gps.lat * 1e-7,
        hil_gps.lon * 1e-7,
        hil_gps.alt * 1e-3
    };*/

    GeodeticPosition currentPosition = {
        DataRefManager::getFloat("sim/flightmodel/position/latitude"),
        DataRefManager::getFloat("sim/flightmodel/position/longitude"),
        DataRefManager::getFloat("sim/flightmodel/position/elevation")
    };

    if (DataRefManager::calculateDistance(currentPosition, DataRefManager::lastPosition) > DataRefManager::UPDATE_THRESHOLD) {
        XPLMDebugString("px4xplane: Mag ValidityTreshold Reached!\n");

        DataRefManager::updateEarthMagneticFieldNED(currentPosition);
        DataRefManager::lastPosition = currentPosition;
    }
}




/**
 * @brief Sends the HIL state quaternion message.
 *
 * This function fetches the required data, populates the MAVLink HIL state
 * quaternion message, and sends it.
 */
void MAVLinkManager::sendHILStateQuaternion() {
    if (!ConnectionManager::isConnected()) return;

    mavlink_message_t msg;
    mavlink_hil_state_quaternion_t hil_state;

    populateHILStateQuaternion(hil_state);

    mavlink_msg_hil_state_quaternion_encode(1, 1, &msg, &hil_state);
    sendData(msg);
}

/**
 * @brief Populates the HIL state quaternion structure.
 *
 * This function fetches the required data from the DataRefManager and sets
 * the appropriate fields in the provided hil_state structure.
 *
 * @param hil_state Reference to the MAVLink HIL state quaternion structure to be populated.
 */
void MAVLinkManager::populateHILStateQuaternion(mavlink_hil_state_quaternion_t& hil_state) {
    hil_state.time_usec = static_cast<uint64_t>(DataRefManager::getFloat("sim/time/total_flight_time_sec") * 1e6);

    std::vector<float> q = DataRefManager::getFloatArray("sim/flightmodel/position/q");
    for (int i = 0; i < 4; ++i) {
        hil_state.attitude_quaternion[i] = q[i];
    }

    hil_state.rollspeed = DataRefManager::getFloat("sim/flightmodel/position/Prad");
    hil_state.pitchspeed = DataRefManager::getFloat("sim/flightmodel/position/Qrad");
    hil_state.yawspeed = DataRefManager::getFloat("sim/flightmodel/position/Rrad");

    hil_state.lat = static_cast<int32_t>(DataRefManager::getFloat("sim/flightmodel/position/latitude") * 1e7);
    hil_state.lon = static_cast<int32_t>(DataRefManager::getFloat("sim/flightmodel/position/longitude") * 1e7);
    hil_state.alt = static_cast<int32_t>(DataRefManager::getFloat("sim/flightmodel/position/elevation") * 1e3);

    // Convert OGL local velocities to NED frame
    float ogl_vx = DataRefManager::getFloat("sim/flightmodel/position/local_vx") * 100;
    float ogl_vy = DataRefManager::getFloat("sim/flightmodel/position/local_vy") * 100;
    float ogl_vz = DataRefManager::getFloat("sim/flightmodel/position/local_vz") * 100;

    // Local OGL to NED transformation:
    // OGL: X: East, Y: Up, Z: South
    // NED: North, East, Down
    hil_state.vx = -1 * ogl_vz;  // North (NED) from South (OGL)
    hil_state.vy = ogl_vx;       // East (NED) from East (OGL)
    hil_state.vz = -1 * ogl_vy;  // Down (NED) from Up (OGL)

    hil_state.ind_airspeed = static_cast<uint16_t>(DataRefManager::getFloat("sim/flightmodel/position/indicated_airspeed") * 100);
    hil_state.true_airspeed = static_cast<uint16_t>(DataRefManager::getFloat("sim/flightmodel/position/true_airspeed") * 100);

    hil_state.xacc = DataRefManager::getFloat("sim/flightmodel/forces/g_axil") * DataRefManager::g_earth * -1;
    hil_state.yacc = DataRefManager::getFloat("sim/flightmodel/forces/g_side") * DataRefManager::g_earth * -1;
    hil_state.zacc = DataRefManager::getFloat("sim/flightmodel/forces/g_nrml") * DataRefManager::g_earth * -1;
}


/**
 * @brief Sends the provided MAVLink message.
 *
 * This function encodes the provided MAVLink message into a buffer and sends it.
 *
 * @param msg The MAVLink message to be sent.
 */
void MAVLinkManager::sendData(const mavlink_message_t& msg) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buffer, &msg);
    ConnectionManager::sendData(buffer, len);
}



/**
 * @brief Sends the HIL RC Inputs to the MAVLink connection.
 *
 * This function fetches the required RC input data from the DataRefManager,
 * maps the values to the appropriate MAVLink format, and sends the HIL RC
 * Inputs message over the MAVLink connection.
 */
void MAVLinkManager::sendHILRCInputs() {
    if (!ConnectionManager::isConnected()) return;

    mavlink_message_t msg;
    mavlink_hil_rc_inputs_raw_t hil_rc_inputs;

    hil_rc_inputs.time_usec = static_cast<uint64_t>(DataRefManager::getFloat("sim/time/total_flight_time_sec") * 1e6);
    hil_rc_inputs.chan1_raw = mapRCChannel(DataRefManager::getFloat("sim/joystick/yoke_roll_ratio"), -1, +1);
    hil_rc_inputs.chan2_raw = mapRCChannel(DataRefManager::getFloat("sim/joystick/yoke_pitch_ratio"), -1, +1);
    hil_rc_inputs.chan3_raw = mapRCChannel(DataRefManager::getFloat("sim/cockpit2/engine/actuators/throttle_ratio_all"), 0, +1);
    hil_rc_inputs.chan4_raw = mapRCChannel(DataRefManager::getFloat("sim/joystick/yoke_heading_ratio"), -1, +1);
    hil_rc_inputs.chan5_raw = mapRCChannel(0, -1, +1);  // Example for a channel with a constant value
    // Map the remaining channels as needed

    constexpr uint8_t RSSI_MAX_VALUE = 255;
    hil_rc_inputs.rssi = RSSI_MAX_VALUE; // Set the RSSI field to its maximum value

    mavlink_msg_hil_rc_inputs_raw_encode(1, 1, &msg, &hil_rc_inputs);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buffer, &msg);

    ConnectionManager::sendData(buffer, len);
}

/**
 * @brief Maps the RC channel value to the MAVLink format.
 *
 * This helper function maps an RC channel value from its original range
 * to the MAVLink expected range of [1000, 2000].
 *
 * @param value The original value of the RC channel.
 * @param min The minimum value of the original range.
 * @param max The maximum value of the original range.
 * @return The mapped value in the MAVLink format.
 */
uint16_t MAVLinkManager::mapRCChannel(float value, float min, float max) {
    return static_cast<uint16_t>(DataRefManager::mapChannelValue(value, min, max, 1000, 2000));
}



/**
 * @brief Receives and processes HIL actuator control messages.
 *
 * This function parses the received buffer for MAVLink messages and processes
 * the HIL actuator control messages by extracting and storing the relevant data.
 *
 * @param buffer Pointer to the received data buffer.
 * @param size Size of the received data buffer.
 */
void MAVLinkManager::receiveHILActuatorControls(uint8_t* buffer, int size) {
    if (!ConnectionManager::isConnected()) return;

    mavlink_message_t msg;
    mavlink_status_t status;
    for (int i = 0; i < size; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
            handleReceivedMessage(msg);
        }
    }
}

/**
 * @brief Handles the received MAVLink message.
 *
 * This function checks the type of the received MAVLink message and processes
 * it accordingly.
 *
 * @param msg The received MAVLink message.
 */
void MAVLinkManager::handleReceivedMessage(const mavlink_message_t& msg) {
    //XPLMDebugString(("px4xplane: Received msgid =  " + std::to_string(msg.msgid) + "\n").c_str());
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
        processHILActuatorControlsMessage(msg);
        break;
        // Handle other MAVLink message types if needed
    default:
        // Handle unrecognized message types if needed
        break;
    }
}

/**
 * Processes the High-Level (HIL) actuator control message received via MAVLink.
 *
 * This function decodes the HIL actuator control message from MAVLink and updates the
 * hilActuatorControlsData member variable. The processing is based on the type of airframe
 * configured (multirotor or fixed-wing). For multirotors, it maps the controls to the correct
 * motors based on the motor mappings specified in ConfigManager. For fixed-wing or other types,
 * the received controls are directly used without modification.
 *
 * This function is essential for translating the actuator control commands received from the
 * autopilot (sent via MAVLink) into a format that can be understood and used by the simulation
 * environment (X-Plane).
 *
 * @param msg The received HIL actuator control MAVLink message, containing control information for actuators.
 */
void MAVLinkManager::processHILActuatorControlsMessage(const mavlink_message_t& msg) {
    mavlink_hil_actuator_controls_t hil_actuator_controls;
    mavlink_msg_hil_actuator_controls_decode(&msg, &hil_actuator_controls);

    // Store the timestamp and mode information
    MAVLinkManager::hilActuatorControlsData.timestamp = hil_actuator_controls.time_usec;
    MAVLinkManager::hilActuatorControlsData.mode = hil_actuator_controls.mode;
    MAVLinkManager::hilActuatorControlsData.flags = hil_actuator_controls.flags;

    // Check the configuration type code
    uint8_t configTypeCode = ConfigManager::getConfigTypeCode();

    if (ConfigManager::getConfigTypeCode() == 1) {
        // For multirotor, map the controls based on the motor mappings from ConfigManager
        for (int i = 0; i < 8; ++i) {
            int xPlaneMotorNumber = ConfigManager::getXPlaneMotorFromPX4(i + 1);
            if (xPlaneMotorNumber != -1) {
                MAVLinkManager::hilActuatorControlsData.controls[xPlaneMotorNumber - 1] = hil_actuator_controls.controls[i];
            }
        }
    }
    else {
        // For fixed-wing or other types, directly use the received controls
        for (int i = 0; i < 16; i++) {
            MAVLinkManager::hilActuatorControlsData.controls[i] = hil_actuator_controls.controls[i];
            // Debug message for each channel
           /* XPLMDebugString(("px4xplane: Fixed-wing actuator channel " + std::to_string(i) +
                " value: " + std::to_string(MAVLinkManager::hilActuatorControlsData.controls[i]) + "\n").c_str());*/
        }
    }
}







/**
 * @brief Sets the acceleration data in the HIL_SENSOR message.
 *
 * This function retrieves the aircraft's current acceleration forces from the simulation,
 * multiplies them by the gravitational constant, and sets them in the provided HIL_SENSOR message.
 *
 * @param hil_sensor Reference to the HIL_SENSOR message where the acceleration data will be set.
 */
void MAVLinkManager::setAccelerationData(mavlink_hil_sensor_t& hil_sensor) {
    hil_sensor.xacc = DataRefManager::getFloat("sim/flightmodel/forces/g_axil") * DataRefManager::g_earth * -1;
    hil_sensor.yacc = DataRefManager::getFloat("sim/flightmodel/forces/g_side") * DataRefManager::g_earth * -1;
    hil_sensor.zacc = DataRefManager::getFloat("sim/flightmodel/forces/g_nrml") * DataRefManager::g_earth * -1;
}

/**
 * @brief Sets the gyroscope data in the HIL_SENSOR message.
 *
 * This function retrieves the aircraft's current rotational rates (in radians) from the simulation
 * and sets them in the provided HIL_SENSOR message.
 *
 * @param hil_sensor Reference to the HIL_SENSOR message where the gyroscope data will be set.
 */
void MAVLinkManager::setGyroData(mavlink_hil_sensor_t& hil_sensor) {
    hil_sensor.xgyro = DataRefManager::getFloat("sim/flightmodel/position/Prad");
    hil_sensor.ygyro = DataRefManager::getFloat("sim/flightmodel/position/Qrad");
    hil_sensor.zgyro = DataRefManager::getFloat("sim/flightmodel/position/Rrad");
}

/**
 * @brief Sets the pressure data in the HIL_SENSOR message.
 *
 * This function retrieves the aircraft's current barometric pressure and pressure altitude from the simulation.
 * It adds noise to the barometric pressure reading to simulate sensor inaccuracies.
 * The differential pressure is approximated using the indicated airspeed (IAS), based on the formula for dynamic pressure.
 * The resulting pressure data is set in the provided HIL_SENSOR message.
 *
 * @param hil_sensor Reference to the HIL_SENSOR message where the pressure data will be set.
 */
void MAVLinkManager::setPressureData(mavlink_hil_sensor_t& hil_sensor) {
    float basePressure = DataRefManager::getFloat("sim/weather/barometer_current_inhg") * 33.8639;
    float pressureNoise = noiseDistribution(gen);
  

    // Calculate differential pressure using indicated airspeed
    float ias = DataRefManager::getFloat("sim/flightmodel/position/indicated_airspeed") * 0.514444; //in m/s
    float dynamicPressure = 0.01 * (0.5 * DataRefManager::AirDensitySeaLevel * ias * ias); // in hPa
    hil_sensor.diff_pressure = dynamicPressure;

    hil_sensor.abs_pressure = basePressure + pressureNoise;
    hil_sensor.pressure_alt = DataRefManager::getDouble("sim/flightmodel2/position/pressure_altitude") * 0.3048;

}


/**
 * @brief Sets the magnetic field data in the HIL_SENSOR message.
 *
 * This function retrieves the aircraft's current orientation and position, then uses the precalculated
 * Earth's magnetic field in NED coordinates. It rotates this magnetic field to the body frame of the aircraft
 * and adds noise to simulate real-world magnetometer readings. The resulting magnetic field data is then set
 * in the provided HIL_SENSOR message.
 *
 * @param hil_sensor Reference to the HIL_SENSOR message where the magnetic field data will be set.
 */
void MAVLinkManager::setMagneticFieldData(mavlink_hil_sensor_t& hil_sensor) {
    // Retrieve the aircraft's current heading, roll, and pitch from the simulation
    float yaw_mag = DataRefManager::getFloat("sim/flightmodel/position/mag_psi");
    float roll = DataRefManager::getFloat("sim/flightmodel/position/true_phi");
    float pitch = DataRefManager::getFloat("sim/flightmodel/position/true_theta");

    // Convert the retrieved angles from degrees to radians for further calculations
    float yaw_rad = (yaw_mag+10) * M_PI / 180.0f;
    float roll_rad = roll * M_PI*0 / 180.0f;
    float pitch_rad = pitch * M_PI*0 / 180.0f;

    // Rotate the precalculated Earth's magnetic field from NED to the aircraft's body frame
    Eigen::Vector3f bodyMagneticField = DataRefManager::convertNEDToBody(DataRefManager::earthMagneticFieldNED, roll_rad, pitch_rad, yaw_rad);

    //bodyMagneticField = DataRefManager::earthMagneticFieldNED;

    // Generate random noise values to simulate real-world magnetometer inaccuracies
    float xmagNoise = noiseDistribution_mag(gen);
    float ymagNoise = noiseDistribution_mag(gen);
    float zmagNoise = noiseDistribution_mag(gen);

    // Set the noisy magnetic field readings in the HIL_SENSOR message
    hil_sensor.xmag = bodyMagneticField(0) + xmagNoise;
    hil_sensor.ymag = bodyMagneticField(1) + ymagNoise;
    hil_sensor.zmag = bodyMagneticField(2) + zmagNoise;
}


/**
 * @brief Sets the time and fix type data for the HIL_GPS message.
 *
 * @param hil_gps Reference to the mavlink_hil_gps_t structure to populate.
 */
void MAVLinkManager::setGPSTimeAndFix(mavlink_hil_gps_t& hil_gps) {
    hil_gps.time_usec = static_cast<uint64_t>(DataRefManager::getFloat("sim/time/total_flight_time_sec") * 1e6);
    hil_gps.fix_type = static_cast<uint8_t>(3); // Assuming always a 3D fix in the simulation environment
}

/**
 * @brief Sets the position data for the HIL_GPS message.
 *
 * @param hil_gps Reference to the mavlink_hil_gps_t structure to populate.
 */
void MAVLinkManager::setGPSPositionData(mavlink_hil_gps_t& hil_gps) {
    hil_gps.lat = static_cast<int32_t>(DataRefManager::getFloat("sim/flightmodel/position/latitude") * 1e7);
    hil_gps.lon = static_cast<int32_t>(DataRefManager::getFloat("sim/flightmodel/position/longitude") * 1e7);
    hil_gps.alt = static_cast<int32_t>(DataRefManager::getFloat("sim/flightmodel/position/elevation") * 1e3);
}

/**
 * @brief Sets the accuracy data for the HIL_GPS message.
 *
 * @param hil_gps Reference to the mavlink_hil_gps_t structure to populate.
 */
void MAVLinkManager::setGPSAccuracyData(mavlink_hil_gps_t& hil_gps) {
    hil_gps.eph = static_cast<uint16_t>(20); // Assuming high accuracy due to simulation environment
    hil_gps.epv = static_cast<uint16_t>(20);
    hil_gps.satellites_visible = static_cast<uint16_t>(14);; // Assuming 14 satellites visible in good conditions
}

/**
 * @brief Sets the velocity data for the HIL_GPS message using the OGL coordinate system.
 *                                     !!NOT USED!!
 * This function is now deprecated due to issues with the OGL (OpenGL) coordinate system
 * in the simulation environment not aligning with the NED (North-East-Down) coordinate system
 * consistently. It extracts the velocity data from the simulation environment, which may lead to
 * incorrect velocity readings due to the OGL coordinate frame problems. The velocities in OGL are
 * transformed to the NED coordinate system, which is commonly used in aviation.
 *
 * @param hil_gps Reference to the mavlink_hil_gps_t structure to populate.
 */
void MAVLinkManager::setGPSVelocityDataOGL(mavlink_hil_gps_t& hil_gps) {
    int16_t ogl_vx = DataRefManager::getFloat("sim/flightmodel/position/local_vx") * 100;
    int16_t ogl_vy = DataRefManager::getFloat("sim/flightmodel/position/local_vy") * 100;
    int16_t ogl_vz = DataRefManager::getFloat("sim/flightmodel/position/local_vz") * 100;

    // Coordinate Transformation:
    // The local OGL (OpenGL) coordinate system in the simulation environment is defined as:
    // X: East
    // Y: Up
    // Z: South
    //
    // The NED (North-East-Down) coordinate system, commonly used in aviation, is defined as:
    // X: North
    // Y: East
    // Z: Down
    //
    // The transformation from OGL to NED for velocities is:
    // NED North (X) = -OGL South (Z)
    // NED East  (Y) =  OGL East  (X)
    // NED Down  (Z) = -OGL Up    (Y)

    hil_gps.vn = static_cast<int16_t> ( - 1 * ogl_vz);
    hil_gps.ve = static_cast<int16_t>(ogl_vx);
    hil_gps.vd = static_cast<int16_t>(-1 * ogl_vy);
    hil_gps.vel = static_cast<uint16_t>(DataRefManager::getFloat("sim/flightmodel/position/groundspeed") * 100.0);
}


/**
 * @brief Sets the velocity data for the HIL_GPS message using NED coordinate system.
 *  *                                     !!NOT USED!!
 *
 * This function calculates the North (Vn), East (Ve), and Down (Vd) components of velocity
 * in the NED coordinate system using groundspeed, hpath, and vpath datarefs provided by the
 * simulation environment. Groundspeed is the total velocity, while hpath and vpath are the
 * horizontal and vertical path angles in degrees, respectively.
 *
 * The function converts these angles and groundspeed into the NED frame velocities, suitable
 * for MAVLink communication with PX4.
 *
 * @param hil_gps Reference to the mavlink_hil_gps_t structure to populate.
 */
void MAVLinkManager::setGPSVelocityDataByPath(mavlink_hil_gps_t& hil_gps) {
    float groundspeed = DataRefManager::getFloat("sim/flightmodel/position/groundspeed");
    float hpath = DataRefManager::getFloat("sim/flightmodel/position/hpath") * (M_PI / 180.0f);
    float vpath = DataRefManager::getFloat("sim/flightmodel/position/vpath") * (M_PI / 180.0f);

    // Calculate NED velocity components
    float Vn = groundspeed * cos(vpath) * cos(hpath);
    float Ve = groundspeed * cos(vpath) * sin(hpath);
    float Vd = groundspeed * sin(vpath);

    // Populate the MAVLink message with NED velocities (converted to cm/s)
    hil_gps.vn = static_cast<int16_t>(Vn * 100.0f);
    hil_gps.ve = static_cast<int16_t>(Ve * 100.0f);
    hil_gps.vd = static_cast<int16_t>(-Vd * 100.0f); // Negate if positive vpath means climbing

    // Calculate and set the total velocity
    hil_gps.vel = static_cast<uint16_t>(sqrt(Vn * Vn + Ve * Ve + Vd * Vd) * 100.0f);
}


/**
 * @brief Sets the velocity data for the HIL_GPS message using the NED coordinate system.
 *
 * This function temporarily uses the TCAS system velocities (sim/cockpit2/tcas/targets/position/vx, vy, vz)
 * from X-Plane for calculating the aircraft's velocity in the NED coordinate system. The TCAS velocities
 * are in the OGL coordinate system and are transformed to NED. This approach is a temporary workaround
 * to address issues with other velocity datarefs and will be replaced by a more accurate solution in the future.
 *
 * Note: TCAS velocities are used here for their availability and ease of access, but they might not
 * provide the most accurate representation of the aircraft's true velocity, especially in complex flight
 * scenarios or with certain aircraft models.
 *
 * @param hil_gps Reference to the mavlink_hil_gps_t structure to populate.
 */
void MAVLinkManager::setGPSVelocityData(mavlink_hil_gps_t& hil_gps) {
    // Read TCAS velocities from X-Plane
    std::vector<float> vxArray = DataRefManager::getFloatArray("sim/cockpit2/tcas/targets/position/vx");
    std::vector<float> vyArray = DataRefManager::getFloatArray("sim/cockpit2/tcas/targets/position/vy");
    std::vector<float> vzArray = DataRefManager::getFloatArray("sim/cockpit2/tcas/targets/position/vz");

    // Assuming the first element in each array contains the required velocity
    float vx = vxArray[0];
    float vy = vyArray[0];
    float vz = vzArray[0];

    // Transform from OGL to NED
    float Vn = -vz;  // North is negative South
    float Ve = vx;   // East remains the same
    float Vd = -vy;  // Down is negative Up

    // Populate the MAVLink message with NED velocities (converted to cm/s)
    hil_gps.vn = static_cast<int16_t>(Vn * 100.0f);
    hil_gps.ve = static_cast<int16_t>(Ve * 100.0f);
    hil_gps.vd = static_cast<int16_t>(Vd * 100.0f);

    // Calculate the total velocity
    hil_gps.vel = static_cast<uint16_t>(sqrt(Vn * Vn + Ve * Ve + Vd * Vd) * 100.0f);
}





/**
 * @brief Sets the heading data for the HIL_GPS message.
 *
 * @param hil_gps Reference to the mavlink_hil_gps_t structure to populate.
 */
void MAVLinkManager::setGPSHeadingData(mavlink_hil_gps_t& hil_gps) {
    uint16_t cog = static_cast<uint16_t>(DataRefManager::getFloat("sim/cockpit2/gauges/indicators/ground_track_mag_copilot") * 100);
    //uint16_t cog = static_cast<uint16_t>(DataRefManager::getFloat("sim/flightmodel/position/hpath") * 100.0f);
    hil_gps.cog = (cog == 36000) ? static_cast < uint16_t>(0001) : cog;
    uint16_t yaw = static_cast<uint16_t>(DataRefManager::getFloat("sim/flightmodel/position/mag_psi") * 100.0f);
    hil_gps.yaw = (yaw == 0) ? static_cast < uint16_t>(35999) : yaw;
}