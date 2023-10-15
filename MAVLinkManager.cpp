#include "MAVLinkManager.h"
#include "mavlink/c_library_v2/common/mavlink.h"
#include "ConnectionManager.h"
#include "DataRefManager.h"
#include <random>
#include <vector>
#include "XPLMUtilities.h"


// Define and initialize the static member
MAVLinkManager::HILActuatorControlsData MAVLinkManager::hilActuatorControlsData = {};


void MAVLinkManager::sendHILSensor() {
    if (!ConnectionManager::isConnected()) return;

    mavlink_message_t msg;
    mavlink_hil_sensor_t hil_sensor;

    hil_sensor.time_usec = static_cast<uint64_t>(DataRefManager::getFloat("sim/time/total_flight_time_sec") * 1e6);
    hil_sensor.id = uint8_t(0);
    hil_sensor.xacc = DataRefManager::getFloat("sim/flightmodel/forces/g_axil") * DataRefManager::g_earth;
    hil_sensor.yacc = DataRefManager::getFloat("sim/flightmodel/forces/g_side") * DataRefManager::g_earth;
    hil_sensor.zacc = DataRefManager::getFloat("sim/flightmodel/forces/g_nrml") * DataRefManager::g_earth*-1;

  /*  float ogl_ax = DataRefManager::getFloat("sim/flightmodel/position/local_ax");
    float ogl_ay = DataRefManager::getFloat("sim/flightmodel/position/local_ay");
    float ogl_az = DataRefManager::getFloat("sim/flightmodel/position/local_az");

    float roll_rad = DataRefManager::getFloat("sim/flightmodel/position/phi") * M_PI / 180.0;
    float pitch_rad = DataRefManager::getFloat("sim/flightmodel/position/theta") * M_PI / 180.0;
    float yaw_rad = DataRefManager::getFloat("sim/flightmodel/position/psi") * M_PI / 180.0;

    float ned_an, ned_ae, ned_ad;
    std::tie(ned_an, ned_ae, ned_ad) = DataRefManager::convertOGLtoNED(ogl_ax, ogl_ay, ogl_az, roll_rad, pitch_rad, yaw_rad);
    hil_sensor.xacc = ned_an;
    hil_sensor.yacc = ned_ae;
    hil_sensor.zacc = ned_ad;*/



    hil_sensor.xgyro = DataRefManager::getFloat("sim/flightmodel/position/Prad");
    hil_sensor.ygyro = DataRefManager::getFloat("sim/flightmodel/position/Qrad");
    hil_sensor.zgyro = DataRefManager::getFloat("sim/flightmodel/position/Rrad");

    // Get the base pressure value
    float basePressure = DataRefManager::getFloat("sim/weather/barometer_current_inhg") * 33.8639;

    // Generate a random noise value
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> noiseDistribution(0.0f, 0.01f); // Adjust the standard deviation as needed

    // Add noise to the pressure value
    float pressureNoise = noiseDistribution(gen);
    hil_sensor.abs_pressure = basePressure + pressureNoise;

    hil_sensor.pressure_alt = DataRefManager::getDouble("sim/flightmodel2/position/pressure_altitude") * 0.3048;
    // Find suitable datarefs or provide default values
    hil_sensor.diff_pressure = 0;

    // Get the heading
    float yaw = DataRefManager::getFloat("sim/flightmodel/position/psi");

    // Convert yaw to radians
    float yaw_rad = yaw * M_PI / 180.0f;

    // Generate random noise values for the magnetometer data
    std::random_device rd_mag;
    std::mt19937 gen_mag(rd_mag());
    std::normal_distribution<float> noiseDistribution_mag(0.0f, 0.02f); // Adjust the standard deviation as needed

    // Magnetic field strengths in Gauss
    float f_xmag_gauss = (28158.2 / 1e5) * cos(yaw_rad) + noiseDistribution_mag(gen_mag); // North Component
    float f_ymag_gauss = (2575.1 / 1e5) * sin(yaw_rad) + noiseDistribution_mag(gen_mag); // East Component
    float f_zmag_gauss = (38411.1 / 1e5) + noiseDistribution_mag(gen_mag); // Vertical Component

    hil_sensor.xmag = f_xmag_gauss;
    hil_sensor.ymag = f_ymag_gauss;
    hil_sensor.zmag = f_zmag_gauss;

        // Add noise to the magnetic vector
       /* hil_sensor.xmag = 0;
        hil_sensor.ymag = 0;
        hil_sensor.zmag = 0;*/

 


    hil_sensor.temperature = DataRefManager::getFloat("sim/cockpit2/temperature/outside_air_temp_degc");

    // Now set the bits for the fields you are updating.
    // Refer to the provided enum values for the fields_updated flags.
    //https://mavlink.io/en/messages/common.html#HIL_SENSOR_UPDATED_FLAGS
    uint32_t fields_updated = 0; // Start with a bitmask of all zeros
    fields_updated |= (1 << 0); // HIL_SENSOR_UPDATED_XACC, the bit shift corresponds to setting the 0th bit to 1
    fields_updated |= (1 << 1); // HIL_SENSOR_UPDATED_YACC, the bit shift corresponds to setting the 1st bit to 1
    fields_updated |= (1 << 2); // HIL_SENSOR_UPDATED_ZACC, the bit shift corresponds to setting the 2nd bit to 1
    fields_updated |= (1 << 3); // HIL_SENSOR_UPDATED_XGYRO, the bit shift corresponds to setting the 3rd bit to 1
    fields_updated |= (1 << 4); // HIL_SENSOR_UPDATED_YGYRO, the bit shift corresponds to setting the 4th bit to 1
    fields_updated |= (1 << 5); // HIL_SENSOR_UPDATED_ZGYRO, the bit shift corresponds to setting the 5th bit to 1
    fields_updated |= (1 << 6); // HIL_SENSOR_UPDATED_XMAG
    fields_updated |= (1 << 7); // HIL_SENSOR_UPDATED_YMAG
    fields_updated |= (1 << 8); // HIL_SENSOR_UPDATED_ZMAG
    fields_updated |= (1 << 9); // HIL_SENSOR_UPDATED_ABS_PRESSURE, the bit shift corresponds to setting the 9th bit to 1
    fields_updated |= (1 << 11); // HIL_SENSOR_UPDATED_PRESSURE_ALT, the bit shift corresponds to setting the 11th bit to 1
    fields_updated |= (1 << 12); // HIL_SENSOR_UPDATED_TEMPERATURE, the bit shift corresponds to setting the 12th bit to 1

    hil_sensor.fields_updated = fields_updated;

    // Finally, assign the bitmask to the hil_sensor message object
    hil_sensor.fields_updated = fields_updated;

    mavlink_msg_hil_sensor_encode(1, 1, &msg, &hil_sensor);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buffer, &msg);
    ConnectionManager::sendData(buffer, len);
}
void MAVLinkManager::sendHILGPS() {
    if (!ConnectionManager::isConnected()) return;

    mavlink_message_t msg;
    mavlink_hil_gps_t hil_gps;

    hil_gps.time_usec = static_cast<uint64_t>(DataRefManager::getFloat("sim/time/total_flight_time_sec") * 1e6); // converted to us
    hil_gps.fix_type = 3; // 3: 3D fix, assuming we always have a 3D fix in the simulation environment

    hil_gps.lat = static_cast<int32_t>(DataRefManager::getFloat("sim/flightmodel/position/latitude") * 1e7); // converted to degE7
    hil_gps.lon = static_cast<int32_t>(DataRefManager::getFloat("sim/flightmodel/position/longitude") * 1e7); // converted to degE7

    hil_gps.alt = static_cast<int32_t>(DataRefManager::getFloat("sim/flightmodel/position/elevation") * 1e3); // converted to mm
    hil_gps.eph = 20; // Example: equivalent to 0.2 in HDOP, assuming very high accuracy due to simulation environment
    hil_gps.epv = 20; // Example: equivalent to 0.2 in VDOP, assuming very high accuracy due to simulation environment
    hil_gps.satellites_visible = 12; // Example: assuming we have 12 satellites visible as it's common in good conditions.

    float ogl_vx = DataRefManager::getFloat("sim/flightmodel/position/local_vx");
    float ogl_vy = DataRefManager::getFloat("sim/flightmodel/position/local_vy");
    float ogl_vz = DataRefManager::getFloat("sim/flightmodel/position/local_vz");

    float roll_rad = DataRefManager::getFloat("sim/flightmodel/position/phi") * M_PI / 180.0;
    float pitch_rad = DataRefManager::getFloat("sim/flightmodel/position/theta") * M_PI / 180.0;
    float yaw_rad = DataRefManager::getFloat("sim/flightmodel/position/psi") * M_PI / 180.0;

    float ned_vn, ned_ve, ned_vd;
    std::tie(ned_vn, ned_ve, ned_vd) = DataRefManager::convertOGLtoNED(ogl_vx, ogl_vy, ogl_vz, roll_rad, pitch_rad, yaw_rad);


    hil_gps.vn = ned_vn;
    hil_gps.ve = ned_ve;
    hil_gps.vd = ned_vd;
    hil_gps.vel = static_cast<int16_t>(DataRefManager::getFloat("sim/flightmodel/position/groundspeed") * 100); // converted to cm/s

    hil_gps.cog = static_cast<uint16_t>(DataRefManager::getFloat("sim/cockpit2/gauges/indicators/ground_track_mag_copilot") * 100); // converted to cdeg

    hil_gps.id = 0; // 0 indexed GPS ID for single GPS
    hil_gps.yaw = DataRefManager::getFloat("sim/flightmodel/position/psi") * 100;

    mavlink_msg_hil_gps_encode(1, 1, &msg, &hil_gps);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buffer, &msg);
    ConnectionManager::sendData(buffer, len);
}


void MAVLinkManager::sendHILStateQuaternion() {

    if (!ConnectionManager::isConnected()) return;

    mavlink_message_t msg;
    mavlink_hil_state_quaternion_t hil_state;

    hil_state.time_usec = static_cast<uint64_t>(DataRefManager::getFloat("sim/time/total_flight_time_sec") * 1e6);
    std::vector<float> q = DataRefManager::getFloatArray("sim/flightmodel/position/q");
    hil_state.attitude_quaternion[0] = q[0];
    hil_state.attitude_quaternion[1] = q[1];
    hil_state.attitude_quaternion[2] = q[2];
    hil_state.attitude_quaternion[3] = q[3];

    hil_state.rollspeed = DataRefManager::getFloat("sim/flightmodel/position/Prad");
    hil_state.pitchspeed = DataRefManager::getFloat("sim/flightmodel/position/Qrad");
    hil_state.yawspeed = DataRefManager::getFloat("sim/flightmodel/position/Rrad");

    hil_state.lat = static_cast<int32_t>(DataRefManager::getFloat("sim/flightmodel/position/latitude") * 1e7);
    hil_state.lon = static_cast<int32_t>(DataRefManager::getFloat("sim/flightmodel/position/longitude") * 1e7);

    hil_state.alt = static_cast<int32_t>(DataRefManager::getFloat("sim/flightmodel/position/elevation") * 1e3);

    hil_state.vx = static_cast<int16_t>(DataRefManager::getFloat("sim/flightmodel/position/local_vx") * 100);
    hil_state.vy = static_cast<int16_t>(DataRefManager::getFloat("sim/flightmodel/position/local_vy") * 100);
    hil_state.vz = static_cast<int16_t>(DataRefManager::getFloat("sim/flightmodel/position/local_vz") * 100);

    hil_state.ind_airspeed = static_cast<uint16_t>(DataRefManager::getFloat("sim/flightmodel/position/indicated_airspeed") * 100);
    hil_state.true_airspeed = static_cast<uint16_t>(DataRefManager::getFloat("sim/flightmodel/position/true_airspeed") * 100);

    hil_state.xacc = static_cast<int16_t>(DataRefManager::getFloat("sim/flightmodel/position/local_ax") * 1000 / 9.81);
    hil_state.yacc = static_cast<int16_t>(DataRefManager::getFloat("sim/flightmodel/position/local_ay") * 1000 / 9.81);
    hil_state.zacc = static_cast<int16_t>(DataRefManager::getFloat("sim/flightmodel/position/local_az") * 1000 / 9.81);

    mavlink_msg_hil_state_quaternion_encode(1, 1, &msg, &hil_state);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buffer, &msg);

    ConnectionManager::sendData(buffer, len);
}
void MAVLinkManager::sendHILRCInputs() {
    if (!ConnectionManager::isConnected()) return;

    mavlink_message_t msg;
    mavlink_hil_rc_inputs_raw_t hil_rc_inputs;

    hil_rc_inputs.time_usec = static_cast<uint64_t>(DataRefManager::getFloat("sim/time/total_flight_time_sec") * 1e6);
    hil_rc_inputs.chan1_raw = static_cast<uint16_t>(DataRefManager::mapChannelValue(DataRefManager::getFloat("sim/joystick/yoke_roll_ratio"),-1,+1, 1000, 2000));
    hil_rc_inputs.chan2_raw = static_cast<uint16_t>(DataRefManager::mapChannelValue(DataRefManager::getFloat("sim/joystick/yoke_pitch_ratio"), -1, +1, 1000, 2000));
    hil_rc_inputs.chan3_raw = static_cast<uint16_t>(DataRefManager::mapChannelValue(DataRefManager::getFloat("sim/cockpit2/engine/actuators/throttle_ratio_all"),0, +1, 1000, 2000));
    hil_rc_inputs.chan4_raw = static_cast<uint16_t>(DataRefManager::mapChannelValue(DataRefManager::getFloat("sim/joystick/yoke_heading_ratio"), -1, +1, 1000, 2000));
    hil_rc_inputs.chan4_raw = static_cast<uint16_t>(DataRefManager::mapChannelValue(0, -1, +1, 1000, 2000));
    // Map the remaining channels as needed

    hil_rc_inputs.rssi = 255; // Set the RSSI field to a constant value if not available

    mavlink_msg_hil_rc_inputs_raw_encode(1, 1, &msg, &hil_rc_inputs);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buffer, &msg);

    ConnectionManager::sendData(buffer, len);
}


void MAVLinkManager::receiveHILActuatorControls(uint8_t* buffer, int size) {
    if (!ConnectionManager::isConnected()) return;

    // Parse the received data
    mavlink_message_t msg;
    mavlink_status_t status;
    for (int i = 0; i < size; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
            // Handle the received message
            switch (msg.msgid) {
            case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS: {
                mavlink_hil_actuator_controls_t hil_actuator_controls;
                mavlink_msg_hil_actuator_controls_decode(&msg, &hil_actuator_controls);

                // Extract the relevant information from the message
                uint64_t timestamp = hil_actuator_controls.time_usec;
                float controls[16];
                for (int i = 0; i < 16; ++i) {
                    controls[i] = hil_actuator_controls.controls[i];
                }
                uint8_t mode = hil_actuator_controls.mode;
                uint64_t flags = hil_actuator_controls.flags;

                // Store the received data in a suitable data structure or variable
                // For example, you can store it in a class member variable or pass it to another function for further processing

                                     // Store the received data in the member variable
                MAVLinkManager::hilActuatorControlsData.timestamp = timestamp;
                for (int i = 0; i < 16; ++i) {
                    MAVLinkManager::hilActuatorControlsData.controls[i] = controls[i];
                }
                MAVLinkManager::hilActuatorControlsData.mode = mode;
                MAVLinkManager::hilActuatorControlsData.flags = flags;

                // Example: Pass the received data to another function for further processing

                break;
            }
                                                     // Handle other MAVLink message types if needed

            default:
                // Handle unrecognized message types if needed
                break;
            }
        }
    }
}
