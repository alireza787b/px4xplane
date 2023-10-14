#include "MAVLinkManager.h"
#include "mavlink/c_library_v2/common/mavlink.h"
#include "ConnectionManager.h"
#include "DataRefManager.h"
#include <random>
#include <vector>

void MAVLinkManager::sendHILSensor() {
    if (!ConnectionManager::isConnected()) return;

    mavlink_message_t msg;
    mavlink_hil_sensor_t hil_sensor;

    hil_sensor.time_usec = static_cast<uint64_t>(DataRefManager::getFloat("sim/time/total_flight_time_sec") * 1e6);
    hil_sensor.id = uint8_t(0);
    hil_sensor.xacc = DataRefManager::getFloat("sim/flightmodel/forces/g_axil") * DataRefManager::g_earth;
    hil_sensor.yacc = DataRefManager::getFloat("sim/flightmodel/forces/g_side") * DataRefManager::g_earth;
    hil_sensor.zacc = DataRefManager::getFloat("sim/flightmodel/forces/g_nrml") * DataRefManager::g_earth;

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
    hil_sensor.xmag = 0;
    hil_sensor.ymag = 0;
    hil_sensor.zmag = 0;

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

    hil_gps.vn = static_cast<int16_t>(DataRefManager::getFloat("sim/flightmodel/position/local_vx") * 100); // converted to cm/s
    hil_gps.ve = static_cast<int16_t>(DataRefManager::getFloat("sim/flightmodel/position/local_vy") * 100); // converted to cm/s
    hil_gps.vd = static_cast<int16_t>(DataRefManager::getFloat("sim/flightmodel/position/local_vz") * 100); // converted to cm/s

    hil_gps.cog = static_cast<uint16_t>(DataRefManager::getFloat("sim/cockpit2/gauges/indicators/ground_track_mag_copilot") * 100); // converted to cdeg

    hil_gps.vel = UINT16_MAX; // Assuming velocity is unknown as per MAVLink definition
    hil_gps.id = 0; // 0 indexed GPS ID for single GPS
    hil_gps.yaw = UINT16_MAX; // Yaw not available

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