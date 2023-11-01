#include "MAVLinkManager.h"
#include "mavlink/c_library_v2/common/mavlink.h"
#include "ConnectionManager.h"
#include "DataRefManager.h"
#include <random>
#include <vector>
#include "XPLMUtilities.h"
#include <Eigen/Dense>
#include "XYZgeomag/src/XYZgeomag.hpp"
#include <cmath>  // for trigonometric functions and M_PI
#include <tuple>  // for std::tuple
#include <Eigen/Geometry>
#include <Eigen/Core>



// Define and initialize the static member
MAVLinkManager::HILActuatorControlsData MAVLinkManager::hilActuatorControlsData = {};

// Function to convert Euler angles (roll, pitch, yaw) to a rotation matrix
Eigen::Matrix3f eulerToRotationMatrix(float roll, float pitch, float yaw) {
    Eigen::Matrix3f R;
    float sr = sin(roll), cr = cos(roll);
    float sp = sin(pitch), cp = cos(pitch);
    float sy = sin(yaw), cy = cos(yaw);

    R << cp * cy, (sr * sp * cy) - (cr * sy), (cr * sp * cy) + (sr * sy),
        cp* sy, (sr * sp * sy) + (cr * cy), (cr * sp * sy) - (sr * cy),
        -sp, sr* cp, cr* cp;

    return R;
}





/**
 * @brief Calculate the magnetic field vector in the body frame of a vehicle.
 *
 * This function takes into account the discrepancy between the convention of magnetic heading
 * (which increases clockwise) and the right-hand rule used in rotations.
 *
 * @param lat Latitude in degrees.
 * @param lon Longitude in degrees.
 * @param alt Altitude in meters above sea level.
 * @param roll Roll angle in radians (rotation about the X-axis).
 * @param pitch Pitch angle in radians (rotation about the Y-axis).
 * @param yaw Yaw angle in radians with respect to Magnetic North (rotation about the Z-axis).
 *            Positive yaw means turning to the right (clockwise).
 * @return Eigen::Vector3f The magnetic field vector in the body frame, in Gauss.
 */
Eigen::Vector3f calculateMagneticVector(float lat, float lon, float alt, float roll, float pitch, float yaw) {
    // Convert latitude and longitude from degrees to radians
    float latRad = lat * M_PI / 180.0;
    float lonRad = lon * M_PI / 180.0;

    // Convert geodetic coordinates to geocentric Cartesian coordinates
    geomag::Vector position = geomag::geodetic2ecef(latRad, lonRad, alt);

    // Calculate magnetic field vector in Earth's geocentric frame of reference
    geomag::Vector mag_field = geomag::GeoMag(2022.5, position, geomag::WMM2020);

    // Convert the magnetic field vector from geocentric cartesian coordinates to magnetic elements (NED components)
    geomag::Elements nedElements = geomag::magField2Elements(mag_field, latRad, lonRad);

    // Create rotation matrix using roll, pitch, and yaw (ZYX Euler angle rotation sequence).
    // Note: We negate the yaw angle to account for the discrepancy between the convention of magnetic heading 
    // and the right-hand rule used in rotations.
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(-yaw, Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f rotationMatrix = (rollAngle * pitchAngle * yawAngle).matrix();

    // Apply rotation matrix to the magnetic field vector in the NED frame to transform it to the body frame
    Eigen::Vector3f bodyVector = rotationMatrix * Eigen::Vector3f(nedElements.north, nedElements.east, nedElements.down);

    // Convert from nT (nanoTesla) to Gauss (1 Tesla = 10,000 Gauss, 1 nT = 0.00001 Gauss)
    bodyVector *= 0.00001;

    // Return the magnetic field vector in the body frame
    return bodyVector;
}



void MAVLinkManager::sendHILSensor() {
    if (!ConnectionManager::isConnected()) return;

    mavlink_message_t msg;
    mavlink_hil_sensor_t hil_sensor;

    hil_sensor.time_usec = static_cast<uint64_t>(DataRefManager::getFloat("sim/time/total_flight_time_sec") * 1e6);
    hil_sensor.id = uint8_t(0);


    const float DEG_TO_RAD = 3.14159265358979323846 / 180.0;  // Conversion factor from degrees to radians
    const float GRAVITY = 9.81;  // Gravitational acceleration in m/s^2

    // Get the accelerations from X-Plane's OGL frame
    float ax_OGL = DataRefManager::getFloat("sim/flightmodel/position/local_ax");
    float ay_OGL = DataRefManager::getFloat("sim/flightmodel/position/local_ay");
    float az_OGL = DataRefManager::getFloat("sim/flightmodel/position/local_az");




    hil_sensor.xacc = DataRefManager::getFloat("sim/flightmodel/forces/g_axil") * DataRefManager::g_earth * -1;
    hil_sensor.yacc = DataRefManager::getFloat("sim/flightmodel/forces/g_side") * DataRefManager::g_earth * -1;
    hil_sensor.zacc = DataRefManager::getFloat("sim/flightmodel/forces/g_nrml") * DataRefManager::g_earth * -1;



    hil_sensor.xgyro = DataRefManager::getFloat("sim/flightmodel/position/Prad");
    hil_sensor.ygyro = DataRefManager::getFloat("sim/flightmodel/position/Qrad") ;
    hil_sensor.zgyro = DataRefManager::getFloat("sim/flightmodel/position/Rrad") ;


    // Get the Location 
    float latitude = DataRefManager::getFloat("sim/flightmodel/position/latitude");
    float longitude = DataRefManager::getFloat("sim/flightmodel/position/longitude");
    float altitude = DataRefManager::getFloat("sim/flightmodel/position/elevation");

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

    // Get the heading, roll, and pitch
    float yaw_mag = DataRefManager::getFloat("sim/flightmodel/position/mag_psi");
    float roll = DataRefManager::getFloat("sim/flightmodel/position/phi");
    float pitch = DataRefManager::getFloat("sim/flightmodel/position/theta");

    // Convert to radians
        float yaw_rad = yaw_mag * M_PI / 180.0f;
    float roll_rad = roll * M_PI / 180.0f;
    float pitch_rad = pitch * M_PI / 180.0f;

    // Generate random noise values for the magnetometer data
    std::random_device rd_mag;
    std::mt19937 gen_mag(rd_mag());
    std::normal_distribution<float> noiseDistribution_mag(0.0f, 0.01f); // Adjust the standard deviation as needed


    // Call the calculateMagneticVector function
    Eigen::Vector3f B_body = calculateMagneticVector(latitude, longitude, altitude, roll_rad, pitch_rad, yaw_rad);

    // Set the magnetic field in the HIL_SENSOR message
    hil_sensor.xmag = B_body(0) + noiseDistribution_mag(gen_mag);
    hil_sensor.ymag = B_body(1) + noiseDistribution_mag(gen_mag);
    hil_sensor.zmag = B_body(2) + noiseDistribution_mag(gen_mag);



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

    float ogl_vx = DataRefManager::getFloat("sim/flightmodel/position/local_vx")*100;
    float ogl_vy = DataRefManager::getFloat("sim/flightmodel/position/local_vy")*100;
    float ogl_vz = DataRefManager::getFloat("sim/flightmodel/position/local_vz")*100;

    float roll_rad = DataRefManager::getFloat("sim/flightmodel/position/phi") * M_PI / 180.0;
    float pitch_rad = DataRefManager::getFloat("sim/flightmodel/position/theta") * M_PI / 180.0;
    float yaw_rad = DataRefManager::getFloat("sim/flightmodel/position/psi") * M_PI / 180.0;


    float hpath = DataRefManager::getFloat(" sim/flightmodel/position/hpath");
    float vpath = DataRefManager::getFloat(" sim/flightmodel/position/vpath");
    float groundspeed = DataRefManager::getFloat(" sim/flightmodel/position/groundspeed");




    //local OGL is :
    // X: East
    // Y: Up
    // Z: South
    hil_gps.vn = -1*ogl_vz;
    hil_gps.ve = ogl_vx;
    hil_gps.vd = -1* ogl_vy;
    hil_gps.vel = groundspeed*100; // converted to cm/s

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
    hil_rc_inputs.chan5_raw = static_cast<uint16_t>(DataRefManager::mapChannelValue(0, -1, +1, 1000, 2000));
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
