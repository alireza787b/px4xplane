#include "MAVLinkManager.h"
#include "mavlink/c_library_v2/all/mavlink.h"
#include "ConnectionManager.h"

void MAVLinkManager::sendHILSensor() {
    if (!ConnectionManager::isConnected()) return;

    mavlink_message_t msg;
    mavlink_hil_sensor_t hil_sensor;

    // Fill in the hil_sensor data with dummy values
    hil_sensor.time_usec = 0;
    hil_sensor.xacc = 1.0f;
    hil_sensor.yacc = 1.0f;
    hil_sensor.zacc = 1.0f;
    hil_sensor.xgyro = 1.0f;
    hil_sensor.ygyro = 1.0f;
    hil_sensor.zgyro = 1.0f;
    hil_sensor.xmag = 1.0f;
    hil_sensor.ymag = 1.0f;
    hil_sensor.zmag = 1.0f;
    hil_sensor.abs_pressure = 1.0f;
    hil_sensor.diff_pressure = 1.0f;
    hil_sensor.pressure_alt = 1.0f;
    hil_sensor.temperature = 20.0f;
    hil_sensor.fields_updated = 0;

    mavlink_msg_hil_sensor_encode(1, 1, &msg, &hil_sensor);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buffer, &msg);
    ConnectionManager::sendData(buffer, len);
}