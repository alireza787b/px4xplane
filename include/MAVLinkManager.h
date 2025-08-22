#include <cstdint>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include "../lib/mavlink/c_library_v2/common/mavlink.h"
#include "../lib/XYZgeomag/src/XYZgeomag.hpp"
#include <random>
#include <cmath>  // for trigonometric functions and M_PI


class MAVLinkManager {
public:
    static void sendHILSensor(uint8_t sensor_id);
    static void sendHILGPS();
    static void sendHILStateQuaternion();
    static void sendHILRCInputs();
    static void setGyroData(mavlink_hil_sensor_t& hil_sensor);
    static void setPressureData(mavlink_hil_sensor_t& hil_sensor);
    static void setMagneticFieldData(mavlink_hil_sensor_t& hil_sensor);
    static void receiveHILActuatorControls(uint8_t* buffer, int size);
    static Eigen::Vector3f computeAcceleration();
    static void setAccelerationData(mavlink_hil_sensor_t& hil_sensor);

    struct HILActuatorControlsData {
        uint64_t timestamp;
        float controls[16];
        uint8_t mode;
        uint64_t flags;
    };
    static HILActuatorControlsData hilActuatorControlsData; // Public variable to store the received data




private:
    static std::random_device rd;
    static std::mt19937 gen;
    static std::normal_distribution<float> noiseDistribution;
    static std::normal_distribution<float> noiseDistribution_mag;
    static std::normal_distribution<float> highFreqNoise;
    static std::normal_distribution<float> lowFreqNoise;
    static void setGPSTimeAndFix(mavlink_hil_gps_t& hil_gps);
    static void setGPSPositionData(mavlink_hil_gps_t& hil_gps);
    static void setGPSAccuracyData(mavlink_hil_gps_t& hil_gps);
    static void setGPSVelocityData(mavlink_hil_gps_t& hil_gps);
    static void setGPSVelocityDataOGL(mavlink_hil_gps_t& hil_gps);
    static void setGPSVelocityDataByPath(mavlink_hil_gps_t& hil_gps);
    static void setGPSHeadingData(mavlink_hil_gps_t& hil_gps);
    static void handleReceivedMessage(const mavlink_message_t& msg);
    static void processHILActuatorControlsMessage(const mavlink_message_t& msg);
    static void populateHILStateQuaternion(mavlink_hil_state_quaternion_t& hil_state);
    static void sendData(const mavlink_message_t& msg);
    static uint16_t mapRCChannel(float value, float min, float max);
    static void updateMagneticFieldIfExceededTreshold(const mavlink_hil_gps_t& hil_gps);

    
};
