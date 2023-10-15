#include <cstdint>

class MAVLinkManager {
public:
    static void sendHILSensor();
    static void sendHILGPS();
    static void sendHILStateQuaternion();
    static void sendHILRCInputs();
    static void receiveHILActuatorControls(uint8_t* buffer, int size);

    struct HILActuatorControlsData {
        uint64_t timestamp;
        float controls[16];
        uint8_t mode;
        uint64_t flags;
    };

    static HILActuatorControlsData hilActuatorControlsData; // Public variable to store the received data
};
