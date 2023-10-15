#include "mavlink.h"
struct HILActuatorControlsData {
    uint64_t timestamp;
    float controls[16];
    uint8_t mode;
    uint64_t flags;
};
