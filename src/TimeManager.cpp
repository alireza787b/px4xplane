#include "TimeManager.h"

uint64_t TimeManager::getCurrentTimeUsec() {
    auto now = HighResClock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}

double TimeManager::getCurrentTimeSec() { // <-- Implement this function
    auto now = HighResClock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();
}

uint64_t TimeManager::calculateElapsedTimeUsec(uint64_t& lastTime) {
    uint64_t currentTime = getCurrentTimeUsec();
    uint64_t elapsedTime = currentTime - lastTime;
    lastTime = currentTime;
    return elapsedTime;
}
