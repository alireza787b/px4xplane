#pragma once
#ifndef TIMEMANAGER_H
#define TIMEMANAGER_H

#include <chrono>

class TimeManager {
public:
    // Get the current time in microseconds since epoch
    static uint64_t getCurrentTimeUsec();

    // Get the current time in seconds since epoch (with fractional seconds)
    static double getCurrentTimeSec(); // <-- Add this line

    // Calculate elapsed time in microseconds since the last call
    static uint64_t calculateElapsedTimeUsec(uint64_t& lastTime);

private:
    // Use a high-resolution clock for time measurements
    using HighResClock = std::chrono::high_resolution_clock;
};

#endif // TIMEMANAGER_H
