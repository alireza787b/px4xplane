#include "TimestampProvider.h"
#include "ConfigManager.h"
#include "DataRefManager.h"

#include <cassert>
#include <chrono>
#include <cstdint>
#include <thread>

namespace {
float g_xplaneTimeSec = 0.0f;
}

bool ConfigManager::debug_log_sensor_timing = false;

float DataRefManager::getFloat(const char* dataRef)
{
    (void)dataRef;
    return g_xplaneTimeSec;
}

extern "C" void XPLMDebugString(const char* inString)
{
    (void)inString;
}

int main()
{
    TimestampProvider::reset();

    g_xplaneTimeSec = 100.0f;
    const uint64_t first = TimestampProvider::getTimestampUsec();
    assert(first == 1000000ULL);

    g_xplaneTimeSec = 100.02f;
    const uint64_t second = TimestampProvider::getTimestampUsec();
    assert(second > first);

    std::this_thread::sleep_for(std::chrono::milliseconds(1100));
    const uint64_t sameFrame = TimestampProvider::getTimestampUsec();
    assert(sameFrame == second + 1ULL);

    g_xplaneTimeSec = 101.02f;
    const uint64_t capped = TimestampProvider::getTimestampUsec();
    assert(capped > sameFrame);
    assert((capped - sameFrame) <= 100001ULL);

    const auto diagnostics = TimestampProvider::getDiagnostics();
    assert(diagnostics.sub_frame_fallbacks >= 1);
    assert(diagnostics.capped_deltas >= 1);

    return 0;
}
