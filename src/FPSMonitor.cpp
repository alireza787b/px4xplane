#include "FPSMonitor.h"
#include <algorithm>
#include <cmath>

FPSMonitor::FPSMonitor(size_t window_size)
    : window_size_(window_size),
      index_(0),
      frame_count_(0),
      last_frame_time_(0) {
    frame_times_.reserve(window_size);
}

void FPSMonitor::recordFrame(uint64_t timestamp_usec) {
    if (frame_count_ > 0) {
        // Calculate time delta from last frame
        uint64_t delta = timestamp_usec - last_frame_time_;

        // Add to circular buffer
        if (frame_times_.size() < window_size_) {
            // Still filling buffer
            frame_times_.push_back(delta);
        } else {
            // Buffer full, overwrite oldest
            frame_times_[index_] = delta;
            index_ = (index_ + 1) % window_size_;
        }
    }

    last_frame_time_ = timestamp_usec;
    frame_count_++;
}

float FPSMonitor::getAverageFPS() const {
    if (frame_times_.empty()) {
        return 0.0f;
    }

    // Calculate mean frame time
    uint64_t sum_delta = 0;
    for (uint64_t delta : frame_times_) {
        sum_delta += delta;
    }

    uint64_t avg_delta = sum_delta / frame_times_.size();

    return deltaToFPS(avg_delta);
}

float FPSMonitor::getMinimumFPS() const {
    if (frame_times_.empty()) {
        return 0.0f;
    }

    // Find maximum delta (= minimum FPS)
    uint64_t max_delta = *std::max_element(frame_times_.begin(), frame_times_.end());

    return deltaToFPS(max_delta);
}

float FPSMonitor::getMaximumFPS() const {
    if (frame_times_.empty()) {
        return 0.0f;
    }

    // Find minimum delta (= maximum FPS)
    uint64_t min_delta = *std::min_element(frame_times_.begin(), frame_times_.end());

    return deltaToFPS(min_delta);
}

bool FPSMonitor::isFPSTooLow(float target_sensor_rate_hz) const {
    float avg_fps = getAverageFPS();

    if (avg_fps == 0.0f) {
        return true;  // No data yet, assume too low
    }

    // FPS should be at least 50% of target rate for 2:1 interpolation
    // For better quality, recommend 75% (1.3:1 interpolation)
    float minimum_recommended_fps = target_sensor_rate_hz * 0.5f;

    return avg_fps < minimum_recommended_fps;
}

float FPSMonitor::suggestOptimalSensorRate() const {
    float avg_fps = getAverageFPS();

    if (avg_fps == 0.0f) {
        return 100.0f;  // Default suggestion if no data
    }

    // Conservative recommendation: sensor rate = FPS * 1.5
    // This allows for some FPS variation without excessive interpolation
    float suggested_rate = avg_fps * 1.5f;

    // Round to common sensor rates
    if (suggested_rate >= 180.0f) {
        return 200.0f;  // High performance
    } else if (suggested_rate >= 140.0f) {
        return 150.0f;  // Medium-high
    } else if (suggested_rate >= 90.0f) {
        return 100.0f;  // Medium (most common)
    } else if (suggested_rate >= 65.0f) {
        return 75.0f;   // Medium-low
    } else {
        return 50.0f;   // Low performance mode
    }
}

void FPSMonitor::reset() {
    frame_times_.clear();
    index_ = 0;
    frame_count_ = 0;
    last_frame_time_ = 0;
}

float FPSMonitor::deltaToFPS(uint64_t delta_usec) const {
    if (delta_usec == 0) {
        return 0.0f;
    }

    // FPS = 1 / (delta_seconds) = 1,000,000 / delta_microseconds
    return 1000000.0f / static_cast<float>(delta_usec);
}
