#include "SensorBuffer.h"
#include <algorithm>  // for std::min

bool SensorBuffer::push(const IMUSample& sample) {
    // Get current write position (atomic read)
    uint32_t current_write = write_index_.load(std::memory_order_relaxed);

    // Calculate actual buffer index using power-of-2 mask (fast modulo)
    size_t buffer_idx = current_write & INDEX_MASK;

    // Write sample to buffer (no lock needed - single producer)
    buffer_[buffer_idx] = sample;

    // Advance write index (atomic increment)
    // Use release semantics to ensure write completes before index update
    write_index_.store(current_write + 1, std::memory_order_release);

    // Always returns true (overwrites oldest if full)
    return true;
}

std::vector<SensorBuffer::IMUSample> SensorBuffer::getLatest(size_t count) {
    std::vector<IMUSample> result;

    // Get current indices (atomic reads)
    uint32_t current_write = write_index_.load(std::memory_order_acquire);
    uint32_t current_read = read_index_.load(std::memory_order_relaxed);

    // Calculate available samples
    uint32_t available_count = current_write - current_read;

    // Limit to buffer capacity (in case of overflow)
    if (available_count > BUFFER_SIZE) {
        // Buffer wrapped around, we lost some samples
        // Adjust read index to oldest valid sample
        current_read = current_write - BUFFER_SIZE;
        available_count = BUFFER_SIZE;
    }

    // Further limit to requested count
    size_t samples_to_read = std::min<size_t>(count, available_count);

    if (samples_to_read == 0) {
        return result;  // Empty buffer
    }

    // Reserve space for efficiency
    result.reserve(samples_to_read);

    // Read samples in reverse chronological order (newest first)
    // Start from most recent sample and work backwards
    for (size_t i = 0; i < samples_to_read; i++) {
        // Calculate index of sample (newest - i)
        uint32_t sample_index = (current_write - 1 - i) & INDEX_MASK;
        result.push_back(buffer_[sample_index]);
    }

    // Update read index to mark these samples as consumed
    read_index_.store(current_write, std::memory_order_relaxed);

    return result;
}

bool SensorBuffer::getLatestSingle(IMUSample& sample) {
    // Get current write index
    uint32_t current_write = write_index_.load(std::memory_order_acquire);
    uint32_t current_read = read_index_.load(std::memory_order_relaxed);

    // Check if any samples available
    if (current_write == current_read) {
        return false;  // Buffer empty
    }

    // Get most recent sample (write_index - 1)
    uint32_t latest_index = (current_write - 1) & INDEX_MASK;
    sample = buffer_[latest_index];

    // Update read index
    read_index_.store(current_write, std::memory_order_relaxed);

    return true;
}

size_t SensorBuffer::available() const {
    // Get current indices (atomic reads)
    uint32_t current_write = write_index_.load(std::memory_order_acquire);
    uint32_t current_read = read_index_.load(std::memory_order_relaxed);

    // Calculate available samples
    uint32_t count = current_write - current_read;

    // Limit to buffer capacity
    return std::min<size_t>(count, BUFFER_SIZE);
}

void SensorBuffer::clear() {
    // Reset indices (not thread-safe - use only during init/shutdown)
    write_index_.store(0, std::memory_order_relaxed);
    read_index_.store(0, std::memory_order_relaxed);
}
