#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

class ByteSendQueue {
public:
    explicit ByteSendQueue(size_t capacityBytes);

    uint64_t enqueue(const uint8_t* data, size_t size);
    const uint8_t* data() const;
    size_t pendingSize() const;
    void consume(size_t size);
    bool isComplete(uint64_t completionToken) const;
    void clear();

private:
    void compact();

    const size_t _capacityBytes;
    std::vector<uint8_t> _buffer;
    size_t _offset{0};
    uint64_t _totalQueued{0};
    uint64_t _totalSent{0};
};
