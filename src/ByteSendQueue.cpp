#include "ByteSendQueue.h"

#include <algorithm>

ByteSendQueue::ByteSendQueue(size_t capacityBytes) : _capacityBytes(capacityBytes)
{
}

uint64_t ByteSendQueue::enqueue(const uint8_t* data, size_t size)
{
    if (data == nullptr || size == 0 || size > _capacityBytes - pendingSize()) {
        return 0;
    }

    if (_offset > 0 &&
        (_offset >= _buffer.size() / 2 || _buffer.size() + size > _capacityBytes)) {
        compact();
    }

    _buffer.insert(_buffer.end(), data, data + size);
    _totalQueued += size;
    return _totalQueued;
}

const uint8_t* ByteSendQueue::data() const
{
    return pendingSize() > 0 ? _buffer.data() + _offset : nullptr;
}

size_t ByteSendQueue::pendingSize() const
{
    return _buffer.size() - _offset;
}

void ByteSendQueue::consume(size_t size)
{
    const size_t consumed = std::min(size, pendingSize());
    _offset += consumed;
    _totalSent += consumed;

    if (_offset == _buffer.size()) {
        _buffer.clear();
        _offset = 0;
    }
}

bool ByteSendQueue::isComplete(uint64_t completionToken) const
{
    return completionToken > 0 && completionToken <= _totalSent;
}

void ByteSendQueue::clear()
{
    _buffer.clear();
    _offset = 0;
    _totalQueued = 0;
    _totalSent = 0;
}

void ByteSendQueue::compact()
{
    _buffer.erase(_buffer.begin(), _buffer.begin() + _offset);
    _offset = 0;
}
