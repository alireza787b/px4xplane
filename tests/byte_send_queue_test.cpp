#include "ByteSendQueue.h"

#include <array>
#include <cassert>
#include <cstdint>

int main()
{
    ByteSendQueue queue(8);
    const std::array<uint8_t, 3> first{{1, 2, 3}};
    const std::array<uint8_t, 3> second{{4, 5, 6}};

    const uint64_t firstToken = queue.enqueue(first.data(), first.size());
    assert(firstToken == 3);
    assert(queue.pendingSize() == 3);
    assert(!queue.isComplete(firstToken));

    queue.consume(2);
    assert(queue.pendingSize() == 1);
    assert(queue.data()[0] == 3);
    assert(!queue.isComplete(firstToken));

    const uint64_t secondToken = queue.enqueue(second.data(), second.size());
    assert(secondToken == 6);
    assert(queue.pendingSize() == 4);
    assert(queue.data()[0] == 3);
    assert(queue.data()[1] == 4);

    queue.consume(1);
    assert(queue.isComplete(firstToken));
    assert(!queue.isComplete(secondToken));
    queue.consume(3);
    assert(queue.isComplete(secondToken));
    assert(queue.pendingSize() == 0);

    const std::array<uint8_t, 8> full{{0, 1, 2, 3, 4, 5, 6, 7}};
    assert(queue.enqueue(full.data(), full.size()) == 14);
    assert(queue.enqueue(first.data(), first.size()) == 0);

    queue.clear();
    assert(queue.pendingSize() == 0);
    assert(!queue.isComplete(1));
    assert(queue.enqueue(first.data(), first.size()) == 3);

    return 0;
}
