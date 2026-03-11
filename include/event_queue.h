// event_queue.h - EVDI event queue management
#pragma once

#include <cstdint>
#include <array>
#include "types.h"
#include "spsc_ring_buffer.h"
#include "constants.h"

namespace createdisp {

struct QueuedEvent {
    EventType event;
    int poll_id;
    std::array<uint8_t, 32> data;
};

extern SpscRingBuffer<QueuedEvent, kEventQueueCapacity> g_event_queue;

} // namespace createdisp
