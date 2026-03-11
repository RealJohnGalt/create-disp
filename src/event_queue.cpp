// event_queue.cpp - EVDI event queue
#include "event_queue.h"

namespace createdisp {

SpscRingBuffer<QueuedEvent, kEventQueueCapacity> g_event_queue;

} // namespace createdisp
