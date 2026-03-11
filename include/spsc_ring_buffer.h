// spsc_ring_buffer.h - SPSC ring buffer
#pragma once

#include <atomic>
#include <cstddef>

namespace createdisp {

// Producer writes to head, consumer reads from tail
template <typename T, size_t Capacity>
class SpscRingBuffer {
    static_assert((Capacity != 0) && ((Capacity & (Capacity - 1)) == 0),
                  "Capacity must be power of 2");

private:
    struct alignas(64) {
        std::atomic<size_t> head{0};
        size_t cached_tail{0};
    } producer_;

    // Consumer state
    struct alignas(64) {
        std::atomic<size_t> tail{0};
        size_t cached_head{0};
    } consumer_;

    T buffer_[Capacity];

public:
    // Producer enqueue
    bool push(const T& item) {
        size_t h = producer_.head.load(std::memory_order_relaxed);
        size_t next = (h + 1) & (Capacity - 1);

        if (next == producer_.cached_tail) {
            producer_.cached_tail = consumer_.tail.load(std::memory_order_acquire);
            if (next == producer_.cached_tail) {
                return false;
            }
        }

        buffer_[h] = item;
        producer_.head.store(next, std::memory_order_release);
        return true;
    }

    // Consumer dequeue
    bool pop(T& item) {
        size_t t = consumer_.tail.load(std::memory_order_relaxed);

        if (t == consumer_.cached_head) {
            consumer_.cached_head = producer_.head.load(std::memory_order_acquire);
            if (t == consumer_.cached_head) {
                return false;
            }
        }

        item = buffer_[t];
        consumer_.tail.store((t + 1) & (Capacity - 1), std::memory_order_release);
        return true;
    }

    bool empty() const {
        return producer_.head.load(std::memory_order_acquire) ==
               consumer_.tail.load(std::memory_order_relaxed);
    }
};

} // namespace createdisp
