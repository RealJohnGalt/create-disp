// sync_primitives.cpp - Synchronization primitives
#include "sync_primitives.h"
#include "display.h"

namespace createdisp {

// Global state mutex
std::mutex g_state_mutex;

// Global running flag
std::atomic<bool> g_running{true};

// Display state map (protected by g_state_mutex)
std::unordered_map<int, Display> g_displays;

// Display resync flags
std::array<std::atomic<bool>, kMaxDriverDisplays> g_resync_pending{};

// Modeset in-flight counter
std::atomic<int> g_modeset_inflight{0};

// Update thread synchronization
std::mutex g_update_mutex;
std::condition_variable g_update_cv;
std::deque<int> g_update_queue;
bool g_pending_update[kMaxDriverDisplays] = {};
bool g_pending_disconnect[kMaxDriverDisplays] = {};
bool g_enqueued[kMaxDriverDisplays] = {};

// Present thread synchronization
std::mutex g_present_mutex;
std::condition_variable g_present_cv;
std::deque<PresentJob> g_present_queue;

// Event thread synchronization
std::mutex g_event_mutex;
std::condition_variable g_event_cv;
std::atomic<bool> g_event_thread_sleeping{false};

// Clear pending work for a display (caller must hold g_update_mutex)
static void clear_pending_work_locked(DisplayId display_id) {
    if (display_id < 0 || display_id >= kMaxDriverDisplays) return;

    g_pending_update[display_id] = false;
    g_pending_disconnect[display_id] = false;
    g_enqueued[display_id] = false;

    for (auto it = g_update_queue.begin(); it != g_update_queue.end(); ) {
        if (*it == display_id) {
            it = g_update_queue.erase(it);
        } else {
            ++it;
        }
    }
}

void schedule_update(DisplayId display_id) {
    {
        std::lock_guard<std::mutex> lk(g_update_mutex);
        if (display_id >= 0 && display_id < kMaxDriverDisplays) {
            g_pending_update[display_id] = true;
            if (!g_enqueued[display_id]) {
                g_update_queue.push_back(display_id);
                g_enqueued[display_id] = true;
            }
        }
    }
    g_update_cv.notify_one();
}

void schedule_disconnect(DisplayId display_id) {
    {
        std::lock_guard<std::mutex> lk(g_update_mutex);
        if (display_id >= 0 && display_id < kMaxDriverDisplays) {
            g_pending_disconnect[display_id] = true;
            g_pending_update[display_id] = false;
            if (!g_enqueued[display_id]) {
                g_update_queue.push_back(display_id);
                g_enqueued[display_id] = true;
            }
        }
    }
    g_update_cv.notify_one();
}

void request_display_resync(DisplayId display_id) {
    if (display_id < 0 || display_id >= kMaxDriverDisplays) return;

    bool expected = false;
    if (!g_resync_pending[display_id].compare_exchange_strong(
            expected, true, std::memory_order_acq_rel)) {
        return;
    }

    schedule_update(display_id);
}

void enqueue_present_job(PresentJob&& job) {
    {
        std::lock_guard<std::mutex> lk(g_present_mutex);

        // Remove old jobs for same display
        for (auto it = g_present_queue.begin(); it != g_present_queue.end(); ) {
            if (it->drv_display_id == job.drv_display_id) {
                it = g_present_queue.erase(it);
            } else {
                ++it;
            }
        }

        g_present_queue.push_back(std::move(job));
    }
    g_present_cv.notify_one();
}

void flush_present_jobs_for_display(DisplayId display_id) {
    std::lock_guard<std::mutex> lk(g_present_mutex);

    for (auto it = g_present_queue.begin(); it != g_present_queue.end(); ) {
        if (it->drv_display_id == display_id) {
            it = g_present_queue.erase(it);
        } else {
            ++it;
        }
    }
}

} // namespace createdisp
