// event_thread.cpp - Event processing thread
#include "event_thread.h"
#include "event_queue.h"
#include "event_handlers.h"
#include "sync_primitives.h"
#include "logging.h"

namespace createdisp {

void event_thread_main() {
    log_info("Event thread started");

    while (g_running.load(std::memory_order_acquire)) {
        QueuedEvent ev;

        if (g_event_queue.pop(ev)) {
            // Process event
            switch (ev.event) {
            case EventType::GetBuf:
                handle_get_buf(ev.data.data(), ev.poll_id);
                break;
            case EventType::SwapTo:
                handle_swap_to(ev.data.data(), ev.poll_id);
                break;
            case EventType::DestroyBuf:
                handle_destroy_buf(ev.data.data(), ev.poll_id);
                break;
            case EventType::CreateBuf:
                handle_create_buf(ev.data.data(), ev.poll_id);
                break;
            default:
                break;
            }
        } else {
            // Queue empty - wait
            std::unique_lock<std::mutex> lk(g_event_mutex);

            // Check again after acquiring lock (prevent lost wakeup)
            if (!g_event_queue.empty()) {
                continue;
            }

            g_event_thread_sleeping.store(true, std::memory_order_release);
            g_event_cv.wait(lk, [] {
                return !g_running.load(std::memory_order_acquire) ||
                       !g_event_queue.empty();
            });
            g_event_thread_sleeping.store(false, std::memory_order_release);
        }
    }

    log_info("Event thread exiting");
}

} // namespace createdisp
