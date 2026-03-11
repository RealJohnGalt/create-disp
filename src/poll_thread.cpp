// poll_thread.cpp - EVDI polling thread
#include "poll_thread.h"
#include "evdi_protocol.h"
#include "evdi_operations.h"
#include "drm_device.h"
#include "event_queue.h"
#include "sync_primitives.h"
#include "logging.h"
#include <cstring>

namespace createdisp {

void poll_thread_main() {
    log_info("Poll thread started");

    int hard_poll_failures = 0;
    constexpr int kMaxHardFailures = 10;

    while (g_running.load(std::memory_order_acquire)) {
        evdi::drm_evdi_poll poll_cmd = {};
        uint8_t poll_payload[32] = {};
        poll_cmd.data = poll_payload;

        int ret = evdi_ioctl(DRM_IOCTL_EVDI_POLL, &poll_cmd);

        if (ret < 0) {
            int err = errno;

            if (err == EINTR || err == EAGAIN) {
                continue;
            }

            if (should_request_reopen(err)) {
                log_warning("Poll ioctl failed with %d, requesting reopen", err);
                g_drm_device.request_reopen();
                break;
            }

            hard_poll_failures++;
            if (hard_poll_failures >= kMaxHardFailures) {
                log_error("Too many consecutive poll failures (%d), exiting poll thread",
                         hard_poll_failures);
                g_running.store(false, std::memory_order_release);
                break;
            }

            continue;
        }

        hard_poll_failures = 0;

        // Queue event if valid
        EventType evt = static_cast<EventType>(poll_cmd.event);
        if (evt != EventType::None) {
            QueuedEvent q_ev;
            q_ev.event = evt;
            q_ev.poll_id = poll_cmd.poll_id;
            std::memcpy(q_ev.data.data(), poll_payload, sizeof(poll_payload));

            if (!g_event_queue.push(q_ev)) {
                log_error("EVDI event queue is full! Dropping event.");
            }

            // Wake event thread if sleeping
            if (g_event_thread_sleeping.load(std::memory_order_acquire)) {
                g_event_cv.notify_one();
            }
        }
    }

    log_info("Poll thread exiting");
}

} // namespace createdisp
