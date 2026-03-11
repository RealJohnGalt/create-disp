// present_thread.cpp - HWC presentation thread
#include "present_thread.h"
#include "sync_primitives.h"
#include "logging.h"
#include "hwc_manager.h"
#include "display.h"

namespace createdisp {

extern std::unordered_map<int, Display> g_displays;

void present_thread_main() {
    log_info("Present thread started");

    while (g_running.load(std::memory_order_acquire)) {
        PresentJob job;

        {
            std::unique_lock<std::mutex> lk(g_present_mutex);
            g_present_cv.wait(lk, [] {
                return !g_running.load(std::memory_order_acquire) ||
                       !g_present_queue.empty();
            });

            if (!g_running.load(std::memory_order_acquire)) {
                break;
            }

            if (g_present_queue.empty()) {
                continue;
            }

            job = std::move(g_present_queue.front());
            g_present_queue.pop_front();
        }

        if (job.drv_display_id < 0 || job.drv_display_id >= kMaxDriverDisplays || !job.rwb) {
            continue;
        }

        hwc2_compat_display_t* hwcDisp = nullptr;

        {
            std::unique_lock<std::mutex> state_lk(g_state_mutex, std::defer_lock);
            std::unique_lock<std::mutex> hwc_lk(g_hwc_manager.get_hwc_mutex(job.drv_display_id), std::defer_lock);
            std::lock(state_lk, hwc_lk);

            Display& D = g_displays[job.drv_display_id];
            hwcDisp = static_cast<hwc2_compat_display_t*>(D.hwcDisplay);

            if (!D.connected || !hwcDisp || D.generation != job.generation) {
                continue;
            }

            state_lk.unlock();

            uint32_t numTypes = 0, numRequests = 0;
            hwc2_error_t err = HWC2_ERROR_NONE;
            native_handle_t* nh = job.rwb ? job.rwb->handle : nullptr;
            ANativeWindowBuffer* anb = reinterpret_cast<ANativeWindowBuffer*>(nh);
            err = hwc2_compat_display_set_client_target(hwcDisp, job.slot, anb,
                                                        -1, HAL_DATASPACE_UNKNOWN);
            if (err != HWC2_ERROR_NONE) {
                log_error("set_client_target failed: %d", static_cast<int>(err));
                request_display_resync(job.drv_display_id);
                continue;
            }

            err = hwc2_compat_display_validate(hwcDisp, &numTypes, &numRequests);
            if (err == HWC2_ERROR_HAS_CHANGES && (numTypes || numRequests)) {
                hwc2_compat_display_accept_changes(hwcDisp);
            } else if (err != HWC2_ERROR_NONE) {
                log_error("validate failed: %d", static_cast<int>(err));
                request_display_resync(job.drv_display_id);
                continue;
            }

            int presentFence = -1;
            err = hwc2_compat_display_present(hwcDisp, &presentFence);
            if (err != HWC2_ERROR_NONE) {
                log_error("present failed: %d", static_cast<int>(err));
                request_display_resync(job.drv_display_id);
                continue;
            }
        }

        {
            std::lock_guard<std::mutex> state_lk(g_state_mutex);
            Display& D = g_displays[job.drv_display_id];
            if (D.connected && D.hwcDisplay && D.generation == job.generation) {
                g_resync_pending[job.drv_display_id].store(false, std::memory_order_release);
            }
        }
    }

    log_info("Present thread exiting");
}

} // namespace createdisp
