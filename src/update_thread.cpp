// update_thread.cpp - Display update thread
#include "update_thread.h"
#include "evdi_operations.h"
#include "drm_device.h"
#include "sync_primitives.h"
#include "hwc_manager.h"
#include "gralloc_wrapper.h"
#include "buffer_manager.h"
#include "logging.h"
#include "display.h"
#include <sstream>
#include <cmath>

namespace createdisp {

extern std::unordered_map<int, Display> g_displays;

static int hz_from_period_ns(int32_t ns) {
    if (ns <= 0) return kDefaultRefreshHz;
    double hz_f = 1e9 / static_cast<double>(ns);
    return static_cast<int>(std::lround(hz_f));
}

static int get_refresh_hz_from_config(const HWC2DisplayConfig* cfg) {
    return hz_from_period_ns(cfg->vsyncPeriod);
}

static int reconnect_display_mode(DisplayId display_id, int width, int height,
                                   int refresh_hz, bool disconnect_first) {
    g_modeset_inflight.fetch_add(1, std::memory_order_acq_rel);

    int rc = 0;

    if (disconnect_first) {
        if (evdi_connect(0, 0, 0, 0, display_id, false) < 0) {
            rc = -1;
        }
    }

    if (rc == 0) {
        rc = evdi_connect(0, width, height, refresh_hz, display_id, true);
    }

    g_modeset_inflight.fetch_sub(1, std::memory_order_acq_rel);
    return rc;
}

int update_display(DisplayId display_id) {
    if (display_id < 0 || display_id >= kMaxDriverDisplays) {
        return -1;
    }

    if (!g_drm_device.is_ready()) {
        log_warning("update_display(%d): DRM not ready, deferring", display_id);
        return -1;
    }

    if (g_modeset_inflight.load(std::memory_order_acquire) > 0) {
        log_warning("update_display(%d): modeset in-flight, deferring", display_id);
        return -1;
    }

    flush_present_jobs_for_display(display_id);

    int target_width = 0, target_height = 0, refresh_hz = kDefaultRefreshHz;
    uint64_t generation = 0;
    uint32_t new_stride = 0;
    bool force_reconnect = g_resync_pending[display_id].exchange(false, std::memory_order_acq_rel);
    bool had_previous_mode = false;
    bool mode_changed = false;

    {
        std::unique_lock<std::mutex> state_lk(g_state_mutex, std::defer_lock);
        std::unique_lock<std::mutex> hwc_lk(g_hwc_manager.get_hwc_mutex(display_id), std::defer_lock);
        std::lock(state_lk, hwc_lk);

        Display& D = g_displays[display_id];
        hwc2_compat_display_t* hwcDisp = static_cast<hwc2_compat_display_t*>(D.hwcDisplay);

        if (!D.connected || !hwcDisp) {
            return -1;
        }

        HWC2DisplayConfig* config = hwc2_compat_display_get_active_config(hwcDisp);
        if (!config) {
            log_warning("update_display(%d): no active HWC config yet", display_id);
            return -1;
        }

        if (config->width <= 0 || config->height <= 0) {
            log_warning("update_display(%d): invalid geometry %dx%d",
                       display_id, config->width, config->height);
            return -1;
        }

        target_width = config->width;
        target_height = config->height;
        refresh_hz = get_refresh_hz_from_config(config);
        had_previous_mode = (D.width > 0 && D.height > 0 && D.stride != 0);
        mode_changed = (D.width != target_width || D.height != target_height);

        if (!force_reconnect && !mode_changed && D.stride != 0) {
            return 0;
        }

        log_info("display %d: %dx%d @ %dHz%s%s",
                display_id, target_width, target_height, refresh_hz,
                mode_changed ? " (mode change)" : "",
                force_reconnect ? " (forced resync)" : "");

        g_buffer_manager.reset_display_bindings_locked(display_id);

        D.generation++;
        generation = D.generation;
        D.width = target_width;
        D.height = target_height;
        D.stride = 0;
    }

    // Determine stride via temporary allocation
    buffer_handle_t handle = nullptr;
    int r = gralloc_allocate(target_width, target_height,
                            HAL_PIXEL_FORMAT_RGBX_8888,
                            kRwbUsage, &handle, &new_stride);

    if (r == 0 && handle) {
        gralloc_release(handle);
    } else {
        log_error("update_display(%d): failed to determine stride for %dx%d",
                 display_id, target_width, target_height);
        return -1;
    }

    if (reconnect_display_mode(display_id, target_width, target_height,
                               refresh_hz, force_reconnect || mode_changed || had_previous_mode) < 0) {
        return -1;
    }

    {
        std::lock_guard<std::mutex> lk(g_state_mutex);
        Display& D = g_displays[display_id];
        if (D.generation == generation) {
            D.width = target_width;
            D.height = target_height;
            D.stride = new_stride;
        }
    }

    return 0;
}

void disconnect_display(DisplayId display_id) {
    if (display_id < 0 || display_id >= kMaxDriverDisplays) {
        return;
    }

    flush_present_jobs_for_display(display_id);

    if (g_drm_device.is_ready()) {
        g_modeset_inflight.fetch_add(1, std::memory_order_acq_rel);
        evdi_connect(0, 0, 0, 0, display_id, false);
        g_modeset_inflight.fetch_sub(1, std::memory_order_acq_rel);
    }

    {
        std::unique_lock<std::mutex> state_lk(g_state_mutex, std::defer_lock);
        std::unique_lock<std::mutex> hwc_lk(g_hwc_manager.get_hwc_mutex(display_id), std::defer_lock);
        std::lock(state_lk, hwc_lk);

        Display& D = g_displays[display_id];
        g_buffer_manager.reset_display_bindings_locked(display_id);
        D.width = 0;
        D.height = 0;
        D.stride = 0;
    }
}

void update_thread_main() {
    log_info("Update thread started");

    while (g_running.load(std::memory_order_acquire)) {
        int display_id = -1;
        bool is_disconnect = false;

        {
            std::unique_lock<std::mutex> lk(g_update_mutex);
            g_update_cv.wait(lk, [] {
                return !g_running.load(std::memory_order_acquire) ||
                       !g_update_queue.empty();
            });

            if (!g_running.load(std::memory_order_acquire)) {
                break;
            }

            if (g_update_queue.empty()) {
                continue;
            }

            display_id = g_update_queue.front();
            g_update_queue.pop_front();

            if (display_id < 0 || display_id >= kMaxDriverDisplays) {
                continue;
            }

            is_disconnect = g_pending_disconnect[display_id];
            g_pending_update[display_id] = false;
            g_pending_disconnect[display_id] = false;
            g_enqueued[display_id] = false;
        }

        if (is_disconnect) {
            disconnect_display(display_id);
        } else {
            update_display(display_id);
        }
    }

    log_info("Update thread exiting");
}

} // namespace createdisp
