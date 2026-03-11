// hwc_callbacks.cpp - HWC event callbacks
#include "hwc_callbacks.h"
#include "hwc_manager.h"
#include "evdi_operations.h"
#include "sync_primitives.h"
#include "logging.h"
#include "display.h"
#include <unordered_map>
#include <inttypes.h>

namespace createdisp {

// External display map (sync_primitives.cpp)
extern std::unordered_map<int, Display> g_displays;

void onVsyncReceived(HWC2EventListener* listener, int32_t sequenceId,
                     hwc2_display_t display, int64_t timestamp) {
    const HwcDisplayId hwc_id = static_cast<HwcDisplayId>(display);

    int drv_id;
    {
        std::lock_guard<std::mutex> lk(g_state_mutex);
        drv_id = g_hwc_manager.get_driver_id_for_hwc(hwc_id);
    }

    if (drv_id >= 0) {
        int ret = evdi_vsync(drv_id);
        if (ret < 0 && errno != ETIMEDOUT && errno != ENODEV && errno != EBADF) {
            log_error("vsync failed for display %d: %d (%s)",
                     drv_id, errno, strerror(errno));
        }
    }
}

void onHotplugReceived(HWC2EventListener* listener, int32_t sequenceId,
                       hwc2_display_t display, bool connected, bool primaryDisplay) {
    log_info("onHotplugReceived(%d, %" PRIu64 ", %s, %s)",
             sequenceId, static_cast<uint64_t>(display),
             connected ? "connected" : "disconnected",
             primaryDisplay ? "primary" : "external");

    hwc2_compat_device_on_hotplug(g_hwc_manager.get_device(), display, connected);

    const HwcDisplayId hwc_id = static_cast<HwcDisplayId>(display);
    int drv_id = -1;

    if (connected) {
        hwc2_compat_display_t* hwc_display =
            hwc2_compat_device_get_display_by_id(g_hwc_manager.get_device(), display);

        {
            std::lock_guard<std::mutex> lk(g_state_mutex);
            drv_id = g_hwc_manager.get_driver_id_for_hwc(hwc_id);
            if (drv_id < 0) {
                drv_id = g_hwc_manager.alloc_driver_slot_for_hwc(hwc_id);
                if (drv_id < 0) {
                    log_error("No free driver display slots; ignoring hotplug for HWC id %" PRId64,
                             hwc_id);
                    return;
                }
            }
        }

        {
            std::unique_lock<std::mutex> state_lk(g_state_mutex, std::defer_lock);
            std::unique_lock<std::mutex> hwc_lk(g_hwc_manager.get_hwc_mutex(drv_id), std::defer_lock);
            std::lock(state_lk, hwc_lk);

            Display& D = g_displays[drv_id];
            D.display_id = drv_id;
            D.hwc_id = hwc_id;
            D.hwcDisplay = hwc_display;
            D.connected = true;
        }

        schedule_update(drv_id);

        if (hwc_display) {
            hwc2_compat_display_set_vsync_enabled(hwc_display, HWC2_VSYNC_ENABLE);
        }
    } else {
        {
            std::lock_guard<std::mutex> lk(g_state_mutex);
            drv_id = g_hwc_manager.get_driver_id_for_hwc(hwc_id);
            if (drv_id < 0) return;
        }

        {
            std::unique_lock<std::mutex> state_lk(g_state_mutex, std::defer_lock);
            std::unique_lock<std::mutex> hwc_lk(g_hwc_manager.get_hwc_mutex(drv_id), std::defer_lock);
            std::lock(state_lk, hwc_lk);

            Display& D = g_displays[drv_id];
            D.connected = false;
        }

        schedule_disconnect(drv_id);
    }
}

void onRefreshReceived(HWC2EventListener* listener, int32_t sequenceId,
                       hwc2_display_t display) {
    const HwcDisplayId hwc_id = static_cast<HwcDisplayId>(display);

    int drv_id;
    {
        std::lock_guard<std::mutex> lk(g_state_mutex);
        drv_id = g_hwc_manager.get_driver_id_for_hwc(hwc_id);
    }

    if (drv_id < 0) return;

    log_info("onRefreshReceived (HWC %" PRIu64 ") -> driver slot %d",
             static_cast<uint64_t>(hwc_id), drv_id);
    schedule_update(drv_id);
}

HWC2EventListener g_event_listener = {
    &onVsyncReceived,
    &onHotplugReceived,
    &onRefreshReceived
};

} // namespace createdisp
