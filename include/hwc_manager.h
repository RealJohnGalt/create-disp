// hwc_manager.h - HWC device and display management
#pragma once

#include <unordered_map>
#include <mutex>
#include <array>
#include <hybris/hwc2/hwc2_compatibility_layer.h>
#include "types.h"
#include "constants.h"

namespace createdisp {

class HwcManager {
public:
    HwcManager();
    ~HwcManager();

    bool initialize();
    void shutdown();
    hwc2_compat_device_t* get_device() const { return hwc_device_; }
    int get_driver_id_for_hwc(HwcDisplayId hwc_id);
    int alloc_driver_slot_for_hwc(HwcDisplayId hwc_id);
    void release_driver_slot_for_hwc(HwcDisplayId hwc_id);
    std::mutex& get_hwc_mutex(DisplayId display_id);

private:
    void init_free_slots();
    hwc2_compat_device_t* hwc_device_ = nullptr;
    std::unordered_map<HwcDisplayId, int> hwc_to_drv_;
    std::unordered_map<int, HwcDisplayId> drv_to_hwc_;
    std::array<int, kMaxDriverDisplays> free_drv_ids_;
    int free_drv_count_ = 0;
    bool free_drv_initialized_ = false;
    std::array<std::mutex, kMaxDriverDisplays> hwc_mutexes_;
};

extern HwcManager g_hwc_manager;

} // namespace createdisp
