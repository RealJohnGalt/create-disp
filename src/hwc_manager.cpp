// hwc_manager.cpp - HWC device management
#include "hwc_manager.h"
#include "logging.h"

namespace createdisp {

HwcManager g_hwc_manager;

HwcManager::HwcManager() = default;

HwcManager::~HwcManager() {
    shutdown();
}

bool HwcManager::initialize() {
    hwc_device_ = hwc2_compat_device_new(false);
    if (!hwc_device_) {
        log_error("Failed to create HWC2 device");
        return false;
    }

    init_free_slots();
    log_info("HWC device initialized");
    return true;
}

void HwcManager::shutdown() {
    hwc_device_ = nullptr;
}

void HwcManager::init_free_slots() {
    if (free_drv_initialized_) return;

    free_drv_count_ = 0;
    for (int i = kMaxDriverDisplays - 1; i >= 0; --i) {
        free_drv_ids_[free_drv_count_++] = i;
    }
    free_drv_initialized_ = true;
}

int HwcManager::get_driver_id_for_hwc(HwcDisplayId hwc_id) {
    auto it = hwc_to_drv_.find(hwc_id);
    return (it == hwc_to_drv_.end()) ? -1 : it->second;
}

int HwcManager::alloc_driver_slot_for_hwc(HwcDisplayId hwc_id) {
    int drv = get_driver_id_for_hwc(hwc_id);
    if (drv >= 0) return drv;

    if (free_drv_count_ <= 0) return -1;

    drv = free_drv_ids_[--free_drv_count_];
    hwc_to_drv_[hwc_id] = drv;
    drv_to_hwc_[drv] = hwc_id;

    return drv;
}

void HwcManager::release_driver_slot_for_hwc(HwcDisplayId hwc_id) {
    auto it = hwc_to_drv_.find(hwc_id);
    if (it == hwc_to_drv_.end()) return;

    int drv = it->second;
    hwc_to_drv_.erase(it);
    drv_to_hwc_.erase(drv);

    // Return to free pool
    for (int i = 0; i < free_drv_count_; ++i) {
        if (free_drv_ids_[i] == drv) return;
    }

    if (free_drv_count_ < kMaxDriverDisplays) {
        free_drv_ids_[free_drv_count_++] = drv;
    } else {
        log_error("release_driver_slot_for_hwc: free list overflow for drv %d", drv);
    }
}

std::mutex& HwcManager::get_hwc_mutex(DisplayId display_id) {
    return hwc_mutexes_[display_id];
}

} // namespace createdisp
