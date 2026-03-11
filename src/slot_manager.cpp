// slot_manager.cpp - HWC buffer slot management
#include "slot_manager.h"
#include <cstdio>

namespace createdisp {

SlotManager::SlotManager() {
    slot_free_.fill(1);
}

uint32_t SlotManager::assign(int buffer_id) {
    // Check if already assigned
    auto it = bufid_to_slot_.find(buffer_id);
    if (it != bufid_to_slot_.end()) {
        slot_lastused_[it->second] = ++use_counter_;
        return it->second;
    }

    // Find free slot
    for (uint32_t i = 0; i < kSlotCapacity; i++) {
        if (slot_free_[i]) {
            slot_free_[i] = 0;
            bufid_to_slot_[buffer_id] = i;
            slot_to_bufid_[i] = buffer_id;
            slot_lastused_[i] = ++use_counter_;
            return i;
        }
    }

    // No free slots - evict LRU
    uint32_t lru_slot = UINT32_MAX;
    uint64_t lru_time = UINT64_MAX;

    for (auto& kv : slot_lastused_) {
        if (kv.second < lru_time) {
            lru_time = kv.second;
            lru_slot = kv.first;
        }
    }

    if (lru_slot == UINT32_MAX) {
        fprintf(stderr, "SlotManager: exhausted all %u slots\n", kSlotCapacity);
        return UINT32_MAX;
    }

    // Evict and reassign
    int evicted = slot_to_bufid_[lru_slot];
    bufid_to_slot_.erase(evicted);

    bufid_to_slot_[buffer_id] = lru_slot;
    slot_to_bufid_[lru_slot] = buffer_id;
    slot_lastused_[lru_slot] = ++use_counter_;

    fprintf(stderr, "SlotManager: evicted bufid %d from slot %u for bufid %d\n",
            evicted, lru_slot, buffer_id);

    return lru_slot;
}

void SlotManager::release(int buffer_id) {
    auto it = bufid_to_slot_.find(buffer_id);
    if (it == bufid_to_slot_.end()) return;

    uint32_t slot = it->second;
    slot_free_[slot] = 1;
    slot_to_bufid_.erase(slot);
    slot_lastused_.erase(slot);
    bufid_to_slot_.erase(it);
}

void SlotManager::reset() {
    bufid_to_slot_.clear();
    slot_to_bufid_.clear();
    slot_lastused_.clear();
    slot_free_.fill(1);
    use_counter_ = 0;
}

} // namespace createdisp
