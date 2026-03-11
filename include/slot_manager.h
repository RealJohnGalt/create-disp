// slot_manager.h - HWC buffer slot management
#pragma once

#include <unordered_map>
#include <array>
#include <cstdint>
#include "constants.h"

namespace createdisp {

// Manages mapping between buffer IDs and HWC slot indices
class SlotManager {
public:
    SlotManager();
    uint32_t assign(int buffer_id);
    void release(int buffer_id);
    void reset();

private:
    std::unordered_map<int, uint32_t> bufid_to_slot_;
    std::unordered_map<uint32_t, int> slot_to_bufid_;
    std::unordered_map<uint32_t, uint64_t> slot_lastused_;
    std::array<uint8_t, kSlotCapacity> slot_free_;
    uint64_t use_counter_ = 0;
};

} // namespace createdisp
