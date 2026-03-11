// display.h - Display state management
#pragma once

#include <cstdint>
#include "types.h"
#include "slot_manager.h"

namespace createdisp {

// Per-display state
struct Display {
    int display_id = -1;
    HwcDisplayId hwc_id = 0;

    // Current mode
    int width = 0;
    int height = 0;
    uint32_t stride = 0;

    // Connection state
    bool connected = false;
    void* hwcDisplay = nullptr;

    // Buffer slot management
    SlotManager slot_mgr;

    // Generation counter incremented on mode change
    uint64_t generation = 1;

    Display() = default;
    Display(const Display&) = default;
    Display& operator=(const Display&) = default;
};

DisplaySnapshot snapshot_display(const Display& d);

} // namespace createdisp
