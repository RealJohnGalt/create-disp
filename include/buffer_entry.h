// buffer_entry.h - Buffer lifecycle and state tracking
#pragma once

#include <cstdint>
#include <hybris/gralloc/gralloc.h>
#include "types.h"

namespace createdisp {

// Tracks a buffer state, ownership, and HWC binding
struct BufferEntry {
    BufferOrigin origin = BufferOrigin::Imported;
    native_handle_t* handle = nullptr;

    // RWB cached instance
    SharedRwb rwb;

    // RWB Geometry
    int rwb_w = 0;
    int rwb_h = 0;
    uint32_t rwb_stride = 0;
    int rwb_format = 0;

    // Buffer properties
    int format = 0;
    uint32_t stride = 0;
    int width = 0;
    int height = 0;

    // Display binding tracking
    int display_id = -1;
    uint64_t generation = 0;

    ~BufferEntry();

    BufferEntry() = default;
    BufferEntry(const BufferEntry&) = delete;
    BufferEntry& operator=(const BufferEntry&) = delete;
};

} // namespace createdisp
