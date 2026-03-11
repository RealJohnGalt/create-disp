// types.h - Core type definitions
#pragma once

#include <cstdint>
#include <memory>
#include <atomic>
#include <hybris/platforms/common/windowbuffer.h>

namespace createdisp {

// Display and buffer IDs
using DisplayId = int;
using BufferId  = int;
using HwcDisplayId = uint64_t;

// Forward declarations
class RemoteWindowBuffer;
struct BufferEntry;
struct Display;

class RemoteWindowBuffer : public ANativeWindowBuffer {
public:
    RemoteWindowBuffer() = default;
    ~RemoteWindowBuffer() = default;

    RemoteWindowBuffer(int w, int h, uint32_t s, uint32_t f, uint32_t u, buffer_handle_t nh)
        : width(w), height(h), stride(s), format(f), usage(u), handle(const_cast<native_handle_t*>(nh)) {}

    int width = 0;
    int height = 0;
    uint32_t stride = 0;
    uint32_t format = 0;
    uint32_t usage = 0;
    native_handle_t* handle = nullptr;
};

// Type aliases
using SharedRwb     = std::shared_ptr<RemoteWindowBuffer>;
using BufferEntryPtr = std::shared_ptr<BufferEntry>;

struct PresentJob {
    DisplayId drv_display_id{-1};
    BufferId  buffer_id{-1};
    uint32_t  slot{0};
    uint64_t  generation{0};
    SharedRwb rwb;
};

// Buffer origin tracking
enum class BufferOrigin : uint8_t {
    Imported = 0,
    Allocated = 1
};

// EVDI event types
enum class EventType : uint8_t {
    None = 0,
    AddBuf,
    GetBuf,
    DestroyBuf,
    SwapTo,
    CreateBuf
};

// Display snapshot for lock-free reads
struct DisplaySnapshot {
    void* hwcDisplay = nullptr;
    int width = 0;
    int height = 0;
    uint32_t stride = 0;
    bool connected = false;
    uint64_t generation = 0;
};

} // namespace createdisp
