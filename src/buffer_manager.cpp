// buffer_manager.cpp - Buffer management
#include "buffer_manager.h"
#include "buffer_entry.h"
#include "display.h"
#include "sync_primitives.h"
#include "logging.h"
#include "gralloc_wrapper.h"
#include <hybris/platforms/common/windowbuffer.h>
#include <vector>

namespace createdisp {

BufferManager g_buffer_manager;

// External display map
extern std::unordered_map<int, Display> g_displays;

BufferManager::BufferManager() = default;

BufferId BufferManager::add_buffer(native_handle_t* handle, BufferOrigin origin,
                                   int format, uint32_t stride,
                                   uint32_t width, uint32_t height) {
    std::lock_guard<std::mutex> lk(g_state_mutex);

    if (next_id_ <= 0) next_id_ = 1;

    BufferId id = next_id_++;
    auto entry = std::make_shared<BufferEntry>();
    entry->origin = origin;
    entry->handle = handle;
    entry->format = format;
    entry->stride = stride;
    entry->width = width;
    entry->height = height;

    buffers_[id] = std::move(entry);
    return id;
}

BufferEntryPtr BufferManager::get_buffer_locked(BufferId id) {
    auto it = buffers_.find(id);
    return (it != buffers_.end()) ? it->second : nullptr;
}

void BufferManager::erase_buffer_locked(BufferId id) {
    auto it = buffers_.find(id);
    if (it != buffers_.end()) {
        buffers_.erase(it);
    }

    // Release from all displays
    for (auto& kv : g_displays) {
        kv.second.slot_mgr.release(id);
    }
}

void BufferManager::reset_buffer_binding_locked(BufferEntryPtr entry) {
    if (!entry) return;

    entry->display_id = -1;
    entry->generation = 0;
}

void BufferManager::reset_display_bindings_locked(DisplayId display_id) {
    Display& D = g_displays[display_id];

    // Collect all buffer IDs
    std::vector<int> buffer_ids;
    buffer_ids.reserve(buffers_.size());
    for (const auto& kv : buffers_) {
        buffer_ids.push_back(kv.first);
    }

    // Release slots
    for (int buf_id : buffer_ids) {
        D.slot_mgr.release(buf_id);
    }

    // Reset bindings
    for (int buf_id : buffer_ids) {
        auto it = buffers_.find(buf_id);
        if (it != buffers_.end()) {
            const BufferEntryPtr& entry = it->second;
            if (entry && entry->display_id == display_id) {
                reset_buffer_binding_locked(entry);
            }
        }
    }

    D.slot_mgr.reset();
}

SharedRwb BufferManager::create_rwb(int width, int height, uint32_t stride,
                                    int format, int usage, buffer_handle_t handle) {
    RemoteWindowBuffer* rb = new (std::nothrow) RemoteWindowBuffer(
        width, height, stride, format, usage, handle);

    if (!rb) return nullptr;

    return std::shared_ptr<RemoteWindowBuffer>(rb, [](RemoteWindowBuffer* p) {
        delete p;
    });
}

} // namespace createdisp
