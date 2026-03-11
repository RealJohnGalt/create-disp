// buffer_manager.h - Buffer allocation and lifecycle
#pragma once

#include <unordered_map>
#include <mutex>
#include <hybris/gralloc/gralloc.h>
#include <hybris/platforms/common/windowbuffer.h>
#include "types.h"

namespace createdisp {

class BufferManager {
public:
    BufferManager();

    BufferId add_buffer(native_handle_t* handle, BufferOrigin origin,
                        int format, uint32_t stride, uint32_t width, uint32_t height);

    BufferEntryPtr get_buffer_locked(BufferId id);

    void erase_buffer_locked(BufferId id);

    void reset_buffer_binding_locked(BufferEntryPtr entry);

    void reset_display_bindings_locked(DisplayId display_id);

    SharedRwb create_rwb(int width, int height, uint32_t stride,
                         int format, int usage, buffer_handle_t handle);

private:
    std::unordered_map<BufferId, BufferEntryPtr> buffers_;
    BufferId next_id_ = 1;
};

extern BufferManager g_buffer_manager;

} // namespace createdisp
