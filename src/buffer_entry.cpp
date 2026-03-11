// buffer_entry.cpp - Buffer lifecycle
#include "buffer_entry.h"
#include "gralloc_wrapper.h"
#include <unistd.h>
#include <cstdlib>

namespace createdisp {

BufferEntry::~BufferEntry() {
    rwb.reset();

    if (!handle) return;

    if (origin == BufferOrigin::Imported) {
        // Close all FDs
        for (int i = 0; i < handle->numFds; i++) {
            if (handle->data[i] >= 0) {
                close(handle->data[i]);
            }
        }
        free(handle);
    } else {
        // Release gralloc-allocated buffer
        gralloc_release((buffer_handle_t)handle);
    }

    handle = nullptr;
}

} // namespace createdisp
