// gralloc_wrapper.cpp - Gralloc operations wrapper
#include "gralloc_wrapper.h"

namespace createdisp {

int gralloc_allocate(uint32_t width, uint32_t height, int format,
                     int usage, buffer_handle_t* handle, uint32_t* stride) {
    return hybris_gralloc_allocate(width, height, format, usage, handle, stride);
}

int gralloc_release(buffer_handle_t handle) {
    return hybris_gralloc_release(handle, 1);
}

} // namespace createdisp
