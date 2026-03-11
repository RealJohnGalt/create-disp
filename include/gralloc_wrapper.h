// gralloc_wrapper.h - Gralloc operations wrapper
#pragma once

#include <cstdint>
#include <hybris/gralloc/gralloc.h>

namespace createdisp {

int gralloc_allocate(uint32_t width, uint32_t height, int format,
                     int usage, buffer_handle_t* handle, uint32_t* stride);

int gralloc_release(buffer_handle_t handle);

} // namespace createdisp

