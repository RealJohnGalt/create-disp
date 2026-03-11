// evdi_operations.h - EVDI ioctl operations
#pragma once

#include <cstdint>
#include "types.h"

namespace createdisp {

int evdi_connect(int device_index, uint32_t width, uint32_t height,
                 uint32_t refresh_rate, uint32_t display_id, bool connect);
int evdi_vsync(DisplayId display_id);
int evdi_ioctl(unsigned long request, void* arg);
bool should_request_reopen(int error_code);

} // namespace createdisp
