// event_handlers.h - EVDI event processing functions
#pragma once

#include <cstdint>

namespace createdisp {

// Event handler functions called by event thread
void handle_get_buf(const void* data, int poll_id);
void handle_swap_to(const void* data, int poll_id);
void handle_destroy_buf(const void* data, int poll_id);
void handle_create_buf(const void* data, int poll_id);

} // namespace createdisp
