// update_thread.h - Display update thread
#pragma once

#include "types.h"

namespace createdisp {

void update_thread_main();
int update_display(DisplayId display_id);
void disconnect_display(DisplayId display_id);

} // namespace createdisp
