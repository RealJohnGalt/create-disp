// signal_handler.h - Signal handling
#pragma once

#include <thread>

namespace createdisp {

void init_signal_handlers();
void handle_signal(int signo);
void kick_thread_out_of_ioctl(std::thread& t);

} // namespace createdisp
