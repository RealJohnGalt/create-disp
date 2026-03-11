// signal_handler.cpp - Signal handling
#include "signal_handler.h"
#include "sync_primitives.h"
#include "constants.h"
#include <csignal>
#include <pthread.h>

namespace createdisp {

void handle_signal(int signo) {
    g_running.store(false, std::memory_order_release);
    g_update_cv.notify_all();
    g_present_cv.notify_all();
    g_event_cv.notify_all();
}

void init_signal_handlers() {
    struct sigaction sa;
    sa.sa_handler = handle_signal;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;

    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    // Ignore SIGUSR1 in main thread (used for kicking threads)
    signal(kShutdownKickSignal, SIG_IGN);
}

void kick_thread_out_of_ioctl(std::thread& t) {
    if (!t.joinable()) return;

    pthread_t native = t.native_handle();
    pthread_kill(native, kShutdownKickSignal);
}

} // namespace createdisp
