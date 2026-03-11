// main.cpp - Entry
#include "drm_device.h"
#include "hwc_manager.h"
#include "hwc_callbacks.h"
#include "sync_primitives.h"
#include "signal_handler.h"
#include "logging.h"
#include "poll_thread.h"
#include "event_thread.h"
#include "update_thread.h"
#include "present_thread.h"
#include <systemd/sd-daemon.h>
#include <cstdlib>
#include <thread>

using namespace createdisp;

int main() {
    log_info("create-disp starting...");

    // Initialize signal handlers
    init_signal_handlers();

    // Initialize DRM device
    sd_notify(0, "STATUS=Initializing DRM device...");
    if (!g_drm_device.initialize()) {
        log_error("Failed to initialize DRM device");
        return EXIT_FAILURE;
    }

    // Initialize HWC device
    sd_notify(0, "STATUS=Initializing HWC device...");
    if (!g_hwc_manager.initialize()) {
        log_error("Failed to initialize HWC device");
        g_drm_device.shutdown();
        return EXIT_FAILURE;
    }

    // Register HWC event listener
    hwc2_compat_device_register_callback(g_hwc_manager.get_device(),
                                         &g_event_listener, 0);

    // Start worker threads
    std::thread poll_thread;
    std::thread event_thread;
    std::thread update_thread;
    std::thread present_thread;

    try {
        sd_notify(0, "STATUS=Starting worker threads...");

        poll_thread = std::thread(poll_thread_main);
        event_thread = std::thread(event_thread_main);
        update_thread = std::thread(update_thread_main);
        present_thread = std::thread(present_thread_main);

    } catch (const std::exception& e) {
        log_error("Failed to create worker threads: %s", e.what());
        g_running.store(false, std::memory_order_release);

        g_update_cv.notify_all();
        g_present_cv.notify_all();
        g_event_cv.notify_all();

        if (poll_thread.joinable()) poll_thread.join();
        if (event_thread.joinable()) event_thread.join();
        if (update_thread.joinable()) update_thread.join();
        if (present_thread.joinable()) present_thread.join();

        g_hwc_manager.shutdown();
        g_drm_device.shutdown();
        return EXIT_FAILURE;
    }

    sd_notify(0, "READY=1\nSTATUS=Running");
    log_info("create-disp initialized successfully");

    // Main event loop
    while (g_running.load(std::memory_order_acquire)) {
        pause();
    }

    // Shutdown sequence
    log_info("Shutting down...");
    sd_notify(0, "STOPPING=1\nSTATUS=Shutting down...");

    g_running.store(false, std::memory_order_release);
    g_update_cv.notify_all();
    g_present_cv.notify_all();
    g_event_cv.notify_all();

    // Stop poll thread
    if (poll_thread.joinable()) {
        kick_thread_out_of_ioctl(poll_thread);
    }

    // Join all threads
    sd_notify(0, "STATUS=Stopping worker threads...");

    if (poll_thread.joinable()) poll_thread.join();
    if (event_thread.joinable()) event_thread.join();
    if (update_thread.joinable()) update_thread.join();
    if (present_thread.joinable()) present_thread.join();

    // Cleanup
    g_hwc_manager.shutdown();
    g_drm_device.shutdown();

    log_info("create-disp shutdown complete");
    return EXIT_SUCCESS;
}
