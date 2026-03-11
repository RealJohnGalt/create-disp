// sync_primitives.h - Synchronization objects
#pragma once

#include <mutex>
#include <condition_variable>
#include <deque>
#include <atomic>
#include <array>
#include "types.h"
#include "constants.h"

namespace createdisp {

// Global state mutex for displays, buffers, ID mappings
extern std::mutex g_state_mutex;

// Global running flag
extern std::atomic<bool> g_running;

// Display resync flags
extern std::array<std::atomic<bool>, kMaxDriverDisplays> g_resync_pending;

// Modeset in-flight counter
extern std::atomic<int> g_modeset_inflight;

// Update thread synchronization
extern std::mutex g_update_mutex;
extern std::condition_variable g_update_cv;
extern std::deque<int> g_update_queue;
extern bool g_pending_update[kMaxDriverDisplays];
extern bool g_pending_disconnect[kMaxDriverDisplays];
extern bool g_enqueued[kMaxDriverDisplays];

// Present thread synchronization
extern std::mutex g_present_mutex;
extern std::condition_variable g_present_cv;
extern std::deque<PresentJob> g_present_queue;

// Event thread synchronization
extern std::mutex g_event_mutex;
extern std::condition_variable g_event_cv;
extern std::atomic<bool> g_event_thread_sleeping;

// Work scheduling functions
void schedule_update(DisplayId display_id);
void schedule_disconnect(DisplayId display_id);
void request_display_resync(DisplayId display_id);

// Present job management
void enqueue_present_job(PresentJob&& job);
void flush_present_jobs_for_display(DisplayId display_id);

} // namespace createdisp
