// drm_device.h - DRM device lifecycle management
#pragma once

#include <atomic>
#include <shared_mutex>

namespace createdisp {

// DRM device management with thread-safe access
class DrmDevice {
public:
    DrmDevice();
    ~DrmDevice();
    bool initialize();
    void shutdown();
    int get_fd() const;
    bool is_ready() const;
    void request_reopen();
    bool reopen_requested() const;

private:
    int find_evdi_device();
    int open_or_create_evdi();
    bool is_evdi_lindroid(int fd);

    mutable std::shared_mutex mutex_;
    int fd_ = -1;
    std::atomic<bool> ready_{false};
    std::atomic<bool> reopen_requested_{false};
};

extern DrmDevice g_drm_device;

} // namespace createdisp
