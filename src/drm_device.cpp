// drm_device.cpp - DRM device lifecycle
#include "drm_device.h"
#include "logging.h"
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <cstring>
#include <fstream>
#include <mutex>
#include <sys/ioctl.h>
#include <xf86drm.h>

namespace createdisp {

DrmDevice g_drm_device;

DrmDevice::DrmDevice() = default;

DrmDevice::~DrmDevice() {
    shutdown();
}

bool DrmDevice::initialize() {
    std::unique_lock<std::shared_mutex> lock(mutex_);

    fd_ = open_or_create_evdi();
    if (fd_ < 0) {
        log_error("Failed to open/create EVDI device");
        return false;
    }

    ready_.store(true, std::memory_order_release);
    log_info("DRM device initialized (fd=%d)", fd_);
    return true;
}

void DrmDevice::shutdown() {
    std::unique_lock<std::shared_mutex> lock(mutex_);

    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }

    ready_.store(false, std::memory_order_release);
}

int DrmDevice::get_fd() const {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return fd_;
}

bool DrmDevice::is_ready() const {
    return ready_.load(std::memory_order_acquire);
}

void DrmDevice::request_reopen() {
    reopen_requested_.store(true, std::memory_order_release);
}

bool DrmDevice::reopen_requested() const {
    return reopen_requested_.load(std::memory_order_acquire);
}

bool DrmDevice::is_evdi_lindroid(int fd) {
    drmVersionPtr version = drmGetVersion(fd);
    if (!version) return false;

    std::string driver_name(version->name, version->name_len);
    drmFreeVersion(version);

    return (driver_name == "evdi-lindroid");
}

int DrmDevice::find_evdi_device() {
    static const char* dri_path = "/dev/dri/";
    DIR* dir = opendir(dri_path);
    if (!dir) return -1;

    int found_fd = -1;
    struct dirent* entry;

    while ((entry = readdir(dir)) != nullptr) {
        if (strncmp(entry->d_name, "card", 4) != 0) continue;

        std::string path = std::string(dri_path) + entry->d_name;
        int fd = open(path.c_str(), O_RDWR | O_CLOEXEC);
        if (fd < 0) continue;

        if (!is_evdi_lindroid(fd)) {
            close(fd);
            continue;
        }

        log_info("Found evdi-lindroid at %s", path.c_str());

        // Drop master if we have it
        if (drmIsMaster(fd)) {
            if (ioctl(fd, DRM_IOCTL_DROP_MASTER, nullptr) < 0) {
                log_error("Failed to drop master on %s: %s",
                         path.c_str(), strerror(errno));
                close(fd);
                break;
            }
        }

        found_fd = fd;
        break;
    }

    closedir(dir);
    return found_fd;
}

int DrmDevice::open_or_create_evdi() {
    int fd = find_evdi_device();
    if (fd >= 0) return fd;

    // Try to create device
    log_info("evdi-lindroid not found. Attempting to create...");

    std::ofstream evdi_add("/sys/devices/evdi-lindroid/add");
    if (!evdi_add) {
        log_error("Failed to write to /sys/devices/evdi-lindroid/add: %s",
                 strerror(errno));
        return -1;
    }

    evdi_add << "1";
    evdi_add.close();

    // Wait for device to appear
    for (int wait_time = 0; wait_time < kDeviceWaitLimit;
         wait_time += kDeviceWaitInterval) {
        fd = find_evdi_device();
        if (fd >= 0) return fd;
        sleep(kDeviceWaitInterval);
    }

    log_error("evdi-lindroid still not available after add attempt");
    return -1;
}

} // namespace createdisp
