// evdi_operations.cpp - EVDI ioctl operations
#include "evdi_operations.h"
#include "evdi_protocol.h"
#include "drm_device.h"
#include "sync_primitives.h"
#include <unistd.h>
#include <cerrno>
#include <cstdio>

namespace createdisp {

static int ioctl_retry(int fd, unsigned long request, void* arg) {
    int rc;
    do {
        rc = ioctl(fd, request, arg);
    } while (rc < 0 && errno == EINTR);
    return rc;
}

int evdi_ioctl(unsigned long request, void* arg) {
    int fd = g_drm_device.get_fd();
    if (fd < 0) {
        errno = EBADF;
        return -1;
    }
    return ioctl_retry(fd, request, arg);
}

int evdi_connect(int device_index, uint32_t width, uint32_t height,
                 uint32_t refresh_rate, uint32_t display_id, bool connect) {
    evdi::drm_evdi_connect cmd = {
        .connected = connect ? 1 : 0,
        .dev_index = device_index,
        .width = width,
        .height = height,
        .refresh_rate = refresh_rate,
        .display_id = display_id,
    };

    if (evdi_ioctl(DRM_IOCTL_EVDI_CONNECT, &cmd) < 0) {
        perror("DRM_IOCTL_EVDI_CONNECT failed");
        return -1;
    }

    return 0;
}

int evdi_vsync(DisplayId display_id) {
    evdi::drm_evdi_vsync cmd = {};
    cmd.display_id = static_cast<uint32_t>(display_id);

    int fd = g_drm_device.get_fd();
    if (fd < 0) return -EBADF;

    return ioctl_retry(fd, DRM_IOCTL_EVDI_VSYNC, &cmd);
}

bool should_request_reopen(int error_code) {
    return g_drm_device.is_ready() &&
           (error_code == ENODEV || error_code == EBADF) &&
           g_modeset_inflight.load(std::memory_order_acquire) == 0;
}

} // namespace createdisp
