// evdi_protocol.h - EVDI kernel interface definitions
#pragma once

#include <cstdint>
#include <sys/ioctl.h>
#include <xf86drm.h>

// EVDI command codes
#define DRM_EVDI_CONNECT          0x00
#define DRM_EVDI_REQUEST_UPDATE   0x01
#define DRM_EVDI_GRABPIX          0x02
#define DRM_EVDI_ENABLE_CURSOR_EVENTS 0x03
#define DRM_EVDI_POLL             0x04
#define DRM_EVDI_GBM_ADD_BUFF     0x05
#define DRM_EVDI_GBM_GET_BUFF     0x06
#define DRM_EVDI_ADD_BUFF_CALLBACK 0x07
#define DRM_EVDI_GET_BUFF_CALLBACK 0x08
#define DRM_EVDI_DESTROY_BUFF_CALLBACK 0x09
#define DRM_EVDI_GBM_DEL_BUFF     0x0B
#define DRM_EVDI_GBM_CREATE_BUFF  0x0C
#define DRM_EVDI_GBM_CREATE_BUFF_CALLBACK 0x0D
#define DRM_EVDI_VSYNC            0x0E

struct drm_evdi_connect {
    int32_t connected;
    int32_t dev_index;
    uint32_t width;
    uint32_t height;
    uint32_t refresh_rate;
    uint32_t display_id;
};

struct drm_evdi_poll {
    int event;
    int poll_id;
    void* data;
};

struct drm_evdi_get_buff_callback {
    int poll_id;
    int version;
    int numFds;
    int numInts;
    int* fd_ints;
    int* data_ints;
};

struct drm_evdi_destroy_buff_callback {
    int poll_id;
};

struct drm_evdi_create_buff_callback {
    int poll_id;
    int id;
    uint32_t stride;
};

struct drm_evdi_vsync {
    uint32_t display_id;
};

#define DRM_IOCTL_EVDI_CONNECT \
    DRM_IOWR(DRM_COMMAND_BASE + DRM_EVDI_CONNECT, struct drm_evdi_connect)
#define DRM_IOCTL_EVDI_POLL \
    DRM_IOWR(DRM_COMMAND_BASE + DRM_EVDI_POLL, struct drm_evdi_poll)
#define DRM_IOCTL_EVDI_GET_BUFF_CALLBACK \
    DRM_IOWR(DRM_COMMAND_BASE + DRM_EVDI_GET_BUFF_CALLBACK, struct drm_evdi_get_buff_callback)
#define DRM_IOCTL_EVDI_DESTROY_BUFF_CALLBACK \
    DRM_IOWR(DRM_COMMAND_BASE + DRM_EVDI_DESTROY_BUFF_CALLBACK, struct drm_evdi_destroy_buff_callback)
#define DRM_IOCTL_EVDI_GBM_CREATE_BUFF_CALLBACK \
    DRM_IOWR(DRM_COMMAND_BASE + DRM_EVDI_GBM_CREATE_BUFF_CALLBACK, struct drm_evdi_create_buff_callback)
#define DRM_IOCTL_EVDI_VSYNC \
    DRM_IOW(DRM_COMMAND_BASE + DRM_EVDI_VSYNC, struct drm_evdi_vsync)

namespace createdisp {
namespace evdi {
using drm_evdi_connect = ::drm_evdi_connect;
using drm_evdi_poll = ::drm_evdi_poll;
using drm_evdi_get_buff_callback = ::drm_evdi_get_buff_callback;
using drm_evdi_destroy_buff_callback = ::drm_evdi_destroy_buff_callback;
using drm_evdi_create_buff_callback = ::drm_evdi_create_buff_callback;
using drm_evdi_vsync = ::drm_evdi_vsync;
}
}
