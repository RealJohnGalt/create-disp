// event_handlers.cpp - EVDI event processing
#include "event_handlers.h"
#include "evdi_protocol.h"
#include "evdi_operations.h"
#include "buffer_entry.h"
#include "buffer_manager.h"
#include "gralloc_wrapper.h"
#include "sync_primitives.h"
#include "present_thread.h"
#include "logging.h"
#include "constants.h"
#include "display.h"
#include <cstring>
#include <unordered_map>

namespace createdisp {

// External display map
extern std::unordered_map<int, Display> g_displays;

void handle_get_buf(const void* data, int poll_id) {
    int id;
    std::memcpy(&id, data, sizeof(int));

    evdi::drm_evdi_get_buff_callback cmd = {};
    cmd.poll_id = poll_id;

    BufferEntryPtr entry;
    native_handle_t* handle = nullptr;

    {
        std::lock_guard<std::mutex> lk(g_state_mutex);
        entry = g_buffer_manager.get_buffer_locked(id);
        handle = (entry && entry->handle) ? entry->handle : nullptr;
    }

    if (!handle) {
        cmd.version = -1;
        cmd.numFds = -1;
        cmd.numInts = -1;
        cmd.fd_ints = nullptr;
        cmd.data_ints = nullptr;
    } else {
        cmd.version = handle->version;
        cmd.numFds = handle->numFds;
        cmd.numInts = handle->numInts;
        cmd.fd_ints = (handle->numFds > 0) ?
            const_cast<int*>(&handle->data[0]) : nullptr;
        cmd.data_ints = (handle->numInts > 0) ?
            const_cast<int*>(&handle->data[handle->numFds]) : nullptr;
    }

    evdi_ioctl(DRM_IOCTL_EVDI_GET_BUFF_CALLBACK, &cmd);
}

void handle_swap_to(const void* data, int poll_id) {
    struct { int id; int display_id; } ex = { -1, 0 };
    std::memcpy(&ex, data, sizeof(ex));

    const int id = ex.id;
    const int drv_display_id = ex.display_id;

    DisplaySnapshot snap;
    BufferEntryPtr entry;
    SharedRwb rwb;
    uint32_t slot = 0;
    uint32_t buf_stride = 0;
    int buf_w = 0, buf_h = 0, buf_format = 0;

    {
        std::lock_guard<std::mutex> lk(g_state_mutex);
        entry = g_buffer_manager.get_buffer_locked(id);

        if (!entry || !entry->handle) {
            request_display_resync(drv_display_id);
            return;
        }

        Display& D = g_displays[drv_display_id];
        snap = snapshot_display(D);

        if (!snap.hwcDisplay || !snap.connected || snap.width == 0 ||
            snap.height == 0 || snap.stride == 0) {
            request_display_resync(drv_display_id);
            return;
        }

        if (!D.connected || !D.hwcDisplay || D.generation != snap.generation) {
            request_display_resync(drv_display_id);
            return;
        }

        if (entry->generation != 0 && entry->generation != snap.generation) {
            request_display_resync(drv_display_id);
            return;
        }

        if (entry->display_id >= 0 && entry->display_id != drv_display_id &&
            entry->generation == snap.generation) {
            request_display_resync(drv_display_id);
            return;
        }

        if (entry->display_id != drv_display_id || entry->generation != snap.generation) {
            if ((entry->width != 0 && entry->width != snap.width) ||
                (entry->height != 0 && entry->height != snap.height)) {
                log_warning("Dropping stale buffer id=%d for display=%d "
                           "(buf=%dx%d, display=%dx%d)",
                           id, drv_display_id, entry->width, entry->height,
                           snap.width, snap.height);
                request_display_resync(drv_display_id);
                return;
            }

            g_buffer_manager.reset_buffer_binding_locked(entry);
            entry->display_id = drv_display_id;
            entry->generation = snap.generation;
        }

        slot = D.slot_mgr.assign(id);
        if (slot == UINT32_MAX) {
            log_error("SlotManager: failed to assign slot for bufid %d on display %d",
                     id, drv_display_id);
            return;
        }

        buf_format = entry->format;

        if (entry->origin == BufferOrigin::Imported) {
            buf_stride = entry->stride;
            buf_w = entry->width;
            buf_h = entry->height;
        } else {
            buf_stride = entry->stride;
            buf_w = entry->width ? entry->width : snap.width;
            buf_h = entry->height ? entry->height : snap.height;
        }

        rwb = entry->rwb;
    }

    if (buf_stride == 0 || buf_w <= 0 || buf_h <= 0) {
        log_error("Invalid buffer geometry for id=%d (w=%d, h=%d, stride=%u)",
                 id, buf_w, buf_h, buf_stride);
        request_display_resync(drv_display_id);
        return;
    }

    // Recreate RWB if geometry changed
    if (!rwb || entry->rwb_w != buf_w || entry->rwb_h != buf_h ||
        entry->rwb_stride != buf_stride || entry->rwb_format != buf_format) {

        rwb = g_buffer_manager.create_rwb(buf_w, buf_h, buf_stride,
                                          buf_format, kRwbUsage, entry->handle);

        if (!rwb) {
            log_error("Failed to allocate RemoteWindowBuffer for id=%d", id);
            return;
        }

        {
            std::lock_guard<std::mutex> lk(g_state_mutex);
            if (g_buffer_manager.get_buffer_locked(id) == entry) {
                entry->rwb = rwb;
                entry->rwb_w = buf_w;
                entry->rwb_h = buf_h;
                entry->rwb_stride = buf_stride;
                entry->rwb_format = buf_format;
            } else {
                return;
            }
        }
    }

    // Enqueue present job
    PresentJob job;
    job.drv_display_id = drv_display_id;
    job.buffer_id = id;
    job.generation = snap.generation;
    job.slot = slot;
    job.rwb = std::move(rwb);

    enqueue_present_job(std::move(job));
}

void handle_destroy_buf(const void* data, int poll_id) {
    int id = *static_cast<const int*>(data);

    {
        std::lock_guard<std::mutex> lk(g_state_mutex);
        g_buffer_manager.erase_buffer_locked(id);
    }

    evdi::drm_evdi_destroy_buff_callback cmd = { .poll_id = poll_id };
    evdi_ioctl(DRM_IOCTL_EVDI_DESTROY_BUFF_CALLBACK, &cmd);
}

void handle_create_buf(const void* data, int poll_id) {
    evdi::drm_evdi_create_buff buff_params;
    std::memcpy(&buff_params, data, sizeof(buff_params));

    const native_handle_t* full_handle;
    int req_format = (buff_params.format != 0) ?
        static_cast<int>(buff_params.format) : HAL_PIXEL_FORMAT_RGBA_8888;

    evdi::drm_evdi_create_buff_callback cmd;

    int ret = gralloc_allocate(buff_params.width, buff_params.height,
                               req_format, kRwbUsage,
                               (buffer_handle_t*)&full_handle, &cmd.stride);

    if (ret != 0) {
        log_error("gralloc_allocate failed: %d", ret);
        cmd.id = -1;
        cmd.poll_id = poll_id;
        cmd.stride = 0;
        evdi_ioctl(DRM_IOCTL_EVDI_GBM_CREATE_BUFF_CALLBACK, &cmd);
        return;
    }

    cmd.id = g_buffer_manager.add_buffer(
        const_cast<native_handle_t*>(full_handle),
        BufferOrigin::Allocated,
        req_format, cmd.stride,
        buff_params.width, buff_params.height);

    cmd.poll_id = poll_id;
    ret = evdi_ioctl(DRM_IOCTL_EVDI_GBM_CREATE_BUFF_CALLBACK, &cmd);

    if (ret < 0) {
        std::lock_guard<std::mutex> lk(g_state_mutex);
        g_buffer_manager.erase_buffer_locked(cmd.id);
    }
}

} // namespace createdisp
