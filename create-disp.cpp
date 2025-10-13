#include <dirent.h>
#include <fcntl.h>
#include <iostream>
#include <inttypes.h>
#include <cstring>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <xf86drm.h>
#include <memory>
#include <cassert>
#include <unordered_map>
#include <vector>
#include <fstream>
#include <cmath>
#include <climits>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

#include <systemd/sd-daemon.h>

#include <hybris/hwc2/hwc2_compatibility_layer.h>
#include <hybris/gralloc/gralloc.h>
#include <hybris/platforms/common/windowbuffer.h>

#define DRM_EVDI_CONNECT          0x00
#define DRM_EVDI_REQUEST_UPDATE   0x01
#define DRM_EVDI_GRABPIX          0x02
#define DRM_EVDI_ENABLE_CURSOR_EVENTS 0x03
#define DRM_EVDI_POLL 0x04
#define DRM_EVDI_GBM_ADD_BUFF 0x05
#define DRM_EVDI_GBM_GET_BUFF 0x06
#define DRM_EVDI_ADD_BUFF_CALLBACK 0x07
#define DRM_EVDI_GET_BUFF_CALLBACK 0x08
#define DRM_EVDI_DESTROY_BUFF_CALLBACK 0x09
#define DRM_EVDI_SWAP_CALLBACK 0x0A
#define DRM_EVDI_GBM_DEL_BUFF 0x0B
#define DRM_EVDI_GBM_CREATE_BUFF 0x0C
#define DRM_EVDI_GBM_CREATE_BUFF_CALLBACK 0x0D

#define DRM_IOCTL_EVDI_CONNECT DRM_IOWR(DRM_COMMAND_BASE +  \
        DRM_EVDI_CONNECT, struct drm_evdi_connect)
#define DRM_IOCTL_EVDI_REQUEST_UPDATE DRM_IOWR(DRM_COMMAND_BASE +  \
        DRM_EVDI_REQUEST_UPDATE, struct drm_evdi_request_update)
#define DRM_IOCTL_EVDI_GRABPIX DRM_IOWR(DRM_COMMAND_BASE +  \
        DRM_EVDI_GRABPIX, struct drm_evdi_grabpix)
#define DRM_IOCTL_EVDI_ENABLE_CURSOR_EVENTS DRM_IOWR(DRM_COMMAND_BASE +  \
        DRM_EVDI_ENABLE_CURSOR_EVENTS, struct drm_evdi_enable_cursor_events)
#define DRM_IOCTL_EVDI_POLL DRM_IOWR(DRM_COMMAND_BASE +  \
        DRM_EVDI_POLL, struct drm_evdi_poll)
#define DRM_IOCTL_EVDI_GBM_ADD_BUFF DRM_IOWR(DRM_COMMAND_BASE +  \
        DRM_EVDI_GBM_ADD_BUFF, struct drm_evdi_gbm_add_buf)
#define DRM_IOCTL_EVDI_GBM_GET_BUFF DRM_IOWR(DRM_COMMAND_BASE +  \
        DRM_EVDI_GBM_GET_BUFF, struct drm_evdi_gbm_get_buff)
#define DRM_IOCTL_EVDI_ADD_BUFF_CALLBACK DRM_IOWR(DRM_COMMAND_BASE +  \
        DRM_EVDI_ADD_BUFF_CALLBACK, struct drm_evdi_add_buff_callabck)
#define DRM_IOCTL_EVDI_GET_BUFF_CALLBACK DRM_IOWR(DRM_COMMAND_BASE +  \
        DRM_EVDI_GET_BUFF_CALLBACK, struct drm_evdi_get_buff_callabck)
#define DRM_IOCTL_EVDI_DESTROY_BUFF_CALLBACK DRM_IOWR(DRM_COMMAND_BASE +  \
        DRM_EVDI_DESTROY_BUFF_CALLBACK, struct drm_evdi_destroy_buff_callback)
#define DRM_IOCTL_EVDI_SWAP_CALLBACK DRM_IOWR(DRM_COMMAND_BASE +  \
        DRM_EVDI_SWAP_CALLBACK, struct drm_evdi_swap_callback)
#define DRM_IOCTL_EVDI_GBM_CREATE_BUFF_CALLBACK DRM_IOWR(DRM_COMMAND_BASE +  \
DRM_EVDI_GBM_CREATE_BUFF_CALLBACK, struct drm_evdi_create_buff_callabck)

enum poll_event_type {
    none,
    add_buf,
    get_buf,
    destroy_buf,
    swap_to,
    create_buf
};

struct drm_evdi_request_update {
    int32_t reserved;
};

struct drm_evdi_connect {
    int32_t connected;
    int32_t dev_index;
    uint32_t width;
    uint32_t height;
    uint32_t refresh_rate;
};

struct drm_evdi_poll {
    poll_event_type event;
    int poll_id;
    void *data;
};

struct drm_evdi_add_buff_callabck {
    int poll_id;
    int buff_id;
};

struct drm_evdi_get_buff_callabck {
    int poll_id;
    int version;
    int numFds;
    int numInts;
    int *fd_ints;
    int *data_ints;
};

struct drm_evdi_destroy_buff_callback {
    int poll_id;
};

struct drm_evdi_swap_callback {
    int poll_id;
};

struct drm_evdi_gbm_get_buff {
    int id;
    void *native_handle;
};

struct drm_evdi_gbm_create_buff {
    int *id;
    uint32_t *stride;
    uint32_t format;
    uint32_t width;
    uint32_t height;
};

struct drm_evdi_create_buff_callabck {
    int poll_id;
    int id;
    uint32_t stride;
};

class DisplayPipeline {
public:
    /* HWC2 */
    hwc2_display_t display_id{0};
    hwc2_compat_display_t* hwc_display{nullptr};
    hwc2_compat_layer_t* layer{nullptr};

    /* DRM/evdi */
    int drm_fd{-1};
    int device_index{0};

    /* Geometry */
    std::atomic<uint32_t> width{0};
    std::atomic<uint32_t> height{0};
    uint32_t stride{0};

    /* Buffer state */
    mutable std::mutex maps_mutex;
    std::unordered_map<int, std::unique_ptr<RemoteWindowBuffer>> buffers_map;
    std::unordered_map<int, std::unique_ptr<native_handle_t>> handles_map;
    std::unordered_map<std::string, int> handle_index;
    std::atomic<int> next_id{0};

    /* Reconfig */
    std::atomic<bool> reconfig_in_progress{false};
    std::atomic<bool> active{false};
    std::thread poll_thread;
    std::atomic<bool> stop_poll{false};

    DisplayPipeline() = default;
    ~DisplayPipeline() { cleanup(); }

    bool initialize(hwc2_display_t id, hwc2_compat_display_t* hwc_disp, int dev_idx);
    void cleanup();
    bool update_geometry();
    void clear_buffers();
    void poll_loop();

    /* Buffer management */
    int add_handle(const native_handle_t& handle);
    native_handle_t* get_handle(int id);

    /* Handle event processing */
    void handle_add_buf(void* data, int poll_id);
    void handle_get_buf(void* data, int poll_id);
    void handle_swap_to(void* data, int poll_id);
    void handle_destroy_buf(void* data, int poll_id);
    void handle_create_buf(void* data, int poll_id);

private:
    std::string make_handle_key(const native_handle_t* h);
    bool ensure_layer_configured();
    int open_evdi_device(int dev_idx);
};

/* Global state */
static hwc2_compat_device_t* g_hwc_device = nullptr;
static std::unordered_map<hwc2_display_t, std::unique_ptr<DisplayPipeline>> g_pipelines;
static std::mutex g_pipelines_mutex;
static std::atomic<bool> g_shutdown{false};
static std::atomic<int> g_next_device_index{0};
static std::condition_variable g_pipe_cv;
static std::mutex g_pipe_cv_mtx;

static bool create_display_pipeline(hwc2_display_t display_id, bool connected);
static void destroy_display_pipeline(hwc2_display_t display_id);
static int hz_from_period_ns(int32_t ns);
static int get_refresh_hz_from_active_config(const HWC2DisplayConfig* cfg);

std::string DisplayPipeline::make_handle_key(const native_handle_t* h) {
    const int total_ints = h->numInts;
    const size_t key_bytes = (3 + total_ints) * sizeof(int);
    std::string key(key_bytes, '\0');
    size_t off = 0;
    memcpy(&key[off], &h->version, sizeof(int)); off += sizeof(int);
    memcpy(&key[off], &h->numFds, sizeof(int));  off += sizeof(int);
    memcpy(&key[off], &h->numInts, sizeof(int)); off += sizeof(int);
    memcpy(&key[off], &h->data[h->numFds], total_ints * sizeof(int));
    return key;
}

int DisplayPipeline::open_evdi_device(int dev_idx) {
    // Try to find existing evdi-lindroid device
    const std::string dri_path = "/dev/dri/";
    std::vector<std::string> candidates;

    if (DIR* dir = opendir(dri_path.c_str())) {
        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            if (strncmp(entry->d_name, "card", 4) == 0) {
                candidates.emplace_back(dri_path + entry->d_name);
            }
        }
        closedir(dir);
    }

    // Try existing cards first
    for (const auto& path : candidates) {
        int fd = open(path.c_str(), O_RDWR | O_CLOEXEC);
        if (fd < 0) continue;

        drmVersionPtr version = drmGetVersion(fd);
        if (version) {
            std::string driver_name(version->name, version->name_len);
            drmFreeVersion(version);
            if (driver_name == "evdi-lindroid") {
                if (drmIsMaster(fd)) {
                    if (ioctl(fd, DRM_IOCTL_DROP_MASTER, nullptr) < 0) {
                        close(fd);
                        continue;
                    }
                }
                return fd;
            }
        }
        close(fd);
    }

    // Create new device if needed
    std::ofstream evdi_add("/sys/devices/evdi-lindroid/add");
    if (!evdi_add) return -1;
    evdi_add << "1";
    evdi_add.close();

    // Wait for device to appear
    for (int i = 0; i < 30; ++i) {
        sleep(1);
        if (DIR* dir = opendir(dri_path.c_str())) {
            struct dirent* entry;
            while ((entry = readdir(dir)) != nullptr) {
                if (strncmp(entry->d_name, "card", 4) == 0) {
                    std::string path = dri_path + entry->d_name;
                    int fd = open(path.c_str(), O_RDWR | O_CLOEXEC);
                    if (fd < 0) continue;

                    drmVersionPtr version = drmGetVersion(fd);
                    if (version) {
                        std::string driver_name(version->name, version->name_len);
                        drmFreeVersion(version);
                        if (driver_name == "evdi-lindroid") {
                            closedir(dir);
                            return fd;
                        }
                    }
                    close(fd);
                }
            }
            closedir(dir);
        }
    }

    return -1;
}

static inline bool pipeline_exists_unlocked(hwc2_display_t id)
{
    return g_pipelines.find(id) != g_pipelines.end();
}

bool DisplayPipeline::initialize(hwc2_display_t id, hwc2_compat_display_t* hwc_disp, int dev_idx) {
    display_id = id;
    hwc_display = hwc_disp;
    device_index = dev_idx;

    drm_fd = open_evdi_device(dev_idx);
    if (drm_fd < 0) return false;

    hwc2_compat_display_set_power_mode(hwc_display, HWC2_POWER_MODE_ON);

    if (!update_geometry()) {
        close(drm_fd);
        drm_fd = -1;
        return false;
    }

    active = true;
    stop_poll = false;
    poll_thread = std::thread(&DisplayPipeline::poll_loop, this);

    return true;
}

void DisplayPipeline::poll_loop()
{
    while (!stop_poll.load(std::memory_order_relaxed)) {
        struct drm_evdi_poll poll_cmd;
        poll_cmd.data = malloc(1024);
        int ret = ioctl(drm_fd, DRM_IOCTL_EVDI_POLL, &poll_cmd);
        if (ret == 0) {
            switch (poll_cmd.event) {
                case add_buf:		handle_add_buf(poll_cmd.data, poll_cmd.poll_id); break;
                case get_buf:		handle_get_buf(poll_cmd.data, poll_cmd.poll_id); break;
                case swap_to:		handle_swap_to(poll_cmd.data, poll_cmd.poll_id); break;
                case destroy_buf:	handle_destroy_buf(poll_cmd.data, poll_cmd.poll_id); break;
                case create_buf:	handle_create_buf(poll_cmd.data, poll_cmd.poll_id); break;
                default: break;
            }
        }
        free(poll_cmd.data);
    }
}


void DisplayPipeline::cleanup() {
    active = false;
    stop_poll = true;
    if (poll_thread.joinable())
        poll_thread.join();

    if (drm_fd >= 0) {
        close(drm_fd);
        drm_fd = -1;
    }
    clear_buffers();
}

bool DisplayPipeline::ensure_layer_configured() {
    if (!layer && hwc_display) {
        layer = hwc2_compat_display_create_layer(hwc_display);
        if (!layer) return false;

        hwc2_compat_layer_set_composition_type(layer, HWC2_COMPOSITION_CLIENT);
        hwc2_compat_layer_set_blend_mode(layer, HWC2_BLEND_MODE_NONE);
    }

    if (layer) {
        uint32_t w = width.load();
        uint32_t h = height.load();
        hwc2_compat_layer_set_source_crop(layer, 0.0f, 0.0f, (float)w, (float)h);
        hwc2_compat_layer_set_display_frame(layer, 0, 0, (int32_t)w, (int32_t)h);
        hwc2_compat_layer_set_visible_region(layer, 0, 0, (int32_t)w, (int32_t)h);
    }

    return layer != nullptr;
}

bool DisplayPipeline::update_geometry() {
    if (!hwc_display) return false;

    // Prevent concurrent geometry updates
    bool expected = false;
    if (!reconfig_in_progress.compare_exchange_strong(expected, true)) {
        return true;
    }

    HWC2DisplayConfig* config = hwc2_compat_display_get_active_config(hwc_display);
    if (!config) {
        reconfig_in_progress = false;
        return false;
    }

    uint32_t new_width = (uint32_t)config->width;
    uint32_t new_height = (uint32_t)config->height;

    bool geometry_changed = (width.load() != new_width || height.load() != new_height);

    if (geometry_changed) {
        width = new_width;
        height = new_height;

        if (!ensure_layer_configured()) {
            reconfig_in_progress = false;
            return false;
        }

        // Reconnect evdi with new geometry
        int refresh_hz = get_refresh_hz_from_active_config(config);
        drm_evdi_connect cmd = {
            .connected = 1,
            .dev_index = device_index,
            .width = new_width,
            .height = new_height,
            .refresh_rate = (uint32_t)refresh_hz,
        };

        if (ioctl(drm_fd, DRM_IOCTL_EVDI_CONNECT, &cmd) < 0) {
            reconfig_in_progress = false;
            return false;
        }

        clear_buffers();

        std::cout << "Display " << display_id << " reconfigured to " 
                  << new_width << "x" << new_height << "@" << refresh_hz << "Hz" << std::endl;
    }

    reconfig_in_progress = false;
    return true;
}

void DisplayPipeline::clear_buffers() {
    std::lock_guard<std::mutex> lock(maps_mutex);
    buffers_map.clear();
    handles_map.clear();
    handle_index.clear();
}

int DisplayPipeline::add_handle(const native_handle_t& handle) {
    std::lock_guard<std::mutex> lock(maps_mutex);

    size_t total_size = sizeof(native_handle_t) + (handle.numFds + handle.numInts) * sizeof(int);
    native_handle_t* copied_handle = (native_handle_t*)malloc(total_size);
    if (!copied_handle) return -1;

    memcpy(copied_handle, &handle, total_size);

    int id = next_id.fetch_add(1);
    handles_map[id] = std::unique_ptr<native_handle_t>(copied_handle);
    return id;
}

native_handle_t* DisplayPipeline::get_handle(int id) {
    std::lock_guard<std::mutex> lock(maps_mutex);
    auto it = handles_map.find(id);
    return (it != handles_map.end()) ? it->second.get() : nullptr;
}

void DisplayPipeline::handle_add_buf(void* data, int poll_id) {
    int fd;
    memcpy(&fd, data, sizeof(int));

    if (fcntl(fd, F_GETFD) == -1) return;
    if (lseek(fd, 0, SEEK_SET) == -1) return;

    int header[3];
    if (read(fd, header, sizeof(header)) != sizeof(header)) return;

    int version = header[0];
    int numFds = header[1];
    int numInts = header[2];

    if (lseek(fd, 0, SEEK_SET) == -1) return;

    size_t total_size = sizeof(native_handle_t) + ((numFds + numInts) * sizeof(int));
    native_handle_t* full_handle = (native_handle_t*)malloc(total_size);
    if (!full_handle) return;

    if (read(fd, full_handle, total_size) != total_size) {
        free(full_handle);
        return;
    }

    int id = -1;
    {
        std::lock_guard<std::mutex> lock(maps_mutex);
        const std::string key = make_handle_key(full_handle);
        auto it = handle_index.find(key);
        if (it != handle_index.end()) {
            id = it->second;
            free(full_handle);
        } else {
            id = add_handle(*full_handle);
            handle_index.emplace(key, id);
            free(full_handle);
        }
    }

    close(fd);

    struct drm_evdi_add_buff_callabck cmd = {.poll_id = poll_id, .buff_id = id};
    ioctl(drm_fd, DRM_IOCTL_EVDI_ADD_BUFF_CALLBACK, &cmd);
}

void DisplayPipeline::handle_get_buf(void* data, int poll_id) {
    int id;
    memcpy(&id, data, sizeof(int));

    native_handle_t* handle = get_handle(id);
    struct drm_evdi_get_buff_callabck cmd;

    if (!handle) {
        cmd = {.poll_id = poll_id, .version = -1, .numFds = -1, .numInts = -1, 
               .fd_ints = nullptr, .data_ints = nullptr};
    } else {
        cmd = {.poll_id = poll_id, .version = handle->version, .numFds = handle->numFds, 
               .numInts = handle->numInts, 
               .fd_ints = const_cast<int*>(&handle->data[0]), 
               .data_ints = const_cast<int*>(&handle->data[handle->numFds])};
    }

    ioctl(drm_fd, DRM_IOCTL_EVDI_GET_BUFF_CALLBACK, &cmd);
}

void DisplayPipeline::handle_swap_to(void* data, int poll_id) {
    int id;
    memcpy(&id, data, sizeof(int));

    auto send_swap_cb = [this, poll_id]() {
        struct drm_evdi_swap_callback cmd = {.poll_id = poll_id};
        ioctl(drm_fd, DRM_IOCTL_EVDI_SWAP_CALLBACK, &cmd);
    };

    native_handle_t* in_handle = get_handle(id);
    if (!in_handle) { send_swap_cb(); return; }

    uint32_t curr_width = width.load();
    uint32_t curr_height = height.load();
    RemoteWindowBuffer* buf = nullptr;

    {
        std::lock_guard<std::mutex> lock(maps_mutex);
        auto it = buffers_map.find(id);
        if (it == buffers_map.end()) {
            auto new_buf = std::make_unique<RemoteWindowBuffer>(
                curr_width, curr_height, stride,
                HAL_PIXEL_FORMAT_RGBA_8888,
                GRALLOC_USAGE_HW_TEXTURE | GRALLOC_USAGE_HW_RENDER | GRALLOC_USAGE_HW_COMPOSER,
                in_handle);
            buf = new_buf.get();
            buffers_map[id] = std::move(new_buf);
        } else {
            buf = it->second.get();
        }
    }

    if (buf->width != curr_width || buf->height != curr_height) {
        send_swap_cb();
        return;
    }

    if (hwc_display) {
        hwc2_compat_display_set_client_target(hwc_display, 0, buf, -1, HAL_DATASPACE_UNKNOWN);
        int presentFence;
        (void)hwc2_compat_display_present(hwc_display, &presentFence);
    }

    send_swap_cb();
}

void DisplayPipeline::handle_destroy_buf(void* data, int poll_id) {
    int id = *(int*)data;

    {
        std::lock_guard<std::mutex> lock(maps_mutex);
        auto handle_it = handles_map.find(id);
        if (handle_it != handles_map.end()) {
            native_handle_close(handle_it->second.get());
            handles_map.erase(handle_it);
        }
        buffers_map.erase(id);
    }

    struct drm_evdi_destroy_buff_callback cmd = {.poll_id = poll_id};
    ioctl(drm_fd, DRM_IOCTL_EVDI_DESTROY_BUFF_CALLBACK, &cmd);
}

void DisplayPipeline::handle_create_buf(void* data, int poll_id) {
    struct drm_evdi_gbm_create_buff buff_params;
    memcpy(&buff_params, data, sizeof(struct drm_evdi_gbm_create_buff));

    const native_handle_t* full_handle;
    uint32_t stride_out;
    int ret = hybris_gralloc_allocate(buff_params.width, buff_params.height, 
                                     HAL_PIXEL_FORMAT_RGBA_8888,
                                     GRALLOC_USAGE_HW_TEXTURE | GRALLOC_USAGE_HW_RENDER | GRALLOC_USAGE_HW_COMPOSER, 
                                     &full_handle, &stride_out);

    struct drm_evdi_create_buff_callabck cmd;
    if (ret == 0) {
        cmd.id = add_handle(*full_handle);
        cmd.stride = stride_out;
    } else {
        cmd.id = -1;
        cmd.stride = 0;
    }
    cmd.poll_id = poll_id;

    ioctl(drm_fd, DRM_IOCTL_EVDI_GBM_CREATE_BUFF_CALLBACK, &cmd);
}

static int hz_from_period_ns(int32_t ns) {
    if (ns <= 0) return 60;
    const double hz_f = 1e9 / static_cast<double>(ns);
    return static_cast<int>(std::lround(hz_f));
}

static int get_refresh_hz_from_active_config(const HWC2DisplayConfig* cfg) {
    return hz_from_period_ns(cfg->vsyncPeriod);
}

static bool create_display_pipeline(hwc2_display_t display_id, bool connected) {
    if (!connected) {
        destroy_display_pipeline(display_id);
        return true;
    }

    {
        std::lock_guard<std::mutex> lock(g_pipelines_mutex);
        if (pipeline_exists_unlocked(display_id)) {
            return true;
        }
    }

    hwc2_compat_display_t* hwc_disp = hwc2_compat_device_get_display_by_id(g_hwc_device, display_id);
    if (!hwc_disp) return false;

    auto pipeline = std::make_unique<DisplayPipeline>();
    int dev_idx = g_next_device_index.fetch_add(1);

    if (!pipeline->initialize(display_id, hwc_disp, dev_idx)) {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(g_pipelines_mutex);
        g_pipelines[display_id] = std::move(pipeline);
        g_pipe_cv.notify_all();
    }

    std::cout << "Created pipeline for display " << display_id << std::endl;
    return true;
}

static void destroy_display_pipeline(hwc2_display_t display_id) {
    std::unique_ptr<DisplayPipeline> pipeline;

    {
        std::lock_guard<std::mutex> lock(g_pipelines_mutex);
        auto it = g_pipelines.find(display_id);
        if (it != g_pipelines.end()) {
            pipeline = std::move(it->second);
            g_pipelines.erase(it);
            g_pipe_cv.notify_all();
        }
    }

    if (pipeline) {
        pipeline->cleanup();
        std::cout << "Destroyed pipeline for display " << display_id << std::endl;
    }
}

void onVsyncReceived(HWC2EventListener* listener, int32_t sequenceId,
                     hwc2_display_t display, int64_t timestamp) {
}

void onHotplugReceived(HWC2EventListener* listener, int32_t sequenceId,
                       hwc2_display_t display, bool connected, bool primaryDisplay) {
    std::cout << "Hotplug: display " << display << (connected ? " connected" : " disconnected") 
              << (primaryDisplay ? " (primary)" : " (external)") << std::endl;

    hwc2_compat_device_on_hotplug(g_hwc_device, display, connected);
    create_display_pipeline(display, connected);
}

void onRefreshReceived(HWC2EventListener* listener, int32_t sequenceId, hwc2_display_t display) {
    std::lock_guard<std::mutex> lock(g_pipelines_mutex);
    auto it = g_pipelines.find(display);
    if (it != g_pipelines.end() && it->second) {
        it->second->update_geometry();
    }
}

static HWC2EventListener g_event_listener = {
    &onVsyncReceived,
    &onHotplugReceived,
    &onRefreshReceived
};

int main() {
    sd_notifyf(0, "MAINPID=%lu", (unsigned long)getpid());
    sd_notify(0, "STATUS=Initializing create-disp…");

    g_hwc_device = hwc2_compat_device_new(false);
    if (!g_hwc_device) {
        std::cerr << "Failed to create HWC2 device" << std::endl;
        return EXIT_FAILURE;
    }

    hwc2_compat_device_register_callback(g_hwc_device, &g_event_listener, 0);

    for (int i = 0; i < 5000; ++i) {
        hwc2_compat_display_t* primary = hwc2_compat_device_get_display_by_id(g_hwc_device, 0);
        if (primary) {
            create_display_pipeline(0, true);
            break;
        }
        usleep(1000);
    }

    sd_notify(0, "READY=1");
    sd_notify(0, "STATUS=create-disp ready.");

    std::unique_lock<std::mutex> lk(g_pipe_cv_mtx);
    g_pipe_cv.wait(lk, []{
        return false;
    });

    g_shutdown = true;

    {
        std::lock_guard<std::mutex> lock(g_pipelines_mutex);
        for (auto& kv : g_pipelines) {
            kv.second->cleanup();
        }
        g_pipelines.clear();
    }

    sd_notify(0, "STATUS=Shutting down…");
    return EXIT_SUCCESS;
}
