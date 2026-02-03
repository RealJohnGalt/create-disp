#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <sys/eventfd.h>
#include <cinttypes>
#include <mutex>
#include <iostream>
#include <sstream>
#include <cstring>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <xf86drm.h>
#include <array>
#include <memory>
#include <cassert>
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <vector>
#include <new>
#include <fstream>
#include <cmath>
#include <climits>
#include <cstdint>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <condition_variable>

#include <systemd/sd-daemon.h>

#include <hybris/hwc2/hwc2_compatibility_layer.h>
#include <hybris/gralloc/gralloc.h>
#include <hybris/platforms/common/windowbuffer.h>

struct drm_evdi_swap_callback;
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
#define DRM_EVDI_SET_ACQUIRE_FENCE 0x0E

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
#define DRM_IOCTL_EVDI_SET_ACQUIRE_FENCE DRM_IOWR(DRM_COMMAND_BASE +  \
        DRM_EVDI_SET_ACQUIRE_FENCE, struct drm_evdi_set_acquire_fence)

static const int kMaxDriverDisplays = 5;
static std::unordered_map<long long, int> g_hwc_to_drv;
static std::unordered_map<int, long long> g_drv_to_hwc;
static std::vector<int> g_free_drv_ids;
static volatile bool drm_ready = false;

namespace {
struct SmallBufPool {
    static constexpr size_t kBuckets[5] = {256, 512, 1024, 2048, 4096};
    std::vector<void*> free_[5];

    void* alloc(size_t need) {
        for (int i = 0; i < 5; ++i) {
            if (need <= kBuckets[i]) {
                if (!free_[i].empty()) {
                    void* p = free_[i].back();
                    free_[i].pop_back();
                    return p;
                }
                return ::malloc(kBuckets[i]);
            }
        }
        return ::malloc(need);
    }
    void dealloc(void* p, size_t had) {
        if (!p) return;
        for (int i = 0; i < 5; ++i) {
            if (had <= kBuckets[i]) {
                free_[i].push_back(p);
                return;
            }
        }
        ::free(p);
    }
    ~SmallBufPool() {
        for (int i = 0; i < 5; ++i) {
            for (void* p : free_[i]) ::free(p);
            free_[i].clear();
        }
    }
};
static SmallBufPool g_small_pool;

static inline uint64_t fnv1a64(const void* data, size_t len) {
    const uint8_t* b = static_cast<const uint8_t*>(data);
    uint64_t h = 1469598103934665603ull;
    for (size_t i = 0; i < len; ++i) {
        h ^= b[i];
        h *= 1099511628211ull;
    }
    return h;
}
} // namespace

namespace {
struct RwbPool {
    std::vector<void*> free_blocks;
    void* acquire() {
        if (!free_blocks.empty()) {
            void* p = free_blocks.back();
            free_blocks.pop_back();
            return p;
        }
        return ::operator new(sizeof(RemoteWindowBuffer));
    }
    void release(void* p) { if (p) free_blocks.push_back(p); }
    ~RwbPool() {
        for (void* p : free_blocks) ::operator delete(p);
        free_blocks.clear();
    }
};
static RwbPool g_rwb_pool;
struct RwbDeleter { void operator()(RemoteWindowBuffer* p) const { if (!p) return; p->~RemoteWindowBuffer(); g_rwb_pool.release(static_cast<void*>(p)); } };
using UniqueRwb = std::unique_ptr<RemoteWindowBuffer, RwbDeleter>;
} // namespace

struct Display {
    int display_id = -1;
    int width = 0;
    int height = 0;
    uint32_t stride = 0;
    hwc2_compat_display_t* hwcDisplay = nullptr;
    hwc2_compat_layer_t* layer = nullptr;
    std::atomic<bool> seen_vsync{false};
};

static std::unordered_map<int, Display> g_displays;

// Poll thread
static std::thread g_poll_thread;
static std::atomic<bool> g_running{true};

#ifndef likely
#define likely(x)	__builtin_expect(!!(x), 1)
#define unlikely(x)	__builtin_expect(!!(x), 0)
#endif

static constexpr int kRwbUsage =
        GRALLOC_USAGE_HW_TEXTURE |
        GRALLOC_USAGE_HW_RENDER |
        GRALLOC_USAGE_HW_COMPOSER;

static inline int drv_id_for_hwc(long long hwc_id);

static inline Display& get_or_create_display(int display_id) {
    auto it = g_displays.find(display_id);
    if (it != g_displays.end()) return it->second;
    auto res = g_displays.emplace(std::piecewise_construct,
                                  std::forward_as_tuple(display_id),
                                  std::forward_as_tuple());
    Display &d = res.first->second;
    d.display_id = display_id;
    return d;
}

struct HandleInfo {
    std::unique_ptr<native_handle_t> handle;
    int id;
};

int drm_fd;
hwc2_compat_device_t* hwcDevice;
static std::unordered_map<int, UniqueRwb> buffers_map;
static std::unordered_map<int, std::unique_ptr<native_handle_t>> handles_map;
static std::unordered_map<uint64_t, std::vector<int>> handle_index;
static std::unordered_map<int, uint64_t> handle_hash_by_id;
static inline bool handles_equal(const native_handle_t* a, const native_handle_t* b) {
    if (!a || !b) return false;
    if (a->version != b->version) return false;
    if (a->numFds  != b->numFds)  return false;
    if (a->numInts != b->numInts) return false;
    const int ints = a->numInts;
    return std::memcmp(&a->data[a->numFds], &b->data[b->numFds], sizeof(int) * ints) == 0;
}
static inline uint64_t make_handle_hash(const native_handle_t* h) {
    const int ints = h->numInts;
    struct Hdr { int v, f, i; } hdr{h->version, h->numFds, h->numInts};
    uint64_t h1 = fnv1a64(&hdr, sizeof(Hdr));
    uint64_t h2 = fnv1a64(&h->data[h->numFds], sizeof(int) * ints);
    /* Mix */
    return (h1 ^ (h2 + 0x9e3779b97f4a7c15ull + (h1<<6) + (h1>>2)));
}

static constexpr size_t kExpectedHandles = 4096;
/* buffer ids must be > 0 to not break PRIME export */
int next_id = 1;
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
        uint32_t display_id;
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

struct drm_evdi_swap_event {
        int id;
        int display_id;
        int acquire_fence_fd;
};

struct drm_evdi_set_acquire_fence {
        int id;
        uint32_t display_id;
        int acquire_fence_fd;
};

struct drm_evdi_swap_callback {
        int poll_id;
        int release_fence_fd;
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

static inline void evdi_swap_ack(int poll_id, int drm_fd, int release_fence_fd)
{
        struct drm_evdi_swap_callback cmd;
        int rc;

        cmd.poll_id = poll_id;
        cmd.release_fence_fd = release_fence_fd;
        rc = ioctl(drm_fd, DRM_IOCTL_EVDI_SWAP_CALLBACK, &cmd);
        (void)rc;

        if (release_fence_fd >= 0)
                close(release_fence_fd);
}

static std::atomic<bool> g_present_inflight[kMaxDriverDisplays];
static std::atomic<bool> g_need_present[kMaxDriverDisplays];

struct pending_swap {
    bool valid = false;
    int id = 0;
    int poll_id = 0;
    int acquire_fence_fd = -1;
};

static std::thread g_present_thread;
static int g_present_wake_efd = -1;
static int g_shutdown_efd = -1;
static std::mutex g_present_mu[kMaxDriverDisplays];
static pending_swap g_pending[kMaxDriverDisplays];

/* Serialize HWC / display state changes vs present thread. */
static std::mutex g_hwc_mu;

static inline void present_worker_wake()
{
    if (g_present_wake_efd < 0)
        return;
    uint64_t one = 1;
    ssize_t rc = write(g_present_wake_efd, &one, sizeof(one));
    (void)rc;
}

native_handle_t* get_handle(int id);

static int wait_fence_interruptible(int fence_fd, int interrupt_fd) {
    if (fence_fd < 0)
        return 0;

    struct pollfd fds[2] = {
        { .fd = fence_fd,      .events = POLLIN },
        { .fd = interrupt_fd,  .events = POLLIN }
    };

    int ret = poll(fds, 2, 1000);

    if (ret < 0)
        return -errno;
    if (ret == 0)
        return -ETIMEDOUT;
    if (fds[1].revents & (POLLIN | POLLHUP | POLLERR))
        return -EINTR;

    return 0;
}

static int present_one(int drm_fd, int drv_display_id, int id, int poll_id, int acquire_fence_fd)
{
    std::lock_guard<std::mutex> hwc_lk(g_hwc_mu);

    if (unlikely(drv_display_id < 0 || drv_display_id >= kMaxDriverDisplays)) {
        if (acquire_fence_fd >= 0)
            close(acquire_fence_fd);
        evdi_swap_ack(poll_id, drm_fd, -1);
        return 0;
    }

    buffer_handle_t in_handle = get_handle(id);
    if (unlikely(in_handle == nullptr)) {
        if (acquire_fence_fd >= 0)
            close(acquire_fence_fd);
        g_present_inflight[drv_display_id].store(false, std::memory_order_release);
        evdi_swap_ack(poll_id, drm_fd, -1);
        return 0;
    }

    Display& D = get_or_create_display(drv_display_id);
    if (unlikely(!D.hwcDisplay || D.width == 0 || D.height == 0)) {
        if (acquire_fence_fd >= 0)
            close(acquire_fence_fd);
        g_present_inflight[drv_display_id].store(false, std::memory_order_release);
        evdi_swap_ack(poll_id, drm_fd, -1);
        return 0;
    }

    RemoteWindowBuffer *buf = nullptr;
    auto it_buf = buffers_map.find(id);
    if (it_buf == buffers_map.end()) {
        void* mem = g_rwb_pool.acquire();
        RemoteWindowBuffer* rb = new (mem) RemoteWindowBuffer(
            D.width, D.height, D.stride,
            HAL_PIXEL_FORMAT_RGBA_8888,
            kRwbUsage, in_handle);
        it_buf = buffers_map.emplace(id, UniqueRwb(rb)).first;
    }

    buf = it_buf->second.get();
    if (unlikely(buf->width != D.width || buf->height != D.height || buf->stride != D.stride)) {
        buf->~RemoteWindowBuffer();
        new (buf) RemoteWindowBuffer(
            D.width, D.height, D.stride,
            HAL_PIXEL_FORMAT_RGBA_8888,
            kRwbUsage, in_handle);
    }

    if (acquire_fence_fd >= 0) {
        struct drm_evdi_set_acquire_fence fence_cmd;
        fence_cmd.id = id;
        fence_cmd.display_id = (uint32_t)drv_display_id;
        fence_cmd.acquire_fence_fd = acquire_fence_fd;
        (void)ioctl(drm_fd, DRM_IOCTL_EVDI_SET_ACQUIRE_FENCE, &fence_cmd);
        int ret = wait_fence_interruptible(acquire_fence_fd, g_shutdown_efd);
        if (ret == -EINTR || !g_running.load(std::memory_order_acquire)) {
            if (acquire_fence_fd >= 0)
                close(acquire_fence_fd);
            g_present_inflight[drv_display_id].store(false, std::memory_order_release);
            evdi_swap_ack(poll_id, drm_fd, -1);
            return -EINTR; 
        }
    }

    int presentFence = -1;
    uint32_t numTypes = 0;
    uint32_t numRequests = 0;

    hwc2_error_t error = hwc2_compat_display_validate(D.hwcDisplay, &numTypes, &numRequests);

    if (error == HWC2_ERROR_HAS_CHANGES)
        error = hwc2_compat_display_accept_changes(D.hwcDisplay);

    if (error != HWC2_ERROR_NONE) {
        if (acquire_fence_fd >= 0)
            close(acquire_fence_fd);
        g_present_inflight[drv_display_id].store(false, std::memory_order_release);
        return 0;
    }
    hwc2_compat_display_set_client_target(D.hwcDisplay, /* slot */0, buf,
                                          acquire_fence_fd, HAL_DATASPACE_UNKNOWN);
    // now owned by hwc
    acquire_fence_fd = -1;

    error = hwc2_compat_display_present(D.hwcDisplay, &presentFence);
    if (error != HWC2_ERROR_NONE) {
        if (presentFence >= 0)
            close(presentFence);
        if (acquire_fence_fd >= 0)
            close(acquire_fence_fd);
        g_present_inflight[drv_display_id].store(false, std::memory_order_release);
        evdi_swap_ack(poll_id, drm_fd, -1);
        return 0;
    }
    if (presentFence >= 0) {
        int pret = wait_fence_interruptible(presentFence, g_shutdown_efd);
        if (pret == -EINTR || !g_running.load(std::memory_order_acquire)) {
            close(presentFence);
            g_present_inflight[drv_display_id].store(false, std::memory_order_release);
            evdi_swap_ack(poll_id, drm_fd, -1);
            return -EINTR;
        }
        close(presentFence);
        presentFence = -1;
    }
    evdi_swap_ack(poll_id, drm_fd, -1);
    g_present_inflight[drv_display_id].store(false, std::memory_order_release);
    return 0;
}

static void present_worker_main()
{
    struct pollfd pfd;
    std::memset(&pfd, 0, sizeof(pfd));
    pfd.fd = g_present_wake_efd;
    pfd.events = POLLIN;

    while (g_running.load(std::memory_order_acquire)) {
        int rc = poll(&pfd, 1, 500 /* ms */);
        if (rc <= 0)
            continue;

        uint64_t v;
        while (read(g_present_wake_efd, &v, sizeof(v)) == (ssize_t)sizeof(v)) {}

        for (int d = 0; d < kMaxDriverDisplays; d++) {
            pending_swap ps;
            {
                std::lock_guard<std::mutex> lk(g_present_mu[d]);
                if (!g_pending[d].valid)
                    continue;
                ps = g_pending[d];
                g_pending[d].valid = false;
            }
            if (g_present_inflight[d].load(std::memory_order_acquire)) {
                std::lock_guard<std::mutex> lk(g_present_mu[d]);
                if (!g_pending[d].valid) {
                    g_pending[d] = ps;
                    g_pending[d].valid = true;
                } else {
                    if (ps.acquire_fence_fd >= 0)
                        close(ps.acquire_fence_fd);
                    evdi_swap_ack(ps.poll_id, drm_fd, -1);
                }
                continue;
            }

            g_present_inflight[d].store(true, std::memory_order_release);
            g_need_present[d].store(false, std::memory_order_release);
            if (present_one(drm_fd, d, ps.id, ps.poll_id, ps.acquire_fence_fd) == -EINTR)
                break;
        }
    }
}

int add_handle(const native_handle_t& handle) {
    const size_t total_size = sizeof(native_handle_t) + (handle.numFds + handle.numInts) * sizeof(int);
    native_handle_t* copied_handle = (native_handle_t*)malloc(total_size);
    if (!copied_handle) {
        printf("Memory allocation failed for handle copy\n");
        return -1;
    }
    memcpy(copied_handle, &handle, total_size);

    if (next_id <= 0)
        next_id = 1;

    int id = next_id++;
    handles_map[id] = std::unique_ptr<native_handle_t>(copied_handle);
    return id;
}

native_handle_t* get_handle(int id) {
    auto it = handles_map.find(id);
    return (it != handles_map.end()) ? it->second.get() : nullptr;
}

static inline void init_free_driver_slots_once() {
    if (!g_free_drv_ids.empty()) return;
    g_free_drv_ids.reserve(kMaxDriverDisplays);
    for (int i = kMaxDriverDisplays - 1; i >= 0; --i) {
        g_free_drv_ids.push_back(i);
    }
}

static int drm_auth_magic(int fd, drm_magic_t magic) {
    drm_auth_t auth{};
    auth.magic = magic;
    if (ioctl(fd, DRM_IOCTL_AUTH_MAGIC, &auth)) {
        return -errno;
    }
    return 0;
}

static bool drm_is_master(int fd) {
    return drm_auth_magic(fd, 0) != -EACCES;
}

bool is_evdi_lindroid(int fd) {
    drmVersionPtr version = drmGetVersion(fd);
    if (version) {
        std::string driver_name(version->name, version->name_len);
        drmFreeVersion(version);
        return (driver_name == "evdi-lindroid");
    }
    return false;
}

int find_evdi_lindroid_device() {
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

    for (const auto& path : candidates) {
        int fd = open(path.c_str(), O_RDWR | O_CLOEXEC);
        if (fd < 0) continue;

        if (is_evdi_lindroid(fd)) {
            std::cout << "Found evdi-lindroid at " << path << std::endl;

            if (drmIsMaster(fd)) {
                if (ioctl(fd, DRM_IOCTL_DROP_MASTER, nullptr) < 0) {
                    std::cerr << "Failed to drop master on " << path << ": " << strerror(errno) << std::endl;
                    close(fd);
                    return -1;
                }
            }

            return fd;
        }

        close(fd);
    }

    return -1;
}

int open_evdi_lindroid_or_create() {
    int fd = find_evdi_lindroid_device();
    if (fd >= 0) {
        return fd;
    }

    //try to create device
    std::cout << "evdi-lindroid not found. Attempting to create..." << std::endl;
    std::ofstream evdi_add("/sys/devices/evdi-lindroid/add");
    if (!evdi_add) {
        std::cerr << "Failed to write to /sys/devices/evdi-lindroid/add: " << strerror(errno) << std::endl;
        return -1;
    }

    evdi_add << "1";
    evdi_add.close();

    int wait_interval = 1; // interval between evdi device check
    int total_wait_limit = 30; // total wait time limit for evdi device check
    for (int wait_time = 0; wait_time < total_wait_limit; wait_time += wait_interval) {
        fd = find_evdi_lindroid_device();
        if (fd >= 0) {
            return fd;
        }
        sleep(wait_interval);
    }

    std::cerr << "evdi-lindroid still not available after add attempt." << std::endl;
    return -1;
}

static inline int evdi_connect(int fd, int device_index,
                               uint32_t width, uint32_t height,
                               uint32_t refresh_rate, uint32_t display_id,
                               int connected) {
    drm_evdi_connect cmd = {
        .connected = connected,
        .dev_index = device_index,
        .width = width,
        .height = height,
        .refresh_rate = refresh_rate,
        .display_id = display_id,
    };

    if (ioctl(fd, DRM_IOCTL_EVDI_CONNECT, &cmd) < 0) {
        perror("DRM_IOCTL_EVDI_CONNECT failed");
        return -1;
    }

    return 0;
}

int update_display(int display_id);

void onVsyncReceived(HWC2EventListener* listener, int32_t sequenceId,
                     hwc2_display_t display, int64_t timestamp)
{
    const long long hwc_id = (long long)display;
    const int drv_id = drv_id_for_hwc(hwc_id);
    if (drv_id >= 0 && drv_id < kMaxDriverDisplays) {
        Display& D = get_or_create_display(drv_id);
        D.seen_vsync.store(true, std::memory_order_release);
        if (g_need_present[drv_id].load(std::memory_order_acquire) &&
            !g_present_inflight[drv_id].load(std::memory_order_acquire)) {
            present_worker_wake();
        }
    }
}

static inline int drv_id_for_hwc(long long hwc_id) {
    auto it = g_hwc_to_drv.find(hwc_id);
    return it == g_hwc_to_drv.end() ? -1 : it->second;
}

static inline int alloc_driver_slot_for_hwc(long long hwc_id) {
    int drv = drv_id_for_hwc(hwc_id);
    if (drv >= 0) return drv;
    if (g_free_drv_ids.empty()) return -1;
    drv = g_free_drv_ids.back();
    g_free_drv_ids.pop_back();
    g_hwc_to_drv[hwc_id] = drv;
    g_drv_to_hwc[drv] = hwc_id;
    return drv;
}

static inline void release_driver_slot_for_hwc(long long hwc_id) {
    auto it = g_hwc_to_drv.find(hwc_id);
    if (it == g_hwc_to_drv.end()) return;
    int drv = it->second;
    g_hwc_to_drv.erase(it);
    g_drv_to_hwc.erase(drv);
    g_free_drv_ids.push_back(drv);
}

void onHotplugReceived(HWC2EventListener* listener, int32_t sequenceId,
                       hwc2_display_t display, bool connected,
                       bool primaryDisplay)
{
        printf("onHotplugReceived(%d, %" PRIu64 ", %s, %s)\n",
               sequenceId, display,
               connected ? "connected" : "disconnected",
               primaryDisplay ? "primary" : "external");

        hwc2_compat_device_on_hotplug(hwcDevice, display, connected);
        init_free_driver_slots_once();

        const long long hwc_id = (long long)display;
        int drv_id = drv_id_for_hwc(hwc_id);

        if (connected) {
            if (drv_id < 0) {
                drv_id = alloc_driver_slot_for_hwc(hwc_id);
                if (drv_id < 0) {
                    std::cerr << "No free driver display slots; ignoring hotplug for HWC id " << hwc_id << std::endl;
                    return;
                }
            }
            Display& D = get_or_create_display(drv_id);
            D.display_id = drv_id;
            D.hwcDisplay = hwc2_compat_device_get_display_by_id(hwcDevice, display);
            if (!D.hwcDisplay) {
                std::cerr << "HWC display handle not available for id " << hwc_id << std::endl;
                return;
            }
            hwc2_compat_display_set_power_mode(D.hwcDisplay, HWC2_POWER_MODE_ON);
            if (drm_ready && drm_fd >= 0) {
                update_display(drv_id);
            } else {
                printf("Deferring CONNECT for driver slot %d (HWC %" PRIu64 ")\n",
                       drv_id, (uint64_t)hwc_id);
            }
        } else {
            if (drv_id < 0) return;
            Display& D = get_or_create_display(drv_id);
            evdi_connect(drm_fd, 0, 0, 0, 0, (uint32_t)drv_id, 0);
            if (D.layer && D.hwcDisplay)
                hwc2_compat_display_destroy_layer(D.hwcDisplay, D.layer);
            D.layer = nullptr;
            D.hwcDisplay = nullptr;
            D.width = D.height = 0;
            D.stride = 0;
            release_driver_slot_for_hwc(hwc_id);
        }
}

void onRefreshReceived(HWC2EventListener* listener,
                       int32_t sequenceId, hwc2_display_t display)
{
    const long long hwc_id = (long long)display;
    int drv_id = drv_id_for_hwc(hwc_id);
    if (drv_id < 0) return;
    printf("onRefreshReceived (HWC %" PRIu64 ") -> driver slot %d\n", (uint64_t)hwc_id, drv_id);
    Display& D = get_or_create_display(drv_id);
    if (D.hwcDisplay)
        update_display(drv_id);
}

HWC2EventListener eventListener = {
    &onVsyncReceived,
    &onHotplugReceived,
    &onRefreshReceived
};

void add_buf_to_map(void *data, int poll_id, int drm_fd) {
    int fd;
    native_handle_t handle;
    int id = -1;
    memcpy(&fd, data, sizeof(int));
    if (fcntl(fd, F_GETFD) == -1) {
        printf("Invalid or closed file descriptor: %d\n", fd);
        return;
    }
    int header[3];
    if (pread(fd, header, sizeof(header), 0) != (ssize_t)sizeof(header)) {
        printf("Fd1 read failed fd: %d\n", fd);
        return;
    }
    int version = header[0];
    int numFds = header[1];
    int numInts = header[2];

    size_t total_size = sizeof(buffer_handle_t) + ((size_t)numFds + (size_t)numInts) * sizeof(int);
    void *blk = g_small_pool.alloc(total_size);
    native_handle_t *full_handle = (native_handle_t*)blk;
    if (!full_handle) {
        printf("malloc failed size: %zu\n", total_size);
        return;
    }
    std::memcpy(full_handle, header, sizeof(header));
    const size_t already = sizeof(header);
    const size_t remain = (total_size > already) ? (total_size - already) : 0;
    if (remain) {
        if (pread(fd, (char*)full_handle + already, remain, (off_t)already) != (ssize_t)remain) {
            g_small_pool.dealloc(blk, total_size);
            printf("Fd read failed fd: %d", fd);
            return;
        }
    }
    {
        const uint64_t h = make_handle_hash(full_handle);
        auto it = handle_index.find(h);
        if (it != handle_index.end()) {
            for (int cand_id : it->second) {
                native_handle_t *cand = get_handle(cand_id);
                if (handles_equal(full_handle, cand)) {
                    id = cand_id;
                    break;
                }
            }
        }
        if (id == -1) {
            id = add_handle(*full_handle);
            handle_hash_by_id[id] = h;
            auto &vec = handle_index[h];
            if (vec.empty()) vec.reserve(4);
            vec.push_back(id);
        }
    }

    close(fd);
    g_small_pool.dealloc(full_handle, total_size);

    struct drm_evdi_add_buff_callabck cmd = {.poll_id=poll_id, .buff_id=id};
    ioctl(drm_fd, DRM_IOCTL_EVDI_ADD_BUFF_CALLBACK, &cmd);
}

void get_buf_from_map(void *data, int poll_id, int drm_fd) {
    int id;
    struct drm_evdi_get_buff_callabck cmd;
    memcpy(&id, data, sizeof(int));

    buffer_handle_t handle = get_handle(id);
    if(!handle) {
        cmd = {.poll_id = poll_id, .version = -1, .numFds = -1, .numInts = -1, .fd_ints = nullptr, .data_ints = nullptr};
    } else {
        cmd = {.poll_id = poll_id, .version = handle->version, .numFds = handle->numFds, .numInts = handle->numInts, .fd_ints = const_cast<int *>(&handle->data[0]), .data_ints = const_cast<int *>(&handle->data[handle->numFds])};
    }
//    printf("get_buf_from_map id: %d, version: %d\n", id, handle->version);
    ioctl(drm_fd, DRM_IOCTL_EVDI_GET_BUFF_CALLBACK, &cmd);
}

void swap_to_buff(void *data, int poll_id, int drm_fd) {
    drm_evdi_swap_event ex = { -1, 0, -1 };
    memcpy(&ex, data, sizeof(ex));
    const int id = ex.id;
    const int drv_display_id = ex.display_id;
    const int acquire_fence_fd = ex.acquire_fence_fd;

    if (unlikely(drv_display_id < 0 || drv_display_id >= kMaxDriverDisplays)) {
        if (acquire_fence_fd >= 0)
            close(acquire_fence_fd);
        evdi_swap_ack(poll_id, drm_fd, -1);
        return;
    }

    {
        std::lock_guard<std::mutex> lk(g_present_mu[drv_display_id]);
        if (g_pending[drv_display_id].valid) {
            if (g_pending[drv_display_id].acquire_fence_fd >= 0)
                close(g_pending[drv_display_id].acquire_fence_fd);
            evdi_swap_ack(g_pending[drv_display_id].poll_id, drm_fd, -1);
        }
        g_pending[drv_display_id].valid = true;
        g_pending[drv_display_id].id = id;
        g_pending[drv_display_id].poll_id = poll_id;
        g_pending[drv_display_id].acquire_fence_fd = acquire_fence_fd;
    }
    g_need_present[drv_display_id].store(true, std::memory_order_release);
    Display& D = get_or_create_display(drv_display_id);
    if (!D.seen_vsync.load(std::memory_order_acquire))
        present_worker_wake();
}

void destroy_buff(void *data, int poll_id, int drm_fd) {
        int id = *(int *)data;
        int ret;
        native_handle_t *handle = get_handle(id);
        if(handle) {
                native_handle_close(handle);
        }
        auto it_hh = handle_hash_by_id.find(id);
        if (it_hh != handle_hash_by_id.end()) {
                const uint64_t hh = it_hh->second;
                auto it_vec = handle_index.find(hh);
                if (it_vec != handle_index.end()) {
                        auto &vec = it_vec->second;
                        for (size_t i = 0; i < vec.size(); ++i) {
                                if (vec[i] == id) {
                                        vec[i] = vec.back();
                                        vec.pop_back();
                                        break;
                                }
                        }
                        if (vec.empty()) {
                                handle_index.erase(it_vec);
                        }
                }
                handle_hash_by_id.erase(it_hh);
        }
	buffers_map.erase(id);
        handles_map.erase(id);
        struct drm_evdi_destroy_buff_callback cmd = {.poll_id=poll_id};
        ret=ioctl(drm_fd, DRM_IOCTL_EVDI_DESTROY_BUFF_CALLBACK, &cmd);
}


void create_buff(void *data, int poll_id, int drm_fd) {
//printf("Hi from create_buff\n");
    struct drm_evdi_gbm_create_buff buff_params;
    struct drm_evdi_create_buff_callabck cmd;
    memcpy(&buff_params, data, sizeof(struct drm_evdi_gbm_create_buff));
    const native_handle_t *full_handle;
    int ret = hybris_gralloc_allocate(buff_params.width, buff_params.height, HAL_PIXEL_FORMAT_RGBA_8888, GRALLOC_USAGE_HW_TEXTURE | GRALLOC_USAGE_HW_RENDER | GRALLOC_USAGE_HW_COMPOSER, &full_handle, &cmd.stride);
    if (ret != 0) {
        fprintf(stderr, "[libgbm-hybris] hybris_gralloc_allocate failed: %d\n", ret);
    }
    cmd.id = add_handle(*full_handle);
    cmd.poll_id = poll_id;
    ioctl(drm_fd, DRM_IOCTL_EVDI_GBM_CREATE_BUFF_CALLBACK, &cmd);
}

static inline int hz_from_period_ns(int32_t ns)
{
    if (ns <= 0) return 60;
    const double hz_f = 1e9 / static_cast<double>(ns);
    int hz = static_cast<int>(std::lround(hz_f));
    return hz;
}

static inline int get_refresh_hz_from_active_config(const HWC2DisplayConfig* cfg)
{
    return hz_from_period_ns(cfg->vsyncPeriod);
}
int update_display(int display_id) {
    Display& D = get_or_create_display(display_id);
    if (!D.hwcDisplay) return -1;
    HWC2DisplayConfig* config = hwc2_compat_display_get_active_config(D.hwcDisplay);
    if (!config) {
        fprintf(stderr, "update_display(%d): no active HWC config yet, will retry on next refresh\n",
                display_id);
        return -1;
    }

    if (!D.hwcDisplay) {
        long long hwc_id = g_drv_to_hwc[display_id];
        D.hwcDisplay = hwc2_compat_device_get_display_by_id(hwcDevice, (hwc2_display_t)hwc_id);
        if (D.hwcDisplay) hwc2_compat_display_set_power_mode(D.hwcDisplay, HWC2_POWER_MODE_ON);
    }

    if (config->width <= 0 || config->height <= 0) {
        fprintf(stderr, "update_display(%d): invalid geometry %dx%d, deferring\n",
                display_id, config->width, config->height);
        return -1;
    }

    if (!drm_ready || drm_fd < 0) {
        fprintf(stderr, "update_display(%d): DRM not ready, deferring CONNECT\n", display_id);
        return -1;
    }

    printf("display %d width: %i height: %i\n", display_id, config->width, config->height);
    if (D.width != config->width || D.height != config->height) {
        buffers_map.clear();
        D.width = config->width;
        D.height = config->height;
        buffer_handle_t handle = NULL;

        hybris_gralloc_allocate(D.width, D.height, HAL_PIXEL_FORMAT_RGBA_8888,
                                GRALLOC_USAGE_HW_TEXTURE | GRALLOC_USAGE_HW_RENDER | GRALLOC_USAGE_HW_COMPOSER,
                                &handle, &D.stride);

        if (D.layer) {
            hwc2_compat_display_destroy_layer(D.hwcDisplay, D.layer);
            D.layer = nullptr;
        }
        D.layer = hwc2_compat_display_create_layer(D.hwcDisplay);

        hwc2_compat_layer_set_composition_type(D.layer, HWC2_COMPOSITION_CLIENT);
        hwc2_compat_layer_set_blend_mode(D.layer, HWC2_BLEND_MODE_NONE);
        hwc2_compat_layer_set_source_crop(D.layer, 0.0f, 0.0f, config->width, config->height);
        hwc2_compat_layer_set_display_frame(D.layer, 0, 0, config->width, config->height);
        hwc2_compat_layer_set_visible_region(D.layer, 0, 0, config->width, config->height);
        hwc2_compat_display_set_vsync_enabled(D.hwcDisplay, HWC2_VSYNC_ENABLE);

        int refresh_hz = get_refresh_hz_from_active_config(config);

        std::ostringstream oss;
        oss << "EDID for " << config->width << "x" << config->height
            << "@" << refresh_hz << "Hz 'Lindroid display " << display_id << "'";
        std::cout << oss.str() << std::endl;

        if (evdi_connect(drm_fd, 0,
                         (uint32_t)config->width, (uint32_t)config->height,
                         (uint32_t)refresh_hz, (uint32_t)display_id, 1) < 0) {
            return EXIT_FAILURE;
        }
    }
    return 0;
}

// Dedicated poll thread
static void poll_thread_main()
{
    for (;;) {
        if (!g_running.load(std::memory_order_acquire))
            break;

        drm_evdi_poll poll_cmd;
        // Match EVDI_EVENT_PAYLOAD_MAX
        uint8_t poll_payload[32];
        poll_cmd.data = poll_payload;

        int ret = ioctl(drm_fd, DRM_IOCTL_EVDI_POLL, &poll_cmd);
        if (ret) {
            /* Shutdown should break blocking ioctls */
            if (errno == EINTR &&
                !g_running.load(std::memory_order_acquire))
                break;
            continue;
        }

        switch (poll_cmd.event) {
        case add_buf:
            add_buf_to_map(poll_cmd.data, poll_cmd.poll_id, drm_fd);
            break;
        case get_buf:
            get_buf_from_map(poll_cmd.data, poll_cmd.poll_id, drm_fd);
            break;
        case swap_to:
            swap_to_buff(poll_cmd.data, poll_cmd.poll_id, drm_fd);
            break;
        case destroy_buf:
            destroy_buff(poll_cmd.data, poll_cmd.poll_id, drm_fd);
            break;
        case create_buf:
            create_buff(poll_cmd.data, poll_cmd.poll_id, drm_fd);
            break;
        default:
            break;
        }
    }
}

static void handle_signal(int signo)
{
    (void)signo;
    g_running.store(false, std::memory_order_release);
    present_worker_wake();
}

static void handle_wake_signal(int signo)
{
    (void)signo;
}

static void install_signal_handlers()
{
    struct sigaction sa;
    struct sigaction sa_wake;
    std::memset(&sa, 0, sizeof(sa));
    sa.sa_handler = handle_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    std::memset(&sa_wake, 0, sizeof(sa_wake));
    sa_wake.sa_handler = handle_wake_signal;
    sigemptyset(&sa_wake.sa_mask);
    sigaction(SIGUSR1, &sa_wake, nullptr);
}

int main() {
    int device_index = 0;
    int composerSequenceId = 0;
    int ret =0;

    sd_notifyf(0, "MAINPID=%lu", (unsigned long)getpid());
    sd_notify(0, "STATUS=Initializing create-disp…");

    init_free_driver_slots_once();

    handle_index.max_load_factor(0.5f);
    handle_index.reserve(kExpectedHandles);
    handles_map.reserve(kExpectedHandles);
    buffers_map.reserve(kExpectedHandles);
    g_displays.reserve(kMaxDriverDisplays);
    g_hwc_to_drv.reserve(kMaxDriverDisplays);
    g_drv_to_hwc.reserve(kMaxDriverDisplays);

    for (int i = 0; i < kMaxDriverDisplays; ++i) {
        g_present_inflight[i].store(false, std::memory_order_relaxed);
        g_need_present[i].store(false, std::memory_order_relaxed);
    }

    // Wait up to 5s for evdi; then open
    drm_fd = -1;
    for (int i = 0; i < 5 * 1000; ++i) {
        drm_fd = find_evdi_lindroid_device();
        if (drm_fd >= 0)
            break;
        usleep(1000);
    }
    if (drm_fd < 0) drm_fd = open_evdi_lindroid_or_create();
    if (drm_fd < 0) return EXIT_FAILURE;
    drm_ready = true;

    g_present_wake_efd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
    if (g_present_wake_efd >= 0) {
        try {
            g_present_thread = std::thread(present_worker_main);
        } catch (...) {
            close(g_present_wake_efd);
            g_present_wake_efd = -1;
        }
    }

    hwcDevice = hwc2_compat_device_new(false);
    if (!hwcDevice)
        return EXIT_FAILURE;
    assert(hwcDevice);
    hwc2_compat_device_register_callback(hwcDevice, &eventListener,
                                         composerSequenceId);

    for (const auto& kv : g_hwc_to_drv) {
        int drv_id = kv.second;
        auto it = g_displays.find(drv_id);
        if (it == g_displays.end())
            continue;
        Display& D = it->second;
        if (!D.hwcDisplay) {
            long long hwc_id = g_drv_to_hwc[drv_id];
            D.hwcDisplay = hwc2_compat_device_get_display_by_id(hwcDevice, (hwc2_display_t)hwc_id);
        }
        if (D.hwcDisplay)
            (void)update_display(drv_id);
    }

    install_signal_handlers();

    sd_notify(0, "READY=1");
    sd_notify(0, "STATUS=create-disp ready.");

    // Start poll thread
    try {
        g_poll_thread = std::thread(poll_thread_main);
    } catch (...) {
        fprintf(stderr, "Failed to create poll thread\n");
        close(drm_fd);
        return EXIT_FAILURE;
    }

     // Main thread loop.
    while (g_running.load(std::memory_order_acquire)) {
        pause();
    }

    // Shutdown
    sd_notify(0, "STATUS=Stopping poll thread…");
    if (g_poll_thread.joinable())
        pthread_kill(g_poll_thread.native_handle(), SIGUSR1);
    if (g_poll_thread.joinable())
        g_poll_thread.join();

    present_worker_wake();
    if (g_present_thread.joinable())
        g_present_thread.join();
    if (g_present_wake_efd >= 0)
        close(g_present_wake_efd);
    g_present_wake_efd = -1;

    close(drm_fd);
    sd_notify(0, "STATUS=Shutting down…");
    return EXIT_SUCCESS;
}
