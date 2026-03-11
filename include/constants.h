// constants.h - System-wide constants
#pragma once

#include <cstdint>
#include <csignal>
#include <hybris/hwc2/hwc2_compatibility_layer.h>
#include <hybris/gralloc/gralloc.h>
#include <hybris/platforms/common/windowbuffer.h>

namespace createdisp {

// Display limits
constexpr int kMaxDriverDisplays = 5;
constexpr int kDefaultRefreshHz = 60;

// Buffer management
constexpr size_t kExpectedHandles = 4096;
constexpr uint32_t kSlotCapacity = 32;

// Gralloc usage flags
constexpr int kRwbUsage =
    GRALLOC_USAGE_HW_TEXTURE |
    GRALLOC_USAGE_HW_RENDER |
    GRALLOC_USAGE_HW_COMPOSER;

// Event queue
constexpr size_t kEventQueueCapacity = 256;

// Timing
constexpr int kDeviceWaitInterval = 1;   // seconds
constexpr int kDeviceWaitLimit = 30;     // seconds

// Signals
constexpr int kShutdownKickSignal = SIGUSR1;

// Logging
enum class LogLevel {
    Debug,
    Info,
    Warning,
    Error
};

} // namespace createdisp
