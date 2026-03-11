// hwc_callbacks.h - HWC event callbacks
#pragma once

#include <hybris/hwc2/hwc2_compatibility_layer.h>

namespace createdisp {

void onVsyncReceived(HWC2EventListener* listener, int32_t sequenceId,
                     hwc2_display_t display, int64_t timestamp);

void onHotplugReceived(HWC2EventListener* listener, int32_t sequenceId,
                       hwc2_display_t display, bool connected, bool primaryDisplay);

void onRefreshReceived(HWC2EventListener* listener, int32_t sequenceId,
                       hwc2_display_t display);

extern HWC2EventListener g_event_listener;

} // namespace createdisp
