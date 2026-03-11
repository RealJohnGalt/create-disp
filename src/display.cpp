// display.cpp - Display state
#include "display.h"

namespace createdisp {

DisplaySnapshot snapshot_display(const Display& d) {
    DisplaySnapshot s;
    s.hwcDisplay = d.hwcDisplay;
    s.width = d.width;
    s.height = d.height;
    s.stride = d.stride;
    s.connected = d.connected;
    s.generation = d.generation;
    return s;
}

} // namespace createdisp
