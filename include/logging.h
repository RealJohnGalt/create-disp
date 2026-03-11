// logging.h
#pragma once

#include <cstdio>
#include "constants.h"

namespace createdisp {

void set_log_level(LogLevel level);

void log_debug(const char* fmt, ...);
void log_info(const char* fmt, ...);
void log_warning(const char* fmt, ...);
void log_error(const char* fmt, ...);

} // namespace createdisp
