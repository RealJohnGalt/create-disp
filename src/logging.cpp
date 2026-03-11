// logging.cpp
#include "logging.h"
#include <cstdarg>
#include <ctime>
#include <mutex>

namespace createdisp {

static LogLevel g_log_level = LogLevel::Error;
static std::mutex g_log_mutex;

void set_log_level(LogLevel level) {
    g_log_level = level;
}

static void log_message(LogLevel level, const char* level_str, const char* fmt, va_list args) {
    if (level < g_log_level) return;

    std::lock_guard<std::mutex> lk(g_log_mutex);

    // Timestamp
    time_t now = time(nullptr);
    char timebuf[32];
    strftime(timebuf, sizeof(timebuf), "%Y-%m-%d %H:%M:%S", localtime(&now));

    fprintf(stderr, "[%s] [%s] ", timebuf, level_str);
    vfprintf(stderr, fmt, args);
    fprintf(stderr, "\n");
}

void log_debug(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    log_message(LogLevel::Debug, "DEBUG", fmt, args);
    va_end(args);
}

void log_info(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    log_message(LogLevel::Info, "INFO", fmt, args);
    va_end(args);
}

void log_warning(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    log_message(LogLevel::Warning, "WARN", fmt, args);
    va_end(args);
}

void log_error(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    log_message(LogLevel::Error, "ERROR", fmt, args);
    va_end(args);
}

} // namespace createdisp
