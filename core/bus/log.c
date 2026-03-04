#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include "log.h"

static void log_v(const char *level, const char *tag, const char *fmt, va_list ap) {
    time_t now = time(NULL);
    struct tm tm_now;
#if defined(_WIN32)
    localtime_s(&tm_now, &now);
#else
    localtime_r(&now, &tm_now);
#endif
    char ts[32];
    strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &tm_now);
    fprintf(stdout, "%s [%s] [%s] ", ts, level, tag);
    vfprintf(stdout, fmt, ap);
    fputc('\n', stdout);
}

void log_info(const char *tag, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    log_v("INFO", tag, fmt, ap);
    va_end(ap);
}

void log_warn(const char *tag, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    log_v("WARN", tag, fmt, ap);
    va_end(ap);
}

void log_error(const char *tag, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    log_v("ERROR", tag, fmt, ap);
    va_end(ap);
}
