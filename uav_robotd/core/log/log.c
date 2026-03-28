#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "log.h"

static FILE *log_file_handle(void) {
    static int initialized = 0;
    static FILE *fp = NULL;

    if (!initialized) {
        const char *path = getenv("UAV_LOG_FILE");
        if (path == NULL || path[0] == '\0') {
            path = "/tmp/uav_robotd.log";
        }
        fp = fopen(path, "a");
        initialized = 1;
    }
    return fp;
}

static void log_v(const char *level, const char *tag, const char *fmt, va_list ap) {
    time_t now = time(NULL);
    struct tm tm_now;
#if defined(_WIN32)
    localtime_s(&tm_now, &now);
#else
    struct tm *tm_ptr = localtime(&now);
    if (tm_ptr != NULL) {
        tm_now = *tm_ptr;
    } else {
        memset(&tm_now, 0, sizeof(tm_now));
    }
#endif
    char ts[32];
    char msg[512];
    FILE *fp;

    strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &tm_now);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    fprintf(stdout, "%s [%s] [%s] %s\n", ts, level, tag, msg);
    fflush(stdout);

    fp = log_file_handle();
    if (fp != NULL) {
        fprintf(fp, "%s [%s] [%s] %s\n", ts, level, tag, msg);
        fflush(fp);
    }
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
