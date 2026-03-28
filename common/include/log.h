#ifndef UAV_LOG_H
#define UAV_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

void log_info(const char *tag, const char *fmt, ...);
void log_warn(const char *tag, const char *fmt, ...);
void log_error(const char *tag, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif
