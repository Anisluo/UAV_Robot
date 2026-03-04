#ifndef UAV_LOG_H
#define UAV_LOG_H

void log_info(const char *tag, const char *fmt, ...);
void log_warn(const char *tag, const char *fmt, ...);
void log_error(const char *tag, const char *fmt, ...);

#endif
