/**
 * @file psl_log.h
 * @brief Uniform serial logging macros for PicoShakeLog
 *
 * Every log line follows the same fixed-width layout so that the colon is always
 * in the same column:
 *
 *     [LEVEL] SCOPE  : message
 *
 * LEVEL (5 chars) : DEBUG, INFO, OK, WARN, ERROR, FATAL
 * SCOPE (6 chars) : BOOT, INIT, STATE, IMU, DMA, FIFO, WIFI, TCP, CORE0, CORE1
 *
 * Usage:
 *     LOG_I(WIFI, "DNS active");
 *     LOG_E(IMU, "WHO_AM_I mismatch: 0x%02X", id);
 *
 * The scope is given as a bare token (stringified by the macro). The prefix and the
 * message are emitted by a single printf() call so lines from both cores do not
 * interleave. The format must be a string literal. A trailing '\n' is added by the macro.
 *
 * The minimum level compiled in is selected with LOG_LEVEL (default: LOG_LEVEL_INFO),
 * e.g. -DLOG_LEVEL=LOG_LEVEL_DEBUG to enable DEBUG output (including the Core 1
 * diagnostics). Disabled levels cost nothing at run time.
 *
 * @author Frédéric Marçon
 * @date 2026
 * @copyright Academic Free License v3.0
 */

#ifndef PSL_LOG_H
#define PSL_LOG_H

#include <stdio.h>

#define LOG_LEVEL_DEBUG 0
#define LOG_LEVEL_INFO  1 /* INFO and OK */
#define LOG_LEVEL_WARN  2
#define LOG_LEVEL_ERROR 3 /* ERROR and FATAL */
#define LOG_LEVEL_NONE  4

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_INFO
#endif

#define LOG_EMIT(level, scope, fmt, ...)                                                           \
    printf("[%-5s] %-6s : " fmt "\n", level, #scope, ##__VA_ARGS__)

#if LOG_LEVEL <= LOG_LEVEL_DEBUG
#define LOG_D(scope, fmt, ...) LOG_EMIT("DEBUG", scope, fmt, ##__VA_ARGS__)
#else
#define LOG_D(scope, fmt, ...) ((void)0)
#endif

#if LOG_LEVEL <= LOG_LEVEL_INFO
#define LOG_I(scope, fmt, ...)  LOG_EMIT("INFO", scope, fmt, ##__VA_ARGS__)
#define LOG_OK(scope, fmt, ...) LOG_EMIT("OK", scope, fmt, ##__VA_ARGS__)
#else
#define LOG_I(scope, fmt, ...)  ((void)0)
#define LOG_OK(scope, fmt, ...) ((void)0)
#endif

#if LOG_LEVEL <= LOG_LEVEL_WARN
#define LOG_W(scope, fmt, ...) LOG_EMIT("WARN", scope, fmt, ##__VA_ARGS__)
#else
#define LOG_W(scope, fmt, ...) ((void)0)
#endif

#if LOG_LEVEL <= LOG_LEVEL_ERROR
#define LOG_E(scope, fmt, ...) LOG_EMIT("ERROR", scope, fmt, ##__VA_ARGS__)
#define LOG_F(scope, fmt, ...) LOG_EMIT("FATAL", scope, fmt, ##__VA_ARGS__)
#else
#define LOG_E(scope, fmt, ...) ((void)0)
#define LOG_F(scope, fmt, ...) ((void)0)
#endif

#endif /* PSL_LOG_H */
