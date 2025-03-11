#ifndef _LOGGER_H
#define _LOGGER_H

#ifdef __cplusplus
extern "C" {
#endif
#include <syslog.h>
#include <stdio.h>

#ifndef DEBUG_STDOUT
#define DEBUG_STDOUT 0
#endif

#define SYS_LOG_DEBUG(fmt, args...)                                              \
    do {                                                                         \
        if (DEBUG_STDOUT) {                                                      \
            printf("[ DEBUG ] %s(): Line %d: " fmt, __func__, __LINE__, ##args); \
        }                                                                        \
    } while (0)

#define SYS_LOG_INFO(fmt, args...)                                             \
    do {                                                                       \
        if (DEBUG_STDOUT)                                                      \
            printf("[ INFO ] %s() Line %d: " fmt, __func__, __LINE__, ##args); \
        syslog(LOG_INFO, "%s Line %d: " fmt, __func__, __LINE__, ##args);      \
    } while (0)

#define SYS_LOG_ERROR(fmt, args...)                                             \
    do {                                                                        \
        if (DEBUG_STDOUT)                                                       \
            printf("[ ERROR ] %s() Line %d: " fmt, __func__, __LINE__, ##args); \
        syslog(LOG_ERR, "%s Line %d: " fmt, __func__, __LINE__, ##args);        \
    } while (0)

#ifdef __cplusplus
}
#endif

#endif

#ifndef _LOGGER_H
#define _LOGGER_H

#ifdef __cplusplus
extern "C" {
#endif
#include <syslog.h>
#include <stdio.h>

#ifndef DEBUG_STDOUT
#define DEBUG_STDOUT 0
#endif

#define SYS_LOG_DEBUG(fmt, args...)                                              \
    do {                                                                         \
        if (DEBUG_STDOUT) {                                                      \
            printf("[ DEBUG ] %s(): Line %d: " fmt, __func__, __LINE__, ##args); \
        }                                                                        \
    } while (0)

#define SYS_LOG_INFO(fmt, args...)                                             \
    do {                                                                       \
        if (DEBUG_STDOUT)                                                      \
            printf("[ INFO ] %s() Line %d: " fmt, __func__, __LINE__, ##args); \
        syslog(LOG_INFO, "%s Line %d: " fmt, __func__, __LINE__, ##args);      \
    } while (0)

#define SYS_LOG_ERROR(fmt, args...)                                             \
    do {                                                                        \
        if (DEBUG_STDOUT)                                                       \
            printf("[ ERROR ] %s() Line %d: " fmt, __func__, __LINE__, ##args); \
        syslog(LOG_ERR, "%s Line %d: " fmt, __func__, __LINE__, ##args);        \
    } while (0)

#ifdef __cplusplus
}
#endif

#endif
