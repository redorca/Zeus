
#ifndef NRF_LOG_H_
#define NRF_LOG_H_

#include <nuttx/config.h>
#include <stdio.h>
#include <debug.h>

#define NRF_LOG_ERROR(...)                     _err(__VA_ARGS__)
#define NRF_LOG_WARNING(...)                   _warn( __VA_ARGS__)
//#define NRF_LOG_INFO(...)                      NRF_LOG_INTERNAL_INFO( __VA_ARGS__)
#define NRF_LOG_INFO(...)                      _info( __VA_ARGS__)
#define NRF_LOG_DEBUG(...)                     _info( __VA_ARGS__)

/**
 * @brief A macro for logging a formatted string without any prefix or timestamp.
 */
#define NRF_LOG_RAW_INFO(...)                  _warn( __VA_ARGS__)

#endif

