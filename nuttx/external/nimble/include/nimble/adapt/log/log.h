/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */
#ifndef __SYS_LOG_FULL_H__
#define __SYS_LOG_FULL_H__

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include "syscfg/syscfg.h"
#include "log/ignore.h"

#include <os/queue.h>

#ifdef __cplusplus
extern "C" {
#endif


#define LOG_LEVEL_DEBUG    (0)
#define LOG_LEVEL_INFO     (1)
#define LOG_LEVEL_WARN     (2)
#define LOG_LEVEL_ERROR    (3)
#define LOG_LEVEL_CRITICAL (4)
/* Up to 7 custom log levels. */
#define LOG_LEVEL_MAX      (8)

#define LOG_NAME_MAX_LEN    (64)

#define LOG_PRINTF_MAX_ENTRY_LEN (128)

extern int vsnprintf(FAR char *buf, size_t size,
                     FAR const IPTR char *format, va_list ap);

static inline void 
ble_log(char *msg, ...)
{
  va_list ap;

  va_start(ap, msg);
  vsyslog(LOG_ERR, msg, ap);
  va_end(ap);

  return;
}

#define BLE_LOG(msg,...) ble_log(msg, ##__VA_ARGS__)

#if MYNEWT_VAL(LOG_LEVEL) <= LOG_LEVEL_DEBUG
#define BLE_LOG_DEBUG(__l, __mod, __msg, ...) ble_log(__msg, ##__VA_ARGS__)
#else
#define BLE_LOG_DEBUG(__l, __mod, ...) IGNORE(__VA_ARGS__)
#endif

#if MYNEWT_VAL(LOG_LEVEL) <= LOG_LEVEL_INFO
#define BLE_LOG_INFO(__l, __mod, __msg, ...) ble_log(__msg, ##__VA_ARGS__)
#else
#define BLE_LOG_INFO(__l, __mod, ...) IGNORE(__VA_ARGS__)
#endif

#if MYNEWT_VAL(LOG_LEVEL) <= LOG_LEVEL_WARN
#define BLE_LOG_WARN(__l, __mod, __msg, ...) ble_log(__msg, ##__VA_ARGS__)
#else
#define BLE_LOG_WARN(__l, __mod, ...) IGNORE(__VA_ARGS__)
#endif

#if MYNEWT_VAL(LOG_LEVEL) <= LOG_LEVEL_ERROR
#define BLE_LOG_ERROR(__l, __mod, __msg, ...) ble_log(__msg, ##__VA_ARGS__)
#else
#define BLE_LOG_ERROR(__l, __mod, ...) IGNORE(__VA_ARGS__)
#endif

#if MYNEWT_VAL(LOG_LEVEL) <= LOG_LEVEL_CRITICAL
#define BLE_LOG_CRITICAL(__l, __mod, __msg, ...) ble_log(__msg, ##__VA_ARGS__)
#else
#define BLE_LOG_CRITICAL(__l, __mod, ...) IGNORE(__VA_ARGS__)
#endif

#ifndef MYNEWT_VAL_LOG_LEVEL
#define LOG_SYSLEVEL    ((uint8_t)0xff)
#else
#define LOG_SYSLEVEL    ((uint8_t)MYNEWT_VAL_LOG_LEVEL)
#endif

struct log {

};


#ifdef __cplusplus
}
#endif

#endif /* __SYS_LOG_FULL_H__ */
