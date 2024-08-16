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

#ifndef _OS_TASK_H
#define _OS_TASK_H

#include <nuttx/config.h>

#include <sys/types.h>
#include <pthread.h>
#include <time.h>
#include <errno.h>

#include "os/os.h"
#include "os/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The highest and lowest task priorities */
#define OS_TASK_PRI_HIGHEST (0)
#define OS_TASK_PRI_LOWEST  (0xff)

typedef void (*os_task_func_t)(void *);

#define OS_TASK_MAX_NAME_LEN (32)

struct os_task {
    pthread_t   thread;
};

int os_task_init(struct os_task *, const char *, os_task_func_t, void *,
        uint8_t, os_time_t, os_stack_t *, uint16_t);

int os_task_remove(struct os_task *t);

uint8_t os_task_count(void);



#ifdef __cplusplus
}
#endif

#endif /* _OS_TASK_H */
