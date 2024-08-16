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

#include "sysinit/sysinit.h"
#include "os/os.h"
#include "os/queue.h"
#include "os/os_dev.h"
#include "os/os_trace_api.h"
#include "os_priv.h"

#include "hal/hal_os_tick.h"
#include "hal/hal_bsp.h"
#include "hal/hal_watchdog.h"

#include <assert.h>

/**
 * @defgroup OSKernel Operating System Kernel
 * @brief This section contains documentation for the core operating system kernel
 * of Apache Mynewt.
 * @{
 *   @addtogroup OSGeneral General Functions
 *   @{
 */

struct os_task g_idle_task;
os_stack_t g_idle_task_stack[OS_STACK_ALIGN(OS_IDLE_STACK_SIZE)];

uint32_t g_os_idle_ctr;

//static struct os_task os_main_task;
//static os_stack_t os_main_stack[OS_STACK_ALIGN(OS_MAIN_STACK_SIZE)];

/* Default zero.  Set by the architecture specific code when os is started.
 */
int g_os_started;

#ifdef ARCH_sim
#define MIN_IDLE_TICKS  1
#else
#define MIN_IDLE_TICKS  (100 * OS_TICKS_PER_SEC / 1000) /* 100 msec */
#endif
#define MAX_IDLE_TICKS  (600 * OS_TICKS_PER_SEC)        /* 10 minutes */


/**
 * Idle operating system task, runs when no other tasks are running.
 * The idle task operates in tickless mode, which means it looks for
 * the next time an event in the system needs to run, and then tells
 * the architecture specific functions to sleep until that time.
 *
 * @param arg unused
 */


/**
 * Has the operating system started.
 *
 * @return 1 if the operating system has started, 0 if it hasn't
 */
int
os_started(void)
{
    return 1;
}

/**
 * os sched get current task
 *
 * Returns the currently running task. Note that this task may or may not be
 * the highest priority task ready to run.
 *
 *
 * @return struct os_task*
 */
struct os_task cur_task = {0};
struct os_task *
os_sched_get_current_task(void)
{
    uint16_t pid;
    uint32_t ret;
    pid = (uint16_t)getpid();
    ret = 0x0000ffff & pid;
    return (struct os_task *)ret;
}

/**
 *   }@ General OS functions
 * }@ OS Kernel
 */
