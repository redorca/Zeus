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

#include <nuttx/config.h>
#include <sys/types.h>
#include <pthread.h>
#include <sched.h>

#include "os/os.h"
#include "os_priv.h"
#include "os/os_trace_api.h"

#include <assert.h>
#include <string.h>

/**
 * @addtogroup OSKernel
 * @{
 *   @defgroup OSTask Tasks
 *   @{
 */

uint8_t g_task_id;

#if 0
static void
_clear_stack(os_stack_t *stack_bottom, int size)
{
    int i;

    for (i = 0; i < size; i++) {
        stack_bottom[i] = OS_STACK_PATTERN;
    }
}
#endif

static inline uint8_t
os_task_next_id(void)
{
    uint8_t rc;
    os_sr_t sr;

    OS_ENTER_CRITICAL(sr);
    rc = g_task_id;
    g_task_id++;
    OS_EXIT_CRITICAL(sr);

    return (rc);
}

/**
 * Return the number of tasks initialized.
 *
 * @return number of tasks initialized
 */
uint8_t
os_task_count(void)
{
    return (g_task_id);
}

/**
 * Initialize a task.
 *
 * This function initializes the task structure pointed to by t,
 * clearing and setting it's stack pointer, provides sane defaults
 * and sets the task as ready to run, and inserts it into the operating
 * system scheduler.
 *
 * @param t The task to initialize
 * @param name The name of the task to initialize
 * @param func The task function to call
 * @param arg The argument to pass to this task function
 * @param prio The priority at which to run this task
 * @param sanity_itvl The time at which this task should check in with the
 *                    sanity task.  OS_WAIT_FOREVER means never check in
 *                    here.
 * @param stack_bottom A pointer to the bottom of a task's stack
 * @param stack_size The overall size of the task's stack.
 *
 * @return 0 on success, non-zero on failure.
 */
int
os_task_init(struct os_task *t, const char *name, os_task_func_t func,
        void *arg, uint8_t prio, os_time_t sanity_itvl,
        os_stack_t *stack_bottom, uint16_t stack_size)
{
    int rc;
    pthread_attr_t attr_thread;
    struct sched_param    priority;
    uint8_t nuttx_prio;

    /*Task init paramters come from Mynewt, we need convert the priotity nuttx*/
    nuttx_prio = (SCHED_PRIORITY_MAX - prio);
    if (nuttx_prio < 1) {
        nuttx_prio = 1;
    }

    rc = pthread_attr_init(&attr_thread);

    /* the stack_bottom can't be used in pthread api , so all the caller should
     * NOT pass themselves stack point , but only stack size
     */
    rc |= pthread_attr_setschedpolicy(&attr_thread, SCHED_RR);
    rc |= pthread_attr_setstacksize(&attr_thread, stack_size);

    priority.sched_priority = nuttx_prio;
    rc |= pthread_attr_setschedparam(&attr_thread, &priority);

    rc |= pthread_create(&t->thread, &attr_thread, (pthread_startroutine_t)func, arg);

    return rc;

}

/*
 * Removes specified task
 * XXX
 * NOTE: This interface is currently experimental and not ready for common use
 */
int
os_task_remove(struct os_task *t)
{
    int rc = 0;

    return rc;
}

/**
 * Iterate through tasks, and return the following information about them:
 *
 * - Priority
 * - Task ID
 * - State (ACTIVE, SLEEP)
 * - Total Stack Usage
 * - Stack Size
 * - Context Switch Count
 * - Runtime
 * - Last & Next Sanity checkin
 * - Task Name
 *
 * To get the first task in the list, call os_task_info_get_next() with a
 * NULL pointer in the prev argument, and os_task_info_get_next() will
 * return a pointer to the task structure, and fill out the os_task_info
 * structure pointed to by oti.
 *
 * To get the next task in the list, provide the task structure returned
 * by the previous call to os_task_info_get_next(), and os_task_info_get_next()
 * will fill out the task structure pointed to by oti again, and return
 * the next task in the list.
 *
 * @param prev The previous task returned by os_task_info_get_next(), or NULL
 *             to begin iteration.
 * @param oti  The OS task info structure to fill out.
 *
 * @return A pointer to the OS task that has been read, or NULL when finished
 *         iterating through all tasks.
 */


/**
 *   @} OSTask
 * @} OSKernel
 */
