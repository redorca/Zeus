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
#include <limits.h>
#include <time.h>
#include <semaphore.h>
#include <sys/types.h>
#include <nuttx/clock.h>

#include "os/os.h"
#include "os/os_trace_api.h"
#include <assert.h>

/**
 * @addtogroup OSKernel
 * @{
 *   @defgroup OSMutex Mutexes
 *   @{
 */


/**
 * os mutex create
 *
 * Create a mutex and initialize it.
 *
 * @param mu Pointer to mutex
 *
 * @return os_error_t
 *      OS_INVALID_PARM     Mutex passed in was NULL.
 *      OS_OK               no error.
 */
os_error_t
os_mutex_init(struct os_mutex *mu)
{
    assert(NULL != mu);

    if (!mu) {
        return OS_INVALID_PARM;
    }

    sem_init(&mu->owner, 0, 1);

    return OS_OK;
}

/**
 * os mutex release
 *
 * Release a mutex.
 *
 * @param mu Pointer to the mutex to be released
 *
 * @return os_error_t
 *      OS_INVALID_PARM Mutex passed in was NULL.
 *      OS_BAD_MUTEX    Mutex was not granted to current task (not owner).
 *      OS_OK           No error
 */
os_error_t
os_mutex_release(struct os_mutex *mu)
{
    assert(NULL != mu);

    /* Check for valid mutex */
    if (!mu) {
        return OS_INVALID_PARM;
    }

    sem_post(&mu->owner);

    return OS_OK;
}

/**
 * os mutex pend
 *
 * Pend (wait) for a mutex.
 *
 * @param mu Pointer to mutex.
 * @param timeout Timeout, in os ticks. A timeout of 0 means do
 *                not wait if not available. A timeout of
 *                0xFFFFFFFF means wait forever.
 *
 *
 * @return os_error_t
 *      OS_INVALID_PARM     Mutex passed in was NULL.
 *      OS_TIMEOUT          Mutex was owned by another task and timeout=0
 *      OS_OK               no error.
 */
os_error_t
os_mutex_pend(struct os_mutex *mu, uint32_t timeout)
{
    int ret = 0;

    assert(NULL != mu);

    /* Check for valid mutex */
    if (!mu) {
        return OS_INVALID_PARM;
    }

    if(0xFFFFFFFFUL == timeout) {

      ret = sem_wait(&mu->owner);

    } else {

      /* First convert timeout tick into nuttx time unit */
      struct timespec abstime;

      timeout = TICK2NSEC(timeout);

      clock_gettime(CLOCK_REALTIME, &abstime);

      abstime.tv_sec += timeout / NSEC_PER_SEC;
      abstime.tv_nsec += (timeout % NSEC_PER_SEC);
      if(abstime.tv_nsec > NSEC_PER_SEC) {
        abstime.tv_nsec -= NSEC_PER_SEC;
        abstime.tv_sec++;
      }

      ret = sem_timedwait(&mu->owner, &abstime);
    }
    if(OK == ret){
        return OS_OK;
    }

    if(ETIMEDOUT == ret){
        return OS_TIMEOUT;
    }

    return OS_ERROR;

}


/**
 *   @} OSMutex
 * @} OSKernel
 */
