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

#include "os/os.h"
#include <assert.h>


/**
 * @addtogroup OSKernel
 * @{
 *   @defgroup OSSem Semaphores
 *   @{
 */

/* XXX:
 * 1) Should I check to see if we are within an ISR for some of these?
 * 2) Would I do anything different for os_sem_release() if we were in an
 *    ISR when this was called?
 */

/**
 * os sem initialize
 *
 * Initialize a semaphore
 *
 * @param sem Pointer to semaphore
 *        tokens: # of tokens the semaphore should contain initially.
 *
 * @return os_error_t
 *      OS_INVALID_PARM     Semaphore passed in was NULL.
 *      OS_OK               no error.
 */
os_error_t
os_sem_init(struct os_sem *sem, uint16_t tokens)
{
    assert(sem != NULL);

    if (!sem) {
        return OS_INVALID_PARM;
    }

    if(OK == sem_init(&sem->sem_tokens, 0, tokens)){
        return OS_OK;
    }

    return OS_ERROR;
}

/**
 * os sem release
 *
 * Release a semaphore.
 *
 * @param sem Pointer to the semaphore to be released
 *
 * @return os_error_t
 *      OS_INVALID_PARM Semaphore passed in was NULL.
 *      OS_OK No error
 */
os_error_t
os_sem_release(struct os_sem *sem)
{
    assert(sem != NULL);

    /* Check for valid semaphore */
    if (!sem) {
        return OS_INVALID_PARM;
    }
    sem_post(&sem->sem_tokens);

    return OS_OK;
}

/**
 * os sem pend
 *
 * Pend (wait) for a semaphore.
 *
 * @param mu Pointer to semaphore.
 * @param timeout Timeout, in os ticks. A timeout of 0 means do
 *                not wait if not available. A timeout of
 *                0xFFFFFFFF means wait forever.
 *
 *
 * @return os_error_t
 *      OS_INVALID_PARM     Semaphore passed in was NULL.
 *      OS_TIMEOUT          Semaphore was owned by another task and timeout=0
 *      OS_OK               no error.
 */
os_error_t
os_sem_pend(struct os_sem *sem, uint32_t timeout)
{
    int ret = 0;

    struct timespec abstime;
    struct timespec curtime;

    assert(sem != NULL);

    /* Check for valid semaphore */
    if (!sem) {
        return OS_INVALID_PARM;
    }

    ret = clock_gettime(CLOCK_REALTIME, &curtime);
    if (ret != OK) {
        return OS_ERROR;
    }
    else if (curtime.tv_sec < 0 || curtime.tv_nsec < 0 || curtime.tv_nsec >= NSEC_PER_SEC) {
        return OS_ERROR;
    }

    /* First convert timeout tick into nuttx time unit */


    timeout = TICK2USEC(timeout);

    abstime.tv_nsec = ((timeout*NSEC_PER_USEC + curtime.tv_nsec) % NSEC_PER_SEC);
    abstime.tv_sec = ((timeout*NSEC_PER_USEC + curtime.tv_nsec) / NSEC_PER_SEC) + curtime.tv_sec;

    ret = sem_timedwait(&sem->sem_tokens, &abstime);

    if(OK == ret) {
        return OS_OK;
    }

    if(ETIMEDOUT == ret) {
        return OS_TIMEOUT;
    }

    return OS_ERROR;
}


/**
 *   @} OSSched
 * @} OSKernel
 */
