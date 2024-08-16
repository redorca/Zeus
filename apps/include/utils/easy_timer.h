/****************************************************************************
 * include/utils/easy_timer.h
 *
 *   Copyright (C) 2007-2009, 2014-2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __EASY_TIMER_H
#define __EASY_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <signal.h>
#include <time.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/
typedef struct et_time_t
{
  uint32_t sec;                   /* Seconds */
  uint32_t nsec;                  /* Nanoseconds */
} et_time_t;

typedef struct delay_t
{
  et_time_t start;               /* First be excuted, if 0, never be excuted.*/
  et_time_t interval;            /* Inteval after first be excuted, if 0, excute one time.*/
} et_delay_t;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: easy_timer_create
 *
 * Description:
 *   Create a timer with timer call back function.
 *
 * Input parameters:
 *   timerid - The timer id, should be defined by user and valued by this
 *             API.
 *
 *   timer_cb - Timer callback function, defined by user.
 *
 * Returned Value:
 *   On success, a 0(OK) is returned to the caller.
 *   If an error occurs, the function will return a value of -1 (ERROR)
 *   and set errno to indicate the error.
 *
 ****************************************************************************/
int32_t easy_timer_create(timer_t *timerid, sigev_notify_function_t timer_cb);


/****************************************************************************
 * Name: easy_timer_start
 *
 * Description:
 *   Start a timer with timer delay.
 *
 * Input parameters:
 *   timerid - The timer id.
 *
 *   delay - Timer callback will be called after delay time.
 *
 * Returned Value:
 *   On success, a 0(OK) is returned to the caller.
 *   If an error occurs, the function will return a value of -1 (ERROR)
 *   and set errno to indicate the error.
 *
 ****************************************************************************/
int32_t easy_timer_start(timer_t timerid, et_delay_t delay);

/****************************************************************************
 * Name: easy_timer_stop
 *
 * Description:
 *   Stop a timer.
 *
 * Input parameters:
 *   timerid - The timer id.
 *
 * Returned Value:
 *   On success, a 0(OK) is returned to the caller.
 *   If an error occurs, the function will return a value of -1 (ERROR)
 *   and set errno to indicate the error.
 *
 ****************************************************************************/
int32_t easy_timer_stop(timer_t timerid);

/****************************************************************************
 * Name: easy_timer_delete
 *
 * Description:
 *   Delete a timer.
 *
 * Input parameters:
 *   timerid - The timer id.
 *
 * Returned Value:
 *   On success, a 0(OK) is returned to the caller.
 *   If an error occurs, the function will return a value of -1 (ERROR)
 *   and set errno to indicate the error.
 *
 ****************************************************************************/
int32_t easy_timer_delete(timer_t timerid);

#endif /* __EASY_TIMER_H */
