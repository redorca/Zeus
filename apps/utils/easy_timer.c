/****************************************************************************
 * apps/utils/easy_timer.c
 *
 *   Copyright (C) 2014, 2016 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Bob Doiron
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <unistd.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>
#include <utils/easy_timer.h>


#ifdef CONFIG_EASY_TIMER

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/
#define EASY_TIMER_SIGNAL   (17)
#define EASY_SIGVALUE_INT   (42)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t easy_timer_create(timer_t *timerid, sigev_notify_function_t timer_cb)
{
  struct sigevent notify;
  int32_t status;

#ifdef CONFIG_SIG_EVTHREAD
  notify.sigev_notify            = SIGEV_THREAD;
#endif
  notify.sigev_signo             = EASY_TIMER_SIGNAL;
  notify.sigev_value.sival_int   = EASY_SIGVALUE_INT;
  notify.sigev_notify_function   = timer_cb;
  notify.sigev_notify_attributes = NULL;

  status = timer_create(CLOCK_REALTIME, &notify, timerid);
  if (status != OK)
    {
      _err("sigev_thread_test: timer_create failed, errno=%d\n", errno);
    }

  return status;
}

int32_t easy_timer_start(timer_t timerid, et_delay_t delay)
{
  struct itimerspec timer;
  int32_t status;

  ASSERT(NULL != timerid);

  timer.it_value.tv_sec     = delay.start.sec;
  timer.it_value.tv_nsec    = delay.start.nsec;

  timer.it_interval.tv_sec  = delay.interval.sec;
  timer.it_interval.tv_nsec = delay.interval.nsec;

  status = timer_settime(timerid, 0, &timer, NULL);

  if (status != OK)
    {
      _err("sigev_thread_test: timer_settime failed, errno=%d\n", errno);
    }

  return status;
}

int32_t easy_timer_stop(timer_t timerid)
{
  struct itimerspec timer;
  int32_t status;

  ASSERT(NULL != timerid);

  timer.it_value.tv_sec     = 0;
  timer.it_value.tv_nsec    = 0;

  status = timer_settime(timerid, 0, &timer, NULL);

  if (status != OK)
    {
      _err("sigev_thread_test: timer_settime failed, errno=%d\n", errno);
    }

  return status;
}

int32_t easy_timer_delete(timer_t timerid)
{
  int32_t status;

  status = timer_delete(timerid);

  if (status != OK)
    {
      _err("sigev_thread_test: timer_settime failed, errno=%d\n", errno);
    }

  return status;
}

#endif /* CONFIG__EASY_TIMER */
