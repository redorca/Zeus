/****************************************************************************
 * arch/arm/src/nrf52/nrf52_oneshot.c
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2018 Zglue Inc. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Levin Li     <zhiqiang@zglue.com>
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
 * 3. Neither the names NuttX nor Atmel nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
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

#include <sys/types.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include "nrf.h"
#include "nrf_rtc.h"
#include "chip/nrf52_priv_rtc.h"
#include "nrf52_oneshot.h"

#ifdef CONFIG_NRF52_ONESHOT

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
#define UINT24_MAX  16777215u

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct nrf52_oneshot_s *g_oneshot[CONFIG_NRF52_ONESHOT_MAXTIMERS];

#define FREERUN_HANDLER   g_oneshot[0]->rtc

#define FREERUN_REG       g_oneshot[0]->rtc->base
/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_oneshot_handler
 *
 * Description:
 *   Common timer interrupt callback.  When any oneshot timer interrupt
 *   expires, this function will be called.  It will forward the call to
 *   the next level up.
 *
 * Input Parameters:
 *   oneshot - The state associated with the expired timer
 *
 * Returned Value:
 *   Always returns OK
 *
 ****************************************************************************/

static int nrf52_oneshot_handler(int irq, void *context, void *arg)
{
  struct nrf52_oneshot_s *oneshot = (struct nrf52_oneshot_s *) arg;
  struct nrf52_rtc_priv_s *dev = (struct nrf52_rtc_priv_s *)oneshot->rtc;
  oneshot_handler_t oneshot_handler;
  FAR void *oneshot_arg;

  tmrinfo("Expired...\n");

  /* The clock was stopped, but not disabled when the RC match occurred.
   * Disable the TC now and disable any further interrupts.
   */

  uint32_t i;
  uint32_t int_mask = (uint32_t)NRF_RTC_INT_COMPARE0_MASK;
  nrf_rtc_event_t event = NRF_RTC_EVENT_COMPARE_0;

  for (i = 0; i < dev->channel_count; i++)
    {
      if (nrf_rtc_int_is_enabled((NRF_RTC_Type *)dev->base, int_mask)
          && nrf_rtc_event_pending((NRF_RTC_Type *)dev->base, event))
        {
          DEBUGASSERT(oneshot != NULL && oneshot->handler);

          nrf_rtc_event_disable((NRF_RTC_Type *)dev->base, int_mask);
          nrf_rtc_int_disable((NRF_RTC_Type *)dev->base, int_mask);
          nrf_rtc_event_clear((NRF_RTC_Type *)dev->base, event);

          /* The timer is no longer running */
          oneshot->running = false;

          /* Forward the event, clearing out any vestiges */
          oneshot_handler  = (oneshot_handler_t)oneshot->handler;
          oneshot->handler = NULL;
          oneshot_arg      = (void *)oneshot->arg;
          oneshot->arg     = NULL;
          if (oneshot_handler)
            {
              oneshot_handler(oneshot_arg);
            }

        }
      int_mask <<= 1;
      event    = (nrf_rtc_event_t)((uint32_t)event + sizeof(uint32_t));
    }

  /* check tick event */

  event = NRF_RTC_EVENT_TICK;
  if (nrf_rtc_int_is_enabled((NRF_RTC_Type *)dev->base, NRF_RTC_INT_TICK_MASK) &&
      nrf_rtc_event_pending((NRF_RTC_Type *)dev->base, event))
    {
      nrf_rtc_event_clear((NRF_RTC_Type *)dev->base, event);
    }

  /* check compare event */
  event = NRF_RTC_EVENT_OVERFLOW;
  if (nrf_rtc_int_is_enabled((NRF_RTC_Type *)dev->base, NRF_RTC_INT_OVERFLOW_MASK) &&
      nrf_rtc_event_pending((NRF_RTC_Type *)dev->base, event))
    {
      nrf_rtc_event_clear((NRF_RTC_Type *)dev->base, event);

      if (oneshot->freerun_handler)
        {
          up_putc('F');
          oneshot->freerun_handler((void *)oneshot->freerun_arg);
        }
    }


  return OK;
}

/****************************************************************************
 * Name: nrf52_allocate_handler
 *
 * Description:
 *   Allocate a timer callback handler for the oneshot instance.
 *
 * Input Parameters:
 *   oneshot - The state instance the new oneshot timer
 *
 * Returned Value:
 *   Returns zero (OK) on success.  This can only fail if the number of
 *   timers exceeds CONFIG_NRF52_ONESHOT_MAXTIMERS.
 *
 ****************************************************************************/

static inline int nrf52_allocate_handler(struct nrf52_oneshot_s *oneshot)
{
#if CONFIG_NRF52_ONESHOT_MAXTIMERS > 1
  int ret = -EBUSY;
  int i;

  /* Search for an unused handler */

  sched_lock();
  for (i = 0; i < CONFIG_NRF52_ONESHOT_MAXTIMERS; i++)
    {
      /* Is this handler available? */

      if (g_oneshot[i] == NULL)
        {
          /* Yes... assign it to this oneshot */

          g_oneshot[i]   = oneshot;
          oneshot->cbndx = i;
          ret            = OK;
          break;
        }
    }

  sched_unlock();
  return ret;

#else
  if (g_oneshot[0] == NULL)
    {
      g_oneshot[0] = oneshot;
      return OK;
    }

  return -EBUSY;
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_oneshot_initialize
 *
 * Description:
 *   Initialize the oneshot timer wrapper
 *
 * Input Parameters:
 *   oneshot    Caller allocated instance of the oneshot state structure
 *   chan       Timer counter channel to be used.
 *   resolution The required resolution of the timer in units of
 *              microseconds.  NOTE that the range is restricted to the
 *              range of uint16_t (excluding zero).
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int nrf52_oneshot_initialize(FAR struct nrf52_oneshot_s *oneshot,
                             int chan, uint16_t resolution)
{
  uint32_t frequency;
  irqstate_t flags;

  tmrinfo("chan=%d resolution=%d usec\n", chan, resolution);
  DEBUGASSERT(oneshot && resolution > 0);

  /* Get the TC frequency the corresponds to the requested resolution */

  frequency = USEC_PER_SEC / (uint32_t)resolution;
  oneshot->frequency = frequency;

  oneshot->rtc = nrf52_rtc_init(chan);
  if (!oneshot->rtc)
    {
      tmrerr("ERROR: Failed to allocate TIM%d\n", chan);
      return -EBUSY;
    }

  nrf52_rtc_set_clock(oneshot->rtc, frequency);

  /* Initialize the remaining fields in the state structure. */

  oneshot->running    = false;
  oneshot->handler    = NULL;
  oneshot->arg        = NULL;

  g_oneshot[0] = oneshot;

  flags = enter_critical_section();

  /* disable tick event */
  nrf_drv_rtc_tick_disable(oneshot->rtc);

  /* Assign a callback handler to the oneshot */
  nrf52_rtc_setisr(oneshot->rtc, nrf52_oneshot_handler, (void *)oneshot, 0);

  nrf52_drv_rtc_cc_set(oneshot->rtc, 0, 0xFFFFFFFFUL, false);

  nrf52_rtc_enable(oneshot->rtc);

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: nrf52_oneshot_max_delay
 *
 * Description:
 *   Determine the maximum delay of the one-shot timer (in microseconds)
 *
 ****************************************************************************/

int nrf52_oneshot_max_delay(FAR struct nrf52_oneshot_s *oneshot,
                            FAR uint64_t *usec)
{
  DEBUGASSERT(oneshot != NULL && usec != NULL);

  /* RTC only had 24bit count register */
  *usec = (uint64_t)( UINT24_MAX / oneshot->frequency) *
          (uint64_t)USEC_PER_SEC;
  return OK;
}

int nrf52_onshot_freerun_set_isr(freerun_handler_t handler, void *arg)
{
  g_oneshot[0]->freerun_handler = handler;
  g_oneshot[0]->freerun_arg = arg;

  return OK;
}

int nrf52_onshot_freerun_enable(void)
{
  nrf_drv_rtc_overflow_enable(FREERUN_HANDLER);
  return OK;
}

int nrf52_onshot_freerun_disable(void)
{
  nrf_drv_rtc_overflow_disable(FREERUN_HANDLER);
  return OK;
}

uint32_t nrf52_onshot_freerun_get_counter(void)
{
  uint32_t count;

  count = nrf_rtc_counter_get((NRF_RTC_Type *)FREERUN_REG);
  return count;
}

uint32_t nrf52_onshot_freerun_get_event(void)
{
  uint32_t event;

  event = nrf_rtc_event_pending((NRF_RTC_Type *)FREERUN_REG, NRF_RTC_EVENT_OVERFLOW);
  return event;
}

void nrf52_onshot_freerun_clear_event(void)
{
  nrf_rtc_event_clear((NRF_RTC_Type *)FREERUN_REG, NRF_RTC_EVENT_OVERFLOW);
}

/****************************************************************************
 * Name: nrf52_oneshot_start
 *
 * Description:
 *   Start the oneshot timer
 *
 * Input Parameters:
 *   oneshot Caller allocated instance of the oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           nrf52_oneshot_initialize();
 *   handler The function to call when when the oneshot timer expires.
 *   arg     An opaque argument that will accompany the callback.
 *   ts      Provides the duration of the one shot timer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int nrf52_oneshot_start(FAR struct nrf52_oneshot_s *oneshot,
                        oneshot_handler_t handler, FAR void *arg,
                        FAR const struct timespec *ts)
{
  uint64_t usec;
  uint32_t period;
  uint32_t count;
  irqstate_t flags;

  tmrinfo("handler=%p arg=%p, ts=(%lu, %lu)\n",
          handler, arg, (unsigned long)ts->tv_sec, (unsigned long)ts->tv_nsec);
  DEBUGASSERT(oneshot && handler && ts);
  DEBUGASSERT(oneshot->rtc);

  /* Was the oneshot already running? */

  flags = enter_critical_section();
  if (oneshot->running)
    {
      /* Yes.. then cancel it */

      tmrinfo("Already running... cancelling\n");
      (void)nrf52_oneshot_cancel(oneshot, NULL);
    }

  /* Save the new handler and its argument */

  oneshot->handler = handler;
  oneshot->arg     = arg;

  /* Express the delay in microseconds */

  usec = (uint64_t)ts->tv_sec * USEC_PER_SEC +
         (uint64_t)(ts->tv_nsec / NSEC_PER_USEC);

  /* Get the timer counter frequency and determine the number of counts need
   * to achieve the requested delay.
   *
   *   frequency = ticks / second
   *   ticks     = seconds * frequency
   *             = (usecs * frequency) / USEC_PER_SEC;
   */

  period = (usec * (uint64_t)oneshot->frequency) / USEC_PER_SEC;

  DEBUGASSERT(period <= UINT32_MAX);

  /* Set timer period */

  oneshot->period = (uint32_t)period;

  /* val is tick unit , set tick period and enable irq */
  count = nrf_rtc_counter_get((NRF_RTC_Type *)oneshot->rtc->base);
  oneshot->start_cnt = count;

  tmrinfo("usec=%llu period=%u, start_cc %d\n", usec, period, count);

  nrf52_drv_rtc_cc_set(oneshot->rtc, 0, period + count, true);
  oneshot->handler = handler;

  oneshot->running = true;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: nrf52_oneshot_cancel
 *
 * Description:
 *   Cancel the oneshot timer and return the time remaining on the timer.
 *
 *   NOTE: This function may execute at a high rate with no timer running (as
 *   when pre-emption is enabled and disabled).
 *
 * Input Parameters:
 *   oneshot Caller allocated instance of the oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           nrf52_oneshot_initialize();
 *   ts      The location in which to return the time remaining on the
 *           oneshot timer.  A time of zero is returned if the timer is
 *           not running.  ts may be zero in which case the time remaining
 *           is not returned.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A call to up_timer_cancel() when
 *   the timer is not active should also return success; a negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

int nrf52_oneshot_cancel(FAR struct nrf52_oneshot_s *oneshot,
                         FAR struct timespec *ts)
{
  irqstate_t flags;
  uint64_t usec;
  uint64_t sec;
  uint64_t nsec;
  uint32_t count;
  uint32_t period;

  /* Was the timer running? */

  flags = enter_critical_section();
  if (!oneshot->running)
    {
      /* No.. Just return zero timer remaining and successful cancellation.
       * This function may execute at a high rate with no timer running
       * (as when pre-emption is enabled and disabled).
       */

      ts->tv_sec  = 0;
      ts->tv_nsec = 0;
      leave_critical_section(flags);
      return OK;
    }

  /* Yes.. Get the timer counter and period registers and stop the counter.
   * If the counter expires while we are doing this, the counter clock will
   * be stopped, but the clock will not be disabled.
   *
   * The expected behavior is that the the counter register will freezes at
   * a value equal to the RC register when the timer expires.  The counter
   * should have values between 0 and RC in all other cased.
   *
   * REVISIT:  This does not appear to be the case.
   */

  tmrinfo("Cancelling...\n");

  count  = nrf_rtc_counter_get((NRF_RTC_Type *)oneshot->rtc->base);
  count -= oneshot->start_cnt;
  period = oneshot->period;

  /* Now we can disable the interrupt and stop the timer. */

  nrf_drv_rtc_cc_disable(oneshot->rtc, 0);

  oneshot->running = false;
  oneshot->handler = NULL;
  oneshot->arg     = NULL;
  leave_critical_section(flags);

  /* Did the caller provide us with a location to return the time
   * remaining?
   */

  if (ts)
    {
      /* Yes.. then calculate and return the time remaining on the
       * oneshot timer.
       */

      tmrinfo("period=%lu count=%lu\n",
              (unsigned long)period, (unsigned long)count);

      /* REVISIT: I am not certain why the timer counter value sometimes
       * exceeds RC.  Might be a bug, or perhaps the counter does not stop
       * in all cases.
       */

      if (count >= period)
        {
          /* No time remaining (?) */

          ts->tv_sec  = 0;
          ts->tv_nsec = 0;
        }
      else
        {
          /* The total time remaining is the difference.  Convert the that
           * to units of microseconds.
           *
           *   frequency = ticks / second
           *   seconds   = ticks * frequency
           *   usecs     = (ticks * USEC_PER_SEC) / frequency;
           */

          usec        = (((uint64_t)(period - count)) * USEC_PER_SEC) /
                        oneshot->frequency;

          /* Return the time remaining in the correct form */

          sec         = usec / USEC_PER_SEC;
          nsec        = ((usec) - (sec * USEC_PER_SEC)) * NSEC_PER_USEC;

          ts->tv_sec  = (time_t)sec;
          ts->tv_nsec = (unsigned long)nsec;
        }

      tmrinfo("remaining (%lu, %lu)\n",
              (unsigned long)ts->tv_sec, (unsigned long)ts->tv_nsec);
    }

  return OK;
}

#endif /* CONFIG_NRF52_ONESHOT */
