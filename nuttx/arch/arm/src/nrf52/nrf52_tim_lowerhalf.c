/****************************************************************************
 * arch/arm/src/nrf52/nrf52_tim_lowerhalf.c
 *
 *   Copyright (C) 2015 Wail Khemir. All rights reserved.
 *   Copyright (C) 2015 Omni Hoverboards Inc. All rights reserved.
 *   Copyright (C) 2016 Sebastien Lorquet All rights reserved.
 *   Authors: Wail Khemir <khemirwail@gmail.com>
 *            Paul Alexander Patience <paul-a.patience@polymtl.ca>
 *            dev@ziggurat29.com
 *            Sebastien Lorquet <sebastien@lorquet.fr>
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

#include <sys/types.h>

#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>

#include <nuttx/irq.h>
#include <nuttx/timers/timer.h>
#include <nuttx/board.h>
#include <debug.h>

#include <arch/board/board.h>

#include "nrf52_tim.h"
#include "chip/nrf52_tim.h"


#if defined(CONFIG_TIMER) && \
    (defined(CONFIG_NRF52_TIM0)  || defined(CONFIG_NRF52_TIM1)  || \
     defined(CONFIG_NRF52_TIM2)  || defined(CONFIG_NRF52_TIM3)  || \
     defined(CONFIG_NRF52_TIM4))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * timer_lowerhalf_s structure.
 */

struct nrf52_lowerhalf_s
{
  FAR const struct timer_ops_s *ops;        /* Lower half operations */
  FAR struct nrf52_tim_dev_s *tim;          /* nrf52 timer driver */
  tccb_t                        callback;   /* Current upper half interrupt callback */
  FAR void                     *arg;        /* Argument passed to upper half callback */
  bool                          started;    /* True: Timer has been started */
  const uint8_t                 resolution; /* Number of bits in the timer (16 or 32 bits) */
  const uint8_t                 channel_count;
  uint32_t                    timeout;    /* Current timeout in uS  */
};


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Interrupt handling *******************************************************/

static int nrf52_timer_handler(int irq, void *context, void *arg);

/* "Lower half" driver methods **********************************************/

static int nrf52_start(FAR struct timer_lowerhalf_s *lower);
static int nrf52_stop(FAR struct timer_lowerhalf_s *lower);
static int nrf52_getstatus(FAR struct timer_lowerhalf_s *lower,
                           FAR struct timer_status_s *status);
static int nrf52_settimeout(FAR struct timer_lowerhalf_s *lower,
                            uint32_t timeout);
static void nrf52_setcallback(FAR struct timer_lowerhalf_s *lower,
                              tccb_t callback, FAR void *arg);
static int  nrf52_ioctl(FAR struct timer_lowerhalf_s *lower, int cmd, unsigned long arg);


/****************************************************************************
 * Private Data
 ****************************************************************************/
/* "Lower half" driver methods */

static const struct timer_ops_s g_timer_ops =
{
  .start       = nrf52_start,
  .stop        = nrf52_stop,
  .getstatus   = nrf52_getstatus,
  .settimeout  = nrf52_settimeout,
  .setcallback = nrf52_setcallback,
  .ioctl       = nrf52_ioctl,
};

#ifdef CONFIG_NRF52_TIM0
static struct nrf52_lowerhalf_s g_tim0_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = NRF_TIMER_BIT_WIDTH_32,
  .started     = false,
  .callback    = NULL,
  .channel_count = TIMER0_CC_NUM,
};
#endif
#ifdef CONFIG_NRF52_TIM1
static struct nrf52_lowerhalf_s g_tim1_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = NRF_TIMER_BIT_WIDTH_32,
  .started     = false,
  .callback    = NULL,
  .channel_count = TIMER1_CC_NUM,

};
#endif

#ifdef CONFIG_NRF52_TIM2
static struct nrf52_lowerhalf_s g_tim2_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = NRF_TIMER_BIT_WIDTH_32,
  .started     = false,
  .callback    = NULL,
  .channel_count = TIMER2_CC_NUM,

};
#endif

#ifdef CONFIG_NRF52_TIM3
static struct nrf52_lowerhalf_s g_tim3_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = NRF_TIMER_BIT_WIDTH_32,
  .started     = false,
  .callback    = NULL,
  .channel_count = TIMER3_CC_NUM,

};
#endif

#ifdef CONFIG_NRF52_TIM4
static struct nrf52_lowerhalf_s g_tim4_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = NRF_TIMER_BIT_WIDTH_32,
  .started     = false,
  .callback    = NULL,
  .channel_count = TIMER4_CC_NUM,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: nrf52_ioctl
 *
 * Description:
 *   ioctl handler
 *
 * Input Parameters:
 *
 * Returned Values:
 *
 ****************************************************************************/
static int  nrf52_ioctl(FAR struct timer_lowerhalf_s *lower, int cmd, unsigned long arg)
{
  FAR struct nrf52_lowerhalf_s *priv = (FAR struct nrf52_lowerhalf_s *)lower;

  DEBUGASSERT(priv && priv->tim);

  /* Handle the command */
  switch (cmd)
    {
      case TCIOC_SET_CHANNEL_TIMEOUT:
        {
          uint32_t  channel_number = ((uint32_t *)arg)[0];
          uint32_t  timeout = ((uint32_t *)arg)[1];
          uint32_t  extended_compare =  ((uint32_t *)arg)[2];
          uint32_t  mask =  ((uint32_t *)arg)[3];
          uint32_t  int_enable = ((uint32_t *)arg)[4];
          uint32_t time_ticks = nrf52_timer_us_to_ticks(priv->tim, timeout);
          /* Check if the channel number is correct for the timer. If not return */
          if ((channel_number > priv->channel_count) || (channel_number < 0))
            {
              return -EINVAL;
            }
          /* Check if the timer is already started. If yes, return */
          if (priv->started)
            {
              return -EPERM;
            }

          /* Check if it is rregular compare or extender compare with a channel mask */
          if (extended_compare)
            {
              nrf52_timer_extended_compare(priv->tim, channel_number, time_ticks, mask, false);
            }
          else
            {
              nrf52_timer_compare(priv->tim, channel_number, time_ticks, false);
            }

          /* Check if interrupts need to be enabled and enable them for a specific channel */
          if (int_enable)
            {
              NRF52_TIM_ENABLEINT(priv->tim, channel_number);
            }

          priv->timeout = timeout;
        }
        break;
      case TCIOC_TIMER_PAUSE:
        {
          nrf52_timer_pause(priv->tim);
          break;
        }
        break;
      case TCIOC_TIMER_RESUME:
        {
          nrf52_timer_resume(priv->tim);
          break;
        }
        break;
      case TCIOC_ENABLE_CHANNEL_INTERRUPT:
        {
          uint32_t  channel_number = ((uint32_t *)arg)[0];
          if ((channel_number > priv->channel_count) || (channel_number < 0))
            {
              return -EINVAL;
            }
          NRF52_TIM_ENABLEINT(priv->tim, channel_number);
        }
        break;
      case TCIOC_DISABLE_CHANNEL_INTERRUPT:
        {
          uint32_t  channel_number = ((uint32_t *)arg)[0];
          if ((channel_number > priv->channel_count) || (channel_number < 0))
            {
              return -EINVAL;
            }
          NRF52_TIM_DISABLEINT(priv->tim, channel_number);
        }
        break;
      default:
        break;
    }

  return OK;
}


/****************************************************************************
 * Name: nrf52_timer_handler
 *
 * Description:
 *   timer interrupt handler
 *
 * Input Parameters:
 *
 * Returned Values:
 *
 ****************************************************************************/

static int nrf52_timer_handler(int irq, void *context, void *arg)
{
  int i = 0;
  FAR struct nrf52_lowerhalf_s *priv = (struct nrf52_lowerhalf_s *)arg;
  uint32_t next_interval_us = 0;
  for (i = 0; i < priv->channel_count; i++)
    {
      if (NRF52_TIM_CHECKINT(priv->tim, i) == true)
        {
          NRF52_TIM_CLEARINT(priv->tim, i);
        }
    }

  tmrwarn("Channel Inerrupt\n");

  if (priv->callback)
    {
      priv->callback(&next_interval_us, priv->arg);
    }
  return OK;
}

/****************************************************************************
 * Name: nrf52_start
 *
 * Description:
 *   Start the timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nrf52_start(FAR struct timer_lowerhalf_s *lower)
{
  FAR struct nrf52_lowerhalf_s *priv = (FAR struct nrf52_lowerhalf_s *)lower;
  if (!priv->started)
    {
      NRF52_TIM_SETISR(priv->tim, nrf52_timer_handler, priv, 0);
      NRF52_TIM_ENABLEINT(priv->tim, NRF_TIMER_CC_CHANNEL0);
      nrf52_timer_enable(priv->tim);
      priv->started = true;
      return OK;
    }

  /* Return EBUSY to indicate that the timer was already running */

  return -EBUSY;
}

/****************************************************************************
 * Name: nrf52_stop
 *
 * Description:
 *   Stop the timer
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nrf52_stop(FAR struct timer_lowerhalf_s *lower)
{
  FAR struct nrf52_lowerhalf_s *priv = (FAR struct nrf52_lowerhalf_s *)lower;

  if (priv->started)
    {
      nrf52_timer_disable(priv->tim);
      priv->started = false;
      return OK;
    }

  /* Return ENODEV to indicate that the timer was not running */

  return -ENODEV;
}

/****************************************************************************
 * Name: nrf52_getstatus
 *
 * Description:
 *   Get the current timer status
 *
 * Input Parameters:
 *   lower  - A pointer the publicly visible representation of the "lower-
 *            half" driver state structure.
 *   status - The location to return the status information.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nrf52_getstatus(FAR struct timer_lowerhalf_s *lower,
                           FAR struct timer_status_s *status)
{
  FAR struct nrf52_lowerhalf_s *priv = (FAR struct nrf52_lowerhalf_s *)lower;
  uint32_t elapsed;

  DEBUGASSERT(priv);

  /* Return the status bit */

  status->flags = 0;
  if (priv->started)
    {
      status->flags |= TCFLAGS_ACTIVE;
    }

  if (priv->callback)
    {
      status->flags |= TCFLAGS_HANDLER;
    }

  /* Return the actual timeout is milliseconds */

  status->timeout = priv->timeout;
  elapsed = nrf52_timer_capture(priv->tim);

  /* Get the time remaining until the timer expires (in microseconds) */
  /* TODO - check on the +1 in the time left calculation */

  status->timeleft = priv->timeout > elapsed ? priv->timeout - elapsed : 0;

  return OK;
}

/****************************************************************************
 * Name: nrf52_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the "lower-half"
 *             driver state structure.
 *   timeout - The new timeout value in microseconds.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nrf52_settimeout(FAR struct timer_lowerhalf_s *lower,
                            uint32_t timeout)
{
  uint32_t time_ticks;
  FAR struct nrf52_lowerhalf_s *priv = (FAR struct nrf52_lowerhalf_s *)lower;
  if (!priv->started)
    {
      time_ticks = nrf52_timer_us_to_ticks(priv->tim, timeout);
      nrf52_timer_extended_compare(priv->tim, NRF_TIMER_CC_CHANNEL0,
                                   time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);

      /* Update the channel number and the timeout in the structure */
      priv->timeout = timeout;

      return OK;
    }

  return -EPERM;
}

/****************************************************************************
 * Name: nrf52_sethandler
 *
 * Description:
 *   Call this user provided timeout handler.
 *
 * Input Parameters:
 *   lower    - A pointer the publicly visible representation of the "lower-half"
 *              driver state structure.
 *   callback - The new timer expiration function pointer.  If this
 *              function pointer is NULL, then the reset-on-expiration
 *              behavior is restored,
 *   arg      - Argument that will be provided in the callback
 *
 * Returned Values:
 *   The previous timer expiration function pointer or NULL is there was
 *   no previous function pointer.
 *
 ****************************************************************************/

static void nrf52_setcallback(FAR struct timer_lowerhalf_s *lower,
                              tccb_t callback, FAR void *arg)
{
  FAR struct nrf52_lowerhalf_s *priv = (FAR struct nrf52_lowerhalf_s *)lower;
  irqstate_t flags = enter_critical_section();

  priv->callback = callback;
  priv->arg     = arg;

  if (callback != NULL && priv->started)
    {
      NRF52_TIM_SETISR(priv->tim, nrf52_timer_handler, priv, 0);
      NRF52_TIM_ENABLEINT(priv->tim, NRF_TIMER_CC_CHANNEL0);
    }
  else
    {
      NRF52_TIM_DISABLEINT(priv->tim, NRF_TIMER_CC_CHANNEL0);
      NRF52_TIM_SETISR(priv->tim, nrf52_timer_handler, priv, 0);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_timer_initialize
 *
 * Description:
 *   Bind the configuration timer to a timer lower half instance and
 *   register the timer drivers at 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the timer device.  This should be of the
 *     form /dev/timer0
 *   timer - the timer's number.
 *
 * Returned Values:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

struct nrf52_tim_dev_s *nrf52_timer_initialize(FAR const char *devpath, int timer)
{
  FAR struct nrf52_lowerhalf_s *lower;

  switch (timer)
    {
#ifdef CONFIG_NRF52_TIM0
      case 0:
        lower = &g_tim0_lowerhalf;
        break;
#endif
#ifdef CONFIG_NRF52_TIM1
      case 1:
        lower = &g_tim1_lowerhalf;
        break;
#endif
#ifdef CONFIG_NRF52_TIM2
      case 2:
        lower = &g_tim2_lowerhalf;
        break;
#endif
#ifdef CONFIG_NRF52_TIM3
      case 3:
        lower = &g_tim3_lowerhalf;
        break;
#endif
#ifdef CONFIG_NRF52_TIM4
      case 4:
        lower = &g_tim4_lowerhalf;
        break;
#endif

      default:
        return NULL;
    }

  /* Initialize the elements of lower half state structure */

  lower->started  = false;
  lower->callback = NULL;


  lower->tim  = nrf52_timer_init(timer);

  if (lower->tim == NULL)
    {
      return (struct nrf52_tim_dev_s *)NULL;
    }

  /* Register the timer driver as /dev/timerX.  The returned value from
   * timer_register is a handle that could be used with timer_unregister().
   * REVISIT: The returned handle is discard here.
   */

  FAR void *drvr = timer_register(devpath,
                                  (FAR struct timer_lowerhalf_s *)lower);
  if (drvr == NULL)
    {
      /* The actual cause of the failure may have been a failure to allocate
       * perhaps a failure to register the timer driver (such as if the
       * 'depath' were not unique).  We know here but we return EEXIST to
       * indicate the failure (implying the non-unique devpath).
       */
      tmrerr("Can't register timer %d.\n", timer);
      return (struct nrf52_tim_dev_s *)NULL;
    }

  return lower->tim;
}

#endif /* CONFIG_TIMER */
