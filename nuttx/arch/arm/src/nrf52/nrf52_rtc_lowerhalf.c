/****************************************************************************
 * arch/arm/src/nrf52/nrf52_rtc_lowerhalf.c
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
#include <stddef.h>
#include <nuttx/irq.h>
#include <nuttx/timers/counter.h>
#include <nuttx/board.h>
#include <nuttx/clock.h>
#include <arch/board/board.h>

#include "nrf52_rtc.h"
#include "chip/nrf52_priv_rtc.h"


#if defined(CONFIG_COUNTER) && \
    (defined(CONFIG_NRF52_RTC0)  || defined(CONFIG_NRF52_RTC1)  || \
     defined(CONFIG_NRF52_RTC2))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * rtc_lowerhalf_s structure.
 */

struct nrf52_lowerhalf_s
{
  FAR const struct rtc_ops_s *ops;        /* Lower half operations */
  FAR struct nrf52_rtc_priv_s *rtc;          /* nrf52 rtc driver */
  tccb_t                        callback;   /* Current upper half interrupt callback */
  FAR void                     *arg;        /* Argument passed to upper half callback */
  bool                          started;    /* True: RTC has been started */
  const uint8_t                 channel_count; /* Number of channels supported by the RTC device */
  uint32_t timeout;                         /* The current timeout value (us) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Interrupt handling *******************************************************/
static int nrf52_rtc_handler(int irq, void *context, void *arg);

/* "Lower half" driver methods **********************************************/
static int nrf52_rtc_start(FAR struct rtc_lowerhalf_s *lower);
static int nrf52_rtc_stop(FAR struct rtc_lowerhalf_s *lower);
static int nrf52_rtc_getstatus(FAR struct rtc_lowerhalf_s *lower,
                               FAR struct rtc_status_s *status);
static int nrf52_rtc_settimeout(FAR struct rtc_lowerhalf_s *lower,
                                uint32_t timeout);
static void nrf52_rtc_setcallback(FAR struct rtc_lowerhalf_s *lower,
                                  tccb_t callback, FAR void *arg);
static int  nrf52_rtc_ioctl(FAR struct rtc_lowerhalf_s *lower, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* "Lower half" driver methods */

static const struct rtc_ops_s g_rtc_ops =
{
  .start       = nrf52_rtc_start,
  .stop        = nrf52_rtc_stop,
  .getstatus   = nrf52_rtc_getstatus,
  .settimeout  = nrf52_rtc_settimeout,
  .setcallback = nrf52_rtc_setcallback,
  .ioctl       = nrf52_rtc_ioctl,
};

#ifdef CONFIG_NRF52_RTC0
static struct nrf52_lowerhalf_s g_rtc0_lowerhalf =
{
  .ops         = &g_rtc_ops,
  .started     = false,
  .callback    = NULL,
  .channel_count = RTC0_CC_NUM,
};
#endif
#ifdef CONFIG_NRF52_RTC1
static struct nrf52_lowerhalf_s g_rtc1_lowerhalf =
{
  .ops         = &g_rtc_ops,
  .started     = false,
  .callback    = NULL,
  .channel_count = RTC1_CC_NUM,
};
#endif

#ifdef CONFIG_NRF52_RTC2
static struct nrf52_lowerhalf_s g_rtc2_lowerhalf =
{
  .ops         = &g_rtc_ops,
  .started     = false,
  .callback    = NULL,
  .channel_count = RTC2_CC_NUM,
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
static int  nrf52_rtc_ioctl(FAR struct rtc_lowerhalf_s *lower, int cmd, unsigned long arg)
{

  return OK;
}

/****************************************************************************
 * Name: nrf52_rtc_handler
 *
 * Description:
 *   rtc interrupt handler
 *
 * Input Parameters:
 *
 * Returned Values:
 *
 ****************************************************************************/

static int nrf52_rtc_handler(int irq, void *context, void *arg)
{
  uint32_t next_interval_us = 0;
  FAR struct nrf52_lowerhalf_s *priv = (struct nrf52_lowerhalf_s *)arg;
  nrf52_rtc_irq_handleint(priv->rtc);
  priv->callback(&next_interval_us, priv->arg);
  return OK;
}

/****************************************************************************
 * Name: nrf52_rtc_start
 *
 * Description:
 *   Start the rtc, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nrf52_rtc_start(FAR struct rtc_lowerhalf_s *lower)
{

  FAR struct nrf52_lowerhalf_s *priv = (FAR struct nrf52_lowerhalf_s *)lower;
  if (!priv->started)
    {
      if (priv->callback != NULL)
        {
          nrf52_rtc_enable(priv->rtc);
        }

      priv->started = true;
      return OK;
    }

  /* Return EBUSY to indicate that the rtc was already running */

  return -EBUSY;
}

/****************************************************************************
 * Name: nrf52_rtc_stop
 *
 * Description:
 *   Stop the rtc
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nrf52_rtc_stop(FAR struct rtc_lowerhalf_s *lower)
{

  FAR struct nrf52_lowerhalf_s *priv = (FAR struct nrf52_lowerhalf_s *)lower;

  if (priv->started)
    {
      if (priv->callback != NULL)
        {
          nrf52_rtc_disable(priv->rtc);
        }

      priv->started = false;
      return OK;
    }

  /* Return ENODEV to indicate that the rtc was not running */
  return -ENODEV;
}

/****************************************************************************
 * Name: nrf52_getstatus
 *
 * Description:
 *   Get the current rtc status
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

static int nrf52_rtc_getstatus(FAR struct rtc_lowerhalf_s *lower,
                               FAR struct rtc_status_s *status)
{
  FAR struct nrf52_lowerhalf_s *priv = (FAR struct nrf52_lowerhalf_s *)lower;
  //uint32_t elapsed;

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
  status->timeout  = priv->timeout;
  /* Needs fix. Currently the counter keeps increasing and goes greater than timeout */
  status->timeleft = (nrf52_get_current_rtc_timeout(priv->rtc) * USEC_PER_SEC) / (RTC_DEFAULT_CONFIG_FREQUENCY / (RTC_PRESCALER + 1));

  return OK;
}

/****************************************************************************
 * Name: nrf52_rtc_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the rtc)
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

static int nrf52_rtc_settimeout(FAR struct rtc_lowerhalf_s *lower,
                                uint32_t timeout)
{
  FAR struct nrf52_lowerhalf_s *priv = (FAR struct nrf52_lowerhalf_s *)lower;
  if (!priv->started)
    {
      nrf52_drv_rtc_cc_set(priv->rtc, 0, ((timeout / USEC_PER_SEC) * (RTC_DEFAULT_CONFIG_FREQUENCY / (RTC_PRESCALER + 1))), true);
      priv->timeout = timeout;
      return OK;
    }

  return -EPERM;
}

/****************************************************************************
 * Name: nrf52_rtc_setcallback
 *
 * Description:
 *   Call this user provided timeout handler.
 *
 * Input Parameters:
 *   lower    - A pointer the publicly visible representation of the "lower-half"
 *              driver state structure.
 *   callback - The new rtc expiration function pointer.  If this
 *              function pointer is NULL, then the reset-on-expiration
 *              behavior is restored,
 *   arg      - Argument that will be provided in the callback
 *
 * Returned Values:
 *   The previous rtc expiration function pointer or NULL is there was
 *   no previous function pointer.
 *
 ****************************************************************************/

static void nrf52_rtc_setcallback(FAR struct rtc_lowerhalf_s *lower,
                                  tccb_t callback, FAR void *arg)
{
  FAR struct nrf52_lowerhalf_s *priv = (FAR struct nrf52_lowerhalf_s *)lower;
  irqstate_t flags = enter_critical_section();

  priv->callback = callback;
  priv->arg     = arg;

  if (callback != NULL)
    {
      nrf52_rtc_setisr(priv->rtc, nrf52_rtc_handler, priv, 0);
    }

  leave_critical_section(flags);

}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_rtc_initialize
 *
 * Description:
 *   Bind the configuration rtc to a rtc lower half instance and
 *   register the rtc drivers at 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the rtc device.  This should be of the
 *     form /dev/rtc0
 *   rtc - the rtc's number.
 *
 * Returned Values:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int nrf52_rtc_initialize(FAR const char *devpath, int rtc)
{
  FAR struct nrf52_lowerhalf_s *lower;

  switch (rtc)
    {
#ifdef CONFIG_NRF52_RTC0
      case 0:
        lower = &g_rtc0_lowerhalf;
        break;
#endif
#ifdef CONFIG_NRF52_RTC1
      case 1:
        lower = &g_rtc1_lowerhalf;
        break;
#endif
#ifdef CONFIG_NRF52_RTC2
      case 2:
        lower = &g_rtc2_lowerhalf;
        break;
#endif
      default:
        return -ENODEV;
    }

  /* Initialize the elements of lower half state structure */

  lower->started  = false;
  lower->callback = NULL;

  lower->rtc  = nrf52_rtc_init(rtc);

  if (lower->rtc == NULL)
    {
      return -EINVAL;
    }
  /* Register the rtc driver as /dev/rtcX.  The returned value from
   * rtc_register is a handle that could be used with rtc_unregister().
   * REVISIT: The returned handle is discard here.
   */

  FAR void *drvr = rtc_register(devpath,
                                (FAR struct rtc_lowerhalf_s *)lower);
  if (drvr == NULL)
    {
      /* The actual cause of the failure may have been a failure to allocate
       * perhaps a failure to register the rtc driver (such as if the
       * 'depath' were not unique).  We know here but we return EEXIST to
       * indicate the failure (implying the non-unique devpath).
       */

      return -EEXIST;
    }

  return OK;
}

#endif /* CONFIG_COUNTER */
