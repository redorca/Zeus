/****************************************************************************
 *  arch/arm/src/nrf52/nrf52_oneshot_lowerhalf.c
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

#include <stdint.h>
#include <time.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/timers/oneshot.h>

#include "nrf52_oneshot.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of the oneshot timer lower-half driver */

struct nrf52_oneshot_lowerhalf_s
{
  /* This is the part of the lower half driver that is visible to the upper-
   * half client of the driver.  This must be the first thing in this
   * structure so that pointers to struct oneshot_lowerhalf_s are cast
   * compatible to struct nrf52_oneshot_lowerhalf_s and vice versa.
   */

  struct oneshot_lowerhalf_s lh;  /* Common lower-half driver fields */

  /* Private lower half data follows */

  struct nrf52_oneshot_s oneshot; /* NRF52-specific oneshot state */
  oneshot_callback_t callback;    /* internal handler that receives callback */
  FAR void *arg;                  /* Argument that is passed to the handler */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void nrf52_oneshot_handler(void *arg);

static int nrf52_max_delay(FAR struct oneshot_lowerhalf_s *lower,
                           FAR struct timespec *ts);
static int nrf52_start(FAR struct oneshot_lowerhalf_s *lower,
                       oneshot_callback_t callback, FAR void *arg,
                       FAR const struct timespec *ts);
static int nrf52_cancel(FAR struct oneshot_lowerhalf_s *lower,
                        FAR struct timespec *ts);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Lower half operations */

static const struct oneshot_operations_s g_oneshot_ops =
{
  .max_delay = nrf52_max_delay,
  .start     = nrf52_start,
  .cancel    = nrf52_cancel,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_oneshot_handler
 *
 * Description:
 *   Timer expiration handler
 *
 * Input Parameters:
 *   arg - Should be the same argument provided when nrf52_oneshot_start()
 *         was called.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nrf52_oneshot_handler(void *arg)
{
  FAR struct nrf52_oneshot_lowerhalf_s *priv =
    (FAR struct nrf52_oneshot_lowerhalf_s *)arg;
  oneshot_callback_t callback;
  FAR void *cbarg;

  DEBUGASSERT(priv != NULL);

  /* Perhaps the callback was nullified in a race condition with
   * nrf52_cancel?
   */

  if (priv->callback)
    {
      /* Sample and nullify BEFORE executing callback (in case the callback
       * restarts the oneshot).
       */

      callback       = priv->callback;
      cbarg          = priv->arg;
      priv->callback = NULL;
      priv->arg      = NULL;

      /* Then perform the callback */

      callback(&priv->lh, cbarg);
    }
}

/****************************************************************************
 * Name: nrf52_max_delay
 *
 * Description:
 *   Determine the maximum delay of the one-shot timer (in microseconds)
 *
 * Input Parameters:
 *   lower   An instance of the lower-half oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           oneshot_initialize();
 *   ts      The location in which to return the maxumum delay.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int nrf52_max_delay(FAR struct oneshot_lowerhalf_s *lower,
                           FAR struct timespec *ts)
{
  FAR struct nrf52_oneshot_lowerhalf_s *priv =
    (FAR struct nrf52_oneshot_lowerhalf_s *)lower;
  uint64_t usecs;
  int ret;

  DEBUGASSERT(priv != NULL && ts != NULL);
  ret = nrf52_oneshot_max_delay(&priv->oneshot, &usecs);
  if (ret >= 0)
    {
      uint64_t sec = usecs / 1000000;
      usecs -= 1000000 * sec;

      ts->tv_sec  = (time_t)sec;
      ts->tv_nsec = (long)(usecs * 1000);
    }

  return ret;
}

/****************************************************************************
 * Name: nrf52_start
 *
 * Description:
 *   Start the oneshot timer
 *
 * Input Parameters:
 *   lower   An instance of the lower-half oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           oneshot_initialize();
 *   handler The function to call when when the oneshot timer expires.
 *   arg     An opaque argument that will accompany the callback.
 *   ts      Provides the duration of the one shot timer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int nrf52_start(FAR struct oneshot_lowerhalf_s *lower,
                       oneshot_callback_t callback, FAR void *arg,
                       FAR const struct timespec *ts)
{
  FAR struct nrf52_oneshot_lowerhalf_s *priv =
    (FAR struct nrf52_oneshot_lowerhalf_s *)lower;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(priv != NULL && callback != NULL && ts != NULL);

  /* Save the callback information and start the timer */

  flags          = enter_critical_section();
  priv->callback = callback;
  priv->arg      = arg;
  ret            = nrf52_oneshot_start(&priv->oneshot,
                                       nrf52_oneshot_handler, priv, ts);
  leave_critical_section(flags);

  if (ret < 0)
    {
      tmrerr("ERROR: nrf52_oneshot_start failed: %d\n", flags);
    }

  return ret;
}

/****************************************************************************
 * Name: nrf52_cancel
 *
 * Description:
 *   Cancel the oneshot timer and return the time remaining on the timer.
 *
 *   NOTE: This function may execute at a high rate with no timer running (as
 *   when pre-emption is enabled and disabled).
 *
 * Input Parameters:
 *   lower   Caller allocated instance of the oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           oneshot_initialize();
 *   ts      The location in which to return the time remaining on the
 *           oneshot timer.  A time of zero is returned if the timer is
 *           not running.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A call to up_timer_cancel() when
 *   the timer is not active should also return success; a negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

static int nrf52_cancel(FAR struct oneshot_lowerhalf_s *lower,
                        FAR struct timespec *ts)
{
  FAR struct nrf52_oneshot_lowerhalf_s *priv =
    (FAR struct nrf52_oneshot_lowerhalf_s *)lower;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Cancel the timer */

  flags          = enter_critical_section();
  ret            = nrf52_oneshot_cancel(&priv->oneshot, ts);
  priv->callback = NULL;
  priv->arg      = NULL;
  leave_critical_section(flags);

  if (ret < 0)
    {
      tmrerr("ERROR: nrf52_oneshot_cancel failed: %d\n", flags);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: oneshot_initialize
 *
 * Description:
 *   Initialize the oneshot timer and return a oneshot lower half driver
 *   instance.
 *
 * Input Parameters:
 *   chan       Timer counter channel to be used.
 *   resolution The required resolution of the timer in units of
 *              microseconds.  NOTE that the range is restricted to the
 *              range of uint16_t (excluding zero).
 *
 * Returned Value:
 *   On success, a non-NULL instance of the oneshot lower-half driver is
 *   returned.  NULL is return on any failure.
 *
 ****************************************************************************/

FAR struct oneshot_lowerhalf_s *oneshot_initialize(int chan,
                                                   uint16_t resolution)
{
  FAR struct nrf52_oneshot_lowerhalf_s *priv;
  int ret;

  /* Allocate an instance of the lower half driver */

  priv = (FAR struct nrf52_oneshot_lowerhalf_s *)
         kmm_zalloc(sizeof(struct nrf52_oneshot_lowerhalf_s));

  if (priv == NULL)
    {
      tmrerr("ERROR: Failed to initialized state structure\n");
      return NULL;
    }

  /* Initialize the lower-half driver structure */

  priv->lh.ops = &g_oneshot_ops;

  /* Initialize the contained STM32 oneshot timer */

  ret = nrf52_oneshot_initialize(&priv->oneshot, chan, resolution);
  if (ret < 0)
    {
      tmrerr("ERROR: nrf52_oneshot_initialize failed: %d\n", ret);
      kmm_free(priv);
      return NULL;
    }

  return &priv->lh;
}
