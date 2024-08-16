/************************************************************************************
 * arm/arm/src/nrf52/nrf52_tim.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
 *
 * With modifications and updates by:
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           <dev@ziggurat29.com>
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <sys/types.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include "chip.h"
#include "up_internal.h"
#include "up_arch.h"
#include "nrf.h"
#include "nrf_rtc.h"
#include "nrf52_gpio.h"
#include "nrf52_rtc.h"
#include "chip/nrf52_priv_rtc.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/************************************************************************************
 * Private Data
 ************************************************************************************/

#if defined(CONFIG_NRF52_RTC0)  || defined(CONFIG_NRF52_RTC1)  || \
    defined(CONFIG_NRF52_RTC2)


#ifdef CONFIG_NRF52_RTC0
struct nrf52_rtc_priv_s nrf52_rtc0_priv =
{
  .base           = NRF_RTC0_BASE,
  .state          = NRF_DRV_STATE_UNINITIALIZED,
  .channel_count  = RTC0_CC_NUM,
  .channel_number   = NRF_RTC_CC_CHANNEL0,
};
#endif
#ifdef CONFIG_NRF52_RTC1
struct nrf52_rtc_priv_s nrf52_rtc1_priv =
{
  .base           = NRF_RTC1_BASE,
  .state          = NRF_DRV_STATE_UNINITIALIZED,
  .channel_count  = RTC1_CC_NUM,
  .channel_number   = NRF_RTC_CC_CHANNEL0,
};
#endif

#ifdef CONFIG_NRF52_RTC2
struct nrf52_rtc_priv_s nrf52_rtc2_priv =
{
  .base           = NRF_RTC2_BASE,
  .state          = NRF_DRV_STATE_UNINITIALIZED,
  .channel_count  = RTC2_CC_NUM,
  .channel_number   = NRF_RTC_CC_CHANNEL0,
};
#endif

/************************************************************************************
 * Name: nrf52_rtc_init
 *
 * Description:
 *
 ************************************************************************************/
FAR struct nrf52_rtc_priv_s *nrf52_rtc_init(int rtc)
{

  struct nrf52_rtc_priv_s *dev = NULL;

  /* Get structure and enable power */

  switch (rtc)
    {
#ifdef CONFIG_NRF52_RTC0
      case 0:
        dev = &nrf52_rtc0_priv;
        break;
#endif
#ifdef CONFIG_NRF52_RTC1
      case 1:
        dev = &nrf52_rtc1_priv;
        break;
#endif
#ifdef CONFIG_NRF52_RTC2
      case 2:
        dev = &nrf52_rtc2_priv;
        break;
#endif

      default:
        return NULL;
    }

  /* Is device already allocated */
  if (dev->state != NRF_DRV_STATE_UNINITIALIZED)
    {
      return NULL;
    }

  nrf_rtc_prescaler_set((NRF_RTC_Type *)dev->base, RTC_PRESCALER);
  dev->reliable = RTC_DEFAULT_CONFIG_RELIABLE;
  dev->tick_latency = RTC_US_TO_TICKS(NRF_MAXIMUM_LATENCY_US, RTC_DEFAULT_CONFIG_FREQUENCY);
  dev->state = NRF_DRV_STATE_INITIALIZED;

  return dev;
}


/************************************************************************************
 * Name: nrf52_drv_rtc_tick_enable
 *
 * Description:
 *
 ************************************************************************************/
void nrf_drv_rtc_tick_enable(FAR struct nrf52_rtc_priv_s *dev)
{
  nrf_rtc_event_t event = NRF_RTC_EVENT_TICK;
  uint32_t mask = NRF_RTC_INT_TICK_MASK;

  nrf_rtc_event_clear((NRF_RTC_Type *)dev->base, event);
  nrf_rtc_event_enable((NRF_RTC_Type *)dev->base, mask);
  nrf_rtc_int_enable((NRF_RTC_Type *)dev->base, mask);

  rtcinfo("Tick events enable.");

  return;
}

void nrf_drv_rtc_tick_disable(FAR struct nrf52_rtc_priv_s *dev)
{
  uint32_t mask = NRF_RTC_INT_TICK_MASK;

  nrf_rtc_event_disable((NRF_RTC_Type *)dev->base, mask);
  nrf_rtc_int_disable((NRF_RTC_Type *)dev->base, mask);
  rtcinfo("Tick events disabled.");
}


void nrf_drv_rtc_overflow_enable(FAR struct nrf52_rtc_priv_s *dev)
{
  nrf_rtc_event_t event = NRF_RTC_EVENT_OVERFLOW;
  uint32_t mask = NRF_RTC_INT_OVERFLOW_MASK;

  nrf_rtc_event_clear((NRF_RTC_Type *)dev->base, event);
  nrf_rtc_event_enable((NRF_RTC_Type *)dev->base, mask);
  nrf_rtc_int_enable((NRF_RTC_Type *)dev->base, mask);

  rtcinfo("Tick overflow events enable.");

  return;
}

void nrf_drv_rtc_overflow_disable(FAR struct nrf52_rtc_priv_s *dev)
{
  uint32_t mask = NRF_RTC_INT_OVERFLOW_MASK;
  nrf_rtc_event_disable((NRF_RTC_Type *)dev->base, mask);
  nrf_rtc_int_disable((NRF_RTC_Type *)dev->base, mask);

  rtcinfo("Tick overflow events disabled.");

}

ret_code_t nrf_drv_rtc_cc_disable(FAR struct nrf52_rtc_priv_s *dev, uint32_t channel)
{

  ret_code_t err_code;
  uint32_t int_mask = RTC_CHANNEL_INT_MASK(channel);
  nrf_rtc_event_t event    = RTC_CHANNEL_EVENT_ADDR(channel);

  nrf_rtc_event_disable((NRF_RTC_Type *)dev->base, int_mask);
  if (nrf_rtc_int_is_enabled((NRF_RTC_Type *)dev->base, int_mask))
    {
      nrf_rtc_int_disable((NRF_RTC_Type *)dev->base, int_mask);
      if (nrf_rtc_event_pending((NRF_RTC_Type *)dev->base, event))
        {
          nrf_rtc_event_clear((NRF_RTC_Type *)dev->base, event);
          err_code = ETIMEDOUT;
          return err_code;
        }
    }

  rtcinfo("RTC id: %d, channel disabled: %d.", dev->rtc->base, channel);
  err_code = OK;
  return err_code;
}

/************************************************************************************
 * Name: nrf52_drv_rtc_cc_set
 *
 * Description:
 *
 ************************************************************************************/
ret_code_t nrf52_drv_rtc_cc_set(FAR struct nrf52_rtc_priv_s *dev,
                                uint32_t channel,
                                uint32_t val,
                                bool enable_irq)
{
  ASSERT(dev->state != NRF_DRV_STATE_UNINITIALIZED);
  ASSERT(channel < dev->channel_count);

  ret_code_t err_code;
  dev->timeout = val;
  uint32_t int_mask = RTC_CHANNEL_INT_MASK(channel);
  nrf_rtc_event_t event    = RTC_CHANNEL_EVENT_ADDR(channel);
  nrf_rtc_event_disable((NRF_RTC_Type *)dev->base, int_mask);
  nrf_rtc_int_disable((NRF_RTC_Type *)dev->base, int_mask);
  val = RTC_WRAP(val);

  if (dev->reliable)
    {
      nrf_rtc_cc_set((NRF_RTC_Type *)dev->base, channel, val);

      uint32_t cnt = nrf_rtc_counter_get((NRF_RTC_Type *)dev->base);

      int32_t diff = cnt - val;

      if (cnt < val)
        {
          diff += RTC_COUNTER_COUNTER_Msk;
        }
      if (diff < dev->tick_latency)
        {
          err_code = ETIMEDOUT;
          return err_code;
        }

    }
  else
    {
      nrf_rtc_cc_set((NRF_RTC_Type *)dev->base, channel, val);
    }

  if (enable_irq)
    {
      nrf_rtc_event_clear((NRF_RTC_Type *)dev->base, event);
      nrf_rtc_int_enable((NRF_RTC_Type *)dev->base, int_mask);
    }

  nrf_rtc_event_enable((NRF_RTC_Type *)dev->base, int_mask);

  return OK;
}


/************************************************************************************
 * Name: nrf52_get_current_rtc_timeout
 *
 * Description:
 *
 ************************************************************************************/
uint32_t nrf52_get_current_rtc_timeout(FAR struct nrf52_rtc_priv_s *dev)
{
  return nrf_rtc_counter_get((NRF_RTC_Type *)dev->base);;
}

/************************************************************************************
 * Name: nrf52_rtc_set_clock
 *
 * Description:
 *
 ************************************************************************************/
uint32_t nrf52_rtc_set_clock(FAR struct nrf52_rtc_priv_s *dev, uint32_t frequency)
{
  uint32_t prescaler;

  prescaler = RTC_DEFAULT_CONFIG_FREQUENCY / frequency;
  nrf_rtc_prescaler_set((NRF_RTC_Type *)dev->base, prescaler);

  return OK;
}


/************************************************************************************
 * Name: nrf52_rtc_setisr
 *
 * Description:
 *
 ************************************************************************************/

int nrf52_rtc_setisr(FAR struct nrf52_rtc_priv_s *dev,
                     xcpt_t handler, void *arg, int source)
{

  int vectorno;
  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(source == 0);

  switch (dev->base)
    {
#ifdef CONFIG_NRF52_RTC0
      case NRF_RTC0_BASE:
        vectorno = RTC0_IRQn;
        break;
#endif
#ifdef CONFIG_NRF52_RTC1
      case NRF_RTC1_BASE:
        vectorno = RTC1_IRQn;
        break;
#endif
#ifdef CONFIG_NRF52_RTC2
      case NRF_RTC2_BASE:
        vectorno = RTC2_IRQn;
        break;
#endif
      default:
        return -EINVAL;
    }

  /* Disable interrupt when callback is removed */
  if (!handler)
    {
      up_disable_irq(vectorno);
      irq_detach(vectorno);
      return OK;
    }

  /* Otherwise set callback and enable interrupt */
  irq_attach(vectorno, handler, arg);
  up_enable_irq(vectorno);

#ifdef CONFIG_ARCH_IRQPRIO
  /* Set the interrupt priority */
  up_prioritize_irq(vectorno, NVIC_SYSH_PRIORITY_DEFAULT);
#endif
  return OK;
}


/************************************************************************************
 * Name: nrf52_rtc_irq_checkint
 *
 * Description:
 *
 ************************************************************************************/
bool nrf52_rtc_irq_handleint(FAR struct nrf52_rtc_priv_s *dev)
{
  uint32_t i;
  uint32_t int_mask = (uint32_t)NRF_RTC_INT_COMPARE0_MASK;
  nrf_rtc_event_t event = NRF_RTC_EVENT_COMPARE_0;

  for (i = 0; i < dev->channel_count; i++)
    {
      if (nrf_rtc_int_is_enabled((NRF_RTC_Type *)dev->base, int_mask)
          && nrf_rtc_event_pending((NRF_RTC_Type *)dev->base, event))
        {
          nrf_rtc_event_disable((NRF_RTC_Type *)dev->base, int_mask);
          nrf_rtc_int_disable((NRF_RTC_Type *)dev->base, int_mask);
          nrf_rtc_event_clear((NRF_RTC_Type *)dev->base, event);
        }
      int_mask <<= 1;
      event    = (nrf_rtc_event_t)((uint32_t)event + sizeof(uint32_t));
    }
  event = NRF_RTC_EVENT_TICK;
  if (nrf_rtc_int_is_enabled((NRF_RTC_Type *)dev->base, NRF_RTC_INT_TICK_MASK) &&
      nrf_rtc_event_pending((NRF_RTC_Type *)dev->base, event))
    {
      nrf_rtc_event_clear((NRF_RTC_Type *)dev->base, event);
    }

  return OK;

}


/************************************************************************************
 * Name: nrf52_rtc_enable
 *
 * Description:
 *
 ************************************************************************************/
void nrf52_rtc_enable(FAR struct nrf52_rtc_priv_s *dev)
{
  ASSERT(dev->state == NRF_DRV_STATE_INITIALIZED);
  nrf_rtc_task_trigger((NRF_RTC_Type *)dev->base, NRF_RTC_TASK_START);
  dev->state  = NRF_DRV_STATE_POWERED_ON;
}

/************************************************************************************
 * Name: nrf52_rtc_disable
 *
 * Description:
 *
 ************************************************************************************/
void nrf52_rtc_disable(FAR struct nrf52_rtc_priv_s *dev)
{
  ASSERT(dev->state == NRF_DRV_STATE_POWERED_ON);
  /* Clearing the counter and disabling the RTC */
  nrf_rtc_task_trigger((NRF_RTC_Type *)dev->base, NRF_RTC_TASK_CLEAR);
  nrf_rtc_task_trigger((NRF_RTC_Type *)dev->base, NRF_RTC_TASK_STOP);
  dev->state = NRF_DRV_STATE_INITIALIZED;

}
/************************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time.  All RTC implementations must be able to
 *   set their time based on a standard timespec.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

int up_rtc_settime(FAR const struct timespec *tp)
{

  return OK;
}

#endif
