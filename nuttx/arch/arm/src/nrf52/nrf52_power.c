/****************************************************************************
  * arch/arm/src/nrf52/nrf52_power.c
  *
  *   Copyright (C) 2012, 2018 Levin Li. All rights reserved.
  *   Author: Levin Li <zhiqiang@zglue.com>
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
#include <nuttx/arch.h>

#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <arch/board/board.h>

#include "nvic.h"
#include "cache.h"
#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

#include "nrf.h"
#include "nrf_power.h"
#include "nrf52_power.h"


/* power callback : 0 is for sleep; 1 is for usbd */
static power_callback power_irq_callback[2];

uint32_t nrf52_power_get_usbphy_status(void)
{
#if NRF_POWER_HAS_USBREG
  uint32_t status;
  status = nrf_power_usbregstatus_get();

  uinfo("USBPHY Power Status: %d\n", status);

  if (NRF_POWER_USBREGSTATUS_OUTPUTRDY_MASK & status)
    {
      return NRF52_POWER_USBD_PWRRDY;
    }
  else if (NRF_POWER_USBREGSTATUS_VBUSDETECT_MASK & status)
    {
      return NRF52_POWER_USBD_DETECT;
    }

  return NRF52_POWER_USBD_REMOVE;

#else

  return NRF52_POWER_USBD_REMOVE;

#endif
}
static int nrf52_power_irq(int irq, FAR void *context, FAR void *arg)
{

  uint32_t event = 0;
  uint32_t usb_type = 0;
  uint32_t sleep_type = 0;

#if NRF_POWER_HAS_USBREG
  if (nrf_power_event_get_and_clear(NRF_POWER_EVENT_USBDETECTED))
    {
      event = NRF52_POWER_USBD_DETECT;
      usb_type = 1;
    }

  if (nrf_power_event_get_and_clear(NRF_POWER_EVENT_USBPWRRDY))
    {
      event |= NRF52_POWER_USBD_PWRRDY;
      usb_type = 1;
    }

  if (nrf_power_event_get_and_clear(NRF_POWER_EVENT_USBREMOVED))
    {
      event = NRF52_POWER_USBD_REMOVE;
      usb_type = 1;
    }
#endif
  /* is there multi-event generating at the same time ??? */
  if (usb_type && power_irq_callback[POWER_CALLBACK_USBD])
    {
      power_irq_callback[POWER_CALLBACK_USBD](event);
    }

#if NRF_POWER_HAS_SLEEPEVT
  if (nrf_power_event_get_and_clear(NRF_POWER_EVENT_SLEEPENTER))
    {
      event = NRF52_POWER_SLEEP_ENTER;
      sleep_type = 1;
    }

  if (nrf_power_event_get_and_clear(NRF_POWER_EVENT_SLEEPEXIT))
    {
      event = NRF52_POWER_SLEEP_EXIT;
      sleep_type = 1;
    }
#endif

  if (sleep_type && power_irq_callback[POWER_CALLBACK_SLEEP])
    {
      power_irq_callback[POWER_CALLBACK_SLEEP](event);
    }

  return OK;
}

int  nrf52_power_register_callback(power_callback callback,            uint8_t type)
{
  ASSERT(POWER_CALLBACK_SLEEP == type || POWER_CALLBACK_USBD == type);

  uint32_t mask = 0;
  irqstate_t flag;

#if NRF_POWER_HAS_USBREG

  if (POWER_CALLBACK_USBD == type)
    {
      /* enable detect, remove, pwrdy interrupt */
      mask = NRF_POWER_INT_USBDETECTED_MASK;
      mask |= NRF_POWER_INT_USBREMOVED_MASK;
      mask |= NRF_POWER_INT_USBPWRRDY_MASK;
    }
#endif

#if NRF_POWER_HAS_SLEEPEVT

  if (POWER_CALLBACK_SLEEP == type)
    {
      /* enable sleep enter, exit interrupt */
      mask = NRF_POWER_INT_SLEEPENTER_MASK;
      mask |= NRF_POWER_INT_SLEEPEXIT_MASK;
    }
#endif

  flag = enter_critical_section();

  /* clear all event ; then enable or disable interrtup */

  nrf_power_event_clear(0xFFFFUL);

  power_irq_callback[type] = callback;
  if (callback)
    {
      nrf_power_int_enable(mask);
    }
  else
    {
      nrf_power_int_disable(mask);
    }

  /* there is anyone callback , the interrput should be kept as enable */
  if (power_irq_callback[0] || power_irq_callback[1])
    {
      up_enable_irq(POWER_CLOCK_IRQn);
    }
  else
    {
      up_disable_irq(POWER_CLOCK_IRQn);
    }

  leave_critical_section(flag);

  return OK;
}

void nrf52_power_initlialize(void)
{
  irq_attach(POWER_CLOCK_IRQn, nrf52_power_irq, NULL);
}
