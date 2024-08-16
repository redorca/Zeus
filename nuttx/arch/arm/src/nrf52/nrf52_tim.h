/************************************************************************************
 * arch/arm/src/nrf52/nrf52_tim.h
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
 *
 * With modifications and updates by:
 *
 *   Copyright (C) 2011-2012, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           dev@ziggurat29.com
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

#ifndef __ARCH_ARM_SRC_NRF52_NRF52_TIM_H
#define __ARCH_ARM_SRC_NRF52_NRF52_TIM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/
#include <stddef.h>
#include "nrf.h"


/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
#define NRF52_TIM_SETISR(d,hnd,arg,s)   ((d)->ops->setisr(d,hnd,arg,s))
#define NRF52_TIM_ENABLEINT(d,s)            ((d)->ops->enableint(d,s))
#define NRF52_TIM_DISABLEINT(d,s)           ((d)->ops->disableint(d,s))
#define NRF52_TIM_CHECKINT(d,s)           ((d)->ops->checkint(d,s))
#define NRF52_TIM_CLEARINT(d,s)           ((d)->ops->clearint(d,s))



#define TCIOC_SET_CHANNEL_TIMEOUT       _TCIOC(TC_NRF52_FIRST)
#define TCIOC_TIMER_PAUSE             _TCIOC(TC_NRF52_FIRST+1)
#define TCIOC_TIMER_RESUME              _TCIOC(TC_NRF52_FIRST+2)
#define TCIOC_ENABLE_CHANNEL_INTERRUPT    _TCIOC(TC_NRF52_FIRST+3)
#define TCIOC_DISABLE_CHANNEL_INTERRUPT   _TCIOC(TC_NRF52_FIRST+4)


/************************************************************************************
 * Public Types
 ************************************************************************************/
#ifdef CONFIG_TIMER

/* TIM Device Structure */

struct nrf52_tim_dev_s
{
  struct nrf52_tim_ops_s *ops;
};

struct nrf52_tim_ops_s
{
  /* Basic Timers */
  void(*enable)(FAR struct nrf52_tim_dev_s *dev);
  void (*pause)(FAR struct nrf52_tim_dev_s *dev);
  void (*resume)(FAR struct nrf52_tim_dev_s *dev);
  void (*disable)(FAR struct nrf52_tim_dev_s *dev);
  void (*clear)(FAR struct nrf52_tim_dev_s *dev);

  /* Timer interrupts */
  int  (*setisr)(FAR struct nrf52_tim_dev_s *dev, xcpt_t handler, void *arg, int source);
  void (*enableint)(FAR struct nrf52_tim_dev_s *dev, uint32_t channel);
  void (*disableint)(FAR struct nrf52_tim_dev_s *dev, uint32_t channel);
  bool (*checkint)(FAR struct nrf52_tim_dev_s *dev, uint32_t channel);
  void (*clearint)(FAR struct nrf52_tim_dev_s *dev, uint32_t channel);
};

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

struct nrf52_tim_dev_s *nrf52_timer_initialize(FAR const char *devpath, int timer);


#endif /* CONFIG_TIMER */

#endif /* __ARCH_ARM_SRC_NRF52_NRF52_TIM_H */
