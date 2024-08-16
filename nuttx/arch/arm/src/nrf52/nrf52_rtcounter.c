/************************************************************************************
 * arch/arm/src/nrf52/nrf52_rtcounter.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
 *
 * With extensions, modifications by:
 *
 *   Copyright (C) 2011-2013, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregroy Nutt <gnutt@nuttx.org>
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
#include <nuttx/timers/rtc.h>
#include <arch/board/board.h>

#include <stdlib.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>

#include "up_arch.h"
#include "nrf_rtc.h"
#include "chip/nrf52_priv_rtc.h"
#include "nrf52_clock.h"
/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
#if defined(CONFIG_RTC)
#define RTC_BASE NRF_RTC1_BASE
#define RTC_IRQ RTC1_IRQn
#endif
/************************************************************************************
 * Private Types
 ************************************************************************************/



/************************************************************************************
 * Private Data
 ************************************************************************************/


/************************************************************************************
 * Public Data
 ************************************************************************************/
/* Variable determines the state of the LSE oscillator.
 * Possible errors:
 *   - on start-up
 *   - during operation, reported by LSE interrupt
 */

volatile bool g_rtc_enabled = false;

/************************************************************************************
 * Private Functions
 ************************************************************************************/


/************************************************************************************
 * Name: nrf52_rtc_interrupt
 *
 * Description:
 *    RTC interrupt service routine
 *
 * Input Parameters:
 *   irq - The IRQ number that generated the interrupt
 *   context - Architecture specific register save information.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value on failure.
 *
 ************************************************************************************/


#if defined(CONFIG_RTC_TICKS)
static int nrf52_rtc_interrupt(int irq, void *context, FAR void *arg)
{
  ((NRF_RTC_Type *)RTC_BASE)->EVENTS_TICK = 0;
  sched_process_timer();
  return OK;
}
#endif
/************************************************************************************
 * Public Functions
 ************************************************************************************/

int up_rtc_initialize(void)
{
  nrf52_clock_init();
  /* Select the lower power external 32,768Hz (Low-Speed External, LSE) oscillator
   * as RTC Clock Source and enable the Clock */
  nrf52_clock_lsclk_start();

  /* TODO: Get state from this function, if everything is
  *   okay and whether it is already enabled (if it was disabled
  *   reset upper time register)
  */
  /* Configure prescaler, note that these are write-only registers */
  ((NRF_RTC_Type *)RTC_BASE)->PRESCALER = (RTC_DEFAULT_CONFIG_FREQUENCY / CLOCKS_PER_SEC);

  /* Enable RTC Tick interrupt */
  ((NRF_RTC_Type *)RTC_BASE)->INTENSET = NRF_RTC_INT_TICK_MASK;

  g_rtc_enabled = true;

  /* Enable RTC tick */
  ((NRF_RTC_Type *)RTC_BASE)->TASKS_START = 1;

  return OK;
}

#if defined(CONFIG_RTC_TICKS)
void arm_timer_initialize(void)
{
  irqstate_t flag;
  flag = enter_critical_section();

  /* Configure RTC interrupt to catch overflow and alarm interrupts. */
  irq_attach(RTC_IRQ, nrf52_rtc_interrupt, NULL);
  up_enable_irq(RTC_IRQ);
#ifdef CONFIG_ARCH_IRQPRIO
  /* Set the interrupt priority */
  up_prioritize_irq(RTC_IRQ, NVIC_SYSH_PRIORITY_DEFAULT);
#endif

  leave_critical_section(flag);
}
#endif

/************************************************************************************
 * Name: up_rtc_time
 *
 * Description:
 *   Get the current time in seconds.  This is similar to the standard time()
 *   function.  This interface is only required if the low-resolution RTC/counter
 *   hardware implementation selected.  It is only used by the RTOS during
 *   initialization to set up the system time when CONFIG_RTC is set but neither
 *   CONFIG_RTC_HIRES nor CONFIG_RTC_DATETIME are set.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current time in seconds
 *
 ************************************************************************************/

#ifndef CONFIG_RTC_HIRES
time_t up_rtc_time(void)
{
  uint32_t counter;
  time_t   second;

  /* get counter firstly , then convert it to second */
  counter = ((NRF_RTC_Type *)RTC_BASE)->COUNTER;

  second = counter * CONFIG_USEC_PER_TICK / USEC_PER_SEC;

  return second;
}
#endif


