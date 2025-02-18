/****************************************************************************
 *  arch/arm/src/nrf52/nrf52_idle.c
 *
 *   Copyright (C) 2011-2012, 2015-2016 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2017-2018 Zglue Inc. All rights reserved.
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

#include <arch/board/board.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/board.h>
#include <nuttx/power/pm.h>

#include "nvic.h"
#include "cache.h"
#include "chip.h"
#include "nrf.h"
#include "nrf_power.h"
#include "up_internal.h"
#include "up_arch.h"
#ifdef CONFIG_SEGGER_SYSVIEW
#include <sysview/SEGGER_SYSVIEW.h>
#endif
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Does the board support an IDLE LED to indicate that the board is in the
 * IDLE state?
 */

#if defined(CONFIG_ARCH_LEDS) && defined(LED_IDLE)
#  define BEGIN_IDLE() board_autoled_on(LED_IDLE)
#  define END_IDLE()   board_autoled_off(LED_IDLE)
#else
#  define BEGIN_IDLE()
#  define END_IDLE()
#endif

#define PM_IDLE_DOMAIN 0 /* Revisit */

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static void issue_low_power_command(void)
{
  uint32_t regval;

  /* firstly enable deep sleep mode for CPU */
  regval  = getreg32(NVIC_SYSCON);

  regval |= NVIC_SYSCON_SEVONPEND;

  regval |= NVIC_SYSCON_SLEEPDEEP;

  putreg32(regval, NVIC_SYSCON);

  /* let CPU enter  wait for event mode */
  __asm("sev");
  __asm("wfe");
  __asm("wfe");
}

#ifdef CONFIG_PM
static void low_power_mode(enum pm_state_e state)
{

  irqinfo("Enter low power mode: %c\n", '0' + state);
  switch (state)
    {
      case PM_IDLE:
        nrf_power_task_trigger(NRF_POWER_TASK_CONSTLAT);
        issue_low_power_command();
        break;
      case PM_STANDBY: /* sleep mode */
        nrf_power_task_trigger(NRF_POWER_TASK_LOWPWR);
        issue_low_power_command();
        break;
      case PM_SLEEP: /* deep sleep mode */
        nrf_power_system_off();
        break;
      default:
        break;
    }

  irqinfo("Leave low power mode: %c\n");

}

/****************************************************************************
 * Name: up_idlepm
 *
 * Description:
 *   Perform IDLE state power management.
 *
 ****************************************************************************/

static void up_idlepm(void)
{
  static enum pm_state_e oldstate = PM_NORMAL;
  enum pm_state_e newstate;
  irqstate_t flags;
  int ret;

  /* Decide, which power saving level can be obtained */

  newstate = pm_checkstate(PM_IDLE_DOMAIN);

  /* Check for state changes */

  if (newstate != oldstate)
    {
      flags = enter_critical_section();

      /* Perform board-specific, state-dependent logic here */

      _info("newstate= %d oldstate=%d\n", newstate, oldstate);

      /* Then force the global state change */

      ret = pm_changestate(PM_IDLE_DOMAIN, newstate);
      if (ret < 0)
        {
          /* The new state change failed, revert to the preceding state */

          (void)pm_changestate(PM_IDLE_DOMAIN, oldstate);
        }
      else
        {
          /* Save the new state */

          oldstate = newstate;
        }

      /* MCU-specific power management logic */

      low_power_mode(newstate);

      leave_critical_section(flags);
    }
}
#else
void up_idlepm(void)
{
  /* Sleep until an interrupt occurs to save power */
  nrf_power_task_trigger(NRF_POWER_TASK_LOWPWR);

  issue_low_power_command();
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_idle
 *
 * Description:
 *   up_idle() is the logic that will be executed when their is no other
 *   ready-to-run task.  This is processor idle time and will continue until
 *   some interrupt occurs to cause a context switch from the idle task.
 *
 *   Processing in this state may be processor-specific. e.g., this is where
 *   power management operations might be performed.
 *
 ****************************************************************************/



void up_idle(void)
{

#ifdef CONFIG_SEGGER_SYSVIEW
  {
    /* temp put systemview init here */
    static bool view_init;
    if (0 == view_init)
      {
        view_init = true;
        SEGGER_SYSVIEW_Conf();
      }
  }
#endif

#if defined(CONFIG_SUPPRESS_INTERRUPTS) || defined(CONFIG_SUPPRESS_TIMER_INTS)
  /* If the system is idle and there are no timer interrupts, then process
   * "fake" timer interrupts. Hopefully, something will wake up.
   */

  sched_process_timer();
#else

  /*
   * SEGGER_SYSVIEW_OnIdle();
   */

  /* Perform IDLE mode power management */
#if defined(CONFIG_SCHED_TICKLESS) || defined(CONFIG_RTC_TICKS)
  /* only enable low power under tickless or RTC_TICK
   * if Systick as system timer , the Systick will stop when enter wfi | wfe
   * or you can you sofdevice lowpower api for sleep mode
   */
  BEGIN_IDLE();
  irqinfo("Enter Low Power\n");
  up_idlepm();
  irqinfo("Leave Low Power\n");
  END_IDLE();
#endif

#endif
}
