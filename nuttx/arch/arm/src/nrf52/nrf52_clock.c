/****************************************************************************
  * arch/arm/src/nrf52/nrf52_clock.c
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

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

#include "nrf.h"
#include "nrf_clock.h"

#if defined(CONFIG_NRF52_LSCLK_RC_CLOCK)
#define CONFIG_CLOCK_LF_SRC  NRF_CLOCK_LFCLK_RC
#elif defined(CONFIG_NRF52_LSCLK_XTAL_CLOCK)
#define CONFIG_CLOCK_LF_SRC  NRF_CLOCK_LFCLK_Xtal
#elif defined(CONFIG_NRF52_LSCLK_HS_CLOCK)
#define CONFIG_CLOCK_LF_SRC  NRF_CLOCK_LFCLK_Synth
#elif defined(CONFIG_NRF52_LSCLK_XTAL_FULL_SWING)
#define CONFIG_CLOCK_LF_SRC  NRF_CLOCK_LFCLK_Xtal
#define ENABLE_EXTERNAL 0x01
#define ENABLE_BYPASS 0x01
#else
#error "No define  LS Clock Source.\n"
#endif


/****************************************************************************
 * Private Functions
 ****************************************************************************/


/****************************************************************************
 * Name: nrf52_clock_init
 *
 * Description    : Initialize the Low frequency 32.768KHz clock
 *
 ****************************************************************************/
void nrf52_clock_init(void)
{
  static bool init;

  if (!init)
    {
      init = true;
#if defined(CONFIG_NRF52_LSCLK_RC_CLOCK) | defined(CONFIG_NRF52_LSCLK_XTAL_CLOCK) | defined(CONFIG_NRF52_LSCLK_HS_CLOCK)
      nrf_clock_lf_src_set((nrf_clock_lfclk_t)CONFIG_CLOCK_LF_SRC);
#elif defined(CONFIG_NRF52_LSCLK_XTAL_FULL_SWING)
      nrf_clock_lf_src_set_with_byp_and_external((nrf_clock_lfclk_t)CONFIG_CLOCK_LF_SRC, ENABLE_EXTERNAL, ENABLE_BYPASS);
#endif
    }

  return;
}

/****************************************************************************
 * Name: nrf52_clock_lsclk_start
 *
 * Description    : Start the low frequeny clock
 *
 ****************************************************************************/
void nrf52_clock_lsclk_start(void)
{
  nrf52_clock_init();

#ifdef CONFIG_NRF52_LSCLK_XTAL_CLOCK
  if (false == nrf_clock_lf_is_running())
    {
      nrf_clock_task_trigger(NRF_CLOCK_TASK_LFCLKSTART);

      do {}
      while (!nrf_clock_event_check(NRF_CLOCK_EVENT_LFCLKSTARTED));
      nrf_clock_event_clear(NRF_CLOCK_EVENT_LFCLKSTARTED);
    }
#endif

}

/****************************************************************************
 * Name: nrf52_clock_lsclk_stop
 *
 * Description    : Stop the low frequency clock
 *
 ****************************************************************************/
void nrf52_clock_lsclk_stop(void)
{
#ifdef CONFIG_NRF52_LSCLK_XTAL_CLOCK
  nrf_clock_task_trigger(NRF_CLOCK_TASK_LFCLKSTOP);
  while (nrf_clock_lf_is_running())
    {}
#endif
}

/****************************************************************************
 * Name: nrf52_clock_hsclk_start
 *
 * Description    : Start the High frequency clock
 *
 ****************************************************************************/
void nrf52_clock_hsclk_start(void)
{
  if (false == nrf_clock_hf_is_running(NRF_CLOCK_HFCLK_HIGH_ACCURACY))
    {
      nrf_clock_task_trigger(NRF_CLOCK_TASK_HFCLKSTART);

      do {}
      while (!nrf_clock_event_check(NRF_CLOCK_EVENT_HFCLKSTARTED));
      nrf_clock_event_clear(NRF_CLOCK_EVENT_HFCLKSTARTED);
    }
}

/****************************************************************************
 * Name: nrf52_clock_hsclk_stop
 *
 * Description    : Stop the high frequency clock
 *
 ****************************************************************************/
void nrf52_clock_hsclk_stop(void)
{
  nrf_clock_task_trigger(NRF_CLOCK_TASK_HFCLKSTOP);
  while (nrf_clock_hf_is_running(NRF_CLOCK_HFCLK_HIGH_ACCURACY))
    {}
}
