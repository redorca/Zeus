/****************************************************************************
 * configs/nrf52840_dk/src/nrf52_buttons.c
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2016-2018 Zglue Inc. All rights reserved.
 *   Author: Levin Li   <zhiqiang@zglue.com>
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
#include <errno.h>
#include "nrf.h"
#include "nrf52_gpio.h"

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>


#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* Pin configuration for each button.  This array is indexed by
 * the BUTTON_* definitions in board.h
 */
#if BUTTONS_NUMBER > 0
static const uint8_t m_board_btn_list[BUTTONS_NUMBER] = BUTTONS_LIST;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.
 *   After that, board_buttons() may be called to collect the current state
 *   of all buttons or board_button_irq() may be called to register button
 *   interrupt handlers.
 *
 ****************************************************************************/

void board_button_initialize(void)
{
  int i;

  /* Configure the GPIO pins as inputs.  NOTE that EXTI interrupts are
   * configured for all pins.
   */

  for (i = 0; i < BUTTONS_NUMBER; i++)
    {
      nrf_gpio_cfg_output(m_board_btn_list[i]);
    }
}

/****************************************************************************
 * Name: board_buttons
 ****************************************************************************/

uint8_t board_buttons(void)
{
  uint8_t ret = 0;
  int i;

  /* Check that state of each key */

  for (i = 0; i < BUTTONS_NUMBER; i++)
    {
      /* A LOW value means that the key is pressed. */

      bool released = nrf_gpio_pin_read(m_board_btn_list[i]);

      /* Accumulate the set of depressed (not released) keys */

      if (!released)
        {
          ret |= (1 << i);
        }
    }

  return ret;
}

/*****************************************************************************
 * Button support.
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.
 *   After that, board_buttons() may be called to collect the current state
 *   of all buttons or board_button_irq() may be called to register button
 *   interrupt handlers.
 *
 *   After board_button_initialize() has been called, board_buttons() may be
 *   called to collect the state of all buttons.  board_buttons() returns an
 *   8-bit bit set with each bit associated with a button.  See the
 *   BUTTON_*_BIT definitions in board.h for the meaning of each bit.
 *
 *   board_button_irq() may be called to register an interrupt handler that
 *   will be called when a button is depressed or released.  The ID value
 *   is a button enumeration value that uniquely identifies a button resource.
 *   See the BUTTON_* definitions in board.h for the meaning of enumeration
 *   value.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, FAR void *arg)
{
  int ret = -EINVAL;

  if (id >= MIN_IRQBUTTON && id <= MAX_IRQBUTTON)
    {
      ret = nrf52_gpiosetevent(m_board_btn_list[id], true, true, true,
                               irqhandler, arg);
    }

  return ret;
}
#endif
#endif /* CONFIG_ARCH_BUTTONS */
