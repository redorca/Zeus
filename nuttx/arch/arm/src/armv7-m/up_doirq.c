/****************************************************************************
 * arch/arm/src/armv7-m/up_doirq.c
 *
 *   Copyright (C) 2009, 2011, 2013-2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"

#ifdef CONFIG_SEGGER_SYSVIEW
#include <sysview/SEGGER_SYSVIEW.h>
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t *up_doirq(int irq, uint32_t *regs)
{
  board_autoled_on(LED_INIRQ);
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  PANIC();
#else
  uint32_t *savestate;

  /* Nested interrupts are not supported in this implementation.  If you want
   * to implement nested interrupts, you would have to (1) change the way that
   * CURRENT_REGS is handled and (2) the design associated with
   * CONFIG_ARCH_INTERRUPTSTACK.  The savestate variable will not work for
   * that purpose as implemented here because only the outermost nested
   * interrupt can result in a context switch.
   */

  /* Current regs non-zero indicates that we are processing an interrupt;
   * CURRENT_REGS is also used to manage interrupt level context switches.
   */

  savestate    = (uint32_t *)CURRENT_REGS;
  CURRENT_REGS = regs;

#ifdef CONFIG_SEGGER_SYSVIEW
  SEGGER_SYSVIEW_RecordEnterISR();
#endif

  /* Acknowledge the interrupt */

  up_ack_irq(irq);

  /* Deliver the IRQ */

  irq_dispatch(irq, regs);

  /* If a context switch occurred while processing the interrupt then
   * CURRENT_REGS may have change value.  If we return any value different
   * from the input regs, then the lower level will know that a context
   * switch occurred during interrupt processing.
   */

#ifdef CONFIG_SEGGER_SYSVIEW
  SEGGER_SYSVIEW_RecordExitISR();
#endif

  regs = (uint32_t *)CURRENT_REGS;

  /* Restore the previous value of CURRENT_REGS.  NULL would indicate that
   * we are no longer in an interrupt handler.  It will be non-NULL if we
   * are returning from a nested interrupt.
   */

  CURRENT_REGS = savestate;
#endif
  board_autoled_off(LED_INIRQ);
  return regs;
}
