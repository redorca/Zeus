/****************************************************************************
 * arch/arm/src/nrf52_/nrf52_irq.c
 *
 *   Copyright (C) 2009-2016 Gregory Nutt. All rights reserved.
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
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>
#include "chip.h"

#include "nvic.h"
#include "ram_vectors.h"
#include "up_arch.h"
#include "up_internal.h"
#include "nrf.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Get a 32-bit version of the default priority */

#define DEFPRIORITY32 \
  (NVIC_SYSH_PRIORITY_DEFAULT << 24 | \
   NVIC_SYSH_PRIORITY_DEFAULT << 16 | \
   NVIC_SYSH_PRIORITY_DEFAULT << 8  | \
   NVIC_SYSH_PRIORITY_DEFAULT)

/* Given the address of a NVIC ENABLE register, this is the offset to
 * the corresponding CLEAR ENABLE register.
 */

#define NVIC_ENA_OFFSET    (0)
#define NVIC_CLRENA_OFFSET (NVIC_IRQ0_31_CLEAR - NVIC_IRQ0_31_ENABLE)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_current_regs[] holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the macro
 * CURRENT_REGS for portability.
 */

volatile uint32_t *g_current_regs[1];

/* This is the address of the  exception vector table (determined by the
 * linker script).
 */

extern uint32_t _vectors[];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_dumpnvic
 *
 * Description:
 *   Dump some interesting NVIC registers
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_IRQ_INFO)
static void nrf52_dumpnvic(const char *msg, int irq)
{
  irqstate_t flags;

  flags = enter_critical_section();

  irqinfo("NVIC (%s, irq=%d):\n", msg, irq);
  irqinfo("  INTCTRL:    %08x VECTAB:  %08x\n",
          getreg32(NVIC_INTCTRL), getreg32(NVIC_VECTAB));
  irqinfo("  SYSH ENABLE MEMFAULT: %08x BUSFAULT: %08x USGFAULT: %08x SYSTICK: %08x\n",
          getreg32(NVIC_SYSHCON_MEMFAULTENA), getreg32(NVIC_SYSHCON_BUSFAULTENA),
          getreg32(NVIC_SYSHCON_USGFAULTENA), getreg32(NVIC_SYSTICK_CTRL_ENABLE));
  irqinfo("  IRQ ENABLE: %08x %08x %08x\n",
          getreg32(NVIC_IRQ0_31_ENABLE), getreg32(NVIC_IRQ32_63_ENABLE),
          getreg32(NVIC_IRQ64_95_ENABLE));
  irqinfo("  SYSH_PRIO:  %08x %08x %08x\n",
          getreg32(NVIC_SYSH4_7_PRIORITY), getreg32(NVIC_SYSH8_11_PRIORITY),
          getreg32(NVIC_SYSH12_15_PRIORITY));
  irqinfo("  IRQ PRIO:   %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ0_3_PRIORITY), getreg32(NVIC_IRQ4_7_PRIORITY),
          getreg32(NVIC_IRQ8_11_PRIORITY), getreg32(NVIC_IRQ12_15_PRIORITY));
  irqinfo("              %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ16_19_PRIORITY), getreg32(NVIC_IRQ20_23_PRIORITY),
          getreg32(NVIC_IRQ24_27_PRIORITY), getreg32(NVIC_IRQ28_31_PRIORITY));
  irqinfo("              %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ32_35_PRIORITY), getreg32(NVIC_IRQ36_39_PRIORITY),
          getreg32(NVIC_IRQ40_43_PRIORITY), getreg32(NVIC_IRQ44_47_PRIORITY));
  irqinfo("              %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ48_51_PRIORITY), getreg32(NVIC_IRQ52_55_PRIORITY),
          getreg32(NVIC_IRQ56_59_PRIORITY), getreg32(NVIC_IRQ60_63_PRIORITY));
  irqinfo("              %08x\n",
          getreg32(NVIC_IRQ64_67_PRIORITY));

  leave_critical_section(flags);
}
#else
#  define nrf52_dumpnvic(msg, irq)
#endif

/****************************************************************************
 * Name: nrf52_nmi, nrf52_busfault, nrf52_usagefault, nrf52_pendsv,
 *       nrf52_dbgmonitor, nrf52_pendsv, nrf52_reserved
 *
 * Description:
 *   Handlers for various execptions.  None are handled and all are fatal
 *   error conditions.  The only advantage these provided over the default
 *   unexpected interrupt handler is that they provide a diagnostic output.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
static int nrf52_nmi(int irq, FAR void *context, FAR void *arg)
{
  (void)up_irq_save();
  _err("PANIC!!! NMI received\n");
  PANIC();
  return 0;
}

static int nrf52_busfault(int irq, FAR void *context, FAR void *arg)
{
  (void)up_irq_save();
  _err("PANIC!!! Bus fault received: %08x\n", getreg32(NVIC_CFAULTS));
  PANIC();
  return 0;
}

static int nrf52_usagefault(int irq, FAR void *context, FAR void *arg)
{
  (void)up_irq_save();
  _err("PANIC!!! Usage fault received: %08x\n", getreg32(NVIC_CFAULTS));
  PANIC();
  return 0;
}

static int nrf52_pendsv(int irq, FAR void *context, FAR void *arg)
{
  (void)up_irq_save();
  _err("PANIC!!! PendSV received\n");
  PANIC();
  return 0;
}

static int nrf52_dbgmonitor(int irq, FAR void *context, FAR void *arg)
{
  (void)up_irq_save();
  _err("PANIC!!! Debug Monitor received\n");
  PANIC();
  return 0;
}

static int nrf52_reserved(int irq, FAR void *context, FAR void *arg)
{
  (void)up_irq_save();
  _err("PANIC!!! Reserved interrupt\n");
  PANIC();
  return 0;
}
#endif

/****************************************************************************
 * Name: nrf52_prioritize_syscall
 *
 * Description:
 *   Set the priority of an exception.  This function may be needed
 *   internally even if support for prioritized interrupts is not enabled.
 *
 ****************************************************************************/

#ifdef CONFIG_ARMV7M_USEBASEPRI
static inline void nrf52_prioritize_syscall(int priority)
{
  uint32_t regval;

  /* SVCALL is system handler 11 */

  regval  = getreg32(NVIC_SYSH8_11_PRIORITY);
  regval &= ~NVIC_SYSH_PRIORITY_PR11_MASK;
  regval |= (priority << NVIC_SYSH_PRIORITY_PR11_SHIFT);
  putreg32(regval, NVIC_SYSH8_11_PRIORITY);
}
#endif

/****************************************************************************
 * Name: nrf52_irqinfo
 *
 * Description:
 *   Given an IRQ number, provide the register and bit setting to enable or
 *   disable the irq.
 *
 ****************************************************************************/

static int nrf52_irqinfo(int irq, uintptr_t *regaddr, uint32_t *bit,
                         uintptr_t offset)
{
  int n;

  DEBUGASSERT(irq >= NRF52_IRQ_NMI && irq < NR_IRQS);

  /* Check for external interrupt */

  if (irq >= NRF52_IRQ_EXTINT)
    {
      n        = irq - NRF52_IRQ_EXTINT;
      *regaddr = NVIC_IRQ_ENABLE(n) + offset;
      *bit     = (uint32_t)1 << (n & 0x1f);
    }

  /* Handle processor exceptions.  Only a few can be disabled */

  else
    {
      *regaddr = NVIC_SYSHCON;
      if (irq == NRF52_IRQ_MEMFAULT)
        {
          *bit = NVIC_SYSHCON_MEMFAULTENA;
        }
      else if (irq == NRF52_IRQ_BUSFAULT)
        {
          *bit = NVIC_SYSHCON_BUSFAULTENA;
        }
      else if (irq == NRF52_IRQ_USAGEFAULT)
        {
          *bit = NVIC_SYSHCON_USGFAULTENA;
        }
      else if (irq == NRF52_IRQ_SYSTICK)
        {
          *regaddr = NVIC_SYSTICK_CTRL;
          *bit = NVIC_SYSTICK_CTRL_ENABLE;
        }
      else
        {
          return ERROR; /* Invalid or unsupported exception */
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  uint32_t regaddr;
  int num_priority_registers;
  int i;

  /* Disable all interrupts & systick
   * systick may be enabled from bootloader
   */
  up_disable_irq(NRF52_IRQ_SYSTICK);
  putreg32(NVIC_INTCTRL_PENDSTCLR, NVIC_INTCTRL);

  for (i = 0; i < NR_IRQS - NRF52_IRQ_EXTINT; i += 32)
    {
      putreg32(0xffffffff, NVIC_IRQ_CLEAR(i));
    }

  /* The standard location for the vector table is at the beginning of FLASH
   * at address 0x0800:0000.  If we are using the STMicro DFU bootloader, then
   * the vector table will be offset to a different location in FLASH and we
   * will need to set the NVIC vector location to this alternative location.
   */

#if defined(CONFIG_NRF52_BLUETOOTH)
  /* The bluetooth option incurs loading of a softdevice which
   * expects to use its vector table rather than nuttx's.  This
   * means that the vector table register should be 0 and not
   * the address of the nuttx _vectors table.
   */
  putreg32((uint32_t)_vectors, NVIC_VECTAB);
  putreg32((uint32_t)0, NVIC_VECTAB); //XXX Add this line to assign vector table from 0x00 for softdevice
#else
  putreg32((uint32_t)_vectors, NVIC_VECTAB);
#endif

#ifdef CONFIG_ARCH_RAMVECTORS
  /* If CONFIG_ARCH_RAMVECTORS is defined, then we are using a RAM-based
   * vector table that requires special initialization.
   */

  up_ramvec_initialize();
#endif

  /* Set all interrupts (and exceptions) to the default priority */

  putreg32(DEFPRIORITY32, NVIC_SYSH4_7_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_SYSH12_15_PRIORITY);

#ifndef CONFIG_NRF52_BLUETOOTH
  /* The priorities of the interrupts reserved by the SoftDevice cannot be changed.
   *  This includes the SVC interrupt. */

  putreg32(DEFPRIORITY32, NVIC_SYSH8_11_PRIORITY);
#endif


#if defined(CONFIG_NRF52_BLUETOOTH)
  /* The NVIC ICTR register (bits 0-4) holds the number of of interrupt
   * lines that the NVIC supports:
   *
   *  0 -> 32 interrupt lines,  8 priority registers
   *  1 -> 64 "       " "   ", 16 priority registers
   *  2 -> 96 "       " "   ", 32 priority registers
   *  ...
   */

  num_priority_registers = (getreg32(NVIC_ICTR) + 1) * 32;

  uint8_t nrf52_softdev[] =
  {
    RADIO_IRQn,
    TIMER0_IRQn,   /* timer0 */
    RTC0_IRQn,
    CCM_AAR_IRQn, /* CCM && AAR */
    SWI2_EGU2_IRQn,
    SWI4_EGU4_IRQn,
    SWI5_EGU5_IRQn,
  };

  /* Now set all of the interrupt lines to the default priority
   * except softdevice require irq */

  regaddr = NVIC_IRQ0_3_PRIORITY;
  int irq = 0;
  while (irq < num_priority_registers)
    {
      uint8_t flag = true;
      for (i = 0; i < sizeof(nrf52_softdev) / sizeof(uint8_t); i++)
        {
          if ((irq + NRF52_IRQ_EXTINT) == nrf52_softdev[i])
            {
              flag = false;
              break;
            }
        }
      if (flag)
        {
          putreg8((uint8_t)DEFPRIORITY32, regaddr);
        }
      irq++;
      regaddr += 1;
    }

#else
  /* The NVIC ICTR register (bits 0-4) holds the number of of interrupt
   * lines that the NVIC supports:
   *
   *  0 -> 32 interrupt lines,  8 priority registers
   *  1 -> 64 "       " "   ", 16 priority registers
   *  2 -> 96 "       " "   ", 32 priority registers
   *  ...
   */

  num_priority_registers = (getreg32(NVIC_ICTR) + 1) * 8;

  /* Now set all of the interrupt lines to the default priority */

  regaddr = NVIC_IRQ0_3_PRIORITY;
  while (num_priority_registers--)
    {
      putreg32(DEFPRIORITY32, regaddr);
      regaddr += 4;
    }
#endif
  /* currents_regs is non-NULL only while processing an interrupt */

  CURRENT_REGS = NULL;

  /* Attach the SVCall and Hard Fault exception handlers.  The SVCall
   * exception is used for performing context switches; The Hard Fault
   * must also be caught because a SVCall may show up as a Hard Fault
   * under certain conditions.
   */

  irq_attach(NRF52_IRQ_SVCALL, up_svcall, NULL);
  irq_attach(NRF52_IRQ_HARDFAULT, up_hardfault, NULL);

  /* Set the priority of the SVCall interrupt */

#ifdef CONFIG_ARCH_IRQPRIO
  /* up_prioritize_irq(NRF52_IRQ_PENDSV, NVIC_SYSH_PRIORITY_MIN); */
#endif
#if defined(CONFIG_ARMV7M_USEBASEPRI) && !defined(CONFIG_NRF52_BLUETOOTH)
  nrf52_prioritize_syscall(NVIC_SYSH_SVCALL_PRIORITY);
#endif

  /* If the MPU is enabled, then attach and enable the Memory Management
   * Fault handler.
   */

#ifdef CONFIG_ARM_MPU
  irq_attach(NRF52_IRQ_MEMFAULT, up_memfault, NULL);
  up_enable_irq(NRF52_IRQ_MEMFAULT);
#endif

  /* Attach all other processor exceptions (except reset and sys tick) */

#ifdef CONFIG_DEBUG_FEATURES
  irq_attach(NRF52_IRQ_NMI, nrf52_nmi, NULL);
#ifndef CONFIG_ARM_MPU
  irq_attach(NRF52_IRQ_MEMFAULT, up_memfault, NULL);
#endif
  irq_attach(NRF52_IRQ_BUSFAULT, nrf52_busfault, NULL);
  irq_attach(NRF52_IRQ_USAGEFAULT, nrf52_usagefault, NULL);
  irq_attach(NRF52_IRQ_PENDSV, nrf52_pendsv, NULL);
  irq_attach(NRF52_IRQ_DBGMONITOR, nrf52_dbgmonitor, NULL);
  irq_attach(NRF52_IRQ_RESERVED, nrf52_reserved, NULL);
#endif

  nrf52_dumpnvic("initial", NR_IRQS);

#ifndef CONFIG_SUPPRESS_INTERRUPTS

  /* And finally, enable interrupts */

  up_irq_enable();
#endif
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t bit;

  if (nrf52_irqinfo(irq, &regaddr, &bit, NVIC_CLRENA_OFFSET) == 0)
    {
      /* Modify the appropriate bit in the register to disable the interrupt.
       * For normal interrupts, we need to set the bit in the associated
       * Interrupt Clear Enable register.  For other exceptions, we need to
       * clear the bit in the System Handler Control and State Register.
       */

      if (irq >= NRF52_IRQ_EXTINT)
        {
          putreg32(bit, regaddr);
        }
      else
        {
          regval  = getreg32(regaddr);
          regval &= ~bit;
          putreg32(regval, regaddr);
        }
    }

  // nrf52_dumpnvic("disable", irq);
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t bit;

  if (nrf52_irqinfo(irq, &regaddr, &bit, NVIC_ENA_OFFSET) == 0)
    {
      /* Modify the appropriate bit in the register to enable the interrupt.
       * For normal interrupts, we need to set the bit in the associated
       * Interrupt Set Enable register.  For other exceptions, we need to
       * set the bit in the System Handler Control and State Register.
       */

      if (irq >= NRF52_IRQ_EXTINT)
        {
          putreg32(bit, regaddr);
        }
      else
        {
          regval  = getreg32(regaddr);
          regval |= bit;
          putreg32(regval, regaddr);
        }
    }

  // nrf52_dumpnvic("enable", irq);
}

/****************************************************************************
 * Name: up_ack_irq
 *
 * Description:
 *   Acknowledge the IRQ (Clear the bit)
 *
 ****************************************************************************/

void up_ack_irq(int irq)
{

  /* This will be called on each interrupt exit whether the interrupt can be
   * enambled or not.  So this assertion is necessarily lame.
   */

  DEBUGASSERT((unsigned)irq < NR_IRQS);

  /* Check for an external interrupt */

  if ((irq >= NRF52_IRQ_EXTINT) && (irq < LAST_IRQn))
    {
      // putreg32(1 << ((irq - NRF52_IRQ_EXTINT) & 0x1F), NVIC_IRQ_CLEAR(irq));
      putreg32(1 << ((irq - NRF52_IRQ_EXTINT) & 0x1F), NVIC_IRQ_CLRPEND(irq));
    }
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{
  uint32_t regaddr;
  uint32_t regval;
  int shift;

  DEBUGASSERT(irq >= NRF52_IRQ_MEMFAULT && irq < NR_IRQS &&
              (unsigned)priority <= NVIC_SYSH_PRIORITY_MIN);

  if (irq < NRF52_IRQ_EXTINT)
    {
      /* NVIC_SYSH_PRIORITY() maps {0..15} to one of three priority
       * registers (0-3 are invalid)
       */

      regaddr = NVIC_SYSH_PRIORITY(irq);
      irq    -= 4;
    }
  else
    {
      /* NVIC_IRQ_PRIORITY() maps {0..} to one of many priority registers */

      irq    -= NRF52_IRQ_EXTINT;
      regaddr = NVIC_IRQ_PRIORITY(irq);
    }

  regval      = getreg32(regaddr);
  shift       = ((irq & 3) << 3);
  regval     &= ~(0xff << shift);
  regval     |= (priority << shift);
  putreg32(regval, regaddr);

#ifdef CONFIG_DEBUG_IRQ_INFO
  nrf52_dumpnvic("prioritize", irq);
#endif

  return OK;
}
#endif

void nrf_drv_common_irq_enable(IRQn_Type IRQn, uint8_t priority)
{
  irqstate_t flags;

  flags = enter_critical_section();

  ASSERT(priority < 8);

  /* nrf52832 only had 3bits for priority */

  priority <<= 5;

#ifdef CONFIG_ARCH_IRQPRIO
  up_prioritize_irq(IRQn, priority);
#endif
  up_ack_irq(IRQn);
  up_enable_irq(IRQn);

  leave_critical_section(flags);
}

