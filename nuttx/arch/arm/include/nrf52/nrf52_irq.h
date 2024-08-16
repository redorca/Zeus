/****************************************************************************************************
 * arch/arm/include/nrf52/nrf52_irq.h
 *
 *   Copyright (C) 2015 Sebastien Lorquet. All rights reserved.
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
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
 ****************************************************************************************************/

/* This file should never be included directed but, rather, only indirectly through arch/irq.h */

#ifndef __ARCH_ARM_INCLUDE_NRF52_NRF52_IRQ_H
#define __ARCH_ARM_INCLUDE_NRF52_NRF52_IRQ_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/
/* IRQ numbers.  The IRQ number corresponds vector number and hence map directly to bits in the
 * NVIC.  This does, however, waste several words of memory in the IRQ to handle mapping tables.
 *
 * Processor Exceptions (vectors 0-15).  These common definitions can be found in the file
 * nuttx/arch/arm/include/nrf52/irq.h which includes this file
 *
 * External interrupts (vectors >= 16)
 */

/* -------------------------  Interrupt Number Definition  ------------------------ */

#ifndef __ASSEMBLY__
typedef enum
{

  /* ----------------------  nrf52 Specific Interrupt Numbers  ----------------------   */
  POWER_CLOCK_IRQn = NRF52_IRQ_EXTINT,  /*   0  POWER_CLOCK     */
  RADIO_IRQn,                /*   1  RADIO                     */
  UARTE0_UART0_IRQn,         /*   2  UARTE0_UART0              */
  SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn, /*   3  SPIM0,SPIS0,TWIM0,TWIS0,SPI0,TWI0   */
  SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, /*   4  SPIM1,SPIS1,TWIM1,TWIS1,SPI1,TWI1   */
  NFCT_IRQn,                 /*   5  NFCT                      */
  GPIOTE_IRQn,               /*   6  GPIOTE                    */
  SAADC_IRQn,                /*   7  SAADC                     */
  TIMER0_IRQn,               /*   8  TIMER0                    */
  TIMER1_IRQn,               /*   9  TIMER1                    */
  TIMER2_IRQn,               /*  10  TIMER2                    */
  RTC0_IRQn,                 /*  11  RTC0                      */
  TEMP_IRQn,                 /*  12  TEMP                      */
  RNG_IRQn,                  /*  13  RNG                       */
  ECB_IRQn,                  /*  14  ECB                       */
  CCM_AAR_IRQn,              /*  15  CCM_AAR                   */
  WDT_IRQn,                  /*  16  WDT                       */
  RTC1_IRQn,                 /*  17  RTC1                      */
  QDEC_IRQn,                 /*  18  QDEC                      */
  COMP_LPCOMP_IRQn,          /*  19  COMP_LPCOMP               */
  SWI0_EGU0_IRQn,            /*  20  SWI0_EGU0                 */
  SWI1_EGU1_IRQn,            /*  21  SWI1_EGU1                 */
  SWI2_EGU2_IRQn,            /*  22  SWI2_EGU2                 */
  SWI3_EGU3_IRQn,            /*  23  SWI3_EGU3                 */
  SWI4_EGU4_IRQn,            /*  24  SWI4_EGU4                 */
  SWI5_EGU5_IRQn,            /*  25  SWI5_EGU5                 */
  TIMER3_IRQn,               /*  26  TIMER3                    */
  TIMER4_IRQn,               /*  27  TIMER4                    */
  PWM0_IRQn,                 /*  28  PWM0                      */
  PDM_IRQn,                  /*  29  PDM                       */
  BLANK5,                    /*  30  */
  BLANK6,                    /*  31  */
  MWU_IRQn,                  /*  32  MWU                       */
  PWM1_IRQn,                 /*  33  PWM1                      */
  PWM2_IRQn,                 /*  34  PWM2                      */
  SPIM2_SPIS2_SPI2_IRQn,     /*  35  SPIM2_SPIS2_SPI2          */
  RTC2_IRQn,                 /*  36  RTC2                      */
  I2S_IRQn,                  /*  37  I2S                       */
  FPU_IRQn,                  /*  38  FPU                       */
#if defined (CONFIG_ARCH_CHIP_NRF52840)
  USBD_IRQn,                 /*!<  39  USBD                    */
  UARTE1_IRQn,               /*!<  40  UARTE1                  */
  QSPI_IRQn,                 /*!<  41  QSPI                    */
  CRYPTOCELL_IRQn,           /*!<  42  CRYPTOCELL              */
  PWM3_IRQn,                 /*!<  45  PWM3                    */
  SPIM3_IRQn,                /*!<  47  SPIM3                   */
#endif
  LAST_IRQn,
} IRQn_Type;
#endif

#if defined (CONFIG_ARCH_CHIP_NRF52840)
#define NR_INTERRUPTS         (48)
#else
#define NR_INTERRUPTS         (39)
#endif

#define NR_VECTORS            (NRF52_IRQ_EXTINT + NR_INTERRUPTS)

/* EXTI interrupts (Do not use IRQ numbers) */

#define NR_IRQS               NR_VECTORS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_NRF52_NRF52_IRQ_H */
