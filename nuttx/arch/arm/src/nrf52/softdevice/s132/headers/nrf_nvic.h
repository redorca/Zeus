/*
 * Copyright (c) Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 *   3. Neither the name of Nordic Semiconductor ASA nor the names of other
 *   contributors to this software may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 *   4. This software must only be used in a processor manufactured by Nordic
 *   Semiconductor ASA, or in a processor manufactured by a third party that
 *   is used in combination with a processor manufactured by Nordic Semiconductor.
 *
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**
 * @defgroup nrf_nvic_api SoftDevice NVIC API
 * @{
 *
 * @note In order to use this module, the following code has to be added to a .c file:
 *     \code
 *     nrf_nvic_state_t nrf_nvic_state = {0};
 *     \endcode
 *
 * @note Definitions and declarations starting with __ (double underscore) in this header file are
 * not intended for direct use by the application.
 *
 * @brief APIs for the accessing NVIC when using a SoftDevice.
 *
 */

#ifndef NRF_NVIC_H__
#define NRF_NVIC_H__

#include <stdint.h>
#include "nrf.h"
#include "chip.h"
#include "nrf_error_soc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@addtogroup NRF_NVIC_DEFINES Defines
 * @{ */

/**@defgroup NRF_NVIC_ISER_DEFINES SoftDevice NVIC internal definitions
 * @{ */

#define __NRF_NVIC_NVMC_IRQn (30) /**< The peripheral ID of the NVMC. IRQ numbers are used to identify peripherals, but the NVMC doesn't have an IRQ number in the MDK. */

#ifdef NRF51
#define __NRF_NVIC_ISER_COUNT (1) /**< The number of ISER/ICER registers in the NVIC that are used. */

/**@brief Interrupts used by the SoftDevice. */

#define __NRF_NVIC_SD_IRQS_0 ((uint32_t)( \
        (1U << POWER_CLOCK_IRQn) \
      | (1U << RADIO_IRQn) \
      | (1U << RTC0_IRQn) \
      | (1U << TIMER0_IRQn) \
      | (1U << RNG_IRQn) \
      | (1U << ECB_IRQn) \
      | (1U << CCM_AAR_IRQn) \
      | (1U << TEMP_IRQn) \
      | (1U << __NRF_NVIC_NVMC_IRQn) \
      | (1U << (uint32_t)SWI4_IRQn) \
      | (1U << (uint32_t)SWI5_IRQn) \
    ))


/**@brief Interrupts available for to application. */
#define __NRF_NVIC_APP_IRQS_0 (~__NRF_NVIC_SD_IRQS_0)
#endif

#ifdef NRF52
#define __NRF_NVIC_ISER_COUNT (2) /**< The number of ISER/ICER registers in the NVIC that are used. */

/**@brief Interrupts used by the SoftDevice. */
#define __NRF_NVIC_SD_IRQS_0 ((uint32_t)( \
        (1U << (POWER_CLOCK_IRQn-NRF52_IRQ_EXTINT)) \
      | (1U << (RADIO_IRQn-NRF52_IRQ_EXTINT)) \
      | (1U << (RTC0_IRQn-NRF52_IRQ_EXTINT)) \
      | (1U << (TIMER0_IRQn-NRF52_IRQ_EXTINT)) \
      | (1U << (RNG_IRQn-NRF52_IRQ_EXTINT)) \
      | (1U << (ECB_IRQn-NRF52_IRQ_EXTINT)) \
      | (1U << (CCM_AAR_IRQn-NRF52_IRQ_EXTINT)) \
      | (1U << (TEMP_IRQn-NRF52_IRQ_EXTINT)) \
      | (1U << (__NRF_NVIC_NVMC_IRQn-NRF52_IRQ_EXTINT)) \
      | (1U << (uint32_t)(SWI4_EGU4_IRQn-NRF52_IRQ_EXTINT)) \
      | (1U << (uint32_t)(SWI5_EGU5_IRQn-NRF52_IRQ_EXTINT)) \
    ))



#define __NRF_NVIC_SD_IRQS_1 ((uint32_t)0)

/**@brief Interrupts available for to application. */
#define __NRF_NVIC_APP_IRQS_0 (~__NRF_NVIC_SD_IRQS_0)
#define __NRF_NVIC_APP_IRQS_1 (~__NRF_NVIC_SD_IRQS_1)
#endif
/**@} */

/**@} */

/**@addtogroup NRF_NVIC_VARIABLES Variables
 * @{ */

/**@brief Type representing the state struct for the SoftDevice NVIC module. */
typedef struct
{
  uint32_t volatile __irq_masks[__NRF_NVIC_ISER_COUNT]; /**< IRQs enabled by the application in the NVIC. */
  uint32_t volatile __cr_flag;                          /**< Non-zero if already in a critical region */
} nrf_nvic_state_t;

/**@brief Variable keeping the state for the SoftDevice NVIC module. This must be declared in an
 * application source file. */
nrf_nvic_state_t nrf_nvic_state;

/**
  \ingroup    CMSIS_core_register
  \defgroup   CMSIS_NVIC  Nested Vectored Interrupt Controller (NVIC)
  \brief      Type definitions for the NVIC Registers
  @{
 */

#ifndef NVIC
/**
  \brief  Structure type to access the Nested Vectored Interrupt Controller (NVIC).
 */
typedef struct
{
  volatile uint32_t ISER[8U];               /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
  uint32_t RESERVED0[24U];
  volatile uint32_t ICER[8U];               /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
  uint32_t RSERVED1[24U];
  volatile uint32_t ISPR[8U];               /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
  uint32_t RESERVED2[24U];
  volatile uint32_t ICPR[8U];               /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
  uint32_t RESERVED3[24U];
  volatile uint32_t IABR[8U];               /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register */
  uint32_t RESERVED4[56U];
  volatile uint8_t  IP[240U];               /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) */
  uint32_t RESERVED5[644U];
  volatile  uint32_t STIR;                   /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register */
}  NVIC_Type;

/**
  \brief  Structure type to access the System Control Block (SCB).
 */
typedef struct
{
  __IM  uint32_t CPUID;                  /*!< Offset: 0x000 (R/ )  CPUID Base Register */
  __IOM uint32_t ICSR;                   /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register */
  __IOM uint32_t VTOR;                   /*!< Offset: 0x008 (R/W)  Vector Table Offset Register */
  __IOM uint32_t AIRCR;                  /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register */
  __IOM uint32_t SCR;                    /*!< Offset: 0x010 (R/W)  System Control Register */
  __IOM uint32_t CCR;                    /*!< Offset: 0x014 (R/W)  Configuration Control Register */
  __IOM uint8_t  SHPR[12U];              /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
  __IOM uint32_t SHCSR;                  /*!< Offset: 0x024 (R/W)  System Handler Control and State Register */
  __IOM uint32_t CFSR;                   /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register */
  __IOM uint32_t HFSR;                   /*!< Offset: 0x02C (R/W)  HardFault Status Register */
  __IOM uint32_t DFSR;                   /*!< Offset: 0x030 (R/W)  Debug Fault Status Register */
  __IOM uint32_t MMFAR;                  /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register */
  __IOM uint32_t BFAR;                   /*!< Offset: 0x038 (R/W)  BusFault Address Register */
  __IOM uint32_t AFSR;                   /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register */
  __IM  uint32_t ID_PFR[2U];             /*!< Offset: 0x040 (R/ )  Processor Feature Register */
  __IM  uint32_t ID_DFR;                 /*!< Offset: 0x048 (R/ )  Debug Feature Register */
  __IM  uint32_t ID_AFR;                 /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register */
  __IM  uint32_t ID_MFR[4U];             /*!< Offset: 0x050 (R/ )  Memory Model Feature Register */
  __IM  uint32_t ID_ISAR[5U];            /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register */
  uint32_t RESERVED0[1U];
  __IM  uint32_t CLIDR;                  /*!< Offset: 0x078 (R/ )  Cache Level ID register */
  __IM  uint32_t CTR;                    /*!< Offset: 0x07C (R/ )  Cache Type register */
  __IM  uint32_t CCSIDR;                 /*!< Offset: 0x080 (R/ )  Cache Size ID Register */
  __IOM uint32_t CSSELR;                 /*!< Offset: 0x084 (R/W)  Cache Size Selection Register */
  __IOM uint32_t CPACR;                  /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register */
  uint32_t RESERVED3[93U];
  __OM  uint32_t STIR;                   /*!< Offset: 0x200 ( /W)  Software Triggered Interrupt Register */
  uint32_t RESERVED4[15U];
  __IM  uint32_t MVFR0;                  /*!< Offset: 0x240 (R/ )  Media and VFP Feature Register 0 */
  __IM  uint32_t MVFR1;                  /*!< Offset: 0x244 (R/ )  Media and VFP Feature Register 1 */
  __IM  uint32_t MVFR2;                  /*!< Offset: 0x248 (R/ )  Media and VFP Feature Register 1 */
  uint32_t RESERVED5[1U];
  __OM  uint32_t ICIALLU;                /*!< Offset: 0x250 ( /W)  I-Cache Invalidate All to PoU */
  uint32_t RESERVED6[1U];
  __OM  uint32_t ICIMVAU;                /*!< Offset: 0x258 ( /W)  I-Cache Invalidate by MVA to PoU */
  __OM  uint32_t DCIMVAC;                /*!< Offset: 0x25C ( /W)  D-Cache Invalidate by MVA to PoC */
  __OM  uint32_t DCISW;                  /*!< Offset: 0x260 ( /W)  D-Cache Invalidate by Set-way */
  __OM  uint32_t DCCMVAU;                /*!< Offset: 0x264 ( /W)  D-Cache Clean by MVA to PoU */
  __OM  uint32_t DCCMVAC;                /*!< Offset: 0x268 ( /W)  D-Cache Clean by MVA to PoC */
  __OM  uint32_t DCCSW;                  /*!< Offset: 0x26C ( /W)  D-Cache Clean by Set-way */
  __OM  uint32_t DCCIMVAC;               /*!< Offset: 0x270 ( /W)  D-Cache Clean and Invalidate by MVA to PoC */
  __OM  uint32_t DCCISW;                 /*!< Offset: 0x274 ( /W)  D-Cache Clean and Invalidate by Set-way */
  uint32_t RESERVED7[6U];
  __IOM uint32_t ITCMCR;                 /*!< Offset: 0x290 (R/W)  Instruction Tightly-Coupled Memory Control Register */
  __IOM uint32_t DTCMCR;                 /*!< Offset: 0x294 (R/W)  Data Tightly-Coupled Memory Control Registers */
  __IOM uint32_t AHBPCR;                 /*!< Offset: 0x298 (R/W)  AHBP Control Register */
  __IOM uint32_t CACR;                   /*!< Offset: 0x29C (R/W)  L1 Cache Control Register */
  __IOM uint32_t AHBSCR;                 /*!< Offset: 0x2A0 (R/W)  AHB Slave Control Register */
  uint32_t RESERVED8[1U];
  __IOM uint32_t ABFSR;                  /*!< Offset: 0x2A8 (R/W)  Auxiliary Bus Fault Status Register */
} SCB_Type;

#define SCS_BASE            (0xE000E000UL)                            /*!< System Control Space Base Address */
#define NVIC_BASE           (SCS_BASE +  0x0100UL)                    /*!< NVIC Base Address */
#define SCB_BASE            (SCS_BASE +  0x0D00UL)                    /*!< System Control Block Base Address */

#define SCB                 ((SCB_Type       *)     SCB_BASE      )   /*!< SCB configuration struct */
#define NVIC                ((NVIC_Type      *)     NVIC_BASE     )   /*!< NVIC configuration struct */

void up_systemreset(void);

#endif

/**@} */

/**@addtogroup NRF_NVIC_INTERNAL_FUNCTIONS SoftDevice NVIC internal functions
 * @{ */

/**@brief Disables IRQ interrupts globally, including the SoftDevice's interrupts.
 *
 * @retval  The value of PRIMASK prior to disabling the interrupts.
 */
__STATIC_INLINE int __sd_nvic_irq_disable(void);

/**@brief Enables IRQ interrupts globally, including the SoftDevice's interrupts.
 */
__STATIC_INLINE void __sd_nvic_irq_enable(void);

/**@brief Checks if IRQn is available to application
 * @param[in]  IRQn  irq to check
 *
 * @retval  1 (true) if the irq to check is available to the application
 */
__STATIC_INLINE uint32_t __sd_nvic_app_accessible_irq(IRQn_Type IRQn);

/**@brief Checks if priority is available to application
 * @param[in]  priority  priority to check
 *
 * @retval  1 (true) if the priority to check is available to the application
 */
__STATIC_INLINE uint32_t __sd_nvic_is_app_accessible_priority(uint32_t priority);

/**@} */

/**@addtogroup NRF_NVIC_FUNCTIONS SoftDevice NVIC public functions
 * @{ */

/**@brief Enable External Interrupt.
 * @note Corresponds to NVIC_EnableIRQ in CMSIS.
 *
 * @pre IRQn is valid and not reserved by the stack.
 *
 * @param[in] IRQn See the NVIC_EnableIRQ documentation in CMSIS.
 *
 * @retval ::NRF_SUCCESS The interrupt was enabled.
 * @retval ::NRF_ERROR_SOC_NVIC_INTERRUPT_NOT_AVAILABLE The interrupt is not available for the application.
 * @retval ::NRF_ERROR_SOC_NVIC_INTERRUPT_PRIORITY_NOT_ALLOWED The interrupt has a priority not available for the application.
 */
__STATIC_INLINE uint32_t sd_nvic_EnableIRQ(IRQn_Type IRQn);

/**@brief  Disable External Interrupt.
 * @note Corresponds to NVIC_DisableIRQ in CMSIS.
 *
 * @pre IRQn is valid and not reserved by the stack.
 *
 * @param[in] IRQn See the NVIC_DisableIRQ documentation in CMSIS.
 *
 * @retval ::NRF_SUCCESS The interrupt was disabled.
 * @retval ::NRF_ERROR_SOC_NVIC_INTERRUPT_NOT_AVAILABLE The interrupt is not available for the application.
 */
__STATIC_INLINE uint32_t sd_nvic_DisableIRQ(IRQn_Type IRQn);

/**@brief  Get Pending Interrupt.
 * @note Corresponds to NVIC_GetPendingIRQ in CMSIS.
 *
 * @pre IRQn is valid and not reserved by the stack.
 *
 * @param[in]   IRQn          See the NVIC_GetPendingIRQ documentation in CMSIS.
 * @param[out]  p_pending_irq Return value from NVIC_GetPendingIRQ.
 *
 * @retval ::NRF_SUCCESS The interrupt is available for the application.
 * @retval ::NRF_ERROR_SOC_NVIC_INTERRUPT_NOT_AVAILABLE IRQn is not available for the application.
 */
__STATIC_INLINE uint32_t sd_nvic_GetPendingIRQ(IRQn_Type IRQn, uint32_t *p_pending_irq);

/**@brief  Set Pending Interrupt.
 * @note Corresponds to NVIC_SetPendingIRQ in CMSIS.
 *
 * @pre IRQn is valid and not reserved by the stack.
 *
 * @param[in] IRQn See the NVIC_SetPendingIRQ documentation in CMSIS.
 *
 * @retval ::NRF_SUCCESS The interrupt is set pending.
 * @retval ::NRF_ERROR_SOC_NVIC_INTERRUPT_NOT_AVAILABLE IRQn is not available for the application.
 */
__STATIC_INLINE uint32_t sd_nvic_SetPendingIRQ(IRQn_Type IRQn);

/**@brief  Clear Pending Interrupt.
 * @note Corresponds to NVIC_ClearPendingIRQ in CMSIS.
 *
 * @pre IRQn is valid and not reserved by the stack.
 *
 * @param[in] IRQn See the NVIC_ClearPendingIRQ documentation in CMSIS.
 *
 * @retval ::NRF_SUCCESS The interrupt pending flag is cleared.
 * @retval ::NRF_ERROR_SOC_NVIC_INTERRUPT_NOT_AVAILABLE IRQn is not available for the application.
 */
__STATIC_INLINE uint32_t sd_nvic_ClearPendingIRQ(IRQn_Type IRQn);

/**@brief Set Interrupt Priority.
 * @note Corresponds to NVIC_SetPriority in CMSIS.
 *
 * @pre IRQn is valid and not reserved by the stack.
 * @pre Priority is valid and not reserved by the stack.
 *
 * @param[in] IRQn      See the NVIC_SetPriority documentation in CMSIS.
 * @param[in] priority  A valid IRQ priority for use by the application.
 *
 * @retval ::NRF_SUCCESS The interrupt and priority level is available for the application.
 * @retval ::NRF_ERROR_SOC_NVIC_INTERRUPT_NOT_AVAILABLE IRQn is not available for the application.
 * @retval ::NRF_ERROR_SOC_NVIC_INTERRUPT_PRIORITY_NOT_ALLOWED The interrupt priority is not available for the application.
 */
__STATIC_INLINE uint32_t sd_nvic_SetPriority(IRQn_Type IRQn, uint32_t priority);

/**@brief Get Interrupt Priority.
 * @note Corresponds to NVIC_GetPriority in CMSIS.
 *
 * @pre IRQn is valid and not reserved by the stack.
 *
 * @param[in]  IRQn         See the NVIC_GetPriority documentation in CMSIS.
 * @param[out] p_priority   Return value from NVIC_GetPriority.
 *
 * @retval ::NRF_SUCCESS The interrupt priority is returned in p_priority.
 * @retval ::NRF_ERROR_SOC_NVIC_INTERRUPT_NOT_AVAILABLE - IRQn is not available for the application.
 */
__STATIC_INLINE uint32_t sd_nvic_GetPriority(IRQn_Type IRQn, uint32_t *p_priority);

/**@brief System Reset.
 * @note Corresponds to NVIC_SystemReset in CMSIS.
 *
 * @retval ::NRF_ERROR_SOC_NVIC_SHOULD_NOT_RETURN
 */
__STATIC_INLINE uint32_t sd_nvic_SystemReset(void);

/**@brief Enter critical region.
 *
 * @post Application interrupts will be disabled.
 * @note sd_nvic_critical_region_enter() and ::sd_nvic_critical_region_exit() must be called in matching pairs inside each
 * execution context
 * @sa sd_nvic_critical_region_exit
 *
 * @param[out] p_is_nested_critical_region If 1, the application is now in a nested critical region.
 *
 * @retval ::NRF_SUCCESS
 */
__STATIC_INLINE uint32_t sd_nvic_critical_region_enter(uint8_t *p_is_nested_critical_region);

/**@brief Exit critical region.
 *
 * @pre Application has entered a critical region using ::sd_nvic_critical_region_enter.
 * @post If not in a nested critical region, the application interrupts will restored to the state before ::sd_nvic_critical_region_enter was called.
 *
 * @param[in] is_nested_critical_region If this is set to 1, the critical region won't be exited. @sa sd_nvic_critical_region_enter.
 *
 * @retval ::NRF_SUCCESS
 */
__STATIC_INLINE uint32_t sd_nvic_critical_region_exit(uint8_t is_nested_critical_region);

/**@} */

#ifndef SUPPRESS_INLINE_IMPLEMENTATION

#ifndef __NVIC_PRIO_BITS
#define __NVIC_PRIO_BITS 3
#endif

__STATIC_INLINE int __sd_nvic_irq_disable(void)
{
  int pm = getprimask();
  up_irq_disable();
  return pm;
}

__STATIC_INLINE void __sd_nvic_irq_enable(void)
{
  up_irq_enable();
}

__STATIC_INLINE uint32_t __sd_nvic_app_accessible_irq(IRQn_Type IRQn)
{
  if (IRQn < 32)
    {
      return ((1UL << IRQn) & __NRF_NVIC_APP_IRQS_0) != 0;
    }
#ifdef NRF52
  else if (IRQn < 64)
    {
      return ((1UL << (IRQn - 32)) & __NRF_NVIC_APP_IRQS_1) != 0;
    }
#endif
  else
    {
      return 1;
    }
}

__STATIC_INLINE uint32_t __sd_nvic_is_app_accessible_priority(uint32_t priority)
{
  if (priority >= (1 << __NVIC_PRIO_BITS))
    {
      return 0;
    }
#ifdef NRF51
  if (   priority == 0
         || priority == 2
     )
    {
      return 0;
    }
#endif
#ifdef NRF52
  if (   priority == 0
         || priority == 1
         || priority == 4
         || priority == 5
     )
    {
      return 0;
    }
#endif
  return 1;
}

/**
  \brief   Get Interrupt Priority
  \details Reads the priority of an interrupt.
           The interrupt number can be positive to specify an external (device specific) interrupt,
           or negative to specify an internal (core) interrupt.
  \param [in]   IRQn  Interrupt number.
  \return             Interrupt Priority.
                      Value is aligned automatically to the implemented priority bits of the microcontroller.
 */
__STATIC_INLINE uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) < 0)
    {
      return (((uint32_t)SCB->SHPR[(((uint32_t)(int32_t)IRQn) & 0xFUL) - 4UL] >> (8U - __NVIC_PRIO_BITS)));
    }
  else
    {
      return (((uint32_t)NVIC->IP[((uint32_t)(int32_t)IRQn)]                >> (8U - __NVIC_PRIO_BITS)));
    }
}

__STATIC_INLINE uint32_t sd_nvic_EnableIRQ(IRQn_Type IRQn)
{
  if (!__sd_nvic_app_accessible_irq(IRQn))
    {
      return NRF_ERROR_SOC_NVIC_INTERRUPT_NOT_AVAILABLE;
    }
  if (!__sd_nvic_is_app_accessible_priority(NVIC_GetPriority(IRQn)))
    {
      return NRF_ERROR_SOC_NVIC_INTERRUPT_PRIORITY_NOT_ALLOWED;
    }

  if (nrf_nvic_state.__cr_flag)
    {
      nrf_nvic_state.__irq_masks[(uint32_t)((int32_t)IRQn) >> 5] |= (uint32_t)(1 << ((uint32_t)((int32_t)IRQn) & (uint32_t)0x1F));
    }
  else
    {
      up_enable_irq((int)IRQn);
    }
  return NRF_SUCCESS;
}

__STATIC_INLINE uint32_t sd_nvic_DisableIRQ(IRQn_Type IRQn)
{
  if (!__sd_nvic_app_accessible_irq(IRQn))
    {
      return NRF_ERROR_SOC_NVIC_INTERRUPT_NOT_AVAILABLE;
    }

  if (nrf_nvic_state.__cr_flag)
    {
      nrf_nvic_state.__irq_masks[(uint32_t)((int32_t)IRQn) >> 5] &= ~(1UL << ((uint32_t)(IRQn) & 0x1F));
    }
  else
    {
      up_disable_irq((int)IRQn);
    }

  return NRF_SUCCESS;
}

/**
  \brief   Get Pending Interrupt
  \details Reads the pending register in the NVIC and returns the pending bit for the specified interrupt.
  \param [in]      IRQn  Interrupt number.
  \return             0  Interrupt status is not pending.
  \return             1  Interrupt status is pending.
 */
__STATIC_INLINE uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return ((uint32_t)(((NVIC->ISPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL :
                     0UL));
}


/**
  \brief   Set Pending Interrupt
  \details Sets the pending bit of an external interrupt.
  \param [in]      IRQn  Interrupt number. Value cannot be negative.
 */
__STATIC_INLINE void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  NVIC->ISPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}


/**
  \brief   Clear Pending Interrupt
  \details Clears the pending bit of an external interrupt.
  \param [in]      IRQn  External interrupt number. Value cannot be negative.
 */
__STATIC_INLINE void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  NVIC->ICPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}

__STATIC_INLINE uint32_t sd_nvic_GetPendingIRQ(IRQn_Type IRQn, uint32_t *p_pending_irq)
{
  if (__sd_nvic_app_accessible_irq(IRQn))
    {
      *p_pending_irq = NVIC_GetPendingIRQ(IRQn);
      return NRF_SUCCESS;
    }
  else
    {
      return NRF_ERROR_SOC_NVIC_INTERRUPT_NOT_AVAILABLE;
    }
}

__STATIC_INLINE uint32_t sd_nvic_SetPendingIRQ(IRQn_Type IRQn)
{
  if (__sd_nvic_app_accessible_irq(IRQn))
    {
      NVIC_SetPendingIRQ(IRQn);
      return NRF_SUCCESS;
    }
  else
    {
      return NRF_ERROR_SOC_NVIC_INTERRUPT_NOT_AVAILABLE;
    }
}

__STATIC_INLINE uint32_t sd_nvic_ClearPendingIRQ(IRQn_Type IRQn)
{
  if (__sd_nvic_app_accessible_irq(IRQn))
    {
      NVIC_ClearPendingIRQ(IRQn);
      return NRF_SUCCESS;
    }
  else
    {
      return NRF_ERROR_SOC_NVIC_INTERRUPT_NOT_AVAILABLE;
    }
}

__STATIC_INLINE uint32_t sd_nvic_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if (!__sd_nvic_app_accessible_irq(IRQn))
    {
      return NRF_ERROR_SOC_NVIC_INTERRUPT_NOT_AVAILABLE;
    }

  if (!__sd_nvic_is_app_accessible_priority(priority))
    {
      return NRF_ERROR_SOC_NVIC_INTERRUPT_PRIORITY_NOT_ALLOWED;
    }

  up_prioritize_irq((int)IRQn, (uint32_t)priority);
  return NRF_SUCCESS;
}

__STATIC_INLINE uint32_t sd_nvic_GetPriority(IRQn_Type IRQn, uint32_t *p_priority)
{
  if (__sd_nvic_app_accessible_irq(IRQn))
    {
      *p_priority = (NVIC_GetPriority(IRQn) & 0xFF);
      return NRF_SUCCESS;
    }
  else
    {
      return NRF_ERROR_SOC_NVIC_INTERRUPT_NOT_AVAILABLE;
    }
}

__STATIC_INLINE uint32_t sd_nvic_SystemReset(void)
{
  up_systemreset();
  return NRF_ERROR_SOC_NVIC_SHOULD_NOT_RETURN;
}

__STATIC_INLINE uint32_t sd_nvic_critical_region_enter(uint8_t *p_is_nested_critical_region)
{
  int was_masked = __sd_nvic_irq_disable();
  if (!nrf_nvic_state.__cr_flag)
    {
      nrf_nvic_state.__cr_flag = 1;
      nrf_nvic_state.__irq_masks[0] = ( NVIC->ICER[0] & __NRF_NVIC_APP_IRQS_0 );
      NVIC->ICER[0] = __NRF_NVIC_APP_IRQS_0;
#ifdef NRF52
      nrf_nvic_state.__irq_masks[1] = ( NVIC->ICER[1] & __NRF_NVIC_APP_IRQS_1 );
      NVIC->ICER[1] = __NRF_NVIC_APP_IRQS_1;
#endif
      *p_is_nested_critical_region = 0;
    }
  else
    {
      *p_is_nested_critical_region = 1;
    }
  if (!was_masked)
    {
      __sd_nvic_irq_enable();
    }
  return NRF_SUCCESS;
}

__STATIC_INLINE uint32_t sd_nvic_critical_region_exit(uint8_t is_nested_critical_region)
{
  if (nrf_nvic_state.__cr_flag && (is_nested_critical_region == 0))
    {
      int was_masked = __sd_nvic_irq_disable();
      NVIC->ISER[0] = nrf_nvic_state.__irq_masks[0];
#ifdef NRF52
      NVIC->ISER[1] = nrf_nvic_state.__irq_masks[1];
#endif
      nrf_nvic_state.__cr_flag = 0;
      if (!was_masked)
        {
          __sd_nvic_irq_enable();
        }
    }

  return NRF_SUCCESS;
}

#endif /* SUPPRESS_INLINE_IMPLEMENTATION */

#ifdef __cplusplus
}
#endif

#endif // NRF_NVIC_H__

/**@} */
