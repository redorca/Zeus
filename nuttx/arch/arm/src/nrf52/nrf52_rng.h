/****************************************************************************
 * arch/arm/src/nrf52/nrf52_rng.h
 *
 *   Copyright (C) 2012 Max Holtzberg. All rights reserved.
 *   Author: Max Holtzberg <mh@uvc.de>
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

#ifndef _ARCH_ARM_SRC_NRF52_NRF52_RNG_H
#define _ARCH_ARM_SRC_NRF52_NRF52_RNG_H

#if defined(CONFIG_NRF52_RNG)

#if !defined(CONFIG_DEV_RANDOM) && !defined(CONFIG_DEV_URANDOM)
#error Alert! The RNG is only partially configured. DEV_RANDOM or DEV_URANDOM must be chosen as well
#endif

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
EXTERN
{
#else
#define EXTERN extern
#endif

  /* ====================================================================== *
   * ===============                  RNG                ================== *
   * ===============        (Random Number Generator)    ================== *
   * ====================================================================== */


  /**************************************************************************
   * Private Type Definitions
   **************************************************************************/
  typedef enum
  {
    RNG_TASK_START = 0x0000,
    RNG_TASK_STOP = 0x0004,
    RNG_EVENT_VALRDY = 0x0100,
    RNG_SHORTS = 0x0200,
    RNG_INTENSET = 0x0304,
    RNG_INTENCLR = 0x0308,
    RNG_CONFIG = 0x504,
    RNG_VALUE = 0x0508,
  } nrf52_rng_regs_e;

  /**************************************************************************
   * Public Type Definitions
   **************************************************************************/

#ifdef SOFTDEVICE_PRESENT
#define SD_RAND_POOL_SIZE           (32)
  /*  STATIC_ASSERT(RNG_CONFIG_POOL_SIZE == SD_RAND_POOL_SIZE); */
#define NRF_DRV_RNG_SD_IS_ENABLED() softdevice_handler_is_enabled()
#define NRF_DRV_RNG_LOCK()        \
    {                                  \
        uint8_t __CR_NESTED = 0;       \
        sd_nvic_critical_region_enter(&__CR_NESTED);

#define NRF_DRV_RNG_RELEASE() \
        sd_nvic_critical_region_exit(__CR_NESTED); \
    }
#else /* ! SOFTDEVICE_PRESENT */
#define NRF_DRV_RNG_LOCK() \
    {                           \
        irq_state_t f;          \
        f = enter_critical_section();

#define NRF_DRV_RNG_RELEASE() \
        leave_critical_section(f); \
    }

#define NRF_DRV_RNG_SD_IS_ENABLED() false

#endif /* SOFTDEVICE_PRESENT */

  /* RNG driver ioctl definitions *********************************************/
#define _RNGIOCVALID(c)   (_IOC_TYPE(c)==_RNGBASE_)
#define _RNGIOC(nr)       _IOC(_RNGBASE_,nr)
#define  RNG_SETFILTER    _RNGIOC(1)
#define  RNG_CLRFILTER    _RNGIOC(2)
#define  RNG_START_CONV   _RNGIOC(3)
#define  RNG_STOP_CONV    _RNGIOC(4)

  /*
   * Independently controls whether the inline or not definitions
   * of a function have been verified. Presumably, once verified
   * then the inline functions can use the same methods and also
   * become verified.
   */
#define RNG_FUNCS_VERIFY
#define RNG_REG_ADDR(module_offset)   _MODULE_REG_ADDR(NRF_RNG_BASE, module_offset)
#if !defined(SUPPRESS_INLINE_IMPLEMENTATION) && ! defined(RNG_FUNCS_VERIFY)
#error Should not be here!
  __STATIC_INLINE uint32_t rng_reg_write32(uint32_t reg_offset, uint32_t data)
  {
    return putreg32(data, RNG_REG_ADDR(reg_offset));
    /*   return putreg32(data, ((uint32_t *)NRF_RNG_BASE + ((uint32_t) reg_offset >> 2))); */
  }

  __STATIC_INLINE uint32_t rng_reg_read32(uint32_t reg_offset)
  {
    return getreg32(RNG_REG_ADDR(reg_offset));
    /*   return getreg32((uint32_t *)NRF_RNG_BASE + ((uint32_t) reg_offset >> 2)); */
  }
#else
  __STATIC_ uint32_t rng_reg_write32(uint32_t reg_offset, uint32_t data);
  __STATIC_ uint32_t rng_reg_read32(uint32_t reg_offset);
#endif

#define SET             1
#define RESET           0
#define TRIGGER         1
#define INT_CLR         1
#define CLEAR           1
#define NRF52_RNG_TASK_START(a)        rng_reg_write32(RNG_TASK_START, a)       /* a = 1 to trigger start     */
#define NRF52_RNG_TASK_STOP(a)         rng_reg_write32(RNG_TASK_STOP,  a)       /* a = 1 to trigger start     */
#define NRF52_RNG_EVENT_CLR(a)         rng_reg_write32(RNG_EVENT_VALRDY, a)     /* a = 0 to clear event       */
#define NRF52_RNG_EVENT_GET()          rng_reg_read32(RNG_EVENT_VALRDY)
#define NRF52_RNG_INTENSET(a)          rng_reg_write32(RNG_INTENSET, a)         /* a = 1 to trigger start     */
#define NRF52_RNG_INT_STAT()           rng_reg_read32(RNG_INTENCLR)
#define NRF52_RNG_INTENCLR(a)          rng_reg_write32(RNG_INTENCLR, a)          /* a = 1 to clear interrupt   */
#define NRF52_RNG_SHORTS_EN(a)         rng_reg_write32(RNG_SHORTS, a)            /* a = 1 to enable            */
#define NRF52_RNG_SHORTS_CLR(a)        rng_reg_write32(RNG_SHORTS, a)            /* a = 0 to clear shorts link */
#define NRF52_RNG_SHORTS_DATA()        rng_reg_read32(RNG_SHORTS)
#define NRF52_RNG_CONFIG_BIAS_SET(a)   rng_reg_write32(RNG_CONFIG, a)            /* a = 1 to set bias config   */
#define NRF52_RNG_CONFIG_BIAS_CLR(a)   rng_reg_write32(RNG_CONFIG, a)            /* a = 0 to clear bias config */
#define NRF52_RNG_CONFIG_READ()        rng_reg_read32(RNG_CONFIG)
#define NRF52_RNG_READ_u8VALUE()      (uint8_t) (rng_reg_read32(RNG_VALUE) & 0x000000FF)
#define NRF52_RNG_READ_VALUE()        (rng_reg_read32(RNG_VALUE))

  /**************************************************************************
   * Private Function Prototypes
   **************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NRF52_RNG */
#endif /* _ARCH_ARM_SRC_NRF52_NRF52_RNG_H */

