/******************************************************************************
 * arch/arm/src/nrf52/nrf52_pwm.h
 *
 *   Copyright (C) 2011, 2015 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2015 Omni Hoverboards Inc. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Levin Li <zhiqiang@zglue.com>
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
 *****************************************************************************/

#ifndef __ARCH_ARM_SRC_NRF52_NRF52_PWM_H
#define __ARCH_ARM_SRC_NRF52_NRF52_PWM_H

/******************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_NRF52_PWM
/******************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/
/* Configuration *************************************************************/
/* There are totally three hardware PWM controller for NRF52
 * Please make sure: PWM_CLK / expected frequency > duty<<16/100
 * this is due to NRF52 hardware limited ,  countstop > duty_value
 *  NRF52 PWM module can support up to 4 channels for different
 *  duty cycle base on same period
 */


/******************************************************************************
 * Public Types
 *****************************************************************************/
/**
 * @brief This value can be provided instead of a pin number for any channel
 *        to specify that its output is not used and therefore does not need
 *        to be connected to a pin.
 */
#define NRF52_PWM_PIN_NOT_USED    0xFF

/**
 * @brief This value can be added to a pin number to inverse its polarity
 *        (set idle state = 1).
 */
#define NRF52_PWM_PIN_INVERTED    0x80

/* nrf52832 pwm each module have 4 pwm channel */
#define NRF52_MAX_PWM_NCHANNELS   4

struct pwm_pinmux_t
{
  uint32_t pincfg;    /* Output pin configuration */
  uint16_t channel;   /* channel number */
};

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/******************************************************************************
 * Public Data
 *****************************************************************************/
typedef enum
{
  NRF52_PWM_POLARITY_RISING = 0,
  NRF52_PWM_POLARITY_FAILING = 1
} nrf52_pwm_ploarity_t;

typedef enum
{
  /* Up counter (edge-aligned PWM duty cycle) */

  NRF52_PWM_MODE_UP = 0,
  /* Up and down counter (center-aligned PWM duty cycle) */

  NRF52_PWM_MODE_CENTER = 1
} nrf52_pwm_mode_t;

#ifdef CONFIG_PWM_MULTICHAN
struct nrf52_pwm_chan_s
{
  /* Duty of the pulse train, "1"-to-"0" duration.
   * Maximum: 65535/65536 (0x0000ffff)
   * Minimum:     1/65536 (0x00000001) */

  uint16_t  duty;
  uint16_t  polarity; /* nrf52_pwm_ploarity_t */
  uint8_t   channel;
};
#endif

struct nrf52_pwm_info_s
{
  uint32_t                  frequency; /* Frequency of the pulse train */
  nrf52_pwm_mode_t          mode; /* nrf52_pwm_mode_t */

#ifdef CONFIG_PWM_MULTICHAN
  /* Per-channel output state */
  struct nrf52_pwm_chan_s   channels[CONFIG_PWM_NCHANNELS];
#else
  /* Duty of the pulse train, "1"-to-"0" duration.
   * Maximum: 65535/65536 (0x0000ffff)
   * Minimum:     1/65536 (0x00000001) */

  uint16_t                  duty;
  nrf52_pwm_ploarity_t      polarity;
#endif /* CONFIG_PWM_MULTICHAN */
};

/* parameter is struct nrf52_pwm_info_s */
#define PWMIOC_START_NRF52_INFO _PWMIOC(10)

#define PWMIOC_STOP_NRF52_INFO _PWMIOC(11)  /* parameter is  NULL */


/******************************************************************************
 * Public Functions
 *****************************************************************************/

/******************************************************************************
 * Name: nrf52_pwm_initialize
 *
 * Description:
 *   Initialize PWM module and register pwm driver.
 *
 * Input Parameters:
 *   port   : indicated which pwm module will be used
 *   pinmux   : indicated pwm channel gpio pin config
 *   channel   : indicated how many pinmux configs
 *
 * Returned Value:
 *   On success, return OK.
 *
 *****************************************************************************/

FAR int nrf52_pwm_initialize(uint32_t port, struct pwm_pinmux_t *pinmux, uint32_t channels);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_NRF52_PWM */
#endif /* __ARCH_ARM_SRC_NRF52_NRF52_PWM_H */

