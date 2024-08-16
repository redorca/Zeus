/****************************************************************************************************
 * arch/arm/src/nrf52/chip/nrf52_priv_rtc.h
 *
 *   Copyright (C) 2009, 2011-2012 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2018 Zglue Inc. All rights reserved.
 *
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_NRF52_CHIP_NRF52_PRIV_RTC_H
#define __ARCH_ARM_SRC_NRF52_CHIP_NRF52_PRIV_RTC_H

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/
#include <nuttx/config.h>
#include <stddef.h>
#include <stdbool.h>
#include "chip.h"
#include "nrf.h"


#ifdef __cplusplus
extern "C" {
#endif

#define RTC_PRESCALER 327  // RTC freq=100Hz

#ifndef RTC_DEFAULT_CONFIG_FREQUENCY
#define RTC_DEFAULT_CONFIG_FREQUENCY 32768
#endif

// <q> RTC_DEFAULT_CONFIG_RELIABLE  - Ensures safe compare event triggering
#ifndef RTC_DEFAULT_CONFIG_RELIABLE
#define RTC_DEFAULT_CONFIG_RELIABLE 0
#endif

// <o> NRF_MAXIMUM_LATENCY_US - Maximum possible time[us] in highest priority interrupt
#ifndef NRF_MAXIMUM_LATENCY_US
#define NRF_MAXIMUM_LATENCY_US 2000
#endif

/**@brief Macro to convert microseconds into ticks. */
#define RTC_US_TO_TICKS(us,freq) ((us * freq) / 1000000)


/**
 * @brief RTC capture/compare channels.
 */
typedef enum
{
  NRF_RTC_CC_CHANNEL0 = 0, ///< RTC capture/compare channel 0.
  NRF_RTC_CC_CHANNEL1,     ///< RTC capture/compare channel 1.
  NRF_RTC_CC_CHANNEL2,     ///< RTC capture/compare channel 2.
  NRF_RTC_CC_CHANNEL3,     ///< RTC capture/compare channel 3.
} nrf_rtc_cc_channel_t;


/* RTC Device Structure */
struct nrf52_rtc_priv_s
{
  uint32_t              base;           /* RTCn base address */
  nrfx_drv_state_t       state;          /* Current rtc state */
  nrf_rtc_cc_channel_t  channel_number; /* Current channel number */
  uint32_t              timeout;        /* Current timeout in uS  */
  uint8_t               channel_count;
  bool                  reliable;       /**< Reliable mode flag. */
  uint8_t               tick_latency;   /**< Maximum length of interrupt handler in ticks (max 7.7 ms). */
};


/**
 * @brief FUnction for setting the ISR
 *
 * @param[in] p_reg             Pointer to the peripheral registers structure.
 * @param[in] rtc_shorts_mask Shortcuts to disable.
 */
int nrf52_rtc_setisr(FAR struct nrf52_rtc_priv_s *dev,
                     xcpt_t handler, void *arg, int source);

/**
 * @brief FUnction for setting the ISR
 *
 * @param[in] p_reg             Pointer to the peripheral registers structure.
 * @param[in] rtc_shorts_mask Shortcuts to disable.
 */
bool nrf52_rtc_irq_handleint(FAR struct nrf52_rtc_priv_s *dev);

/**
 * @brief Function for turning on the rtc.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
void nrf52_rtc_enable(FAR struct nrf52_rtc_priv_s *dev);

/**
 * @brief Function for turning off the rtc.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
void nrf52_rtc_disable(FAR struct nrf52_rtc_priv_s *dev);


/**@brief Function for enabling tick.
 *
 * This function enables the tick event and optionally the interrupt. The function asserts if the instance is not
 *       powered on.
 *
 * @param[in]  p_instance         Pointer to the driver instance structure.
 */
void nrf_drv_rtc_tick_enable(FAR struct nrf52_rtc_priv_s *dev);

/**@brief Function for disable tick.
 *
 * This function disable the tick event and optionally the interrupt. The function asserts if the instance is not
 *       powered on.
 *
 * @param[in]  p_instance         Pointer to the driver instance structure.
 */
void nrf_drv_rtc_tick_disable(FAR struct nrf52_rtc_priv_s *dev);


/**@brief Function for setting a compare channel.
 *
 * The function asserts if the instance is not initialized or if the channel parameter is
 *       wrong. The function powers on the instance if the instance was in power off state.
 *
 * The driver is not entering a critical section when configuring RTC, which means that it can be
 *       preempted for a certain amount of time. When the driver was preempted and the value to be set
 *       is short in time, there is a risk that the driver sets a compare value that is
 *       behind. If RTCn_CONFIG_RELIABLE is 1 for the given instance, the Reliable mode handles that case.
 *       However, to detect if the requested value is behind, this mode makes the following assumptions:
 *        -  The maximum preemption time in ticks (8 - bit value) is known and is less than 7.7 ms
 *         (for prescaler = 0, RTC frequency 32 kHz).
 *        -  The requested absolute compare value is not bigger than (0x00FFFFFF) - tick_latency. It is
 *         the user's responsibility to ensure that.
 *
 * @param[in]  p_instance         Pointer to the driver instance structure.
 * @param[in]  channel            One of the instance's channels.
 * @param[in]  val                Absolute value to be set in the compare register.
 * @param[in]  enable_irq         True to enable the interrupt. False to disable the interrupt.
 *
 * @retval     NRF_SUCCESS         If the procedure was successful.
 * @retval     NRF_ERROR_TIMEOUT   If the compare was not set because the request value is behind the current counter
 *                                 value. This error can only be reported if RTCn_CONFIG_RELIABLE = 1.
 */
uint32_t nrf52_drv_rtc_cc_set(FAR struct nrf52_rtc_priv_s *dev,
                              uint32_t channel,
                              uint32_t val,
                              bool enable_irq);

/**
 * @brief Set the current RTC frequency
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] frequency  the expect frequency.
 */

uint32_t nrf52_rtc_set_clock(FAR struct nrf52_rtc_priv_s *dev, uint32_t frequency);

void nrf_drv_rtc_overflow_enable(FAR struct nrf52_rtc_priv_s *dev);

void nrf_drv_rtc_overflow_disable(FAR struct nrf52_rtc_priv_s *dev);

ret_code_t nrf_drv_rtc_cc_disable(FAR struct nrf52_rtc_priv_s *dev, uint32_t channel);

/**
 * @brief Get the current timeout value in the private structure
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
uint32_t nrf52_get_current_rtc_timeout(FAR struct nrf52_rtc_priv_s *dev);


/**
 * @brief Get the current counter value in the private structure
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
uint32_t nrf52_get_current_rtc_counter(FAR struct nrf52_rtc_priv_s *dev);

/**
 * @brief Function for initializing the rtc.
 *
 * @param[in] p_instance          Pointer to the driver instance structure.
 * @param[in] p_config            Initial configuration.
 *                                If NULL, the default configuration is used.
 * @param[in] rtc_event_handler Event handler provided by the user.
 *                                Must not be NULL.
 *
 * @retval NRF_SUCCESS             If initialization was successful.
 * @retval NRF_ERROR_INVALID_STATE If the instance is already initialized.
 * @retval NRF_ERROR_INVALID_PARAM If no handler was provided.
 */
FAR struct nrf52_rtc_priv_s *nrf52_rtc_init(int rtc);

#ifdef __cplusplus
}
#endif

#endif /*__ARCH_ARM_SRC_NRF52_CHIP_NRF52_PRIV_RTC_H*/

