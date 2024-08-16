/****************************************************************************************************
 * arch/arm/src/nrf52/chip/nrf52_tim.h
 *
 *   Copyright (C) 2009, 2011-2012 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_NRF52_CHIP_NRF52_TIM_H
#define __ARCH_ARM_SRC_NRF52_CHIP_NRF52_TIM_H

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

#include <stddef.h>
#include <stdbool.h>
#include "chip.h"
#include "nrf.h"
#include "nrf_timer.h"
#include <arch/nrf52/nordic_common.h>

#ifdef CONFIG_TIMER

#ifdef CONFIG_NRF52_TIM0
#define TIMER0_ENABLED 1
#else
#define TIMER0_ENABLED 0
#endif

// <q> TIMER1_ENABLED  - Enable TIMER1 instance


#ifdef CONFIG_NRF52_TIM1
#define TIMER1_ENABLED 1
#else
#define TIMER1_ENABLED 0
#endif

// <q> TIMER2_ENABLED  - Enable TIMER2 instance


#ifdef CONFIG_NRF52_TIM2
#define TIMER2_ENABLED 1
#else
#define TIMER2_ENABLED 0
#endif

// <q> TIMER3_ENABLED  - Enable TIMER3 instance


#ifdef CONFIG_NRF52_TIM3
#define TIMER3_ENABLED 1
#else
#define TIMER3_ENABLED 0
#endif

// <q> TIMER4_ENABLED  - Enable TIMER4 instance


#ifdef CONFIG_NRF52_TIM4
#define TIMER4_ENABLED 1
#else
#define TIMER4_ENABLED 0
#endif

#define ENABLED_TIMER_COUNT (TIMER0_ENABLED+TIMER1_ENABLED+TIMER2_ENABLED+TIMER3_ENABLED+TIMER4_ENABLED)

// <q> TIMER0_ENABLED  - Enable TIMER0 instance

#define TIMER0_INSTANCE_INDEX 0
#define TIMER1_INSTANCE_INDEX TIMER0_INSTANCE_INDEX+TIMER1_ENABLED
#define TIMER2_INSTANCE_INDEX TIMER1_INSTANCE_INDEX+TIMER2_ENABLED
#define TIMER3_INSTANCE_INDEX TIMER2_INSTANCE_INDEX+TIMER3_ENABLED
#define TIMER4_INSTANCE_INDEX TIMER3_INSTANCE_INDEX+TIMER4_ENABLED

/**
 * @brief Macro for creating a timer driver instance.
 */
#define NRF_DRV_TIMER_INSTANCE(id) \
{                                                             \
    .p_reg            = CONCAT_2(NRF_TIMER, id),              \
    .instance_id      = CONCAT_3(TIMER, id, _INSTANCE_INDEX), \
    .cc_channel_count = NRF_TIMER_CC_CHANNEL_COUNT(id),       \
}

/**
 * @brief Timer driver instance configuration structure.
 */
typedef struct
{
  nrf_timer_frequency_t frequency;          ///< Frequency.
  nrf_timer_mode_t      mode;               ///< Mode of operation.
  nrf_timer_bit_width_t bit_width;          ///< Bit width.
  uint8_t               interrupt_priority; ///< Interrupt priority.
  void                 *p_context;          ///< Context passed to interrupt handler.
} nrf52_timer_config_t;


/**
 * @brief Timer driver instance default configuration.
 */
#define NRF_DRV_TIMER_DEFAULT_CONFIG \
{                                                                               \
    .frequency          = (nrf_timer_frequency_t)TIMER_DEFAULT_CONFIG_FREQUENCY,\
    .mode               = (nrf_timer_mode_t)TIMER_DEFAULT_CONFIG_MODE,          \
    .bit_width          = (nrf_timer_bit_width_t)TIMER_DEFAULT_CONFIG_BIT_WIDTH,\
    .interrupt_priority = TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,                    \
    .p_context          = NULL                                                  \
}

/**
 * @brief Timer driver event handler type.
 *
 * @param[in] event_type Timer event.
 * @param[in] p_context  General purpose parameter set during initialization of
 *                       the timer. This parameter can be used to pass
 *                       additional information to the handler function, for
 *                       example, the timer ID.
 */
typedef void (* nrf_timer_event_handler_t)(nrf_timer_event_t event_type,
                                           void *p_context);



/**
 * @brief FUnction for setting the ISR
 *
 * @param[in] p_reg             Pointer to the peripheral registers structure.
 * @param[in] timer_shorts_mask Shortcuts to disable.
 */
int nrf52_tim_setisr(FAR struct nrf52_tim_dev_s *dev,
                     xcpt_t handler, void *arg, int source);

/**
 * @brief FUnction for setting the ISR
 *
 * @param[in] p_reg             Pointer to the peripheral registers structure.
 * @param[in] timer_shorts_mask Shortcuts to disable.
 */
bool nrf52_timer_irq_checkint(FAR struct nrf52_tim_dev_s *dev,
                              uint32_t channel);

/**
 * @brief FUnction for setting the ISR
 *
 * @param[in] p_reg             Pointer to the peripheral registers structure.
 * @param[in] timer_shorts_mask Shortcuts to disable.
 */
void nrf52_timer_irq_clearint(FAR struct nrf52_tim_dev_s *dev,
                              uint32_t channel);


/**
 * @brief Function for enabling specified interrupts.
 *
 * @param[in] p_reg          Pointer to the peripheral registers structure.
 * @param[in] timer_int_mask Interrupts to enable.
 */
void nrf52_timer_irq_clear(FAR struct nrf52_tim_dev_s *dev);




/**
 * @brief Timer driver event handler type.
 *
 * @param[in] event_type Timer event.
 * @param[in] p_context  General purpose parameter set during initialization of
 *                       the timer. This parameter can be used to pass
 *                       additional information to the handler function, for
 *                       example, the timer ID.
 */
typedef void (* nrf_timer_event_handler_t)(nrf_timer_event_t event_type,
                                           void *p_context);

/**
 * @brief Function for initializing the timer.
 *
 * @param[in] p_instance          Pointer to the driver instance structure.
 * @param[in] p_config            Initial configuration.
 *                                If NULL, the default configuration is used.
 * @param[in] timer_event_handler Event handler provided by the user.
 *                                Must not be NULL.
 *
 * @retval NRF_SUCCESS             If initialization was successful.
 * @retval NRF_ERROR_INVALID_STATE If the instance is already initialized.
 * @retval NRF_ERROR_INVALID_PARAM If no handler was provided.
 */
FAR struct nrf52_tim_dev_s *nrf52_timer_init(int timer);

/**
 * @brief Function for uninitializing the timer.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
void nrf52_timer_uninit(struct nrf52_tim_dev_s *dev);

/**
 * @brief Function for turning on the timer.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
void nrf52_timer_enable(FAR struct nrf52_tim_dev_s *dev);

/**
 * @brief Function for turning off the timer.
 *
 * Note that the timer will allow to enter the lowest possible SYSTEM_ON state
 * only after this function is called.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
void nrf52_timer_disable(FAR struct nrf52_tim_dev_s *dev);

/**
 * @brief Function for pausing the timer.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
void nrf52_timer_pause(FAR struct nrf52_tim_dev_s *dev);

/**
 * @brief Function for resuming the timer.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
void nrf52_timer_resume(FAR struct nrf52_tim_dev_s *dev);

/**
 * @brief Function for clearing the timer.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
void nrf52_timer_clear(FAR struct nrf52_tim_dev_s *dev);

/**
 * @brief Function for incrementing the timer.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
void nrf52_timer_increment(struct nrf52_tim_dev_s *dev);

/**
 * @brief Get the current timeout value in the private structure
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
uint32_t nrf52_get_current_timeout(FAR struct nrf52_tim_dev_s *dev);

/**
 * @brief Function for capturing the timer value.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] cc_channel Capture channel number.
 *
 * @return Captured value.
 */
uint32_t nrf52_timer_capture(struct nrf52_tim_dev_s *dev);

/**
 * @brief Function for setting the timer channel in compare mode.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] cc_channel Compare channel number.
 * @param[in] cc_value   Compare value.
 * @param[in] enable_int Enable or disable the interrupt for the compare channel.
 */
void nrf52_timer_compare(FAR struct nrf52_tim_dev_s *dev,
                         nrf_timer_cc_channel_t cc_channel,
                         uint32_t               cc_value,
                         bool                   enable_int);

/**
 * @brief Function for setting the timer channel in extended compare mode.
 *
 * @param[in] p_instance       Pointer to the driver instance structure.
 * @param[in] cc_channel       Compare channel number.
 * @param[in] cc_value         Compare value.
 * @param[in] timer_short_mask Shortcut between the compare event on the channel
 *                             and the timer task (STOP or CLEAR).
 * @param[in] enable_int       Enable or disable the interrupt for the compare
 *                             channel.
 */
void nrf52_timer_extended_compare(FAR struct nrf52_tim_dev_s *dev,
                                  nrf_timer_cc_channel_t cc_channel,
                                  uint32_t               cc_value,
                                  nrf_timer_short_mask_t timer_short_mask,
                                  bool                   enable_int);

/**
 * @brief Function for converting time in microseconds to timer ticks.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] time_us    Time in microseconds.
 *
 * @return Number of ticks.
 */
uint32_t nrf52_timer_us_to_ticks(
  FAR struct nrf52_tim_dev_s *dev,
  uint32_t time_us);

/**
 * @brief Function for converting time in milliseconds to timer ticks.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] time_ms    Time in milliseconds.
 *
 * @return Number of ticks.
 */
uint32_t nrf52_timer_ms_to_ticks(
  FAR struct nrf52_tim_dev_s *dev,
  uint32_t time_ms);

/**
 * @brief Function for enabling timer compare interrupt.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] channel    Compare channel.
 */
void nrf52_timer_compare_int_enable(FAR struct nrf52_tim_dev_s *dev, uint32_t channel);

/**
 * @brief Function for disabling timer compare interrupt.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] channel    Compare channel.
 */
void nrf52_timer_compare_int_disable(FAR struct nrf52_tim_dev_s *dev, uint32_t channel);


/**
 * @brief Function for returning the address of a specific timer task.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] timer_task Timer task.
 *
 * @return Task address.
 */
uint32_t nrf52_timer_task_address_get(
  FAR struct nrf52_tim_dev_s *dev,
  nrf_timer_task_t timer_task);

uint32_t nrf52_timer_event_address_get(
  FAR struct nrf52_tim_dev_s *dev,
  nrf_timer_event_t timer_event);

#endif /* CONFIG_TIMER */
#endif /* __ARCH_ARM_SRC_NRF52_CHIP_NRF52_TIM_H */
