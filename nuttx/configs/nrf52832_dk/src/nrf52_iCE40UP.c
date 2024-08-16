/****************************************************************************
 * configs/nrf52832_dk/src/nrf52_iCE40UP.c
 *
 *   Copyright (C) 2018 zGlue Inc. All rights reserved.
 *   Author: Zhengwei Wang <zhengwei@zglue.com>
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>

#include <arch/board/board.h>
#include <arch/chip/nordic_common.h>

#include "up_arch.h"
#include "nrf.h"
#include "nrf52_gpio.h"
#include "nrf52_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_timer.h"
#include "nrf_error.h"
#include "nrf_ppi.h"

#include <nuttx/sensors/iCE40UP.h>

#ifdef CONFIG_ICE40UP

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
#ifdef CONFIG_ICE40_RESULT_PWM
#define RISING_EDGE_MAX_COUNT 276
#define MONITOR_LOTOHI
#else
#define MONITOR_LOTOHI
#endif

#define BUZZER_CONTROL_ENABLE


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int iCE40UP_attach_gpio_irq(iCE40UP_int_cb_t cb, void *arg);
static int iCE40UP_enable_int_pin(bool enable);


/****************************************************************************
 * Private Data
 ****************************************************************************/

static iCE40UP_int_cb_t iCE40UP_cb = NULL;
static void *iCE40UP_int_cb_arg = NULL;

FAR const struct iCE40UP_low_level_operations_s iCE40UP_ll_op =
{
  iCE40UP_attach_gpio_irq,
  iCE40UP_enable_int_pin,
};



/****************************************************************************
 * Private Functions
 ****************************************************************************/
#ifdef CONFIG_ICE40_RESULT_LEVEL
int nrf52_timer2_interrupt(int irq, FAR void *context, FAR void *arg)
{
  nrf_timer_event_t event = nrf_timer_compare_event_get(NRF_TIMER_CC_CHANNEL0);
  nrf_timer_int_mask_t int_mask = nrf_timer_compare_int_get(NRF_TIMER_CC_CHANNEL0);

  if (nrf_timer_event_check(NRF_TIMER2, event) &&
      nrf_timer_int_enable_check(NRF_TIMER2, int_mask))
    {
      nrf_timer_event_clear(NRF_TIMER2, event);

#ifdef MONITOR_LOTOHI
      if (nrf_gpio_pin_read(ICE40UP_INT_PIN))
#else
      if (!nrf_gpio_pin_read(ICE40UP_INT_PIN))
#endif
        {
          iCE40UP_cb(iCE40UP_int_cb_arg, true);
#ifdef BUZZER_CONTROL_ENABLE
          nrf_gpio_pin_set(ICE40UP_BUZZER_PIN);
#endif
        }
      else
        {
          iCE40UP_cb(iCE40UP_int_cb_arg, false);
#ifdef BUZZER_CONTROL_ENABLE
          nrf_gpio_pin_clear(ICE40UP_BUZZER_PIN);
#endif
        }
    }

  return OK;
}


static int iCE40UP_attach_gpio_irq(iCE40UP_int_cb_t cb, void *arg)
{
  int ret = OK;
  int i;

  /*attach callback function*/
  iCE40UP_cb = cb;
  iCE40UP_int_cb_arg = arg;

  /*initial interrupt pin*/
  nrf_drv_gpiote_in_config_t pin_cfg =
  {
    \
    .is_watcher = false,                     \
    .hi_accuracy = true,                  \
    .pull = GPIO_PIN_CNF_PULL_Pullup,             \
    .sense = NRF_GPIOTE_POLARITY_TOGGLE,     \
  };

  ret = nrf_drv_gpiote_in_init(ICE40UP_INT_PIN, &pin_cfg, NULL);

  /*check result*/
  if (ret != NRF_SUCCESS)
    {
      ret = -EIO;
      return ret;
    }

#ifdef BUZZER_CONTROL_ENABLE
  /*initial buzzer pin*/
  nrf_gpio_cfg_output(ICE40UP_BUZZER_PIN);
#endif

  /*initial timer2, this timer is used for debounce*/
  for (i = 0; i < NRF_TIMER_CC_CHANNEL_COUNT(2); ++i)
    {
      nrf_timer_event_clear(NRF_TIMER2, nrf_timer_compare_event_get(i));
    }

  nrf_timer_mode_set(NRF_TIMER2, NRF_TIMER_MODE_TIMER);
  nrf_timer_bit_width_set(NRF_TIMER2, NRF_TIMER_BIT_WIDTH_32);
  nrf_timer_frequency_set(NRF_TIMER2, NRF_TIMER_FREQ_125kHz);
  nrf_timer_cc_write(NRF_TIMER2, NRF_TIMER_CC_CHANNEL0, 5000);

  nrf_timer_shorts_enable(NRF_TIMER2,
                          (TIMER_SHORTS_COMPARE0_STOP_Msk  << NRF_TIMER_CC_CHANNEL0));

  /*enable timer2 channel0 compare interrupt*/
  nrf_timer_event_clear(NRF_TIMER2, nrf_timer_compare_event_get(NRF_TIMER_CC_CHANNEL0));

  nrf_timer_int_enable(NRF_TIMER2, nrf_timer_compare_int_get(NRF_TIMER_CC_CHANNEL0));

  irq_attach(TIMER2_IRQn, nrf52_timer2_interrupt, NULL);

  /*set up PPI, pin interrupt --->timer clear and start*/
  nrf_ppi_channel_endpoint_setup(NRF_PPI_CHANNEL0,
                                 nrf_drv_gpiote_in_event_addr_get(ICE40UP_INT_PIN),
                                 (uint32_t)nrf_timer_task_address_get(NRF_TIMER2, NRF_TIMER_TASK_START));
  nrf_ppi_fork_endpoint_setup(NRF_PPI_CHANNEL0,
                              (uint32_t)nrf_timer_task_address_get(NRF_TIMER2, NRF_TIMER_TASK_CLEAR));


  return ret;
}

static int iCE40UP_enable_int_pin( bool enable)
{
  if (enable)
    {
      nrf_drv_gpiote_in_event_enable(ICE40UP_INT_PIN, false);
      nrf_ppi_channel_enable(NRF_PPI_CHANNEL0);
      up_enable_irq(TIMER2_IRQn);
    }
  else
    {
      nrf_drv_gpiote_in_event_disable(ICE40UP_INT_PIN);
      nrf_ppi_channel_disable(NRF_PPI_CHANNEL0);
      up_disable_irq(TIMER2_IRQn);
    }

  return OK;
}
#endif

#ifdef CONFIG_ICE40_RESULT_PWM
/*timer2 interrupt is triggerd when we have a continuous PWM*/
int nrf52_timer2_interrupt(int irq, FAR void *context, FAR void *arg)
{
  nrf_timer_event_t event = nrf_timer_compare_event_get(NRF_TIMER_CC_CHANNEL0);
  nrf_timer_int_mask_t int_mask = nrf_timer_compare_int_get(NRF_TIMER_CC_CHANNEL0);

  if (nrf_timer_event_check(NRF_TIMER2, event) &&
      nrf_timer_int_enable_check(NRF_TIMER2, int_mask))
    {
      /*clear interrupt source*/
      nrf_timer_event_clear(NRF_TIMER2, event);

      iCE40UP_cb(iCE40UP_int_cb_arg, true);

      /*enable timer3 interrupt and wait for its interrupt*/
      nrf_timer_event_clear(NRF_TIMER3, nrf_timer_compare_event_get(NRF_TIMER_CC_CHANNEL0));

      up_enable_irq(TIMER3_IRQn);
    }

  return OK;
}

/*timer3 interrupt is triggerd when the continuous PWM paused*/
int nrf52_timer3_interrupt(int irq, FAR void *context, FAR void *arg)
{
  nrf_timer_event_t event = nrf_timer_compare_event_get(NRF_TIMER_CC_CHANNEL0);
  nrf_timer_int_mask_t int_mask = nrf_timer_compare_int_get(NRF_TIMER_CC_CHANNEL0);

  if (nrf_timer_event_check(NRF_TIMER3, event) &&
      nrf_timer_int_enable_check(NRF_TIMER3, int_mask))
    {
      nrf_timer_event_clear(NRF_TIMER3, event);

      iCE40UP_cb(iCE40UP_int_cb_arg, false);

      /*only the first timer3 interrupt after timer2 interrupt is valuable, we ignore the others*/
      up_disable_irq(TIMER3_IRQn);
    }

  return OK;
}



static int iCE40UP_attach_gpio_irq(iCE40UP_int_cb_t cb, void *arg)
{
  int ret = OK;
  uint8_t i;

  iCE40UP_cb = cb;
  iCE40UP_int_cb_arg = arg;

#ifdef MONITOR_LOTOHI
  nrf_drv_gpiote_in_config_t pin_cfg = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
#else
  nrf_drv_gpiote_in_config_t pin_cfg = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
#endif

  /*gpiote initialize*/
  ret = nrf_drv_gpiote_in_init(ICE40UP_INT_PIN, &pin_cfg, NULL);

  if (ret != NRF_SUCCESS)
    {
      ret = -EIO;
      return ret;
    }

  nrf_gpio_cfg_output(ICE40UP_ENABLE_PIN);

#ifdef BUZZER_CONTROL_ENABLE
  nrf_drv_gpiote_out_config_t buzzer_pin_cfg =
  {
    .init_state = NRF_GPIOTE_INITIAL_VALUE_LOW,
    .task_pin   = true,
    .action     = NRF_GPIOTE_POLARITY_HITOLO,
  };

  ret = nrf_drv_gpiote_out_init(ICE40UP_BUZZER_PIN, &buzzer_pin_cfg);

  if (ret != NRF_SUCCESS)
    {
      ret = -EIO;
      return ret;
    }
#endif

  /*timer initialize*/

  for (i = 0; i < NRF_TIMER_CC_CHANNEL_COUNT(2); ++i)
    {
      nrf_timer_event_clear(NRF_TIMER2, nrf_timer_compare_event_get(i));
    }

  nrf_timer_mode_set(NRF_TIMER2, NRF_TIMER_MODE_COUNTER);
  nrf_timer_bit_width_set(NRF_TIMER2, NRF_TIMER_BIT_WIDTH_32);
  nrf_timer_cc_write(NRF_TIMER2, NRF_TIMER_CC_CHANNEL0, RISING_EDGE_MAX_COUNT);

  nrf_timer_shorts_enable(NRF_TIMER2,
                          (TIMER_SHORTS_COMPARE0_STOP_Msk  << NRF_TIMER_CC_CHANNEL0) |
                          (TIMER_SHORTS_COMPARE0_CLEAR_Msk << NRF_TIMER_CC_CHANNEL0));


  for (i = 0; i < NRF_TIMER_CC_CHANNEL_COUNT(3); ++i)
    {
      nrf_timer_event_clear(NRF_TIMER3, nrf_timer_compare_event_get(i));

    }

  nrf_timer_mode_set(NRF_TIMER3, NRF_TIMER_MODE_TIMER);
  nrf_timer_bit_width_set(NRF_TIMER3, NRF_TIMER_BIT_WIDTH_32);
  nrf_timer_frequency_set(NRF_TIMER3, NRF_TIMER_FREQ_250kHz);
  nrf_timer_cc_write(NRF_TIMER3, NRF_TIMER_CC_CHANNEL0, 1000);

  nrf_timer_shorts_enable(NRF_TIMER3,
                          (TIMER_SHORTS_COMPARE0_STOP_Msk  << NRF_TIMER_CC_CHANNEL0));

  nrf_timer_event_clear(NRF_TIMER2, nrf_timer_compare_event_get(NRF_TIMER_CC_CHANNEL0));

  nrf_timer_int_enable(NRF_TIMER2, nrf_timer_compare_int_get(NRF_TIMER_CC_CHANNEL0));

  irq_attach(TIMER2_IRQn, nrf52_timer2_interrupt, NULL);

  up_enable_irq(TIMER2_IRQn);

  nrf_timer_event_clear(NRF_TIMER3, nrf_timer_compare_event_get(NRF_TIMER_CC_CHANNEL0));

  nrf_timer_int_enable(NRF_TIMER3, nrf_timer_compare_int_get(NRF_TIMER_CC_CHANNEL0));

  irq_attach(TIMER3_IRQn, nrf52_timer3_interrupt, NULL);


  /*PPI initialize*/
  /*pin rising edge -> timer2 count*/
  nrf_ppi_channel_endpoint_setup(NRF_PPI_CHANNEL0,
                                 nrf_drv_gpiote_in_event_addr_get(ICE40UP_INT_PIN),
                                 (uint32_t)nrf_timer_task_address_get(NRF_TIMER2, NRF_TIMER_TASK_COUNT));

  /*pin rising edge -> timer3 clear and start*/
  nrf_ppi_channel_endpoint_setup(NRF_PPI_CHANNEL1,
                                 nrf_drv_gpiote_in_event_addr_get(ICE40UP_INT_PIN),
                                 (uint32_t)nrf_timer_task_address_get(NRF_TIMER3, NRF_TIMER_TASK_START));
  nrf_ppi_fork_endpoint_setup(NRF_PPI_CHANNEL1,
                              (uint32_t)nrf_timer_task_address_get(NRF_TIMER3, NRF_TIMER_TASK_CLEAR));

  /*timer3 compare -> timer2 clear and start*/
  nrf_ppi_channel_endpoint_setup(NRF_PPI_CHANNEL2,
                                 (uint32_t)nrf_timer_event_address_get(NRF_TIMER3, nrf_timer_compare_event_get(NRF_TIMER_CC_CHANNEL0)),
                                 (uint32_t)nrf_timer_task_address_get(NRF_TIMER2, NRF_TIMER_TASK_START));
  nrf_ppi_fork_endpoint_setup(NRF_PPI_CHANNEL2,
                              (uint32_t)nrf_timer_task_address_get(NRF_TIMER2, NRF_TIMER_TASK_CLEAR));

#ifdef BUZZER_CONTROL_ENABLE
  /*timer3 compare -> clear buzzer pin*/
  nrf_ppi_channel_endpoint_setup(NRF_PPI_CHANNEL3,
                                 (uint32_t)nrf_timer_event_address_get(NRF_TIMER3, nrf_timer_compare_event_get(NRF_TIMER_CC_CHANNEL0)),
                                 (uint32_t)nrf_drv_gpiote_clr_task_addr_get(ICE40UP_BUZZER_PIN));

  /*timer2 compare -> set buzzer pin*/
  nrf_ppi_channel_endpoint_setup(NRF_PPI_CHANNEL6,
                                 (uint32_t)nrf_timer_event_address_get(NRF_TIMER2, nrf_timer_compare_event_get(NRF_TIMER_CC_CHANNEL0)),
                                 (uint32_t)nrf_drv_gpiote_set_task_addr_get(ICE40UP_BUZZER_PIN));
#endif

  return ret;
}



static int iCE40UP_enable_int_pin( bool enable)
{
  if (enable)
    {
      nrf_gpio_pin_set(ICE40UP_ENABLE_PIN);
      nrf_drv_gpiote_in_event_enable(ICE40UP_INT_PIN, false);
      nrf_ppi_channel_enable(NRF_PPI_CHANNEL0);
      nrf_ppi_channel_enable(NRF_PPI_CHANNEL1);
      nrf_ppi_channel_enable(NRF_PPI_CHANNEL2);

#ifdef BUZZER_CONTROL_ENABLE
      nrf_drv_gpiote_out_task_enable(ICE40UP_BUZZER_PIN);
      nrf_ppi_channel_enable(NRF_PPI_CHANNEL3);
      nrf_ppi_channel_enable(NRF_PPI_CHANNEL6);
#endif
    }
  else
    {
      nrf_gpio_pin_clear(ICE40UP_ENABLE_PIN);
      nrf_drv_gpiote_in_event_disable(ICE40UP_INT_PIN);
      nrf_ppi_channel_disable(NRF_PPI_CHANNEL0);
      nrf_ppi_channel_disable(NRF_PPI_CHANNEL1);
      nrf_ppi_channel_disable(NRF_PPI_CHANNEL2);

#ifdef BUZZER_CONTROL_ENABLE
      nrf_drv_gpiote_out_task_disable(ICE40UP_BUZZER_PIN);
      nrf_ppi_channel_disable(NRF_PPI_CHANNEL3);
      nrf_ppi_channel_disable(NRF_PPI_CHANNEL6);
#endif

    }

  return OK;
}

#endif





/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: nrf52_iCE40UP_initialize
 *
 * Description:
 *   Initialize and register the iCE40 Ultra Plus family AI chips.
 *
 ************************************************************************************/
int nrf52_iCE40UP_initialize(void)
{
  int ret;

  ret = iCE40UP_register("/dev/iCE40UP", &iCE40UP_ll_op);

  return ret;
}



#endif /* CONFIG_ICE40UP */
