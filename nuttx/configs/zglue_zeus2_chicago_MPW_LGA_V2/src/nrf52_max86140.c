/****************************************************************************
 * configs/zglue_zeus2_chicago/src/nrf52_max86140.c
 *
 *   Copyright (C) 2018 zGlue Inc. All rights reserved.
 *   Author: HT <tao@zglue.com>
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

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/spi/spi.h>

#include <arch/board/board.h>
#include <nuttx/sensors/max86140.h>

#include "up_arch.h"
#include "nrf.h"
#include "nrf52_spi.h"
#include "nrf52_i2c.h"
#include "nrf52_gpio.h"
#include "nrf52_gpiote.h"
#include "nrf_gpio.h"

#ifdef CONFIG_MAX86140

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
#define MAX86140_MAX_NUM  (8)
/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct max86140_dev_s *priv_list[MAX86140_MAX_NUM];
static uint8_t priv_num = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: max86140_int_handler
 *
 * Description:
 *   max86140 alert intrrupt handler.
 *
 ************************************************************************************/
static void max86140_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{

  int i;
  for (i = 0; i < priv_num; i++)
    {
      if (priv_list[i]->config.int_pin == pin)
        {
          break;
        }
    }

  if ((i < priv_num) && (priv_list[i]->int_cb))
    {
      priv_list[i]->int_cb(priv_list[i]);
    }
}

/************************************************************************************
 * Name: max86140_dev_register
 *
 * Description:
 *   Register max86140 device information.
 *
 ************************************************************************************/
int max86140_dev_register(FAR struct max86140_dev_s *priv)
{
  assert(priv);

  if (priv_num >= MAX86140_MAX_NUM)
    {
      syslog(LOG_ERR, "max86140: Failed to init the interrupt, get to MAX\n");
      return ERROR;
    }

  priv_list[priv_num++] = priv;
  return OK;
}

/************************************************************************************
 * Name: max86140_int_initialize
 *
 * Description:
 *   Initialize max86140 alert pin to interrupt handler.
 *
 ************************************************************************************/
int max86140_int_initialize(FAR struct max86140_dev_s *priv)
{
  int ret;

  nrf_drv_gpiote_in_config_t pin_cfg =
  {
    .is_watcher = false,
    .hi_accuracy = true,
    .pull = NRF_GPIO_PIN_NOPULL,
    .sense = GPIOTE_CONFIG_POLARITY_HiToLo,
  };

  assert(priv);

  ret = nrf_drv_gpiote_in_init(priv->config.int_pin, &pin_cfg, max86140_int_handler);
  ret = nrf_sdk_retcode_to_nuttx(ret);
  if (ret < 0)
    {
      snerr("ERROR: max86140 register INT failed %d. \n", ret);
      return ret;
    }
  nrf_drv_gpiote_in_event_enable(priv->config.int_pin, true);

  return ret;
}

#endif /* CONFIG_MAX86140 */
