/****************************************************************************
 * configs/zglue_zeus2_chicago/src/nrf52_bq2512x.c
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

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "nrf.h"
#include "nrf52_i2c.h"
#include "nrf52_gpio.h"
#include "nrf52_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_error.h"

#include <nuttx/power/bq2512x.h>
#include <nuttx/power/battery_charger.h>


#if defined(CONFIG_BQ2512X)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
#if  (!defined(CONFIG_NRF52_I2C0)) && (BQ2512X_I2C_PORTNO == 0)
#  error CONFIG_NRF52_I2C0 is required if you want to use BQ2512X in this board.
#endif

#if  (!defined(CONFIG_NRF52_I2C1)) && (BQ2512X_I2C_PORTNO == 1)
#  error CONFIG_NRF52_I2C1 is required if you want to use BQ2512X in this board.
#endif


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static void bq2512x_INT_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
static int bq2512x_cfg_INT(bq25120_gpio_int_cb_t cb, void *param);
static int bq2512x_enable_INT(bool enable);

#ifdef CONFIG_BQ2512X_NRESET_TO_GPIO
static void bq2512x_RESET_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
static int bq2512x_cfg_RESET(bq25120_gpio_int_cb_t cb, void *param);
static int bq2512x_enable_RESET(bool enable);
#endif

#ifdef CONFIG_BQ2512X_LSCTRL_TO_GPIO
static int bq2512x_cfg_LSCTRL(void);
static int bq2512x_write_LSCTRL(bool value);
#endif

#ifdef CONFIG_BQ2512X_NPG_TO_GPIO
static int bq2512x_cfg_NPG(void);
static int bq2512x_read_NPG(bool *value);
#endif



/****************************************************************************
 * Private Data
 ****************************************************************************/

static bq25120_gpio_int_cb_t bq2512x_INT_pin_cb = NULL;
static void *INT_pin_cb_param = NULL;

#ifdef CONFIG_BQ2512X_NRESET_TO_GPIO
static bq25120_gpio_int_cb_t bq2512x_RESET_pin_cb = NULL;
static void *RESET_pin_cb_param = NULL;
#endif

const struct bq25120_low_level_operations_s bq2512x_ll_op =
{
  .bq25120_INT_cfg_interrupt = bq2512x_cfg_INT,
  .bq25120_INT_enable = bq2512x_enable_INT,
#ifdef CONFIG_BQ2512X_NRESET_TO_GPIO
  .bq25120_RESET_cfg_interrupt = bq2512x_cfg_RESET,
  .bq25120_RESET_enable = bq2512x_enable_RESET,
#endif

#ifdef CONFIG_BQ2512X_LSCTRL_TO_GPIO
  .bq25120_LSCTRL_cfg_output = bq2512x_cfg_LSCTRL,
  .bq25120_LSCTRL_write = bq2512x_write_LSCTRL,
#endif

#ifdef CONFIG_BQ2512X_NPG_TO_GPIO
  .bq25120_NPG_cfg_input = bq2512x_cfg_NPG,
  .bq25120_NPG_read = bq2512x_read_NPG,
#endif
};


/****************************************************************************
 * Private Functions
 ****************************************************************************/


static void bq2512x_INT_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  if (bq2512x_INT_pin_cb != NULL)
    {
      bq2512x_INT_pin_cb(INT_pin_cb_param);
    }
}

#ifdef CONFIG_BQ2512X_NRESET_TO_GPIO
static void bq2512x_RESET_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  if (bq2512x_RESET_pin_cb != NULL)
    {
      bq2512x_RESET_pin_cb(RESET_pin_cb_param);
    }
}
#endif



static int bq2512x_cfg_INT(bq25120_gpio_int_cb_t cb, void *param)
{
  int ret = OK;
  nrf_drv_gpiote_in_config_t pin_cfg = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);

  bq2512x_INT_pin_cb = cb;
  INT_pin_cb_param = param;
  ret = nrf_drv_gpiote_in_init(BQ2512X_INT_PIN, &pin_cfg, bq2512x_INT_pin_handler);
  ret = nrf_sdk_retcode_to_nuttx(ret);

  return ret;
}

#ifdef CONFIG_BQ2512X_NRESET_TO_GPIO
static int bq2512x_cfg_RESET(bq25120_gpio_int_cb_t cb, void *param)
{
  int ret = OK;
  nrf_drv_gpiote_in_config_t pin_cfg = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);

  bq2512x_RESET_pin_cb = cb;
  RESET_pin_cb_param = param;
  ret = nrf_drv_gpiote_in_init(BQ2512X_RESET_PIN, &pin_cfg, bq2512x_RESET_pin_handler);
  ret = nrf_sdk_retcode_to_nuttx(ret);

  return ret;
}
#endif

static int bq2512x_enable_INT(bool enable)
{
  if (enable)
    {
      nrf_drv_gpiote_in_event_enable(BQ2512X_INT_PIN, true);
    }
  else
    {
      nrf_drv_gpiote_in_event_disable(BQ2512X_INT_PIN);
    }
  return OK;
}

#ifdef CONFIG_BQ2512X_NRESET_TO_GPIO
static int bq2512x_enable_RESET(bool enable)
{
  if (enable)
    {
      nrf_drv_gpiote_in_event_enable(BQ2512X_RESET_PIN, true);
    }
  else
    {
      nrf_drv_gpiote_in_event_disable(BQ2512X_RESET_PIN);
    }

  return OK;
}
#endif

#ifdef CONFIG_BQ2512X_LSCTRL_TO_GPIO
static int bq2512x_cfg_LSCTRL(void)
{
  nrf_gpio_cfg_output(BQ2512X_LSCTRL_PIN);
  return OK;
}

static int bq2512x_write_LSCTRL(bool value)
{
  nrf_gpio_pin_write(BQ2512X_LSCTRL_PIN, value);
  return OK;
}
#endif

#ifdef CONFIG_BQ2512X_NPG_TO_GPIO
static int bq2512x_cfg_NPG(void)
{
  nrf_gpio_cfg_input(BQ2512X_NPG_PIN, GPIO_PIN_CNF_PULL_Disabled);
  return OK;

}

static int bq2512x_read_NPG(bool *value)
{
  *value = nrf_gpio_pin_read(BQ2512X_NPG_PIN);
  return OK;

}
#endif



/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: nrf52_bq2512x_initialize
 *
 * Description:
 *   Initialize and register the bq2512x PMIC device driver.
 *
 ************************************************************************************/
int nrf52_bq2512x_initialize(void)
{
  int ret = OK;
  FAR struct i2c_master_s *i2c;
  struct battery_charger_dev_s *bq2512x_dev;

  /* Initialize I2C */
  i2c = nrf52_i2cbus_initialize(BQ2512X_I2C_PORTNO);

  if (!i2c)
    {
      snerr("ERROR: FAILED to initialize i2c port\n");
      return -ENODEV;
    }


  /* Then register the PMIC BQ25120 sensor */
  bq2512x_dev = bq2512x_initialize(i2c,
                                   CONFIG_BQ2512X_ADDR,
                                   CONFIG_BQ2512X_I2C_FREQ,
                                   &bq2512x_ll_op);

  if (bq2512x_dev == NULL)
    {
      snerr("\nERROR: Error registering bq2512\n");
    }
  else
    {
      ret = battery_charger_register("/dev/bq2512", bq2512x_dev);
      if (ret < 0)
        {
          snerr("ERROR: Error registering BQ battery charger\n");
        }
    }

  return ret;
}



#endif /* CONFIG_BQ2512X */
