/****************************************************************************
 * configs/nrf52832_dk/src/nrf52_bmm150.c
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
#include "nrf52_spi.h"
#include "nrf52_i2c.h"
#include "nrf52_gpio.h"
#include "nrf52_gpiote.h"
#include "nrf_gpio.h"

#include <nuttx/sensors/bmm150.h>

#ifdef CONFIG_BMM150

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
#ifdef CONFIG_BMM150_INTF_SPI

#if  (!defined(CONFIG_NRF52_SPI0)) && (BMM150_SPI_PORTNO == 0)
#  error CONFIG_NRF52_SPI0 is required if you want to use BMM150 in this board.
#endif

#if  (!defined(CONFIG_NRF52_SPI1)) && (BMM150_SPI_PORTNO == 1)
#  error CONFIG_NRF52_SPI1 is required if you want to use BMM150 in this board.
#endif

#if  (!defined(CONFIG_NRF52_SPI2)) && (BMM150_SPI_PORTNO == 2)
#  error CONFIG_NRF52_SPI2 is required if you want to use BMM150 in this board.
#endif

#elif defined (CONFIG_BMM150_INTF_I2C)

#if  (!defined(CONFIG_NRF52_I2C0)) && (BMM150_I2C_PORTNO == 0)
#  error CONFIG_NRF52_I2C0 is required if you want to use BMM150 in this board.
#endif

#if  (!defined(CONFIG_NRF52_I2C1)) && (BMM150_I2C_PORTNO == 1)
#  error CONFIG_NRF52_I2C1 is required if you want to use BMM150 in this board.
#endif

#else

# error either CONFIG_BMM150_INTF_SPI or CONFIG_BMM150_INTF_I2C should be enabled if you want to communicate with BMM150.

#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static void bmm150_INT_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
static void bmm150_DRDY_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
static int bmm150_cfg_INT(bmm150_gpio_int_cb_t cb, void *param);
static int bmm150_enable_INT(bool enable);
static int bmm150_cfg_DRDY(bmm150_gpio_int_cb_t cb, void *param);
static int bmm150_enable_DRDY(bool enable);


/****************************************************************************
 * Private Data
 ****************************************************************************/
const static struct bmm150_low_level_operations_s pin_op =
{
#if (defined BMM150_DRDY_PIN) && (BMM150_DRDY_PIN != NRF52_INVALID_GPIO_PIN)
  .bmm150_DRDY_cfg_interrupt = bmm150_cfg_DRDY,
  .bmm150_DRDY_enable = bmm150_enable_DRDY,
#else
  .bmm150_DRDY_cfg_interrupt = NULL,
  .bmm150_DRDY_enable = NULL,
#endif

#if (defined BMM150_INT_PIN) && (BMM150_INT_PIN != NRF52_INVALID_GPIO_PIN)
  .bmm150_INT_cfg_interrupt = bmm150_cfg_INT,
  .bmm150_INT_enable = bmm150_enable_INT,
#else
  .bmm150_INT_cfg_interrupt = NULL,
  .bmm150_INT_enable = NULL,
#endif

};

static bmm150_gpio_int_cb_t bmm150_INT_pin_cb = NULL;
static void *INT_pin_cb_param = NULL;

static bmm150_gpio_int_cb_t bmm150_DRDY_pin_cb = NULL;
static void *DRDY_pin_cb_param = NULL;


/****************************************************************************
 * Private Functions
 ****************************************************************************/
static void bmm150_INT_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  bmm150_INT_pin_cb(INT_pin_cb_param);
}


static void bmm150_DRDY_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  bmm150_DRDY_pin_cb(DRDY_pin_cb_param);
}



static int bmm150_cfg_INT(bmm150_gpio_int_cb_t cb, void *param)
{
  int ret = OK;
  nrf_drv_gpiote_in_config_t pin_cfg = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);

  bmm150_INT_pin_cb = cb;
  INT_pin_cb_param = param;
  ret = nrf_drv_gpiote_in_init(BMM150_INT_PIN, &pin_cfg, bmm150_INT_pin_handler);
  ret = nrf_sdk_retcode_to_nuttx(ret);

  return ret;
}

static int bmm150_cfg_DRDY(bmm150_gpio_int_cb_t cb, void *param)
{
  int ret = OK;
  nrf_drv_gpiote_in_config_t pin_cfg = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);

  bmm150_DRDY_pin_cb = cb;
  DRDY_pin_cb_param = param;
  ret = nrf_drv_gpiote_in_init(BMM150_DRDY_PIN, &pin_cfg, bmm150_DRDY_pin_handler);
  ret = nrf_sdk_retcode_to_nuttx(ret);

  return ret;
}

static int bmm150_enable_INT(bool enable)
{
  if (enable)
    {
      nrf_drv_gpiote_in_event_enable(BMM150_INT_PIN, true);
    }
  else
    {
      nrf_drv_gpiote_in_event_disable(BMM150_INT_PIN);
    }
  return OK;
}


static int bmm150_enable_DRDY(bool enable)
{
  if (enable)
    {
      nrf_drv_gpiote_in_event_enable(BMM150_DRDY_PIN, true);
    }
  else
    {
      nrf_drv_gpiote_in_event_disable(BMM150_DRDY_PIN);
    }

  return OK;
}




/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: nrf52_bmm150_initialize
 *
 * Description:
 *   Initialize and register the bmm150 magnetometer driver.
 *
 ************************************************************************************/
int nrf52_bmm150_initialize(void)
{
  int ret;

#ifdef CONFIG_BMM150_INTF_SPI
  FAR struct spi_dev_s *spi;

  spi = nrf52_spibus_initialize(BMM150_SPI_PORTNO, true);

  if (!spi)
    {
      snerr("ERROR: FAILED to initialize SPI port\n");
      return -ENODEV;
    }

  struct bmm150_bus_config_s bmm150_bus_cfg =
  {
    .bus_selection = BMM150_USE_SPI,
    .bus_config.spi_config =
    {
      .spi = spi,
#if defined(CONFIG_BMM150_SPI_125K)
      .spi_frequency = 125000,
#elif defined(CONFIG_BMM150_SPI_250K)
      .spi_frequency = 250000,
#elif defined(CONFIG_BMM150_SPI_500K)
      .spi_frequency = 500000,
#elif defined(CONFIG_BMM150_SPI_1M)
      .spi_frequency = 1000000,
#elif defined(CONFIG_BMM150_SPI_2M)
      .spi_frequency = 2000000,
#elif defined(CONFIG_BMM150_SPI_4M)
      .spi_frequency = 4000000,
#elif defined(CONFIG_BMM150_SPI_8M)
      .spi_frequency = 8000000,
#endif
      .spi_devid = SPIDEV_MAGNETOMETER(0),
      .spi_mode = SPIDEV_MODE3,
    },
  };
#elif defined(CONFIG_BMM150_INTF_I2C)

  FAR struct i2c_master_s *i2c;
  /* Initialize I2C */
  i2c = nrf52_i2cbus_initialize(BMM150_I2C_PORTNO);

  if (!i2c)
    {
      snerr("ERROR: FAILED to initialize i2c port\n");
      return -ENODEV;
    }

  struct bmm150_bus_config_s bmm150_bus_cfg =
  {
    .bus_selection = BMM150_USE_I2C,
    .bus_config.i2c_config =
    {
      .i2c = i2c,
      .config =
      {
        .frequency = CONFIG_BMM150_I2C_SPEED,
#if defined(CONFIG_BMM150_I2C_ADDR_0x10)
        .address = 0x10,
#elif defined(CONFIG_BMM150_I2C_ADDR_0x11)
        .address = 0x11,
#elif defined(CONFIG_BMM150_I2C_ADDR_0x12)
        .address = 0x12,
#elif defined(CONFIG_BMM150_I2C_ADDR_0x13)
        .address = 0x13,
#endif
        .addrlen = 7,
      },
    },
  };
#endif

  ret = bmm150_register("/dev/mag0", &bmm150_bus_cfg, &pin_op);

  if (ret < 0)
    {
      snerr("ERROR: Error registering bmm150\n");
    }

  return ret;
}



#endif /* CONFIG_BMM150 */
