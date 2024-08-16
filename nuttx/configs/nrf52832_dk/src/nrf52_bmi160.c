/****************************************************************************
 * configs/nrf52832_dk/src/nrf52_bmi160.c
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
#include "nrf_error.h"

#include <nuttx/sensors/bmi160.h>

#ifdef CONFIG_BMI160

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
#ifdef CONFIG_BMI160_INTF_SPI

#if  (!defined(CONFIG_NRF52_SPI0)) && (BMI160_SPI_PORTNO == 0)
#  error CONFIG_NRF52_SPI0 is required if you want to use BMI160 in this board.
#endif

#if  (!defined(CONFIG_NRF52_SPI1)) && (BMI160_SPI_PORTNO == 1)
#  error CONFIG_NRF52_SPI1 is required if you want to use BMI160 in this board.
#endif

#if  (!defined(CONFIG_NRF52_SPI2)) && (BMI160_SPI_PORTNO == 2)
#  error CONFIG_NRF52_SPI2 is required if you want to use BMI160 in this board.
#endif

#elif defined (CONFIG_BMI160_INTF_I2C)

#if  (!defined(CONFIG_NRF52_I2C0)) && (BMI160_I2C_PORTNO == 0)
#  error CONFIG_NRF52_I2C0 is required if you want to use BMI160 in this board.
#endif

#if  (!defined(CONFIG_NRF52_I2C1)) && (BMI160_I2C_PORTNO == 1)
#  error CONFIG_NRF52_I2C1 is required if you want to use BMI160 in this board.
#endif

#else

# error either CONFIG_BMI160_INTF_SPI or CONFIG_BMI160_INTF_I2C should be enabled if you want to communicate with BMI160.

#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static void bmi160_int_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
static int bmi160_attach_gpio_irq(FAR enum bmi160_int_channel int_channel, bmi160_int_cb_t cb);
static int bmi160_enable_int_pin(enum bmi160_int_channel int_channel, bool enable);


/****************************************************************************
 * Private Data
 ****************************************************************************/

static bmi160_int_cb_t bmi160_int_1_cb = NULL;
static bmi160_int_cb_t bmi160_int_2_cb = NULL;

FAR const struct bmi160_low_level_operations_s bmi160_ll_op =
{
  bmi160_attach_gpio_irq,
  bmi160_enable_int_pin,
};



/****************************************************************************
 * Private Functions
 ****************************************************************************/
static void bmi160_int_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  if (action == NRF_GPIOTE_POLARITY_LOTOHI)
    {
      if ((pin == BMI160_INT1_PIN) && (bmi160_int_1_cb != NULL))
        {
          bmi160_int_1_cb(BMI160_INT_CHANNEL_1);
        }
      else if ((pin == BMI160_INT2_PIN) && (bmi160_int_2_cb != NULL))
        {
          bmi160_int_2_cb(BMI160_INT_CHANNEL_2);
        }
    }
}

static int bmi160_attach_gpio_irq(FAR enum bmi160_int_channel int_channel, bmi160_int_cb_t cb)
{
  uint32_t pin_no;
  int ret = OK;
  nrf_drv_gpiote_in_config_t pin_cfg = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);


  if (int_channel == BMI160_INT_CHANNEL_1)
    {
      pin_no = BMI160_INT1_PIN;
      bmi160_int_1_cb = cb;
      ret = nrf_drv_gpiote_in_init(pin_no, &pin_cfg, bmi160_int_pin_handler);
      ret = nrf_sdk_retcode_to_nuttx(ret);
    }
  else if (int_channel == BMI160_INT_CHANNEL_2)
    {
      pin_no = BMI160_INT2_PIN;
      bmi160_int_2_cb = cb;
      ret = nrf_drv_gpiote_in_init(pin_no, &pin_cfg, bmi160_int_pin_handler);
      ret = nrf_sdk_retcode_to_nuttx(ret);
    }
  else
    {
      snerr("ERROR: please choose between INT_CHANNEL_1 and INT_CHANNEL_2\n");
      ret = -EINVAL;
    }

  return ret;
}

static int bmi160_enable_int_pin(enum bmi160_int_channel int_channel, bool enable)
{

  switch (int_channel)
    {
      case BMI160_INT_CHANNEL_1:
        {
          if (enable)
            {
              nrf_drv_gpiote_in_event_enable(BMI160_INT1_PIN, true);
            }
          else
            {
              nrf_drv_gpiote_in_event_disable(BMI160_INT1_PIN);
            }
        }
        break;
      case BMI160_INT_CHANNEL_2:
        {
          if (enable)
            {
              nrf_drv_gpiote_in_event_enable(BMI160_INT2_PIN, true);
            }
          else
            {
              nrf_drv_gpiote_in_event_disable(BMI160_INT2_PIN);
            }
        }
        break;
      case BMI160_INT_CHANNEL_BOTH:
        {
          if (enable)
            {
              nrf_drv_gpiote_in_event_enable(BMI160_INT1_PIN, true);
              nrf_drv_gpiote_in_event_enable(BMI160_INT2_PIN, true);
            }
          else
            {
              nrf_drv_gpiote_in_event_disable(BMI160_INT1_PIN);
              nrf_drv_gpiote_in_event_disable(BMI160_INT2_PIN);
            }
        }
        break;
      default:
        break;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: nrf52_bmi160_initialize
 *
 * Description:
 *   Initialize and register the bmi160 accelerometer and gyro comb device driver.
 *
 ************************************************************************************/
int nrf52_bmi160_initialize(void)
{
  int ret;

#ifdef CONFIG_BMI160_INTF_SPI
  FAR struct spi_dev_s *spi;

  spi = nrf52_spibus_initialize(BMI160_SPI_PORTNO, true);

  if (!spi)
    {
      snerr("ERROR: FAILED to initialize SPI port\n");
      return -ENODEV;
    }

  struct bmi160_bus_config_s bmi160_bus_cfg =
  {
    .bus_selection = BMI160_SPI_INTF,
    .bus_config.spi_config =
    {
      .spi = spi,
#if defined(CONFIG_BMI160_SPI_125K)
      .spi_frequency = 125000,
#elif defined(CONFIG_BMI160_SPI_250K)
      .spi_frequency = 250000,
#elif defined(CONFIG_BMI160_SPI_500K)
      .spi_frequency = 500000,
#elif defined(CONFIG_BMI160_SPI_1M)
      .spi_frequency = 1000000,
#elif defined(CONFIG_BMI160_SPI_2M)
      .spi_frequency = 2000000,
#elif defined(CONFIG_BMI160_SPI_4M)
      .spi_frequency = 4000000,
#elif defined(CONFIG_BMI160_SPI_8M)
      .spi_frequency = 8000000,
#endif
      .spi_devid = SPIDEV_ACC_GYRO_COMB(0),
      .spi_mode = SPIDEV_MODE3,
    },
  };
#elif defined(CONFIG_BMI160_INTF_I2C)

  FAR struct i2c_master_s *i2c;
  /* Initialize I2C */
  i2c = nrf52_i2cbus_initialize(BMI160_I2C_PORTNO);

  if (!i2c)
    {
      snerr("ERROR: FAILED to initialize i2c port\n");
      return -ENODEV;
    }

  struct bmi160_bus_config_s bmi160_bus_cfg =
  {
    .bus_selection = BMI160_I2C_INTF,
    .bus_config.i2c_config =
    {
      .i2c = i2c,
      .config =
      {
        .frequency = CONFIG_BMI160_I2C_SPEED,
#if defined(CONFIG_BMI160_I2C_ADDR_0x68)
        .address = 0x68,
#elif defined(CONFIG_BMI160_I2C_ADDR_0x69)
        .address = 0x69,
#endif
        .addrlen = 7,
      },
    },
  };
#endif

  ret = bmi160_register("/dev/bmi160", &bmi160_bus_cfg, &bmi160_ll_op);

  if (ret < 0)
    {
      snerr("ERROR: Error registering bmi160\n");
    }

  return ret;
}



#endif /* CONFIG_BMI160 */
