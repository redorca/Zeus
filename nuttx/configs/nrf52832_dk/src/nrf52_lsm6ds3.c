/****************************************************************************
 * configs/nrf52832_dk/src/nrf52_lsm6ds3.c
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

#include <nuttx/sensors/lsm6ds3.h>

#ifdef CONFIG_LSM6DS3

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
#ifdef CONFIG_LSM6DS3_INTF_SPI

#if  (!defined(CONFIG_NRF52_SPI0)) && (LSM6DS3_SPI_PORTNO == 0)
#  error CONFIG_NRF52_SPI0 is required if you want to use LSM6DS3 in this board.
#endif

#if  (!defined(CONFIG_NRF52_SPI1)) && (LSM6DS3_SPI_PORTNO == 1)
#  error CONFIG_NRF52_SPI1 is required if you want to use LSM6DS3 in this board.
#endif

#if  (!defined(CONFIG_NRF52_SPI2)) && (LSM6DS3_SPI_PORTNO == 2)
#  error CONFIG_NRF52_SPI2 is required if you want to use LSM6DS3 in this board.
#endif

#elif defined (CONFIG_LSM6DS3_INTF_I2C)

#if  (!defined(CONFIG_NRF52_I2C0)) && (LSM6DS3_I2C_PORTNO == 0)
#  error CONFIG_NRF52_I2C0 is required if you want to use LSM6DS3 in this board.
#endif

#if  (!defined(CONFIG_NRF52_I2C1)) && (LSM6DS3_I2C_PORTNO == 1)
#  error CONFIG_NRF52_I2C1 is required if you want to use LSM6DS3 in this board.
#endif

#else

# error either CONFIG_LSM6DS3_INTF_SPI or CONFIG_LSM6DS3_INTF_I2C should be enabled if you want to communicate with LSM6DS3.

#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/


/****************************************************************************
 * Private Data
 ****************************************************************************/



/****************************************************************************
 * Private Functions
 ****************************************************************************/


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: nrf52_lsm6ds3_initialize
 *
 * Description:
 *   Initialize and register the lsm6ds3 accelerometer and gyro comb device driver.
 *
 ************************************************************************************/
int nrf52_lsm6ds3_initialize(void)
{
  int ret;

#ifdef CONFIG_LSM6DS3_INTF_SPI
  FAR struct spi_dev_s *spi;

  spi = nrf52_spibus_initialize(LSM6DS3_SPI_PORTNO, true);

  if (!spi)
    {
      snerr("ERROR: FAILED to initialize SPI port\n");
      return -ENODEV;
    }

  struct lsm6ds3_bus_config_s lsm6ds3_bus_cfg =
  {
    .bus_selection = LSM6DS3_SPI_INTF,
    .bus_config.spi_config =
    {
      .spi = spi,
#if defined(CONFIG_LSM6DS3_SPI_125K)
      .spi_frequency = 125000,
#elif defined(CONFIG_LSM6DS3_SPI_250K)
      .spi_frequency = 250000,
#elif defined(CONFIG_LSM6DS3_SPI_500K)
      .spi_frequency = 500000,
#elif defined(CONFIG_LSM6DS3_SPI_1M)
      .spi_frequency = 1000000,
#elif defined(CONFIG_LSM6DS3_SPI_2M)
      .spi_frequency = 2000000,
#elif defined(CONFIG_LSM6DS3_SPI_4M)
      .spi_frequency = 4000000,
#elif defined(CONFIG_LSM6DS3_SPI_8M)
      .spi_frequency = 8000000,
#endif
      .spi_devid = SPIDEV_ACC_GYRO_COMB(0),
      .spi_mode = SPIDEV_MODE3,
    },
  };
#elif defined(CONFIG_LSM6DS3_INTF_I2C)

  FAR struct i2c_master_s *i2c;
  /* Initialize I2C */
  i2c = nrf52_i2cbus_initialize(LSM6DS3_I2C_PORTNO);

  if (!i2c)
    {
      snerr("ERROR: FAILED to initialize i2c port\n");
      return -ENODEV;
    }

  struct lsm6ds3_bus_config_s lsm6ds3_bus_cfg =
  {
    .bus_selection = LSM6DS3_I2C_INTF,
    .bus_config.i2c_config =
    {
      .i2c = i2c,
      .config =
      {
        .frequency = CONFIG_LSM6DS3_I2C_SPEED,
#if defined(CONFIG_LSM6DS3_I2C_ADDR_0x6A)
        .address = 0x6A,
#elif defined(CONFIG_LSM6DS3_I2C_ADDR_0x6B)
        .address = 0x6B,
#endif
        .addrlen = 7,
      },
    },
  };
#endif

  ret = lsm6ds3_register("/dev/lsm6ds3", &lsm6ds3_bus_cfg);

  if (ret < 0)
    {
      snerr("ERROR: Error registering lsm6ds3\n");
    }

  return ret;
}



#endif /* CONFIG_LSM6DS3 */
