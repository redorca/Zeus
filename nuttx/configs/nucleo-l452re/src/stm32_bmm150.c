/****************************************************************************
 * configs/nucleo-l452re/src/stm32_spi.c
 *
 *   Copyright (C) 2014, 2018 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi/spi.h>

#include <up_arch.h>
#include <chip.h>
#include <stm32l4.h>

#include "nucleo-l452re.h"

#include <arch/board/board.h>

#include <nuttx/sensors/bmm050.h>

#ifdef CONFIG_BMM150

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/*If you change this, you should also check the corresponding stm32l4_spixselect function
 *in stm32_spi.c
 */
#define BMM150_SPI_PORTNO 1   /* On SPI1 */

#if  (!defined(CONFIG_STM32L4_SPI1)) && (BMM150_SPI_PORTNO == 1)
#  error CONFIG_STM32L4_SPI1 is required if you want to use bmm150 in this board.
#endif

#if  (!defined(CONFIG_STM32L4_SPI2)) && (BMM150_SPI_PORTNO == 2)
#  error CONFIG_STM32L4_SPI1 is required if you want to use bmm150 in this board.
#endif

#if  (!defined(CONFIG_STM32L4_SPI3)) && (BMM150_SPI_PORTNO == 3)
#  error CONFIG_STM32L4_SPI1 is required if you want to use bmm150 in this board.
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: stm32l4_bmm150_setup
 *
 * Description:
 *   Initialize and register the bmm150 magnetometer driver.
 *
 ************************************************************************************/

int stm32l4_bmm150_setup(void)
{
  int ret;
  FAR struct spi_dev_s *spi;
  struct bmm050_config_s bmm050_config =
  {
    .spi_devid = SPIDEV_MAGNETOMETER(0),
#ifdef CONFIG_BMM150_INTERRUPT_ENABLE
    /*issue:we haven't implement the interrupt mode until now*/
    .irq = 0;
    .attach = NULL,
#else
    .attach = NULL,
#endif

  };


  spi = stm32l4_spibus_initialize(BMM150_SPI_PORTNO);
  if (!spi)
    {
      snerr("ERROR: FAILED to initialize SPI port 1\n");
      return -ENODEV;
    }

  ret = bmm050_register("/dev/mag0", spi, &bmm050_config);

  if (ret < 0)
    {
      snerr("ERROR: Error registering the BMM150\n");
    }

  return ret;
}



/************************************************************************************
 * Public Functions
 ************************************************************************************/



#endif /* CONFIG_BMM150 */
