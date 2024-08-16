/****************************************************************************
 *   configs/fast_nrf52832_dk/src/nrf52_user_spi.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2017 Zglue  Inc. All rights reserved.
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
#include <arch/board/board.h>

#include <up_arch.h>
#include <chip.h>
#include "nrf52_gpio.h"

#include "nrf52_spi.h"


/************************************************************************************
 * Public Data
 ************************************************************************************/
/* Global driver instances */


/************************************************************************************
 * Public Functions
 ************************************************************************************/

static void user_cs_select(uint32_t cs_pin, bool selected)
{
  if (cs_pin >= NRF52_INVALID_GPIO_PIN)
    {
      spiinfo("Wrong CS_PIN configuration.\n");
      return;
    }

  if (selected)
    {
      nrf_gpio_pin_clear(cs_pin);
    }
  else
    {
      nrf_gpio_pin_set(cs_pin);
    }

  return;
}

static void user_cs_config(uint32_t cs_pin)
{
  if (cs_pin >= NRF52_INVALID_GPIO_PIN)
    {
      spiinfo("Wrong CS_PIN configuration.\n");
      return;
    }
  nrf_gpio_cfg_output(cs_pin);
  nrf_gpio_pin_set(cs_pin);

  return;
}

#if defined(CONFIG_NRF52_SPI2)
static void nrf52_spi2_select(uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

  if (SPIDEVTYPE_ZGLUE_FAST == SPIDEVID_TYPE(devid))
    {
      user_cs_select(BOARD_FAST_CS, selected);
    }
  if (SPIDEVTYPE_HEARTRATE == SPIDEVID_TYPE(devid))
    {
      user_cs_select(BOARD_MAX86140_CS, selected);
    }
#ifdef CONFIG_BMM150
  if (SPIDEVTYPE_MAGNETOMETER ==  SPIDEVID_TYPE(devid))
    {
      user_cs_select(BOARD_BMM150_CS, selected);
    }
#endif

}
#endif

static void customer_spi_select(nrf_spi_bus_t busid, uint32_t devid, bool selected)
{
  spiinfo("busid: %d, devid: %d CS: %s\n", busid, (int)devid, selected ? "assert" : "de-assert");

#if defined(CONFIG_NRF52_SPI2)
  if (SPI_NRF52_BUS_2 == busid)
    {
      nrf52_spi2_select(devid, selected);
      return;
    }
#endif
  /*if running here , means error */
  spierr("Wrong Bus ID %d.\n", busid);

  return;
}

void customer_spi_cs_select(void)
{
#if defined(CONFIG_NRF52_SPI2)
  user_cs_config(BOARD_FAST_CS);
  user_cs_config(BOARD_MAX86140_CS);
#ifdef CONFIG_BMM150
  user_cs_config(BOARD_BMM150_CS);
#endif
#endif
  nrf52_spibus_register(customer_spi_select);
}

