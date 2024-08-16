/************************************************************************************
 * arch/arm/src/nrf52/nrf52_spi.h
 *
 *   Copyright (C) 2009, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Copyright (C) 2017 Zglue  Inc. All rights reserved.
 *           Arjun Hary   <arjun@zglue.com>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_NRF52_NRF52832_SPI_H
#define __ARCH_ARM_SRC_NRF52_NRF52832_SPI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/
/* EasyDMA spi hardware had issue for one byte transfer , so we use legacy driver
 * without EasyDMA feature
 */
struct spi_pinmux_t
{
  uint8_t      cs_pin;      /* CS GPIO pin number */
  uint8_t      miso_pin;    /* MISO GPIO pin number */
  uint8_t      mosi_pin;    /* MOSI GPIO pin number */
  uint8_t      sck_pin;     /* Clock GPIO pin number */
};

#ifdef CONFIG_NRF52_SPI
/************************************************************************************
 * Public Functions
 ************************************************************************************/

/****************************************************************************
 * Name: nrf_drv_easy_dma_mode
 *
 * Description:
 *   selects whether easy_dma or legacy spi driver is used
 *
 ****************************************************************************/

uint32_t nrf_drv_easy_dma_mode(FAR struct spi_dev_s *priv, bool easy_dma_mode_flag);

/************************************************************************************
 * Name: nrf52_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus
 *
 * Input Parameter:
 *   bus number (for hardware that has mutiple SPI interfaces)
 *   reset_cs_pin_flag (if true, calling this function also pulls cs_pin high)
 *
 * Returned Value:
 *   Valid SPI device structure reference on succcess; a NULL on failure
 *
 ************************************************************************************/

FAR struct spi_dev_s *nrf52_spibus_initialize(int bus, bool reset_cs_pin_flag);

/************************************************************************************
 * Name: nrf52_spibus_register
 *
 * Description:
 *   register the cs selection customization api
 *
 * Input Parameter:
 *   bus number (for hardware that has mutiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on succcess; a NULL on failure
 *
 ************************************************************************************/

typedef void  (*SPI_CS_CallBack)(nrf_spi_bus_t busid, uint32_t devid, bool selected);

FAR int nrf52_spibus_register(SPI_CS_CallBack cs_select);

#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_NRF52_NRF52832_SPI_H */

