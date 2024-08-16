/*****************************************************************************
 *   configs/fast_nrf52832_dk/include/board.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2016 Motorola Mobility, LLC.
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
 *****************************************************************************/
/*****************************************************************************
 *
 * Copyrifht (C) 2017 zGlue Inc.  All rights reserved.
 *
 *****************************************************************************/

#ifndef __CONFIGS_NRF52382_MDK_INCLUDE_BOARD_H
#define __CONFIGS_NRF52382_MDK_INCLUDE_BOARD_H

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
# include <time.h>
#endif

#include "pca10040.h"

/*****************************************************************************
 * Definitions
 *****************************************************************************/
#define NRF52832_DK_SYSCLK_FREQUENCY  64000000ul
#define NRF52_HCLK_FREQUENCY    NRF52832_DK_SYSCLK_FREQUENCY

/* Clocking ******************************************************************/

/* The desired timer interrupt frequency is provided by the definition
 * CLK_TCK (see include/time.h).  CLK_TCK defines the desired number of
 * system clock ticks per second.  That value is a user configurable setting
 * that defaults to 100 (100 ticks per second = 10 MS interval).
 */

#define SYSTICK_RELOAD ((NRF52_HCLK_FREQUENCY / CLK_TCK) - 1)

/* The size of the reload field is 24 bits.  Verify that the reload value
 * will fit in the reload register.
 */

#if SYSTICK_RELOAD > 0x00ffffff
#error "SYSTICK_RELOAD exceeds the range of the RELOAD register"
#endif

/* Alternate function pin selections *****************************************/


/* UART I2C SPI Pins *********************************************************/
/*
 * The following definitions must be provided so that the NRF52 serial
 * driver can set up the UART for the serial console properly.
 * You can modify the pin according to your board spec and usage.
 */

#define HWFC           true
/* UART serial pin config */
#define BOARD_UART0_RX_PIN    13//(GPIO_INPUT  | GPIO_PIN23)
#define BOARD_UART0_TX_PIN    12//(GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PIN24)
#define BOARD_UART0_CTS_PIN   7//0xFF
#define BOARD_UART0_RTS_PIN   5//0xFF

/*Now there is NO uart1 , only 52840 chip had two uart */
#define BOARD_UART1_RX_PIN    NRF52_INVALID_GPIO_PIN
#define BOARD_UART1_TX_PIN    NRF52_INVALID_GPIO_PIN
#define BOARD_UART1_CTS_PIN   NRF52_INVALID_GPIO_PIN
#define BOARD_UART1_RTS_PIN   NRF52_INVALID_GPIO_PIN

/* I2C pin config */
#ifdef CONFIG_NRF52_I2C0_400K
#define BOARD_I2C0_FREQ       NRF_TWIM_FREQ_400K
#elif CONFIG_NRF52_I2C0_250K
#define BOARD_I2C0_FREQ       NRF_TWIM_FREQ_250K
#elif CONFIG_NRF52_I2C0_100K
#define BOARD_I2C0_FREQ       NRF_TWIM_FREQ_100K
#endif

#ifdef CONFIG_NRF52_I2C0
/* you should define your i2c0 buffer sda & scl here */
#define BOARD_I2C0_SDA_PIN    0 // CONFIG_NRF52_I2C0_SDA_PIN
#define BOARD_I2C0_SCL_PIN    1 // CONFIG_NRF52_I2C0_SCL_PIN

#endif

#ifdef CONFIG_NRF52_I2C1_400K
#define BOARD_I2C1_FREQ       NRF_TWIM_FREQ_400K
#elif CONFIG_NRF52_I2C1_250K
#define BOARD_I2C1_FREQ       NRF_TWIM_FREQ_250K
#elif CONFIG_NRF52_I2C1_100K
#define BOARD_I2C1_FREQ       NRF_TWIM_FREQ_100K
#endif

#ifdef CONFIG_NRF52_I2C1
/* you should define your i2c1 buffer sda & scl here */
#define BOARD_I2C1_SDA_PIN    0 // CONFIG_NRF52_I2C1_SDA_PIN
#define BOARD_I2C1_SCL_PIN    1 // CONFIG_NRF52_I2C1_SCL_PIN
#endif

/* SPI pin config : cs pin is controlled by user*/
#define SPI0_FLASH_CS         24
#define SPI1_FLASH_CS         24
#define SPI2_FLASH_CS         24


#define BOARD_SPI0_MISO_PIN   25
#define BOARD_SPI0_MOSI_PIN   23
#define BOARD_SPI0_SCL_PIN    26

#define BOARD_SPI1_MISO_PIN   NRF52_INVALID_GPIO_PIN
#define BOARD_SPI1_MOSI_PIN   NRF52_INVALID_GPIO_PIN
#define BOARD_SPI1_SCL_PIN    NRF52_INVALID_GPIO_PIN

#define BOARD_SPI2_MISO_PIN   6 /* CONFIG_NRF52_SPI2_MISO_PIN */
#define BOARD_SPI2_MOSI_PIN   7 /* CONFIG_NRF52_SPI2_MOSI_PIN */
#define BOARD_SPI2_SCL_PIN    8 /* CONFIG_NRF52_SPI2_SCL_PIN */


/* FAST JTAG pins*/
#define BOARD_JTAG_TMS_PIN 29
#define BOARD_JTAG_TCK_PIN 28
#define BOARD_JTAG_TDI_PIN 31
#define BOARD_JTAG_TDO_PIN 30


/* FAST board pin */
#define BOARD_FAST_POWER_PIN            11
#define BOARD_FAST_READY_PIN            20
#define BOARD_FAST_PKBYP_PIN            22
#define BOARD_FAST_PWR_CLK_RDY_PIN      23
#define BOARD_FAST_G_COL_PIN            24
#define BOARD_FAST_D_COL_PIN            25
#define BOARD_FAST_EXT_RESETB_PIN       26
#define BOARD_FAST_EXT_RESETIOB_PIN     27



#define BOARD_PCA10040  1
/*****************************************************************************
 * Public Data
 *****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#define ENABLED(a)  defined(a) && a == 1
/*****************************************************************************
 * Public Function Prototypes
 *****************************************************************************/
/*****************************************************************************
 * Name: nrf52_boardinitialize
 *
 * Description:
 *   All nRF52 architectures must provide the following entry point.  This entry
 *   point is called early in the initialization -- after all memory has been
 *   configured and mapped but before any devices have been initialized.
 *
 *****************************************************************************/

void nrf52_boardinitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIGS_STM32L476_MDK_INCLUDE_BOARD_H */
