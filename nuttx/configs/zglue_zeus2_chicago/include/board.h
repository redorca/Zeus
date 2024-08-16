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

#ifndef __CONFIGS_ZGLUE_ZEUS2_CHICAGO_BOARD_H
#define __CONFIGS_ZGLUE_ZEUS2_CHICAGO_BOARD_H

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

/* UART I2C SPI Pins *********************************************************/
/*
 * The following definitions must be provided so that the NRF52 serial
 * driver can set up the UART for the serial console properly.
 * You can modify the pin according to your board spec and usage.
 */

#define HWFC           false
/* UART serial pin config */
#define BOARD_UART0_RX_PIN    23//(GPIO_INPUT  | GPIO_PIN23)
#define BOARD_UART0_TX_PIN    24//(GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PIN24)
#define BOARD_UART0_CTS_PIN   NRF52_INVALID_GPIO_PIN
#define BOARD_UART0_RTS_PIN   NRF52_INVALID_GPIO_PIN

#if defined(CONFIG_ARCH_LOWPUTC)
#define BOARD_LOWPUTC_RX_PIN    BOARD_UART0_RX_PIN//(GPIO_INPUT  | GPIO_PIN23)
#define BOARD_LOWPUTC_TX_PIN    BOARD_UART0_TX_PIN//(GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PIN24)
#define BOARD_LOWPUTC_CTS_PIN   NRF52_INVALID_GPIO_PIN//0xFF
#define BOARD_LOWPUTC_RTS_PIN   NRF52_INVALID_GPIO_PIN//0xFF

#define CONFIG_LOWPUTC_PARITY   0
#define CONFIG_LOWPUTC_BITS     8
#define CONFIG_LOWPUTC_2STOP    0
#define CONFIG_LOWPUTC_BAUD     115200
#endif

/*Now there is NO uart1 , only 52840 chip had two uart */
#define BOARD_UART1_RX_PIN    NRF52_INVALID_GPIO_PIN
#define BOARD_UART1_TX_PIN    NRF52_INVALID_GPIO_PIN
#define BOARD_UART1_CTS_PIN   NRF52_INVALID_GPIO_PIN
#define BOARD_UART1_RTS_PIN   NRF52_INVALID_GPIO_PIN

/* I2C pin config */
#ifdef CONFIG_NRF52_I2C0_400K
#define BOARD_I2C0_FREQ       400000
#elif CONFIG_NRF52_I2C0_250K
#define BOARD_I2C0_FREQ       250000
#elif CONFIG_NRF52_I2C0_100K
#define BOARD_I2C0_FREQ       100000
#endif

#ifdef CONFIG_NRF52_I2C0
/* you should define your i2c0 sda & scl here */
#define BOARD_I2C0_SDA_PIN    7 // CONFIG_NRF52_I2C0_SDA_PIN
#define BOARD_I2C0_SCL_PIN    6 // CONFIG_NRF52_I2C0_SCL_PIN
#endif

#ifdef CONFIG_NRF52_I2C1_400K
#define BOARD_I2C1_FREQ       400000
#elif CONFIG_NRF52_I2C1_250K
#define BOARD_I2C1_FREQ       250000
#elif CONFIG_NRF52_I2C1_100K
#define BOARD_I2C1_FREQ       100000
#endif

#ifdef CONFIG_NRF52_I2C1
/* you should define your i2c1 sda & scl here */
#define BOARD_I2C1_SDA_PIN    12 // CONFIG_NRF52_I2C1_SDA_PIN
#define BOARD_I2C1_SCL_PIN    11 // CONFIG_NRF52_I2C1_SCL_PIN
#endif

/* FAST API ULPM wake up pin */
#define BOARD_ULPM_WAKEUP_PIN     31

/* SPI pin config */
#define BOARD_FAST_CS         26
#define BOARD_MAX86140_CS     2


#define BOARD_SPI0_MISO_PIN   NRF52_INVALID_GPIO_PIN
#define BOARD_SPI0_MOSI_PIN   NRF52_INVALID_GPIO_PIN
#define BOARD_SPI0_SCL_PIN    NRF52_INVALID_GPIO_PIN

#define BOARD_SPI1_MISO_PIN   NRF52_INVALID_GPIO_PIN
#define BOARD_SPI1_MOSI_PIN   NRF52_INVALID_GPIO_PIN
#define BOARD_SPI1_SCL_PIN    NRF52_INVALID_GPIO_PIN

#define BOARD_SPI2_MISO_PIN   3 /* CONFIG_NRF52_SPI2_MISO_PIN */
#define BOARD_SPI2_MOSI_PIN   4 /* CONFIG_NRF52_SPI2_MOSI_PIN */
#define BOARD_SPI2_SCL_PIN    5 /* CONFIG_NRF52_SPI2_SCL_PIN */

#define BOARD_VIBRATOR    22

#define BOARD_TMP108_ALERT (22)

#define BOARD_MAX86140_INT (13)

#ifdef CONFIG_BMM150

/*bmm150 SPI or I2C port config*/
#ifdef CONFIG_BMM150_INTF_SPI
#define BMM150_SPI_PORTNO 2   /* On SPI1 */
#elif defined CONFIG_BMM150_INTF_I2C
#define BMM150_I2C_PORTNO 0   /* On I2C1 */
#endif
#define BOARD_BMM150_CS     20
#define BMM150_INT_PIN       14
#define BMM150_DRDY_PIN       28
#endif

#ifdef CONFIG_LSM6DS3

/*LSM6DS3 SPI or I2C port config*/
#ifdef CONFIG_LSM6DS3_INTF_SPI
#define LSM6DS3_SPI_PORTNO 2   /* On SPI1 */
#elif defined CONFIG_LSM6DS3_INTF_I2C
#define LSM6DS3_I2C_PORTNO 0   /* On I2C1 */
#endif

#endif

#ifdef CONFIG_MC3672

/*mc3672 SPI or I2C port config*/
#ifdef CONFIG_MC3672_INTF_SPI
#define MC3672_SPI_PORTNO 0   /* On SPI1 */
#elif defined CONFIG_MC3672_INTF_I2C
#define MC3672_I2C_PORTNO 0   /* On I2C1 */
#endif

#define MC3672_INT_PIN       8

#endif

#ifdef CONFIG_BQ2512X

/*BQ2512X I2C port config*/
#define BQ2512X_I2C_PORTNO    1   /* On I2C1 */

#define BQ2512X_INT_PIN       15
#define BQ2512X_RESET_PIN     16
#define BQ2512X_LSCTRL_PIN    17
#define BQ2512X_NPG_PIN       19


#endif


/*
 *
 * CMD:           BOARDIOC_G_VERSION
 * DESCRIPTION:   Get the Firmware Version
 * ARG:           char * point ( its length should 16 bytes)
 * CONFIGURATION: None
 * DEPENDENCIES:  CONFIG_BOARDCTL_IOCTL
 *
 *
 * CMD:           BOARDIOC_G_MODEL
 * DESCRIPTION:   Get the device model
 * ARG:           char * point ( its length should 16 bytes)
 * CONFIGURATION: None
 * DEPENDENCIES:  CONFIG_BOARDCTL_IOCTL
 *
 *
 * CMD:           BOARDIOC_G_DEVNAME
 * DESCRIPTION:   Get the Device Name
 * ARG:           char * point ( its length should 16 bytes)
 * CONFIGURATION: None
 * DEPENDENCIES:  CONFIG_BOARDCTL_IOCTL
 *
 */
#define BOARDIOC_MAX_STRLEN      20
#define BOARDIOC_G_VERSION       ( BOARDIOC_USER + 1 )
#define BOARDIOC_G_MODEL         ( BOARDIOC_USER + 2 )
#define BOARDIOC_G_DEVNAME       ( BOARDIOC_USER + 3 )


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
#endif  /* __CONFIGS_ZGLUE_ZEUS2_CHICAGO_BOARD_H */