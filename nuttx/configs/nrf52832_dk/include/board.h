/*****************************************************************************
 * configs/nrf52832_dk/include/board.h
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

#ifndef __CONFIGS_NRF52382_DK_INCLUDE_BOARD_H
#define __CONFIGS_NRF52382_DK_INCLUDE_BOARD_H

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

#define BSP_BOARD_LED_0 0
#define BSP_BOARD_LED_1 1
#define BSP_BOARD_LED_2 2
#define BSP_BOARD_LED_3 3

#ifdef BSP_LED_0
#define BSP_LED_0_MASK (1<<BSP_LED_0)
#else
#define BSP_LED_0_MASK 0
#endif
#ifdef BSP_LED_1
#define BSP_LED_1_MASK (1<<BSP_LED_1)
#else
#define BSP_LED_1_MASK 0
#endif
#ifdef BSP_LED_2
#define BSP_LED_2_MASK (1<<BSP_LED_2)
#else
#define BSP_LED_2_MASK 0
#endif
#ifdef BSP_LED_3
#define BSP_LED_3_MASK (1<<BSP_LED_3)
#else
#define BSP_LED_3_MASK 0
#endif

#define LED_STARTED                 0  /* OFF      */
#define LED_HEAPALLOCATE            0  /* OFF      */
#define LED_IRQSENABLED             0  /* OFF      */
#define LED_STACKCREATED            1  /* ON       */
#define LED_INIRQ                   2  /* NC       */
#define LED_SIGNAL                  2  /* NC       */
#define LED_ASSERTION               2  /* NC       */
#define LED_PANIC                   3  /* Flashing */

/* UART I2C SPI Pins *********************************************************/
/*
 * The following definitions must be provided so that the NRF52 serial
 * driver can set up the UART for the serial console properly.
 * You can modify the pin according to your board spec and usage.
 */

#define HWFC           true

#ifdef CONFIG_NRF52_UART0
/* UART serial pin config */
#define BOARD_UART0_RX_PIN    8//(GPIO_INPUT  | GPIO_PIN23)
#define BOARD_UART0_TX_PIN    6//(GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PIN24)
#define BOARD_UART0_CTS_PIN   7//0xFF
#define BOARD_UART0_RTS_PIN   5//0xFF
#endif

#ifdef CONFIG_NRF52_UART1
/*Now there is NO uart1 , only 52840 chip had two uart */
#define BOARD_UART1_RX_PIN    NRF52_INVALID_GPIO_PIN
#define BOARD_UART1_TX_PIN    NRF52_INVALID_GPIO_PIN
#define BOARD_UART1_CTS_PIN   NRF52_INVALID_GPIO_PIN
#define BOARD_UART1_RTS_PIN   NRF52_INVALID_GPIO_PIN
#endif

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
#define BOARD_I2C0_SDA_PIN    11 // CONFIG_NRF52_I2C0_SDA_PIN
#define BOARD_I2C0_SCL_PIN    12 // CONFIG_NRF52_I2C0_SCL_PIN

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
#define BOARD_I2C1_SDA_PIN    11 // CONFIG_NRF52_I2C1_SDA_PIN
#define BOARD_I2C1_SCL_PIN    12 // CONFIG_NRF52_I2C1_SCL_PIN
#endif

#ifdef CONFIG_NRF52_PDM
#define BOARD_PDM_CLK_PIN     30
#define BOARD_PDM_DIN_PIN     31
#endif

#ifdef CONFIG_NRF52_I2S
#define BOARD_I2S_MCK_PIN     0xFF
#define BOARD_I2S_SCK_PIN     26
#define BOARD_I2S_WS_PIN      27
#define BOARD_I2S_SDOUT_PIN   28
#define BOARD_I2S_SDIN_PIN    0xFF
#endif

#ifdef CONFIG_AUDIO_PCM510X
#define BOARD_PCM510X_XSMT_PIN 29
#endif
/* SPI pin config : CS pin is controlled by user*/
#define SPI0_FLASH_CS         24
#define SPI1_FLASH_CS         24
#define SPI2_FLASH_CS         24

#define SPI0_BMM150_CS        14
#define SPI1_BMM150_CS        14
#define SPI2_BMM150_CS        14

#define SPI0_BMI160_CS        28
#define SPI1_BMI160_CS        28
#define SPI2_BMI160_CS        28


#if defined(CONFIG_NRF52_SPI0)
#define BOARD_SPI0_MISO_PIN   29
#define BOARD_SPI0_MOSI_PIN   30
#define BOARD_SPI0_SCL_PIN    31
#endif

#if defined(CONFIG_NRF52_SPI1)
#define BOARD_SPI1_MISO_PIN   NRF52_INVALID_GPIO_PIN
#define BOARD_SPI1_MOSI_PIN   NRF52_INVALID_GPIO_PIN
#define BOARD_SPI1_SCL_PIN    NRF52_INVALID_GPIO_PIN
#endif

#if defined(CONFIG_NRF52_SPI2)
#define BOARD_SPI2_MISO_PIN   25 /* CONFIG_NRF52_SPI2_MISO_PIN */
#define BOARD_SPI2_MOSI_PIN   23 /* CONFIG_NRF52_SPI2_MOSI_PIN */
#define BOARD_SPI2_SCL_PIN    22 /* CONFIG_NRF52_SPI2_SCL_PIN */
#endif

#ifdef CONFIG_BMI160

/*bmi160 SPI or I2C port config*/
#ifdef CONFIG_BMI160_INTF_SPI
#define BMI160_SPI_PORTNO 0   /* On SPI1 */
#elif defined CONFIG_BMI160_INTF_I2C
#define BMI160_I2C_PORTNO 1   /* On I2C1 */
#endif

/*bmi160 interrupt pin config*/
#define BMI160_INT1_PIN       15
#define BMI160_INT2_PIN       16
#endif

#ifdef CONFIG_BMM150

/*bmm150 SPI or I2C port config*/
#ifdef CONFIG_BMM150_INTF_SPI
#define BMM150_SPI_PORTNO 0   /* On SPI1 */
#elif defined CONFIG_BMM150_INTF_I2C
#define BMM150_I2C_PORTNO 1   /* On I2C1 */
#endif

#define BMM150_INT_PIN       15
#define BMM150_DRDY_PIN       16

#endif

#ifdef CONFIG_LSM6DS3

/*LSM6DS3 SPI or I2C port config*/
#ifdef CONFIG_LSM6DS3_INTF_SPI
#define LSM6DS3_SPI_PORTNO 0   /* On SPI1 */
#elif defined CONFIG_LSM6DS3_INTF_I2C
#define LSM6DS3_I2C_PORTNO 1   /* On I2C1 */
#endif

#endif

#ifdef CONFIG_ICE40UP
#define ICE40UP_INT_PIN 15
#define ICE40UP_BUZZER_PIN 16
#define ICE40UP_ENABLE_PIN 18
#endif

#ifdef CONFIG_MC3672

/*mc3672 SPI or I2C port config*/
#ifdef CONFIG_MC3672_INTF_SPI
#define MC3672_SPI_PORTNO 0   /* On SPI1 */
#elif defined CONFIG_MC3672_INTF_I2C
#define MC3672_I2C_PORTNO 1   /* On I2C1 */
#endif

#define MC3672_INT_PIN       15

#endif

#ifdef CONFIG_BQ2512X

/*BQ2512X I2C port config*/
#define BQ2512X_I2C_PORTNO    1   /* On I2C1 */

#define BQ2512X_INT_PIN       15
#define BQ2512X_RESET_PIN     16
#define BQ2512X_LSCTRL_PIN    17
#define BQ2512X_NPG_PIN       19


#endif



#ifdef CONFIG_NRF52_QDECODER
#define NRF52_QDEC_PHASE_A    26
#define NRF52_QDEC_PHASE_B    27
/* if you use led feature , please define LED_OUTPUT  gpio for QDEC */
#define NRF52_QDEC_LED_OUTPUT 28
#endif

/* PWM module channel GPIO pinmux
 * NRF52_PWM_PIN_INVERTED indicated the pin output polarity
 */
#ifdef CONFIG_NRF52_PWM_M0
#define BOARD_PWM0_CHAN_1_PIN     (NRF52_PWM_PIN_INVERTED|17)
#define BOARD_PWM0_CHAN_2_PIN     18
#define BOARD_PWM0_CHAN_3_PIN     19
#define BOARD_PWM0_CHAN_4_PIN     20
#endif

#ifdef CONFIG_NRF52_PWM_M1
#define BOARD_PWM1_CHAN_1_PIN     NRF52_INVALID_GPIO_PIN
#define BOARD_PWM1_CHAN_2_PIN     NRF52_INVALID_GPIO_PIN
#define BOARD_PWM1_CHAN_3_PIN     NRF52_INVALID_GPIO_PIN
#define BOARD_PWM1_CHAN_4_PIN     NRF52_INVALID_GPIO_PIN
#endif

#ifdef CONFIG_NRF52_PWM_M2
#define BOARD_PWM2_CHAN_1_PIN     NRF52_INVALID_GPIO_PIN
#define BOARD_PWM2_CHAN_2_PIN     NRF52_INVALID_GPIO_PIN
#define BOARD_PWM2_CHAN_3_PIN     NRF52_INVALID_GPIO_PIN
#define BOARD_PWM2_CHAN_4_PIN     NRF52_INVALID_GPIO_PIN
#endif

/* Buttons *******************************************************************/
/* The board only has one button */

#define BSP_BOARD_BUTTON_0 0
#define BSP_BOARD_BUTTON_1 1
#define BSP_BOARD_BUTTON_2 2
#define BSP_BOARD_BUTTON_3 3

#ifdef BSP_BUTTON_0
#define BSP_BUTTON_0_MASK (1<<BSP_BUTTON_0)
#else
#define BSP_BUTTON_0_MASK 0
#endif
#ifdef BSP_BUTTON_1
#define BSP_BUTTON_1_MASK (1<<BSP_BUTTON_1)
#else
#define BSP_BUTTON_1_MASK 0
#endif
#ifdef BSP_BUTTON_2
#define BSP_BUTTON_2_MASK (1<<BSP_BUTTON_2)
#else
#define BSP_BUTTON_2_MASK 0
#endif
#ifdef BSP_BUTTON_3
#define BSP_BUTTON_3_MASK (1<<BSP_BUTTON_3)
#else
#define BSP_BUTTON_3_MASK 0
#endif

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
#endif  /* __CONFIGS_NRF52832_MK_INCLUDE_BOARD_H */
