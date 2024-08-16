/************************************************************************************
 * arch/arm/src/nrf52/nrf52_uart.h
 *
 *   Copyright (C) 2017, 2018 Zglue Inc. All rights reserved.
 *   Author: Bill Rees    <bill@zglue.com>
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

#ifndef __ARCH_ARM_SRC_NRF52_UART_H
#define __ARCH_ARM_SRC_NRF52_UART_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Make sure that we have not enabled more U[S]ARTs than are supported by the
 * device.
 */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/


/************************************************************************************
 * Public Functions
 ************************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/* CONSOLE_UART is a count of UARTS not an index so start at 1. */
#if defined(CONFIG_SEGGER_RTT_CONSOLE)
#define CONSOLE_UART 0
#elif defined(CONFIG_UART0_SERIAL_CONSOLE)
#define CONSOLE_UART 1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#define CONSOLE_UART 2
#elif defined(CONFIG_ARCH_LOWPUTC)
#define CONSOLE_UART 0
#endif


/**@brief UART communication structure holding configuration settings for the peripheral.
 */
typedef struct
{
  uint32_t    uartbase;   /* Base address of USART registers */
  uint32_t    baud_rate;  /**< Baud rate configuration. */
  uint16_t    rxd_pin;    /**< RX pin number. */
  uint16_t    txd_pin;    /**< TX pin number. */
  uint16_t    rts_pin;    /**< RTS pin number, only used if flow control is enabled. */
  uint16_t    cts_pin;    /**< CTS pin number, only used if flow control is enabled. */
  uint8_t     parity;     /* 0=none, 1=odd, 2=even */
  uint8_t     bits;       /* Number of bits (7 or 8) */
  bool        stopbits2;  /* True: Configure with 2 stop bits instead of 1 */
  uint8_t     rx_len;     /* easydma RX buffer length */
  uint8_t     tx_len;     /* easydma TX buffer length */
  uint8_t     *rx_buf;    /* internal RX easydma buffer */
  uint8_t     *tx_buf;    /* internal TX easydma buffer */
  uint8_t     easydma;    /* whether to use easydma for transfer data */
} uart_config_t;

int nrf52_uart_apply_config(uart_config_t *config);

void nrf52_uart_legacy_send(uint32_t reg_base, uint8_t ch);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_NRF52_UART_H */

