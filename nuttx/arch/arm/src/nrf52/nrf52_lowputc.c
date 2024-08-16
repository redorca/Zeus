/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_lowputc.c
 *
 *   Copyright (C) 2009, 2012 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/board.h>


#include <nuttx/kmalloc.h>
#include <nuttx/serial/serial.h>
#include <stdint.h>

#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "string.h"
#include "nrf.h"
#include "nrf_uart.h"
#include "nrf_uarte.h"
#include "nrf52_gpio.h"
#include "nrf52_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

static const uart_config_t  g_lowput_config =
{
#if (CONSOLE_UART == 1)
  .uartbase       = NRF_UARTE0_BASE,
  .easydma     = false,
  .rxd_pin     = BOARD_UART0_RX_PIN,
  .txd_pin     = BOARD_UART0_TX_PIN,
  .cts_pin     = BOARD_UART0_CTS_PIN,
  .rts_pin     = BOARD_UART0_RTS_PIN,

  .parity      = CONFIG_UART0_PARITY,
  .bits        = CONFIG_UART0_BITS,
  .stopbits2   = CONFIG_UART0_2STOP,
  .baud_rate   = CONFIG_UART0_BAUD,
#elif (CONSOLE_UART == 2)
  .uartbase       = NRF_UARTE1_BASE,
  .easydma     = false,
  .rxd_pin     = BOARD_UART1_RX_PIN,
  .txd_pin     = BOARD_UART1_TX_PIN,
  .cts_pin     = BOARD_UART1_CTS_PIN,
  .rts_pin     = BOARD_UART1_RTS_PIN,

  .parity      = CONFIG_UART1_PARITY,
  .bits        = CONFIG_UART1_BITS,
  .stopbits2   = CONFIG_UART1_2STOP,
  .baud_rate   = CONFIG_UART1_BAUD,
#else
  .uartbase       = NRF_UARTE0_BASE,
  .easydma     = false,
  .rxd_pin     = BOARD_LOWPUTC_RX_PIN,
  .txd_pin     = BOARD_LOWPUTC_TX_PIN,
  .cts_pin     = BOARD_LOWPUTC_CTS_PIN,
  .rts_pin     = BOARD_LOWPUTC_RTS_PIN,

  .parity      = CONFIG_LOWPUTC_PARITY,
  .bits        = CONFIG_LOWPUTC_BITS,
  .stopbits2   = CONFIG_LOWPUTC_2STOP,
  .baud_rate   = CONFIG_LOWPUTC_BAUD,

#endif
};


/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/
#define LOWPUT_LEN 2
static char  lowput_tx[LOWPUT_LEN];
void up_lowputc(char ch)
{
  NRF_UARTE_Type *uart = (NRF_UARTE_Type *)g_lowput_config.uartbase;

  if (g_lowput_config.easydma)
    {
      static uint8_t idx;

      lowput_tx[(idx++) % LOWPUT_LEN] = ch;

      if (idx == LOWPUT_LEN)
        {
          idx = 0;
          nrf_uarte_tx_buffer_set(uart, (uint8_t *)lowput_tx, LOWPUT_LEN);

          nrf_uarte_event_clear((NRF_UARTE_Type *)uart, NRF_UARTE_EVENT_TXDRDY);
          nrf_uarte_event_clear((NRF_UARTE_Type *)uart, NRF_UARTE_EVENT_ENDTX);

          nrf_uarte_task_trigger(uart, NRF_UARTE_TASK_STARTTX);
          while (!nrf_uarte_event_check(uart, NRF_UARTE_EVENT_ENDTX));

          nrf_uarte_event_clear((NRF_UARTE_Type *)uart, NRF_UARTE_EVENT_TXDRDY);
          nrf_uarte_event_clear((NRF_UARTE_Type *)uart, NRF_UARTE_EVENT_ENDTX);
        }
    }
  else
    {
      nrf52_uart_legacy_send(g_lowput_config.uartbase, ch);
    }
  return;
}

/****************************************************************************
 * Name: nrf52_lowsetup
 *
 * Description:
 *   This performs basic initialization of the USART used for the serial
 *   console.  Its purpose is to get the console output availabe as soon
 *   as possible.
 *
 ****************************************************************************/

void nrf52_lowsetup(void)
{
  int err_code;
  err_code = nrf52_uart_apply_config((uart_config_t *)&g_lowput_config);

  if (err_code != 0)
    {
      __asm("bkpt");
    }

  if (g_lowput_config.easydma)
    {
      nrf_uarte_enable((NRF_UARTE_Type *)g_lowput_config.uartbase);
    }
  else
    {
      nrf_uart_enable((NRF_UART_Type *)g_lowput_config.uartbase);
    }
}

