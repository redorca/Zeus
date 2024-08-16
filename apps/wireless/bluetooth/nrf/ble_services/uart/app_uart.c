/**
 * Copyright (c) 2015 - 2017, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "sdk_common.h"
#if NRF_MODULE_ENABLED(APP_UART)
#include <arch/board/board.h>
#include "app_uart.h"
#include "nrf_assert.h"

#if 0
static uint8_t tx_buffer[1];
static uint8_t rx_buffer[1];
static volatile bool rx_done;
static nrf_drv_uart_t app_uart_inst = NRF_DRV_UART_INSTANCE(APP_UART_DRIVER_INSTANCE);
#endif

extern uart_control_block_t *g_dev1;
extern uart_control_block_t *g_dev2;
extern uart_control_block_t *g_dev3;
extern void (*copy_app_handle)(nrf_drv_uart_event_t *p_event, void *p_context);

static app_uart_event_handler_t   m_event_handler;            /**< Event handler function. */
uart_control_block_t app_uart_cb;
static struct up_dev_s app_uart_inst =
{
  .dev =
  {
    .recv     =
    {
      .size   = CONFIG_UART0_RXBUFSIZE,
      .buffer = 0,
    },
    .xmit     =
    {
      .size   = CONFIG_UART0_TXBUFSIZE,
      .buffer = 0,
    },
    .priv     = &app_uart_inst,
  },

  .p_cb           = &app_uart_cb,
  .parity         = NRF_UART_PARITY_EXCLUDED,
  .baud           = UART_BAUDRATE_BAUDRATE_Baud115200,
  .uartbase       = NRF_UART0_BASE,
  .tx_gpio        = TX_PIN_NUMBER,
  .rx_gpio        = RX_PIN_NUMBER,
  .irq_prio       = _PRIO_APP_LOW,
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  .oflow          = false,
  .cts_gpio       = UART_PIN_DISCONNECTED,
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .iflow          = false,
  .rts_gpio       = UART_PIN_DISCONNECTED,
#endif
};

static void uart_event_handler(nrf_drv_uart_event_t *p_event, void *p_context)
{
  if (p_event->type == NRF_DRV_UART_EVT_RX_DONE)
    {
      app_uart_evt_t app_uart_event;
      app_uart_event.evt_type   = APP_UART_DATA;
      app_uart_event.data.value = p_event->data.rxtx.p_data[0];
      app_uart_inst.p_cb->p_rx_buffer = p_event->data.rxtx.p_data;
      app_uart_inst.p_cb->rx_buffer_length = p_event->data.rxtx.bytes;
      m_event_handler(&app_uart_event);
    }
  else if (p_event->type == NRF_DRV_UART_EVT_ERROR)
    {
      app_uart_evt_t app_uart_event;
      app_uart_event.evt_type                 = APP_UART_COMMUNICATION_ERROR;
      app_uart_event.data.error_communication = p_event->data.error.error_mask;
      m_event_handler(&app_uart_event);
    }
  else if (p_event->type == NRF_DRV_UART_EVT_TX_DONE)
    {
      // Last byte from FIFO transmitted, notify the application.
      // Notify that new data is available if this was first byte put in the buffer.
      app_uart_evt_t app_uart_event;
      app_uart_event.evt_type = APP_UART_TX_EMPTY;
      m_event_handler(&app_uart_event);
    }
}

uint32_t app_uart_init(app_uart_event_handler_t   app_event_handler)
{
  uint32_t err_code;
  err_code = nrf_drv_uart_init(&app_uart_inst, NULL, uart_event_handler);
  VERIFY_SUCCESS(err_code);
  m_event_handler = app_event_handler;
  return NRF_SUCCESS;
}


uint32_t app_uart_get(uint8_t *p_byte, uint16_t *p_len)
{
  ASSERT(p_byte);
  uint32_t err_code = NRF_SUCCESS;
  p_byte = app_uart_inst.p_cb->p_rx_buffer;
  *p_len = app_uart_inst.p_cb->rx_buffer_length;
  return err_code;
}

void app_uart_put(uint8_t byte)
{
  up_send_one(&app_uart_inst.dev, byte);
}

uint32_t app_uart_flush(void)
{
  return NRF_SUCCESS;
}

uint32_t app_uart_close(void)
{
  nrf_drv_uart_uninit(&app_uart_inst);
  return NRF_SUCCESS;
}
#endif //NRF_MODULE_ENABLED(APP_UART)
