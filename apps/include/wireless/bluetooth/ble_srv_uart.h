/****************************************************************************
 *   include/wireless/bluetooth/ble_srv_uart.h
 *
 *   Copyright (C) 2007-2017 Zglue. All rights reserved.
 *   Author: taohan <taohan@zglue.org>
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

#ifndef BLE_SRV_UART_H__
#define BLE_SRV_UART_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
 * Structs
 ****************************************************************************/

typedef enum
{
  BLE_UART_RX_DATA,
  BLE_UART_NTF_STARTED,
  BLE_UART_NTF_STOPPED,
  BLE_UART_CONNECTED,
  BLE_UART_DISCONNECTED,
  BLE_UART_TX_COMPLETE,
  BLE_UART_INVALID
} ble_uart_evt_type_t;

typedef struct
{
  ble_uart_evt_type_t type;
  uint8_t            *data; /*Received data from remote device.*/
  uint16_t             len; /*Length of the data.*/
} ble_uart_evt_t;

/*Handle uart input and output data.*/
/*This handler implementation is depends on the app itsself.*/
typedef void (*ble_uart_data_handler_t) (ble_uart_evt_t *p_evt);

/*Initialization stuct of the ble uart services.*/
typedef struct
{
  ble_uart_data_handler_t uart_data_handler;
} ble_uart_init_t;

/****************************************************************************
 * Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ble_srv_uart_init
 *
 * Description:
 *  BLE uart services initialization.
 *
 * Input Parameters:
 *  ble_uart_init  Uart parameter struct of Initialization.
 *
 * Returned Value:
 *   0: Successful
 *
 ****************************************************************************/
uint32_t ble_srv_uart_init(ble_uart_init_t ble_uart_init);

/****************************************************************************
 * Name: ble_uart_data_send
 *
 * Description:
 * BLE uart send data to remote devices.
 *
 * Input Parameters:
 *  data  Data content will be sent.
 *
 *  len  Data length will be sent.
 *
 * Returned Value:
 *   0: Successful
 *
 ****************************************************************************/
uint32_t ble_uart_data_send(uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* BLE_SRV_UART_H__*/

