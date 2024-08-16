/****************************************************************************
 *   examples/ble_app_uart/hello_main.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
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
#include <unistd.h>
#include <errno.h>
#include <debug.h>
#include <signal.h>
#include <wireless/bluetooth/bt_mq.h>
#include <wireless/bluetooth/ble_app_api.h>
#include <wireless/bluetooth/ble_srv_uart.h>
#include <wireless/bluetooth/ble_app_error.h>
#include <utils/easy_timer.h>
#include <utils/app_timer.h>
/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/
#define UART_NAME "/dev/console"
#define MAX_RX_CHAR_LEN (20)
#define MSEC_TO_NSEC    (1000*1000)
#define UART_NTF_START_TIME (10*MSEC_TO_NSEC)
#define UART_NTF_INTERVAL_TIMESTART_TIME (100*MSEC_TO_NSEC)

#define BLE_UART_TEST_NAME "ZG_BLE_APP"

#define BLE_CHIP_ID "/proc/devid"
#define BLE_CHIP_ID_LEN (8)

/****************************************************************************
 * Private Data
 ****************************************************************************/
static FILE *m_uart_file = NULL;
timer_t timerid = NULL;

/****************************************************************************
 * Private Function
 ****************************************************************************/

void ble_uart_msg_handler(uint8_t *data, uint16_t len)
{
  ble_uart_data_send(data, len);
}

void source_data_handler(uint8_t *data, uint16_t len)
{
  bt_mq_msg_t msg;
  memset(&msg, 0, sizeof(bt_mq_msg_t));
  msg.data = data;
  msg.len = len;
  msg.handler = ble_uart_msg_handler;
  bt_mq_send_message(&msg, 0);
}

void thread_timer_cb(union sigval value)
{
  if (NULL != m_uart_file)
    {
      /*Read file have issue with NON_BLOKC, use fake data now.*/
      //len = fread(p_data, 1, MAX_RX_CHAR_LEN, m_uart_file);
      uint8_t p_data[MAX_RX_CHAR_LEN] = {0};
      uint16_t len;
      static uint8_t val = 0;
      val++;
      len = MAX_RX_CHAR_LEN;
      p_data[MAX_RX_CHAR_LEN - 1] = val;
      p_data[0] = val + 1;
      source_data_handler(p_data, len);
    }
  else
    {
      APP_LOG_ERROR("\nUART file open [%s] failed [%d]...\n", UART_NAME, __LINE__);
    }
}

/*User app define the handler details, like data source, timer...*/
void uart_data_handler_demo (ble_uart_evt_t *p_evt)
{
  uint32_t ret;

  switch (p_evt->type)
    {
      case BLE_UART_RX_DATA:
        fwrite(p_evt->data, p_evt->len, 1, m_uart_file);
        fflush(m_uart_file);
        break;

      case BLE_UART_NTF_STARTED:
        APP_LOG_INFO("UART NTF start...\n", UART_NAME);
        et_delay_t delay = {0};
        delay.start.nsec = UART_NTF_START_TIME;
        delay.interval.nsec = UART_NTF_INTERVAL_TIMESTART_TIME;
        easy_timer_start(timerid, delay);
        break;

      case BLE_UART_NTF_STOPPED:
        APP_LOG_INFO("UART NTF stop...\n", UART_NAME);
        easy_timer_stop(timerid);
        break;

      case BLE_UART_CONNECTED:
        APP_LOG_INFO("Connected, open UART...\n", UART_NAME);
        m_uart_file = fopen(UART_NAME, "r+");
        if (NULL == m_uart_file)
          {
            APP_LOG_ERROR("\nUART file open [%s] failed...\n", UART_NAME);
          }
        else
          {
            easy_timer_create(&timerid, thread_timer_cb);
          }
        break;

      case BLE_UART_DISCONNECTED:
        APP_LOG_INFO("Disconnected, stop the NTF and close UART...\n", UART_NAME);
        if (NULL != timerid)
          {
            easy_timer_delete(timerid);
            timerid = NULL;
          }
        if (NULL != m_uart_file)
          {
            ret = fclose(m_uart_file);
            if (0 != ret)
              {
                APP_LOG_ERROR("\nUART file close failed[%x]...\n", errno);
              }
            else
              {
                m_uart_file = NULL;
              }
          }
        break;

      case BLE_UART_TX_COMPLETE:
        break;

      case BLE_UART_INVALID:
        break;

      default:
        break;
    }
}

#ifdef CONFIG_NRF52_PROCFS_DEVID
void set_chip_ID_to_BD_address(void)
{
  char dev_id[BLE_CHIP_ID_LEN] = {0};
  ble_app_addr_t addr;
  uint32_t err_code;

  /*1. Open device ID file and read id value.*/
  FILE *devid_file = fopen(BLE_CHIP_ID, "r");
  fread(dev_id, 1, BLE_CHIP_ID_LEN, devid_file);

  /*2. Set staic random address*/
  addr.addr_type = BLE_APP_ADDR_TYPE_RANDOM_STATIC;
  for (int i = 0; i < BLE_APP_ADDR_LEN; i++)
    {
      addr.addr[i] = dev_id[i];
    }
  addr.addr[BLE_APP_ADDR_LEN - 1] |= BLE_APP_ADDR_TYPE_RANDOM_STATIC_MSB_BIT;
  err_code = ble_app_set_address(&addr);
  BLE_APP_ERROR_CHECK_NO_RETURN(err_code);

  /*3. Close device id file. */
  fclose(devid_file);
}
#endif

int hello_main(int argc, char *argv[])
{
  uint32_t err_code;
  ble_adv_params_t advpara;
  ble_app_advdata_t advdata;
  ble_uart_init_t ble_uart_init;

  /*BLE core initialization.*********************************************/
  err_code = ble_init();
  BLE_APP_ERROR_CHECK(err_code);

  /*Set device name*/
#ifndef _BLE_DEV_NAME
  err_code = ble_app_set_device_name((const uint8_t *)BLE_UART_TEST_NAME, sizeof(BLE_UART_TEST_NAME) - 1);
  BLE_APP_ERROR_CHECK(err_code);
#endif

  /*Add chip ID as defaut staic random BD address, so every chip have unique BD address.*/
#ifdef CONFIG_NRF52_PROCFS_DEVID
  set_chip_ID_to_BD_address();
#endif

  /*BLE Uart service initialization.*************************************/
  ble_uart_init.uart_data_handler = uart_data_handler_demo;
  err_code = ble_srv_uart_init(ble_uart_init);
  BLE_APP_ERROR_CHECK(err_code);

  /*BLE ADV data initialization..*****************************************/
  memset(&advdata, 0, sizeof(ble_app_advdata_t));
  advdata.name_type = BLE_ADV_DATA_FULL_NAME;
  err_code = ble_adv_set_data(&advdata, NULL);
  BLE_APP_ERROR_CHECK(err_code);

  /*BLE ADV paramter initialization..*************************************/
  memset(&advpara, 0, sizeof(ble_adv_params_t));
  advpara.timeout = BLE_APP_ADV_MAX_TIMEOUT;
  err_code = ble_adv_start(&advpara);
  BLE_APP_ERROR_CHECK(err_code);

  /*BLE thread loop initialization.**************************************/
  while (1)
    {
      sleep(1);
    }

  return 0;
}
