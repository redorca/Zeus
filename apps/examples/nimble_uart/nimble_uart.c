/****************************************************************************
 * examples/nimble/nimble_uart_main.c
 *
 *   Copyright (C) 2007-2013, 2017 Gregory Nutt. All rights reserved.
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

#include <sys/stat.h>
#include <stdint.h>
#include <stdio.h>
#include <sched.h>
#include <errno.h>
#include <utils/easy_timer.h>
#include <hal/hal_bsp.h>
#include <hal/hal_timer.h>
#include "host/ble_gap.h"
#include "host/ble_att.h"
#include "services/bleuart/bleuart.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "services/ans/ble_svc_ans.h"

#include "wireless/bluetooth/ble_app_api.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define BLE_ADV_MIN_ITVL         (160)                       /**Uint of 0.625ms . */
#define BLE_ADV_MAX_ITVL         (200)                       /**Uint of 0.625ms . */
#define BLE_INVALID_HANDLE       (0xff)

#define UART_FILE "/dev/console"
#define UART_MAX_RX_CHAR_LEN (5)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
//static timer_t timerid = NULL;
static void sync_cb(void);
static uint16_t uart_conn_handle = BLE_INVALID_HANDLE;
static FILE *m_uart_file = NULL;
struct os_task nimble_uart_task;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
#if (BLETEST_THROUGHPUT_TEST == 1)
void bletest_completed_bytes(uint16_t handle, uint8_t len, bool retrans)
{
  return;
}
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/
#if 0
static void easy_timer_cb(union sigval value)
{
  ble_svc_trans_ntf_val((uint8_t *)&ntf_val, ntf_bytes);
}
#endif

static void print_conn_info(uint16_t conn_handle)
{
  int rc;
  struct ble_gap_conn_desc out_desc;
  rc = ble_gap_conn_find(conn_handle, &out_desc);
  if (rc != 0)
    {
      APP_LOG_WARNING("Find conn failed... = %x\n", rc);
    }

  APP_LOG_WARNING("Connection info: handle=%x,itvl=%x,latency=%x,mca=%x,role=%x,state=%x,timeout=%x\n",
                  out_desc.conn_handle,
                  out_desc.conn_itvl,
                  out_desc.conn_latency,
                  out_desc.master_clock_accuracy,
                  out_desc.role,
                  out_desc.sec_state,
                  out_desc.supervision_timeout
                 );

}

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that forms.
 * bleprph uses the same callback for all connections.
 *
 * @param event                 The type of event being signalled.
 * @param ctxt                  Various information pertaining to the event.
 * @param arg                   Application-specified argument; unuesd by
 *                                  bleprph.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
static int
bleprph_gap_event(struct ble_gap_event *event, void *arg)
{
  int32_t rc = 0;
  (void)rc;

  switch (event->type)
    {
      case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        APP_LOG_INFO("connection %s; status=%x ",
                     event->connect.status == 0 ? "established" : "failed",
                     event->connect.status);
        if (event->connect.status == 0)
          {
            uart_conn_handle = event->connect.conn_handle;
            bleuart_set_conn_handle(uart_conn_handle);
            print_conn_info(event->connect.conn_handle);
          }
        else
          {
            /* Connection failed; resume advertising. */
            BLE_HS_LOG(ERROR, "\nconnection wrong status\n");
            sync_cb();
          }
        return 0;

      case BLE_GAP_EVENT_DISCONNECT:
        APP_LOG_WARNING("disconnect; reason=%x \n", event->disconnect.reason);
        uart_conn_handle = 0xff;
        /* Connection terminated; resume advertising. */
        sync_cb();
        return 0;

      case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        APP_LOG_WARNING("connection updated; status=%x \n",
                        event->conn_update.status);
        print_conn_info(event->connect.conn_handle);
        return 0;

      case BLE_GAP_EVENT_ADV_COMPLETE:
        APP_LOG_WARNING("advertise complete; reason=%x \n",
                        event->adv_complete.reason);
        sync_cb();
        return 0;

      case BLE_GAP_EVENT_ENC_CHANGE:
        /* Encryption has been enabled or disabled for this connection. */
        APP_LOG_WARNING("encryption change event; status=%d \n",
                        event->enc_change.status);
        return 0;

      case BLE_GAP_EVENT_NOTIFY_TX:
        return 0;

      case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.cur_notify == 1)
          {
            APP_LOG_WARNING("Enable notify...\n");
          }
        else
          {
            APP_LOG_WARNING("Disable notify...[%d]\n", event->subscribe.cur_notify);
          }
        return 0;

      case BLE_GAP_EVENT_MTU:
        APP_LOG_WARNING("mtu update event; conn_handle=%d cid=%d mtu=%d\n",
                        event->mtu.conn_handle,
                        event->mtu.channel_id,
                        event->mtu.value);
        return 0;
    }

  return 0;
}

static void sync_cb(void)
{
  int rc;
  struct ble_gap_adv_params adv_params;
  struct ble_hs_adv_fields fields;
  const char *name = "Z_nimble_uart";
  /*This could be random or public address.*/
  /*Random address must be set before use.*/
  ble_addr_t out_addr;

  (void)name;
  (void)rc;
  memset(&fields, 0, sizeof fields);

  /* Advertise two flags:
  *     o Discoverability in forthcoming advertisement (general)
  *     o BLE-only (BR/EDR unsupported).
  */
  fields.flags = BLE_HS_ADV_F_DISC_GEN |
                 BLE_HS_ADV_F_BREDR_UNSUP;

  /* Indicate that the TX power level field should be included; have the
  * stack fill this value automatically.  This is done by assiging the
  * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
  */
  fields.tx_pwr_lvl_is_present = 1;
  fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

#ifdef _BLE_DEV_NAME
  fields.name = (uint8_t *)_BLE_DEV_NAME;
  fields.name_len = sizeof(_BLE_DEV_NAME) - 1;
#else
  fields.name = (uint8_t *)name;
  fields.name_len = strlen(name);
#endif

  ble_svc_gap_device_name_set((const char *)fields.name);

  fields.name_is_complete = 1;

  fields.uuids16 = (ble_uuid16_t[])
  {
    BLE_UUID16_INIT(0x1811)
  };
  fields.num_uuids16 = 1;
  fields.uuids16_is_complete = 1;

  rc = ble_gap_adv_set_fields(&fields);
  if (rc != 0)
    {
      assert(rc == 0);
      return;
    }

  memset(&adv_params, 0, sizeof adv_params);
  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  adv_params.itvl_min = BLE_ADV_MIN_ITVL;
  adv_params.itvl_max = BLE_ADV_MAX_ITVL;


  ble_hs_id_gen_rnd(false, &out_addr);
  ble_hs_id_set_rnd(out_addr.val);

  rc = ble_gap_adv_start(BLE_OWN_ADDR_RANDOM, NULL, BLE_HS_FOREVER, &adv_params, bleprph_gap_event, NULL);

  BLE_HS_LOG(ERROR, "\n ADV_CB[%d]\n", rc);
  if (rc != 0)
    {
      assert(rc == 0);
    }
}

__attribute__((__noreturn__)) void
uart_read_task(void *arg)
{
  uint32_t len;
  uint8_t p_data[UART_MAX_RX_CHAR_LEN] = {0};

  m_uart_file = fopen(UART_FILE, "r+");
  if (NULL == m_uart_file)
    {
      APP_LOG_ERROR("UART file open [%s] failed...\n", UART_FILE);
    }
  else
    {
      APP_LOG_WARNING("UART file open [%s] successed...\n", UART_FILE);
    }

  while (1)
    {
      if (NULL == m_uart_file)
        {
          APP_LOG_ERROR("\nUART file closed abnormal...\n");
        }
      len = fread(p_data, 1, UART_MAX_RX_CHAR_LEN, m_uart_file);

      if (BLE_INVALID_HANDLE != uart_conn_handle)
        {
          bleuart_uart_read(len, p_data);
        }
    }
}

/****************************************************************************
 * Name: nimble_uart_main
 ****************************************************************************/

__attribute__((__noreturn__))
#ifdef CONFIG_BUILD_KERNEL
void main(int argc, FAR char *argv[])
#else
void nimble_uart_main(int argc, char *argv[])
#endif
{
  int rc = 0;
  (void)rc;

  APP_LOG_WARNING("NIMBLE_UART_APP_START...\n");

  /*Nimble stack init.*/
  ble_hs_cfg.reset_cb = NULL;
  ble_hs_cfg.sync_cb = &sync_cb;
  ble_hs_cfg.gatts_register_cb = NULL;
  ble_hs_cfg.store_status_cb = NULL;
  nimble_sysinit_start();
  nimble_hal_init();
  /*Default services of nimble*/
  ble_svc_gap_init();
  ble_svc_gatt_init();
  nimble_sysinit_end();

  /*Customer services of nimble*/
  bleuart_init();
  APP_LOG_WARNING("NIMBLE stack initialize done...\n");


  /* Initialize the host task */
  os_task_init(&nimble_uart_task, "nimble_uart", uart_read_task, NULL,
               100, OS_WAIT_FOREVER, NULL,
               128);

  APP_LOG_WARNING("Uart APP initialized done...\n");

  while (1)
    {
      os_eventq_run(os_eventq_dflt_get());
    }
}
