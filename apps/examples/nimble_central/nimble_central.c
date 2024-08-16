/****************************************************************************
 * examples/nimble/nimble_central.c
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
#include "services/cus_inc/ble_svc_trans.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "services/ans/ble_svc_ans.h"
#include "wireless/bluetooth/ble_app_api.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define BLE_ADV_MIN_ITVL         (160)                       /**Uint of 0.625ms . */
#define BLE_ADV_MAX_ITVL         (320)                       /**Uint of 0.625ms . */
#define BLE_THROUGHPUT_ITVL_US   (0)
#define BLE_THROUGHPUT_ITVL_S    (1)

#define BLE_PHY_SETTING          (1)
#define BLE_DLE_SETTING          (0)
#define BLE_GPIO_DEBUG           (0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nimble_central
 ****************************************************************************/
int m_cnt = 0;

uint8_t setting_addr[BLE_DEV_ADDR_LEN] = {0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0x11};
static timer_t timerid = NULL;

void nimble_central_sync_cb(void);
extern int up_putc(int ch);

const ble_uuid128_t ble_trans_ntf_uuid =
  BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                   0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);

uint32_t througput_time = 0;
uint32_t througput_val = 0;
uint32_t curr = 0;
uint32_t prev = 0;
uint8_t  ble_att_svr_data[BLE_ATT_MTU_MAX];
uint16_t ble_att_svr_data_len;

static uint16_t central_conn_handle = 0xff;

extern uint16_t ble_att_preferred_mtu_val;

#if (BLETEST_THROUGHPUT_TEST == 1)
void bletest_completed_bytes(uint16_t handle, uint8_t len, bool retrans)
{
}
#endif

void throughput_timer_cb(union sigval value)
{
  uint32_t delta = 0;

  (void)delta;
  delta = througput_val - prev;
  APP_LOG_INFO("\n Single Data length = [%d] Currnet Throughput = [%d] Average = [%d].\n",
               ble_att_svr_data_len,
               delta,
               througput_time == 0 ? 0 : througput_val / througput_time);
  if (central_conn_handle != 0xff)
    {
      uint8_t tx_phy = 0xff, rx_phy = 0xff;
      ble_gap_set_prefered_le_phy(central_conn_handle, BLE_GAP_LE_PHY_2M_MASK,
                                  BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_CODED_ANY);
      ble_gap_read_le_phy(central_conn_handle, &tx_phy, &rx_phy);
      througput_time++;
      APP_LOG_INFO( "\n TX_PHY[%d], RX_PHY[%d]...\n", tx_phy, rx_phy);
    }
  prev = througput_val;
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

int central_ble_gatt_chr_fn(uint16_t conn_handle,
                            const struct ble_gatt_error *error,
                            const struct ble_gatt_chr *chr, void *arg)
{
  if (chr == 0)
    {
      APP_LOG_INFO("\n ERROR! NO CHAR...\n");
    }
  APP_LOG_INFO("\n char_conn_handle[%x]--char[%x]=[def_h=%x, val_h=%x, prop=%x]\n",
               conn_handle,
               chr,
               chr->def_handle,
               chr->val_handle,
               chr->properties);

#if 1
  if (chr->uuid.u.type == BLE_UUID_TYPE_16)
    {
      APP_LOG_INFO("\n char16=[%x]\n", chr->uuid.u16.value);
    }
  else if (chr->uuid.u.type == BLE_UUID_TYPE_32)
    {
      APP_LOG_INFO("\n char32=[%x]\n", chr->uuid.u32.value);
    }
  else if (chr->uuid.u.type == BLE_UUID_TYPE_128)
    {
      int i = 0;
      APP_LOG_INFO("\nCHAR 128=");
      for (i = 0; i < 16; i++)
        {
          APP_LOG_INFO("_%2x_", chr->uuid.u128.value[i]);
        }
      APP_LOG_INFO("\n");
    }
  else
    {
      APP_LOG_INFO("\n CHAR UUID not match...[%d]\n", chr->uuid.u.type);
    }
#endif

  if (chr->uuid.u.type == BLE_UUID_TYPE_128)
    {
      if (0 == ble_uuid_cmp((ble_uuid_t *)&chr->uuid, (ble_uuid_t *)&ble_trans_ntf_uuid))
        {
          int rc = 0;
          uint16_t a = 0x01;
          rc = ble_gattc_write_no_rsp_flat(conn_handle, chr->val_handle + 1, &a, 2);
          if (0 != rc)
            {
              APP_LOG_INFO("\nWrite ntf val4 = %d. \n", rc);
            }
        }
    }
  APP_LOG_INFO("\n");
  APP_LOG_INFO("\n");
  return 0;
}

int central_ble_gatt_disc_svc_fn(uint16_t conn_handle,
                                 const struct ble_gatt_error *error,
                                 const struct ble_gatt_svc *service,
                                 void *arg)
{
  if (service == NULL)
    {
      APP_LOG_INFO("\n ERROR! NO service...\n");

    }

  APP_LOG_INFO("\nSVC_CB: conn_handle=%x, error=%x, service[%x], svc_handle[s=%x, e=%x]\n",
               conn_handle,
               error,
               service,
               service->start_handle,
               service->end_handle);
  if (service->uuid.u.type == BLE_UUID_TYPE_16)
    {
      APP_LOG_INFO("\n SVC16=[%x]\n", service->uuid.u16.value);
    }
  else if (service->uuid.u.type == BLE_UUID_TYPE_32)
    {
      APP_LOG_INFO("\n SVC32=[%x]\n", service->uuid.u32.value);
    }
  else if (service->uuid.u.type == BLE_UUID_TYPE_128)
    {
      int i = 0;
      APP_LOG_INFO("\nSVC128=");
      for (i = 0; i < 16; i++)
        {
          APP_LOG_INFO("_%02x_", service->uuid.u128.value[i]);
        }
      APP_LOG_INFO("\n");
    }
  else
    {
      APP_LOG_INFO("\n SVC UUID not match...%d\n", service->uuid.u.type);
    }
  APP_LOG_INFO("\n");
  APP_LOG_INFO("\n");
  return 0;
}

void print_conn_info(uint16_t conn_handle)
{
  int rc;
  struct ble_gap_conn_desc out_desc;
  rc = ble_gap_conn_find(conn_handle, &out_desc);
  if (rc != 0)
    {
      APP_LOG_INFO( "\n Find conn failed... = %x\n", rc);
    }

  printf("\nConnection info: handle=%x,itvl=%x,latency=%x,mca=%x,role=%x,state=%x,timeout=%x\n",
         out_desc.conn_handle,
         out_desc.conn_itvl,
         out_desc.conn_latency,
         out_desc.master_clock_accuracy,
         out_desc.role,
         out_desc.sec_state,
         out_desc.supervision_timeout
        );

}

extern int ble_hs_hci_util_set_data_len(uint16_t conn_handle, uint16_t tx_octets, uint16_t tx_time);
static int
blecentral_gap_event(struct ble_gap_event *event, void *arg)
{
  int rc;

  switch (event->type)
    {
      case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        APP_LOG_INFO( "\nConnection EVENT...\n");
        APP_LOG_INFO( "\n handle %d; status=%x \n", event->connect.conn_handle, event->connect.status);

        if (event->connect.status == 0)
          {
            central_conn_handle = event->connect.conn_handle;
#if (BLE_PHY_SETTING)
            rc = ble_gap_set_prefered_le_phy(central_conn_handle, BLE_GAP_LE_PHY_2M_MASK,
                                             BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_CODED_ANY);
            printf( "\n LINE__[%d], rc[%x]...\n", __LINE__, rc);
#endif

#if (BLE_DLE_SETTING)
            rc = ble_hs_hci_util_set_data_len(event->connect.conn_handle, BLE_HCI_SET_DATALEN_TX_OCTETS_MAX, BLE_HCI_SET_DATALEN_TX_TIME_MAX);
            printf( "\n LINE__[%d], rc[%x]...\n", __LINE__, rc);
#endif
            rc = ble_gattc_exchange_mtu(event->connect.conn_handle, NULL, NULL);
            printf("\nrc = %x\n", rc);

            print_conn_info(event->connect.conn_handle);

            rc = ble_gattc_disc_all_svcs(event->connect.conn_handle, central_ble_gatt_disc_svc_fn, NULL);
            ASSERT(0 == rc);

            rc = ble_gattc_disc_all_chrs(event->connect.conn_handle, 2, 0xffff, central_ble_gatt_chr_fn, NULL);
            ASSERT(0 == rc);
          }


        return 0;

      case BLE_GAP_EVENT_DISCONNECT:
        APP_LOG_INFO( "disconnect; reason=%x \n", event->disconnect.reason);
        return 0;

      case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        APP_LOG_INFO( "connection updated; status=%x \n",
                      event->conn_update.status);
        print_conn_info(event->connect.conn_handle);
        return 0;

      case BLE_GAP_EVENT_NOTIFY_RX:
        {

          ble_att_svr_data_len = OS_MBUF_PKTLEN(event->notify_rx.om);
          rc = os_mbuf_copydata(event->notify_rx.om, 0, ble_att_svr_data_len, ble_att_svr_data);
          througput_val += ble_att_svr_data_len;
        }
        return 0;

      case BLE_GAP_EVENT_DISC:
        {
          ble_addr_t comp;
          uint8_t comp_addr[BLE_DEV_ADDR_LEN] = {0x12, 0x12, 0x12, 0x12, 0x12, 0x13};
          memcpy(&comp.val, comp_addr, 6);
          comp.type = 0;

          if (0 == ble_addr_cmp(&comp, &event->disc.addr))
            {
              APP_LOG_INFO( "\n Disc event; type = %d, rssi=%d data_len=%d addr_type=%d\n",
                            event->disc.event_type,
                            event->disc.rssi,
                            event->disc.length_data,
                            event->disc.addr.type);

              APP_LOG_INFO( "\n Disc event; Remote addr____[%x:%x:%x:%x:%x:%x]\n",
                            event->disc.addr.val[0],
                            event->disc.addr.val[1],
                            event->disc.addr.val[2],
                            event->disc.addr.val[3],
                            event->disc.addr.val[4],
                            event->disc.addr.val[5]);
              {
                struct ble_gap_conn_params conn_params;
                conn_params.scan_itvl = 0x0010;
                conn_params.scan_window = 0x0010;
                conn_params.itvl_min = BLE_GAP_INITIAL_CONN_ITVL_MIN / 3;
                conn_params.itvl_max = BLE_GAP_INITIAL_CONN_ITVL_MIN / 3; //BLE_GAP_INITIAL_CONN_ITVL_MAX;
                conn_params.latency = BLE_GAP_INITIAL_CONN_LATENCY;
                conn_params.supervision_timeout = BLE_GAP_INITIAL_SUPERVISION_TIMEOUT;
                conn_params.min_ce_len = BLE_GAP_INITIAL_CONN_MIN_CE_LEN;
                conn_params.max_ce_len = BLE_GAP_INITIAL_CONN_MAX_CE_LEN;
                ble_gap_disc_cancel();
                rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC,
                                     &event->disc.addr,
                                     BLE_HS_FOREVER,
                                     &conn_params,
                                     blecentral_gap_event, NULL);

                ASSERT(0 == rc);
              }
            }
        }
        return 0;

      case BLE_GAP_EVENT_DISC_COMPLETE:
        APP_LOG_INFO( "\nBLE_GAP_EVENT_DISC_COMPLETE_______________\n");
        return 0;

      case BLE_GAP_EVENT_REPEAT_PAIRING:
        APP_LOG_INFO( "\nBLE_GAP_EVENT_REPEAT_PAIRING_______________\n");
        return 0;
    }

  return 0;
}

void nimble_central_sync_cb(void)
{
  int rc;
  struct ble_gap_disc_params disc_params;

  memset(&disc_params, 0, sizeof(disc_params));

  disc_params.filter_duplicates = 0;
  disc_params.filter_policy = 0;
  disc_params.itvl = BLE_HCI_SCAN_ITVL_MAX;
  disc_params.limited = 0;
  disc_params.passive = 1;
  disc_params.window = 500;

  rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &disc_params, blecentral_gap_event, NULL);
  ASSERT(0 == rc);
  APP_LOG_INFO("\n BLE_DISCOVING....[%d]\n", rc);
}

#ifdef CONFIG_BUILD_KERNEL
int
main(int argc, FAR char *argv[])
#else
int central_main(int argc, char *argv[])
#endif
{
  int exitval = 0;
  int rc;
  (void)rc;

  nimble_sysinit_start();
  /* Call all C++ static constructors */
  nimble_hal_init();
  nimble_sysinit_end();

  memcpy(g_dev_addr, setting_addr, 6);
  ble_hs_cfg.reset_cb = NULL;
  ble_hs_cfg.sync_cb = &nimble_central_sync_cb;
  ble_hs_cfg.gatts_register_cb = NULL;
  ble_hs_cfg.store_status_cb = NULL;
  ble_att_set_preferred_mtu(BLE_ATT_MTU_MAX);

  easy_timer_create(&timerid, throughput_timer_cb);
  et_delay_t delay;
  delay.interval.nsec = BLE_THROUGHPUT_ITVL_US;
  delay.interval.sec = BLE_THROUGHPUT_ITVL_S;
  delay.start.nsec = 0;
  delay.start.sec = 1;
  easy_timer_start(timerid, delay);

  APP_LOG_INFO("\n\n\n\n\n\n\n\n%s__%d__CNT[%d]\n", __FUNCTION__, __LINE__, m_cnt);

#if (BLE_GPIO_DEBUG)
  DEBUG_GPIO_INIT(34, 42);

  DEBUG_GPIO_TOGGLE(34, 1);
  DEBUG_GPIO_TOGGLE(35, 2);
  DEBUG_GPIO_TOGGLE(36, 3);

  DEBUG_GPIO_TOGGLE(37, 4);
  DEBUG_GPIO_TOGGLE(38, 5);
  DEBUG_GPIO_TOGGLE(39, 6);

  DEBUG_GPIO_TOGGLE(40, 7);
  DEBUG_GPIO_TOGGLE(42, 8);
#endif

  while (1)
    {
      m_cnt++;
      os_eventq_run(os_eventq_dflt_get());
      //sleep(5);
    }

  return exitval;
}
