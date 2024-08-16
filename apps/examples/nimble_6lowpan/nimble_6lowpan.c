/****************************************************************************
 * examples/nimble/nimble_main.c
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
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdio.h>
#include <fixedmath.h>
#include <sched.h>
#include <errno.h>
#include <utils/easy_timer.h>
#include <sys/boardctl.h>
#include <nuttx/board.h>
#include <arch/board/board.h>
#include <fcntl.h>
#include <nuttx/sensors/tmp108.h>

#ifdef CONFIG_ARCH_HAVE_FAST
#include <nuttx/drivers/zglue_fast.h>
#include <nuttx/zglue_fast/fast_api.h>
#endif
#include <hal/hal_bsp.h>
#include <hal/hal_timer.h>
#include "host/ble_gap.h"
#include "host/ble_att.h"
#include "services/cus_inc/ble_svc_trans.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "services/ans/ble_svc_ans.h"

#include "wireless/bluetooth/ble_app_api.h"

#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/dns.h"
//#include "netif/etharp.h"
#if LWIP_IPV6
#include "lwip/ethip6.h"
#include "lwip/nd6.h"
#endif

#include "lwip/apps/httpd.h"
#include "lwip/apps/snmp.h"
#include "lwip/apps/lwiperf.h"
#include "lwip/apps/mdns.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define BLE_ADV_MIN_ITVL         (160)                       /**Uint of 0.625ms . */
#define BLE_ADV_MAX_ITVL         (200)                       /**Uint of 0.625ms . */

#define BLE_PHY_SECURITY         (0)
#define BLE_PHY_SETTING          (1)
#define BLE_DLE_SETTING          (1)
#define BLE_GPIO_DEBUG           (1)

#define BLE_TPTEST_1M_NO_DLE     (1)
#define BLE_TPTEST_2M_NO_DLE     (2)
#define BLE_TPTEST_2M_DLE        (3)

/*L2CAP MAX 251, L2CAP Header 4 bytes, ATT packet MAX 247, raw payload 244*/
#define BLE_NTF_MTU       (128)
#define BLE_NTF_MTU_CNT   (3)
#define BLE_NTF_ITVL_US   (100000000)
#define BLE_NTF_ITVL_S    (0)

#define BLE_THROUGHPUT_ITVL_US   (0)
#define BLE_THROUGHPUT_ITVL_S    (1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
static int m_cnt = 0;
static timer_t timerid = NULL;

static timer_t tp_timerid = NULL;

static uint16_t peripheral_conn_handle = 0xff;
static void sync_cb(void);

#ifdef CONFIG_BOARDCTL_IOCTL
#define BOARDIOC_IMG_VERSION_LEN    (32)
#define BOARDIOC_DEV_MODEL_LEN      (32)
#define BOARDIOC_DEV_NAME_LEN       (32)

uint8_t dev_imgver[BOARDIOC_IMG_VERSION_LEN];
uint8_t dev_model[BOARDIOC_DEV_MODEL_LEN];
uint8_t dev_name[BOARDIOC_DEV_NAME_LEN];
#endif

static double sensor_tmp_read(void);
/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
#if (BLETEST_THROUGHPUT_TEST == 1)
void bletest_completed_bytes(uint16_t handle, uint8_t len, bool retrans)
{
  if (handle == peripheral_conn_handle)
    {
      tp_total_bytes += len;
    }
}
#endif

extern int up_putc(int ch);

extern int ble_hs_hci_util_set_data_len(uint16_t conn_handle, uint16_t tx_octets, uint16_t tx_time);
/****************************************************************************
 * Private Functions
 ****************************************************************************/
static void tp_timer_cb(union sigval value)
{
  if (peripheral_conn_handle != 0xff)
    {
      char send[20];
      sprintf(send, "%4.2f", sensor_tmp_read());
      //printf("__%s__%d\n",send,strlen(send));
      ble_svc_trans_ntf_throughput((uint8_t *)send, strlen(send));
    }
}

static void easy_timer_cb(union sigval value)
{

#ifdef CONFIG_BOARDCTL_IOCTL
  if (peripheral_conn_handle != 0xff)
    {
      uint8_t send_val[BLE_NTF_MTU];
      memset(&send_val, 0, BLE_NTF_MTU);
      memcpy(&send_val, &dev_imgver, BOARDIOC_IMG_VERSION_LEN);
      memcpy(&send_val[BOARDIOC_IMG_VERSION_LEN], &dev_model, BOARDIOC_DEV_MODEL_LEN);
      memcpy(&send_val[BOARDIOC_IMG_VERSION_LEN + BOARDIOC_DEV_MODEL_LEN], &dev_name, BOARDIOC_DEV_NAME_LEN);

      ble_svc_trans_ntf_val((uint8_t *)&send_val, BOARDIOC_DEV_NAME_LEN * 3);
    }
#endif
}

static void print_conn_info(uint16_t conn_handle)
{
  int rc;
  struct ble_gap_conn_desc out_desc;
  rc = ble_gap_conn_find(conn_handle, &out_desc);
  if (rc != 0)
    {
      BLE_HS_LOG(INFO, "\n Find conn failed... = %x\n", rc);
    }

  BLE_HS_LOG(INFO, "\nConnection info: handle=%x,itvl=%x,latency=%x,mca=%x,role=%x,state=%x,timeout=%x\n",
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
        ble_svc_trans_set_conn_handle(event->connect.conn_handle);
        if (event->connect.status == 0)
          {
            peripheral_conn_handle = event->connect.conn_handle;
            print_conn_info(event->connect.conn_handle);

            if (0 == ble_store_util_bonded_check(event->connect.conn_handle))
              {
#if (BLE_PHY_SECURITY)
                /* Default we not initialize the security process,
                 * but the feature is enabled,
                 * custom can enable it by themself.
                 */
                ble_gap_security_initiate(event->connect.conn_handle);
#endif
                BLE_HS_LOG(INFO, "\nNot bonding, connection for first time...\n");
              }
            else
              {
                BLE_HS_LOG(INFO, "\nAlready bonding...\n");
              }
          }

        if (event->connect.status != 0)
          {
            /* Connection failed; resume advertising. */
            BLE_HS_LOG(ERROR, "\nconnection wrong status\n");
            sync_cb();
          }
        return 0;

      case BLE_GAP_EVENT_DISCONNECT:
        BLE_HS_LOG(INFO, "disconnect; reason=%x \n", event->disconnect.reason);
        peripheral_conn_handle = 0xff;
        //easy_timer_stop(timerid);
        /* Connection terminated; resume advertising. */
        sync_cb();
        return 0;

      case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        BLE_HS_LOG(INFO, "connection updated; status=%x \n",
                   event->conn_update.status);
        print_conn_info(event->connect.conn_handle);
        return 0;

      case BLE_GAP_EVENT_ADV_COMPLETE:
        BLE_HS_LOG(INFO, "advertise complete; reason=%x \n",
                   event->adv_complete.reason);
        sync_cb();
        return 0;

      case BLE_GAP_EVENT_ENC_CHANGE:
        /* Encryption has been enabled or disabled for this connection. */
        BLE_HS_LOG(INFO, "encryption change event; status=%d \n",
                   event->enc_change.status);
        return 0;

      case BLE_GAP_EVENT_NOTIFY_TX:

#if (BLE_GPIO_DEBUG)
        DEBUG_GPIO_TOGGLE(37, 2);
#endif
        return 0;

      case BLE_GAP_EVENT_SUBSCRIBE:
        BLE_HS_LOG(INFO, "sub event; conn_handle=%d attr_handle=%d "
                   "reason=%d prevn=%d curn=%d previ=%d curi=%d\n",
                   event->subscribe.conn_handle,
                   event->subscribe.attr_handle,
                   event->subscribe.reason,
                   event->subscribe.prev_notify,
                   event->subscribe.cur_notify,
                   event->subscribe.prev_indicate,
                   event->subscribe.cur_indicate);
        return 0;

      case BLE_GAP_EVENT_MTU:
        BLE_HS_LOG(INFO, "mtu update event; conn_handle=%d cid=%d mtu=%d\n",
                   event->mtu.conn_handle,
                   event->mtu.channel_id,
                   event->mtu.value);
        return 0;

#if (BLE_PHY_SECURITY)
      case BLE_GAP_EVENT_REPEAT_PAIRING:
        BLE_HS_LOG(INFO, "\nBLE_GAP_EVENT_REPEAT_PAIRING\n");
        struct ble_gap_conn_desc desc;
        /* We already have a bond with the peer, but it is attempting to
         * establish a new secure link.  This app sacrifices security for
         * convenience: just throw away the old bond and accept the new link.
         */

        /* Delete the old bond. */
        rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        assert(rc == 0);
        ble_store_util_delete_peer(&desc.peer_id_addr);
        /* Return BLE_GAP_REPEAT_PAIRING_RETRY to indicate that the host should
         * continue with the pairing operation.
         */
        ble_gap_security_initiate(event->connect.conn_handle);
        BLE_HS_LOG(INFO, "\nShould not happened..., re-iintialize pairing here...\n");
        return BLE_GAP_REPEAT_PAIRING_RETRY;
#endif
    }

  return 0;
}

static void sync_cb(void)
{
  int rc;
  struct ble_gap_adv_params adv_params;
  struct ble_hs_adv_fields fields;
  const char *name = "Z_6lowpan_demo";
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

  fields.appearance = BLE_SVC_GAP_APPEARANCE_GEN_COMPUTER;
  fields.appearance_is_present = 1;

#ifdef _BLE_DEV_NAME
  fields.name = (uint8_t *)_BLE_DEV_NAME;
  fields.name_len = sizeof(_BLE_DEV_NAME) - 1;
#else
  fields.name = (uint8_t *)name;
  fields.name_len = strlen(name);
#endif
  fields.name_is_complete = 1;

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

/****************************************************************************
 * Name: nimble_main
 ****************************************************************************/
int sensor_fast_led_blinky(bool blinky_flag)
{
#ifdef CONFIG_ARCH_HAVE_FAST
#define FAST_DEV  "/dev/fast"
  static bool first_flag = true;
  static int led_fd;
  uint32_t param[10];

  if (first_flag)
    {
      first_flag = false;
      led_fd = open(FAST_DEV, O_RDONLY);
      if (led_fd < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to open %s: %d\n",
                 FAST_DEV, errno);
          return -1;
        }
    }

  if (led_fd)
    {
      /* set the debug level */
      param[0] = FAST_DEBUG_LEVEL_1;
      ioctl(led_fd, FAST_IOCTL_SET_DEBUG_LEVEL, (unsigned long)param);
      /* Fast api config  LED2  */
      param[0] = (uint32_t) FAST_LED2;
      param[1] = (uint32_t) FAST_LED_DUTY_CYCLE_50_0;
      param[2] = (uint32_t) FAST_LED_PERIOD_0_5_S;
      param[3] = (uint32_t) FAST_LED_SCALE_12_8_mA;
      param[4] = (uint32_t) FAST_LED_BRIGHTNESS_25;
      param[5] = (uint32_t) false;
      if (blinky_flag)
        {
          ioctl(led_fd, FAST_IOCTL_CONFIGURE_LED, (unsigned long)param);
          ioctl(led_fd, FAST_IOCTL_ENABLE_LED, (unsigned long)param);
          syslog(LOG_ERR, "Turn ON LED\n");
        }
      else
        {
          ioctl(led_fd, FAST_IOCTL_DISABLE_LED, (unsigned long)param);
          syslog(LOG_ERR, "Turn Off LED\n");
        }
    }

#endif
  return OK;
}

static double sensor_tmp_read(void)
{
  double out_tmp = 0;

#ifdef CONFIG_SENSORS_TMP108
  static int g_tmp;
//  static int flags_first = true;
  b16_t temp16;

  g_tmp  = open("/dev/temp0", O_RDONLY);

  if (g_tmp < 0)
    {
      syslog(LOG_ERR, "Can't Open temp Sensor.\n");
    }


  if (g_tmp)
    {
      ioctl(g_tmp, SNIOC_CENTIGRADE, 0);

      read(g_tmp, &temp16, sizeof(b16_t));

      out_tmp = (double)temp16 / 16.0;
      syslog(LOG_INFO, "temperature is %.02f\n", out_tmp);
    }

  close(g_tmp);
#endif
  return out_tmp;
}

__attribute__((__noreturn__))
#ifdef CONFIG_BUILD_KERNEL
void main(int argc, FAR char *argv[])
#else
void nimble_6lowpan_main(int argc, char *argv[])
#endif
{
  int rc = 0;
  et_delay_t tp_delay;

  (void)rc;
  nimble_sysinit_start();
  /* Call all C++ static constructors */
  nimble_hal_init();
  /*Default services of nimble*/
  //ble_svc_gap_init();
  //ble_svc_gatt_init();
  //ble_svc_ans_init();

#ifdef CONFIG_LWIP
  lwip_init();
#endif

#ifdef CONFIG_BOARDCTL_IOCTL
  boardctl(BOARDIOC_G_VERSION, (uintptr_t)dev_imgver);
  syslog(LOG_WARNING, "FW_VERSION: %s, len = %d\n", dev_imgver, strlen((const char *)dev_imgver));

  boardctl(BOARDIOC_G_MODEL, (uintptr_t)dev_model);
  syslog(LOG_WARNING, "MODEL: %s, len = %d\n", dev_model, strlen((const char *)dev_model));

  boardctl(BOARDIOC_G_DEVNAME, (uintptr_t)dev_name);
  syslog(LOG_WARNING, "DEV: %s, len = %d\n", dev_name, strlen((const char *)dev_name));
#endif

  sensor_fast_led_blinky(true);
  sleep(1);

  sensor_tmp_read();

  sensor_fast_led_blinky(false);

  easy_timer_create(&timerid, &easy_timer_cb);
  tp_delay.interval.nsec = 0;
  tp_delay.interval.sec = 1;
  tp_delay.start.nsec = 0;
  tp_delay.start.sec = 1;
  easy_timer_start(timerid, tp_delay);

  easy_timer_create(&tp_timerid, &tp_timer_cb);
  tp_delay.interval.nsec = 0;
  tp_delay.interval.sec = 1;
  tp_delay.start.nsec = 0;
  tp_delay.start.sec = 1;
  easy_timer_start(tp_timerid, tp_delay);

  /*Customer services of nimble*/
  ble_svc_trans_init();
  nimble_sysinit_end();

  ble_hs_cfg.reset_cb = NULL;
  ble_hs_cfg.sync_cb = &sync_cb;
  ble_hs_cfg.gatts_register_cb = NULL;
  ble_hs_cfg.store_status_cb = NULL;
  ble_att_set_preferred_mtu(BLE_NTF_MTU);

  APP_LOG_INFO("\n\n\n\n\n\n111__%s__%d__CNT[%d]\n", __FUNCTION__, __LINE__, m_cnt);

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
}

