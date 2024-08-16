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
#define BLE_ADV_MIN_ITVL         (100)                       /**Uint of 0.625ms . */
#define BLE_ADV_MAX_ITVL         (160)                       /**Uint of 0.625ms . */

#define BLE_ADV_KEEP_SEC         (10)

#define BLE_SLT_RSSI_THRESHOLD   (-50)
#define BLE_SLT_SCAN_TIMEOUT     (500)                       /*Uint of ms*/

#define BLE_SLT_RSSI_PASS_BIT    (0x01)
#define BLE_SLT_SCAN_DONE_BIT    (0x02)

#define BLE_SLT_TEST_LOOP_TIME   (10)                       /*Uint of second*/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
struct os_task g_ble_default_task;
uint32_t SLT_test_flag = 0;


/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
extern  int ble_hs_adv_find_field(uint8_t type,
                                  const uint8_t *data,
                                  uint8_t length,
                                  const struct ble_hs_adv_field **out);

extern void up_systemreset(void);
/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/
#if 0
extern void up_systemreset(void);
static void adv_start(void)
{
  int rc;
  struct ble_gap_adv_params adv_params;
  struct ble_hs_adv_fields fields;
  const char *name = "ble_nsh_test";
  /*This could be random or public address.*/
  /*Random address must be set before use.*/

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

  rc = ble_gap_adv_set_fields(&fields);
  if (rc != 0)
    {
      assert(rc == 0);
      return;
    }

  memset(&adv_params, 0, sizeof adv_params);
  adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  adv_params.itvl_min = BLE_ADV_MIN_ITVL;
  adv_params.itvl_max = BLE_ADV_MAX_ITVL;

  APP_LOG_WARNING("\n__%s__%d__advertising status[%d]...\n", __FUNCTION__, __LINE__, ble_gap_adv_active());
  rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, NULL, NULL);
  if (rc != 0)
    {
      assert(rc == 0);
    }
  APP_LOG_WARNING("\n__%s__%d__Nimble advertising start...\n", __FUNCTION__, __LINE__);
  APP_LOG_WARNING("\n__%s__%d__advertising status[%d]...\n", __FUNCTION__, __LINE__, ble_gap_adv_active());
  APP_LOG_WARNING("\nSleep some time for check...\n");

  sleep(BLE_ADV_KEEP_SEC);
  rc = ble_gap_adv_stop();
  if (rc != 0)
    {
      assert(rc == 0);
    }
  APP_LOG_WARNING("\n__%s__%d_advertising stop, status[%d]...\n", __FUNCTION__, __LINE__, ble_gap_adv_active());
  APP_LOG_WARNING( "\n System_reset...[%d]\n", rc);

  sleep(1);
  up_systemreset();

}

#endif
static int
nimble_slt_gap_event(struct ble_gap_event *event, void *arg)
{
  switch (event->type)
    {
      case BLE_GAP_EVENT_CONNECT:
      case BLE_GAP_EVENT_DISCONNECT:
      case BLE_GAP_EVENT_CONN_UPDATE:
      case BLE_GAP_EVENT_NOTIFY_RX:
        return 0;

      case BLE_GAP_EVENT_DISC:
        {

          int32_t ret;
          uint8_t incom_name[20] = {0};
          uint8_t com_name[32] = {0};
          const struct ble_hs_adv_field *field;

          ret = ble_hs_adv_find_field(BLE_HS_ADV_TYPE_INCOMP_NAME, event->disc.data,
                                      event->disc.length_data, &field);
          if (0 == ret)
            {
              memcpy(incom_name, field->value, field->length);
            }

          ret = ble_hs_adv_find_field(BLE_HS_ADV_TYPE_COMP_NAME, event->disc.data,
                                      event->disc.length_data, &field);
          if (0 == ret)
            {
              memcpy(com_name, field->value, field->length);
            }

          syslog(LOG_INFO, "Remote DEV name[%s] address[%02x:%02x:%02x:%02x:%02x:%02x] rssi= [%d]\n",
                 (com_name[0] == 0 ? incom_name : com_name),
                 event->disc.addr.val[0],
                 event->disc.addr.val[1],
                 event->disc.addr.val[2],
                 event->disc.addr.val[3],
                 event->disc.addr.val[4],
                 event->disc.addr.val[5],
                 event->disc.rssi);

          if (event->disc.rssi > BLE_SLT_RSSI_THRESHOLD)
            {
              SLT_test_flag |= BLE_SLT_RSSI_PASS_BIT;
            }
        }
        return 0;

      case BLE_GAP_EVENT_DISC_COMPLETE:
        SLT_test_flag |= BLE_SLT_SCAN_DONE_BIT;
        APP_LOG_WARNING( "\nBLE discovering complete...\n");
        return 0;

    }

  return 0;
}

static void nimble_SLT_test(void)
{
  int rc;
  struct ble_gap_disc_params disc_params;

  syslog(LOG_INFO, "Nimble SLT test start...\n");

  memset(&disc_params, 0, sizeof(disc_params));

  disc_params.filter_duplicates = 0;
  disc_params.filter_policy = 0;
  disc_params.itvl = BLE_HCI_SCAN_ITVL_MAX;
  disc_params.limited = 0;
  disc_params.passive = 1;
  disc_params.window = 500;

  SLT_test_flag = 0;

  rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, 500, &disc_params, nimble_slt_gap_event, NULL);
  ASSERT(0 == rc);
  syslog(LOG_INFO, "BLE start discovering....[%d]\n\n", rc);

}

__attribute__((__noreturn__)) void
ble_default_task(void *arg)
{
  struct ble_npl_event *ev;
  while (1)
    {
      ev = ble_npl_eventq_get((struct ble_npl_eventq *)os_eventq_dflt_get(), BLE_NPL_TIME_FOREVER);
      assert(ev);
      ble_npl_event_run(ev);
    }
  exit(0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nimble_main
 ****************************************************************************/

extern struct os_task g_ble_hs_task;
extern struct os_task g_ble_ll_task;
extern struct os_task os_callout_task_handle;

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int nimble_main(int argc, char *argv[])
#endif
{
  int32_t count = 0, rc = 0;

  (void)rc;
  nimble_sysinit_start();
  /* Call all C++ static constructors */
  ble_hs_cfg.reset_cb = NULL;
  //ble_hs_cfg.sync_cb = &adv_start;
  ble_hs_cfg.sync_cb = nimble_SLT_test;
  ble_hs_cfg.gatts_register_cb = NULL;
  ble_hs_cfg.store_status_cb = NULL;
  nimble_hal_init();
  nimble_sysinit_end();
  syslog(LOG_INFO, "Nimble initialize done...\n");

  os_task_init(&g_ble_default_task, "ble_default", ble_default_task, NULL, 3, OS_WAIT_FOREVER, NULL, 1024);

  count = 0;
  while (count < BLE_SLT_TEST_LOOP_TIME)
    {

      if (SLT_test_flag & BLE_SLT_SCAN_DONE_BIT)
        {
          break;
        }

      count++;
      sleep(1);

    }


  if (count < BLE_SLT_TEST_LOOP_TIME)
    {
      syslog(LOG_INFO, "\nTime %d seconds passd...\n", count);
    }
  else
    {
      syslog(LOG_INFO, "Nimble SLT test failed, SCAN can't be terminated normally, will reset the system...\n");
      sleep(1);
      up_systemreset();
    }


  if (SLT_test_flag & BLE_SLT_RSSI_PASS_BIT)
    {
      syslog(LOG_INFO, "Nimble SLT test passed...\n");
    }
  else
    {
      syslog(LOG_INFO, "Nimble SLT test failed, no rssi exceed [%d]\n", BLE_SLT_RSSI_THRESHOLD);
    }
  /*End the default task.*/
  nimble_hal_destory();
  pthread_cancel(g_ble_default_task.thread);
  APP_LOG_WARNING("\n__%s__%d__Nimble return ...\n", __FUNCTION__, __LINE__);

  return OK;
}
