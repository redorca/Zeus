/****************************************************************************
 * examples/nimble/nimble_dfu_main.c
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
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <crc16.h>
#include <utils/easy_timer.h>
#include <hal/hal_bsp.h>
#include <hal/hal_timer.h>
#include <sys/boardctl.h>
#include <nuttx/board.h>
#include <arch/board/board.h>
#include <fcntl.h>

#include "host/ble_gap.h"
#include "host/ble_att.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "services/cus_inc/ble_svc_dfu.h"

#include "wireless/bluetooth/ble_app_api.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define BLE_ADV_MIN_ITVL         (160)                       /**Uint of 0.625ms . */
#define BLE_ADV_MAX_ITVL         (200)                       /**Uint of 0.625ms . */
#define BLE_INVALID_HANDLE       (0xff)

#define BLE_DFU_CONN_ITVL_MIN    (8)                         /**Uint of 1.25ms . */
#define BLE_DFU_CONN_ITVL_MAX    (8)                         /**Uint of 1.25ms . */

#define BLE_DFU_TIMEOUT          (500)                       /**Uint of ms . */

#ifdef CONFIG_BOARDCTL_IOCTL
#define BOARDIOC_IMG_VERSION_LEN    (BOARDIOC_MAX_STRLEN)
#define BOARDIOC_DEV_MODEL_LEN      (BOARDIOC_MAX_STRLEN)
#define BOARDIOC_DEV_NAME_LEN       (BOARDIOC_MAX_STRLEN)

typedef struct img_info
{
  uint8_t img_ver[BOARDIOC_IMG_VERSION_LEN];
  uint8_t model[BOARDIOC_DEV_MODEL_LEN];
  uint8_t sn[BOARDIOC_DEV_NAME_LEN];
  uint32_t img_size;
  uint16_t crc;
} img_info;

img_info dfu_img_info = {0};

#endif

#define BEL_DFU_IMAGE_PATH       "/dev/mtdblock4"
static int32_t image_fd = 0;

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static void dfu_read_local_info(uint8_t *data, uint16_t *len);
static void dfu_image_write(uint8_t *data, uint16_t len);
static void ble_dfu_reset(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/
//static timer_t timerid = NULL;
static void sync_cb(void);
static uint16_t dfu_conn_handle = BLE_INVALID_HANDLE;

static uint8_t mfg_data[BLE_SVC_DFU_MFG_DATA_LEN] = {0};

ble_dfu_access_handler handler =
{
  .read = &dfu_read_local_info,
  .write = &dfu_image_write,
  .reset = &ble_dfu_reset,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
extern void up_systemreset(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static void dfu_image_write(uint8_t *data, uint16_t len)
{
  int bytes;
  assert(image_fd != 0);
  bytes = write(image_fd, data, len);
  if (bytes < 0)
    {
      BLE_HS_LOG(ERROR, "Write image file %d failed: %d\n", image_fd, errno);
    }
}

static void dfu_read_local_info(uint8_t *data, uint16_t *len)
{

  memset(&dfu_img_info, 0, sizeof(img_info));

#ifdef CONFIG_BOARDCTL_IOCTL
  boardctl(BOARDIOC_G_VERSION, (uintptr_t)dfu_img_info.img_ver);
  syslog(LOG_WARNING, "FW_VERSION: %s, len = %d\n", dfu_img_info.img_ver, strlen((const char *)dfu_img_info.img_ver));

  boardctl(BOARDIOC_G_MODEL, (uintptr_t)dfu_img_info.model);
  syslog(LOG_WARNING, "MODEL: %s, len = %d\n", dfu_img_info.model, strlen((const char *)dfu_img_info.model));

  boardctl(BOARDIOC_G_DEVNAME, (uintptr_t)dfu_img_info.sn);
  syslog(LOG_WARNING, "DEV: %s, len = %d\n", dfu_img_info.sn, strlen((const char *)dfu_img_info.sn));
#endif

  dfu_img_info.img_size = CONFIG_SRAM_LENGTH;
  dfu_img_info.crc = crc16((FAR const uint8_t *)&dfu_img_info, sizeof(img_info) - sizeof(uint16_t));

  APP_LOG_WARNING("Image information: img_size = %d Size = [%d], FW_VERSION=[%s], DEV_name = [%s], SN = [%s]\n",
                  CONFIG_SRAM_LENGTH,
                  sizeof(dfu_img_info),
                  dfu_img_info.img_ver,
                  dfu_img_info.model,
                  dfu_img_info.sn);
  memcpy(data, (uint8_t *)&dfu_img_info, sizeof(dfu_img_info));
  *len = sizeof(dfu_img_info);

}

static void ble_dfu_reset(void)
{
  /*1. Disconnect */
  if (BLE_INVALID_HANDLE != dfu_conn_handle)
    {
      APP_LOG_WARNING("Terminate the connection %d ... \n", dfu_conn_handle);
      ble_gap_terminate(dfu_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }

  if (image_fd != 0)
    {
      close(image_fd);
    }
  /*2. Reset device */
  APP_LOG_WARNING("Reset device to updating... \n");
  up_systemreset();
}

static void print_conn_info(uint16_t conn_handle)
{
  int rc;
  struct ble_gap_conn_desc out_desc;
  rc = ble_gap_conn_find(conn_handle, &out_desc);
  if (rc != 0)
    {
      APP_LOG_WARNING("Find conn failed... = %x\n", rc);
    }

  APP_LOG_WARNING("Connection info: handle=%x,itvl=%x(1.25ms),latency=%x(1.25ms),mca=%x,role=%x,sec_state=%x,timeout=%x\n",
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
            struct ble_gap_upd_params params;
            dfu_conn_handle = event->connect.conn_handle;
            ble_svc_dfu_set_conn_handle(dfu_conn_handle);
            print_conn_info(event->connect.conn_handle);

            params.itvl_max = BLE_DFU_CONN_ITVL_MIN;
            params.itvl_min = BLE_DFU_CONN_ITVL_MAX;
            params.latency = 0;
            params.supervision_timeout = BLE_DFU_TIMEOUT;
            params.max_ce_len = BLE_GAP_INITIAL_CONN_MAX_CE_LEN;
            params.max_ce_len = BLE_GAP_INITIAL_CONN_MIN_CE_LEN;
            rc = ble_gap_update_params(dfu_conn_handle, &params);
            if (rc != 0)
              {
                APP_LOG_ERROR("Update parameter failed: %d\n", rc);
              }

            rc = ble_gap_set_prefered_le_phy(dfu_conn_handle, BLE_GAP_LE_PHY_2M_MASK,
                                             BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_CODED_ANY);
            if (0 != rc)
              {
                APP_LOG_ERROR( "\n BLE set phy rate failed[%x]...\n", rc);
              }
            rc = ble_hs_hci_util_set_data_len(event->connect.conn_handle, BLE_HCI_SET_DATALEN_TX_OCTETS_MAX, BLE_HCI_SET_DATALEN_TX_TIME_MAX);
            if (0 != rc)
              {
                APP_LOG_ERROR( "\n BLE set data len failed[%x]...\n", rc);
              }

            image_fd = open(BEL_DFU_IMAGE_PATH, O_RDWR | O_CREAT);
            if (image_fd < 0)
              {
                APP_LOG_ERROR("Open image file %d failed: %d\n", image_fd, errno);
              }
            else
              {
                APP_LOG_INFO("Open image file %d successed!: %d\n", image_fd, errno);
              }
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
        dfu_conn_handle = 0xff;
        if (image_fd != 0)
          {
            rc = close(image_fd);
            if (rc < 0)
              {
                APP_LOG_ERROR("Close %s file failed: %d\n", image_fd, errno);
              }
            image_fd = 0;
          }
        /* Connection terminated; resume advertising. */
        ble_svc_dfu_reset();
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
  const char *name = "Z_dfu_demo";
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

  *((uint16_t *)&mfg_data[0]) = BLE_SVC_DFU_MFG_COMPANY_ID;
  *((uint16_t *)&mfg_data[2]) = BLE_SVC_DFU_MFG_OTA_ID;

  fields.mfg_data = mfg_data;

  fields.mfg_data_len = sizeof(mfg_data);

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
 * Name: nimble_dfu_main
 ****************************************************************************/
__attribute__((__noreturn__))
#ifdef CONFIG_BUILD_KERNEL
void main(int argc, FAR char *argv[])
#else
void nimble_dfu_main(int argc, char *argv[])
#endif
{
  int rc = 0;
  (void)rc;
  APP_LOG_WARNING("NIMBLE_dfu_APP_START...\n");

  /*Nimble stack init.*/
  ble_hs_cfg.reset_cb = NULL;
  ble_hs_cfg.sync_cb = &sync_cb;
  ble_hs_cfg.gatts_register_cb = NULL;
  ble_hs_cfg.store_status_cb = NULL;
  nimble_sysinit_start();
  nimble_hal_init();

  nimble_sysinit_end();
  ble_att_set_preferred_mtu(BLE_SVC_DFU_MAX_DATA_LEN);

  /*Customer services of nimble*/
  ble_svc_dfu_init(&handler);
  APP_LOG_WARNING("IMAGE1 ready for test...%d\n", rc);

  while (1)
    {
      os_eventq_run(os_eventq_dflt_get());
    }
}
