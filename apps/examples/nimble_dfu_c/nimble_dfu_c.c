/****************************************************************************
 * examples/nimble/nimble_dfu_c.c
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
#include <stdlib.h>
#include <sched.h>
#include <fcntl.h>
#include <errno.h>
#include <crc16.h>
#include <utils/easy_timer.h>
#include <os/os.h>
#include <hal/hal_bsp.h>
#include <hal/hal_timer.h>
#include "host/ble_gap.h"
#include "host/ble_att.h"
#include "services/cus_inc/ble_svc_dfu.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "wireless/bluetooth/ble_app_api.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define BLE_DFU_SCAN_ITVL               (1600)            /* (4~16384)(2.5ms~10240ms) units of 625us*/
#define BLE_DFU_SCAN_WINDOW             (160)             /* (4~16384)(2.5ms~10240ms) units of 625us*/

#define BLE_DFU_ITVL_US   (0)
#define BLE_DFU_ITVL_S    (1)

#define BLE_PHY_SETTING          (0)
#define BLE_DLE_SETTING          (0)

#define BEL_DFU_IMAGE_PATH       "/dev/mtdblock4"
/****************************************************************************
 * Private Types
 ****************************************************************************/
typedef enum ble_dfu_c_status_type
{
  BLE_DFU_C_STATUS_INIT = 0,
  BLE_DFU_C_STATUS_ADV_REPORTED,
  BLE_DFU_C_STATUS_CONNECTED,
  BLE_DFU_C_STATUS_STORE_CHAR_HANDLE,
  BLE_DFU_C_STATUS_READ_PERIPH_INFO,
  BLE_DFU_C_STATUS_WRITE_START_PKG_ACK,
  BLE_DFU_C_STATUS_WRITE_UPDATING_PKG_ACK,
  BLE_DFU_C_STATUS_WRITE_END_PKG_ACK,
  BLE_DFU_C_STATUS_ERROR,
  BLE_DFU_C_STATUS_MAX
} ble_dfu_c_status_type;

typedef enum ble_dfu_c_err_type
{
  BLE_DFU_C_ERR_NONE = 0,
  BLE_DFU_C_ERR_DISC_CHAR_ERR,
  BLE_DFU_C_ERR_CHAR_ATTR_MISS_ERR,
  BLE_DFU_C_ERR_READ_PEIRPH_INFO_ERR,
  BLE_DFU_C_ERR_WRITE_START_PKG_ERR,
  BLE_DFU_C_ERR_WRITE_UPDATE_PKG_ERR,
  BLE_DFU_C_ERR_WRITE_END_PKG_ERR,
  BLE_DFU_C_ERR_REMOTE_ERR,
  BLE_DFU_C_ERR_MAX
} ble_dfu_c_err_type;

typedef struct ble_dfu_c_process_ctrl_type
{
  uint8_t pre_status;
  uint8_t cur_status;
  ble_addr_t periph_addr;
  uint16_t periph_flash_size;
  uint16_t periph_img_ver;
  uint16_t block_cnt;
  uint16_t block_num;
  uint16_t block_size;
  uint16_t conn_handle;
  uint16_t read_handle;
  uint16_t write_handle;
  uint16_t ntf_handle;
  uint16_t ack_info;
  uint16_t err_code;
} ble_dfu_c_process_ctrl_type;

typedef void (* dfu_ev_handler)(void);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int ble_dfu_c_gap_event(struct ble_gap_event *event, void *arg);
static void ble_dfu_c_write_start_pkg(void);
static void ble_dfu_c_write_update_pkg(void);
static void ble_dfu_c_write_end_pkg(void);
static int ble_dfu_c_crc16_check(uint16_t len, uint8_t *data);
static void ble_dfu_c_recv_NTF_pkg(uint16_t len, uint8_t *data);

static void ble_dfu_c_recv_adv_report_handler(void);
static void ble_dfu_c_recv_connected_handler(void);
static void ble_dfu_c_recv_store_char_handle_handler(void);
static void ble_dfu_c_recv_read_periph_info_handler(void);
static void ble_dfu_c_recv_write_start_pkg_ack_handler(void);
static void ble_dfu_c_recv_write_updating_pkg_ack_handler(void);
static void ble_dfu_c_recv_write_end_pkg_ack_handler(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/
static uint8_t dfu_c_addr[BLE_DEV_ADDR_LEN] = {0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0x11};

static uint8_t ntf_pkg_type_to_status[BLE_SVC_DFU_PKG_TYPE_MAX] =
{
  BLE_DFU_C_STATUS_INIT,
  BLE_DFU_C_STATUS_WRITE_START_PKG_ACK,
  BLE_DFU_C_STATUS_WRITE_UPDATING_PKG_ACK,
  BLE_DFU_C_STATUS_WRITE_END_PKG_ACK
};

static dfu_ev_handler dfu_ev_handle_table[BLE_DFU_C_STATUS_MAX] =
{
  NULL,
  ble_dfu_c_recv_adv_report_handler,
  ble_dfu_c_recv_connected_handler,
  ble_dfu_c_recv_store_char_handle_handler,
  ble_dfu_c_recv_read_periph_info_handler,
  ble_dfu_c_recv_write_start_pkg_ack_handler,
  ble_dfu_c_recv_write_updating_pkg_ack_handler,
  ble_dfu_c_recv_write_end_pkg_ack_handler,
  NULL,
};

static struct ble_npl_event dfu_c_process_ev;

static ble_dfu_c_process_ctrl_type process_ctrl = {0};

uint8_t  ble_att_svr_data[BLE_ATT_MTU_MAX];
uint16_t ble_att_svr_data_len;

int32_t image_fd = 0;

/****************************************************************************
 * Public Data
 ********************************-********************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
extern  int ble_hs_adv_find_field(uint8_t type,
                                  const uint8_t *data,
                                  uint8_t length,
                                  const struct ble_hs_adv_field **out);

extern int ble_hs_hci_util_set_data_len(uint16_t conn_handle,
                                        uint16_t tx_octets,
                                        uint16_t tx_time);

/****************************************************************************
 * Name: nimble_central
 ****************************************************************************/


/****************************************************************************
 * Private Functions
 ****************************************************************************/

int ble_dfu_c_mtu_fn(uint16_t conn_handle,
                     const struct ble_gatt_error *error,
                     uint16_t mtu, void *arg)
{
  APP_LOG_WARNING( "Current connection MTU = %d \n", mtu);
  return 0;
}

void ble_dfu_c_print_conn_info(uint16_t conn_handle)
{
  int ret;
  struct ble_gap_conn_desc out_desc;
  ret = ble_gap_conn_find(conn_handle, &out_desc);
  if (ret != 0)
    {
      APP_LOG_ERROR( "\n Find conn failed... = %x\n", ret);
    }

  APP_LOG_WARNING("\nConnection info: handle=%x,itvl=%x,latency=%x,mca=%x,role=%x,state=%x,timeout=%x\n",
                  out_desc.conn_handle,
                  out_desc.conn_itvl,
                  out_desc.conn_latency,
                  out_desc.master_clock_accuracy,
                  out_desc.role,
                  out_desc.sec_state,
                  out_desc.supervision_timeout
                 );
}

void ble_dfu_c_error_handle(void)
{
  process_ctrl.pre_status = process_ctrl.cur_status;
  process_ctrl.cur_status = BLE_DFU_C_STATUS_ERROR;
  APP_LOG_ERROR("\n BLE DFU ERR HANDLE[%d]\n", process_ctrl.err_code);
  /*Terminate the connection*/
  ble_gap_terminate(process_ctrl.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
}

void ble_dfu_c_sync_cb(void)
{
  int ret;
  struct ble_gap_disc_params disc_params;

  memset(&disc_params, 0, sizeof(disc_params));

  disc_params.filter_duplicates = 0;
  disc_params.filter_policy = 0;
  disc_params.itvl = BLE_DFU_SCAN_ITVL;
  disc_params.limited = 0;
  disc_params.passive = 1;
  disc_params.window = BLE_DFU_SCAN_WINDOW;

  ret = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &disc_params, ble_dfu_c_gap_event, NULL);
  ASSERT(0 == ret);
  APP_LOG_INFO("\n BLE_DISCOVING....[%d]\n", ret);
}

int ble_dfu_c_gatt_read_periph_info_fn(uint16_t conn_handle,
                                       const struct ble_gatt_error *error,
                                       struct ble_gatt_attr *attr,
                                       void *arg)
{
  int len;
  ble_dfu_periph_info_type periph_info;
  len = OS_MBUF_PKTLEN(attr->om);

  os_mbuf_copydata(attr->om, 0, len, (uint8_t *)&periph_info);

  process_ctrl.periph_flash_size = periph_info.flash_size;
  process_ctrl.periph_img_ver = periph_info.img_version;
  APP_LOG_INFO("\nPeriph info ready = flash_size[%x] img_ver[%x] \n", periph_info.flash_size, periph_info.img_version);
  ble_dfu_c_write_start_pkg();
  return 0;
}

int ble_dfu_c_chr_fn(uint16_t conn_handle,
                     const struct ble_gatt_error *error,
                     const struct ble_gatt_chr *chr, void *arg)
{
  int ret = 0;
  if (chr == 0)
    {
      APP_LOG_WARNING("\n ERROR! NO CHAR...\n");
    }
  APP_LOG_INFO("\n CHAR_CB: conn_handle[%x]--char[%x]=[def_h=%x, val_h=%x, prop=%x]\n",
               conn_handle,
               chr,
               chr->def_handle,
               chr->val_handle,
               chr->properties);

  if (0 == ble_uuid_cmp((ble_uuid_t *)&chr->uuid, (ble_uuid_t *)&gatt_svr_chr_dfu_read_uuid))
    {
      process_ctrl.read_handle = chr->val_handle;
    }
  else if (0 == ble_uuid_cmp((ble_uuid_t *)&chr->uuid, (ble_uuid_t *)&gatt_svr_chr_dfu_write_uuid))
    {
      process_ctrl.write_handle = chr->val_handle;
    }
  else if (0 == ble_uuid_cmp((ble_uuid_t *)&chr->uuid, (ble_uuid_t *)&gatt_svr_chr_dfu_ack_uuid))
    {
      process_ctrl.ntf_handle = chr->val_handle;
    }
  else
    {
      APP_LOG_WARNING("\n Not match CHAR ATTR....[%d]\n", ret);
    }
  return 0;
}

static int
ble_dfu_c_gap_event(struct ble_gap_event *event, void *arg)
{
  int ret;

  switch (event->type)
    {
      case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        APP_LOG_INFO( "\nConnection EVENT...\n");
        APP_LOG_INFO( "\n handle %d; status=%x \n", event->connect.conn_handle, event->connect.status);

        if (event->connect.status == 0)
          {
#if (BLE_PHY_SETTING)
            ret = ble_gap_set_prefered_le_phy(central_conn_handle, BLE_GAP_LE_PHY_2M_MASK,
                                              BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_CODED_ANY);
            if (0 != ret)
              {
                APP_LOG_ERROR( "\n BLE set phy rate failed[%x]...\n", ret);
              }
#endif

#if (BLE_DLE_SETTING)
            ret = ble_hs_hci_util_set_data_len(event->connect.conn_handle, BLE_HCI_SET_DATALEN_TX_OCTETS_MAX, BLE_HCI_SET_DATALEN_TX_TIME_MAX);
            if (0 != ret)
              {
                APP_LOG_ERROR( "\n BLE set data len failed[%x]...\n", ret);
              }
#endif
            ret = ble_gattc_exchange_mtu(event->connect.conn_handle, ble_dfu_c_mtu_fn, NULL);
            if (0 != ret)
              {
                APP_LOG_ERROR( "\n BLE set exchange MTU failed[%x]...\n", ret);
              }

            ble_dfu_c_print_conn_info(event->connect.conn_handle);
            process_ctrl.pre_status = process_ctrl.cur_status;
            process_ctrl.cur_status = BLE_DFU_C_STATUS_CONNECTED;
            process_ctrl.conn_handle = event->connect.conn_handle;
            ble_npl_eventq_put((struct ble_npl_eventq *)os_eventq_dflt_get(), &dfu_c_process_ev);
          }


        return 0;

      case BLE_GAP_EVENT_DISCONNECT:
        APP_LOG_INFO( "disconnect; reason=%x \n", event->disconnect.reason);
        memset(&process_ctrl, 0, sizeof(ble_dfu_c_process_ctrl_type));
        ble_dfu_c_sync_cb();
        return 0;

      case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        APP_LOG_INFO( "connection updated; status=%x \n",
                      event->conn_update.status);
        ble_dfu_c_print_conn_info(event->connect.conn_handle);
        return 0;

      case BLE_GAP_EVENT_NOTIFY_RX:
        {
          uint16_t len;
          uint8_t data[BLE_SVC_DFU_ACK_PKG_LEN];

          len = OS_MBUF_PKTLEN(event->notify_rx.om);
          if (len != BLE_SVC_DFU_ACK_PKG_LEN)
            {
              APP_LOG_ERROR( "NOTIFY RX NOT DFU PKG...\n");
              return 0;
            }
          ret = os_mbuf_copydata(event->notify_rx.om, 0, len, data);
          if (0 != ret)
            {
              APP_LOG_ERROR( "NOTIFY get data failed...\n");
              return 0;
            }

          if (ble_dfu_c_crc16_check(len, data))
            {
              ble_dfu_c_recv_NTF_pkg(len, data);
            }
        }
        return 0;
      case BLE_GAP_EVENT_DISC:
        {
          const struct ble_hs_adv_field *field;

          ret = ble_hs_adv_find_field(BLE_HS_ADV_TYPE_MFG_DATA, event->disc.data,
                                      event->disc.length_data, &field);
          if (ret == 0)
            {
              uint16_t company_id;
              uint16_t val;
              company_id = *((uint16_t *)&field->value[0]);
              val = *((uint16_t *)&field->value[2]);
              if ((company_id == BLE_SVC_DFU_MFG_COMPANY_ID) && (val == BLE_SVC_DFU_MFG_OTA_ID))
                {
                  process_ctrl.pre_status = process_ctrl.cur_status;
                  process_ctrl.cur_status = BLE_DFU_C_STATUS_ADV_REPORTED;
                  process_ctrl.periph_addr = event->disc.addr;
                  ble_npl_eventq_put((struct ble_npl_eventq *)os_eventq_dflt_get(), &dfu_c_process_ev);
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

static void
ble_dfu_c_crc16(uint16_t len, uint8_t *data)
{

  assert(len >= BLE_SVC_DFU_MIN_DATA_LEN);
  /*The last two types always crc16 verification.*/
  *((uint16_t *)&data[len - sizeof(uint16_t)]) = crc16((FAR const uint8_t *)data, len - sizeof(uint16_t));
}

static int
ble_dfu_c_crc16_check(uint16_t len, uint8_t *data)
{
  uint16_t crc;

  assert(len >= BLE_SVC_DFU_MIN_DATA_LEN);
  crc = crc16((FAR const uint8_t *)data, len - sizeof(uint16_t));

  if (crc == *((uint16_t *)&data[len - sizeof(uint16_t)]))
    {
      return TRUE;
    }
  else
    {
      return FALSE;
    }
}

static void
ble_dfu_c_write_start_pkg(void)
{
  int ret;
  uint16_t block_c = 0, block_s = 0;
  ble_dfu_start_pkg_type start_pkg = {0};

  start_pkg.type = BLE_SVC_DFU_PKG_TYPE_START;
  image_fd = open(BEL_DFU_IMAGE_PATH, O_RDONLY);
  if (image_fd < 0)
    {
      APP_LOG_ERROR("Open %s file failed: %d\n", image_fd, errno);
    }
  else
    {
      APP_LOG_INFO("Open %s file successed!: %d\n", image_fd);
      /*To do read real information from flash.*/
      block_c = 100;
      block_s = 100;
    }

  /*Get total couont and block size from flash.*/
  process_ctrl.block_size = block_s;
  process_ctrl.block_cnt = block_c;
  process_ctrl.block_num = 1;
  process_ctrl.pre_status = process_ctrl.cur_status;
  process_ctrl.cur_status = BLE_DFU_C_STATUS_ADV_REPORTED;
  start_pkg.block_cnt = process_ctrl.block_cnt;
  start_pkg.block_size = process_ctrl.block_size;

  ble_dfu_c_crc16(sizeof(ble_dfu_start_pkg_type), (uint8_t *)&start_pkg);

  APP_LOG_INFO("\nStart PKG type[%d], info[%d] err[%d] crc[%d]\n",
               start_pkg.type, start_pkg.block_cnt, start_pkg.block_size, start_pkg.crc);

  APP_LOG_INFO("\n3--Write IMG INFO to peripheral.\n");
  ret = ble_gattc_write_no_rsp_flat(process_ctrl.conn_handle, process_ctrl.write_handle, &start_pkg, sizeof(ble_dfu_start_pkg_type));
  if (0 != ret)
    {
      process_ctrl.err_code = BLE_DFU_C_ERR_WRITE_START_PKG_ERR;
      ble_dfu_c_error_handle();
    }
}

static void
ble_dfu_c_write_update_pkg(void)
{
  int ret;
  uint8_t len = 0;
  uint8_t bytes = 0;
  uint8_t update_data[BLE_SVC_DFU_MAX_DATA_LEN] = {0};

  *((uint16_t *)&update_data[len]) = BLE_SVC_DFU_PKG_TYPE_UPDATING;
  len += sizeof(uint16_t);
  *((uint16_t *)&update_data[len]) = process_ctrl.block_num;
  len += sizeof(uint16_t);
  *((uint16_t *)&update_data[len]) = process_ctrl.block_size;
  len += sizeof(uint16_t);

  bytes = read(image_fd, &update_data[len], process_ctrl.block_size);
  if (bytes < process_ctrl.block_size)
    {
      APP_LOG_WARNING("\Read size of the end--len[%d]_cnt[%d].\n", len, process_ctrl.block_cnt);
    }
  else
    {
      APP_LOG_INFO("\Read size of the end--len[%d]_cnt[%d].\n", len, process_ctrl.block_num, process_ctrl.block_size);
    }
  //memcpy(&update_data[len], update_data, process_ctrl.block_size);
  len += bytes;

  /*Add two bytes for crc16.*/
  len += sizeof(uint16_t);
  ble_dfu_c_crc16(len, update_data);

  assert(len < BLE_SVC_DFU_MAX_DATA_LEN);
  ret = ble_gattc_write_no_rsp_flat(process_ctrl.conn_handle, process_ctrl.write_handle, update_data, len);
  if (0 != ret)
    {
      process_ctrl.err_code = BLE_DFU_C_ERR_WRITE_START_PKG_ERR;
      ble_dfu_c_error_handle();
    }
}

static void
ble_dfu_c_write_end_pkg(void)
{
  int ret;
  ble_dfu_end_pkg_type end_pkg = {0};

  end_pkg.type = BLE_SVC_DFU_PKG_TYPE_END;

  ble_dfu_c_crc16(sizeof(ble_dfu_start_pkg_type), (uint8_t *)&end_pkg);

  APP_LOG_INFO("\nEND PKG type[%d], crc[%d]\n",
               end_pkg.type, end_pkg.crc);

  APP_LOG_INFO("\n3--Write IMG INFO to peripheral.\n");
  ret = ble_gattc_write_no_rsp_flat(process_ctrl.conn_handle, process_ctrl.write_handle, &end_pkg, sizeof(ble_dfu_start_pkg_type));
  if (0 != ret)
    {
      process_ctrl.err_code = BLE_DFU_C_ERR_WRITE_START_PKG_ERR;
      ble_dfu_c_error_handle();
    }

  ret = close(image_fd);
  if (ret < 0)
    {
      APP_LOG_ERROR("Close %s file failed: %d\n", image_fd, errno);
    }

}

static void
ble_dfu_c_recv_NTF_pkg(uint16_t len, uint8_t *data)
{
  ble_dfu_pkg_ack_type *ack_pkg;

  ack_pkg = (ble_dfu_pkg_ack_type *)data;

  if (0 != ack_pkg->error)
    {
      APP_LOG_ERROR("ACK type[%d], info[%d] err[%d] crc[%d]\n",
                    ack_pkg->ack_type,
                    ack_pkg->ack_info,
                    ack_pkg->error,
                    ack_pkg->crc);
      process_ctrl.err_code = BLE_DFU_C_ERR_REMOTE_ERR;
      ble_dfu_c_error_handle();
      return;
    }
  process_ctrl.pre_status = process_ctrl.cur_status;
  process_ctrl.cur_status = ntf_pkg_type_to_status[ack_pkg->ack_type];
  process_ctrl.ack_info = ack_pkg->ack_info;
  ble_npl_eventq_put((struct ble_npl_eventq *)os_eventq_dflt_get(), &dfu_c_process_ev);
  return;
}

static void
ble_dfu_c_recv_adv_report_handler(void)
{
  int ret;
  struct ble_gap_conn_params conn_params;
  conn_params.scan_itvl = BLE_DFU_SCAN_WINDOW;
  conn_params.scan_window = BLE_DFU_SCAN_WINDOW;
  conn_params.itvl_min = BLE_GAP_INITIAL_CONN_ITVL_MIN;
  conn_params.itvl_max = BLE_GAP_INITIAL_CONN_ITVL_MIN;
  conn_params.latency = BLE_GAP_INITIAL_CONN_LATENCY;
  conn_params.supervision_timeout = BLE_GAP_INITIAL_SUPERVISION_TIMEOUT;
  conn_params.min_ce_len = BLE_GAP_INITIAL_CONN_MIN_CE_LEN;
  conn_params.max_ce_len = BLE_GAP_INITIAL_CONN_MAX_CE_LEN;
  ble_gap_disc_cancel();
  ret = ble_gap_connect(BLE_OWN_ADDR_PUBLIC,
                        &process_ctrl.periph_addr,
                        BLE_HS_FOREVER,
                        &conn_params,
                        ble_dfu_c_gap_event, NULL);

  ASSERT(0 == ret);
}

static void
ble_dfu_c_recv_connected_handler(void)
{
  int ret;
  APP_LOG_INFO("\n1--Read ALL CHAR handle and save it.\n");
  ret = ble_gattc_disc_all_chrs(process_ctrl.conn_handle,
                                BLE_GATT_HANDLE_START,
                                BLE_GATT_HANDLE_END,
                                ble_dfu_c_chr_fn,
                                NULL);
  if (0 != ret)
    {
      process_ctrl.err_code = BLE_DFU_C_ERR_DISC_CHAR_ERR;
      ble_dfu_c_error_handle();
    }
  else
    {
      process_ctrl.pre_status = process_ctrl.cur_status;
      process_ctrl.cur_status = BLE_DFU_C_STATUS_STORE_CHAR_HANDLE;
      sleep(1);
      ble_npl_eventq_put((struct ble_npl_eventq *)os_eventq_dflt_get(), &dfu_c_process_ev);
    }
}

static void
ble_dfu_c_recv_store_char_handle_handler(void)
{
  if ((process_ctrl.read_handle && process_ctrl.write_handle && process_ctrl.ntf_handle) == 0)
    {
      process_ctrl.err_code = BLE_DFU_C_ERR_CHAR_ATTR_MISS_ERR;
      ble_dfu_c_error_handle();
      return;
    }

  APP_LOG_INFO("\n2--ALL CHAR handle is ready.\n");
  process_ctrl.pre_status = process_ctrl.cur_status;
  process_ctrl.cur_status = BLE_DFU_C_STATUS_READ_PERIPH_INFO;
  ble_npl_eventq_put((struct ble_npl_eventq *)os_eventq_dflt_get(), &dfu_c_process_ev);
}

static void
ble_dfu_c_recv_read_periph_info_handler(void)
{
  int ret;

  APP_LOG_INFO("\n3--Read Periph INFO.\n");
  ret = ble_gattc_read(process_ctrl.conn_handle, process_ctrl.read_handle, ble_dfu_c_gatt_read_periph_info_fn, NULL);
  if (0 != ret)
    {
      process_ctrl.err_code = BLE_DFU_C_ERR_READ_PEIRPH_INFO_ERR;
      ble_dfu_c_error_handle();
    }
}

static void
ble_dfu_c_recv_write_start_pkg_ack_handler(void)
{
  APP_LOG_INFO("\n4--Received start ack, keep update.\n");
  ble_dfu_c_write_update_pkg();
}

static void
ble_dfu_c_recv_write_updating_pkg_ack_handler(void)
{

  assert(process_ctrl.block_num <= process_ctrl.block_cnt);
  if (process_ctrl.block_cnt == process_ctrl.block_num)
    {
      APP_LOG_INFO("\n6--Update done[%d/%d]...\n", process_ctrl.block_num, process_ctrl.block_cnt);
      ble_dfu_c_write_end_pkg();
    }
  else
    {
      APP_LOG_INFO("\n5--Update acked, num = [%d].\n", process_ctrl.block_num);
      if (process_ctrl.block_num != process_ctrl.ack_info)
        {
          process_ctrl.err_code = BLE_DFU_C_ERR_WRITE_UPDATE_PKG_ERR;
          ble_dfu_c_error_handle();
          return;
        }
      process_ctrl.block_num++;
      ble_dfu_c_write_update_pkg();
    }
}

static void
ble_dfu_c_recv_write_end_pkg_ack_handler(void)
{
}

static void
ble_dfu_c_recv_ev_handle(struct ble_npl_event *ev)
{
  if (NULL != dfu_ev_handle_table[process_ctrl.cur_status])
    {
      dfu_ev_handle_table[process_ctrl.cur_status]();
    }
}


__attribute__((__noreturn__))
#ifdef CONFIG_BUILD_KERNEL
void main(int argc, FAR char *argv[])
#else
void nimble_dfu_c_main(int argc, char *argv[])
#endif
{
  int ret;
  (void)ret;

  nimble_sysinit_start();
  nimble_hal_init();
  nimble_sysinit_end();

  memcpy(g_dev_addr, dfu_c_addr, 6);
  ble_hs_cfg.reset_cb = NULL;
  ble_hs_cfg.sync_cb = &ble_dfu_c_sync_cb;
  ble_hs_cfg.gatts_register_cb = NULL;
  ble_hs_cfg.store_status_cb = NULL;
  ret = ble_att_set_preferred_mtu(BLE_SVC_DFU_MAX_DATA_LEN);

  APP_LOG_INFO("\n\n\n\n\n\n\n\n%s__%d__%d\n", __FUNCTION__, __LINE__, ret );

  //DEBUG_GPIO_INIT(34, 36);

  //DEBUG_GPIO_TOGGLE(34, 1);
  //DEBUG_GPIO_TOGGLE(35, 2);
  //DEBUG_GPIO_TOGGLE(36, 3);

  ble_npl_event_init(&dfu_c_process_ev, ble_dfu_c_recv_ev_handle, NULL);
  memset(&process_ctrl, 0, sizeof(ble_dfu_c_process_ctrl_type));

  while (1)
    {
      os_eventq_run(os_eventq_dflt_get());
    }
}
