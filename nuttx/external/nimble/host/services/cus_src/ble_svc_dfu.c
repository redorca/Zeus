/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <crc16.h>
#include "sysinit/sysinit.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "os/endian.h"
#include "services/cus_inc/ble_svc_dfu.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/
#define IMG_SECTOR_SIZR (1024)

#define UPDATE_PROCESS_5_PERCENT (20)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/
/* UUID generated by http://www.guidgenerator.com {c9b6b5bd-4540-43f4-9715-c0743af859ca}*/

/* {c9b60001-4540-43f4-9715-c0743af859ca} */
const ble_uuid128_t gatt_svr_svc_dfu_uuid =
    BLE_UUID128_INIT(0xca, 0x59, 0xf8, 0x3a, 0x74, 0xc0, 0x15, 0x97,
                     0xf4, 0x43, 0x40, 0x45, 0x01, 0x00, 0xb6, 0xc9);

/* {c9b60002-4540-43f4-9715-c0743af859ca} */
const ble_uuid128_t gatt_svr_chr_dfu_read_uuid =
    BLE_UUID128_INIT(0xca, 0x59, 0xf8, 0x3a, 0x74, 0xc0, 0x15, 0x97,
                     0xf4, 0x43, 0x40, 0x45, 0x02, 0x00, 0xb6, 0xc9);

/* {c9b60003-4540-43f4-9715-c0743af859ca} */
const ble_uuid128_t gatt_svr_chr_dfu_write_uuid =
    BLE_UUID128_INIT(0xca, 0x59, 0xf8, 0x3a, 0x74, 0xc0, 0x15, 0x97,
                     0xf4, 0x43, 0x40, 0x45, 0x03, 0x00, 0xb6, 0xc9);

/* {c9b60004-4540-43f4-9715-c0743af859ca} */
const ble_uuid128_t gatt_svr_chr_dfu_ack_uuid =
    BLE_UUID128_INIT(0xca, 0x59, 0xf8, 0x3a, 0x74, 0xc0, 0x15, 0x97,
                     0xf4, 0x43, 0x40, 0x45, 0x04, 0x00, 0xb6, 0xc9);

/****************************************************************************
 * Private Data
 ****************************************************************************/
static ble_dfu_process_ctl_type ble_dfu_pc = {0};
static ble_dfu_access_handler access_handler = {0};
static uint8_t * img_buf = NULL;
static uint16_t  img_buf_len = 0;

/**
 * The vendor specific "ble_svc_dfu" service consists of one write no-rsp characteristic
 * and one notification only read charateristic
 *     o "write no-rsp": a single-byte characteristic that can be written only
 *       over a non-encrypted connection
 *     o "read": a single-byte characteristic that can always be read only via
 *       notifications
 */

/* ble dfu attr handle */
static uint16_t g_ble_svc_dfu_attr_read_handle;

static uint16_t g_ble_svc_dfu_attr_write_handle;

static uint16_t g_ble_svc_dfu_attr_ack_handle;

static uint16_t g_svc_dfu_conn_handle;

static int
ble_svc_dfu_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_svc_def gatt_dfu_svr_svcs[] = {
    {
        /* Service: uart */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_dfu_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
          {
              .uuid = &gatt_svr_chr_dfu_read_uuid.u,
              .val_handle = &g_ble_svc_dfu_attr_read_handle,
              .access_cb = ble_svc_dfu_chr_access,
              .flags = BLE_GATT_CHR_F_READ,
          },
          {
              .uuid = &gatt_svr_chr_dfu_write_uuid.u,
              .val_handle = &g_ble_svc_dfu_attr_write_handle,
              .access_cb = ble_svc_dfu_chr_access,
              .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
          },
          {
              .uuid = &gatt_svr_chr_dfu_ack_uuid.u,
              .val_handle = &g_ble_svc_dfu_attr_ack_handle,
              .access_cb = ble_svc_dfu_chr_access,
              .flags = BLE_GATT_CHR_F_NOTIFY,
          },
          {
              0, /* No more characteristics in this service */
          }
        },
    },
    {
        0, /* No more services */
    },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static void
ble_svc_dfu_send_data(uint8_t * data, uint32_t len)
{
    struct os_mbuf *om;

    om = ble_hs_mbuf_from_flat(data, len);
    if (!om) {
        return;
    }
    ble_gattc_notify_custom(g_svc_dfu_conn_handle,
                            g_ble_svc_dfu_attr_ack_handle, om);
}

static uint32_t
ble_svc_dfu_pkg_ack(uint16_t status, uint16_t error_code, uint16_t info)
{
    uint32_t ret = 0;
    ble_dfu_pkg_ack_type ack_pkg = {0};

    /*Ack remote device.*/
    ack_pkg.ack_type = status;
    ack_pkg.ack_info = info;
    ack_pkg.error = error_code;
    ack_pkg.crc = crc16((FAR const uint8_t *)&ack_pkg, sizeof(ble_dfu_pkg_ack_type)-sizeof(uint16_t));
    BLE_HS_LOG(DEBUG, "Notify_size(%d)_type(%x),info(%x),err(%x),crc(%x)...\n",
                      sizeof(ble_dfu_pkg_ack_type),
                      ack_pkg.ack_type,
                      ack_pkg.ack_info,
                      ack_pkg.error,
                      ack_pkg.crc);
    ble_svc_dfu_send_data((uint8_t *)&ack_pkg, sizeof(ble_dfu_pkg_ack_type));
    return ret;
}

static uint32_t
ble_svc_dfu_write_handle(struct os_mbuf *om)
{
    uint32_t ret = 0;
    uint16_t len = 0;
    uint16_t crc = 0;
    uint16_t type;
    uint8_t * write_data;

    write_data = malloc(BLE_SVC_DFU_MAX_DATA_LEN);
    assert(write_data != NULL);

    /*Central will controll the total size not over BLE_SVC_DFU_MAX_DATA_LEN*/
    len = OS_MBUF_PKTLEN(om);
    assert(len <= BLE_SVC_DFU_MAX_DATA_LEN);
    ret = os_mbuf_copydata(om, 0, len, write_data);
    type = *(uint16_t *)&write_data[0];
    /*Read Type*/
    if(len > BLE_SVC_DFU_MAX_DATA_LEN || len < BLE_SVC_DFU_MIN_DATA_LEN || ret != 0)
    {
      ret = BLE_HS_EBADDATA;
      BLE_HS_LOG(ERROR, "Data Format ERROR[%d]...\n", ret);
      ble_svc_dfu_pkg_ack(type, BLE_SVC_DFU_ERR_BAD_DATA, ble_dfu_pc.cur_block);
      free(write_data);
      return ret;
    }

    /*CRC check*/
    crc = crc16(write_data, len-2);
    if(crc != *((uint16_t *)&write_data[len-2]))
    {
      ret = BLE_HS_EBADDATA;
      BLE_HS_LOG(ERROR, "Data len(%d) CRC[%x,%x] check ERROR[%d]...\n", len, crc, *((uint16_t *)&write_data[len-2]), ret);
      ble_svc_dfu_pkg_ack(type, BLE_SVC_DFU_ERR_CRC, ble_dfu_pc.cur_block);
      free(write_data);
      return ret;
    }

    switch (type)
    {
      case BLE_SVC_DFU_PKG_TYPE_START:
      {
          BLE_HS_LOG(DEBUG,"%s__%d: Type[%d] Enter DFU start case...\n",__FUNCTION__,__LINE__,type);
          ble_dfu_start_pkg_type * start_pkg;
          uint16_t error_code = BLE_SVC_DFU_ERR_NONE;

          start_pkg = (ble_dfu_start_pkg_type * )write_data;
          /*Check status*/
          if(ble_dfu_pc.status != BLE_SVC_DFU_PKG_TYPE_END
             && ble_dfu_pc.status != BLE_SVC_DFU_PKG_TYPE_INIT)
          {
              ret = BLE_HS_EAPP;
              error_code = BLE_SVC_DFU_ERR_STATUS;
              BLE_HS_LOG(ERROR, "DFU start status[%d] not correct\n", ble_dfu_pc.status);
          }
          else if(len != sizeof(ble_dfu_start_pkg_type))
          {
              ret = BLE_HS_EAPP;
              error_code = BLE_SVC_DFU_ERR_BAD_DATA;
              BLE_HS_LOG(ERROR, "start Data length[%d] not correct\n", len);
          }
          else
          {
              ble_dfu_pc.cur_block = 0;
              ble_dfu_pc.status = BLE_SVC_DFU_PKG_TYPE_START;
              ble_dfu_pc.block_cnt = start_pkg->block_cnt;
              ble_dfu_pc.block_size = start_pkg->block_size;
              BLE_HS_LOG(DEBUG,"Block size[%d], Total count[%d].\n",start_pkg->block_size,start_pkg->block_cnt);

              img_buf = malloc(IMG_SECTOR_SIZR);
              assert(img_buf != NULL);
          }

          ble_svc_dfu_pkg_ack(type, error_code, ble_dfu_pc.cur_block);
      }
        break;

      case BLE_SVC_DFU_PKG_TYPE_UPDATING:
      {
          ble_dfu_updating_pkg_type * update_pkg;
          uint16_t error_code = BLE_SVC_DFU_ERR_NONE;
          update_pkg = (ble_dfu_updating_pkg_type * )write_data;
          /*Check status*/
          if(ble_dfu_pc.status != BLE_SVC_DFU_PKG_TYPE_START
            &&ble_dfu_pc.status != BLE_SVC_DFU_PKG_TYPE_UPDATING)
          {
              ret = BLE_HS_EAPP;
              error_code = BLE_SVC_DFU_ERR_STATUS;
              BLE_HS_LOG(ERROR, "DFU update status[%d] not correct\n", ble_dfu_pc.status);
          }
          else if((ble_dfu_pc.cur_block+1 != update_pkg->block_num) || (ble_dfu_pc.block_cnt < update_pkg->block_num))
          {
              ret = BLE_HS_EAPP;
              error_code = BLE_SVC_DFU_ERR_WRONG_BLOCK_NO;
              BLE_HS_LOG(ERROR, "Data length not correct,cur[%d] coming[%d], total = [%d]\n",ble_dfu_pc.cur_block,update_pkg->block_num,ble_dfu_pc.block_cnt);
          }
          else if((update_pkg->block_len > ble_dfu_pc.block_size) ||
                  ((update_pkg->block_len < ble_dfu_pc.block_size) && (update_pkg->block_num != ble_dfu_pc.block_cnt))
                 )
          {
              ret = BLE_HS_EAPP;
              error_code = BLE_SVC_DFU_ERR_WRONG_BLOCK_SIZE;
              BLE_HS_LOG(ERROR, "Data size not correct,data_num[%d] data_len[%d], total_num[%d], block_size[%d]\n",
                                                                                              update_pkg->block_num,
                                                                                              update_pkg->block_len,
                                                                                              ble_dfu_pc.block_cnt,
                                                                                              ble_dfu_pc.block_size);
          }
          else
          {
              uint16_t copy_len = 0;

              if(update_pkg->block_num % (ble_dfu_pc.block_cnt/UPDATE_PROCESS_5_PERCENT) == 0)
                {
                  BLE_HS_LOG(WARN,"Image updated to --%d%%--\n",update_pkg->block_num*100/ble_dfu_pc.block_cnt);
                }
              if(update_pkg->block_num == ble_dfu_pc.block_cnt)
                {
                  BLE_HS_LOG(WARN,"Image updated to --%d%%--\n",100);
                }

              ble_dfu_pc.cur_block = update_pkg->block_num;
              ble_dfu_pc.status = BLE_SVC_DFU_PKG_TYPE_UPDATING;

              /*To improve the flash operation efficiency, we write 1K one time.*/
              /*If lenght not get to 1K*/
              if(img_buf_len + update_pkg->block_len <= IMG_SECTOR_SIZR)
                {
                  copy_len = update_pkg->block_len;
                }
              /*If lenght exceed 1K*/
              else if(img_buf_len + update_pkg->block_len > IMG_SECTOR_SIZR)
                {
                  copy_len = IMG_SECTOR_SIZR - img_buf_len;
                }
              else
                {
                  BLE_HS_LOG(ERROR, "ERROR dat length %d %d %d.\n",img_buf_len, update_pkg->block_len, IMG_SECTOR_SIZR);
                }

              memcpy(&img_buf[img_buf_len], &write_data[sizeof(ble_dfu_updating_pkg_type)], copy_len);
              img_buf_len += copy_len;
              assert(img_buf_len <= IMG_SECTOR_SIZR);
              if(img_buf_len == IMG_SECTOR_SIZR)
                {
                  access_handler.write(img_buf, img_buf_len);
                  memset(img_buf, 0, IMG_SECTOR_SIZR);
                  img_buf_len = update_pkg->block_len - copy_len;
                  memcpy(img_buf, &write_data[sizeof(ble_dfu_updating_pkg_type)+copy_len], img_buf_len);
                }
          }
          ble_svc_dfu_pkg_ack(type, error_code, update_pkg->block_num);
      }
        break;

      case BLE_SVC_DFU_PKG_TYPE_END:
        BLE_HS_LOG(DEBUG,"\nImage update procedure is done and ready to reset, pleas wait second...\n");
        ble_dfu_pc.status = BLE_SVC_DFU_PKG_TYPE_END;
        ble_svc_dfu_pkg_ack(type, 0, 0);
        ble_svc_dfu_reset();
        sleep(1);
        access_handler.reset();
        break;

      default:
        BLE_HS_LOG(WARN,"%s__%d: Type[%d] not matched, enter default case...\n",__FUNCTION__,__LINE__,type);
        break;
    }

    free(write_data);
    return ret;
}

static int
ble_svc_dfu_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint32_t ret;

    (void)ret;
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            BLE_HS_LOG(DEBUG, "WRITE CHR.\n");
            ret = ble_svc_dfu_write_handle(ctxt->om);
            if(ret != 0){
              BLE_HS_LOG(ERROR, "Write data handle failed[%d]...\n", ret);
            }
            return 0;

        case BLE_GATT_ACCESS_OP_WRITE_DSC:
            BLE_HS_LOG(DEBUG, "WRITE DSC.\n");
            return 0;

        case BLE_GATT_ACCESS_OP_READ_CHR:
          {
            uint16_t len = 0;
            uint8_t * data;
            data = malloc(BLE_SVC_DFU_MAX_DATA_LEN);
            assert(data != NULL);
            BLE_HS_LOG(DEBUG, "READ CHAR.\n");
            assert(access_handler.read != 0);
            access_handler.read(data, &len);
            if(data != NULL){
              os_mbuf_append(ctxt->om, data, len);
            } else {
              BLE_HS_LOG(ERROR, "Get local dfu information failed...\n");
            }
            free(data);
            return 0;
          }
        case BLE_GATT_ACCESS_OP_READ_DSC:
            BLE_HS_LOG(DEBUG, "READ DSC.\n");
            return 0;

        default:
            assert(0);
            return BLE_ATT_ERR_UNLIKELY;
    }
}

/**
 * ble_svc_dfu GATT server initialization
 *
 */
static int
ble_svc_dfu_gatt_svr_init(void)
{
    int rc;

    rc = ble_gatts_count_cfg(gatt_dfu_svr_svcs);
    if (rc != 0) {
        goto err;
    }

    rc = ble_gatts_add_svcs(gatt_dfu_svr_svcs);
    if (rc != 0) {
        return rc;
    }

err:
    return rc;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/**
 * Reset the dfu process control information.
 *
 * @param connection handle
 */
void
ble_svc_dfu_reset(void)
{
  memset(&ble_dfu_pc, 0, sizeof(ble_dfu_process_ctl_type));
}

/**
 * Sets the global connection handle
 *
 * @param connection handle
 */
void
ble_svc_dfu_set_conn_handle(uint16_t conn_handle)
{
  g_svc_dfu_conn_handle = conn_handle;
}

/**
 * Disconnection handler
 *
 * @param connection handle
 */
void
ble_svc_dfu_disconnect(void)
{
  memset(&ble_dfu_pc, 0, sizeof(ble_dfu_process_ctl_type));
}

/**
 * ble_svc_dfu sevices initialization
 *
 */
void
ble_svc_dfu_init(ble_dfu_access_handler * handler)
{
  ble_svc_dfu_gatt_svr_init();
  memset(&ble_dfu_pc, 0, sizeof(ble_dfu_process_ctl_type));
  access_handler.read = handler->read;
  access_handler.write = handler->write;
  access_handler.reset = handler->reset;
}
