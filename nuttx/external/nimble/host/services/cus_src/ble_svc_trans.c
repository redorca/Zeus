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

#include "sysinit/sysinit.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/cus_inc/ble_svc_trans.h"
#include "os/endian.h"

#define WRITE_DATA_LEN          (100)

/* ble trans attr read handle */
static uint16_t g_ble_svc_trans_attr_read_handle;

/* ble trans attr throughput handle */
static uint16_t g_ble_svc_trans_attr_throughput_handle;

/* ble trans attr write handle */
static uint16_t g_ble_svc_trans_attr_write_handle;

/* ble trans attr write check handle */
static uint16_t g_ble_svc_trans_attr_write_check_handle;

/* Pointer to a data buffer */
static uint8_t g_write_ctrl_val[WRITE_DATA_LEN+1] = {0};

static uint8_t notify_mode = 0;

static uint16_t g_data_conn_handle;

extern int ble_hs_hci_util_set_data_len(uint16_t conn_handle, uint16_t tx_octets,uint16_t tx_time);

/**
 * The vendor specific "ble_svc_trans" service consists of one write no-rsp characteristic
 * and one notification only read charateristic
 *     o "write no-rsp": a single-byte characteristic that can be written only
 *       over a non-encrypted connection
 *     o "read": a single-byte characteristic that can always be read only via
 *       notifications
 */

/* {6E400001-B5A3-F393-E0A9-E50E24DCCA9E} */
const ble_uuid128_t gatt_svr_svc_trans_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);

/* {6E400002-B5A3-F393-E0A9-E50E24DCCA9E} */
const ble_uuid128_t gatt_svr_chr_trans_read_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);

/* {6E400003-B5A3-F393-E0A9-E50E24DCCA9E} */
const ble_uuid128_t gatt_svr_chr_trans_throughput_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

/* {6E400004-B5A3-F393-E0A9-E50E24DCCA9E} */
const ble_uuid128_t gatt_svr_chr_trans_write_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x04, 0x00, 0x40, 0x6e);

/* {6E400005-B5A3-F393-E0A9-E50E24DCCA9E} */
const ble_uuid128_t gatt_svr_chr_trans_write_check_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x05, 0x00, 0x40, 0x6e);

static int
gatt_svr_chr_access_trans_write(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_svc_def gatt_trnas_svr_svcs[] = {
    {
        /* Service: trans */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_trans_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) { {
            .uuid = &gatt_svr_chr_trans_read_uuid.u,
            .val_handle = &g_ble_svc_trans_attr_read_handle,
            .access_cb = gatt_svr_chr_access_trans_write,
            .flags = BLE_GATT_CHR_F_NOTIFY,
        }, {
            /* Characteristic: throughput */
            .uuid = &gatt_svr_chr_trans_throughput_uuid.u,
            .val_handle = &g_ble_svc_trans_attr_throughput_handle,
            .access_cb = gatt_svr_chr_access_trans_write,
            .flags = BLE_GATT_CHR_F_NOTIFY,
        }, {
            /* Characteristic: Write */
            .uuid = &gatt_svr_chr_trans_write_uuid.u,
            .val_handle = &g_ble_svc_trans_attr_write_handle,
            .access_cb = gatt_svr_chr_access_trans_write,
            .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
        }, {
            /* Characteristic: throughput */
            .uuid = &gatt_svr_chr_trans_write_check_uuid.u,
            .val_handle = &g_ble_svc_trans_attr_write_check_handle,
            .access_cb = gatt_svr_chr_access_trans_write,
            .flags = BLE_GATT_CHR_F_NOTIFY,
        }, {
            0, /* No more characteristics in this service */
        } },
    },

    {
        0, /* No more services */
    },
};

static int
gatt_svr_chr_access_trans_write(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    struct os_mbuf *om = ctxt->om;
    BLE_HS_LOG(INFO,"\nAccess op = %d\n", ctxt->op);

    switch (ctxt->op) {
        uint16_t data_len;
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
              data_len = OS_MBUF_PKTLEN(om);
              if(data_len > WRITE_DATA_LEN) {
                  data_len = WRITE_DATA_LEN;
                }
              memset(g_write_ctrl_val, 0, WRITE_DATA_LEN+1);
              g_write_ctrl_val[0] = data_len;
              os_mbuf_copydata(om, 0, data_len, &g_write_ctrl_val[1]);
              ble_svc_trans_ntf_write_check();
              return 0;
        default:
            BLE_HS_LOG(ERROR,"\n");
            return BLE_ATT_ERR_UNLIKELY;
    }
}

/**
 * ble_svc_trans GATT server initialization
 *
 * @param eventq
 * @return 0 on success; non-zero on failure
 */
int
ble_svc_trans_gatt_svr_init(void)
{
    int rc;

    rc = ble_gatts_count_cfg(gatt_trnas_svr_svcs);
    if (rc != 0) {
        goto err;
    }

    rc = ble_gatts_add_svcs(gatt_trnas_svr_svcs);
    if (rc != 0) {
        return rc;
    }

err:
    return rc;
}


uint32_t
ble_svc_trans_ntf_val(uint8_t *data, uint16_t len)
{
    int rc;

    struct os_mbuf *om;
    om = ble_hs_mbuf_from_flat(data, len);
    if(NULL == om)
    {
      BLE_HS_LOG(ERROR,"\nBuffer malloc failed on NTF.\n");
      return BLE_HS_ENOMEM;
    }
    rc = ble_gattc_notify_custom(g_data_conn_handle,
                            g_ble_svc_trans_attr_read_handle, om);
    if(0 != rc)
    {
      BLE_HS_LOG(DEBUG,"\nNTF failed rc = %x.\n", rc);
    }
    return rc;
}

uint32_t
ble_svc_trans_ntf_throughput(uint8_t *data, uint16_t len)
{
    int rc;

    struct os_mbuf *om;
    om = ble_hs_mbuf_from_flat(data, len);
    if(NULL == om)
    {
      BLE_HS_LOG(ERROR,"\nBuffer malloc failed on NTF.\n");
      return BLE_HS_ENOMEM;
    }
    rc = ble_gattc_notify_custom(g_data_conn_handle,
                            g_ble_svc_trans_attr_throughput_handle, om);
    if(0 != rc)
    {
      BLE_HS_LOG(ERROR,"\nNTF failed rc = %x.\n", rc);
    }
    return rc;
}

uint32_t
ble_svc_trans_ntf_write_check()
{
    int rc;

    struct os_mbuf *om;
    om = ble_hs_mbuf_from_flat(&g_write_ctrl_val[1], g_write_ctrl_val[0]);
    if(NULL == om)
    {
      BLE_HS_LOG(ERROR,"\nBuffer malloc failed on NTF.\n");
      return BLE_HS_ENOMEM;
    }
    rc = ble_gattc_notify_custom(g_data_conn_handle,
                            g_ble_svc_trans_attr_write_check_handle, om);
    if(0 != rc)
    {
      BLE_HS_LOG(ERROR,"\nNTF failed rc = %x.\n", rc);
    }

    if(1 == g_write_ctrl_val[0])
      {
        notify_mode = g_write_ctrl_val[1];
      }

    return rc;
}

uint32_t
ble_svc_trans_get_val(uint8_t *data, uint16_t len)
{
  for(int i = 0; i< len; i++)
  {
    printf("[%d] = %d",i,data[i]);
  }
  printf("\n");
  return 0;
}

uint8_t
ble_svc_trans_get_ctrl_val(void)
{
  return notify_mode;
}


/**
 * Sets the global connection handle
 *
 * @param connection handle
 */
void
ble_svc_trans_set_conn_handle(uint16_t conn_handle)
{
    g_data_conn_handle = conn_handle;
}

/**
 * ble_svc_trans data initialization
 *
 * @param Maximum input
 */
void
ble_svc_trans_init(void)
{
  ble_svc_trans_gatt_svr_init();
}
