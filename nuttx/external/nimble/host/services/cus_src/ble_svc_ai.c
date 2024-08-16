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
#include "os/endian.h"

enum
{
  BEL_AI_DIABLE_ALL = 0,
  BEL_AI_ENABLE_AI,
  BEL_AI_ENABLE_AI_GS,
  BEL_AI_FUN_MAX
};

enum
{
  BEL_AI_STATUS_INTI = 0,
  BEL_AI_STATUS_RUNING,
  BEL_AI_STATUS_FACE_DETECTED,
  BEL_AI_STATUS_MAX
};

/* ble ai attr read handle */
static uint8_t g_ai_sensor_op;

/* ble ai attr write handle */
static uint16_t g_ble_svc_ai_attr_write_handle;

static uint16_t g_ble_svc_ai_attr_read_handle;

static uint16_t g_svc_ai_conn_handle;

static uint8_t ble_svc_ai_status;

static struct ble_npl_event ble_svc_ai_ev_send_ai_status;

/**
 * The vendor specific "ble_svc_ai" service consists of one write no-rsp characteristic
 * and one notification only read charateristic
 *     o "write no-rsp": a single-byte characteristic that can be written only
 *       over a non-encrypted connection
 *     o "read": a single-byte characteristic that can always be read only via
 *       notifications
 */

/* {6E400001-B5A3-F393-E0A9-E50E24DCCA9E} */
const ble_uuid128_t gatt_svr_svc_ai_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);

/* {6E400002-B5A3-F393-E0A9-E50E24DCCA9E} */
const ble_uuid128_t gatt_svr_chr_ai_read_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);

/* {6E400003-B5A3-F393-E0A9-E50E24DCCA9E} */
const ble_uuid128_t gatt_svr_chr_ai_write_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

static int
gatt_svr_chr_access_ai_read(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg);

static int
gatt_svr_chr_access_ai_write(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_svc_def gatt_ai_svr_svcs[] = {
    {
        /* Service: uart */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_ai_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) { 
        {
            .uuid = &gatt_svr_chr_ai_read_uuid.u,
            .val_handle = &g_ble_svc_ai_attr_read_handle,
            .access_cb = gatt_svr_chr_access_ai_read,
            .flags = BLE_GATT_CHR_F_NOTIFY,
        }, 
        {
            /* Characteristic: Write */
            .uuid = &gatt_svr_chr_ai_write_uuid.u,
            .val_handle = &g_ble_svc_ai_attr_write_handle,
            .access_cb = gatt_svr_chr_access_ai_write,
            .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
        }, {
            0, /* No more characteristics in this service */
        } },
    },
    {
        0, /* No more services */
    },
};

static int
gatt_svr_chr_access_ai_read(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
  return 0;
}
                               
static int
gatt_svr_chr_access_ai_write(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    struct os_mbuf *om = ctxt->om;
    BLE_HS_LOG(INFO,"\nAccess op = %d\n", ctxt->op);

    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
              while(om) {
                  BLE_HS_LOG(WARN, "LEN=%d_[",om->om_len);
                  for(int i = 0; i < om->om_len; i++)
                  {
                    BLE_HS_LOG(WARN, "%d ",om->om_data[i]);
                  }
                  if(1 == om->om_len)
                  {
                     uint8_t val;
                     val = om->om_data[0];
                     switch (val)
                       {
                       case BEL_AI_DIABLE_ALL:
                         g_ai_sensor_op = BEL_AI_DIABLE_ALL;
                         BLE_HS_LOG(WARN, "\nDisable all.\n");
                         break;
                       case BEL_AI_ENABLE_AI:
                         g_ai_sensor_op = BEL_AI_ENABLE_AI;
                         BLE_HS_LOG(WARN, "\nEnable ai.\n");
                         break;
                       case BEL_AI_ENABLE_AI_GS:
                         g_ai_sensor_op = BEL_AI_ENABLE_AI_GS;
                         BLE_HS_LOG(WARN, "\nEnable ai and gsendor.\n");
                         break;

                       default:
                         BLE_HS_LOG(WARN, "\nNo behavior matched.\n");
                         break;
                       }
                  } else {
                    BLE_HS_LOG(WARN, "\nNo behavior matched.\n");
                  }
                  
                  om = SLIST_NEXT(om, om_next);
              }
              BLE_HS_LOG(WARN, "]\n");
              return 0;
        default:
            assert(0);
            return BLE_ATT_ERR_UNLIKELY;
    }
}

void
ble_svc_ai_send_data(uint32_t len, uint8_t * data)
{
    struct os_mbuf *om;

    om = ble_hs_mbuf_from_flat(data, len);
    if (!om) {
        return;
    }
    ble_gattc_notify_custom(g_svc_ai_conn_handle,
                            g_ble_svc_ai_attr_read_handle, om);
}

static void
ble_svc_ai_event_ai_status_notify(struct ble_npl_event *ev)
{
  ble_svc_ai_send_data(1, &ble_svc_ai_status);
}

uint8_t
ble_svc_ai_fun_status_read(void)
{
    return g_ai_sensor_op;
}

void
ble_svc_ai_status_update(uint8_t status)
{
  ble_svc_ai_status = status;
  ble_npl_eventq_put((struct ble_npl_eventq *)os_eventq_dflt_get(), &ble_svc_ai_ev_send_ai_status);
}

/**
 * Sets the global connection handle
 *
 * @param connection handle
 */
void
ble_svc_ai_set_conn_handle(uint16_t conn_handle)
{
    g_svc_ai_conn_handle = conn_handle;
}

/**
 * ble_svc_ai GATT server initialization
 *
 * @param eventq
 * @return 0 on success; non-zero on failure
 */
int
ble_svc_ai_gatt_svr_init(void)
{
    int rc;

    rc = ble_gatts_count_cfg(gatt_ai_svr_svcs);
    if (rc != 0) {
        goto err;
    }

    rc = ble_gatts_add_svcs(gatt_ai_svr_svcs);
    if (rc != 0) {
        return rc;
    }

err:
    return rc;
}

/**
 * ble_svc_ai data initialization
 *
 * @param Maximum input
 */
void
ble_svc_ai_init(void)
{
  ble_svc_ai_gatt_svr_init();
  ble_npl_event_init(&ble_svc_ai_ev_send_ai_status, ble_svc_ai_event_ai_status_notify, NULL);
  ble_svc_ai_status = BEL_AI_STATUS_INTI;  
  g_ai_sensor_op = BEL_AI_DIABLE_ALL;
}
