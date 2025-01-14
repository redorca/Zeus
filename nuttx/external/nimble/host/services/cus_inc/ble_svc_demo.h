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

#ifndef _BLE_SVC_DEMO_H_
#define _BLE_SVC_DEMO_H_

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/


/* BLE_ATT_MTU_MAX is 527， it can be adjusted.*/
#define BLE_SVC_DEMO_MAX_MTU                (512)

#define BLE_SVC_DEMO_INFO_LEN               (100)

#define BLE_SVC_DEMO_MAX_DATA_LEN           (32)

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef void (* demo_write)(uint8_t *data, uint16_t len);
typedef void (* demo_read)(uint8_t *data, uint16_t *len);
typedef void (* demo_reset)(void);

typedef struct ble_demo_handler {
  demo_read read;
  demo_write write;
  demo_reset reset;
} ble_demo_handler;


/****************************************************************************
 * Public Data
 ****************************************************************************/
/* UUID generated by http://www.guidgenerator.com {3eb20001-8977-4323-8912-7074e88f9607}*/

/* {3eb20001-8977-4323-8912-7074e88f9607} */
extern const ble_uuid128_t gatt_svr_svc_demo_uuid;
/* {3eb20002-8977-4323-8912-7074e88f9607} */
extern const ble_uuid128_t gatt_svr_chr_demo_read_uuid;
/* {3eb20003-8977-4323-8912-7074e88f9607} */
extern const ble_uuid128_t gatt_svr_chr_demo_write_uuid;
/* {3eb20004-8977-4323-8912-7074e88f9607} */
extern const ble_uuid128_t gatt_svr_chr_demo_ack_uuid;

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/**
 * Notify the ble data to remote device.
 *
 * @param data: the data pointer
 *
 * @param data: the data length
 *
 */
void
ble_svc_demo_send_data(uint8_t * data, uint32_t len);

/**
 * Sets the global connection handle
 *
 * @param conn_handle: connection handle
 */
void
ble_svc_demo_set_conn_handle(uint16_t conn_handle);

/**
 * ble_svc_demo sevices initialization
 *
 * @param handler: Application callback handler
 *
 */
void
ble_svc_demo_init(ble_demo_handler * handler);

#ifdef __cplusplus
}
#endif

#endif /* _BLE_SVC_DFU_H_ */
