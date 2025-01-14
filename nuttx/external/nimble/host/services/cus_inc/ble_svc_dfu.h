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

#ifndef _BLE_SVC_DFU_H_
#define _BLE_SVC_DFU_H_

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/*
 *1. Central read peripheral local info                               <--read
 *|-byte0-------------byte59-|-byte60--byte63-|-byte64-byte65-|
 *|-ver(20)-model(20)-sn(20)-|-total img size-|CRC verfication|
 *
 *
 *2. Central write First package to peripheral.
 *|-byte0-byte1-|-byte2-byte3-|-byte4-byte5-|-byte5---byte6-|
 *|Type(start)  |block   count|block    size|CRC verfication|        -->send
 *   Peripheral ack to central
 *|-byte0-byte1-|-byte2-byte3-|-byte4-byte5-|-byte5---byte6-|
 *|Type(start)  |block   count|block    size|CRC verfication|        <--ack
 *
 *
 *3. Central write raw data package to peripheral.
 *|-byte0---byte1-|-byte2-byte3-|--n bytes--|---n+4---n+5---|
 *|Type (updating)|block  number|raw    data|CRC verfication|         -->send
 *   Peripheral ack to central
 *|-byte0---byte1-|-byte1-byte2-|-byte3---byte4-|
 *|Type (updating)|block  number|CRC verfication|                     <--ack
 *
 *
 *4. Central write end to peripheral.
 *|-byte0-byte1-|-byte1---byte2-|
 *|Type(end)    |CRC verfication|                                         -->send
 *   Peripheral ack to central
 *|-byte0-byte1-|-byte1---byte2-|
 *|Type(end)    |CRC verfication|                                         <--ack
 */
#define BLE_SVC_DFU_PERIPH_INFO_LEN         (sizeof(ble_dfu_periph_info_type))

#define BLE_SVC_DFU_START_PKG_LEN           (sizeof(ble_dfu_start_pkg_type))

#define BLE_SVC_DFU_END_PKG_LEN             (sizeof(ble_dfu_end_pkg_type))

#define BLE_SVC_DFU_ACK_PKG_LEN             (sizeof(ble_dfu_pkg_ack_type))

/* BLE_ATT_MTU_MAX is 527， it can be adjusted.*/
#define BLE_SVC_DFU_MAX_DATA_LEN            (512)

#define BLE_SVC_DFU_MIN_DATA_LEN            (4)


#define BLE_SVC_DFU_MFG_DATA_LEN            (4)

#define BLE_SVC_DFU_MFG_COMPANY_ID          (0xABCD)

#define BLE_SVC_DFU_MFG_OTA_ID              (0x1234)

/****************************************************************************
 * Private Types
 ****************************************************************************/
typedef struct ble_dfu_periph_info_type {
  uint16_t flash_size;
  uint16_t img_version;
  uint16_t crc;
} ble_dfu_periph_info_type;

typedef struct ble_dfu_start_pkg_type {
  uint16_t type;
  uint16_t block_cnt;
  uint16_t block_size;
  uint16_t crc;
} ble_dfu_start_pkg_type;

typedef struct ble_dfu_updating_pkg_type {
  uint16_t type;
  uint16_t block_num;
  uint16_t block_len;
} ble_dfu_updating_pkg_type;

typedef struct ble_dfu_end_pkg_type {
  uint16_t type;
  uint16_t crc;
} ble_dfu_end_pkg_type;

typedef struct ble_dfu_pkg_ack_type {
  uint16_t ack_type;
  uint16_t ack_info;
  uint16_t error;
  uint16_t crc;
} ble_dfu_pkg_ack_type;

typedef struct ble_dfu_process_ctl_type {
  uint16_t status;
  uint16_t block_cnt;
  uint16_t block_size;
  uint16_t cur_block;
} ble_dfu_process_ctl_type;

typedef void (* dfu_write)(uint8_t *data, uint16_t len);
typedef void (* dfu_read)(uint8_t *data, uint16_t *len);
typedef void (* dfu_reset)(void);

typedef struct ble_dfu_access_handler {
  dfu_read read;
  dfu_write write;
  dfu_reset reset;
} ble_dfu_access_handler;

typedef enum ble_dfu_pkg_type
{
  BLE_SVC_DFU_PKG_TYPE_INIT = 0,
  BLE_SVC_DFU_PKG_TYPE_START,
  BLE_SVC_DFU_PKG_TYPE_UPDATING,
  BLE_SVC_DFU_PKG_TYPE_END,
  BLE_SVC_DFU_PKG_TYPE_MAX
} ble_dfu_pkg_type;

enum
{
  BLE_SVC_DFU_ERR_NONE = 0,
  BLE_SVC_DFU_ERR_CRC,
  BLE_SVC_DFU_ERR_STATUS,
  BLE_SVC_DFU_ERR_BAD_DATA,
  BLE_SVC_DFU_ERR_WRONG_BLOCK_NO,
  BLE_SVC_DFU_ERR_WRONG_BLOCK_SIZE,
  BLE_SVC_DFU_ERR_MAX
};

/****************************************************************************
 * Public Data
 ****************************************************************************/
/* UUID generated by http://www.guidgenerator.com {c9b6b5bd-4540-43f4-9715-c0743af859ca}*/

/* {c9b60001-4540-43f4-9715-c0743af859ca} */
extern const ble_uuid128_t gatt_svr_svc_dfu_uuid;
/* {c9b60002-4540-43f4-9715-c0743af859ca} */
extern const ble_uuid128_t gatt_svr_chr_dfu_read_uuid;
/* {c9b60003-4540-43f4-9715-c0743af859ca} */
extern const ble_uuid128_t gatt_svr_chr_dfu_write_uuid;
/* {c9b60004-4540-43f4-9715-c0743af859ca} */
extern const ble_uuid128_t gatt_svr_chr_dfu_ack_uuid;

/****************************************************************************
 * Public Functions
 ****************************************************************************/
void
ble_svc_dfu_reset(void);
void
ble_svc_dfu_set_conn_handle(uint16_t conn_handle);
void
ble_svc_dfu_init(ble_dfu_access_handler * handler);
void
ble_svc_dfu_disconnect(void);

#ifdef __cplusplus
}
#endif

#endif /* _BLE_SVC_DFU_H_ */
