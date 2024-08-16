/****************************************************************************
 *   include/wireless/bluetooth/ble_app_api.h
 *
 *   Copyright (C) 2007-2017 Zglue. All rights reserved.
 *   Author: taohan <taohan@zglue.org>
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


#ifndef BLE_APP_API_H__
#define BLE_APP_API_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "ble.h"

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
 * Definations
 ****************************************************************************/
#define BLE_APP_MAX_MTU_SIZE                 (23)                         /**< MTU size, now SD support 23*/

#define BLE_APP_CENTRAL_LINK_COUNT           (0)
#define BLE_APP_PERIPHERAL_LINK_COUNT        (1)

#define BLE_APP_ADV_DEVICE_NAME              "ZG_BLE_APP"                 /**< Name of device`*/
#define BLE_APP_ADV_MANUFACTURER_NAME        "ZGLEU"                      /**< Manufacturer. Will be passed to Device Information Service. */
#define BLE_APP_ADV_INTERVAL_FAST            (48)                         /**< In units of 0.625 ms. */
#define BLE_APP_ADV_TIMEOUT_FAST             (30)                         /**< In units of seconds. */
#define BLE_APP_ADV_INTERVAL_SLOW            (1600)                       /**< In units of 0.625 ms. */
#define BLE_APP_ADV_TIMEOUT_SLOW             (0)                          /**< In units of seconds. */

#define BLE_APP_ADV_MIN_INTERVAL             (0x20)                       /**< In units of 0.625 ms, minimum is 20ms*/
#define BLE_APP_ADV_MAX_INTERVAL             (0xFFFFFF)                   /**< In units of 0.625 ms, maximum about 3 hours*/
#define BLE_APP_ADV_MAX_TIMEOUT              (0)                          /**< 0 means no timeout, The units also depends different chip implementation.*/

#define BLE_APP_ADV_MAX_UUID_NUM             (20)
#define BLE_APP_ADV_MAX_SERVICES_NUM         (5)

#define BLE_APP_ADV_FP_ANY                   (0x00)
#define BLE_APP_ADV_FP_FILTER_SCANREQ        (0x01)
#define BLE_APP_ADV_FP_FILTER_CONNREQ        (0x02)
#define BLE_APP_ADV_FP_FILTER_BOTH           (0x03)
#define BLE_APP_ADV_PF_MAX                   BLE_APP_ADV_FP_FILTER_BOTH

#define BLE_APP_ADV_TYPE_IND                 (0x00)
#define BLE_APP_ADV_TYPE_DIRECT_IND          (0x01)
#define BLE_APP_ADV_TYPE_SCAN_IND            (0x02)
#define BLE_APP_ADV_TYPE_NONCONN_IND         (0x03)
#define BLE_APP_ADV_TYPE_MAX                 BLE_APP_ADV_TYPE_NONCONN_IND

#define BLE_APP_CONN_MIN_INTERVAL            (0x06)                       /**< Units of 1.25ms, minimum is 7.5ms */
#define BLE_APP_CONN_MAX_INTERVAL            (0xC80)                      /**< Units of 1.25ms, maximum is 4s */
#define BLE_APP_CONN_MIN_SLAVE_LATENCY       (0)                          /**< Units of Interval, minimum is 0*/
#define BLE_APP_CONN_MAX_SLAVE_LATENCY       (500)                        /**< Units of Interval, 0 to Min[((connSupervisionTimeout / (connInterval*2)) - 1),500] */
#define BLE_APP_CONN_MIN_SUP_TIMEOUT         (0x10)                       /**< Units of 10ms, minimum is MIN[100ms,(1 + connSlaveLatency) * connInterval * 2]*/
#define BLE_APP_CONN_MAX_SUP_TIMEOUT         (0xC80)                      /**< Units of 10ms, maximum is 32s */

#define BLE_APP_SCAN_MIN_INTERVAL             (4)                         /* (4~16384)(2.5ms~10240ms) units of 625us*/
#define BLE_APP_SCAN_MAX_INTERVAL             (0x3FDC)                    /* (4~16384)(2.5ms~10240ms) units of 625us*/
#define BLE_APP_SCAN_GEN_INTERVAL             (800)                       /* 500ms*/
#define BLE_APP_SCAN_MIN_WINDOW               (4)                         /* (4~16384)(2.5ms~10240ms) units of 625us*/
#define BLE_APP_SCAN_MAX_WINDOW               (0x3FDC)                    /* (4~16384)(2.5ms~10240ms) units of 625us*/
#define BLE_APP_SCAN_GEN_WINDOW               (160)                       /* 100ms*/

#define BLE_APP_ADDR_TYPE_PUBLIC                        (0x00)
#define BLE_APP_ADDR_TYPE_RANDOM_STATIC                 (0x01)
#define BLE_APP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE     (0x02)
#define BLE_APP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE (0x03)

#define BLE_APP_ADDR_TYPE_RANDOM_STATIC_MSB_BIT         (0xC0)

#define BLE_APP_ADDR_LEN                     (6)

/* BLE_ATT_MTU_MAX is 527， it can be adjusted.*/
#define BLE_APP_MAX_MTU_LEN                  (527)


/* We need raw data get to 512, header will take several bytes.*/
#define BLE_APP_MAX_DATA_LEN                 (520)

#define BLE_APP_COMMAND_LEN                  (2)

#define BLE_APP_INVALID_CONN_HANDLE          (0xff)

#define BLE_APP_IS_CONNECTED(handle)         (handle != BLE_APP_INVALID_CONN_HANDLE)

#define BLE_APP_INVALID_FD                   (-1)

#define BLE_APP_MAJOR_VER                    (1)
#define BLE_APP_MINOR_VER                    (0)
#define BLE_APP_PATCH_VER                    (0)

#define BLE_APP_EVT_CB_HANDLER_MAX_NUM       (32)

#define BLE_APP_MFG_Z_COMPANY_ID             (0x47)
#define BLE_APP_MFG_VOICE_ID                 (0x47a1)

#define BLE_APP_EVET_STATUS_IDLE             (0)

#define BLE_APP_GATT_HANDLE_START            (0x0001)
#define BLE_APP_GATT_HANDLE_END              (0xFFFF)

#define BLE_APP_GATT_GET_ALL_CHAR_DEAY       (1)                       /* Get all CHARs will take about lees than 1s.*/

#define APP_LOG_ALERT(...)                   _alert(__VA_ARGS__)
#define APP_LOG_ERROR(...)                   _err(__VA_ARGS__)
#define APP_LOG_WARNING(...)                 _warn(__VA_ARGS__)
#define APP_LOG_INFO(...)                    _info(__VA_ARGS__)


/****************************************************************************
 * Structs
 ****************************************************************************/

typedef void (* app_read)(uint8_t *data, uint16_t *len);
typedef void (* app_write)(uint8_t *data, uint16_t len);
typedef void (* app_ntf_ind)(uint8_t *data, uint16_t len);
typedef void (* app_set_conn_handle)(uint16_t handle);
typedef void (* app_reset)(void);
typedef void (* app_disconn)(void);

typedef struct ble_app_handler
{
  app_read              read;
  app_write             write;
  app_ntf_ind           notify;
  app_ntf_ind           indicate;
  app_set_conn_handle   set_conn_handle;
  app_reset             reset;
  app_disconn           disconn;
} ble_app_handler;

typedef void (* ble_app_ev_handler)(void);
typedef struct ble_app_event_type
{
  uint8_t event_id;
  uint8_t event_status;
  struct ble_npl_event   event_handle;
} ble_app_event_type;

/**@brief User call back event handler. */
typedef void (* ble_evt_cb_handler)(void *p_context);

/**@brief BLE profile call back structure. */
typedef struct
{
  ble_evt_cb_handler  handler[BLE_APP_EVT_CB_HANDLER_MAX_NUM];   /**< User handler list. */
  uint8_t              handler_cnt;   /**< Handler count. */
} ble_evt_cb_list_t;

/**@brief GAP advertising parameters. */
/**@brief Bluetooth Low Energy address. */
typedef struct
{
  uint8_t addr_type;
  uint8_t addr[BLE_APP_ADDR_LEN];   /**< 48-bit address, LSB format. */
} ble_app_addr_t;

typedef struct
{
  uint8_t               type;                 /**< Advertising Types. */
  ble_app_addr_t const  *p_peer_addr;         /**< Address of a known peer. */
  uint8_t               fp;                   /**< Filter Policy, see @ref BLE_GAP_ADV_FILTER_POLICIES. */
  uint16_t
  interval;             /**< Advertising interval between 0x0020 and 0x4000 in 0.625 ms units (20ms to 10.24s). */
  uint16_t
  timeout;              /**< Advertising timeout between 0x0001 and 0x3FFF in seconds, 0x0000 disables timeout. */
  uint8_t               channel_mask;         /**< Advertising channel mask. */
} ble_adv_params_t;

/**@brief Advertising data name type.  */
typedef enum
{
  BLE_ADV_DATA_NO_NAME,
  BLE_ADV_DATA_SHORT_NAME,
  BLE_ADV_DATA_FULL_NAME
} ble_app_advdata_name_type_t;

/* Generic UUID type*/
typedef struct
{
  uint8_t   type;
  uint16_t  value;
} ble_uuid_16_t;

typedef struct
{
  uint8_t   type;
  uint32_t  value;
} ble_uuid_32_t;

typedef struct
{
  uint8_t  type;
  uint8_t  value[16];
} ble_uuid_128_t;

typedef struct
{
  uint8_t        uuid_cnt;
  ble_uuid_16_t   *p_uuids;
} ble_uuid16_list_t;

typedef struct
{
  uint8_t        uuid_cnt;
  ble_uuid_32_t   *p_uuids;
} ble_uuid32_list_t;

typedef struct
{
  uint8_t        uuid_cnt;
  ble_uuid_128_t   *p_uuids;
} ble_uuid128_list_t;


/**@brief Connection interval range structure. */
typedef struct
{
  uint16_t  min_conn_interval;        /**< In units of 1.25 ms(6~3200). */
  uint16_t  max_conn_interval;        /**< In units of 1.25 ms(6~3200). 0xFFFF means no specific maximum. */
} ble_app_conn_interval_t;

/**@brief Service data structure. */
typedef struct
{
  uint16_t  uuid;                      /**< Service/Company UUID. */
  uint16_t  size;                      /**< Number of array entries. */
  uint8_t   *p_data;                   /**< Pointer to array entries. */
} ble_app_advdata_data_t;

/**@brief Advertising data structure. */
typedef struct
{
  ble_app_advdata_name_type_t   name_type;                           /**< Type of device name. */
  uint8_t                       short_name_len;                      /**< Length of short device name. */
  bool                          include_appearance;                  /**< Determines if Appearance shall be included. */
  uint16_t                      appearance;                          /**< ID of the appearance. */
  uint8_t                       flags;                               /**< Advertising data Flags field. */
  uint8_t                       tx_power_level;                      /**< TX Power Level field. */
  ble_uuid16_list_t             uuid16_list;                         /**< List of 16 UUIDs . */
  ble_uuid32_list_t             uuid32_list;                         /**< List of 32 UUIDs . */
  ble_uuid128_list_t            uuid128_list;                        /**< List of 128 UUIDs. */
  ble_app_conn_interval_t       slave_conn_int;                      /**< Slave Connection Interval Range. */
  ble_app_advdata_data_t        manuf_specific_data;                 /**< Manufacturer specific data. */
  ble_app_advdata_data_t        *p_service_data_array;               /**< Array of Service data structures. */
  uint8_t                       service_data_count;                  /**< Number of Service data structures. */
} ble_app_advdata_t;

/**@brief Peripheral preferred connection parameters. */
typedef struct
{
  uint16_t min_conn_interval;         /**< Minimum Connection Interval in 1.25 ms units.*/
  uint16_t max_conn_interval;         /**< Maximum Connection Interval in 1.25 ms units.*/
  uint16_t slave_latency;             /**< Slave Latency in number of connection events.*/
  uint16_t conn_sup_timeout;          /**< Connection Supervision Timeout in 10 ms unit.*/
} ble_app_ppcp_t;

/**@brief Peripheral preferred connection parameters. */
typedef struct
{
  uint8_t major_version;         /*APP version big change from previous, will increased this number.*/
  uint8_t minor_version;         /*APP update or have new API, will increased this number.*/
  uint8_t patch_version;         /*APP fix bugs or refinment, will increased this number.*/
} ble_app_version;
/****************************************************************************
 * Functions
 ****************************************************************************/
/****************************************************************************
 * Name: ble_app_get_version
 *
 * Description:
 *   Get BLE app version.
 *
 * Input Parameters:
 *   ver   version struct, will store version details.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
void ble_app_get_version(ble_app_version *ver);

/****************************************************************************
 * Name: ble_profile_handler_register
 *
 * Description:
 *   BLE app user callback function register.
 *
 * Input Parameters:
 *   func   Function will be registed.
 *
 * Returned Value:
 *   0: Successful
 *
 ****************************************************************************/
uint32_t ble_evt_cb_handler_register(ble_evt_cb_handler func);

/****************************************************************************
 * Name: ble_app_set_address
 *
 * Description:
 *   Set BD address name.
 *
 * Input Parameters:
 * addr   address type and value.
 *
 * Returned Value:
 *   0: Successful
 *
 ****************************************************************************/
uint32_t ble_app_set_address(const ble_app_addr_t *addr);

/****************************************************************************
 * Name: ble_app_set_device_name
 *
 * Description:
 *   Set device name.
 *
 * Input Parameters:
 * data   device name.
 * len    length of the name.
 *
 * Returned Value:
 *   0: Successful
 *
 ****************************************************************************/
uint32_t ble_app_set_device_name(const uint8_t *data, uint16_t len);

/****************************************************************************
 * Name: ble_app_set_ppcp
 *
 * Description:
 *   Set peripheral preferred connection parameters.
 *
 * Input Parameters:
 * ppcp   peripheral preferred connection parameters.
 *
 * Returned Value:
 *   0: Successful
 *
 ****************************************************************************/
uint32_t ble_app_set_ppcp(ble_app_ppcp_t *ppcp);

/****************************************************************************
 * Name: ble_adv_init
 *
 * Description:
 *   BLE initialize adverting data.
 *
 * Input Parameters:
 * p_advdata   adverting data.
 * p_advdata   scan response data.
 *
 * Returned Value:
 *   0: Successful
 *
 ****************************************************************************/
uint32_t ble_adv_set_data(const ble_app_advdata_t *p_advdata, const ble_app_advdata_t *p_srdata);

/****************************************************************************
 * Name: ble_adv_start
 *
 * Description:
 *   BLE adverting start.
 *
 * Input Parameters:
 * p_adv_para adverting parameter.
 *
 * Returned Value:
 *   0: Successful
 *
 ****************************************************************************/
uint32_t ble_adv_start(ble_adv_params_t *p_adv_para);

/****************************************************************************
 * Name: ble_adv_stop
 *
 * Description:
 *   BLE adverting stop.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   0: Successful
 *
 ****************************************************************************/
uint32_t ble_adv_stop(void);

/****************************************************************************
 * Name: ble_enable
 *
 * Description:
 *   BLE moudle enable and base initialization.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   0: Successful
 *
 ****************************************************************************/
uint32_t ble_init(void);

extern  int ble_hs_adv_find_field(uint8_t type,
                                  const uint8_t *data,
                                  uint8_t length,
                                  const struct ble_hs_adv_field **out);

extern int ble_hs_hci_util_set_data_len(uint16_t conn_handle,
                                        uint16_t tx_octets,
                                        uint16_t tx_time);

#ifdef __cplusplus
}
#endif

#endif // BLE_GAP_API_H__

