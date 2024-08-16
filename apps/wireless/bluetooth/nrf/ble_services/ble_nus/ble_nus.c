/**
 * Copyright (c) 2012 - 2017, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_NUS)
#include "ble.h"
#include "ble_nus.h"
#include "ble_srv_common.h"
#include "nrf_log.h"
#include "app_error.h"
#include <utils/app_timer.h>
#include "app_uart.h"
#include <errno.h>

#include <wireless/bluetooth/bt_mq.h>
#include <wireless/bluetooth/ble_app_api.h>
/****************************************************************************
 * Macro defination
 ****************************************************************************/
#define BLE_NUS_CHAR_RX_NAME  "UART_OUTPUT";
#define BLE_NUS_CHAR_TX_NAME  "UART_INPUT";

//#define FUTURE_DEBUG

/****************************************************************************
 * Local variables
 ****************************************************************************/
static ble_nus_t m_nus;

//APP_TIMER_DEF(m_data_transfer_timer_id);

/****************************************************************************
 * Functions
 ****************************************************************************/

/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the SoftDevice.
 *
 * @param[in] p_nus     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_connect(ble_nus_t *p_nus, ble_evt_t const *p_ble_evt)
{
  ble_uart_evt_t uart_evt;

  p_nus->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

  uart_evt.type = BLE_UART_CONNECTED;
  p_nus->data_handler(&uart_evt);
}

/**@brief Function for handling the @ref BLE_GAP_EVT_DISCONNECTED event from the SoftDevice.
 *
 * @param[in] p_nus     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_disconnect(ble_nus_t *p_nus, ble_evt_t const *p_ble_evt)
{
  ble_uart_evt_t uart_evt;

  UNUSED_PARAMETER(p_ble_evt);
  p_nus->conn_handle = BLE_CONN_HANDLE_INVALID;

  uart_evt.type = BLE_UART_DISCONNECTED;
  p_nus->data_handler(&uart_evt);
}


/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the SoftDevice.
 *
 * @param[in] p_nus     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(ble_nus_t *p_nus, ble_evt_t const *p_ble_evt)
{
  ble_uart_evt_t uart_evt;
  uart_evt.type = BLE_UART_INVALID;

  ble_gatts_evt_write_t const *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

  if (p_evt_write->handle == p_nus->tx_handles.cccd_handle)
    {
      if (p_evt_write->len != sizeof(uint16_t))
        {
          APP_LOG_WARNING("CCCD is invalid len: %x", (uint16_t)p_evt_write->len);
        }
      else
        {
          uint16_t cccd_value = uint16_decode(p_evt_write->data);
          if (cccd_value == BLE_GATT_HVX_NOTIFICATION)
            {
              p_nus->is_notification_enabled = true;
              uart_evt.type = BLE_UART_NTF_STARTED;
            }
          else if (cccd_value == 0)
            {
              p_nus->is_notification_enabled = false;
              uart_evt.type = BLE_UART_NTF_STOPPED;
            }
          else
            {
              APP_LOG_WARNING("CCCD is invalid val: %x", cccd_value);
            }
        }
    }
  else if (p_evt_write->handle == p_nus->rx_handles.value_handle)
    {
      uart_evt.data = (uint8_t *)p_evt_write->data;
      uart_evt.len = p_evt_write->len;
      uart_evt.type = BLE_UART_RX_DATA;
    }
  else
    {
      // Do Nothing. This event is not relevant for this service.
      uart_evt.type = BLE_UART_INVALID;
    }
  p_nus->data_handler(&uart_evt);
}


/**@brief Function for adding TX characteristic.
 *
 * @param[in] p_nus       Nordic UART Service structure.
 * @param[in] p_nus_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t tx_char_add(ble_nus_t *p_nus, ble_nus_init_t const *p_nus_init)
{
  /**@snippet [Adding proprietary characteristic to the SoftDevice] */
  ble_gatts_char_md_t char_md;
  ble_gatts_attr_md_t cccd_md;
  ble_gatts_attr_t    attr_char_value;
  ble_uuid_t          ble_uuid;
  ble_gatts_attr_md_t attr_md;
  uint8_t char_name[] = BLE_NUS_CHAR_TX_NAME;

  memset(&cccd_md, 0, sizeof(cccd_md));

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

  cccd_md.vloc = BLE_GATTS_VLOC_STACK;

  memset(&char_md, 0, sizeof(char_md));

  char_md.char_props.notify = 1;
  char_md.p_char_user_desc  = NULL;
  char_md.p_char_pf         = NULL;
  char_md.p_user_desc_md    = NULL;
  char_md.p_cccd_md         = &cccd_md;
  char_md.p_sccd_md         = NULL;
  char_md.p_char_user_desc  = char_name;
  char_md.char_user_desc_size  = sizeof(char_name);
  char_md.char_user_desc_max_size  = char_md.char_user_desc_size;

  ble_uuid.type = p_nus->uuid_type;
  ble_uuid.uuid = BLE_UUID_NUS_TX_CHARACTERISTIC;

  memset(&attr_md, 0, sizeof(attr_md));

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

  attr_md.vloc    = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth = 0;
  attr_md.wr_auth = 0;
  attr_md.vlen    = 1;

  memset(&attr_char_value, 0, sizeof(attr_char_value));

  attr_char_value.p_uuid    = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len  = sizeof(uint8_t);
  attr_char_value.init_offs = 0;
  attr_char_value.max_len   = BLE_NUS_MAX_RX_CHAR_LEN;

  return sd_ble_gatts_characteristic_add(p_nus->service_handle,
                                         &char_md,
                                         &attr_char_value,
                                         &p_nus->tx_handles);
  /**@snippet [Adding proprietary characteristic to the SoftDevice] */
}


/**@brief Function for adding RX characteristic.
 *
 * @param[in] p_nus       Nordic UART Service structure.
 * @param[in] p_nus_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t rx_char_add(ble_nus_t *p_nus, const ble_nus_init_t *p_nus_init)
{
  ble_gatts_char_md_t char_md;
  ble_gatts_attr_t    attr_char_value;
  ble_uuid_t          ble_uuid;
  ble_gatts_attr_md_t attr_md;
  uint8_t char_name[] = BLE_NUS_CHAR_RX_NAME;

  memset(&char_md, 0, sizeof(char_md));

  char_md.char_props.write         = 1;
  char_md.char_props.write_wo_resp = 1;
  char_md.p_char_pf                = NULL;
  char_md.p_user_desc_md           = NULL;
  char_md.p_cccd_md                = NULL;
  char_md.p_sccd_md                = NULL;
  char_md.p_char_user_desc  = char_name;
  char_md.char_user_desc_size  = sizeof(char_name);
  char_md.char_user_desc_max_size  = char_md.char_user_desc_size;

  ble_uuid.type = p_nus->uuid_type;
  ble_uuid.uuid = BLE_UUID_NUS_RX_CHARACTERISTIC;

  memset(&attr_md, 0, sizeof(attr_md));

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

  attr_md.vloc    = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth = 0;
  attr_md.wr_auth = 0;
  attr_md.vlen    = 1;

  memset(&attr_char_value, 0, sizeof(attr_char_value));

  attr_char_value.p_uuid    = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len  = 1;
  attr_char_value.init_offs = 0;
  attr_char_value.max_len   = BLE_NUS_MAX_RX_CHAR_LEN;
  return sd_ble_gatts_characteristic_add(p_nus->service_handle,
                                         &char_md,
                                         &attr_char_value,
                                         &p_nus->rx_handles);
}


void ble_nus_on_ble_evt(void *p_context)
{

  ble_evt_t *p_ble_evt = (ble_evt_t *)p_context;
  ble_nus_t *p_nus = &m_nus;

  if ((p_nus == NULL) || (p_ble_evt == NULL))
    {
      return;
    }

  switch (p_ble_evt->header.evt_id)
    {
      case BLE_GAP_EVT_CONNECTED:
        on_connect(p_nus, p_ble_evt);
        break;

      case BLE_GAP_EVT_DISCONNECTED:
        on_disconnect(p_nus, p_ble_evt);
        break;

      case BLE_GATTS_EVT_WRITE:
        on_write(p_nus, p_ble_evt);
        break;

      case BLE_GATTS_EVT_HVN_TX_COMPLETE:
        {
          /*notify with empty data that some tx was completed.*/
          /*Need do nothing...*/
          printf("\nTX complete in BLE_NUS...\n");
          break;
        }
      default:
        // No implementation needed.
        break;
    }
}

#ifdef FUTURE_DEBUG
static void nus_data_transfer_timer_handler(void *p_context)
{
  ble_gatts_hvx_params_t hvx_params;
  uint16_t p_len = 0;
  uint8_t p_data[BLE_NUS_MAX_RX_CHAR_LEN] = {0};

  ble_nus_t *p_nus = &m_nus;

  if (NULL != m_uart_file)
    {
      p_len = fread(p_data, 1, BLE_NUS_MAX_RX_CHAR_LEN, m_uart_file);
    }
  else
    {
      NRF_LOG_INFO("\nUART file open [%s] failed [%d]...\n", UART_NAME, __LINE__);
    }

  /*No data updated...*/
  if (0 == p_len)
    {
      return;
    }

  if ((p_nus->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_nus->is_notification_enabled))
    {

      NRF_LOG_WARNING("Notification is not enabled...HANDLE[%x]--NTF_FLAG[%x]\n", p_nus->conn_handle, p_nus->is_notification_enabled);
      return;
    }

  memset(&hvx_params, 0, sizeof(hvx_params));
  hvx_params.handle = p_nus->tx_handles.value_handle;
  hvx_params.p_data = p_data;
  hvx_params.p_len  = &p_len;
  hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

  sd_ble_gatts_hvx(p_nus->conn_handle, &hvx_params);
  if (*(hvx_params.p_len) != p_len)
    {
      NRF_LOG_WARNING("\nNotification is not complete all data...\n");
    }
}
#endif

uint32_t ble_nus_init(ble_nus_t *p_nus, ble_nus_init_t const *p_nus_init)
{
  uint32_t      err_code;
  ble_uuid_t    ble_uuid;
  ble_uuid128_t nus_base_uuid = NUS_BASE_UUID;

  VERIFY_PARAM_NOT_NULL(p_nus);
  VERIFY_PARAM_NOT_NULL(p_nus_init);

  // Initialize the service structure.
  p_nus->conn_handle             = BLE_CONN_HANDLE_INVALID;
  p_nus->data_handler            = p_nus_init->data_handler;
  p_nus->is_notification_enabled = false;

  /**@snippet [Adding proprietary Service to the SoftDevice] */
  // Add a custom base UUID.
  err_code = sd_ble_uuid_vs_add(&nus_base_uuid, &p_nus->uuid_type);
  VERIFY_SUCCESS(err_code);

  ble_uuid.type = p_nus->uuid_type;
  ble_uuid.uuid = BLE_UUID_NUS_SERVICE;

  // Add the service.
  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                      &ble_uuid,
                                      &p_nus->service_handle);
  /**@snippet [Adding proprietary Service to the SoftDevice] */
  VERIFY_SUCCESS(err_code);

  // Add the RX Characteristic.
  err_code = rx_char_add(p_nus, p_nus_init);
  VERIFY_SUCCESS(err_code);

  // Add the TX Characteristic.
  err_code = tx_char_add(p_nus, p_nus_init);
  VERIFY_SUCCESS(err_code);

  //The timer moudule is not ready....
  //app_timer_create(&m_data_transfer_timer_id, APP_TIMER_MODE_SINGLE_SHOT, nus_data_transfer_timer_handler);  //HHTT

  return NRF_SUCCESS;
}


uint32_t ble_uart_data_send(uint8_t *data, uint16_t len)
{

  uint32_t ret = 0;
  ble_gatts_hvx_params_t hvx_params;
  /*This parameter will be changed in sd_ble_gatts_hvx, so we re-define it.*/
  uint16_t send_len;

  if ((m_nus.conn_handle == BLE_CONN_HANDLE_INVALID) || (!m_nus.is_notification_enabled))
    {
      return NRF_ERROR_INVALID_STATE;
    }

  if (len > BLE_NUS_MAX_RX_CHAR_LEN)
    {
      return NRF_ERROR_INVALID_PARAM;
    }

  send_len = len;

  memset(&hvx_params, 0, sizeof(hvx_params));

  hvx_params.handle = m_nus.tx_handles.value_handle;
  hvx_params.p_data = data;
  hvx_params.p_len  = &send_len;
  hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

  ret = sd_ble_gatts_hvx(m_nus.conn_handle, &hvx_params);

  return ret;
}

uint32_t ble_srv_uart_init(ble_uart_init_t ble_uart_init)
{
  uint32_t       err_code;
  ble_nus_init_t nus_init;

  memset(&nus_init, 0, sizeof(nus_init));
  nus_init.data_handler = ble_uart_init.uart_data_handler;
  err_code = ble_nus_init(&m_nus, &nus_init);
  APP_ERROR_CHECK(err_code);

  err_code = ble_evt_cb_handler_register(ble_nus_on_ble_evt);
  APP_ERROR_CHECK(err_code);
  return err_code;
}

#endif // NRF_MODULE_ENABLED(BLE_NUS)
