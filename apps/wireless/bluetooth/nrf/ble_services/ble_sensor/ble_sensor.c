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
#if NRF_MODULE_ENABLED(BLE_SENSOR)
#include "ble.h"
#include "ble_sensor.h"
#include "ble_srv_common.h"
#include "nrf_log.h"
#include "app_error.h"
#include <utils/app_timer.h>
#include "app_uart.h"

//#define FUTURE_DEBUG

/****************************************************************************
 * Global variables
 ****************************************************************************/

/**< Structure used to identify the uart service. */
static ble_sensor_t m_sensor;

//APP_TIMER_DEF(m_data_transfer_timer_id);

/****************************************************************************
 * Functions
 ****************************************************************************/

/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the SoftDevice.
 *
 * @param[in] p_sensor     Sensor Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_connect(ble_sensor_t *p_sensor, ble_evt_t const *p_ble_evt)
{
  p_sensor->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

/**@brief Function for handling the @ref BLE_GAP_EVT_DISCONNECTED event from the SoftDevice.
 *
 * @param[in] p_sensor     Sensor Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_disconnect(ble_sensor_t *p_sensor, ble_evt_t const *p_ble_evt)
{
  UNUSED_PARAMETER(p_ble_evt);
  p_sensor->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief   Function for sending a string to the peer.
 *
 * @details This function sends the input string as an RX characteristic notification to the
 *          peer.
 *
 * @param[in] p_sensor       Pointer to the Sensor Service structure.
 * @param[in] p_string    String to be sent.
 * @param[inout] p_length Pointer Length of the string. Amount of sent bytes.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
static uint32_t ble_sensor_string_send(void)
{
  //return app_timer_start(m_data_transfer_timer_id, BLE_SENSOR_DATA_NOTIFICATION_TICK, NULL);  //HHTT
  return OK;
}

/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the SoftDevice.
 *
 * @param[in] p_sensor     Sensor Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(ble_sensor_t *p_sensor, ble_evt_t const *p_ble_evt)
{
  ble_gatts_evt_write_t const *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

  if ((p_evt_write->handle == p_sensor->tx_data_handle.cccd_handle)
      && (p_evt_write->len == 2))
    {
      if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
          p_sensor->is_notification_enabled = true;
        }
      else
        {
          p_sensor->is_notification_enabled = false;
        }
    }
  else if (p_evt_write->handle == p_sensor->enable_handle.value_handle)
    {
      if ((p_evt_write->data[0] == BLE_SENSOR_ENABLE_REPORT) && (p_sensor->conn_handle != BLE_CONN_HANDLE_INVALID))
        {
          //Enable sensor data reporting
          NRF_LOG_INFO("\nStart reporting...");
          ble_sensor_string_send();
        }
      else if (p_evt_write->data[0] == BLE_SENSOR_DISABLE_REPORT)
        {
          //Disable sensor data reporting
          NRF_LOG_INFO("\nStop reporting...");
          //app_timer_stop(m_data_transfer_timer_id);  //HHTT
        }
      else
        {
          //To do something later...
        }
    }
  else
    {
      // Do Nothing. This event is not relevant for this service.
    }
}

void ble_sensor_on_ble_evt(ble_evt_t *p_ble_evt)
{
  ble_sensor_t *p_sensor;

  p_sensor = &m_sensor;

  if ((p_sensor == NULL) || (p_ble_evt == NULL))
    {
      return;
    }

  switch (p_ble_evt->header.evt_id)
    {
      case BLE_GAP_EVT_CONNECTED:
        on_connect(p_sensor, p_ble_evt);
        break;

      case BLE_GAP_EVT_DISCONNECTED:
        on_disconnect(p_sensor, p_ble_evt);
        break;

      case BLE_GATTS_EVT_WRITE:
        on_write(p_sensor, p_ble_evt);
        break;

      case BLE_GATTS_EVT_HVN_TX_COMPLETE:
        break;

      default:
        // No implementation needed.
        break;
    }
}

#ifdef FUTURE_DEBUG
static void sensor_data_transfer_timer_handler(void *p_context)
{
  ble_gatts_hvx_params_t hvx_params;
  uint16_t  *p_len = NULL;
  uint8_t  *p_data = NULL;
  ble_sensor_t *p_sensor = &m_sensor;


  printf("\nTimer...");

  if ((p_sensor->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_sensor->is_notification_enabled))
    {

      NRF_LOG_WARNING("Notification is not enabled...HANDLE[%x]--NTF_FLAG[%x]\n", p_sensor->conn_handle,
                      p_sensor->is_notification_enabled);
      return;
    }

  //@HT Get data

  ASSERT(p_data != NULL);
  ASSERT(p_len != NULL);

  if (*p_len > BLE_SENSOR_MAX_TX_DATA_LEN)
    {
      NRF_LOG_WARNING("Data length is out of range...Len = [%x]\n", *p_len);
      return;
    }


  memset(&hvx_params, 0, sizeof(hvx_params));
  hvx_params.handle = p_sensor->tx_data_handle.value_handle;
  hvx_params.p_data = p_data;
  hvx_params.p_len  = p_len;
  hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

  sd_ble_gatts_hvx(p_sensor->conn_handle, &hvx_params);
}
#endif

/**@brief Function for notify sensor data characteristic.
 *
 * @param[in] p_sensor       Sensor Service structure.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t g_sensor_tx_char_add(ble_sensor_t *p_sensor)
{
  /**@snippet [Adding proprietary characteristic to the SoftDevice] */
  ble_gatts_char_md_t char_md;
  ble_gatts_attr_md_t cccd_md;
  ble_gatts_attr_t    attr_char_value;
  ble_uuid_t          ble_uuid;
  ble_gatts_attr_md_t attr_md;
  uint8_t gsensor_data[] = "GS DATA[CNT|X|Y|Z]";

  memset(&cccd_md, 0, sizeof(cccd_md));
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
  cccd_md.vloc        = BLE_GATTS_VLOC_STACK;
  char_md.p_cccd_md   = &cccd_md;

  memset(&char_md, 0, sizeof(char_md));
  char_md.char_props.notify = 1;
  char_md.char_props.read = 1;
  char_md.p_char_user_desc  = gsensor_data;
  char_md.char_user_desc_size  = sizeof(gsensor_data);
  char_md.char_user_desc_max_size  = char_md.char_user_desc_size;

  memset(&attr_md, 0, sizeof(attr_md));
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
  attr_md.vloc    = BLE_GATTS_VLOC_STACK;

  ble_uuid.type = p_sensor->uuid_type;
  ble_uuid.uuid = BLE_UUID_SENSOR_TX_CHARACTERISTIC;

  memset(&attr_char_value, 0, sizeof(attr_char_value));
  attr_char_value.p_uuid    = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len  = BLE_SENSOR_DATA_BYTES;
  attr_char_value.max_len   = BLE_SENSOR_DATA_BYTES;

  return sd_ble_gatts_characteristic_add(p_sensor->service_handle,
                                         &char_md,
                                         &attr_char_value,
                                         &p_sensor->tx_data_handle);
}


/**@brief Function for adding RX characteristic.
 *
 * @param[in] p_sensor       Sensor Service structure.
 * @param[in] p_sensor_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t g_sensor_enable_char_add(ble_sensor_t *p_sensor)
{
  ble_gatts_char_md_t char_md;
  ble_gatts_attr_md_t attr_md;
  ble_gatts_attr_t    attr_char_value;
  ble_uuid_t          ble_uuid;
  uint8_t gsensor_enable[] = "GS[1-enable|0-disable]";

  memset(&char_md, 0, sizeof(char_md));
  char_md.char_props.write         = 1;
  char_md.char_props.write_wo_resp = 1;
  char_md.p_char_user_desc  = gsensor_enable;
  char_md.char_user_desc_size  = sizeof(gsensor_enable);
  char_md.char_user_desc_max_size  = char_md.char_user_desc_size;


  memset(&attr_md, 0, sizeof(attr_md));
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
  attr_md.vloc    = BLE_GATTS_VLOC_STACK;

  ble_uuid.type = p_sensor->uuid_type;
  ble_uuid.uuid = BLE_UUID_SENSOR_ENABLE_CHARACTERISTIC;

  memset(&attr_char_value, 0, sizeof(attr_char_value));
  attr_char_value.p_uuid    = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len  = BLE_SENSOR_ENABLE_BYTES;
  attr_char_value.max_len   = BLE_SENSOR_ENABLE_BYTES;

  return sd_ble_gatts_characteristic_add(p_sensor->service_handle,
                                         &char_md,
                                         &attr_char_value,
                                         &p_sensor->enable_handle);
}

uint32_t ble_sensor_init(ble_sensor_t *p_sensor)
{
  uint32_t      err_code;
  ble_uuid_t    ble_uuid;
  ble_uuid128_t sensor_base_uuid = SENSOR_BASE_UUID;

  VERIFY_PARAM_NOT_NULL(p_sensor);

  // Initialize the service structure.
  p_sensor->conn_handle             = BLE_CONN_HANDLE_INVALID;
  p_sensor->is_notification_enabled = false;

  // Add a Vendor Specific base UUID
  err_code = sd_ble_uuid_vs_add(&sensor_base_uuid, &p_sensor->uuid_type);
  VERIFY_SUCCESS(err_code);

  ble_uuid.type = p_sensor->uuid_type;
  ble_uuid.uuid = BLE_UUID_SENSOR_SERVICE;

  // Add the service.
  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_sensor->service_handle);
  VERIFY_SUCCESS(err_code);

  // Add the gravity enable Characteristic.
  err_code = g_sensor_enable_char_add(p_sensor);
  VERIFY_SUCCESS(err_code);

  // Add the gravity TX Characteristic.
  err_code = g_sensor_tx_char_add(p_sensor);
  VERIFY_SUCCESS(err_code);

  //The timer moudule is not ready....
  //app_timer_create(&m_data_transfer_timer_id, APP_TIMER_MODE_SINGLE_SHOT, sensor_data_transfer_timer_handler); HHTT

  return NRF_SUCCESS;
}

uint32_t zg_ble_sensor_init(void)
{
  uint32_t       err_code;
  err_code = ble_sensor_init(&m_sensor);
  APP_ERROR_CHECK(err_code);
  return err_code;
}

#endif // NRF_MODULE_ENABLED(BLE_SENSOR)
