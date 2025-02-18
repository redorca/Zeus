/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include "sdk_common.h"
#if NRF_MODULE_ENABLED(PEER_MANAGER)
#include "gatts_cache_manager.h"

#include <string.h>
#include "ble_gap.h"
#include "peer_manager_types.h"
#include "peer_manager_internal.h"
#include "peer_database.h"
#include "id_manager.h"


// Syntactic sugar, two spoons.
#define SYS_ATTR_SYS                    (BLE_GATTS_SYS_ATTR_FLAG_SYS_SRVCS)
#define SYS_ATTR_USR                    (BLE_GATTS_SYS_ATTR_FLAG_USR_SRVCS)
#define SYS_ATTR_BOTH                   (SYS_ATTR_SYS | SYS_ATTR_USR)

// The number of registered event handlers.
#define GSCM_EVENT_HANDLERS_CNT         (sizeof(m_evt_handler) / sizeof(m_evt_handler[0]))


// GATTS Cache Manager event handler in Peer Manager.
extern void gcm_gscm_evt_handler(gscm_evt_t const *p_event);

// GATTS Cache Manager events' handlers.
// The number of elements in this array is GSCM_EVENT_HANDLERS_CNT.
static gscm_evt_handler_t m_evt_handler[] =
{
  gcm_gscm_evt_handler
};

static bool               m_module_initialized;
static pm_peer_id_t       m_current_sc_store_peer_id;


/**@brief Function for resetting the module variable(s) of the GSCM module.
 */
static void internal_state_reset(void)
{
  m_module_initialized       = false;
  m_current_sc_store_peer_id = PM_PEER_ID_INVALID;
}


static void evt_send(gscm_evt_t const *p_event)
{
  for (uint32_t i = 0; i < GSCM_EVENT_HANDLERS_CNT; i++)
    {
      m_evt_handler[i](p_event);
    }
}


//lint -save -e550
/**@brief Function for storing service_changed_pending = true to flash for all peers, in sequence.
 *
 * This function aborts if it gets @ref NRF_ERROR_BUSY when trying to store. A subsequent call will
 * continue where the last call was aborted.
 */
static void service_changed_pending_set(void)
{
  NRF_PM_DEBUG_CHECK(m_module_initialized);

  ret_code_t err_code;
  // Use a uint32_t to enforce 4-byte alignment.
  static const uint32_t service_changed_pending = true;

  //lint -save -e65 -e64
  pm_peer_data_const_t peer_data =
  {
    .data_id                   = PM_PEER_DATA_ID_SERVICE_CHANGED_PENDING,
    .length_words              = PM_SC_STATE_N_WORDS(),
    .p_service_changed_pending = (bool *) &service_changed_pending,
  };
  //lint -restore

  err_code = pdb_raw_store(m_current_sc_store_peer_id, &peer_data, NULL);
  while ((m_current_sc_store_peer_id != PM_PEER_ID_INVALID) && (err_code != NRF_ERROR_BUSY))
    {
      m_current_sc_store_peer_id = pdb_next_peer_id_get(m_current_sc_store_peer_id);
      err_code = pdb_raw_store(m_current_sc_store_peer_id, &peer_data, NULL);
    }
}
//lint -restore



/**@brief Event handler for events from the Peer Database module.
 *        This function is extern in Peer Database.
 *
 * @param[in]  p_event The event that has happend with peer id and flags.
 */
void gscm_pdb_evt_handler(pdb_evt_t const *p_event)
{
  if (p_event->evt_id == PDB_EVT_RAW_STORED)
    {
      if (p_event->data_id == PM_PEER_DATA_ID_SERVICE_CHANGED_PENDING)
        {
          ret_code_t           err_code;
          pm_peer_data_flash_t peer_data;

          err_code = pdb_peer_data_ptr_get(p_event->peer_id,
                                           PM_PEER_DATA_ID_SERVICE_CHANGED_PENDING,
                                           &peer_data);

          if (err_code == NRF_SUCCESS)
            {
              gscm_evt_t gscm_evt;
              gscm_evt.evt_id = GSCM_EVT_SC_STATE_STORED;
              gscm_evt.peer_id = p_event->peer_id;
              //gscm_evt.params.sc_state_stored.state = &peer_data.p_service_changed_pending;
              gscm_evt.params.sc_state_stored.state =
                *peer_data.p_service_changed_pending;  // XXX changed the above line to this line to avoid warning.
              evt_send(&gscm_evt);
            }
        }
    }

  if (m_current_sc_store_peer_id != PM_PEER_ID_INVALID)
    {
      service_changed_pending_set();
    }
}


ret_code_t gscm_init()
{
  NRF_PM_DEBUG_CHECK(!m_module_initialized);

  internal_state_reset();
  m_module_initialized = true;

  return NRF_SUCCESS;
}


ret_code_t gscm_local_db_cache_update(uint16_t conn_handle)
{
  NRF_PM_DEBUG_CHECK(m_module_initialized);

  pm_peer_id_t peer_id = im_peer_id_get_by_conn_handle(conn_handle);
  ret_code_t   err_code;

  if (peer_id == PM_PEER_ID_INVALID)
    {
      return BLE_ERROR_INVALID_CONN_HANDLE;
    }
  else
    {
      pm_peer_data_t peer_data;
      uint16_t       n_bufs = 1;
      bool           retry_with_bigger_buffer = false;

      do
        {
          retry_with_bigger_buffer = false;

          err_code = pdb_write_buf_get(peer_id, PM_PEER_DATA_ID_GATT_LOCAL, n_bufs++, &peer_data);
          if (err_code == NRF_SUCCESS)
            {
              pm_peer_data_local_gatt_db_t *p_local_gatt_db = peer_data.p_local_gatt_db;

              p_local_gatt_db->flags = SYS_ATTR_BOTH;

              err_code = sd_ble_gatts_sys_attr_get(conn_handle, &p_local_gatt_db->data[0], &p_local_gatt_db->len, p_local_gatt_db->flags);

              if (err_code == NRF_SUCCESS)
                {
                  err_code = pdb_write_buf_store(peer_id, PM_PEER_DATA_ID_GATT_LOCAL);
                }
              else
                {
                  if (err_code == NRF_ERROR_DATA_SIZE)
                    {
                      // The sys attributes are bigger than the requested write buffer.
                      retry_with_bigger_buffer = true;
                    }
                  else if (err_code == NRF_ERROR_NOT_FOUND)
                    {
                      // There are no sys attributes in the GATT db, so nothing needs to be stored.
                      err_code = NRF_SUCCESS;
                    }

                  ret_code_t err_code_release = pdb_write_buf_release(peer_id, PM_PEER_DATA_ID_GATT_LOCAL);
                  if (err_code_release != NRF_SUCCESS)
                    {
                      err_code = NRF_ERROR_INTERNAL;
                    }
                }
            }
          else if (err_code == NRF_ERROR_INVALID_PARAM)
            {
              // The sys attributes are bigger than the entire write buffer.
              err_code = NRF_ERROR_DATA_SIZE;
            }
        }
      while (retry_with_bigger_buffer);
    }

  return err_code;
}


ret_code_t gscm_local_db_cache_apply(uint16_t conn_handle)
{
  NRF_PM_DEBUG_CHECK(m_module_initialized);

  pm_peer_id_t         peer_id = im_peer_id_get_by_conn_handle(conn_handle);
  ret_code_t           err_code;
  pm_peer_data_flash_t peer_data;
  uint8_t      const *p_sys_attr_data = NULL;
  uint16_t             sys_attr_len    = 0;
  uint32_t             sys_attr_flags  = (SYS_ATTR_BOTH);
  bool                 all_attributes_applied = true;

  if (peer_id != PM_PEER_ID_INVALID)
    {
      err_code = pdb_peer_data_ptr_get(peer_id, PM_PEER_DATA_ID_GATT_LOCAL, &peer_data);
      if (err_code == NRF_SUCCESS)
        {
          pm_peer_data_local_gatt_db_t const *p_local_gatt_db;

          p_local_gatt_db = peer_data.p_local_gatt_db;
          p_sys_attr_data = p_local_gatt_db->data;
          sys_attr_len    = p_local_gatt_db->len;
          sys_attr_flags  = p_local_gatt_db->flags;
        }
    }

  do
    {
      err_code = sd_ble_gatts_sys_attr_set(conn_handle, p_sys_attr_data, sys_attr_len, sys_attr_flags);

      if (err_code == NRF_ERROR_NO_MEM)
        {
          err_code = NRF_ERROR_BUSY;
        }
      else if (err_code == NRF_ERROR_INVALID_STATE)
        {
          err_code = NRF_SUCCESS;
        }
      else if (err_code == NRF_ERROR_INVALID_DATA)
        {
          all_attributes_applied = false;

          if (sys_attr_flags & SYS_ATTR_USR)
            {
              // Try setting only system attributes.
              sys_attr_flags = SYS_ATTR_SYS;
            }
          else if (p_sys_attr_data || sys_attr_len)
            {
              // Try reporting that none exist.
              p_sys_attr_data = NULL;
              sys_attr_len    = 0;
              sys_attr_flags  = SYS_ATTR_BOTH;
            }
          else
            {
              err_code = NRF_ERROR_INTERNAL;
            }
        }
    }
  while (err_code == NRF_ERROR_INVALID_DATA);

  if (!all_attributes_applied)
    {
      err_code = NRF_ERROR_INVALID_DATA;
    }

  return err_code;
}


ret_code_t gscm_local_db_cache_set(pm_peer_id_t peer_id, pm_peer_data_local_gatt_db_t *p_local_db)
{
  NRF_PM_DEBUG_CHECK(m_module_initialized);

  pm_peer_data_const_t peer_data;

  memset(&peer_data, 0, sizeof(pm_peer_data_const_t));
  peer_data.data_id = PM_PEER_DATA_ID_GATT_LOCAL;
  peer_data.p_local_gatt_db = p_local_db;

  return pdb_raw_store(peer_id, &peer_data, NULL);
}


void gscm_local_database_has_changed(void)
{
  NRF_PM_DEBUG_CHECK(m_module_initialized);
  m_current_sc_store_peer_id = pdb_next_peer_id_get(PM_PEER_ID_INVALID);
  service_changed_pending_set();
}


bool gscm_service_changed_ind_needed(uint16_t conn_handle)
{
  ret_code_t err_code;
  pm_peer_data_flash_t peer_data;
  pm_peer_id_t peer_id = im_peer_id_get_by_conn_handle(conn_handle);

  err_code = pdb_peer_data_ptr_get(peer_id, PM_PEER_DATA_ID_SERVICE_CHANGED_PENDING, &peer_data);

  if (err_code != NRF_SUCCESS)
    {
      return false;
    }

  return *peer_data.p_service_changed_pending;
}


ret_code_t gscm_service_changed_ind_send(uint16_t conn_handle)
{
  static uint16_t start_handle = 0x0000;
  const  uint16_t end_handle   = 0xFFFF;
  ret_code_t err_code;

  do
    {
      err_code = sd_ble_gatts_service_changed(conn_handle, start_handle, end_handle);
      if (err_code == BLE_ERROR_INVALID_ATTR_HANDLE)
        {
          start_handle += 1;
        }
    }
  while (err_code == BLE_ERROR_INVALID_ATTR_HANDLE);

  return err_code;
}


void gscm_db_change_notification_done(pm_peer_id_t peer_id)
{
  NRF_PM_DEBUG_CHECK(m_module_initialized);

  // Use a uint32_t to enforce 4-byte alignment.
  static const uint32_t service_changed_pending = false;

  //lint -save -e65 -e64
  pm_peer_data_const_t peer_data =
  {
    .data_id                   = PM_PEER_DATA_ID_SERVICE_CHANGED_PENDING,
    .length_words              = PM_SC_STATE_N_WORDS(),
    .p_service_changed_pending = (bool *) &service_changed_pending,
  };
  //lint -restore

  // Don't need to check return code, because all error conditions can be ignored.
  //lint -save -e550
  (void) pdb_raw_store(peer_id, &peer_data, NULL);
  //lint -restore
}
#endif // NRF_MODULE_ENABLED(PEER_MANAGER)
