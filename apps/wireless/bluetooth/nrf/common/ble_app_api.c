/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
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


/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include <nuttx/board.h>
#include <errno.h>
#include <stdio.h>
#include <fcntl.h>
#include <pthread.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <sys/time.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "softdevice_handler.h"
#include "fds.h"
#include "fstorage.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "../ble_bsp/bsp.h"
#include "ble_advdata.h"
#include "../peer_manager/peer_manager.h"
#include "../nrf_ble_gatt/nrf_ble_gatt.h"
#include "../common/ble_conn_params.h"
#include "../common/ble_conn_state.h"
#include "../ble_advertising/ble_advertising.h"
#include "../common/ble_srv_common.h"

#include <wireless/bluetooth/ble_app_api.h>
#include <wireless/bluetooth/ble_app_error.h>
#include <wireless/bluetooth/bt_mq.h>

#include "nrf_rng.h"

/****************************************************************************
 * Local variables
 ****************************************************************************/

#define FIRST_CONN_PARAMS_UPDATE_DELAY       (5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY        (30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT         (3)                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                       (1)                          /**< Perform bonding. */
#define SEC_PARAM_MITM                       (0)                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                       (0)                          /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                   (0)                          /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES            BLE_GAP_IO_CAPS_NONE         /**< No I/O capabilities. */
#define SEC_PARAM_OOB                        (0)                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE               (7)                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE               (16)                         /**< Maximum encryption key size. */

#define PEER_MANAGE_IN_USE

#define BLE_APP_INTL_ERR_CHK(err_code)                                      \
        do                                                                  \
        {                                                                   \
          uint32_t ret = BLE_APP_SUCCESS;                                   \
          if(err_code != NRF_SUCCESS)                                       \
            {                                                               \
              ret = ble_app_err_code_convert(err_code);                     \
              APP_LOG_ERROR("\n Error(%d) happen on file %s line %d.\n",    \
              err_code, __FILE__, __LINE__);                                \
              return ret;                                                   \
            }                                                               \
        }while(0)

/****************************************************************************
 * Structs
 ****************************************************************************/

/****************************************************************************
 * Local variables
 ****************************************************************************/
static ble_evt_cb_list_t ble_evt_cb_list = {0};

static uint16_t  m_conn_handle = BLE_CONN_HANDLE_INVALID;

static ble_adv_mode_t adv_mode;

static sem_t m_app_ble_ready;

/****************************************************************************
 * Local Functions
 ****************************************************************************/
static uint32_t ble_app_err_code_convert(uint32_t err_code);

/****************************************************************************
 * Name: sleep_mode_enter
 *
 * Description:
 *   Function for putting the chip into sleep mode.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/
static void sleep_mode_enter(void)
{
  uint32_t err_code;

  // Go to system-off mode (this function will not return; wakeup will cause a reset).
  err_code = sd_power_system_off();
  APP_ERROR_CHECK(err_code);
}

/****************************************************************************
 * Name: on_adv_evt
 *
 * Description:
 *   Function for handling advertising events.
 *
 * Input Parameters:
 *   ble_adv_evt        - advertising event id
 *
 *
 * Returned Value:
 *   0: void
 *
 ****************************************************************************/
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
  uint32_t err_code;
  switch (ble_adv_evt)
    {
      case BLE_ADV_EVT_DIRECTED:
        APP_LOG_INFO("DIRECTED Adverstising...\r\n");
        err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
        APP_ERROR_CHECK(err_code);
        break;

      case BLE_ADV_EVT_FAST:
        APP_LOG_INFO("FAST Adverstising...\r\n");
        err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
        APP_ERROR_CHECK(err_code);
        break;

      case BLE_ADV_EVT_SLOW:
        break;

      case BLE_ADV_EVT_FAST_WHITELIST:
        break;

      case BLE_ADV_EVT_SLOW_WHITELIST:
        break;

      case BLE_ADV_EVT_IDLE:
        APP_LOG_INFO("ADV IDLE...\r\n");
        sleep_mode_enter();
        break;

      case BLE_ADV_EVT_WHITELIST_REQUEST:
        break;

      case BLE_ADV_EVT_PEER_ADDR_REQUEST:
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: ble_evt_handler
 *
 * Description:
 *  Function for receiving the Application's BLE Stack events.
 *
 * Input Parameters:
 *   p_ble_evt   Bluetooth stack event.
 *
 * Returned Value:
 *
 ****************************************************************************/
static void on_ble_evt(ble_evt_t *p_ble_evt)
{
  uint32_t err_code;

  switch (p_ble_evt->header.evt_id)
    {
      case BLE_GAP_EVT_CONNECTED:
        APP_LOG_INFO("Connected\r\n");
        err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
        APP_ERROR_CHECK(err_code);
        m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        break;

      case BLE_GAP_EVT_DISCONNECTED:
        APP_LOG_INFO("Disconnected\r\n");
        m_conn_handle = BLE_CONN_HANDLE_INVALID;
        break;

      case BLE_GATTC_EVT_TIMEOUT:
        /* Disconnect on GATT Client timeout event.*/
        APP_LOG_INFO("GATT Client Timeout.\r\n");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

      case BLE_GATTS_EVT_TIMEOUT:
        /*Disconnect on GATT Server timeout event.*/
        APP_LOG_INFO("GATT Server Timeout.\r\n");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

      case BLE_EVT_USER_MEM_REQUEST:
        err_code = sd_ble_user_mem_reply(m_conn_handle, NULL);
        APP_ERROR_CHECK(err_code);
        break;

      case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
          /* BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST*/
          ble_gatts_evt_rw_authorize_request_t  req;
          ble_gatts_rw_authorize_reply_params_t auth_reply;

          req = p_ble_evt->evt.gatts_evt.params.authorize_request;

          if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
              if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                  (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                  (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                  if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                      auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                  else
                    {
                      auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                  auth_reply.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_APP_END;
                  err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                             &auth_reply);
                  APP_ERROR_CHECK(err_code);
                }
            }
        }
        break;

#if (NRF_SD_BLE_API_VERSION >= 3)
      /* BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST */
      case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
        err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                   BLE_APP_MAX_MTU_SIZE);
        APP_ERROR_CHECK(err_code);
        break;
#endif

      default:
        /*No implementation needed.*/
        break;
    }
}

/****************************************************************************
 * Name: on_conn_params_evt
 *
 * Description:
 *   This function will be called for all events in the Connection
 *          Parameters Module which are passed to the application.
 *
 * Input Parameters:
 *   p_evt  Event received from the Connection Parameters Module.
 *
 * Returned Value:
 *
 ****************************************************************************/
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt)
{
  uint32_t err_code;

  if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
      err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
      APP_ERROR_CHECK(err_code);
    }
}

/****************************************************************************
 * Name: ble_evt_handler
 *
 * Description:
 *  This function is called to handler
 *  after a BLE event has been received
 *
 * Input Parameters:
 *   sys_evt   System stack event.
 *
 * Returned Value:
 *
 ****************************************************************************/
static void ble_evt_handler(ble_evt_t *p_ble_evt)
{
  uint8_t i = 0;

  on_ble_evt(p_ble_evt);
  /** The Connection state module has to be fed BLE events in order to function correctly
   * Remember to call ble_conn_state_on_ble_evt before calling any ble_conns_state_* functions. */
  ble_advertising_on_ble_evt(p_ble_evt);
  ble_conn_state_on_ble_evt(p_ble_evt);
#ifdef PEER_MANAGE_IN_USE
  ble_conn_params_on_ble_evt(p_ble_evt);
  pm_on_ble_evt(p_ble_evt);
#endif

  for (i = 0; i < ble_evt_cb_list.handler_cnt; ++i)
    {
      (*ble_evt_cb_list.handler[i])((void *)p_ble_evt);
    }
}

/****************************************************************************
 * Name: sys_evt_handler
 *
 * Description:
 *  This function is called to handler
 *  after a system event has been received
 *
 * Input Parameters:
 *   sys_evt   System stack event.
 *
 * Returned Value:
 *   0: void
 *
 ****************************************************************************/
static void sys_evt_handler(uint32_t sys_evt)
{
  // Dispatch the system event to the fstorage module, where it will be
  // dispatched to the Flash Data Storage (FDS) module.
  fs_sys_event_handler(sys_evt);

  // Dispatch to the Advertising module last, since it will check if there are any
  // pending flash operations in fstorage. Let fstorage process system events first,
  // so that it can report correctly to the Advertising module.
  ble_advertising_on_sys_evt(sys_evt);
}

/****************************************************************************
 * Name: conn_params_error_handler
 *
 * Description:
 *   Function for handling Peer Manager events.
 *
 * Input Parameters:
 *   p_evt  Peer Manager event..
 *
 *
 * Returned Value:
 *   0: void
 *
 ****************************************************************************/
#ifdef PEER_MANAGE_IN_USE
static void pm_evt_handler(pm_evt_t const *p_evt)
{
  ret_code_t err_code;

  switch (p_evt->evt_id)
    {
      case PM_EVT_BONDED_PEER_CONNECTED:
        {
          APP_LOG_INFO("Connected to a previously bonded device.\r\n");
        }
        break;

      case PM_EVT_CONN_SEC_SUCCEEDED:
        {
          APP_LOG_INFO("Connection secured. Role: %d. conn_handle: %d, Procedure: %d\r\n",
                       ble_conn_state_role(p_evt->conn_handle),
                       p_evt->conn_handle,
                       p_evt->params.conn_sec_succeeded.procedure);
        }
        break;

      case PM_EVT_CONN_SEC_FAILED:
        {
          /* Often, when securing fails, it shouldn't be restarted, for security reasons.
           * Other times, it can be restarted directly.
           * Sometimes it can be restarted, but only after changing some Security Parameters.
           * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
           * Sometimes it is impossible, to secure the link, or the peer device does not support it.
           * How to handle this error is highly application dependent. */
        } break;

      case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
          // Reject pairing request from an already bonded peer.
          pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
          pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        }
        break;

      case PM_EVT_STORAGE_FULL:
        {
          // Run garbage collection on the flash.
          err_code = fds_gc();
          if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
              // Retry.
            }
          else
            {
              APP_ERROR_CHECK(err_code);
            }
        }
        break;

      case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
          ble_advertising_start(adv_mode);
        }
        break;

      case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        {
          // The local database has likely changed, send service changed indications.
          pm_local_database_has_changed();
        }
        break;

      case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
          // Assert.
          APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        }
        break;

      case PM_EVT_PEER_DELETE_FAILED:
        {
          // Assert.
          APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        }
        break;

      case PM_EVT_PEERS_DELETE_FAILED:
        {
          // Assert.
          APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        }
        break;

      case PM_EVT_ERROR_UNEXPECTED:
        {
          // Assert.
          APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        }
        break;

      case PM_EVT_CONN_SEC_START:
      case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
      case PM_EVT_PEER_DELETE_SUCCEEDED:
      case PM_EVT_LOCAL_DB_CACHE_APPLIED:
      case PM_EVT_SERVICE_CHANGED_IND_SENT:
      case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
      default:
        break;
    }
}
#endif

/****************************************************************************
 * Name: conn_params_error_handler
 *
 * Description:
 *   Connection parameters error handler.
 *
 * Input Parameters:
 *   nrf_error  Error number.
 *
 * Returned Value:
 *
 ****************************************************************************/
static void conn_params_error_handler(uint32_t nrf_error)
{
  APP_ERROR_HANDLER(nrf_error);
}

/****************************************************************************
 * Name: ble_event_thread
 *
 * Description:
 *   BLE event thread.
 *
 * Input Parameters:
 *   nrf_error  Error number.
 *
 * Returned Value:
 *
 ****************************************************************************/
void *ble_event_thread(void *unused)
{
  while (1)
    {
      sem_wait(&m_app_ble_ready);
      intern_softdevice_events_execute();
    }
}

/****************************************************************************
 * Name: ble_irq_handler
 *
 * Description:
 *   BLE IRQ handler from SD.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   0: Successful
 *
 ****************************************************************************/

static uint32_t ble_irq_handler(void)
{
  int32_t ret;
  ret = sem_post(&m_app_ble_ready);
  assert(ret == 0);
  return ret;
}

/****************************************************************************
 * Name: ble_stack_init
 *
 * Description:
 *   BLE stack initialize.
 *
 * Input Parameters:
 *
 *
 * Returned Value:
 *   0: Successful
 *
 ****************************************************************************/
static uint32_t ble_stack_init(void)
{
  uint32_t err_code;

  nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

  // Initialize the SoftDevice handler module.
  SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, ble_irq_handler);

  ble_enable_params_t ble_enable_params;
  err_code = softdevice_enable_get_default_config(BLE_APP_CENTRAL_LINK_COUNT,
                                                  BLE_APP_PERIPHERAL_LINK_COUNT,
                                                  &ble_enable_params);
  BLE_APP_INTL_ERR_CHK(err_code);


  // Check the ram settings against the used number of links
  CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);

  // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION >= 3)
  ble_enable_params.gatt_enable_params.att_mtu = BLE_APP_MAX_MTU_SIZE;
#endif
  err_code = softdevice_enable(&ble_enable_params);
  BLE_APP_INTL_ERR_CHK(err_code);

  // Register with the SoftDevice handler module for BLE events.
  err_code = softdevice_ble_evt_handler_set(ble_evt_handler);
  BLE_APP_INTL_ERR_CHK(err_code);

  // Register with the SoftDevice handler module for BLE events.
  err_code = softdevice_sys_evt_handler_set(sys_evt_handler);
  BLE_APP_INTL_ERR_CHK(err_code);

  return err_code;

}

/****************************************************************************
 * Name: gap_params_init
 *
 * Description:
 *   This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *   device including the device name, appearance, and the preferred connection parameters.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   0: Successful
 *
 ****************************************************************************/
static uint32_t gap_params_init(void)
{
  uint32_t       err_code;
  ble_app_ppcp_t ppcp;
  ble_app_addr_t addr;

  memset(&ppcp, 0, sizeof(ppcp));

  ppcp.min_conn_interval = CONFIG_BLE_APP_PPCP_MIN_INTERVAL;
  ppcp.max_conn_interval = CONFIG_BLE_APP_PPCP_MAX_INTERVAL;
  ppcp.slave_latency     = CONFIG_BLE_APP_PPCP_SLAVE_LATENCY;
  ppcp.conn_sup_timeout  = CONFIG_BLE_APP_CONN_SUP_TIMEOUT;

#ifdef _BLE_DEV_NAME
  err_code = ble_app_set_device_name((const uint8_t *)_BLE_DEV_NAME,
                                     sizeof(_BLE_DEV_NAME) - 1);
  BLE_APP_INTL_ERR_CHK(err_code);
#else
  err_code = ble_app_set_device_name((const uint8_t *)CONFIG_BLE_APP_ADV_DEVICE_NAME,
                                     strlen(CONFIG_BLE_APP_ADV_DEVICE_NAME));
  BLE_APP_INTL_ERR_CHK(err_code);
#endif

  /*Set BLE address, current just use random static as default.*/
  addr.addr_type = BLE_APP_ADDR_TYPE_RANDOM_STATIC;
  for (uint8_t i = 0; i < BLE_APP_ADDR_LEN; i ++)
    {
      addr.addr[i] = (uint8_t)nrf_rng_random_value_get();
    }
  addr.addr[BLE_APP_ADDR_LEN - 1] |= BLE_APP_ADDR_TYPE_RANDOM_STATIC_MSB_BIT;
  ble_app_set_address(&addr);

  err_code = ble_app_set_ppcp(&ppcp);
  BLE_APP_INTL_ERR_CHK(err_code);

  return err_code;
}

/****************************************************************************
 * Name: peer_manager_init
 *
 * Description:
 *   Peer devices infomation manager initialize.
 *
 * Input Parameters:
 *   erase_bonds  Indicates whether bonding information should be cleared from
 *                persistent storage during initialization of the Peer Manager
 *
 * Returned Value:
 *   0: Successful
 *
 ****************************************************************************/
#ifdef PEER_MANAGE_IN_USE
static uint32_t peer_manager_init(bool erase_bonds)
{
  ble_gap_sec_params_t sec_param;
  ret_code_t           err_code;

  err_code = pm_init();
  BLE_APP_INTL_ERR_CHK(err_code);

  if (erase_bonds)
    {
      err_code = pm_peers_delete();
      BLE_APP_INTL_ERR_CHK(err_code);
    }

  memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

  // Security parameters to be used for all security procedures.
  sec_param.bond           = SEC_PARAM_BOND;
  sec_param.mitm           = SEC_PARAM_MITM;
  sec_param.lesc           = SEC_PARAM_LESC;
  sec_param.keypress       = SEC_PARAM_KEYPRESS;
  sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
  sec_param.oob            = SEC_PARAM_OOB;
  sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
  sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
  sec_param.kdist_own.enc  = 1;
  sec_param.kdist_own.id   = 1;
  sec_param.kdist_peer.enc = 1;
  sec_param.kdist_peer.id  = 1;

  err_code = pm_sec_params_set(&sec_param);
  BLE_APP_INTL_ERR_CHK(err_code);

  err_code = pm_register(pm_evt_handler);
  BLE_APP_INTL_ERR_CHK(err_code);

  return err_code;
}
#endif

/****************************************************************************
 * Name: conn_params_init
 *
 * Description:
 *   Connection parameters initialize.
 *
 * Input Parameters:
 *
 *
 * Returned Value:
 *   0: Successful
 *
 ****************************************************************************/
static uint32_t conn_params_init(void)
{
  uint32_t               err_code;
  ble_conn_params_init_t cp_init;

  memset(&cp_init, 0, sizeof(cp_init));

  cp_init.p_conn_params                  = NULL;
  cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
  cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
  cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
  cp_init.disconnect_on_fail             = false;
  cp_init.evt_handler                    = on_conn_params_evt;
  cp_init.error_handler                  = conn_params_error_handler;

  err_code = ble_conn_params_init(&cp_init);
  BLE_APP_INTL_ERR_CHK(err_code);

  return err_code;
}

/****************************************************************************
 * Name: ble_adv_parameter_check
 *
 * Description:
 *   Check BEL parameter range and value.
 *
 * Input Parameters:
 *   dest        - pointer to parameter struct
 *
 *
 * Returned Value:
 *   0: Successful
 *
 ****************************************************************************/
static uint32_t ble_adv_parameter_check(ble_adv_params_t *p_adv_para)
{

  if (p_adv_para->type > BLE_APP_ADV_TYPE_MAX)
    {
      p_adv_para->type = BLE_APP_ADV_TYPE_IND;
    }

  //Not support custom to add peer address now on NORDIC.
  if ( (BLE_APP_ADV_TYPE_DIRECT_IND == p_adv_para->type) && (NULL == p_adv_para->p_peer_addr))
    {
      APP_LOG_ERROR("In directly ADV, peer address is necesary...");
    }

  if (p_adv_para->fp > BLE_APP_ADV_PF_MAX)
    {
      p_adv_para->type = BLE_APP_ADV_FP_ANY;
    }

  if ((p_adv_para->interval < BLE_APP_ADV_MIN_INTERVAL) || (p_adv_para->interval > BLE_APP_ADV_MAX_INTERVAL))
    {
      p_adv_para->interval = BLE_APP_ADV_INTERVAL_FAST;
    }

  if (p_adv_para->timeout < BLE_APP_ADV_MAX_TIMEOUT)
    {
      p_adv_para->timeout = BLE_APP_ADV_TIMEOUT_FAST;
    }

  return NRF_SUCCESS;
}

/****************************************************************************
 * Name: ble_adv_data_convert
 *
 * Description:
 *   BEL data covert from source to destination.
 *
 * Input Parameters:
 *   dest        - pointer to destination struct
 *   ori         - pointer to source struct
 *
 * Returned Value:
 *
 *
 ****************************************************************************/
static void ble_adv_data_convert(ble_advdata_t *dest, const ble_app_advdata_t *ori)
{
  uint16_t  i = 0;

  if ((NULL == ori) || (NULL == dest))
    {
      return ;
    }

  dest->name_type               = (ble_advdata_name_type_t)ori->name_type;
  dest->flags                   = ori->flags;

  /** The BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED flag must be set. */
  if (((dest->flags & BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED) == 0))
    {
      dest->flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    }
  dest->short_name_len          = ori->short_name_len;
  dest->include_appearance      = ori->include_appearance;
  if (ori->tx_power_level == 0)
    {
      dest->p_tx_power_level    = NULL;
    }
  else
    {
      *dest->p_tx_power_level   = ori->tx_power_level;
    }

  for (i = 0; i < ori->uuid16_list.uuid_cnt; i++)
    {
      if (BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE == ori->uuid16_list.p_uuids[i].type)
        {
          dest->uuids_more_available.p_uuids[i].type = BLE_UUID_TYPE_BLE;
          dest->uuids_more_available.p_uuids[i].uuid = ori->uuid16_list.p_uuids[i].value;
          dest->uuids_more_available.uuid_cnt++;
        }
      else if (BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE == ori->uuid16_list.p_uuids[i].type)
        {
          dest->uuids_complete.p_uuids[i].type = BLE_UUID_TYPE_BLE;
          dest->uuids_complete.p_uuids[i].uuid = ori->uuid16_list.p_uuids[i].value;
          dest->uuids_complete.uuid_cnt++;
        }
      else if (BLE_GAP_AD_TYPE_SOLICITED_SERVICE_UUIDS_16BIT == ori->uuid16_list.p_uuids[i].type)
        {
          dest->uuids_solicited.p_uuids[i].type = BLE_UUID_TYPE_BLE;
          dest->uuids_solicited.p_uuids[i].uuid = ori->uuid16_list.p_uuids[i].value;
          dest->uuids_solicited.uuid_cnt++;
        }
      else
        {
          APP_LOG_WARNING("\n Unknown uuid types...\n");
        }
    }

  if ((0 == ori->slave_conn_int.max_conn_interval) &&
      (0 == ori->slave_conn_int.min_conn_interval))
    {
      dest->p_slave_conn_int = NULL;
    }
  else
    {
      dest->p_slave_conn_int->max_conn_interval = ori->slave_conn_int.max_conn_interval;
      dest->p_slave_conn_int->min_conn_interval = ori->slave_conn_int.min_conn_interval;
    }

  if (ori->manuf_specific_data.size == 0)
    {
      dest->p_manuf_specific_data = NULL;
    }
  else
    {
      dest->p_manuf_specific_data->company_identifier = ori->manuf_specific_data.uuid;
      dest->p_manuf_specific_data->data.size = ori->manuf_specific_data.size;
      dest->p_manuf_specific_data->data.p_data = ori->manuf_specific_data.p_data;
    }

  dest->service_data_count = ori->service_data_count;

  if (dest->service_data_count > 0)
    {

      for (i = 0; i < dest->service_data_count; i ++)
        {
          dest->p_service_data_array[i].service_uuid = ori->p_service_data_array[i].uuid;
          dest->p_service_data_array[i].data.size = ori->p_service_data_array[i].size;
          dest->p_service_data_array[i].data.p_data = ori->p_service_data_array[i].p_data;
        }
    }
}

static uint32_t ble_app_err_code_convert(uint32_t err_code)
{
  uint32_t ret = BLE_APP_SUCCESS;

  switch (err_code)
    {
      case NRF_SUCCESS:
        ret = BLE_APP_SUCCESS;
        break;

      case NRF_ERROR_NO_MEM:
        ret = BLE_APP_ERROR_OUT_OF_MEM;
        break;

      case NRF_ERROR_NOT_FOUND:
      case NRF_ERROR_NOT_SUPPORTED:
        ret = BLE_APP_ERROR_NOT_SUPPORTED;
        break;

      case NRF_ERROR_INVALID_PARAM:
      case NRF_ERROR_INVALID_STATE:
      case NRF_ERROR_INVALID_FLAGS:
      case NRF_ERROR_INVALID_DATA:
        ret = BLE_APP_ERROR_INVALID_PARAM;
        break;

      case NRF_ERROR_INVALID_LENGTH:
      case NRF_ERROR_DATA_SIZE:
        ret = BLE_APP_ERROR_INVALID_LENGTH;
        break;

      case NRF_ERROR_TIMEOUT:
        ret = BLE_APP_ERROR_TIMEOUT;
        break;

      case NRF_ERROR_NULL:
      case NRF_ERROR_FORBIDDEN:
        ret = BLE_APP_ERROR_NO_ENOUGH_PERMISSON;
        break;

      case NRF_ERROR_INVALID_ADDR:
        ret = BLE_APP_ERROR_INVALID_ADDR;
        break;

      case NRF_ERROR_BUSY:
        ret = BLE_APP_ERROR_BUSY;
        break;

      case NRF_ERROR_CONN_COUNT:
        ret = BLE_APP_ERROR_EXCEED_MAX_CONN_COUNT;
        break;

      case NRF_ERROR_RESOURCES:
      case NRF_ERROR_SVC_HANDLER_MISSING:
      case NRF_ERROR_SOFTDEVICE_NOT_ENABLED:
      case NRF_ERROR_INTERNAL:
      default:
        ret = BLE_APP_ERROR_FROM_PLATFORM;
        break;
    }
  return ret;
}

/****************************************************************************
 * Pulic funtions
 ****************************************************************************/
void ble_app_get_version(ble_app_version *ver)
{
  ver->major_version = BLE_APP_MAJOR_VER;
  ver->minor_version = BLE_APP_MINOR_VER;
  ver->patch_version = BLE_APP_PATCH_VER;
  return;
}

uint32_t ble_evt_cb_handler_register(ble_evt_cb_handler func)
{
  if (ble_evt_cb_list.handler_cnt >= BLE_APP_EVT_CB_HANDLER_MAX_NUM)
    {
      APP_LOG_ERROR("\n Exceed the MAX number.\n");
      return NRF_ERROR_NO_MEM;
    }
  ble_evt_cb_list.handler[ble_evt_cb_list.handler_cnt++] = func;
  return NRF_SUCCESS;
}

uint32_t ble_app_set_ppcp(ble_app_ppcp_t *ppcp)
{
  uint32_t                err_code;
  ble_gap_conn_params_t bgcp;
  memset(&bgcp, 0, sizeof(bgcp));

  if ((ppcp->max_conn_interval > BLE_APP_CONN_MAX_INTERVAL)    ||
      (ppcp->min_conn_interval < BLE_APP_CONN_MIN_INTERVAL)    ||
      (ppcp->conn_sup_timeout  > BLE_APP_CONN_MAX_SUP_TIMEOUT) ||
      (ppcp->conn_sup_timeout  < BLE_APP_CONN_MIN_SUP_TIMEOUT) ||
      (ppcp->slave_latency     > BLE_APP_CONN_MAX_SLAVE_LATENCY))
    {
      err_code = NRF_ERROR_INVALID_PARAM;
      return err_code;
    }
  bgcp.conn_sup_timeout  = ppcp->conn_sup_timeout;
  bgcp.max_conn_interval = ppcp->max_conn_interval;
  bgcp.min_conn_interval = ppcp->min_conn_interval;
  bgcp.slave_latency     = ppcp->slave_latency;
  err_code = ble_set_preferred_conn_paramter(&bgcp);
  BLE_APP_INTL_ERR_CHK(err_code);
  return err_code;
}

uint32_t ble_app_set_address(const ble_app_addr_t *addr)
{
  uint32_t err_code = 0;
  ble_gap_addr_t nrf_addr;
  switch (addr->addr_type)
    {
      case BLE_APP_ADDR_TYPE_PUBLIC:
      case BLE_APP_ADDR_TYPE_RANDOM_STATIC:
        nrf_addr.addr_id_peer = 0;
        nrf_addr.addr_type = (addr->addr_type == BLE_APP_ADDR_TYPE_PUBLIC) ? BLE_GAP_ADDR_TYPE_PUBLIC : BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
        memcpy(nrf_addr.addr, addr->addr, BLE_APP_ADDR_LEN);
        if (BLE_APP_ADDR_TYPE_RANDOM_STATIC == addr->addr_type)
          {
            if ((nrf_addr.addr[BLE_APP_ADDR_LEN - 1] & BLE_APP_ADDR_TYPE_RANDOM_STATIC_MSB_BIT) !=
                BLE_APP_ADDR_TYPE_RANDOM_STATIC_MSB_BIT)
              {
                APP_LOG_WARNING("\n Random static address MSB Bits is wrong [%x].\n", nrf_addr.addr[BLE_APP_ADDR_LEN - 1]);
                nrf_addr.addr[BLE_APP_ADDR_LEN - 1] |= BLE_APP_ADDR_TYPE_RANDOM_STATIC_MSB_BIT;
                APP_LOG_WARNING("\n Correct the random static address to [%x].\n", nrf_addr.addr[BLE_APP_ADDR_LEN - 1]);
              }
          }
        err_code = sd_ble_gap_addr_set(&nrf_addr);
        BLE_APP_INTL_ERR_CHK(err_code);
        break;

      case BLE_APP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE:
      case BLE_APP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE:
        {
          ble_gap_privacy_params_t ble_gap_privacy_params = {0};
          ble_gap_privacy_params.private_addr_type = (addr->addr_type == BLE_APP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE) ?
                                                     BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE :
                                                     BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE;
          ble_gap_privacy_params.privacy_mode = BLE_GAP_PRIVACY_MODE_DEVICE_PRIVACY;
          ble_gap_privacy_params.private_addr_cycle_s = BLE_GAP_DEFAULT_PRIVATE_ADDR_CYCLE_INTERVAL_S;
          ble_gap_privacy_params.p_device_irk = NULL;
          err_code = sd_ble_gap_privacy_set(&ble_gap_privacy_params);
          BLE_APP_INTL_ERR_CHK(err_code);
        }
      default:
        break;

    }
  return err_code;
}

uint32_t ble_app_set_device_name(const uint8_t *data, uint16_t len)
{
  uint32_t                err_code;
  ble_gap_conn_sec_mode_t sec_mode;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

  err_code = sd_ble_gap_device_name_set(&sec_mode, data, len);
  BLE_APP_INTL_ERR_CHK(err_code);
  return err_code;
}

uint32_t ble_adv_set_data(const ble_app_advdata_t *p_advdata, const ble_app_advdata_t *p_srdata)
{
  uint32_t                   err_code;
  ble_advdata_t              advdata;
  ble_advdata_t              srdata;
  ble_advdata_t              *p_sr_data;
  ble_adv_modes_config_t     options;

  int8_t                     ad_tx_power_level = 0;
  ble_advdata_conn_int_t     ad_slave_conn_int = {0};
  ble_advdata_manuf_data_t   ad_manuf_specific_data = {0};
  ble_advdata_service_data_t ad_service_data[BLE_APP_ADV_MAX_SERVICES_NUM] = {0};
  ble_uuid_t                 ad_ma_uuids[BLE_APP_ADV_MAX_UUID_NUM] = {0};
  ble_uuid_t                 ad_cp_uuids[BLE_APP_ADV_MAX_UUID_NUM] = {0};
  ble_uuid_t                 ad_sl_uuids[BLE_APP_ADV_MAX_UUID_NUM] = {0};

  int8_t                     sr_tx_power_level = 0;
  ble_advdata_conn_int_t     sr_slave_conn_int = {0};
  ble_advdata_manuf_data_t   sr_manuf_specific_data = {0};
  ble_advdata_service_data_t sr_service_data[BLE_APP_ADV_MAX_SERVICES_NUM] = {0};
  ble_uuid_t                 sr_ma_uuids[BLE_APP_ADV_MAX_UUID_NUM] = {0};
  ble_uuid_t                 sr_cp_uuids[BLE_APP_ADV_MAX_UUID_NUM] = {0};
  ble_uuid_t                 sr_sl_uuids[BLE_APP_ADV_MAX_UUID_NUM] = {0};


  memset(&advdata, 0, sizeof(ble_advdata_t));
  memset(&srdata, 0, sizeof(ble_advdata_t));

  advdata.p_tx_power_level = &ad_tx_power_level;
  advdata.p_slave_conn_int = &ad_slave_conn_int;
  advdata.p_manuf_specific_data = &ad_manuf_specific_data;
  advdata.p_service_data_array = ad_service_data;
  advdata.uuids_more_available.p_uuids = ad_ma_uuids;
  advdata.uuids_complete.p_uuids = ad_cp_uuids;
  advdata.uuids_solicited.p_uuids = ad_sl_uuids;

  ble_adv_data_convert(&advdata, p_advdata);

  if (NULL == p_srdata)
    {
      p_sr_data = NULL;
    }
  else
    {
      srdata.p_tx_power_level = &sr_tx_power_level;
      srdata.p_slave_conn_int = &sr_slave_conn_int;
      srdata.p_manuf_specific_data = &sr_manuf_specific_data;
      srdata.p_service_data_array = sr_service_data;
      srdata.uuids_more_available.p_uuids = sr_ma_uuids;
      srdata.uuids_complete.p_uuids = sr_cp_uuids;
      srdata.uuids_solicited.p_uuids = sr_sl_uuids;

      ble_adv_data_convert(&srdata, p_srdata);
      p_sr_data = &srdata;
    }

  memset(&options, 0, sizeof(options));
  options.ble_adv_fast_enabled  = true;
  options.ble_adv_fast_interval = BLE_APP_ADV_INTERVAL_FAST;
  options.ble_adv_fast_timeout  = BLE_APP_ADV_TIMEOUT_FAST;

  err_code = ble_advertising_init(&advdata, p_sr_data, &options, on_adv_evt, NULL);
  BLE_APP_INTL_ERR_CHK(err_code);

  return err_code;
}

uint32_t ble_adv_start(ble_adv_params_t *p_adv_para)
{
  uint32_t err_code;
  ble_adv_modes_config_t options;


  ble_adv_parameter_check(p_adv_para);

  memset(&options, 0, sizeof(options));
  if (BLE_APP_ADV_FP_ANY == p_adv_para->fp)
    {
      options.ble_adv_whitelist_enabled = false;
    }
  else
    {
      options.ble_adv_whitelist_enabled = true;
    }


  // Set advertising parameters and events according to selected advertising mode.
  switch (p_adv_para->type)
    {
      case BLE_GAP_ADV_TYPE_ADV_IND:
        options.ble_adv_fast_enabled  = true;
        options.ble_adv_fast_interval = p_adv_para->interval;
        options.ble_adv_fast_timeout  = p_adv_para->timeout;
        adv_mode = BLE_ADV_MODE_FAST;
        break;

      case BLE_GAP_ADV_TYPE_ADV_DIRECT_IND:
        options.ble_adv_directed_slow_enabled  = true;
        options.ble_adv_directed_slow_interval = p_adv_para->interval;
        options.ble_adv_directed_slow_timeout  = p_adv_para->timeout;
        adv_mode = BLE_ADV_MODE_DIRECTED_SLOW;
        break;

      case BLE_GAP_ADV_TYPE_ADV_SCAN_IND:
        //Not support in NRF current code, need implement if need.
        break;

      case BLE_GAP_ADV_TYPE_ADV_NONCONN_IND:
        //Not support in NRF current code, need implement if need.
        break;

      default:
        break;
    }

  ble_adv_mode_set(&options);

  err_code = ble_advertising_start(adv_mode);
  return err_code;
}

uint32_t ble_adv_stop(void)
{
  return sd_ble_gap_adv_stop();
}

uint32_t ble_init()
{
  uint32_t err_code;
  pthread_t m_ble_event_thread;

  bt_mq_init();

  err_code = sem_init(&m_app_ble_ready, 0, 0);
  BLE_APP_INTL_ERR_CHK(err_code);

  err_code = ble_stack_init();
  BLE_APP_INTL_ERR_CHK(err_code);

#ifdef PEER_MANAGE_IN_USE
  bool     erase_bonds = true;
  err_code = peer_manager_init(erase_bonds);
  BLE_APP_INTL_ERR_CHK(err_code);
#endif

  err_code = gap_params_init();
  BLE_APP_INTL_ERR_CHK(err_code);

  err_code = conn_params_init();
  BLE_APP_INTL_ERR_CHK(err_code);

  err_code = pthread_create(&m_ble_event_thread, NULL, ble_event_thread, NULL);
  BLE_APP_INTL_ERR_CHK(err_code);

  return err_code;
}

