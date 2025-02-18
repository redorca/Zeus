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

/**
 * @file
 *
 * @defgroup ble_conn_state Connection state
 * @ingroup ble_sdk_lib
 * @{
 * @brief Module for storing data on BLE connections.
 *
 * @details This module stores certain states for each connection, which can be queried by
 *          connection handle. The module uses BLE events to keep the states updated.
 *
 *          In addition to the preprogrammed states, this module can also keep track of a number of
 *          binary user states, or <i>user flags</i>. These are reset to 0 for new connections, but
 *          otherwise not touched by this module.
 *
 *          This module uses the @ref sdk_mapped_flags module, with connection handles as keys and
 *          the connection states as flags.
 *
 * @note A connection handle is not immediately invalidated when it is disconnected. Certain states,
 *       such as the role, can still be queried until the next time a new connection is established
 *       to any device.
 *
 *          To function properly, this module must be provided with BLE events from the SoftDevice
 *          through the @ref ble_conn_state_on_ble_evt() function. This module should be the first
 *          to receive BLE events if they are dispatched to multiple modules.
 */

#ifndef BLE_CONN_STATE_H__
#define BLE_CONN_STATE_H__

#include <stdbool.h>
#include <stdint.h>
#include "ble.h"
#include "sdk_mapped_flags.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief Connection handle statuses.
 */
typedef enum
{
  BLE_CONN_STATUS_INVALID,       /**< The connection handle is invalid. */
  BLE_CONN_STATUS_DISCONNECTED,  /**< The connection handle refers to a connection that has been disconnected, but not yet invalidated. */
  BLE_CONN_STATUS_CONNECTED,     /**< The connection handle refers to an active connection. */
} ble_conn_state_status_t;

#define BLE_CONN_STATE_N_USER_FLAGS 24  /**< The number of available user flags. */


/**@brief One ID for each user flag collection.
 *
 * @details These IDs are used to identify user flag collections in the API calls.
 */
typedef enum
{
  BLE_CONN_STATE_USER_FLAG0 = 0,
  BLE_CONN_STATE_USER_FLAG1,
  BLE_CONN_STATE_USER_FLAG2,
  BLE_CONN_STATE_USER_FLAG3,
  BLE_CONN_STATE_USER_FLAG4,
  BLE_CONN_STATE_USER_FLAG5,
  BLE_CONN_STATE_USER_FLAG6,
  BLE_CONN_STATE_USER_FLAG7,
  BLE_CONN_STATE_USER_FLAG8,
  BLE_CONN_STATE_USER_FLAG9,
  BLE_CONN_STATE_USER_FLAG10,
  BLE_CONN_STATE_USER_FLAG11,
  BLE_CONN_STATE_USER_FLAG12,
  BLE_CONN_STATE_USER_FLAG13,
  BLE_CONN_STATE_USER_FLAG14,
  BLE_CONN_STATE_USER_FLAG15,
  BLE_CONN_STATE_USER_FLAG16,
  BLE_CONN_STATE_USER_FLAG17,
  BLE_CONN_STATE_USER_FLAG18,
  BLE_CONN_STATE_USER_FLAG19,
  BLE_CONN_STATE_USER_FLAG20,
  BLE_CONN_STATE_USER_FLAG21,
  BLE_CONN_STATE_USER_FLAG22,
  BLE_CONN_STATE_USER_FLAG23,
  BLE_CONN_STATE_USER_FLAG_INVALID,
} ble_conn_state_user_flag_id_t;


/**
 * @defgroup ble_conn_state_functions BLE connection state functions
 * @{
 */


/**@brief Function for initializing or resetting the module.
 *
 * @details This function sets all states to their default, removing all records of connection handles.
 */
void ble_conn_state_init(void);


/**@brief Function for providing BLE SoftDevice events to the connection state module.
 *
 * @param[in]  p_ble_evt    The SoftDevice event.
 */
void ble_conn_state_on_ble_evt(ble_evt_t *p_ble_evt);


/**@brief Function for querying whether a connection handle represents a valid connection.
 *
 * @details A connection might be valid and have a BLE_CONN_STATUS_DISCONNECTED status.
 *          Those connections are invalidated after a new connection occurs.
 *
 * @param[in]  conn_handle  Handle of the connection.
 *
 * @retval true   If conn_handle represents a valid connection, thus a connection for which
                  we have a record.
 * @retval false  If conn_handle is @ref BLE_GAP_ROLE_INVALID, or if it has never been recorded.
 */
bool ble_conn_state_valid(uint16_t conn_handle);


/**@brief Function for querying the role of the local device in a connection.
 *
 * @param[in]  conn_handle  Handle of the connection to get the role for.
 *
 * @return  The role of the local device in the connection (see @ref BLE_GAP_ROLES).
 *          If conn_handle is not valid, the function returns BLE_GAP_ROLE_INVALID.
 */
uint8_t ble_conn_state_role(uint16_t conn_handle);


/**@brief Function for querying the status of a connection.
 *
 * @param[in]  conn_handle  Handle of the connection.
 *
 * @return  The status of the connection.
 *          If conn_handle is not valid, the function returns BLE_CONN_STATE_INVALID.
 */
ble_conn_state_status_t ble_conn_state_status(uint16_t conn_handle);


/**@brief Function for querying whether a connection is encrypted.
 *
 * @param[in]  conn_handle  Handle of connection to get the encryption state for.
 *
 * @retval true   If the connection is encrypted.
 * @retval false  If the connection is not encrypted or conn_handle is invalid.
 */
bool ble_conn_state_encrypted(uint16_t conn_handle);


/**@brief Function for querying whether a connection encryption is protected from Man in the Middle
 *        attacks.
 *
 * @param[in]  conn_handle  Handle of connection to get the MITM state for.
 *
 * @retval true   If the connection is encrypted with MITM protection.
 * @retval false  If the connection is not encrypted, or encryption is not MITM protected, or
 *                conn_handle is invalid.
 */
bool ble_conn_state_mitm_protected(uint16_t conn_handle);


/**@brief Function for querying the total number of connections.
 *
 * @return  The total number of valid connections for which the module has a record.
 */
uint32_t ble_conn_state_n_connections(void);


/**@brief Function for querying the total number of connections in which the role of the local
 *        device is @ref BLE_GAP_ROLE_CENTRAL.
 *
 * @return  The number of connections in which the role of the local device is
 *          @ref BLE_GAP_ROLE_CENTRAL.
 */
uint32_t ble_conn_state_n_centrals(void);


/**@brief Function for querying the total number of connections in which the role of the local
 *        device is @ref BLE_GAP_ROLE_PERIPH.
 *
 * @return  The number of connections in which the role of the local device is
 *          @ref BLE_GAP_ROLE_PERIPH.
 */
uint32_t ble_conn_state_n_peripherals(void);


/**@brief Function for obtaining a list of all connection handles for which the module has a record.
 *
 * @details This function takes into account connections whose state is BLE_CONN_STATUS_DISCONNECTED.
 *
 * @return  A list of all valid connection handles for which the module has a record.
 */
sdk_mapped_flags_key_list_t ble_conn_state_conn_handles(void);


/**@brief Function for obtaining a list of connection handles in which the role of the local
 *        device is @ref BLE_GAP_ROLE_CENTRAL.
 *
 * @details This function takes into account connections whose state is BLE_CONN_STATUS_DISCONNECTED.
 *
 * @return  A list of all valid connection handles for which the module has a record and in which
 *          the role of local device is @ref BLE_GAP_ROLE_CENTRAL.
 */
sdk_mapped_flags_key_list_t ble_conn_state_central_handles(void);


/**@brief Function for obtaining the handle for the connection in which the role of the local device
 *        is @ref BLE_GAP_ROLE_PERIPH.
 *
 * @details This function takes into account connections whose state is BLE_CONN_STATUS_DISCONNECTED.
 *
 * @return  A list of all valid connection handles for which the module has a record and in which
 *          the role of local device is @ref BLE_GAP_ROLE_PERIPH.
 */
sdk_mapped_flags_key_list_t ble_conn_state_periph_handles(void);


/**@brief Function for obtaining exclusive access to one of the user flag collections.
 *
 * @details The acquired collection contains one flag for each connection. These flags can be set
 *          and read individually for each connection.
 *
 *          The state of user flags will not be modified by the connection state module, except to
 *          set it to 0 for a connection when that connection is invalidated.
 *
 * @return  The ID of the acquired flag, or BLE_CONN_STATE_USER_FLAG_INVALID if none are available.
 */
ble_conn_state_user_flag_id_t ble_conn_state_user_flag_acquire(void);


/**@brief Function for reading the value of a user flag.
 *
 * @param[in]  conn_handle  Handle of connection to get the flag state for.
 * @param[in]  flag_id      Which flag to get the state for.
 *
 * @return  The state of the flag. If conn_handle is invalid, the function returns false.
 */
bool ble_conn_state_user_flag_get(uint16_t conn_handle, ble_conn_state_user_flag_id_t flag_id);


/**@brief Function for setting the value of a user flag.
 *
 * @param[in]  conn_handle  Handle of connection to set the flag state for.
 * @param[in]  flag_id      Which flag to set the state for.
 * @param[in]  value        Value to set the flag state to.
 */
void ble_conn_state_user_flag_set(uint16_t                      conn_handle,
                                  ble_conn_state_user_flag_id_t flag_id,
                                  bool                          value);


/**@brief Function for getting the state of a user flag for all connection handles.
 *
 * @details The returned collection can be used with the @ref sdk_mapped_flags API. The returned
 *          collection is a copy, so modifying it has no effect on the conn_state module.
 *
 * @param[in]  flag_id  Which flag to get states for.
 *
 * @return  The collection of flag states. The collection is always all zeros when the flag_id is
 *          unregistered.
 */
sdk_mapped_flags_t ble_conn_state_user_flag_collection(ble_conn_state_user_flag_id_t flag_id);

/** @} */
/** @} */


#ifdef __cplusplus
}
#endif

#endif /* BLE_CONN_STATE_H__ */
