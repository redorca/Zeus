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


#ifndef PEER_DATA_STORAGE_H__
#define PEER_DATA_STORAGE_H__


#include <stdint.h>
#include "sdk_errors.h"
#include "ble_gap.h"
#include "peer_manager_types.h"
#include "peer_manager_internal.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @cond NO_DOXYGEN
 * @defgroup peer_data_storage Peer Data Storage
 * @ingroup peer_manager
 * @{
 * @brief An internal module of @ref peer_manager. This module provides a Peer Manager-specific API
 *        to the persistent storage.
 *
 * @details This module uses Flash Data Storage (FDS) to interface with persistent storage.
 */

#define PDS_PREPARE_TOKEN_INVALID     (0)       /**< Invalid value for prepare token. */
#define PDS_FIRST_RESERVED_FILE_ID    (0xC000)  /**< The beginning of the range of file IDs reserved for Peer Manager. */
#define PDS_LAST_RESERVED_FILE_ID     (0xFFFE)  /**< The end of the range of file IDs reserved for Peer Manager. */
#define PDS_FIRST_RESERVED_RECORD_KEY (0xC000)  /**< The beginning of the range of record keys reserved for Peer Manager. */
#define PDS_LAST_RESERVED_RECORD_KEY  (0xFFFE)  /**< The end of the range of record keys reserved for Peer Manager. */

#define PEER_ID_TO_FILE_ID            ( PDS_FIRST_RESERVED_FILE_ID)    //!< Macro for converting a @ref pm_peer_id_t to an FDS file ID.
#define FILE_ID_TO_PEER_ID            (-PDS_FIRST_RESERVED_FILE_ID)    //!< Macro for converting an FDS file ID to a @ref pm_peer_id_t.
#define DATA_ID_TO_RECORD_KEY         ( PDS_FIRST_RESERVED_RECORD_KEY) //!< Macro for converting a @ref pm_peer_data_id_t to an FDS record ID.
#define RECORD_KEY_TO_DATA_ID         (-PDS_FIRST_RESERVED_RECORD_KEY) //!< Macro for converting an FDS record ID to a @ref pm_peer_data_id_t.


/**@brief The events that come from this module.
 */
typedef enum
{
  PDS_EVT_STORED,                 //!< The specified data has been successfully stored.
  PDS_EVT_UPDATED,                //!< The specified data has been successfully updated.
  PDS_EVT_CLEARED,                //!< The specified data has been successfully cleared.
  PDS_EVT_ERROR_STORE,            //!< The specified data could not be stored.
  PDS_EVT_ERROR_UPDATE,           //!< The specified data could not be updated.
  PDS_EVT_ERROR_CLEAR,            //!< The specified data could not be cleared.
  PDS_EVT_PEER_ID_CLEAR,          //!< The peer id has been successfully cleared.
  PDS_EVT_ERROR_PEER_ID_CLEAR,    //!< The peer id has been successfully cleared.
  PDS_EVT_COMPRESSED,             //!< A compress procedure has finished successfully.
  PDS_EVT_ERROR_UNEXPECTED,       //!< An unexpected, possibly fatal error occurred.
} pds_evt_id_t;


/**@brief The event structure for events generated by the this module.
 */
typedef struct
{
  pds_evt_id_t      evt_id;      /**< The type of event. */
  pm_peer_id_t      peer_id;     /**< The peer the event pertains to. */
  pm_peer_data_id_t data_id;     /**< The data the event pertains to. */
  pm_store_token_t
  store_token; /**< A unique identifier for the operation. Can be compare to the token received when starting the operation. */
  ret_code_t        result;      /**< The result of the operation, or the unexpected error. */
} pds_evt_t;


/**@brief Event handler for events from the peer_data_storage module.
 *
 * @param[in]  event    The event that has happened.
 * @param[in]  peer_id  The id of the peer the event pertains to.
 * @param[in]  flags    The data the event pertains to.
 */
typedef void (*pds_evt_handler_t)(pds_evt_t const *p_event);


/**@brief Function for initializing the module.
 *
 * @retval NRF_SUCCESS             If initialization was successful.
 * @retval NRF_ERROR_STORAGE_FULL  If no flash pages were available for use.
 * @retval NRF_ERROR_INTERNAL      If the module couldn't register with the flash filesystem.
 */
ret_code_t pds_init(void);


/**@brief Function for reading peer data in flash.
 *
 * @param[in]  peer_id     The peer the data belongs to.
 * @param[in]  data_id     The data to retrieve.
 * @param[out] p_data      The peer data. May not be @c NULL.
 * @param[in]  p_buf_len   Length of the provided buffer, in bytes. Pass @c NULL to only copy
 *                         a pointer to the data in flash.
 *
 * @retval NRF_SUCCESS              If the operation was successful.
 * @retval NRF_ERROR_INVALID_PARAM  If @p peer_id or @p data_id are invalid.
 * @retval NRF_ERROR_NOT_FOUND      If the data was not found in flash.
 * @retval NRF_ERROR_NO_MEM         If the provided buffer is too small.
 */
ret_code_t pds_peer_data_read(pm_peer_id_t                    peer_id,
                              pm_peer_data_id_t               data_id,
                              pm_peer_data_t * const p_data,
                              uint32_t          const * const p_buf_len);


/**@brief Function to prepare iterating over peer data in flash using @ref pds_peer_data_iterate.
 *        Call this function once each time before iterating using @ref pds_peer_data_iterate.
 */
void pds_peer_data_iterate_prepare(void);


/**@brief Function for iterating peers' data in flash.
 *        Always call @ref pds_peer_data_iterate_prepare before starting iterating.
 *
 * @param[in]  data_id    The peer data to iterate over.
 * @param[out] p_peer_id  The peer the data belongs to.
 * @param[out] p_data     The peer data in flash.
 *
 * @retval true   If the operation was successful.
 * @retval false  If the data was not found in flash, or another error occurred.
 */
bool pds_peer_data_iterate(pm_peer_data_id_t            data_id,
                           pm_peer_id_t * const p_peer_id,
                           pm_peer_data_flash_t * const p_data);


/**@brief Function for reserving space in flash to store data.
 *
 * @param[in]  p_peer_data      The data to be stored in flash. Only data length and type (ID) are
 *                              relevant for this operation. May not be @c NULL.
 * @param[out] p_prepare_token  A token identifying the reserved space. May not be @c NULL.
 *
 * @retval NRF_SUCCESS               If the operation was successful.
 * @retval NRF_ERROR_INVALID_PARAM   If the data ID in @p p_peer_data is invalid.
 * @retval NRF_ERROR_INVALID_LENGTH  If data length exceeds the maximum allowed length.
 * @retval NRF_ERROR_STORAGE_FULL    If no space is available in flash.
 * @retval NRF_ERROR_INTERNAL        If an unexpected error occurred.
 */
ret_code_t pds_space_reserve(pm_peer_data_const_t const *p_peer_data,
                             pm_prepare_token_t          *p_prepare_token);


/**@brief Function for undoing a previous call to @ref pds_space_reserve.
 *
 * @param[in]  prepare_token  A token identifying the reservation to cancel.
 *
 * @retval NRF_SUCCESS          If the operation was successful.
 * @retval NRF_ERROR_INTERNAL   If an unexpected error occurred.
 */
ret_code_t pds_space_reserve_cancel(pm_prepare_token_t prepare_token);


/**@brief Function for storing peer data in flash. If the same piece of data already exists for the
 *        given peer, it will be updated. This operation is asynchronous.
 *        Expect a @ref PDS_EVT_STORED or @ref PDS_EVT_ERROR_STORE event.
 *
 * @param[in]  peer_id        The peer the data belongs to.
 * @param[in]  p_peer_data    The peer data. May not be @c NULL.
 * @param[in]  prepare_token  A token identifying the reservation made in flash to store the data.
 *                            Pass @ref PDS_PREPARE_TOKEN_INVALID if no space was reserved.
 * @param[out] p_store_token  A token identifying this particular store operation. The token can be
 *                            used to identify events pertaining to this operation. Pass @p NULL
 *                            if not used.
 *
 * @retval NRF_SUCCESS              If the operation was initiated successfully.
 * @retval NRF_ERROR_INVALID_PARAM  If @p peer_id or the data ID in @p_peer_data are invalid.
 * @retval NRF_ERROR_STORAGE_FULL   If no space is available in flash. This can only happen if
 *                                  @p p_prepare_token is @ref PDS_PREPARE_TOKEN_INVALID.
 * @retval NRF_ERROR_BUSY           If the flash filesystem was busy.
 * @retval NRF_ERROR_INTERNAL       If an unexpected error occurred.
 */
ret_code_t pds_peer_data_store(pm_peer_id_t                 peer_id,
                               pm_peer_data_const_t const *p_peer_data,
                               pm_prepare_token_t           prepare_token,
                               pm_store_token_t            *p_store_token);


/**@brief Function for deleting peer data in flash. This operation is asynchronous.
 *        Expect a @ref PDS_EVT_CLEARED or @ref PDS_EVT_ERROR_CLEAR event.
 *
 * @param[in]  peer_id  The peer the data belongs to
 * @param[in]  data_id  The data to delete.
 *
 * @retval NRF_SUCCESS              If the operation was initiated successfully.
 * @retval NRF_ERROR_INVALID_PARAM  If @p peer_id or @p data_id are invalid.
 * @retval NRF_ERROR_NOT_FOUND      If data was not found in flash.
 * @retval NRF_ERROR_BUSY           If the flash filesystem was busy.
 * @retval NRF_ERROR_INTERNAL       If an unexpected error occurred.
 */
ret_code_t pds_peer_data_delete(pm_peer_id_t peer_id, pm_peer_data_id_t data_id);


/**@brief Function for claiming an unused peer ID.
 *
 * @retval  PM_PEER_ID_INVALID  If no peer ID was available.
 */
pm_peer_id_t pds_peer_id_allocate(void);


/**@brief Function for freeing a peer ID and deleting all data associated with it in flash.
 *
 * @param[in]  peer_id  The ID of the peer to free.
 *
 * @retval NRF_SUCCESS             The operation was initiated successfully.
 * @retval NRF_ERROR_INVALID_PARAM If @p peer_id is invalid.
 */
ret_code_t pds_peer_id_free(pm_peer_id_t peer_id);


/**@brief Function for finding out whether a peer ID is in use.
 *
 * @param[in]  peer_id  The peer ID to inquire about.
 *
 * @retval  true   @p peer_id is in use.
 * @retval  false  @p peer_id is free.
 */
bool pds_peer_id_is_allocated(pm_peer_id_t peer_id);


/**@brief Function for getting the next peer ID in the sequence of all used peer IDs. Can be
 *        used to loop through all used peer IDs.
 *
 * @note @ref PM_PEER_ID_INVALID is considered to be before the first and after the last ordinary
 *       peer ID.
 *
 * @param[in]  prev_peer_id  The previous peer ID.
 *
 * @return  The first ordinary peer ID  If @p prev_peer_id is @ref PM_PEER_ID_INVALID.
 * @retval  PM_PEER_ID_INVALID          If @p prev_peer_id is the last ordinary peer ID or the module
 *                                      is not initialized.
 */
pm_peer_id_t pds_next_peer_id_get(pm_peer_id_t prev_peer_id);


/**@brief Function for getting the next peer ID in the sequence of all peer IDs pending deletion.
 *        Can be used to loop through all used peer IDs.
 *
 * @note @ref PM_PEER_ID_INVALID is considered to be before the first and after the last ordinary
 *       peer ID.
 *
 * @param[in]  prev_peer_id  The previous peer ID.
 *
 * @return  The next peer ID pending deletion.
 * @return  The first ordinary peer ID  if prev_peer_id was @ref PM_PEER_ID_INVALID.
 * @retval  PM_PEER_ID_INVALID          if prev_peer_id was the last ordinary peer ID or the module
 *                                      is not initialized.
 */
pm_peer_id_t pds_next_deleted_peer_id_get(pm_peer_id_t prev_peer_id);


/**@brief Function for querying the number of valid peer IDs available. I.E the number of peers
 *        in persistent storage.
 *
 * @return  The number of valid peer IDs.
 */
uint32_t pds_peer_count_get(void);


/** @}
 * @endcond
 */


#ifdef __cplusplus
}
#endif

#endif /* PEER_DATA_STORAGE_H__ */
