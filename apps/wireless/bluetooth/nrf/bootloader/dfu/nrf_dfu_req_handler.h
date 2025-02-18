/* Copyright (c) 2016 Nordic Semiconductor. All Rights Reserved.
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

/**@file
 *
 * @defgroup sdk_nrf_dfu_req_handler Request handling
 * @{
 * @ingroup  sdk_nrf_dfu
 */

#ifndef NRF_DFU_REQ_HANDLER_H__
#define NRF_DFU_REQ_HANDLER_H__

#include <stdint.h>
#include <stdbool.h>
#include "nrf_dfu_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**@brief DFU object types.
 */
typedef enum
{
  NRF_DFU_OBJ_TYPE_INVALID,                   /**< Invalid object type.*/
  NRF_DFU_OBJ_TYPE_COMMAND,                   /**< Command object packet.*/
  NRF_DFU_OBJ_TYPE_DATA,                      /**< Data object.*/
} nrf_dfu_obj_type_t;


/**@brief DFU request operation codes.
 *
 * @details The DFU transport layer creates request events of these types. The implementation of @ref nrf_dfu_req_handler_on_req handles requests of these types.
 */
typedef enum
{

  NRF_DFU_OBJECT_OP_NONE          = 0,        /**< No operation set. */
  NRF_DFU_OBJECT_OP_CREATE        = 1,        /**< Create operation. The length of the request indicates the required size. When called, the created object is selected. */
  NRF_DFU_OBJECT_OP_WRITE         = 2,        /**< Write operation. When called, offset and CRC of the selected object are reported back. */
  NRF_DFU_OBJECT_OP_EXECUTE       = 3,        /**< Execute operation. When called, the selected object is executed. */
  NRF_DFU_OBJECT_OP_CRC           = 4,        /**< Calculate checksum operation. When called, offset and CRC of the selected object are reported back. */
  NRF_DFU_OBJECT_OP_SELECT        = 6,        /**< Select operation. When called, the object of the given type is selected, and information about the object is reported back. */
  NRF_DFU_OBJECT_OP_OTHER         = 7,        /**< A user-defined DFU request type. The application must define how to interpret the request. */
} nrf_dfu_req_op_t;


/**@brief DFU request result codes.
 *
 * @details The DFU transport layer creates request events of types @ref nrf_dfu_req_op_t,
 * which are handled by @ref nrf_dfu_req_handler_on_req. That functions returns one of these result codes.
 */
typedef enum
{
  NRF_DFU_RES_CODE_INVALID                 = 0x00,     /**< Invalid opcode. */
  NRF_DFU_RES_CODE_SUCCESS                 = 0x01,     /**< Operation successful. */
  NRF_DFU_RES_CODE_OP_CODE_NOT_SUPPORTED   = 0x02,     /**< Opcode not supported. */
  NRF_DFU_RES_CODE_INVALID_PARAMETER       = 0x03,     /**< Missing or invalid parameter value. */
  NRF_DFU_RES_CODE_INSUFFICIENT_RESOURCES  = 0x04,     /**< Not enough memory for the data object. */
  NRF_DFU_RES_CODE_INVALID_OBJECT          = 0x05,     /**< Data object does not match the firmware and hardware requirements, the signature is missing, or parsing the command failed. */
  NRF_DFU_RES_CODE_UNSUPPORTED_TYPE        = 0x07,     /**< Not a valid object type for a Create request. */
  NRF_DFU_RES_CODE_OPERATION_NOT_PERMITTED = 0x08,     /**< The state of the DFU process does not allow this operation. */
  NRF_DFU_RES_CODE_OPERATION_FAILED        = 0x0A,     /**< Operation failed. */
  NRF_DFU_RES_CODE_EXT_ERROR               = 0x0B,     /**< Extended error. */
} nrf_dfu_res_code_t;


#if defined(__CC_ARM)
#  pragma push
#  pragma anon_unions
#elif defined(__ICCARM__)
#  pragma language=extended
#elif defined(__GNUC__)
// Anonymous unions are enabled by default.
#endif


/** @brief Definition of a DFU request sent from the transport layer.
 *
 * @details When the transport layer gets a DFU event, it calls the function @ref nrf_dfu_req_handler_on_req to handle the DFU request.
 */
typedef struct
{
  nrf_dfu_req_op_t    req_type;       /**< Request operation type. */

  union
  {
    struct
    {
      uint32_t        obj_type;       /**< Object type of the object to be created for a request of type @ref NRF_DFU_OBJECT_OP_CREATE. */
      uint32_t
      object_size;    /**< Size of the object to be created for a request of type @ref NRF_DFU_OBJECT_OP_CREATE. Note that the object size is not the same as the size of the firmware. */
    };

    struct
    {
      uint8_t    *p_req;      /**< Pointer to an array holding the serialized version of the request. */
      uint32_t    req_len;    /**< Length of the request array. */
    };
  };
} nrf_dfu_req_t;


/** @brief Response used during DFU operations.
 */
typedef struct
{
  union
  {
    struct
    {
      uint8_t        *p_res;          /**< Pointer to an array holding the serialized version of the response. */
      uint32_t        res_len;        /**< Length of the response array. */
    };

    struct
    {
      uint32_t        max_size;       /**< Maximum size of the object of a given type. */
      uint32_t        offset;         /**< Current offset. */
      uint32_t        crc;            /**< Current CRC. */
    };
  };
} nrf_dfu_res_t;

#if defined(__CC_ARM)
#  pragma pop
#elif defined(__ICCARM__)
// Leave anonymous unions enabled.
#elif defined(__GNUC__)
// Anonymous unions are enabled by default.
#endif


/** @brief Function for initializing the request handling module.
 *
 * @details This function initializes the flash with or without the SoftDevice, depending on the project configuration.
 *
 * @retval NRF_SUCCESS             If the operation was successful.
 * @retval NRF_ERROR_INVALID_STATE If the fstorage module could not be initiated or the SoftDevice could not set the event handler.
 */
uint32_t nrf_dfu_req_handler_init(void);


/** @brief  Function type for handling a DFU request.
 *
 * @param[in,out]   p_context   Pointer to context-specific RAM required for
 *                              running the command request.
 *                              This value may be NULL if the command request
 *                              does not require context-specific RAM.
 * @param[in,out]   p_req       Pointer to the structure holding the DFU request.
 * @param[in,out]   p_res       Pointer to the structure holding the DFU response.
 *
 * @retval NRF_DFU_RES_CODE_SUCCESS     If the command request was executed successfully.
 *                                      Any other error code indicates that the request
 *                                      could not be handled.
 */
nrf_dfu_res_code_t nrf_dfu_req_handler_on_req(void *p_context, nrf_dfu_req_t *p_req, nrf_dfu_res_t *p_res);


#ifdef __cplusplus
}
#endif

#endif // NRF_DFU_REQ_HANDLER_H__

/** @} */
