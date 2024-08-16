/****************************************************************************
 *  wireless/bluetooth/ble_app_error.h
 *
 *   Copyright (C) 2007-2017 Zglue. All rights reserved.
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


#ifndef BLE_APP_ERROR_H__
#define BLE_APP_ERROR_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
 * Public Definitions
 ****************************************************************************/
#define BLE_APP_SUCCESS                      (0)

#define BLE_APP_ERROR_INVALID_PARAM               BLE_APP_SUCCESS+(1)
#define BLE_APP_ERROR_INVALID_LENGTH              BLE_APP_ERROR_INVALID_PARAM+(1)
#define BLE_APP_ERROR_INVALID_ADDR                BLE_APP_ERROR_INVALID_LENGTH+(1)
#define BLE_APP_ERROR_OUT_OF_MEM                  BLE_APP_ERROR_INVALID_ADDR+(1)
#define BLE_APP_ERROR_NO_ENOUGH_PERMISSON         BLE_APP_ERROR_OUT_OF_MEM+(1)
#define BLE_APP_ERROR_TIMEOUT                     BLE_APP_ERROR_NO_ENOUGH_PERMISSON+(1)
#define BLE_APP_ERROR_EXCEED_MAX_CONN_COUNT       BLE_APP_ERROR_TIMEOUT+(1)
#define BLE_APP_ERROR_BUSY                        BLE_APP_ERROR_EXCEED_MAX_CONN_COUNT+(1)
#define BLE_APP_ERROR_NOT_SUPPORTED               BLE_APP_ERROR_BUSY+(1)
#define BLE_APP_ERROR_OPEN_FILE_FAILED            BLE_APP_ERROR_NOT_SUPPORTED+(1)
#define BLE_APP_ERROR_FROM_PLATFORM               BLE_APP_ERROR_OPEN_FILE_FAILED+(1)

typedef struct
{
  uint16_t        line_num;
  uint8_t const *p_file_name;
  uint32_t        err_code;
} ble_app_err_info_t;

#define BLE_APP_ERROR_CHECK(ERR_CODE)                         \
      do                                                      \
      {                                                       \
          const uint32_t LOCAL_ERR_CODE = (ERR_CODE);         \
          if (LOCAL_ERR_CODE != BLE_APP_SUCCESS)              \
          {                                                   \
              ble_app_err_info_t info;                        \
              info.line_num = __LINE__;                       \
              info.p_file_name = (uint8_t*) __FILE__;         \
              info.err_code = LOCAL_ERR_CODE;                 \
              ble_app_set_error_info(&info);                  \
              _err("\n ERROR: %s On file %s, line %d",        \
              ble_app_get_err_str(LOCAL_ERR_CODE),            \
              info.p_file_name,info.line_num);                \
              return ERR_CODE;                                \
          }                                                   \
      } while (0)

#define BLE_APP_ERROR_CHECK_NO_RETURN(ERR_CODE)               \
      do                                                      \
      {                                                       \
          const uint32_t LOCAL_ERR_CODE = (ERR_CODE);         \
          if (LOCAL_ERR_CODE != BLE_APP_SUCCESS)              \
          {                                                   \
              _err("\n ERROR: %s On file %s, line %d",        \
              ble_app_get_err_str(LOCAL_ERR_CODE),            \
              __FILE__,__LINE__);                             \
          }                                                   \
      } while (0)

#ifdef __cplusplus
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: ble_app_set_error_info
 *
 * Description:
 *   BLE set error info for application layer.
 *
 * Input Parameters:
 *   ble_app_err_info_t * info
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
void ble_app_set_error_info(ble_app_err_info_t *info);

/****************************************************************************
 * Name: ble_app_get_error_info
 *
 * Description:
 *   BLE get error info of application layer.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   ble_app_err_info_t: Error infomation.
 *
 ****************************************************************************/
ble_app_err_info_t *ble_app_get_error_info(void);

/****************************************************************************
 * Name: ble_app_get_err_str
 *
 * Description:
 *   BLE get error string message.
 *
 * Input Parameters:
 *   uint32_t ： err_code
 *
 * Returned Value:
 *   uint8_t * ： String message
 *
 ****************************************************************************/

const char *ble_app_get_err_str(uint32_t err_code);

#endif // BLE_GAP_ERROR_H__

