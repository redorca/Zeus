/****************************************************************************
 * apps/wireless/ble_app_error.c
 *
 *   Copyright (C) 2007-2017. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdio.h>
#include <unistd.h>

#include <wireless/bluetooth/ble_app_error.h>
/****************************************************************************
 * Private Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/
/****************************************************************************
 * Public Data
 ****************************************************************************/
ble_app_err_info_t ble_app_err_info;

/****************************************************************************
 * Private Data
 ****************************************************************************/
static const char *err_str[] =
{
  "Success.",
  "Invalid parameters.",
  "Invalid lenght.",
  "Invalid address.",
  "Out of memory.",
  "No enough permisson.",
  "Timeout.",
  "Exceed maximum connection count.",
  "System is busy.",
  "Not supported.",
  "Open file failed."
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
void ble_app_set_error_info(ble_app_err_info_t *info)
{
  ble_app_err_info.err_code = info->err_code;
  ble_app_err_info.line_num = info->line_num;
  ble_app_err_info.p_file_name = info->p_file_name;
}

ble_app_err_info_t *ble_app_get_error_info()
{
  return &ble_app_err_info;
}

const char *ble_app_get_err_str(uint32_t err_code)
{
  return err_str[err_code];
}

