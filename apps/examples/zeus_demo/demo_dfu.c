/****************************************************************************
 * examples/nimble/nimble_dfu_main.c
 *
 *   Copyright (C) 2007-2013, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <nuttx/config.h>

#include <sys/stat.h>
#include <stdint.h>
#include <stdio.h>
#include <sched.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <crc16.h>
#include <utils/easy_timer.h>
#include <hal/hal_bsp.h>
#include <hal/hal_timer.h>
#include <sys/boardctl.h>
#include <nuttx/board.h>
#include <arch/board/board.h>
#include <fcntl.h>

#include "host/ble_gap.h"
#include "host/ble_att.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "services/cus_inc/ble_svc_dfu.h"

#include "wireless/bluetooth/ble_app_api.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define BLE_ADV_MIN_ITVL         (160)                       /**Uint of 0.625ms . */
#define BLE_ADV_MAX_ITVL         (200)                       /**Uint of 0.625ms . */
#define BLE_INVALID_HANDLE       (0xff)

#define BLE_DFU_CONN_ITVL_MIN    (8)                         /**Uint of 1.25ms . */
#define BLE_DFU_CONN_ITVL_MAX    (8)                         /**Uint of 1.25ms . */

#define BLE_DFU_TIMEOUT          (500)                       /**Uint of ms . */

#ifdef CONFIG_BOARDCTL_IOCTL
#define BOARDIOC_IMG_VERSION_LEN    (BOARDIOC_MAX_STRLEN)
#define BOARDIOC_DEV_MODEL_LEN      (BOARDIOC_MAX_STRLEN)
#define BOARDIOC_DEV_NAME_LEN       (BOARDIOC_MAX_STRLEN)

typedef struct img_info
{
  uint8_t img_ver[BOARDIOC_IMG_VERSION_LEN];
  uint8_t model[BOARDIOC_DEV_MODEL_LEN];
  uint8_t sn[BOARDIOC_DEV_NAME_LEN];
  uint32_t img_size;
  uint16_t crc;
} img_info;

img_info dfu_img_info = {0};

#endif

#define BEL_DFU_IMAGE_PATH       "/dev/mtdblock4"
static int32_t image_fd = 0;

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static void dfu_read_local_info(uint8_t *data, uint16_t *len);
static void dfu_image_write(uint8_t *data, uint16_t len);
static void ble_dfu_reset(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/
static uint16_t dfu_conn_handle = BLE_INVALID_HANDLE;

ble_dfu_access_handler demo_dfu_handler =
{
  .read = &dfu_read_local_info,
  .write = &dfu_image_write,
  .reset = &ble_dfu_reset,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
extern void up_systemreset(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static void dfu_image_write(uint8_t *data, uint16_t len)
{
  int bytes;
  assert(image_fd != 0);
  bytes = write(image_fd, data, len);
  if (bytes < 0)
    {
      BLE_HS_LOG(ERROR, "Write image file %d failed: %d\n", image_fd, errno);
    }
}

static void dfu_read_local_info(uint8_t *data, uint16_t *len)
{

  memset(&dfu_img_info, 0, sizeof(img_info));

#ifdef CONFIG_BOARDCTL_IOCTL
  boardctl(BOARDIOC_G_VERSION, (uintptr_t)dfu_img_info.img_ver);
  syslog(LOG_WARNING, "FW_VERSION: %s, len = %d\n", dfu_img_info.img_ver, strlen((const char *)dfu_img_info.img_ver));

  boardctl(BOARDIOC_G_MODEL, (uintptr_t)dfu_img_info.model);
  syslog(LOG_WARNING, "MODEL: %s, len = %d\n", dfu_img_info.model, strlen((const char *)dfu_img_info.model));

  boardctl(BOARDIOC_G_DEVNAME, (uintptr_t)dfu_img_info.sn);
  syslog(LOG_WARNING, "DEV: %s, len = %d\n", dfu_img_info.sn, strlen((const char *)dfu_img_info.sn));
#endif

  dfu_img_info.img_size = CONFIG_SRAM_LENGTH;
  dfu_img_info.crc = crc16((FAR const uint8_t *)&dfu_img_info, sizeof(img_info) - sizeof(uint16_t));

  APP_LOG_WARNING("Image information: img_size = %d Size = [%d], FW_VERSION=[%s], DEV_name = [%s], SN = [%s]\n",
                  CONFIG_SRAM_LENGTH,
                  sizeof(dfu_img_info),
                  dfu_img_info.img_ver,
                  dfu_img_info.model,
                  dfu_img_info.sn);
  memcpy(data, (uint8_t *)&dfu_img_info, sizeof(dfu_img_info));
  *len = sizeof(dfu_img_info);

}

static void ble_dfu_reset(void)
{
  /*1. Disconnect */
  if (BLE_INVALID_HANDLE != dfu_conn_handle)
    {
      APP_LOG_WARNING("Terminate the connection %d ... \n", dfu_conn_handle);
      ble_gap_terminate(dfu_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }

  if (image_fd != 0)
    {
      close(image_fd);
    }
  /*2. Reset device */
  APP_LOG_WARNING("Reset device to updating... \n");
  up_systemreset();
}

/****************************************************************************
 * Pubilc Functions:
 ****************************************************************************/
void deom_dfu(void)
{
  /*Customer services of nimble*/
  ble_svc_dfu_init(&demo_dfu_handler);
}
