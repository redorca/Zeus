/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */
#include <nuttx/config.h>

#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdbool.h>
#include <fcntl.h>
#include <assert.h>
#include <nuttx/progmem.h>
#include <errno.h>
#include <string.h>
#include <nuttx/mtd/mtd.h>
#include <zdk_nrf52.h>
#include <flash_map/flash_map.h>
#include <hal/hal_flash.h>
#include <sysflash/sysflash.h>

#define BOOT_LOG_LEVEL BOOT_LOG_LEVEL_INFO
#include "bootutil/bootutil_log.h"


static struct mtd_dev_s *qspi_dev;
static struct mtd_geometry_s qspi_geo;

static uint32_t    g_buffer[256];
#define INTERNAL_BUF_LEN  sizeof(g_buffer)

#define QSPI_DEV   qspi_dev
#define SEC_SIZE   qspi_geo.erasesize
#define PAGE_SIZE  qspi_geo.blocksize

extern struct mtd_dev_s * zdk_platform_qspi_device(struct mtd_geometry_s *geo);
/*
 * Start using flash area.
 */
int qspi_flash_open(uint8_t id, const struct flash_area **area)
{
    if (NULL == QSPI_DEV) {
        QSPI_DEV = zdk_platform_qspi_device(&qspi_geo);
    }

    if (NULL == QSPI_DEV) {
        BOOT_LOG_ERR("Can't Open QSPI device !!!\n");
        return -1;
    }

    return 0;
}

void qspi_flash_close(const struct flash_area *erea)
{
    return;
}

/*
 * Read/write/erase. Offset is relative from beginning of flash area.
 */
int qspi_flash_read(const struct flash_area *area, uint32_t off, void *dst,
                    uint32_t len)
{
    ssize_t size, adj_len;
    uint8_t *internal_buf;

    /* NRF QSPI driver request : the read length should be multiple 4*/
    if (len < INTERNAL_BUF_LEN) {
        adj_len = (len + 0x3)&(~0x3);
        internal_buf = (uint8_t *)g_buffer;
    } else {
        adj_len = len;
        internal_buf = dst;
    }
    size = MTD_READ(QSPI_DEV, off, adj_len, internal_buf);

    if (size != adj_len)
      return size;

    if (len < INTERNAL_BUF_LEN)
      memcpy(dst, internal_buf, len);

    return OK;
}

int qspi_flash_write(const struct flash_area *area, uint32_t off, const void *src,
                     uint32_t len)
{
    ssize_t nblock;

    ASSERT(off % PAGE_SIZE == 0);

    nblock = MTD_BWRITE(QSPI_DEV, off/PAGE_SIZE, len / PAGE_SIZE, src);

    if (nblock != (len / PAGE_SIZE)) {
        BOOT_LOG_ERR("QSPI: Write block is %d\n", nblock);
        return -1;
    }

    return OK;
}

int qspi_flash_erase(const struct flash_area *area, uint32_t off, uint32_t len)
{

    ASSERT(0 == (len % SEC_SIZE));

    MTD_ERASE(QSPI_DEV, off/SEC_SIZE, len / SEC_SIZE);

    return 0;

}

int qspi_flash_get_sectors(int fa_id, uint32_t *cnt,
                           struct flash_sector *sectors)
{
    uint32_t area_off;
    uint32_t area_len;
    uint32_t max_cnt = *cnt;
    uint32_t record_len = 0;
    struct flash_sector *p_flash_sec = sectors;

    if (*cnt < 1) {
        return -EINVAL;
    }

    /*get the area offset and length*/
    flash_map_get_offset_image_len(fa_id, &area_off, &area_len);


    *cnt = 0;

    while (record_len < area_len && *cnt < max_cnt) {

        /*record the page information(page offset and page size)*/
        p_flash_sec->fs_off = record_len;
        p_flash_sec->fs_size = SEC_SIZE;
        p_flash_sec++;

        /*prepare to hanlde the next page*/
        *cnt = *cnt + 1;
        record_len += SEC_SIZE;
    }

    /*if we exit from above loop because flash_sector array is run out*/
    if (*cnt >= max_cnt) {
        BOOT_LOG_ERR("flash area %d sector count overflow\n", fa_id);
        return -EINVAL;
    }

    return 0;
}

const flash_ops qspi_flash =
{
  .dev_base           = FLASH_DEV_BASE_QSPI,
  .open               = qspi_flash_open,
  .close              = qspi_flash_close,
  .read               = qspi_flash_read,
  .write              = qspi_flash_write,
  .erase              = qspi_flash_erase,
  .get_sectors        = qspi_flash_get_sectors
};

