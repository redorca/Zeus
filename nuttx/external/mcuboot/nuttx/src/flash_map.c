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

#include <nuttx/progmem.h>
#include <errno.h>
#include <string.h>

#include <zdk_nrf52.h>
#include <flash_map/flash_map.h>
#include <hal/hal_flash.h>
#include <sysflash/sysflash.h>

#define BOOT_LOG_LEVEL BOOT_LOG_LEVEL_INFO
#include "bootutil/bootutil_log.h"

/*
 * For now, we only support boot from the chip internal flash.
 *
 * Pick a random device ID for it that's unlikely to collide with
 * anything "real".
 */
#define FLASH_MAP_ENTRY_MAGIC 0xd00dbeef

#define CONTAINER_OF(ptr, type, member)                 \
        (type *)((char *)ptr - offsetof(type, member))

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))


static int internal_open(uint8_t id, const struct flash_area **area);
static void internal_close(const struct flash_area *area);
static int internal_read(const struct flash_area *area, uint32_t off, void *dst,
                    uint32_t len);
static int internal_write(const struct flash_area *area, uint32_t off, const void *src,
                     uint32_t len);
static int internal_erase(const struct flash_area *area, uint32_t off, uint32_t len);
static int internal_get_sectors(int fa_id, uint32_t *cnt, struct flash_sector *ret);

struct flash_map_entry {
    const uint32_t          magic;
    const struct flash_area area;
    uint32_t                ref_count;
    const flash_ops         *flash_dev;
};

const flash_ops internal_flash =
{
  .dev_base       = FLASH_DEV_BASE_INTERNAL,
  .open           = internal_open,
  .close          = internal_close,
  .read           = internal_read,
  .write          = internal_write,
  .erase          = internal_erase,
  .get_sectors    = internal_get_sectors
};

#ifdef CONFIG_MCUBOOT_SECOND_IMAGE_QSPI
extern  const flash_ops qspi_flash;
#define SECOND_IMAGE_FLASH_OPS    &qspi_flash
#else
#define SECOND_IMAGE_FLASH_OPS    &internal_flash
#endif

/*
 * The flash area describes essentially the partition table of the
 * flash.  In this case, it starts with FLASH_AREA_IMAGE_0.
 */
static struct flash_map_entry part_map[] = {
    {
        .magic = FLASH_MAP_ENTRY_MAGIC,
        .area = {
            .fa_id = FLASH_AREA_IMAGE_0,
            .fa_device_id = FLASH_AREA_IMAGE_0_DEV_ID,
            .fa_off = FLASH_AREA_IMAGE_0_OFFSET,
            .fa_size = FLASH_AREA_IMAGE_0_SIZE,
        },
        .flash_dev = &internal_flash,
    },
    {
        .magic = FLASH_MAP_ENTRY_MAGIC,
        .area = {
            .fa_id = FLASH_AREA_IMAGE_1,
            .fa_device_id = FLASH_AREA_IMAGE_1_DEV_ID,
            .fa_off = FLASH_AREA_IMAGE_1_OFFSET,
            .fa_size = FLASH_AREA_IMAGE_1_SIZE,
        },
        .flash_dev = SECOND_IMAGE_FLASH_OPS,
    },
    {
        .magic = FLASH_MAP_ENTRY_MAGIC,
        .area = {
            .fa_id = FLASH_AREA_IMAGE_SCRATCH,
            .fa_device_id = FLASH_DEV_ID_INTERNAL,
            .fa_off = FLASH_AREA_IMAGE_SCRATCH_OFFSET,
            .fa_size = FLASH_AREA_IMAGE_SCRATCH_SIZE,
        },
        .flash_dev = &internal_flash,
    }
};


/*
 * Retrieve a memory-mapped flash device's base address.
 *
 * On success, the address will be stored in the value pointed to by
 * ret.
 *
 * Returns 0 on success, or an error code on failure.
 */

int flash_device_base(uint8_t fa_device_id, uintptr_t *ret)
{

    for (int i = 0; i < sizeof(part_map)/sizeof(part_map[0]); i++) {
        if (fa_device_id == part_map[i].area.fa_device_id) {
            *ret = part_map[i].flash_dev->dev_base;
            return 0;
        }
    }

    BOOT_LOG_ERR("Can't Find (%d) device base\n", fa_device_id);

    return -1;
}


/*
 * `open` a flash area.  The `area` in this case is not the individual
 * sectors, but describes the particular flash area in question.
 */
static int internal_open(uint8_t id, const struct flash_area **area)
{
    return 0;
}

int flash_area_open(uint8_t id, const struct flash_area **area)
{
    int slot ;
    struct flash_map_entry *entry;

    BOOT_LOG_DBG("%s: area %d\n", __FUNCTION__, id);

    slot = flash_area_id_to_image_slot(id);
    entry = &part_map[slot];

    if(0 == entry->ref_count) {
      entry->flash_dev->open(id, area);
    }
    entry->ref_count++;
    *area = &entry->area;

    return 0;
}


static void internal_close(const struct flash_area *area)
{
    return;
}

void flash_area_close(const struct flash_area *area)
{
    int slot;
    struct flash_map_entry *entry;

    BOOT_LOG_DBG("%s: area id %d\n", __FUNCTION__, area->fa_id);

    slot = flash_area_id_to_image_slot(area->fa_id);
    entry = &part_map[slot];

    if (0 == entry->ref_count) {
        BOOT_LOG_ERR("area %u use count underflow\n", area->fa_id);
        return;
    }

    entry->ref_count--;
    if(0 == entry->ref_count)
        entry->flash_dev->close(area);

}

static int internal_read(const struct flash_area *area, uint32_t off, void *dst,
                    uint32_t len)
{
    if ((off+len) <= area->fa_size) {
        memcpy(dst, (void *)(area->fa_off + off), len);
        return 0;
    } else {
        BOOT_LOG_ERR("Reading data out of Flash. fa_size:%#x, len:%x\n", area->fa_size, (off + len));
        return -EINVAL;
    }

}

int flash_area_read(const struct flash_area *area, uint32_t off, void *dst,
                    uint32_t len)
{
    int slot;

    BOOT_LOG_DBG("Read: area=%d, off=%x, len=%x", area->fa_id, off, len);
    slot = flash_area_id_to_image_slot(area->fa_id);

    return part_map[slot].flash_dev->read(area, off, dst, len);

}

static int internal_write(const struct flash_area *area, uint32_t off, const void *src,
                     uint32_t len)
{
    int rc;

    rc = up_progmem_write(area->fa_off + off, src, len);
    if (rc == len) {
        rc =0;
    }

    return rc;

}
int flash_area_write(const struct flash_area *area, uint32_t off, const void *src,
                     uint32_t len)
{
    int slot;

    BOOT_LOG_DBG("Write: area=%d, off=%x, len=%x\n", area->fa_id, off, len);
    slot = flash_area_id_to_image_slot(area->fa_id);

    return part_map[slot].flash_dev->write(area, off, src, len);

}

static int internal_erase(const struct flash_area *area, uint32_t off, uint32_t len)
{
    int rc = 0;
    uint32_t erased_page_no;
    uint32_t page_size;
    uint32_t erased_byte_count = 0;

    /*get the page number of the first page to be erased*/
    erased_page_no = up_progmem_getpage(area->fa_off + off);
    while (erased_byte_count < len) {
        /*erase one page*/
        rc = up_progmem_erasepage(erased_page_no);
        /*get the page size*/
        page_size = up_progmem_pagesize(erased_page_no);
        /*check the erase is success or not*/
        if(rc != page_size)
          return rc;
        /*if success, add page size to total counter*/
        erased_byte_count += page_size;
        /*prepare to erase next page*/
        erased_page_no ++;
    }

    return 0;

}

int flash_area_erase(const struct flash_area *area, uint32_t off, uint32_t len)
{
    int slot;

    BOOT_LOG_DBG("Erase: area=%d, off=%x, len=%x\n", area->fa_id, off, len);
    slot = flash_area_id_to_image_slot(area->fa_id);

    return part_map[slot].flash_dev->erase(area, off, len);

}

uint8_t flash_area_align(const struct flash_area *area)
{
    return hal_flash_align(area->fa_id);
}


typedef CODE void (*flash_page_cb)(int idx, uint32_t off, uint32_t size, void **ret);

/*
 * Lookup the sector map for a given flash area.  This should fill in
 * `ret` with all of the sectors in the area.  `*cnt` will be set to
 * the storage at `ret` and should be set to the final number of
 * sectors in this area.
 */
static int flash_area_layout(int fa_id, uint32_t *cnt, void *ret, flash_page_cb cb)
{
    uint32_t area_off = 0;
    uint32_t area_len = 0;
    uint32_t max_cnt = *cnt;
    uint32_t record_len = 0;
    uint32_t page_no;
    uint32_t page_size;

    if (*cnt < 1) {
        return -EINVAL;
    }

    flash_map_get_offset_image_len(fa_id, &area_off, &area_len);

    *cnt = 0;

    while (record_len < area_len && *cnt < max_cnt) {
        /*get the size of the page to be handled*/
        page_no = up_progmem_getpage(area_off + record_len);
        page_size = up_progmem_pagesize(page_no);

        /*make sure we will not cross the area boundary*/
        if ((page_size + record_len) > area_len) {
            BOOT_LOG_ERR("area %d size 0x%x could not be devided to sectors\n",
                     fa_id, area_len);
            return -EINVAL;
        }

        /*record the page information(page offset and page size)*/
        cb(fa_id, area_off + record_len, page_size, &ret);

        /*prepare to hanlde the next page*/
        *cnt = *cnt + 1;
        record_len += page_size;
    }

    /*if we exit from above loop because flash_sector array is run out*/
    if (*cnt >= max_cnt) {
        BOOT_LOG_ERR("flash area %d sector count overflow\n", fa_id);
        return -EINVAL;
    }

    return 0;
  }

/*
 * Write page information(page offset and page size) to flash_sector struct and
 * move the write destination pointer
 */
static void get_sectors_cb(int fa_id, uint32_t off, uint32_t size, void **ret)
{
    struct flash_sector *ptr = (struct flash_sector *)(*ret);
    ptr->fs_off = off;
    ptr->fs_size = size;
    *ret += sizeof(struct flash_sector);
}

static int internal_get_sectors(int fa_id, uint32_t *cnt, struct flash_sector *ret)
{
    return flash_area_layout(fa_id, cnt, (void *)ret, get_sectors_cb);

}

int flash_area_get_sectors(int fa_id, uint32_t *cnt, struct flash_sector *ret)
{
    int slot;

    BOOT_LOG_DBG("Get_Sector: area=%d \n", fa_id);

    slot = flash_area_id_to_image_slot(fa_id);

    return part_map[slot].flash_dev->get_sectors(fa_id, cnt, ret);
}

#ifndef MCUBOOT_USE_FLASH_AREA_GET_SECTORS
/*
 * Write page information(page offset and page size) to flash_area struct and
 * move the write destination pointer
 */
static void to_sectors_cb(int fa_id, uint32_t off, uint32_t size, void **ret)
{
    struct flash_area *ptr = (struct flash_area *)(*ret);
    ptr->fa_id = fa_id;
    ptr->fa_device_id = 0;
    ptr->pad16 = 0;
    ptr->fa_off = off;
    ptr->fa_size = size;
    *ret += sizeof(struct flash_area);
}


int flash_area_to_sectors(int fa_id, int *cnt, struct flash_area *ret)
{
    int rc;

    if(*cnt < 0)
      rc = -EINVAL;
    else
      rc = flash_area_layout(fa_id, (uint32_t *)cnt, (void *)ret, to_sectors_cb);

    return rc;
}
#endif

/*
 * This depends on the mappings defined in sysflash.h, and assumes
 * that slot 0, slot 1, and the scratch area area contiguous.
 */
int flash_area_id_from_image_slot(int slot)
{
    return slot + FLASH_AREA_IMAGE_0;
}

int flash_area_id_to_image_slot(int area_id)
{
    return area_id - FLASH_AREA_IMAGE_0;
}

int flash_map_get_offset_image_len(int fa_id, uint32_t *area_off, uint32_t *area_len)
{
    /*check input paraemter*/
    if (fa_id < FLASH_AREA_IMAGE_0 || fa_id > FLASH_AREA_IMAGE_SCRATCH) {
        return -EINVAL;
    }

    /*get the area offset and length*/
    *area_off = part_map[fa_id - FLASH_AREA_IMAGE_0].area.fa_off;
    *area_len = part_map[fa_id - FLASH_AREA_IMAGE_0].area.fa_size;

    return OK;
}

