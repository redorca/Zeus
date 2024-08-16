/****************************************************************************
 * configs/nrf52832_dk/src/nrf52_internal_flash.c
 *
 *   Copyright (C) 2018 Zglue  Inc. All rights reserved.
 *           Levin Li     <zhiqiang@zglue.com>
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

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mount.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <stdlib.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/drivers/drivers.h>

#ifdef CONFIG_FS_FAT
#include "fsutils/mkfatfs.h"
#endif
#include <arch/board/board.h>

#include "nrf_nvmc.h"


/* This file will mcu internal flash usage initialize
 */

int nrf52_internal_flash_fs_initialize(void)
{
  int ret = OK;

#ifdef CONFIG_MTD_PROGMEM

  FAR struct mtd_dev_s *mtd;
  FAR struct mtd_geometry_s geo;
  FAR struct mtd_dev_s *mtd_partiton;
  char blockdev[18];
  char chardev[12];
  uint32_t flash_size;
  uint32_t start_address;
  uint32_t end_address;
  uint32_t data_size;
  uint32_t first_block = 0;

  /* Create an instance of the NRF52 FLASH program memory device driver */

  flash_size = nrf_nvmc_get_flash_size();
  printf("Internal Flash %#x,  RAM %#x.\n", flash_size, nrf_nvmc_get_ram_size());

  mtd = progmem_initialize();
  if (!mtd)
    {
      syslog(LOG_ERR, "ERROR: progmem_initialize failed\n");
      return -EINVAL;
    }

  /* Use the FTL layer to wrap the MTD driver as a block driver */

  ret = ftl_initialize(PROGMEM_MTD_MINOR, mtd);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize the FTL layer: %d\n", ret);
      return ret;
    }

  /* Use the minor number to create device paths */

  snprintf(blockdev, 18, "%s%d", MTD_BLOCK_MOUNT, PROGMEM_MTD_MINOR);
  snprintf(chardev, 12, "%s%d", MTD_CHAR_MOUNT, PROGMEM_MTD_MINOR);

  /* Now create a character device on the block device */

  ret = bchdev_register(blockdev, chardev, false);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: bchdev_register %s failed: %d\n",
             chardev, ret);
      return ret;
    }

  /* get image information */

  start_address = system_image_start_address();
  end_address = system_image_ro_section_end();
  data_size = system_image_data_section_size();
  syslog(LOG_INFO, "Image: start_address: %#x, end_address: %#x , data size: %#x\n",
         start_address, end_address, data_size);

  /* Get the geometry of the FLASH device */

  ret = MTD_IOCTL(mtd, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&geo));
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: mtd->ioctl failed: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "Internal Flash Geometry:\n");
  syslog(LOG_INFO, "  blocksize:      %lu\n", (unsigned long)geo.blocksize);
  syslog(LOG_INFO, "  erasesize:      %lu\n", (unsigned long)geo.erasesize);
  syslog(LOG_INFO, "  neraseblocks:   %lu\n", (unsigned long)geo.neraseblocks);

  /* caculate internal flash file system start address */
  first_block = INTERNAL_FLASH_FS_SIZE;
  first_block = flash_size - first_block;

  if (first_block > flash_size || first_block < (end_address + data_size))
    {
      syslog(LOG_ERR, "Attempt to create partition is out of Flash Size.\n");
      return -1;
    }

  /* convert block from byte to block number */

  first_block /= geo.blocksize;
  uint32_t blkpererase = geo.erasesize / geo.blocksize;
#ifdef CONFIG_MTD_PARTITION

  mtd_partiton = mtd_partition(mtd, first_block, blkpererase * geo.neraseblocks - first_block);
  if (NULL == mtd_partiton)
    {
      syslog(LOG_ERR, "ERROR: mtd_partition failed. offset=%lu nblocks=%lu\n",
             (unsigned long)first_block, (unsigned long)blkpererase * geo.neraseblocks - first_block);
      return -ENODEV;
    }

#ifdef CONFIG_MTD_PARTITION_NAMES
  mtd_setpartitionname(mtd_partiton, INTERNAL_FLASH_FS_LABEL);
#endif

  /* Initialize to provide an FTL block driver on the MTD FLASH interface */

  snprintf(blockdev, sizeof(blockdev) / sizeof(blockdev[0]),
           "%s%d", MTD_BLOCK_MOUNT, PROGMEM_FS_MTD_MINOR);
  snprintf(chardev, sizeof(blockdev) / sizeof(blockdev[0]),
           "%s%d", MTD_CHAR_MOUNT, PROGMEM_FS_MTD_MINOR);

  ret = ftl_initialize(PROGMEM_FS_MTD_MINOR, mtd_partiton);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: ftl_initialize %s failed: %d\n", blockdev, ret);
      return ret;
    }

  /* Now create a character device on the block device */

  ret = bchdev_register(blockdev, chardev, false);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: bchdev_register %s failed: %d\n", chardev, ret);
      return ret;
    }

#if defined(CONFIG_FS_FAT) && defined(CONFIG_INTERNAL_FLASH_FILESYSTEM)

  ret = mount(blockdev, INTERNAL_FLASH_MOUNT_POINT, INTERNAL_FLASH_FS_TYPE, 0, NULL);
  if (ret < 0)
    {
      struct fat_format_s fmt = FAT_FORMAT_INITIALIZER;
//        fmt.ff_fattype = 32;
      memcpy(fmt.ff_volumelabel, INTERNAL_FLASH_FS_LABEL, INTERNAL_FLASH_FS_LABEL_LEN);

      syslog(LOG_NOTICE, "did not find valid fat filesystem on %s, ret : %d\n",
             blockdev, ret);

#if !defined(CONFIG_BUILD_PROTECTED)
      /* mkfatfs api is in user space layer , if enable PROTECT build ,
       * it should be called from user space
       */
      syslog(LOG_NOTICE, "Starting to create FAT on %s \n", blockdev);
      ret = mkfatfs(blockdev, &fmt);

      if (OK == ret)
        {
          mount(blockdev, INTERNAL_FLASH_MOUNT_POINT, INTERNAL_FLASH_FS_TYPE, 0, NULL);
          syslog(LOG_INFO, "Mounted dev [%s]  on point [%s]FAT for vFAT .111\n", blockdev,
                 INTERNAL_FLASH_MOUNT_POINT);
        }
      else
        {
          syslog(LOG_ERR, "Can't create FAT filesystem on %s, ret: %d\n", blockdev, ret);
        }
#endif
    }
  else
    {
      syslog(LOG_INFO, "Mounted dev [%s]  on point [%s]FAT for vFAT.\n", blockdev, INTERNAL_FLASH_MOUNT_POINT);
    }
#endif
#endif
#endif

  return ret;
}


