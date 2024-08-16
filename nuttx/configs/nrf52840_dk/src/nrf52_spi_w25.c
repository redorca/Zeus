/****************************************************************************
 * configs/nrf52832_dk/src/nrf52_spi_w25.c
 *
 *   Copyright (C) 2018 Zglue  Inc. All rights reserved.
 *           Arjun Hary   <arjun@zglue.com>
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
#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>
#include <nuttx/drivers/drivers.h>

#ifdef CONFIG_FS_FAT
#include "fsutils/mkfatfs.h"
#endif
#include <arch/board/board.h>

#include "nrf.h"
#include "nrf52_spi.h"

/* This file will handle all spi flash related initialize ccode
 */
int nrf52_spi_flash_fs_initialize(void)
{
  int ret = OK;

#if defined(CONFIG_NRF52_SPI)

  /* w25 spi flash is connected to SPI2 bus */
#ifdef CONFIG_NRF52_SPI2
  struct spi_dev_s *spi;

  spi = nrf52_spibus_initialize(2, true);
  if (NULL == spi)
    {
      syslog(LOG_ERR, "SPI2 Bus init  Error.\n");
      return -ENODEV;
    }


#ifdef CONFIG_MTD_W25

  FAR struct mtd_geometry_s geo;
  FAR struct mtd_dev_s *mtd_block;
  FAR struct mtd_dev_s *mtd_ftl;
  char blockdev[18];
  char chardev[12];

  mtd_block = w25_initialize(spi);
  if (NULL == mtd_block)
    {
      syslog(LOG_ERR, "w25 device initialize Error.\n");
      return -ENODEV;
    }

  ret = mtd_block->ioctl(mtd_block, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&geo));
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: mtd->ioctl failed: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "W25 Flash Geometry:\n");
  syslog(LOG_INFO, "  blocksize:      %lu\n", (unsigned long)geo.blocksize);
  syslog(LOG_INFO, "  erasesize:      %lu\n", (unsigned long)geo.erasesize);
  syslog(LOG_INFO, "  neraseblocks:   %lu\n", (unsigned long)geo.neraseblocks);

#ifdef W25_TEST

  uint8_t *p_buf = (uint8_t *)malloc(geo.erasesize);
  if (NULL == p_buf)
    {
      syslog(LOG_WARNING, "Can't malloc buffer for w25 test.\n");
    }
  else
    {
      uint32_t nbytes;
      uint32_t start_block = 0;
      uint32_t end_block = 10;

      /* test erase command */

      syslog(LOG_WARNING, "Starting to Erase:\n");
      nbytes = mtd_block->erase(mtd_block, start_block, end_block);
      syslog(LOG_WARNING, "Erase block %d Done.\n", nbytes);

      /* Read Test to verify Erase */
      syslog(LOG_WARNING, "Starting to Reading:\n");

      memset(p_buf, 0, geo.erasesize);
      nbytes = mtd_block->read(mtd_block, start_block * geo.blocksize, geo.blocksize, p_buf);
      syslog(LOG_WARNING, "Reading %d bytes from block %d to block %d.\n",
             nbytes, start_block, 1);
      for (int i = 0; i < geo.blocksize; i++)
        {
          if (p_buf[i] != 0xFF)
            {
              syslog(LOG_WARNING, "Erasing Error at block %d, index %d, value %d.\n",
                     start_block, i, p_buf[i]);
            }
        }

      /* Write Test */

      syslog(LOG_WARNING, "Starting to write block size %d:\n", geo.blocksize);
      for (int i = 0; i < geo.blocksize; i++)
        {
          p_buf[i] = (uint8_t)i;
        }

      nbytes = mtd_block->write(mtd_block, start_block * geo.blocksize, geo.blocksize, p_buf);

      memset(p_buf, 0, geo.erasesize);
      nbytes = mtd_block->read(mtd_block, start_block * geo.blocksize, geo.blocksize, p_buf);
      syslog(LOG_WARNING, "Reading %d bytes from block %d to block %d.\n",
             nbytes, start_block, 1);

      for (int i = 0; i < geo.blocksize; i++)
        {
          if (p_buf[i] != (uint8_t)i)
            {
              syslog(LOG_WARNING, "Writing Error at block %d, index %d, expect value %d, actual value %d.\n",
                     start_block, i, (uint8_t)i, p_buf[i]);
            }
        }
      free(p_buf);
    }

#endif

  /* Initialize to provide an FTL block driver on the MTD FLASH interface */

  snprintf(blockdev, sizeof(blockdev) / sizeof(blockdev[0]),
           "%s%d", MTD_BLOCK_MOUNT, SPI_FLASH_FS_MTD_MINOR);
  snprintf(chardev, sizeof(blockdev) / sizeof(blockdev[0]),
           "%s%d", MTD_CHAR_MOUNT, SPI_FLASH_FS_MTD_MINOR);

#ifdef CONFIG_MTD_SECT512
  mtd_ftl = s512_initialize(mtd_block);
#else
  mtd_ftl = mtd_block;
#endif

  ret = ftl_initialize(SPI_FLASH_FS_MTD_MINOR, mtd_ftl);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: W25 Failed to initialize the FTL layer: %d\n", ret);
      return ret;
    }

  /* Now create a character device on the block device */

  ret = bchdev_register(blockdev, chardev, false);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: bchdev_register %s failed: %d\n", chardev, ret);
      return ret;
    }

  mkdir(SPI_FLASH_MOUNT_POINT, 0777);

#ifdef CONFIG_FS_SMARTFS

  ret = smart_initialize(SPI_FLASH_FS_SMARTFS_MINOR, mtd_block, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: smart_initialize failed: %d\n", ret);
      return ret;
    }

#elif defined(CONFIG_FS_FAT)

  ret = mount(blockdev, SPI_FLASH_MOUNT_POINT, SPI_FLASH_FS_TYPE, 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_NOTICE, "did not find valid fat filesystem on %s, ret : %d\n",
             blockdev, ret);

#if !defined(CONFIG_BUILD_PROTECTED)
      /* mkfatfs api is in user space layer , if enable PROTECT build ,
       * it should be called from user space
       */
      struct fat_format_s fmt = FAT_FORMAT_INITIALIZER;
//      fmt.ff_fattype = 32;
      memcpy(fmt.ff_volumelabel, SPI_FLASH_FS_LABEL, SPI_FLASH_FS_LABEL_LEN);

      syslog(LOG_NOTICE, "Starting to create FAT on %s \n", blockdev);
      ret = mkfatfs(blockdev, &fmt);

      if (OK == ret)
        {
          mount(blockdev, SPI_FLASH_MOUNT_POINT, SPI_FLASH_FS_TYPE, 0, NULL);
        }
      else
        {
          syslog(LOG_ERR, "Can't create FAT filesystem on %s, ret: %d\n", blockdev, ret);
        }
#endif
    }

#endif

#endif

#endif
#endif

  return ret;

}

