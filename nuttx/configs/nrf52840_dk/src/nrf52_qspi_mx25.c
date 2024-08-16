/****************************************************************************
 * configs/nrf52832_dk/src/nrf52_spi_w25.c
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
#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>
#include <nuttx/drivers/drivers.h>

#if defined(CONFIG_FS_FAT) && !defined(CONFIG_BUILD_PROTECTED)
#include "fsutils/mkfatfs.h"
#endif
#include <arch/board/board.h>

#include "nrf.h"
#include "nrf52_qspi.h"

struct mtd_dev_s *zdk_platform_qspi_device(struct mtd_geometry_s *geo)
{

  struct qspi_dev_s *spi;
  FAR struct mtd_dev_s *mtd_block = NULL;

  /* mx25r spi flash is connected to QSPI bus */

  spi = nrf52_qspi_initialize(0);
  if (NULL == spi)
    {
      syslog(LOG_ERR, "QSPI Bus init  Error.\n");
      return NULL;
    }

#ifdef CONFIG_MTD_MX25RXX


  mtd_block = mx25rxx_initialize(spi, false);
  if (NULL == mtd_block)
    {
      syslog(LOG_ERR, "MX25R device initialize Error.\n");
      return NULL;
    }

  int ret = OK;

  ret = MTD_IOCTL(mtd_block, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)geo));
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: mtd->ioctl failed: %d\n", ret);
      return NULL;
    }

  syslog(LOG_INFO, "MX25RXX Flash Geometry:\n");
  syslog(LOG_INFO, "  blocksize:      %lu\n", (unsigned long)geo->blocksize);
  syslog(LOG_INFO, "  erasesize:      %lu\n", (unsigned long)geo->erasesize);
  syslog(LOG_INFO, "  neraseblocks:   %lu\n", (unsigned long)geo->neraseblocks);

#endif
  return mtd_block;
}

int nrf52_qspi_flash_fs_init(void)
{
  int ret = OK;

#ifdef CONFIG_MTD_MX25RXX

  FAR struct mtd_geometry_s geo;
  FAR struct mtd_dev_s *mtd_block;
  FAR struct mtd_dev_s *mtd_ftl;
  char blockdev[18];
  char chardev[12];

  mtd_block = zdk_platform_qspi_device(&geo);

  if (NULL == mtd_block)
    {
      syslog(LOG_WARNING, " MX25RXX MTD Device is NULL.\n");
      return -ENODEV;
    }
#ifdef MX25R_TEST

  uint8_t *p_buf = (uint8_t *)malloc(geo.erasesize);
  if (NULL == p_buf)
    {
      syslog(LOG_WARNING, "Can't malloc buffer for mx25r test.\n");
    }
  else
    {
      uint32_t nbytes;
      uint32_t start_block = 0;
      uint32_t end_block = 10;

      /* test erase command */

      syslog(LOG_WARNING, "Starting to Erase:\n");
      nbytes = MTD_ERASE(mtd_block, start_block, end_block);
      syslog(LOG_WARNING, "Erase block %d Done.\n", nbytes);

      /* Read Test to verify Erase */
      syslog(LOG_WARNING, "Starting to Reading:\n");

      memset(p_buf, 0, geo.erasesize);
      nbytes = MTD_READ(mtd_block, start_block * geo.blocksize, geo.blocksize, p_buf);
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

      nbytes = MTD_BWRITE(mtd_block, start_block, 1, p_buf);

      memset(p_buf, 0, geo.erasesize);
      nbytes = MTD_READ(mtd_block, start_block * geo.blocksize, geo.blocksize, p_buf);
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

#ifdef CONFIG_QSPI_FW_PARTITION
  /* convert from Byte to bit Unit */
  uint32_t end_block = QSPI_FW_SIZE / geo.blocksize;
  struct mtd_dev_s *mtd_fw = mtd_partition(mtd_block, 0, end_block);

#ifdef CONFIG_MTD_PARTITION_NAMES
  mtd_setpartitionname(mtd_fw, QSPI_FW_MTD_NAME);
#endif

  ret = ftl_initialize(QSPI_FW_MTD_MINOR, mtd_fw);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: MX25R Failed to initialize the FW FTL layer: %d\n", ret);
      return ret;
    }

  snprintf(blockdev, sizeof(blockdev) / sizeof(blockdev[0]),
           "%s%d", MTD_BLOCK_MOUNT, QSPI_FW_MTD_MINOR);
  snprintf(chardev, sizeof(blockdev) / sizeof(blockdev[0]),
           "%s%d", MTD_CHAR_MOUNT, QSPI_FW_MTD_MINOR);

  ret = bchdev_register(blockdev, chardev, false);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: bchdev_register %s failed: %d\n", chardev, ret);
      return ret;
    }

  /* the rest space as vFAT partition */
  uint32_t blkpererase = geo.erasesize / geo.blocksize;
  mtd_fw = mtd_partition(mtd_block, end_block, blkpererase * geo.neraseblocks - end_block);
  mtd_block = mtd_fw;
#ifdef CONFIG_MTD_PARTITION_NAMES
  mtd_setpartitionname(mtd_block, QSPI_F_FS_LABEL);
#endif

#endif

#if defined(CONFIG_MTD_SECT512) && (!defined(CONFIG_USBMSC))
  mtd_ftl = s512_initialize(mtd_block);
#else
  mtd_ftl = mtd_block;
#endif

  ret = ftl_initialize(QSPI_F_FS_MTD_MINOR, mtd_ftl);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: MX25R Failed to initialize the FTL layer: %d\n", ret);
      return ret;
    }

  /* Now create a character device on the block device */
  snprintf(blockdev, sizeof(blockdev) / sizeof(blockdev[0]),
           "%s%d", MTD_BLOCK_MOUNT, QSPI_F_FS_MTD_MINOR);
  snprintf(chardev, sizeof(blockdev) / sizeof(blockdev[0]),
           "%s%d", MTD_CHAR_MOUNT, QSPI_F_FS_MTD_MINOR);

  ret = bchdev_register(blockdev, chardev, false);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: bchdev_register %s failed: %d\n", chardev, ret);
      return ret;
    }

  mkdir(QSPI_F_MOUNT_POINT, 0777);

#if defined(CONFIG_FS_FAT) && (!defined(CONFIG_USBMSC))

  ret = mount(blockdev, QSPI_F_MOUNT_POINT, QSPI_F_FS_TYPE, 0, NULL);
  if (ret < 0)
    {

      syslog(LOG_NOTICE, "did not find valid fat filesystem on %s, ret : %d\n",
             blockdev, ret);

#if !defined(CONFIG_BUILD_PROTECTED)
      /* mkfatfs api is in user space layer , if enable PROTECT build ,
       * it should be called from user space
       */

      struct fat_format_s fmt = FAT_FORMAT_INITIALIZER;
      memcpy(fmt.ff_volumelabel, QSPI_F_FS_LABEL, QSPI_F_FS_LABEL_LEN);

      syslog(LOG_NOTICE, "Starting to create FAT on %s \n", blockdev);
      ret = mkfatfs(blockdev, &fmt);

      if (OK == ret)
        {
          mount(blockdev, QSPI_F_MOUNT_POINT, QSPI_F_FS_TYPE, 0, NULL);
        }
      else
        {
          syslog(LOG_ERR, "Can't create FAT filesystem on %s, ret: %d\n", blockdev, ret);
        }
#endif
    }
  else
    {
      syslog(LOG_INFO, "Mounted dev [%s]  on point [%s]FAT for vFAT.\n", blockdev, QSPI_F_MOUNT_POINT);
    }
#endif

#endif


  return ret;

}

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
#ifdef CONFIG_USBMSC
#ifndef CONFIG_SYSTEM_USBMSC_DEVMINOR1
#  define CONFIG_SYSTEM_USBMSC_DEVMINOR1 0
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_usbmsc_initialize
 *
 * Description:
 *   Perform architecture specific initialization of the USB MSC device.
 *
 ****************************************************************************/

int board_usbmsc_initialize(int port)
{
  /* If system/usbmsc is built as an NSH command, then SD slot should
   * already have been initialized in board_app_initialize()
   * In this case, there is nothing further to be done here.
   */

  return OK;
}
#endif

