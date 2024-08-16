/****************************************************************************
 *   configs/fast_nrf52832_dk/src/nrf52_appinit.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mount.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <stdlib.h>
#include <sys/boardctl.h>

#include <nuttx/version.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/fs/nxffs.h>

#ifdef CONFIG_FS_FAT
#include "fsutils/mkfatfs.h"
#endif

#include "nrf.h"
#include <arch/board/board.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>
#include <nuttx/drivers/drivers.h>
#include "nrf52_i2c.h"

#include "nrf52_spi.h"

#include "nrf52_tim.h"
#include "nrf52_rtc.h"
#include "nrf52_wdg.h"
#include "nrf52_pwm.h"
#include "nrf52_qdec.h"
#include "chip/nrf52_tim.h"
#include "nrf52_ppi.h"
#include "nrf52_gpiote.h"
#include "nrf_nvmc.h"
#include "boards.h"


#ifdef CONFIG_MC3672
#include <nuttx/sensors/mc3672.h>
#endif

#ifdef CONFIG_MAX86140
#include <nuttx/sensors/max86140.h>
#endif

#ifdef CONFIG_SENSORS_TMP108
#include <nuttx/sensors/tmp108.h>
#endif

#if defined(CONFIG_SYSTEM_FAST_DRIVER)
#include <nuttx/drivers/zglue_fast.h>
#endif

#include "nrf52_adc.h"
#include "nrf52_comp.h"
#include "nrf52_uniqueid.h"

#ifdef CONFIG_NRF52_PROCFS_DEVID
#include "nrf52_procfs.h"
#endif

#ifdef HAVE_RTC_DRIVER
#  include <nuttx/timers/rtc.h>
#  include "nrf52_rtc.h"
#endif

#ifdef CONFIG_NRF52_JTAG
#include "nrf52_jtag.h"
#endif

#ifdef CONFIG_MCUBOOT
#include "bootutil/image.h"
#include "bootutil/bootutil.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/
#ifdef CONFIG_NRF52_CS_CONTROL_BY_USER
extern void customer_spi_cs_select(void);
#endif

#ifdef CONFIG_CODE_ANALYSIS
extern int zdk_code_analysis(void);
#endif

/****************************************************************************
 * Name: nrf52_i2c_register
 *
 * Description:
 *   Register one I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#ifdef CONFIG_SYSTEM_I2CTOOL
#if defined(CONFIG_NRF52_I2C0)||defined(CONFIG_NRF52_I2C1)
static void nrf52_i2c_register(int bus)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  i2c = nrf52_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
                 bus, ret);
          nrf52_i2cbus_uninitialize(i2c);
        }
    }
}
#endif
#endif


/****************************************************************************
 * Name: nrf52_i2ctool
 *
 * Description:
 *   Register I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#ifdef CONFIG_SYSTEM_I2CTOOL
static void nrf52_i2ctool(void)
{
  return;
#ifdef CONFIG_NRF52_I2C0
  nrf52_i2c_register(0);
#endif
#ifdef CONFIG_NRF52_I2C1
  nrf52_i2c_register(1);
#endif
}
#else
#  define nrf52_i2ctool()
#endif

int nrf52_proc_fs_initialize(void)
{
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = mount(NULL, PROCFS_MOUNT, PROCFS_TYPE, 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at  %s: %d\n",
             PROCFS_MOUNT, ret);
    }

#ifdef CONFIG_NRF52_PROCFS_DEVID
  ret = devid_procfs_register();
#endif

#endif

  return ret;
}

#ifdef CONFIG_BOARDCTL_IOCTL

int board_uniqueid(uint8_t *uniqueid)
{
  if (uniqueid == 0)
    {
      return -EINVAL;
    }

  nrf52_get_uniqueid(uniqueid);
  return OK;
}

int board_ioctl(unsigned int cmd, uintptr_t arg)
{
  int ret = OK;
  uint8_t uniqid[8];
  switch (cmd)
    {
      case BOARDIOC_G_VERSION:
        strncpy((char *)arg, CONFIG_VERSION_BUILD, BOARDIOC_MAX_STRLEN);
        break;
      case BOARDIOC_G_MODEL:
        strncpy((char *)arg, CONFIG_ARCH_BOARD, BOARDIOC_MAX_STRLEN);
        break;
      case BOARDIOC_G_DEVNAME:
        board_uniqueid(uniqid);
        snprintf((char *)arg, BOARDIOC_MAX_STRLEN, "%02x%02x%02x%02x:%02x%02x%02x%02x", \
                 uniqid[0], uniqid[1], uniqid[2], uniqid[3], \
                 uniqid[4], uniqid[5], uniqid[6], uniqid[7]);
        break;
      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}
#endif

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

  ret = mtd->ioctl(mtd, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&geo));
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: mtd->ioctl failed: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "Internal Flash Geometry:\n");
  syslog(LOG_INFO, "  blocksize:      %lu\n", (unsigned long)geo.blocksize);
  syslog(LOG_INFO, "  erasesize:      %lu\n", (unsigned long)geo.erasesize);
  syslog(LOG_INFO, "  neraseblocks:   %lu\n", (unsigned long)geo.neraseblocks);

#ifdef CONFIG_NRF52_BLUETOOTH
  first_block = SYSTEM_SOFTDEVICE_IMAGE_SIZE;
#endif

  first_block += SYSTEM_BOOTLOADER_IMAGE_SIZE;
  first_block += SYSTEM_APP_IMAGE_SIZE;

  if (first_block > flash_size || first_block < (end_address + data_size))
    {
      syslog(LOG_ERR, "Attempt to create partition is out of Flash Size.\n");
      return -1;
    }

  /* convert block from byte to block number */

  first_block /= geo.blocksize;
  uint32_t blkpererase = geo.erasesize / geo.blocksize;

  mtd_partiton = mtd_partition(mtd, first_block, blkpererase * geo.neraseblocks - first_block);
  if (NULL == mtd_partiton)
    {
      syslog(LOG_ERR, "ERROR: mtd_partition failed. offset=%lu nblocks=%lu\n",
             (unsigned long)first_block, (unsigned long)geo.neraseblocks - first_block);
      return -ENODEV;
    }

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

#if defined(CONFIG_FS_FAT)

  ret = mount(blockdev, INTERNAL_FLASH_MOUNT_POINT, INTERNAL_FLASH_FS_TYPE, 0, NULL);
  if (ret < 0)
    {
      struct fat_format_s fmt = FAT_FORMAT_INITIALIZER;
//        fmt.ff_fattype = 32;
      memcpy(fmt.ff_volumelabel, INTERNAL_FLASH_FS_LABEL, INTERNAL_FLASH_FS_LABEL_LEN);

      syslog(LOG_NOTICE, "did not find valid fat filesystem on %s, ret : %d\n",
             blockdev, ret);

      syslog(LOG_NOTICE, "Starting to create FAT on %s \n", blockdev);
      ret = mkfatfs(blockdev, &fmt);

      if (OK == ret)
        {
          mount(blockdev, INTERNAL_FLASH_MOUNT_POINT, INTERNAL_FLASH_FS_TYPE, 0, NULL);
        }
      else
        {
          syslog(LOG_ERR, "Can't create FAT filesystem on %s, ret: %d\n", blockdev, ret);
        }
    }
#endif
#endif

  return ret;
}

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initalization logic and the the
 *         matching application logic.  The value cold be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_LIB_BOARDCTL
int board_app_initialize(uintptr_t arg)
{
  int ret = OK;

  nrf52_internal_flash_fs_initialize();

#ifdef CONFIG_BOARDCTL_IOCTL
  char data[BOARDIOC_MAX_STRLEN];
  boardctl(BOARDIOC_G_VERSION, (uintptr_t)data);
  syslog(LOG_INFO, "Firmware V: %s\n", data);

  boardctl(BOARDIOC_G_MODEL, (uintptr_t)data);
  syslog(LOG_INFO, "Hardware M: %s\n", data);

  boardctl(BOARDIOC_G_DEVNAME, (uintptr_t)data);
  syslog(LOG_INFO, "Dev S/N: %s\n", data);
#endif

#if (CONFIG_FLASH_ORIGIN != 0)
  /* indicated there is bootloader , then confirm slot 0 as permanet */
  ret = boot_set_confirmed();
  _warn("Confirm Slot-0 as Permanet Image : %d.\n", ret);
#endif

#ifdef CONFIG_CODE_ANALYSIS
  zdk_code_analysis();
#endif

  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to do nrf clock init: %d\n",
             ret);
      return ret;
    }

#ifdef CONFIG_COUNTER
#ifdef CONFIG_NRF52_RTC0
  ret = nrf52_rtc_initialize("/dev/rtc0", 0);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the rtc driver: %d\n",
             ret);
      return ret;
    }
#endif
#ifdef CONFIG_NRF52_RTC1
  ret = nrf52_rtc_initialize("/dev/rtc1", 1);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the rtc driver: %d\n",
             ret);
      return ret;
    }
#endif
#ifdef CONFIG_NRF52_RTC2
  ret = nrf52_rtc_initialize("/dev/rtc2", 2);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the rtc driver: %d\n",
             ret);
      return ret;
    }
#endif
#endif

#ifdef CONFIG_TIMER
  /* Initialize and register the timer driver */

#ifdef CONFIG_NRF52_TIM0
  FAR struct nrf52_tim_dev_s *tim0;
  tim0 = nrf52_timer_initialize("/dev/timer0", 0);
  if (tim0 == NULL)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the timer driver: %d\n",
             -ENODEV);
      return -ENODEV;
    }
#endif

#ifdef CONFIG_NRF52_TIM1
  FAR struct nrf52_tim_dev_s *tim1;
  tim1 = nrf52_timer_initialize("/dev/timer1", 1);
  if (tim1 == NULL)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the timer driver: %d\n",
             -ENODEV);
      return -ENODEV;
    }
#endif

#endif

#if defined(CONFIG_SYSTEM_FAST_DRIVER)
  struct zg_fast_driver_s fastapi_driver;
  struct zg_fast_config_s fastapi_config;
  memset(&fastapi_driver, 0, sizeof(struct zg_fast_driver_s));
  memset(&fastapi_config, 0, sizeof(struct zg_fast_config_s));
#endif


#ifdef CONFIG_I2C

#ifdef CONFIG_NRF52_I2C0
  FAR struct i2c_master_s *i2c0;
  /* Initialize I2C */
  i2c0 = nrf52_i2cbus_initialize(0);
  if (!i2c0)
    {
      syslog(LOG_ERR, "ERROR: Failed to init the i2c0 driver\n");
      return -ENODEV;
    }
#ifdef CONFIG_SYSTEM_I2CTOOL
  /* Register I2C drivers on behalf of the I2C tool */
  i2c_register(i2c0, 0);
#endif

#if defined(CONFIG_FAST_I2C)
  fastapi_driver.i2c = i2c0;
#endif

#endif//CONFIG_NRF52_I2C0

#ifdef CONFIG_NRF52_I2C1
  FAR struct i2c_master_s *i2c1;
  /* Initialize I2C */
  i2c1 = nrf52_i2cbus_initialize(1);
  if (!i2c1)
    {
      syslog(LOG_ERR, "ERROR: Failed to init the i2c1 driver\n");
      return -ENODEV;
    }
#if defined(CONFIG_SENSORS_TMP108)
  /* Then register the accelerometer sensor */
  ret = tmp108_register("/dev/temp0", i2c1, CONFIG_TMP108_I2C_ADDR, BOARD_TMP108_ALERT);
  if (ret < 0)
    {
      snerr("ERROR: Error registering tmp108\n");
      return -ENODEV;
    }
#endif


#ifdef CONFIG_SYSTEM_I2CTOOL
  /* Register I2C drivers on behalf of the I2C tool */
  i2c_register(i2c1, 1);
#endif

#endif//CONFIG_NRF52_I2C1
#endif//CONFIG_I2C


#ifdef CONFIG_SPI

#ifdef CONFIG_NRF52_CS_CONTROL_BY_USER
  customer_spi_cs_select();
#endif

#if defined(CONFIG_NRF52_SPI2)
  FAR struct spi_dev_s *spi2;

#ifdef CONFIG_NRF52_SPI2
  /* Initialize SPI2 */
  spi2 = nrf52_spibus_initialize(2, true);
  if (!spi2)
    {
      return -ENODEV;
    }
#endif

#if defined(CONFIG_FAST_SPI)
  fastapi_config.spi_devid = SPIDEV_ID(SPIDEVTYPE_ZGLUE_FAST, 0);
  fastapi_driver.spi = spi2;
#endif

#if defined(CONFIG_MAX86140)

  struct max86140_config_s max86140_config;
  memset(&max86140_config, 0, sizeof(struct max86140_config_s));
  max86140_config.spi_devid = SPIDEV_ID(SPIDEVTYPE_HEARTRATE, 0);
  max86140_config.int_pin = BOARD_MAX86140_INT;
  /* Then register the a heart rate sensor */
  ret = max86140_register("/dev/heartrate0", spi2, &max86140_config);
  if (ret < 0)
    {
      snerr("ERROR: Error registering MAX86140\n");
      return -ENODEV;
    }
#endif

#endif /* CONFIG_NRF52_SPI2 */
#endif /* CONFIG_SPI */

#if defined(CONFIG_SYSTEM_FAST_DRIVER)
  /* Then register the FAST API device */
  ret = fast_register(&fastapi_driver, &fastapi_config);
  if (ret < 0)
    {
      snerr("ERROR: Error registering the FASTAPI DEVICE\n");
      return -ENODEV;
    }
#endif

#ifdef CONFIG_FAST_ULPM_WAKEUP_PIN
  nrf_gpio_cfg_output(BOARD_ULPM_WAKEUP_PIN);
  gpio_pin_write(BOARD_ULPM_WAKEUP_PIN, 0);
#endif

#ifdef CONFIG_NRF52_PPI
  ret = nrf_drv_ppi_init();
  if (ret != 0)
    {
      snerr("ERROR: PPI Init error\n");
      return -ENODEV;

    }
#endif

#ifdef CONFIG_NRF52_EXAMPLES_GPIOTE_PPI
  uint32_t compare_evt_addr_0 = 0, compare_evt_addr_1 = 0, compare_evt_addr_2 = 0;
  compare_evt_addr_0 = nrf52_timer_event_address_get(tim0, NRF_TIMER_EVENT_COMPARE0);
  compare_evt_addr_1 = nrf52_timer_event_address_get(tim0, NRF_TIMER_EVENT_COMPARE1);
  compare_evt_addr_2 = nrf52_timer_event_address_get(tim0, NRF_TIMER_EVENT_COMPARE2);
#endif

#ifdef CONFIG_NRF52_GPIOTE
  ret = nrf_drv_gpiote_init();
  if (ret != 0)
    {
      snerr("ERROR: GPIOTE Init error\n");
      return -ENODEV;

    }
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Configure CPU load estimation */

  cpuload_initialize_once();
#endif

#ifdef CONFIG_SYSTEM_I2CTOOL
  /* Register I2C drivers on behalf of the I2C tool */
  nrf52_i2ctool();
#endif


  nrf52_proc_fs_initialize();

#ifdef CONFIG_BMM150
  ret = nrf52_bmm150_initialize();
  if (ret < 0)
    {
      serr("ERROR: bmm150 initialize Fail\n");
    }
#endif


#ifdef CONFIG_LSM6DS3
  ret = nrf52_lsm6ds3_initialize();
  if (ret < 0)
    {
      serr("ERROR: lsm6ds3 initialize Fail\n");
    }
#endif

#ifdef CONFIG_MC3672
  ret = nrf52_mc3672_initialize();
  if (ret < 0)
    {
      serr("ERROR: mc3672 initialize Fail\n");
    }

#endif
#if defined(CONFIG_BQ2512X)
  nrf52_bq2512x_initialize();
#endif


  return OK;
}
#endif /* CONFIG_LIB_BOARDCTL */


