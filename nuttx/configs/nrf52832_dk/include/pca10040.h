/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
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
#ifndef PCA10040_H
#define PCA10040_H

#ifdef __cplusplus
extern "C" {
#endif

/* procfs mount point and type */

#define PROCFS_MOUNT   "/proc"
#define PROCFS_TYPE    "procfs"

/* tmpfs mount point and type */

#define TMPFS_MOUNT   "/tmp"
#define TMPFS_TYPE    "tmpfs"

/* block and char device mount point */

#define MTD_BLOCK_MOUNT   "/dev/mtdblock"
#define MTD_CHAR_MOUNT    "/dev/mtd"

/* the mtd partition for internal whole flash */

#define PROGMEM_MTD_MINOR  0

/* the fs mtd partition based on internal flash */

#define PROGMEM_FS_MTD_MINOR  1
#define INTERNAL_FLASH_MOUNT_POINT   "/internal"
#define INTERNAL_FLASH_FS_TYPE      "vfat"
#define INTERNAL_FLASH_FS_LABEL   "N52FAT"
#define INTERNAL_FLASH_FS_LABEL_LEN (sizeof(INTERNAL_FLASH_FS_LABEL))


/* the fs mtd partition based on spi nor flash */

#define SPI_FLASH_FS_MTD_MINOR  2

#define SPI_FLASH_FS_SMARTFS_MINOR 0

#define SPI_FLASH_MOUNT_POINT   "/w25"
#define SPI_FLASH_FS_TYPE      "vfat"
#define SPI_FLASH_FS_LABEL   "W25FAT"
#define SPI_FLASH_FS_LABEL_LEN (sizeof(SPI_FLASH_FS_LABEL))


/* NRF52 internal Flash memory layer propose
 * it is better to align image to 4K , 52832 page is 4K
 *   1. no softwaredevice :
 *     16K    : bootloader
 *     320K    : app
 *     others : progmem nxffs filesystem
 *   2. BLE case:
 *     16K    : bootloader
 *     64K    : softdevice
 *     320K    : app
 *     others : progmem nxffs filesystem
 *
 */
#define UNIT_1K  (1024)
#define SYSTEM_BOOTLOADER_IMAGE_SIZE (16*UNIT_1K)   /* assume system bootloader size */
#define SYSTEM_SOFTDEVICE_IMAGE_SIZE (128*UNIT_1K)   /* softdevice size */
#define SYSTEM_APP_IMAGE_SIZE  (320*UNIT_1K)        /* assume system image size   */




// LEDs definitions for PCA10040
#define LEDS_NUMBER    4

#define LED_START      17
#define LED_1          17
#define LED_2          18
#define LED_3          19
#define LED_4          20
#define LED_STOP       20

#define LEDS_ACTIVE_STATE 0

#define LEDS_INV_MASK  LEDS_MASK

#define LEDS_LIST { LED_1, LED_2, LED_3, LED_4 }

#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2
#define BSP_LED_2      LED_3
#define BSP_LED_3      LED_4

#define BUTTONS_NUMBER 4

#define BUTTON_START   13
#define BUTTON_1       13
#define BUTTON_2       14
#define BUTTON_3       15
#define BUTTON_4       16
#define BUTTON_STOP    16
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0

#define BUTTONS_LIST { BUTTON_1, BUTTON_2, BUTTON_3, BUTTON_4 }

#define BSP_BUTTON_0   BUTTON_1
#define BSP_BUTTON_1   BUTTON_2
#define BSP_BUTTON_2   BUTTON_3
#define BSP_BUTTON_3   BUTTON_4

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

#ifdef __cplusplus
}
#endif

#endif // PCA10040_H
