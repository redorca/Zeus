/**
 * Copyright (c) 2016 - 2017, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef PCA10056_H
#define PCA10056_H

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
#define INTERNAL_FLASH_FS_SIZE    (256*1024UL)
#define INTERNAL_FLASH_FS_LABEL_LEN (sizeof(INTERNAL_FLASH_FS_LABEL))


/* the fs mtd partition based on spi nor flash */

#define SPI_FLASH_FS_MTD_MINOR  2

#define SPI_FLASH_FS_SMARTFS_MINOR 0

#define SPI_FLASH_MOUNT_POINT   "/w25"
#define SPI_FLASH_FS_TYPE      "vfat"
#define SPI_FLASH_FS_LABEL   "W25FAT"
#define SPI_FLASH_FS_LABEL_LEN (sizeof(SPI_FLASH_FS_LABEL))


/* the fs mtd partition based on qspi           flash */

#define QSPI_F_FS_MTD_MINOR  3

#define QSPI_F_FS_SMARTFS_MINOR 0

#define QSPI_F_MOUNT_POINT   "/mx25r"
#define QSPI_F_FS_TYPE      "vfat"
#define QSPI_F_FS_LABEL   "MX25FAT"
#define QSPI_F_FS_LABEL_LEN (sizeof(QSPI_F_FS_LABEL))

/* split the firstly 1MB of mx25rxx as firmware */
#define QSPI_FW_SIZE   (1024*1024UL)
#define QSPI_FW_MTD_MINOR  4
#define QSPI_FW_MTD_NAME   "firmware"


/* NRF52 internal Flash memory layer propose
 * it is better to align image to 4K , 52840 page is 4K
 *     64K    : bootloader
 *     500K    : app
 *     others : progmem nxffs filesystem
 *
 */
#define UNIT_1K  (1024)
#define SYSTEM_APP_IMAGE_SIZE  (500*UNIT_1K)        /* assume system image size   */


// LEDs definitions for PCA10056
#define LEDS_NUMBER    4

#define LED_1          NRF_GPIO_PIN_MAP(0,13)
#define LED_2          NRF_GPIO_PIN_MAP(0,14)
#define LED_3          NRF_GPIO_PIN_MAP(0,15)
#define LED_4          NRF_GPIO_PIN_MAP(0,16)

#define LEDS_ACTIVE_STATE 0

#define LEDS_LIST { LED_1, LED_2, LED_3, LED_4 }

#define LEDS_INV_MASK  LEDS_MASK

#define BSP_LED_0      13
#define BSP_LED_1      14
#define BSP_LED_2      15
#define BSP_LED_3      16

#define BUTTONS_NUMBER 4

#define BUTTON_1       11
#define BUTTON_2       12
#define BUTTON_3       24
#define BUTTON_4       25
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

#endif // PCA10056_H
