/*
 * Copyright (c) 2017 Zglue Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file
 * @brief Bootloader device specific configuration.
 */

#ifndef _ZDK_NRF52_H_
#define _ZDK_NRF52_H_

/*********************The Flash Image Layout**********************************
 *
 *INTERNAL_FLASH
 *   0x0000 0000:  bootloader area
 *   ------- ----    it should be 64K or 128K
 *
 *   0x0001 0000:  The start location of Application
 *   ............    this is the image header field , about 256B
 *   0x0001 0200:  the real start address of application image
 *   ............    application image context
 *   ------- ----    Padded to FLASH_IMAGE_SIZE
 *
 *   0x000X 0000:  The second image location: Start_IMAGE_0 + FLASH_IMAGE_SIZE
 *   ............    second application image context
 *   ------- ----    Padded to FLASH_IMAGE_SIZE
 *
 *   0x00XX 0000:  The SCRATCH area: STart_IMAGE_1 + FLASH_IMAGE_SIZE
 *   ............   swap block , the size should be align to MIN_BLOCK_SIZE
 *   ------- ----
 *
 ****************** QSPI  as second image area ******************************
 * INTERNAL_FLASH
 *   0x0000 0000:  bootloader area
 *   ------- ----    it should be 64K or 128K
 *
 *   0x0001 0000:  The start location of Application
 *   ............    this is the image header field , about 256B
 *   0x0001 0200:  the real start address of application image
 *   ............    application image context
 *   ------- ----    Padded to FLASH_IMAGE_SIZE
 *   0x00XX 0000:  The SCRATCH area: STart_IMAGE_0 + FLASH_IMAGE_SIZE
 *   ............   swap block , the size should be align to MIN_BLOCK_SIZE
 *   ------- ----
 *
 * QSPI
 *   0x000X 0000:  The second image location: start offset in QSPI flash
 *   ............    second application image context
 *   ------- ----    Padded to FLASH_IMAGE_SIZE
 *
 ****************************************************************************/

/* The image base address in flash device */

#define FLASH_DEV_BASE_INTERNAL     0
#define FLASH_DEV_BASE_QSPI         0

/* image device id */

#define FLASH_DEV_ID_INTERNAL       100
#define FLASH_DEV_ID_QSPI           200

#if defined(CONFIG_MCUBOOT_APP_IMAGE_MAX_SIZE)
#define FLASH_IMAGE_SIZE                  CONFIG_MCUBOOT_APP_IMAGE_MAX_SIZE
#else
#define FLASH_IMAGE_SIZE                  (192*1024)
#endif

#if defined(CONFIG_MCUBOOT_APP_IMAGE_OFFSET)
#define FLASH_AREA_IMAGE_0_OFFSET         CONFIG_MCUBOOT_APP_IMAGE_OFFSET
#else
#define FLASH_AREA_IMAGE_0_OFFSET         (64*1024)
#endif

#define FLASH_AREA_IMAGE_0_SIZE           FLASH_IMAGE_SIZE
#define FLASH_AREA_IMAGE_1_SIZE           FLASH_IMAGE_SIZE

#define FLASH_AREA_IMAGE_0_DEV_ID         FLASH_DEV_ID_INTERNAL

#ifdef CONFIG_MCUBOOT_SECOND_IMAGE_QSPI

/* QSPI image definition */

#define FLASH_AREA_IMAGE_1_DEV_ID         FLASH_DEV_ID_QSPI

#define FLASH_AREA_IMAGE_1_OFFSET         FLASH_DEV_BASE_QSPI

#define FLASH_AREA_IMAGE_SCRATCH_OFFSET   (FLASH_AREA_IMAGE_0_OFFSET + FLASH_AREA_IMAGE_0_SIZE)

#else

#define FLASH_AREA_IMAGE_1_DEV_ID         FLASH_DEV_ID_INTERNAL

#define FLASH_AREA_IMAGE_1_OFFSET         (FLASH_AREA_IMAGE_0_OFFSET + FLASH_AREA_IMAGE_0_SIZE)

#define FLASH_AREA_IMAGE_SCRATCH_OFFSET   (FLASH_AREA_IMAGE_1_OFFSET + FLASH_AREA_IMAGE_1_SIZE)

#endif

#ifdef  CONFIG_MCUBOOT_APP_IMAGE_SCRATCH_SIZE
#define FLASH_AREA_IMAGE_SCRATCH_SIZE     CONFIG_MCUBOOT_APP_IMAGE_SCRATCH_SIZE
#else
#define FLASH_AREA_IMAGE_SCRATCH_SIZE     (8*1024)
#endif

#endif /* _ZDK_NRF52_H_ */

