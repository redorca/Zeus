/****************************************************************************************************//**
 * @file     nrf_bluetooth.h
 *
 * @brief    softdevice macro Header File for nrf52.
 *
 * @version  V1
 * @date     8. March 2018
 *
 * @note     Generated with SVDConv V2.81d
 *           from CMSIS SVD File 'nrf52.svd' Version 1,
 *
 * @par      Copyright (c) 2010 - 2018, Nordic Semiconductor ASA All rights reserved.
 *
 *           Redistribution and use in source and binary forms, with or without
 *           modification, are permitted provided that the following conditions are met:
 *
 *           1. Redistributions of source code must retain the above copyright notice, this
 *           list of conditions and the following disclaimer.
 *
 *           2. Redistributions in binary form must reproduce the above copyright
 *           notice, this list of conditions and the following disclaimer in the
 *           documentation and/or other materials provided with the distribution.
 *
 *           3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *           contributors may be used to endorse or promote products derived from this
 *           software without specific prior written permission.
 *
 *           THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *           AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *           IMPLIED WARRANTIES OF MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE
 *           ARE DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 *           LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *           CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *           SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *           INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *           CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *           ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *           POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *******************************************************************************************************/

#ifndef NRF_BLUETOOTH_H
#define NRF_BLUETOOTH_H
/* ========================================================================= */
/* =========      Bluetooth Settings Section     ================ */
/* ========================================================================= */
#if defined(CONFIG_NRF52_BLUETOOTH)

#if defined(CONFIG_ARCH_CHIP_NRF52840)
#define S140 1
#elif defined(CONFIG_ARCH_CHIP_NRF52832)
#define S132 1
#else
#error "Please select the right softdevice option"
#endif

#define BLE_STACK_SUPPORT_REQD 0
#define  __STACK_SIZE CONFIG_BLE__STACK_SIZE /* =2048 */
#define __HEAP_SIZE  CONFIG_BLE___HEAP_SIZE /* =1024 */
#define  SOFTDEVICE_PRESENT 1

#if defined(CONFIG_GATT_BLE_MTU_SIZE_DEFAULT)
#define GATT_MTU_SIZE_DEFAULT CONFIG_BLE_GATT_MTU_SIZE_DEFAULT
#endif

#if defined(CONFIG_NRF_SD_BLE_API_VERSION)
#define NRF_SD_BLE_API_VERSION CONFIG_NRF_SD_BLE_API_VERSION
#endif

#if defined(CONFIG_NRF52_BOOTLOADER)
#define NRF_DFU_SETTINGS_VERSION 1
#define __LINT__ 0
#endif

#endif /* CONFIG_NRF52_BLUETOOTH */

#endif

