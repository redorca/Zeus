/****************************************************************************
  * arch/arm/src/nrf52/nrf52_power.h
  *
  *   Copyright (C) 2012, 2018 Levin Li. All rights reserved.
  *   Author: Levin Li <zhiqiang@zglue.com>
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

#ifndef __ARCH_ARM_SRC_NRF52_NRF52_POWER_H
#define __ARCH_ARM_SRC_NRF52_NRF52_POWER_H

enum POWER_STATUS
{
  NRF52_POWER_SLEEP_ENTER = 0x0,
  NRF52_POWER_SLEEP_EXIT,
  NRF52_POWER_USBD_DETECT = 0x10,
  NRF52_POWER_USBD_REMOVE,
  NRF52_POWER_USBD_PWRRDY,
};

/**
 * @brief Function for checking usb phy power status
 *
 * This function is used to get whatever common POWER_CLOCK common interrupt
 * should be disabled or not if @ref nrf_drv_clock tries to disable the interrupt.
 *
 * @retval NRF52_POWER_USBD_DETECT  VBUS power has been detected
 * @retval NRF52_POWER_USBD_REMOVE  VBUS power is lost
 * @retval NRF52_POWER_USBD_PWRRDY  USB PHY is ready to do comunnication
 *
 * @sa nrf52_power_get_usbphy_status
 */
uint32_t nrf52_power_get_usbphy_status(void);

/************************************************************************************
 * Name: power_callback
 *
 * Description:
 *    this function is called from interrupt, it will pass power_status
 *
 ************************************************************************************/
typedef void (*power_callback)(uint32_t power_status);


#define POWER_CALLBACK_USBD   0
#define POWER_CALLBACK_SLEEP  1

/************************************************************************************
 * Name: nrf52_power_register_callback
 *
 * Description:
 *   This functions provide the register callback, you can register usbd or sleep
 *   callback.
 * Parameter: callback the callback will be called when power status detected
 *            type is  POWER_CALLBACK_USBD or POWER_CALLBACK_SLEEP
 *
 ************************************************************************************/

int  nrf52_power_register_callback(power_callback callback,            uint8_t type);

void nrf52_power_initlialize(void);

#endif

