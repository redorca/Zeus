/**
 * Copyright (c) 2015 - 2018, Zglue Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __ARCH_ARM_SRC_NRF52_NRF52_GPIO_H__
#define __ARCH_ARM_SRC_NRF52_NRF52_GPIO_H__

#include <stdbool.h>
#include <stdlib.h>
#include "nrf.h"
#include "nrf_gpio.h"


#ifdef __cplusplus
extern "C" {
#endif

#define NRF52_INVALID_GPIO_PIN      0xFF

/******************************************************************************
 * FUNCTION DECLARATIONS
 *****************************************************************************/
/*Write one or zero to the selected GPIO pin */
void gpio_pin_write(uint32_t pinset, bool value);
/*Read one or zero from the selected GPIO pin */
bool gpio_pin_read(uint32_t pinset);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_NRF52_NRF52_GPIO_H__ */
