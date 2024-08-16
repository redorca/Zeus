/****************************************************************************
 * include/utils/zglue_fast_api.h
 *
 *   Copyright (C) 2007-2009, 2014-2015 Gregory Nutt. All rights reserved.
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

#ifndef __ZGLUE_FAST_API_H
#define __ZGLUE_FAST_API_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <system/fast_cmd.h>

/****************************************************************************
 * Name: fast_led_blinky
 *
 * Description:
 *   Make fast led bliky.
 *
 * Input parameters:
 *   led_num - Led numbler..
 *
 * Returned Value:
 *   On success, a 0(OK) is returned to the caller.
 *   If an error occurs, the function will return a value of -1 (ERROR)
 *   and set errno to indicate the error.
 *
 ****************************************************************************/
int fast_led_blinky(uint8_t led_num);

/****************************************************************************
 * Name: fast_led_on
 *
 * Description:
 *   Make fast led on/off.
 *
 * Input parameters:
 *   led_on_off - Led on/off swither, 1 means on, 0 means off.
 *
 *   led_num - Led numbler.
 *
 * Returned Value:
 *   On success, a 0(OK) is returned to the caller.
 *   If an error occurs, the function will return a value of -1 (ERROR)
 *   and set errno to indicate the error.
 *
 ****************************************************************************/
int fast_led_on(bool led_on_off, uint8_t led_num);

#endif /* __ZGLUE_FAST_API_H */
