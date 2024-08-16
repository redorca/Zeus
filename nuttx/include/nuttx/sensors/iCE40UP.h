/********************************************************************************************
 * include/nuttx/sensors/iCE40UP.h
 *
 *   Copyright (C) 2018 zGlue Inc. All rights reserved.
 *   Author: Zhengwei Wang <zhengwei@zglue.com>
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
 ********************************************************************************************/

#ifndef __INCLUDE_NUTTX_SENSORS_ICE40UP_H
#define __INCLUDE_NUTTX_SENSORS_ICE40UP_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/
#include <nuttx/config.h>

#ifdef CONFIG_ICE40UP

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/*!
 *  @brief interrupt callback.
 *
 *  @param[in]
 *
 *  @return
 */
typedef void (*iCE40UP_int_cb_t)(void *arg, bool status);

struct iCE40UP_low_level_operations_s
{
  /*!
   *  @brief This function is called when the device is registed.
   *  Call this function to initialize the interrput pin
   *  and attach the callback function to the corresponding GPIO interrput enent.
   *
   *  @param[in] cb            : callback function when the corresponding interrupt is happend.
   *
   *  @return Result of API execution status
   *  @retval zero -> Success  / -ve value -> Error
   */

  CODE int (*iCE40UP_attach)(iCE40UP_int_cb_t cb, void *arg);


  /*!
   *  @brief This function is called to enable or disable the iCE40UP interrupt.
   *         Call this function will enable or disable corresponding GPIO interrput enent.
   *
   *  @param[in] enable: true if you want enable, false if you want disable
   *
   *  @return Result of API execution status
   *  @retval zero -> Success  / -ve value -> Error
   */

  CODE int (*iCE40UP_int_enable)(bool enable);

};





/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/



/********************************************************************************************
 * Name: iCE40UP_register
 *
 * Description:
 *   Register the iCE40UP character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/iCE40UP"
 *   ll_operation - interrupt pin related operations
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ********************************************************************************************/

int iCE40UP_register(FAR const char *devpath,
                     FAR const struct iCE40UP_low_level_operations_s *ll_operation);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_ICE40UP */
#endif /* __INCLUDE_NUTTX_SENSORS_ICE40UP_H */
