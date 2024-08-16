/********************************************************************************************
 * include/nuttx/power/bq2512x.h
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


#ifndef __NUTTX_POWER_BQ2512X_H
#define __NUTTX_POWER_BQ2512X_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/
#include <nuttx/config.h>


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/*interrupt source flag*/

#define BQ2512X_INT_VIN_OV          0X01
#define BQ2512X_INT_VIN_UV          0X02
#define BQ2512X_INT_BAT_UVLO        0X04
#define BQ2512X_INT_BAT_OCP         0X08
#define BQ2512X_INT_TS_COLD_HOT     0X10
#define BQ2512X_INT_TS_COOL         0X20
#define BQ2512X_INT_TS_WARM         0X40
#define BQ2512X_INT_TS_OFF          0X80


/*bat temperature status*/
#define BQ2512X_BAT_NORMAL          0
#define BQ2512X_BAT_COLD_HOT        1
#define BQ2512X_BAT_COOL            2
#define BQ2512X_BAT_WARM            3

/****************************************************************************
 * Public Types
 ****************************************************************************/

/*!
 *  @brief interrupt callback.This function will be called when a gpio interrupt is
 *         triggerd, this gpio may connect to INT or NRESET pin of bq25120 chip.
 *
 *  @param[in] param: pointers to the associated parameters
 *
 *  @return
 */

typedef void (*bq25120_gpio_int_cb_t)(void *param);

struct bq25120_low_level_operations_s
{
  /*!
   *  @brief This function is called when the device is registed.
   *  Call this function to initialize the MCU gpio connecting to INT pin of bq25120
   *  and attach the callback function to the corresponding GPIO falling edge enent.
   *
   *  @param[in] cb   : callback function when the corresponding interrupt is happend.
   *             param: parameters to pass to the callback function when it is called
   *
   *  @return Result of API execution status
   *  @retval zero -> Success  / -ve value -> Error
   */

  CODE int (*bq25120_INT_cfg_interrupt)(bq25120_gpio_int_cb_t cb, void *param);

#ifdef CONFIG_BQ2512X_NRESET_TO_GPIO

  /*!
   *  @brief This function is called when the device is registed.
   *  Call this function to initialize the MCU gpio connecting to NRESET pin of bq25120
   *  and attach the callback function to the corresponding GPIO falling edge enent.
   *
   *  @param[in] cb:    callback function when the corresponding interrupt is happend.
   *             param: parameters to pass to the callback function when it is called
   *
   *  @return Result of API execution status
   *  @retval zero -> Success  / -ve value -> Error
   */

  CODE int (*bq25120_RESET_cfg_interrupt)(bq25120_gpio_int_cb_t cb, void *param);

#endif

  /*!
   *  @brief This function is called to enable or disable detection of falling
   *         edge in INT pin.
   *
   *  @param[in] enable: true if you want enable, false if you want disable
   *
   *  @return Result of API execution status
   *  @retval zero -> Success  / -ve value -> Error
   */

  CODE int (*bq25120_INT_enable)(bool enable);

#ifdef CONFIG_BQ2512X_NRESET_TO_GPIO

  /*!
   *  @brief This function is called to enable or disable detection of falling
   *         edge in INT pin.
   *
   *  @param[in] enable: true if you want enable, false if you want disable
   *
   *  @return Result of API execution status
   *  @retval zero -> Success  / -ve value -> Error
   */

  CODE int (*bq25120_RESET_enable)(bool enable);

#endif

#ifdef CONFIG_BQ2512X_LSCTRL_TO_GPIO
  /*!
   *  @brief This function is called to config the LSCTRL pin as output.
   *
   *  @param[in]
   *
   *  @return Result of API execution status
   *  @retval zero -> Success  / -ve value -> Error
   */

  CODE int (*bq25120_LSCTRL_cfg_output)(void);

  /*!
   *  @brief This function is called to write the gpio connecting the LSCTRL pin.
   *
   *  @param[in] enable: true if you want to pull high,
   *                     false if you want to pull low
   *
   *  @return Result of API execution status
   *  @retval zero -> Success  / -ve value -> Error
   */

  CODE int (*bq25120_LSCTRL_write)(bool high_or_low);
#endif

#ifdef CONFIG_BQ2512X_NPG_TO_GPIO
  /*!
   *  @brief This function is called to config the NPG pin as INPUT.
   *
   *  @param[in]
   *
   *  @return Result of API execution status
   *  @retval zero -> Success  / -ve value -> Error
   */

  CODE int (*bq25120_NPG_cfg_input)(void);

  /*!
   *  @brief This function is called to read the gpio connecting the NCD pin.
   *
   *  @param[in] level: true if the voltage level is high, false if the
   *                    voltage level is low
   *
   *  @return Result of API execution status
   *  @retval zero -> Success  / -ve value -> Error
   */

  CODE int (*bq25120_NPG_read)(bool *level);
#endif



};



#endif /* __DRIVERS_POWER_BQ2512X_H */
