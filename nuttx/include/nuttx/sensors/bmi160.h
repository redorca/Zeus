/********************************************************************************************
 * include/nuttx/sensors/bmg160.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_BMI160_H
#define __INCLUDE_NUTTX_SENSORS_BMI160_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/irq.h>
#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/sensors/internal_bmi160.h>
#include <nuttx/sensors/ioctl.h>

#ifdef CONFIG_BMI160

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/********************************************************************************************
 * Public Types
 ********************************************************************************************/
struct bmi160_i2c_bus_config_s
{
  FAR struct i2c_master_s *i2c;/* Pointer to the i2c instance */
  FAR struct i2c_config_s config;
};

struct bmi160_spi_bus_config_s
{
  FAR struct spi_dev_s *spi;/* Pointer to the SPI instance */
  FAR uint32_t spi_frequency;
  FAR uint32_t spi_devid;
  FAR enum spi_mode_e spi_mode;
};

union bus_config_s
{
  struct bmi160_i2c_bus_config_s i2c_config;
  struct bmi160_spi_bus_config_s spi_config;
};

struct bmi160_bus_config_s
{
  /*! 0 - I2C , 1 - SPI Interface */
  uint8_t bus_selection;
  union bus_config_s bus_config;
};

/*!
 *  @brief interrupt callback.
 *
 *  @param[in] int_channel: interrput channel, BMI160_INT_CHANNEL_1
 *                          or BMI160_INT_CHANNEL_2
 *
 *  @return
 */

typedef void (*bmi160_int_cb_t)(enum bmi160_int_channel int_channel);

struct bmi160_low_level_operations_s
{
  /*!
   *  @brief This function is called when the device is registed. BMI160 has two
   *  interrupt output pins, call this function to initialize the interrput pin
   *  and attach the callback function to the corresponding GPIO interrput enent.
   *
   *  @param[in] int_channel     : interrput channel, INT_PIN_1 or INT_PIN_2
   *  @param[in] cb            : callback function when the corresponding interrupt is happend.
   *
   *  @return Result of API execution status
   *  @retval zero -> Success  / -ve value -> Error
   */

  CODE int (*bmi160_attach)(enum bmi160_int_channel int_channel, bmi160_int_cb_t cb);


  /*!
   *  @brief This function is called to enable or disable the bmi160 interrupt.
   *         BMI160 has two interrupt output pins, call this function will enable
   *         or disable corresponding GPIO interrput enent.
   *
   *  @param[in] int_channel: interrput channel, BMI160_INT_CHANNEL_1
   *                          or BMI160_INT_CHANNEL_2
   *  @param[in] enable: true if you want enable, false if you want disable
   *
   *  @return Result of API execution status
   *  @retval zero -> Success  / -ve value -> Error
   */

  CODE int (*bmi160_int_enable)(enum bmi160_int_channel int_channel, bool enable);
};





/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/********************************************************************************************
 * Name: bmi160_register
 *
 * Description:
 *   Register the BMI160 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/gyr0"
 *   bus - An struct describing the bus which use to communicate with BMI160
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ********************************************************************************************/

int bmi160_register(FAR const char *devpath,
                    FAR struct bmi160_bus_config_s *bus_config,
                    FAR const struct bmi160_low_level_operations_s *ll_operation);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SPI && CONFIG_BMG160 */
#endif /* __INCLUDE_NUTTX_SENSORS_BMG160_H */
