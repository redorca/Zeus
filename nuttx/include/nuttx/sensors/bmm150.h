/********************************************************************************************
 * include/nuttx/sensors/bmm150.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_BMM150_H
#define __INCLUDE_NUTTX_SENSORS_BMM150_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/irq.h>
#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/sensors/ioctl.h>

#if (defined CONFIG_BMM150)

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
enum bmm150_bus_type
{
  BMM150_USE_I2C,
  BMM150_USE_SPI,
};

struct bmm150_i2c_bus_config_s
{
  FAR struct i2c_master_s *i2c;/* Pointer to the i2c instance */
  FAR struct i2c_config_s config;
};

struct bmm150_spi_bus_config_s
{
  FAR struct spi_dev_s *spi;/* Pointer to the SPI instance */
  FAR uint32_t spi_frequency;
  FAR uint32_t spi_devid;
  FAR enum spi_mode_e spi_mode;
};

union bus_config_s
{
  struct bmm150_i2c_bus_config_s i2c_config;
  struct bmm150_spi_bus_config_s spi_config;
};

struct bmm150_bus_config_s
{
  /*! 0 - I2C , 1 - SPI Interface */
  enum bmm150_bus_type bus_selection;
  union bus_config_s bus_config;
};

/*interrupt type, select different type when using SNIOC_INT_ENABLE IOctrl command*/
enum bmm150_int_type
{
  BMM150_DATA_OVERRUN = 0x01,
  BMM150_OVERFLOW = 0x02,
  BMM150_HIGH_X = 0x04,
  BMM150_HIGH_Y = 0x08,
  BMM150_HIGH_Z = 0x10,
  BMM150_LOW_X = 0x20,
  BMM150_LOW_Y = 0x40,
  BMM150_LOW_Z = 0x80,
};

/*!
 *  @brief interrupt callback.This function will be called when a gpio interrupt is
 *         triggerd, this gpio may connect to DRDY or INT pin of bq25120 chip.
 *
 *  @param[in] param: pointers to the associated parameters
 *
 *  @return
 */

typedef void (*bmm150_gpio_int_cb_t)(void *param);

struct bmm150_low_level_operations_s
{
  /*!
   *  @brief This function is called when the device is registed.
   *  Call this function to initialize the MCU gpio connecting to DRDY pin of bmm150
   *  and attach the callback function to the corresponding GPIO falling/rising edge event.
   *
   *  @param[in] cb   : callback function when the corresponding interrupt is happend.
   *             param: parameters to pass to the callback function when it is called
   *
   *  @return Result of API execution status
   *  @retval zero -> Success  / -ve value -> Error
   */

  CODE int (*bmm150_DRDY_cfg_interrupt)(bmm150_gpio_int_cb_t cb, void *param);

  /*!
   *  @brief This function is called when the device is registed.
   *  Call this function to initialize the MCU gpio connecting to INT pin of bmm150
   *  and attach the callback function to the corresponding GPIO falling/rising edge event.
   *
   *  @param[in] cb:    callback function when the corresponding interrupt is happend.
   *             param: parameters to pass to the callback function when it is called
   *
   *  @return Result of API execution status
   *  @retval zero -> Success  / -ve value -> Error
   */

  CODE int (*bmm150_INT_cfg_interrupt)(bmm150_gpio_int_cb_t cb, void *param);

  /*!
   *  @brief This function is called to enable or disable detection of falling
   *         edge in DRDY pin.
   *
   *  @param[in] enable: true if you want enable, false if you want disable
   *
   *  @return Result of API execution status
   *  @retval zero -> Success  / -ve value -> Error
   */

  CODE int (*bmm150_DRDY_enable)(bool enable);

  /*!
   *  @brief This function is called to enable or disable detection of falling
   *         edge in INT pin.
   *
   *  @param[in] enable: true if you want enable, false if you want disable
   *
   *  @return Result of API execution status
   *  @retval zero -> Success  / -ve value -> Error
   */

  CODE int (*bmm150_INT_enable)(bool enable);

};



/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/



/********************************************************************************************
 * Name: bmm150_register
 *
 * Description:
 *   Register the BMM150 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/bmm150"
 *   bus - An struct describing the bus which use to communicate with BMM150
 *   ll_op -- functions related to GPIO
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ********************************************************************************************/

int bmm150_register(FAR const char *devpath,
                    FAR struct bmm150_bus_config_s *bus_config,
                    const struct bmm150_low_level_operations_s *ll_op);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_BMM150*/
#endif /* __INCLUDE_NUTTX_SENSORS_BMM150_H */
