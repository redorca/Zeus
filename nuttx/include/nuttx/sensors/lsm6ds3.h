/********************************************************************************************
 * include/nuttx/sensors/lsm6ds3.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_LSM6DS3_H
#define __INCLUDE_NUTTX_SENSORS_LSM6DS3_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/irq.h>
#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/sensors/internal_lsm6ds3.h>
#include <nuttx/sensors/ioctl.h>

#ifdef CONFIG_LSM6DS3

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
enum lsm6ds3_bus_type
{
  LSM6DS3_I2C_INTF,
  LSM6DS3_SPI_INTF,
};

struct lsm6ds3_i2c_bus_config_s
{
  FAR struct i2c_master_s *i2c;/* Pointer to the i2c instance */
  FAR struct i2c_config_s config;
};

struct lsm6ds3_spi_bus_config_s
{
  FAR struct spi_dev_s *spi;/* Pointer to the SPI instance */
  FAR uint32_t spi_frequency;
  FAR uint32_t spi_devid;
  FAR enum spi_mode_e spi_mode;
};

union bus_config_s
{
  struct lsm6ds3_i2c_bus_config_s i2c_config;
  struct lsm6ds3_spi_bus_config_s spi_config;
};

struct lsm6ds3_bus_config_s
{
  /*! 0 - I2C , 1 - SPI Interface */
  enum lsm6ds3_bus_type bus_selection;
  union bus_config_s bus_config;
};

/*!
 * @brief lsm6ds3 interrupt channel selection structure
 */
enum lsm6ds3_int_channel
{
  /*! Un-map both channels */
  LSM6DS3_INT_CHANNEL_NONE,
  /*! interrupt Channel 1 */
  LSM6DS3_INT_CHANNEL_1,
  /*! interrupt Channel 2 */
  LSM6DS3_INT_CHANNEL_2,
  /*! Map both channels */
  LSM6DS3_INT_CHANNEL_BOTH
};

enum lsm6ds_int_types
{
  LSM6DS3_ACC_FREE_FALLING_INT,
  LSM6DS3_ACC_WAKEUP_INT,
  LSMD6S3_ACC_6D_ORIENTATION_INT,
  LSMD6S3_ACC_4D_ORIENTATION_INT,
  LSMD6S3_ACC_DOUBLE_TAP_INT,
  LSMD6S3_ACC_SINGLE_TAP_INT,
  LSMD6S3_ACC_INACTIVE_INT,
};

/*interrupt status mask*/
#define LSM6DS3_FREE_FALLING_INT_MASK       (1<<LSM6DS3_ACC_FREE_FALLING_INT)
#define LSM6DS3_WAKEUP_INT_MASK             (1<<LSM6DS3_ACC_WAKEUP_INT)
#define LSMD6S3_6D_ORIENTATION_INT_MASK     (1<<LSMD6S3_ACC_6D_ORIENTATION_INT)
#define LSMD6S3_4D_ORIENTATION_INT_MASK     (1<<LSMD6S3_ACC_4D_ORIENTATION_INT)
#define LSMD6S3_DOUBLE_TAP_INT_MASK         (1<<LSMD6S3_ACC_DOUBLE_TAP_INT)
#define LSMD6S3_SINGLE_TAP_INT_MASK         (1<<LSMD6S3_ACC_SINGLE_TAP_INT)
#define LSMD6S3_INACTIVE_INT_MASK           (1<<LSMD6S3_ACC_INACTIVE_INT)



struct lsm6ds_acc_6D_4D_int_cfg
{
  uint8_t threshold;/*50, 60, 70 or 80 degree*/
  bool use_LPF2;/*use low pass filter or not*/
};


union lsm6ds_int_type_cfg
{
  /*! free falling interrupt structure */
  /*! wake up interrupt structure */
  /*! double tap interrupt structure */
  /*! single tap interrupt structure */
  /*! inavtive interrupt structure */

  /*! 6D 4D ORIENTATION interrupt structure */
  struct lsm6ds_acc_6D_4D_int_cfg acc_6D_4D_int;

};


struct lsm6ds3_int_settg
{
  /*! Interrupt channel */
  enum lsm6ds3_int_channel int_channel;
  /*! Select Interrupt */
  enum lsm6ds_int_types int_type;
  /*! Union configures required interrupt */
  union lsm6ds_int_type_cfg int_type_cfg;
};



/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/



/********************************************************************************************
 * Name: lsm6ds3_register
 *
 * Description:
 *   Register the LSM6DS3 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/lsm6ds3"
 *   bus - An struct describing the bus which use to communicate with LSM6DS3
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ********************************************************************************************/

int lsm6ds3_register(FAR const char *devpath,
                     FAR struct lsm6ds3_bus_config_s *bus_config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_LSM6DS3 */
#endif /* __INCLUDE_NUTTX_SENSORS_LSM6DS3_H */
