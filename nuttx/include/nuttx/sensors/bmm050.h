/********************************************************************************************
 * include/nuttx/sensors/bmm050.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_BMM050_H
#define __INCLUDE_NUTTX_SENSORS_BMM050_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/irq.h>
#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/sensors/ioctl.h>

#if (defined CONFIG_BMM050) || (defined CONFIG_BMM150)

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
enum bmm050_bus_type
{
  BMM050_I2C_INTF,
  BMM050_SPI_INTF,
};

struct bmm050_i2c_bus_config_s
{
  FAR struct i2c_master_s *i2c;/* Pointer to the i2c instance */
  FAR struct i2c_config_s config;
};

struct bmm050_spi_bus_config_s
{
  FAR struct spi_dev_s *spi;/* Pointer to the SPI instance */
  FAR uint32_t spi_frequency;
  FAR uint32_t spi_devid;
  FAR enum spi_mode_e spi_mode;
};

union bus_config_s
{
  struct bmm050_i2c_bus_config_s i2c_config;
  struct bmm050_spi_bus_config_s spi_config;
};

struct bmm050_bus_config_s
{
  /*! 0 - I2C , 1 - SPI Interface */
  enum bmm050_bus_type bus_selection;
  union bus_config_s bus_config;
};


/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/



/********************************************************************************************
 * Name: bmm050_register
 *
 * Description:
 *   Register the BMM050 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/bmm050"
 *   bus - An struct describing the bus which use to communicate with BMM050
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ********************************************************************************************/

int bmm050_register(FAR const char *devpath,
                    FAR struct bmm050_bus_config_s *bus_config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_BMM050 CONFIG_BMM150 */
#endif /* __INCLUDE_NUTTX_SENSORS_BMM050_H */
