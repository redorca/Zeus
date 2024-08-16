/****************************************************************************
 * include/nuttx/sensors/tmp108.h
 *
 *   Copyright (C) 2011-2012, 2015 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_SENSORS_TMP108_H
#define __INCLUDE_NUTTX_SENSORS_TMP108_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_I2C) && defined(CONFIG_TMP108_I2C)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************
 * CONFIG_I2C - Enables support for I2C drivers
 * CONFIG_TMP108_I2C - Enables support for the TMP-108 driver
 */

#define CONFIG_TMP108_BASEADDR 0x48
#define CONFIG_TMP108_ADDR0 (CONFIG_TMP108_BASEADDR + 0)
#define CONFIG_TMP108_ADDR1 (CONFIG_TMP108_BASEADDR + 1)
#define CONFIG_TMP108_ADDR2 (CONFIG_TMP108_BASEADDR + 2)
#define CONFIG_TMP108_ADDR3 (CONFIG_TMP108_BASEADDR + 3)

/* TMP-108 Register Definitions ***********************************************/
/* TMP-108 Registers addresses */

#define TMP108_TEMP_REG      0x00     /* Temperature Register */
#define TMP108_CONF_REG      0x01     /* Configuration Register */
#define TMP108_TLOW_REG      0x02     /* Temperature Low seting register */
#define TMP108_THIGH_REG     0x03     /* Temperature High seting register */

/* Configuration Register Bit Definitions */

#define TMP108_CONF_OP_MODE           (3 << 0)  /* Bit 0 - 1: Mode Mode 0 : Shutdown mode Mode 1: One shot Mode 2: CCM */
#define TMP108_CONF_THERMOSTAT_MODE   (1 << 2)  /* Bit 2: Thermostat mode */
#define TMP108_CONF_TMP_WDG_FLAGS     (3 << 3)  /* Bits 3 Set when Temp is lower than TLOW Bit4 set  when temp is greater than THIGH*/
#define TMP108_CONF_CR_RATE           (3 << 4)  /* Bits 5 -6 0 -.25Hz, 1 - 1Hz, 2  - 4Hz, 3 - 16Hz*/
#define TMP108_CONF_HYS               (3 << 12) /* Hysteresis setting 0 - 0c, 1 - 1c , 2 -2c, 3 -4c */
#define TMP108_CONF_ALERT_POL         (1 << 15) /* Alert pin polarity setting */

/* TMP108 operating modes */
#define TMP108_CONF_SHUTDOWN_MODE 0
#define TMP108_CONF_ONE_SHOT_MODE 1
#define TMP108_CONF_CONTINUOS_CONVERSION_MODE 2


/* NOTE: When temperature values are read, they are return as b16_t, fixed
 * precision integer values (see include/fixedmath.h).
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct tmp108_dev_s
{
  uint32_t alert_pin;           /* GPIO pin used for the alert interrupt  */
  pid_t receive_pid;            /* The task to be signalled */
  FAR struct i2c_master_s *i2c; /* I2C interface */
  void (*alert_cb)(FAR struct tmp108_dev_s *priv);
  /* GPIO pin used for the alert interrupt  */
  bool fahrenheit;              /* true: temperature will be reported in fahrenheit */
  uint8_t addr;                 /* I2C address */
  uint8_t signo;                /* signo to use when signaling a interrupt */
};

struct i2c_master_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: tmp108_register
 *
 * Description:
 *   Register the TMP-108 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   i2c - An instance of the I2C interface to use to communicate with TMP108
 *   addr - The I2C address of the LM-75.  The base I2C address of the TMP108
 *   is 0x48.  Bits 0-3 can be controlled to get 8 unique addresses from 0x48
 *   through 0x4f.
 *   pin - GPIO pin of the alert INT
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int tmp108_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                    uint8_t addr, uint8_t pin);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_TMP108_I2C */
#endif /* __INCLUDE_NUTTX_SENSORS_TMP108_H */
