/************************************************************************************
 * include/nuttx/sensors/max86140.h
 *
 *   Copyright (C) 2009, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Copyright (C) 2018 Zglue  Inc. All rights reserved.
 *           Min Yang <min.yang@zglue.com>
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
 ************************************************************************************/

#ifndef  __INCLUDE_NUTTX_SENSORS_MAX86140_H
#define  __INCLUDE_NUTTX_SENSORS_MAX86140_H


#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif


#if defined(CONFIG_MAX86140)
struct max86140_config_s
{
  /* Since multiple device can be connected to the same SPI bus we need
   * to use multiple spi device ids which are employed by NuttX to select/
   * deselect the desired device via their chip select inputs.
   */
  int spi_devid;
  uint32_t int_pin;           /* GPIO pin used for the interrupt  */
  uint32_t gpio1_pin;           /* GPIO pin used for GPIO_1 test  */
  uint32_t gpio2_pin;           /* GPIO pin used for GPIO_2 test  */
};

struct max86140_dev_s
{
  uint32_t spi_frequency;               /* spi frequency */
  pid_t receive_pid;                    /* The task to be signalled */
  uint8_t signo;                        /* signo to use when signaling a interrupt */
  struct spi_dev_s *spi;                /* SPI interface */
  void (*int_cb)(FAR struct max86140_dev_s *priv);  /* GPIO pin used for the interrupt  */
  struct max86140_config_s config;      /* configuration of the max86140 device */
};

int max86140_register(FAR const char *devpath, FAR struct spi_dev_s *spi, FAR struct max86140_config_s *config);
#endif /*CONFIG_MAX86140*/

#if defined(__cplusplus)
}
#endif

#endif /*__INCLUDE_NUTTX_SENSORS_MAX86140_H */
