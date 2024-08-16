/****************************************************************************
 * include/nuttx/audio/pcm510x.h
 *
 * drivers for pcm5100x/pcm5101x/pcm5102x
 *
 *   Copyright (C) 2018 zGlue Inc. All rights reserved.
 *   Author: Zhengwei Wang <zhengwei@zglue.com>
 *
 * References:
 *
 * -  The framework for this driver is based on Ken Pettit's VS1053 driver.
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

#ifndef __DRIVERS_AUDIO_PCM510X_H
#define __DRIVERS_AUDIO_PCM510X_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#ifdef CONFIG_AUDIO_PCM510X

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************
 *
 * CONFIG_AUDIO_PCM510X - Enables PCM510X support
 * CONFIG_PCM510X_INFLIGHT - Maximum number of buffers that the PCM510X driver
 *   will send to the I2S driver before any have completed.
 * CONFIG_PCM510X_MSG_PRIO - Priority of messages sent to the PCM510X worker
 *   thread.
 * CONFIG_PCM510X_BUFFER_SIZE - Preferred buffer size
 * CONFIG_PCM510X_NUM_BUFFERS - Preferred number of buffers
 * CONFIG_PCM510X_WORKER_STACKSIZE - Stack size to use when creating the the
 *   PCM510X worker thread.
 * CONFIG_PCM510X_SOFT_MUTE_ENABLE - enable Soft Mute / Soft Un-Mute feature
 *   of PCM510X, it requires to specify the gpio pin which connect to pcm510x's
 *   XSMT pin.
 */

/* Pre-requisites */

#ifndef CONFIG_AUDIO
#  error CONFIG_AUDIO is required for audio subsystem support
#endif

#ifndef CONFIG_I2S
#  error CONFIG_I2S is required by the PCM510X driver
#endif

/* Default configuration values */

#ifndef CONFIG_PCM510X_INFLIGHT
#  define CONFIG_PCM510X_INFLIGHT          4
#endif

#if CONFIG_PCM510X_INFLIGHT > 255
#  error CONFIG_PCM510X_INFLIGHT must fit in a uint8_t
#endif

#ifndef CONFIG_PCM510X_MSG_PRIO
#  define CONFIG_PCM510X_MSG_PRIO          1
#endif

#ifndef CONFIG_PCM510X_BUFFER_SIZE
#  define CONFIG_PCM510X_BUFFER_SIZE       1024
#endif

#ifndef CONFIG_PCM510X_NUM_BUFFERS
#  define CONFIG_PCM510X_NUM_BUFFERS       4
#endif

#ifndef CONFIG_PCM510X_WORKER_STACKSIZE
#  define CONFIG_PCM510X_WORKER_STACKSIZE  768
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/
struct pcm510x_lower_s
{
  /* GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the pcm510x driver from differences in GPIO
   * handling by varying boards and MCUs.
   *
   * xsmt_pin_init  - init the gpio as output
   * xsmt_pin_set  - true, set the pin to high level;
   *                 false,set the pin to low level.
   */

  CODE void (*xsmt_pin_init)(void);
  CODE void (*xsmt_pin_set)(bool enable);
};


/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: pcm510x_initialize
 *
 * Description:
 *   Initialize the PCM510X device.
 *
 * Input Parameters:
 *   i2s     - An I2S driver instance
 *
 * Returned Value:
 *   A new lower half audio interface for the PCM510X device is returned on
 *   success; NULL is returned on failure.
 *
 ****************************************************************************/

struct i2s_dev_s;         /* Forward reference. Defined in include/nuttx/audio/i2s.h */
struct audio_lowerhalf_s; /* Forward reference. Defined in nuttx/audio/audio.h */

FAR struct audio_lowerhalf_s *pcm510x_initialize(const FAR struct pcm510x_lower_s *lower,
                                                 FAR struct i2s_dev_s *i2s);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_AUDIO_PCM510X */
#endif /* __INCLUDE_NUTTX_AUDIO_PCM510X_H */

