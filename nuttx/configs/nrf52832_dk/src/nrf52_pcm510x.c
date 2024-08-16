/************************************************************************************
 * configs/nrf52840_dk/src/nrf52_pcm510x.c
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
 ************************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/audio/pcm.h>
#include <nuttx/audio/pcm510x.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf52_i2s.h"

#if (defined CONFIG_AUDIO_PCM510X) && (defined CONFIG_NRF52_I2S)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the pcm510x driver from differences in GPIO
 * handling by varying boards and MCUs.
 *
 * xsmt_pin_init  - init the gpio as output
 * xsmt_pin_set  - true, set the pin to high level;
 *                 false,set the pin to low level.
 */

static void pcm510x_xsmt_pin_init(void);
static void pcm510x_xsmt_pin_set(bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the pcm510x
 * driver.  This structure provides information about the configuration
 * of the pcm510x and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.
 */

const struct pcm510x_lower_s g_pcm510x_xsmt_pin_op =
{
  .xsmt_pin_init = pcm510x_xsmt_pin_init,
  .xsmt_pin_set = pcm510x_xsmt_pin_set,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

void pcm510x_xsmt_pin_init(void)
{
  nrf_gpio_cfg_output(BOARD_PCM510X_XSMT_PIN);
}

void pcm510x_xsmt_pin_set(bool enable)
{
  if (enable)
    {
      nrf_gpio_pin_set(BOARD_PCM510X_XSMT_PIN);
    }
  else
    {
      nrf_gpio_pin_clear(BOARD_PCM510X_XSMT_PIN);
    }
}

#ifdef CONFIG_AUDIO_FAKE_MUSIC_FILE
extern int fake_music_file_register(void);
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_pcm510x_initialize
 *
 * Description:
 *   This function is called by platform-specific, setup logic to configure
 *   and register the pcm510x device.  This function will register the driver
 *   as /dev/pcm510x.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int nrf52_pcm510x_initialize(void)
{
  FAR struct audio_lowerhalf_s *pcm510x;
  FAR struct audio_lowerhalf_s *pcm;
  FAR struct i2s_dev_s *i2s;
  static bool initialized = false;
  int ret;

  /* Have we already initialized?  Since we never uninitialize we must prevent
   * multiple initializations.  This is necessary, for example, when the
   * touchscreen example is used as a built-in application in NSH and can be
   * called numerous time.  It will attempt to initialize each time.
   */

  if (!initialized)
    {

      /*register a fake music file to /music directory, this is for test purpose only.*/
#ifdef CONFIG_AUDIO_FAKE_MUSIC_FILE
      fake_music_file_register();
#endif

      /* Get an instance of the I2S interface for the pcm510x data channel */

      i2s = nrf52_i2s_initialize();
      if (!i2s)
        {
          auderr("ERROR: Failed to initialize NRF52 I2S\n");
          ret = -ENODEV;
          goto errout;
        }

      /* Now we can use this I2S interfaces to initialize the
       * PCM510X which will return an audio interface.
       */

      pcm510x = pcm510x_initialize(&g_pcm510x_xsmt_pin_op, i2s);
      if (!pcm510x)
        {
          auderr("ERROR: Failed to initialize the PCM510X\n");
          ret = -ENODEV;
          goto errout;
        }

      /* No we can embed the PCM510X/I2S conglomerate into a PCM decoder
       * instance so that we will have a PCM front end for the the PCM510X
       * driver.
       */

      pcm = pcm_decode_initialize(pcm510x);
      if (!pcm)
        {
          auderr("ERROR: Failed create the PCM decoder\n");
          ret = -ENODEV;
          goto errout;
        }


      /* Finally, we can register the PCM/PCM510X/I2S audio device.
       */

      ret = audio_register("pcm510x", pcm);
      if (ret < 0)
        {
          auderr("ERROR: Failed to register /dev/pcm510x device: %d\n", ret);
          goto errout;
        }

      /* Now we are initialized */

      initialized = true;
    }

  return OK;

  /* Error exits.  Unfortunately there is no mechanism in place now to
   * recover resources from most errors on initialization failures.
   */

errout:
  return ret;
}

#endif /*CONFIG_AUDIO_PCM510X*/