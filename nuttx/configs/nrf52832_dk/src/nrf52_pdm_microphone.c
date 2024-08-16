/************************************************************************************
 * configs/nrf52832_dk/src/nrf52_pdm_microphone.c
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

#include <nuttx/audio/pdm.h>
#include <nuttx/audio/pcm.h>
#include <nuttx/audio/pdm_microphone.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "nrf.h"
#include "nrf52_pdm.h"

#if (defined CONFIG_AUDIO_PDM_MIC) && (defined CONFIG_NRF52_PDM)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/


/****************************************************************************
 * Private Data
 ****************************************************************************/


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_pdm_microphone_initialize
 *
 * Description:
 *   This function is called by platform-specific, setup logic to configure
 *   and register the pdm microphone device.  This function will register the driver
 *   as /dev/pdm_microphone.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int nrf52_pdm_microphone_initialize(void)
{
  FAR struct audio_lowerhalf_s *pdm_mic;
  FAR struct audio_lowerhalf_s *pcm;
  FAR struct pdm_dev_s *pdm;
  static bool initialized = false;
  int ret;

  /* Have we already initialized?  Since we never uninitialize we must prevent
   * multiple initializations.  This is necessary, for example, when the
   * touchscreen example is used as a built-in application in NSH and can be
   * called numerous time.  It will attempt to initialize each time.
   */

  if (!initialized)
    {

      /* Get an instance of the I2S interface for the pcm510x data channel */

      pdm = nrf52_pdm_initialize();
      if (!pdm)
        {
          auderr("ERROR: Failed to initialize NRF52 PDM\n");
          ret = -ENODEV;
          goto errout;
        }

      /* Now we can use these pdm interfaces to initialize the
       * pdm_microphone which will return an audio interface.
       */

      pdm_mic = pdm_mic_initialize(pdm);
      if (!pdm_mic)
        {
          auderr("ERROR: Failed to initialize the pdm microphone\n");
          ret = -ENODEV;
          goto errout;
        }

      /* No we can embed the pdm_mic/pdm conglomerate into a PCM encoder
       * instance so that we will have a PCM front end for the the pdm_microphone
       * driver.
       */

      pcm = pcm_encode_initialize(pdm_mic);
      if (!pcm)
        {
          auderr("ERROR: Failed create the PCM encoder\n");
          ret = -ENODEV;
          goto errout;
        }


      /* Finally, we can register the PCM/pdm_mic/PDM audio device.
       */

      ret = audio_register("pdm_microphone", (FAR struct audio_lowerhalf_s *)pcm);
      if (ret < 0)
        {
          auderr("ERROR: Failed to register /dev/pdm_microphone device: %d\n", ret);
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

#endif /*CONFIG_AUDIO_PDM_MIC*/