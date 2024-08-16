/**************************************************************************
 * configs/nrf52840_dk/src/nrf52_comp.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/analog/comp.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "nrf.h"
#include "nrf_comp.h"
#include "nrf_lpcomp.h"
#include "nrf52_comp.h"

#ifdef CONFIG_COMP

#if !defined(CONFIG_NRF52_COMP) && !defined(CONFIG_NRF52_LPCOMP)
#  error "comparater device is required,you should enable CONFIG_NRF52_COMP or CONFIG_NRF52_LPCOMP"
#endif

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
 * Name: nrf52_comp_initialize
 *
 * Description:
 *   This function is called by platform-specific, setup logic to configure
 *   and register comp device.  This function will register the driver
 *   as /dev/comp or /dev/lpcomp.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int nrf52_comp_initialize(void)
{
  FAR struct comp_dev_s *comp;
  static bool initialized = false;
  int ret;

  struct nrf52_comp_int_cfg interrupt =
  {
    .rising     = true,
    .falling    = true,
  };

#ifdef CONFIG_NRF52_COMP
  char *path = "/dev/comp";
#else
  char *path = "/dev/lpcomp";
#endif

  /* Have we already initialized?  Since we never uninitialize we must prevent
   * multiple initializations.  This is necessary, for example, when the
   * touchscreen example is used as a built-in application in NSH and can be
   * called numerous time.  It will attempt to initialize each time.
   */

  if (!initialized)
    {
#ifdef CONFIG_NRF52_COMP
      struct nrf52_comp_reg_config_s comp_config =
      {
        .input = NRF_COMP_INPUT_1,
        .reference = COMP_REFSEL_REFSEL_Int1V8,
        .ext_ref = NRF_COMP_EXT_REF_0,
        .threshold.th_down = 60,
        .threshold.th_up = 63,
        .main_mode = NRF_COMP_MAIN_MODE_SE,
        .speed_mode = NRF_COMP_SP_MODE_Low,
        .hyst = NRF_COMP_HYST_50mV,
#if defined(COMP_ISOURCE_ISOURCE_Msk)
        .isource = NRF_COMP_ISOURCE_Off,
#endif
      };

      /* Get an instance of the comp peripheral*/
      comp = nrf52_compinitialize(0, &interrupt, (void *)&comp_config);
#else

      struct nrf52_lpcomp_reg_config_s lpcomp_config =
      {
        .input = NRF_LPCOMP_INPUT_1,
        .low_cfg =
        {
          .reference = NRF_LPCOMP_REF_SUPPLY_4_8,
          .detection = NRF_LPCOMP_DETECT_CROSS,
          .hyst = NRF_LPCOMP_HYST_50mV,
        },
      };

      /* Get an instance of the lpcomp peripheral*/
      comp = nrf52_compinitialize(1, &interrupt, (void *)&lpcomp_config);
#endif

      if (!comp)
        {
          aerr("ERROR: Failed to initialize NRF52 COMP/LPCOMP\n");
          ret = -ENODEV;
          goto errout;
        }

      /* Then, we register this COMP peripheral as a char device.
       */

      ret =   comp_register(path, comp);
      if (ret < 0)
        {
          aerr("ERROR: Failed to register %s device: %d\n", path, ret);
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

#endif /*CONFIG_COMP*/