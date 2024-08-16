/************************************************************************************
 * arch/arm/src/nrf52/nrf52_comp.h
 *
 *   Copyright (C) 2009, 2011, 2017 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2017   zGlue Inc. All Rights Reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Zhengwei Wang  <zhengwei@zglue.com>
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

#ifndef __ARCH_ARM_SRC_NRF52_NRF52_COMP_H
#define __ARCH_ARM_SRC_NRF52_NRF52_COMP_H


/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include "nrf.h"
#include "nrf_comp.h"
#include "nrf_lpcomp.h"

#if (defined CONFIG_NRF52_COMP) ||(defined CONFIG_NRF52_LPCOMP)

#if (defined CONFIG_NRF52_COMP) && (defined CONFIG_NRF52_LPCOMP)
# error "COMP and LPCOMP can't be enabled at the same time!"
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/
/**@brief COMP configuration.
 */
struct nrf52_comp_reg_config_s
{
  nrf_comp_input_t         input;              /**< Input to be monitored. */
  nrf_comp_ref_t           reference;          /**< Reference selection. */
  nrf_comp_ext_ref_t       ext_ref;            /**< External analog reference selection. */
  nrf_comp_th_t            threshold;          /**< Structure holding THDOWN and THUP values needed by the COMP_TH register. */
  nrf_comp_main_mode_t     main_mode;          /**< Main operation mode. */
  nrf_comp_sp_mode_t       speed_mode;         /**< Speed and power mode. */
  nrf_comp_hyst_t          hyst;               /**< Comparator hysteresis.*/
#if defined(COMP_ISOURCE_ISOURCE_Msk)
  nrf_isource_t            isource;            /**< Current source selected on analog input. */
#endif
};

struct nrf52_lpcomp_reg_config_s
{
  nrf_lpcomp_input_t          input; /* LPCOMP input. */
  nrf_lpcomp_config_t         low_cfg; /*This include the configration of
                                        *reference, detection and hyst
                                        */
};

struct nrf52_comp_int_cfg
{
  FAR const struct comp_callback_s *cb;
  bool                              rising;
  bool                              falling;
};



/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: nrf52_compinitialize
 *
 * Description:
 *   Initialize the COMP.
 *
 * Input Parameters:
 *   intf - The COMP interface number.
            0 stands for comp, 1 statnds for lpcomp.
 *   int_cfg - The COMP interrupt configration.
 *   reg_cfg  -The COMP register configuration,
 *       when intf equals 0, this should points to nrf52_comp_reg_config_s type.
 *       when intf equals 1, this should points to nrf52_lpcomp_reg_config_s type.
 *
 * Returned Value:
 *   Valid COMP device structure reference on success; a NULL on failure.
 *
 * Assumptions:
 *   1. Board-specific logic has already configured
 *
 ****************************************************************************/

FAR struct comp_dev_s *nrf52_compinitialize(int interface,
                                            FAR const struct nrf52_comp_int_cfg *int_cfg,
                                            FAR const void *reg_cfg);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* (defined CONFIG_NRF52_COMP) ||(defined CONFIG_NRF52_LPCOMP) */

#endif /* __ARCH_ARM_SRC_NRF52_NRF52_COMP_H */

