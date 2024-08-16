/****************************************************************************
 * arch/arm/src/nrf52/nrf52_comp.c
 *
 *   Copyright (C) 2011, 2013, 2015-2017 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2017 Zglue  Inc. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Zhengwei Wang <zhengwei@zglue.com>
 *
 * Based on COMP driver from the Motorola MDK:
 *
 *   Copyright (c) 2016 Motorola Mobility, LLC. All rights reserved.
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

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <semaphore.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <unistd.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/analog/comp.h>
#include <nuttx/analog/ioctl.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "nrf.h"
#include "nrf_comp.h"
#include "nrf_lpcomp.h"
#include "nrf52_comp.h"


#include <errno.h>

#if (defined CONFIG_NRF52_COMP) ||(defined CONFIG_NRF52_LPCOMP)

/****************************************************************************
 * Pre-processor Definitions
 */
#define EVT_TO_STR(event)   (event == NRF_LPCOMP_EVENT_READY ? "NRF_COMP_EVENT_READY" :                \
                            (event == NRF_LPCOMP_EVENT_DOWN ? "NRF_COMP_EVENT_DOWN" :                  \
                            (event == NRF_LPCOMP_EVENT_UP ? "NRF_COMP_EVENT_UP" :                      \
                            (event == NRF_LPCOMP_EVENT_CROSS ? "NRF_COMP_EVENT_CROSS" : "UNKNOWN EVENT"))))


/****************************************************************************
 * Private Types
 ****************************************************************************/
struct nrf52_comp_priv_s
{
  int intf; /*0 stands for comp, 1 stands for lpcomp*/
  struct nrf52_comp_int_cfg int_cfg; /*interrupt config*/
  union
  {
    struct nrf52_comp_reg_config_s comp_reg_cfg;
    struct nrf52_lpcomp_reg_config_s lpcomp_reg_cfg;
  } reg_cfg;/*register config*/
};


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* COMP Register access */
static int nrf52_comp_isr(int irq, FAR void *context, FAR void *arg);
static int nrf52_comp_config_int(FAR const struct nrf52_comp_int_cfg *cfg);
static int nrf52_comp_config_reg(FAR const struct nrf52_comp_reg_config_s *cfg);
static int nrf52_lpcomp_config_reg(FAR const struct nrf52_lpcomp_reg_config_s *cfg);


/* COMP Driver Methods */

static void nrf52_comp_shutdown(FAR struct comp_dev_s *dev);
static int nrf52_comp_setup(FAR struct comp_dev_s *dev);
static int nrf52_comp_read(FAR struct comp_dev_s *dev);
static int nrf52_comp_ioctl(FAR struct comp_dev_s *dev, int cmd,
                            unsigned long arg);
static int nrf52_comp_bind(FAR struct comp_dev_s *dev,
                           FAR const struct comp_callback_s *callback);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct comp_ops_s g_compops =
{
  .ao_shutdown  = nrf52_comp_shutdown,
  .ao_setup     = nrf52_comp_setup,
  .ao_read      = nrf52_comp_read,
  .ao_ioctl     = nrf52_comp_ioctl,
  .ao_bind      = nrf52_comp_bind,
};

#ifdef CONFIG_NRF52_COMP

static struct nrf52_comp_priv_s g_comp0priv =
{
  .intf = 0,
  .int_cfg    =
  {
    .cb         = NULL, /* will be bound to upper-half driver */
    .rising     = true,
    .falling    = true,
  },
  .reg_cfg.comp_reg_cfg =
  {
    .input = NRF_COMP_INPUT_3,
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
  },
};

static struct comp_dev_s g_comp0dev =
{
  .ad_ops       = &g_compops,
  .ad_priv      = &g_comp0priv,
};

#endif

#ifdef CONFIG_NRF52_LPCOMP

static struct nrf52_comp_priv_s g_comp1priv =
{
  .intf = 1,
  .int_cfg    =
  {
    .cb         = NULL, /* will be bound to upper-half driver */
    .rising     = true,
    .falling    = true,
  },
  .reg_cfg.lpcomp_reg_cfg =
  {
    .input = NRF_LPCOMP_INPUT_3,
    .low_cfg =
    {
      .reference = NRF_LPCOMP_REF_SUPPLY_4_8,
      .detection = NRF_LPCOMP_DETECT_CROSS,
      .hyst = NRF_LPCOMP_HYST_50mV,
    },
  },
};

static struct comp_dev_s g_comp1dev =
{
  .ad_ops       = &g_compops,
  .ad_priv      = &g_comp1priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: comp_setup
 *
 * Description:
 *   Configure the COMP. This method is called the first time that the COMP
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching COMP interrupts.
 *
 * Input Parameters:
 *   dev - pointer to device structure used by the driver
 *
 * Returned Value:
 *
 ****************************************************************************/

static int nrf52_comp_setup(FAR struct comp_dev_s *dev)
{
  FAR struct nrf52_comp_priv_s *priv =
    (FAR struct nrf52_comp_priv_s *)dev->ad_priv;

  if (priv->intf == 0)
    {
      nrf52_comp_config_reg(&priv->reg_cfg.comp_reg_cfg);
    }
  else if (priv->intf == 1)
    {
      nrf52_lpcomp_config_reg(&priv->reg_cfg.lpcomp_reg_cfg);
    }
  else
    {
      aerr("unsupported interface!\n");
      return -ENODEV;
    }

  /*COMP and LPCOMP share the same intterupt source register,so we use this
   *function to config the interrupt source.
   */
  nrf52_comp_config_int(&priv->int_cfg);

  /*attach interrupt handler*/
  irq_attach(COMP_LPCOMP_IRQn, nrf52_comp_isr, dev);


  if (priv->intf == 0)
    {
      /*enable the COMP peripheral*/
      nrf_comp_enable();

      /*trigger a start task and wait it be ready(this cost no more than 40us)*/
      nrf_comp_task_trigger(NRF_COMP_TASK_START);
      while (!nrf_comp_event_check(NRF_COMP_EVENT_READY));
    }
  else
    {
      /*enable the LPCOMP peripheral*/
      nrf_lpcomp_enable();

      /*trigger a start task and wait it be ready(this cost no more than 140us)*/
      nrf_lpcomp_task_trigger(NRF_COMP_TASK_START);
      while (!nrf_lpcomp_event_check(NRF_COMP_EVENT_READY));
    }

  /*enable interrupt*/
  up_enable_irq(COMP_LPCOMP_IRQn);

  return OK;
}

/****************************************************************************
 * Name: comp_shutdown
 *
 * Description:
 *   Disable the COMP.  This method is called when the COMP device is closed.
 *   This method reverses the operation the setup method.
 *   Works only if COMP device is not locked.
 *
 * Input Parameters:
 *   dev - pointer to device structure used by the driver
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nrf52_comp_shutdown(FAR struct comp_dev_s *dev)
{
  FAR struct nrf52_comp_priv_s *priv =
    (FAR struct nrf52_comp_priv_s *)dev->ad_priv;

  if (priv->intf == 0)
    {
      nrf_comp_task_trigger(NRF_COMP_TASK_STOP);
      nrf_comp_disable();
    }
  else if (priv->intf == 1)
    {
      nrf_lpcomp_task_trigger(NRF_COMP_TASK_STOP);
      nrf_lpcomp_disable();
    }
  else
    {
      aerr("unsupported interface!\n");
    }

  up_disable_irq(COMP_LPCOMP_IRQn);
  irq_detach(COMP_LPCOMP_IRQn);
}

/****************************************************************************
 * Name: nrf52_comp_read
 *
 * Description:
 *  Get the COMP output state.
 *
 * Input Parameters:
 *   dev - pointer to device structure used by the driver
 *
 * Returned Value:
 *   0 if output is low (non-inverting input below inverting input),
 *   1 if output is high (non inverting input above inverting input).
 *
 ****************************************************************************/

static int nrf52_comp_read(FAR struct comp_dev_s *dev)
{
  FAR struct nrf52_comp_priv_s *priv =
    (FAR struct nrf52_comp_priv_s *)dev->ad_priv;
  int ret;

  if (priv->intf == 0)
    {
      nrf_comp_task_trigger(NRF_COMP_TASK_SAMPLE);
      ret = nrf_comp_result_get();
    }
  else if (priv->intf == 1)
    {
      nrf_lpcomp_task_trigger(NRF_LPCOMP_TASK_SAMPLE);
      ret = nrf_lpcomp_result_get();
    }
  else
    {
      aerr("unsupported interface!\n");
      return -ENODEV;
    }

  return ret;
}

/****************************************************************************
 * Name: nrf52_comp_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *
 * Input Parameters:
 *   dev - pointer to device structure used by the driver
 *   cmd - command
 *   arg - arguments passed with command
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nrf52_comp_ioctl(FAR struct comp_dev_s *dev, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: nrf52_comp_bind
 *
 * Description:
 *   Bind upper half callback.
 *
 * Input Parameters:
 *   dev - pointer to device structure used by the driver
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nrf52_comp_bind(FAR struct comp_dev_s *dev,
                           FAR const struct comp_callback_s *callback)
{
  FAR struct nrf52_comp_priv_s *priv =
    (FAR struct nrf52_comp_priv_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->int_cfg.cb = callback;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_comp_interrupt
 *
 * Description:
 *   Common comparator interrupt handler.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static int nrf52_comp_isr(int irq, FAR void *context, FAR void *arg)
{
  FAR struct comp_dev_s *dev = (FAR struct comp_dev_s *)arg;
  FAR struct nrf52_comp_priv_s *priv =
    (FAR struct nrf52_comp_priv_s *)(dev->ad_priv);

  if ( nrf_comp_event_check(NRF_COMP_EVENT_READY))
    {
      nrf_comp_event_clear(NRF_COMP_EVENT_READY);
      ainfo("Event: %s.\r\n", EVT_TO_STR(NRF_COMP_EVENT_READY));
      nrf_comp_int_disable(COMP_INTEN_READY_Msk);
    }

  if (nrf_comp_event_check(NRF_COMP_EVENT_UP))
    {
      nrf_comp_event_clear(NRF_COMP_EVENT_UP);
      ainfo("Event: %s.\r\n", EVT_TO_STR(NRF_COMP_EVENT_UP));
      if ((priv->int_cfg.rising == true) && (priv->int_cfg.falling == false))
        {
          if (priv->int_cfg.cb->au_notify)
            {
              priv->int_cfg.cb->au_notify(dev, nrf52_comp_read(dev));
            }
        }
    }

  if (nrf_comp_event_check(NRF_COMP_EVENT_DOWN))
    {
      nrf_comp_event_clear(NRF_COMP_EVENT_DOWN);
      ainfo("Event: %s.\r\n", EVT_TO_STR(NRF_COMP_EVENT_UP));
      if ((priv->int_cfg.rising == false) && (priv->int_cfg.falling == true))
        {
          if (priv->int_cfg.cb->au_notify)
            {
              priv->int_cfg.cb->au_notify(dev, nrf52_comp_read(dev));
            }
        }
    }

  if (nrf_comp_event_check(NRF_COMP_EVENT_CROSS))
    {
      nrf_comp_event_clear(NRF_COMP_EVENT_CROSS);
      ainfo("Event: %s.\r\n", EVT_TO_STR(NRF_COMP_EVENT_UP));
      if ((priv->int_cfg.rising == true) && (priv->int_cfg.falling == true))
        {
          if (priv->int_cfg.cb && priv->int_cfg.cb->au_notify)
            {
              priv->int_cfg.cb->au_notify(dev, nrf52_comp_read(dev));
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: nrf52_comp_config_int
 *
 * Description:
 *   Configure COMP and LPCOMP interrupt source
 *
 * Input Parameters:
 *  cfg - configuration
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int nrf52_comp_config_int(FAR const struct nrf52_comp_int_cfg *cfg)
{
  // Clear events to be sure there are no leftovers.
  nrf_comp_event_clear(NRF_COMP_EVENT_READY);
  nrf_comp_event_clear(NRF_COMP_EVENT_DOWN);
  nrf_comp_event_clear(NRF_COMP_EVENT_UP);
  nrf_comp_event_clear(NRF_COMP_EVENT_CROSS);

  nrf_comp_int_disable(COMP_INTEN_READY_Msk | COMP_INTEN_DOWN_Msk | COMP_INTEN_UP_Msk | COMP_INTEN_CROSS_Msk);

  if (cfg->rising && cfg->falling)
    {
      nrf_comp_int_enable(COMP_INTEN_CROSS_Msk);
    }
  else if (cfg->rising)
    {
      nrf_comp_int_enable(COMP_INTEN_UP_Msk);
    }
  else if (cfg->falling)
    {
      nrf_comp_int_enable(COMP_INTEN_DOWN_Msk);
    }

  return OK;

}

/****************************************************************************
 * Name: nrf52_comp_config_reg
 *
 * Description:
 *   Configure COMP register
 *
 * Input Parameters:
 *  cfg - configuration
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int nrf52_comp_config_reg(FAR const struct nrf52_comp_reg_config_s *cfg)
{
  /*disable comparator device*/
  nrf_comp_task_trigger(NRF_COMP_TASK_STOP);
  nrf_comp_disable();

  nrf_comp_shorts_disable(COMP_SHORTS_READY_SAMPLE_Pos | COMP_SHORTS_READY_STOP_Msk |
                          COMP_SHORTS_CROSS_STOP_Msk | COMP_SHORTS_UP_STOP_Msk |
                          COMP_SHORTS_DOWN_STOP_Msk);

  /* set negative end input */
  nrf_comp_ref_set(cfg->reference);
  /* set positive end input */
  nrf_comp_input_select(cfg->input);
  nrf_comp_ext_ref_set(cfg->ext_ref);

  nrf_comp_th_set(cfg->threshold);
  nrf_comp_main_mode_set(cfg->main_mode);
  nrf_comp_speed_mode_set(cfg->speed_mode);

#if defined(COMP_ISOURCE_ISOURCE_Msk)
  nrf_comp_isource_set(cfg->isource);
#endif

  nrf_comp_hysteresis_set(cfg->hyst);

  return OK;
}


/****************************************************************************
 * Name: nrf52_lpcompconfig_reg
 *
 * Description:
 *   Configure LPCOMP register
 *
 * Input Parameters:
 *  cfg - configuration
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int nrf52_lpcomp_config_reg(FAR const struct nrf52_lpcomp_reg_config_s *cfg)
{
  nrf_lpcomp_configure(&cfg->low_cfg);
  nrf_lpcomp_input_select(cfg->input);
  return OK;
}

/****************************************************************************
 * Name: nrf52_compinitialize
 *
 * Description:
 *   Initialize the COMP.
 *
 * Input Parameters:
 *   interface - The COMP interface number.
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
                                            FAR const void *reg_cfg)
{
  FAR struct comp_dev_s *dev;
  FAR struct nrf52_comp_priv_s *priv;

  switch (interface)
    {
#ifdef CONFIG_NRF52_COMP
      case 0:
        ainfo("COMP selected\n");
        dev = &g_comp0dev;
        priv = (FAR struct nrf52_comp_priv_s *)g_comp0dev.ad_priv;
        break;
#endif

#ifdef CONFIG_NRF52_LPCOMP
      case 1:
        ainfo("LPCOMP selected\n");
        dev = &g_comp1dev;
        priv = (FAR struct nrf52_comp_priv_s *)g_comp1dev.ad_priv;
        break;
#endif

      default:
        aerr("ERROR: No COMP interface defined\n");
        return NULL;
    }

  if (int_cfg)
    {
      memcpy(&(priv->int_cfg), int_cfg, sizeof(struct nrf52_comp_int_cfg));
    }

  if (reg_cfg)
    {
      if (priv->intf == 0)
        {
          memcpy(&(priv->reg_cfg.comp_reg_cfg), reg_cfg, sizeof(struct nrf52_comp_reg_config_s));
        }
      else
        {
          memcpy(&(priv->reg_cfg.lpcomp_reg_cfg), reg_cfg, sizeof(struct nrf52_lpcomp_reg_config_s));
        }
    }

  return dev;
}

#endif /* CONFIG_NRF52_COMP_LPCOMP */
