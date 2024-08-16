/************************************************************************************
 * arch/arm/src/nrf52/nrf_qdecoder.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2017 Zglue  Inc. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Levin Li <zhiqiang@zglue.com>
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/sensors/qdecoder.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_internal.h"
#include "up_arch.h"

#include "nrf.h"

#include "nrf_qdec.h"
#include "nrf52_gpio.h"
#include "nrf52_qdec.h"

#ifdef CONFIG_NRF52_QDECODER

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Clocking *************************************************************************/


/* Input filter *********************************************************************/

#ifdef CONFIG_NRF52_QDEC_LED

#define CONFIG_NRF52_QDEC_LED_OUTPUT CONFIG_NRF52_QDEC_LED_PIN

#ifdef CONFIG_NRF52_QDEC_LED_ACTIVE_HIGH
#define CONFIG_NRF52_QDEC_LED_ACTIVE 1
#endif

#ifdef CONFIG_NRF52_QDEC_LED_ACTIVE_LOW
#define CONFIG_NRF52_QDEC_LED_ACTIVE 0
#endif

#else

#define CONFIG_NRF52_QDEC_LED_OUTPUT  0xFF

#define CONFIG_NRF52_QDEC_LED_ACTIVE 0

#endif
/* Debug ****************************************************************************/
/* Non-standard debug that may be enabled just for testing the quadrature encoder */

#define qdinfo    _info
#define qdwarn    _warn
#define qderr     _err

/************************************************************************************
 * Private Types
 ************************************************************************************/

#define NRF52_QDEC_PRIORITY  7
#define INVALID_SAMPLE  (2)
/* Constant configuration structure that is retained in FLASH */

/* Overall, RAM-based state structure */

struct nrf52_qdec_s
{
  /* The first field of this state structure must be a pointer to the lower-
   * half callback structure:
   */

  FAR const struct qd_ops_s *ops;  /* Lower half callback structure */

  /* NRF52 driver-specific fields: */
  uint8_t  irqid;
  uint8_t  irq_mask;
  uint8_t  pinsel_a;        /* Phase A pin */
  uint8_t  pinsel_b;        /* Phase B pin */
  uint8_t  pinsel_led;      /* LED pin */
  uint8_t  led_polarity;    /* LED active polarity */
  uint8_t  debounceflag;    /* debounce filter flag */
  uint8_t  state;           /* driver status */

  sem_t   cnt_notice;
  int32_t waitercnt;          /* decoder cnt */
  int32_t qdec_cnt;

  qdec_sample_period_s sample_period;

};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/
/* Helper functions */


/* Lower-half Quadrature Encoder Driver Methods */

static int nrf52_setup(FAR struct qd_lowerhalf_s *lower);
static int nrf52_shutdown(FAR struct qd_lowerhalf_s *lower);
static int nrf52_start(FAR struct qd_lowerhalf_s *lower);
static int nrf52_stop(FAR struct qd_lowerhalf_s *lower);
static int nrf52_reset(FAR struct qd_lowerhalf_s *lower);
static int nrf52_ioctl(FAR struct qd_lowerhalf_s *lower, int cmd, unsigned long arg);

/************************************************************************************
 * Private Data
 ************************************************************************************/
/* The lower half callback structure */

static const struct qd_ops_s g_qdecops =
{
  .setup    = nrf52_setup,
  .shutdown = nrf52_shutdown,
  .start    = nrf52_start,
  .stop     = nrf52_stop,
  .reset    = nrf52_reset,
  .ioctl    = nrf52_ioctl,
};

static struct nrf52_qdec_s g_qdec_dev =
{
  .ops          = &g_qdecops,
  .irqid        = QDEC_IRQn,
  .pinsel_a     = NRF52_QDEC_PHASE_A,
  .pinsel_b     = NRF52_QDEC_PHASE_B,
  .pinsel_led   = NRF52_QDEC_LED_OUTPUT,
  .led_polarity = CONFIG_NRF52_QDEC_LED_ACTIVE,
  .state        = NRF_DRV_STATE_UNINITIALIZED
};
/************************************************************************************
 * Private Functions
 ************************************************************************************/


/************************************************************************************
 * Name: nrf52_interrupt
 *
 * Description:
 *   Common timer interrupt handling.  NOTE: Only 16-bit timers require timer
 *   interrupts.
 *
 ************************************************************************************/

static int nrf52_qdec_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct nrf52_qdec_s *priv = (FAR struct nrf52_qdec_s *)arg;
  int16_t sample;

  DEBUGASSERT(priv != NULL);

  if ( nrf_qdec_event_check(NRF_QDEC_EVENT_SAMPLERDY) &&
       nrf_qdec_int_enable_check(NRF_QDEC_INT_SAMPLERDY_MASK) )
    {
      nrf_qdec_event_clear(NRF_QDEC_EVENT_SAMPLERDY);

      sample = (int16_t)nrf_qdec_sample_get();
      if (INVALID_SAMPLE != sample)
        {
          priv->qdec_cnt += sample;
        }

      irqinfo("Event: NRF_QDEC_EVENT_SAMPLERDY, total : %d , sample %d.\n", nrf_qdec_acc_get(), sample);

      if (priv->qdec_cnt >= priv->waitercnt)
        {
          nrf_qdec_task_trigger(NRF_QDEC_TASK_STOP);
          sem_post(&priv->cnt_notice);
        }
    }

  if ( nrf_qdec_event_check(NRF_QDEC_EVENT_REPORTRDY) &&
       nrf_qdec_int_enable_check(NRF_QDEC_INT_REPORTRDY_MASK) )
    {
      nrf_qdec_event_clear(NRF_QDEC_EVENT_REPORTRDY);
      irqinfo("Event: NRF_QDEC_INT_REPORTRDY_MASK, total : %d, period: %d, reporter: %d.\n",
              nrf_qdec_acc_get(), nrf_qdec_sampleper_reg_get(), nrf_qdec_reportper_reg_get());

      priv->qdec_cnt = nrf_qdec_acc_get();

      if (priv->qdec_cnt >= priv->waitercnt)
        {
          nrf_qdec_task_trigger(NRF_QDEC_TASK_STOP);
          sem_post(&priv->cnt_notice);
        }
    }

  if ( nrf_qdec_event_check(NRF_QDEC_EVENT_ACCOF) &&
       nrf_qdec_int_enable_check(NRF_QDEC_INT_ACCOF_MASK) )
    {
      nrf_qdec_event_clear(NRF_QDEC_EVENT_ACCOF);
      irqwarn("Event: NRF_QDEC_EVENT_ACCOF.\n");
    }

  return OK;
}

/************************************************************************************
 * Name: nrf52_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   The initial position value should be zero. *
 *
 ************************************************************************************/

static int nrf52_setup(FAR struct qd_lowerhalf_s *lower)
{
  FAR struct nrf52_qdec_s *priv = (FAR struct nrf52_qdec_s *)lower;


  nrf_gpio_cfg_input(priv->pinsel_a, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(priv->pinsel_b, NRF_GPIO_PIN_NOPULL);

  if (0xFF == priv->pinsel_led)
    {
      nrf_qdec_pio_assign( priv->pinsel_a, priv->pinsel_b, 0xFFFFFFFF);
    }
  else
    {
      nrf_gpio_cfg_input(priv->pinsel_led, NRF_GPIO_PIN_NOPULL);
      nrf_qdec_pio_assign( priv->pinsel_a, priv->pinsel_b, priv->pinsel_led);
      nrf_qdec_ledpol_set(priv->led_polarity);
    }

  if (priv->debounceflag)
    {
      nrf_qdec_dbfen_enable();
    }
  else
    {
      nrf_qdec_dbfen_disable();
    }

  irq_attach(priv->irqid, nrf52_qdec_interrupt, (FAR void *)lower);

  priv->state = NRF_DRV_STATE_INITIALIZED;

  qdinfo("setup-qdec.\n");

  return OK;
}

/************************************************************************************
 * Name: nrf52_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   should stop data collection, free any resources, disable timer hardware, and
 *   put the system into the lowest possible power usage state *
 *
 ************************************************************************************/

static int nrf52_shutdown(FAR struct qd_lowerhalf_s *lower)
{
  FAR struct nrf52_qdec_s *priv = (FAR struct nrf52_qdec_s *)lower;
  irqstate_t flags;

  /* disable qdec firstly */

  nrf_qdec_task_trigger(NRF_QDEC_TASK_STOP);
  nrf_qdec_disable();

  /* Disable the update/global interrupt at the NVIC */

  flags = enter_critical_section();

  up_disable_irq(priv->irqid);

  /* Detach the interrupt handler */

  irq_detach(priv->irqid);

  priv->state = NRF_DRV_STATE_UNINITIALIZED;

  leave_critical_section(flags);

  qdinfo("shutdown-qdec.\n");

  return OK;
}

/************************************************************************************
 * Name: nrf52_start
 *
 * Description:
 *   Start queadrature decoder.
 *
 ************************************************************************************/

static int nrf52_start(FAR struct qd_lowerhalf_s *lower)
{
  FAR struct nrf52_qdec_s *priv = (FAR struct nrf52_qdec_s *)lower;

  if (NRF_DRV_STATE_POWERED_ON == priv->state)
    {
      return OK;
    }

  ASSERT(priv->state == NRF_DRV_STATE_INITIALIZED);

  nrf_qdec_enable();
  nrf_qdec_task_trigger(NRF_QDEC_TASK_START);

  priv->state = NRF_DRV_STATE_POWERED_ON;
  qdinfo("Enabled.\n");

  return OK;
}

/************************************************************************************
 * Name: nrf52_stop
 *
 * Description:
 *   Stop queadrature decoder.
 *
 ************************************************************************************/

static int nrf52_stop(FAR struct qd_lowerhalf_s *lower)
{
  FAR struct nrf52_qdec_s *priv = (FAR struct nrf52_qdec_s *)lower;

  if (priv->state == NRF_DRV_STATE_POWERED_ON)
    {
      return OK;
    }

  nrf_qdec_task_trigger(NRF_QDEC_TASK_READCLRACC);
  nrf_qdec_task_trigger(NRF_QDEC_TASK_STOP);
  nrf_qdec_disable();

  priv->state = NRF_DRV_STATE_INITIALIZED;
  qdinfo("Disabled.\n");

  return OK;
}

/************************************************************************************
 * Name: nrf52_reset
 *
 * Description:
 *   reset queadrature decoder.
 *
 ************************************************************************************/

static int nrf52_reset(FAR struct qd_lowerhalf_s *lower)
{
  FAR struct nrf52_qdec_s *priv = (FAR struct nrf52_qdec_s *)lower;

  ASSERT(priv->state == NRF_DRV_STATE_POWERED_ON);

  nrf_qdec_task_trigger(NRF_QDEC_TASK_READCLRACC);

  qdinfo("reset position.\n");

  return OK;
}

/************************************************************************************
 * Name: nrf52_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 ************************************************************************************/

static int nrf52_ioctl(FAR struct qd_lowerhalf_s *lower, int cmd, unsigned long arg)
{
  FAR struct nrf52_qdec_s *priv = (FAR struct nrf52_qdec_s *)lower;
  int ret = -ENOTTY;

  /* TODO add an IOCTL to control the encoder pulse count prescaler */
  if (QDIOC_SETTING == cmd)
    {
      FAR qdec_setting_s *setting = (FAR qdec_setting_s *)arg;
      uint32_t ledpre;

      ASSERT(setting != NULL);

      if (NRF_DRV_STATE_INITIALIZED != priv->state)
        {
          qderr("Qdec is not under right status [%d]\n", priv->state);
          return -EBUSY;
        }
      nrf_qdec_sampleper_set(setting->sample_period);

      /* caculate led period according sample period
       * ledpre = sample_period / 2: only 9bit valid
       */

      ledpre = 1 << (setting->sample_period + 6);
      ledpre /= 2;
      ledpre %= ((1 << 9) - 1);
      nrf_qdec_ledpre_set(ledpre);

      if (setting->dfilter_enable)
        {
          priv->debounceflag = true;
          nrf_qdec_dbfen_enable();
        }
      else
        {
          priv->debounceflag = false;
          nrf_qdec_dbfen_disable();
        }

      ret = OK;
    }
  else if (QDIOG_POSITION == cmd)
    {
      FAR qdec_data_s *data = (FAR qdec_data_s *)arg;
      int32_t sample;
      ASSERT(data != NULL);

      if (NRF_DRV_STATE_POWERED_ON != priv->state)
        {
          qderr("Qdec is not under right status [%d]\n", priv->state);
          return -EBUSY;
        }

      memset(data, 0, sizeof(qdec_data_s));
      data->x    = (int16_t)nrf_qdec_acc_get();
      data->xflag = true;
      sample = nrf_qdec_sample_get();
      if (-2 == sample)
        {
          qdwarn("Invalid Direction\n");
          sample = 0;
        }
      ret = OK;
    }
  else if (QDIOC_DEC_CNT == cmd)
    {
      priv->waitercnt = (int32_t)arg;
      priv->qdec_cnt = 0;

      /* if the waiter count is multiple 10 , using reporter interrupt */

      if ((priv->waitercnt / 10) && (!(priv->waitercnt % 10)))
        {
          nrf_qdec_reportper_set(NRF_QDEC_REPORTPER_10);
          priv->irq_mask = NRF_QDEC_INT_REPORTRDY_MASK;
        }
      else
        {
          priv->irq_mask = NRF_QDEC_INT_SAMPLERDY_MASK;
        }

      priv->irq_mask |= NRF_QDEC_INT_ACCOF_MASK;
      nrf_qdec_int_enable(priv->irq_mask);
      nrf_drv_common_irq_enable(QDEC_IRQn, NRF52_QDEC_PRIORITY);
      nrf52_start(lower);
      sem_wait(&priv->cnt_notice);

      nrf_qdec_int_disable(priv->irq_mask);

      nrf52_stop(lower);
      qdwarn("Had Generated  %d  sample.\n", priv->waitercnt);
      priv->waitercnt = 0;
      ret = OK;
    }
  return ret;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: nrf52_qeinitialize
 *
 * Description:
 *   Initialize a quadrature encoder interface.  This function must be called from
 *   board-specific logic.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/qdec0"
 *
 * Returned Values:
 *   Zero on success; A negated errno value is returned on failure.
 *
 ************************************************************************************/

int nrf52_qdec_initialize(FAR const char *devpath)
{
  FAR struct nrf52_qdec_s *priv = (FAR struct nrf52_qdec_s *)&g_qdec_dev;
  int ret;


  /* Make sure that it is available */

  if (NRF_DRV_STATE_UNINITIALIZED != priv->state)
    {
      qderr("Error: Qdec under wrong state [%d]\n", priv->state);
      return -EBUSY;
    }

  /* Initialize semaphores */

  sem_init(&priv->cnt_notice, 0, 0);
  sem_setprotocol(&priv->cnt_notice, SEM_PRIO_NONE);

  /* Register the priv-half driver */
  ret = qd_register(devpath, (FAR struct qd_lowerhalf_s *)priv);
  if (ret < 0)
    {
      qderr("ERROR: qe_register failed: %d\n", ret);
      return ret;
    }

  return OK;
}

#endif /* CONFIG_QDECODER */
