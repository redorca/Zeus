/****************************************************************************
 * arch/arm/src/nrf52/nrf52_pwm.c
 *
 *   Copyright (C) 2011-2012, 2016 Gregory Nutt. All rights reserved.
 *   Copyright (c) 2017 Zglue Semiconductor. All Rights Reserved.
 *
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/drivers/pwm.h>
#include <arch/chip/chip.h>
#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"
#include "chip.h"

#include "nrf_pwm.h"
#include "nrf52_pwm.h"
#include "chip/nrf52_gpio.h"

/* This module then only compiles if there is at least one enabled timer
 * intended for use with the PWM upper half driver.
 */

#if defined(CONFIG_NRF52_PWM_M0)  || defined(CONFIG_NRF52_PWM_M1)  || \
    defined(CONFIG_NRF52_PWM_M2)

#define PWM_CONFIG_IRQ_PRIORITY  7
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* PWM/Timer Definitions ****************************************************/
/* The following definitions are used to identify the various time types */


/* Debug ********************************************************************/
//#define  NRF52_TEST_DATA


/****************************************************************************
 * Private Types
 ****************************************************************************/
#define MAX_CLK  (16*1000*1000)
#define MIN_CLK  (125*1000)
#define MAX_TOP_COUNT  (32767)  /* see datasheet : 47.5.7 COUNTERTOP , only 15bits */

/**
 * @brief PWM flags providing additional playback options.
 */
typedef enum
{
  NRF_DRV_PWM_FLAG_STOP = 0x01, /**  When the requested playback is finished,
                                       the peripheral should be stopped.
                                       @note The STOP task is triggered when
                                       the last value of the final sequence is
                                       loaded from RAM, and the peripheral stops
                                       at the end of the current PWM period.
                                       For sequences with configured repeating
                                       of duty cycle values, this might result in
                                       less than the requested number of repeats
                                       of the last value. */
  NRF_DRV_PWM_FLAG_LOOP = 0x02, /**  When the requested playback is finished,
                                       it should be started from the beginning.
                                       This flag is ignored if used together
                                       with @ref NRF_DRV_PWM_FLAG_STOP.
                                       @note The playback restart is done via a
                                       shortcut configured in the PWM peripheral.
                                       This shortcut triggers the proper starting
                                       task when the final value of previous
                                       playback is read from RAM and applied to
                                       the pulse generator counter.
                                       When this mechanism is used together with
                                       the @ref NRF_PWM_STEP_TRIGGERED mode,
                                       the playback restart will occur right
                                       after switching to the final value (this
                                       final value will be played only once). */
} nrf_drv_pwm_flag_t;


/* This structure represents the state of one PWM timer */

struct nrf52_pwm_s
{
  FAR const struct pwm_ops_s *ops;    /* PWM operations */
  struct pwm_pinmux_t         channels[NRF52_MAX_PWM_NCHANNELS];
  uint32_t                    channel_num;
  uint32_t                    base;       /* base of PWM register */
  IRQn_Type                   irqid;      /* IRQ id for PWM */

  /* pre-scaler for PWM controller clock */

  uint16_t                    prescaler;

  /* The frequency of the PWM controller clock */

  uint32_t                    pwm_clk;

  uint32_t                    frequency;  /* Current frequency setting */
  nrf_pwm_mode_t              mode;       /* up or up_down */
  uint16_t                    top_value;  /* countstop value */
  nrfx_drv_state_t volatile    state;
  uint16_t                    duty[NRF52_MAX_PWM_NCHANNELS];
  nrf_pwm_sequence_t          sequence;
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/
/* PWM driver methods */
static int nrf52_pwm_setup(FAR struct pwm_lowerhalf_s *dev);
static int nrf52_pwm_shutdown(FAR struct pwm_lowerhalf_s *dev);

#ifdef CONFIG_PWM_PULSECOUNT
static int nrf52_pwm_start(FAR struct pwm_lowerhalf_s *dev,
                           FAR const struct pwm_info_s *info,
                           FAR void *handle);
#else
static int nrf52_pwm_start(FAR struct pwm_lowerhalf_s *dev,
                           FAR const struct pwm_info_s *info);
#endif

static int nrf52_pwm_stop(FAR struct pwm_lowerhalf_s *dev);
static int nrf52_pwm_ioctl(FAR struct pwm_lowerhalf_s *dev,
                           int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* This is the list of lower half PWM driver methods used by the upper half driver */

static const struct pwm_ops_s g_pwmops =
{
  .setup       = nrf52_pwm_setup,
  .shutdown    = nrf52_pwm_shutdown,
  .start       = nrf52_pwm_start,
  .stop        = nrf52_pwm_stop,
  .ioctl       = nrf52_pwm_ioctl,
};

#ifdef NRF52_TEST_DATA
static nrf_pwm_values_individual_t /* const*/ seq_values[] =
{
  { 0x2000,      0,      0,      0 },
};
#endif

#ifdef CONFIG_NRF52_PWM_M0
static struct nrf52_pwm_s g_pwm0_dev =
{
  .ops      = &g_pwmops,
  .base     = NRF_PWM0_BASE,
  .irqid    = PWM0_IRQn,
  .pwm_clk  = CONFIG_NRF52_PWM_M0_CLK_FREQUENCY,
};
#endif

#ifdef CONFIG_NRF52_PWM_M1
static struct nrf52_pwm_s g_pwm1_dev =
{
  .ops      = &g_pwmops,
  .base     = NRF_PWM1_BASE,
  .irqid    = PWM1_IRQn,
  .pwm_clk  = CONFIG_NRF52_PWM_M1_CLK_FREQUENCY,
};
#endif

#ifdef CONFIG_NRF52_PWM_M2
static struct nrf52_pwm_s g_pwm2_dev =
{
  .ops      = &g_pwmops,
  .base     = NRF_PWM2_BASE,
  .irqid    = PWM2_IRQn,
  .pwm_clk  = CONFIG_NRF52_PWM_M2_CLK_FREQUENCY,
};
#endif


/****************************************************************************
 * Private Functions
 ****************************************************************************/
static uint16_t calculate_prescale(uint32_t pwm_clk)
{
  uint16_t result;
  uint16_t degree = 0;

  result = pwm_clk / MIN_CLK;
  while (result)
    {
      degree++;
      result /= 2;
    }

  return (8 - degree);
}


static uint32_t nrf_drv_pwm_playback(NRF_PWM_Type *p_registers,
                                     nrf_pwm_sequence_t const *p_sequence,
                                     uint16_t                   playback_count,
                                     uint32_t                   flags)
{
  uint32_t shorts_mask;
  bool odd;

  ASSERT(playback_count > 0);
  ASSERT(nrf_is_in_ram(p_sequence->values.p_raw));

  /* To take advantage of the looping mechanism, we need to use
   * both sequences (single sequence can be played back only once).
   */

  nrf_pwm_sequence_set(p_registers, 0, p_sequence);
  nrf_pwm_sequence_set(p_registers, 1, p_sequence);

  odd = (playback_count & 1);
  nrf_pwm_loop_set(p_registers, (playback_count / 2) + (odd ? 1 : 0));

  if (flags & NRF_DRV_PWM_FLAG_STOP)
    {
      shorts_mask = NRF_PWM_SHORT_LOOPSDONE_STOP_MASK;
    }
  else if (flags & NRF_DRV_PWM_FLAG_LOOP)
    {
      shorts_mask = odd ? NRF_PWM_SHORT_LOOPSDONE_SEQSTART1_MASK
                    : NRF_PWM_SHORT_LOOPSDONE_SEQSTART0_MASK;
    }
  else
    {
      shorts_mask = 0;
    }

  nrf_pwm_shorts_set(p_registers, shorts_mask);

  nrf_pwm_event_clear(p_registers, NRF_PWM_EVENT_STOPPED);
  nrf_pwm_task_trigger(p_registers,
                       odd ? NRF_PWM_TASK_SEQSTART1 : NRF_PWM_TASK_SEQSTART0);

  return OK;
}

static int nrf_pwm_calculate_param(struct nrf52_pwm_s *priv,
                                   void *param, uint32_t type)
{
  uint16_t top_value;
  uint32_t frequency;
  uint16_t max_duty = 0;
  FAR struct pwm_info_s *standinfo = (FAR struct pwm_info_s *)param;
  FAR struct nrf52_pwm_info_s *info = (FAR struct nrf52_pwm_info_s *)param;

#ifdef CONFIG_PWM_MULTICHAN
  for (int16_t i = 0; i < CONFIG_PWM_NCHANNELS; i++)
#endif
    {
#ifdef CONFIG_PWM_MULTICHAN
      uint8_t  channel;
      ub16_t   duty;
      uint16_t polarity = 0;

      if (0 == type)
        {
          duty = standinfo->channels[i].duty;
          channel = standinfo->channels[i].channel;
          priv->mode = NRF_PWM_MODE_UP;
          frequency = standinfo->frequency;
        }
      else
        {
          duty = info->channels[i].duty;
          channel = info->channels[i].channel;
          polarity = info->channels[i].polarity;
          priv->mode = info->mode;
          frequency = info->frequency;
        }

      /* A value of zero means to skip this channel */

      if (channel == 0)
        {
          continue;
        }

      /* Find the channel : 0xFF mean channel is not config*/

      if (channel > NRF52_MAX_PWM_NCHANNELS \
          || priv->channels[channel - 1].pincfg == NRF52_INVALID_GPIO_PIN)
        {
          pwmerr("ERROR: total channel %d, no such channel %d\n",
                 NRF52_MAX_PWM_NCHANNELS, channel);
          return -EINVAL;
        }
      /*the nrf52 duty only 15bit , we used the high 15bit from app */
      duty = duty >> 1;
      priv->duty[channel - 1] = duty | (polarity << 15);
      if (duty > max_duty)
        {
          max_duty = duty;
        }
#else

      if (0 == type)
        {
          frequency = standinfo->frequency;
          priv->duty[0] = standinfo->duty >> 1;
          max_duty = standinfo->duty >> 1;
          priv->mode = NRF_PWM_MODE_UP;
        }
      else
        {
          frequency = info->frequency;
          priv->duty[0] = (info->duty >> 1) | (info->polarity << 15);
          max_duty = info->duty >> 1;
          priv->mode = info->mode;
        }


#endif
    }

#ifdef NRF52_TEST_DATA
  priv->sequence.values.p_individual =
    (nrf_pwm_values_individual_t *)seq_values;

  priv->sequence.length = NRF_PWM_VALUES_LENGTH(seq_values);
#else
  priv->sequence.values.p_individual =
    (nrf_pwm_values_individual_t *)priv->duty;

  priv->sequence.length = NRF_PWM_VALUES_LENGTH(priv->duty);
#endif

  priv->sequence.repeats = 0;
  priv->sequence.end_delay = 0;

  /* calculate the countstop value :  the countstop should > duty */

  top_value = priv->pwm_clk / frequency;

  if (max_duty > top_value)
    {
      /* duty is large , should increase CLK by multiple 2
       * because pwm_clk only have 8 option */

      uint32_t clk;
      pwmwarn("countstop %d , duty %d\n", top_value, max_duty);

      clk = priv->pwm_clk * 2;

      /* make sure clk | countstop value is under valid scope
       *new clk > MAX or stopval < duty or  stopval > MAX_VAL */

      if (MAX_CLK < clk || max_duty > 2 * top_value || MAX_TOP_COUNT < 2 * top_value)
        {
          pwmerr("Wrong Param: clk %d, duty %d, countstop %d \n",
                 clk, max_duty, top_value * 2);
          return -EINVAL;
        }
      priv->pwm_clk *= 2;

      /* calculate new count stop */

      top_value = priv->pwm_clk / frequency;
    }

  if (MAX_TOP_COUNT < top_value)
    {
      uint32_t clk;
      pwmwarn("PWM CLK [%d] topval[%d] is too LARGE, will be changed\n",
              clk, top_value);
      clk = priv->pwm_clk / 2;

      if (MIN_CLK > clk || max_duty > top_value / 2)
        {
          pwmerr("CLK %d, max_duty %d ,  countstop %d  NOT matched\n",
                 clk, max_duty, top_value / 2);
          return -EINVAL;
        }

      /* calculate new count stop */

      priv->pwm_clk /= 2;
      top_value = priv->pwm_clk / frequency;
    }

  /* Caculate the pre-scaler */

  priv->prescaler = calculate_prescale(priv->pwm_clk);
  priv->top_value = top_value;
  priv->frequency = frequency;
  pwminfo("Clock: %d, pre-scaler: %d\n", priv->pwm_clk, priv->prescaler);
  pwminfo("duty %d , top_value %d \n", max_duty, top_value);

  return OK;
}

static void nrf_pwm_start(struct nrf52_pwm_s *dev)
{
  FAR struct nrf52_pwm_s *priv = (FAR struct nrf52_pwm_s *)dev;
  FAR NRF_PWM_Type *p_registers = (FAR NRF_PWM_Type *)priv->base;

  nrf_pwm_configure(p_registers, priv->prescaler, NRF_PWM_MODE_UP,
                    priv->top_value);
  nrf_pwm_decoder_set(p_registers, NRF_PWM_LOAD_INDIVIDUAL, NRF_PWM_STEP_AUTO);

  nrf_pwm_shorts_set(p_registers, 0);
  nrf_pwm_int_set(p_registers, 0);
  nrf_pwm_event_clear(p_registers, NRF_PWM_EVENT_LOOPSDONE);
  nrf_pwm_event_clear(p_registers, NRF_PWM_EVENT_SEQEND0);
  nrf_pwm_event_clear(p_registers, NRF_PWM_EVENT_SEQEND1);
  nrf_pwm_event_clear(p_registers, NRF_PWM_EVENT_STOPPED);


  /* Now: didn't use interrupt , then pwm can work under standby mode indepedent */
  /*
  nrf_drv_common_irq_enable(priv->irqid, PWM_CONFIG_IRQ_PRIORITY);
  */
  nrf_drv_pwm_playback( p_registers, &priv->sequence,
                        1, NRF_DRV_PWM_FLAG_LOOP);

  return;
}

static void irq_handler(NRF_PWM_Type *p_pwm, struct nrf52_pwm_s *priv)
{
  /* The user handler is called for SEQEND0 and SEQEND1 events only when the
  *  user asks for it (by setting proper flags when starting the playback). */
  if (nrf_pwm_event_check(p_pwm, NRF_PWM_EVENT_SEQEND0))
    {
      nrf_pwm_event_clear(p_pwm, NRF_PWM_EVENT_SEQEND0);
    }
  if (nrf_pwm_event_check(p_pwm, NRF_PWM_EVENT_SEQEND1))
    {
      nrf_pwm_event_clear(p_pwm, NRF_PWM_EVENT_SEQEND1);
    }

  /* For LOOPSDONE the handler is called by default, but the user can disable
  *  this (via flags). */
  if (nrf_pwm_event_check(p_pwm, NRF_PWM_EVENT_LOOPSDONE))
    {
      nrf_pwm_event_clear(p_pwm, NRF_PWM_EVENT_LOOPSDONE);
    }

  /* The STOPPED event is always propagated to the user handler. */
  if (nrf_pwm_event_check(p_pwm, NRF_PWM_EVENT_STOPPED))
    {
      nrf_pwm_event_clear(p_pwm, NRF_PWM_EVENT_STOPPED);
      priv->state = NRF_DRV_STATE_INITIALIZED;
    }
}

static int nrf52_pwm_isr(int irq, void *context, FAR void *arg)
{
  FAR struct nrf52_pwm_s *priv = (FAR struct nrf52_pwm_s *)arg;

  irq_handler((NRF_PWM_Type *)priv->base, priv);

  return OK;
}

/****************************************************************************
 * Name: nrf52_pwm_update_duty
 *
 * Description:
 *   Try to change only channel duty.
 *
 * Input parameters:
 *   priv    - A reference to the lower half PWM driver state structure
 *   channel - Channel to by updated
 *   duty    - New duty.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/


/****************************************************************************
 * Name: nrf52_pwm_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   It should not, however, output pulses until the start method is called.
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *   APB1 or 2 clocking for the GPIOs has already been configured by the RCC
 *   logic at power up.
 *
 ****************************************************************************/

static int nrf52_pwm_setup(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct nrf52_pwm_s *priv = (FAR struct nrf52_pwm_s *)dev;
  uint32_t output_pin[NRF52_MAX_PWM_NCHANNELS];
  uint32_t pincfg;
  int i;
  irqstate_t flags;


  if (MAX_CLK < priv->pwm_clk && priv->pwm_clk < MIN_CLK)
    {
      pwmerr("Wrong PWM Clock Config . pwm_clk: %d, max_clk: %d, min_clk: %d\n",
             priv->pwm_clk, MAX_CLK, MIN_CLK);
      return -EBUSY;
    }

  if (NRF_DRV_STATE_POWERED_ON == priv->state)
    {
      pwmerr("PWM module is running\n");
      return -EBUSY;
    }

  /* Configure the PWM output pins */
  memset(output_pin, 0xFF, sizeof(output_pin));

  for (i = 0; i < priv->channel_num; i++)
    {
      pincfg = priv->channels[i].pincfg;
      if (pincfg != NRF52_INVALID_GPIO_PIN)
        {
          bool inverted = pincfg &  NRF52_PWM_PIN_INVERTED;
          pincfg   = pincfg & ~NRF52_PWM_PIN_INVERTED;

          if (inverted)
            {
              nrf_gpio_pin_set(pincfg);
            }
          else
            {
              nrf_gpio_pin_clear(pincfg);
            }

          nrf_gpio_cfg_output(pincfg);
          output_pin[i] = pincfg;
        }
    }

  nrf_pwm_pins_set((FAR NRF_PWM_Type *)priv->base, output_pin);
  nrf_pwm_enable((FAR NRF_PWM_Type *)priv->base);

  /* enable hardware PWM controller */

  flags = enter_critical_section();
  irq_attach(priv->irqid, nrf52_pwm_isr, priv);
  leave_critical_section(flags);

  priv->state = NRF_DRV_STATE_INITIALIZED;

  return OK;
}

/****************************************************************************
 * Name: nrf52_pwm_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   stop pulsed output, free any resources, disable the timer hardware, and
 *   put the system into the lowest possible power usage state
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int nrf52_pwm_shutdown(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct nrf52_pwm_s *priv = (FAR struct nrf52_pwm_s *)dev;
  uint32_t output_pin[NRF_PWM_CHANNEL_COUNT];
  int i;
  irqstate_t flags;

  /* Make sure that the output has been stopped */

  nrf52_pwm_stop(dev);

  /* Disable PWM controller IRQ */

  flags = enter_critical_section();
  irq_attach(priv->irqid, NULL, NULL);
  leave_critical_section(flags);

  /* Then disconnect GPIO from PWM controller & hardware PWM controller */

  for (i = 0; i < NRF_PWM_CHANNEL_COUNT; i++)
    {
      output_pin[i] = NRF_PWM_PIN_NOT_CONNECTED;
    }
  nrf_pwm_disable((FAR NRF_PWM_Type *)priv->base);
  nrf_pwm_pins_set((FAR NRF_PWM_Type *)priv->base, output_pin);

  priv->state = NRF_DRV_STATE_UNINITIALIZED;

  return OK;
}

/****************************************************************************
 * Name: nrf52_pwm_start
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int nrf52_pwm_start(FAR struct pwm_lowerhalf_s *dev,
                           FAR const struct pwm_info_s *info)
{
  int ret = OK;
  FAR struct nrf52_pwm_s *priv = (FAR struct nrf52_pwm_s *)dev;

  if (NRF_DRV_STATE_INITIALIZED != priv->state)
    {
      pwmerr("PWM module not initialized \n");
      return -EBUSY;
    }

  ret = nrf_pwm_calculate_param(priv, (void *)info, 0);
  if (OK != ret)
    {
      return ret;
    }

  nrf_pwm_start(priv);
  priv->state = NRF_DRV_STATE_POWERED_ON;

  return ret;
}

/****************************************************************************
 * Name: nrf52_pwm_stop
 *
 * Description:
 *   Stop the pulsed output and reset the timer resources
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *   This function is called to stop the pulsed output at anytime.  This
 *   method is also called from the timer interrupt handler when a repetition
 *   count expires... automatically stopping the timer.
 *
 ****************************************************************************/

static int nrf52_pwm_stop(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct nrf52_pwm_s *priv = (FAR struct nrf52_pwm_s *)dev;

  if (NRF_DRV_STATE_POWERED_ON != priv->state)
    {
      return OK;
    }

  /* Disable interrupts momentary to stop any ongoing isr processing and
   * disable PWM controller.
   */

  pwminfo("Stop NRF52 PWM ...\n");
  nrf_pwm_task_trigger((FAR NRF_PWM_Type *)priv->base, NRF_PWM_TASK_STOP);

  do
    {
      /* If interrupts are disabled, we must check the STOPPED event here. */

      if (nrf_pwm_event_check((FAR NRF_PWM_Type *)priv->base,
                              NRF_PWM_EVENT_STOPPED))
        {
          pwminfo("NRF52 PWM Stoping Done.\n");
          break;
        }
    }
  while (1);

  up_disable_irq(priv->irqid);
  priv->state = NRF_DRV_STATE_INITIALIZED;

  return OK;
}

/****************************************************************************
 * Name: nrf52_pwm_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   cmd - The ioctl command
 *   arg - The argument accompanying the ioctl command
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int nrf52_pwm_ioctl(FAR struct pwm_lowerhalf_s *dev, int cmd,
                           unsigned long arg)
{
  FAR struct nrf52_pwm_s *priv = (FAR struct nrf52_pwm_s *)dev;

  pwmwarn("NRF52 PWM IOCTL cmd: %#x...\n", cmd);

  /* There are no platform-specific ioctl commands */
  if (cmd == PWMIOC_START_NRF52_INFO)
    {
      int ret;

      if (NRF_DRV_STATE_UNINITIALIZED == priv->state
          || NRF_DRV_STATE_POWERED_ON == priv->state)
        {
          pwmerr("PWM module not initialized or had been started state [%d]\n",
                 priv->state);
          return -EBUSY;
        }

      ret = nrf_pwm_calculate_param(priv, (void *)arg, 1);
      if (OK != ret)
        {
          return ret;
        }

      nrf_pwm_start(priv);
      priv->state = NRF_DRV_STATE_POWERED_ON;
      return OK;
    }

  if (cmd == PWMIOC_STOP_NRF52_INFO)
    {
      return nrf52_pwm_stop(dev);
    }

  return -EINVAL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_pwm_initialize
 *
 * Description:
 *   Initialize PWM controller for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, OK is returned.
 *   Other value is returned on any failure.
 *
 ****************************************************************************/

FAR int nrf52_pwm_initialize(uint32_t port,
                             struct pwm_pinmux_t *pinmux, uint32_t channels)
{
  int ret;
  struct nrf52_pwm_s  *pwm_dev = NULL;
  struct pwm_pinmux_t *pwm_pinmux;
  char devname[16];

  pwminfo("Starting to register NRF52 PWM\n");

#ifdef CONFIG_NRF52_PWM_M0
  pwm_dev = &g_pwm0_dev;
#endif

#ifdef CONFIG_NRF52_PWM_M1
  pwm_dev = &g_pwm1_dev;
#endif

#ifdef CONFIG_NRF52_PWM_M2
  pwm_dev = &g_pwm2_dev;
#endif

  if (NULL == pwm_dev || NULL == pinmux || channels > NRF52_MAX_PWM_NCHANNELS)
    {
      pwmerr("Invalid Param: port %d, chanel number %d\n", port, channels);
      return EINVAL;
    }
  pwm_pinmux = pwm_dev->channels;

  for (int i = 0; i < channels ; i++)
    {
      pwm_pinmux->pincfg = pinmux[i].pincfg;
      pwm_pinmux->channel = pinmux[i].channel;
    }

  pwm_dev->channel_num = channels;

  /* Register the PWM driver at "/dev/pwm" */

  snprintf(devname, 16, "%s%d", "/dev/pwm", port);
  ret = pwm_register(devname, (FAR struct pwm_lowerhalf_s *)pwm_dev);
  if (ret < 0)
    {
      pwmerr("ERROR: pwm_register pwm %d failed: %d\n", port, ret);
      return ret;
    }

  return ret;
}

#endif /* CONFIG_NRF52_PWM_M0 | CONFIG_NRF52_PWM_M1 | CONFIG_NRF52_PWM_M2 */
