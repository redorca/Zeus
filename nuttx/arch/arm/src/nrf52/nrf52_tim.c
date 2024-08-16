/************************************************************************************
 * arm/arm/src/nrf52/nrf52_tim.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
 *
 * With modifications and updates by:
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           <dev@ziggurat29.com>
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
#include <nuttx/arch.h>
#include <nuttx/irq.h>


#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_internal.h"
#include "up_arch.h"

#include "nrf.h"
#include "nrf52_gpio.h"
#include "nrf52_tim.h"
#include "chip/nrf52_tim.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/************************************************************************************
 * Private Data
 ************************************************************************************/

#if defined(CONFIG_NRF52_TIM0)  || defined(CONFIG_NRF52_TIM1)  || \
    defined(CONFIG_NRF52_TIM2)  || defined(CONFIG_NRF52_TIM3)  || \
    defined(CONFIG_NRF52_TIM4)

static const struct nrf52_tim_ops_s nrf52_tim_ops =
{
  .enable     = &nrf52_timer_enable,
  .pause      = &nrf52_timer_pause,
  .resume     = &nrf52_timer_resume,
  .disable    = &nrf52_timer_disable,
  .clear      = &nrf52_timer_clear,
  .setisr     = &nrf52_tim_setisr,
  .enableint  = &nrf52_timer_compare_int_enable,
  .disableint = &nrf52_timer_compare_int_disable,
  .checkint   = &nrf52_timer_irq_checkint,
  .clearint   = &nrf52_timer_irq_clearint,
  /* .increment = nrf52_timer_increment,
    .decrement = nrf52_timer_decrement,
    .capture   = nrf52_timer_capture,
    .compare   = nrf52_timer_compare,
  */
};


/* TIM Device Structure */
struct nrf52_tim_priv_s
{
  const struct nrf52_tim_ops_s *ops;
  nrf_timer_mode_t             mode;
  uint32_t                     base;           /* TIMn base address */
  nrfx_drv_state_t              state;          /* Current timer state */
  nrf_timer_bit_width_t        bit_width;      /* Current bit width   */
  nrf_timer_frequency_t        frequency;
  nrf_timer_cc_channel_t       channel_number; /* Current channel number */
  nrf_timer_cc_channel_t       capture_channel_number; /* Current channel number */
  uint8_t                      channel_count;
};

#ifdef CONFIG_NRF52_TIM0
struct nrf52_tim_priv_s nrf52_tim0_priv =
{
  .ops            = &nrf52_tim_ops,
  .mode           = NRF_TIMER_MODE_TIMER,
  .base           = NRF_TIMER0_BASE,
  .state          = NRF_DRV_STATE_UNINITIALIZED,
  .channel_count  = TIMER0_CC_NUM,
  .frequency      = NRF_TIMER_FREQ_1MHz, /* resolution is 1us*/
  .bit_width      = NRF_TIMER_BIT_WIDTH_32,
  .channel_number = NRF_TIMER_CC_CHANNEL0,
  .capture_channel_number = NRF_TIMER_CC_CHANNEL1,
};
#endif
#ifdef CONFIG_NRF52_TIM1
struct nrf52_tim_priv_s nrf52_tim1_priv =
{
  .ops            = &nrf52_tim_ops,
  .mode           = NRF_TIMER_MODE_TIMER,
  .base           = NRF_TIMER1_BASE,
  .state          = NRF_DRV_STATE_UNINITIALIZED,
  .channel_count  = TIMER1_CC_NUM,
  .frequency      = NRF_TIMER_FREQ_1MHz,
  .bit_width      = NRF_TIMER_BIT_WIDTH_32,
  .channel_number = NRF_TIMER_CC_CHANNEL0,
  .capture_channel_number = NRF_TIMER_CC_CHANNEL1,
};
#endif

#ifdef CONFIG_NRF52_TIM2
struct nrf52_tim_priv_s nrf52_tim2_priv =
{
  .ops            = &nrf52_tim_ops,
  .mode           = NRF_TIMER_MODE_TIMER,
  .base           = NRF_TIMER2_BASE,
  .state          = NRF_DRV_STATE_UNINITIALIZED,
  .channel_count  = TIMER2_CC_NUM,
  .frequency      = NRF_TIMER_FREQ_1MHz,
  .bit_width      = NRF_TIMER_BIT_WIDTH_32,
  .channel_number = NRF_TIMER_CC_CHANNEL0,
  .capture_channel_number = NRF_TIMER_CC_CHANNEL1,
};
#endif

#ifdef CONFIG_NRF52_TIM3
struct nrf52_tim_priv_s nrf52_tim3_priv =
{
  .ops            = &nrf52_tim_ops,
  .mode           = NRF_TIMER_MODE_TIMER,
  .base           = NRF_TIMER3_BASE,
  .state          = NRF_DRV_STATE_UNINITIALIZED,
  .channel_count  = TIMER3_CC_NUM,
  .frequency      = NRF_TIMER_FREQ_1MHz,
  .bit_width      = NRF_TIMER_BIT_WIDTH_32,
  .channel_number = NRF_TIMER_CC_CHANNEL0,
  .capture_channel_number = NRF_TIMER_CC_CHANNEL1,
};
#endif

#ifdef CONFIG_NRF52_TIM4
struct nrf52_tim_priv_s nrf52_tim4_priv =
{
  .ops            = &nrf52_tim_ops,
  .mode           = NRF_TIMER_MODE_TIMER,
  .base           = NRF_TIMER4_BASE,
  .state          = NRF_DRV_STATE_UNINITIALIZED,
  .channel_count  = TIMER4_CC_NUM,
  .frequency      = NRF_TIMER_FREQ_1MHz,
  .bit_width      = NRF_TIMER_BIT_WIDTH_32,
  .channel_number = NRF_TIMER_CC_CHANNEL0,
  .capture_channel_number = NRF_TIMER_CC_CHANNEL1,
};
#endif
/************************************************************************************
 * Name: nrf52_timer_init
 *
 * Description:
 *
 ************************************************************************************/
FAR struct nrf52_tim_dev_s *nrf52_timer_init(int timer)
{

  struct nrf52_tim_dev_s *dev = NULL;
  struct nrf52_tim_priv_s *priv =  NULL;

  /* Get structure and enable power */

  switch (timer)
    {
#ifdef CONFIG_NRF52_TIM0
      case 0:
        dev = (struct nrf52_tim_dev_s *)&nrf52_tim0_priv;
        break;
#endif
#ifdef CONFIG_NRF52_TIM1
      case 1:
        dev = (struct nrf52_tim_dev_s *)&nrf52_tim1_priv;
        break;
#endif
#ifdef CONFIG_NRF52_TIM2
      case 2:
        dev = (struct nrf52_tim_dev_s *)&nrf52_tim2_priv;
        break;
#endif
#ifdef CONFIG_NRF52_TIM3
      case 3:
        dev = (struct nrf52_tim_dev_s *)&nrf52_tim3_priv;
        break;
#endif
#ifdef CONFIG_NRF52_TIM4
      case 4:
        dev = (struct nrf52_tim_dev_s *)&nrf52_tim4_priv;
        break;
#endif
      default:
        return NULL;
    }

  priv = (struct nrf52_tim_priv_s *)dev;
  /* Is device already allocated */
  if (priv->state != NRF_DRV_STATE_UNINITIALIZED)
    {
      return NULL;
    }


  /* Warning 685: Relational operator '<=' always evaluates to 'true'"
   * Warning in NRF_TIMER_IS_BIT_WIDTH_VALID macro. Macro validate timers resolution.
   * Not necessary in nRF52 based systems. Obligatory in nRF51 based systems.
   */

  ASSERT(NRF_TIMER_IS_BIT_WIDTH_VALID((NRF_TIMER_Type *)priv->base,
                                      priv->bit_width));

  for (uint8_t i = 0; i < priv->channel_count; ++i)
    {
      nrf_timer_event_clear((NRF_TIMER_Type *)priv->base,
                            nrf_timer_compare_event_get(i));
    }

  nrf_timer_mode_set((NRF_TIMER_Type *)priv->base, priv->mode);
  nrf_timer_bit_width_set((NRF_TIMER_Type *)priv->base, priv->bit_width);
  nrf_timer_frequency_set((NRF_TIMER_Type *)priv->base, priv->frequency);

  priv->state = NRF_DRV_STATE_INITIALIZED;

  return dev;
}



/************************************************************************************
 * Name: nrf52_timer_uninit
 *
 * Description:
 *
 ************************************************************************************/
void nrf52_timer_uninit(struct nrf52_tim_dev_s *dev)
{
  struct nrf52_tim_priv_s *priv = (struct nrf52_tim_priv_s *)dev;

  //up_disable_irq(nrf_drv_get_IRQn(p_instance->p_reg));

#define DISABLE_ALL UINT32_MAX
  nrf_timer_shorts_disable((NRF_TIMER_Type *)priv->base, DISABLE_ALL);
  nrf_timer_int_disable((NRF_TIMER_Type *)priv->base, DISABLE_ALL);
#undef DISABLE_ALL

  if (priv->state == NRF_DRV_STATE_POWERED_ON)
    {
      nrf52_timer_disable(dev);
    }

  priv->state = NRF_DRV_STATE_UNINITIALIZED;
}

/************************************************************************************
 * Name: nrf52_timer_enable
 *
 * Description:
 *
 ************************************************************************************/
void nrf52_timer_enable(FAR struct nrf52_tim_dev_s *dev)
{
  ASSERT(((struct nrf52_tim_priv_s *)dev)->state == NRF_DRV_STATE_INITIALIZED);
  nrf_timer_task_trigger((NRF_TIMER_Type *)((struct nrf52_tim_priv_s *)dev)->base, NRF_TIMER_TASK_START);
  ((struct nrf52_tim_priv_s *)dev)->state  = NRF_DRV_STATE_POWERED_ON;
}

/************************************************************************************
 * Name: nrf52_timer_disable
 *
 * Description:
 *
 ************************************************************************************/
void nrf52_timer_disable(FAR struct nrf52_tim_dev_s *dev)
{
  ASSERT(((struct nrf52_tim_priv_s *)dev)->state == NRF_DRV_STATE_POWERED_ON);
  nrf_timer_task_trigger((NRF_TIMER_Type *)((struct nrf52_tim_priv_s *)dev)->base, NRF_TIMER_TASK_SHUTDOWN);
  ((struct nrf52_tim_priv_s *)dev)->state = NRF_DRV_STATE_INITIALIZED;
}

/************************************************************************************
 * Name: nrf52_timer_resume
 *
 * Description:
 *
 ************************************************************************************/
void nrf52_timer_resume(struct nrf52_tim_dev_s *dev)
{
  ASSERT(((struct nrf52_tim_priv_s *)dev)->state == NRF_DRV_STATE_POWERED_ON);
  nrf_timer_task_trigger((NRF_TIMER_Type *)((struct nrf52_tim_priv_s *)dev)->base, NRF_TIMER_TASK_START);
}

/************************************************************************************
 * Name: nrf52_timer_pause
 *
 * Description:
 *
 ************************************************************************************/
void nrf52_timer_pause(struct nrf52_tim_dev_s *dev)
{
  ASSERT(((struct nrf52_tim_priv_s *)dev)->state == NRF_DRV_STATE_POWERED_ON);
  nrf_timer_task_trigger((NRF_TIMER_Type *)((struct nrf52_tim_priv_s *)dev)->base, NRF_TIMER_TASK_STOP);
}

/************************************************************************************
 * Name: nrf52_timer_clear
 *
 * Description:
 *
 ************************************************************************************/
void nrf52_timer_clear(struct nrf52_tim_dev_s *dev)
{
  ASSERT(((struct nrf52_tim_priv_s *)dev)->state  != NRF_DRV_STATE_UNINITIALIZED);
  nrf_timer_task_trigger((NRF_TIMER_Type *)((struct nrf52_tim_priv_s *)dev)->base, NRF_TIMER_TASK_CLEAR);
}

/************************************************************************************
 * Name: nrf52_timer_increment
 *
 * Description:
 *
 ************************************************************************************/
void nrf52_timer_increment(struct nrf52_tim_dev_s *dev)
{
  ASSERT(((struct nrf52_tim_priv_s *)dev)->state == NRF_DRV_STATE_POWERED_ON);
  ASSERT(nrf_timer_mode_get((NRF_TIMER_Type *)((struct nrf52_tim_priv_s *)dev)->base) != NRF_TIMER_MODE_TIMER);

  nrf_timer_task_trigger((NRF_TIMER_Type *)((struct nrf52_tim_priv_s *)dev)->base, NRF_TIMER_TASK_COUNT);
}

/************************************************************************************
 * Name: nrf52_timer_capture
 *
 * Description:
 *
 ************************************************************************************/
uint32_t nrf52_timer_capture(struct nrf52_tim_dev_s *dev)
{
  struct nrf52_tim_priv_s *priv = (struct nrf52_tim_priv_s *)dev;

  ASSERT(priv->capture_channel_number < priv->channel_count);

  nrf_timer_task_trigger((NRF_TIMER_Type *)priv->base,
                         nrf_timer_capture_task_get(priv->capture_channel_number));
  return nrf_timer_cc_read((NRF_TIMER_Type *)priv->base, priv->capture_channel_number);
}

/************************************************************************************
 * Name: nrf52_timer_compare
 *
 * Description:
 *
 ************************************************************************************/
void nrf52_timer_compare(struct nrf52_tim_dev_s *dev,
                         nrf_timer_cc_channel_t cc_channel,
                         uint32_t               cc_value,
                         bool                   enable_int)
{
  struct nrf52_tim_priv_s *priv = (struct nrf52_tim_priv_s *)dev;
  nrf_timer_int_mask_t timer_int = nrf_timer_compare_int_get(cc_channel);

  nrf_timer_cc_write((NRF_TIMER_Type *)priv->base, cc_channel, cc_value);

  if (enable_int)
    {
      nrf_timer_int_enable((NRF_TIMER_Type *)priv->base, timer_int);
    }
  else
    {
      nrf_timer_int_disable((NRF_TIMER_Type *)priv->base, timer_int);
    }

}

/************************************************************************************
 * Name: nrf52_timer_extended_compare
 *
 * Description:
 *
 ************************************************************************************/
void nrf52_timer_extended_compare(struct nrf52_tim_dev_s *dev,
                                  nrf_timer_cc_channel_t cc_channel,
                                  uint32_t               cc_value,
                                  nrf_timer_short_mask_t timer_short_mask,
                                  bool                   enable_int)
{
  struct nrf52_tim_priv_s *priv = (struct nrf52_tim_priv_s *)dev;

  ASSERT(cc_channel < priv->channel_count);
  nrf_timer_shorts_disable((NRF_TIMER_Type *)priv->base,
                           (TIMER_SHORTS_COMPARE0_STOP_Msk  << cc_channel) |
                           (TIMER_SHORTS_COMPARE0_CLEAR_Msk << cc_channel));

  nrf_timer_shorts_enable((NRF_TIMER_Type *)priv->base, timer_short_mask);

  (void)nrf52_timer_compare(dev, cc_channel, cc_value, enable_int);
}

/************************************************************************************
 * Name: nrf52_timer_compare_int_enable
 *
 * Description:
 *
 ************************************************************************************/
void nrf52_timer_compare_int_enable(struct nrf52_tim_dev_s *dev, uint32_t channel)
{
  ASSERT(((struct nrf52_tim_priv_s *)dev)->state != NRF_DRV_STATE_UNINITIALIZED);
  ASSERT(channel < ((struct nrf52_tim_priv_s *)dev)->channel_count);


  nrf_timer_event_clear((NRF_TIMER_Type *)((struct nrf52_tim_priv_s *)dev)->base,
                        nrf_timer_compare_event_get(channel));
  nrf_timer_int_enable((NRF_TIMER_Type *)((struct nrf52_tim_priv_s *)dev)->base,
                       nrf_timer_compare_int_get(channel));
}

/************************************************************************************
 * Name: nrf52_timer_compare_int_disable
 *
 * Description:
 *
 ************************************************************************************/
void nrf52_timer_compare_int_disable(FAR struct nrf52_tim_dev_s *dev, uint32_t channel)
{
  ASSERT(((struct nrf52_tim_priv_s *)dev)->state != NRF_DRV_STATE_UNINITIALIZED);
  ASSERT(channel < ((struct nrf52_tim_priv_s *)dev)->channel_count);

  nrf_timer_int_disable((NRF_TIMER_Type *)((struct nrf52_tim_priv_s *)dev)->base,
                        nrf_timer_compare_int_get(channel));
}

/************************************************************************************
 * Name: nrf52_timer_us_to_ticks
 *
 * Description:
 *
 ************************************************************************************/
uint32_t nrf52_timer_us_to_ticks(FAR struct nrf52_tim_dev_s *dev,
                                 uint32_t timer_us)
{
  return nrf_timer_us_to_ticks(timer_us,
                               nrf_timer_frequency_get((NRF_TIMER_Type *)((struct nrf52_tim_priv_s *)dev)->base));
}

/************************************************************************************
 * Name: nrf52_timer_ms_to_ticks
 *
 * Description:
 *
 ************************************************************************************/
uint32_t nrf52_timer_ms_to_ticks(
  FAR struct nrf52_tim_dev_s *dev,
  uint32_t timer_ms)
{
  return nrf_timer_ms_to_ticks(timer_ms,
                               nrf_timer_frequency_get((NRF_TIMER_Type *)((struct nrf52_tim_priv_s *)dev)->base));
}

/************************************************************************************
 * Name: nrf52_tim_setisr
 *
 * Description:
 *
 ************************************************************************************/

int nrf52_tim_setisr(FAR struct nrf52_tim_dev_s *dev,
                     xcpt_t handler, void *arg, int source)
{
  int vectorno;

  DEBUGASSERT(dev != NULL);
  //DEBUGASSERT(source == 0);

  switch (((struct nrf52_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_NRF52_TIM0
      case NRF_TIMER0_BASE:
        vectorno = TIMER0_IRQn;
        break;
#endif
#ifdef CONFIG_NRF52_TIM1
      case NRF_TIMER1_BASE:
        vectorno = TIMER1_IRQn;
        break;
#endif
#ifdef CONFIG_NRF52_TIM2
      case NRF_TIMER2_BASE:
        vectorno = TIMER2_IRQn;
        break;
#endif
#ifdef CONFIG_NRF52_TIM3
      case NRF_TIMER3_BASE:
        vectorno = TIMER3_IRQn;
        break;
#endif
#ifdef CONFIG_NRF52_TIM4
      case NRF_TIMER4_BASE:
        vectorno = TIMER4_IRQn;
        break;
#endif
      default:
        return -EINVAL;
    }

  /* Disable interrupt when callback is removed */

  if (!handler)
    {
      up_disable_irq(vectorno);
      irq_detach(vectorno);
      return OK;
    }

  /* Otherwise set callback and enable interrupt */

  irq_attach(vectorno, handler, arg);
  up_enable_irq(vectorno);

#ifdef CONFIG_ARCH_IRQPRIO
  /* Set the interrupt priority */

  up_prioritize_irq(vectorno, NVIC_SYSH_PRIORITY_DEFAULT);
#endif

  return OK;
}

/************************************************************************************
 * Name: nrf52_timer_irq_checkint
 *
 * Description:
 *
 ************************************************************************************/
bool nrf52_timer_irq_checkint(FAR struct nrf52_tim_dev_s *dev, uint32_t channel)
{
  struct nrf52_tim_priv_s *priv = (struct nrf52_tim_priv_s *)dev;

  nrf_timer_event_t event = nrf_timer_compare_event_get(channel);
  nrf_timer_int_mask_t int_mask = nrf_timer_compare_int_get(channel);

  if (nrf_timer_event_check((NRF_TIMER_Type *)priv->base, event) &&
      nrf_timer_int_enable_check((NRF_TIMER_Type *)priv->base, int_mask))
    {
      return true;
    }
  return false;
}

/************************************************************************************
 * Name: nrf52_timer_irq_clearint
 *
 * Description:
 *
 ************************************************************************************/
void nrf52_timer_irq_clearint(FAR struct nrf52_tim_dev_s *dev, uint32_t channel)
{
  nrf_timer_event_t event = nrf_timer_compare_event_get(channel);
  nrf_timer_event_clear((NRF_TIMER_Type *)((struct nrf52_tim_priv_s *)dev)->base, event);
}


/************************************************************************************
 * Name: nrf52_timer_irq_clear
 *
 * Description:
 *
 ************************************************************************************/
void nrf52_timer_irq_clear(FAR struct nrf52_tim_dev_s *dev)
{
  struct nrf52_tim_priv_s *priv = (struct nrf52_tim_priv_s *)dev;

  for (int i = 0; i < priv->channel_count ; i++)
    {
      nrf_timer_event_t event = nrf_timer_compare_event_get(i);
      nrf_timer_int_mask_t int_mask = nrf_timer_compare_int_get(i);

      if (nrf_timer_event_check((NRF_TIMER_Type *)priv->base, event) &&
          nrf_timer_int_enable_check((NRF_TIMER_Type *)priv->base, int_mask))
        {
          nrf_timer_event_clear((NRF_TIMER_Type *)priv->base, event);
        }
    }
}

uint32_t nrf52_timer_task_address_get(
  FAR struct nrf52_tim_dev_s *dev,
  nrf_timer_task_t timer_task)
{
  return (uint32_t)nrf_timer_task_address_get((NRF_TIMER_Type *)((struct nrf52_tim_priv_s *)dev)->base, timer_task);
}

uint32_t nrf52_timer_capture_task_address_get(
  FAR struct nrf52_tim_dev_s *dev,
  uint32_t channel)
{
  ASSERT(channel < ((struct nrf52_tim_priv_s *)dev)->channel_count);
  return (uint32_t)nrf_timer_task_address_get((NRF_TIMER_Type *)((struct nrf52_tim_priv_s *)dev)->base,
                                              nrf_timer_capture_task_get(channel));
}

uint32_t nrf52_timer_event_address_get(
  FAR struct nrf52_tim_dev_s *dev,
  nrf_timer_event_t timer_event)
{
  return (uint32_t)nrf_timer_event_address_get((NRF_TIMER_Type *)((struct nrf52_tim_priv_s *)dev)->base, timer_event);
}

uint32_t nrf52_timer_compare_event_address_get(
  FAR struct nrf52_tim_dev_s *dev,
  uint32_t channel)
{
  ASSERT(channel < ((struct nrf52_tim_priv_s *)dev)->channel_count);
  return (uint32_t)nrf_timer_event_address_get((NRF_TIMER_Type *)((struct nrf52_tim_priv_s *)dev)->base,
                                               nrf_timer_compare_event_get(channel));
}

uint32_t nrf52_timer_capture_get(
  FAR struct nrf52_tim_dev_s *dev,
  nrf_timer_cc_channel_t cc_channel)
{
  return nrf_timer_cc_read((NRF_TIMER_Type *)((struct nrf52_tim_priv_s *)dev)->base, cc_channel);
}

#endif

