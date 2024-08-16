/****************************************************************************
 * arch/arm/src/nrf52/nrf52_i2s.c
 *
 *   Copyright (C) 2018 zGlue Inc. All rights reserved.
 *   Author: Zhengwei Wang <zhengwei@zglue.com>
 *
 * References:
 *
 * -  The framework for this driver is based on stm32l4 sai driver
 *    written by Gregory Nutt.
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
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <assert.h>
#include <queue.h>
#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/audio/audio.h>

#include <arch/board/board.h>


#include "chip.h"
#include "nrf.h"
#include "nrf_i2s.h"
#include "nrf_gpio.h"
#include "nrf52_gpio.h"
#include "nrf52_i2s.h"

#ifdef CONFIG_NRF52_I2S

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SCHED_WORKQUEUE
#  error Work queue support is required (CONFIG_SCHED_WORKQUEUE)
#endif

#ifndef CONFIG_AUDIO
#  error CONFIG_AUDIO required by this driver
#endif

#ifndef CONFIG_I2S
#  error CONFIG_I2S required by this driver
#endif

#ifndef CONFIG_NRF52_I2S_DEFAULT_SAMPLERATE
#  define CONFIG_NRF52_I2S_DEFAULT_SAMPLERATE  (48000)
#endif

#ifndef CONFIG_NRF52_I2S_DEFAULT_DATALEN
#  define CONFIG_NRF52_I2S_DEFAULT_DATALEN     (16)
#endif

#ifndef CONFIG_NRF52_I2S_DEFAULT_CHANNEL
#  define CONFIG_NRF52_I2S_DEFAULT_CHANNEL     (2)
#endif


#ifndef CONFIG_NRF52_I2S_MAXINFLIGHT
#  define CONFIG_NRF52_I2S_MAXINFLIGHT         (16)
#endif

#define NRF52_I2S_DATA_ALIGN 3

#define EVT_TO_STR(event)   (event == NRF_I2S_EVENT_RXPTRUPD ? "NRF_I2S_EVENT_RXPTRUPD" :                \
                            (event == NRF_I2S_EVENT_TXPTRUPD ? "NRF_I2S_EVENT_TXPTRUPD" :                \
                            (event == NRF_I2S_EVENT_STOPPED ? "NRF_I2S_EVENT_STOPPED" : "UNKNOWN EVENT")))


/****************************************************************************
 * Private Types
 ****************************************************************************/

/* I2S buffer container */

struct i2s_buffer_s
{
  struct i2s_buffer_s *flink;  /* Supports a singly linked list */
  i2s_callback_t callback;     /* Function to call when the transfer completes */
  uint32_t timeout;            /* The timeout value to use with transfers */
  void *arg;                   /* The argument to be returned with the callback */
  struct ap_buffer_s *apb;     /* The audio buffer */
  int result;                  /* The result of the transfer */
};


/* The state of the one I2S peripheral */

struct nrf52_i2s_s
{
  struct i2s_dev_s dev;        /* Externally visible I2S interface */
  sem_t exclsem;               /* Assures mutually exclusive acess to i2s */
  sq_queue_t pend;             /* A queue of pending transfers */
  sq_queue_t done;             /* A queue of completed transfers */
  struct work_s work;          /* this work will clear all apb buffers in the done queue */

  /* I2S setting */

  uint8_t datalen;             /* Data width */
  uint8_t channel;             /* mono or stereo */
  uint8_t channel_setted;      /* new setted channel number */
  uint32_t samplerate;         /* Data sample rate */
  const struct nrf52_i2s_clk_s *clk_config_current;/*pointer to the current clock config*/
  const struct nrf52_i2s_clk_s *clk_config_setted;/*pointer to the new setted clock config*/
  bool rxenab;            /* True: RX transfers enabled */
  bool txenab;            /* True: TX transfers enabled */


  /* Pre-allocated pool of buffer containers */

  sem_t bufsem;                   /* Buffer wait semaphore */
  struct i2s_buffer_s *freelist;  /* A list a free buffer containers */
  struct i2s_buffer_s containers[CONFIG_NRF52_I2S_MAXINFLIGHT];
  struct i2s_buffer_s *ended;/*point to the apb_buffer that has been successfully sended or received just now*/
  struct i2s_buffer_s *transmitting;/*point to the apb_buffer that is now being transmitting by the i2s*/
  struct i2s_buffer_s *loaded;/*point to the apb_buffer that has been just handded over to i2s peripheral*/
};

struct nrf52_i2s_clk_s
{
  uint32_t samplerate;
  uint8_t  samplewidth;
  nrf_i2s_swidth_t swidth;
  nrf_i2s_mck_t mck;
  nrf_i2s_ratio_t ratio;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Semaphore helpers */

static void     nrf52_i2s_exclsem_take(struct nrf52_i2s_s *priv);
#define         nrf52_i2s_exclsem_give(priv) sem_post(&priv->exclsem)

static void     nrf52_i2s_bufsem_take(struct nrf52_i2s_s *priv);
#define         nrf52_i2s_bufsem_give(priv) sem_post(&priv->bufsem)

/* Buffer container helpers */

static struct i2s_buffer_s *
nrf52_i2s_buf_allocate(struct nrf52_i2s_s *priv);
static void     nrf52_i2s_buf_free(struct nrf52_i2s_s *priv,
                                   struct i2s_buffer_s *bfcontainer);
static void     nrf52_i2s_buf_initialize(struct nrf52_i2s_s *priv);

/*interrupt handler*/

static void     nrf52_i2s_schedule(struct nrf52_i2s_s *priv, int result);
static int      nrf52_i2s_interrupt(int irq, FAR void *context, FAR void *arg);

/*i2s config handlers*/

static int nrf52_i2s_setbitrate(struct nrf52_i2s_s *priv,
                                uint32_t samplerate, uint8_t samplewidth);



/* I2S methods */

static uint32_t nrf52_i2s_samplerate(struct i2s_dev_s *dev, uint32_t rate);
static uint32_t nrf52_i2s_datawidth(struct i2s_dev_s *dev, int bits);
static uint32_t nrf52_i2s_channel(struct i2s_dev_s *dev, int channel);
static int      nrf52_i2s_receive(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                                  i2s_callback_t callback, void *arg, uint32_t timeout);
static int      nrf52_i2s_send(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                               i2s_callback_t callback, void *arg,
                               uint32_t timeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2S device operations */

static const struct i2s_ops_s g_i2sops =
{
  /* Receiver methods */

  .i2s_rxsamplerate = nrf52_i2s_samplerate,
  .i2s_rxdatawidth  = nrf52_i2s_datawidth,
  .i2s_rxchannel    = nrf52_i2s_channel,
  .i2s_receive      = nrf52_i2s_receive,

  /* Transmitter methods */

  .i2s_txsamplerate = nrf52_i2s_samplerate,
  .i2s_txdatawidth  = nrf52_i2s_datawidth,
  .i2s_txchannel    = nrf52_i2s_channel,
  .i2s_send         = nrf52_i2s_send,
};

/* i2s status */
static struct nrf52_i2s_s g_nrf52_i2s_priv =
{
  .dev.ops     = &g_i2sops,
  .datalen     = CONFIG_NRF52_I2S_DEFAULT_DATALEN,
  .samplerate  = CONFIG_NRF52_I2S_DEFAULT_SAMPLERATE,
  .channel     = CONFIG_NRF52_I2S_DEFAULT_CHANNEL,
  .channel_setted = CONFIG_NRF52_I2S_DEFAULT_CHANNEL,
  .clk_config_current  = NULL,
  .clk_config_current  = NULL,
  .txenab = false,
  .rxenab = false,
  .loaded = NULL,
  .transmitting = NULL,
  .ended = NULL,

};

static const struct nrf52_i2s_clk_s nrf52_i2s_clk_table[] =
{
  {8000,  8,  NRF_I2S_SWIDTH_8BIT, NRF_I2S_MCK_32MDIV125, NRF_I2S_RATIO_32X},
  {11025, 8,  NRF_I2S_SWIDTH_8BIT, NRF_I2S_MCK_32MDIV15,  NRF_I2S_RATIO_192X},
  {16000, 8,  NRF_I2S_SWIDTH_8BIT, NRF_I2S_MCK_32MDIV31,  NRF_I2S_RATIO_64X},
  {22050, 8,  NRF_I2S_SWIDTH_8BIT, NRF_I2S_MCK_32MDIV30,  NRF_I2S_RATIO_48X},
  {32000, 8,  NRF_I2S_SWIDTH_8BIT, NRF_I2S_MCK_32MDIV32,  NRF_I2S_RATIO_32X},
  {44100, 8,  NRF_I2S_SWIDTH_8BIT, NRF_I2S_MCK_32MDIV15,  NRF_I2S_RATIO_48X},
  {48000, 8,  NRF_I2S_SWIDTH_8BIT, NRF_I2S_MCK_32MDIV21,  NRF_I2S_RATIO_32X},
  {96000, 8,  NRF_I2S_SWIDTH_8BIT, NRF_I2S_MCK_32MDIV10,  NRF_I2S_RATIO_32X},
  {8000,  16, NRF_I2S_SWIDTH_16BIT, NRF_I2S_MCK_32MDIV125, NRF_I2S_RATIO_32X},
  {11025, 16, NRF_I2S_SWIDTH_16BIT, NRF_I2S_MCK_32MDIV15,  NRF_I2S_RATIO_192X},
  {16000, 16, NRF_I2S_SWIDTH_16BIT, NRF_I2S_MCK_32MDIV31,  NRF_I2S_RATIO_64X},
  {22050, 16, NRF_I2S_SWIDTH_16BIT, NRF_I2S_MCK_32MDIV15,  NRF_I2S_RATIO_96X},
  {32000, 16, NRF_I2S_SWIDTH_16BIT, NRF_I2S_MCK_32MDIV32,  NRF_I2S_RATIO_32X},
  {44100, 16, NRF_I2S_SWIDTH_16BIT, NRF_I2S_MCK_32MDIV23,  NRF_I2S_RATIO_32X},
  {48000, 16, NRF_I2S_SWIDTH_16BIT, NRF_I2S_MCK_32MDIV21,  NRF_I2S_RATIO_32X},
  {96000, 16, NRF_I2S_SWIDTH_16BIT, NRF_I2S_MCK_32MDIV10,  NRF_I2S_RATIO_32X},
  {8000,  24, NRF_I2S_SWIDTH_24BIT, NRF_I2S_MCK_32MDIV42,  NRF_I2S_RATIO_96X},
  {11025, 24, NRF_I2S_SWIDTH_24BIT, NRF_I2S_MCK_32MDIV31,  NRF_I2S_RATIO_96X},
  {16000, 24, NRF_I2S_SWIDTH_24BIT, NRF_I2S_MCK_32MDIV42,  NRF_I2S_RATIO_48X},
  {22050, 24, NRF_I2S_SWIDTH_24BIT, NRF_I2S_MCK_32MDIV30,  NRF_I2S_RATIO_48X},
  {32000, 24, NRF_I2S_SWIDTH_24BIT, NRF_I2S_MCK_32MDIV21,  NRF_I2S_RATIO_48X},
  {44100, 24, NRF_I2S_SWIDTH_24BIT, NRF_I2S_MCK_32MDIV15,  NRF_I2S_RATIO_48X},
};

#define I2S_NULL_BUFFER_LENGTH 2
static uint32_t i2s_null_buffer[I2S_NULL_BUFFER_LENGTH] = {0, 0};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_i2s_getbitrate
 *
 * Description:
 *   Get the currently configured bitrate
 *
 * Input Parameters:
 *   priv   - private I2S device structure
 *
 * Returned Value:
 *   The current bitrate
 *
 ****************************************************************************/

static inline uint32_t nrf52_i2s_getbitrate(struct nrf52_i2s_s *priv)
{
  /* Calculate the bitrate in Hz */

  return 2 * priv->samplerate * priv->datalen;
}

/****************************************************************************
 * Name: nrf52_i2s_exclsem_take
 *
 * Description:
 *   Take the exclusive access semaphore handling any exceptional conditions
 *
 * Input Parameters:
 *   priv - A reference to the I2S peripheral state
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void nrf52_i2s_exclsem_take(struct nrf52_i2s_s *priv)
{
  int ret;

  /* Wait until we successfully get the semaphore.  EINTR is the only
   * expected 'failure' (meaning that the wait for the semaphore was
   * interrupted by a signal).
   */

  do
    {
      ret = sem_wait(&priv->exclsem);
      DEBUGASSERT(ret == 0 || errno == EINTR);
    }
  while (ret < 0);
}


/****************************************************************************
 * Name: nrf52_i2s_reload
 *
 * Description:
 *   updata the TX/RX source/destination address of I2S peripheral, if the new
 *   buffer is NULL, we just stop the transreceive.
 *
 * Input Parameters:
 *   priv - nrf52 i2s device instance
 *   container -- new buffer that will be loaded to I2S peripheral
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 *
 ****************************************************************************/

static int nrf52_i2s_reload(struct nrf52_i2s_s *priv, struct i2s_buffer_s *container)
{
  struct ap_buffer_s *apb;
  uintptr_t samp;
  apb_samp_t nbytes;


  if (container)
    {
      apb = container->apb;

      /* Get the transfer information, accounting for any data offset */

      samp = (uintptr_t)&apb->samp[apb->curbyte];

      /* config tr/rx buffer */

      if (priv->txenab)
        {
          nbytes = apb->nbytes - apb->curbyte;
          DEBUGASSERT((samp & NRF52_I2S_DATA_ALIGN) == 0 && (nbytes & NRF52_I2S_DATA_ALIGN) == 0);

          /*reload TX buffer*/

          nrf_i2s_tx_buffer_set(NRF_I2S, (uint32_t *)samp);
          nrf_i2s_txrx_length_set(NRF_I2S, nbytes / 4);

        }
      else if (priv->rxenab)
        {
          nbytes = apb->nmaxbytes - apb->curbyte;
          DEBUGASSERT((samp & NRF52_I2S_DATA_ALIGN) == 0 && (nbytes & NRF52_I2S_DATA_ALIGN) == 0);

          /*reload RX buffer*/

          nrf_i2s_rx_buffer_set(NRF_I2S, (uint32_t *)samp);
          nrf_i2s_txrx_length_set(NRF_I2S, nbytes / 4);
        }

    }
  else
    {
      /* we need to load this buffer when we have nothing to send/receive, so that the currently
       * transmitting/receiving buffer will not be read/write twice.
       */
      if (priv->txenab)
        {
          /*reload TX buffer*/

          nrf_i2s_tx_buffer_set(NRF_I2S, (uint32_t *)i2s_null_buffer);
          nrf_i2s_txrx_length_set(NRF_I2S, I2S_NULL_BUFFER_LENGTH);

        }
      else if (priv->rxenab)
        {
          /*reload RX buffer*/

          nrf_i2s_rx_buffer_set(NRF_I2S, (uint32_t *)i2s_null_buffer);
          nrf_i2s_txrx_length_set(NRF_I2S, I2S_NULL_BUFFER_LENGTH);
        }

    }

  priv->ended = priv->transmitting;
  priv->transmitting = priv->loaded;
  priv->loaded = container;

  return OK;

}



/****************************************************************************
 * Name: nrf52_i2s_start
 *
 * Description:
 *   Setup and start i2s transfer
 *
 * Input Parameters:
 *   priv - nrf52 i2s device instance
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 * Assumptions:
 *   Interrupts are disabled and the I2S peripheral is in idle status
 *
 ****************************************************************************/
static int nrf52_i2s_start(struct nrf52_i2s_s *priv)
{
  struct i2s_buffer_s *bfcontainer;
  nrf_i2s_channels_t i2s_channel;
  irqstate_t flags;

  /* If there are no pending transfer, then bail returning success */

  if (sq_empty(&priv->pend))
    {
      return OK;
    }


  flags = enter_critical_section();

  bfcontainer = (struct i2s_buffer_s *)sq_remfirst(&priv->pend);
  DEBUGASSERT(bfcontainer && bfcontainer->apb);

  leave_critical_section(flags);

  /*set channel*/

  i2s_channel = (priv->channel_setted == 2) ? NRF_I2S_CHANNELS_STEREO : NRF_I2S_CHANNELS_LEFT;
  nrf_i2s_set_channel(NRF_I2S, i2s_channel);
  priv->channel = priv->channel_setted;

  /*set mck, swidth, ration register*/

  nrf_i2s_clk_width_set(NRF_I2S, priv->clk_config_setted->swidth,
                        priv->clk_config_setted->mck,
                        priv->clk_config_setted->ratio);
  priv->clk_config_current = priv->clk_config_setted;


  /*set the new buffer as the I2S TX/RX buffer*/

  nrf52_i2s_reload(priv, bfcontainer);

  if (priv->txenab)
    {
      /*enable i2s tx function and disable rx function*/

      nrf_i2s_tx_enable_set(NRF_I2S, true);
      nrf_i2s_rx_enable_set(NRF_I2S, false);
    }
  else
    {
      /*enable i2s rx function and disable tx function*/

      nrf_i2s_tx_enable_set(NRF_I2S, false);
      nrf_i2s_rx_enable_set(NRF_I2S, true);
    }

  /*enable i2s*/

  nrf_i2s_enable(NRF_I2S);

  /*trigger start task*/

  nrf_i2s_task_trigger(NRF_I2S, NRF_I2S_TASK_START);

  /*enable I2S interrupt*/

  up_enable_irq(I2S_IRQn);


  return OK;
}

/****************************************************************************
 * Name: nrf52_i2s_clear_worker
 *
 * Description:
 *   Transfer done worker
 *
 * Input Parameters:
 *   arg - the I2S device instance cast to void*
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nrf52_i2s_clear_worker(void *arg)
{
  struct nrf52_i2s_s *priv = (struct nrf52_i2s_s *)arg;
  struct i2s_buffer_s *bfcontainer;
  irqstate_t flags;

  DEBUGASSERT(priv);

  /* Process each buffer in the done queue */

  while (sq_peek(&priv->done) != NULL)
    {
      /* Remove the buffer container from the done queue.  NOTE that
       * interupts must be enabled to do this because the done queue is
       * also modified from the interrupt level.
       */

      flags = enter_critical_section();
      bfcontainer = (struct i2s_buffer_s *)sq_remfirst(&priv->done);
      leave_critical_section(flags);

      /* Perform the transfer done callback */

      DEBUGASSERT(bfcontainer && bfcontainer->callback);
      bfcontainer->callback(&priv->dev, bfcontainer->apb,
                            bfcontainer->arg, bfcontainer->result);

      /* Release our reference on the audio buffer.  This may very likely
       * cause the audio buffer to be freed.
       */

      apb_free(bfcontainer->apb);

      /* And release the buffer container */

      nrf52_i2s_buf_free(priv, bfcontainer);
    }

}

/****************************************************************************
 * Name: nrf52_i2s_schedule
 *
 * Description:
 *   An transfer completion or timeout has occurred. Reload the TX/RX
 *   source/destination address and schedule the work in worker thread
 *   the working thread.
 *
 * Input Parameters:
 *   priv   - nrf52 i2s device instance
 *   result - The result of the transfer
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Interrupts are disabled
 *   - The timeout has been canceled.
 *
 ****************************************************************************/

static void nrf52_i2s_schedule(struct nrf52_i2s_s *priv, int result)
{
  struct i2s_buffer_s *bfcontainer;
  int ret;

  /*priv->loaded==NULL means we want to stop*/
  if (priv->loaded == NULL)
    {
      bfcontainer = NULL;
      /*priv->loaded == priv->transmitting ==NULL means we have already handled
       *the last meaningful buffer and we are OK to trigger the stop task.
       */
      if (priv->transmitting != NULL)
        {
          /*trigger STOP task*/
          nrf_i2s_task_trigger(NRF_I2S, NRF_I2S_TASK_STOP);

        }
    }
  else
    {
      /* If there are no pending transfer, then bail returning success */
      if (sq_empty(&priv->pend))
        {
          bfcontainer = NULL;
        }
      else
        {
          /*If the configration is changed, we will trigger a stop task and update the I2S
          register when we restart.*/
          if ((priv->channel_setted != priv->channel) ||
              (priv->clk_config_current != priv->clk_config_setted))
            {
              bfcontainer = NULL;
            }
          else
            {
              bfcontainer = (struct i2s_buffer_s *)sq_remfirst(&priv->pend);
            }
        }

    }

  nrf52_i2s_reload(priv, bfcontainer);


  /*priv->ended now pointed to the container which has just finished the transfer*/

  bfcontainer = priv->ended;

  if (bfcontainer != NULL)
    {
      /* Report the result of the transfer */

      bfcontainer->result = result;

      /* Add the completed buffer container to the tail of the done queue */

      sq_addlast((sq_entry_t *)bfcontainer, &priv->done);
    }

  /* If the worker has completed running, then reschedule the working thread.
   * REVISIT:  There may be a race condition here.  So we do nothing is the
   * worker is not available.
   */

  if ((!sq_empty(&priv->done)) && work_available(&priv->work))
    {
      /* Schedule the done processing to occur on the worker thread. */

      ret = work_queue(LPWORK, &priv->work, nrf52_i2s_clear_worker, priv, 0);
      if (ret != 0)
        {
          i2serr("ERROR: Failed to queue work: %d\n", ret);
        }
    }
}

/****************************************************************************
 * Name: nrf52_i2s_interrupt
 *
 * Description:
 *   This callback function is invoked at the completion of the I2S transfer.
 *
 * Input Parameters:
 *   handle - The DMA handler
 *   isr    - The interrupt status of the DMA transfer
 *   arg    - A pointer to the nrf52 i2s device instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
int nrf52_i2s_interrupt(int irq, FAR void *context, FAR void *arg)
{
  struct nrf52_i2s_s *priv = (struct nrf52_i2s_s *)arg;
  DEBUGASSERT(priv);

  if (nrf_i2s_event_check(NRF_I2S, NRF_I2S_EVENT_TXPTRUPD))
    {
      nrf_i2s_event_clear(NRF_I2S, NRF_I2S_EVENT_TXPTRUPD);
      i2sinfo("Event: %s.\r\n", (uint32_t)EVT_TO_STR(NRF_I2S_EVENT_TXPTRUPD));

      // If transmission is not enabled, but for some reason the TXPTRUPD
      // event has been generated, just ignore it.
      if (priv->txenab)
        {
          nrf52_i2s_schedule(priv, OK);
        }

    }

  if (nrf_i2s_event_check(NRF_I2S, NRF_I2S_EVENT_RXPTRUPD))
    {
      nrf_i2s_event_clear(NRF_I2S, NRF_I2S_EVENT_RXPTRUPD);
      i2sinfo("Event: %s.\r\n", (uint32_t)EVT_TO_STR(NRF_I2S_EVENT_RXPTRUPD));

      // If reception is not enabled, but for some reason the RXPTRUPD event
      // has been generated, just ignore it.
      if (priv->rxenab)
        {
          nrf52_i2s_schedule(priv, OK);
        }

    }

  /* we must handle the stopped event after TXPTRUPD/RXPTRUPD event*/
  if (nrf_i2s_event_check(NRF_I2S, NRF_I2S_EVENT_STOPPED))
    {
      nrf_i2s_event_clear(NRF_I2S, NRF_I2S_EVENT_STOPPED);
      i2sinfo("Event: %s.\r\n", (uint32_t)EVT_TO_STR(NRF_I2S_EVENT_STOPPED));

      nrf_i2s_disable(NRF_I2S);
      up_disable_irq(I2S_IRQn);
      nrf_i2s_tx_enable_set(NRF_I2S, false);
      nrf_i2s_rx_enable_set(NRF_I2S, false);

      priv->ended = NULL;
      priv->transmitting = NULL;
      priv->loaded = NULL;

      if (sq_empty(&priv->pend))
        {
          priv->txenab = false;
          priv->rxenab = false;
        }
      else
        {
          nrf52_i2s_start(priv);
        }
    }

  return OK;
}


/****************************************************************************
 * Name: nrf52_i2s_samplerate
 *
 * Description:
 *   Set the I2S RX/TX sample rate.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   rate - The I2S sample rate in samples (not bits) per second
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

static uint32_t nrf52_i2s_samplerate(struct i2s_dev_s *dev, uint32_t rate)
{
  struct nrf52_i2s_s *priv = (struct nrf52_i2s_s *)dev;
  int res;

  DEBUGASSERT(priv && rate > 0);

  /* set sample rate */
  res = nrf52_i2s_setbitrate(priv, rate, priv->datalen);

  if (res == OK)
    {

      /* Save the new data width */

      priv->samplerate = rate;
      return nrf52_i2s_getbitrate(priv);
    }
  else
    {
      i2serr("ERROR: Unsupported or invalid data sample rate: %d\n", rate);
      return 0;
    }

}

/****************************************************************************
 * Name: nrf52_i2s_datawidth
 *
 * Description:
 *   Set the I2S data width.  The bitrate is determined by
 *   2*sample_rate * data_width.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   width - The I2S data with in bits.
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

static uint32_t nrf52_i2s_datawidth(struct i2s_dev_s *dev, int bits)
{
  struct nrf52_i2s_s *priv = (struct nrf52_i2s_s *)dev;
  int res;

  DEBUGASSERT(priv && bits >= 8);

  res = nrf52_i2s_setbitrate(priv, priv->samplerate, bits);

  if (res == OK)
    {

      /* Save the new data width */

      priv->datalen = bits;
      return nrf52_i2s_getbitrate(priv);
    }
  else
    {
      i2serr("ERROR: Unsupported or invalid data width: %d\n", bits);
      return 0;
    }
}

/****************************************************************************
 * Name: nrf52_i2s_channel
 *
 * Description:
 *   Set the I2S channel.  The bitrate is determined by
 *   2*sample_rate * data_width.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   channel - The I2S channel.
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

static uint32_t nrf52_i2s_channel(struct i2s_dev_s *dev, int channel)
{
  struct nrf52_i2s_s *priv = (struct nrf52_i2s_s *)dev;

  DEBUGASSERT(priv && ((channel == 1) || (channel == 2)));

  /* Save the new data width */

  priv->channel_setted = (uint8_t)channel;
  return nrf52_i2s_getbitrate(priv);
}


/****************************************************************************
 * Name: nrf52_i2s_receive
 *
 * Description:
 *   Receive a block of data from I2S.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   apb      - A pointer to the audio buffer in which to recieve data
 *   callback - A user provided callback function that will be called at
 *              the completion of the transfer.  The callback will be
 *              performed in the context of the worker thread.
 *   arg      - An opaque argument that will be provided to the callback
 *              when the transfer complete
 *   timeout  - The timeout value to use.  The transfer will be canceled
 *              and an ETIMEDOUT error will be reported if this timeout
 *              elapsed without completion of the DMA transfer.  Units
 *              are system clock ticks.  Zero means no timeout.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.  NOTE:  This function
 *   only enqueues the transfer and returns immediately.  Success here only
 *   means that the transfer was enqueued correctly.
 *
 *   When the transfer is complete, a 'result' value will be provided as
 *   an argument to the callback function that will indicate if the transfer
 *   failed.
 *
 ****************************************************************************/

static int nrf52_i2s_receive(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                             i2s_callback_t callback, void *arg, uint32_t timeout)
{
  struct nrf52_i2s_s *priv = (struct nrf52_i2s_s *)dev;
  struct i2s_buffer_s *bfcontainer;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(priv && apb);
  i2sinfo("apb=%p nbytes=%d arg=%p timeout=%d\n",
          apb, apb->nbytes - apb->curbyte, arg, timeout);

  /* Allocate a buffer container in advance */

  bfcontainer = nrf52_i2s_buf_allocate(priv);
  DEBUGASSERT(bfcontainer);

  /* Get exclusive access to the SAI driver data */

  nrf52_i2s_exclsem_take(priv);

  /* Verify not already TX'ing */

  if (priv->txenab)
    {
      i2serr("ERROR: i2s has no receiver\n");
      ret = -EAGAIN;
      goto errout_with_exclsem;
    }

  /* Add a reference to the audio buffer */

  apb_reference(apb);

  /*we should make the audio sample buffer align to word(32bit),
  because it's required by the hardware.*/

  /*apb_realign_to_word(apb);*/

  /* Initialize the buffer container structure */

  bfcontainer->callback = (void *)callback;
  bfcontainer->timeout  = timeout;
  bfcontainer->arg      = arg;
  bfcontainer->apb      = apb;
  bfcontainer->result   = -EBUSY;

  /* Add the buffer container to the end of the pending queue */

  flags = enter_critical_section();
  sq_addlast((sq_entry_t *)bfcontainer, &priv->pend);
  leave_critical_section(flags);

  /* Then start the next transfer.  If there is already a transfer in progess,
   * then this will do nothing.
   */
  if (priv->rxenab == false)
    {
      priv->rxenab = true;
      ret = nrf52_i2s_start(priv);
      DEBUGASSERT(ret == OK);
    }

  nrf52_i2s_exclsem_give(priv);

  return OK;

errout_with_exclsem:
  nrf52_i2s_exclsem_give(priv);
  nrf52_i2s_buf_free(priv, bfcontainer);
  return ret;
}


/****************************************************************************
 * Name: nrf52_i2s_send
 *
 * Description:
 *   Send a block of data on I2S.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   apb      - A pointer to the audio buffer from which to send data
 *   callback - A user provided callback function that will be called at
 *              the completion of the transfer.  The callback will be
 *              performed in the context of the worker thread.
 *   arg      - An opaque argument that will be provided to the callback
 *              when the transfer complete
 *   timeout  - The timeout value to use.  The transfer will be canceled
 *              and an ETIMEDOUT error will be reported if this timeout
 *              elapsed without completion of the DMA transfer.  Units
 *              are system clock ticks.  Zero means no timeout.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.  NOTE:  This function
 *   only enqueues the transfer and returns immediately.  Success here only
 *   means that the transfer was enqueued correctly.
 *
 *   When the transfer is complete, a 'result' value will be provided as
 *   an argument to the callback function that will indicate if the transfer
 *   failed.
 *
 ****************************************************************************/

static int nrf52_i2s_send(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                          i2s_callback_t callback, void *arg, uint32_t timeout)
{
  struct nrf52_i2s_s *priv = (struct nrf52_i2s_s *)dev;
  struct i2s_buffer_s *bfcontainer;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(priv && apb);
  i2sinfo("apb=%p nbytes=%d arg=%p timeout=%d\n",
          apb, apb->nbytes - apb->curbyte, arg, timeout);

  /* Allocate a buffer container in advance */

  bfcontainer = nrf52_i2s_buf_allocate(priv);
  DEBUGASSERT(bfcontainer);

  /* Get exclusive access to the SAI driver data */

  nrf52_i2s_exclsem_take(priv);

  /* Verify not already RX'ing */

  if (priv->rxenab)
    {
      i2serr("ERROR: i2s has no transmitter\n");
      ret = -EAGAIN;
      goto errout_with_exclsem;
    }



  /* Add a reference to the audio buffer */

  apb_reference(apb);

  /*we should make the audio sample buffer align to word(32bit),
  because it's required by the hardware.*/

  /*apb_realign_to_word(apb);*/

  /* Initialize the buffer container structure */

  bfcontainer->callback = (void *)callback;
  bfcontainer->timeout  = timeout;
  bfcontainer->arg      = arg;
  bfcontainer->apb      = apb;
  bfcontainer->result   = -EBUSY;

  /* Add the buffer container to the end of the pending queue */

  flags = enter_critical_section();
  sq_addlast((sq_entry_t *)bfcontainer, &priv->pend);
  leave_critical_section(flags);

  /* Then start the next transfer.  If there is already a transfer in progess,
   * then this will do nothing.
   */
  if (priv->txenab == false)
    {
      priv->txenab = true;
      ret = nrf52_i2s_start(priv);
      DEBUGASSERT(ret == OK);
    }

  nrf52_i2s_exclsem_give(priv);

  return OK;

errout_with_exclsem:
  nrf52_i2s_exclsem_give(priv);
  nrf52_i2s_buf_free(priv, bfcontainer);
  return ret;
}

/****************************************************************************
 * Name: nrf52_i2s_bufsem_take
 *
 * Description:
 *   Take the buffer semaphore handling any exceptional conditions
 *
 * Input Parameters:
 *   priv - A reference to the I2S peripheral state
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void nrf52_i2s_bufsem_take(struct nrf52_i2s_s *priv)
{
  int ret;

  /* Wait until we successfully get the semaphore.  EINTR is the only
   * expected 'failure' (meaning that the wait for the semaphore was
   * interrupted by a signal).
   */

  do
    {
      ret = sem_wait(&priv->bufsem);
      DEBUGASSERT(ret == 0 || errno == EINTR);
    }
  while (ret < 0);
}

/****************************************************************************
 * Name: nrf52_i2s_buf_allocate
 *
 * Description:
 *   Allocate a buffer container by removing the one at the head of the
 *   free list
 *
 * Input Parameters:
 *   priv - nrf52 i2s device instance
 *
 * Returned Value:
 *   A non-NULL pointer to the allocate buffer container on success; NULL if
 *   there are no available buffer containers.
 *
 * Assumptions:
 *   The caller does NOT have exclusive access to the nrf52 i2s device structure.
 *   That would result in a deadlock!
 *
 ****************************************************************************/

static struct i2s_buffer_s *nrf52_i2s_buf_allocate(struct nrf52_i2s_s *priv)
{
  struct i2s_buffer_s *bfcontainer;
  irqstate_t flags;

  /* Set aside a buffer container.  By doing this, we guarantee that we will
   * have at least one free buffer container.
   */

  nrf52_i2s_bufsem_take(priv);

  /* Get the buffer from the head of the free list */

  flags = enter_critical_section();
  bfcontainer = priv->freelist;
  ASSERT(bfcontainer);

  /* Unlink the buffer from the freelist */

  priv->freelist = bfcontainer->flink;
  leave_critical_section(flags);
  return bfcontainer;
}

/****************************************************************************
 * Name: nrf52_i2s_buf_free
 *
 * Description:
 *   Free buffer container by adding it to the head of the free list
 *
 * Input Parameters:
 *   priv - nrf52 i2s device instance
 *   bfcontainer - The buffer container to be freed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The caller has exclusive access to the nrf52 i2s device structure
 *
 ****************************************************************************/

static void nrf52_i2s_buf_free(struct nrf52_i2s_s *priv, struct i2s_buffer_s *bfcontainer)
{
  irqstate_t flags;

  /* Put the buffer container back on the free list */

  flags = enter_critical_section();
  bfcontainer->flink  = priv->freelist;
  priv->freelist = bfcontainer;
  leave_critical_section(flags);

  /* Wake up any threads waiting for a buffer container */

  nrf52_i2s_bufsem_give(priv);
}

/****************************************************************************
 * Name: nrf52_i2s_buf_initialize
 *
 * Description:
 *   Initialize the buffer container allocator by adding all of the
 *   pre-allocated buffer containers to the free list
 *
 * Input Parameters:
 *   priv - nrf52 i2s device instance
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called early in I2S initialization so that there are no issues with
 *   concurrency.
 *
 ****************************************************************************/

static void nrf52_i2s_buf_initialize(struct nrf52_i2s_s *priv)
{
  int i;

  priv->freelist = NULL;
  sem_init(&priv->bufsem, 0, CONFIG_NRF52_I2S_MAXINFLIGHT);

  for (i = 0; i < CONFIG_NRF52_I2S_MAXINFLIGHT; i++)
    {
      nrf52_i2s_buf_free(priv, &priv->containers[i]);
    }
}

/****************************************************************************
 * Name: nrf52_i2s_setbitrate
 *
 * Description:
 *   set i2s sample rate and simple width, biterate = 2*samplerate*samplewidth
 *
 * Input Parameter:
 *   priv - nrf52 i2s device instance
 *   samplerate - sample rate, 8000/16000/22050/32000/44100/48000/96000HZ
 *   samplewidth - sample width, 8/16/24bit
 *
 * Returned Value:
 *   OK - success
 *   -EINVAL - the samplerate or samplewidth is not supported by the hardware
 ****************************************************************************/
static int nrf52_i2s_setbitrate(struct nrf52_i2s_s *priv,
                                uint32_t samplerate, uint8_t samplewidth)
{
  const struct nrf52_i2s_clk_s *ptr = nrf52_i2s_clk_table;
  const struct nrf52_i2s_clk_s *end = (struct nrf52_i2s_clk_s *)nrf52_i2s_clk_table + \
                                      ARRAY_SIZE(nrf52_i2s_clk_table);

  while (ptr < end)
    {
      /*if we find the specified combination of sample rate and sample width
        in the table*/
      if ((ptr->samplerate == samplerate) && (ptr->samplewidth == samplewidth))
        {
          /*we cannot set the corresponding registers right now,
          because I2S may be executing a TX/RX task currently,
          we just updata this variable and handle it later on.*/
          priv->clk_config_setted = ptr;
          break;
        }
      else
        {
          ptr++;
        }
    }

  if (ptr < end)
    {
      return OK;
    }
  else
    {
      i2serr("ERROR: nrf52_i2s_set_bit_rate: specified combination of sample rate \
              and sample width is not supported!\n");
      return -EINVAL;
    }
}

/****************************************************************************
 * Name: nrf52_i2s_config
 *
 * Description:
 *   config i2s registers
 *
 * Input Parameter:
 *
 * Returned Value:
 *
 ****************************************************************************/

static void nrf52_i2s_config(void)
{
#ifdef CONFIG_NRF52_I2S_MODE_MASTER
  nrf_i2s_set_mode(NRF_I2S, NRF_I2S_MODE_MASTER);
#endif

#ifdef CONFIG_NRF52_I2S_MODE_SLAVE
  nrf_i2s_set_mode(NRF_I2S, NRF_I2S_MODE_SLAVE);
#endif

#if (defined CONFIG_NRF52_I2S_MODE_MASTER) || (defined CONFIG_NRF52_I2S_MCK_OUTPUT)
  nrf_i2s_set_mck_enable(NRF_I2S, true);
#else
  nrf_i2s_set_mck_enable(NRF_I2S, false);
#endif

#ifdef CONFIG_NRF52_I2S_FORMAT_ORIGINAL_I2S
  nrf_i2s_set_format(NRF_I2S, NRF_I2S_FORMAT_I2S);
#endif

#ifdef CONFIG_NRF52_I2S_FORMAT_ALIGNED
  nrf_i2s_set_format(NRF_I2S, NRF_I2S_FORMAT_ALIGNED);
#endif

#ifdef CONFIG_NRF52_I2S_ALIGN_LEFT
  nrf_i2s_set_align(NRF_I2S, NRF_I2S_ALIGN_LEFT);
#endif

#ifdef CONFIG_NRF52_I2S_ALIGN_RIGHT
  nrf_i2s_set_align(NRF_I2S, NRF_I2S_ALIGN_RIGHT);
#endif


  nrf_i2s_event_clear(NRF_I2S, NRF_I2S_EVENT_RXPTRUPD);
  nrf_i2s_event_clear(NRF_I2S, NRF_I2S_EVENT_TXPTRUPD);
  nrf_i2s_event_clear(NRF_I2S, NRF_I2S_EVENT_STOPPED);
  nrf_i2s_int_enable(NRF_I2S,
                     NRF_I2S_INT_RXPTRUPD_MASK | \
                     NRF_I2S_INT_TXPTRUPD_MASK | \
                     NRF_I2S_INT_STOPPED_MASK);

}

/****************************************************************************
 * Name: nrf52_i2s_config_pin
 *
 * Description:
 *   config pins used by i2s peripheral
 *
 * Input Parameter:
 *
 * Returned Value:
 *
 ****************************************************************************/

static void nrf52_i2s_config_pin(void)
{
  uint32_t mck_pin, sdout_pin, sdin_pin;


  // Configure pins used by the peripheral:

  // - SCK and LRCK (required) - depending on the mode of operation these
  //   pins are configured as outputs (in Master mode) or inputs (in Slave
  //   mode).

#ifdef CONFIG_NRF52_I2S_MODE_MASTER
  nrf_gpio_cfg_output(BOARD_I2S_SCK_PIN);
  nrf_gpio_cfg_output(BOARD_I2S_WS_PIN);
#endif

#ifdef CONFIG_NRF52_I2S_MODE_SLAVE
  nrf_gpio_cfg_input(BOARD_I2S_SCK_PIN,  NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(BOARD_I2S_WS_PIN, NRF_GPIO_PIN_NOPULL);
#endif


  // - MCK (optional) - always output,
  if (BOARD_I2S_MCK_PIN != NRF52_INVALID_GPIO_PIN)
    {
      mck_pin = BOARD_I2S_MCK_PIN;
      nrf_gpio_cfg_output(mck_pin);
    }
  else
    {
      mck_pin = NRF_I2S_PIN_NOT_CONNECTED;
    }

  // - SDOUT (optional) - always output,
  if (BOARD_I2S_SDOUT_PIN != NRF52_INVALID_GPIO_PIN)
    {
      sdout_pin = BOARD_I2S_SDOUT_PIN;
      nrf_gpio_cfg_output(sdout_pin);
    }
  else
    {
      sdout_pin = NRF_I2S_PIN_NOT_CONNECTED;
    }

  // - SDIN (optional) - always input.
  if (BOARD_I2S_SDIN_PIN != NRF52_INVALID_GPIO_PIN)
    {
      sdin_pin = BOARD_I2S_SDIN_PIN;
      nrf_gpio_cfg_input(sdin_pin, NRF_GPIO_PIN_NOPULL);
    }
  else
    {
      sdin_pin = NRF_I2S_PIN_NOT_CONNECTED;
    }

  nrf_i2s_pins_set(NRF_I2S, BOARD_I2S_SCK_PIN, BOARD_I2S_WS_PIN,
                   mck_pin, sdout_pin, sdin_pin);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_i2s_initialize
 *
 * Description:
 *   Initialize the nrf52 i2s driver
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Valid I2S device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct i2s_dev_s *nrf52_i2s_initialize(void)
{
  struct nrf52_i2s_s *priv;
  int ret;

  i2sinfo("NRF52 i2s Initialize!\n");
  priv = &g_nrf52_i2s_priv;

  /* config peripheral registers */

  nrf52_i2s_config();

  /*set sample rate and sample width*/

  ret = nrf52_i2s_setbitrate(priv, priv->samplerate, priv->datalen);

  if (ret < 0)
    {
      i2serr("ERROR: nrf52 i2s configration failed!\n");
      goto err;
    }

  /* config pins used by the peripheral */

  nrf52_i2s_config_pin();

  sem_init(&priv->exclsem, 0, 1);

  /* Initialize queue */

  dq_init(&priv->pend);
  dq_init(&priv->done);

  /* Initialize buffering */

  nrf52_i2s_buf_initialize(priv);

  /* Attach the pdm interrupt */

  ret = irq_attach(I2S_IRQn, nrf52_i2s_interrupt, priv);
  if (ret < 0)
    {
      auderr("ERROR: I2S irq attach failed: %d\n", ret);
      goto err;
    }


  return &priv->dev;

err:
  return NULL;
}

#endif



