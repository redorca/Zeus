/****************************************************************************
 * arch/arm/src/nrf52/nrf52_pdm.c
 *
 *   Copyright (C) 2018 zGlue Inc. All rights reserved.
 *   Author: Zhengwei Wang <zhengwei@zglue.com>
 *
 * References:
 *
 * -  The framework for this driver is based on stm32l4_sai driver
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
#include "nrf_pdm.h"
#include "nrf_gpio.h"
#include "nrf52_gpio.h"
#include "nrf52_pdm.h"

#ifdef CONFIG_NRF52_PDM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SCHED_WORKQUEUE
#  error Work queue support is required (CONFIG_SCHED_WORKQUEUE)
#endif

#ifndef CONFIG_AUDIO
#  error CONFIG_AUDIO required by this driver
#endif

#ifndef CONFIG_PDM
#  error CONFIG_PDM required by this driver
#endif

#ifndef CONFIG_NRF52_PDM_GAINL
#define CONFIG_NRF52_PDM_GAINL 0x28
#endif

#ifndef CONFIG_NRF52_PDM_GAINR
#define CONFIG_NRF52_PDM_GAINR 0x28
#endif

#ifndef CONFIG_NRF52_PDM_DEFAULT_CHANNEL
#  define CONFIG_NRF52_PDM_DEFAULT_CHANNEL     (2)
#endif


#ifndef CONFIG_NRF52_PDM_MAXINFLIGHT
#  define CONFIG_NRF52_PDM_MAXINFLIGHT         (16)
#endif

/* NRF52 PDM peripheral only support 16kHZ samplerate and 16bit sample length */
#define NRF52_PDM_SAMPLERATE    16000
#define NRF52_PDM_SAMPLELENGTH  16

#define NRF52_PDM_DATA_ALIGN 1

#define EVT_TO_STR(event)   (event == NRF_PDM_EVENT_STARTED ? "NRF_PDM_EVENT_STARTED" : \
                            (event == NRF_PDM_EVENT_STOPPED ? "NRF_PDM_EVENT_STOPPED" : \
                            (event == NRF_PDM_EVENT_END ? "NRF_PDM_EVENT_END" : "UNKNOWN EVENT")))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* PDM buffer container */

struct pdm_buffer_s
{
  struct pdm_buffer_s *flink;  /* Supports a singly linked list */
  pdm_callback_t callback;     /* Function to call when the transfer completes */
  uint32_t timeout;            /* The timeout value to use with transfers */
  void *arg;                   /* The argument to be returned with the callback */
  struct ap_buffer_s *apb;     /* The audio buffer */
  int result;                  /* The result of the transfer */
};


/* The state of the one PDM peripheral */

struct nrf52_pdm_s
{
  struct pdm_dev_s dev;        /* Externally visible PDM interface */
  sem_t exclsem;               /* Assures mutually exclusive acess to pdm */
  sq_queue_t pend;             /* A queue of pending transfers */
  sq_queue_t done;             /* A queue of completed transfers */
  struct work_s work;          /* this work will clear all apb buffers in the done queue */

  /* PDM setting */
  uint8_t channel;             /* mono or stereo */
  uint8_t channel_setted;      /* new setted channel number */
  bool receiving;

  /* Pre-allocated pool of buffer containers */

  sem_t bufsem;                   /* Buffer wait semaphore */
  struct pdm_buffer_s *freelist;  /* A list a free buffer containers */
  struct pdm_buffer_s containers[CONFIG_NRF52_PDM_MAXINFLIGHT];

  struct pdm_buffer_s *working;/*point to the apb_buffer that is using currently by the pdm*/
  struct pdm_buffer_s *loaded;/*point to the apb_buffer that has been just handded over to pdm peripheral*/
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Semaphore helpers */

static void     nrf52_pdm_exclsem_take(struct nrf52_pdm_s *priv);
#define         nrf52_pdm_exclsem_give(priv) sem_post(&priv->exclsem)

static void     nrf52_pdm_bufsem_take(struct nrf52_pdm_s *priv);
#define         nrf52_pdm_bufsem_give(priv) sem_post(&priv->bufsem)

/* Buffer container helpers */

static struct pdm_buffer_s *nrf52_pdm_buf_allocate(struct nrf52_pdm_s *priv);
static void     nrf52_pdm_buf_free(struct nrf52_pdm_s *priv,
                                   struct pdm_buffer_s *bfcontainer);
static void     nrf52_pdm_buf_initialize(struct nrf52_pdm_s *priv);

/*interrupt handler*/
static int      nrf52_pdm_interrupt(int irq, FAR void *context, FAR void *arg);


/* PDM methods */

static uint32_t nrf52_pdm_samplerate(struct pdm_dev_s *dev, uint32_t rate);
static uint32_t nrf52_pdm_datawidth(struct pdm_dev_s *dev, int bits);
static uint32_t nrf52_pdm_channel(struct pdm_dev_s *dev, int channel);
static int      nrf52_pdm_receive(struct pdm_dev_s *dev, struct ap_buffer_s *apb,
                                  pdm_callback_t callback, void *arg, uint32_t timeout);
static int      nrf52_pdm_send(struct pdm_dev_s *dev, struct ap_buffer_s *apb,
                               pdm_callback_t callback, void *arg,
                               uint32_t timeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* PDM device operations */

static const struct pdm_ops_s g_pdmops =
{
  /* Receiver methods */

  .pdm_rxsamplerate = nrf52_pdm_samplerate,
  .pdm_rxdatawidth  = nrf52_pdm_datawidth,
  .pdm_rxchannel    = nrf52_pdm_channel,
  .pdm_receive      = nrf52_pdm_receive,

  /* Transmitter methods */

  .pdm_txsamplerate = nrf52_pdm_samplerate,
  .pdm_txdatawidth  = nrf52_pdm_datawidth,
  .pdm_txchannel    = nrf52_pdm_channel,
  .pdm_send         = nrf52_pdm_send,
};

/* pdm status */
static struct nrf52_pdm_s g_nrf52_pdm_priv =
{
  .dev.ops     = &g_pdmops,
  .channel     = CONFIG_NRF52_PDM_DEFAULT_CHANNEL,
  .channel_setted = CONFIG_NRF52_PDM_DEFAULT_CHANNEL,
  .receiving = false,
  .loaded = NULL,
  .working = NULL,
};

#define PDM_NULL_BUFFER_LENGTH 4
static uint16_t pdm_null_buffer[PDM_NULL_BUFFER_LENGTH] = {0, 0, 0, 0};


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_pdm_getbitrate
 *
 * Description:
 *   Get the currently configured bitrate
 *
 * Input Parameters:
 *   priv   - private PDM device structure
 *
 * Returned Value:
 *   The current bitrate
 *
 ****************************************************************************/

static inline uint32_t nrf52_pdm_getbitrate(struct nrf52_pdm_s *priv)
{
  /* Calculate the bitrate in Hz */

  return priv->channel * NRF52_PDM_SAMPLERATE * NRF52_PDM_SAMPLELENGTH;
}

/****************************************************************************
 * Name: nrf52_pdm_exclsem_take
 *
 * Description:
 *   Take the exclusive access semaphore handling any exceptional conditions
 *
 * Input Parameters:
 *   priv - A reference to the PDM peripheral state
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void nrf52_pdm_exclsem_take(struct nrf52_pdm_s *priv)
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
 * Name: nrf52_pdm_reload
 *
 * Description:
 *   updata the buffer address of PDM peripheral
 *
 * Input Parameters:
 *   priv - nrf52 pdm device instance
 *   container -- new buffer that will be loaded to PDM peripheral
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 *
 ****************************************************************************/

static int nrf52_pdm_reload(struct nrf52_pdm_s *priv, struct pdm_buffer_s *container)
{
  struct ap_buffer_s *apb;
  uintptr_t samp;
  apb_samp_t nbytes;


  if (container)
    {
      apb = container->apb;

      /* Get the transfer information, accounting for any data offset */

      samp = (uintptr_t)&apb->samp[apb->curbyte];
      nbytes = apb->nmaxbytes - apb->curbyte;

      DEBUGASSERT((samp & NRF52_PDM_DATA_ALIGN) == 0 && (nbytes & NRF52_PDM_DATA_ALIGN) == 0);

      /*reload target buffer*/

      nrf_pdm_buffer_set((uint32_t *)samp, nbytes / 2);

    }
  else
    {
      nrf_pdm_buffer_set((uint32_t *)pdm_null_buffer, PDM_NULL_BUFFER_LENGTH);
    }

  priv->working = priv->loaded;
  priv->loaded = container;

  return OK;

}



/****************************************************************************
 * Name: nrf52_pdm_start
 *
 * Description:
 *   Setup and start pdm transfer
 *
 * Input Parameters:
 *   priv - nrf52 pdm device instance
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 * Assumptions:
 *   Interrupts are disabled and the PDM peripheral is in idle status
 *
 ****************************************************************************/
static int nrf52_pdm_start(struct nrf52_pdm_s *priv)
{
  struct pdm_buffer_s *bfcontainer;
  nrf_pdm_mode_t pdm_channel;
  nrf_pdm_edge_t pdm_edge;
  irqstate_t flags;

  /* If there are no pending transfer, then bail returning success */

  if (sq_empty(&priv->pend))
    {
      return OK;
    }


  flags = enter_critical_section();

  bfcontainer = (struct pdm_buffer_s *)sq_remfirst(&priv->pend);
  DEBUGASSERT(bfcontainer && bfcontainer->apb);

  leave_critical_section(flags);

  /*set channel*/

  nrf_pdm_mode_get(&pdm_channel, &pdm_edge);
  pdm_channel = (priv->channel_setted == 2) ? NRF_PDM_MODE_STEREO : NRF_PDM_MODE_MONO;
  nrf_pdm_mode_set(pdm_channel, pdm_edge);
  priv->channel = priv->channel_setted;

  /*set the new buffer as the PDM TX/RX buffer*/

  nrf52_pdm_reload(priv, bfcontainer);

  //clear event
  nrf_pdm_event_clear(NRF_PDM_EVENT_STARTED);
  nrf_pdm_event_clear(NRF_PDM_EVENT_STOPPED);
  nrf_pdm_event_clear(NRF_PDM_EVENT_END);

  /*enable pdm*/

  nrf_pdm_enable();

  /*trigger start task*/

  nrf_pdm_task_trigger(NRF_PDM_TASK_START);

  /*enable PDM interrupt*/

  up_enable_irq(PDM_IRQn);


  return OK;
}

/****************************************************************************
 * Name: nrf52_pdm_clear_worker
 *
 * Description:
 *   Transfer done worker
 *
 * Input Parameters:
 *   arg - the PDM device instance cast to void*
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nrf52_pdm_clear_worker(void *arg)
{
  struct nrf52_pdm_s *priv = (struct nrf52_pdm_s *)arg;
  struct pdm_buffer_s *bfcontainer;
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
      bfcontainer = (struct pdm_buffer_s *)sq_remfirst(&priv->done);
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

      nrf52_pdm_buf_free(priv, bfcontainer);
    }

}

/****************************************************************************
 * Name: nrf52_pdm_interrupt
 *
 * Description:
 *   This callback function is invoked at the completion of the PDM transfer.
 *
 * Input Parameters:
 *   handle - The DMA handler
 *   isr    - The interrupt status of the DMA transfer
 *   arg    - A pointer to the nrf52 pdm device instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
int nrf52_pdm_interrupt(int irq, FAR void *context, FAR void *arg)
{
  struct nrf52_pdm_s *priv = (struct nrf52_pdm_s *)arg;
  struct pdm_buffer_s *bfcontainer;
  int ret;

  DEBUGASSERT(priv);

  if (nrf_pdm_event_check(NRF_PDM_EVENT_END))
    {
      nrf_pdm_event_clear(NRF_PDM_EVENT_END);
      audinfo("Event: %s.\r\n", (uint32_t)EVT_TO_STR(NRF_PDM_EVENT_END));


      /*priv->working now pointed to the container which has just finished the transfer*/

      bfcontainer = priv->working;

      if (bfcontainer != NULL)
        {
          /*set the apb->nbytes to tell the user how many bytes we have received*/

          bfcontainer->apb->nbytes = bfcontainer->apb->nmaxbytes;

          /* Report the result of the transfer */

          bfcontainer->result = OK;

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

          ret = work_queue(LPWORK, &priv->work, nrf52_pdm_clear_worker, priv, 0);
          if (ret != 0)
            {
              pdmerr("ERROR: Failed to queue work: %d\n", ret);
            }
        }
    }
  else if (nrf_pdm_event_check(NRF_PDM_EVENT_STARTED))
    {
      nrf_pdm_event_clear(NRF_PDM_EVENT_STARTED);
      audinfo("Event: %s.\r\n", (uint32_t)EVT_TO_STR(NRF_PDM_EVENT_STARTED));

      DEBUGASSERT((priv->loaded != NULL)  || (priv->working != NULL));

      if (priv->loaded == NULL)
        {
          bfcontainer = NULL;
          /*priv->loaded == NULL means we have already handled
           *the last meaningful buffer and we are OK to trigger the stop task.
           */
          nrf_pdm_task_trigger(NRF_PDM_TASK_STOP);
        }
      else
        {
          if (sq_empty(&priv->pend))
            {
              bfcontainer = NULL;
            }
          else
            {
              /*If the configration is changed, we will trigger a stop task and update the PDM
              register when we restart.*/
              if (priv->channel_setted != priv->channel)
                {
                  bfcontainer = NULL;
                }
              else
                {
                  bfcontainer = (struct pdm_buffer_s *)sq_remfirst(&priv->pend);
                }
            }

        }




      nrf52_pdm_reload(priv, bfcontainer);


    }
  else if (nrf_pdm_event_check(NRF_PDM_EVENT_STOPPED))
    {
      nrf_pdm_event_clear(NRF_PDM_EVENT_STOPPED);
      audinfo("Event: %s.\r\n", (uint32_t)EVT_TO_STR(NRF_PDM_EVENT_STOPPED));

      DEBUGASSERT((priv->loaded == NULL)  && (priv->working == NULL));
      nrf_pdm_disable();
      up_disable_irq(PDM_IRQn);

      priv->working = NULL;
      priv->loaded = NULL;

      if (sq_empty(&priv->pend))
        {
          priv->receiving = false;
        }
      else
        {
          nrf52_pdm_start(priv);
        }

    }

  return OK;


}


/****************************************************************************
 * Name: nrf52_pdm_samplerate
 *
 * Description:
 *   Set the PDM RX/TX sample rate.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   rate - The PDM sample rate in samples (not bits) per second
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

static uint32_t nrf52_pdm_samplerate(struct pdm_dev_s *dev, uint32_t rate)
{
  struct nrf52_pdm_s *priv = (struct nrf52_pdm_s *)dev;

  DEBUGASSERT(priv);

  if (rate == NRF52_PDM_SAMPLERATE)
    {
      return nrf52_pdm_getbitrate(priv);
    }
  else
    {
      pdmerr("ERROR: Unsupported or invalid data sample rate: %d\n", rate);
      return 0;
    }

}

/****************************************************************************
 * Name: nrf52_pdm_datawidth
 *
 * Description:
 *   Set the PDM data width.  The bitrate is determined by
 *   channel * sample_rate * data_width.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   width - The PDM data with in bits.
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

static uint32_t nrf52_pdm_datawidth(struct pdm_dev_s *dev, int bits)
{
  struct nrf52_pdm_s *priv = (struct nrf52_pdm_s *)dev;

  DEBUGASSERT(priv);

  if (bits == NRF52_PDM_SAMPLELENGTH)
    {
      return nrf52_pdm_getbitrate(priv);
    }
  else
    {
      pdmerr("ERROR: Unsupported or invalid data width: %d\n", bits);
      return 0;
    }
}

/****************************************************************************
 * Name: nrf52_pdm_channel
 *
 * Description:
 *   Set the PDM channel.  The bitrate is determined by
 *   channel * sample_rate * data_width.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   channel - The PDM channel.
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

static uint32_t nrf52_pdm_channel(struct pdm_dev_s *dev, int channel)
{
  struct nrf52_pdm_s *priv = (struct nrf52_pdm_s *)dev;

  DEBUGASSERT(priv && ((channel == 1) || (channel == 2)));

  /* Save the new data width */

  priv->channel_setted = (uint8_t)channel;
  return nrf52_pdm_getbitrate(priv);
}


/****************************************************************************
 * Name: nrf52_pdm_receive
 *
 * Description:
 *   Receive a block of data from PDM.
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

static int nrf52_pdm_receive(struct pdm_dev_s *dev, struct ap_buffer_s *apb,
                             pdm_callback_t callback, void *arg, uint32_t timeout)
{
  struct nrf52_pdm_s *priv = (struct nrf52_pdm_s *)dev;
  struct pdm_buffer_s *bfcontainer;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(priv && apb);
  pdminfo("apb=%p nbytes=%d arg=%p timeout=%d\n",
          apb, apb->nmaxbytes - apb->curbyte, arg, timeout);

  /* Allocate a buffer container in advance */

  bfcontainer = nrf52_pdm_buf_allocate(priv);
  DEBUGASSERT(bfcontainer);

  /* Get exclusive access to the SAI driver data */

  nrf52_pdm_exclsem_take(priv);

  /* Add a reference to the audio buffer */

  apb_reference(apb);

  /*we should make the audio sample buffer align to halfword(16bit),
  because it's required by the hardware.*/

  /*apb_realign_to_hword(apb);*/

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
  if (priv->receiving == false)
    {
      priv->receiving = true;
      ret = nrf52_pdm_start(priv);
      DEBUGASSERT(ret == OK);
    }

  nrf52_pdm_exclsem_give(priv);

  return OK;
}


/****************************************************************************
 * Name: nrf52_pdm_send
 *
 * Description:
 *   Send a block of data on PDM.
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

static int nrf52_pdm_send(struct pdm_dev_s *dev, struct ap_buffer_s *apb,
                          pdm_callback_t callback, void *arg, uint32_t timeout)
{

  /* nrf52 PDM peripheral only supports data input, data output is not supported*/

  return -ENODEV;
}

/****************************************************************************
 * Name: nrf52_pdm_bufsem_take
 *
 * Description:
 *   Take the buffer semaphore handling any exceptional conditions
 *
 * Input Parameters:
 *   priv - A reference to the PDM peripheral state
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void nrf52_pdm_bufsem_take(struct nrf52_pdm_s *priv)
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
 * Name: nrf52_pdm_buf_allocate
 *
 * Description:
 *   Allocate a buffer container by removing the one at the head of the
 *   free list
 *
 * Input Parameters:
 *   priv - nrf52 pdm device instance
 *
 * Returned Value:
 *   A non-NULL pointer to the allocate buffer container on success; NULL if
 *   there are no available buffer containers.
 *
 * Assumptions:
 *   The caller does NOT have exclusive access to the nrf52 pdm device structure.
 *   That would result in a deadlock!
 *
 ****************************************************************************/

static struct pdm_buffer_s *nrf52_pdm_buf_allocate(struct nrf52_pdm_s *priv)
{
  struct pdm_buffer_s *bfcontainer;
  irqstate_t flags;

  /* Set aside a buffer container.  By doing this, we guarantee that we will
   * have at least one free buffer container.
   */

  nrf52_pdm_bufsem_take(priv);

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
 * Name: nrf52_pdm_buf_free
 *
 * Description:
 *   Free buffer container by adding it to the head of the free list
 *
 * Input Parameters:
 *   priv - nrf52 pdm device instance
 *   bfcontainer - The buffer container to be freed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The caller has exclusive access to the nrf52 pdm device structure
 *
 ****************************************************************************/

static void nrf52_pdm_buf_free(struct nrf52_pdm_s *priv, struct pdm_buffer_s *bfcontainer)
{
  irqstate_t flags;

  /* Put the buffer container back on the free list */

  flags = enter_critical_section();
  bfcontainer->flink  = priv->freelist;
  priv->freelist = bfcontainer;
  leave_critical_section(flags);

  /* Wake up any threads waiting for a buffer container */

  nrf52_pdm_bufsem_give(priv);
}

/****************************************************************************
 * Name: nrf52_pdm_buf_initialize
 *
 * Description:
 *   Initialize the buffer container allocator by adding all of the
 *   pre-allocated buffer containers to the free list
 *
 * Input Parameters:
 *   priv - nrf52 pdm device instance
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called early in PDM initialization so that there are no issues with
 *   concurrency.
 *
 ****************************************************************************/

static void nrf52_pdm_buf_initialize(struct nrf52_pdm_s *priv)
{
  int i;

  priv->freelist = NULL;
  sem_init(&priv->bufsem, 0, CONFIG_NRF52_PDM_MAXINFLIGHT);

  for (i = 0; i < CONFIG_NRF52_PDM_MAXINFLIGHT; i++)
    {
      nrf52_pdm_buf_free(priv, &priv->containers[i]);
    }
}


/****************************************************************************
 * Name: nrf52_pdm_config
 *
 * Description:
 *   config pdm registers
 *
 * Input Parameter:
 *
 * Returned Value:
 *
 ****************************************************************************/

static void nrf52_pdm_config(void)
{

#ifdef CONFIG_NRF52_PDM_MONO_SAMMPLE_LEFT
  nrf_pdm_mode_set(NRF_PDM_MODE_STEREO, NRF_PDM_EDGE_LEFTFALLING);
#else
  nrf_pdm_mode_set(NRF_PDM_MODE_STEREO, NRF_PDM_EDGE_LEFTRISING);
#endif

  nrf_pdm_clock_set(NRF_PDM_FREQ_1032K);
  nrf_pdm_gain_set(CONFIG_NRF52_PDM_GAINL, CONFIG_NRF52_PDM_GAINR);


  /*clear event*/
  nrf_pdm_event_clear(NRF_PDM_EVENT_STARTED);
  nrf_pdm_event_clear(NRF_PDM_EVENT_STOPPED);
  nrf_pdm_event_clear(NRF_PDM_EVENT_END);

  /*enable pdm interrupt*/
  nrf_pdm_int_enable(NRF_PDM_INT_STARTED | NRF_PDM_INT_END | NRF_PDM_INT_STOPPED);


}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_pdm_initialize
 *
 * Description:
 *   Initialize the nrf52 pdm driver
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Valid PDM device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct pdm_dev_s *nrf52_pdm_initialize(void)
{
  struct nrf52_pdm_s *priv;
  int ret;

  pdminfo("NRF52 pdm Initialize!\n");
  priv = &g_nrf52_pdm_priv;

  /* config peripheral registers */

  nrf52_pdm_config();

  /* config pins used by the peripheral */

  nrf_gpio_cfg_output(BOARD_PDM_CLK_PIN);
  nrf_gpio_pin_clear(BOARD_PDM_CLK_PIN);
  nrf_gpio_cfg_input(BOARD_PDM_DIN_PIN, NRF_GPIO_PIN_NOPULL);
  nrf_pdm_psel_connect(BOARD_PDM_CLK_PIN, BOARD_PDM_DIN_PIN );


  sem_init(&priv->exclsem, 0, 1);

  /* Initialize queue */

  dq_init(&priv->pend);
  dq_init(&priv->done);

  /* Initialize buffering */

  nrf52_pdm_buf_initialize(priv);

  /* Attach the pdm interrupt */

  ret = irq_attach(PDM_IRQn, nrf52_pdm_interrupt, priv);
  if (ret < 0)
    {
      auderr("ERROR: PDM irq attach failed: %d\n", ret);
      goto err;
    }


  return &priv->dev;

err:
  return NULL;
}

#endif



