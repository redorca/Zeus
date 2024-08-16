/****************************************************************************
 * drivers/audio/pcm510x.c
 *
 * drivers for pcm5100x/pcm5101x/pcm5102x
 *
 *   Copyright (C) 2018 zGlue Inc. All rights reserved.
 *   Author: Zhengwei Wang <zhengwei@zglue.com>
 *
 * References:
 *
 * -  The framework for this driver is based on Ken Pettit's VS1053 driver.
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
#include <sys/ioctl.h>

#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <fixedmath.h>
#include <queue.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/pcm510x.h>
#include <nuttx/lib/math.h>

#ifdef CONFIG_AUDIO_PCM510X

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
struct pcm510x_dev_s;

static void     pcm510x_takesem(sem_t *sem);
#define         pcm510x_givesem(s) sem_post(s)

/* Audio lower half methods (and close friends) */

static int      pcm510x_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                                FAR struct audio_caps_s *caps);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      pcm510x_configure(FAR struct audio_lowerhalf_s *dev,
                                  FAR void *session, FAR const struct audio_caps_s *caps);
#else
static int      pcm510x_configure(FAR struct audio_lowerhalf_s *dev,
                                  FAR const struct audio_caps_s *caps);
#endif
static int      pcm510x_shutdown(FAR struct audio_lowerhalf_s *dev);
static void     pcm510x_senddone(FAR struct i2s_dev_s *i2s,
                                 FAR struct ap_buffer_s *apb, FAR void *arg, int result);
static void     pcm510x_returnbuffers(FAR struct pcm510x_dev_s *priv);
static int      pcm510x_sendbuffer(FAR struct pcm510x_dev_s *priv);

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      pcm510x_start(FAR struct audio_lowerhalf_s *dev,
                              FAR void *session);
#else
static int      pcm510x_start(FAR struct audio_lowerhalf_s *dev);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      pcm510x_stop(FAR struct audio_lowerhalf_s *dev,
                             FAR void *session);
#else
static int      pcm510x_stop(FAR struct audio_lowerhalf_s *dev);
#endif
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      pcm510x_pause(FAR struct audio_lowerhalf_s *dev,
                              FAR void *session);
static int      pcm510x_resume(FAR struct audio_lowerhalf_s *dev,
                               FAR void *session);
#else
static int      pcm510x_pause(FAR struct audio_lowerhalf_s *dev);
static int      pcm510x_resume(FAR struct audio_lowerhalf_s *dev);
#endif
#endif
static int      pcm510x_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                      FAR struct ap_buffer_s *apb);
static int      pcm510x_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                                     FAR struct ap_buffer_s *apb);
static int      pcm510x_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                              unsigned long arg);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      pcm510x_reserve(FAR struct audio_lowerhalf_s *dev,
                                FAR void **session);
#else
static int      pcm510x_reserve(FAR struct audio_lowerhalf_s *dev);
#endif
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      pcm510x_release(FAR struct audio_lowerhalf_s *dev,
                                FAR void *session);
#else
static int      pcm510x_release(FAR struct audio_lowerhalf_s *dev);
#endif

static void    *pcm510x_workerthread(pthread_addr_t pvarg);

/****************************************************************************
 * Private Types
 ****************************************************************************/
struct pcm510x_dev_s
{
  struct audio_lowerhalf_s dev;             /* PCM510X audio lower half (this device) */

  /* Our specific driver data goes here */

  FAR struct i2s_dev_s   *i2s;              /* I2S driver to use */
  const FAR struct pcm510x_lower_s *lower;              /* I2S driver to use */
  struct dq_queue_s       pendq;            /* Queue of pending buffers to be sent */
  struct dq_queue_s       doneq;            /* Queue of sent buffers to be returned */
  mqd_t                   mq;               /* Message queue for receiving messages */
  char                    mqname[16];       /* Our message queue name */
  pthread_t               threadid;         /* ID of our thread */
  sem_t                   pendsem;          /* Protect pendq */
  uint16_t                samprate;         /* Configured samprate (samples/sec) */
  uint8_t                 nchannels;        /* Number of channels (1 or 2) */
  uint8_t                 bpsamp;           /* Bits per sample (8 or 16) */
  volatile uint8_t        inflight;         /* Number of audio buffers in-flight */
  bool                    running;          /* True: Worker thread is running */
  bool                    paused;           /* True: Playing is paused */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  bool                    terminating;      /* True: Stop requested */
#endif
  bool                    reserved;         /* True: Device is reserved */
  volatile int            result;           /* The result of the last transfer */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct audio_ops_s g_audioops =
{
  .getcaps       = pcm510x_getcaps,       /* getcaps        */
  .configure     = pcm510x_configure,     /* configure      */
  .shutdown      = pcm510x_shutdown,      /* shutdown       */
  .start         = pcm510x_start,         /* start          */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  .stop          = pcm510x_stop,          /* stop           */
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  .pause         = pcm510x_pause,         /* pause          */
  .resume        = pcm510x_resume,        /* resume         */
#endif
  .allocbuffer   = NULL,                  /* allocbuffer    */
  .freebuffer    = NULL,                  /* freebuffer     */
  .enqueuebuffer = pcm510x_enqueuebuffer, /* enqueue_buffer */
  .cancelbuffer  = pcm510x_cancelbuffer,  /* cancel_buffer  */
  .ioctl         = pcm510x_ioctl,         /* ioctl          */
  .read          = NULL,                  /* read           */
  .write         = NULL,                  /* write          */
  .reserve       = pcm510x_reserve,       /* reserve        */
  .release       = pcm510x_release        /* release        */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/************************************************************************************
 * Name: pcm510x_takesem
 *
 * Description:
 *  Take a semaphore count, handling the nasty EINTR return if we are interrupted
 *  by a signal.
 *
 ************************************************************************************/

static void pcm510x_takesem(sem_t *sem)
{
  int ret;

  do
    {
      ret = sem_wait(sem);
      DEBUGASSERT(ret == 0 || errno == EINTR);
    }
  while (ret < 0);
}

/****************************************************************************
 * Name: pcm510x_getcaps
 *
 * Description:
 *   Get the audio device capabilities
 *
 ****************************************************************************/

static int pcm510x_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                           FAR struct audio_caps_s *caps)
{
  /* Validate the structure */

  DEBUGASSERT(caps && caps->ac_len >= sizeof(struct audio_caps_s));
  audinfo("type=%d ac_type=%d\n", type, caps->ac_type);

  /* Fill in the caller's structure based on requested info */

  caps->ac_format.hw  = 0;
  caps->ac_controls.w = 0;

  switch (caps->ac_type)
    {
      /* Caller is querying for the types of units we support */

      case AUDIO_TYPE_QUERY:

        /* Provide our overall capabilities.  The interfacing software
         * must then call us back for specific info for each capability.
         */

        caps->ac_channels = 2;       /* Stereo output */

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:
              /* We don't decode any formats!  Only something above us in
               * the audio stream can perform decoding on our behalf.
               */

              /* The types of audio units we implement */

              caps->ac_controls.b[0] = AUDIO_TYPE_OUTPUT;

              break;

            default:
              break;
          }

        break;

      /* Provide capabilities of our OUTPUT unit */

      case AUDIO_TYPE_OUTPUT:

        caps->ac_channels = 2;

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:

              /* Report the Sample rates we support */

              caps->ac_controls.hw[0] = AUDIO_SAMP_RATE_8K | AUDIO_SAMP_RATE_11K |
                                        AUDIO_SAMP_RATE_16K | AUDIO_SAMP_RATE_22K |
                                        AUDIO_SAMP_RATE_32K | AUDIO_SAMP_RATE_44K |
                                        AUDIO_SAMP_RATE_48K | AUDIO_SAMP_RATE_96K |
                                        AUDIO_SAMP_RATE_128K | AUDIO_SAMP_RATE_160K |
                                        AUDIO_SAMP_RATE_172K | AUDIO_SAMP_RATE_192K;
              break;

            default:
              break;
          }

        break;

      /* All others we don't support */

      default:

        /* Zero out the fields to indicate no support */

        caps->ac_subtype = 0;
        caps->ac_channels = 0;

        break;
    }

  /* Return the length of the audio_caps_s struct for validation of
   * proper Audio device type.
   */

  return caps->ac_len;
}

/****************************************************************************
 * Name: pcm510x_configure
 *
 * Description:
 *   Configure the audio device for the specified  mode of operation.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int pcm510x_configure(FAR struct audio_lowerhalf_s *dev,
                             FAR void *session,
                             FAR const struct audio_caps_s *caps)
#else
static int pcm510x_configure(FAR struct audio_lowerhalf_s *dev,
                             FAR const struct audio_caps_s *caps)
#endif
{
  FAR struct pcm510x_dev_s *priv = (FAR struct pcm510x_dev_s *)dev;
  int ret = OK;

  DEBUGASSERT(priv && caps);
  audinfo("ac_type: %d\n", caps->ac_type);

  /* Process the configure operation */

  switch (caps->ac_type)
    {

      case AUDIO_TYPE_OUTPUT:
        {
          audinfo("  AUDIO_TYPE_OUTPUT:\n");
          audinfo("    Number of channels: %u\n", caps->ac_channels);
          audinfo("    Sample rate:        %u\n", caps->ac_controls.hw[0]);
          audinfo("    Sample width:       %u\n", caps->ac_controls.b[2]);

          /* Verify that all of the requested values are supported */

          ret = -ERANGE;
          if (caps->ac_channels != 1 && caps->ac_channels != 2)
            {
              auderr("ERROR: Unsupported number of channels: %d\n",
                     caps->ac_channels);
              break;
            }

          if ((caps->ac_controls.b[2] != 16) && (caps->ac_controls.b[2] != 24)
              && (caps->ac_controls.b[2] != 32))
            {
              auderr("ERROR: Unsupported bits per sample: %d\n",
                     caps->ac_controls.b[2]);
              break;
            }

          /* Save the current stream configuration */

          priv->samprate  = caps->ac_controls.hw[0];
          priv->nchannels = caps->ac_channels;
          priv->bpsamp    = caps->ac_controls.b[2];

          /* Config I2S to support the resulting number or channels,
           * bits per sample, and bitrate.
           */

          I2S_TXDATAWIDTH(priv->i2s, priv->bpsamp);
          I2S_TXSAMPLERATE(priv->i2s, priv->samprate);
          I2S_TXCHANNEL(priv->i2s, priv->nchannels);

          ret = OK;
        }
        break;

      default:
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: pcm510x_shutdown
 *
 * Description:
 *   Shutdown the PCM510X chip and put it in the lowest power state possible.
 *
 ****************************************************************************/

static int pcm510x_shutdown(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct pcm510x_dev_s *priv = (FAR struct pcm510x_dev_s *)dev;

  DEBUGASSERT(priv);

  /*we donot need to shutdown the pcm510x, it will automatically
  enter power saving mode when no data transfered throught I2S.*/
  return OK;
}

/****************************************************************************
 * Name: pcm510x_senddone
 *
 * Description:
 *   This is the I2S callback function that is invoked when the transfer
 *   completes.
 *
 ****************************************************************************/

static void  pcm510x_senddone(FAR struct i2s_dev_s *i2s,
                              FAR struct ap_buffer_s *apb, FAR void *arg,
                              int result)
{
  FAR struct pcm510x_dev_s *priv = (FAR struct pcm510x_dev_s *)arg;
  struct audio_msg_s msg;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(i2s && priv && priv->running && apb);
  audinfo("apb=%p inflight=%d result=%d\n", apb, priv->inflight, result);

  /* We do not place any restriction on the context in which this function
   * is called.  It may be called from an interrupt handler.  Therefore, the
   * doneq and in-flight values might be accessed from the interrupt level.
   * Not the best design.  But we will use interrupt controls to protect
   * against that possibility.
   */

  flags = enter_critical_section();

  /* Add the completed buffer to the end of our doneq.  We do not yet
   * decrement the reference count.
   */

  dq_addlast((FAR dq_entry_t *)apb, &priv->doneq);

  /* And decrement the number of buffers in-flight */

  DEBUGASSERT(priv->inflight > 0);
  priv->inflight--;

  /* Save the result of the transfer */
  /* REVISIT:  This can be overwritten */

  priv->result = result;
  leave_critical_section(flags);

  /* Now send a message to the worker thread, informing it that there are
   * buffers in the done queue that need to be cleaned up.
   */

  msg.msgId = AUDIO_MSG_COMPLETE;
  ret = mq_send(priv->mq, (FAR const char *)&msg, sizeof(msg),
                CONFIG_PCM510X_MSG_PRIO);
  if (ret < 0)
    {
      auderr("ERROR: mq_send failed: %d\n", errno);
    }
}

/****************************************************************************
 * Name: pcm510x_returnbuffers
 *
 * Description:
 *   This function is called after the complete of one or more data
 *   transfers.  This function will empty the done queue and release our
 *   reference to each buffer.
 *
 ****************************************************************************/

static void pcm510x_returnbuffers(FAR struct pcm510x_dev_s *priv)
{
  FAR struct ap_buffer_s *apb;
  irqstate_t flags;

  /* The doneq and in-flight values might be accessed from the interrupt
   * level in some implementations.  Not the best design.  But we will
   * use interrupt controls to protect against that possibility.
   */

  flags = enter_critical_section();
  while (dq_peek(&priv->doneq) != NULL)
    {
      /* Take the next buffer from the queue of completed transfers */

      apb = (FAR struct ap_buffer_s *)dq_remfirst(&priv->doneq);
      leave_critical_section(flags);

      audinfo("Returning: apb=%p curbyte=%d nbytes=%d flags=%04x\n",
              apb, apb->curbyte, apb->nbytes, apb->flags);

      /* Are we returning the final buffer in the stream? */

      if ((apb->flags & AUDIO_APB_FINAL) != 0)
        {
          /* Both the pending and the done queues should be empty and there
           * should be no buffers in-flight.
           */

          DEBUGASSERT(dq_empty(&priv->doneq) && dq_empty(&priv->pendq) &&
                      priv->inflight == 0);

          /* Set the terminating flag.  This will, eventually, cause the
           * worker thread to exit (if it is not already terminating).
           */

          audinfo("Terminating\n");
          priv->terminating = true;
        }

      /* Release our reference to the audio buffer */

      apb_free(apb);

      /* Send the buffer back up to the previous level. */

#ifdef CONFIG_AUDIO_MULTI_SESSION
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK, NULL);
#else
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
#endif
      flags = enter_critical_section();
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: pcm510x_sendbuffer
 *
 * Description:
 *   Start the transfer an audio buffer to the PCM510X via I2S.  This
 *   will not wait for the transfer to complete but will return immediately.
 *   the pcm510x_senddone called will be invoked when the transfer
 *   completes, stimulating the worker thread to call this function again.
 *
 ****************************************************************************/

static int pcm510x_sendbuffer(FAR struct pcm510x_dev_s *priv)
{
  FAR struct ap_buffer_s *apb;
  irqstate_t flags;
  uint32_t timeout;
  int shift;
  int ret = OK;

  /* Loop while there are audio buffers to be sent and we have few than
   * CONFIG_PCM510X_INFLIGHT then "in-flight"
   *
   * The 'inflight' value might be modified from the interrupt level in some
   * implementations.  We will use interrupt controls to protect against
   * that possibility.
   *
   * The 'pendq', on the other hand, is protected via a semaphore.  Let's
   * hold the semaphore while we are busy here and disable the interrupts
   * only while accessing 'inflight'.
   */

  pcm510x_takesem(&priv->pendsem);
  while (priv->inflight < CONFIG_PCM510X_INFLIGHT &&
         dq_peek(&priv->pendq) != NULL && !priv->paused)
    {
      /* Take next buffer from the queue of pending transfers */

      apb = (FAR struct ap_buffer_s *)dq_remfirst(&priv->pendq);
      audinfo("Sending apb=%p, size=%d inflight=%d\n",
              apb, apb->nbytes, priv->inflight);

      /* Increment the number of buffers in-flight before sending in order
       * to avoid a possible race condition.
       */

      flags = enter_critical_section();
      priv->inflight++;
      leave_critical_section(flags);

      /* Send the entire audio buffer via I2S.  What is a reasonable timeout
       * to use?  This would depend on the bit rate and size of the buffer.
       *
       * Samples in the buffer (samples):
       *   = buffer_size * 8 / bpsamp                           samples
       * Sample rate (samples/second):
       *   = samplerate * nchannels
       * Expected transfer time (seconds):
       *   = (buffer_size * 8) / bpsamp / samplerate / nchannels
       *
       * We will set the timeout about twice that.
       *
       * NOTES:
       * - The multiplier of 8 becomes 16000 for 2x and units of
       *   milliseconds.
       * - 16000 is a approximately 16384 (1 << 14), bpsamp is either
       *   (1 << 4) or (1 << 5), and nchannels is either (1 << 0) or
       *   (1 << 1).  So this can be simplifies to (milliseconds):
       *
       *   = (buffer_size << shift) / samplerate
       */

      shift  = (priv->bpsamp == 16) ? 14 - 4 : 14 - 5;
      shift -= (priv->nchannels > 1) ? 1 : 0;

      timeout = MSEC2TICK(((uint32_t)(apb->nbytes - apb->curbyte) << shift) /
                          (uint32_t)priv->samprate);

      ret = I2S_SEND(priv->i2s, apb, pcm510x_senddone, priv, timeout);
      if (ret < 0)
        {
          auderr("ERROR: I2S_SEND failed: %d\n", ret);
          break;
        }
    }

  pcm510x_givesem(&priv->pendsem);
  return ret;
}

/****************************************************************************
 * Name: pcm510x_start
 *
 * Description:
 *   Start the configured operation (audio streaming, volume enabled, etc.).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int pcm510x_start(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int pcm510x_start(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct pcm510x_dev_s *priv = (FAR struct pcm510x_dev_s *)dev;
  struct sched_param sparam;
  struct mq_attr attr;
  pthread_attr_t tattr;
  FAR void *value;
  int ret;

  audinfo("Entry\n");

  /* Exit reduced power modes of operation */
  /* REVISIT */

  /* Create a message queue for the worker thread */

  snprintf(priv->mqname, sizeof(priv->mqname), "/tmp/%X", priv);

  attr.mq_maxmsg  = 16;
  attr.mq_msgsize = sizeof(struct audio_msg_s);
  attr.mq_curmsgs = 0;
  attr.mq_flags   = 0;

  priv->mq = mq_open(priv->mqname, O_RDWR | O_CREAT, 0644, &attr);
  if (priv->mq == NULL)
    {
      /* Error creating message queue! */

      auderr("ERROR: Couldn't allocate message queue\n");
      return -ENOMEM;
    }

  /* Join any old worker thread we had created to prevent a memory leak */

  if (priv->threadid != 0)
    {
      audinfo("Joining old thread\n");
      pthread_join(priv->threadid, &value);
    }

  /* Start our thread for sending data to the device */

  pthread_attr_init(&tattr);
  sparam.sched_priority = sched_get_priority_max(SCHED_FIFO) - 3;
  (void)pthread_attr_setschedparam(&tattr, &sparam);
  (void)pthread_attr_setstacksize(&tattr, CONFIG_PCM510X_WORKER_STACKSIZE);

  audinfo("Starting worker thread\n");
  ret = pthread_create(&priv->threadid, &tattr, pcm510x_workerthread,
                       (pthread_addr_t)priv);
  if (ret != OK)
    {
      auderr("ERROR: pthread_create failed: %d\n", ret);
    }
  else
    {
      pthread_setname_np(priv->threadid, "pcm510x");
      audinfo("Created worker thread\n");
    }

  return ret;
}

/****************************************************************************
 * Name: pcm510x_stop
 *
 * Description: Stop the configured operation (audio streaming, volume
 *              disabled, etc.).
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int pcm510x_stop(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int pcm510x_stop(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct pcm510x_dev_s *priv = (FAR struct pcm510x_dev_s *)dev;
  struct audio_msg_s term_msg;
  FAR void *value;

  /* Send a message to stop all audio streaming */

  term_msg.msgId = AUDIO_MSG_STOP;
  term_msg.u.data = 0;
  mq_send(priv->mq, (FAR const char *)&term_msg, sizeof(term_msg),
          CONFIG_PCM510X_MSG_PRIO);

  /* Join the worker thread */

  pthread_join(priv->threadid, &value);
  priv->threadid = 0;

  /* Enter into a reduced power usage mode */
  /* REVISIT: */

  return OK;
}
#endif

/****************************************************************************
 * Name: pcm510x_pause
 *
 * Description: Pauses the playback.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int pcm510x_pause(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int pcm510x_pause(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct pcm510x_dev_s *priv = (FAR struct pcm510x_dev_s *)dev;

  if (priv->running && !priv->paused)
    {
      /* Disable interrupts to prevent us from suppling any more data */

      priv->paused = true;

#if CONFIG_PCM510X_SOFT_MUTE_ENABLE
      if (priv->lower->xsmt_pin_set)
        {
          priv->lower->xsmt_pin_set(false);
        }
#endif

    }

  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: pcm510x_resume
 *
 * Description: Resumes the playback.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int pcm510x_resume(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int pcm510x_resume(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct pcm510x_dev_s *priv = (FAR struct pcm510x_dev_s *)dev;

  if (priv->running && priv->paused)
    {
      priv->paused = false;

#if CONFIG_PCM510X_SOFT_MUTE_ENABLE
      if (priv->lower->xsmt_pin_set)
        {
          priv->lower->xsmt_pin_set(true);
        }

#endif

      /* Enable interrupts to allow sampling data */

      pcm510x_sendbuffer(priv);
    }

  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: pcm510x_enqueuebuffer
 *
 * Description: Enqueue an Audio Pipeline Buffer for playback/ processing.
 *
 ****************************************************************************/

static int pcm510x_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                 FAR struct ap_buffer_s *apb)
{
  FAR struct pcm510x_dev_s *priv = (FAR struct pcm510x_dev_s *)dev;
  struct audio_msg_s  term_msg;
  int ret;

  audinfo("Enqueueing: apb=%p curbyte=%d nbytes=%d flags=%04x\n",
          apb, apb->curbyte, apb->nbytes, apb->flags);

  /* Take a reference on the new audio buffer */

  apb_reference(apb);

  /* Add the new buffer to the tail of pending audio buffers */

  pcm510x_takesem(&priv->pendsem);
  apb->flags |= AUDIO_APB_OUTPUT_ENQUEUED;
  dq_addlast(&apb->dq_entry, &priv->pendq);
  pcm510x_givesem(&priv->pendsem);

  /* Send a message to the worker thread indicating that a new buffer has been
   * enqueued.  If mq is NULL, then the playing has not yet started.  In that
   * case we are just "priming the pump" and we don't need to send any message.
   */

  ret = OK;
  if (priv->mq != NULL)
    {
      term_msg.msgId  = AUDIO_MSG_ENQUEUE;
      term_msg.u.data = 0;

      ret = mq_send(priv->mq, (FAR const char *)&term_msg, sizeof(term_msg),
                    CONFIG_PCM510X_MSG_PRIO);
      if (ret < 0)
        {
          int errcode = errno;
          DEBUGASSERT(errcode > 0);

          auderr("ERROR: mq_send failed: %d\n", errcode);
          UNUSED(errcode);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: pcm510x_cancelbuffer
 *
 * Description: Called when an enqueued buffer is being cancelled.
 *
 ****************************************************************************/

static int pcm510x_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                                FAR struct ap_buffer_s *apb)
{
  audinfo("apb=%p\n", apb);
  return OK;
}

/****************************************************************************
 * Name: pcm510x_ioctl
 *
 * Description: Perform a device ioctl
 *
 ****************************************************************************/

static int pcm510x_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                         unsigned long arg)
{
#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
  FAR struct ap_buffer_info_s *bufinfo;
#endif

  /* Deal with ioctls passed from the upper-half driver */

  switch (cmd)
    {
      /* Check for AUDIOIOC_HWRESET ioctl.  This ioctl is passed straight
       * through from the upper-half audio driver.
       */

      case AUDIOIOC_HWRESET:
        {
          /* REVISIT:  Should we completely re-initialize the chip?   We
           * can't just issue a software reset; that would puts all PCM510X
           * registers back in their default state.
           */

          audinfo("AUDIOIOC_HWRESET:\n");
        }
        break;

        /* Report our preferred buffer size and quantity */

#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
      case AUDIOIOC_GETBUFFERINFO:
        {
          audinfo("AUDIOIOC_GETBUFFERINFO:\n");
          bufinfo              = (FAR struct ap_buffer_info_s *) arg;
          bufinfo->buffer_size = CONFIG_PCM510X_BUFFER_SIZE;
          bufinfo->nbuffers    = CONFIG_PCM510X_NUM_BUFFERS;
        }
        break;
#endif

      default:
        audinfo("Ignored\n");
        break;
    }

  return OK;
}

/****************************************************************************
 * Name: pcm510x_reserve
 *
 * Description: Reserves a session (the only one we have).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int pcm510x_reserve(FAR struct audio_lowerhalf_s *dev,
                           FAR void **session)
#else
static int pcm510x_reserve(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct pcm510x_dev_s *priv = (FAR struct pcm510x_dev_s *) dev;
  int   ret = OK;

  /* Borrow the APBQ semaphore for thread sync */

  pcm510x_takesem(&priv->pendsem);
  if (priv->reserved)
    {
      ret = -EBUSY;
    }
  else
    {
      /* Initialize the session context */

#ifdef CONFIG_AUDIO_MULTI_SESSION
      *session           = NULL;
#endif
      priv->inflight    = 0;
      priv->running     = false;
      priv->paused      = false;
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
      priv->terminating = false;
#endif
      priv->reserved    = true;
    }

  pcm510x_givesem(&priv->pendsem);

  return ret;
}

/****************************************************************************
 * Name: pcm510x_release
 *
 * Description: Releases the session (the only one we have).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int pcm510x_release(FAR struct audio_lowerhalf_s *dev,
                           FAR void *session)
#else
static int pcm510x_release(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct pcm510x_dev_s *priv = (FAR struct pcm510x_dev_s *)dev;
  void  *value;

  /* Join any old worker thread we had created to prevent a memory leak */

  if (priv->threadid != 0)
    {
      pthread_join(priv->threadid, &value);
      priv->threadid = 0;
    }

  /* Borrow the APBQ semaphore for thread sync */

  pcm510x_takesem(&priv->pendsem);

  /* Really we should free any queued buffers here */

  priv->reserved = false;
  pcm510x_givesem(&priv->pendsem);

  return OK;
}



/****************************************************************************
 * Name: pcm510x_workerthread
 *
 *  This is the thread that feeds data to the chip and keeps the audio
 *  stream going.
 *
 ****************************************************************************/

static void *pcm510x_workerthread(pthread_addr_t pvarg)
{
  FAR struct pcm510x_dev_s *priv = (struct pcm510x_dev_s *) pvarg;
  struct audio_msg_s msg;
  FAR struct ap_buffer_s *apb;
  int msglen;
  int prio;

  audinfo("Entry\n");

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  priv->terminating = false;
#endif

  /* Mark ourself as running and make sure that PCM510X interrupts are
   * enabled.
   */

  priv->running = true;

#if CONFIG_PCM510X_SOFT_MUTE_ENABLE
  if (priv->lower->xsmt_pin_set)
    {
      priv->lower->xsmt_pin_set(true);
    }
#endif


  /* Loop as long as we are supposed to be running and as long as we have
   * buffers in-flight.
   */

  while (priv->running || priv->inflight > 0)
    {
      /* Check if we have been asked to terminate.  We have to check if we
       * still have buffers in-flight.  If we do, then we can't stop until
       * birds come back to roost.
       */

      if (priv->terminating && priv->inflight <= 0)
        {
          /* We are IDLE.  Break out of the loop and exit. */

          break;
        }
      else
        {
          /* Check if we can send more audio buffers to the PCM510X */

          pcm510x_sendbuffer(priv);
        }

      /* Wait for messages from our message queue */

      msglen = mq_receive(priv->mq, (FAR char *)&msg, sizeof(msg), &prio);

      /* Handle the case when we return with no message */

      if (msglen < sizeof(struct audio_msg_s))
        {
          auderr("ERROR: Message too small: %d\n", msglen);
          continue;
        }

      /* Process the message */

      switch (msg.msgId)
        {
          /* The ISR has requested more data.  We will catch this case at
           * the top of the loop.
           */

          case AUDIO_MSG_DATA_REQUEST:
            audinfo("AUDIO_MSG_DATA_REQUEST\n");
            break;

            /* Stop the playback */

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
          case AUDIO_MSG_STOP:
            /* Indicate that we are terminating */

            audinfo("AUDIO_MSG_STOP: Terminating\n");
            priv->terminating = true;
            break;
#endif

          /* We have a new buffer to send.  We will catch this case at
           * the top of the loop.
           */

          case AUDIO_MSG_ENQUEUE:
            audinfo("AUDIO_MSG_ENQUEUE\n");
            break;

          /* We will wake up from the I2S callback with this message */

          case AUDIO_MSG_COMPLETE:
            audinfo("AUDIO_MSG_COMPLETE\n");
            pcm510x_returnbuffers(priv);
            break;

          default:
            auderr("ERROR: Ignoring message ID %d\n", msg.msgId);
            break;
        }
    }

#if CONFIG_PCM510X_SOFT_MUTE_ENABLE
  if (priv->lower->xsmt_pin_set)
    {
      priv->lower->xsmt_pin_set(false);
    }
#endif

  /* Return any pending buffers in our pending queue */

  pcm510x_takesem(&priv->pendsem);
  while ((apb = (FAR struct ap_buffer_s *)dq_remfirst(&priv->pendq)) != NULL)
    {
      /* Release our reference to the buffer */

      apb_free(apb);

      /* Send the buffer back up to the previous level. */

#ifdef CONFIG_AUDIO_MULTI_SESSION
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK, NULL);
#else
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
#endif
    }

  pcm510x_givesem(&priv->pendsem);

  /* Return any pending buffers in our done queue */

  pcm510x_returnbuffers(priv);

  /* Close the message queue */

  mq_close(priv->mq);
  mq_unlink(priv->mqname);
  priv->mq = NULL;

  /* Send an AUDIO_MSG_COMPLETE message to the client */

#ifdef CONFIG_AUDIO_MULTI_SESSION
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK, NULL);
#else
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK);
#endif

  audinfo("Exit\n");
  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pcm510x_initialize
 *
 * Description:
 *   Initialize the PCM510X device.
 *
 * Input Parameters:
 *   lower   - gpio related function
 *   i2s     - An I2S driver instance
 *
 * Returned Value:
 *   A new lower half audio interface for the PCM510X device is returned on
 *   success; NULL is returned on failure.
 *
 ****************************************************************************/

FAR struct audio_lowerhalf_s *pcm510x_initialize(const FAR struct pcm510x_lower_s *lower,
                                                 FAR struct i2s_dev_s *i2s)
{
  FAR struct pcm510x_dev_s *priv;

  /* Sanity check */
  DEBUGASSERT(lower);
  DEBUGASSERT(i2s);

  /* Allocate a PCM510X device structure */

  priv = (FAR struct pcm510x_dev_s *)kmm_zalloc(sizeof(struct pcm510x_dev_s));
  if (priv)
    {
      /* Initialize the PCM510X device structure.  Since we used kmm_zalloc,
       * only the non-zero elements of the structure need to be initialized.
       */

      priv->dev.ops    = &g_audioops;
      priv->i2s        = i2s;
      priv->lower        = lower;

      sem_init(&priv->pendsem, 0, 1);
      dq_init(&priv->pendq);
      dq_init(&priv->doneq);

#if CONFIG_PCM510X_SOFT_MUTE_ENABLE
      if (priv->lower->xsmt_pin_init)
        {
          priv->lower->xsmt_pin_init();
        }
#endif


      return &priv->dev;
    }

  return NULL;
}


#endif/*CONFIG_AUDIO_PCM510X*/