/****************************************************************************
 * audio/pcm_encode.c
 *
 *   Copyright (C) 2018 zGlue Inc. All rights reserved.
 *   Author: Zhengwei Wang <zhengwei@zglue.com>
 *
 * Based on the original audio framework from:
 *
 *   Author: Ken Pettit <pettitkd@gmail.com>
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
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/pcm.h>

#if defined(CONFIG_AUDIO) && defined(CONFIG_AUDIO_FORMAT_PCM)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#define CONFIG_PCM_DEBUG 1 /* For now */

/* Often defined and re-defined macros */

#ifndef MIN
#  define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a,b) (((a) > (b)) ? (a) : (b))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure describes the internal state of the PCM encoder */

struct pcm_encode_s
{
  /* This is is our our appearance to the outside world.  This *MUST* be the
   * first element of the structure so that we can freely cast between types
   * struct audio_lowerhalf and struct pcm_encode_s.
   */

  struct audio_lowerhalf_s export;

  /* These are our operations that intervene between the player application
   * and the lower level driver.  Unlike the ops in the struct
   * audio_lowerhalf_s, these are writeable because we need to customize a
   * few of the methods based upon what is supported by the lower level
   * driver.
   */

  struct audio_ops_s ops;

  /* This is the contained, low-level DAC-type device and will receive the
   * encoded PCM audio data.
   */

  FAR struct audio_lowerhalf_s *lower;

  /* Session returned from the lower level driver */

#ifdef CONFIG_AUDIO_MULTI_SESSION
  FAR void *session;
#endif

  /* These are values extracted from WAV file header */

  uint32_t samprate;               /* 8000, 44100, ... */
  uint32_t byterate;               /* samprate * nchannels * bpsamp / 8 */
  uint8_t  align;                  /* nchannels * bpsamp / 8 */
  uint8_t  bpsamp;                 /* Bits per sample: 8 bits = 8, 16 bits = 16 */
  uint8_t  nchannels;              /* Mono=1, Stereo=2 */
  bool     streaming;              /* Streaming PCM data chunk */
};

struct sample_rate_map_s
{
  uint16_t flag;
  uint16_t value;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Actually, I think the audio system has a defact here:
 * When we use the AUDIOIOC_GETCAPS command, we get the samplerate expressed
 * in macro definition, which range from 8K to 192K.However, when we set the
 * configration using AUDIOIOC_CONFIGURE command, we use the actual value
 * and it is uint16_t type, whose range is 0 - 65535. I don't know how to
 * handle it, so I use the following map and can only set the samplerate
 * up to 48K.
 */
static const struct sample_rate_map_s sample_rate_flag_value_map[] =
{
  {AUDIO_SAMP_RATE_48K, 48000},
  {AUDIO_SAMP_RATE_44K, 44100},
  {AUDIO_SAMP_RATE_32K, 32000},
  {AUDIO_SAMP_RATE_22K, 22050},
  {AUDIO_SAMP_RATE_16K, 16000},
  {AUDIO_SAMP_RATE_11K, 11025},
  {AUDIO_SAMP_RATE_8K, 8000},
};


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helper functions *********************************************************/
#ifdef CONFIG_ENDIAN_BIG
static uint16_t pcm_leuint16(uint16_t value);
static uint16_t pcm_leuint32(uint32_t value);
#else
#  define pcm_leuint16(v) (v)
#  define pcm_leuint32(v) (v)
#endif

/* struct audio_lowerhalf_s methods *****************************************/

static int  pcm_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                        FAR struct audio_caps_s *caps);

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  pcm_configure(FAR struct audio_lowerhalf_s *dev,
                          FAR void *session, FAR const struct audio_caps_s *caps);
#else
static int  pcm_configure(FAR struct audio_lowerhalf_s *dev,
                          FAR const struct audio_caps_s *caps);
#endif

static int  pcm_shutdown(FAR struct audio_lowerhalf_s *dev);

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  pcm_start(FAR struct audio_lowerhalf_s *dev, FAR void *session);
#else
static int  pcm_start(FAR struct audio_lowerhalf_s *dev);
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  pcm_stop(FAR struct audio_lowerhalf_s *dev, FAR void *session);
#else
static int  pcm_stop(FAR struct audio_lowerhalf_s *dev);
#endif
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  pcm_pause(FAR struct audio_lowerhalf_s *dev, FAR void *session);
#else
static int  pcm_pause(FAR struct audio_lowerhalf_s *dev);
#endif

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  pcm_resume(FAR struct audio_lowerhalf_s *dev, FAR void *session);
#else
static int  pcm_resume(FAR struct audio_lowerhalf_s *dev);
#endif
#endif

static int  pcm_allocbuffer(FAR struct audio_lowerhalf_s *dev,
                            FAR struct audio_buf_desc_s *apb);
static int  pcm_freebuffer(FAR struct audio_lowerhalf_s *dev,
                           FAR struct audio_buf_desc_s *apb);
static int  pcm_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                              FAR struct ap_buffer_s *apb);
static int  pcm_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                             FAR struct ap_buffer_s *apb);
static int  pcm_ioctl(FAR struct audio_lowerhalf_s *dev,
                      int cmd, unsigned long arg);

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  pcm_reserve(FAR struct audio_lowerhalf_s *dev, FAR void **session);
#else
static int  pcm_reserve(FAR struct audio_lowerhalf_s *dev);
#endif

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  pcm_release(FAR struct audio_lowerhalf_s *dev, FAR void *session);
#else
static int  pcm_release(FAR struct audio_lowerhalf_s *dev);
#endif

/* Audio callback */

#ifdef CONFIG_AUDIO_MULTI_SESSION
static void pcm_callback(FAR void *arg, uint16_t reason,
                         FAR struct ap_buffer_s *apb, uint16_t status,
                         FAR void *session);
#else
static void pcm_callback(FAR void *arg, uint16_t reason,
                         FAR struct ap_buffer_s *apb, uint16_t status);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/



/****************************************************************************
 * Name: pcm_leuint16
 *
 * Description:
 *   Get a 16-bit value stored in little endian order for a big-endian
 *   machine.
 *
 ****************************************************************************/

#ifdef CONFIG_ENDIAN_BIG
static uint16_t pcm_leuint16(uint16_t value)
{
  return (((value & 0x00ff) << 8) |
          ((value >> 8) & 0x00ff));
}
#endif

/****************************************************************************
 * Name: pcm_leuint16
 *
 * Description:
 *   Get a 16-bit value stored in little endian order for a big-endian
 *   machine.
 *
 ****************************************************************************/

#ifdef CONFIG_ENDIAN_BIG
static uint16_t pcm_leuint32(uint32_t value)
{
  return (((value & 0x000000ff) << 24) |
          ((value & 0x0000ff00) <<  8) |
          ((value & 0x00ff0000) >>  8) |
          ((value & 0xff000000) >> 24));
}
#endif


/****************************************************************************
 * Name: pcm_add_wav_header
 *
 * Description:
 *   add WAV file header.  A WAV file is a particular packaging of an audio file;
 *
 ****************************************************************************/

static void pcm_add_wav_header(FAR struct pcm_encode_s *priv, uint8_t *data)
{
  FAR struct wav_header_s *wav = (FAR struct wav_header_s *)data;

  DEBUGASSERT(wav);

  /* Transfer the purported WAV file header into our stack storage,
   * correcting for endian issues as needed.
   */

  wav->hdr.chunkid   = pcm_leuint32(WAV_HDR_CHUNKID);

  /* We don't know the actual length of this file at this point, so we set this value
   * as big as we can, and it is safe for most of the audio players.
   */
  wav->hdr.chunklen  = pcm_leuint32(0xffffff24);

  wav->hdr.format    = pcm_leuint32(WAV_HDR_FORMAT);

  wav->fmt.chunkid   = pcm_leuint32(WAV_FMT_CHUNKID);
  wav->fmt.chunklen  = pcm_leuint32(WAV_FMT_CHUNKLEN);
  wav->fmt.format    = pcm_leuint16(WAV_FMT_FORMAT);
  wav->fmt.nchannels = pcm_leuint16(priv->nchannels);
  wav->fmt.samprate  = pcm_leuint32(priv->samprate);
  wav->fmt.byterate  = pcm_leuint32(priv->byterate);
  wav->fmt.align     = pcm_leuint16(priv->align);
  wav->fmt.bpsamp    = pcm_leuint16(priv->bpsamp);

  wav->data.chunkid  = pcm_leuint32(WAV_DATA_CHUNKID);

  /* We don't know the actual length of this file at this point, so we set this value
   * as big as we can, and it is safe for most of the audio players.
   */

  wav->data.chunklen = pcm_leuint32(0xffffff00);

}


/****************************************************************************
 * Name: pcm_getcaps
 *
 * Description:
 *   This method is called to retrieve the lower-half device capabilities.
 *   It will be called with device type AUDIO_TYPE_QUERY to request the
 *   overall capabilities, such as to determine the types of devices supported
 *   audio formats supported, etc.  Then it may be called once or more with
 *   reported supported device types to determine the specific capabilities
 *   of that device type (such as MP3 encoder, WMA encoder, PCM output, etc.).
 *
 ****************************************************************************/

static int pcm_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                       FAR struct audio_caps_s *caps)
{
  FAR struct pcm_encode_s *priv = (FAR struct pcm_encode_s *)dev;
  FAR struct audio_lowerhalf_s *lower;
  int ret;

  DEBUGASSERT(priv);

  /* Defer the operation to the lower device driver */

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->getcaps);

  /* Get the capabilities of the lower-level driver */

  ret = lower->ops->getcaps(lower, type, caps);
  if (ret < 0)
    {
      auderr("ERROR: Lower getcaps() failed: %d\n", ret);
      return ret;
    }

  /* Modify the capabilities reported by the lower driver:  PCM is the only
   * supported format that we will report, regardless of what the lower driver
   * reported.
   */

  if (caps->ac_subtype == AUDIO_TYPE_QUERY)
    {
      caps->ac_format.hw = (1 << (AUDIO_FMT_PCM - 1));
    }

  return caps->ac_len;
}

/****************************************************************************
 * Name: pcm_configure
 *
 * Description:
 *   This method is called to bind the lower-level driver to the upper-level
 *   driver and to configure the driver for a specific mode of
 *   operation defined by the parameters selected in supplied device caps
 *   structure.  The lower-level device should perform any initialization
 *   needed to prepare for operations in the specified mode.  It should not,
 *   however, process any audio data until the start method is called.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int pcm_configure(FAR struct audio_lowerhalf_s *dev,
                         FAR void *session, FAR const struct audio_caps_s *caps)
#else
static int pcm_configure(FAR struct audio_lowerhalf_s *dev,
                         FAR const struct audio_caps_s *caps)
#endif
{
  FAR struct pcm_encode_s *priv = (FAR struct pcm_encode_s *)dev;
  FAR struct audio_lowerhalf_s *lower;
  int ret;

  DEBUGASSERT(priv);

  /* Defer all other operations to the lower device driver */

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->configure);

  audinfo("Defer to lower configure\n");
#ifdef CONFIG_AUDIO_MULTI_SESSION
  ret = lower->ops->configure(lower, session, caps);
#else
  ret = lower->ops->configure(lower, caps);
#endif

  if ((caps->ac_type == AUDIO_TYPE_INPUT) && (ret == OK))
    {
      priv->samprate  = caps->ac_controls.hw[0];
      priv->nchannels = caps->ac_channels;
      priv->bpsamp    = caps->ac_controls.b[2];
      priv->byterate  = priv->samprate * priv->nchannels * priv->bpsamp / 8;
      priv->align     = priv->nchannels * priv->bpsamp / 8;
    }

  return ret;
}


/****************************************************************************
 * Name: pcm_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   should stop processing audio data, including terminating any active
 *   output generation.  It should also disable the audio hardware and put
 *   it into the lowest possible power usage state.
 *
 *   Any enqueued Audio Pipeline Buffers that have not been processed / dequeued
 *   should be dequeued by this function.
 *
 ****************************************************************************/

static int pcm_shutdown(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct pcm_encode_s *priv = (FAR struct pcm_encode_s *)dev;
  FAR struct audio_lowerhalf_s *lower;

  DEBUGASSERT(priv);

  /* We are no longer streaming audio */

  priv->streaming = false;

  /* Defer the operation to the lower device driver */

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->start);

  audinfo("Defer to lower shutdown\n");
  return lower->ops->shutdown(lower);
}

/****************************************************************************
 * Name: pcm_start
 *
 * Description:
 *   Start audio streaming in the configured mode.  For input and synthesis
 *   devices, this means it should begin sending streaming audio data.  For output
 *   or processing type device, it means it should begin processing of any enqueued
 *   Audio Pipeline Buffers.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int pcm_start(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int pcm_start(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct pcm_encode_s *priv = (FAR struct pcm_encode_s *)dev;
  FAR struct audio_lowerhalf_s *lower;

  DEBUGASSERT(priv);

  /* Defer the operation to the lower device driver */

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->start);

  audinfo("Defer to lower start\n");
#ifdef CONFIG_AUDIO_MULTI_SESSION
  return lower->ops->start(lower, session);
#else
  return lower->ops->start(lower);
#endif
}

/****************************************************************************
 * Name: pcm_stop
 *
 * Description:
 *   Stop audio streaming and/or processing of enqueued Audio Pipeline
 *   Buffers
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int pcm_stop(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int pcm_stop(FAR struct audio_lowerhalf_s *dev)
#endif
#endif
{
  FAR struct pcm_encode_s *priv = (FAR struct pcm_encode_s *)dev;
  FAR struct audio_lowerhalf_s *lower;

  DEBUGASSERT(priv);

  /* We are no longer streaming */

  priv->streaming = false;

  /* Defer the operation to the lower device driver */

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->stop);

  audinfo("Defer to lower stop\n");
#ifdef CONFIG_AUDIO_MULTI_SESSION
  return lower->ops->stop(lower, session);
#else
  return lower->ops->stop(lower);
#endif
}

/****************************************************************************
 * Name: pcm_pause
 *
 * Description:
 *   Pause the audio stream.  Should keep current playback context active
 *   in case a resume is issued.  Could be called and then followed by a stop.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int pcm_pause(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int pcm_pause(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct pcm_encode_s *priv = (FAR struct pcm_encode_s *)dev;
  FAR struct audio_lowerhalf_s *lower;

  DEBUGASSERT(priv);

  /* Defer the operation to the lower device driver */

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->pause);

  audinfo("Defer to lower pause\n");
#ifdef CONFIG_AUDIO_MULTI_SESSION
  return lower->ops->pause(lower, session);
#else
  return lower->ops->pause(lower);
#endif
}

/****************************************************************************
 * Name: pcm_resume
 *
 * Description:
 *   Resumes audio streaming after a pause.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int pcm_resume(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int pcm_resume(FAR struct audio_lowerhalf_s *dev)
#endif
#endif
{
  FAR struct pcm_encode_s *priv = (FAR struct pcm_encode_s *)dev;
  FAR struct audio_lowerhalf_s *lower;

  DEBUGASSERT(priv);

  /* Defer the operation to the lower device driver */

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->resume);

  audinfo("Defer to lower resume\n");
#ifdef CONFIG_AUDIO_MULTI_SESSION
  return lower->ops->resume(lower, session);
#else
  return lower->ops->resume(lower);
#endif
}

/****************************************************************************
 * Name: pcm_allocbuffer
 *
 * Description:
 *   Allocate an audio pipeline buffer.  This routine provides the
 *   lower-half driver with the opportunity to perform special buffer
 *   allocation if needed, such as allocating from a specific memory
 *   region (DMA-able, etc.).  If not supplied, then the top-half
 *   driver will perform a standard kumm_malloc using normal user-space
 *   memory region.
 *
 ****************************************************************************/

static int pcm_allocbuffer(FAR struct audio_lowerhalf_s *dev,
                           FAR struct audio_buf_desc_s *apb)
{
  FAR struct pcm_encode_s *priv = (FAR struct pcm_encode_s *)dev;
  FAR struct audio_lowerhalf_s *lower;

  DEBUGASSERT(priv);

  /* Defer the operation to the lower device driver */

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->allocbuffer);

  audinfo("Defer to lower allocbuffer\n");
  return lower->ops->allocbuffer(lower, apb);
}

/****************************************************************************
 * Name: pcm_freebuffer
 *
 * Description:
 *   Free an audio pipeline buffer.  If the lower-level driver provides an
 *   allocbuffer routine, it should also provide the freebuffer routine to
 *   perform the free operation.
 *
 ****************************************************************************/

static int pcm_freebuffer(FAR struct audio_lowerhalf_s *dev,
                          FAR struct audio_buf_desc_s *apb)
{
  FAR struct pcm_encode_s *priv = (FAR struct pcm_encode_s *)dev;
  FAR struct audio_lowerhalf_s *lower;

  DEBUGASSERT(priv);

  /* Defer the operation to the lower device driver */

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->freebuffer);

  audinfo("Defer to lower freebuffer, apb=%p\n", apb);
  return lower->ops->freebuffer(lower, apb);
}

/****************************************************************************
 * Name: pcm_samplerate_map
 *
 * Description:
 *   map the sample rate expressed in micro define to actual value.
 *
 ****************************************************************************/

static int16_t pcm_samplerate_map(uint16_t samplerate_flag)
{
  const struct sample_rate_map_s *ptr = sample_rate_flag_value_map;
  const struct sample_rate_map_s *end = (struct sample_rate_map_s *)sample_rate_flag_value_map + \
                                        (sizeof(sample_rate_flag_value_map) / sizeof(sample_rate_flag_value_map[0]));


  while (ptr < end)
    {
      /*if we find a supported samplerate*/
      if (ptr->flag & samplerate_flag)
        {
          return ptr->value;
        }
      else
        {
          ptr++;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: pcm_config_default
 *
 * Description:
 *   Ask the lower audio for its supported samplerate, choose the highest one
 *   and set it.
 *
 ****************************************************************************/

static int pcm_config_default(FAR struct pcm_encode_s *priv)
{
  FAR struct audio_lowerhalf_s *lower;
  struct audio_caps_s caps;
  int ret;
  int16_t samplerate;


  DEBUGASSERT(priv);

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->getcaps && lower->ops->configure);

  /* We have the device file open.  Now issue an AUDIO ioctls to
   * get the capabilities
   */

  caps.ac_len = sizeof(caps);
  caps.ac_type = AUDIO_TYPE_INPUT;
  caps.ac_subtype = AUDIO_TYPE_QUERY;

  ret = lower->ops->getcaps(lower, AUDIO_TYPE_INPUT, &caps);

  if (ret == caps.ac_len)
    {
      samplerate = pcm_samplerate_map(caps.ac_controls.hw[0]);
      if (samplerate == 0)
        {
          auderr("pcm_encode can't translate samplerate flag to value!\n");
          return -ERANGE;
        }
    }

  /* Configure the lower level for the number of channels, bitrate,
   * and sample bitwidth.
   */

  caps.ac_controls.hw[0] = samplerate;
  caps.ac_controls.b[2]  = 16;

#ifdef CONFIG_AUDIO_MULTI_SESSION
  ret = lower->ops->configure(lower, priv->session, &caps);
#else
  ret = lower->ops->configure(lower, &caps);
#endif
  if (ret < 0)
    {
      return ret;
    }
  else
    {
      priv->samprate  = caps.ac_controls.hw[0];
      priv->nchannels = caps.ac_channels;
      priv->bpsamp    = caps.ac_controls.b[2];
      priv->byterate  = priv->samprate * priv->nchannels * priv->bpsamp / 8;
      priv->align     = priv->nchannels * priv->bpsamp / 8;
    }

  return OK;

}


/****************************************************************************
 * Name: pcm_enqueuebuffer
 *
 * Description:
 *   Enqueue a buffer for processing.  This is a non-blocking enqueue
 *   operation.  If the lower-half driver's buffer queue is full, then it
 *   should return an error code of -ENOMEM, and the upper-half driver can
 *   decide to either block the calling thread or deal with it in a non-
 *   blocking manner.
 *
 *   For each call to enqueuebuffer, the lower-half driver must call
 *   audio_dequeuebuffer when it is finished processing the bufferr, passing
 *   the previously enqueued apb and a dequeue status so that the upper-half
 *   driver can decide if a waiting thread needs to be release, if the
 *   dequeued buffer should be passed to the next block in the Audio
 *   Pipeline, etc.
 *
 ****************************************************************************/

static int pcm_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                             FAR struct ap_buffer_s *apb)
{
  FAR struct pcm_encode_s *priv = (FAR struct pcm_encode_s *)dev;
  FAR struct audio_lowerhalf_s *lower;
  apb_samp_t bytesleft;
  int ret;

  DEBUGASSERT(priv);
  audinfo("Received buffer %p, streaming=%d\n", apb, priv->streaming);

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->enqueuebuffer && lower->ops->configure);

  /* Are we streaming yet? */

  if (priv->streaming)
    {
      /* Yes, we are streaming */
      /* Check for the last audio buffer in the stream */

      if ((apb->flags & AUDIO_APB_FINAL) != 0)
        {
          /* Yes.. then we are no longer streaming */

          priv->streaming = false;
        }

      /* Then give the audio buffer to the lower driver */

      audinfo("Pass to lower enqueuebuffer: apb=%p curbyte=%d nbytes=%d\n",
              apb, apb->curbyte, apb->nbytes);

      return lower->ops->enqueuebuffer(lower, apb);
    }

  /* In general, user will config the audio device before using it.
   * However, in some case, user may use the device without configration,
   * so we will help the user choose a default config. Currently, we choose,
   * by default, the highest samplerate among which the device supports.*/
  if (priv->samprate == 0)
    {
      ret = pcm_config_default(priv);
      if (ret != OK)
        {
          auderr("ERROR: Failed to set PCM configuration: %d\n", ret);
          return ret;
        }
    }

  /* No.. then this must be the first buffer that we have seen (since we
   * will error out out if the first buffer is smaller than the WAV file
   * header.  There is no attempt to reconstruct the full header from
   * fragments in multiple, tiny audio buffers).
   */

  bytesleft = apb->nmaxbytes - apb->curbyte;
  audinfo("curbyte=%d nbytes=%d nmaxbytes=%d bytesleft=%d\n",
          apb->curbyte, apb->nbytes, apb->nmaxbytes, bytesleft);

  if (bytesleft >= sizeof(struct wav_header_s))
    {

      pcm_add_wav_header(priv, &apb->samp[apb->curbyte]);

      apb->curbyte += sizeof(struct wav_header_s);

      /* Then give the audio buffer to the lower driver */

      audinfo("Pass to lower enqueuebuffer: apb=%p curbyte=%d nbytes=%d\n",
              apb, apb->curbyte, apb->nbytes);

      ret = lower->ops->enqueuebuffer(lower, apb);
      if (ret == OK)
        {
          /* Now we are streaming.  Unless for some reason there is only
           * one audio buffer in the audio stream.  In that case, this
           * will be marked as the final buffer
           */

          priv->streaming = ((apb->flags & AUDIO_APB_FINAL) == 0);
          return OK;
        }

      auderr("ERROR: Invalid PCM WAV file\n");

      /* The normal protocol for streaming errors is as follows:
       *
       * (1) Fail the enqueueing by returned a negated error value.  The
       *     upper level then knows that this buffer was not queue.
       * (2) Return all queued buffers to the caller using the
       *     AUDIO_CALLBACK_DEQUEUE callback
       * (3) Terminate playing using the AUDIO_CALLBACK_COMPLETE
       *     callback.
       *
       * In this case we fail on the very first buffer and we need only
       * do (1) and (3).
       */

#ifdef CONFIG_AUDIO_MULTI_SESSION
      priv->export.upper(priv->export.priv, AUDIO_CALLBACK_COMPLETE,
                         NULL, OK, NULL);
#else
      priv->export.upper(priv->export.priv, AUDIO_CALLBACK_COMPLETE,
                         NULL, OK);
#endif
    }

  /* This is not a WAV file! */

  auderr("ERROR: Invalid PCM WAV file\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: pcm_cancelbuffer
 *
 * Description:
 *   Cancel a previously enqueued buffer.
 *
 ****************************************************************************/

static int pcm_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                            FAR struct ap_buffer_s *apb)
{
  FAR struct pcm_encode_s *priv = (FAR struct pcm_encode_s *)dev;
  FAR struct audio_lowerhalf_s *lower;

  DEBUGASSERT(priv);

  /* Defer the operation to the lower device driver */

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->cancelbuffer);

  audinfo("Defer to lower cancelbuffer, apb=%p\n", apb);
  return lower->ops->cancelbuffer(lower, apb);
}

/****************************************************************************
 * Name: pcm_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands.
 *
 ****************************************************************************/

static int pcm_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                     unsigned long arg)
{
  FAR struct pcm_encode_s *priv = (FAR struct pcm_encode_s *)dev;
  FAR struct audio_lowerhalf_s *lower;

  DEBUGASSERT(priv);

  /* Defer the operation to the lower device driver */

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->ioctl);

  audinfo("Defer to lower ioctl, cmd=%d arg=%ld\n");
  return lower->ops->ioctl(lower, cmd, arg);
}

/****************************************************************************
 * Name: pcm_reserve
 *
 * Description:
 *   Reserve a session (may only be one per device or may be multiple) for
 *   use by a client.  Client software can open audio devices and issue
 *   AUDIOIOC_GETCAPS calls freely, but other operations require a
 *   reservation.  A session reservation will assign a context that must
 *   be passed with
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int pcm_reserve(FAR struct audio_lowerhalf_s *dev, FAR void **session)
#else
static int pcm_reserve(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct pcm_encode_s *priv = (FAR struct pcm_encode_s *)dev;
  FAR struct audio_lowerhalf_s *lower;
  int ret;

#ifdef CONFIG_AUDIO_MULTI_SESSION
  DEBUGASSERT(priv && session);
#else
  DEBUGASSERT(priv);
#endif

  /* It is not necessary to reserve the upper half.  What we really need to
   * do is to reserved the lower device driver for exclusive use by the PCM
   * encoder.  That effectively reserves the upper PCM encoder along with
   * the lower driver (which is then not available for use by other
   * encoders).
   *
   * We do, however, need to remember the session returned by the lower
   * level.
   */

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->reserve);

  audinfo("Defer to lower reserve\n");
#ifdef CONFIG_AUDIO_MULTI_SESSION
  ret = lower->ops->reserve(lower, &priv->session);

  /* Return a copy of the session to the caller */

  *session = priv->session;

#else
  ret = lower->ops->reserve(lower);
#endif

  return ret;
}

/****************************************************************************
 * Name: pcm_release
 *
 * Description:
 *   Release a session.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int pcm_release(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int pcm_release(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct pcm_encode_s *priv = (FAR struct pcm_encode_s *)dev;
  FAR struct audio_lowerhalf_s *lower;

  DEBUGASSERT(priv);

  /* Release the lower driver.. it is then available for use by other
   * encoders (and we cannot use the lower driver wither unless we re-
   * reserve it).
   */

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->release);

  audinfo("Defer to lower release\n");
#ifdef CONFIG_AUDIO_MULTI_SESSION
  return lower->ops->release(lower, session);
#else
  return lower->ops->release(lower);
#endif
}

/****************************************************************************
 * Name: pcm_callback
 *
 * Description:
 *   Lower-to-upper level callback for buffer dequeueing.
 *
 * Input Parameters:
 *   priv - The value of the 'priv' field from out audio_lowerhalf_s.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static void pcm_callback(FAR void *arg, uint16_t reason,
                         FAR struct ap_buffer_s *apb, uint16_t status,
                         FAR void *session)
#else
static void pcm_callback(FAR void *arg, uint16_t reason,
                         FAR struct ap_buffer_s *apb, uint16_t status)
#endif
{
  FAR struct pcm_encode_s *priv = (FAR struct pcm_encode_s *)arg;

  DEBUGASSERT(priv && priv->export.upper);

  /* The buffer belongs to to an upper level.  Just forward the event to
   * the next level up.
   */

#ifdef CONFIG_AUDIO_MULTI_SESSION
  priv->export.upper(priv->export.priv, reason, apb, status, session);
#else
  priv->export.upper(priv->export.priv, reason, apb, status);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pcm_encode_initialize
 *
 * Description:
 *   Initialize the PCM device.  The PCM device accepts and contains a
 *   low-level audio input device.  It then returns a new audio lower
 *   half interface at adds a PCM encoding from end to the low-level
 *   audio device
 *
 * Input Parameters:
 *   dev - A reference to the low-level audio input device to contain.
 *
 * Returned Value:
 *   On success, a new audio device instance is returned that wraps the
 *   low-level device and provides a PCM encoding front end.  NULL is
 *   returned on failure.
 *
 ****************************************************************************/

FAR struct audio_lowerhalf_s *
pcm_encode_initialize(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct pcm_encode_s *priv;
  FAR struct audio_ops_s *ops;

  DEBUGASSERT(dev);

  /* Allocate an instance of our private data structure */

  priv = (FAR struct pcm_encode_s *)kmm_zalloc(sizeof(struct pcm_encode_s));
  if (!priv)
    {
      auderr("ERROR: Failed to allocate driver structure\n");
      return NULL;
    }

  /* Initialize our private data structure.  Since kmm_zalloc() was used for
   * the allocation, we need to initialize only non-zero, non-NULL, non-
   * false fields.
   */

  /* Setup our operations */

  ops                  = &priv->ops;
  ops->getcaps         = pcm_getcaps;
  ops->configure       = pcm_configure;
  ops->shutdown        = pcm_shutdown;
  ops->start           = pcm_start;

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  ops->stop            = pcm_stop;
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  ops->pause           = pcm_pause;
  ops->resume          = pcm_resume;
#endif

  if (dev->ops->allocbuffer)
    {
      DEBUGASSERT(dev->ops->freebuffer);
      ops->allocbuffer = pcm_allocbuffer;
      ops->freebuffer  = pcm_freebuffer;
    }

  ops->enqueuebuffer   = pcm_enqueuebuffer;
  ops->cancelbuffer    = pcm_cancelbuffer;
  ops->ioctl           = pcm_ioctl;
  ops->reserve         = pcm_reserve;
  ops->release         = pcm_release;

  /* Set up our struct audio_lower_half that we will register with the
   * system.  The registration process will fill in the priv->export.upper
   * and priv fields with the correct callback information.
   */

  priv->export.ops     = &priv->ops;

  /* Save the struct audio_lower_half of the low-level audio device.  Set
   * out callback information for the lower-level audio device.  Our
   * callback will simply forward to the upper callback.
   */

  priv->lower          = dev;
  dev->upper           = pcm_callback;
  dev->priv            = priv;

  return &priv->export;
}

#endif /* CONFIG_AUDIO && CONFIG_AUDIO_FORMAT_PCM */

