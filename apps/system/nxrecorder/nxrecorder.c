/****************************************************************************
 * apps/system/nxrecorder/nxrecorder.c
 *
 * Developed by:
 *
 *   Copyright (C) 2018  zGlue INC All rights reserved.
 *   Author: Zhengwei Wang <zhengwei@zglue.com>
 *
 * References:
 *
 * -  The framework for this app is based on Ken Pettit's nxplayer.
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
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <dirent.h>
#include <debug.h>

#include <nuttx/audio/audio.h>
#include "system/nxrecorder.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_AUDIO_EXCLUDE_STOP
#  error CONFIG_AUDIO_EXCLUDE_STOP must be enabled if you want to use this app
#endif

#define NXRECORDER_STATE_IDLE      0
#define NXRECORDER_STATE_PLAYING   1
#define NXRECORDER_STATE_PAUSED    2

#ifndef CONFIG_AUDIO_NUM_BUFFERS
#  define CONFIG_AUDIO_NUM_BUFFERS  4
#endif

#ifndef CONFIG_AUDIO_BUFFER_NUMBYTES
#  define CONFIG_AUDIO_BUFFER_NUMBYTES  1024
#endif

#ifndef CONFIG_NXRECORDER_MSG_PRIO
#  define CONFIG_NXRECORDER_MSG_PRIO  1
#endif

#ifndef CONFIG_NXRECORDER_PLAYTHREAD_STACKSIZE
#  define CONFIG_NXRECORDER_PLAYTHREAD_STACKSIZE    1500
#endif

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

#ifdef CONFIG_NXRECORDER_FMT_FROM_EXT
struct nxrecorder_ext_fmt_s
{
  const char  *ext;
  uint16_t    format;
  CODE int    (*getsubformat)(FAR FILE *fd);
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_AUDIO_FORMAT_MIDI
int nxrecorder_getmidisubformat(FAR FILE *fd);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_NXRECORDER_FMT_FROM_EXT
static const struct nxrecorder_ext_fmt_s g_known_ext[] =
{
#ifdef CONFIG_AUDIO_FORMAT_AC3
  { "ac3",      AUDIO_FMT_AC3, NULL },
#endif
#ifdef CONFIG_AUDIO_FORMAT_MP3
  { "mp3",      AUDIO_FMT_MP3, NULL },
#endif
#ifdef CONFIG_AUDIO_FORMAT_DTS
  { "dts",      AUDIO_FMT_DTS, NULL },
#endif
#ifdef CONFIG_AUDIO_FORMAT_WMA
  { "wma",      AUDIO_FMT_WMA, NULL },
#endif
#ifdef CONFIG_AUDIO_FORMAT_PCM
  { "wav",      AUDIO_FMT_PCM, NULL },
#endif
#ifdef CONFIG_AUDIO_FORMAT_MIDI
  { "mid",      AUDIO_FMT_MIDI, nxrecorder_getmidisubformat },
  { "midi",     AUDIO_FMT_MIDI, nxrecorder_getmidisubformat },
#endif
#ifdef CONFIG_AUDIO_FORMAT_OGG_VORBIS
  { "ogg",      AUDIO_FMT_OGG_VORBIS, NULL }
#endif
};
static const int g_known_ext_count = sizeof(g_known_ext) /
                                     sizeof(struct nxrecorder_ext_fmt_s);
#endif /* CONFIG_NXRECORDER_FMT_FROM_EXT */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxrecorder_opendevice
 *
 *   nxrecorder_opendevice() either searches the Audio system for a device
 *   that is compatible with the specified audio format and opens it, or
 *   tries to open the prefered device if specified and validates that
 *   it supports the requested format.
 *
 * Return:
 *    OK        if compatible device opened (searched or preferred)
 *    -ENODEV   if no compatible device opened.
 *    -ENOENT   if preferred device couldn't be opened.
 *
 ****************************************************************************/

static int nxrecorder_opendevice(FAR struct nxrecorder_s *pRecorder,
                                 int format,
                                 int subfmt)
{
  /* If we have a preferred device, then open it */

#ifdef CONFIG_NXRECORDER_INCLUDE_PREFERRED_DEVICE
  if (pRecorder->prefdevice[0] != '\0')
    {
      /* Use the saved prefformat to test if the requested
       * format is specified by the device
       */

      if ((pRecorder->prefformat & (1 << (format - 1))) == 0 ||
          (pRecorder->preftype & AUDIO_TYPE_INPUT) == 0)
        {
          /* Format not supported by the device */

          auderr("ERROR: Format not supported by device: %d\n", format);
          return -ENODEV;
        }

      /* Device supports the format.  Open the device file. */

      pRecorder->devFd = open(pRecorder->prefdevice, O_RDWR);
      if (pRecorder->devFd == -1)
        {
          int errcode = errno;
          DEBUGASSERT(errcode > 0);

          auderr("ERROR: Failed to open %s: %d\n", -errcode);
          UNUSED(errcode);
          return -ENOENT;
        }

      return OK;
    }
#endif

#if defined(CONFIG_NXRECORDER_INCLUDE_PREFERRED_DEVICE) && \
    defined(CONFIG_NXRECORDER_INCLUDE_DEVICE_SEARCH)

  else

#endif

#ifdef CONFIG_NXRECORDER_INCLUDE_DEVICE_SEARCH
    {
      struct audio_caps_s caps;
      FAR struct dirent *pDevice;
      FAR DIR *dirp;
      char path[64];
      uint8_t supported = true;
      uint8_t x;

      /* Search for a device in the audio device directory */

#ifdef CONFIG_AUDIO_CUSTOM_DEV_PATH
#ifdef CONFIG_AUDIO_DEV_ROOT
      dirp = opendir("/dev");
#else
      dirp = opendir(CONFIG_AUDIO_DEV_PATH);
#endif  /* CONFIG_AUDIO_DEV_ROOT */
#else
      dirp = opendir("/dev/audio");
#endif  /* CONFIG_AUDIO_CUSTOM_DEV_PATH */
      if (dirp == NULL)
        {
          int errcode = errno;
          DEBUGASSERT(errcode > 0);

          auderr("ERROR: Failed to open /dev/audio: %d\n", -errcode);
          UNUSED(errcode);
          return -ENODEV;
        }

      while ((pDevice = readdir(dirp)) != NULL)
        {
          /* We found the next device.  Try to open it and
           * get its audio capabilities.
           */

#ifdef CONFIG_AUDIO_CUSTOM_DEV_PATH
#ifdef CONFIG_AUDIO_DEV_ROOT
          snprintf(path,  sizeof(path), "/dev/%s", pDevice->d_name);
#else
          snprintf(path,  sizeof(path), CONFIG_AUDIO_DEV_PATH "/%s", pDevice->d_name);
#endif  /* CONFIG_AUDIO_DEV_ROOT */
#else
          snprintf(path,  sizeof(path), "/dev/audio/%s", pDevice->d_name);
#endif  /* CONFIG_AUDIO_CUSTOM_DEV_PATH */

          if ((pRecorder->devFd = open(path, O_RDWR)) != -1)
            {
              /* We have the device file open.  Now issue an AUDIO ioctls to
               * get the capabilities
               */

              caps.ac_len = sizeof(caps);
              caps.ac_type = AUDIO_TYPE_QUERY;
              caps.ac_subtype = AUDIO_TYPE_QUERY;

              if (ioctl(pRecorder->devFd, AUDIOIOC_GETCAPS, (unsigned long) &caps)
                  == caps.ac_len)
                {
                  /* Test if this device supports the format we want */

                  if (((caps.ac_format.hw & (1 << (format - 1))) != 0) &&
                      (caps.ac_controls.b[0] & AUDIO_TYPE_INPUT))
                    {
                      /* Do subformat detection */

                      if (subfmt != AUDIO_FMT_UNDEF)
                        {
                          /* Prepare to get sub-formats for this main format */

                          caps.ac_subtype = format;
                          caps.ac_format.b[0] = 0;

                          while (ioctl(pRecorder->devFd, AUDIOIOC_GETCAPS,
                                       (unsigned long) &caps) == caps.ac_len)
                            {
                              /* Check the next set of 4 controls to find the subformat */

                              for (x = 0; x < sizeof(caps.ac_controls); x++)
                                {
                                  if (caps.ac_controls.b[x] == subfmt)
                                    {
                                      /* Sub format supported! */

                                      break;
                                    }
                                  else if (caps.ac_controls.b[x] == AUDIO_SUBFMT_END)
                                    {
                                      /* Sub format not supported */

                                      supported = false;
                                      break;
                                    }
                                }

                              /* If we reached the end of the subformat list, then
                               * break out of the loop.
                               */

                              if (x != sizeof(caps.ac_controls))
                                {
                                  break;
                                }

                              /* Increment ac_format.b[0] to get next set of subformats */

                              caps.ac_format.b[0]++;
                            }
                        }

                      /* Test if subformat needed and detected */

                      if (supported)
                        {
                          /* Yes, it supports this format.  Use this device */

                          closedir(dirp);
                          return OK;
                        }
                    }
                }

              /* Not this device! */

              close(pRecorder->devFd);
            }
        }

      /* Close the directory */

      closedir(dirp);
    }
#endif  /* CONFIG_NXRECORDER_INCLUDE_DEVICE_SEARCH */

  /* Device not found */

  auderr("ERROR: Device not found\n");
  pRecorder->devFd = -1;
  return -ENODEV;
}

/****************************************************************************
 * Name: nxrecorder_getmidisubformat
 *
 *   nxrecorder_getmidisubformat() reads the MIDI header and determins the
 *   MIDI format of the file.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_FORMAT_MIDI
int nxrecorder_getmidisubformat(FAR FILE *fd)
{
  char    type[2];
  int     ret;

  /* Seek to location 8 in the file (the format type) */

  fseek(fd, 8, SEEK_SET);
  fread(type, 1, 2, fd);

  /* Set return value based on type */

  switch (type[1])
    {
      case 0:
        ret = AUDIO_SUBFMT_MIDI_0;
        break;

      case 1:
        ret = AUDIO_SUBFMT_MIDI_1;
        break;

      case 2:
        ret = AUDIO_SUBFMT_MIDI_2;
        break;
    }
  fseek(fd, 0, SEEK_SET);

  return ret;
}
#endif

/****************************************************************************
 * Name: nxrecorder_fmtfromextension
 *
 *   nxrecorder_fmtfromextension() tries to determine the file format based
 *   on the extension of the supplied filename.
 *
 ****************************************************************************/

#ifdef CONFIG_NXRECORDER_FMT_FROM_EXT
static inline int nxrecorder_fmtfromextension(FAR struct nxrecorder_s *pRecorder,
                                              FAR const char *pFilename,
                                              FAR int *subfmt)
{
  const char  *pExt;
  uint8_t      x;
  uint8_t      c;

  /* Find the file extension, if any */

  x = strlen(pFilename) - 1;
  while (x > 0)
    {
      /* Seach backward for the first '.' */

      if (pFilename[x] == '.')
        {
          /* First '.' found.  Now compare with known extensions */

          pExt = &pFilename[x + 1];
          for (c = 0; c < g_known_ext_count; c++)
            {
              /* Test for extension match */

              if (strcasecmp(pExt, g_known_ext[c].ext) == 0)
                {
                  /* Test if we have a sub-format detection routine */

                  if (subfmt && g_known_ext[c].getsubformat)
                    {
                      *subfmt = g_known_ext[c].getsubformat(pRecorder->fileFd);
                    }

                  /* Return the format for this extension */

                  return g_known_ext[c].format;
                }
            }
        }

      /* Stop if we find a '/' */

      if (pFilename[x] == '/')
        {
          break;
        }

      x--;
    }

  return AUDIO_FMT_UNDEF;
}
#endif  /* CONFIG_NXRECORDER_FMT_FROM_EXT */

/****************************************************************************
 * Name: nxrecorder_mediasearch
 *
 *   nxrecorder_mediasearch() searches the subdirectories in the mediadir
 *   for the specified media file.  We borrow the caller's path stack
 *   variable (playfile) to conserve stack space.
 *
 ****************************************************************************/

#if defined(CONFIG_NXRECORDER_MEDIA_SEARCH) && defined(CONFIG_NXRECORDER_INCLUDE_MEDIADIR)
static int nxrecorder_mediasearch(FAR struct nxrecorder_s *pRecorder,
                                  FAR const char *pFilename,
                                  FAR const char *path, int pathmax)
{
  return -ENOENT;
}
#endif

/****************************************************************************
 * Name: nxrecorder_writebuffer
 *
 *  Read the next block of data from the media file into the specified
 *  buffer.
 *
 ****************************************************************************/

static int nxrecorder_writebuffer(FAR struct nxrecorder_s *pRecorder,
                                  FAR struct ap_buffer_s *apb)
{

  int write_len;

  /* Validate the file is still open.  It will be closed automatically when
   * we encounter the end of file (or, perhaps, a read error that we cannot
   * handle.
   */

  if (pRecorder->fileFd == NULL)
    {
      /* Return -ENODATA to indicate that there is nothing more to read from
       * the file.
       */

      return -ENODATA;
    }



  /* write the buffer to the medie file */

  write_len = fwrite(&apb->samp, 1, apb->nbytes, pRecorder->fileFd);

  if (write_len < apb->nbytes)
    {
#ifdef CONFIG_DEBUG_AUDIO_INFO
      int writeerror = ferror(pRecorder->fileFd);

      audinfo("Closing audio file, nbytes=%d writeerr=%d\n",
              write_len, writeerror);
#endif

      /* write error.. We are finished with this file in any
       * event.
       */

      fclose(pRecorder->fileFd);
      pRecorder->fileFd = NULL;

    }
  else
    {
      audinfo("write buffer: apb:%x length:%d\r\n", apb, apb->nbytes);
    }


  apb->nbytes = 0;
  apb->curbyte = 0;

  return OK;
}


/****************************************************************************
 * Name: nxrecorder_enqueuebuffer
 *
 * Description:
 *   Enqueue the audio buffer in the downstream device.  Normally we are
 *   called with a buffer of data to be enqueued in the audio stream.
 *
 *   Be we may also receive an empty length buffer (with only the
 *   AUDIO_APB_FINAL set) in the event of certin read error occurs or in the
 *   event that the file was an exact multiple of the nmaxbytes size of the
 *   audio buffer.  In that latter case, we have an end of file with no bytes
 *   read.
 *
 *   These infrequent zero length buffers have to be passed through because
 *   the include the AUDIO_APB_FINAL flag that is needed to terminate the
 *   audio stream.
 *
 ****************************************************************************/

static int nxrecorder_enqueuebuffer(FAR struct nxrecorder_s *pRecorder,
                                    FAR struct ap_buffer_s *apb)
{
  struct audio_buf_desc_s bufdesc;
  int ret;

  /* Now enqueue the buffer with the audio device.  If the number of
   * bytes in the file happens to be an exact multiple of the audio
   * buffer size, then we will receive the last buffer size = 0.  We
   * encode this buffer also so the audio system knows its the end of
   * the file and can do proper clean-up.
   */

#ifdef CONFIG_AUDIO_MULTI_SESSION
  bufdesc.session   = pRecorder->session;
#endif
  bufdesc.numbytes  = apb->nbytes;
  bufdesc.u.pBuffer = apb;

  ret = ioctl(pRecorder->devFd, AUDIOIOC_ENQUEUEBUFFER,
              (unsigned long)&bufdesc);
  if (ret < 0)
    {
      int errcode = errno;
      DEBUGASSERT(errcode > 0);

      auderr("ERROR: AUDIOIOC_ENQUEUEBUFFER ioctl failed: %d\n", errcode);
      return -errcode;
    }

  /* Return OK to indicate that we successfully read data from the file
   * (and we are not yet at the end of file)
   */

  return OK;
}

/****************************************************************************
 * Name: nxrecorder_thread_playthread
 *
 *  This is the thread that reads the audio file file and enqueues /
 *  dequeues buffers to the selected and opened audio device.
 *
 ****************************************************************************/

static void *nxrecorder_recordthread(pthread_addr_t pvarg)
{
  struct nxrecorder_s           *pRecorder = (struct nxrecorder_s *) pvarg;
  struct audio_msg_s          msg;
  struct audio_buf_desc_s     buf_desc;
  ssize_t                     size;
  bool                        running = true;
  bool                        streaming = true;
  bool                        failed = false;
#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
  struct ap_buffer_info_s     buf_info;
  FAR struct ap_buffer_s    **pBuffers;
#else
  FAR struct ap_buffer_s     *pBuffers[CONFIG_AUDIO_NUM_BUFFERS];
#endif
#ifdef CONFIG_DEBUG_FEATURES
  int                         outstanding = 0;
#endif
  int                         prio;
  int                         x;
  int                         ret;

  audinfo("Entry\n");

  /* Query the audio device for it's preferred buffer size / qty */

#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
  if ((ret = ioctl(pRecorder->devFd, AUDIOIOC_GETBUFFERINFO,
                   (unsigned long) &buf_info)) != OK)
    {
      /* Driver doesn't report it's buffer size.  Use our default. */

      buf_info.buffer_size = CONFIG_AUDIO_BUFFER_NUMBYTES;
      buf_info.nbuffers = CONFIG_AUDIO_NUM_BUFFERS;
    }

  /* Create array of pointers to buffers */

  pBuffers = (FAR struct ap_buffer_s **) malloc(buf_info.nbuffers * sizeof(FAR void *));
  if (pBuffers == NULL)
    {
      /* Error allocating memory for buffer storage! */

      ret = -ENOMEM;
      running = false;
      goto err_out;
    }

  /* Create our audio pipeline buffers to use for queueing up data */

  for (x = 0; x < buf_info.nbuffers; x++)
    {
      pBuffers[x] = NULL;
    }

  for (x = 0; x < buf_info.nbuffers; x++)
#else /* CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFER */

  for (x = 0; x < CONFIG_AUDIO_NUM_BUFFERS; x++)
    {
      pBuffers[x] = NULL;
    }

  for (x = 0; x < CONFIG_AUDIO_NUM_BUFFERS; x++)
#endif /* CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFER */
    {
      /* Fill in the buffer descriptor struct to issue an alloc request */

#ifdef CONFIG_AUDIO_MULTI_SESSION
      buf_desc.session = pRecorder->session;
#endif
#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
      buf_desc.numbytes = buf_info.buffer_size;
#else
      buf_desc.numbytes = CONFIG_AUDIO_BUFFER_NUMBYTES;
#endif
      buf_desc.u.ppBuffer = &pBuffers[x];

      ret = ioctl(pRecorder->devFd, AUDIOIOC_ALLOCBUFFER,
                  (unsigned long) &buf_desc);
      if (ret != sizeof(buf_desc))
        {
          /* Buffer alloc Operation not supported or error allocating! */

          auderr("ERROR: Could not allocate buffer %d\n", x);
          running = false;
          goto err_out;
        }
    }


  /* Fill up the pipeline with enqueued buffers */

#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
  for (x = 0; x < buf_info.nbuffers; x++)
#else
  for (x = 0; x < CONFIG_AUDIO_NUM_BUFFERS; x++)
#endif
    {

      ret = nxrecorder_enqueuebuffer(pRecorder, pBuffers[x]);
      if (ret != OK)
        {
          /* Failed to enqueue the buffer.  The driver is not happy with
           * the buffer.  Perhaps a decoder has detected something that it
           * does not like in the stream and has stopped streaming.  This
           * would happen normally if we send a file in the incorrect format
           * to an audio decoder.
           *
           * We must stop streaming as gracefully as possible.  Close the
           * file so that no further data is read.
           */

          fclose(pRecorder->fileFd);
          pRecorder->fileFd = NULL;

          /* We are no longer streaming data from the file.  Be we will
           * need to wait for any outstanding buffers to be recovered.  We
           * also still expect the audio driver to send a AUDIO_MSG_COMPLETE
           * message after all queued buffers have been returned.
           */

          streaming = false;
          failed = true;
          break;
        }
#ifdef CONFIG_DEBUG_FEATURES
      else
        {
          /* The audio driver has one more buffer */

          outstanding++;
        }
#endif
    }

  audinfo("%d buffers queued, running=%d streaming=%d\n",
          x, running, streaming);

  if (running && !failed)
    {
      /* Indicate we are playing a file */

      pRecorder->state = NXRECORDER_STATE_PLAYING;

      /* Set initial parameters such as volume, bass, etc.
       * REVISIT:  Shouldn't this actually be done BEFORE we start playing?
       */

#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
      (void)nxrecorder_setvolume(pRecorder, pRecorder->volume);
#ifndef CONFIG_AUDIO_EXCLUDE_BALANCE
      nxrecorder_setbalance(pRecorder, pRecorder->balance);
#endif
#endif


#ifndef CONFIG_AUDIO_EXCLUDE_TONE
      nxrecorder_setbass(pRecorder, pRecorder->bass);
      nxrecorder_settreble(pRecorder, pRecorder->treble);
#endif

      /* Start the audio device */

#ifdef CONFIG_AUDIO_MULTI_SESSION
      ret = ioctl(pRecorder->devFd, AUDIOIOC_START,
                  (unsigned long) pRecorder->session);
#else
      ret = ioctl(pRecorder->devFd, AUDIOIOC_START, 0);
#endif

      if (ret < 0)
        {
          /* Error starting the audio stream!  We need to continue running
           * in order to recover the audio buffers that have already been
           * queued.
           */

          failed = true;
        }
    }


  /* Loop until we specifically break.  running == true means that we are
   * still recording. The recording is not complete until we get the
   * AUDIO_MSG_COMPLETE (or AUDIO_MSG_STOP) message.
   *
   * The normal protocol for streaming errors detected by the audio driver
   * is as follows:
   *
   * (1) The audio driver will indicated the error by returning a negated
   *     error value when the next buffer is enqueued.  The upper level
   *     then knows that this buffer was not queue.
   * (2) The audio driver must return all queued buffers using the
   *     AUDIO_MSG_DEQUEUE message, and
   * (3) Terminate playing by sending the AUDIO_MSG_COMPLETE message.
   */

  audinfo("%s\n", running ? "Playing..." : "Not runnning");
  while (running)
    {
      /* Wait for a signal either from the Audio driver that it needs
       * additional buffer data, or from a user-space signal to pause,
       * stop, etc.
       */

      size = mq_receive(pRecorder->mq, (FAR char *)&msg, sizeof(msg), &prio);

      /* Validate a message was received */

      if (size != sizeof(msg))
        {
          /* Interrupted by a signal? What to do? */

          continue;
        }

      /* Perform operation based on message id */

      switch (msg.msgId)
        {
          /* An audio buffer is being dequeued by the driver */

          case AUDIO_MSG_DEQUEUE:
#ifdef CONFIG_DEBUG_FEATURES
            /* Make sure that we believe that the audio driver has at
             * least one buffer.
             */

            DEBUGASSERT(msg.u.pPtr && outstanding > 0);
            outstanding--;
#endif

            /* write the buffer we received to media file*/
            ret = nxrecorder_writebuffer(pRecorder, msg.u.pPtr);
            if (ret != OK)
              {
                /* Out of data.  Stay in the loop until the device sends
                 * us a COMPLETE message, but stop trying to play more
                 * data.
                 */

                streaming = false;
                failed = true;
              }

            /* Requeue this buffer to audio device.
             * streaming == true means that we can keep recording.
             */

            if (streaming)
              {
                /* Enqueue buffer by sending it to the audio driver */

                ret = nxrecorder_enqueuebuffer(pRecorder, msg.u.pPtr);
                if (ret != OK)
                  {
                    /* There is some issue from the audio driver.
                     * Perhaps a problem in the file format?
                     *
                     * We must stop streaming as gracefully as possible.
                     * Close the file so that no further data is read.
                     */

                    fclose(pRecorder->fileFd);
                    pRecorder->fileFd = NULL;

                    /* Stop streaming and wait for buffers to be
                     * returned and to receive the AUDIO_MSG_COMPLETE
                     * indication.
                     */

                    streaming = false;
                    failed = true;
                  }
#ifdef CONFIG_DEBUG_FEATURES
                else
                  {
                    /* The audio driver has one more buffer */

                    outstanding++;
                  }
#endif
              }
            break;

          /* Someone wants to stop the playback. */

          case AUDIO_MSG_STOP:
            /* Send a stop message to the device */

            audinfo("Stopping! outstanding=%d\n", outstanding);

#ifdef CONFIG_AUDIO_MULTI_SESSION
            ioctl(pRecorder->devFd, AUDIOIOC_STOP,
                  (unsigned long) pRecorder->session);
#else
            ioctl(pRecorder->devFd, AUDIOIOC_STOP, 0);
#endif
            /* Stay in the running loop (without sending more data).
             * we will need to recover our audio buffers.  We will
             * loop until AUDIO_MSG_COMPLETE is received.
             */

            streaming = false;
            break;

          /* Message indicating the playback is complete */

          case AUDIO_MSG_COMPLETE:
            audinfo("Play complete.  outstanding=%d\n", outstanding);
            DEBUGASSERT(outstanding == 0);
            running = false;
            break;

          /* Unknown / unsupported message ID */

          default:
            break;
        }
    }

  /* Release our audio buffers and unregister / release the device */

err_out:
  audinfo("Clean-up and exit\n");

  /* Unregister the message queue and release the session */

  ioctl(pRecorder->devFd, AUDIOIOC_UNREGISTERMQ, (unsigned long) pRecorder->mq);
#ifdef CONFIG_AUDIO_MULTI_SESSION
  ioctl(pRecorder->devFd, AUDIOIOC_RELEASE, (unsigned long) pRecorder->session);
#else
  ioctl(pRecorder->devFd, AUDIOIOC_RELEASE, 0);
#endif

  /* Cleanup */

  while (sem_wait(&pRecorder->sem) < 0)
    ;

#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
  if (pBuffers != NULL)
    {
      audinfo("Freeing buffers\n");
      for (x = 0; x < buf_info.nbuffers; x++)
        {
          /* Fill in the buffer descriptor struct to issue a free request */

          if (pBuffers[x] != NULL)
            {
              buf_desc.u.pBuffer = pBuffers[x];
              ioctl(pRecorder->devFd, AUDIOIOC_FREEBUFFER, (unsigned long) &buf_desc);
            }
        }

      /* Free the pointers to the buffers */

      free(pBuffers);
    }
#else
  audinfo("Freeing buffers\n");
  for (x = 0; x < CONFIG_AUDIO_NUM_BUFFERS; x++)
    {
      /* Fill in the buffer descriptor struct to issue a free request */

      if (pBuffers[x] != NULL)
        {
          buf_desc.u.pBuffer = pBuffers[x];
          ioctl(pRecorder->devFd, AUDIOIOC_FREEBUFFER, (unsigned long) &buf_desc);
        }
    }
#endif

  /* Close the files */

  if (pRecorder->fileFd != NULL)
    {
      fclose(pRecorder->fileFd);            /* Close the file */
      pRecorder->fileFd = NULL;             /* Clear out the FD */
    }

  close(pRecorder->devFd);                  /* Close the device */
  pRecorder->devFd = -1;                    /* Mark device as closed */
  mq_close(pRecorder->mq);                  /* Close the message queue */
  mq_unlink(pRecorder->mqname);             /* Unlink the message queue */
  pRecorder->state = NXRECORDER_STATE_IDLE;   /* Go to IDLE */

  sem_post(&pRecorder->sem);                /* Release the semaphore */

  /* The playthread is done with the context.  Release it, which may
   * actually cause the context to be freed if the creator has already
   * abandoned (released) the context too.
   */

  nxrecorder_release(pRecorder);

  audinfo("Exit\n");

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxrecorder_setvolume
 *
 *   nxrecorder_setvolume() sets the volume.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
int nxrecorder_setvolume(FAR struct nxrecorder_s *pRecorder, uint16_t volume)
{
  struct audio_caps_desc_s  cap_desc;
  int ret;

  /* Thread sync using the semaphore */

  while (sem_wait(&pRecorder->sem) < 0)
    ;

  /* If we are currently playing, then we need to post a message to
   * the playthread to perform the volume change operation.  If we
   * are not playing, then just store the volume setting and it will
   * be applied before the next playback begins.
   */

  if (pRecorder->state == NXRECORDER_STATE_PLAYING)
    {
      /* Send a CONFIGURE ioctl to the device to set the volume */

#ifdef CONFIG_AUDIO_MULTI_SESSION
      cap_desc.session = pRecorder->session;
#endif
      cap_desc.caps.ac_len            = sizeof(struct audio_caps_s);
      cap_desc.caps.ac_type           = AUDIO_TYPE_FEATURE;
      cap_desc.caps.ac_format.hw      = AUDIO_FU_VOLUME;
      cap_desc.caps.ac_controls.hw[0] = volume;
      ret = ioctl(pRecorder->devFd, AUDIOIOC_CONFIGURE, (unsigned long) &cap_desc);
      if (ret < 0)
        {
          int errcode = errno;
          DEBUGASSERT(errcode > 0);

          auderr("ERROR: AUDIOIOC_CONFIGURE ioctl failed: %d\n", errcode);
          return -errcode;
        }
    }

  /* Store the volume setting */

  pRecorder->volume = volume;
  sem_post(&pRecorder->sem);

  return OK;
}
#endif  /* CONFIG_AUDIO_EXCLUDE_VOLUME */

/****************************************************************************
 * Name: nxrecorder_setequalization
 *
 *   Sets the level on each band of an equalizer.  Each band setting is
 *   represented in one percent increments, so the range is 0-100.
 *
 * Input Parameters:
 *   pRecorder      - Pointer to the context to initialize
 *   equalization - Pointer to array of equalizer settings of size
 *                  CONFIG_AUDIO_EQUALIZER_NBANDS bytes.  Each byte
 *                  represents the setting for one band in the range of
 *                  0-100.
 *
 * Returned Value:
 *   OK if equalization was set correctly.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_EQUALIZER
int nxrecorder_setequalization(FAR struct nxrecorder_s *pRecorder,
                               FAR uint8_t *equalization)
{
#warning Missing logic
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: nxrecorder_setbass
 *
 *   nxrecorder_setbass() sets the bass level and range.
 *
 * Input:
 *   pRecorder  - Pointer to the nxrecorder context
 *   level    - Bass level in percentage (0-100)
 *   range    - Bass range in percentage (0-100)
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_TONE
int nxrecorder_setbass(FAR struct nxrecorder_s *pRecorder, uint8_t level)
{
  struct audio_caps_desc_s  cap_desc;

  /* Thread sync using the semaphore */

  while (sem_wait(&pRecorder->sem) < 0)
    ;

  /* If we are currently playing, then we need to post a message to
   * the playthread to perform the volume change operation.  If we
   * are not playing, then just store the bass setting and it will
   * be applied before the next playback begins.
   */

  if (pRecorder->state == NXRECORDER_STATE_PLAYING)
    {
      /* Send a CONFIGURE ioctl to the device to set the volume */

#ifdef CONFIG_AUDIO_MULTI_SESSION
      cap_desc.session = pRecorder->session;
#endif
      cap_desc.caps.ac_len           = sizeof(struct audio_caps_s);
      cap_desc.caps.ac_type          = AUDIO_TYPE_FEATURE;
      cap_desc.caps.ac_format.hw     = AUDIO_FU_BASS;
      cap_desc.caps.ac_controls.b[0] = level;
      ioctl(pRecorder->devFd, AUDIOIOC_CONFIGURE, (unsigned long) &cap_desc);
    }

  /* Store the volume setting */

  pRecorder->bass = level;

  sem_post(&pRecorder->sem);

  return -ENOENT;
}
#endif /* CONFIG_AUDIO_EXCLUDE_TONE */

/****************************************************************************
 * Name: nxrecorder_settreble
 *
 *   nxrecorder_settreble() sets the treble level and range.
 *
 * Input:
 *   pRecorder  - Pointer to the nxrecorder context
 *   level    - Treble level in percentage (0-100)
 *   range    - Treble range in percentage (0-100)
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_TONE
int nxrecorder_settreble(FAR struct nxrecorder_s *pRecorder, uint8_t level)
{
  struct audio_caps_desc_s  cap_desc;

  /* Thread sync using the semaphore */

  while (sem_wait(&pRecorder->sem) < 0)
    ;

  /* If we are currently playing, then we need to post a message to
   * the playthread to perform the volume change operation.  If we
   * are not playing, then just store the treble setting and it will
   * be applied before the next playback begins.
   */

  if (pRecorder->state == NXRECORDER_STATE_PLAYING)
    {
      /* Send a CONFIGURE ioctl to the device to set the volume */

#ifdef CONFIG_AUDIO_MULTI_SESSION
      cap_desc.session = pRecorder->session;
#endif
      cap_desc.caps.ac_len           = sizeof(struct audio_caps_s);
      cap_desc.caps.ac_type          = AUDIO_TYPE_FEATURE;
      cap_desc.caps.ac_format.hw     = AUDIO_FU_TREBLE;
      cap_desc.caps.ac_controls.b[0] = level;
      ioctl(pRecorder->devFd, AUDIOIOC_CONFIGURE, (unsigned long) &cap_desc);
    }

  /* Store the volume setting */

  pRecorder->treble = level;

  sem_post(&pRecorder->sem);

  return -ENOENT;
}
#endif /* CONFIG_AUDIO_EXCLUDE_TONE */

/****************************************************************************
 * Name: nxrecorder_setbalance
 *
 *   nxrecorder_setbalance() sets the volume.
 *
 ****************************************************************************/
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
#ifndef CONFIG_AUDIO_EXCLUDE_BALANCE
int nxrecorder_setbalance(FAR struct nxrecorder_s *pRecorder, uint16_t balance)
{
  struct audio_caps_desc_s cap_desc;

  /* Thread sync using the semaphore */

  while (sem_wait(&pRecorder->sem) < 0)
    ;

  /* If we are currently playing, then we need to post a message to
   * the playthread to perform the volume change operation.  If we
   * are not playing, then just store the volume setting and it will
   * be applied before the next playback begins.
   */

  if (pRecorder->state == NXRECORDER_STATE_PLAYING)
    {
      /* Send a CONFIGURE ioctl to the device to set the volume */

#ifdef CONFIG_AUDIO_MULTI_SESSION
      cap_desc.session                = pRecorder->session;
#endif
      cap_desc.caps.ac_len            = sizeof(struct audio_caps_s);
      cap_desc.caps.ac_type           = AUDIO_TYPE_FEATURE;
      cap_desc.caps.ac_format.hw      = AUDIO_FU_BALANCE;
      cap_desc.caps.ac_controls.hw[0] = balance;
      ioctl(pRecorder->devFd, AUDIOIOC_CONFIGURE, (unsigned long) &cap_desc);
    }

  /* Store the volume setting */

  pRecorder->balance = balance;

  sem_post(&pRecorder->sem);

  return -ENOENT;
}
#endif
#endif

/****************************************************************************
 * Name: nxrecorder_pause
 *
 *   nxrecorder_pause() pauses playback without cancelling it.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
int nxrecorder_pause(FAR struct nxrecorder_s *pRecorder)
{
  int   ret = OK;

  if (pRecorder->state == NXRECORDER_STATE_PLAYING)
    {
#ifdef CONFIG_AUDIO_MULTI_SESSION
      ret = ioctl(pRecorder->devFd, AUDIOIOC_PAUSE,
                  (unsigned long) pRecorder->session);
#else
      ret = ioctl(pRecorder->devFd, AUDIOIOC_PAUSE, 0);
#endif
      if (ret == OK)
        {
          pRecorder->state = NXRECORDER_STATE_PAUSED;
        }
    }

  return ret;
}
#endif  /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: nxrecorder_resume
 *
 *   nxrecorder_resume() resumes playback after a pause operation.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
int nxrecorder_resume(FAR struct nxrecorder_s *pRecorder)
{
  int ret = OK;

  if (pRecorder->state == NXRECORDER_STATE_PAUSED)
    {
#ifdef CONFIG_AUDIO_MULTI_SESSION
      ret = ioctl(pRecorder->devFd, AUDIOIOC_RESUME,
                  (unsigned long) pRecorder->session);
#else
      ret = ioctl(pRecorder->devFd, AUDIOIOC_RESUME, 0);
#endif
      if (ret == OK)
        {
          pRecorder->state = NXRECORDER_STATE_PLAYING;
        }
    }

  return ret;
}
#endif  /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: nxrecorder_setdevice
 *
 *   nxrecorder_setdevice() sets the perferred audio device to use with the
 *   provided nxrecorder context.
 *
 ****************************************************************************/

#ifdef CONFIG_NXRECORDER_INCLUDE_PREFERRED_DEVICE
int nxrecorder_setdevice(FAR struct nxrecorder_s *pRecorder, FAR const char *pDevice)
{
  int                   tempFd;
  struct audio_caps_s   caps;

  DEBUGASSERT(pRecorder != NULL);
  DEBUGASSERT(pDevice != NULL);

  /* Try to open the device */

  tempFd = open(pDevice, O_RDWR);
  if (tempFd == -1)
    {
      /* Error opening the device */

      return -ENOENT;
    }

  /* Validate it's an Audio device by issuing an AUDIOIOC_GETCAPS ioctl */

  caps.ac_len     = sizeof(caps);
  caps.ac_type    = AUDIO_TYPE_QUERY;
  caps.ac_subtype = AUDIO_TYPE_QUERY;
  if (ioctl(tempFd, AUDIOIOC_GETCAPS, (unsigned long) &caps) != caps.ac_len)
    {
      /* Not an Audio device! */

      close(tempFd);
      return -ENODEV;
    }

  /* Close the file */

  close(tempFd);

  /* Save the path and format capabilities of the preferred device */

  strncpy(pRecorder->prefdevice, pDevice, sizeof(pRecorder->prefdevice));
  pRecorder->prefformat = caps.ac_format.b[0] | (caps.ac_format.b[1] << 8);
  pRecorder->preftype = caps.ac_controls.b[0];

  return OK;
}
#endif  /* CONFIG_NXRECORDER_INCLUDE_PREFERRED_DEVICE */

/****************************************************************************
 * Name: nxrecorder_stop
 *
 *   nxrecorder_stop() stops the current playback and closes the file and
 *   the associated device.
 *
 * Input:
 *   pRecorder    Pointer to the initialized MRecorder context
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
int nxrecorder_stop(FAR struct nxrecorder_s *pRecorder)
{
  struct audio_msg_s  term_msg;
  FAR void           *value;

  DEBUGASSERT(pRecorder != NULL);

  /* Validate we are not in IDLE state */

  sem_wait(&pRecorder->sem);                      /* Get the semaphore */
  if (pRecorder->state == NXRECORDER_STATE_IDLE)
    {
      sem_post(&pRecorder->sem);                  /* Release the semaphore */
      return OK;
    }

  sem_post(&pRecorder->sem);

  /* Notify the playback thread that it needs to cancel the playback */

  term_msg.msgId = AUDIO_MSG_STOP;
  term_msg.u.data = 0;
  mq_send(pRecorder->mq, (FAR const char *)&term_msg, sizeof(term_msg),
          CONFIG_NXRECORDER_MSG_PRIO);

  /* Join the thread.  The thread will do all the cleanup. */

  pthread_join(pRecorder->playId, &value);
  pRecorder->playId = 0;

  return OK;
}
#endif  /* CONFIG_AUDIO_EXCLUDE_STOP */

/****************************************************************************
 * Name: nxrecorder_recordfile
 *
 *   nxrecorder_recordfile() tries to play the specified file using the Audio
 *   system.  If a preferred device is specified, it will try to use that
 *   device otherwise it will perform a search of the Audio device files
 *   to find a suitable device.
 *
 * Input:
 *   pRecorder    Pointer to the initialized MRecorder context
 *   pFilename  Pointer to the filename to play
 *   filefmt    Format of the file or AUD_FMT_UNDEF if unknown / to be
 *              determined by nxrecorder_playfile()
 *
 * Returns:
 *   OK         File is being played
 *   -EBUSY     The media device is busy
 *   -ENOSYS    The media file is an unsupported type
 *   -ENODEV    No audio device suitable to play the media type
 *   -ENOENT    The media file was not found
 *
 ****************************************************************************/

int nxrecorder_recordfile(FAR struct nxrecorder_s *pRecorder,
                          FAR const char *pFilename, int filefmt, int subfmt)
{
  struct mq_attr      attr;
  struct sched_param  sparam;
  pthread_attr_t      tattr;
  void               *value;
#ifdef CONFIG_NXRECORDER_INCLUDE_MEDIADIR
  char                path[128];
#endif
  int                 tmpsubfmt = AUDIO_FMT_UNDEF;
  int                 ret;

  DEBUGASSERT(pRecorder != NULL);
  DEBUGASSERT(pFilename != NULL);

  if (pRecorder->state != NXRECORDER_STATE_IDLE)
    {
      return -EBUSY;
    }

  audinfo("==============================\n");
  audinfo("Recording file %s\n", pFilename);
  audinfo("==============================\n");

#ifdef CONFIG_NXRECORDER_INCLUDE_MEDIADIR
  /*If we specified a mediadir, create the file in that dir*/
  snprintf(path, sizeof(path), "%s/%s", pRecorder->mediadir, pFilename);

  if ((pRecorder->fileFd = fopen(path, "w")) == NULL)
    {
      auderr("ERROR: Could not open %s\n", path);
      return -ENOENT;
    }

#else   /* CONFIG_NXRECORDER_INCLUDE_MEDIADIR */
  /*If not, create it locally.*/

  if ((pRecorder->fileFd = fopen(pFilename, "w")) == NULL)
    {
      auderr("ERROR: Could not open %s\n", pFilename);
      return -ENOENT;
    }
#endif  /* CONFIG_NXRECORDER_INCLUDE_MEDIADIR */

#ifdef CONFIG_NXRECORDER_FMT_FROM_EXT
  /* Try to determine the format of audio file based on the extension */

  if (filefmt == AUDIO_FMT_UNDEF)
    {
      filefmt = nxrecorder_fmtfromextension(pRecorder, pFilename, &tmpsubfmt);
    }
#endif

  /* Test if we determined the file format */

  if (filefmt == AUDIO_FMT_UNDEF)
    {
      /* Hmmm, it's some unknown / unsupported type */

      auderr("ERROR: Unsupported format: %d \n", filefmt);
      ret = -ENOSYS;
      goto err_out_nodev;
    }

  /* Test if we have a sub format assignment from above */

  if (subfmt == AUDIO_FMT_UNDEF)
    {
      subfmt = tmpsubfmt;
    }

  /* Try to open the device */

  ret = nxrecorder_opendevice(pRecorder, filefmt, subfmt);
  if (ret < 0)
    {
      /* Error opening the device */

      auderr("ERROR: nxrecorder_opendevice failed: %d\n", ret);
      goto err_out_nodev;
    }

  /* Try to reserve the device */

#ifdef CONFIG_AUDIO_MULTI_SESSION
  ret = ioctl(pRecorder->devFd, AUDIOIOC_RESERVE,
              (unsigned long)&pRecorder->session);
#else
  ret = ioctl(pRecorder->devFd, AUDIOIOC_RESERVE, 0);
#endif
  if (ret < 0)
    {
      /* Device is busy or error */

      auderr("ERROR: Failed to reserve device: %d\n", ret);
      ret = -errno;
      goto err_out;
    }

  /* Create a message queue for the playthread */

  attr.mq_maxmsg  = 16;
  attr.mq_msgsize = sizeof(struct audio_msg_s);
  attr.mq_curmsgs = 0;
  attr.mq_flags   = 0;

  snprintf(pRecorder->mqname, sizeof(pRecorder->mqname), "/tmp/%0lx",
           (unsigned long)((uintptr_t)pRecorder));

  pRecorder->mq = mq_open(pRecorder->mqname, O_RDWR | O_CREAT, 0644, &attr);
  if (pRecorder->mq == NULL)
    {
      /* Unable to open message queue! */

      ret = -errno;
      auderr("ERROR: mq_open failed: %d\n", ret);
      goto err_out;
    }

  /* Register our message queue with the audio device */

  ioctl(pRecorder->devFd, AUDIOIOC_REGISTERMQ, (unsigned long) pRecorder->mq);

  /* Check if there was a previous thread and join it if there was
   * to perform clean-up.
   */

  if (pRecorder->playId != 0)
    {
      pthread_join(pRecorder->playId, &value);
    }

  /* Start the playfile thread to stream the media file to the
   * audio device.
   */

  pthread_attr_init(&tattr);
  sparam.sched_priority = sched_get_priority_max(SCHED_FIFO) - 9;
  (void)pthread_attr_setschedparam(&tattr, &sparam);
  (void)pthread_attr_setstacksize(&tattr, CONFIG_NXRECORDER_PLAYTHREAD_STACKSIZE);

  /* Add a reference count to the recorder for the thread and start the
   * thread.  We increment for the thread to avoid thread start-up
   * race conditions.
   */

  nxrecorder_reference(pRecorder);
  ret = pthread_create(&pRecorder->playId, &tattr, nxrecorder_recordthread,
                       (pthread_addr_t) pRecorder);
  if (ret != OK)
    {
      auderr("ERROR: Failed to create playthread: %d\n", ret);
      goto err_out;
    }

  /* Name the thread */

  pthread_setname_np(pRecorder->playId, "recordthread");
  return OK;

err_out:
  close(pRecorder->devFd);
  pRecorder->devFd = -1;

err_out_nodev:
  if (pRecorder->fileFd != NULL)
    {
      fclose(pRecorder->fileFd);
      pRecorder->fileFd = NULL;
    }

  return ret;
}

/****************************************************************************
 * Name: nxrecorder_stream
 *
 *   nxrecorder_stream() tries to record audio in pcm format, then transfer it
 *   through the specified stream device(uart,usb,internet,etc).
 *   If a preferred audio device is specified, it will try to use that
 *   device otherwise it will perform a search of the Audio device files
 *   to find a suitable device.
 *
 * Input:
 *   pRecorder    Pointer to the initialized MRecorder context
 *   pStreamdev  Pointer to the stream device
 *
 * Returns:
 *   OK         File is being played
 *   -EBUSY     The media device is busy
 *   -ENOSYS    The media file is an unsupported type
 *   -ENODEV    No audio device suitable to play the media type
 *   -ENOENT    The media file was not found
 *
 ****************************************************************************/

int nxrecorder_stream(FAR struct nxrecorder_s *pRecorder,
                      FAR const char *pStreamdev)
{
  struct mq_attr      attr;
  struct sched_param  sparam;
  pthread_attr_t      tattr;
  void               *value;
#ifdef CONFIG_NXRECORDER_INCLUDE_MEDIADIR
  char                path[128];
#endif
  int                 ret;
  FILE               *fd;

  DEBUGASSERT(pRecorder != NULL);
  DEBUGASSERT(pStreamdev != NULL);

  /* Try to open the stream device */

  if (pRecorder->state != NXRECORDER_STATE_IDLE)
    {
      return -EBUSY;
    }

  audinfo("==============================\n");
  audinfo("Stream to device: %s\n", pStreamdev);
  audinfo("==============================\n");

  /*make sure the stream device exists*/
  if ((fd = fopen(pStreamdev, "r")) == NULL)
    {
      auderr("ERROR: stream device %s don't exist!\n", pStreamdev);
      return -ENOENT;
    }

  fclose(fd);

  /*open the stream device in write-only mode*/

  if ((pRecorder->fileFd = fopen(pStreamdev, "w")) == NULL)
    {
      auderr("ERROR: Could not open %s\n", pStreamdev);
      return -ENOENT;
    }

  /* Try to open the audio device */

  ret = nxrecorder_opendevice(pRecorder, AUDIO_FMT_PCM, AUDIO_FMT_UNDEF);
  if (ret < 0)
    {
      /* Error opening the device */

      auderr("ERROR: nxrecorder_opendevice failed: %d\n", ret);
      goto err_out_nodev;
    }

  /* Try to reserve the device */

#ifdef CONFIG_AUDIO_MULTI_SESSION
  ret = ioctl(pRecorder->devFd, AUDIOIOC_RESERVE,
              (unsigned long)&pRecorder->session);
#else
  ret = ioctl(pRecorder->devFd, AUDIOIOC_RESERVE, 0);
#endif
  if (ret < 0)
    {
      /* Device is busy or error */

      auderr("ERROR: Failed to reserve device: %d\n", ret);
      ret = -errno;
      goto err_out;
    }

  /* Create a message queue for the playthread */

  attr.mq_maxmsg  = 16;
  attr.mq_msgsize = sizeof(struct audio_msg_s);
  attr.mq_curmsgs = 0;
  attr.mq_flags   = 0;

  snprintf(pRecorder->mqname, sizeof(pRecorder->mqname), "/tmp/%0lx",
           (unsigned long)((uintptr_t)pRecorder));

  pRecorder->mq = mq_open(pRecorder->mqname, O_RDWR | O_CREAT, 0644, &attr);
  if (pRecorder->mq == NULL)
    {
      /* Unable to open message queue! */

      ret = -errno;
      auderr("ERROR: mq_open failed: %d\n", ret);
      goto err_out;
    }

  /* Register our message queue with the audio device */

  ioctl(pRecorder->devFd, AUDIOIOC_REGISTERMQ, (unsigned long) pRecorder->mq);

  /* Check if there was a previous thread and join it if there was
   * to perform clean-up.
   */

  if (pRecorder->playId != 0)
    {
      pthread_join(pRecorder->playId, &value);
    }

  /* Start the playfile thread to stream the media file to the
   * audio device.
   */

  pthread_attr_init(&tattr);
  sparam.sched_priority = sched_get_priority_max(SCHED_FIFO) - 9;
  (void)pthread_attr_setschedparam(&tattr, &sparam);
  (void)pthread_attr_setstacksize(&tattr, CONFIG_NXRECORDER_PLAYTHREAD_STACKSIZE);

  /* Add a reference count to the recorder for the thread and start the
   * thread.  We increment for the thread to avoid thread start-up
   * race conditions.
   */

  nxrecorder_reference(pRecorder);
  ret = pthread_create(&pRecorder->playId, &tattr, nxrecorder_recordthread,
                       (pthread_addr_t) pRecorder);
  if (ret != OK)
    {
      auderr("ERROR: Failed to create playthread: %d\n", ret);
      goto err_out;
    }

  /* Name the thread */

  pthread_setname_np(pRecorder->playId, "recordthread");
  return OK;

err_out:
  close(pRecorder->devFd);
  pRecorder->devFd = -1;

err_out_nodev:
  if (pRecorder->fileFd != NULL)
    {
      fclose(pRecorder->fileFd);
      pRecorder->fileFd = NULL;
    }

  return ret;
}


/****************************************************************************
 * Name: nxrecorder_setmediadir
 *
 *   nxrecorder_setmediadir() sets the root path for media searches.
 *
 ****************************************************************************/

#ifdef CONFIG_NXRECORDER_INCLUDE_MEDIADIR
void nxrecorder_setmediadir(FAR struct nxrecorder_s *pRecorder,
                            FAR const char *mediadir)
{
  strncpy(pRecorder->mediadir, mediadir, sizeof(pRecorder->mediadir));
}
#endif

/****************************************************************************
 * Name: nxrecorder_create
 *
 *   nxrecorder_create() allocates and initializes a nxrecorder context for
 *   use by further nxrecorder operations.  This routine must be called before
 *   to perform the create for proper reference counting.
 *
 * Input Parameters:  None
 *
 * Returned values:
 *   Pointer to the created context or NULL if there was an error.
 *
 ****************************************************************************/

FAR struct nxrecorder_s *nxrecorder_create(void)
{
  FAR struct nxrecorder_s *pRecorder;

  /* Allocate the memory */

  pRecorder = (FAR struct nxrecorder_s *) malloc(sizeof(struct nxrecorder_s));
  if (pRecorder == NULL)
    {
      return NULL;
    }

  /* Initialize the context data */

  pRecorder->state = NXRECORDER_STATE_IDLE;
  pRecorder->devFd = -1;
  pRecorder->fileFd = NULL;
#ifdef CONFIG_NXRECORDER_INCLUDE_PREFERRED_DEVICE
  pRecorder->prefdevice[0] = '\0';
  pRecorder->prefformat = 0;
  pRecorder->preftype = 0;
#endif
  pRecorder->mq = NULL;
  pRecorder->playId = 0;
  pRecorder->crefs = 1;

#ifndef CONFIG_AUDIO_EXCLUDE_TONE
  pRecorder->bass = 50;
  pRecorder->treble = 50;
#endif



#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
#ifndef CONFIG_AUDIO_EXCLUDE_BALANCE
  pRecorder->balance = 500;
#endif
  pRecorder->volume = 400;
#endif

#ifdef CONFIG_AUDIO_MULTI_SESSION
  pRecorder->session = NULL;
#endif

#ifdef CONFIG_NXRECORDER_INCLUDE_MEDIADIR
  strncpy(pRecorder->mediadir, CONFIG_NXRECORDER_DEFAULT_MEDIADIR,
          sizeof(pRecorder->mediadir));
#endif
  sem_init(&pRecorder->sem, 0, 1);

  return pRecorder;
}

/****************************************************************************
 * Name: nxrecorder_release
 *
 *   nxrecorder_release() reduces the reference count by one and if it
 *   reaches zero, frees the context.
 *
 * Input Parameters:
 *   pRecorder    Pointer to the NxRecorder context
 *
 * Returned values:    None
 *
 ****************************************************************************/

void nxrecorder_release(FAR struct nxrecorder_s *pRecorder)
{
  int         refcount;
  FAR void   *value;

  /* Grab the semaphore */

  while (sem_wait(&pRecorder->sem) < 0)
    {
      int errcode = errno;
      DEBUGASSERT(errcode > 0);

      if (errcode != EINTR)
        {
          auderr("ERROR: sem_wait failed: %d\n", errcode);
          return;
        }
    }

  /* Check if there was a previous thread and join it if there was */

  if (pRecorder->playId != 0)
    {
      sem_post(&pRecorder->sem);
      pthread_join(pRecorder->playId, &value);
      pRecorder->playId = 0;

      while (sem_wait(&pRecorder->sem) < 0)
        {
          int errcode = errno;
          DEBUGASSERT(errcode > 0);

          if (errcode != -EINTR)
            {
              auderr("ERROR: sem_wait failed: %d\n", errcode);
              return;
            }
        }
    }

  /* Reduce the reference count */

  refcount = pRecorder->crefs--;
  sem_post(&pRecorder->sem);

  /* If the ref count *was* one, then free the context */

  if (refcount == 1)
    {
      free(pRecorder);
    }
}

/****************************************************************************
 * Name: nxrecorder_reference
 *
 *   nxrecorder_reference() increments the reference count by one.
 *
 * Input Parameters:
 *   pRecorder    Pointer to the NxRecorder context
 *
 * Returned values:    None
 *
 ****************************************************************************/

void nxrecorder_reference(FAR struct nxrecorder_s *pRecorder)
{
  /* Grab the semaphore */

  while (sem_wait(&pRecorder->sem) < 0)
    {
      int errcode = errno;
      DEBUGASSERT(errcode > 0);

      if (errcode != -EINTR)
        {
          auderr("ERROR: sem_wait failed: %d\n", errcode);
          return;
        }
    }

  /* Increment the reference count */

  pRecorder->crefs++;
  sem_post(&pRecorder->sem);
}

/****************************************************************************
 * Name: nxrecorder_systemreset
 *
 *   nxrecorder_systemreset() performs a HW reset on all registered
 *   audio devices.
 *
 ****************************************************************************/

#ifdef CONFIG_NXRECORDER_INCLUDE_SYSTEM_RESET
int nxrecorder_systemreset(FAR struct nxrecorder_s *pRecorder)
{
  struct dirent *pDevice;
  DIR           *dirp;
  char           path[64];

  /* Search for a device in the audio device directory */

#ifdef CONFIG_AUDIO_CUSTOM_DEV_PATH
#ifdef CONFIG_AUDIO_DEV_ROOT
  dirp = opendir("/dev");
#else
  dirp = opendir(CONFIG_AUDIO_DEV_PATH);
#endif
#else
  dirp = opendir("/dev/audio");
#endif
  if (dirp == NULL)
    {
      return -ENODEV;
    }

  while ((pDevice = readdir(dirp)) != NULL)
    {
      /* We found the next device.  Try to open it and
       * get its audio capabilities.
       */

#ifdef CONFIG_AUDIO_CUSTOM_DEV_PATH
#ifdef CONFIG_AUDIO_DEV_ROOT
      snprintf(path,  sizeof(path), "/dev/%s", pDevice->d_name);
#else
      snprintf(path,  sizeof(path), CONFIG_AUDIO_DEV_PATH "/%s", pDevice->d_name);
#endif
#else
      snprintf(path,  sizeof(path), "/dev/audio/%s", pDevice->d_name);
#endif
      if ((pRecorder->devFd = open(path, O_RDWR)) != -1)
        {
          /* We have the device file open.  Now issue an
           * AUDIO ioctls to perform a HW reset
           */

          ioctl(pRecorder->devFd, AUDIOIOC_HWRESET, 0);

          /* Now close the device */

          close(pRecorder->devFd);
        }

    }

  pRecorder->devFd = -1;
  return OK;
}
#endif  /* CONFIG_NXRECORDER_INCLUDE_SYSTEM_RESET */

