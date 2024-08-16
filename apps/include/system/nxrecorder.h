/****************************************************************************
 * apps/include/system/nxrecorder.h
 *
 *   Copyright (C) 2018  zGlue INC All rights reserved.
 *   Author: Zhengwei Wang <zhengwei@zglue.com>
 *
 * References:
 *
 * -  The framework for this file is based on Ken Pettit's nxplayer.
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

#ifndef __APPS_INCLUDE_SYSTEM_NXRECORDER_H
#define __APPS_INCLUDE_SYSTEM_NXRECORDER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Type Declarations
 ****************************************************************************/
/* This structure describes the internal state of the NxRecorder */

struct nxrecorder_s
{
  int         state;          /* Current recorder state */
  int         devFd;          /* File descriptor of active device */
  mqd_t       mq;             /* Message queue for the playthread */
  char        mqname[16];     /* Name of our message queue */
  pthread_t   playId;         /* Thread ID of the playthread */
  int         crefs;          /* Number of references to the recorder */
  sem_t       sem;            /* Thread sync semaphore */
  FILE       *fileFd;         /* File descriptor of open file */
#ifdef CONFIG_NXRECORDER_INCLUDE_PREFERRED_DEVICE
  char        prefdevice[CONFIG_NAME_MAX]; /* Preferred audio device */
  int         prefformat;     /* Formats supported by preferred device */
  int         preftype;       /* Types supported by preferred device */
#endif
#ifdef CONFIG_NXRECORDER_INCLUDE_MEDIADIR
  char        mediadir[CONFIG_NAME_MAX];   /* Root media directory where media is located */
#endif
#ifdef CONFIG_AUDIO_MULTI_SESSION
  FAR void   *session;        /* Session assigment from device */
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
  uint16_t    volume;         /* Volume as a whole percentage (0-100) */
#ifndef CONFIG_AUDIO_EXCLUDE_BALANCE
  uint16_t    balance;        /* Balance as a whole % (0=left off, 100=right off) */
#endif
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_TONE
  uint16_t    treble;         /* Treble as a whole % */
  uint16_t    bass;           /* Bass as a whole % */
#endif
};

typedef int (*nxrecorder_func)(FAR struct nxrecorder_s *pRecorder, char *pargs);

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nxrecorder_create
 *
 *   Allocates and Initializes a NxRecorder context that is passed to all
 *   nxrecorder routines.  The recorder MUST be destroyed using the
 *   nxrecorder_destroy() routine since the context is reference counted.
 *   The context can be used in a mode where the caller creates the
 *   context, starts recording to a file, and then forgets about the context
 *   and it will self free.  This is because the nxrecorder_playfile
 *   will also create a reference to the context, so the client calling
 *   nxrecorder_destroy() won't actually de-allocate anything.  The freeing
 *   will occur after the playthread has completed.
 *
 *   Alternately, the caller can create the objec and hold on to it, then
 *   the context will persist until the original creator destroys it.
 *
 * Input Parameters:    None
 *
 * Returned Value:
 *   Pointer to created NxRecorder context or NULL if error.
 *
 ****************************************************************************/

FAR struct nxrecorder_s *nxrecorder_create(void);

/****************************************************************************
 * Name: nxrecorder_release
 *
 *   Reduces the reference count to the recorder and if it reaches zero,
 *   frees all memory used by the context.
 *
 * Input Parameters:
 *   pRecorder    Pointer to the NxRecorder context
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxrecorder_release(FAR struct nxrecorder_s *pRecorder);

/****************************************************************************
 * Name: nxrecorder_reference
 *
 *   Increments the reference count to the recorder.
 *
 * Input Parameters:
 *   pRecorder    Pointer to the NxRecorder context
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxrecorder_reference(FAR struct nxrecorder_s *pRecorder);

/****************************************************************************
 * Name: nxrecorder_setdevice
 *
 *   Sets the preferred Audio device to use with the instance of the
 *   nxrecorder.  Without a preferred device set, the nxrecorder will search
 *   the audio subsystem to find a suitable device depending on the
 *   type of audio operation requested (i.e. an MP3 encoder device when
 *   the user want to record the audio to an MP3 file, a WAV encoder device
 *   for a WAV file, etc.).
 *
 * Input Parameters:
 *   pRecorder   - Pointer to the context to initialize
 *   device    - Pointer to pathname of the preferred device
 *
 * Returned Value:
 *   OK if context initialized successfully, error code otherwise.
 *
 ****************************************************************************/

int nxrecorder_setdevice(FAR struct nxrecorder_s *pRecorder,
                         FAR const char *device);

/****************************************************************************
 * Name: nxrecorder_playfile
 *
 *   record to the specified media file (from the filesystem) using the
 *   Audio system.  If a preferred device has been set, that device
 *   will be used for recording, otherwise the first suitable device
 *   found in the /dev/audio directory will be used.
 *
 * Input Parameters:
 *   pRecorder   - Pointer to the context to initialize
 *   filename  - Pointer to pathname of the file to play
 *   filefmt   - Format of audio in filename if known, AUDIO_FMT_UNDEF
 *               to let nxrecorder_playfile() determine automatically.
 *   subfmt    - Sub-Format of audio in filename if known, AUDIO_FMT_UNDEF
 *               to let nxrecorder_playfile() determine automatically.
 *
 * Returned Value:
 *   OK if file found, device found, and record started.
 *
 ****************************************************************************/

int nxrecorder_recordfile(FAR struct nxrecorder_s *pRecorder,
                          FAR const char *filename, int filefmt, int subfmt);

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
                      FAR const char *pStreamdev);


/****************************************************************************
 * Name: nxrecorder_stop
 *
 *   Stops current record.
 *
 * Input Parameters:
 *   pRecorder   - Pointer to the context to initialize
 *
 * Returned Value:
 *   OK if file found, device found, and record started.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
int nxrecorder_stop(FAR struct nxrecorder_s *pRecorder);
#endif

/****************************************************************************
 * Name: nxrecorder_pause
 *
 *   Pauses current record.
 *
 * Input Parameters:
 *   pRecorder   - Pointer to the context to initialize
 *
 * Returned Value:
 *   OK if file found, device found, and record started.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
int nxrecorder_pause(FAR struct nxrecorder_s *pRecorder);
#endif

/****************************************************************************
 * Name: nxrecorder_resume
 *
 *   Resumes current record.
 *
 * Input Parameters:
 *   pRecorder   - Pointer to the context to initialize
 *
 * Returned Value:
 *   OK if file found, device found, and record started.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
int nxrecorder_resume(FAR struct nxrecorder_s *pRecorder);
#endif

/****************************************************************************
 * Name: nxrecorder_setvolume
 *
 *   Sets the playback volume.  The volume is represented in 1/10th of a
 *   percent increments, so the range is 0-1000.  A value of 10 would mean
 *   1%.
 *
 * Input Parameters:
 *   pRecorder   - Pointer to the context to initialize
 *   volume    - Volume level to set in 1/10th percent increments
 *
 * Returned Value:
 *   OK if file found, device found, and playback started.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
int nxrecorder_setvolume(FAR struct nxrecorder_s *pRecorder, uint16_t volume);
#endif

/****************************************************************************
 * Name: nxrecorder_setbalance
 *
 *   Sets the playback balance.  The balance is represented in 1/10th of a
 *   percent increments, so the range is 0-1000.  A value of 10 would mean
 *   1%.
 *
 * Input Parameters:
 *   pRecorder   - Pointer to the context to initialize
 *   balance   - Balance level to set in 1/10th percent increments
 *
 * Returned Value:
 *   OK if file found, device found, and playback started.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
#ifndef CONFIG_AUDIO_EXCLUDE_BALANCE
int nxrecorder_setbalance(FAR struct nxrecorder_s *pRecorder, uint16_t balance);
#endif
#endif

/****************************************************************************
 * Name: nxrecorder_setmediadir
 *
 *   Sets the root media directory for non-path qualified file searches.
 *
 * Input Parameters:
 *   pRecorder   - Pointer to the context to initialize
 *   mediadir  - Pointer to pathname of the media directory
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxrecorder_setmediadir(FAR struct nxrecorder_s *pRecorder,
                            FAR const char *mediadir);

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
                               FAR uint8_t *equalization);
#endif

/****************************************************************************
 * Name: nxrecorder_setbass
 *
 *   Sets the playback bass level.  The bass is represented in one percent
 *   increments, so the range is 0-100.
 *
 * Input Parameters:
 *   pRecorder   - Pointer to the context to initialize
 *   bass      - Bass level to set in one percent increments
 *
 * Returned Value:
 *   OK if the bass level was set successfully
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_TONE
int nxrecorder_setbass(FAR struct nxrecorder_s *pRecorder, uint8_t bass);
#endif

/****************************************************************************
 * Name: nxrecorder_settreble
 *
 *   Sets the playback treble level.  The bass is represented in one percent
 *   increments, so the range is 0-100.
 *
 * Input Parameters:
 *   pRecorder   - Pointer to the context to initialize
 *   treble    - Treble level to set in one percent increments
 *
 * Returned Value:
 *   OK if the treble level was set successfully
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_TONE
int nxrecorder_settreble(FAR struct nxrecorder_s *pRecorder, uint8_t treble);
#endif

/****************************************************************************
 * Name: nxrecorder_systemreset
 *
 *   Performs an audio system reset, including a hardware reset on all
 *   registered audio devices.
 *
 * Input Parameters:
 *   pRecorder   - Pointer to the context to initialize
 *
 * Returned Value:
 *   OK if device found.
 *
 ****************************************************************************/

#ifdef CONFIG_NXRECORDER_INCLUDE_SYSTEM_RESET
int nxrecorder_systemreset(FAR struct nxrecorder_s *pRecorder);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __APPS_INCLUDE_SYSTEM_NXRECORDER_H */
