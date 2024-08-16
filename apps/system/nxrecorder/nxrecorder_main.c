/****************************************************************************
 * apps/system/nxrecorder/nxrecorder_main.c
 *
 *   Copyright (C) 2018  zGlue INC All rights reserved.
 *   Author: Zhengwei Wang <zhengwei@zglue.com>
 *
 * References:
 *
 * -  The framework for this app is based on Ken Pettit's nxplayer.
.*
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
#include <nuttx/audio/audio.h>

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <assert.h>

#include "system/readline.h"
#include "system/nxrecorder.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NXRECORDER_VER  "1.00"

#ifdef CONFIG_NXRECORDER_INCLUDE_HELP
#  define NXRECORDER_HELP_TEXT(x)  #x
#else
#  define NXRECORDER_HELP_TEXT(x)
#endif

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

struct mp_cmd_s
{
  const char      *cmd;       /* The command text */
  const char      *arghelp;   /* Text describing the args */
  nxrecorder_func  pFunc;     /* Pointer to command handler */
  const char      *help;      /* The help text */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int nxrecorder_cmd_quit(FAR struct nxrecorder_s *pRecorder, char *parg);
static int nxrecorder_cmd_record(FAR struct nxrecorder_s *pRecorder, char *parg);
static int nxrecorder_cmd_stream(FAR struct nxrecorder_s *pRecorder, char *parg);


#ifdef CONFIG_NXRECORDER_INCLUDE_SYSTEM_RESET
static int nxrecorder_cmd_reset(FAR struct nxrecorder_s *pRecorder, char *parg);
#endif

#ifdef CONFIG_NXRECORDER_INCLUDE_PREFERRED_DEVICE
static int nxrecorder_cmd_device(FAR struct nxrecorder_s *pRecorder, char *parg);
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int nxrecorder_cmd_pause(FAR struct nxrecorder_s *pRecorder, char *parg);
static int nxrecorder_cmd_resume(FAR struct nxrecorder_s *pRecorder, char *parg);
#endif

#ifdef CONFIG_NXRECORDER_INCLUDE_MEDIADIR
static int nxrecorder_cmd_mediadir(FAR struct nxrecorder_s *pRecorder, char *parg);
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int nxrecorder_cmd_stop(FAR struct nxrecorder_s *pRecorder, char *parg);
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
static int nxrecorder_cmd_volume(FAR struct nxrecorder_s *pRecorder, char *parg);
#ifndef CONFIG_AUDIO_EXCLUDE_BALANCE
static int nxrecorder_cmd_balance(FAR struct nxrecorder_s *pRecorder, char *parg);
#endif
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_TONE
static int nxrecorder_cmd_bass(FAR struct nxrecorder_s *pRecorder, char *parg);
static int nxrecorder_cmd_treble(FAR struct nxrecorder_s *pRecorder, char *parg);
#endif

#ifdef CONFIG_NXRECORDER_INCLUDE_HELP
static int nxrecorder_cmd_help(FAR struct nxrecorder_s *pRecorder, char *parg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mp_cmd_s g_nxrecorder_cmds[] =
{
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
#ifndef CONFIG_AUDIO_EXCLUDE_BALANCE
  { "balance",  "d%",       nxrecorder_cmd_balance,   NXRECORDER_HELP_TEXT(Set balance percentage ( < 50 % means more left)) },
#endif
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_TONE
  { "bass",     "d%",       nxrecorder_cmd_bass,      NXRECORDER_HELP_TEXT(Set bass level percentage) },
#endif
#ifdef CONFIG_NXRECORDER_INCLUDE_PREFERRED_DEVICE
  { "device",   "devfile",  nxrecorder_cmd_device,    NXRECORDER_HELP_TEXT(Specify a preferred audio device) },
#endif
#ifdef CONFIG_NXRECORDER_INCLUDE_HELP
  { "h",        "",         nxrecorder_cmd_help,      NXRECORDER_HELP_TEXT(Display help for commands) },
  { "help",     "",         nxrecorder_cmd_help,      NXRECORDER_HELP_TEXT(Display help for commands) },
#endif
#ifdef CONFIG_NXRECORDER_INCLUDE_MEDIADIR
  { "mediadir", "path",     nxrecorder_cmd_mediadir,  NXRECORDER_HELP_TEXT(Change the media directory) },
#endif
  { "record",   "filename", nxrecorder_cmd_record,      NXRECORDER_HELP_TEXT(record to a media file) },
  { "stream",   "stream_dev", nxrecorder_cmd_stream,      NXRECORDER_HELP_TEXT(record and stream out the pcm data) },
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  { "pause",    "",         nxrecorder_cmd_pause,     NXRECORDER_HELP_TEXT(Pause playback) },
#endif
#ifdef CONFIG_NXRECORDER_INCLUDE_SYSTEM_RESET
  { "reset",    "",         nxrecorder_cmd_reset,     NXRECORDER_HELP_TEXT(Perform a HW reset) },
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  { "resume",   "",         nxrecorder_cmd_resume,    NXRECORDER_HELP_TEXT(Resume playback) },
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  { "stop",     "",         nxrecorder_cmd_stop,      NXRECORDER_HELP_TEXT(Stop playback) },
#endif
  { "tone",     "freq secs", NULL,                   NXRECORDER_HELP_TEXT(Produce a pure tone) },
#ifndef CONFIG_AUDIO_EXCLUDE_TONE
  { "treble",   "d%",       nxrecorder_cmd_treble,    NXRECORDER_HELP_TEXT(Set treble level percentage) },
#endif
  { "q",        "",         nxrecorder_cmd_quit,      NXRECORDER_HELP_TEXT(Exit NxRecorder) },
  { "quit",     "",         nxrecorder_cmd_quit,      NXRECORDER_HELP_TEXT(Exit NxRecorder) },
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
  { "volume",   "d%",       nxrecorder_cmd_volume,    NXRECORDER_HELP_TEXT(Set volume to level specified) }
#endif
};
static const int g_nxrecorder_cmd_count = sizeof(g_nxrecorder_cmds) / sizeof(struct mp_cmd_s);


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxrecorder_cmd_play
 *
 *   nxrecorder_cmd_play() plays the specified media file using the nxrecorder
 *   context.
 *
 ****************************************************************************/

static int nxrecorder_cmd_record(FAR struct nxrecorder_s *pRecorder, char *parg)
{
  int     ret;

  /* Try to play the file specified */

  ret = nxrecorder_recordfile(pRecorder, parg, AUDIO_FMT_UNDEF, AUDIO_FMT_UNDEF);

  /* nxrecorder_playfile returned values:
   *
   *   OK         File is being played
   *   -EBUSY     The media device is busy
   *   -ENOSYS    The media file is an unsupported type
   *   -ENODEV    No audio device suitable to play the media type
   *   -ENOENT    The media file was not found
   */

  switch (-ret)
    {
      case OK:
        break;

      case ENODEV:
        printf("No suitable Audio Device found\n");
        break;

      case EBUSY:
        printf("Audio device busy\n");
        break;

      case ENOENT:
        printf("File %s not found\n", parg);
        break;

      case ENOSYS:
        printf("Unknown audio format\n");
        break;

      default:
        printf("Error playing file: %d\n", -ret);
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: nxrecorder_cmd_stream
 *
 *   nxrecorder_cmd_stream() record audio in pcm formate and transfer it throught
 *   the specified stream device.
 *
 ****************************************************************************/

static int nxrecorder_cmd_stream(FAR struct nxrecorder_s *pRecorder, char *parg)
{
  int     ret;

  /* Try to play the file specified */

  ret = nxrecorder_stream(pRecorder, parg);

  /* nxrecorder_playfile returned values:
   *
   *   OK         File is being played
   *   -EBUSY     The media device is busy
   *   -ENOSYS    The media file is an unsupported type
   *   -ENODEV    No audio device suitable to play the media type
   *   -ENOENT    The media file was not found
   */

  switch (-ret)
    {
      case OK:
        break;

      case ENODEV:
        printf("No suitable Audio Device found\n");
        break;

      case EBUSY:
        printf("Audio device busy\n");
        break;

      case ENOENT:
        printf("Stream device %s not found\n", parg);
        break;

      case ENOSYS:
        printf("Unknown audio format\n");
        break;

      default:
        printf("Error playing file: %d\n", -ret);
        break;
    }

  return ret;
}



/****************************************************************************
 * Name: nxrecorder_cmd_volume
 *
 *   nxrecorder_cmd_volume() sets the volume level.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
static int nxrecorder_cmd_volume(FAR struct nxrecorder_s *pRecorder, char *parg)
{
  uint16_t   percent;

  /* If no arg given, then print current volume */

  if (parg == NULL || *parg == '\0')
    {
      printf("volume: %d\n", pRecorder->volume / 10);
    }
  else
    {
      /* Get the percentage value from the argument */

      percent = (uint16_t) (atof(parg) * 10.0);
      nxrecorder_setvolume(pRecorder, percent);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: nxrecorder_cmd_bass
 *
 *   nxrecorder_cmd_bass() sets the bass level and range.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_TONE
static int nxrecorder_cmd_bass(FAR struct nxrecorder_s *pRecorder, char *parg)
{
  uint8_t   level_percent;

  /* If no arg given, then print current bass */

  if (parg == NULL || *parg == '\0')
    {
      printf("bass: %d\n", pRecorder->bass);
    }
  else
    {
      /* Get the level and range percentage value from the argument */

      level_percent = (uint8_t) atoi(parg);
      nxrecorder_setbass(pRecorder, level_percent);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: nxrecorder_cmd_treble
 *
 *   nxrecorder_cmd_treble() sets the treble level and range.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_TONE
static int nxrecorder_cmd_treble(FAR struct nxrecorder_s *pRecorder, char *parg)
{
  uint8_t   level_percent;

  /* If no arg given, then print current bass */

  if (parg == NULL || *parg == '\0')
    {
      printf("treble: %d\n", pRecorder->treble);
    }
  else
    {
      /* Get the level and range percentage value from the argument */

      level_percent = (uint8_t) atoi(parg);
      nxrecorder_settreble(pRecorder, level_percent);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: nxrecorder_cmd_balance
 *
 *   nxrecorder_cmd_balance() sets the balance level.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
#ifndef CONFIG_AUDIO_EXCLUDE_BALANCE
static int nxrecorder_cmd_balance(FAR struct nxrecorder_s *pRecorder, char *parg)
{
  uint16_t   percent;

  /* If no arg given, then print current volume */

  if (parg == NULL || *parg == '\0')
    {
      printf("balance: %d\n", pRecorder->volume / 10);
    }
  else
    {
      /* Get the percentage value from the argument */

      percent = (uint16_t) (atof(parg) * 10.0);
      nxrecorder_setbalance(pRecorder, percent);
    }

  return OK;
}
#endif
#endif

/****************************************************************************
 * Name: nxrecorder_cmd_reset
 *
 *   nxrecorder_cmd_reset() performs a HW reset of all the audio devices.
 *
 ****************************************************************************/

#ifdef CONFIG_NXRECORDER_INCLUDE_SYSTEM_RESET
static int nxrecorder_cmd_reset(FAR struct nxrecorder_s *pRecorder, char *parg)
{
  nxrecorder_systemreset(pRecorder);

  return OK;
}
#endif

/****************************************************************************
 * Name: nxrecorder_cmd_mediadir
 *
 *   nxrecorder_cmd_mediadir() displays or changes the media directory
 *   context.
 *
 ****************************************************************************/

#ifdef CONFIG_NXRECORDER_INCLUDE_MEDIADIR
static int nxrecorder_cmd_mediadir(FAR struct nxrecorder_s *pRecorder, char *parg)
{
  /* If no arg given, then print current media dir */

  if (parg == NULL || *parg == '\0')
    {
      printf("%s\n", pRecorder->mediadir);
    }
  else
    {
      nxrecorder_setmediadir(pRecorder, parg);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: nxrecorder_cmd_stop
 *
 *   nxrecorder_cmd_stop() stops playback of currently playing file
 *   context.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int nxrecorder_cmd_stop(FAR struct nxrecorder_s *pRecorder, char *parg)
{
  /* Stop the playback */

  nxrecorder_stop(pRecorder);

  return OK;
}
#endif

/****************************************************************************
 * Name: nxrecorder_cmd_pause
 *
 *   nxrecorder_cmd_pause() pauses playback of currently playing file
 *   context.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int nxrecorder_cmd_pause(FAR struct nxrecorder_s *pRecorder, char *parg)
{
  /* Pause the playback */

  nxrecorder_pause(pRecorder);

  return OK;
}
#endif

/****************************************************************************
 * Name: nxrecorder_cmd_resume
 *
 *   nxrecorder_cmd_resume() resumes playback of currently playing file
 *   context.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int nxrecorder_cmd_resume(FAR struct nxrecorder_s *pRecorder, char *parg)
{
  /* Resume the playback */

  nxrecorder_resume(pRecorder);

  return OK;
}
#endif

/****************************************************************************
 * Name: nxrecorder_cmd_device
 *
 *   nxrecorder_cmd_device() sets the preferred audio device for playback
 *
 ****************************************************************************/

#ifdef CONFIG_NXRECORDER_INCLUDE_PREFERRED_DEVICE
static int nxrecorder_cmd_device(FAR struct nxrecorder_s *pRecorder, char *parg)
{
  int     ret;
  char    path[32];

  /* First try to open the file directly */

  ret = nxrecorder_setdevice(pRecorder, parg);
  if (ret == -ENOENT)
    {
      /* Append the /dev/audio path and try again */

#ifdef CONFIG_AUDIO_CUSTOM_DEV_PATH
#ifdef CONFIG_AUDIO_DEV_ROOT
      snprintf(path,  sizeof(path), "/dev/%s", parg);
#else
      snprintf(path,  sizeof(path), CONFIG_AUDIO_DEV_PATH "/%s", parg);
#endif
#else
      snprintf(path, sizeof(path), "/dev/audio/%s", parg);
#endif
      ret = nxrecorder_setdevice(pRecorder, path);
    }

  /* Test if the device file exists */

  if (ret == -ENOENT)
    {
      /* Device doesn't exit.  Report error */

      printf("Device %s not found\n", parg);
      return ret;
    }

  /* Test if is is an audio device */

  if (ret == -ENODEV)
    {
      printf("Device %s is not an audio device\n", parg);
      return ret;
    }

  if (ret < 0)
    {
      return ret;
    }

  /* Device set successfully */

  return OK;
}
#endif  /* CONFIG_NXRECORDER_INCLUDE_PREFERRED_DEVICE */

/****************************************************************************
 * Name: nxrecorder_cmd_quit
 *
 *   nxrecorder_cmd_quit() terminates the application
 ****************************************************************************/

static int nxrecorder_cmd_quit(FAR struct nxrecorder_s *pRecorder, char *parg)
{
  /* Stop the playback if any */

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  nxrecorder_stop(pRecorder);
#endif

  return OK;
}

/****************************************************************************
 * Name: nxrecorder_cmd_help
 *
 *   nxrecorder_cmd_help() displays the application's help information on
 *   supported commands and command syntax.
 ****************************************************************************/

#ifdef CONFIG_NXRECORDER_INCLUDE_HELP
static int nxrecorder_cmd_help(FAR struct nxrecorder_s *pRecorder, char *parg)
{
  int   x, len, maxlen = 0;
  int   c;

  /* Calculate length of longest cmd + arghelp */

  for (x = 0; x < g_nxrecorder_cmd_count; x++)
    {
      len = strlen(g_nxrecorder_cmds[x].cmd) + strlen(g_nxrecorder_cmds[x].arghelp);
      if (len > maxlen)
        {
          maxlen = len;
        }
    }

  printf("NxRecorder commands\n================\n");
  for (x = 0; x < g_nxrecorder_cmd_count; x++)
    {
      /* Print the command and it's arguments */

      printf("  %s %s", g_nxrecorder_cmds[x].cmd, g_nxrecorder_cmds[x].arghelp);

      /* Calculate number of spaces to print before the help text */

      len = maxlen - (strlen(g_nxrecorder_cmds[x].cmd) + strlen(g_nxrecorder_cmds[x].arghelp));
      for (c = 0; c < len; c++)
        {
          printf(" ");
        }

      printf("  : %s\n", g_nxrecorder_cmds[x].help);
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxrecorder
 *
 *   nxrecorder() reads in commands from the console using the readline
 *   system add-in and implemets a command-line based media recorder that
 *   uses the NuttX audio system to play media files read in from the
 *   file system.  Commands are provided for setting volume, base and
 *   other audio features, as well as for pausing and stoping the
 *   playback.
 *
 * Input Parameters:
 *   buf       - The user allocated buffer to be filled.
 *   buflen    - the size of the buffer.
 *   instream  - The stream to read characters from
 *   outstream - The stream to each characters to.
 *
 * Returned values:
 *   On success, the (positive) number of bytes transferred is returned.
 *   EOF is returned to indicate either an end of file condition or a
 *   failure.
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int nxrecorder_main(int argc, char *argv[])
#endif
{
  char                    buffer[64];
  int                     len, x, running;
  char                    *cmd, *arg;
  FAR struct nxrecorder_s   *pRecorder;

  printf("NxRecorder version " NXRECORDER_VER "\n");
  printf("h for commands, q to exit\n");
  printf("\n");

  /* Initialize our NxRecorder context */

  pRecorder = nxrecorder_create();
  if (pRecorder == NULL)
    {
      printf("Error:  Out of RAM\n");
      return -ENOMEM;
    }

  /* Loop until the user exits */

  running = TRUE;
  while (running)
    {
      /* Print a prompt */

      printf("nxrecorder> ");
      fflush(stdout);

      /* Read a line from the terminal */

      len = readline(buffer, sizeof(buffer), stdin, stdout);
      buffer[len] = '\0';
      if (len > 0)
        {
          if (buffer[len - 1] == '\n')
            {
              buffer[len - 1] = '\0';
            }

          /* Parse the command from the argument */

          cmd = strtok_r(buffer, " \n", &arg);
          if (cmd == NULL)
            {
              continue;
            }

          /* Remove leading spaces from arg */

          while (*arg == ' ')
            {
              arg++;
            }

          /* Find the command in our cmd array */

          for (x = 0; x < g_nxrecorder_cmd_count; x++)
            {
              if (strcmp(cmd, g_nxrecorder_cmds[x].cmd) == 0)
                {
                  /* Command found.  Call it's handler if not NULL */

                  if (g_nxrecorder_cmds[x].pFunc != NULL)
                    {
                      g_nxrecorder_cmds[x].pFunc(pRecorder, arg);
                    }

                  /* Test if it is a quit command */

                  if (g_nxrecorder_cmds[x].pFunc == nxrecorder_cmd_quit)
                    {
                      running = FALSE;
                    }
                  break;
                }
            }

          /* Test for Unknown command */

          if (x == g_nxrecorder_cmd_count)
            {
              printf("%s:  unknown nxrecorder command\n", buffer);
            }
        }
    }

  /* Release the NxRecorder context */

//  nxrecorder_detach(pRecorder);
  nxrecorder_release(pRecorder);

  return OK;
}
