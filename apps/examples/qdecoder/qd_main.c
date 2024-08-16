/****************************************************************************
 * examples/qe/qe_main.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <limits.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/sensors/qdecoder.h>

#include "qd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct qd_example_s g_qdexample;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qd_devpath
 ****************************************************************************/

static void qd_devpath(FAR const char *devpath)
{
  /* Get rid of any old device path */

  if (g_qdexample.devpath)
    {
      free(g_qdexample.devpath);
    }

  /* The set-up the new device path by copying the string */

  g_qdexample.devpath = strdup(devpath);
}

/****************************************************************************
 * Name: qe_help
 ****************************************************************************/

#ifdef CONFIG_NSH_BUILTIN_APPS
static void qd_help(void)
{
  printf("\nUsage: qd [OPTIONS]\n\n");
  printf("OPTIONS include:\n");
  printf("  [-p devpath] QD device path\n");
  printf("  [-n samples] Number of samples\n");
  printf("  [-t sec]    Delay between samples (sec)\n");
  printf("  [-r]         Reset the position to zero\n");
  printf("  [-h]         Shows this message and exits\n\n");
}
#endif

/****************************************************************************
 * Name: arg_string
 ****************************************************************************/

#ifdef CONFIG_NSH_BUILTIN_APPS
static int arg_string(FAR char **arg, FAR char **value)
{
  FAR char *ptr = *arg;

  if (ptr[2] == '\0')
    {
      *value = arg[1];
      return 2;
    }
  else
    {
      *value = &ptr[2];
      return 1;
    }
}
#endif

/****************************************************************************
 * Name: arg_decimal
 ****************************************************************************/

#ifdef CONFIG_NSH_BUILTIN_APPS
static int arg_decimal(FAR char **arg, FAR long *value)
{
  FAR char *string;
  int ret;

  ret = arg_string(arg, &string);
  *value = strtol(string, NULL, 10);
  return ret;
}
#endif

/****************************************************************************
 * Name: parse_args
 ****************************************************************************/

#ifdef CONFIG_NSH_BUILTIN_APPS
static void parse_args(int argc, FAR char **argv)
{
  FAR char *ptr;
  FAR char *str;
  long value;
  int index;
  int nargs;

  g_qdexample.reset  = true;
  g_qdexample.nloops = 1;
  g_qdexample.delay  = CONFIG_EXAMPLES_QDECODER_DELAY;

  for (index = 1; index < argc; )
    {
      ptr = argv[index];
      if (ptr[0] != '-')
        {
          printf("Invalid options format: %s\n", ptr);
          exit(0);
        }

      switch (ptr[1])
        {
          case 'n':
            nargs = arg_decimal(&argv[index], &value);
            if (value < 0 || value > INT_MAX)
              {
                printf("Sample count out of range: %ld\n", value);
                exit(1);
              }

            g_qdexample.nloops = (unsigned int)value;
            index += nargs;
            break;

          case 'p':
            nargs = arg_string(&argv[index], &str);
            qd_devpath(str);
            index += nargs;
            break;

          case 't':
            nargs = arg_decimal(&argv[index], &value);
            if (value < 0 || value > INT_MAX)
              {
                printf("Sample delay out of range: %ld\n", value);
                exit(1);
              }

            g_qdexample.delay = (unsigned int)value;
            index += nargs;
            break;

          case 'r':
            g_qdexample.reset = true;
            index++;
            break;

          case 'h':
            qd_help();
            exit(EXIT_SUCCESS);

          default:
            printf("Unsupported option: %s\n", ptr);
            qd_help();
            exit(EXIT_FAILURE);
        }
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qd_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int qd_main(int argc, FAR char *argv[])
#endif
{
  qdec_data_s position;
  qdec_setting_s setting;
  int fd;
  int exitval = EXIT_SUCCESS;
  int ret;
#if defined(CONFIG_NSH_BUILTIN_APPS) || CONFIG_EXAMPLES_QDECODER_NSAMPLES > 0
  int nloops;
#endif

  /* Set the default values */

  qd_devpath(CONFIG_EXAMPLES_QDECODER_DEVPATH);

  /* Parse command line arguments */

#ifdef CONFIG_NSH_BUILTIN_APPS
  parse_args(argc, argv);
#endif

  /* Open the encoder device for reading */

  printf("qd_main: Hardware initialized. Opening the decoder device: %s\n",
         g_qdexample.devpath);

  fd = open(g_qdexample.devpath, O_RDWR);
  if (fd < 0)
    {
      printf("qd_main: open %s failed: %d\n", g_qdexample.devpath, errno);
      exitval = EXIT_FAILURE;
      goto errout_with_dev;
    }

  setting.sample_period = QDEC_SAMPLE_PERIOD_4096US;
  setting.dfilter_enable = true;
  ret = ioctl(fd, QDIOC_SETTING, (unsigned long)&setting);
  if (ret < 0)
    {
      printf("QDIOC_SETTING Cmd Fail\n");
      exitval = EXIT_FAILURE;
      goto errout_with_dev;
    }

  /* Now loop the appropriate number of times, displaying the collected
   * encoder samples.
   */

  ret = ioctl(fd, QDIOC_START, 0);
  if (ret < 0)
    {
      printf("Start Cmd Fail\n");
      exitval = EXIT_FAILURE;
      goto errout_with_dev;
    }

#if defined(CONFIG_NSH_BUILTIN_APPS)
  printf("qd_main: Number of samples: %u\n", g_qdexample.nloops);
  for (nloops = 0; nloops < g_qdexample.nloops; nloops++)
#elif CONFIG_EXAMPLES_QDECODER_NSAMPLES > 0
  printf("qd_main: Number of samples: %d\n", CONFIG_EXAMPLES_QENCODER_NSAMPLES);
  for (nloops = 0; nloops < CONFIG_EXAMPLES_QENCODER_NSAMPLES; nloops++)
#else
  for (;;)
#endif
    {
      /* Reset the count if so requested */

#ifdef CONFIG_NSH_BUILTIN_APPS
      if (g_qdexample.reset)
        {
          printf("qd_main: Resetting the count...\n");
          ret = ioctl(fd, QDIOC_RESET, 0);
          if (ret < 0)
            {
              printf("qd_main: ioctl(QDIOC_RESET) failed: %d\n", errno);
              exitval = EXIT_FAILURE;
              goto errout_with_dev;
            }
        }
#endif

      /* Delay a little bit */

#if defined(CONFIG_NSH_BUILTIN_APPS)
      printf("qe_main:1 delay %d second to wait decoder\n", g_qdexample.delay);
      usleep(g_qdexample.delay * 1000000);
#else
      printf("qe_main:2 delay %d second to wait decoder\n", CONFIG_EXAMPLES_QDECODER_DELAY);
      usleep(CONFIG_EXAMPLES_QDECODER_DELAY * 1000000);
#endif

      /* Get the positions data using the ioctl */

      ret = ioctl(fd, QDIOG_POSITION, (unsigned long)&position);
      if (ret < 0)
        {
          printf("qe_main: ioctl(QDIOG_POSITION) failed: %d\n", errno);
          exitval = EXIT_FAILURE;
          goto errout_with_dev;
        }

      /* Print the sample data on successful return */

      else
        {
#if defined(CONFIG_NSH_BUILTIN_APPS) || CONFIG_EXAMPLES_QDECODER_NSAMPLES > 0
          printf("qd_main: %3d. data:%d\n", nloops + 1, position.x);
#else
          printf("qd_main: data:%d\n", position.x );
#endif
        }

      ioctl(fd, QDIOC_RESET, 0);

      /* test decoder N sample command */
      printf("ioctl(QDIOC_DEC_CNT) : waiting for 100 sample\n");
      ret = ioctl(fd, QDIOC_DEC_CNT, 100);
      if (ret < 0)
        {
          printf("qe_main: ioctl(QDIOC_DEC_CNT) failed: %d\n", errno);
          exitval = EXIT_FAILURE;
          goto errout_with_dev;
        }

    }

  ret = ioctl(fd, QDIOC_STOP, 0);
  if (ret < 0)
    {
      printf("Stop Cmd Fail\n");
      exitval = EXIT_FAILURE;
      goto errout_with_dev;
    }

errout_with_dev:
  close(fd);

  printf("Terminating!\n");
  return exitval;
}

