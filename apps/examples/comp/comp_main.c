/****************************************************************************
 *   examples/comp/comp_main.c
 *
 *   Copyright (C) 2011-2012, 2015, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Zhengwei Wang<zhengwei@zglue.com>
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
#include <errno.h>
#include <debug.h>

#include <nuttx/analog/comp.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_NSH_BUILTIN_APPS - Build the COMP test as an NSH built-in function.
 *  Default: Built as a standalone program
 * CONFIG_EXAMPLES_COMP_DEVPATH - The default path to the COMP device. Default: /dev/comp
 * CONFIG_EXAMPLES_COMP_NSAMPLES - If CONFIG_NSH_BUILTIN_APPS
 *   is defined, then the number of samples is provided manually on the command line
 *   and this value is ignored.  Otherwise, this number of samples is
 *   collected and the program terminates.  Default:  Samples are collected
 *   indefinitely.
 */

#ifndef CONFIG_COMP
#  error "COMP device support is not enabled (CONFIG_COMP)"
#endif

#ifndef CONFIG_EXAMPLES_COMP_DEVPATH
#ifdef CONFIG_NRF52_COMP
#  define CONFIG_EXAMPLES_COMP_DEVPATH "/dev/comp"
#else
#  define CONFIG_EXAMPLES_COMP_DEVPATH "/dev/lpcomp"
#endif
#endif

/* Use CONFIG_EXAMPLES_COMP_NSAMPLES == 0 to mean to collect samples
 * indefinitely.
 */

#ifndef CONFIG_EXAMPLES_COMP_NSAMPLES
#  define CONFIG_EXAMPLES_COMP_NSAMPLES 0
#endif


/****************************************************************************
 * Private Types
 ****************************************************************************/
struct comp_state_s
{
  bool                initialized;
  FAR char           *devpath;
#if defined(CONFIG_NSH_BUILTIN_APPS) || defined(CONFIG_EXAMPLES_COMP_NSAMPLES)
  int                count;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct comp_state_s g_comp_state;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: comp_devpath
 ****************************************************************************/

static void comp_devpath(FAR struct comp_state_s *comp, FAR const char *devpath)
{
  /* Get rid of any old device path */

  if (comp->devpath)
    {
      free(comp->devpath);
    }

  /* Then set-up the new device path by copying the string */

  comp->devpath = strdup(devpath);
}


/****************************************************************************
 * Name: comp_help
 ****************************************************************************/

#ifdef CONFIG_NSH_BUILTIN_APPS
static void comp_help(FAR struct comp_state_s *comp)
{
  printf("Usage: comp [OPTIONS]\n");
  printf("\nArguments are \"sticky\".  For example, once the comp device is\n");
  printf("specified, that device will be re-used until it is changed.\n");
  printf("\n\"sticky\" OPTIONS include:\n");
  printf("  [-p devpath] selects the COMP device.  "
         "Default: %s Current: %s\n",
         CONFIG_EXAMPLES_COMP_DEVPATH, g_comp_state.devpath ? g_comp_state.devpath : "NONE");
  printf("  [-n count] selects the samples to collect.  "
         "Default: %d Current: %d\n", CONFIG_EXAMPLES_COMP_NSAMPLES, comp->count);
  printf("  [-h] shows this message and exits\n");
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
 * Name: parse_args
 ****************************************************************************/

#ifdef CONFIG_NSH_BUILTIN_APPS
static void parse_args(FAR struct comp_state_s *comp, int argc, FAR char **argv)
{
  FAR char *ptr;
  FAR char *str;
  long value;
  int index;
  int nargs;

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
            nargs = arg_string(&argv[index], &str);
            value = strtol(str, NULL, 10);
            if (value < 0)
              {
                printf("Count must be non-negative: %ld\n", value);
                exit(1);
              }

            comp->count = (uint32_t)value;
            index += nargs;
            break;

          case 'p':
            nargs = arg_string(&argv[index], &str);
            comp_devpath(comp, str);
            index += nargs;
            break;

          case 'h':
            comp_help(comp);
            exit(0);

          default:
            printf("Unsupported option: %s\n", ptr);
            comp_help(comp);
            exit(1);
        }
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: comp_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int comp_main(int argc, char *argv[])
#endif
{
  int fd;
  int errval = 0;
  int ret;
  int result;
  uint i;

  /* Check if we have initialized */

  if (!g_comp_state.initialized)
    {
      /* Initialization of the comp hardware must be performed by board-specific
       * logic prior to running this test.
       */


      /* Set the default values */

      comp_devpath(&g_comp_state, CONFIG_EXAMPLES_COMP_DEVPATH);

#if CONFIG_EXAMPLES_COMP_NSAMPLES > 0
      g_comp_state.count = CONFIG_EXAMPLES_COMP_NSAMPLES;
#else
      g_comp_state.count = 1;
#endif
      g_comp_state.initialized = true;
    }

  /* Parse the command line */

#ifdef CONFIG_NSH_BUILTIN_APPS
  parse_args(&g_comp_state, argc, argv);
#endif

  /* If this example is configured as an NX add-on, then limit the number of
   * samples that we collect before returning.  Otherwise, we never return
   */

#if defined(CONFIG_NSH_BUILTIN_APPS) || CONFIG_EXAMPLES_COMP_NSAMPLES > 0
  printf("comp_main: g_comp_state.count: %d\n", g_comp_state.count);
#endif

  /* Open the comp device for reading */

  printf("comp_main: Hardware initialized. Opening the comp device: %s\n",
         g_comp_state.devpath);

  fd = open(g_comp_state.devpath, O_RDONLY);
  if (fd < 0)
    {
      printf("comp_main: open %s failed: %d\n", g_comp_state.devpath, errno);
      errval = 2;
      goto errout;
    }

  /* Now loop the appropriate number of times.
   */

#if defined(CONFIG_NSH_BUILTIN_APPS) || (CONFIG_EXAMPLES_COMP_NSAMPLES > 0)
  for (i = g_comp_state.count; i > 0; i--)
#else
  UNUSED(i);
  for (;;)
#endif
    {
      /* Flush any output before the loop entered or from the previous pass
       * through the loop.
       */

      fflush(stdout);

      /* read the comp current status */

      ret = read(fd, &result, sizeof(result));
      if (ret < 0)
        {
          int errcode = errno;
          printf("comp_main: read failed: %d\n", errcode);
          errval = 8;
          goto errout_with_dev;

        }

      printf("comp_main: get comparator current status:%d\n", result);

      sleep(1);
    }


  close(fd);
  return OK;

  /* Error exits */

errout_with_dev:
  close(fd);

errout:
  printf("Terminating!\n");
  fflush(stdout);
  return errval;
}
