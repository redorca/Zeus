/****************************************************************************
 * examples/lsm6ds3/lsm6ds3_main.c
 *
 *   Copyright (C) 2018 zGlue Inc. All rights reserved.
 *   Author: Zhengwei Wang <zhengwei@zglue.com>
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

#include <nuttx/sensors/lsm6ds3.h>


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_NSH_BUILTIN_APPS - Build the LSM6DS3 test as an NSH built-in function.
 *  Default: Built as a standalone program
 * CONFIG_EXAMPLES_LSM6DS3_DEVPATH - The default path to the iNEMO inertial module.
     Default: /dev/lsm6ds3
 * CONFIG_EXAMPLES_LSM6DS3_NSAMPLES - If CONFIG_NSH_BUILTIN_APPS
 *   is defined, then the number of samples is provided manually on the command line
 *   and this value is ignored.  Otherwise, this number of samples is
 *   collected and the program terminates.  Default:  Samples are collected
 *   indefinitely.
 */

#ifndef CONFIG_LSM6DS3
#  error "LSM6DS3 device support is not enabled (CONFIG_LSM6DS3)"
#endif

#ifndef CONFIG_EXAMPLES_LSM6DS3_DEVPATH
#  define CONFIG_EXAMPLES_LSM6DS3_DEVPATH "/dev/lsm6ds3"
#endif

/* Use CONFIG_EXAMPLES_LSM6DS3_NSAMPLES == 0 to mean to collect samples
 * indefinitely.
 */

#ifndef CONFIG_EXAMPLES_LSM6DS3_NSAMPLES
#  define CONFIG_EXAMPLES_LSM6DS3_NSAMPLES 0
#endif


/****************************************************************************
 * Private Types
 ****************************************************************************/
struct lsm6ds3_state_s
{
  bool                initialized;
  FAR char           *devpath;
#if defined(CONFIG_NSH_BUILTIN_APPS) || defined(CONFIG_EXAMPLES_LSM6DS3_NSAMPLES)
  int                count;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lsm6ds3_state_s g_lsm6ds3_state;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm6ds3_devpath
 ****************************************************************************/

static void lsm6ds3_devpath(FAR struct lsm6ds3_state_s *lsm6ds3, FAR const char *devpath)
{
  /* Get rid of any old device path */

  if (lsm6ds3->devpath)
    {
      free(lsm6ds3->devpath);
    }

  /* Then set-up the new device path by copying the string */

  lsm6ds3->devpath = strdup(devpath);
}


/****************************************************************************
 * Name: lsm6ds3_help
 ****************************************************************************/

#ifdef CONFIG_NSH_BUILTIN_APPS
static void lsm6ds3_help(FAR struct lsm6ds3_state_s *lsm6ds3)
{
  printf("Usage: lsm6ds3 [OPTIONS]\n");
  printf("\nArguments are \"sticky\".  For example, once the lsm6ds3 device is\n");
  printf("specified, that device will be re-used until it is changed.\n");
  printf("\n\"sticky\" OPTIONS include:\n");
  printf("  [-p devpath] selects the lsm6ds3 device.  "
         "Default: %s Current: %s\n",
         CONFIG_EXAMPLES_LSM6DS3_DEVPATH, lsm6ds3->devpath ? lsm6ds3->devpath : "NONE");
  printf("  [-n count] selects the samples to collect.  "
         "Default: %d Current: %d\n", CONFIG_EXAMPLES_LSM6DS3_NSAMPLES, lsm6ds3->count);
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
static void parse_args(FAR struct lsm6ds3_state_s *lsm6ds3, int argc, FAR char **argv)
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

            lsm6ds3->count = (uint32_t)value;
            index += nargs;
            break;

          case 'p':
            nargs = arg_string(&argv[index], &str);
            lsm6ds3_devpath(lsm6ds3, str);
            index += nargs;
            break;

          case 'h':
            lsm6ds3_help(lsm6ds3);
            exit(0);

          default:
            printf("Unsupported option: %s\n", ptr);
            lsm6ds3_help(lsm6ds3);
            exit(1);
        }
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm6ds3_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int lsm6ds3_main(int argc, char *argv[])
#endif
{
  int fd;
  int errval = 0;
  int ret;
  uint32_t chip_id = 0;

  /* Check if we have initialized */

  if (!g_lsm6ds3_state.initialized)
    {
      /* Initialization of the lsm6ds3 hardware must be performed by board-specific
       * logic prior to running this test.
       */


      /* Set the default values */

      lsm6ds3_devpath(&g_lsm6ds3_state, CONFIG_EXAMPLES_LSM6DS3_DEVPATH);

#if CONFIG_EXAMPLES_LSM6DS3_NSAMPLES > 0
      g_lsm6ds3_state.count = CONFIG_EXAMPLES_LSM6DS3_NSAMPLES;
#else
      g_lsm6ds3_state.count = 1;
#endif
      g_lsm6ds3_state.initialized = true;
    }

  /* Parse the command line */

#ifdef CONFIG_NSH_BUILTIN_APPS
  parse_args(&g_lsm6ds3_state, argc, argv);
#endif

  /* If this example is configured as an NX add-on, then limit the number of
   * samples that we collect before returning.  Otherwise, we never return
   */

#if defined(CONFIG_NSH_BUILTIN_APPS) || CONFIG_EXAMPLES_LSM6DS3_NSAMPLES > 0
  printf("lsm6ds3_main: g_lsm6ds3_state.count: %d\n", g_lsm6ds3_state.count);
#endif

  /* Open the comp device for reading */

  printf("lsm6ds3_main: Hardware initialized. Opening the lsm6ds3 device: %s\n",
         g_lsm6ds3_state.devpath);

  fd = open(g_lsm6ds3_state.devpath, O_RDONLY);
  if (fd < 0)
    {
      printf("lsm6ds3_main: open %s failed: %d\n", g_lsm6ds3_state.devpath, errno);
      errval = 2;
      goto errout;
    }

  /* check sensor id */
  ret = ioctl(fd, SNIOC_READID, (unsigned long)&chip_id);
  if (ret < 0)
    {
      int errcode = errno;
      printf("lsm6ds3_main: SNIOC_READID ioctl failed: %d\n", errcode);
      errval = 3;
      goto errout_with_dev;
    }
  else
    {
      printf("lsm6ds3_main: read sensor ID:%d \n", chip_id);
    }

  if (chip_id != LSM6DS3_ACC_GYRO_WHO_AM_I)
    {
      printf("lsm6ds3_main: sensor ID check failed! \n");
      errval = 4;
      goto errout_with_dev;
    }

  /* Flush any output before the loop entered or from the previous pass
   * through the loop.
   */

  fflush(stdout);

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
