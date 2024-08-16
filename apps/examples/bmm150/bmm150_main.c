/****************************************************************************
 * examples/bmm150/bmm150_main.c
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

#include <nuttx/sensors/internal_bmm150.h>
#include <nuttx/sensors/bmm150.h>
#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_NSH_BUILTIN_APPS - Build the BMM150 test as an NSH built-in function.
 *   Default: Built as a standalone program
 * CONFIG_EXAMPLES_BMM150_DEVPATH - The default path to the bmm150 device.Default: /dev/mag0
 * CONFIG_EXAMPLES_BMM150_NSAMPLES - If CONFIG_NSH_BUILTIN_APPS
 *   is defined, then the number of samples is provided manually on the command line
 *   and this value is ignored.  Otherwise, this number of samples is
 *   collected and the program terminates.  Default:  Samples are collected
 *   indefinitely.
 */

#ifndef CONFIG_BMM150
#  error "BMM150 device support is not enabled (CONFIG_BMM150)"
#endif

#ifndef CONFIG_EXAMPLES_BMM150_DEVPATH
#  define CONFIG_EXAMPLES_BMM150_DEVPATH "/dev/mag0"
#endif

#ifndef CONFIG_BMM150_USE_DRDY
#  define BMM150_DEF_SLEEP_TIME_MS 100
#endif


/* Use CONFIG_EXAMPLES_BMM150_NSAMPLES == 0 to mean to collect samples
 * indefinitely.
 */

#ifndef CONFIG_EXAMPLES_BMM150_NSAMPLES
#  define CONFIG_EXAMPLES_BMM150_NSAMPLES 0
#endif

#define BMM150_LOW_THRESHOLD 0
#define BMM150_HIGH_THRESHOLD 0


/****************************************************************************
 * Private Types
 ****************************************************************************/
struct bmm150_state_s
{
  bool                initialized;
  FAR char           *devpath;
#if defined(CONFIG_NSH_BUILTIN_APPS) || defined(CONFIG_EXAMPLES_BMM150_NSAMPLES)
  int                count;
#endif
  uint8_t                 high_threshold;
  bool                high_threshold_updated;
  uint8_t                 low_threshold;
  bool                low_threshold_updated;
  bool                signal_rcv;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct bmm150_state_s g_bmm150_state;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmm150_devpath
 ****************************************************************************/

static void bmm150_devpath(FAR struct bmm150_state_s *bmm150, FAR const char *devpath)
{
  /* Get rid of any old device path */

  if (bmm150->devpath)
    {
      free(bmm150->devpath);
    }

  /* Then set-up the new device path by copying the string */

  bmm150->devpath = strdup(devpath);
}


/****************************************************************************
 * Name: bmm150_help
 ****************************************************************************/

#ifdef CONFIG_NSH_BUILTIN_APPS
static void bmm150_help(FAR struct bmm150_state_s *bmm150)
{
  printf("Usage: bmm150 [OPTIONS]\n");
  printf("\nArguments are \"sticky\".  For example, once the bmm150 device is\n");
  printf("specified, that device will be re-used until it is changed.\n");
  printf("\n\"sticky\" OPTIONS include:\n");
  printf("  [-p devpath] selects the bmm150 device.  "
         "Default: %s Current: %s\n",
         CONFIG_EXAMPLES_BMM150_DEVPATH, bmm150->devpath ? bmm150->devpath : "NONE");
  printf("  [-n count] selects the samples to collect.  "
         "Default: %d Current: %d\n", CONFIG_EXAMPLES_BMM150_NSAMPLES, bmm150->count);
  printf("  [-t threshold] set high threshold.  "
         "Default: %d Current: %d\n", BMM150_LOW_THRESHOLD, bmm150->high_threshold);
  printf("  [-l threshold] set low threshold.  "
         "Default: %d Current: %d\n", BMM150_HIGH_THRESHOLD, bmm150->low_threshold);
  printf("  [-h] shows this message and exits\n");
}
#endif

static void handle_pin_interrupt(int signo)
{
  g_bmm150_state.signal_rcv = true;
}



static int bmm150_get_int_status(int fd)
{
  int ret;
  uint16_t int_status;

  ret = ioctl(fd, SNIOC_INT_STATUS, (unsigned long)&int_status);

  if (ret == OK)
    {
      printf("Interrupt status: \n");

      printf("LOW_THRESHOLD_X: %d\n", (int_status & BMM150_LOW_THRESHOLD_INT_X) ? true : false);
      printf("LOW_THRESHOLD_Y: %d\n", (int_status  & BMM150_LOW_THRESHOLD_INT_Y) ? true : false);
      printf("LOW_THRESHOLD_Z: %d\n", (int_status  & BMM150_LOW_THRESHOLD_INT_Z) ? true : false);
      printf("HIGH_THRESHOLD_X: %d\n", (int_status & BMM150_HIGH_THRESHOLD_INT_X) ? true : false);
      printf("HIGH_THRESHOLD_Y: %d\n", (int_status  & BMM150_HIGH_THRESHOLD_INT_Y) ? true : false);
      printf("HIGH_THRESHOLD_Z: %d\n", (int_status  & BMM150_HIGH_THRESHOLD_INT_Z) ? true : false);
      printf("DATA_OVERFLOW: %d\n", (int_status  & BMM150_DATA_OVERFLOW_INT) ? true : false);
      printf("DATA_OVERRUN: %d\n", (int_status  & BMM150_DATA_OVERRUN_INT) ? true : false);
      printf("DATA_READY: %d\n", (int_status  & BMM150_DATA_READY_INT) ? true : false);
    }

  return ret;
}

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
static void parse_args(FAR struct bmm150_state_s *bmm150, int argc, FAR char **argv)
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

            bmm150->count = (uint32_t)value;
            index += nargs;
            break;

          case 't':
            nargs = arg_string(&argv[index], &str);
            value = strtol(str, NULL, 10);
            if (value < 0)
              {
                printf("threshold must be non-negative: %ld\n", value);
                exit(1);
              }

            bmm150->high_threshold = (uint32_t)value;
            bmm150->high_threshold_updated = true;
            index += nargs;
            break;

          case 'l':
            nargs = arg_string(&argv[index], &str);
            value = strtol(str, NULL, 10);
            if (value < 0)
              {
                printf("threshold must be non-negative: %ld\n", value);
                exit(1);
              }

            bmm150->low_threshold = (uint32_t)value;
            bmm150->low_threshold_updated = true;
            index += nargs;
            break;



          case 'p':
            nargs = arg_string(&argv[index], &str);
            bmm150_devpath(bmm150, str);
            index += nargs;
            break;

          case 'h':
            bmm150_help(bmm150);
            exit(0);

          default:
            printf("Unsupported option: %s\n", ptr);
            bmm150_help(bmm150);
            exit(1);
        }
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmm150_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int bmm150_main(int argc, char *argv[])
#endif
{
  int fd;
  int ret;
  uint i;
  struct bmm150_mag_data mag_data;
  uint32_t chip_id = 0;
  struct sigaction act;
  uint8_t int_sel;


  /* Check if we have initialized */

  if (!g_bmm150_state.initialized)
    {
      /* Initialization of the bmm150 hardware must be performed by board-specific
       * logic prior to running this test.
       */


      /* Set the default values */

      bmm150_devpath(&g_bmm150_state, CONFIG_EXAMPLES_BMM150_DEVPATH);

#if CONFIG_EXAMPLES_BMM150_NSAMPLES > 0
      g_bmm150_state.count = CONFIG_EXAMPLES_BMM150_NSAMPLES;
#else
      g_bmm150_state.count = 1;
#endif

      g_bmm150_state.high_threshold = BMM150_HIGH_THRESHOLD;
      g_bmm150_state.high_threshold_updated = true;
      g_bmm150_state.high_threshold = BMM150_LOW_THRESHOLD;
      g_bmm150_state.low_threshold_updated = true;

      g_bmm150_state.initialized = true;
      g_bmm150_state.signal_rcv = false;
    }

  /* Parse the command line */

#ifdef CONFIG_NSH_BUILTIN_APPS
  parse_args(&g_bmm150_state, argc, argv);
#endif

  /* If this example is configured as an NX add-on, then limit the number of
   * samples that we collect before returning.  Otherwise, we never return
   */

#if defined(CONFIG_NSH_BUILTIN_APPS) || CONFIG_EXAMPLES_BMM150_NSAMPLES > 0
  printf("bmm150_main: g_bmm150_state.count: %d\n", g_bmm150_state.count);
#endif

  /* Open the comp device for reading */

  printf("bmm150_main: Hardware initialized. Opening the bmm150 device: %s\n",
         g_bmm150_state.devpath);

  fd = open(g_bmm150_state.devpath, O_RDONLY);
  if (fd < 0)
    {
      printf("bmm150_main: open %s failed: %d\n", g_bmm150_state.devpath, errno);
      goto errout;
    }

  /* check sensor id */
  ret = ioctl(fd, SNIOC_READID, (unsigned long)&chip_id);
  if (ret < 0)
    {
      int errcode = errno;
      printf("bmm150_main: SNIOC_READID ioctl failed: %d\n", errcode);
      goto errout_with_dev;
    }
  else
    {
      printf("bmm150_main: read sensor ID:%d \n", chip_id);
    }


  if (chip_id != BMM150_CHIP_ID)
    {
      printf("bmm150_main: sensor ID check failed! \n");
      goto errout_with_dev;
    }

  /* register to receive interrupt */
  ret = ioctl(fd, SNIOC_GA_REGISTER_INT, SIGUSR1);
  if (ret < 0)
    {
      int errcode = errno;
      printf("bmm150_main: SNIOC_GA_REGISTER_INT failed: %d\n", errcode);
      goto errout_with_dev;
    }

  /* Set up so that siguser_action will respond to SIGUSR1*/

  memset(&act, 0, sizeof(struct sigaction));
  act.sa_u._sa_handler = handle_pin_interrupt;
  act.sa_flags     = 0;

  ret = sigaction(SIGUSR1, &act, NULL);
  if (ret != OK)
    {
      printf("Failed to install SIGUSR1 handler, errno=%d\n", errno);
      goto errout_with_dev;
    }


  if (g_bmm150_state.high_threshold_updated == true)
    {
      /* set high threshold */
      ret = ioctl(fd, SNIOC_HIGH_THRESHOLD, (uint32_t)&g_bmm150_state.high_threshold);
      if (ret < 0)
        {
          int errcode = errno;
          printf("bmm150_main: SNIOC_HIGH_THRESHOLD failed: %d\n", errcode);
          goto errout_with_dev;
        }

      g_bmm150_state.high_threshold_updated = false;
    }

  if (g_bmm150_state.low_threshold_updated == true)
    {

      /* set low threshold */
      ret = ioctl(fd, SNIOC_LOW_THRESHOLD, (uint32_t)&g_bmm150_state.low_threshold);
      if (ret < 0)
        {
          int errcode = errno;
          printf("bmm150_main: SNIOC_HIGH_THRESHOLD failed: %d\n", errcode);
          goto errout_with_dev;
        }
      g_bmm150_state.low_threshold_updated = false;
    }

  int_sel = BMM150_HIGH_X | BMM150_HIGH_Y | BMM150_HIGH_Z | BMM150_LOW_X | BMM150_LOW_Y | BMM150_LOW_Z;

  /* set high threshold */
  ret = ioctl(fd, SNIOC_INT_ENABLE, (uint32_t)&int_sel);
  if (ret < 0)
    {
      int errcode = errno;
      printf("bmm150_main: SNIOC_INT_ENABLE failed: %d\n", errcode);
      goto errout_with_dev;
    }

  /* start bmm150 device,use low power presetmode */
  ret = ioctl(fd, SNIOC_START, 0);
  if (ret < 0)
    {
      int errcode = errno;
      printf("bmm150_main: SNIOC_START ioctl failed: %d\n", errcode);
      goto errout_with_dev;
    }

#if defined(CONFIG_NSH_BUILTIN_APPS) || (CONFIG_EXAMPLES_BMM150_NSAMPLES > 0)
  for (i = g_bmm150_state.count; i > 0; i--)
#else
  UNUSED(i);
  for (;;)
#endif
    {
      /* Flush any output before the loop entered or from the previous pass
       * through the loop.
       */

      fflush(stdout);

#ifndef CONFIG_BMM150_USE_DRDY
      usleep(BMM150_DEF_SLEEP_TIME_MS * USEC_PER_MSEC);
#endif


      /* read magmeter data*/
      ret = read(fd, &mag_data, sizeof(struct bmm150_mag_data));
      if (ret <= 0)
        {
          int errcode = errno;
          printf("bmm150_main: BMM150 read failed: %d\n", errcode);
          goto errout_with_dev;
        }
      else
        {
#ifdef CONFIG_BMM150_USE_DRDY
          printf("bmm150_main: Interrupt received from DRDY pin! \n");
#endif

#ifdef CONFIG_BMM150_FLOAT_ENABLE
          printf("bmm150_main: X : %0.2f \t Y : %0.2f \t Z : %0.2f \n",
                 mag_data.x, mag_data.y, mag_data.z);
#else
          printf("bmm150_main: X : %d \t Y : %d \t Z : %d \n",
                 mag_data.x, mag_data.y, mag_data.z);
#endif

        }

      if (g_bmm150_state.signal_rcv)
        {
          g_bmm150_state.signal_rcv = false;

          printf("bmm150_main: interrupt received from INT pin!\n");
          /* print interrupt status */
          ret = bmm150_get_int_status(fd);

          if (ret != OK)
            {
              int errcode = errno;
              printf("bmm150_main: SNIOC_INT_STATUS failed: %d\n", errcode);
              goto errout_with_dev;
            }
        }
      else
        {
          printf("bmm150_main: NO interrupt received!\n");
        }

    }

  printf("Terminating!\n");
  close(fd);
  return OK;

  /* Error exits */

errout_with_dev:
  printf("Failed with device operation!\n");
  close(fd);

errout:
  fflush(stdout);
  return 0;
}
