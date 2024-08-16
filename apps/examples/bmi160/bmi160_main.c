/****************************************************************************
 * examples/bmi160/bmi160_main.c
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

#include <nuttx/sensors/bmi160.h>


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_NSH_BUILTIN_APPS - Build the BMI160 test as an NSH built-in function.
 *  Default: Built as a standalone program
 * CONFIG_EXAMPLES_BMI160_DEVPATH - The default path to the COMP device.Default: /dev/bmi160
 * CONFIG_EXAMPLES_BMI160_NSAMPLES - If CONFIG_NSH_BUILTIN_APPS
 *   is defined, then the number of samples is provided manually on the command line
 *   and this value is ignored.  Otherwise, this number of samples is
 *   collected and the program terminates.  Default:  Samples are collected
 *   indefinitely.
 */

#ifndef CONFIG_BMI160
#  error "BMI160 device support is not enabled (CONFIG_BMI160)"
#endif

#ifndef CONFIG_EXAMPLES_BMI160_DEVPATH
#  define CONFIG_EXAMPLES_BMI160_DEVPATH "/dev/bmi160"
#endif

/* Use CONFIG_EXAMPLES_BMI160_NSAMPLES == 0 to mean to collect samples
 * indefinitely.
 */

#ifndef CONFIG_EXAMPLES_BMI160_NSAMPLES
#  define CONFIG_EXAMPLES_BMI160_NSAMPLES 0
#endif

#define INT_NO_TO_STR(INT)   (INT == BMI160_INT_CHANNEL_1 ? "CHANNEL_1" :"CHANNEL_2")


/****************************************************************************
 * Private Types
 ****************************************************************************/
struct bmi160_state_s
{
  bool                initialized;
  FAR char           *devpath;
#if defined(CONFIG_NSH_BUILTIN_APPS) || defined(CONFIG_EXAMPLES_BMI160_NSAMPLES)
  int                count;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct bmi160_state_s g_bmi160_state;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmi160_devpath
 ****************************************************************************/

static void bmi160_devpath(FAR struct bmi160_state_s *bmi160, FAR const char *devpath)
{
  /* Get rid of any old device path */

  if (bmi160->devpath)
    {
      free(bmi160->devpath);
    }

  /* Then set-up the new device path by copying the string */

  bmi160->devpath = strdup(devpath);
}


/****************************************************************************
 * Name: bmi160_help
 ****************************************************************************/

#ifdef CONFIG_NSH_BUILTIN_APPS
static void bmi160_help(FAR struct bmi160_state_s *bmi160)
{
  printf("Usage: bmi160 [OPTIONS]\n");
  printf("\nArguments are \"sticky\".  For example, once the bmi160 device is\n");
  printf("specified, that device will be re-used until it is changed.\n");
  printf("\n\"sticky\" OPTIONS include:\n");
  printf("  [-p devpath] selects the bmi160 device.  "
         "Default: %s Current: %s\n",
         CONFIG_EXAMPLES_BMI160_DEVPATH, bmi160->devpath ? bmi160->devpath : "NONE");
  printf("  [-n count] selects the samples to collect.  "
         "Default: %d Current: %d\n", CONFIG_EXAMPLES_BMI160_NSAMPLES, bmi160->count);
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
static void parse_args(FAR struct bmi160_state_s *bmi160, int argc, FAR char **argv)
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

            bmi160->count = (uint32_t)value;
            index += nargs;
            break;

          case 'p':
            nargs = arg_string(&argv[index], &str);
            bmi160_devpath(bmi160, str);
            index += nargs;
            break;

          case 'h':
            bmi160_help(bmi160);
            exit(0);

          default:
            printf("Unsupported option: %s\n", ptr);
            bmi160_help(bmi160);
            exit(1);
        }
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmi160_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int bmi160_main(int argc, char *argv[])
#endif
{
  int fd;
  int errval = 0;
  int ret;
  struct timespec ts;
  sigset_t set;
  struct siginfo info;
  uint i;
  sn_ga_raw_s raw;
  uint32_t chip_id = 0;

  /* Check if we have initialized */

  if (!g_bmi160_state.initialized)
    {
      /* Initialization of the bmi160 hardware must be performed by board-specific
       * logic prior to running this test.
       */


      /* Set the default values */

      bmi160_devpath(&g_bmi160_state, CONFIG_EXAMPLES_BMI160_DEVPATH);

#if CONFIG_EXAMPLES_BMI160_NSAMPLES > 0
      g_bmi160_state.count = CONFIG_EXAMPLES_BMI160_NSAMPLES;
#else
      g_bmi160_state.count = 1;
#endif
      g_bmi160_state.initialized = true;
    }

  /* Parse the command line */

#ifdef CONFIG_NSH_BUILTIN_APPS
  parse_args(&g_bmi160_state, argc, argv);
#endif

  /* If this example is configured as an NX add-on, then limit the number of
   * samples that we collect before returning.  Otherwise, we never return
   */

#if defined(CONFIG_NSH_BUILTIN_APPS) || CONFIG_EXAMPLES_BMI160_NSAMPLES > 0
  printf("bmi160_main: g_bmi160_state.count: %d\n", g_bmi160_state.count);
#endif

  /* Open the comp device for reading */

  printf("bmi160_main: Hardware initialized. Opening the bmi160 device: %s\n",
         g_bmi160_state.devpath);

  fd = open(g_bmi160_state.devpath, O_RDONLY);
  if (fd < 0)
    {
      printf("bmi160_main: open %s failed: %d\n", g_bmi160_state.devpath, errno);
      errval = 2;
      goto errout;
    }

  /* check sensor id */
  ret = ioctl(fd, SNIOC_READID, (unsigned long)&chip_id);
  if (ret < 0)
    {
      int errcode = errno;
      printf("bmi160_main: SNIOC_READID ioctl failed: %d\n", errcode);
      errval = 3;
      goto errout_with_dev;
    }
  else
    {
      printf("bmi160_main: read sensor ID:%d \n", chip_id);
    }

  if (chip_id != BMI160_CHIP_ID)
    {
      printf("bmi160_main: sensor ID check failed! \n");
      errval = 4;
      goto errout_with_dev;
    }


  /* put accelerometer to low power mode */
  sn_ga_param_s cfg_accl =
  {
    /*! power mode */
    .power_mode = GA_MODE_LOWPOWER,
    /*! output data rate */
    .odr = 500,
    /*! range */
    .range = 4,
  };

  /* set the accelemeter configration */
  ret = ioctl(fd, SNIOC_A_SPARAM, (unsigned long)&cfg_accl);
  if (ret < 0)
    {
      int errcode = errno;
      printf("bmi160_main: SNIOC_A_SPARAM ioctl failed: %d\n", errcode);
      errval = 5;
      goto errout_with_dev;
    }
  else
    {
      printf("bmi160_main: put accelemeter to low power mode\n");
    }

  /* set the accelemeter bandwidth */
  ret = ioctl(fd, SNIOC_ACCEL_SETBW, (unsigned long)BMI160_ACCEL_BW_OSR4_AVG1);
  if (ret < 0)
    {
      int errcode = errno;
      printf("bmi160_main: SNIOC_ACCEL_SETBW ioctl failed: %d\n", errcode);
      errval = 6;
      goto errout_with_dev;
    }
  else
    {
      printf("bmi160_main: set accelemeter bandwidth\n");
    }

  /* put gyro to suspended mode */
  sn_ga_param_s cfg_gyro =
  {
    /*! power mode */
    .power_mode = GA_MODE_SUSPEND,
    /*! output data rate */
    .odr = 32000,
    /*! range */
    .range = 2000,
  };

  /* set the gyro configration */
  ret = ioctl(fd, SNIOC_G_SPARAM, (unsigned long)&cfg_gyro);
  if (ret < 0)
    {
      int errcode = errno;
      printf("bmi160_main: SNIOC_G_SPARAM ioctl failed: %d\n", errcode);
      errval = 7;
      goto errout_with_dev;
    }
  else
    {
      printf("bmi160_main: put gyro to suspended mode\n");
    }

  /* set the gyro bandwidth */
  ret = ioctl(fd, SNIOC_GYRO_SETBW, (unsigned long)BMI160_GYRO_BW_NORMAL_MODE);
  if (ret < 0)
    {
      int errcode = errno;
      printf("bmi160_main: SNIOC_GYRO_SETBW ioctl failed: %d\n", errcode);
      errval = 8;
      goto errout_with_dev;
    }
  else
    {
      printf("bmi160_main: set gyro bandwidth\n");
    }

  struct bmi160_int_settg bmi160_int1_cfg =
  {
    .int_channel = BMI160_INT_CHANNEL_1,
    .int_type = BMI160_ACC_SIG_MOTION_INT,
    .int_pin_settg =
    {
      .output_en = BMI160_ENABLE,
      .output_mode = BMI160_DISABLE,
      .output_type = BMI160_ENABLE,
      .edge_ctrl = BMI160_ENABLE,
      .input_en = BMI160_DISABLE,
      .latch_dur = BMI160_LATCH_DUR_40_MILLI_SEC,
    },
    .int_type_cfg.acc_sig_motion_int =
    {
      .sig_mot_skip = 1,
      .sig_mot_proof = 3,
      .sig_data_src = 0,
      .sig_en = BMI160_ENABLE,
      .sig_mot_thres = 50,
    },
    .fifo_full_int_en = BMI160_DISABLE,
    .fifo_WTM_int_en = BMI160_DISABLE,
  };

  struct bmi160_int_settg bmi160_int2_cfg =
  {
    .int_channel = BMI160_INT_CHANNEL_2,
    .int_type = BMI160_ACC_SLOW_NO_MOTION_INT,
    .int_pin_settg =
    {
      .output_en = BMI160_ENABLE,
      .output_mode = BMI160_DISABLE,
      .output_type = BMI160_ENABLE,
      .edge_ctrl = BMI160_ENABLE,
      .input_en = BMI160_DISABLE,
      .latch_dur = BMI160_LATCH_DUR_40_MILLI_SEC,
    },
    .int_type_cfg.acc_no_motion_int =
    {
      .no_motion_x  = BMI160_ENABLE,
      .no_motion_y  = BMI160_ENABLE,
      .no_motion_z  = BMI160_ENABLE,
      .no_motion_dur = 5,
      .no_motion_sel = BMI160_ENABLE,
      .no_motion_src = 0,
      .no_motion_thres = 100,
    },
    .fifo_full_int_en = BMI160_DISABLE,
    .fifo_WTM_int_en = BMI160_DISABLE,
  };

  /* config interrupt source */
  ret = ioctl(fd, SNIOC_GA_CONFIG_ISR, (unsigned long)&bmi160_int1_cfg);
  if (ret < 0)
    {
      int errcode = errno;
      printf("bmi160_main: BMI160_CONFIG_INT ioctl failed: %d\n", errcode);
      errval = 9;
      goto errout_with_dev;
    }
  else
    {
      printf("bmi160_main: configurated INT1 source\n");
    }

  ret = ioctl(fd, SNIOC_GA_CONFIG_ISR, (unsigned long)&bmi160_int2_cfg);
  if (ret < 0)
    {
      int errcode = errno;
      printf("bmi160_main: BMI160_CONFIG_INT ioctl failed: %d\n", errcode);
      errval = 10;
      goto errout_with_dev;
    }
  else
    {
      printf("bmi160_main: configurated INT2 source\n");
    }


  /* register to receive interrupt */
  ret = ioctl(fd, SNIOC_GA_REGISTER_INT, SIGUSR1);
  if (ret < 0)
    {
      int errcode = errno;
      printf("bmi160_main: BMI160_REGISTER_INT failed: %d\n", errcode);
      errval = 11;
      goto errout_with_dev;
    }
  else
    {
      printf("bmi160_main: register to receive signo from interrupt, signo:%d\n", SIGUSR1);
    }

  /* prepare the signal set */
  (void)sigemptyset(&set);
  (void)sigaddset(&set, SIGUSR1);

  /* Now loop the appropriate number of times.
   */

#if defined(CONFIG_NSH_BUILTIN_APPS) || (CONFIG_EXAMPLES_BMI160_NSAMPLES > 0)
  for (i = g_bmi160_state.count; i > 0; i--)
#else
  UNUSED(i);
  for (;;)
#endif
    {
      /* Flush any output before the loop entered or from the previous pass
       * through the loop.
       */

      fflush(stdout);

      /* wait for 10 seconds to receive signo */

      ts.tv_sec  = 10;
      ts.tv_nsec = 0;

      ret = sigtimedwait(&set, &info, &ts);

      if (ret < 0)
        {
          int errcode = errno;
          if (errcode == EAGAIN)
            {
              /* 5 seconds passed and we didn't receive the signo, and we just continue*/
              printf("bmi160_main:   [10 seconds timeout with no signal]\n");
            }
          else
            {
              printf("bmi160_main: signal wait failed: %d\n", errcode);
              errval = 12;
              goto errout_with_dev;
            }
        }
      else
        {
          printf("bmi160_main: interrupt received on %s\n", INT_NO_TO_STR(info.si_value.sival_int));
          /* read raw data */
          ret = ioctl(fd, SNIOC_A_GRAW, (unsigned long)&raw);
          if (ret < 0)
            {
              int errcode = errno;
              printf("bmi160_main: SNIOC_A_GRAW failed: %d\n", errcode);
              errval = 13;
              goto errout_with_dev;
            }
          else
            {
              printf("bmi160_main: get accelerometer raw data, X:%d,Y:%d,Z:%d\n",
                     raw.x_axis, raw.y_axis, raw.z_axis);
            }

        }
    }


  /* unregister*/
  ret = ioctl(fd, SNIOC_GA_UNREGISTER_INT, 0);
  if (ret < 0)
    {
      int errcode = errno;
      printf("bmi160_main: BMI160_UNREGISTER_INT ioctl failed: %d\n", errcode);
      errval = 14;
      goto errout_with_dev;
    }
  else
    {
      printf("bmi160_main: unregister signal, signo:%d\n", SIGUSR1);
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
