/****************************************************************************
 * examples/AI/AI_main.c
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

#include <nuttx/sensors/mc3672.h>
#include <nuttx/sensors/lsm6ds3.h>
#include <nuttx/sensors/iCE40UP.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_NSH_BUILTIN_APPS - Build the AI test as an NSH built-in function.
 *  Default: Built as a standalone program
 * CONFIG_AI_MOTION_MC3672 - Select MC3672 as the motion detect sensor
 * CONFIG_AI_MOTION_LSM6DS3 - Select LSM6DS3 as the motion detect sensor
 */

#ifndef CONFIG_ICE40UP
#  error "AI device support is not enabled (CONFIG_ICE40UP)"
#endif

#if defined CONFIG_AI_MOTION_MC3672 && !defined CONFIG_MC3672
#  error "Driver for mc3672 is not enabled, can't be select as the motion detection sensor!"
#endif

#if defined CONFIG_AI_MOTION_LSM6DS3 && !defined CONFIG_MC3672
#  error "Driver for mc3672 is not enabled, can't be select as the motion detection sensor!"
#endif

#ifdef CONFIG_AI_MOTION_MC3672
#define MOTION_SENSOR_PATH "/dev/accel1"
#elif CONFIG_AI_MOTION_LSM6DS3
#define MOTION_SENSOR_PATH "/dev/lsm6ds3"
#endif

#define MC3672_ACCL_ODR 140   /*14HZ*/
#define LSM6DS3_ACCL_ODR 4160 /*416HZ*/

#ifdef CONFIG_AI_MOTION_MC3672
#define MOTION_DETECTION_SENSOR_ODR MC3672_ACCL_ODR
#define MOTION_DETECTION_SENSOR_RANGE 2 /*2g*/
#define MOTION_DETECTION_DEBOUNCE  20
#elif CONFIG_AI_MOTION_LSM6DS3
#define MOTION_DETECTION_SENSOR_ODR LSM6DS3_ACCL_ODR
#define MOTION_DETECTION_SENSOR_RANGE 4 /*4g*/
#define MOTION_DETECTION_DEBOUNCE  600
#endif


#define MOTION_DETECTION_SENSOR_RESO 12
#define MOTION_DETECTION_SENSOR_MAX_VALUE ((1<<(MOTION_DETECTION_SENSOR_RESO -1)) - 1)
#define MOTION_DETECTION_THRESHOLD (0.6*(MOTION_DETECTION_SENSOR_MAX_VALUE / MOTION_DETECTION_SENSOR_RANGE))
#define SLEEP_PERIOD               100*1000L /*100ms*/

/****************************************************************************
 * Private Types
 ****************************************************************************/
enum
{
  BEL_AI_STATUS_INTI = 0,
  BEL_AI_STATUS_RUNING,
  BEL_AI_STATUS_FACE_DETECTED,
  BEL_AI_STATUS_MAX
};

enum
{
  BEL_AI_DIABLE_ALL = 0,
  BEL_AI_ENABLE_AI,
  BEL_AI_ENABLE_AI_GS,
  BEL_AI_FUN_MAX
};


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

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
 * Name: iCE40UP_signal_action
 ****************************************************************************/
extern void ble_ai_init(void);
extern void ble_svc_ai_status_update(uint8_t status);
extern uint8_t ble_svc_ai_fun_status_read(void);


void iCE40UP_signal_action(int signo, siginfo_t *siginfo, void *arg)
{
  bool face_detected;

  if (signo == SIGUSR1)
    {
      face_detected = siginfo->si_value.sival_int;

      if (face_detected)
        {
          ble_svc_ai_status_update(BEL_AI_STATUS_FACE_DETECTED);
        }
      else
        {
          ble_svc_ai_status_update(BEL_AI_STATUS_RUNING);
        }
    }
  else
    {
      printf("  ERROR: Unexpected signal\n");
    }
}

int device_open(const char      *path)
{
  int fd;

  fd = open(path, O_RDONLY);
  if (fd < 0)
    {
      syslog(LOG_ERR, "AI_main: open %s failed: %d\n", path, errno);
    }
  else
    {
      syslog(LOG_INFO, "AI_main: Open %s device succefully!\n", path);
    }

  return fd;
}

int AI_chip_init(int fd)
{
  int ret = OK;
  sigset_t set;
  struct sigaction act;

  syslog(LOG_INFO, "AI_main:  register to receive the iCE40UP interrupt signal\n");

  /* register to receive interrupt */

  ret = ioctl(fd, SNIOC_GA_REGISTER_INT, SIGUSR1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "AI_main: ICE40UP_REGISTER_INT failed: %d\n", errno);
      return ret;
    }

  /* Set up so that siguser_action will respond to SIGUSR1 */

  memset(&act, 0, sizeof(struct sigaction));
  act.sa_sigaction = iCE40UP_signal_action;
  act.sa_flags     = SA_SIGINFO;

  /* prepare the signal set */
  (void)sigemptyset(&set);
  (void)sigaddset(&set, SIGUSR1);

  ret = sigaction(SIGUSR1, &act, NULL);
  if (ret != OK)
    {
      syslog(LOG_ERR, "AI_main: Failed to install SIGUSR1 handler, errno=%d\n", errno);
      return ret;
    }

  return ret;
}


int motion_sensor_init(int fd)
{
  int ret = OK;

  syslog(LOG_INFO, "AI_main:  config and start motion detection sensor\n");

  /* put accelerometer to low power mode */
  sn_ga_param_s cfg_accl =
  {
    /*! power mode */
    .power_mode = GA_MODE_NORMAL,
    /*! output data rate */
    .odr = MOTION_DETECTION_SENSOR_ODR,
    /*! range */
    .range = MOTION_DETECTION_SENSOR_RANGE,
    /*resolution*/
    .resolution = MOTION_DETECTION_SENSOR_RESO,
  };

  /* set the accelemeter configration */
  ret = ioctl(fd, SNIOC_A_SPARAM, (unsigned long)&cfg_accl);
  if (ret < 0)
    {
      syslog(LOG_ERR, "AI_main: SNIOC_A_SPARAM ioctl failed: %d\n", errno);
      return ret;
    }

#ifdef CONFIG_AI_MOTION_LSM6DS3
  syslog(LOG_INFO, "AI_main:  config lsm6ds3 interrupt source\n");

  struct lsm6ds3_int_settg int_setting =
  {
    .int_channel = LSM6DS3_INT_CHANNEL_NONE,
    .int_type = LSMD6S3_ACC_6D_ORIENTATION_INT,
    .int_type_cfg.acc_6D_4D_int =
    {
      .threshold = 50,
      .use_LPF2 = true,
    },
  };


  /* config interrupt source */
  ret = ioctl(fd, SNIOC_GA_CONFIG_ISR, (unsigned long)&int_setting);
  if (ret < 0)
    {
      syslog(LOG_ERR, "AI_main: SNIOC_GA_CONFIG_ISR ioctl failed: %d\n", errno);
      return ret;
    }
#endif

  return ret;
}

int AI_Chip_poweron(int fd, bool enable)
{
  int ret;

  if (enable)
    {
      /*start ice40up*/
      ret = ioctl(fd, SNIOC_START, 0);
    }
  else
    {
      /*stop ice40up*/
      ret = ioctl(fd, SNIOC_STOP, 0);
    }

  if (ret < 0)
    {
      syslog(LOG_ERR, "AI_main: SNIOC_STOP failed: %d\n", errno);
    }

  return ret;
}

#ifdef CONFIG_AI_MOTION_MC3672

int motion_sensor_read(int fd, bool *active)
{
  int ret;
  sn_ga_raw_s raw;
  bool result = false;

  /* read raw data */
  ret = ioctl(fd, SNIOC_A_GRAW, (unsigned long)&raw);

  if (ret < 0)
    {
      syslog(LOG_ERR, "AI_main: SNIOC_A_GRAW failed: %d\n", errno);
    }
  else
    {
      syslog(LOG_INFO, "AI_main: get accelerometer raw data, X:%d,Y:%d,Z:%d\n",
             raw.x_axis, raw.y_axis, raw.z_axis);

      if (abs(raw.z_axis) < MOTION_DETECTION_THRESHOLD)
        {
          result = true;
        }
    }

  *active = result;

  return ret;
}
#endif

#ifdef CONFIG_AI_MOTION_LSM6DS3

int motion_sensor_read(int fd, bool *active)
{
  int ret;
  uint32_t int_status;
  bool result = false;


  /* get interrupt status*/
  ret = ioctl(fd, SNIOC_GA_GINTSTATUS, (unsigned long)&int_status);
  if (ret < 0)
    {
      syslog(LOG_ERR, "AI_main: SNIOC_GA_GINTSTATUS failed: %d\n", errno);
    }
  else
    {
      syslog(LOG_INFO, "AI_main: get interrupt status:%d\n", int_status);

      if (int_status != 0)
        {
          result = true;
        }
    }

  *active = result;

  return ret;
}

#endif

int AI_chip_stop_message(int fd)
{
  int ret;

  ret = ioctl(fd, SNIOC_GA_UNREGISTER_INT, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "AI_main: SNIOC_GA_UNREGISTER_INT: %d\n", errno);
    }
  else
    {
      syslog(LOG_INFO, "AI_main: unregister signal, signo:%d\n", SIGUSR1);
    }

  return ret;
}

/****************************************************************************
 * Name: AI_help
 ****************************************************************************/


/****************************************************************************
 * Name: arg_string
 ****************************************************************************/

/****************************************************************************
 * Name: parse_args
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: AI_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int AI_main(int argc, char *argv[])
#endif
{
  int fd_iCE40UP;
  int fd_motion;
  int errval = 0;
  int ret;


  static uint8_t g_ai_sensor_pre_op = BEL_AI_DIABLE_ALL;
  uint8_t g_ai_sensor_op;

  bool motion_status;
  static bool motion_detected = false;
  static uint32_t counter = 0;

  bool AI_chip_enable;
  static bool AI_chip_status = false;

  setlogmask(LOG_UPTO(LOG_ERR));

  /* Start Nimble */

  syslog(LOG_INFO, "AI_main: Start Nimble\n");

  ble_ai_init();

  /* Open the iCE40UP device*/
  fd_iCE40UP = device_open("/dev/iCE40UP");
  if (fd_iCE40UP < 0)
    {
      errval = 2;
      goto errout;
    }

  fd_motion = device_open(MOTION_SENSOR_PATH);
  if (fd_motion < 0)
    {
      errval = 3;
      goto errout_with_ice40up;
    }

  ret = AI_chip_init(fd_iCE40UP);
  if (ret < 0)
    {
      errval = 4;
      goto errout_with_dev;
    }

  ret = motion_sensor_init(fd_motion);
  if (ret < 0)
    {
      errval = 5;
      goto errout_with_dev;
    }

  ret = AI_Chip_poweron(fd_iCE40UP, false);
  if (ret != OK)
    {
      errval = 6;
      goto errout_with_dev;
    }

  ble_svc_ai_status_update(BEL_AI_STATUS_INTI);


  /* Now start endless loop.
   */
  for (;;)
    {
      /* Flush any output before the loop entered or from the previous pass
       * through the loop.
       */

      fflush(stdout);

      usleep(SLEEP_PERIOD);

      g_ai_sensor_op = ble_svc_ai_fun_status_read();

      if (g_ai_sensor_pre_op != g_ai_sensor_op)
        {
          switch (g_ai_sensor_op)
            {
              case BEL_AI_DIABLE_ALL:
                AI_chip_enable = false;
                break;
              case BEL_AI_ENABLE_AI:
                AI_chip_enable = true;
                break;
              case BEL_AI_ENABLE_AI_GS:
                AI_chip_enable = false;
                motion_detected = false;
                counter = 0;
                break;
              default:
                break;
            }
          g_ai_sensor_pre_op = g_ai_sensor_op;

        }

      if (g_ai_sensor_op == BEL_AI_ENABLE_AI_GS)
        {
          ret = motion_sensor_read(fd_motion, &motion_status);

          if (ret == OK)
            {
              if (motion_status == true)
                {
                  counter = 0;
                  if (motion_detected == false)
                    {
                      syslog(LOG_INFO, "AI_main:motion detected!\n");

                      AI_chip_enable = true;

                      motion_detected = true;
                    }
                }
              else
                {
                  counter ++;
                  if (counter >= MOTION_DETECTION_DEBOUNCE)
                    {
                      counter = 0;

                      if (motion_detected == true)
                        {
                          AI_chip_enable = false;

                          motion_detected = false;
                        }
                    }

                }
            }
          else
            {
              errval = 10;
              goto errout_with_dev;
            }

        }

      if (AI_chip_enable != AI_chip_status)
        {
          ret = AI_Chip_poweron(fd_iCE40UP, AI_chip_enable);

          if (ret != OK)
            {
              errval = 11;
              goto errout_with_dev;
            }

          AI_chip_status = AI_chip_enable;
        }
    }

  /*stop ice40up*/
  ret = AI_Chip_poweron(fd_iCE40UP, false);
  if (ret != OK)
    {
      errval = 12;
      goto errout_with_dev;
    }

  /* unregister*/
  ret = AI_chip_stop_message(fd_iCE40UP);
  if (ret != OK)
    {
      errval = 13;
      goto errout_with_dev;
    }

  close(fd_motion);
  close(fd_iCE40UP);
  return OK;

  /* Error exits */

errout_with_dev:
  close(fd_motion);

errout_with_ice40up:
  close(fd_iCE40UP);

errout:
  printf("Terminating!\n");
  fflush(stdout);
  return errval;
}
