/****************************************************************************
 *   examples/fast_api_demo/fast_api_main.c
 *
 *   Copyright (C) 2018  zGlue INC All rights reserved.
 *   Author: Min Yang <min.yang@zglue.com>
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

#include <sys/stat.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sched.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <nuttx/drivers/zglue_fast.h>
#include <nuttx/zglue_fast/fast_api.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FAST_DEV   "/dev/fast"
#define LED_COUNT  FAST_LED3


/****************************************************************************
 * Name: blinky_zglue_main
 ****************************************************************************/

#if defined(BUILD_MODULE)
int main(int argc, FAR char *argv[])
#else
int blinky_main(int argc, char *argv[])
#endif
{
  int fd = 0;
  int ret;
  uint32_t param[10];
  printf("\nZglue smartfabric LED blink example\r\n");

  /* Open the driver */
  fd = open(FAST_DEV, O_RDONLY);
  if (fd < 0)
    {
      fprintf(stderr, "ERROR: Failed to open %s: %d\n",
              FAST_DEV, errno);
      return EXIT_FAILURE;
    }

  /* set the debug level */
  param[0] = FAST_DEBUG_LEVEL_1;
  ret = ioctl(fd, FAST_IOCTL_SET_DEBUG_LEVEL, (unsigned long)param);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to set debug level: %d\n", errno);
      close(fd);
      return EXIT_FAILURE;
    }

  /* Fast api config  LED2  */
  param[0] = (uint32_t) FAST_LED2;
  param[1] = (uint32_t) FAST_LED_DUTY_CYCLE_50_0;
  param[2] = (uint32_t) FAST_LED_PERIOD_0_5_S;
  param[3] = (uint32_t) FAST_LED_SCALE_12_8_mA;
  param[4] = (uint32_t) FAST_LED_BRIGHTNESS_50;
  param[5] = (uint32_t) false;

  ret = ioctl(fd, FAST_IOCTL_CONFIGURE_LED, (unsigned long)param);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to configure led: %d\n", errno);
      close(fd);
      return EXIT_FAILURE;
    }

  /* Fast api blink LED2  */
  ret = ioctl(fd, FAST_IOCTL_ENABLE_LED, (unsigned long)param);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to enable led: %d\n", ret);
      close(fd);
      return EXIT_FAILURE;
    }

  close(fd);
  return 0;

}
