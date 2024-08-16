/****************************************************************************
 * examples/max86140/max86140_main.c
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
#include <poll.h>

#include <string.h>
#include <nuttx/sensors/ioctl.h>
#include <nuttx/sensors/max86140.h>
#include <nuttx/arch.h>


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define DEV_MAX86140   "/dev/heartrate"
#define DEV_MAX     2
#define SAMPLE_COUNT     10

#define SLT_TEST_SAMPLE_ISR_BIT         (0x01)
#define SLT_TEST_SAMPLE_RED_BIT         (0x02)
#define SLT_TEST_SAMPLE_GREEN_BIT       (0x04)
#define SLT_TEST_INT_BIT                (0x08)
#define SLT_TEST_GPIO1_BIT              (0x10)
#define SLT_TEST_GPIO2_BIT              (0x20)

/****************************************************************************
 * Private Data
 ****************************************************************************/
int max86140_fd;
uint32_t slt_test_flag = 0;
/****************************************************************************
 * Private Functions
 ****************************************************************************/
static void max86140_int_signo(int signo, FAR siginfo_t *siginfo, FAR void *context)
{
  int32_t ret;
  ret = ioctl(max86140_fd, SNIOC_DISABLE_INT1, (unsigned long)1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "MAX86140: SNIOC_DISABLE_INT1 ioctl failed: %d\n", errno);
    }
  ret = ioctl(max86140_fd, SNIOC_CLEAR_INT1, (unsigned long)SIGUSR1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "MAX86140: SNIOC_DISABLE_INT1 ioctl failed: %d\n", errno);
    }
  slt_test_flag |= SLT_TEST_INT_BIT;
}


static int max86140_SLT_test(void)
{
  int i = 0;
  int32_t ret = 0, gpio_test_ret = 0;

  struct sigaction   act;
  act.sa_sigaction = max86140_int_signo;
  act.sa_flags  = SA_SIGINFO;

  syslog(LOG_INFO, "\nMAX86140 SLT TEST STARTED-----\n", DEV_MAX86140, i);

  (void)sigemptyset(&act.sa_mask);
  (void)sigaddset(&act.sa_mask, SIGUSR1);

  ret = sigaction(SIGUSR1, &act, NULL);
  if (ret != OK)
    {
      syslog(LOG_ERR, "\nMax86140 SLT test: ERROR sigaction failed, ret=%d\n", ret);
      return ret;
    }

  /*1. TEST INT PIN.*/

  /*Register INT signo call back fun.*/
  ret = ioctl(max86140_fd, SNIOC_GA_REGISTER_INT, (unsigned long)SIGUSR1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "\nMAX86140: SNIOC_GA_REGISTER_INT ioctl failed: %d\n", errno);
    }

  /*Enable INT data ready and data full.*/
  ret = ioctl(max86140_fd, SNIOC_ENABLE_INT1, (unsigned long)0xc0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "\nMAX86140: SNIOC_ENABLE_INT1 ioctl failed: %d\n", errno);
    }

  if (slt_test_flag & (SLT_TEST_INT_BIT))
    {
      syslog(LOG_INFO, "\n1. MAX86140 interrupt pin test passed...\n\n");
    }
  else
    {
      syslog(LOG_ERR, "\n1. MAX86140 interrupt pin test failed...\n\n");
    }

  /*2. TEST SAMPLE.*/
  for (i = 0; i < SAMPLE_COUNT; i++)
    {
      int IRSample = 0, redSample = 0, greenSample = 0;
      uint8_t buf[3] = {0};
      ret = ioctl(max86140_fd, SNIOC_A_READ_FIFO, (unsigned long)buf);
      if (ret < 0)
        {
          syslog(LOG_ERR, "\nMAX86140: SNIOC_A_READ_FIFO ioctl failed: %d\n", errno);
        }
      up_udelay(15000); /*delay 15ms -- Read a little faster than the speed of sensor produce data*/
      switch (buf[0] & 0xF8)
        {
          case 0x08:
            IRSample = (((buf[0] & 0x7) << 16) | (buf[1] << 8) | buf[2]);
            slt_test_flag |= SLT_TEST_SAMPLE_ISR_BIT;
            syslog(LOG_INFO, "IRSample:\t0x%x\r\n", IRSample);
            break;
          case 0x10:
            redSample = (((buf[0] & 0x7) << 16) | (buf[1] << 8) | buf[2]);
            slt_test_flag |= SLT_TEST_SAMPLE_RED_BIT;
            syslog(LOG_INFO, "Red Sample:\t0x%x\r\n", redSample);
            break;
          case 0x18:
            greenSample = (((buf[0] & 0x7) << 16) | (buf[1] << 8) | buf[2]);
            slt_test_flag |= SLT_TEST_SAMPLE_GREEN_BIT;
            syslog(LOG_INFO, "Green Sample:\t0x%x\r\n", greenSample);
            break;
          default:
            syslog(LOG_INFO, "NO matched:%d\n", buf[0]);
        }
    }

  if (slt_test_flag & (SLT_TEST_SAMPLE_ISR_BIT ||
                       SLT_TEST_SAMPLE_GREEN_BIT ||
                       SLT_TEST_SAMPLE_RED_BIT)
     )
    {
      syslog(LOG_INFO, "\n2. MAX86140 sample read test passed...\n");
    }
  else
    {
      syslog(LOG_ERR, "\n2. MAX86140 sample read test failed...\n");
    }

  /*3. GIPO TEST*/

  ret = ioctl(max86140_fd, SNIOC_GPIO_1_2_TEST, (unsigned long)&gpio_test_ret);
  if (ret < 0)
    {
      syslog(LOG_ERR, "\nMAX86140: SNIOC_GPIO_1_2_TEST ioctl failed: %d\n", errno);
    }

  if (!gpio_test_ret)
    {
      slt_test_flag |= SLT_TEST_GPIO1_BIT | SLT_TEST_GPIO2_BIT;
      syslog(LOG_ERR, "\n3. MAX86140 GOIO_1_2 test passed...\n");
    }
  else
    {
      syslog(LOG_ERR, "\n3. MAX86140 GOIO_1_2 test failed...\n");
    }

  syslog(LOG_INFO, "\nMAX86140 SLT TEST COMPELETE-----\n", DEV_MAX86140, i);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
int max86140_slt_main(void)
{
  int i = 0;
  char dev_name[16];

  for (i = 0; i < DEV_MAX; i++)
    {
      sprintf(dev_name, "%s%d", DEV_MAX86140, i);
      syslog(LOG_INFO, "Starting to Read %s%d device \n", DEV_MAX86140, i);
      max86140_fd = open(dev_name, O_RDONLY);
      if (max86140_fd < 0)
        {
          printf("Can't Open Device for Read. Reason:%d\n", max86140_fd);
        }
      else
        {
          break;
        }
    }

  if (max86140_fd < 0)
    {
      printf("There is NO valid HR chip MAX86140 device.\n");
      return -1;
    }

  max86140_SLT_test();

  close(max86140_fd);
  return OK;
}

