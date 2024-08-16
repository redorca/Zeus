/****************************************************************************
 * apps/system/sensor/sensor.c
 *
 *   Copyright (C) 2018 Zglue
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
#include <nuttx/progmem.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <nuttx/sensors/ioctl.h>
#include <nuttx/sensors/max86140.h>
#include <nuttx/arch.h>
/****************************************************************************
 * Private Functions
 ****************************************************************************/

#define DEV_MAX86140   "/dev/heartrate"
#define DEV_MAX     2
#define SAMPLE_COUNT     1000000  //Changed from 30 to 500

/****************************************************************************
 * Public Functions
 ****************************************************************************/
int max86140_main(void)
{
#ifdef CONFIG_MAX86140
  int filefd;
  int i = 0;
  char dev_name[16];
  for (i = 0; i < DEV_MAX; i++)
    {
      sprintf(dev_name, "%s%d", DEV_MAX86140, i);
      printf("Starting to Read %s%d device \n", DEV_MAX86140, i);
      filefd = open(dev_name, O_RDONLY);
      if (filefd < 0)
        {
          printf("Can't Open Device for Read. Reason:%d\n", filefd);
        }
      else
        {
          break;
        }
    }

  if (filefd < 0)
    {
      printf("There is NO valid HR chip MAX86140 device.\n");
      return -1;
    }

  /* Starting to read fifo mode */
  //int IRSample = 0, redSample = 0, greenSample = 0;
  int greenSample = 0;
  uint8_t buf[3] = {0};
  int count = 0;
  printf("IR       RED         GREEN\r\n");
  for (i = 0; i < SAMPLE_COUNT; i++)
    {
      ioctl(filefd, SNIOC_A_READ_FIFO, (unsigned long)buf);
      //up_udelay(15000); /*delay 15ms -- Read a little faster than the speed of sensor produce data*/
      up_udelay(906); //Changed

      switch (buf[0] & 0xF8)
        {
          case 0x08:
            //IRSample = (((buf[0] & 0x7) << 16) | (buf[1] << 8) | buf[2]);
            //printf("0x%x\t",IRSample);
            count++;
            //printf("IRSample:\t0x%x\r\n", IRSample);

            break;
          case 0x10:
            //redSample = (((buf[0] & 0x7) << 16) | (buf[1] << 8) | buf[2]);
            //printf("0x%x\t",redSample);
            count++;
            //printf("Red Sample:\t0x%x\r\n", redSample);
            break;
          case 0x18:
            greenSample = (((buf[0] & 0x7) << 16) | (buf[1] << 8) | buf[2]);
            printf("Green Sample: 0x%x\t", greenSample);
            count++;
            //printf("Green Sample:\t0x%x\r\n", greenSample);
            break;
        }
      if (count == 3)
        {
          printf("\r\n");
          count = 0;
        }
    }
#endif
  return OK;
}
