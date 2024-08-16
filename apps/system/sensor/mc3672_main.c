/****************************************************************************
 * apps/system/sensor/sensor.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Levin Li <zhiqiang@zglue.com>
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
#include <nuttx/sensors/mc3672.h>
#include <nuttx/sensors/internal_mc3672.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#define DEV_MC3672   "/dev/accel"
#define DEV_MAX     1
#define SAMPLE_COUNT     1

#define DEFAULT_MC3672_ODR (540)
#define DEFAULT_MC3672_RANGE (4)
#define DEFAULT_MC3672_RESOLUTION (8)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mc3672_main(void)
{
  int filefd;
  int ret;
#ifdef CONFIG_MC3672
  ssize_t len;
  ssize_t buffer_len;
  int i = 0;
  char dev_name[16];
  sn_ga_param_s param;
  int error_code;
  struct mc3672_acc_t *mc3672_rawdata;
  int sample_number;


  for (i = 0; i < DEV_MAX; i++)
    {
      sprintf(dev_name, "%s%d", DEV_MC3672, i);
      printf("Open %s device for read \n", dev_name);
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
      printf("There is NO valid MC3672 device.\n");
      return -1;
    }

  printf("\nStarting to Write MC3672 Parameter:\n\n");
  param.resolution = DEFAULT_MC3672_RESOLUTION;
  param.odr = DEFAULT_MC3672_ODR;
  param.power_mode = GA_MODE_NORMAL;
  param.range = DEFAULT_MC3672_RANGE;
  printf("\tresolution:%d, samplerate:%d\n",
         param.resolution, param.odr);
  ret = ioctl(filefd, SNIOC_A_SPARAM, (unsigned long)&param);
  if (OK != ret)
    {
      error_code = errno;
      printf("Error: SNIOC_A_SPARAM IOCtrl failed %d\r\n", error_code);
      goto error_out_fd;
    }

  /* Starting device */
  ret = ioctl(filefd, SNIOC_START, (unsigned long)&param);
  if (OK != ret)
    {
      error_code = errno;
      printf("Error: SNIOC_START IOCtrl failed %d\r\n", error_code);
      goto error_out_fd;
    }

  /*read data*/
  buffer_len = SAMPLE_COUNT * sizeof(struct mc3672_acc_t);
  mc3672_rawdata = (struct mc3672_acc_t *)malloc(buffer_len);

  if (mc3672_rawdata == NULL)
    {
      printf("Error: memory malloc failed!\r\n");
      goto error_out_mem;
    }

  memset(mc3672_rawdata, 0x00, buffer_len);


  len = read(filefd, mc3672_rawdata, buffer_len);

  if (len >= 0)
    {
      sample_number = len / sizeof(struct mc3672_acc_t);

      printf("\r\n\nInterrupt received from INT pin.");

      printf("\r\n\n%d sample(s) fetched from MC3672:", sample_number);

      for (i = 0; i < sample_number; i++)
        {
          printf("\nData index : %d  , X : %d, Y: %d, Z: %d", i, mc3672_rawdata[i].XAxis, mc3672_rawdata[i].YAxis,
                 mc3672_rawdata[i].ZAxis);
        }
    }
  else if (len < 0)
    {
      error_code = errno;
      printf("Error: read sensor failed %d\n", error_code);
    }

  /* stopping device */
  ret = ioctl(filefd, SNIOC_STOP, (unsigned long)&param);
  if (OK != ret)
    {
      error_code = errno;
      printf("Error: SNIOC_STOP IOCtrl failed %d\n", error_code);
      goto error_out_mem;
    }

  printf("\nEnd\n");
error_out_mem:
  free(mc3672_rawdata);
error_out_fd:
  close(filefd);
#else
  UNUSED(filefd);
  UNUSED(ret);
#endif
  return OK;
}
