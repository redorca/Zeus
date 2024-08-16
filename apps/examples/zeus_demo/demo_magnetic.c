/****************************************************************************
 * examples/nimble/nimble_demo_main.c
 *
 *   Copyright (C) 2007-2013, 2017 Gregory Nutt. All rights reserved.
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

#include <sys/stat.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <sched.h>
#include <errno.h>
#include <wchar.h>
#include <math.h>
#include <nuttx/lib/math.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <nuttx/sensors/internal_bmm150.h>
#include <nuttx/sensors/bmm150.h>
#include <nuttx/sensors/ioctl.h>

#ifdef CONFIG_EXAMPLES_DEMO_MAGNETIC
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/*BMM150*********************************************************************/
#define CONFIG_EXAMPLES_BMM150_DEVPATH "/dev/mag0"
#define PI 3.1415926


/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/


/****************************************************************************
 * Private Data
 ****************************************************************************/

/*BMM150*********************************************************************/
static int bmm150_fd;
static struct bmm150_mag_data mag_data;
static struct bmm150_mag_data mag_offset = {0};


/****************************************************************************
 * Public Data
 ****************************************************************************/


/****************************************************************************
 * Private Functions
 ****************************************************************************/
void mag_dev_open(void)
{
  uint32_t bmm150_id = 0xff;
  int ret;
  /*Open and inicialize the bmm150 sensor.*/
  bmm150_fd = open(CONFIG_EXAMPLES_BMM150_DEVPATH, O_RDONLY);
  if (bmm150_fd < 0)
    {
      syslog(LOG_ERR, "\n Open compass sensor failed... [%d]\n", bmm150_fd);
    }
  syslog(LOG_INFO, "\n Open compass sensor sucessful... [%d]\n", bmm150_fd);

  /* check sensor id */
  ret = ioctl(bmm150_fd, SNIOC_READID, (unsigned long)&bmm150_id);
  if (ret < 0)
    {
      int errcode = errno;
      syslog(LOG_ERR, "Magnetic dev SNIOC_READID ioctl failed: %d\n", errcode);
    }
  else
    {
      syslog(LOG_INFO, "Magnetic dev read sensor ID:%d \n", bmm150_id);
    }
}

void mag_dev_close(void)
{
  if (bmm150_fd < 0)
    {
      syslog(LOG_ERR, "There is NO valid HR chip MAX86140 device.\n");
      return;
    }
  close(bmm150_fd);
}

void mag_dev_enable(void)
{
  int errcode = errno;
  int ret;

  if (bmm150_fd < 0)
    {
      syslog(LOG_ERR, "\n Open magnetic dev failed... [%d]\n", bmm150_fd);
      return;
    }

  /* start bmm150 device,use low power presetmode */
  ret = ioctl(bmm150_fd, SNIOC_START, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "SNIOC_START ioctl failed: %d\n", errcode);
    }

}

void mag_dev_disable(void)
{
  int errcode = errno;
  int ret;

  if (bmm150_fd < 0)
    {
      syslog(LOG_ERR, "\n Open magnetic dev failed... [%d]\n", bmm150_fd);
      return;
    }

  /* start bmm150 device,use low power presetmode */
  ret = ioctl(bmm150_fd, SNIOC_STOP, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "SNIOC_STOP ioctl failed: %d\n", errcode);
    }
}

void mag_dev_read(void)
{
  int ret;

  if (bmm150_fd < 0)
    {
      syslog(LOG_ERR, "\n Open magnetic dev failed... [%d]\n", bmm150_fd);
      return;
    }

  /* read magmeter data*/
  ret = read(bmm150_fd, &mag_data, sizeof(struct bmm150_mag_data));
  if (ret <= 0)
    {
      int errcode = errno;
      printf("Read magnetic raw data failed: errno = %d, ret = %d\n", errcode, ret);
    }
  else
    {
    }
}

void mag_calibrate(void)
{
  int16_t value_x_min = 0;
  int16_t value_x_max = 0;
  int16_t value_y_min = 0;
  int16_t value_y_max = 0;
  int16_t value_z_min = 0;
  int16_t value_z_max = 0;
  uint32_t timeStart = 0;

  if (bmm150_fd < 0)
    {
      syslog(LOG_ERR, "\n Open magnetic dev failed... [%d]\n", bmm150_fd);
      return;
    }

  mag_dev_read();
  value_x_min = mag_data.x;
  value_x_max = mag_data.x;
  value_y_min = mag_data.y;
  value_y_max = mag_data.y;
  value_z_min = mag_data.z;
  value_z_max = mag_data.z;
  syslog(LOG_WARNING, "Calibrate Start..........\n");
  usleep(3000000);

  while (timeStart < 100)
    {
      timeStart++;

      /* Update x-Axis max/min value */
      mag_dev_read();
      if (value_x_min > mag_data.x)
        {
          value_x_min = mag_data.x;
        }
      else if (value_x_max < mag_data.x)
        {
          value_x_max = mag_data.x;
        }

      /* Update y-Axis max/min value */
      if (value_y_min > mag_data.y)
        {
          value_y_min = mag_data.y;
        }
      else if (value_y_max < mag_data.y)
        {
          value_y_max = mag_data.y;
        }

      /* Update z-Axis max/min value */
      if (value_z_min > mag_data.z)
        {
          value_z_min = mag_data.z;

        }
      else if (value_z_max < mag_data.z)
        {
          value_z_max = mag_data.z;
        }

      usleep(100000);

    }

  mag_offset.x = value_x_min + (value_x_max - value_x_min) / 2;
  mag_offset.y = value_y_min + (value_y_max - value_y_min) / 2;
  mag_offset.z = value_z_min + (value_z_max - value_z_min) / 2;
  syslog(LOG_WARNING, "OFFSET: X : %d \t Y : %d \t Z : %d \n", mag_offset.x, mag_offset.y, mag_offset.z);

}

void mag_angel_caculate(uint8_t *data)
{

  double xyHeading, yxHeading, zxHeading, zyHeading;
  double xy_angle, yx_angle, zx_angle, zy_angle;
  struct bmm150_mag_data value;

  (void)xy_angle;
  (void)yx_angle;
  (void)zx_angle;
  (void)zy_angle;

  if (bmm150_fd < 0)
    {
      syslog(LOG_ERR, "\n Open magnetic dev failed... [%d]\n", bmm150_fd);
      return;
    }

  value.x = mag_data.x - mag_offset.x;
  value.y = mag_data.y - mag_offset.y;
  value.z = mag_data.z - mag_offset.z;

  xyHeading = atan2(value.x, value.y);
  yxHeading = atan2(value.y, value.x);
  zxHeading = atan2(value.z, value.x);
  zyHeading = atan2(value.z, value.y);

  if (xyHeading < 0)
    {
      xyHeading += 2 * PI;
    }
  if (xyHeading > 2 * PI)
    {
      xyHeading -= 2 * PI;
    }
  xy_angle = xyHeading * 180 / M_PI;
  yx_angle = yxHeading * 180 / M_PI;
  zx_angle = zxHeading * 180 / M_PI;
  zy_angle = zyHeading * 180 / M_PI;

  memcpy(data, &xy_angle, sizeof(double));
  //syslog(LOG_WARNING, "Magnetic angle: x-->y[%f] : y-->x[%f] : z-->x[%f] : z-->y[%f] \n",xy_angle, yx_angle, zx_angle, zy_angle);
}

#endif // CONFIG_EXAMPLES_DEMO_MAGNETIC
