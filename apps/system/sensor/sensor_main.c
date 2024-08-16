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
#include "sensors.h"
/****************************************************************************
 * Private Functions
 ****************************************************************************/

#define DEV_MC3672   "/dev/accel"
#define DEV_MAX     2
#define SAMPLE_COUNT     2

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int sensor_main(int argc, char **argv)
#endif
{
  if (argc == 1)
    {
      printf("Type 'sensor 1' for mc3672 sensor\r\nType 'sensor 2' for max86140 sensor\r\n");
      return OK;
    }
  if (argc == 2)
    {
      uint32_t arg = (uint32_t) strtol(argv[1], NULL, 10);
      if (arg == 1)
        {
#ifdef CONFIG_MC3672
          mc3672_main();
#endif
        }
      else if (arg == 2)
        {
#ifdef CONFIG_MAX86140
          max86140_main();
#endif
        }
      else
        {
          printf("no such command arg\r\n");
        }
    }

  return OK;
}
