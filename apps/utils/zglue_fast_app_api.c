/****************************************************************************
 * apps/utils/zglue_fast_app_api.c
 *
 *   Copyright (C) 2014, 2016 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Bob Doiron
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
#include <utils/zglue_fast_app_api.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FAST_DEV  "/dev/fast"
#define FAST_LED_PARA_LEN  (10)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int fast_led_blinky(uint8_t led_num)
{
  int32_t ret = OK;
#ifdef CONFIG_ARCH_HAVE_FAST
  int32_t led_fd;
  uint32_t param[FAST_LED_PARA_LEN];

  led_fd = open(FAST_DEV, O_RDONLY);
  if (led_fd <= 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to open %s: %d\n",
             FAST_DEV, errno);
      return -1;
    }

  /* set the debug level */
  param[0] = FAST_DEBUG_LEVEL_1;
  ioctl(led_fd, FAST_IOCTL_SET_DEBUG_LEVEL, (unsigned long)param);
  param[0] = (uint32_t) led_num;
  param[1] = (uint32_t) FAST_LED_DUTY_CYCLE_50_0;
  param[2] = (uint32_t) FAST_LED_PERIOD_0_5_S;
  param[3] = (uint32_t) FAST_LED_SCALE_3_2_mA;
  param[4] = (uint32_t) FAST_LED_BRIGHTNESS_37_5;
  param[5] = (uint32_t) false;

  /*Enable led use led bit mask, config led use led id.*/
  ioctl(led_fd, FAST_IOCTL_CONFIGURE_LED, (unsigned long)param);

  param[0] = (uint32_t) (1 << (led_num - 1));
  ret = ioctl(led_fd, FAST_IOCTL_ENABLE_LED, (unsigned long)param);
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: IO control failed: %d_%d\n", ret, errno);
    }
  close(led_fd);

#endif
  return ret;
}


int fast_led_on(bool led_on_off, uint8_t led_num)
{
  int32_t ret;
#ifdef CONFIG_ARCH_HAVE_FAST
  int32_t led_fd;
  uint32_t param[FAST_LED_PARA_LEN];

  led_fd = open(FAST_DEV, O_RDONLY);
  if (led_fd <= 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to open %s: %d\n",
             FAST_DEV, errno);
      return -1;
    }

  /* set the debug level */
  param[0] = FAST_DEBUG_LEVEL_1;
  ioctl(led_fd, FAST_IOCTL_SET_DEBUG_LEVEL, (unsigned long)param);
  param[0] = (uint32_t) led_num;
  param[1] = (uint32_t) FAST_LED_DUTY_CYCLE_6_25;
  param[2] = (uint32_t) FAST_LED_PERIOD_8_0_S;
  param[3] = (uint32_t) FAST_LED_SCALE_3_2_mA;
  param[4] = (uint32_t) FAST_LED_BRIGHTNESS_37_5;
  param[5] = (uint32_t) false;

  /*Enable led use led bit mask, config led use led id.*/
  ioctl(led_fd, FAST_IOCTL_CONFIGURE_LED, (unsigned long)param);

  param[0] = (uint32_t) (1 << (led_num - 1));
  if (led_on_off)
    {
      ret = ioctl(led_fd, FAST_IOCTL_ENABLE_LED, (unsigned long)param);
      if (ret != OK)
        {
          syslog(LOG_ERR, "ERROR: IO control failed: %d_%d\n", ret, errno);
        }
    }
  else
    {
      ret = ioctl(led_fd, FAST_IOCTL_DISABLE_LED, (unsigned long)param);
      if (ret != OK)
        {
          syslog(LOG_ERR, "ERROR: IO control failed: %d_%d\n", ret, errno);
        }
    }
  close(led_fd);
#endif
  return OK;
}


