/****************************************************************************
 *   system/zglue_fast/fast_led.c
 *
 *   Copyright (C) 2011, 2016, 2018 Gregory Nutt. All rights reserved.
 *   Author: Yu Peng <yupeng@zglue.com>
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
#include <fast_cmd.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int32_t do_led_help(int32_t argc, char **argv);
static int32_t do_config_led(int32_t argc, char **argv);
static int32_t do_config_def_led(int32_t argc, char **argv);
static int32_t do_enable_led(int32_t argc, char **argv);
static int32_t do_disable_led(int32_t argc, char **argv);

static const struct fast_cmdmap_s g_fast_led[] =
{
  { "help",          do_led_help,       "",                              NULL},
  { "config",        do_config_led,     "configures led",                "\n\t [led_id (1, 2, 3)]\n\t [duty_cycle (0=6.25%, 15=100%)]\n\t [period (0=0.5s, 15=8s)]\n\t [intensity_range (0=OFF, 1=0.2-3.2mA, 2=0.2-6.4mA, 3=0.4-12.8mA)]\n\t [brightness (0-31)]\n\t [invert_pwm (0=active_high, 1=active_low)]"},
  { "config_def",    do_config_def_led, "configures led w/ defaults\n  ie, fast led config led_id 50% 0.5s 0.4-12.8mA 16 false",                                               "\n\t [led_id (1, 2, 3)]"},
  { "enable",        do_enable_led,     "enables led",                   "\n\t [led_mask (Bit0 : LED1, Bit1:LED2, Bit3:LED3)]"},
  { "disable",       do_disable_led,    "disables led",                  "\n\t [led_mask (Bit0 : LED1, Bit1:LED2, Bit3:LED3)]"},
  { NULL,            NULL,              NULL,                            NULL },
};

/* Common, data container */
extern uint32_t fast_ioctl_arg[FAST_IOCTL_ARG_COUNT];

/* Common, message formats */

extern const char g_fast_argrequired[];
extern const char g_fast_arginvalid[];
extern const char g_fast_argrange[];
extern const char g_fast_cmdnotfound[];
extern const char g_fast_toomanyargs[];
extern const char g_fast_cmdfailed[];
extern const char g_fast_xfrerror[];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: do_led_help
 ****************************************************************************/
static int32_t do_led_help(int32_t argc, char **argv)
{
  const struct fast_cmdmap_s *ptr;

  printf("Usage: fast led <cmd> [arguments]\n");
  printf("Where <cmd> is one of:\n\n");
  for (ptr = (g_fast_led + 1); ptr->cmd; ptr++)
    {
      if (ptr->usage)
        {
          printf("  %s: %s %s\n\n", ptr->cmd, ptr->desc,  ptr->usage);
        }
      else
        {
          printf("  %s: %s\n\n", ptr->cmd, ptr->desc);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: do_config_led
 ****************************************************************************/
static int32_t do_config_led(int32_t argc, char **argv)
{
  /* fast led !config !argv[1] ... !argv[6] */
  if (argc < 7)
    {
      do_led_help(argc, argv);
      return -EINVAL;
    }

  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  fast_ioctl_arg[1] = (uint32_t) strtol(argv[2], NULL, 10);
  fast_ioctl_arg[2] = (uint32_t) strtol(argv[3], NULL, 10);
  fast_ioctl_arg[3] = (uint32_t) strtol(argv[4], NULL, 10);
  fast_ioctl_arg[4] = (uint32_t) strtol(argv[5], NULL, 10);
  fast_ioctl_arg[5] = (uint32_t) strtol(argv[6], NULL, 10);

  return send_fast_ioctl(FAST_IOCTL_CONFIGURE_LED, (ulong_t) fast_ioctl_arg);
}

/****************************************************************************
 * Name: do_config_def_led
 ****************************************************************************/
static int32_t do_config_def_led(int32_t argc, char **argv)
{
  /* fast led !config_def !argv[1] */
  if (argc < 2)
    {
      do_led_help(argc, argv);
      return -EINVAL;
    }

  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  fast_ioctl_arg[1] = (uint32_t) FAST_LED_DUTY_CYCLE_50_0;
  fast_ioctl_arg[2] = (uint32_t) FAST_LED_PERIOD_0_5_S;
  fast_ioctl_arg[3] = (uint32_t) FAST_LED_SCALE_12_8_mA;
  fast_ioctl_arg[4] = (uint32_t) FAST_LED_BRIGHTNESS_50;
  fast_ioctl_arg[5] = (uint32_t) false;

  return send_fast_ioctl(FAST_IOCTL_CONFIGURE_LED, (ulong_t) fast_ioctl_arg);
}

/****************************************************************************
 * Name: do_enable_led
 ****************************************************************************/
static int32_t do_enable_led(int32_t argc, char **argv)
{
  /* fast led !enable !argv[1] */
  if (argc < 2)
    {
      do_led_help(argc, argv);
      return -EINVAL;
    }

  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);

  return send_fast_ioctl(FAST_IOCTL_ENABLE_LED, (ulong_t)fast_ioctl_arg);
}

/****************************************************************************
 * Name: do_disable_led
 ****************************************************************************/
static int32_t do_disable_led(int32_t argc, char **argv)
{
  /* fast led !disable !argv[1] */
  if (argc < 2)
    {
      do_led_help(argc, argv);
      return -EINVAL;
    }

  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);

  return send_fast_ioctl(FAST_IOCTL_DISABLE_LED, (ulong_t)fast_ioctl_arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fast_led
 ****************************************************************************/

int32_t fast_led(int32_t argc, FAR char **argv)
{
  int32_t ret = -1;

  set_errno(0);
  dbg_tag();
  if (argc == 1)
    {
      do_led_help(argc, argv);
      return -EINVAL;
    }
  ret = fast_parse(g_fast_led, argc, argv);
  return ret;
}
