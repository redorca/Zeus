/****************************************************************************
 *   system/zglue_fast/fast_gpio_exp.c
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

static int32_t do_gpio_exp_help(int32_t argc, char **argv);
static int32_t do_config_pin(int32_t argc, char **argv);
static int32_t do_set_pin(int32_t argc, char **argv);
static int32_t do_get_pin(int32_t argc, char **argv);
static int32_t do_clear_irq(int32_t argc, char **argv);
static int32_t do_disable_pin(int32_t argc, char **argv);

static const struct fast_cmdmap_s g_fast_gpio_exp[] =
{
  { "help",             do_gpio_exp_help,   "",                     NULL},
  { "config_pin",       do_config_pin,      "configures gpio pin",  "\n\t [gpio_port (0=zeus2_default)]\n\t [gpio_pin (0,1,2,3,4,5,6,7)]\n\t [function (0=output, 1=input_rising_edge, 2=input_falling_edge, 3=input_both_edges)]\n\t [pin_level (0=low, 1=high)]"},
  { "set_pin",    do_set_pin,   "sets gpio pin level",  "\n\t [gpio_port (0=zeus2_default)]\n\t [gpio_pin (0,1,2,3,4,5,6,7)]\n\t [pin_level (0=low, 1=high)]"},
  { "get_pin",    do_get_pin,   "gets gpio pin level",  "\n\t [gpio_port (0=zeus2_default)]\n\t [gpio_pin (0,1,2,3,4,5,6,7)]"},
  { "clear_irq",        do_clear_irq,       "reads and clears all interrupts",  NULL},
  { "disable_pin",      do_disable_pin,     "disables gpio pin",    "\n\t [gpio_port (0=zeus2_default)]\n\t [gpio_pin (0,1,2,3,4,5,6,7)]"},
  { NULL,               NULL,               NULL,                   NULL },
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
 * Name: do_gpio_exp_help
 ****************************************************************************/
static int32_t do_gpio_exp_help(int32_t argc, char **argv)
{
  const struct fast_cmdmap_s *ptr;

  printf("Usage: fast gpio <cmd> [arguments]\n");
  printf("Where <cmd> is one of:\n\n");
  for (ptr = (g_fast_gpio_exp + 1); ptr->cmd; ptr++)
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
 * Name: do_config_pin
 ****************************************************************************/
static int32_t do_config_pin(int32_t argc, char **argv)
{
  /* fast gpio !config !argv[1] ... !argv[4] */
  if (argc != 5)
    {
      do_gpio_exp_help(argc, argv);
      return -EINVAL;
    }

  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  fast_ioctl_arg[1] = (uint32_t) strtol(argv[2], NULL, 10);
  fast_ioctl_arg[2] = (uint32_t) strtol(argv[3], NULL, 10);
  fast_ioctl_arg[3] = (uint32_t) strtol(argv[4], NULL, 10);
  return send_fast_ioctl(FAST_IOCTL_GPIO_CONFIGURE_PIN, (ulong_t) fast_ioctl_arg);
}

/****************************************************************************
 * Name: do_set_pin
 ****************************************************************************/
static int32_t do_set_pin(int32_t argc, char **argv)
{
  /* fast gpio !config !argv[1] ... !argv[3] */
  if (argc != 4)
    {
      do_gpio_exp_help(argc, argv);
      return -EINVAL;
    }

  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  fast_ioctl_arg[1] = (uint32_t) strtol(argv[2], NULL, 10);
  fast_ioctl_arg[2] = (uint32_t) strtol(argv[3], NULL, 10);
  return send_fast_ioctl(FAST_IOCTL_GPIO_SET_PIN_LEVEL, (ulong_t) fast_ioctl_arg);
}

/****************************************************************************
 * Name: do_get_pin
 ****************************************************************************/
static int32_t do_get_pin(int32_t argc, char **argv)
{
  int32_t ret_code = -1;
  /* fast gpio !config !argv[1] !argv[2] */
  if (argc != 3)
    {
      do_gpio_exp_help(argc, argv);
      return -EINVAL;
    }

  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  fast_ioctl_arg[1] = (uint32_t) strtol(argv[2], NULL, 10);
  ret_code = send_fast_ioctl(FAST_IOCTL_GPIO_GET_PIN_LEVEL, (ulong_t) fast_ioctl_arg);
  if (ret_code == FAST_OK)
    {
      printf("FAST GPIO pin level port: %d pin: %d pin level : %d\n", fast_ioctl_arg[0], fast_ioctl_arg[1], fast_ioctl_arg[2]);
    }
  return ret_code;
}

/****************************************************************************
 * Name: do_clear_irq
 ****************************************************************************/
static int32_t do_clear_irq(int32_t argc, char **argv)
{
  int32_t ret_code = -1;
  ret_code = send_fast_ioctl(FAST_IOCTL_GPIO_CLEAR_IRQ, (ulong_t) fast_ioctl_arg);
  if (ret_code == FAST_OK)
    {
      printf("FAST GPIO IRQ status : %d\n", fast_ioctl_arg[0]);
    }
  return ret_code;
}

/****************************************************************************
 * Name: do_disable_pin
 ****************************************************************************/
static int32_t do_disable_pin(int32_t argc, char **argv)
{
  /* fast gpio !disable_pin !argv[1] !argv[2] */
  if (argc != 3)
    {
      return -1;
    }
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  fast_ioctl_arg[1] = (uint32_t) strtol(argv[2], NULL, 10);
  return send_fast_ioctl(FAST_IOCTL_GPIO_DISABLE_PIN, (ulong_t) fast_ioctl_arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fast_gpio_exp
 ****************************************************************************/
int32_t fast_gpio_exp(int32_t argc, FAR char **argv)
{
  int32_t ret = -1;
  if (argc == 1)
    {
      do_gpio_exp_help(argc, argv);
      return -EINVAL;
    }
  set_errno(0);
  dbg_tag();
  ret = fast_parse(g_fast_gpio_exp, argc, argv);
  return ret;
}
