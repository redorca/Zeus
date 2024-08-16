/****************************************************************************
 *   system/zglue_fast/fast_powermgt.c
 *
 *   Copyright (C) 2011, 2016 Gregory Nutt. All rights reserved.
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
#include <fast_cmd.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int32_t do_powerstate_help(int32_t argc, char **argv);
static int32_t do_powerstate_set(int32_t argc, char **argv);
static int32_t do_powerstate_get(int32_t argc, char **argv);
static int32_t do_lpm_fsm_enable(int32_t argc, char **argv);
static int32_t do_lpm(int32_t argc, char **argv);
#if defined(CONFIG_ZEUS2_DEVKIT_FEATURES) && defined(CONFIG_ZEUS2)
static int32_t do_power_on(int32_t argc, char **argv);
static int32_t do_power_off(int32_t argc, char **argv);
static int32_t do_power_status(int32_t argc, char **argv);
#endif

static const struct fast_cmdmap_s g_fast_power[] =
{
  { "help",      do_powerstate_help,        "",  NULL},
  { "set",       do_powerstate_set,         "fast power set",  "\n\t [Power state (0=FPM, 1=RCM, 2=LPM, 3=ULPM)]"},
  { "get",       do_powerstate_get,         "fast power get (Returns the current power state)", NULL},
  { "lpm_fsm_enable",   do_lpm_fsm_enable,  "fast power lpm_fsm_enable",  "\n\t [(1=Enable, 0=Disable)]"},
  { "lpm",       do_lpm,                    "fast power lpm (Triggers the LPM HW  FSM to go into LPM)",  NULL},
#if defined(CONFIG_ZEUS2_DEVKIT_FEATURES) && defined(CONFIG_ZEUS2)
  { "on",        do_power_on,               "fast power on (Turns on the chip by asserting EN_L and EN_H)",  NULL},
  { "off",       do_power_off,              "fast power off (Turns off the chip using mcu power off)",  NULL},
  { "status",    do_power_status,           "fast power status (Returns the status of whole fast chip)",  NULL},
#endif
  { NULL,   NULL,        NULL,             NULL },
};

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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: do_powerstate_help
 ****************************************************************************/

static int32_t do_powerstate_help(int32_t argc, char **argv)
{
  const struct fast_cmdmap_s *ptr;

  printf("Usage: fast power <cmd> [arguments]\n");
  printf("Where <cmd> is one of:\n\n");
  for (ptr = (g_fast_power + 1); ptr->cmd; ptr++)
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
 * Name: do_powerstate_set
 ****************************************************************************/

static int32_t do_powerstate_set(int32_t argc, char **argv)
{
  if (argc < 2)
    {
      do_powerstate_help(argc, argv);
      return -EINVAL;
    }
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  return send_fast_ioctl(FAST_IOCTL_ENTER_POWER_STATE, (ulong_t) fast_ioctl_arg);
}

/****************************************************************************
 * Name: do_powerstate_get
 ****************************************************************************/
static int32_t do_powerstate_get(int32_t argc, char **argv)
{
  int32_t ret = -1;
  ret = send_fast_ioctl(FAST_IOCTL_GET_CURRENT_POWER_STATE, (ulong_t)fast_ioctl_arg);
  if (ret == 0)
    {
      switch ((fast_power_state_t)fast_ioctl_arg[0])
        {
          case FAST_FULL_PROGRAMMABLE_MODE:
            printf("\r\nCurrent FAST power state: Full programmable mode(FPM)\n");
            break;
          case FAST_LIMITED_PROGRAMMABLE_MODE:
            printf("\r\nCurrent FAST power state: Limited programming mode\n");
            break;
          case FAST_LPM:
            printf("\r\nCurrent FAST power state: Low power mode(LPM)\n");
            break;
          case FAST_ULPM:
            printf("\r\nCurrent FAST power state: Ultra low power mode(ULPM)\n");
            break;
          default:
            printf("\r\nCurrent FAST power state: Invalid\n");
            break;
        }
    }
  return ret;
}

/****************************************************************************
 * Name: do_lpm_fsm_enable
 ****************************************************************************/

static int32_t do_lpm_fsm_enable(int32_t argc, char **argv)
{
  if (argc < 2)
    {
      do_powerstate_help(argc, argv);
      return -EINVAL;
    }
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  return send_fast_ioctl(FAST_IOCTL_LPM_FSM_CONTROL, (ulong_t) fast_ioctl_arg);
}

/****************************************************************************
 * Name: do_lpm
 ****************************************************************************/
static int32_t do_lpm(int32_t argc, char **argv)
{
  return send_fast_ioctl(FAST_IOCTL_LPM_FSM_TRIGGER, (ulong_t) fast_ioctl_arg);
}

#if defined(CONFIG_ZEUS2_DEVKIT_FEATURES) && defined(CONFIG_ZEUS2)
/****************************************************************************
 * Name: do_power_on
 ****************************************************************************/
static int32_t do_power_on(int32_t argc, char **argv)
{
  return send_fast_ioctl(FAST_IOCTL_DEVKIT_ZEUS2_PWRON, (ulong_t) fast_ioctl_arg);
}

/****************************************************************************
 * Name: do_power_off
 ****************************************************************************/
static int32_t do_power_off(int32_t argc, char **argv)
{
  return send_fast_ioctl(FAST_IOCTL_DEVKIT_ZEUS2_PWROFF, (ulong_t) fast_ioctl_arg);
}

/****************************************************************************
 * Name: do_power_off
 ****************************************************************************/
static int32_t do_power_status(int32_t argc, char **argv)
{
  int32_t ret = -1;
  ret = send_fast_ioctl(FAST_IOCTL_DEVKIT_ZEUS2_STATUS, (ulong_t) fast_ioctl_arg);
  if (ret == 0)
    {
      printf("\r\nCurrent FAST status: 0x%X\r\n", (uint16_t)fast_ioctl_arg[0]);
    }
  return ret;
}
#endif


/****************************************************************************
 * Name: fast_power
 ****************************************************************************/

int32_t fast_powermgt(int32_t argc, FAR char **argv)
{
  int32_t ret = -1;
  if (argc == 1)
    {
      do_powerstate_help(argc, argv);
      return -EINVAL;
    }
  ret = fast_parse(g_fast_power, argc, argv);
  return ret;
}
