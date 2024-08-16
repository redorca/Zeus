/****************************************************************************
 *   system/zglue_fast/fast_pmic.c
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

static int32_t do_pmic_help(int32_t argc, char **argv);
static int32_t do_pmic_boost(int32_t argc, char **argv);
static int32_t do_pmic_hvldo(int32_t argc, char **argv);
static int32_t do_pmic_ldo_config(int32_t argc, char **argv);
static int32_t do_pmic_ldo_vout(int32_t argc, char **argv);
static int32_t do_pmic_ldo_vout_get(int32_t argc, char **argv);
static int32_t do_pmic_thermal_config(int32_t argc, char **argv);
static int32_t do_pmic_thermal_enable(int32_t argc, char **argv);
static int32_t do_pmic_thermal_disable(int32_t argc, char **argv);
static int32_t do_pmic_ldo_enable(int32_t argc, char **argv);
static int32_t do_pmic_ldo_disable(int32_t argc, char **argv);
extern uint32_t fast_ioctl_arg[FAST_IOCTL_ARG_COUNT];


/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct fast_cmdmap_s g_fast_pmic[] =
{
  { "help",      do_pmic_help,        "FAST PMIC help",  NULL},
  { "boost",     do_pmic_boost,       "FAST PMIC boost controls",  "\n\t [voltage out (0=2.5V, 15=4.5V)]\n\t [Bypass mode (0=Disabled, 1=Force bypass, 2=Auto bypass)]\n\t [Current limit (0=200mA, 1=300mA, 2=400mA, 3=500mA)]"},
  { "sysldo",     do_pmic_hvldo,       "FAST PMIC sysldo controls",  "\n\t [voltage out (0=0.5V, 31=3.6V)]\n\t [Bypass mode (0=Disabled, 1=Force bypass, 2=Auto bypass)]\n\t [Current limit Enable (1=Enable, 0=Disable)]"},
  { "ldo_config",     do_pmic_ldo_config,       "FAST PMIC ldo controls",  "\n\t [ldo_id (1, 2, 3)]\n\t [Bypass mode (1=Enable, 0=Disable)]\n\t [Current limit Enable (1=Enable, 0=Disable)]"},
  { "ldo_enable",     do_pmic_ldo_enable,       "FAST PMIC ldo enable",  "\n\t [ldo mask (Bit0 : LDO1, Bit1:LDO2, Bit3:LDO3)]"},
  { "ldo_disable",     do_pmic_ldo_disable,       "FAST PMIC ldo disable",  "\n\t [ldo mask (Bit0 : LDO1, Bit1:LD02, Bit3:LD03)]"},
  { "ldo_vout_set",     do_pmic_ldo_vout,       "Set FAST PMIC ldo outout voltage",  "\n\t [ldo_id (1, 2, 3)]\n\t [voltage out (0=0.5V, 31=3.6V)]\n\t"},
  { "ldo_vout_get",     do_pmic_ldo_vout_get,   "Get FAST PMIC ldo outout voltage",  "\n\t [ldo_id (1, 2, 3)]\n\t"},
  { "thermal_config",      do_pmic_thermal_config,        "FAST PMIC thermal monitor control configure",  "\n\t [alarm temp (0=45C, 1=65C, 2=85C, 3=105C, 4=125C, 5=145C, 6=165C)]\n\t [shutdown temp (0=110C, 1=135C, 2=150C, 3=165C)]"},
  { "thermal_enable",      do_pmic_thermal_enable,        "FAST PMIC thermal monitor control enable"},
  { "thermal_disable",      do_pmic_thermal_disable,      "FAST PMIC thermal monitor control disable"},
  { NULL,   NULL,        NULL,             NULL },
};

/****************************************************************************
 * Name: do_pmic_help
 ****************************************************************************/
static int32_t do_pmic_help(int32_t argc, char **argv)
{
  const struct fast_cmdmap_s *ptr;

  printf("Usage: fast pmic <cmd> [arguments]\n");
  printf("Where <cmd> is one of:\n\n");
  for (ptr = (g_fast_pmic + 1); ptr->cmd; ptr++)
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
 * Name: do_pmic_boost
 ****************************************************************************/
static int32_t do_pmic_boost(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc < 4)
    {
      do_pmic_help(argc, argv);
      return -EINVAL;
    }
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  fast_ioctl_arg[1] = (uint32_t) strtol(argv[2], NULL, 10);
  fast_ioctl_arg[2] = (uint32_t) strtol(argv[3], NULL, 10);
  ret = send_fast_ioctl(FAST_IOCTL_PMIC_BOOST_CONFIGURE, (ulong_t)fast_ioctl_arg);
  return ret;
}

/****************************************************************************
 * Name: do_pmic_hvldo
 ****************************************************************************/
static int32_t do_pmic_hvldo(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc < 4)
    {
      do_pmic_help(argc, argv);
      return -EINVAL;
    }
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  fast_ioctl_arg[1] = (uint32_t) strtol(argv[2], NULL, 10);
  fast_ioctl_arg[2] = (uint32_t) strtol(argv[3], NULL, 10);
  ret = send_fast_ioctl(FAST_IOCTL_PMIC_HVLDO_CONFIGURE, (ulong_t)fast_ioctl_arg);
  return ret;
}

/****************************************************************************
 * Name: do_pmic_ldo_config
 ****************************************************************************/
static int32_t do_pmic_ldo_config(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc < 4)
    {
      do_pmic_help(argc, argv);
      return -EINVAL;
    }
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  fast_ioctl_arg[1] = (uint32_t) strtol(argv[2], NULL, 10);
  fast_ioctl_arg[2] = (uint32_t) strtol(argv[3], NULL, 10);
  ret = send_fast_ioctl(FAST_IOCTL_PMIC_VRAIL_CONFIGURE, (ulong_t)fast_ioctl_arg);
  return ret;
}

/****************************************************************************
 * Name: do_pmic_ldo_enable
 ****************************************************************************/
static int32_t do_pmic_ldo_enable(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc < 2)
    {
      do_pmic_help(argc, argv);
      return -EINVAL;
    }
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  ret = send_fast_ioctl(FAST_IOCTL_PMIC_LDO_ENABLE, (ulong_t)fast_ioctl_arg);
  return ret;
}

/****************************************************************************
 * Name: do_pmic_ldo_disable
 ****************************************************************************/
static int32_t do_pmic_ldo_disable(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc < 2)
    {
      do_pmic_help(argc, argv);
      return -EINVAL;
    }
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  ret = send_fast_ioctl(FAST_IOCTL_PMIC_LDO_DISABLE, (ulong_t)fast_ioctl_arg);
  return ret;
}

/****************************************************************************
 * Name: do_pmic_ldo_vout
 ****************************************************************************/
static int32_t do_pmic_ldo_vout(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc < 3)
    {
      do_pmic_help(argc, argv);
      return -EINVAL;
    }
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  fast_ioctl_arg[1] = (uint32_t) strtol(argv[2], NULL, 10);
  ret = send_fast_ioctl(FAST_IOCTL_PMIC_VRAIL_VOUT, (ulong_t)fast_ioctl_arg);
  return ret;
}

/****************************************************************************
 * Name: do_pmic_ldo_vout_get
 ****************************************************************************/
static int32_t do_pmic_ldo_vout_get(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc < 2)
    {
      do_pmic_help(argc, argv);
      return -EINVAL;
    }
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  ret = send_fast_ioctl(FAST_IOCTL_PMIC_VRAIL_VOUT_GET, (ulong_t)fast_ioctl_arg);
  if (ret == 0)
    {
      printf("LDO %d current voltage setting : %.1fV(%d)\n", fast_ioctl_arg[0], ((0.5) + 0.1 * fast_ioctl_arg[1]), fast_ioctl_arg[1]);
    }
  return ret;
}


/****************************************************************************
 * Name: do_pmic_thermal_config
 ****************************************************************************/
static int32_t do_pmic_thermal_config(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc < 2)
    {
      do_pmic_help(argc, argv);
      return -EINVAL;
    }
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  fast_ioctl_arg[1] = (uint32_t) strtol(argv[2], NULL, 10);
  ret = send_fast_ioctl(FAST_IOCTL_PMIC_THERMAL_MONITOR_CONFIGURE, (ulong_t)fast_ioctl_arg);

  return ret;
}

/****************************************************************************
 * Name: do_pmic_thermal_enable
 ****************************************************************************/
static int32_t do_pmic_thermal_enable(int32_t argc, char **argv)
{
  int32_t ret = -1;
  ret = send_fast_ioctl(FAST_IOCTL_PMIC_THERMAL_MONITOR_ENABLE, (ulong_t)fast_ioctl_arg);
  return ret;
}

/****************************************************************************
 * Name: do_pmic_thermal_disable
 ****************************************************************************/
static int32_t do_pmic_thermal_disable(int32_t argc, char **argv)
{
  int32_t ret = -1;
  ret = send_fast_ioctl(FAST_IOCTL_PMIC_THERMAL_MONITOR_DISABLE, (ulong_t)fast_ioctl_arg);
  return ret;
}

/****************************************************************************
 * Name: fast_pmic
 ****************************************************************************/
int32_t fast_pmic(int32_t argc, FAR char **argv)
{
  int32_t ret = -1;
  set_errno(0);
  dbg_tag();
  if (argc == 1)
    {
      do_pmic_help(argc, argv);
      return -EINVAL;
    }
  ret = fast_parse(g_fast_pmic, argc, argv);
  return ret;
}
