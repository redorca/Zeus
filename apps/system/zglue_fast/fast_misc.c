/****************************************************************************
 *   system/zglue_fast/fast_misc.c
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

static int32_t do_init_help(int32_t argc, char **argv);
static int32_t do_program_help(int32_t argc, char **argv);
static int32_t do_realignment_help(int32_t argc, char **argv);
static int32_t do_control_help(int32_t argc, char **argv);

/******************************************************************************
 * Public Data
 *****************************************************************************/
/* Common, message formats */

extern const char g_fast_argrequired[];
extern const char g_fast_arginvalid[];
extern const char g_fast_argrange[];
extern const char g_fast_cmdnotfound[];
extern const char g_fast_toomanyargs[];
extern const char g_fast_cmdfailed[];
extern const char g_fast_xfrerror[];

static int32_t do_debug_help(int32_t argc, char **argv);
static int32_t do_set_debuglevel(int32_t argc, char **argv);
static int32_t do_get_debuglevel(int32_t argc, char **argv);

static int32_t do_interface_help(int32_t argc, char **argv);
static int32_t do_set_interface(int32_t argc, char **argv);
static int32_t do_get_interface(int32_t argc, char **argv);
static int32_t do_fast_spi(int32_t argc, char **argv);
static int32_t do_fast_i2c(int32_t argc, char **argv);
#if defined(CONFIG_ZEUS2)
static int32_t do_zcad_program(int32_t argc, char **argv);
static int32_t do_zcad_version(int32_t argc, char **argv);
#endif
extern uint32_t fast_ioctl_arg[10];
extern uint8_t fast_debug_level;
uint8_t fast_print_status = 0;

static const struct fast_cmdmap_s g_fast_init[] =
{
  { "help",          do_init_help,       "",                              NULL},
#if defined(CONFIG_ZEUS1)
  { "init",        fast_nsh_init,     "fast init",                "\n\t [mcu on fast or not (1=mcu containing fast api is on fast, 0=mcu is off fast)]\n\t [debug level (1=print error msg, 2=print warning msg, 3=print all info)]\n\t [operation timeout(0-100ms)]"},
#elif defined(CONFIG_ZEUS2)
  { "init",        fast_nsh_init,     "fast init",                "\n\t [mcu on fast or not (1=mcu containing fast api is on fast, 0=mcu is off fast)]\n\t [group mode command enable (1=enable, 0=disable)]\n\t [(optional)debug level (1(Default)=print error msg, 2=print warning msg, 3=print all info)]\n\t [(optional)operation timeout 0-100ms Default : 50ms]\n\t [(optional)Status print (0(Default)=No status print, 1=print status]"},
#else
#error No ZEUS architecture defined: ZEUS1 | ZEUS2
#endif
  { NULL,            NULL,              NULL,                            NULL },
};

static const struct fast_cmdmap_s g_fast_debug[] =
{
  { "help",      do_debug_help,       "",  NULL},
  { "set",       do_set_debuglevel,        "fast debug set",  "\n\t [Debug level (0=No prints, 1=Errors, 2=Warnings, 3=Info)]"},
  { "get",       do_get_debuglevel,        "fast debug get (Returns the current debug level)",  NULL},
  { NULL,   NULL,        NULL,             NULL },
};

static const struct fast_cmdmap_s g_fast_interface[] =
{
  { "help",      do_interface_help,       "",  NULL},
  { "set",       do_set_interface,        "fast interface set",  "\n\t [Interface (0=SPI, 1=I2C, 2=JTAG)]"},
  { "get",       do_get_interface,        "fast interface get (Returns the current interface used)", NULL},
  { "spi",       do_fast_spi,             "fast interface spi",  "\n\t [bit order (0=MSB, 1=LSB)]\n\t [bit mode (0=8bit, 1=16bit, 2=32bit)]\n\t [cpol (0, 1)]\n\t [cpha (0, 1)]"},
  { "i2c",       do_fast_i2c,             "fast interface i2c",  "\n\t [(1=Enable, 0=Disable)]  \n\t [I2c addr<hex>]"},
  { NULL,   NULL,        NULL,             NULL },
};

static const struct fast_cmdmap_s g_fast_program_cmd[] =
{
  { "help",          do_program_help,       "",                              NULL},
  { "program",  fast_program_cmd,     "fast program registers",    "\n\t [file with programing info]"  "\n\t [connections on/off] (1=turn on/off, 0=no action )"},
  { NULL,            NULL,              NULL,                            NULL },
};

#if defined(CONFIG_ZEUS2)
static const struct fast_cmdmap_s g_fast_zcad[] =
{
  { "help",      do_debug_help,       "",  NULL},
  { "program",       do_zcad_program,        "fast program zcad data",  "\n\t [file]" },
  { "version",       do_zcad_version,        "fast get zcad file version",  NULL},
  { NULL,   NULL,        NULL,             NULL },
};
#endif

static const struct fast_cmdmap_s g_fast_realignment_cmd[] =
{
  { "help",          do_realignment_help,       "",                              NULL},
  { "realign",  fast_realignment,     "fast realignment ",    "\n\t [input file]"  "\n\t [output file]"},
  { NULL,            NULL,              NULL,                            NULL },
};

static const struct fast_cmdmap_s g_fast_control[] =
{
  { "help",          do_control_help,       "",                              NULL},
  { "bcbyp",  do_control_bootcfg_byp,     "fast set bootconfig_bypass",    "\n\t [0=use OTP]"  "\n\t [1=bypass OTP]"},
  { "nRF_EN_SW",  do_control_nRF_EN_SW,     "fast set nRF_EN_SW",    "\n\t [0=Set voltage level low]"  "\n\t [1=Set voltage level high]"},
  { NULL,            NULL,              NULL,                            NULL },
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: do_init_help
 ****************************************************************************/
static int32_t do_init_help(int32_t argc, char **argv)
{
  const struct fast_cmdmap_s *ptr;

  printf("Usage: fast <cmd> [arguments]\n");
  printf("Where <cmd> is one of:\n\n");
  for (ptr = (g_fast_init + 1); ptr->cmd; ptr++)
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
  return -EINVAL;
}

/****************************************************************************
 * Name: fast_reset
 ****************************************************************************/
#if defined(CONFIG_ZEUS1)
int32_t fast_reset(int32_t argc, char **argv)
{
  return send_fast_ioctl(FAST_IOCTL_ZEUS1_RESET, (ulong_t)fast_ioctl_arg);
}
#endif
/****************************************************************************
 * Name: fast_nsh_init
 ****************************************************************************/
int32_t fast_nsh_init(int32_t argc, FAR char **argv)
{
  int32_t ret = -1;
#if defined(CONFIG_ZEUS1)
  if (argc < 2)
#elif defined(CONFIG_ZEUS2)
  if (argc < 3)
#endif
    {
      do_init_help(argc, argv);
      return -EINVAL;
    }
  uint8_t mcu_on_fast_local = 0, debug_level = 0;
  uint16_t timeout = 0;
  mcu_on_fast_local = strtoul(argv[1], NULL, 10);
#if defined(CONFIG_ZEUS1)
  debug_level = strtoul(argv[2], NULL, 10);
  timeout = strtoul(argv[3], NULL, 10);
  fast_print_status = strtoul(argv[4], NULL, 10);
  if (argc == 2)
    {
      debug_level = FAST_DEBUG_LEVEL_1; //setting default debug level to 1
      timeout = 50; //setting defaul timeout to 50ms
      fast_print_status = 0; //set the default status print to 0
      printf("FAST init setting defaults debug level : %d timeout : %d\r\n", debug_level, timeout);
    }
  else if (argc == 3)
    {
      timeout = 50; //setting defaul timeout to 50ms
      fast_print_status = 0; //set the default status print to 0
      printf("FAST init setting defaults timeout : %d\r\n", timeout);
    }
#elif defined(CONFIG_ZEUS2)
  debug_level = strtoul(argv[3], NULL, 10);
  timeout = strtoul(argv[4], NULL, 10);
  fast_print_status = strtoul(argv[5], NULL, 10);
  if (argc == 3)
    {
      debug_level = FAST_DEBUG_LEVEL_1; //setting default debug level to 1
      timeout = 50; //setting defaul timeout to 50ms
      fast_print_status = 0; //set the default status print to 0
      printf("FAST init setting defaults debug level : %d timeout : %d Status print : %d\r\n", debug_level, timeout, fast_print_status);
    }
  else if (argc == 4)
    {
      timeout = 50; //setting default timeout to 50ms
      fast_print_status = 0; //set the default status print to 0
      printf("FAST init setting defaults timeout : %d Status print : %d\r\n", timeout, fast_print_status);
    }
#endif

  fast_ioctl_arg[0] = (uint32_t)debug_level;
  ret = send_fast_ioctl(FAST_IOCTL_SET_DEBUG_LEVEL, (ulong_t)fast_ioctl_arg);
#if defined(CONFIG_ZEUS2)
  uint8_t group_mode_enable = 0;
  group_mode_enable = strtoul(argv[2], NULL, 10);
  fast_ioctl_arg[0] = (uint32_t)group_mode_enable;
  ret = send_fast_ioctl(FAST_IOCTL_GROUP_MODE_CONFIG, (ulong_t)fast_ioctl_arg);
#endif
  fast_ioctl_arg[0] = (uint32_t)timeout;
  ret = send_fast_ioctl(FAST_IOCTL_SET_TIMEOUT, (ulong_t)fast_ioctl_arg);
  fast_ioctl_arg[0] = (uint32_t)mcu_on_fast_local;
  ret = send_fast_ioctl(FAST_IOCTL_API_INIT, (ulong_t)fast_ioctl_arg);
  return ret;
}



/****************************************************************************
 * Name: fast_close
 ****************************************************************************/
int32_t fast_nsh_close(int32_t argc, FAR char **argv)
{
  int32_t ret = -1;
  fast_ioctl_arg[0] = strtoul(argv[2], NULL, 10);
  ret = send_fast_ioctl(FAST_IOCTL_API_CLOSE, (ulong_t)fast_ioctl_arg);
  return ret;
}


/****************************************************************************
 * Name: fast_connect
 ****************************************************************************/
int32_t fast_id(int32_t argc, FAR char **argv)
{
  int32_t ret = -1;
  set_errno(0);
  ret = send_fast_ioctl(FAST_IOCTL_READ_ID, (ulong_t)fast_ioctl_arg);
  if (ret == 0)
    {
      printf("Chip ID:0x%llX\n", (uint64_t) * (uint64_t *)fast_ioctl_arg);
    }
  return ret;
}

/****************************************************************************
 * Name: do_debug_help
 ****************************************************************************/
static int32_t do_debug_help(int32_t argc, char **argv)
{
  const struct fast_cmdmap_s *ptr;

  printf("Usage: fast debug <cmd> [arguments]\n");
  printf("Where <cmd> is one of:\n\n");
  for (ptr = (g_fast_debug + 1); ptr->cmd; ptr++)
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
 * Name: do_get_debuglevel
 ****************************************************************************/
static int32_t do_get_debuglevel(int32_t argc, char **argv)
{
  int32_t ret = -1;
  ret = send_fast_ioctl(FAST_IOCTL_GET_DEBUG_LEVEL, (ulong_t) fast_ioctl_arg);
  printf("\tCurrent Debug level : %d\r\n", fast_ioctl_arg[0]);
  return ret;
}


/****************************************************************************
 * Name: do_set_debuglevel
 ****************************************************************************/
static int32_t do_set_debuglevel(int32_t argc, char **argv)
{
  int32_t ret = -1;
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  ret = send_fast_ioctl(FAST_IOCTL_SET_DEBUG_LEVEL, (ulong_t)fast_ioctl_arg);
  return ret;
}

/****************************************************************************
 * Name: do_set_debuglevel
 ****************************************************************************/
int32_t fast_debug(int32_t argc, FAR char **argv)
{
  int32_t ret = -1;
  set_errno(0);
  dbg_tag();
  if (argc == 1)
    {
      do_debug_help(argc, argv);
      return -EINVAL;
    }
  ret = fast_parse(g_fast_debug, argc, argv);
  return ret;
}

/****************************************************************************
 * Name: do_set_debuglevel
 ****************************************************************************/
int32_t fast_interface(int32_t argc, FAR char **argv)
{
  int32_t ret = -1;
  set_errno(0);
  dbg_tag();
  if (argc == 1)
    {
      do_interface_help(argc, argv);
      return -EINVAL;
    }
  ret = fast_parse(g_fast_interface, argc, argv);
  return ret;
}

/****************************************************************************
 * Name: do_set_interface
 ****************************************************************************/
static int32_t do_set_interface(int32_t argc, char **argv)
{
  int32_t ret = -1;
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  ret = send_fast_ioctl(FAST_IOCTL_INTERFACE_SET, (ulong_t)fast_ioctl_arg);
  return ret;
}

/****************************************************************************
 * Name: do_get_interface
 ****************************************************************************/
static int32_t do_get_interface(int32_t argc, char **argv)
{
  int32_t ret = -1;
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  ret = send_fast_ioctl(FAST_IOCTL_INTERFACE_GET, (ulong_t)fast_ioctl_arg);
  switch (fast_ioctl_arg[0])
    {
      case 0:
        printf("\r\nCurrent FAST interface: SPI\r\n");
        break;
      case 1:
        printf("\r\nCurrent FAST interface: I2C\r\n");
        break;
      case 2:
        printf("\r\nCurrent FAST interface: JTAG\r\n");
        break;
      default:
        printf("\r\nCurrent FAST interface: Invalid\r\n");
        break;
    }
  return ret;
}

/****************************************************************************
 * Name: do_set_debuglevel
 ****************************************************************************/
static int32_t do_fast_spi(int32_t argc, char **argv)
{
  int32_t ret = -1;
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  fast_ioctl_arg[1] = (uint32_t) strtol(argv[2], NULL, 10);
  fast_ioctl_arg[2] = (uint32_t) strtol(argv[3], NULL, 10);
  fast_ioctl_arg[3] = (uint32_t) strtol(argv[4], NULL, 10);
  ret = send_fast_ioctl(FAST_IOCTL_SPI_CONFIGURE, (ulong_t)fast_ioctl_arg);
  return ret;
}

/****************************************************************************
 * Name: do_set_debuglevel
 ****************************************************************************/
static int32_t do_fast_i2c(int32_t argc, char **argv)
{
  int32_t ret = -1;
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  fast_ioctl_arg[1] = (uint32_t) strtol(argv[2], NULL, 16);
  ret = send_fast_ioctl(FAST_IOCTL_I2C_CONFIGURE, (ulong_t)fast_ioctl_arg);
  return ret;
}

/****************************************************************************
 * Name: do_debug_help
 ****************************************************************************/
static int32_t do_interface_help(int32_t argc, char **argv)
{
  const struct fast_cmdmap_s *ptr;

  printf("Usage: fast interface <cmd> [arguments]\n");
  printf("Where <cmd> is one of:\n\n");
  for (ptr = (g_fast_interface + 1); ptr->cmd; ptr++)
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
 * Name: fast program registers
 ****************************************************************************/
#if defined(CONFIG_ZEUS2)
int32_t fast_program_cmd(int32_t argc, char **argv)
{
  int32_t ret = -1;

  char *string = argv[1];
  fast_ioctl_arg[0] = (uint32_t)string;
  fast_ioctl_arg[1] = 0;
  if (argc == 1)
    {
      do_program_help(argc, argv);
      return -EINVAL;
    }
  if (argc == 3)
    {
      fast_ioctl_arg[1] = (uint32_t) strtol(argv[2], NULL, 10);
    }
  else if (argc == 2)
    {
      fast_ioctl_arg[1] = 0;
    }

  ret = send_fast_ioctl(FAST_IOCTL_PROGRAM, (ulong_t)fast_ioctl_arg);
  return ret;
}
#endif

/****************************************************************************
 * Name: do program registers help
 ****************************************************************************/
static int32_t do_program_help(int32_t argc, char **argv)
{
  const struct fast_cmdmap_s *ptr;

  printf("Usage: fast <cmd> [arguments]\n");
  printf("Where <cmd> is one of:\n\n");
  for (ptr = (g_fast_program_cmd + 1); ptr->cmd; ptr++)
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
 * Name: do_zcad_help
 ****************************************************************************/
#if defined(CONFIG_ZEUS2)
static int32_t do_zcad_help(int32_t argc, char **argv)
{
  const struct fast_cmdmap_s *ptr;
  printf("Usage: fast zcad <cmd> [arguments]\n");
  printf("Where <cmd> is one of:\n\n");
  for (ptr = (g_fast_zcad + 1); ptr->cmd; ptr++)
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
#endif

/****************************************************************************
 * Name: fast_zcad_cmd
 ****************************************************************************/
#if defined(CONFIG_ZEUS2)
int32_t fast_zcad_cmd(int32_t argc, FAR char **argv)
{
  int32_t ret = -1;
  set_errno(0);
  dbg_tag();
  if (argc == 1)
    {
      do_zcad_help(argc, argv);
      return -EINVAL;
    }
  ret = fast_parse(g_fast_zcad, argc, argv);
  return ret;
}
#endif

/****************************************************************************
 * Name: fast zcad program registers
 ****************************************************************************/
#if defined(CONFIG_ZEUS2)
static int32_t do_zcad_program(int32_t argc, char **argv)
{
  int32_t ret = -1;

  char *string = argv[1];
  fast_ioctl_arg[0] = (uint32_t)string;
  if (argc == 1)
    {
      do_zcad_help(argc, argv);
      return -EINVAL;
    }
  ret = send_fast_ioctl(FAST_IOCTL_ZCAD_PROGRAM, (ulong_t)fast_ioctl_arg);
  return ret;
}
#endif

/****************************************************************************
 * Name: fast get zcad file version
 ****************************************************************************/
#if defined(CONFIG_ZEUS2)
static int32_t do_zcad_version(int32_t argc, char **argv)
{
  int32_t ret = -1;
  char version[32] = {0};
  if (argc == 1)
    {
      do_zcad_help(argc, argv);
      return -EINVAL;
    }
  char *string = argv[1];
  fast_ioctl_arg[0] = (uint32_t)string;
  fast_ioctl_arg[1] = (uint32_t)version;
  ret = send_fast_ioctl(FAST_IOCTL_ZCAD_VERSION, (ulong_t)fast_ioctl_arg);
  if (ret == 0)
    {
      for (int i = 0; i < strlen(version); i++)
        {
          printf("%c", version[i]);
        }
      printf("\r\n");
    }
  return ret;
}
#endif

/****************************************************************************
 * Name: fast realignment
 ****************************************************************************/
#if defined(CONFIG_ZEUS2)
int32_t fast_realignment(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc < 3)
    {
      do_realignment_help(argc, argv);
      return -EINVAL;
    }
  char *input_file = argv[1];
  char *output_file = argv[2];
  fast_ioctl_arg[0] = (uint32_t)input_file;
  fast_ioctl_arg[1] = (uint32_t)output_file;
  ret = send_fast_ioctl(FAST_IOCTL_REALIGNMENT, (ulong_t)fast_ioctl_arg);
  return ret;
}
#endif

/****************************************************************************
 * Name: do realignment help
 ****************************************************************************/
static int32_t do_realignment_help(int32_t argc, char **argv)
{
  const struct fast_cmdmap_s *ptr;
  printf("Usage: fast <cmd> [arguments]\n");
  printf("Where <cmd> is one of:\n\n");
  for (ptr = (g_fast_realignment_cmd + 1); ptr->cmd; ptr++)
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
 * Name: do control help
 ****************************************************************************/
static int32_t do_control_help(int32_t argc, char **argv)
{
  const struct fast_cmdmap_s *ptr;
  printf("Usage: fast <cmd> [arguments]\n");
  printf("Where <cmd> is one of:\n\n");
  for (ptr = (g_fast_control + 1); ptr->cmd; ptr++)
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

#if defined(CONFIG_ZEUS2_DEVKIT_FEATURES)

/****************************************************************************
 * Name: fast control smart fabric on devkit
 ****************************************************************************/
int32_t fast_control(int32_t argc, FAR char **argv)
{
  int32_t ret = -1;

  if (argc < 3)
    {
      do_control_help(argc, argv);
      return -EINVAL;
    }
  ret = fast_parse(g_fast_control, argc, argv);
  return ret;
}

/****************************************************************************
 * Name: fast control bootconfig bypass on devkit
 ****************************************************************************/
int32_t do_control_bootcfg_byp(int32_t argc, FAR char **argv)
{
  int32_t ret = -1;
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  ret = send_fast_ioctl(FAST_IOCTL_DEVKIT_ZEUS2_BOOTCFG_BYP, (ulong_t)fast_ioctl_arg);
  return ret;
}

/****************************************************************************
 * Name: fast control nRF_EN_SW on nrf52840 devkit
 ****************************************************************************/
int32_t do_control_nRF_EN_SW(int32_t argc, FAR char **argv)
{
  int32_t ret = -1;
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  ret = send_fast_ioctl(FAST_IOCTL_DEVKIT_ZEUS2_nRF_EN_SW, (ulong_t)fast_ioctl_arg);
  return ret;
}

#endif /*defined(CONFIG_ZEUS2)*/
