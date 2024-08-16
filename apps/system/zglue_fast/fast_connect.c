/****************************************************************************
 *   system/zglue_fast/fast_connect.c
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
#include <string.h>


static int32_t do_connect_help(int32_t argc, char **argv);
static int32_t do_disconnect_help(int32_t argc, char **argv);
static int32_t do_system_help(int32_t argc, char **argv);
static int32_t do_connect_system(int32_t argc, char **argv);
static int32_t do_connect_chip(int32_t argc, char **argv);
static int32_t do_disconnect_system(int32_t argc, char **argv);
static int32_t do_disconnect_chip(int32_t argc, char **argv);
static int32_t do_system_info(int32_t argc, char **argv);
static int32_t do_system_chipinfo(int32_t argc, char **argv);
#if defined(CONFIG_ZEUS2)
static int32_t do_connect_peripheral(int32_t argc, char **argv);
static int32_t do_disconnect_peripheral(int32_t argc, char **argv);
static int32_t do_connect_chip_vrail(int32_t argc, char **argv);
#endif
extern uint32_t fast_ioctl_arg[FAST_IOCTL_ARG_COUNT];

/******************************************************************************
 * Private Data
 *****************************************************************************/

/* Common, message formats */
extern const char g_fast_argrequired[];
extern const char g_fast_arginvalid[];
extern const char g_fast_argrange[];
extern const char g_fast_cmdnotfound[];
extern const char g_fast_toomanyargs[];
extern const char g_fast_cmdfailed[];
extern const char g_fast_xfrerror[];

static const struct fast_cmdmap_s g_fast_connect[] =
{
  { "help",         do_connect_help,       "Help for connect system",       NULL },
  { "chip",         do_connect_chip,       "fast connect chip",  "\n\t [config_file_id]\n\t [chip_id]"},
  { "system",       do_connect_system,     "fast connect system", "\n\t [config_file_id]" },
#if defined(CONFIG_ZEUS2)
  { "peripheral",   do_connect_peripheral, "fast connect peripheral", "\n\t [config_file_id]\n\t [peripheral id] (1=row mux, 2=level translator, 3=vrail )"},
  { "chip_vrail",   do_connect_chip_vrail, "fast connect chip to particular vrail",  "\n\t [config_file_id]\n\t [chip_id] \n\t [vrail_id] \n\t ** To connect to default vrail, assign [vrail_id] = -1"},
#endif
  { NULL,           NULL,                   NULL,             NULL }
};

static const struct fast_cmdmap_s g_fast_disconnect[] =
{
  { "help",         do_disconnect_help,       "Help for disconnect system",       NULL },
  { "chip",         do_disconnect_chip,       "fast disconnect chip",  "\n\t [config_file_id]\n\t [chip_id]"},
  { "system",       do_disconnect_system,     "fast disconnect system",  "\n\t [config_file_id]" },
#if defined(CONFIG_ZEUS2)
  { "peripheral",   do_disconnect_peripheral, "fast disconnect peripheral", "\n\t [config_file_id]\n\t [peripheral id] (1=row mux, 2=level translator, 3=vrail )"},
#endif
  { NULL,           NULL,                   NULL,             NULL }
};

static const struct fast_cmdmap_s g_fast_system[] =
{
  { "help",         do_system_help,       "Help for system",       NULL },
  { "info",         do_system_info,       "fast system info",  NULL},
  { "chipinfo",     do_system_chipinfo,   "fast system chipinfo",  NULL },
  { NULL,           NULL,                   NULL,             NULL }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: do_connect_system
 ****************************************************************************/
static int32_t do_connect_system(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc < 2)
    {
      ret = do_connect_help(argc, argv);
      return OK;
    }
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  ret = send_fast_ioctl(FAST_IOCTL_CONNECT_SYSTEM, (ulong_t)fast_ioctl_arg);

  return ret;
}

/****************************************************************************
 * Name: do_connect_chip
 ****************************************************************************/
static int32_t do_connect_chip(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc < 3)
    {
      ret = do_connect_help(argc, argv);
      return -EINVAL;
    }
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  fast_ioctl_arg[1] = (uint32_t) strtol(argv[2], NULL, 10);
  ret = send_fast_ioctl(FAST_IOCTL_CONNECT_CHIP, (ulong_t)fast_ioctl_arg);
  return ret;
}

/****************************************************************************
 * Name: do_disconnect_system
 ****************************************************************************/
static int32_t do_disconnect_system(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc < 2)
    {
      ret = do_disconnect_help(argc, argv);
      return -EINVAL;
    }
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  ret = send_fast_ioctl(FAST_IOCTL_DISCONNECT_SYSTEM, (ulong_t)fast_ioctl_arg);
  return ret;
}

/****************************************************************************
 * Name: do_disconnect_chip
 ****************************************************************************/
static int32_t do_disconnect_chip(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc != 3)
    {
      ret = do_disconnect_help(argc, argv);
      return -EINVAL;
    }
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  fast_ioctl_arg[1] = (uint32_t) strtol(argv[2], NULL, 10);
  ret = send_fast_ioctl(FAST_IOCTL_DISCONNECT_CHIP, (ulong_t)fast_ioctl_arg);
  return ret;
}

/****************************************************************************
 * Name: do_system_info
 ****************************************************************************/
static int32_t do_system_info(int32_t argc, char **argv)
{
  int32_t ret;
  fast_system_information_t sysinfo;
  fast_ioctl_arg[0] = (uint32_t)&sysinfo;
  ret = send_fast_ioctl(FAST_IOCTL_GET_SYSTEM_INFORMATION, (ulong_t)fast_ioctl_arg);
  if (ret == 0)
    {
      printf("\r\nSystem information --- \r\n");
      printf("System ID : %u\r\nSystem Version : %u\r\n", sysinfo.system_id, sysinfo.system_version);
      printf("Config file ID : %u\r\nConfig file Version : %u\r\n", sysinfo.config_file_id, sysinfo.config_file_version);
      printf("Number of chiplets on tilegrid : %u\r\nSupported zeus : %u\r\n",
             sysinfo.number_of_chiplets_on_fast, sysinfo.supported_zeus);

    }
  return ret;
}

/****************************************************************************
 * Name: do_system_chipinfo
 ****************************************************************************/
static int32_t do_system_chipinfo(int32_t argc, char **argv)
{
  int32_t ret;
  int i = 0;
  char chip_name[(CHIP_DESCRIPTOR_SIZE * 2) + 1];
  fast_system_information_t sysinfo;
  fast_ioctl_arg[0] = (uint32_t)&sysinfo;
  ret = send_fast_ioctl(FAST_IOCTL_GET_SYSTEM_INFORMATION, (ulong_t)fast_ioctl_arg);
  fast_system_chips_information_t *chipinfo;
  chipinfo = (fast_system_chips_information_t *) malloc(sizeof(fast_system_chips_information_t) *
                                                        (sysinfo.number_of_chiplets_on_fast));
  if (chipinfo == NULL)
    {
      return -1;
    }
  fast_ioctl_arg[0] = (uint32_t)chipinfo;
  ret = send_fast_ioctl(FAST_IOCTL_GET_SYSTEM_CHIPS_INFORMATION, (ulong_t)fast_ioctl_arg);
  if (ret == 0)
    {
      if (sysinfo.number_of_chiplets_on_fast != 0)
        {
          printf("\r\nInformation about chips on zeus tilegrid  : \r\n");
          for (i = 0; i < sysinfo.number_of_chiplets_on_fast; i++)
            {
              strncpy((char *)chip_name, (char *)(chipinfo[i].chip_description), (CHIP_DESCRIPTOR_SIZE * 2));
              chip_name[CHIP_DESCRIPTOR_SIZE * 2] = '\0';
              printf("\tChip Information , ID: %d\t Name : %s\r\n", chipinfo[i].chip_global_id, chip_name);
            }
        }
    }
  free(chipinfo);
  return ret;
}


/****************************************************************************
 * Name: do_connect_help
 ****************************************************************************/
static int32_t do_connect_help(int32_t argc, char **argv)
{
  const struct fast_cmdmap_s *ptr;
  printf("Usage: fast connect <cmd> [arguments]\n");
  printf("Where <cmd> is one of:\n\n");
  for (ptr = (g_fast_connect + 1); ptr->cmd; ptr++)
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
 * Name: do_disconnect_help
 ****************************************************************************/
static int32_t do_disconnect_help(int32_t argc, char **argv)
{
  const struct fast_cmdmap_s *ptr;
  printf("Usage: fast disconnect <cmd> [arguments]\n");
  printf("Where <cmd> is one of:\n\n");
  for (ptr = (g_fast_disconnect + 1); ptr->cmd; ptr++)
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
 * Name: do_system_help
 ****************************************************************************/
static int32_t do_system_help(int32_t argc, char **argv)
{
  const struct fast_cmdmap_s *ptr;
  printf("Usage: fast system <cmd> [arguments]\n");
  printf("Where <cmd> is one of:\n\n");
  for (ptr = (g_fast_system + 1); ptr->cmd; ptr++)
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
 * Name: fast_connect
 ****************************************************************************/
int32_t fast_connect(int32_t argc, FAR char **argv)
{
  int32_t ret = -1;
  set_errno(0);
  dbg_tag();
  if (argc == 1)
    {
      do_connect_help(argc, argv);
      return -EINVAL;
    }
  ret = fast_parse(g_fast_connect, argc, argv);
  return ret;
}

/****************************************************************************
 * Name: fast_disconnect
 ****************************************************************************/
int32_t fast_disconnect(int32_t argc, FAR char **argv)
{
  int32_t ret = -1;
  if (argc == 1)
    {
      do_disconnect_help(argc, argv);
      return -EINVAL;
    }
  ret = fast_parse(g_fast_disconnect, argc, argv);
  return ret;
}

/****************************************************************************
 * Name: fast_system
 ****************************************************************************/
int32_t fast_system(int32_t argc, FAR char **argv)
{
  int32_t ret = -1;
  if (argc == 1)
    {
      do_system_help(argc, argv);
      return -EINVAL;
    }
  ret = fast_parse(g_fast_system, argc, argv);
  return ret;
}

/****************************************************************************
 * Name: fast_connect_peripheral
 ****************************************************************************/
#if defined(CONFIG_ZEUS2)
static int32_t do_connect_peripheral(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc != 3)
    {
      ret = do_connect_help(argc, argv);
      return -EINVAL;
    }
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  fast_ioctl_arg[1] = (uint32_t) strtol(argv[2], NULL, 10);
  ret = send_fast_ioctl(FAST_IOCTL_CONNECT_PERIPHERAL, (ulong_t)fast_ioctl_arg);
  return ret;
}
#endif

/****************************************************************************
 * Name: fast_connect_peripheral
 ****************************************************************************/
#if defined(CONFIG_ZEUS2)
static int32_t do_connect_chip_vrail(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc != 4)
    {
      ret = do_connect_help(argc, argv);
      return -EINVAL;
    }
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  fast_ioctl_arg[1] = (uint32_t) strtol(argv[2], NULL, 10);
  fast_ioctl_arg[2] = (uint32_t) strtol(argv[3], NULL, 10);
  ret = send_fast_ioctl(FAST_IOCTL_CONNECT_CHIP_VRAIL, (ulong_t)fast_ioctl_arg);
  return ret;
}
#endif

/****************************************************************************
 * Name: fast_disconnect_peripheral
 ****************************************************************************/
#if defined(CONFIG_ZEUS2)
static int32_t do_disconnect_peripheral(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc != 3)
    {
      ret = do_connect_help(argc, argv);
      return -EINVAL;
    }
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  fast_ioctl_arg[1] = (uint32_t) strtol(argv[2], NULL, 10);
  ret = send_fast_ioctl(FAST_IOCTL_DISCONNECT_PERIPHERAL, (ulong_t)fast_ioctl_arg);
  return ret;
}
#endif
