/****************************************************************************
 *   system/zglue_fast/fast_cfg_file.c
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

static int32_t do_cfg_file_help(int32_t argc, char **argv);
static int32_t do_cfg_file_erase(int32_t argc, char **argv);
static int32_t do_cfg_file_update(int32_t argc, char **argv);


static const struct fast_cmdmap_s g_fast_cfg_file[] =
{
  { "help",      do_cfg_file_help,        "",  NULL},
  { "erase",     do_cfg_file_erase,       "fast config erase",   NULL},
  { "update",    do_cfg_file_update,      "fast config udpate", "\n\t [FW config file]"},
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
 * Name: do_cfg_file_help
 ****************************************************************************/

static int32_t do_cfg_file_help(int32_t argc, char **argv)
{
  const struct fast_cmdmap_s *ptr;

  printf("Usage: fast power <cmd> [arguments]\n");
  printf("Where <cmd> is one of:\n\n");
  for (ptr = (g_fast_cfg_file + 1); ptr->cmd; ptr++)
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
 * Name: do_cfg_file_erase
 ****************************************************************************/
static int32_t do_cfg_file_erase(int32_t argc, char **argv)
{
  return send_fast_ioctl(FAST_IOCTL_CFG_FILE_ERASE, (ulong_t) fast_ioctl_arg);
}


/****************************************************************************
 * Name: do_cfg_file_update
 ****************************************************************************/
static int32_t do_cfg_file_update(int32_t argc, char **argv)
{
  int32_t ret = -1;
  char *fw_config_file = argv[1];
  fast_ioctl_arg[0] = (uint32_t)fw_config_file;
  ret = send_fast_ioctl(FAST_IOCTL_CFG_FILE_UPDATE, (ulong_t)fast_ioctl_arg);
  return ret;
}

/****************************************************************************
 * Name: fast_power
 ****************************************************************************/

int32_t fast_cfgfile_op(int32_t argc, FAR char **argv)
{
  int32_t ret = -1;
  if (argc == 1)
    {
      do_cfg_file_help(argc, argv);
      return -EINVAL;
    }
  ret = fast_parse(g_fast_cfg_file, argc, argv);
  return ret;
}
