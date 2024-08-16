/****************************************************************************
 *   system/zglue_fast/fast_fault.c
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

static int32_t do_fault_help(int32_t argc, char **argv);
static int32_t do_fault_get(int32_t argc, char **argv);
static int32_t do_fault_clear(int32_t argc, char **argv);
static int32_t do_fault_enable(int32_t argc, char **argv);


static const struct fast_cmdmap_s g_fast_fault[] =
{
  { "help",      do_fault_help,        "",  NULL},
  { "get",       do_fault_get,       "fast fault read",  NULL},
  { "clear",     do_fault_clear,     "fast fault clear", NULL},
  { "enable",    do_fault_enable,    "fast fault enable", "\n\t [int_mask<hex>]"},
  { NULL,   NULL,        NULL,             NULL },
};

extern uint32_t fast_ioctl_arg[FAST_IOCTL_ARG_COUNT];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: do_fault_help
 ****************************************************************************/
static int32_t do_fault_help(int32_t argc, char **argv)
{
  const struct fast_cmdmap_s *ptr;

  printf("Usage: fast fault <cmd> [arguments]\n");
  printf("Where <cmd> is one of:\n\n");
  for (ptr = (g_fast_fault + 1); ptr->cmd; ptr++)
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
 * Name: do_fault_get
 ****************************************************************************/
static int32_t do_fault_get(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc < 2)
    {
      do_fault_help(argc, argv);
      return -EINVAL;
    }
  ret = send_fast_ioctl(FAST_IOCTL_GET_FAULT_STATUS, (ulong_t) fast_ioctl_arg);
  printf("\tFAST fault status : %x\r\n", fast_ioctl_arg[0]);
  return ret;
}

/****************************************************************************
 * Name: do_fault_clear
 ****************************************************************************/
static int32_t do_fault_clear(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc < 2)
    {
      do_fault_help(argc, argv);
      return -EINVAL;
    }
  ret = send_fast_ioctl(FAST_IOCTL_CLEAR_FAULT_INTERRUPT, (ulong_t) fast_ioctl_arg);
  printf("\tFAST fault status : %x\r\n", fast_ioctl_arg[0]);
  return ret;
}

/****************************************************************************
 * Name: do_fault_enable
 ****************************************************************************/
static int32_t do_fault_enable(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc < 3)
    {
      do_fault_help(argc, argv);
      return -EINVAL;
    }
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 16);
  ret = send_fast_ioctl(FAST_IOCTL_ENABLE_FAULT_INTERRUPT, (ulong_t)fast_ioctl_arg);
  return ret;
}

/****************************************************************************
 * Name: fast_reg
 ****************************************************************************/
int32_t fast_fault(int32_t argc, FAR char **argv)
{
  int32_t ret = -1;
  if (argc == 1)
    {
      do_fault_help(argc, argv);
      return -EINVAL;
    }
  ret = fast_parse(g_fast_fault, argc, argv);
  return ret;
}
