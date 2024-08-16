/****************************************************************************
 *   include/system/fast_cmd.h
 *
 *   Copyright (C) 2007-2017 Gregory Nutt. All rights reserved.
 *   Author: Bill Rees <bill@zglue.com>
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



#if !defined(__SYSTEM_FAST_FAST_CMD_H)
#define __SYSTEM_FAST_FAST_CMD_H

/******************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>
#include <strings.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/zglue_fast/fast_api.h>
#include <nuttx/drivers/zglue_fast.h>




/******************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

#define POP_ARGV(count, char_starstar)  { \
        count--;  \
        char_starstar = (*char_starstar)++  \
}

#define ULONG_PTR(a) (unsigned long)((uintptr_t *)a)
#define NULL_CHKRTN(ret, errval, msg) {                 \
        if (ret == NULL)                                \
          {                                             \
                set_errno(errval);                      \
                return -1;                              \
          }                                             \
}

#define ERR_CHK(ret, errval, format, ...)  {            \
        if (ret < 0)                                    \
          {                                             \
                strerror_fast(format, ##__VA_ARGS__);   \
                set_errno(errval);                      \
          }                                             \
}

#define ERR_RETURN(ret, errval, format, ...) {          \
        if (ret < 0)                                    \
          {                                             \
                set_errno(errval);                      \
                return ret;                             \
          }                                             \
}

#define MAX_ARGUMENTS CONFIG_NSH_MAXARGUMENTS

/* Are we using the NuttX console for I/O?  Or some other character device? */

#ifdef CONFIG_FAST_TOOL_INDEV
#  define INFD(p)      ((p)->ss_infd)
#  define INSTREAM(p)  ((p)->ss_instream)
#else
#  define INFD(p)      0
#  define INSTREAM(p)  stdin
#endif

#ifdef CONFIG_FAST_TOOL_OUTDEV
#  define OUTFD(p)     ((p)->ss_outfd)
#  define OUTSTREAM(p) ((p)->ss_outstream)
#else
#  define OUTFD(p)     1
#  define OUTSTREAM(p) stdout
#endif

/* Output is via printf but can be changed using this macro */

#ifdef CONFIG_CPP_HAVE_VARARGS
# define fast_output(v, ...) printf(v, ##__VA_ARGS__)
#else
# define fast_output         printf
#endif

#define strerror_fast(format, b)     printf(format, b)

#define dbg_tag()
#if defined(CONFIG_DEBUG_FAST_CMDS)
#undef dbg_tag
#define dbg_tag()   _warn("== %s: argc %d,  argv[0] %s,  argv[1] %s\n", __func__, argc, argv[0], argv[1])
#endif

#define FAST_IOCTL_ARG_COUNT 10

/******************************************************************************
 * Private Data
 *****************************************************************************/

/******************************************************************************
 * Private Types
 *****************************************************************************/


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/******************************************************************************
 * Public Types
 *****************************************************************************/
typedef unsigned long ulong_t;

int32_t send_fast_ioctl(int32_t ioctl_cmd, ulong_t val);
typedef int32_t  (*fast_cmd_t)(int32_t argc, FAR char **argv);
typedef int32_t  (*fast_help_t)(int32_t argc, FAR char **argv);
struct fast_cmdmap_s
{
  FAR const char *cmd;        /* Name of the command */
  fast_cmd_t  handler;        /* Function that handles the command */
  FAR const char *desc;       /* Short description */
  FAR const char *usage;      /* Usage instructions for 'help' command */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t fast_cmd_printf(const char *fmt, ...);

int32_t fast_nsh_init(int32_t argc, FAR char **argv);
int32_t fast_reset(int32_t argc, FAR char **argv);
int32_t fast_nsh_close(int32_t argc, FAR char **argv);
int32_t fast_debug(int32_t argc, FAR char **argv);
int32_t fast_cmd_unrecognized(int32_t argc, char **argv);
int32_t fast_system(int32_t argc, FAR char **argv);
int32_t fast_connect(int32_t argc, FAR char **argv);
int32_t fast_disconnect(int32_t argc, FAR char **argv);
int32_t fast_gpio_exp(int32_t argc, FAR char **argv);
int32_t fast_jtag(int32_t argc, FAR char **argv);
int32_t fast_led(int32_t argc, FAR char **argv);
int32_t fast_id(int32_t argc, FAR char **argv);
int32_t fast_info(int32_t argc, FAR char **argv);
int32_t fast_cfgfile_op(int32_t argc, FAR char **argv);
int32_t fast_parse(const struct fast_cmdmap_s *maps, int32_t argc, char *argv[]);
#ifdef CONFIG_SYSTEM_FAST_DEBUG_API_NSH
int32_t fast_clear(int32_t argc, FAR char **argv);
int32_t fast_read(int32_t argc, FAR char **argv);
int32_t fast_write(int32_t argc, FAR char **argv);
int32_t fast_scan(int32_t argc, FAR char **argv);
#endif
int32_t fast_pmic(int32_t argc, FAR char **argv);
int32_t fast_powermgt(int32_t argc, FAR char **argv);
int32_t send_fast_ioctl(int32_t ioctl_cmd, ulong_t val);
int32_t fast_interface(int32_t argc, FAR char **argv);
int32_t fast_program_cmd(int32_t argc, FAR char **argv);
int32_t fast_realignment(int32_t argc, FAR char **argv);
int32_t fast_zcad_cmd(int32_t argc, FAR char **argv);
int32_t fast_control(int32_t argc, FAR char **argv);
int32_t do_control_bootcfg_byp(int32_t argc, FAR char **argv);
int32_t do_control_nRF_EN_SW(int32_t argc, FAR char **argv);


/****************************************************************************
 * Private Functions
 ****************************************************************************/


#endif /* __SYSTEM_FAST_FAST_CMD_H */
