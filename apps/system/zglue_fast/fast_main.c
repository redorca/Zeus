/****************************************************************************
 *   system/zglue_fast/fast_main.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
#include "dev-stat.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Common, message formats */

extern const char g_fast_argrequired[];
extern const char g_fast_arginvalid[];
extern const char g_fast_argrange[];
extern const char g_fast_cmdnotfound[];
extern const char g_fast_toomanyargs[];
extern const char g_fast_cmdfailed[];
extern const char g_fast_xfrerror[];

#if defined(USE_LOCAL_DECLARATION)
struct fast_cmdmap_s
{
  FAR const char *cmd;        /* Name of the command */
  fast_cmd_t           handler;    /* Function that handles the command */
  FAR const char *desc;       /* Short description */
  FAR const char *usage;      /* Usage instructions for 'help' command */
};
#endif

struct fast_tools_s
{
  int32_t tmp;
};

static struct fast_tools_s ftools;
static int32_t fast_cmd_help(int32_t argc, char **argv);
extern uint8_t fast_print_status;

static const struct fast_cmdmap_s g_fast_cmds[] =
{
  { "help",     fast_cmd_help,  "Help for nsh fast commands", NULL},
  { "debug",    fast_debug,     "Enable/Disable debug features.",  NULL},
  { "init",     fast_nsh_init,  "Initialize the fast api",  NULL},
  { "config",   fast_cfgfile_op,  "Erase/Update the zglue FW config file",       NULL },
#if defined(CONFIG_ARCH_BOARD_ZGLUE_ZEUS1_REMORA_BOARD) && defined(CONFIG_ZEUS1)
  { "reset",     fast_reset,  "Reset the zeus1 chip with RESETB and RESETIOB pins (Works only on remora board)",  NULL},
#endif
  { "system",     fast_system,      "Return system/chiplets information",    NULL },
  { "connect",  fast_connect,   "Connect chips/peripherals to FAST",   NULL },
  { "disconnect",  fast_disconnect,   "Disconnect chips/peripherals from FAST",   NULL },
#if defined(CONFIG_ZEUS2_DEVKIT_FEATURES)
  { "power",    fast_powermgt,     "Manage power options on devkit including power config,states and status",          NULL },
  { "control",    fast_control,     "Control smart fabric on devkit including bootcfg_byp",          NULL },
#else
  { "power",    fast_powermgt,     "Set/get power state",          NULL },
#endif
  { "id",       fast_id,        "Return the FAST chip id",       NULL },
#if defined(CONFIG_ZEUS1)
#ifdef CONFIG_SYSTEM_FAST_DEBUG_API_NSH
  { "scan",      fast_scan,       "FAST scan tile",       NULL },
  { "read",      fast_read,       "FAST read tile",       NULL },
  { "write",      fast_write,     "FAST write tile",       NULL },
#endif
#elif(CONFIG_ZEUS2)
  { "program", fast_program_cmd,  "Program registers/peripherals",       NULL },
  { "zcad",    fast_zcad_cmd,     "Execute zcad command.",  NULL},
  { "realign", fast_realignment,  "Run realignment",       NULL },
#ifdef CONFIG_SYSTEM_FAST_DEBUG_API_NSH
  { "clear",     fast_clear,      "FAST clears tiles/peripherals",       NULL },
  { "read",      fast_read,       "FAST read single reg/tile/peripheral",       NULL },
  { "write",     fast_write,      "FAST write single reg/tile/peripheral",       NULL },
  { "scan",      fast_scan,       "FAST scan regs/tiles/peripherals",       NULL },
#endif
  { "pmic",     fast_pmic,      "Control PMIC",         NULL },
  { "gpio",     fast_gpio_exp,  "Control GPIO expander",        NULL },
  { "led",      fast_led,       "Control LED sinks",            NULL },
  { "interface",    fast_interface,     "Control the FAST interface settings for SPI/I2C/JTAG",  NULL},
#endif
  { "close",    fast_nsh_close,     "Close the fast api",  NULL},
  { NULL,   NULL,        NULL,             NULL }
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

uint32_t fast_ioctl_arg[FAST_IOCTL_ARG_COUNT];

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static inline int32_t fast_setup(struct fast_tools_s *maps);
static void fast_teardown(struct fast_tools_s *maps);

/****************************************************************************
 * Public Functions
 ****************************************************************************/
void fast_cmd_flush(void);

/****************************************************************************
 * Name: fast_cmd_help
 ****************************************************************************/

static int32_t fast_cmd_help(int32_t argc, char **argv)
{
  const struct fast_cmdmap_s *ptr;
  printf("Usage: fast <cmd> [arguments]\n");
  printf("Where <cmd> is one of:\n\n");
  for (ptr = (g_fast_cmds); ptr->cmd; ptr++)
    {
      if (ptr->usage)
        {
          printf("  %s: %s %s\n", ptr->cmd, ptr->desc,  ptr->usage);
        }
      else
        {
          printf("  %s: %s\n", ptr->cmd, ptr->desc);
        }
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: fast_setup
 ****************************************************************************/

static inline int32_t fast_setup(struct fast_tools_s *maps)
{
  /* Initialize the output stream */

#ifdef CONFIG_FAST_CMDS_OUTDEV
#error Should not be here.
  fast_cmd->ss_outfd = open(CONFIG_FAST_CMDS_OUTDEV, O_WRONLY);
  if (fast_cmd->ss_outfd < 0)
    {
      fprintf(stderr, g_fast_cmdfailed, "open", errno);
      return ERROR;
    }

  /* Create a standard C stream on the console device */

  fast_cmd->ss_outstream = fdopen(fast_tools->ss_outfd, "w");
  if (!fast_cmd->ss_outstream)
    {
      fprintf(stderr, g_fast_cmdfailed, "fdopen", errno);
      return ERROR;
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: fast_teardown
 *
 * Description:
 *   Close the output stream if it is not the standard output stream.
 *
 ****************************************************************************/

static void fast_teardown(struct fast_tools_s *maps)
{
  fflush(OUTSTREAM(maps));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fast_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int32_t main(int32_t argc, FAR char *argv[])
#else
int32_t fast_main(int32_t argc, char *argv[])
#endif
{
  int32_t ret;
  int i = 0, index = 0;
  char *fast_cmd = "fast ";
  char nsh_cmd[CONFIG_NSH_LINELEN + 1];
  /* Verify settings */

  /* Parse and process the command line */

  /* fast_setup(&ftools); */
  if (! dev_exists(FAST_DEV_PATH))
    {
      printf("Device does not exist!  Could not stat FAST device.\n");
      return !OK;
    }
  if (argc == 1)
    {
      fast_cmd_help(argc, argv);
      return OK;
    }
  ret = fast_parse(g_fast_cmds, argc, argv);
  if (fast_print_status == 1)
    {
      /* Initialize the string with spaces */
      memset(nsh_cmd, 0x20, (CONFIG_NSH_LINELEN + 1));
      strncpy(&(nsh_cmd[index]), fast_cmd, strlen(fast_cmd));
      index += strlen(fast_cmd);
      for (i = 1 ; i < argc; i++)
        {
          strncpy(&(nsh_cmd[index]), argv[i], strlen(argv[i]));
          index += (strlen(argv[i]) + 1);
        }
      nsh_cmd[index] = '\0';
      printf("Status of command ' %s ' :  %d(%s)\r\n", nsh_cmd, ret, ret == 0 ? "OK" : strerror(ret * -1));
    }
  fast_cmd_flush();
  fast_teardown(&ftools);
  return OK;
}

#if defined(FAST_ENABLE_OUTPUT_DEV)
/****************************************************************************
 * Name: fast_cmd_printf
 *
 * Description:
 *   Print32_t a string to the currently selected stream.
 *
 ****************************************************************************/

int32_t fast_cmd_printf(const char *fmt, ...)
{
  va_list ap;
  UNUSED(ap);

  int32_t     ret = OK;

#if defined(FAST_CMD_STRUCT_DEFINED)
  va_start(ap, fmt);
  ret = vfprintf(OUTSTREAM(fast_cmd), fmt, ap);
  va_end(ap);
#endif

  return ret;
}
#endif /* FAST_ENABLE_OUTPUT_DEV  */

/****************************************************************************
 * Name: fast_cmd_write
 *
 * Description:
 *   write a buffer to the currently selected stream.
 *
 ****************************************************************************/

ssize_t fast_cmd_write(FAR const void *buffer, size_t nbytes)
{
  ssize_t ret = OK;

  /* Write the data to the output stream */

#if defined(FAST_CMD_STRUCT_DEFINED)
  ret = fwrite(buffer, 1, nbytes, OUTSTREAM(fast_cmd));
  if (ret < 0)
    {
      _err("ERROR: [%d] Failed to send buffer: %d\n", OUTFD(fast_cmd), errno);
    }
#endif
  return ret;
}

/****************************************************************************
 * Name: fast_cmd_flush
 *
 * Description:
 *   Flush buffered I/O to the currently selected stream.
 *
 ****************************************************************************/

void fast_cmd_flush(void)
{
#if defined(FAST_CMD_STRUCT_DEFINED)
  fflush(OUTSTREAM(fast_cmd));
#endif
}


/****************************************************************************
 * Name: fast_execute
 ****************************************************************************/

static int32_t fast_execute(const struct fast_cmdmap_s *maps, int32_t argc, char *argv[])
{
  const struct fast_cmdmap_s *cmdmap;
  const char            *cmd;
  fast_cmd_t             handler;
  int32_t                ret;

  /* The form of argv is:
   *
   * argv[0]:      The command name.  This is argv[0] when the arguments
   *               are, finally, received by the command vtblr
   * argv[1]:      The beginning of argument (up to MAX_ARGUMENTS)
   * argv[argc]:   NULL terminating pointer
   */

  dbg_tag();
  /* See if the command is one that we understand */
  cmd     = argv[0];

  handler = maps[0].handler;

  /* Start looking for a match at entry 1 not 0.  0 is a default handler. */
  for (cmdmap = &maps[1]; cmdmap->cmd; cmdmap++)
    {
      if (strcmp(cmdmap->cmd, cmd) == 0)
        {
          handler = cmdmap->handler;
          break;
        }
    }

  ret = handler(argc, argv);
  return ret;
}

/****************************************************************************
 * Name: fast_argument
 ****************************************************************************/

static FAR char *fast_argument(const struct fast_cmdmap_s *maps, int32_t argc, char *argv[], int32_t *pindex)
{
  FAR char *arg;
  int32_t  index = *pindex;

  /* If we are at the end of the arguments with nothing, then return NULL */
  dbg_tag();
  if (index >= argc)
    {
      return NULL;
    }

  /* Get the return parameter */

  arg     = argv[index];
  *pindex = index + 1;

  /* Return the next argument. */
  return arg;
}

/****************************************************************************
 * Name: fast_parse
 ****************************************************************************/

int32_t fast_parse(const struct fast_cmdmap_s *cmd_maps, int32_t argc, char *argv[])
{
  FAR char *newargs[MAX_ARGUMENTS + 2];
  FAR char *cmd;
  int32_t       nargs;
  int32_t       index = 1;

  dbg_tag();
  cmd = fast_argument(cmd_maps, argc, argv, &index);

  /* Check if any command was provided
   * An empty line is not an error and an unprocessed command cannot
   * generate an error, but neither should they change the last
   * command status.
   */
  NULL_CHKRTN(cmd, EIO, g_);

  /* Parse all of the arguments following the command name. */
  newargs[0] = cmd;
  for (nargs = 1; nargs <= MAX_ARGUMENTS; nargs++)
    {
      newargs[nargs] = fast_argument(cmd_maps, argc, argv, &index);
      if (!newargs[nargs])
        {
          break;
        }
    }

  newargs[nargs] = NULL;

  /* Then execute the command */

  return fast_execute(cmd_maps, nargs, newargs);
}

/****************************************************************************
 * Name: send_fast_ioctl()
 *
 *
 ****************************************************************************/

int32_t send_fast_ioctl(int32_t ioctl_cmd, ulong_t val)
{
  int32_t fd;
  int32_t ret = 0xFFFFFFFF;

  fd = open(FAST_DEV_PATH, O_RDONLY);
  if (fd > 0)
    {
      ret = ioctl(fd, ioctl_cmd, val);
      close(fd);
    }
  else
    {
      errno = -EIO;
      ret = -1;
    }

  return ret;
}

