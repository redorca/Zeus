/****************************************************************************
 * examples/BQ2512x/bq2512x_main.c
 *
 *   Copyright (C) 2018 zGlue Inc. All rights reserved.
 *   Author: Zhengwei Wang <zhengwei@zglue.com>
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

#include <nuttx/config.h>
#include <stdio.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include "system/readline.h"
#include <nuttx/power/battery_charger.h>
#include <nuttx/power/battery_ioctl.h>
#include <nuttx/power/bq2512x.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEV_BQ25120   "/dev/bq2512"
#define DEFAULT_LDO_OUTPUT 1800 /*default LDO output voltage, 1.8V*/
#define DEFAULT_CHARGE_CURRENT 10 /*default charge current, 10mA*/
#define DEFAULT_WAIT_PERIOD 6

typedef int (*bq2512x_func)(int filefd, char *pargs);


/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

struct mp_cmd_s
{
  const char      *cmd;       /* The command text */
  const char      *arghelp;   /* Text describing the args */
  bq2512x_func    pFunc;     /* Pointer to command handler */
  const char      *help;      /* The help text */
};

enum bq2512x_int_source
{
  BQ2512X_INT_PIN,
  BQ2512X_RESET_PIN,
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int bq2512x_cmd_enable_LDO(int filefd, char *parg);
static int bq2512x_cmd_disable_LDO(int filefd, char *parg);
static int bq2512x_cmd_charge(int filefd, char *parg);
static int bq2512x_cmd_bat_status(int filefd, char *parg);
static int bq2512x_cmd_bat_health(int filefd, char *parg);
static int bq2512x_cmd_bat_temp(int filefd, char *parg);
#ifdef CONFIG_BQ2512X_NPG_TO_GPIO
static int bq2512x_cmd_read_MR(int filefd, char *parg);
static int bq2512x_cmd_read_PG(int filefd, char *parg);
#endif
static int bq2512x_cmd_bat(int filefd, char *parg);
static int bq2512x_cmd_help(int filefd, char *parg);
static int bq2512x_cmd_quit(int filefd, char *parg);
static int bq2512x_cmd_wait_int(int filefd, char *parg);
static int bq2512x_cmd_wait_reset(int filefd, char *parg);

static int bq2512x_register_interrupt(int filefd, enum bq2512x_int_source src);
static int bq2512x_unregister_interrupt(int filefd, enum bq2512x_int_source src);
static int bq2512x_enable_interrupt(int filefd, enum bq2512x_int_source src);
static int bq2512x_disable_interrupt(int filefd, enum bq2512x_int_source src);
static int bq2512x_fetch_int_source(int filefd);


/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mp_cmd_s g_bq2512x_cmds[] =
{
  { "bat",      "",       bq2512x_cmd_bat,          "display battery charge status and healthy" },
  { "ldo_on",   "%d",     bq2512x_cmd_enable_LDO,   "config LDO output voltage(in unit of mV) and enable it" },
  { "ldo_off",  "",       bq2512x_cmd_disable_LDO,  "turn off LDO" },
  { "charge",   "%d",     bq2512x_cmd_charge,       "config charge current, in unit of mA"},
#ifdef CONFIG_BQ2512X_NPG_TO_GPIO
  { "pg",       "",       bq2512x_cmd_read_PG,      "read PG pin"},
  { "mr",       "",       bq2512x_cmd_read_MR,      "config PG pin as MR shift output and read it"},
  { "reset",    "",       bq2512x_cmd_wait_reset,      "wait for reset signal from NRESET pin"},
#endif
  { "int",      "",       bq2512x_cmd_wait_int,     "wait for interrupt triggerd by INT pin"},
  { "temp",     "",      bq2512x_cmd_bat_temp,      "read battery temperature"},
  { "h",        "",      bq2512x_cmd_help,      "Display help for commands" },
  { "help",     "",      bq2512x_cmd_help,      "Display help for commands" },
  { "q",        "",      bq2512x_cmd_quit,          "Exit" },
  { "quit",     "",      bq2512x_cmd_quit,          "Exit" },
};
static const int g_bq2512x_cmd_count = sizeof(g_bq2512x_cmds) / sizeof(struct mp_cmd_s);


/****************************************************************************
 * Name: bq2512x_cmd_enable_LDO
 *
 *   bq2512x_cmd_enable_LDO() enable LDO voltage output.
 *
 ****************************************************************************/

static int bq2512x_cmd_enable_LDO(int filefd, char *parg)
{
  int ret;
  int volt;
  struct batio_operate_msg_s op_msg;


  /* If no arg given, use the default value */

  if (parg == NULL || *parg == '\0')
    {
      printf("No voltage value followed, use default %dmV\n", DEFAULT_LDO_OUTPUT);
      volt = DEFAULT_LDO_OUTPUT;
    }
  else
    {
      /* Get the voltage value from the argument */

      volt = atoi(parg);
    }

  /*set output voltage*/
  op_msg.operate_type = BATIO_OPRTN_LDO_VOUT_SET;
  op_msg.u32 = volt;
  ret = ioctl(filefd, BATIOC_OPERATE, (unsigned long)&op_msg);
  if (OK != ret)
    {
      printf("LDO output voltage set failed:%d\n", ret);
      return ret;
    }

  /*enable LDO*/
  op_msg.operate_type = BATIO_OPRTN_LDO_ENABLE;
  ret = ioctl(filefd, BATIOC_OPERATE, (unsigned long)&op_msg);
  if (OK != ret)
    {
      printf("LDO enable failed:%d\n", ret);
      return ret;
    }

  printf("set LDO output voltage to %dmV. \n", volt);

  return OK;
}

/****************************************************************************
 * Name: bq2512x_cmd_disable_LDO
 *
 *   bq2512x_cmd_disable_LDO() enable LDO voltage output.
 *
 ****************************************************************************/

static int bq2512x_cmd_disable_LDO(int filefd, char *parg)
{
  int ret;
  struct batio_operate_msg_s op_msg;

  /*enable LDO*/
  op_msg.operate_type = BATIO_OPRTN_LDO_DISABLE;
  ret = ioctl(filefd, BATIOC_OPERATE, (unsigned long)&op_msg);
  if (OK != ret)
    {
      printf("LDO disable failed:%d\n", ret);
      return ret;
    }

  printf("turn off LDO output\n");

  return OK;
}

/****************************************************************************
 * Name: bq2512x_cmd_charge
 *
 *   bq2512x_cmd_charge() set charge current.
 *
 ****************************************************************************/

static int bq2512x_cmd_charge(int filefd, char *parg)
{
  int ret;
  int current;

  /* If no arg given, use the default value */

  if (parg == NULL || *parg == '\0')
    {
      printf("No current value followed, use default %dmA\n", DEFAULT_CHARGE_CURRENT);
      current = DEFAULT_CHARGE_CURRENT;
    }
  else
    {
      /* Get the voltage value from the argument */

      current = atoi(parg);
    }

  /*set charge current*/
  ret = ioctl(filefd, BATIOC_CURRENT, (unsigned long)&current);
  if (OK != ret)
    {
      printf("charge current set failed:%d\n", ret);
      return ret;
    }

  printf("set charge current to %dmA. \n", current);

  return OK;
}



/****************************************************************************
 * Name: bq2512x_cmd_bat_status
 *
 *   bq2512x_cmd_bat_status() get the battery charge status.
 *
 ****************************************************************************/

static int bq2512x_cmd_bat_status(int filefd, char *parg)
{
  int ret;
  enum battery_charger_status_e bat_status;

  ret = ioctl(filefd, BATIOC_STATE, (unsigned long)&bat_status);
  if (OK != ret)
    {
      printf("Battery state read error:%d\n", ret);
      return ret;
    }

  printf("Battery status : ");

  switch (bat_status)
    {
      case BATTERY_FAULT:
        printf("FAULT \n");
        break;
      case BATTERY_FULL:
        printf("FULL \n");
        break;
      case BATTERY_CHARGING:
        printf("CHARGING \n");
        break;
      case BATTERY_DISCHARGING:
        printf("DISCHARGING \n");
        break;
      default:
        printf("%d\n", bat_status);
        break;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2512x_cmd_bat_health
 *
 *   bq2512x_cmd_bat_health() get the battery health status.
 *
 ****************************************************************************/

static int bq2512x_cmd_bat_health(int filefd, char *parg)
{
  int ret;
  enum battery_charger_health_e bat_health;

  ret = ioctl(filefd, BATIOC_HEALTH, (unsigned long)&bat_health);
  if (OK != ret)
    {
      printf("Battery health read error:%d\n", ret);
      return ret;
    }

  printf("Battery health : ");

  switch (bat_health)
    {
      case BATTERY_HEALTH_GOOD:
        printf("GOOD \n");
        break;
      case BATTERY_HEALTH_DEAD:
        printf("DEAD \n");
        break;
      case BATTERY_HEALTH_UNDERVOLTAGE:
        printf("UNDERVOLTAGE \n");
        break;
      case BATTERY_HEALTH_SAFE_TMR_EXP:
        printf("SAFETY TIMER EXPIRED \n");
        break;
      default:
        printf("%d\n", bat_health);
        break;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2512x_cmd_bat_temp
 *
 *   bq2512x_cmd_bat_temp() get the battery temprature.
 *
 ****************************************************************************/

static int bq2512x_cmd_bat_temp(int filefd, char *parg)
{
  int ret;
  /* Read the current vout setting of the PMIC */
  struct batio_operate_msg_s bat_msg;

  bat_msg.operate_type = BATIO_OPRTN_GET_BAT_TEMP;
  bat_msg.u32 = 0;
  ret = ioctl(filefd, BATIOC_OPERATE, (unsigned long)&bat_msg);
  if (OK != ret)
    {
      printf("Battery temprature read error:%d\n", ret);
      return ret;
    }

  printf("Battery temprature : ");

  switch (bat_msg.u32)
    {
      case BQ2512X_BAT_NORMAL:
        printf("Normal\n");
        break;
      case BQ2512X_BAT_COLD_HOT:
        printf("Cold or hot\n");
        break;
      case BQ2512X_BAT_COOL:
        printf("Cool\n");
        break;
      case BQ2512X_BAT_WARM:
        printf("Warm\n");
        break;
      default:
        printf("Unknow %d\n", bat_msg.u32);
        break;
    }

  return OK;
}


#ifdef CONFIG_BQ2512X_NPG_TO_GPIO

/****************************************************************************
 * Name: bq2512x_cmd_read_PG
 *
 *   bq2512x_cmd_read_PG() read PG pin status.
 *
 ****************************************************************************/
static int bq2512x_cmd_read_PG(int filefd, char *parg)
{
  int ret;
  struct batio_operate_msg_s op_msg;

  op_msg.operate_type = BATIO_OPRTN_READ_PG;
  op_msg.u32 = 0;
  ret = ioctl(filefd, BATIOC_OPERATE, (unsigned long)&op_msg);
  if (OK != ret)
    {
      printf("read PG pin error:%d\n", ret);
      return ret;
    }

  printf("PG pin level : ");

  if (op_msg.u32)
    {
      printf("High\n");
    }
  else
    {
      printf("low\n");
    }

  return OK;
}

/****************************************************************************
 * Name: bq2512x_cmd_read_MR
 *
 *   bq2512x_cmd_read_MR() read MR pin status.
 *
 ****************************************************************************/
static int bq2512x_cmd_read_MR(int filefd, char *parg)
{
  int ret;
  struct batio_operate_msg_s op_msg;

  op_msg.operate_type = BATIO_OPRTN_READ_MR;
  op_msg.u32 = 0;
  ret = ioctl(filefd, BATIOC_OPERATE, (unsigned long)&op_msg);
  if (OK != ret)
    {
      printf("read PG pin error:%d\n", ret);
      return ret;
    }

  printf("MR pin level : ");

  if (op_msg.u32)
    {
      printf("High\n");
    }
  else
    {
      printf("low\n");
    }

  return OK;
}

#endif

/****************************************************************************
 * Name: bq2512x_cmd_bat
 *
 *   bq2512x_cmd_bat() display battery related status.
 *
 ****************************************************************************/
static int bq2512x_cmd_bat(int filefd, char *parg)
{

  bq2512x_cmd_bat_status(filefd, parg);
  bq2512x_cmd_bat_health(filefd, parg);
  bq2512x_cmd_bat_temp(filefd, parg);

  return OK;
}

/****************************************************************************
 * Name: bq2512x_cmd_help
 *
 *   bq2512x_cmd_help() displays the application's help information on
 *   supported commands and command syntax.
 ****************************************************************************/


static int bq2512x_cmd_help(int filefd, char *parg)
{
  int   x, len, maxlen = 0;
  int   c;

  /* Calculate length of longest cmd + arghelp */

  for (x = 0; x < g_bq2512x_cmd_count; x++)
    {
      len = strlen(g_bq2512x_cmds[x].cmd) + strlen(g_bq2512x_cmds[x].arghelp);
      if (len > maxlen)
        {
          maxlen = len;
        }
    }

  printf("bq2512x commands\n================\n");
  for (x = 0; x < g_bq2512x_cmd_count; x++)
    {
      /* Print the command and it's arguments */

      printf("  %s %s", g_bq2512x_cmds[x].cmd, g_bq2512x_cmds[x].arghelp);

      /* Calculate number of spaces to print before the help text */

      len = maxlen - (strlen(g_bq2512x_cmds[x].cmd) + strlen(g_bq2512x_cmds[x].arghelp));
      for (c = 0; c < len; c++)
        {
          printf(" ");
        }

      printf("  : %s\n", g_bq2512x_cmds[x].help);
    }

  return OK;
}



/****************************************************************************
 * Name: bq2512x_cmd_quit
 *
 *   bq2512x_cmd_quit().
 *
 ****************************************************************************/
static int bq2512x_cmd_quit(int filefd, char *parg)
{
  /*according to the datasheet, we may need to stop the watchdog timer*/
  return OK;
}

/****************************************************************************
 * Name: bq2512x_register_interrupt
 *
 *   bq2512x_register_interrupt() register signal, a signal will be send from
 *   the driver layer when an interrupt is captured on the INT or RESET pin.
 *
 ****************************************************************************/
static int bq2512x_register_interrupt(int filefd, enum bq2512x_int_source src)
{
  int ret;
  struct batio_operate_msg_s op_msg;

  if (src == BQ2512X_INT_PIN)
    {
      op_msg.operate_type = BATIO_OPRTN_REGISTER_INT;
      op_msg.u32 = SIGUSR1;
    }
#ifdef CONFIG_BQ2512X_NRESET_TO_GPIO
  else if (src == BQ2512X_RESET_PIN)
    {
      op_msg.operate_type = BATIO_OPRTN_REGISTER_RESET;
      op_msg.u32 = SIGUSR2;
    }
#endif
  else
    {
      return -EINVAL;
    }

  ret = ioctl(filefd, BATIOC_OPERATE, (unsigned long)&op_msg);
  if (OK != ret)
    {
      printf("register signal failed:%d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: bq2512x_unregister_interrupt
 *
 *   bq2512x_register_interrupt() unregister signal, stop receiving signals.
 *
 ****************************************************************************/
static int bq2512x_unregister_interrupt(int filefd, enum bq2512x_int_source src)
{
  int ret;
  struct batio_operate_msg_s op_msg;

  if (src == BQ2512X_INT_PIN)
    {
      op_msg.operate_type = BATIO_OPRTN_UNREGISTER_INT;
      op_msg.u32 = 0;
    }
#ifdef CONFIG_BQ2512X_NRESET_TO_GPIO
  else if (src == BQ2512X_RESET_PIN)
    {
      op_msg.operate_type = BATIO_OPRTN_UNREGISTER_RESET;
      op_msg.u32 = 0;
    }
#endif
  else
    {
      return -EINVAL;
    }

  ret = ioctl(filefd, BATIOC_OPERATE, (unsigned long)&op_msg);
  if (OK != ret)
    {
      printf("unregister signal failed:%d\n", ret);
      return ret;
    }

  return ret;
}


/****************************************************************************
 * Name: bq2512x_enable_interrupt
 *
 *   bq2512x_enable_interrupt() start monitoring interrupts on INT or RESET pin.
 *
 ****************************************************************************/

static int bq2512x_enable_interrupt(int filefd, enum bq2512x_int_source src)
{
  int ret;
  struct batio_operate_msg_s op_msg;

  if (src == BQ2512X_INT_PIN)
    {
      op_msg.operate_type = BATIO_OPRTN_ENABLE_INT;
      op_msg.u32 = 0;
    }
#ifdef CONFIG_BQ2512X_NRESET_TO_GPIO
  else if (src == BQ2512X_RESET_PIN)
    {
      op_msg.operate_type = BATIO_OPRTN_ENABLE_RESET;
      op_msg.u32 = 0;
    }
#endif
  else
    {
      return -EINVAL;
    }

  ret = ioctl(filefd, BATIOC_OPERATE, (unsigned long)&op_msg);
  if (OK != ret)
    {
      printf("enable interrupt failed:%d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: bq2512x_disable_interrupt
 *
 *   bq2512x_disable_interrupt() stop monitoring interrupts on INT or RESET pin.
 *
 ****************************************************************************/

static int bq2512x_disable_interrupt(int filefd, enum bq2512x_int_source src)
{
  int ret;
  struct batio_operate_msg_s op_msg;

  if (src == BQ2512X_INT_PIN)
    {
      op_msg.operate_type = BATIO_OPRTN_DISABLE_INT;
      op_msg.u32 = 0;
    }
#ifdef CONFIG_BQ2512X_NRESET_TO_GPIO
  else if (src == BQ2512X_RESET_PIN)
    {
      op_msg.operate_type = BATIO_OPRTN_DISABLE_RESET;
      op_msg.u32 = 0;
    }
#endif
  else
    {
      return -EINVAL;
    }

  ret = ioctl(filefd, BATIOC_OPERATE, (unsigned long)&op_msg);
  if (OK != ret)
    {
      printf("disable interrupt failed:%d\n", ret);
      return ret;
    }

  return ret;
}

static int bq2512x_cmd_wait_interrupt(int filefd, enum bq2512x_int_source src)
{
  int ret;
  struct timespec ts;
  sigset_t set;
  struct siginfo info;


  ret = bq2512x_register_interrupt(filefd, src);
  if (ret != OK)
    {
      return ret;
    }

  ret = bq2512x_enable_interrupt(filefd, src);
  if (ret != OK)
    {
      return ret;
    }

  /* prepare the signal set */
  (void)sigemptyset(&set);

  if (src == BQ2512X_INT_PIN)
    {
      (void)sigaddset(&set, SIGUSR1);
    }
#ifdef CONFIG_BQ2512X_NRESET_TO_GPIO
  else if (src == BQ2512X_RESET_PIN)
    {
      (void)sigaddset(&set, SIGUSR2);
    }
#endif
  else
    {
      return -EINVAL;
    }

  /* wait for 6 seconds to receive signo */

  ts.tv_sec  = DEFAULT_WAIT_PERIOD;
  ts.tv_nsec = 0;

  ret = sigtimedwait(&set, &info, &ts);

  if (ret < 0)
    {
      int errcode = errno;
      if (errcode == EAGAIN)
        {
          /* 5 seconds passed and we didn't receive the signo, and we just continue*/
          printf("%d seconds timeout with no signal\n", DEFAULT_WAIT_PERIOD);
        }
      else
        {
          printf("signal wait failed: %d\n", errcode);
          return -errcode;
        }
    }
  else
    {
      if (src == BQ2512X_INT_PIN)
        {
          printf("interrupt receiverd from the INT pin, interrupt source: \n");
          bq2512x_fetch_int_source(filefd);
        }
      else
        {
          printf("interrupt receiverd from the RESET pin\n");
        }
    }

  ret = bq2512x_unregister_interrupt(filefd, src);
  if (ret != OK)
    {
      return ret;
    }

  ret = bq2512x_disable_interrupt(filefd, src);
  if (ret != OK)
    {
      return ret;
    }

  return OK;
}


static int bq2512x_cmd_wait_int(int filefd, char *parg)
{
  int ret;

  ret = bq2512x_cmd_wait_interrupt(filefd, BQ2512X_INT_PIN);

  return ret;
}

#ifdef CONFIG_BQ2512X_NRESET_TO_GPIO
static int bq2512x_cmd_wait_reset(int filefd, char *parg)
{
  int ret;

  ret = bq2512x_cmd_wait_interrupt(filefd, BQ2512X_RESET_PIN);

  return ret;
}
#endif


static int bq2512x_fetch_int_source(int filefd)
{
  int ret;
  struct batio_operate_msg_s op_msg;

  op_msg.operate_type = BATIO_OPRTN_GET_INT_STA;
  op_msg.u32 = 0;
  ret = ioctl(filefd, BATIOC_OPERATE, (unsigned long)&op_msg);
  if (OK != ret)
    {
      printf("read interrupt source error:%d\n", ret);
      return ret;
    }

  printf("VIN_OV: %d\n", (op_msg.u32 & BQ2512X_INT_VIN_OV) ? true : false);
  printf("VIN_UV: %d\n", (op_msg.u32 & BQ2512X_INT_VIN_UV) ? true : false);
  printf("BAT_UVLO: %d\n", (op_msg.u32 & BQ2512X_INT_BAT_UVLO) ? true : false);
  printf("BAT_OCP: %d\n", (op_msg.u32 & BQ2512X_INT_BAT_OCP) ? true : false);
  printf("TS_COLD_HOT: %d\n", (op_msg.u32 & BQ2512X_INT_TS_COLD_HOT) ? true : false);
  printf("TS_COOL: %d\n", (op_msg.u32 & BQ2512X_INT_TS_COOL) ? true : false);
  printf("TS_WARM: %d\n", (op_msg.u32 & BQ2512X_INT_TS_WARM) ? true : false);
  printf("TS_OFF: %d\n", (op_msg.u32 & BQ2512X_INT_TS_OFF) ? true : false);

  return OK;

}



/****************************************************************************
 * hello_main
 ****************************************************************************/

#if defined(BUILD_MODULE)
int main(int argc, FAR char *argv[])
#else
int bq25120_main(int argc, char *argv[])
#endif
{
  int filefd;
  bool running;
  char buffer[64];
  int len;
  char *cmd;
  char *arg;
  int cnt;

  /*open the file*/

  filefd = open(DEV_BQ25120, O_RDONLY);
  if (filefd < 0)
    {
      printf("Can't Open Device %s for Read. Reason:%d\n", DEV_BQ25120, filefd);
      return 0;
    }

  /* Loop until the user exits */

  running = TRUE;
  while (running)
    {
      /* Print a prompt */

      printf("bq2512x> ");
      fflush(stdout);

      /* Read a line from the terminal */

      len = readline(buffer, sizeof(buffer), stdin, stdout);
      buffer[len] = '\0';
      if (len > 0)
        {
          if (buffer[len - 1] == '\n')
            {
              buffer[len - 1] = '\0';
            }

          /* Parse the command from the argument */

          cmd = strtok_r(buffer, " \n", &arg);
          if (cmd == NULL)
            {
              continue;
            }

          /* Remove leading spaces from arg */

          while (*arg == ' ')
            {
              arg++;
            }

          /* Find the command in our cmd array */

          for (cnt = 0; cnt < g_bq2512x_cmd_count; cnt++)
            {
              if (strcmp(cmd, g_bq2512x_cmds[cnt].cmd) == 0)
                {
                  /* Command found.  Call it's handler if not NULL */

                  if (g_bq2512x_cmds[cnt].pFunc != NULL)
                    {
                      g_bq2512x_cmds[cnt].pFunc(filefd, arg);
                    }

                  /* Test if it is a quit command */

                  if (g_bq2512x_cmds[cnt].pFunc == bq2512x_cmd_quit)
                    {
                      running = FALSE;
                    }
                  break;
                }
            }

          /* Test for Unknown command */

          if (cnt == g_bq2512x_cmd_count)
            {
              printf("%s:  unknown bq2512x command\n", buffer);
              bq2512x_cmd_help(filefd, NULL);
            }
        }
    }

  close(filefd);
  return 0;
}
