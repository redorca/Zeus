/****************************************************************************
 * examples/stepcounting/stepCNT_main.c
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

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include "system/readline.h"
#include "utils/step_counting.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define STEPCNT_CMD_LINE_BUF_LEN 20

/****************************************************************************
 * Private Data
 ****************************************************************************/
static stepCNT_t handle = NULL;

static void stepCNT_help(void)
{
  printf("Usage:\n");
  printf("[total/t], fetch total steps and motion pattern.\n");
  printf("[reset/r], reset the step counter and clear total step value.\n");
  printf("[quit/q], quit from this app.\n");
  printf("[help/h], show this help message.\n");
  printf("\n");
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stepCNT_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int stepCNT_main(int argc, char *argv[])
#endif
{
  int ret;
  uint32_t total;
  step_patt patt;
  char *patt_string;
  bool running;
  char buffer[STEPCNT_CMD_LINE_BUF_LEN];
  int len;
  char *cmd, *arg;




  ret = stepCNT_start(&handle, CONFIG_EXAMPLES_ACCR_DEVPATH);

  if (ret != OK)
    {
      printf("step counting thread start failed:%d\n", ret);
      return 0;
    }

  printf("Step counting thread start\n");

  running = TRUE;
  while (running)
    {
      /* Print a prompt */

      printf("STEP> ");
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


          if ((strcmp(cmd, "reset") == 0) || (strcmp(cmd, "r") == 0))
            {
              ret = stepCNT_reset(handle);
              if (ret != OK)
                {
                  printf("stepCNT:an error happend in reset process:%d\n", ret);
                  break;
                }
            }
          else if ((strcmp(cmd, "q") == 0) || (strcmp(cmd, "quit") == 0))
            {
              running = false;
            }
          else if ((strcmp(cmd, "t") == 0) || (strcmp(cmd, "total") == 0))
            {
              stepCNT_total(handle, &total);
              stepCNT_motion_patt(handle, &patt);

              printf("total step:%d\n", total);

              switch (patt)
                {
                  case STEP_PATT_WALK:
                    patt_string = "walk";
                    break;
                  case STEP_PATT_RUN:
                    patt_string = "run";
                    break;
                  case STEP_PATT_STATIC:
                    patt_string = "static";
                    break;
                  default:
                    patt_string = "unknow";
                    break;
                }

              printf("motion pattern: %s\n", patt_string);

            }
          else if ((strcmp(cmd, "h") == 0) || (strcmp(cmd, "help") == 0))
            {
              stepCNT_help();
            }
          else
            {
              printf("%s:  unknown command\n", buffer);

              stepCNT_help();
            }
        }
    }


  ret = stepCNT_stop(handle);
  if (ret != OK)
    {
      printf("stepCNT:an error happend in stop process:%d\n", ret);
    }

  return 0;

}
