/****************************************************************************
 * examples/iCE40UP/iCE40UP_main.c
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

#include <nuttx/sensors/ioctl.h>
#include <nuttx/sensors/iCE40UP.h>


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_NSH_BUILTIN_APPS - Build the ICE40UP test as an NSH built-in function.
 *  Default: Built as a standalone program
 */

#ifndef CONFIG_ICE40UP
#  error "ICE40UP device support is not enabled (CONFIG_ICE40UP)"
#endif

#define EXAMPLES_ICE40UP_DEVPATH "/dev/iCE40UP"

/****************************************************************************
 * Private Types
 ****************************************************************************/


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iCE40UP_devpath
 ****************************************************************************/


/****************************************************************************
 * Name: iCE40UP_help
 ****************************************************************************/

/****************************************************************************
 * Name: arg_string
 ****************************************************************************/


/****************************************************************************
 * Name: parse_args
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iCE40UP_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int iCE40UP_main(int argc, char *argv[])
#endif
{
  int fd;
  int errval = 0;
  int ret;
  sigset_t set;
  struct siginfo info;
  int errcode;



  /* Open the comp device for reading */

  printf("iCE40UP_main: Hardware initialized. Opening the iCE40UP device: /dev/iCE40UP\n");

  fd = open("/dev/iCE40UP", O_RDONLY);
  if (fd < 0)
    {
      errcode = errno;
      printf("iCE40UP_main: open /dev/iCE40UP failed: %d\n", errno);
      errval = 2;
      goto errout;
    }


  /* register to receive interrupt */
  ret = ioctl(fd, SNIOC_GA_REGISTER_INT, SIGUSR1);
  if (ret < 0)
    {
      errcode = errno;
      printf("iCE40UP_main: ICE40UP_REGISTER_INT failed: %d\n", errcode);
      errval = 3;
      goto errout_with_dev;
    }
  else
    {
      printf("iCE40UP_main: register to receive signo from interrupt, signo:%d\n", SIGUSR1);
    }

  /* prepare the signal set */
  (void)sigemptyset(&set);
  (void)sigaddset(&set, SIGUSR1);

  /* Now loop the appropriate number of times.
   */


  for (;;)
    {
      /* Flush any output before the loop entered or from the previous pass
       * through the loop.
       */

      fflush(stdout);

      ret = sigwaitinfo(&set, &info);

      if (ret < 0)
        {
          errcode = errno;
          printf("iCE40UP_main: signal wait failed: %d\n", errcode);
          errval = 4;
          goto errout_with_dev;
        }
      else
        {
          if (info.si_value.sival_int)
            {
              printf("iCE40UP_main: start!\n");
            }
          else
            {
              printf("iCE40UP_main: stop!\n");
            }
        }
    }


  /* unregister*/
  ret = ioctl(fd, SNIOC_GA_UNREGISTER_INT, 0);
  if (ret < 0)
    {
      errcode = errno;
      printf("iCE40UP_main: ICE40UP_UNREGISTER_INT ioctl failed: %d\n", errcode);
      errval = 14;
      goto errout_with_dev;
    }
  else
    {
      printf("iCE40UP_main: unregister signal, signo:%d\n", SIGUSR1);
    }


  close(fd);
  return OK;

  /* Error exits */

errout_with_dev:
  close(fd);

errout:
  printf("Terminating!\n");
  fflush(stdout);
  return errval;
}
