/****************************************************************************
 *   examples/rtc/rtc_main.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <nuttx/timers/counter.h>


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#ifndef CONFIG_EXAMPLE_COUNTER_DEVNAME
#  define CONFIG_EXAMPLE_COUNTER_DEVNAME "/dev/rtc0"
#endif

#ifndef CONFIG_EXAMPLE_COUNTER_INTERVAL
#  define CONFIG_EXAMPLE_COUNTER_INTERVAL 1000000  //1 second
#endif

#ifndef CONFIG_EXAMPLE_RTC_SIGNO
#  define CONFIG_EXAMPLE_RTC_SIGNO 18
#endif

#ifndef CONFIG_EXAMPLE_RTC_DELAY
#  define CONFIG_EXAMPLE_RTC_DELAY 500000 //0.5 second
#endif

#ifndef CONFIG_EXAMPLE_RTC_NSAMPLES
#  define CONFIG_EXAMPLE_RTC_NSAMPLES 10
#endif
/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
static volatile unsigned long g_nsignals = 0;
static sem_t sem;

/****************************************************************************
 * rtc_sighandler
 ****************************************************************************/

static void rtc_sighandler(int signo, FAR siginfo_t *siginfo,
                           FAR void *context)
{
  /* Does nothing in this example except for increment a count of signals
   * received.
   *
   * NOTE: The use of signal handler is not recommended if you are concerned
   * about the signal latency.  Instead, a dedicated, high-priority thread
   * that waits on sigwaitinfo() is recommended.  High priority is required
   * if you want a deterministic wake-up time when the signal occurs.
   */

  g_nsignals++;

}


/****************************************************************************
 * rtc_status
 ****************************************************************************/
static void rtc_status(int fd)
{
  struct rtc_status_s status;
  int ret;
  /* Get rtc status */

  ret = ioctl(fd, RTCIOC_GETSTATUS, (unsigned long)((uintptr_t)&status));
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to get rtc status: %d\n", errno);
      close(fd);
      exit(EXIT_FAILURE);
    }

  /* Print the rtc status */

  printf("  flags: %08lx timeout: %lu timeleft: %lu nsignals: %lu\n",
         (unsigned long)status.flags, (unsigned long)status.timeout,
         (unsigned long)status.timeleft, g_nsignals);

}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * rtc_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int rtc_main(int argc, char *argv[])
#endif
{
  struct rtc_notify_s notify;
  struct sigaction act;
  int fd = 0;
  int ret;
  int i;

  sem_init(&sem, 0, 0);
  printf("Hello RTC\n");
  /* Open the rtc device */
  printf("Open %s\n", CONFIG_EXAMPLE_COUNTER_DEVNAME);
  fd = open(CONFIG_EXAMPLE_COUNTER_DEVNAME, O_RDONLY);
  printf("fd=%x\r\n", fd);
  if (fd < 0)
    {
      fprintf(stderr, "ERROR: Failed to open %s: %d\n",
              CONFIG_EXAMPLE_COUNTER_DEVNAME, errno);
      return EXIT_FAILURE;
    }

  /* Show the rtc status before attaching the rtc handler */
  rtc_status(fd);

  /* Attach a signal handler to catch the notifications.  NOTE that using
   * signal handler is very slow.  A much more efficient thing to do is to
   * create a separate pthread that waits on sigwaitinfo() for rtc events.
   * Much less overhead in that case.
   */

  g_nsignals       = 0;

  act.sa_sigaction = rtc_sighandler;
  act.sa_flags     = SA_SIGINFO;

  (void)sigfillset(&act.sa_mask);
  (void)sigdelset(&act.sa_mask, CONFIG_EXAMPLE_RTC_SIGNO);

  ret = sigaction(CONFIG_EXAMPLE_RTC_SIGNO, &act, NULL);
  if (ret != OK)
    {
      fprintf(stderr, "ERROR: Fsigaction failed: %d\n", errno);
      close(fd);
      return EXIT_FAILURE;
    }

  /* Register a callback for notifications using the configured signal.
   *
   * NOTE: If no callback is attached, the rtc stop at the first interrupt.
   */

  printf("Attach rtc handler\n");

  notify.arg   = NULL;
  notify.pid   = getpid();
  notify.signo = CONFIG_EXAMPLE_RTC_SIGNO;

  ret = ioctl(fd, RTCIOC_NOTIFICATION, (unsigned long)((uintptr_t)&notify));
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to set the rtc handler: %d\n", errno);
      close(fd);
      return EXIT_FAILURE;
    }

  /* Show the rtc status before setting the counter interval */
  rtc_status(fd);

  /* Set the rtc interval */
  printf("Set rtc interval to %lu\n",
         (unsigned long)CONFIG_EXAMPLE_COUNTER_INTERVAL);


  ret = ioctl(fd, RTCIOC_SETTIMEOUT, CONFIG_EXAMPLE_COUNTER_INTERVAL);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to set the rtc interval: %d\n", errno);
      close(fd);
      return EXIT_FAILURE;
    }

  /* Show the rtc status before starting */
  rtc_status(fd);

  /* Start the rtc */
  printf("Start the rtc\n");

  ret = ioctl(fd, RTCIOC_START, 0);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to start the rtc: %d\n", errno);
      close(fd);
      return EXIT_FAILURE;
    }

  for (i = 0; i < CONFIG_EXAMPLE_RTC_NSAMPLES; i++)
    {
      /* Wait a bit showing rtc status */
      usleep(CONFIG_EXAMPLE_RTC_DELAY);
      rtc_status(fd);
    }

  /* Stop the rtc */
  printf("Stop the rtc\n");

  ret = ioctl(fd, RTCIOC_STOP, 0);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to stop the rtc: %d\n", errno);
      close(fd);
      return EXIT_FAILURE;
    }

  /* Detach the signal handler */
  act.sa_handler = SIG_DFL;
  (void)sigaction(CONFIG_EXAMPLE_RTC_SIGNO, &act, NULL);

  rtc_status(fd);


  return EXIT_SUCCESS;

}
