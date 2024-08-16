/****************************************************************************
 * examples/cpput/test_timer.cxx
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
#include "CppUTest/TestHarness.h"
#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>

#include <nuttx/timers/timer.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_EXAMPLE_TIMER_DEVNAME
#  define CONFIG_EXAMPLE_TIMER_DEVNAME "/dev/timer0"
#endif

#  define CONFIG_EXAMPLE_TIMER_INTERVAL 100000
#  define CONFIG_EXAMPLE_TIMER_DELAY 100000
#  define CONFIG_EXAMPLE_TIMER_NSAMPLES 20

#ifndef CONFIG_EXAMPLE_TIMER_SIGNO
#  define CONFIG_EXAMPLE_TIMER_SIGNO 17
#endif


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static volatile unsigned long g_nsignals;

/****************************************************************************
 * timer_sighandler
 ****************************************************************************/

static void timer_sighandler(int signo, FAR siginfo_t *siginfo,
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
 * timer_status
 ****************************************************************************/

bool chk_timer_status(int fd)
{
  struct timer_status_s status;
  int ret;

  /* Get timer status */

  ret = ioctl(fd, TCIOC_GETSTATUS, (unsigned long)((uintptr_t)&status));
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to get timer status: %d\n", errno);
    }
  else {
  /* Print the timer status */

  printf("  flags: %08lx timeout: %lu timeleft: %lu nsignals: %lu\n",
         (unsigned long)status.flags, (unsigned long)status.timeout,
         (unsigned long)status.timeleft, g_nsignals);
  }
  return (ret >= 0);
}

int open_timer(){
  printf("Open %s\n", CONFIG_EXAMPLE_TIMER_DEVNAME);
  int fd= open(CONFIG_EXAMPLE_TIMER_DEVNAME, O_RDONLY);
  CHECK_TRUE(fd >= 0);
  return fd;
}
void close_timer(int fd){
  printf("Close %s\n", CONFIG_EXAMPLE_TIMER_DEVNAME);
  close(fd);
}
void set_interval(int fd){
  /* Set the timer i/timer_sighandler/.nterval */
  printf("Set timer interval to %lu\n",
         (unsigned long)CONFIG_EXAMPLE_TIMER_INTERVAL);
  CHECK_TRUE((0 <= ioctl(fd, TCIOC_SETTIMEOUT, CONFIG_EXAMPLE_TIMER_INTERVAL)));
}
void set_sigaction(struct sigaction *act){
  act->sa_sigaction = timer_sighandler;
  act->sa_flags     = SA_SIGINFO;

  (void)sigfillset(&act->sa_mask);
  (void)sigdelset(&act->sa_mask, CONFIG_EXAMPLE_TIMER_SIGNO);

  CHECK_TRUE((OK == sigaction(CONFIG_EXAMPLE_TIMER_SIGNO, act, NULL)));
}
void clear_sigaction(struct sigaction *act){
  act->sa_handler = SIG_DFL;
  (void)sigaction(CONFIG_EXAMPLE_TIMER_SIGNO, act, NULL);
}
void set_timer_handler(int fd, struct timer_notify_s *notify){
  printf("Attach timer handler\n");

  notify->arg   = NULL;
  notify->pid   = getpid();
  notify->signo = CONFIG_EXAMPLE_TIMER_SIGNO;

  CHECK_TRUE( (ioctl(fd, TCIOC_NOTIFICATION, (unsigned long)((uintptr_t)notify)) >= 0));
}
void start_timer(int fd){
  printf("Start the timer\n");
  CHECK_TRUE( (0 <= ioctl(fd, TCIOC_START, 0)));
}
void stop_timer(int fd){
  printf("Stop the timer\n");
  CHECK_TRUE( (0 <= ioctl(fd, TCIOC_STOP, 0)));
}
/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * timer_main
 ****************************************************************************/
int test_timer(int fd)
{
  struct timer_notify_s notify;
  struct sigaction act;
  int i;

  set_interval(fd);

  /* Attach a signal handler to catch the notifications.  NOTE that using
   * signal handler is very slow.  A much more efficient thing to do is to
   * create a separate pthread that waits on sigwaitinfo() for timer events.
   * Much less overhead in that case.
   */

  g_nsignals       = 0;
  set_sigaction(&act);
  set_timer_handler(fd, &notify);
  start_timer(fd);

  /* Wait a bit showing timer status */
  for (i =0; i < 20; i++){
  usleep(100000);
  }
  chk_timer_status(fd);
  CHECK_TRUE(g_nsignals >= (CONFIG_EXAMPLE_TIMER_NSAMPLES - 1));
  CHECK_TRUE(g_nsignals <= (CONFIG_EXAMPLE_TIMER_NSAMPLES + 1));

  stop_timer(fd);
  clear_sigaction(&act);
  chk_timer_status(fd);

  return 0;
}


TEST_GROUP(TimerTestGroup)
{
  int fd = -1;
  void setup(){
    fd = open_timer();
  }
  void teardown(){
    close_timer(fd);
  }
};

TEST(TimerTestGroup, Test_Timer_openclose)
{
  //do nothing here; open/close has been called in setup & teardown functions.
}
TEST(TimerTestGroup, Test_Timer_startstop)
{
  start_timer(fd);
  stop_timer(fd);
}
TEST(TimerTestGroup, Test_Timer_interval)
{
  set_interval(fd);
}
TEST(TimerTestGroup, Test_Timer_timerfunc)
{
  test_timer(fd);
}

