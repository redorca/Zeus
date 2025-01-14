/****************************************************************************
 * examples/usbserial/usbserial_main.c
 *
 *   Copyright (C) 2008, 2010-2012, 2015-2016 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/boardctl.h>

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#ifdef CONFIG_CDCACM
#  include <nuttx/usb/cdcacm.h>
#endif


/* below macro control IN traction only */
//#define CONFIG_EXAMPLES_USBSERIAL_INONLY

/* below macro control OUT traction only */
//#define CONFIG_EXAMPLES_USBSERIAL_OUTONLY

#define CONFIG_EXAMPLES_USBSERIAL_ONLYSMALL

#ifdef OUTPUT_ALL
#define usbserial_log   printf()
#else
//#define usbserial_log(...)  syslog(LOG_DEBUG, ##__VA_ARGS__)

#define usbserial_log(...)  do{}while(0)

#endif
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_EXAMPLES_USBSERIAL_INONLY) && defined(CONFIG_EXAMPLES_USBSERIAL_OUTONLY)
#  error "Cannot define both CONFIG_EXAMPLES_USBSERIAL_INONLY and _OUTONLY"
#endif
#if defined(CONFIG_EXAMPLES_USBSERIAL_ONLYSMALL) && defined(CONFIG_EXAMPLES_USBSERIAL_ONLYBIG)
#  error "Cannot define both CONFIG_EXAMPLES_USBSERIAL_ONLYSMALL and _ONLYBIG"
#endif

#if !defined(CONFIG_EXAMPLES_USBSERIAL_ONLYBIG) && !defined(CONFIG_EXAMPLES_USBSERIAL_ONLYSMALL)
#  ifndef CONFIG_EXAMPLES_USBSERIAL_OUTONLY
#    define COUNTER_NEEDED 1
#  endif
#endif

#ifndef CONFIG_USBDEV_TRACE_INITIALIDSET
#  define CONFIG_USBDEV_TRACE_INITIALIDSET 0
#endif

//#define CONFIG_EXAMPLES_USBSERIAL_TRACECLASS
//#define CONFIG_EXAMPLES_USBSERIAL_TRACETRANSFERS
//#define CONFIG_EXAMPLES_USBSERIAL_TRACECONTROLLER

#ifdef CONFIG_EXAMPLES_USBSERIAL_TRACEINIT
#  define TRACE_INIT_BITS       (TRACE_INIT_BIT)
#else
#  define TRACE_INIT_BITS       (0)
#endif

#define TRACE_ERROR_BITS        (TRACE_DEVERROR_BIT|TRACE_CLSERROR_BIT)

#ifdef CONFIG_EXAMPLES_USBSERIAL_TRACECLASS
#  define TRACE_CLASS_BITS      (TRACE_CLASS_BIT|TRACE_CLASSAPI_BIT|TRACE_CLASSSTATE_BIT)
#else
#  define TRACE_CLASS_BITS      (0)
#endif

#ifdef CONFIG_EXAMPLES_USBSERIAL_TRACETRANSFERS
#  define TRACE_TRANSFER_BITS   (TRACE_OUTREQQUEUED_BIT|TRACE_INREQQUEUED_BIT|TRACE_READ_BIT|\
                                 TRACE_WRITE_BIT|TRACE_COMPLETE_BIT)
#else
#  define TRACE_TRANSFER_BITS   (0)
#endif

#ifdef CONFIG_EXAMPLES_USBSERIAL_TRACECONTROLLER
#  define TRACE_CONTROLLER_BITS (TRACE_EP_BIT|TRACE_DEV_BIT)
#else
#  define TRACE_CONTROLLER_BITS (0)
#endif

#ifdef CONFIG_EXAMPLES_USBSERIAL_TRACEINTERRUPTS
#  define TRACE_INTERRUPT_BITS  (TRACE_INTENTRY_BIT|TRACE_INTDECODE_BIT|TRACE_INTEXIT_BIT)
#else
#  define TRACE_INTERRUPT_BITS  (0)
#endif

#define TRACE_BITSET            (TRACE_INIT_BITS|TRACE_ERROR_BITS|TRACE_CLASS_BITS|\
                                 TRACE_TRANSFER_BITS|TRACE_CONTROLLER_BITS|TRACE_INTERRUPT_BITS)
#ifdef CONFIG_CDCACM
#  define USBSER_DEVNAME "/dev/ttyACM0"
#else
#  define USBSER_DEVNAME "/dev/ttyUSB0"
#endif

#ifndef CONFIG_EXAMPLES_USBSERIAL_BUFSIZE
#  define CONFIG_EXAMPLES_USBSERIAL_BUFSIZE 256
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_USBSERIAL_ONLYBIG
static const char g_shortmsg[] = "Hello, World!!\n\r";
#endif

#ifndef CONFIG_EXAMPLES_USBSERIAL_ONLYSMALL
static const char g_longmsg[] =
  "The Spanish Armada a Speech by Queen Elizabeth I of England\n\r"
  "Addressed to the English army at Tilbury Fort - 1588\n\r"
  "My loving people, we have been persuaded by some, that are careful of our "
  "safety, to take heed how we commit ourselves to armed multitudes, for fear "
  "of treachery; but I assure you, I do not desire to live to distrust my "
  "faithful and loving people.\n\r"
  "Let tyrants fear; I have always so behaved myself that, under God, I have "
  "placed my chiefest strength and safeguard in the loyal hearts and good will "
  "of my subjects. And therefore I am come amongst you at this time, not as for "
  "my recreation or sport, but being resolved, in the midst and heat of the "
  "battle, to live or die amongst you all; to lay down, for my God, and for "
  "my kingdom, and for my people, my honour and my blood, even the dust.\n\r"
  "I know I have but the body of a weak and feeble woman; but I have the heart "
  "of a king, and of a king of England, too; and think foul scorn that Parma "
  "or Spain, or any prince of Europe, should dare to invade the borders of my "
  "realms: to which, rather than any dishonour should grow by me, I myself will "
  "take up arms; I myself will be your general, judge, and rewarder of every "
  "one of your virtues in the field.\n\r"
  "I know already, by your forwardness, that you have deserved rewards and "
  "crowns; and we do assure you, on the word of a prince, they shall be duly "
  "paid you. In the mean my lieutenant general shall be in my stead, than whom "
  "never prince commanded a more noble and worthy subject; not doubting by "
  "your obedience to my general, by your concord in the camp, and by your "
  "valour in the field, we shall shortly have a famous victory over the enemies "
  "of my God, of my kingdom, and of my people.\n\r";
#endif

#ifndef CONFIG_EXAMPLES_USBSERIAL_INONLY
static char g_iobuffer[CONFIG_EXAMPLES_USBSERIAL_BUFSIZE];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_USBDEV_TRACE
static int trace_callback(struct usbtrace_s *trace, void *arg)
{
  usbtrace_trprintf((trprintf_t)printf, trace->event, trace->value);
  return 0;
}

static void dumptrace(void)
{
  (void)usbtrace_enumerate(trace_callback, NULL);
}
#else
#  define dumptrace()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * usbserial_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int usbserial_main(int argc, char *argv[])
#endif
{
  struct boardioc_usbdev_ctrl_s ctrl;
  FAR void *handle;
#ifndef CONFIG_EXAMPLES_USBSERIAL_INONLY
  int infd;
#endif
#ifndef CONFIG_EXAMPLES_USBSERIAL_OUTONLY
  int outfd;
#endif
#ifdef COUNTER_NEEDED
  int count = 0;
#endif
  ssize_t nbytes;
#ifndef CONFIG_EXAMPLES_USBSERIAL_INONLY
  int i, j, k;
#endif
  int ret;

  /* Initialize the USB serial driver */

  printf("usbserial_main: Registering USB serial driver\n");

#ifdef CONFIG_CDCACM

  ctrl.usbdev   = BOARDIOC_USBDEV_CDCACM;
  ctrl.action   = BOARDIOC_USBDEV_CONNECT;
  ctrl.instance = 0;
  ctrl.handle   = &handle;

#else

  ctrl.usbdev   = BOARDIOC_USBDEV_PL2303;
  ctrl.action   = BOARDIOC_USBDEV_CONNECT;
  ctrl.instance = 0;
  ctrl.handle   = &handle;

#endif

  ret = boardctl(BOARDIOC_USBDEV_CONTROL, (uintptr_t)&ctrl);
  if (ret < 0)
    {
      printf("usbserial_main: ERROR: Failed to create the USB serial device: %d\n",
             -ret);
      return 1;
    }

  printf("usbserial_main: Successfully registered the serial driver\n");

#if defined(CONFIG_USBDEV_TRACE) && CONFIG_USBDEV_TRACE_INITIALIDSET != 0
  /* If USB tracing is enabled and tracing of initial USB events is specified,
   * then dump all collected trace data to stdout
   */

  sleep(10);
  dumptrace();
#endif

  /* Then, in any event, configure trace data collection as configured */

  usbtrace_enable(TRACE_BITSET);
  setlogmask(LOG_UPTO(LOG_WARNING));

  /* Open the USB serial device for writing (blocking) */

#ifndef CONFIG_EXAMPLES_USBSERIAL_OUTONLY
  do
    {
      printf("usbserial_main: Opening USB serial driver\n");
      outfd = open(USBSER_DEVNAME, O_WRONLY);
      if (outfd < 0)
        {
          int errcode = errno;
          printf("usbserial_main: ERROR: Failed to open " USBSER_DEVNAME
                 " for writing: %d\n", errcode);

          /* ENOTCONN means that the USB device is not yet connected */

          if (errcode == ENOTCONN)
            {
              printf("usbserial_main:        Not connected. Wait and try again.\n");
              sleep(5);
            }
          else
            {
              /* Give up on other errors */

              printf("usbserial_main:        Aborting\n");
              return 2;
            }
        }

      /* If USB tracing is enabled, then dump all collected trace data to stdout */

      dumptrace();
    }
  while (outfd < 0);
#endif

  /* Open the USB serial device for reading (non-blocking) */

#ifndef CONFIG_EXAMPLES_USBSERIAL_INONLY
#ifndef CONFIG_EXAMPLES_USBSERIAL_OUTONLY
  infd = open(USBSER_DEVNAME, O_RDONLY | O_NONBLOCK);
  if (infd < 0)
    {
      printf("usbserial_main: ERROR: Failed to open " USBSER_DEVNAME
             " for reading: %d\n", errno);
      close(outfd);
      return 3;
    }
#else
  do
    {
      infd = open(USBSER_DEVNAME, O_RDONLY | O_NONBLOCK);
      if (infd < 0)
        {
          int errcode = errno;
          printf("usbserial_main: ERROR: Failed to open " USBSER_DEVNAME
                 " for reading: %d\n", errno);

          /* ENOTCONN means that the USB device is not yet connected */

          if (errcode == ENOTCONN)
            {
              printf("usbserial_main:        Not connected. Wait and try again.\n");
              sleep(5);
            }
          else
            {
              /* Give up on other errors */

              printf("usbserial_main:        Aborting\n");
              return 3;
            }
        }

      /* If USB tracing is enabled, then dump all collected trace data to stdout */

      dumptrace();
    }
  while (infd < 0);
#endif
#endif

  printf("usbserial_main: Successfully opened the serial driver\n");
  sleep(3);

  /* Send messages and get responses -- forever */

  for (;;)
    {
      /* Test IN (device-to-host) messages */

#ifndef CONFIG_EXAMPLES_USBSERIAL_OUTONLY
#if !defined(CONFIG_EXAMPLES_USBSERIAL_ONLYBIG) && !defined(CONFIG_EXAMPLES_USBSERIAL_ONLYSMALL)
      if (count < 8)
        {
          printf("usbserial_main: Saying hello\n");
          nbytes = write(outfd, g_shortmsg, sizeof(g_shortmsg));
          count++;
        }
      else
        {
          printf("usbserial_main: Reciting QEI's speech of 1588\n");
          nbytes = write(outfd, g_longmsg, sizeof(g_longmsg));
          count = 0;
        }

#elif !defined(CONFIG_EXAMPLES_USBSERIAL_ONLYSMALL)
      printf("usbserial_main: Reciting QEI's speech of 1588\n");
      nbytes = write(outfd, g_longmsg, sizeof(g_longmsg));

#else /* !defined(CONFIG_EXAMPLES_USBSERIAL_ONLYBIG) */
      printf("usbserial_main: Saying hello\n");
      nbytes = write(outfd, g_shortmsg, sizeof(g_shortmsg));
#endif

      /* Test if the write was successful */

      if (nbytes < 0)
        {
          printf("usbserial_main: ERROR: write failed: %d\n", errno);
#ifndef CONFIG_EXAMPLES_USBSERIAL_INONLY
          close(infd);
#endif
          close(outfd);
          return 4;
        }

      printf("usbserial_main: %ld bytes sent\n", (long)nbytes);
#endif /* CONFIG_EXAMPLES_USBSERIAL_OUTONLY */

      /* Test OUT (host-to-device) messages */

#ifndef CONFIG_EXAMPLES_USBSERIAL_INONLY
      /* Poll for incoming messages */

      printf("usbserial_main: Polling for OUT messages\n");
      uint32_t total_cnt = 0;

      for (i = 0; i < 64; i++)
        {
          memset(g_iobuffer, 'X', CONFIG_EXAMPLES_USBSERIAL_BUFSIZE);
          nbytes = read(infd, g_iobuffer, CONFIG_EXAMPLES_USBSERIAL_BUFSIZE);
          if (nbytes < 0)
            {
              int errorcode = errno;
              if (errorcode != EAGAIN)
                {
                  printf("usbserial_main: ERROR: read failed: %d\n", errno);
                  close(infd);
#ifndef CONFIG_EXAMPLES_USBSERIAL_OUTONLY
                  close(outfd);
#endif
                  return 6;
                }
            }
          else
            {
              total_cnt += nbytes;
              printf("usbserial_main: Received %ld bytes:\n", (long)nbytes);
              if (nbytes > 0)
                {
                  for (j = 0; j < nbytes; j += 16)
                    {
                      usbserial_log("usbserial_main: %03x: ", j);
                      for (k = 0; k < 16; k++)
                        {
                          if (j + k < nbytes)
                            {
                              usbserial_log("%02x", g_iobuffer[j + k]);
                            }
                          else
                            {
                              usbserial_log("  ");
                            }
                        }

                      usbserial_log(" ");
                      for (k = 0; k < 16; k++)
                        {
                          if (j + k < nbytes)
                            {
                              if (g_iobuffer[j + k] >= 0x20 && g_iobuffer[j + k] < 0x7f)
                                {
                                  usbserial_log("%c", g_iobuffer[j + k]);
                                }
                              else
                                {
                                  usbserial_log(".");
                                }
                            }
                          else
                            {
                              usbserial_log(" ");
                            }
                        }

                      usbserial_log("\n");
                    }
                }
            }

          usleep(500000);
        }
      printf("CDCACM: Total rec %ld bytes\n", (long)total_cnt);

#else /* CONFIG_EXAMPLES_USBSERIAL_INONLY */
      printf("usbserial_main: Waiting\n");
      sleep(5);
#endif /* CONFIG_EXAMPLES_USBSERIAL_INONLY */

      /* If USB tracing is enabled, then dump all collected trace data to stdout */

      dumptrace();
    }

  /* Won't get here, but if we did this what we would have to do */

#ifndef CONFIG_EXAMPLES_USBSERIAL_INONLY
  close(infd);
#endif
#ifndef CONFIG_EXAMPLES_USBSERIAL_OUTONLY
  close(outfd);
#endif
  return 0;
}
