/********************************************************************************
 * include/signal.h
 *
 *   Copyright (C) 2007-2009, 2011, 2013-2017 Gregory Nutt. All rights reserved.
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
 ********************************************************************************/

#ifndef __INCLUDE_SIGNAL_H
#define __INCLUDE_SIGNAL_H

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>
#include <time.h>

/********************************************************************************
 * Pre-processor Definitions
 ********************************************************************************/

/* Signal set management definitions and macros. */

#define NULL_SIGNAL_SET ((sigset_t)0x00000000)
#define ALL_SIGNAL_SET  ((sigset_t)0xffffffff)
#define MIN_SIGNO       0
#define MAX_SIGNO       31
#define GOOD_SIGNO(s)   ((((unsigned)(s))<=MAX_SIGNO))
#define SIGNO2SET(s)    ((sigset_t)1 << (s))

/* All signals are "real time" signals */

#define SIGRTMIN        MIN_SIGNO  /* First real time signal */
#define SIGRTMAX        MAX_SIGNO  /* Last real time signal */

/* A few of the real time signals are used within the OS.  They have
 * default values that can be overridden from the configuration file. The
 * rest are all user signals.
 *
 * The signal number zero is wasted for the most part.  It is a valid
 * signal number, but has special meaning at many interfaces (e.g., Kill()).
 *
 * These are the semi-standard signal definitions:
 */

#ifndef CONFIG_SIG_SIGUSR1
#  define SIGUSR1       1  /* User signal 1 */
#else
#  define SIGUSR1       CONFIG_SIG_SIGUSR1
#endif

#ifndef CONFIG_SIG_SIGUSR2
#  define SIGUSR2       2  /* User signal 2 */
#else
#  define SIGUSR2       CONFIG_SIG_SIGUSR2
#endif

#ifndef CONFIG_SIG_SIGALARM
#  define SIGALRM       3  /* Default signal used with POSIX timers (used only */
                           /* no other signal is provided) */
#else
#  define SIGALRM       CONFIG_SIG_SIGALARM
#endif

#ifdef CONFIG_SCHED_HAVE_PARENT
#  ifndef CONFIG_SIG_SIGCHLD
#    define SIGCHLD     4  /* Used by child threads to signal parent thread */
#  else
#    define SIGCHLD     CONFIG_SIG_SIGCHLD
#  endif
#endif

#ifdef CONFIG_FS_AIO
#  ifndef CONFIG_SIG_POLL
#    define SIGPOLL     5  /* Sent when an asynchronous I/O event occurs */
#  else
#    define SIGPOLL     CONFIG_SIG_POLL
#  endif
#endif

/* The following are non-standard signal definitions */

#ifndef CONFIG_DISABLE_PTHREAD
#  ifndef CONFIG_SIG_SIGCONDTIMEDOUT
#    define SIGCONDTIMEDOUT 16  /* Used in the implementation of pthread_cond_timedwait */
#  else
#    define SIGCONDTIMEDOUT CONFIG_SIG_SIGCONDTIMEDOUT
#  endif
#endif

/* SIGWORK is used to wake up various internal, NuttX worker thread */

#if defined(CONFIG_SCHED_WORKQUEUE) || defined(CONFIG_PAGING)
#  ifndef CONFIG_SIG_SIGWORK
#    define SIGWORK     17  /* Used to wake up the work queue */
#  else
#    define SIGWORK     CONFIG_SIG_SIGWORK
#  endif
#endif

/* sigprocmask() "how" definitions. Only one of the following can be specified: */

#define SIG_BLOCK       1  /* Block the given signals */
#define SIG_UNBLOCK     2  /* Unblock the given signals */
#define SIG_SETMASK     3  /* Set the signal mask to the current set */

/* struct sigaction flag values */

#define SA_NOCLDSTOP    (1 << 0) /* Do not generate SIGCHILD when
                                  * children stop (ignored) */
#define SA_SIGINFO      (1 << 1) /* Invoke the signal-catching function
                                  * with 3 args instead of 1
                                  * (always assumed) */
#define SA_NOCLDWAIT    (1 << 2) /* If signo=SIGCHLD, exit status of child
                                  * processes will be discarded */

/* These are the possible values of the signfo si_code field */

#define SI_USER         0  /* Signal sent from kill, raise, or abort */
#define SI_QUEUE        1  /* Signal sent from sigqueue */
#define SI_TIMER        2  /* Signal is result of timer expiration */
#define SI_ASYNCIO      3  /* Signal is the result of asynch IO completion */
#define SI_MESGQ        4  /* Signal generated by arrival of a message on an */
                           /* empty message queue */
#define CLD_EXITED      5  /* Child has exited (SIGCHLD only) */
#define CLD_KILLED      6  /* Child was killed (SIGCHLD only) */
#define CLD_DUMPED      7  /* Child terminated abnormally (SIGCHLD only) */
#define CLD_TRAPPED     8  /* Traced child has trapped (SIGCHLD only) */
#define CLD_STOPPED     9  /* Child has stopped (SIGCHLD only) */
#define CLD_CONTINUED   10 /* Stopped child had continued (SIGCHLD only) */

/* Values for the sigev_notify field of struct sigevent */

#define SIGEV_NONE      0 /* No asynchronous notification is delivered */
#define SIGEV_SIGNAL    1 /* Notify via signal,with an application-defined value */
#ifdef CONFIG_SIG_EVTHREAD
#  define SIGEV_THREAD  3 /* A notification function is called */
#endif

/* Special values of of sa_handler used by sigaction and sigset.  They are all
 * treated like NULL for now.  This is okay for SIG_DFL and SIG_IGN because
 * in NuttX, the default action for all signals is to ignore them.
 */

#define SIG_ERR         ((_sa_handler_t)-1)  /* And error occurred */
#define SIG_DFL         ((_sa_handler_t)0)   /* Default is SIG_IGN for all signals */
#define SIG_IGN         ((_sa_handler_t)0)   /* Ignore the signal */
#define SIG_HOLD        ((_sa_handler_t)1)   /* Used only with sigset() */

/********************************************************************************
 * Public Type Definitions
 ********************************************************************************/

/* This defines a set of 32 signals (numbered 0 through 31).
 * REVISIT: Signal 0 is, however, not generally usable since that value has
 * special meaning in some circumstances (e.g., kill()).
 */

typedef uint32_t sigset_t;   /* Bit set of 32 signals */
#define __SIGSET_T_DEFINED 1

/* Possibly volatile-qualified integer type of an object that can be accessed
 * as an atomic entity, even in the presence of asynchronous interrupts.
 */

typedef volatile int sig_atomic_t;

/* This defines the type of the siginfo si_value field */

union sigval
{
  int       sival_int;       /* Integer value */
  FAR void *sival_ptr;       /* Pointer value */
};

/* This structure contains elements that define a queue signal. The following is
 * used to attach a signal to a message queue to notify a task when a message is
 * available on a queue
 */

#ifdef CONFIG_CAN_PASS_STRUCTS
typedef CODE void (*sigev_notify_function_t)(union sigval value);
#else
typedef CODE void (*sigev_notify_function_t)(FAR void *sival_ptr);
#endif


struct pthread_attr_s; /* forward reference (defined in pthread.h) */

struct sigevent
{
  uint8_t      sigev_notify; /* Notification method: SIGEV_SIGNAL, SIGEV_NONE, or SIGEV_THREAD */
  uint8_t      sigev_signo;  /* Notification signal */
  union sigval sigev_value;  /* Data passed with notification */

#ifdef CONFIG_SIG_EVTHREAD
  sigev_notify_function_t sigev_notify_function; /* Notification function */
  FAR struct pthread_attr_s *sigev_notify_attributes;   /* Notification attributes (not used) */
#endif
};

/* The following types is used to pass parameters to/from signal handlers */

struct siginfo
{
  uint8_t      si_signo;     /* Identifies signal */
  uint8_t      si_code;      /* Source: SI_USER, SI_QUEUE, SI_TIMER, SI_ASYNCIO, or SI_MESGQ */
  uint8_t      si_errno;     /* Zero or errno value associated with signal */
  union sigval si_value;     /* Data passed with signal */
#ifdef CONFIG_SCHED_HAVE_PARENT
  pid_t        si_pid;       /* Sending task ID */
  int          si_status;    /* Exit value or signal (SIGCHLD only). */
#endif
};

typedef struct siginfo siginfo_t;
#define __SIGINFO_T_DEFINED 1

/* Non-standard convenience definition of signal handling function types.
 * These should be used only internally within the NuttX signal logic.
 */

typedef CODE void (*_sa_handler_t)(int signo);
typedef CODE void (*_sa_sigaction_t)(int signo, FAR siginfo_t *siginfo,
                                     FAR void *context);

/* The following structure defines the action to take for given signal */

struct sigaction
{
  union
  {
    _sa_handler_t   _sa_handler;
    _sa_sigaction_t _sa_sigaction;
  } sa_u;
  sigset_t          sa_mask;
  int               sa_flags;
};

/* Definitions that adjust the non-standard naming */

#define sa_handler   sa_u._sa_handler
#define sa_sigaction sa_u._sa_sigaction

/********************************************************************************
 * Public Function Prototypes
 ********************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

int kill(pid_t pid, int signo);
int raise(int signo);
int sigaction(int signo, FAR const struct sigaction *act,
              FAR struct sigaction *oact);
int sigaddset(FAR sigset_t *set, int signo);
int sigdelset(FAR sigset_t *set, int signo);
int sigemptyset(FAR sigset_t *set);
int sigfillset(FAR sigset_t *set);
int sighold(int signo);
int sigismember(FAR const sigset_t *set, int signo);
int sigignore(int signo);
_sa_handler_t signal(int signo, _sa_handler_t func);
int sigpause(int signo);
int sigpending(FAR sigset_t *set);
int sigprocmask(int how, FAR const sigset_t *set, FAR sigset_t *oset);
#ifdef CONFIG_CAN_PASS_STRUCTS
int sigqueue(int pid, int signo, union sigval value);
#else
int sigqueue(int pid, int signo, FAR void *sival_ptr);
#endif
int sigrelse(int signo);
_sa_handler_t sigset(int signo, _sa_handler_t func);
int sigwait(FAR const sigset_t *set, FAR int *sig);
int sigtimedwait(FAR const sigset_t *set, FAR struct siginfo *value,
                 FAR const struct timespec *timeout);
int sigsuspend(FAR const sigset_t *sigmask);
int sigwaitinfo(FAR const sigset_t *set, FAR struct siginfo *value);

#undef EXTERN
#ifdef __cplusplus
}
#endif

/********************************************************************************
 * Minimal Type Definitions
 ********************************************************************************/

#else /* __INCLUDE_SIGNAL_H */

#include <stdint.h>

/* Avoid circular dependencies by assuring that simple type definitions are
 * available in any inclusion ordering.
 */

#ifndef __SIGSET_T_DEFINED
typedef uint32_t sigset_t;
#  define __SIGSET_T_DEFINED 1
#endif

#ifndef __SIGINFO_T_DEFINED
struct siginfo;
typedef struct siginfo siginfo_t;
#  define __SIGINFO_T_DEFINED 1
#endif

#endif /* __INCLUDE_SIGNAL_H */
