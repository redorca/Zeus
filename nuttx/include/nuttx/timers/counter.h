/****************************************************************************
 * include/nuttx/rtcs/rtc.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Bob Doiron
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

#ifndef __INCLUDE_NUTTX_TIMERS_COUNTER_H
#define __INCLUDE_NUTTX_TIMERS_COUNTER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/irq.h>
#include <nuttx/fs/ioctl.h>
#include <stdbool.h>
#include <sys/types.h>

#ifdef CONFIG_COUNTER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands ***********************************************************/
/* The rtc driver uses a standard character driver framework.  However,
 * since the rtc driver is a device control interface and not a data
 * transfer interface, the majority of the functionality is implemented in
 * driver ioctl calls.  The rtc ioctl commands are listed below:
 *
 * These are detected and handled by the "upper half" rtc driver.
 *
 * RTCIOC_START        - Start the rtc
 *                      Argument: Ignored
 * RTCIOC_STOP         - Stop the rtc
 *                      Argument: Ignored
 * RTCIOC_GETSTATUS    - Get the status of the rtc.
 *                      Argument:  A writeable pointer to struct rtc_status_s.
 * RTCIOC_SETTIMEOUT   - Reset the rtc timeout to this value
 *                      Argument: A 32-bit timeout value in microseconds.
 * RTCIOC_NOTIFICATION - Set up to notify an application via a signal when
 *                      the rtc expires.
 *                      Argument: A read-only pointer to an instance of
 *                      stuct rtc_notify_s.
 *
 * WARNING: May change RTCIOC_SETTIMEOUT to pass pointer to 64bit nanoseconds
 * or timespec structure.
 *
 * NOTE: The RTCIOC_SETHANDLER ioctl cannot be supported in the kernel build
 * mode. In that case direct callbacks from kernel space into user space is
 * forbidden.
 *
 * NOTE: _RTCIOC(0x0001) througn _RTCIOC(0x001f) are reserved for use by the
 * rtc driver to assure that the values are unique.  Other rtc drivers,
 * such as the oneshot rtc, must not use IOCTL commands in this numeric
 * range.
 */

#define RTCIOC_START        _RTCIOC(0x0001)
#define RTCIOC_STOP         _RTCIOC(0x0002)
#define RTCIOC_GETSTATUS    _RTCIOC(0x0003)
#define RTCIOC_SETTIMEOUT   _RTCIOC(0x0004)
#define RTCIOC_NOTIFICATION _RTCIOC(0x0005)

#define TC_FIRST           0x0007         /* First required command */

/* User defined ioctl commands are also supported. These will be forwarded
 * by the upper-half QE driver to the lower-half QE driver via the ioctl()
 * method fo the QE lower-half interface.  However, the lower-half driver
 * must reserve a block of commands as follows in order prevent IOCTL
 * command numbers from overlapping.
 */

/* See arch/arm/src/nrf52/nrf52_tim.h (Not usable at that location) */

#define TC_NRF52_FIRST     (TC_FIRST)

/* Bit Settings *************************************************************/
/* Bit settings for the struct rtc_status_s flags field */

#define TCFLAGS_ACTIVE     (1 << 0) /* 1=The rtc is running */
#define TCFLAGS_HANDLER    (1 << 1) /* 1=Call the user function when the
                                     *   rtc expires */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Upper half callback prototype. Returns true to reload the rtc, and the
 * function can modify the next interval if desired.
 */

typedef CODE bool (*tccb_t)(FAR uint32_t *next_interval_us, FAR void *arg);

/* This is the type of the argument passed to the RTCIOC_GETSTATUS ioctl and
 * and returned by the "lower half" getstatus() method.
 */

struct rtc_status_s
{
  uint32_t  flags;          /* See TCFLAGS_* definitions above */
  uint32_t  timeout;        /* The current timeout setting (in microseconds) */
  uint32_t  timeleft;       /* Time left until the rtc expiration
                             * (in microseconds) */
};

/* This is the type of the argument passed to the RTCIOC_NOTIFICATION ioctl */

struct rtc_notify_s
{
  FAR void *arg;            /* An argument to pass with the signal */
  pid_t     pid;            /* The ID of the task/thread to receive the signal */
  uint8_t   signo;          /* The signal number to use in the notification */
};

/* This structure provides the "lower-half" driver operations available to
 * the "upper-half" driver.
 */

struct rtc_lowerhalf_s;
struct rtc_ops_s
{
  /* Required methods ********************************************************/
  /* Start the rtc, resetting the time to the current timeout */

  CODE int (*start)(FAR struct rtc_lowerhalf_s *lower);

  /* Stop the rtc */

  CODE int (*stop)(FAR struct rtc_lowerhalf_s *lower);

  /* Get the current rtc status */

  CODE int (*getstatus)(FAR struct rtc_lowerhalf_s *lower,
                        FAR struct rtc_status_s *status);

  /* Set a new timeout value (and reset the rtc) */

  CODE int (*settimeout)(FAR struct rtc_lowerhalf_s *lower,
                         uint32_t timeout);

  /* Call the NuttX INTERNAL timeout callback on timeout.
   * NOTE:  Providing callback==NULL disable.
   * NOT to call back into applications.
   */

  CODE void (*setcallback)(FAR struct rtc_lowerhalf_s *lower,
                           CODE tccb_t callback, FAR void *arg);

  /* Any ioctl commands that are not recognized by the "upper-half" driver
   * are forwarded to the lower half driver through this method.
   */

  CODE int (*ioctl)(FAR struct rtc_lowerhalf_s *lower, int cmd,
                    unsigned long arg);
};

/* This structure provides the publicly visible representation of the
 * "lower-half" driver state structure.  "lower half" drivers will have an
 * internal structure definition that will be cast-compatible with this
 * structure definitions.
 */

struct rtc_lowerhalf_s
{
  /* Publicly visible portion of the "lower-half" driver state structure. */

  FAR const struct rtc_ops_s  *ops;  /* Lower half operations */

  /* The remainder of the structure is used by the "lower-half" driver
   * for whatever state storage that it may need.
   */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * "Upper-Half" RTC Driver Interfaces
 ****************************************************************************/
/****************************************************************************
 * Name: rtc_register
 *
 * Description:
 *   This function binds an instance of a "lower half" rtc driver with the
 *   "upper half" rtc device and registers that device so that can be used
 *   by application code.
 *
 *   When this function is called, the "lower half" driver should be in the
 *   disabled state (as if the stop() method had already been called).
 *
 *   NOTE:  Normally, this function would not be called by application code.
 *   Rather it is called indirectly through the architecture-specific
 *   initialization.
 *
 * Input parameters:
 *   dev path - The full path to the driver to be registers in the NuttX
 *     pseudo-filesystem.  The recommended convention is to name all rtc
 *     drivers as "/dev/rtc0", "/dev/rtc1", etc.  where the driver
 *     path differs only in the "minor" number at the end of the device name.
 *   lower - A pointer to an instance of lower half rtc driver.  This
 *     instance is bound to the rtc driver and must persists as long as
 *     the driver persists.
 *
 * Returned Value:
 *   On success, a non-NULL handle is returned to the caller.  In the event
 *   of any failure, a NULL value is returned.
 *
 ****************************************************************************/

FAR void *rtc_register(FAR const char *path,
                       FAR struct rtc_lowerhalf_s *lower);

/****************************************************************************
 * Name: rtc_unregister
 *
 * Description:
 *   This function can be called to disable and unregister the rtc
 *   device driver.
 *
 * Input parameters:
 *   handle - This is the handle that was returned by rtc_register()
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void rtc_unregister(FAR void *handle);

/****************************************************************************
 * Kernal internal interfaces.  Thse may not be used by application logic
 ****************************************************************************/

/****************************************************************************
 * Name: rtc_setcallback
 *
 * Description:
 *   This function can be called to add a callback into driver-related code
 *   to handle rtc expirations.  This is a strictly OS internal interface
 *   and may NOT be used by appliction code.
 *
 * Input parameters:
 *   handle   - This is the handle that was returned by rtc_register()
 *   callback - The new rtc interrupt callback
 *   arg      - Argument provided when the callback is called.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef __KERNEL__
int rtc_setcallback(FAR void *handle, tccb_t callback, FAR void *arg);
#endif

/****************************************************************************
 * Platform-Independent "Lower-Half" RTC Driver Interfaces
 ****************************************************************************/

/****************************************************************************
 * Architecture-specific Application Interfaces
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_COUNTER */
#endif  /* __INCLUDE_NUTTX_TIMERS_COUNTER_H */
