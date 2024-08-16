/****************************************************************************
 * drivers/timers/counter.c
 *
 *   Copyright (C) 2014, 2016 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <semaphore.h>
#include <fcntl.h>
#include <signal.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/timers/counter.h>

#ifdef CONFIG_COUNTER

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct rtc_upperhalf_s
{
  uint8_t   crefs;         /* The number of times the device has been opened */
  uint8_t   signo;         /* The signal number to use in the notification */
  pid_t     pid;           /* The ID of the task/thread to receive the signal */
  FAR void *arg;           /* An argument to pass with the signal */
  FAR char *path;          /* Registration path */

  /* The contained lower-half driver */

  FAR struct rtc_lowerhalf_s *lower;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool    rtc_notifier(FAR uint32_t *next_interval_us, FAR void *arg);
static int     rtc_open(FAR struct file *filep);
static int     rtc_close(FAR struct file *filep);
static ssize_t rtc_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen);
static ssize_t rtc_write(FAR struct file *filep, FAR const char *buffer,
                         size_t buflen);
static int     rtc_ioctl(FAR struct file *filep, int cmd,
                         unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_rtcops =
{
  rtc_open,  /* open */
  rtc_close, /* close */
  rtc_read,  /* read */
  rtc_write, /* write */
  NULL,        /* seek */
  rtc_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , NULL       /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL       /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/************************************************************************************
 * Name: rtc_notifier
 *
 * Description:
 *   Notify the application via a signal when the rtc interrupt occurs
 *
 * REVISIT: This function prototype is insufficient to support signaling
 *
 ************************************************************************************/

static bool rtc_notifier(FAR uint32_t *next_interval_us, FAR void *arg)
{
  FAR struct rtc_upperhalf_s *upper = (FAR struct rtc_upperhalf_s *)arg;
#ifdef CONFIG_CAN_PASS_STRUCTS
  union sigval value;
#endif

  DEBUGASSERT(upper != NULL);

  /* Signal the waiter.. if there is one */

#ifdef CONFIG_CAN_PASS_STRUCTS
  value.sival_ptr = upper->arg;
  (void)sigqueue(upper->pid, upper->signo, value);
#else
  (void)sigqueue(upper->pid, upper->signo, upper->arg);
#endif

  return true;
}

/************************************************************************************
 * Name: rtc_open
 *
 * Description:
 *   This function is called whenever the rtc device is opened.
 *
 ************************************************************************************/

static int rtc_open(FAR struct file *filep)
{
  FAR struct inode             *inode = filep->f_inode;
  FAR struct rtc_upperhalf_s *upper = inode->i_private;
  uint8_t                       tmp;
  int                           ret;
  tmrinfo("crefs: %d\n", upper->crefs);

  /* Increment the count of references to the device.  If this the first
   * time that the driver has been opened for this device, then initialize
   * the device.
   */

  tmp = upper->crefs + 1;
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ret = -EMFILE;
      goto errout;
    }

  /* Save the new open count */

  upper->crefs = tmp;
  ret = OK;
errout:
  return ret;
}

/************************************************************************************
 * Name: rtc_close
 *
 * Description:
 *   This function is called when the rtc device is closed.
 *
 ************************************************************************************/

static int rtc_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rtc_upperhalf_s *upper = inode->i_private;

  tmrinfo("crefs: %d\n", upper->crefs);

  /* Decrement the references to the driver.  If the reference count will
   * decrement to 0, then uninitialize the driver.
   */

  if (upper->crefs > 0)
    {
      upper->crefs--;
    }

  return OK;
}

/************************************************************************************
 * Name: rtc_read
 *
 * Description:
 *   A dummy read method.  This is provided only to satisfy the VFS layer.
 *
 ************************************************************************************/

static ssize_t rtc_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  /* Return zero -- usually meaning end-of-file */

  return 0;
}

/************************************************************************************
 * Name: rtc_write
 *
 * Description:
 *   A dummy write method.  This is provided only to satisfy the VFS layer.
 *
 ************************************************************************************/

static ssize_t rtc_write(FAR struct file *filep, FAR const char *buffer,
                         size_t buflen)
{
  return 0;
}

/************************************************************************************
 * Name: rtc_ioctl
 *
 * Description:
 *   The standard ioctl method.  This is where ALL of the rtc work is
 *   done.
 *
 ************************************************************************************/

static int rtc_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode             *inode = filep->f_inode;
  FAR struct rtc_upperhalf_s *upper = inode->i_private;
  FAR struct rtc_lowerhalf_s *lower = upper->lower;
  int                           ret;
  tmrinfo("cmd: %d arg: %ld\n", cmd, arg);
  DEBUGASSERT(upper && lower);

  /* Handle built-in ioctl commands */

  switch (cmd)
    {
      /* cmd:         TCIOC_START
       * Description: Start the rtc
       * Argument:    Ignored
       */

      case RTCIOC_START:
        {
          /* Start the rtc, resetting the time to the current timeout */

          if (lower->ops->start)
            {
              ret = lower->ops->start(lower);
            }
          else
            {
              ret = -ENOSYS;
            }
        }
        break;

      /* cmd:         TCIOC_STOP
       * Description: Stop the rtc
       * Argument:    Ignored
       */

      case RTCIOC_STOP:
        {
          /* Stop the rtc */

          if (lower->ops->stop)
            {
              ret = lower->ops->stop(lower);
            }
          else
            {
              ret = -ENOSYS;
            }
        }
        break;

      /* cmd:         TCIOC_GETSTATUS
       * Description: Get the status of the rtc.
       * Argument:    A writeable pointer to struct rtc_status_s.
       */

      case RTCIOC_GETSTATUS:
        {
          FAR struct rtc_status_s *status;

          /* Get the current rtc status */

          if (lower->ops->getstatus) /* Optional */
            {
              status = (FAR struct rtc_status_s *)((uintptr_t)arg);
              if (status)
                {
                  ret = lower->ops->getstatus(lower, status);
                }
              else
                {
                  ret = -EINVAL;
                }
            }
          else
            {
              ret = -ENOSYS;
            }
        }
        break;

      /* cmd:         TCIOC_SETTIMEOUT
       * Description: Reset the timeout to this value
       * Argument:    A 32-bit timeout value in microseconds.
       *
       * TODO: pass pointer to uint64 ns? Need to determine if these RTCs
       * are 16 or 32 bit...
       */

      case RTCIOC_SETTIMEOUT:
        {
          /* Set a new timeout value (and reset the rtc) */

          if (lower->ops->settimeout) /* Optional */
            {
              ret = lower->ops->settimeout(lower, (uint32_t)arg);
            }
          else
            {
              ret = -ENOSYS;
            }
        }
        break;

      /* cmd:         TCIOC_NOTIFICATION
       * Description: Notify application via a signal when the rtc expires.
       * Argument:    signal number
       *
       * NOTE: This ioctl cannot be support in the kernel build mode. In that
       * case direct callbacks from kernel space into user space is forbidden.
       */

      case RTCIOC_NOTIFICATION:
        {
          FAR struct rtc_notify_s *notify =
            (FAR struct rtc_notify_s *)((uintptr_t)arg);

          if (notify != NULL)
            {
              upper->signo = notify->signo;
              upper->pid   = notify->pid;
              upper->arg   = notify->arg;

              ret = rtc_setcallback((FAR void *)upper, rtc_notifier, upper);
            }
          else
            {
              ret = -EINVAL;
            }
        }
        break;

      /* Any unrecognized IOCTL commands might be platform-specific ioctl commands */

      default:
        {
          tmrinfo("Forwarding unrecognized cmd: %d arg: %ld\n", cmd, arg);

          /* An ioctl commands that are not recognized by the "upper-half"
           * driver are forwarded to the lower half driver through this
           * method.
           */

          if (lower->ops->ioctl) /* Optional */
            {
              ret = lower->ops->ioctl(lower, cmd, arg);
            }
          else
            {
              ret = -ENOSYS;
            }
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
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
                       FAR struct rtc_lowerhalf_s *lower)
{
  FAR struct rtc_upperhalf_s *upper;
  int ret;

  DEBUGASSERT(path && lower);
  tmrinfo("Entry: path=%s\n", path);

  /* Allocate the upper-half data structure */

  upper = (FAR struct rtc_upperhalf_s *)
          kmm_zalloc(sizeof(struct rtc_upperhalf_s));
  if (!upper)
    {
      tmrerr("ERROR: Upper half allocation failed\n");
      goto errout;
    }

  /* Initialize the rtc device structure (it was already zeroed
   * by kmm_zalloc()).
   */

  upper->lower = lower;

  /* Copy the registration path */

  upper->path = strdup(path);
  if (!upper->path)
    {
      tmrerr("ERROR: Path allocation failed\n");
      goto errout_with_upper;
    }

  /* Register the rtc device */

  ret = register_driver(path, &g_rtcops, 0666, upper);
  if (ret < 0)
    {
      tmrerr("ERROR: register_driver failed: %d\n", ret);
      goto errout_with_path;
    }

  return (FAR void *)upper;

errout_with_path:
  kmm_free(upper->path);

errout_with_upper:
  kmm_free(upper);

errout:
  return NULL;
}

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

void rtc_unregister(FAR void *handle)
{
  FAR struct rtc_upperhalf_s *upper;
  FAR struct rtc_lowerhalf_s *lower;

  /* Recover the pointer to the upper-half driver state */

  upper = (FAR struct rtc_upperhalf_s *)handle;
  DEBUGASSERT(upper != NULL && upper->lower != NULL);
  lower = upper->lower;

  tmrinfo("Unregistering: %s\n", upper->path);

  /* Disable the rtc */

  DEBUGASSERT(lower->ops->stop); /* Required */
  (void)lower->ops->stop(lower);

  /* Unregister the rtc device */

  (void)unregister_driver(upper->path);

  /* Then free all of the driver resources */

  kmm_free(upper->path);
  kmm_free(upper);
}

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
 *   arg      - Argument to be provided with the callback
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

int rtc_setcallback(FAR void *handle, tccb_t callback, FAR void *arg)
{
  FAR struct rtc_upperhalf_s *upper;
  FAR struct rtc_lowerhalf_s *lower;

  /* Recover the pointer to the upper-half driver state */

  upper = (FAR struct rtc_upperhalf_s *)handle;
  DEBUGASSERT(upper != NULL && upper->lower != NULL);
  lower = upper->lower;
  DEBUGASSERT(lower->ops != NULL);

  /* Check if the lower half driver supports the setcallback method */

  if (lower->ops->setcallback != NULL) /* Optional */
    {
      /* Yes.. Defer the hander attachment to the lower half driver */

      lower->ops->setcallback(lower, callback, arg);
      return OK;
    }

  return -ENOSYS;
}

#endif /* CONFIG_COUNTER */
