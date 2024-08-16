/****************************************************************************
 * drivers/sensors/qdecoder.c
 *
 *   Copyright (C) 2012-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Author: Levin Li     <zhiqiang@zglue.com>
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
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <semaphore.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/sensors/qdecoder.h>

#include <arch/irq.h>

#ifdef CONFIG_QDECODER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Debug ********************************************************************/

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct qd_upperhalf_s
{
  uint8_t                    crefs;    /* The number of times the device has been opened */
  sem_t                      exclsem;  /* Supports mutual exclusion */
  FAR struct qd_lowerhalf_s *lower;    /* lower-half state */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     qd_open(FAR struct file *filep);
static int     qd_close(FAR struct file *filep);
static ssize_t qd_read(FAR struct file *filep, FAR char *buffer, size_t buflen);
static ssize_t qd_write(FAR struct file *filep, FAR const char *buffer, size_t buflen);
static int     qd_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_qdops =
{
  qd_open,  /* open */
  qd_close, /* close */
  qd_read,  /* read */
  qd_write, /* write */
  0,         /* seek */
  qd_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , 0        /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/************************************************************************************
 * Name: qd_open
 *
 * Description:
 *   This function is called whenever the Quadrature decoder device is opened.
 *
 ************************************************************************************/

static int qd_open(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct qd_upperhalf_s *upper = inode->i_private;
  uint8_t                     tmp;
  int                         ret;

  sninfo("crefs: %d\n", upper->crefs);

  /* Get exclusive access to the device structures */

  ret = sem_wait(&upper->exclsem);
  if (ret < 0)
    {
      ret = -errno;
      goto errout;
    }

  /* Increment the count of references to the device.  If this the first
   * time that the driver has been opened for this device, then initialize
   * the device.
   */

  tmp = upper->crefs + 1;
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ret = -EMFILE;
      goto errout_with_sem;
    }

  /* Check if this is the first time that the driver has been opened. */

  if (tmp == 1)
    {
      FAR struct qd_lowerhalf_s *lower = upper->lower;

      /* Yes.. perform one time hardware initialization. */

      DEBUGASSERT(lower->ops->setup != NULL);
      sninfo("calling setup\n");

      ret = lower->ops->setup(lower);
      if (ret < 0)
        {
          goto errout_with_sem;
        }
    }

  /* Save the new open count on success */

  upper->crefs = tmp;
  ret = OK;

errout_with_sem:
  sem_post(&upper->exclsem);

errout:
  return ret;
}

/************************************************************************************
 * Name: qd_close
 *
 * Description:
 *   This function is called when the Quadrature decoder device is closed.
 *
 ************************************************************************************/

static int qd_close(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct qd_upperhalf_s *upper = inode->i_private;
  int                         ret;

  sninfo("crefs: %d\n", upper->crefs);

  /* Get exclusive access to the device structures */

  ret = sem_wait(&upper->exclsem);
  if (ret < 0)
    {
      ret = -errno;
      goto errout;
    }

  /* Decrement the references to the driver.  If the reference count will
   * decrement to 0, then uninitialize the driver.
   */

  if (upper->crefs > 1)
    {
      upper->crefs--;
    }
  else
    {
      FAR struct qd_lowerhalf_s *lower = upper->lower;

      /* There are no more references to the port */

      upper->crefs = 0;

      /* Disable the Quadrature decoder device */

      DEBUGASSERT(lower->ops->shutdown != NULL);
      sninfo("calling shutdown: %d\n");

      lower->ops->shutdown(lower);
    }

  sem_post(&upper->exclsem);
  ret = OK;

errout:
  return ret;
}

/************************************************************************************
 * Name: qd_read
 *
 * Description:O
 *   A dummy read method.  This is provided only to satsify the VFS layer.
 *
 ************************************************************************************/

static ssize_t qd_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  /* Return zero -- usually meaning end-of-file */

  return 0;
}

/************************************************************************************
 * Name: qd_write
 *
 * Description:
 *   A dummy write method.  This is provided only to satsify the VFS layer.
 *
 ************************************************************************************/

static ssize_t qd_write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
{
  /* Return a failure */

  return -EPERM;
}

/************************************************************************************
 * Name: qd_ioctl
 *
 * Description:
 *   The standard ioctl method.  This is where ALL of the Quadrature  work is done.
 *
 ************************************************************************************/

static int qd_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode          *inode = filep->f_inode;
  FAR struct qd_upperhalf_s *upper = inode->i_private;
  FAR struct qd_lowerhalf_s *lower = upper->lower;
  int                        ret;

  sninfo("cmd: %d arg: %ld\n", cmd, arg);
  DEBUGASSERT(upper && lower);

  /* Get exclusive access to the device structures */

  ret = sem_wait(&upper->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Handle built-in ioctl commands */

  switch (cmd)
    {
      /* QDIOC_START - start to decoder .
       *   Argument: None
       */

      case QDIOC_START:
        {
          DEBUGASSERT(lower->ops->start != NULL);
          ret = lower->ops->start(lower);
        }
        break;

      /* QDIOC_STOP - stop the decoder and then to modify setting.
       *   Argument: None
       */

      case QDIOC_STOP:
        {
          DEBUGASSERT(lower->ops->stop != NULL);
          ret = lower->ops->stop(lower);
        }
        break;

      /* QDIOC_RESET - Reset the position to zero.
       *   Argument: None
       */

      case QDIOC_RESET:
        {
          DEBUGASSERT(lower->ops->reset != NULL);
          ret = lower->ops->reset(lower);
        }
        break;

      /* Any unrecognized IOCTL commands might be platform-specific ioctl commands */

      default:
        {
          sninfo("Forwarding unrecognized cmd: %d arg: %ld\n", cmd, arg);
          DEBUGASSERT(lower->ops->ioctl != NULL);
          ret = lower->ops->ioctl(lower, cmd, arg);
        }
        break;
    }

  sem_post(&upper->exclsem);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qd_register
 *
 * Description:
 *   Register the Quadrature decoder lower half device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/qd0"
 *   lower - An instance of the lower half interface
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.  The following
 *   possible error values may be returned (most are returned by
 *   register_driver()):
 *
 *   EINVAL - 'path' is invalid for this operation
 *   EEXIST - An inode already exists at 'path'
 *   ENOMEM - Failed to allocate in-memory resources for the operation
 *
 ****************************************************************************/

int qd_register(FAR const char *devpath, FAR struct qd_lowerhalf_s *lower)
{
  FAR struct qd_upperhalf_s *upper;

  /* Allocate the upper-half data structure */

  upper = (FAR struct qd_upperhalf_s *)kmm_zalloc(sizeof(struct qd_upperhalf_s));
  if (!upper)
    {
      snerr("ERROR: Allocation failed\n");
      return -ENOMEM;
    }

  /* Initialize the Quadrature decoder device structure (it was already zeroed by kmm_zalloc()) */

  sem_init(&upper->exclsem, 0, 1);
  upper->lower = lower;

  /* Register the Quadrature decoder device */

  sninfo("Registering %s\n", devpath);
  return register_driver(devpath, &g_qdops, 0666, upper);
}

#endif /* CONFIG_QDECODER */
