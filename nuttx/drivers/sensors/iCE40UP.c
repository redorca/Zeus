/****************************************************************************
 * drivers/sensors/iCE40UP.c
 * Character driver for the iCE40 Ultra Plus family AI chips.
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

#include <errno.h>
#include <debug.h>
#include <string.h>
#include <semaphore.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/wdog.h>

#include <nuttx/sensors/ioctl.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/fs/fs.h>
#include <nuttx/sensors/iCE40UP.h>

#ifdef CONFIG_ICE40UP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/


/****************************************************************************
 * Private type
 ****************************************************************************/
struct iCE40UP_dev_s
{

  sem_t          devsem; /* Locks the device, this device can only be used by one thread */
  uint8_t        signo; /* signo to use when signaling a interrupt */
  pid_t          receive_pid; /* The task to be signalled */
  bool           active;/*true if the device is active*/

  const struct iCE40UP_low_level_operations_s *low_level_op;
};



/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int iCE40UP_open(FAR struct file *filep);
static int iCE40UP_close(FAR struct file *filep);
static ssize_t iCE40UP_read(FAR struct file *, FAR char *, size_t);
static ssize_t iCE40UP_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int iCE40UP_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_iCE40UP_fops =
{
  iCE40UP_open,
  iCE40UP_close,
  iCE40UP_read,
  iCE40UP_write,
  NULL,
  iCE40UP_ioctl
#ifndef CONFIG_DISABLE_POLL
  , NULL
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL
#endif
};


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iCE40UP_int_handler
 ****************************************************************************/
static void iCE40UP_int_handler(void *arg, bool status)
{
  union sigval value;
  struct iCE40UP_dev_s *dev = (struct iCE40UP_dev_s *)arg;

  /*check status*/
  if (dev->active != status)
    {
      if (dev->receive_pid != 0)
        {
          /*send signal*/

          value.sival_int = status;
          (void)sigqueue(dev->receive_pid, dev->signo, value);
        }

      dev->active = status;
    }

  /*else we just ignore this interrupt*/

}




/****************************************************************************
 * Name: iCE40UP_open
 ****************************************************************************/

static int iCE40UP_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct iCE40UP_dev_s *priv;
  int ret = OK;


  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private != NULL);
  priv = inode->i_private;

  /* If the port is the middle of closing, wait until the close is finished */

  if (sem_wait(&priv->devsem) != OK)
    {
      ret = -errno;
    }
  else
    {
      priv->receive_pid   = 0;
      priv->signo = 0;
      priv->active = false;
    }

  return ret;
}

/****************************************************************************
 * Name: iCE40UP_close
 ****************************************************************************/

static int iCE40UP_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct iCE40UP_dev_s *priv;
  int                   ret = OK;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private != NULL);
  priv = inode->i_private;

  /*disable the interrupt pin*/

  priv->low_level_op->iCE40UP_int_enable(false);

  /* disable the device */

  priv->receive_pid   = 0;
  priv->signo = 0;
  priv->active = false;

  /* release this device*/
  sem_post(&priv->devsem);

  return ret;
}

/****************************************************************************
 * Name: bmg160_read
 ****************************************************************************/

static ssize_t iCE40UP_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  return 0;
}

/****************************************************************************
 * Name: bmg160_write
 ****************************************************************************/

static ssize_t iCE40UP_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: bmg160_ioctl
 ****************************************************************************/

static int iCE40UP_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  struct iCE40UP_dev_s *dev;
  int ret = OK;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private != NULL);
  dev = inode->i_private;


  switch (cmd)
    {

      /* Command:     SNIOC_GA_REGISTER_INT
       * Description: Register to receive a signal whenever an interrupt
       *              is received.
       * Argument:    The number of signal to be generated when the interrupt
       *              occurs.*/

      case SNIOC_GA_REGISTER_INT:
        {
          DEBUGASSERT(GOOD_SIGNO(arg));

          dev->receive_pid   = getpid();
          dev->signo = (uint8_t)arg;
        }
        break;

      /* Command:     SNIOC_GA_UNREGISTER_INT
       * Description: Stop receiving signals.
       * Argument:    None.*/

      case SNIOC_GA_UNREGISTER_INT:
        {
          dev->receive_pid   = 0;
          dev->signo = 0;
        }
        break;

      case SNIOC_START:
        {
          dev->active = false;

          /*enable the interrupt pin*/

          dev->low_level_op->iCE40UP_int_enable(true);
        }
        break;

      case SNIOC_STOP:
        {
          dev->active = false;

          /*disable the interrupt pin*/

          dev->low_level_op->iCE40UP_int_enable(false);
        }
        break;


      /* Command was not recognized */

      default:
        snerr("ERROR: Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iCE40UP_register
 *
 * Description:
 *   Register the ICE40UP character device as 'devpath'
 *
 * Input Parameters:
 *   devpath -    The full path to the driver to register. E.g., "/dev/gyr0"
 *   bus_config - An struct describing the bus which use to communicate with ICE40UP
 *   attach_irq - function used to initialize the interrput pin and attach
 *                interrput handler.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/
int iCE40UP_register(FAR const char *devpath,
                     FAR const struct iCE40UP_low_level_operations_s *ll_operation)
{
  int ret;
  struct iCE40UP_dev_s *dev;


  /* Sanity check */

  DEBUGASSERT(ll_operation != NULL);
  DEBUGASSERT(ll_operation->iCE40UP_attach != NULL);
  DEBUGASSERT(ll_operation->iCE40UP_int_enable != NULL);

  /* Initialize the iCE40UP device structure */

  dev = (FAR struct iCE40UP_dev_s *)kmm_malloc(sizeof(struct iCE40UP_dev_s));
  if (dev == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  /* Initialize device semphone*/
  sem_init(&dev->devsem, 0, 1);


  dev->receive_pid = 0;
  dev->signo = 0;
  dev->active = false;
  dev->low_level_op = ll_operation;

  ret = dev->low_level_op->iCE40UP_attach(iCE40UP_int_handler, (void *)dev);
  if (ret < 0)
    {
      snerr("ERROR: Failed to attach interrupt\n");
      goto err_dev;
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_iCE40UP_fops, 0666, dev);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      goto err_dev;
    }

  return OK;

err_dev:
  sem_destroy(&dev->devsem);
  kmm_free(dev);
  return ret;

}

#endif /* CONFIG_SPI && CONFIG_BMG160 */
