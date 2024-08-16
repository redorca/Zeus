/****************************************************************************
 * drivers/sensors/mc3672.c
 * Character driver for the mc3672 accelerometer.
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
#include <stddef.h>
#include <stdio.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>

#include <nuttx/fs/fs.h>
#include <nuttx/sensors/internal_mc3672.h>
#include <nuttx/sensors/mc3672.h>


#if (defined CONFIG_MC3672)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#ifndef container_of
#  define container_of(ptr, type, member) \
          ((type *)((intptr_t)(ptr) - offsetof(type, member)))
#endif

#define MC3672_RW_REG_BUFFER_MAX_LENGTH 16
#define MC3672_MAX_SAMPLE_PERIOD 500 /*maximum sample period in unit of milliseconds */
#define MC3672_MAX_FIFO_THRESHOLD 30 /*maximum sample period in unit of milliseconds */


/****************************************************************************
 * Private type
 ****************************************************************************/
struct mc3672_t
{

  FAR struct mc3672_bus_config_s bus;

  uint8_t ocount; /* The number of times the device has been opened */
  sem_t closesem; /* Locks out new opens while close is in progress */
  sem_t ioctrlsem; /* Locks out new IOCtl operation while the old one is in progress */
  sem_t delaysem;/*this semphone is initialize as 0 and never post,
                  *so we can implement millseconds delay using the
                  *sem_timedwait function.
                  */
  sem_t drdysem; /*post this semphone when a DRDY interrupt is triggerd*/

  uint8_t        signo; /* signo to use when signaling a interrupt */
  pid_t          receive_pid; /* The task to be signalled */

  const struct mc3672_low_level_operations_s *ll_op; /*gpio related operation*/
  FAR struct mc3672_dev internal;/* used by the internal_mc3672.c */

};



/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int mc3672_open(FAR struct file *filep);
static int mc3672_close(FAR struct file *filep);
static ssize_t mc3672_read(FAR struct file *, FAR char *, size_t);
static ssize_t mc3672_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int mc3672_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_mc3672_fops =
{
  mc3672_open,
  mc3672_close,
  mc3672_read,
  mc3672_write,
  NULL,
  mc3672_ioctl
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

/*timeout wait*/
static __inline int mc3672_sem_timeout_wait(FAR sem_t *sem, uint32_t timeout)
{
  struct timespec time;
  int ret;
  int errcode;

  do
    {
      clock_gettime(CLOCK_REALTIME, &time);

      time.tv_nsec += (timeout % MSEC_PER_SEC) * NSEC_PER_MSEC;
      time.tv_sec += timeout / MSEC_PER_SEC;

      if (time.tv_nsec >= NSEC_PER_SEC)
        {
          time.tv_sec ++;
          time.tv_nsec -= NSEC_PER_SEC;
        }

      ret = sem_timedwait(sem, &time);

      if (ret != OK)
        {
          errcode = errno;
        }
      else
        {
          errcode = OK;
        }

      DEBUGASSERT(errcode != EINVAL);
    }
  while (errcode == EINTR);

  return -errcode;
}



/****************************************************************************
 * Name: mc3672_i2c_read_reg
 ****************************************************************************/
static int8_t mc3672_i2c_read_reg(const struct mc3672_dev *priv, uint8_t reg_addr,
                                  uint8_t *data, uint16_t len)
{
  FAR struct mc3672_t *mc3672;
  int ret;
  int8_t rslt;

  DEBUGASSERT(priv);
  mc3672 = container_of(priv, struct mc3672_t, internal);


  ret = i2c_writeread(mc3672->bus.bus_config.i2c_config.i2c,
                      &mc3672->bus.bus_config.i2c_config.config,
                      &reg_addr, 1,
                      data, len);

  rslt = (ret == OK) ? MC3672_RETCODE_SUCCESS : MC3672_RETCODE_ERROR_BUS;

  return rslt;
}

/****************************************************************************
 * Name: mc3672_i2c_write_reg
 ****************************************************************************/
static int8_t mc3672_i2c_write_reg(const struct mc3672_dev *priv, uint8_t reg_addr,
                                   uint8_t *data, uint16_t len)
{
  FAR struct mc3672_t *mc3672;
  uint8_t buffer[MC3672_RW_REG_BUFFER_MAX_LENGTH];
  int ret;
  int8_t rslt;


  DEBUGASSERT(priv);
  mc3672 = container_of(priv, struct mc3672_t, internal);

  if (len >  (MC3672_RW_REG_BUFFER_MAX_LENGTH - 1))
    {
      len = MC3672_RW_REG_BUFFER_MAX_LENGTH - 1;
    }

  buffer[0] = reg_addr;
  memcpy(&buffer[1], data, len);

  ret = i2c_write(mc3672->bus.bus_config.i2c_config.i2c,
                  &mc3672->bus.bus_config.i2c_config.config,
                  buffer, (len + 1));

  rslt = (ret == OK) ? MC3672_RETCODE_SUCCESS : MC3672_RETCODE_ERROR_BUS;

  return rslt;


}

/****************************************************************************
 * Name: mc3672_spi_read_reg
 ****************************************************************************/
static int8_t mc3672_spi_read_reg(const struct mc3672_dev *priv, uint8_t reg_addr,
                                  uint8_t *data, uint16_t len)
{

  FAR struct mc3672_t *mc3672;
  struct mc3672_spi_bus_config_s *cfg;


  DEBUGASSERT(priv);
  mc3672 = container_of(priv, struct mc3672_t, internal);

  cfg = &mc3672->bus.bus_config.spi_config;

  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(cfg->spi, true);

  /*Set frequency*/
  SPI_SETFREQUENCY(cfg->spi, cfg->spi_frequency);

  /*Set mode*/
  SPI_SETMODE(cfg->spi, cfg->spi_mode);

  /* Set CS to low which selects the BMG160 */

  SPI_SELECT(cfg->spi, cfg->spi_devid, true);

  /* Transmit the register address from where we want to read - the MSB needs
   * to be set to indicate the read indication.
   */
  SPI_SEND(cfg->spi, reg_addr);

  /* Write idle bytes while receiving the required data */
  if (len > 1)
    {
      SPI_EXCHANGE(cfg->spi, NULL, data, len);
    }
  else
    {
      *data = SPI_SEND(cfg->spi, 0);
    }

  /* Set CS to high which deselects the BMG160 */

  SPI_SELECT(cfg->spi, cfg->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(cfg->spi, false);

  return MC3672_RETCODE_SUCCESS;

}

/****************************************************************************
 * Name: mc3672_spi_write_reg
 ****************************************************************************/
static int8_t mc3672_spi_write_reg(const struct mc3672_dev *priv, uint8_t reg_addr,
                                   uint8_t *data, uint16_t len)
{
  FAR struct mc3672_t *mc3672;
  struct mc3672_spi_bus_config_s *cfg;


  DEBUGASSERT(priv);
  mc3672 = container_of(priv, struct mc3672_t, internal);

  cfg = &mc3672->bus.bus_config.spi_config;

  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(cfg->spi, true);

  /*Set frequency*/
  SPI_SETFREQUENCY(cfg->spi, cfg->spi_frequency);

  /*Set mode*/
  SPI_SETMODE(cfg->spi, cfg->spi_mode);

  /* Set CS to low which selects the BMG160 */

  SPI_SELECT(cfg->spi, cfg->spi_devid, true);

  /* Transmit the register address from where we want to read */

  SPI_SEND(cfg->spi, reg_addr);

  /* Transmit the content which should be written in the register */

  if (len > 1)
    {
      SPI_EXCHANGE(cfg->spi, data, NULL, len);
    }
  else
    {
      SPI_SEND(cfg->spi, *data);
    }

  /* Set CS to high which deselects the BMG160 */

  SPI_SELECT(cfg->spi, cfg->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(cfg->spi, false);

  return MC3672_RETCODE_SUCCESS;
}

/****************************************************************************
 * Name: mc3672_delay
 ****************************************************************************/

void mc3672_delay_msec(const struct mc3672_dev *priv, uint32_t period)
{
  FAR struct mc3672_t *mc3672;

  DEBUGASSERT(priv);
  mc3672 = container_of(priv, struct mc3672_t, internal);

  if (period <= 5)
    {
      /*this delay function is implemented by a count loop,
      it will block the thread with lower priority, so we just uses it to delay 1 millsecond.*/

      up_mdelay(period);
    }
  else
    {

      /*the g_bmi160->delaysem is initialized as 0 and never post, so
       *the current thread will be suspended and wait for the specified time.
       *By this way, we can implement millseconds delay without blocking other threads.*/

      mc3672_sem_timeout_wait(&mc3672->delaysem, period);
    }
}

/****************************************************************************
 * Name: mc3672_INT_pin_handler
 ****************************************************************************/
void mc3672_INT_pin_handler(void *param)
{
  FAR struct mc3672_t *mc3672 = (struct mc3672_t *)param;
  int sem_value;

  sem_getvalue(&mc3672->drdysem, &sem_value);

  if (sem_value <= 0)
    {
      sem_post(&mc3672->drdysem);
    }

}



/****************************************************************************
 * Name: mc3672_open
 ****************************************************************************/

static int mc3672_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct mc3672_t *priv;
  uint8_t tmp;
  int8_t rslt;
  int ret = OK;


  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private != NULL);
  priv = inode->i_private;

  /* If the port is the middle of closing, wait until the close is finished */

  if (sem_wait(&priv->closesem) != OK)
    {
      ret = -errno;
    }
  else
    {
      /* Increment the count of references to the device.  If this the first
       * time that the driver has been opened for this device, then initialize
       * the device.
       */

      tmp = priv->ocount + 1;
      if (tmp == 0)
        {
          /* More than 255 opens; uint8_t overflows to zero */

          ret = -EMFILE;
        }
      else
        {
          /* Check if this is the first time that the driver has been opened. */

          if (tmp == 1)
            {
              /* Yes.. perform one time hardware reset. */
              rslt = mc3672_init(&priv->internal);
              if (rslt == MC3672_RETCODE_SUCCESS)
                {
                  /* Save the new open count on success */

                  priv->ocount = tmp;
                }
              else
                {
                  snerr("ERROR: mc3672_init failed: %d\n", ret);
                  ret = -EIO;
                }
            }
        }

      sem_post(&priv->closesem);
    }

  return ret;
}

/****************************************************************************
 * Name: mc3672_close
 ****************************************************************************/

static int mc3672_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct mc3672_t *dev;
  int                   ret = OK;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private != NULL);
  dev = inode->i_private;

  DEBUGASSERT(dev != NULL);

  /* Perform a reset */
  if (sem_wait(&dev->closesem) != OK)
    {
      ret = -errno;
    }
  else
    {
      /* Decrement the references to the driver.  If the reference count will
       * decrement to 0, then uninitialize the driver.
       */

      if (dev->ocount > 1)
        {
          dev->ocount--;
        }
      else
        {
          /* There are no more references to the port */

          dev->ocount = 0;

          /* disable the device */

          mc3672_reset(&dev->internal);
        }
      sem_post(&dev->closesem);
    }

  return ret;
}

/****************************************************************************
 * Name: mc3672_read
 ****************************************************************************/

static ssize_t mc3672_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode *inode;
  FAR struct mc3672_t *mc3672;
  FAR struct mc3672_dev *priv;
  FAR char *buffer_ptr;
  FAR int sum;
  FAR int count_left;
  FAR uint8_t count_once;
  FAR uint8_t got_count;
  FAR int ret = OK;
  FAR int8_t rslt;

#ifdef CONFIG_MC3672_USE_INT
  FAR uint8_t threshold;
  struct mc3672_fifo_settings fifo_setting;
  uint8_t int_events;
#endif

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private != NULL);
  mc3672 = inode->i_private;
  priv = &mc3672->internal;

  /*get sample amount from the buffer size*/
  sum = buflen / (sizeof(struct mc3672_acc_t)) ;

  if (sum == 0)
    {
      return 0;
    }

#ifdef CONFIG_MC3672_USE_INT

  /*config fifo threshold*/
  threshold = (sum < MC3672_MAX_FIFO_THRESHOLD) ? sum : MC3672_MAX_FIFO_THRESHOLD;

  fifo_setting.fifo_stream_en = MC3672_FEATURE_ENABLE;
  fifo_setting.fifo_mode = MC3672_FIFO_MODE_NORMAL;
  fifo_setting.fifo_threshold = threshold;
  rslt = mc3672_cfg_fifo(priv, &fifo_setting);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return ERROR;
    }

  /*clear pending interrupt*/
  rslt = mc3672_get_int_status(priv, &int_events);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return ERROR;
    }

  /*enable DRADY pin interrupt*/

  if (mc3672->ll_op->mc3672_INT_enable != NULL)
    {
      mc3672->ll_op->mc3672_INT_enable(true);
    }

  /*init variable*/
  buffer_ptr = buffer;
  count_left = sum;

  /*got data*/
  while (count_left)
    {
      ret = mc3672_sem_timeout_wait(&mc3672->drdysem, (2 * MC3672_MAX_SAMPLE_PERIOD));

      if (ret == OK)
        {
          /*got interrupt status*/
          rslt = mc3672_get_int_status(priv, &int_events);

          if (rslt != MC3672_RETCODE_SUCCESS)
            {
              ret = ERROR;
              break;
            }

          if ((int_events & mc3672_int_event_fifo_full) || (int_events & mc3672_int_event_fifo_thresh))
            {

              count_once = (count_left >= MC3672_MAX_FIFO_THRESHOLD) ? MC3672_MAX_FIFO_THRESHOLD : count_left;
              /*fetch data from sensor*/
              rslt = mc3672_read_fifo(priv, (struct mc3672_acc_t *)buffer_ptr, count_once, &got_count);
              if (rslt == MC3672_RETCODE_SUCCESS)
                {
                  /*fill in the buffer*/
                  buffer_ptr += (sizeof(struct mc3672_acc_t) * got_count);
                  count_left -= got_count;
                }
              else
                {
                  ret = ERROR;
                  break;
                }
            }
          else
            {
              continue;
            }
        }
      else /*if timeout or deadlock*/
        {
          break;
        }
    }

  /*disable DRADY pin interrupt*/

  if (mc3672->ll_op->mc3672_INT_enable != NULL)
    {
      mc3672->ll_op->mc3672_INT_enable(false);
    }

  /*disable fifo*/
  mc3672_enable_fifo(priv, false);

#else
  /*init variable*/
  buffer_ptr = buffer;
  count_left = sum;

  count_once = (count_left >= MC3672_MAX_FIFO_THRESHOLD) ? MC3672_MAX_FIFO_THRESHOLD : count_left;
  /*fetch data from sensor*/
  rslt = mc3672_read_fifo(priv, (struct mc3672_acc_t *)buffer_ptr, count_once, &got_count);
  if (rslt == MC3672_RETCODE_SUCCESS)
    {
      /*fill in the buffer*/
      buffer_ptr += (sizeof(struct mc3672_acc_t) * got_count);
      count_left -= got_count;
    }
  else
    {
      ret = ERROR;
    }
#endif


  /*If we got something*/
  if (ret == OK)
    {
      ret = (sum - count_left) * sizeof(struct mc3672_acc_t);
    }

  return ret;

}

/****************************************************************************
 * Name: bmg160_write
 ****************************************************************************/

static ssize_t mc3672_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: bmg160_ioctl
 ****************************************************************************/

static int mc3672_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  struct mc3672_t *dev;
  struct mc3672_dev *priv;
  int ret = OK;
  int8_t rslt;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private != NULL);
  dev = inode->i_private;
  priv = &dev->internal;

  /*IOCtrl operation should not be use by two or more thread at the same time*/
  if (sem_wait(&dev->ioctrlsem) != OK)
    {
      ret = -errno;
    }
  else
    {
      switch (cmd)
        {

          case SNIOC_A_SPARAM:
            {
              sn_ga_param_s *param = (sn_ga_param_s *)arg;
              struct mc3672_cwake_settings cwake_settings;
              ret = OK;

              sninfo("Set: range:%d, resolution:%d, Samplerate:%d, mode:%d\n",
                     param->range, param->resolution, param->odr, param->power_mode);
              switch (param->range)
                {
                  case 2:
                    cwake_settings.range = MC3672_RANGE_2G;
                    break;
                  case 4:
                    cwake_settings.range = MC3672_RANGE_4G;
                    break;
                  case 8:
                    cwake_settings.range = MC3672_RANGE_8G;
                    break;
                  case 12:
                    cwake_settings.range = MC3672_RANGE_12G;
                    break;
                  case 16:
                    cwake_settings.range = MC3672_RANGE_16G;
                    break;
                  default:
                    ret = -EINVAL;
                    break;

                }

              switch (param->resolution)
                {
                  case 6:
                    cwake_settings.resolution = MC3672_RESOLUTION_6BIT;
                    break;
                  case 8:
                    cwake_settings.resolution = MC3672_RESOLUTION_8BIT;
                    break;
                  case 10:
                    cwake_settings.resolution = MC3672_RESOLUTION_10BIT;
                    break;
                  case 12:
                    cwake_settings.resolution = MC3672_RESOLUTION_12BIT;
                    break;
                  case 14:
                    cwake_settings.resolution = MC3672_RESOLUTION_14BIT;
                    break;
                  default:
                    ret = -EINVAL;
                    break;

                }

              switch (param->odr)
                {
                  case 140:
                    cwake_settings.wake_sr = MC3672_CWAKE_SR_LPM_14Hz;
                    break;
                  case 280:
                    cwake_settings.wake_sr = MC3672_CWAKE_SR_LPM_28Hz;
                    break;
                  case 540:
                    cwake_settings.wake_sr =  MC3672_CWAKE_SR_LPM_54Hz;
                    break;
                  case 1050:
                    cwake_settings.wake_sr = MC3672_CWAKE_SR_LPM_105Hz;
                    break;
                  case 2100:
                    cwake_settings.wake_sr = MC3672_CWAKE_SR_LPM_210Hz;
                    break;
                  case 4000:
                    cwake_settings.wake_sr = MC3672_CWAKE_SR_LPM_400Hz;
                    break;
                  case 6000:
                    cwake_settings.wake_sr = MC3672_CWAKE_SR_LPM_600Hz;
                    break;
                  default:
                    ret = -EINVAL;
                    break;

                }

              if (ret == OK)
                {
                  cwake_settings.wake_pm = MC3672_POWER_MODE_LOW;
                  rslt = mc3672_cfg_cwake(priv, &cwake_settings);

                  if (rslt != MC3672_RETCODE_SUCCESS)
                    {
                      ret = -EIO;
                    }
                }
            }
            break;

          case SNIOC_START:
            {
              struct mc3672_int_settings int_setting;

              int_setting.INTN_PIN_IPP = MC3672_INTR_C_IPP_MODE_OPEN_DRAIN;
              int_setting.INTN_PIN_IAH = MC3672_INTR_C_IAH_ACTIVE_LOW;
              int_setting.wake_int_en = MC3672_FEATURE_DISABLE;
              int_setting.acq_int_en = MC3672_FEATURE_DISABLE;
              int_setting.fifo_empty_int_en = MC3672_FEATURE_DISABLE;
              int_setting.fifo_full_int_en = MC3672_FEATURE_ENABLE;
              int_setting.fifo_thresh_int_en = MC3672_FEATURE_ENABLE;
              int_setting.swake_int_en = MC3672_FEATURE_DISABLE;
              rslt = mc3672_cfg_int(priv, &int_setting);

#ifndef CONFIG_MC3672_USE_INT
              struct mc3672_fifo_settings fifo_setting;

              fifo_setting.fifo_stream_en = MC3672_FEATURE_ENABLE;
              fifo_setting.fifo_mode = MC3672_FIFO_MODE_NORMAL;
              fifo_setting.fifo_threshold = MC3672_MAX_FIFO_THRESHOLD;
              rslt += mc3672_cfg_fifo(priv, &fifo_setting);

#endif

              rslt += mc3672_set_mode(priv, MC3672_MODE_CWAKE);


              if (rslt != MC3672_RETCODE_SUCCESS)
                {
                  ret = -EIO;
                }
            }
            break;

          case SNIOC_STOP:
            {
              /* Setting the power mode as STANDBY */
              rslt = mc3672_set_mode(priv, MC3672_MODE_STANDBY);

#ifndef CONFIG_MC3672_USE_INT
              /*stop fifo*/
              rslt += mc3672_enable_fifo(priv, false);
#endif

              if (rslt != MC3672_RETCODE_SUCCESS)
                {
                  ret = ERROR;
                }
            }
            break;


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

          case SNIOC_INT_STATUS:
            {
              uint8_t *status = (uint8_t *)((uintptr_t)arg);

              /* got interrupt status */
              rslt = mc3672_get_int_status(priv, status);

              if (rslt != MC3672_RETCODE_SUCCESS)
                {
                  ret = ERROR;
                }

            }
            break;

          /* Command was not recognized */

          default:
            snerr("ERROR: Unrecognized cmd: %d\n", cmd);
            ret = -ENOTTY;
            break;
        }

      sem_post(&dev->ioctrlsem);
    }
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: mc3672_register
 *
 * Description:
 *   Register the MC3672 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath -    The full path to the driver to register. E.g., "/dev/mc3672"
 *   bus_config - An struct describing the bus which use to communicate with MC3672
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/
int mc3672_register(FAR const char *devpath,
                    FAR struct mc3672_bus_config_s *bus_config,
                    const struct mc3672_low_level_operations_s *ll_op)
{
  int ret;
  struct mc3672_t *mc3672;

  /* Sanity check */

  DEBUGASSERT(bus_config != NULL);


  /* Initialize the BMG160 device structure */

  mc3672 = (FAR struct mc3672_t *)kmm_malloc(sizeof(struct mc3672_t));
  if (mc3672 == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  mc3672->ocount = 0;
  /* Initialize device open semphone*/
  sem_init(&mc3672->closesem, 0, 1);
  /* Initialize millsecond delay semphone */
  sem_init(&mc3672->delaysem, 0, 0);
  /* Initialize ioctrl semphone */
  sem_init(&mc3672->ioctrlsem, 0, 1);
  /* Initialize data ready semphone */
  sem_init(&mc3672->drdysem, 0, 0);


  mc3672->receive_pid = 0;
  mc3672->signo = 0;

  mc3672->ll_op = ll_op;

  memcpy(&mc3672->bus, bus_config, sizeof(struct mc3672_bus_config_s));

  if (mc3672->bus.bus_selection == MC3672_USE_SPI)
    {
      mc3672->internal.dev_id = 0;
      mc3672->internal.intf = MC3672_SPI_INTF;
      mc3672->internal.read = mc3672_spi_read_reg;
      mc3672->internal.write = mc3672_spi_write_reg;
      mc3672->internal.delay_ms = mc3672_delay_msec;
    }
  else
    {
      mc3672->internal.dev_id = 0;/*this doesn't matter, because we save the i2c address in bus_config_s*/
      mc3672->internal.intf = MC3672_I2C_INTF;
      mc3672->internal.read = mc3672_i2c_read_reg;
      mc3672->internal.write = mc3672_i2c_write_reg;
      mc3672->internal.delay_ms = mc3672_delay_msec;
    }

  /*config INT pin*/

  if (mc3672->ll_op && mc3672->ll_op->mc3672_INT_cfg_interrupt)
    {
      mc3672->ll_op->mc3672_INT_cfg_interrupt(mc3672_INT_pin_handler, mc3672);
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_mc3672_fops, 0666, mc3672);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      sem_destroy(&mc3672->closesem);
      sem_destroy(&mc3672->delaysem);
      sem_destroy(&mc3672->ioctrlsem);
      kmm_free(mc3672);
      return ret;
    }

  return OK;

}

#endif /* CONFIG_MC3672*/


