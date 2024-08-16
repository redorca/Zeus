/****************************************************************************
 * drivers/sensors/bmm150.c
 * Character driver for the bmm150 magnetometer.
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
#include <nuttx/sensors/internal_bmm150.h>
#include <nuttx/sensors/bmm150.h>


#if (defined CONFIG_BMM150)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#ifndef container_of
#  define container_of(ptr, type, member) \
          ((type *)((intptr_t)(ptr) - offsetof(type, member)))
#endif

#define BMM150_RW_REG_BUFFER_MAX_LENGTH 16
#define BMM150_MAX_SAMPLE_PERIOD 500 /*maximum sample period in unit of milliseconds */

/****************************************************************************
 * Private type
 ****************************************************************************/
struct bmm150_t
{

  FAR struct bmm150_bus_config_s bus;

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

  const struct bmm150_low_level_operations_s *ll_op; /*gpio related operation*/
  FAR struct bmm150_dev internal;/* used by the internal_bmm150.c */

};



/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int bmm150_open(FAR struct file *filep);
static int bmm150_close(FAR struct file *filep);
static ssize_t bmm150_read(FAR struct file *, FAR char *, size_t);
static ssize_t bmm150_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int bmm150_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_bmm150_fops =
{
  bmm150_open,
  bmm150_close,
  bmm150_read,
  bmm150_write,
  NULL,
  bmm150_ioctl
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
 * Name: bmm150_i2c_read_reg
 ****************************************************************************/
static int8_t bmm150_i2c_read_reg(const struct bmm150_dev *priv, uint8_t reg_addr,
                                  uint8_t *data, uint16_t len)
{
  FAR struct bmm150_t *bmm150;
  int ret;
  int8_t rslt;

  DEBUGASSERT(priv);
  bmm150 = container_of(priv, struct bmm150_t, internal);


  ret = i2c_writeread(bmm150->bus.bus_config.i2c_config.i2c,
                      &bmm150->bus.bus_config.i2c_config.config,
                      &reg_addr, 1,
                      data, len);

  rslt = (ret == OK) ? BMM150_OK : BMM150_E_DEV_NOT_FOUND;

  return rslt;
}

/****************************************************************************
 * Name: bmm150_i2c_write_reg
 ****************************************************************************/
static int8_t bmm150_i2c_write_reg(const struct bmm150_dev *priv, uint8_t reg_addr,
                                   uint8_t *data, uint16_t len)
{
  FAR struct bmm150_t *bmm150;
  uint8_t buffer[BMM150_RW_REG_BUFFER_MAX_LENGTH];
  int ret;
  int8_t rslt;


  DEBUGASSERT(priv);
  bmm150 = container_of(priv, struct bmm150_t, internal);

  if (len >  (BMM150_RW_REG_BUFFER_MAX_LENGTH - 1))
    {
      len = BMM150_RW_REG_BUFFER_MAX_LENGTH - 1;
    }

  buffer[0] = reg_addr;
  memcpy(&buffer[1], data, len);

  ret = i2c_write(bmm150->bus.bus_config.i2c_config.i2c,
                  &bmm150->bus.bus_config.i2c_config.config,
                  buffer, (len + 1));

  rslt = (ret == OK) ? BMM150_OK : BMM150_E_DEV_NOT_FOUND;

  return rslt;


}

/****************************************************************************
 * Name: bmm150_spi_read_reg
 ****************************************************************************/
static int8_t bmm150_spi_read_reg(const struct bmm150_dev *priv, uint8_t reg_addr,
                                  uint8_t *data, uint16_t len)
{

  FAR struct bmm150_t *bmm150;
  struct bmm150_spi_bus_config_s *cfg;


  DEBUGASSERT(priv);
  bmm150 = container_of(priv, struct bmm150_t, internal);

  cfg = &bmm150->bus.bus_config.spi_config;

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

  return BMM150_OK;

}

/****************************************************************************
 * Name: bmm150_spi_write_reg
 ****************************************************************************/
static int8_t bmm150_spi_write_reg(const struct bmm150_dev *priv, uint8_t reg_addr,
                                   uint8_t *data, uint16_t len)
{
  FAR struct bmm150_t *bmm150;
  struct bmm150_spi_bus_config_s *cfg;


  DEBUGASSERT(priv);
  bmm150 = container_of(priv, struct bmm150_t, internal);

  cfg = &bmm150->bus.bus_config.spi_config;

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

  return BMM150_OK;
}

/****************************************************************************
 * Name: bmm150_delay
 ****************************************************************************/

void bmm150_delay_msec(const struct bmm150_dev *priv, uint32_t period)
{
  FAR struct bmm150_t *bmm150;
  struct timespec time;

  DEBUGASSERT(priv);
  bmm150 = container_of(priv, struct bmm150_t, internal);

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

      clock_gettime(CLOCK_REALTIME, &time);

      time.tv_nsec += (period % MSEC_PER_SEC) * NSEC_PER_MSEC;
      time.tv_sec += period / MSEC_PER_SEC;

      if (time.tv_nsec >= NSEC_PER_SEC)
        {
          time.tv_sec ++;
          time.tv_nsec -= NSEC_PER_SEC;
        }

      sem_timedwait(&bmm150->delaysem, &time);
    }
}

/****************************************************************************
 * Name: bmm150_INT_pin_handler
 ****************************************************************************/
void bmm150_INT_pin_handler(void *param)
{
  FAR struct bmm150_t *bmm150 = (struct bmm150_t *)param;

  if (bmm150->receive_pid != 0)
    {
      kill(bmm150->receive_pid, bmm150->signo);
    }
}

/****************************************************************************
 * Name: bmm150_INT_pin_handler
 ****************************************************************************/
void bmm150_DRDY_pin_handler(void *param)
{
  FAR struct bmm150_t *bmm150 = (struct bmm150_t *)param;
  int sem_value;

  sem_getvalue(&bmm150->drdysem, &sem_value);

  if (sem_value <= 0)
    {
      sem_post(&bmm150->drdysem);
    }

}

static __inline int bmm150_sem_timeout_wait(FAR sem_t *sem, uint32_t timeout)
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
 * Name: bmm150_open
 ****************************************************************************/

static int bmm150_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct bmm150_t *priv;
  uint8_t tmp;
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
              ret = bmm150_init(&priv->internal);
              if (ret == OK)
                {
                  /* Save the new open count on success */

                  priv->ocount = tmp;
                }
              else
                {
                  snerr("ERROR: bmm150_init failed: %d\n", ret);
                  ret = ERROR;
                }
            }
        }

      sem_post(&priv->closesem);
    }

  return ret;
}

/****************************************************************************
 * Name: bmm150_close
 ****************************************************************************/

static int bmm150_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct bmm150_t *dev;
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

          bmm150_soft_reset(&dev->internal);
        }
      sem_post(&dev->closesem);
    }

  return ret;
}

/****************************************************************************
 * Name: bmm150_read
 ****************************************************************************/

static ssize_t bmm150_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode *inode;
  FAR struct bmm150_t *bmm150;
  FAR struct bmm150_dev *priv;
  FAR int sum;
  FAR int count_sample;
  FAR int ret = -1;
  FAR int8_t rslt;
  FAR char *buffer_ptr;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private != NULL);
  bmm150 = inode->i_private;
  priv = &bmm150->internal;

  /*get sample amount from the buffer size*/
  sum = buflen / (sizeof(struct bmm150_mag_data)) ;

  if (sum == 0)
    {
      return 0;
    }

#ifdef CONFIG_BMM150_USE_DRDY

  /*enable DRADY pin interrupt*/

  if (bmm150->ll_op->bmm150_DRDY_enable != NULL)
    {
      bmm150->ll_op->bmm150_DRDY_enable(true);
    }

  /*init variable*/
  buffer_ptr = buffer;
  count_sample = 0;

  /*got data*/
  while (count_sample < sum)
    {
      ret = bmm150_sem_timeout_wait(&bmm150->drdysem, (2 * BMM150_MAX_SAMPLE_PERIOD));

      if (ret == OK)
        {
          /*fetch data from sensor*/
          rslt = bmm150_read_mag_data(priv);
          if (rslt == BMM150_OK)
            {
              /*fill in the buffer*/
              memcpy(buffer_ptr, &priv->data, sizeof(struct bmm150_mag_data));
              buffer_ptr += sizeof(struct bmm150_mag_data);
              count_sample ++;
            }
          else
            {
              ret = ERROR;
              break;
            }
        }
      else /*if timeout or deadlock*/
        {
          break;
        }
    }

  /*disable DRADY pin interrupt*/

  if (bmm150->ll_op->bmm150_DRDY_enable != NULL)
    {
      bmm150->ll_op->bmm150_DRDY_enable(false);
    }

#else

  /*init variable*/
  buffer_ptr = buffer;
  count_sample = 0;

  /*fetch data from sensor*/
  rslt = bmm150_read_mag_data(priv);
  if (rslt == BMM150_OK)
    {
      /*fill in the buffer*/
      memcpy(buffer_ptr, &priv->data, sizeof(struct bmm150_mag_data));
      buffer_ptr += sizeof(struct bmm150_mag_data);
      count_sample ++;
    }
  else
    {
      ret = ERROR;
    }


#endif

  if (count_sample)
    {
      ret = count_sample * sizeof(struct bmm150_mag_data);
    }

  return ret;

}

/****************************************************************************
 * Name: bmg160_write
 ****************************************************************************/

static ssize_t bmm150_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: bmg160_ioctl
 ****************************************************************************/

static int bmm150_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  struct bmm150_t *dev;
  struct bmm150_dev *priv;
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

          /* Command:     SNIOC_READID
           * Description: get sensor ID
           * Argument:    pointer to void type*/
          case SNIOC_READID:
            {
              FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
              DEBUGASSERT(ptr != NULL);

              /*we have read this in the bmm150_init function*/
              *ptr = priv->chip_id;
            }

            break;

          case SNIOC_START:
            {
              uint16_t desired_settings;

              /* Setting the power mode as normal */
              priv->settings.pwr_mode = BMM150_NORMAL_MODE;
              rslt = bmm150_set_op_mode(priv);

              /* Setting the preset mode as Low power mode
              i.e. data rate = 10Hz XY-rep = 1 Z-rep = 2*/
              priv->settings.preset_mode = BMM150_PRESETMODE_LOWPOWER;
              rslt += bmm150_set_presetmode(priv);

              /* Set the macros to enable DRDY pin */
              desired_settings = BMM150_DRDY_PIN_EN_SEL | BMM150_DRDY_POLARITY_SEL |
                                 BMM150_INT_PIN_EN_SEL | BMM150_INT_POLARITY_SEL | BMM150_INT_LATCH_SEL;
              /* Set the drdy_pin_en to enable the drdy interrupt  */
              priv->settings.int_settings.drdy_pin_en = BMM150_INT_ENABLE;
              /* Set the polarity as active high on the DRDY pin */
              priv->settings.int_settings.drdy_polarity = BMM150_ACTIVE_HIGH_POLARITY;
              /* Enable the mapping of interrupt to the interrupt pin*/
              priv->settings.int_settings.int_pin_en = BMM150_INT_ENABLE;
              /* Set the interrupt in non latched mode */
              priv->settings.int_settings.int_latch = BMM150_LATCHED;
              /* Set the interrupt polarity as active high */
              priv->settings.int_settings.int_polarity = BMM150_ACTIVE_HIGH_POLARITY;


              /* Set the configurations in the sensor */
              rslt += bmm150_set_sensor_settings(desired_settings, priv);

              if (rslt != BMM150_OK)
                {
                  ret = ERROR;
                }
            }
            break;

          case SNIOC_STOP:
            {
              /* Setting the power mode as normal */
              priv->settings.pwr_mode = BMM150_SLEEP_MODE;
              rslt = bmm150_set_op_mode(priv);

              if (rslt != BMM150_OK)
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

          /* Command:     SNIOC_INT_ENABLE
           * Description: Activates mapping of differnent kinds of interrupts to the INT pin.
           * Argument:    enum bmm150_int_type *int.*/

          case SNIOC_INT_ENABLE:
            {
              uint8_t int_sel = *(uint8_t *)((uintptr_t)arg);
              uint16_t desired_settings = 0;

              if (int_sel | BMM150_DATA_OVERRUN)
                {
                  desired_settings |= BMM150_DATA_OVERRUN_INT_SEL;
                  priv->settings.int_settings.data_overrun_en = BMM150_INT_ENABLE;
                }

              if (int_sel | BMM150_OVERFLOW)
                {
                  desired_settings |= BMM150_OVERFLOW_INT_SEL;
                  priv->settings.int_settings.overflow_int_en = BMM150_INT_ENABLE;
                }

              if (int_sel | BMM150_HIGH_X)
                {
                  desired_settings |= BMM150_HIGH_THRESHOLD_INT_SEL;
                  priv->settings.int_settings.high_int_en &= BMM150_THRESHOLD_X;
                }

              if (int_sel | BMM150_HIGH_Y)
                {
                  desired_settings |= BMM150_HIGH_THRESHOLD_INT_SEL;
                  priv->settings.int_settings.high_int_en &= BMM150_THRESHOLD_Y;
                }

              if (int_sel | BMM150_HIGH_Z)
                {
                  desired_settings |= BMM150_HIGH_THRESHOLD_INT_SEL;
                  priv->settings.int_settings.high_int_en &= BMM150_THRESHOLD_Z;
                }

              if (int_sel | BMM150_LOW_X)
                {
                  desired_settings |= BMM150_LOW_THRESHOLD_INT_SEL;
                  priv->settings.int_settings.low_int_en &= BMM150_THRESHOLD_X;
                }

              if (int_sel | BMM150_LOW_Y)
                {
                  desired_settings |= BMM150_LOW_THRESHOLD_INT_SEL;
                  priv->settings.int_settings.low_int_en &= BMM150_THRESHOLD_Y;
                }

              if (int_sel | BMM150_LOW_Z)
                {
                  desired_settings |= BMM150_LOW_THRESHOLD_INT_SEL;
                  priv->settings.int_settings.low_int_en &= BMM150_THRESHOLD_Z;
                }

              /* Set the configurations in the sensor */
              rslt = bmm150_set_sensor_settings(desired_settings, priv);

              if (rslt != BMM150_OK)
                {
                  ret = ERROR;
                }
              else
                {
                  /*enable interrupt pin*/
                  if (dev->ll_op->bmm150_INT_enable != NULL)
                    {
                      dev->ll_op->bmm150_INT_enable(true);
                    }
                }
            }
            break;

          case SNIOC_INT_DISABLE:
            {
              uint8_t int_sel = *(uint8_t *)((uintptr_t)arg);
              uint16_t desired_settings = 0;

              if (int_sel | BMM150_DATA_OVERRUN)
                {
                  desired_settings |= BMM150_DATA_OVERRUN_INT_SEL;
                  priv->settings.int_settings.data_overrun_en = BMM150_INT_DISABLE;
                }

              if (int_sel | BMM150_OVERFLOW)
                {
                  desired_settings |= BMM150_OVERFLOW_INT_SEL;
                  priv->settings.int_settings.overflow_int_en = BMM150_INT_DISABLE;
                }

              if (int_sel | BMM150_HIGH_X)
                {
                  desired_settings |= BMM150_HIGH_THRESHOLD_INT_SEL;
                  priv->settings.int_settings.high_int_en |= (~BMM150_THRESHOLD_X);
                }

              if (int_sel | BMM150_HIGH_Y)
                {
                  desired_settings |= BMM150_HIGH_THRESHOLD_INT_SEL;
                  priv->settings.int_settings.high_int_en |= (~BMM150_THRESHOLD_Y);
                }

              if (int_sel | BMM150_HIGH_Z)
                {
                  desired_settings |= BMM150_HIGH_THRESHOLD_INT_SEL;
                  priv->settings.int_settings.high_int_en |= (~BMM150_THRESHOLD_Z);
                }

              if (int_sel | BMM150_LOW_X)
                {
                  desired_settings |= BMM150_LOW_THRESHOLD_INT_SEL;
                  priv->settings.int_settings.low_int_en |= (~BMM150_THRESHOLD_X);
                }

              if (int_sel | BMM150_LOW_Y)
                {
                  desired_settings |= BMM150_LOW_THRESHOLD_INT_SEL;
                  priv->settings.int_settings.low_int_en |= (~BMM150_THRESHOLD_Y);
                }

              if (int_sel | BMM150_LOW_Z)
                {
                  desired_settings |= BMM150_LOW_THRESHOLD_INT_SEL;
                  priv->settings.int_settings.low_int_en |= (~BMM150_THRESHOLD_Z);
                }

              /* Set the configurations in the sensor */
              rslt = bmm150_set_sensor_settings(desired_settings, priv);

              if (rslt != BMM150_OK)
                {
                  ret = ERROR;
                }
              else
                {

                  if (priv->settings.int_settings.overflow_int_en != BMM150_INT_DISABLE)
                    {
                      break;
                    }

                  if (priv->settings.int_settings.high_int_en != \
                      ((~BMM150_THRESHOLD_X) | (~BMM150_THRESHOLD_Y) | (~BMM150_THRESHOLD_Z)))
                    {
                      break;
                    }

                  if (priv->settings.int_settings.low_int_en != \
                      ((~BMM150_THRESHOLD_X) | (~BMM150_THRESHOLD_Y) | (~BMM150_THRESHOLD_Z)))
                    {
                      break;
                    }

                  /*If no interrupt still enabled, disable interrupt pin*/
                  if (dev->ll_op->bmm150_INT_enable != NULL)
                    {
                      dev->ll_op->bmm150_INT_enable(false);
                    }
                }

            }
            break;

          case SNIOC_HIGH_THRESHOLD:
            {
              uint8_t high_th = *(uint8_t *)((uintptr_t)arg);
              uint16_t desired_settings = 0;

              desired_settings |= BMM150_HIGH_THRESHOLD_SETTING_SEL;
              priv->settings.int_settings.high_threshold = high_th;

              /* Set the configurations in the sensor */
              rslt = bmm150_set_sensor_settings(desired_settings, priv);

              if (rslt != BMM150_OK)
                {
                  ret = ERROR;
                }

            }
            break;

          case SNIOC_LOW_THRESHOLD:
            {
              uint8_t low_th = *(uint8_t *)((uintptr_t)arg);
              uint16_t desired_settings = 0;

              desired_settings |= BMM150_LOW_THRESHOLD_SETTING_SEL;
              priv->settings.int_settings.high_threshold = low_th;

              /* Set the configurations in the sensor */
              rslt = bmm150_set_sensor_settings(desired_settings, priv);

              if (rslt != BMM150_OK)
                {
                  ret = ERROR;
                }

            }
            break;

          case SNIOC_INT_STATUS:
            {
              uint16_t *status = (uint16_t *)((uintptr_t)arg);

              /* got interrupt status */
              rslt = bmm150_get_interrupt_status(priv);

              if (rslt != BMM150_OK)
                {
                  ret = ERROR;
                }
              else
                {
                  *status = priv->int_status;
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
 * Name: bmm150_register
 *
 * Description:
 *   Register the BMM150 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath -    The full path to the driver to register. E.g., "/dev/bmm150"
 *   bus_config - An struct describing the bus which use to communicate with BMM150
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/
int bmm150_register(FAR const char *devpath,
                    FAR struct bmm150_bus_config_s *bus_config,
                    const struct bmm150_low_level_operations_s *ll_op)
{
  int ret;
  struct bmm150_t *bmm150;

  /* Sanity check */

  DEBUGASSERT(bus_config != NULL);


  /* Initialize the BMG160 device structure */

  bmm150 = (FAR struct bmm150_t *)kmm_malloc(sizeof(struct bmm150_t));
  if (bmm150 == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  bmm150->ocount = 0;
  /* Initialize device open semphone*/
  sem_init(&bmm150->closesem, 0, 1);
  /* Initialize millsecond delay semphone */
  sem_init(&bmm150->delaysem, 0, 0);
  /* Initialize ioctrl semphone */
  sem_init(&bmm150->ioctrlsem, 0, 1);
  /* Initialize data ready semphone */
  sem_init(&bmm150->drdysem, 0, 0);


  bmm150->receive_pid = 0;
  bmm150->signo = 0;

  bmm150->ll_op = ll_op;

  memcpy(&bmm150->bus, bus_config, sizeof(struct bmm150_bus_config_s));

  if (bmm150->bus.bus_selection == BMM150_USE_SPI)
    {
      bmm150->internal.dev_id = 0;
      bmm150->internal.intf = BMM150_SPI_INTF;
      bmm150->internal.read = bmm150_spi_read_reg;
      bmm150->internal.write = bmm150_spi_write_reg;
      bmm150->internal.delay_ms = bmm150_delay_msec;
    }
  else
    {
      bmm150->internal.dev_id = BMM150_DEFAULT_I2C_ADDRESS;
      bmm150->internal.intf = BMM150_I2C_INTF;
      bmm150->internal.read = bmm150_i2c_read_reg;
      bmm150->internal.write = bmm150_i2c_write_reg;
      bmm150->internal.delay_ms = bmm150_delay_msec;
    }

  /*config INT pin*/

  if (bmm150->ll_op && bmm150->ll_op->bmm150_INT_cfg_interrupt)
    {
      bmm150->ll_op->bmm150_INT_cfg_interrupt(bmm150_INT_pin_handler, bmm150);
    }

  /*config DRDY pin*/

  if (bmm150->ll_op && bmm150->ll_op->bmm150_DRDY_cfg_interrupt)
    {
      bmm150->ll_op->bmm150_DRDY_cfg_interrupt(bmm150_DRDY_pin_handler, bmm150);
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_bmm150_fops, 0666, bmm150);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      sem_destroy(&bmm150->closesem);
      sem_destroy(&bmm150->delaysem);
      sem_destroy(&bmm150->ioctrlsem);
      kmm_free(bmm150);
      return ret;
    }

  return OK;

}

#endif /* CONFIG_BMM150*/


