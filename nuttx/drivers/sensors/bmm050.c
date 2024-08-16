/****************************************************************************
 * drivers/sensors/bmm050.c
 * Character driver for the bmm050 magnetometer.
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

#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>

#include <nuttx/fs/fs.h>
#include <nuttx/sensors/internal_bmm050.h>
#include <nuttx/sensors/bmm050.h>

#if (defined CONFIG_BMM050)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#ifndef container_of
#  define container_of(ptr, type, member) \
          ((type *)((intptr_t)(ptr) - offsetof(type, member)))
#endif

#define BMM150_RW_REG_BUFFER_MAX_LENGTH 16

/****************************************************************************
 * Private type
 ****************************************************************************/
struct bmm050_dev_s
{

  FAR struct bmm050_bus_config_s bus;

  uint8_t ocount; /* The number of times the device has been opened */
  sem_t closesem; /* Locks out new opens while close is in progress */
  sem_t ioctrlsem; /* Locks out new IOCtl operation while the old one is in progress */
  sem_t delaysem;/*this semphone is initialize as 0 and never post,
                  *so we can implement millseconds delay using the
                  *sem_timedwait function.
                  */
  uint8_t        signo; /* signo to use when signaling a interrupt */
  pid_t          receive_pid; /* The task to be signalled */

  FAR struct bmm050_t internal;/* used by the internal_bmm050.c */

};



/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int bmm050_open(FAR struct file *filep);
static int bmm050_close(FAR struct file *filep);
static ssize_t bmm050_read(FAR struct file *, FAR char *, size_t);
static ssize_t bmm050_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int bmm050_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_bmm050_fops =
{
  bmm050_open,
  bmm050_close,
  bmm050_read,
  bmm050_write,
  NULL,
  bmm050_ioctl
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
 * Name: bmm050_i2c_read_reg
 ****************************************************************************/
static int bmm050_i2c_read_reg(struct bmm050_dev_s *dev, uint8_t reg_addr,
                               uint8_t *data, uint16_t len)
{
  return i2c_writeread(dev->bus.bus_config.i2c_config.i2c,
                       &dev->bus.bus_config.i2c_config.config,
                       &reg_addr, 1,
                       data, len);

}

/****************************************************************************
 * Name: bmm050_i2c_write_reg
 ****************************************************************************/
static int bmm050_i2c_write_reg(struct bmm050_dev_s *dev, uint8_t reg_addr,
                                uint8_t *data, uint16_t len)
{

  uint8_t buffer[BMM150_RW_REG_BUFFER_MAX_LENGTH];

  if (len >  (BMM150_RW_REG_BUFFER_MAX_LENGTH - 1))
    {
      len = BMM150_RW_REG_BUFFER_MAX_LENGTH - 1;
    }

  buffer[0] = reg_addr;
  memcpy(&buffer[1], data, len);

  return i2c_write(dev->bus.bus_config.i2c_config.i2c,
                   &dev->bus.bus_config.i2c_config.config,
                   buffer, (len + 1));

}

/****************************************************************************
 * Name: bmm050_spi_read_reg
 ****************************************************************************/
static int bmm050_spi_read_reg(struct bmm050_dev_s *dev, uint8_t reg_addr,
                               uint8_t *data, uint16_t len)
{
  struct bmm050_spi_bus_config_s *cfg;

  cfg = &dev->bus.bus_config.spi_config;

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
  SPI_SEND(cfg->spi, BMM050_SPI_RD_MASK | reg_addr);

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

  return OK;

}

/****************************************************************************
 * Name: bmm050_spi_write_reg
 ****************************************************************************/
static int bmm050_spi_write_reg(struct bmm050_dev_s *dev, uint8_t reg_addr,
                                uint8_t *data, uint16_t len)
{
  struct bmm050_spi_bus_config_s *cfg;

  cfg = &dev->bus.bus_config.spi_config;

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

  return OK;
}

/****************************************************************************
 * Name: bmm050_open
 ****************************************************************************/

static int bmm050_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct bmm050_dev_s *priv;
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
              ret = bmm050_init(&priv->internal);
              if (ret == OK)
                {
                  /* Save the new open count on success */

                  priv->ocount = tmp;
                }
              else
                {
                  snerr("ERROR: bmm050_init failed: %d\n", ret);
                  ret = -EIO;
                }
            }
        }

      sem_post(&priv->closesem);
    }

  return ret;
}

/****************************************************************************
 * Name: bmm050_close
 ****************************************************************************/

static int bmm050_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct bmm050_dev_s *dev;
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

          bmm050_soft_rst(&dev->internal);
        }
      sem_post(&dev->closesem);
    }

  return ret;
}

/****************************************************************************
 * Name: bmg160_read
 ****************************************************************************/

static ssize_t bmm050_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode *inode;
  FAR struct bmm050_dev_s *priv;
  s32 data_size = 0;
  int ret = OK;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private != NULL);
  priv = inode->i_private;


#ifdef CONFIG_BMM050_FLOAT_ENABLE
  /* accessing the bmm050_mdata_float parameter by using data_float*/
  /* Reads mag xyz data output as float value*/
  if (buflen >= sizeof(struct bmm050_mag_data_float_t))
    {
      ret = bmm050_read_mag_data_XYZ_float(&priv->internal, (struct bmm050_mag_data_float_t *)buffer);
      data_size = sizeof(struct bmm050_mag_data_float_t);
    }
#else
  /* accessing the bmm050_mdata_s16 parameter by using data_s16*/
  /* Reads mag xyz data output as 16bit value*/
  if (buflen >= sizeof(struct bmm050_mag_data_s16_t))
    {
      ret = bmm050_read_mag_data_XYZ(&priv->internal, (struct bmm050_mag_data_s16_t *)buffer);
      data_size = sizeof(struct bmm050_mag_data_s16_t);
    }
#endif

  if (ret == 0)
    {
      return data_size;
    }
  else
    {
      return 0;
    }

}

/****************************************************************************
 * Name: bmg160_write
 ****************************************************************************/

static ssize_t bmm050_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: bmg160_ioctl
 ****************************************************************************/

static int bmm050_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  struct bmm050_dev_s *dev;
  int ret = OK;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private != NULL);
  dev = inode->i_private;

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
              uint8_t power_status;
              FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
              DEBUGASSERT(ptr != NULL);

              /* get device power status */
              ret = bmm050_get_power_mode(&dev->internal, &power_status);

              /*If it's in low power mode, turn it on*/
              if (power_status == BMM050_OFF)
                {
                  ret += bmm050_set_power_mode(&dev->internal, BMM050_ON);

                  /* wait two millisecond for bmc to settle */
                  bmm050_delay_msec(&dev->internal, BMM050_DELAY_SETTLING_TIME);
                }

              /*Read CHIP_ID and REv. info */
              ret += bmm050_read_register(&dev->internal,
                                          BMM050_CHIP_ID, ptr, BMM050_GEN_READ_WRITE_DATA_LENGTH);
              if (ret != OK)
                {
                  return -EIO;
                }

            }

            break;

          case SNIOC_START:
            {
              uint8_t data_rate;

              ret = bmm050_set_functional_state(&dev->internal, BMM050_NORMAL_MODE);

              /* Set the data rate of the sensor, input   value have to be given
              data rate value set from the register 0x4C bit 3 to 5*/
              data_rate = BMM050_DATA_RATE_30HZ;/* set data rate of 30Hz*/
              ret += bmm050_set_data_rate(&dev->internal, data_rate);

              /* This API used to read back the written value of data rate*/
              ret += bmm050_get_data_rate(&dev->internal, &data_rate);

              if (ret != OK)
                {
                  return -EIO;
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
 * Name: bmm050_write_register
 ****************************************************************************/
int bmm050_write_register(FAR struct bmm050_t *bmm050,
                          uint8_t WriteAddr,
                          uint8_t *pBuffer,
                          uint8_t nBytesToWrite)
{
  int ret;
  FAR struct bmm050_dev_s *dev;

  DEBUGASSERT(bmm050);
  dev = container_of(bmm050, struct bmm050_dev_s, internal);


  if (dev->bus.bus_selection == BMM050_I2C_INTF)
    {
      ret = bmm050_i2c_write_reg(dev, WriteAddr, pBuffer, nBytesToWrite);
    }
  else
    {
      ret = bmm050_spi_write_reg(dev, WriteAddr, pBuffer, nBytesToWrite);
    }

  return ret;

}

/****************************************************************************
 * Name: bmm050_read_register
 ****************************************************************************/
int bmm050_read_register(FAR struct bmm050_t *bmm050,
                         uint8_t ReadAddr,
                         uint8_t *pBuffer,
                         uint8_t nBytesToRead)
{
  int ret;
  FAR struct bmm050_dev_s *dev;

  DEBUGASSERT(bmm050);
  dev = container_of(bmm050, struct bmm050_dev_s, internal);


  if (dev->bus.bus_selection == BMM050_I2C_INTF)
    {
      ret = bmm050_i2c_read_reg(dev, ReadAddr, pBuffer, nBytesToRead);
    }
  else
    {
      ret = bmm050_spi_read_reg(dev, ReadAddr, pBuffer, nBytesToRead);
    }

  return ret;

}

/****************************************************************************
 * Name: bmm050_delay
 ****************************************************************************/

void bmm050_delay_msec(struct bmm050_t *dev, uint32_t period)
{
  FAR struct bmm050_dev_s *priv;
  struct timespec time;

  DEBUGASSERT(dev);
  priv = container_of(dev, struct bmm050_dev_s, internal);

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

      sem_timedwait(&priv->delaysem, &time);
    }
}


/****************************************************************************
 * Name: bmm050_register
 *
 * Description:
 *   Register the BMM050 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath -    The full path to the driver to register. E.g., "/dev/bmm050"
 *   bus_config - An struct describing the bus which use to communicate with BMM050
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/
int bmm050_register(FAR const char *devpath,
                    FAR struct bmm050_bus_config_s *bus_config)
{
  int ret;
  struct bmm050_dev_s *dev;

  /* Sanity check */

  DEBUGASSERT(bus_config != NULL);


  /* Initialize the BMG160 device structure */

  dev = (FAR struct bmm050_dev_s *)kmm_malloc(sizeof(struct bmm050_dev_s));
  if (dev == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  dev->ocount = 0;
  /* Initialize device open semphone*/
  sem_init(&dev->closesem, 0, 1);
  /* Initialize millsecond delay semphone */
  sem_init(&dev->delaysem, 0, 0);
  /* Initialize ioctrl semphone */
  sem_init(&dev->ioctrlsem, 0, 1);

  dev->receive_pid = 0;
  dev->signo = 0;

  memcpy(&dev->bus, bus_config, sizeof(struct bmm050_bus_config_s));

  /* Register the character driver */

  ret = register_driver(devpath, &g_bmm050_fops, 0666, dev);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      sem_destroy(&dev->closesem);
      sem_destroy(&dev->delaysem);
      sem_destroy(&dev->ioctrlsem);
      kmm_free(dev);
      return ret;
    }

  return OK;

}

#endif /* CONFIG_BMM050*/


