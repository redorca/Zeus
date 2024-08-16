/****************************************************************************
 * drivers/sensors/bmi160.c
 * Character driver for the BMi160 6-Axis comb device.
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

#include <nuttx/fs/fs.h>
#include <nuttx/sensors/bmi160.h>

#ifdef CONFIG_BMI160

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define BMI160_RW_REG_BUFFER_MAX_LENGTH 16

/****************************************************************************
 * Private type
 ****************************************************************************/
struct bmi160_dev_s
{

  FAR struct bmi160_bus_config_s bus;

  uint8_t ocount; /* The number of times the device has been opened */
  sem_t closesem; /* Locks out new opens while close is in progress */
  sem_t ioctrlsem; /* Locks out new IOCtl operation while the old one is in progress */
  sem_t delaysem;/*this semphone is initialize as 0 and never post,
                  *so we can implement millseconds delay using the
                  *sem_timedwait function.
                  */

  FAR struct bmi160_dev internal;/* used by the internal_bmi160.c */

  uint8_t        signo; /* signo to use when signaling a interrupt */
  pid_t          receive_pid; /* The task to be signalled */

  struct bmi160_low_level_operations_s low_level_op;
};



/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int bmi160_open(FAR struct file *filep);
static int bmi160_close(FAR struct file *filep);
static ssize_t bmi160_read(FAR struct file *, FAR char *, size_t);
static ssize_t bmi160_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int bmi160_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_bmi160_fops =
{
  bmi160_open,
  bmi160_close,
  bmi160_read,
  bmi160_write,
  NULL,
  bmi160_ioctl
#ifndef CONFIG_DISABLE_POLL
  , NULL
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL
#endif
};

/* struct to store instances of drivers */
static struct bmi160_dev_s *g_bmi160 = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: convert_accel_power_mode
 *
 * Description:
 *   convert global power mode definition to bmi160 specified definition
 *
 ****************************************************************************/

static int convert_accel_power_mode(sn_ga_mode_s            power_mode, uint8_t *bmi160_power_mode)
{
  int ret = OK;

  switch (power_mode)
    {
      case GA_MODE_NORMAL:
        *bmi160_power_mode = BMI160_ACCEL_NORMAL_MODE;
        break;
      case GA_MODE_LOWPOWER:
        *bmi160_power_mode = BMI160_ACCEL_LOWPOWER_MODE;
        break;
      case GA_MODE_SUSPEND:
        *bmi160_power_mode = BMI160_ACCEL_SUSPEND_MODE;
        break;
      default:
        ret = -EINVAL;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: convert_accel_power_mode
 *
 * Description:
 *   convert global power mode definition to bmi160 specified definition
 *
 ****************************************************************************/

static int convert_gyro_power_mode(sn_ga_mode_s            power_mode, uint8_t *bmi160_power_mode)
{
  int ret = OK;

  switch (power_mode)
    {
      case GA_MODE_NORMAL:
        *bmi160_power_mode = BMI160_GYRO_NORMAL_MODE;
        break;
      case GA_MODE_FASTSTARTUP:
        *bmi160_power_mode = BMI160_GYRO_FASTSTARTUP_MODE;
        break;
      case GA_MODE_SUSPEND:
        *bmi160_power_mode = BMI160_GYRO_SUSPEND_MODE;
        break;
      default:
        ret = -EINVAL;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: convert_accel_power_mode
 *
 * Description:
 *   convert global odr definition to bmi160 specified definition
 *
 ****************************************************************************/

static int convert_accel_odr(int odr, uint8_t *bmi160_odr)
{
  int ret = OK;

  switch (odr)
    {
      case 7:
        *bmi160_odr = BMI160_ACCEL_ODR_0_78HZ;
        break;
      case 15:
        *bmi160_odr = BMI160_ACCEL_ODR_1_56HZ;
        break;
      case 31:
        *bmi160_odr = BMI160_ACCEL_ODR_3_12HZ;
        break;
      case 62:
        *bmi160_odr = BMI160_ACCEL_ODR_6_25HZ;
        break;
      case 125:
        *bmi160_odr = BMI160_ACCEL_ODR_12_5HZ;
        break;
      case 250:
        *bmi160_odr = BMI160_ACCEL_ODR_25HZ;
        break;
      case 500:
        *bmi160_odr = BMI160_ACCEL_ODR_50HZ;
        break;
      case 1000:
        *bmi160_odr = BMI160_ACCEL_ODR_100HZ;
        break;
      case 2000:
        *bmi160_odr = BMI160_ACCEL_ODR_200HZ;
        break;
      case 4000:
        *bmi160_odr = BMI160_ACCEL_ODR_400HZ;
        break;
      case 8000:
        *bmi160_odr = BMI160_ACCEL_ODR_800HZ;
        break;
      case 16000:
        *bmi160_odr = BMI160_ACCEL_ODR_1600HZ;
        break;
      default:
        ret = -EINVAL;
        break;
    }

  return ret;

}

/****************************************************************************
 * Name: convert_accel_power_mode
 *
 * Description:
 *   convert global odr definition to bmi160 specified definition
 *
 ****************************************************************************/

static int convert_gyro_odr(int odr, uint8_t *bmi160_odr)
{
  int ret = OK;

  switch (odr)
    {
      case 250:
        *bmi160_odr = BMI160_GYRO_ODR_25HZ;
        break;
      case 500:
        *bmi160_odr = BMI160_GYRO_ODR_50HZ;
        break;
      case 1000:
        *bmi160_odr = BMI160_GYRO_ODR_100HZ;
        break;
      case 2000:
        *bmi160_odr = BMI160_GYRO_ODR_200HZ;
        break;
      case 4000:
        *bmi160_odr = BMI160_GYRO_ODR_400HZ;
        break;
      case 8000:
        *bmi160_odr = BMI160_GYRO_ODR_800HZ;
        break;
      case 16000:
        *bmi160_odr = BMI160_GYRO_ODR_1600HZ;
        break;
      case 32000:
        *bmi160_odr = BMI160_GYRO_ODR_3200HZ;
        break;
      default:
        ret = -EINVAL;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: convert_accel_power_mode
 *
 * Description:
 *   convert global range definition to bmi160 specified definition
 *
 ****************************************************************************/

static int convert_accel_range(int range, uint8_t *bmi160_range)
{
  int ret = OK;

  switch (range)
    {
      case 2:
        *bmi160_range = BMI160_ACCEL_RANGE_2G;
        break;
      case 4:
        *bmi160_range = BMI160_ACCEL_RANGE_4G;
        break;
      case 8:
        *bmi160_range = BMI160_ACCEL_RANGE_8G;
        break;
      case 16:
        *bmi160_range = BMI160_ACCEL_RANGE_16G;
        break;
      default:
        ret = -EINVAL;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: convert_accel_power_mode
 *
 * Description:
 *   convert global range definition to bmi160 specified definition
 *
 ****************************************************************************/

static int convert_gyro_range(int range, uint8_t *bmi160_range)
{
  int ret = OK;

  switch (range)
    {
      case 2000:
        *bmi160_range = BMI160_GYRO_RANGE_2000_DPS;
        break;
      case 1000:
        *bmi160_range = BMI160_GYRO_RANGE_1000_DPS;
        break;
      case 500:
        *bmi160_range = BMI160_GYRO_RANGE_500_DPS;
        break;
      case 250:
        *bmi160_range = BMI160_GYRO_RANGE_250_DPS;
        break;
      case 125:
        *bmi160_range = BMI160_GYRO_RANGE_125_DPS;
        break;
      default:
        ret = -EINVAL;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: convert_accel_power_mode
 *
 * Description:
 *   convert accelerometer configration defined in sn_ga_param_s formate to
 *   bmi160_cfg formate
 *
 ****************************************************************************/

static int convert_accel_ga_param(sn_ga_param_s *param,
                                  struct bmi160_cfg *config)
{
  int ret = OK;

  ret = convert_accel_range(param->range, &config->range);
  if (ret == OK)
    {
      ret = convert_accel_odr(param->odr, &config->odr);
      if (ret == OK )
        {
          ret = convert_accel_power_mode(param->power_mode, &config->power);
          return ret;
        }
      else
        {
          return ret;
        }
    }
  else
    {
      return ret;
    }
}

/****************************************************************************
 * Name: convert_accel_power_mode
 *
 * Description:
 *   convert gyro configration defined in sn_ga_param_s formate to
 *   bmi160_cfg formate
 *
 ****************************************************************************/

static int convert_gyro_ga_param(sn_ga_param_s *param,
                                 struct bmi160_cfg *config)
{
  int ret = OK;

  ret = convert_gyro_range(param->range, &config->range);
  if (ret == OK)
    {
      ret = convert_gyro_odr(param->odr, &config->odr);
      if (ret == OK )
        {
          ret = convert_gyro_power_mode(param->power_mode, &config->power);
          return ret;
        }
      else
        {
          return ret;
        }
    }
  else
    {
      return ret;
    }
}


/****************************************************************************
 * Name: bmi160_i2c_read_reg
 ****************************************************************************/
static int8_t bmi160_i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr,
                                  uint8_t *data, uint16_t len)
{
  return i2c_writeread(g_bmi160->bus.bus_config.i2c_config.i2c,
                       &g_bmi160->bus.bus_config.i2c_config.config,
                       &reg_addr, 1,
                       data, len);

}

/****************************************************************************
 * Name: bmi160_i2c_write_reg
 ****************************************************************************/
static int8_t bmi160_i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr,
                                   uint8_t *data, uint16_t len)
{
  uint8_t buffer[BMI160_RW_REG_BUFFER_MAX_LENGTH];

  if (len >  (BMI160_RW_REG_BUFFER_MAX_LENGTH - 1))
    {
      len = BMI160_RW_REG_BUFFER_MAX_LENGTH - 1;
    }

  buffer[0] = reg_addr;
  memcpy(&buffer[1], data, len);

  return i2c_write(g_bmi160->bus.bus_config.i2c_config.i2c,
                   &g_bmi160->bus.bus_config.i2c_config.config,
                   buffer, (len + 1));
}

/****************************************************************************
 * Name: bmi160_spi_read_reg
 ****************************************************************************/
static int8_t bmi160_spi_read_reg(uint8_t dev_addr, uint8_t reg_addr,
                                  uint8_t *data, uint16_t len)
{
  struct bmi160_spi_bus_config_s *dev;

  dev = &g_bmi160->bus.bus_config.spi_config;

  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(dev->spi, true);

  /*Set frequency*/
  SPI_SETFREQUENCY(dev->spi, dev->spi_frequency);

  /*Set mode*/
  SPI_SETMODE(dev->spi, dev->spi_mode);

  /* Set CS to low which selects the BMG160 */

  SPI_SELECT(dev->spi, dev->spi_devid, true);

  /* Transmit the register address from where we want to read - the MSB needs
   * to be set to indicate the read indication.
   */
  SPI_SEND(dev->spi, reg_addr);

  /* Write idle bytes while receiving the required data */
  if (len > 1)
    {
      SPI_EXCHANGE(dev->spi, NULL, data, len);
    }
  else
    {
      *data = SPI_SEND(dev->spi, 0);
    }

  /* Set CS to high which deselects the BMG160 */

  SPI_SELECT(dev->spi, dev->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);

  return OK;

}

/****************************************************************************
 * Name: bmi160_spi_write_reg
 ****************************************************************************/
static int8_t bmi160_spi_write_reg(uint8_t dev_addr, uint8_t reg_addr,
                                   uint8_t *data, uint16_t len)
{
  struct bmi160_spi_bus_config_s *dev;

  dev = &g_bmi160->bus.bus_config.spi_config;

  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(dev->spi, true);

  /*Set frequency*/
  SPI_SETFREQUENCY(dev->spi, dev->spi_frequency);

  /*Set mode*/
  SPI_SETMODE(dev->spi, dev->spi_mode);

  /* Set CS to low which selects the BMG160 */

  SPI_SELECT(dev->spi, dev->spi_devid, true);

  /* Transmit the register address from where we want to read */

  SPI_SEND(dev->spi, reg_addr);

  /* Transmit the content which should be written in the register */

  if (len > 1)
    {
      SPI_EXCHANGE(dev->spi, data, NULL, len);
    }
  else
    {
      SPI_SEND(dev->spi, *data);
    }

  /* Set CS to high which deselects the BMG160 */

  SPI_SELECT(dev->spi, dev->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);

  return OK;
}

static void bmi160_delay(uint32_t period)
{
  struct timespec time;

  if (period <= BMI160_ONE_MS_DELAY)
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

      sem_timedwait(&g_bmi160->delaysem, &time);
    }
}

/****************************************************************************
 * Name: bmi160_int_handler
 ****************************************************************************/
static void bmi160_int_handler(enum bmi160_int_channel int_channel)
{
  union sigval value;

  if ((int_channel == BMI160_INT_CHANNEL_1) || (int_channel == BMI160_INT_CHANNEL_2))
    {
      value.sival_int = int_channel;
      (void)sigqueue(g_bmi160->receive_pid, g_bmi160->signo, value);
    }
}


/****************************************************************************
 * Name: bmi160_open
 ****************************************************************************/

static int bmi160_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct bmi160_dev_s *priv;
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
              /* Yes.. perform one time hardware initialization. */
              ret = bmi160_init(&priv->internal);
              if (ret == BMI160_OK)
                {
                  /* Save the new open count on success */

                  priv->ocount = tmp;
                }
              else
                {
                  snerr("ERROR: bmi160_init failed: %d\n", ret);
                  ret = -EIO;
                }
            }
        }

      sem_post(&priv->closesem);
    }

  return ret;
}

/****************************************************************************
 * Name: bmi160_close
 ****************************************************************************/

static int bmi160_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct bmi160_dev_s *priv;
  int                   ret = OK;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private != NULL);
  priv = inode->i_private;


  /* Perform a reset */
  if (sem_wait(&priv->closesem) != OK)
    {
      ret = -errno;
    }
  else
    {
      /* Decrement the references to the driver.  If the reference count will
       * decrement to 0, then uninitialize the driver.
       */

      if (priv->ocount > 1)
        {
          priv->ocount--;
        }
      else
        {
          /* There are no more references to the port */

          priv->ocount = 0;

          /* disable the IRQ*/

          g_bmi160->low_level_op.bmi160_int_enable(BMI160_INT_CHANNEL_BOTH, false);

          /* disable the device */

          bmi160_soft_reset(&priv->internal);
        }
      sem_post(&priv->closesem);
    }

  return ret;
}

/****************************************************************************
 * Name: bmg160_read
 ****************************************************************************/

static ssize_t bmi160_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  return 0;
}

/****************************************************************************
 * Name: bmg160_write
 ****************************************************************************/

static ssize_t bmi160_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: bmg160_ioctl
 ****************************************************************************/

static int bmi160_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  struct bmi160_dev_s *dev;
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
              FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
              DEBUGASSERT(ptr != NULL);

              /* Read chip_id */
              ret = bmi160_get_regs(BMI160_CHIP_ID_ADDR, ptr, 1, &dev->internal);
              if (ret != OK)
                {
                  return -EIO;
                }

            }
            break;


          /* Command:     SNIOC_A_SPARAM
           * Description: set accel param
           * Argument:    pointer to the sn_ga_param_s struct*/

          case SNIOC_A_SPARAM:
            {
              sn_ga_param_s *param = (sn_ga_param_s *)arg;

              if (param != NULL)
                {
                  ret = convert_accel_ga_param(param, &dev->internal.accel_cfg);

                  if (ret == OK)
                    {
                      ret = bmi160_set_sens_conf(&dev->internal);
                      if (ret != BMI160_OK)
                        {
                          snerr("ERROR: bmi160_set_sens_conf failed: %d\n", ret);
                          ret = -EIO;
                        }
                    }
                }
              else
                {
                  ret = -EINVAL;
                }
            }
            break;

          /* Command:     SNIOC_G_SPARAM
           * Description: set gyro param
           * Argument:    pointer to the sn_ga_param_s struct*/

          case SNIOC_G_SPARAM:
            {
              sn_ga_param_s *param = (sn_ga_param_s *)arg;

              if (param != NULL)
                {
                  ret = convert_gyro_ga_param(param, &dev->internal.gyro_cfg);

                  if (ret == OK)
                    {
                      ret = bmi160_set_sens_conf(&dev->internal);
                      if (ret != BMI160_OK)
                        {
                          snerr("ERROR: bmi160_set_sens_conf failed: %d\n", ret);
                          ret = -EIO;
                        }
                    }
                }
              else
                {
                  ret = -EINVAL;
                }
            }
            break;

          /* Command:     SNIOC_GA_CONFIG_ISR
           * Description: Configuring sensor interrupt source
           * Argument:    pointer to the interrupt configration struct,
                          different seneor may have different data type*/

          case SNIOC_GA_CONFIG_ISR:
            {
              struct bmi160_int_settg *config = (struct bmi160_int_settg *)arg;

              if (config != NULL)
                {

                  if (config->int_channel == BMI160_INT_CHANNEL_NONE)
                    {
                      g_bmi160->low_level_op.bmi160_int_enable(BMI160_INT_CHANNEL_BOTH, false);
                    }
                  else
                    {
                      g_bmi160->low_level_op.bmi160_int_enable(config->int_channel, false);
                    }


                  ret = bmi160_set_int_config(config, &dev->internal);
                  if (ret != BMI160_OK)
                    {
                      snerr("ERROR: bmi160_set_int_config failed: %d\n", ret);
                      ret = -EIO;
                    }

                  if (config->int_channel != BMI160_INT_CHANNEL_NONE)
                    {
                      g_bmi160->low_level_op.bmi160_int_enable(config->int_channel, true);
                    }

                }
              else
                {
                  ret = -EINVAL;
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

          /* Command:     SNIOC_A_GRAW
           * Description: get accelerometer raw data
           * Argument:    pointer to the sn_ga_raw_s struct*/

          case SNIOC_A_GRAW:
            {
              struct bmi160_sensor_data data;

              sn_ga_raw_s *raw = (sn_ga_raw_s *)arg;

              if (raw != NULL)
                {
                  ret = bmi160_get_sensor_data(BMI160_ACCEL_SEL, &data, NULL, &dev->internal);
                  if (ret == BMI160_OK)
                    {
                      memcpy(raw, &data, sizeof(sn_ga_raw_s));
                    }
                  else
                    {
                      snerr("ERROR: bmi160_get_sensor_data failed: %d\n", ret);
                      ret = -EIO;
                    }
                }
              else
                {
                  ret = -EINVAL;
                }
            }
            break;

          /* Command:     SNIOC_G_GRAW
           * Description: get gyro raw data
           * Argument:    pointer to the sn_ga_raw_s struct*/

          case SNIOC_G_GRAW:
            {
              struct bmi160_sensor_data data;

              sn_ga_raw_s *raw = (sn_ga_raw_s *)arg;

              if (raw != NULL)
                {
                  ret = bmi160_get_sensor_data(BMI160_GYRO_SEL, &data, NULL, &dev->internal);
                  if (ret == BMI160_OK)
                    {
                      memcpy(raw, &data, sizeof(sn_ga_raw_s));
                    }
                  else
                    {
                      snerr("ERROR: bmi160_get_sensor_data failed: %d\n", ret);
                      ret = -EIO;
                    }
                }
              else
                {
                  ret = -EINVAL;
                }
            }
            break;

          /* Command:     SNIOC_ACCEL_SETBW
           * Description: set accelormeter bandwidth
           * Argument:    value of the bandwidth */

          case SNIOC_ACCEL_SETBW:
            {
              dev->internal.accel_cfg.bw = (uint8_t)arg;

              ret = bmi160_set_sens_conf(&dev->internal);
              if (ret != BMI160_OK)
                {
                  snerr("ERROR: bmi160_set_sens_conf failed: %d\n", ret);
                  ret = -EIO;
                }
            }
            break;

          /* Command:     SNIOC_GYRO_SETBW
           * Description: set gyro bandwidth
           * Argument:    value of the bandwidth */

          case SNIOC_GYRO_SETBW:
            {
              dev->internal.gyro_cfg.bw = (uint8_t)arg;

              ret = bmi160_set_sens_conf(&dev->internal);
              if (ret != BMI160_OK)
                {
                  snerr("ERROR: bmi160_set_sens_conf failed: %d\n", ret);
                  ret = -EIO;
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
 * Name: bmi160_register
 *
 * Description:
 *   Register the BMI160 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath -    The full path to the driver to register. E.g., "/dev/gyr0"
 *   bus_config - An struct describing the bus which use to communicate with BMI160
 *   attach_irq - function used to initialize the interrput pin and attach
 *                interrput handler.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/
int bmi160_register(FAR const char *devpath,
                    FAR struct bmi160_bus_config_s *bus_config,
                    FAR const struct bmi160_low_level_operations_s *ll_operation)
{
  int ret;

  /* Sanity check */

  DEBUGASSERT(bus_config != NULL);
  DEBUGASSERT(ll_operation != NULL);
  DEBUGASSERT(ll_operation->bmi160_attach != NULL);
  DEBUGASSERT(ll_operation->bmi160_int_enable != NULL);


  /* Initialize the BMG160 device structure */

  g_bmi160 = (FAR struct bmi160_dev_s *)kmm_malloc(sizeof(struct bmi160_dev_s));
  if (g_bmi160 == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  g_bmi160->ocount = 0;
  /* Initialize device open semphone*/
  sem_init(&g_bmi160->closesem, 0, 1);
  /* Initialize millsecond delay semphone */
  sem_init(&g_bmi160->delaysem, 0, 0);
  /* Initialize ioctrl semphone */
  sem_init(&g_bmi160->ioctrlsem, 0, 1);

  g_bmi160->receive_pid = 0;
  g_bmi160->signo = 0;

  memcpy(&g_bmi160->bus, bus_config, sizeof(struct bmi160_bus_config_s));
  memcpy(&g_bmi160->low_level_op, ll_operation, sizeof(struct bmi160_low_level_operations_s));

  /* Setup SPI frequency and mode */

  if (g_bmi160->bus.bus_selection == BMI160_SPI_INTF)
    {
      g_bmi160->internal.interface = BMI160_SPI_INTF;
      g_bmi160->internal.read = bmi160_spi_read_reg;
      g_bmi160->internal.write = bmi160_spi_write_reg;
      g_bmi160->internal.delay_ms = bmi160_delay;
    }
  else
    {
      struct bmi160_i2c_bus_config_s *i2c_config;
      i2c_config = &g_bmi160->bus.bus_config.i2c_config;

      g_bmi160->internal.interface = BMI160_I2C_INTF;
      g_bmi160->internal.id = (uint8_t)i2c_config->config.address;
      g_bmi160->internal.read = bmi160_i2c_read_reg;
      g_bmi160->internal.write = bmi160_i2c_write_reg;
      g_bmi160->internal.delay_ms = bmi160_delay;
    }

  /* Attach the interrupt channel 1 handler */

  ret = g_bmi160->low_level_op.bmi160_attach(BMI160_INT_CHANNEL_1, bmi160_int_handler);
  if (ret < 0)
    {
      snerr("ERROR: Failed to attach interrupt\n");
      goto err_dev;
    }

  /* Attach the interrupt channel 2 handler */

  ret = g_bmi160->low_level_op.bmi160_attach(BMI160_INT_CHANNEL_2, bmi160_int_handler);
  if (ret < 0)
    {
      snerr("ERROR: Failed to attach interrupt\n");
      goto err_dev;
    }


  /* Register the character driver */

  ret = register_driver(devpath, &g_bmi160_fops, 0666, g_bmi160);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      goto err_dev;
    }

  return OK;

err_dev:
  sem_destroy(&g_bmi160->closesem);
  sem_destroy(&g_bmi160->delaysem);
  sem_destroy(&g_bmi160->ioctrlsem);
  kmm_free(g_bmi160);
  return ret;

}

#endif /* CONFIG_SPI && CONFIG_BMG160 */
