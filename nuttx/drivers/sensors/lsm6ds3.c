/****************************************************************************
 * drivers/sensors/lsm6ds3.c
 * Character driver for the lsm6ds3 6-Axis comb device.
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
#include <nuttx/sensors/lsm6ds3.h>

#ifdef CONFIG_LSM6DS3

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define LSM6DS3_RW_REG_BUFFER_MAX_LENGTH 16

/****************************************************************************
 * Private type
 ****************************************************************************/
struct lsm6ds3_dev_s
{

  FAR struct lsm6ds3_bus_config_s bus;

  uint8_t ocount; /* The number of times the device has been opened */
  sem_t closesem; /* Locks out new opens while close is in progress */
  sem_t ioctrlsem; /* Locks out new IOCtl operation while the old one is in progress */
  sem_t delaysem;/*this semphone is initialize as 0 and never post,
                  *so we can implement millseconds delay using the
                  *sem_timedwait function.
                  */
  uint8_t        signo; /* signo to use when signaling a interrupt */
  pid_t          receive_pid; /* The task to be signalled */
};



/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int lsm6ds3_open(FAR struct file *filep);
static int lsm6ds3_close(FAR struct file *filep);
static ssize_t lsm6ds3_read(FAR struct file *, FAR char *, size_t);
static ssize_t lsm6ds3_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int lsm6ds3_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_lsm6ds3_fops =
{
  lsm6ds3_open,
  lsm6ds3_close,
  lsm6ds3_read,
  lsm6ds3_write,
  NULL,
  lsm6ds3_ioctl
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
 * Name: lsm6ds3_i2c_read_reg
 ****************************************************************************/
static int lsm6ds3_i2c_read_reg(struct lsm6ds3_dev_s *dev, uint8_t reg_addr,
                                uint8_t *data, uint16_t len)
{
  return i2c_writeread(dev->bus.bus_config.i2c_config.i2c,
                       &dev->bus.bus_config.i2c_config.config,
                       &reg_addr, 1,
                       data, len);

}

/****************************************************************************
 * Name: lsm6ds3_i2c_write_reg
 ****************************************************************************/
static int lsm6ds3_i2c_write_reg(struct lsm6ds3_dev_s *dev, uint8_t reg_addr,
                                 uint8_t *data, uint16_t len)
{
  uint8_t buffer[LSM6DS3_RW_REG_BUFFER_MAX_LENGTH];

  if (len >  (LSM6DS3_RW_REG_BUFFER_MAX_LENGTH - 1))
    {
      len = LSM6DS3_RW_REG_BUFFER_MAX_LENGTH - 1;
    }

  buffer[0] = reg_addr;
  memcpy(&buffer[1], data, len);

  return i2c_write(dev->bus.bus_config.i2c_config.i2c,
                   &dev->bus.bus_config.i2c_config.config,
                   buffer, (len + 1));

}

/****************************************************************************
 * Name: lsm6ds3_spi_read_reg
 ****************************************************************************/
static int lsm6ds3_spi_read_reg(struct lsm6ds3_dev_s *dev, uint8_t reg_addr,
                                uint8_t *data, uint16_t len)
{
  struct lsm6ds3_spi_bus_config_s *cfg;

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
  SPI_SEND(cfg->spi, 0x80 | reg_addr);

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
 * Name: lsm6ds3_spi_write_reg
 ****************************************************************************/
static int lsm6ds3_spi_write_reg(struct lsm6ds3_dev_s *dev, uint8_t reg_addr,
                                 uint8_t *data, uint16_t len)
{
  struct lsm6ds3_spi_bus_config_s *cfg;

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
 * Name: lsm6ds3_init
 ****************************************************************************/
static mems_status_t lsm6ds3_init(struct lsm6ds3_dev_s *dev)
{
  mems_status_t rslt;
  uint8_t id = 0;
  uint8_t try = 3;

  /* Null-pointer check */
  DEBUGASSERT(dev != NULL);

  while ((try--) && (id != LSM6DS3_ACC_GYRO_WHO_AM_I))
      {
        /* Read chip_id */
        rslt = LSM6DS3_ACC_GYRO_R_WHO_AM_I((void *)dev, &id);
      }

  if (rslt == MEMS_SUCCESS)
    {
      if (id == LSM6DS3_ACC_GYRO_WHO_AM_I)
        {
          /* Soft reset */
          rslt = LSM6DS3_ACC_GYRO_W_SW_RESET((void *)dev, LSM6DS3_ACC_GYRO_SW_RESET_RESET_DEVICE);

          /*wait for the reset operation finished*/
          up_udelay(50);

        }
      else
        {
          snerr("ERROR: lsm6ds3_init lsm6ds3 device not found\n");
          rslt = MEMS_ERROR;
        }
    }

  return rslt;
}

/****************************************************************************
 * Name: lsm6ds3_set_acc_parameter
 ****************************************************************************/
static mems_status_t lsm6ds3_set_acc_parameter(struct lsm6ds3_dev_s *dev, sn_ga_param_s *parameter)
{
  mems_status_t rslt = MEMS_SUCCESS;
  LSM6DS3_ACC_GYRO_FS_XL_t fs = LSM6DS3_ACC_GYRO_FS_XL_4g;
  LSM6DS3_ACC_GYRO_ODR_XL_t odr = LSM6DS3_ACC_GYRO_ODR_XL_POWER_DOWN;
  LSM6DS3_ACC_GYRO_LP_XL_t lp = LSM6DS3_ACC_GYRO_LP_XL_DISABLED;

  switch (parameter->range)
    {
      case 2:
        fs = LSM6DS3_ACC_GYRO_FS_XL_2g;
        break;
      case 4:
        fs = LSM6DS3_ACC_GYRO_FS_XL_4g;
        break;
      case 8:
        fs = LSM6DS3_ACC_GYRO_FS_XL_8g;
        break;
      case 16:
        fs = LSM6DS3_ACC_GYRO_FS_XL_16g;
        break;
      default:
        fs = LSM6DS3_ACC_GYRO_FS_XL_4g;
        snwarn("WARN: lsm6ds3_set_acc_parameter unsupported FS parameter.Use default 4g\n");
        break;
    }

  switch (parameter->odr)
    {
      case 130:
        odr = LSM6DS3_ACC_GYRO_ODR_XL_13Hz;
        break;
      case 260:
        odr = LSM6DS3_ACC_GYRO_ODR_XL_26Hz;
        break;
      case 520:
        odr = LSM6DS3_ACC_GYRO_ODR_XL_52Hz;
        break;
      case 1040:
        odr = LSM6DS3_ACC_GYRO_ODR_XL_104Hz;
        break;
      case 2080:
        odr = LSM6DS3_ACC_GYRO_ODR_XL_208Hz;
        break;
      case 4160:
        odr = LSM6DS3_ACC_GYRO_ODR_XL_416Hz;
        break;
      case 8330:
        odr = LSM6DS3_ACC_GYRO_ODR_XL_833Hz;
        break;
      case 16600:
        odr = LSM6DS3_ACC_GYRO_ODR_XL_1660Hz;
        break;
      case 33300:
        odr = LSM6DS3_ACC_GYRO_ODR_XL_3330Hz;
        break;
      default:
        odr = LSM6DS3_ACC_GYRO_ODR_XL_52Hz;
        snwarn("WARN: lsm6ds3_set_acc_parameter unsupported odr parameter.Use default 52Hz\n");
        break;
    }

  switch (parameter->power_mode)
    {
      case GA_MODE_SUSPEND:
        odr = LSM6DS3_ACC_GYRO_ODR_XL_POWER_DOWN;
        break;
      case GA_MODE_LOWPOWER:
        lp = LSM6DS3_ACC_GYRO_LP_XL_ENABLED;
        break;
      case GA_MODE_NORMAL:
        lp = LSM6DS3_ACC_GYRO_LP_XL_DISABLED;
        break;
      default:
        snwarn("WARN: lsm6ds3_set_acc_parameter unsupported power mode.Use power dowm mode\n");
        break;

    }


  rslt = LSM6DS3_ACC_GYRO_W_LowPower_XL((void *)dev, lp);
  rslt += LSM6DS3_ACC_GYRO_W_ODR_XL((void *)dev, odr);
  rslt += LSM6DS3_ACC_GYRO_W_FS_XL((void *)dev, fs);

  if (rslt == 3 * MEMS_SUCCESS)
    {
      rslt = MEMS_SUCCESS;
    }
  else
    {
      rslt = MEMS_ERROR;
    }

  return rslt;
}

/****************************************************************************
 * Name: lsm6ds3_set_acc_parameter
 ****************************************************************************/
static mems_status_t lsm6ds3_set_int_config(struct lsm6ds3_dev_s *dev,
                                            struct lsm6ds3_int_settg *int_cfg)
{
  struct lsm6ds_acc_6D_4D_int_cfg *cfg_6d = &int_cfg->int_type_cfg.acc_6D_4D_int;
  LSM6DS3_ACC_GYRO_SIXD_THS_t ths = LSM6DS3_ACC_GYRO_SIXD_THS_80_degree;
  mems_status_t rslt;

  if (int_cfg->int_type == LSMD6S3_ACC_6D_ORIENTATION_INT)
    {
      switch (cfg_6d->threshold)
        {
          case 50:
            ths = LSM6DS3_ACC_GYRO_SIXD_THS_50_degree;
            break;
          case 60:
            ths = LSM6DS3_ACC_GYRO_SIXD_THS_60_degree;
            break;
          case 70:
            ths = LSM6DS3_ACC_GYRO_SIXD_THS_70_degree;
            break;
          case 80:
            ths = LSM6DS3_ACC_GYRO_SIXD_THS_80_degree;
            break;
          default:
            ths = LSM6DS3_ACC_GYRO_SIXD_THS_60_degree;
            break;
        }

      LSM6DS3_ACC_GYRO_W_SIXD_THS((void *)dev, ths);

      if (cfg_6d->use_LPF2 == true)
        {
          LSM6DS3_ACC_GYRO_W_SLOPE_FDS((void *)dev, LSM6DS3_ACC_GYRO_SLOPE_FDS_ENABLED);
          LSM6DS3_ACC_GYRO_W_LOW_PASS_ON_6D((void *)dev, LSM6DS3_ACC_GYRO_LOW_PASS_ON_6D_ON);
        }

      LSM6DS3_ACC_GYRO_W_6DEvOnInt1((void *)dev, LSM6DS3_ACC_GYRO_INT1_6D_ENABLED);

      LSM6DS3_ACC_GYRO_W_LIR((void *)dev, LSM6DS3_ACC_GYRO_LIR_ENABLED);

      rslt  = MEMS_SUCCESS;
    }
  else
    {
      rslt = MEMS_ERROR;
    }

  return rslt;
}

/****************************************************************************
 * Name: lsm6ds3_get_int_status
 ****************************************************************************/
static mems_status_t lsm6ds3_get_int_status(struct lsm6ds3_dev_s *dev,
                                            uint32_t *int_cfg)
{
  mems_status_t rslt;
  LSM6DS3_ACC_GYRO_D6D_EV_STATUS_t orien_6D_status;
  uint32_t status = 0;

  /*acquire 6D orientation interrupt status*/
  rslt = LSM6DS3_ACC_GYRO_R_D6D_EV_STATUS((void *)dev, &orien_6D_status);

  if (rslt == MEMS_ERROR)
    {
      return MEMS_ERROR;
    }

  /*todo:acquire all interrupt status*/

  if (orien_6D_status == LSM6DS3_ACC_GYRO_D6D_EV_STATUS_DETECTED)
    {
      status |= LSMD6S3_6D_ORIENTATION_INT_MASK;
    }

  /*ckeck all the other interrupt source*/

  *int_cfg = status;

  return MEMS_SUCCESS;
}

/****************************************************************************
 * Name: lsm6ds3_open
 ****************************************************************************/

static int lsm6ds3_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct lsm6ds3_dev_s *priv;
  uint8_t tmp;
  mems_status_t ret_ll;
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
              ret_ll = lsm6ds3_init(priv);
              if (ret_ll == MEMS_SUCCESS)
                {
                  priv->ocount = tmp;
                }
              else
                {
                  snerr("ERROR: lsm6ds3_init failed: %d\n", ret);
                  ret = -EIO;
                }
            }
        }

      sem_post(&priv->closesem);
    }

  return ret;
}

/****************************************************************************
 * Name: lsm6ds3_close
 ****************************************************************************/

static int lsm6ds3_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct lsm6ds3_dev_s *dev;
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

          LSM6DS3_ACC_GYRO_W_SW_RESET((void *)dev, LSM6DS3_ACC_GYRO_SW_RESET_RESET_DEVICE);
        }
      sem_post(&dev->closesem);
    }

  return ret;
}

/****************************************************************************
 * Name: bmg160_read
 ****************************************************************************/

static ssize_t lsm6ds3_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  return 0;
}

/****************************************************************************
 * Name: bmg160_write
 ****************************************************************************/

static ssize_t lsm6ds3_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: bmg160_ioctl
 ****************************************************************************/

static int lsm6ds3_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  struct lsm6ds3_dev_s *dev;
  mems_status_t ret_ll;
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
              ret_ll = LSM6DS3_ACC_GYRO_R_WHO_AM_I((void *)dev, ptr);
              if (ret_ll != MEMS_SUCCESS)
                {
                  ret = -EIO;
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
                  ret_ll = lsm6ds3_set_acc_parameter((void *)dev, param);

                  if (ret_ll != MEMS_SUCCESS)
                    {
                      snerr("ERROR: LSM6DS3_set_acc_parameter failed: %d\n", ret_ll);
                      ret = -EIO;
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
              struct lsm6ds3_int_settg *config = (struct lsm6ds3_int_settg *)arg;

              if (config != NULL)
                {
                  ret_ll = lsm6ds3_set_int_config(dev, config);
                  if (ret_ll != MEMS_SUCCESS)
                    {
                      snerr("ERROR: lsm6ds3_set_int_config failed: %d\n", ret_ll);
                      ret = -EIO;
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

          /* Command:     SNIOC_GA_GET_INT_STATUS
           * Description: get the interrupt status.
           * Argument:    pointer to the variable which will hold the interrupt status.
           *              If a interrupt is triggered, the corresponding bits will be set to 1.
           */

          case SNIOC_GA_GINTSTATUS:
            {
              uint32_t *int_status = (uint32_t *)arg;

              if (int_status != NULL)
                {
                  ret_ll = lsm6ds3_get_int_status(dev, int_status);

                  if (ret_ll != MEMS_SUCCESS)
                    {
                      snerr("ERROR: lsm6ds3_ioctl failed: %d\n", ret_ll);
                      ret = -EIO;
                    }
                }
              else
                {
                  ret = -EINVAL;
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
mems_status_t LSM6DS3_ACC_GYRO_WriteReg(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
  struct lsm6ds3_dev_s *dev = (struct lsm6ds3_dev_s *)handle;
  int ret;
  mems_status_t ll_ret;

  if (dev->bus.bus_selection == LSM6DS3_I2C_INTF)
    {
      ret = lsm6ds3_i2c_write_reg(dev, WriteAddr, pBuffer, nBytesToWrite);
    }
  else
    {
      ret = lsm6ds3_spi_write_reg(dev, WriteAddr, pBuffer, nBytesToWrite);
    }


  if (ret == OK)
    {
      ll_ret = MEMS_SUCCESS;
    }
  else
    {
      ll_ret = MEMS_ERROR;
    }

  return ll_ret;
}


mems_status_t LSM6DS3_ACC_GYRO_ReadReg(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
  struct lsm6ds3_dev_s *dev = (struct lsm6ds3_dev_s *)handle;
  int ret;
  mems_status_t ll_ret;


  if (dev->bus.bus_selection == LSM6DS3_I2C_INTF)
    {
      ret = lsm6ds3_i2c_read_reg(dev, ReadAddr, pBuffer, nBytesToRead);
    }
  else
    {
      ret = lsm6ds3_spi_read_reg(dev, ReadAddr, pBuffer, nBytesToRead);
    }

  if (ret == OK)
    {
      ll_ret = MEMS_SUCCESS;
    }
  else
    {
      ll_ret = MEMS_ERROR;
    }

  return ll_ret;

}



/****************************************************************************
 * Name: lsm6ds3_register
 *
 * Description:
 *   Register the LSM6DS3 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath -    The full path to the driver to register. E.g., "/dev/lsm6ds3"
 *   bus_config - An struct describing the bus which use to communicate with LSM6DS3
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/
int lsm6ds3_register(FAR const char *devpath,
                     FAR struct lsm6ds3_bus_config_s *bus_config)
{
  int ret;
  struct lsm6ds3_dev_s *dev;

  /* Sanity check */

  DEBUGASSERT(bus_config != NULL);


  /* Initialize the BMG160 device structure */

  dev = (FAR struct lsm6ds3_dev_s *)kmm_malloc(sizeof(struct lsm6ds3_dev_s));
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

  memcpy(&dev->bus, bus_config, sizeof(struct lsm6ds3_bus_config_s));

  /* Register the character driver */

  ret = register_driver(devpath, &g_lsm6ds3_fops, 0666, dev);
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

#endif /* CONFIG_LSM6DS3 */
