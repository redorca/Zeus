/****************************************************************************
 * drivers/sensors/tmp108.c
 * Character driver for the TI TMP108 Temperature Sensor
 *
 *   Copyright (C) 2011, 2013, 2016 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/tmp108.h>
#include <nuttx/random.h>

#if defined(CONFIG_I2C) && defined(CONFIG_TMP108_I2C)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_TMP108_I2C_FREQUENCY
#  define CONFIG_TMP108_I2C_FREQUENCY 400000
#endif

/* Centigrade to Fahrenheit conversion:  F = 9*C/5 + 32 */

#define B16_9DIV5  (9 * 65536 / 5)
#define B16_32     (32 * 65536)
#define TMP108_TMP_BITS 12

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
extern int tmp108_int_initialize(FAR struct tmp108_dev_s *priv);
extern int tmp108_dev_register(FAR struct tmp108_dev_s *priv);


/****************************************************************************
 * Private
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* I2C Helpers */

static int     tmp108_i2c_write(FAR struct tmp108_dev_s *priv,
                                FAR const uint8_t *buffer, int buflen);
static int     tmp108_i2c_read(FAR struct tmp108_dev_s *priv,
                               FAR uint8_t *buffer, int buflen);
static int     tmp108_readb16(FAR struct tmp108_dev_s *priv, uint8_t regaddr,
                              FAR b16_t *regvalue);
static int     tmp108_writeb16(FAR struct tmp108_dev_s *priv, uint8_t regaddr,
                               b16_t regval);
static int     tmp108_readtemp(FAR struct tmp108_dev_s *priv, FAR b16_t *temp);
static int     tmp108_readconf(FAR struct tmp108_dev_s *priv, FAR uint16_t *conf);
static int     tmp108_writeconf(FAR struct tmp108_dev_s *priv, uint16_t conf);

/* Character driver methods */

static int     tmp108_open(FAR struct file *filep);
static int     tmp108_close(FAR struct file *filep);
static ssize_t tmp108_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t tmp108_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int     tmp108_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

static void    tmp108_alert_handler(FAR struct tmp108_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_tmp108fops =
{
  tmp108_open,
  tmp108_close,
  tmp108_read,
  tmp108_write,
  NULL,
  tmp108_ioctl
#ifndef CONFIG_DISABLE_POLL
  , NULL
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tmp108_i2c_write
 *
 * Description:
 *   Write to the I2C device.
 *
 ****************************************************************************/

static int tmp108_i2c_write(FAR struct tmp108_dev_s *priv,
                            FAR const uint8_t *buffer, int buflen)
{
  struct i2c_msg_s msg;
  int ret;

  /* Setup for the transfer */

  msg.frequency = CONFIG_TMP108_I2C_FREQUENCY;
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = (FAR uint8_t *)buffer;  /* Override const */
  msg.length    = buflen;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: tmp108_i2c_read
 *
 * Description:
 *   Read from the I2C device.
 *
 ****************************************************************************/

static int tmp108_i2c_read(FAR struct tmp108_dev_s *priv,
                           FAR uint8_t *buffer, int buflen)
{
  struct i2c_msg_s msg;
  int ret;

  /* Setup for the transfer */

  msg.frequency = CONFIG_TMP108_I2C_FREQUENCY;
  msg.addr      = priv->addr;
  msg.flags     = I2C_M_READ;
  msg.buffer    = buffer;
  msg.length    = buflen;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: tmp108_readb16
 *
 * Description:
 *   Read a 16-bit register (TMP108_TEMP_REG, TMP108_TLOW_REG, or TMP108_THIGH_REG)
 *
 ****************************************************************************/

static int tmp108_readb16(FAR struct tmp108_dev_s *priv, uint8_t regaddr,
                          FAR b16_t *regvalue)
{
  uint8_t buffer[2];
  int ret;

  /* Write the register address */

  ret = tmp108_i2c_write(priv, &regaddr, 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 16-bits from the register (discarding 7) */

  ret = tmp108_i2c_read(priv, buffer, 2);
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  /* Data format is:  TTTTTTTT TTTTxxxx where TTTTTTTTTTTT is a 12-bit,
   * signed temperature value with LSB = 0.0625 degrees centigrade.  So the
   * raw data is b8_t
   */

  *regvalue = b8tob16(buffer[0] << 8 | (b8_t)buffer[1]);
  sninfo("addr: %02x value: 0x%08x ret: %d\n", regaddr, *regvalue, ret);
  return OK;
}

/****************************************************************************
 * Name: tmp108_writeb16
 *
 * Description:
 *   Write to a 16-bit register (TMP108_TEMP_REG, TMP108_THYS_REG, or TMP108_TOS_REG)
 *
 ****************************************************************************/

static int tmp108_writeb16(FAR struct tmp108_dev_s *priv, uint8_t regaddr,
                           b16_t regval)
{
  uint8_t buffer[3];
  b8_t regb8;

  sninfo("addr: %02x value: %08x\n", regaddr, regval);

  /* Set up a 3 byte message to send */

  buffer[0] = regaddr;

  regb8 = b16tob8(regval);
  buffer[1] = (uint8_t)(regb8 >> 8);
  buffer[2] = (uint8_t)regb8;

  /* Write the register address followed by the data (no RESTART) */

  return tmp108_i2c_write(priv, buffer, 3);
}

/****************************************************************************
 * Name: tmp108_readtemp
 *
 * Description:
 *   Read the temperature register with special scaling (TMP108_TEMP_REG)
 *
 ****************************************************************************/

static int tmp108_readtemp(FAR struct tmp108_dev_s *priv, FAR b16_t *temp)
{
  b16_t temp16;
  int16_t temp16_1 = 0;
  int ret;
  int sign = 1;

  /* Read the raw temperature data (b16_t) */
  ret = tmp108_readb16(priv, TMP108_TEMP_REG, &temp16);
  if (ret < 0)
    {
      snerr("ERROR: tmp108_readb16 failed: %d\n", ret);
      return ret;
    }


  /* Find the sign of the temperature read . It is the 12th bit */
  if (((temp16 >> TMP108_TMP_BITS) & 0x0800) == 0x0800)
    {
      sign = -1;
    }
  temp16 = (int16_t)((temp16 >> TMP108_TMP_BITS) & ~(int16_t)0x0800);


  /* Do sign extension and set the temp correctly */
  if (sign == -1)
    {
      temp16_1 = (temp16 | 0xF800); /* sign extension by masking top 5 bits */
    }
  else
    {
      temp16_1 = temp16;
    }

  *temp = temp16_1;
  return OK;
}

/****************************************************************************
 * Name: tmp108_readconf
 *
 * Description:
 *   Read the 8-bit TMP108 configuration register
 *
 ****************************************************************************/

static int tmp108_readconf(FAR struct tmp108_dev_s *priv, FAR uint16_t *conf)
{
  uint8_t buffer;
  int ret;

  /* Write the configuration register address */

  buffer = TMP108_CONF_REG;

  ret = tmp108_i2c_write(priv, &buffer, 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 16-bits from the register */
  ret = tmp108_i2c_read(priv, (uint8_t *)conf, 2);
  sninfo("conf: %02x ret: %d\n", *conf, ret);
  return ret;
}

/****************************************************************************
 * Name: tmp108_writeconf
 *
 * Description:
 *   Write to a 16-bit TMP108 configuration register.
 *
 ****************************************************************************/

static int tmp108_writeconf(FAR struct tmp108_dev_s *priv, uint16_t conf)
{
  uint8_t buffer[3];

  sninfo("conf: %02x\n", conf);

  /* Set up a 2 byte message to send */

  buffer[0] = TMP108_CONF_REG;
  buffer[1] = conf & 0x00ff;
  buffer[2] = (conf >> 8) & 0xff;

  /* Write the register address followed by the data (no RESTART) */

  return tmp108_i2c_write(priv, buffer, 3);
}

/****************************************************************************
 * Name: tmp108_open
 *
 * Description:
 *   This function is called whenever the TMP108 device is opened.
 *
 ****************************************************************************/

static int tmp108_open(FAR struct file *filep)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct tmp108_dev_s *priv  = inode->i_private;
  int ret   = OK;
  uint16_t cfg_val = 0;
  ret = tmp108_readconf(priv, &cfg_val);
  if (ret == OK)
    {
      ret = tmp108_writeconf(priv, (cfg_val | TMP108_CONF_OP_MODE));
      if (ret != OK)
        {
          snerr(LOG_ERR, "TMP108 open write config failed: %d\n", ret);
          return ret;
        }
      sninfo("Close TMP108:cfg_value = %02x ret: %d\n", cfg_val & ~TMP108_CONF_OP_MODE, ret);
    }
  else
    {
      snerr(LOG_ERR, "TMP108 open read config failed: %d\n", ret);
    }

  tmp108_int_initialize(priv);

  return ret;
}

/****************************************************************************
 * Name: tmp108_close
 *
 * Description:
 *   This routine is called when the TMP108 device is closed.
 *
 ****************************************************************************/

static int tmp108_close(FAR struct file *filep)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct tmp108_dev_s *priv  = inode->i_private;
  int ret   = OK;
  uint16_t cfg_val = 0;

  ret = tmp108_readconf(priv, &cfg_val);
  if (ret == OK)
    {
      ret = tmp108_writeconf(priv, cfg_val & ~TMP108_CONF_OP_MODE);
      if (ret != OK)
        {
          snerr(LOG_ERR, "TMP108 close write config failed: %d\n", ret);
          return ret;
        }
      sninfo("Close TMP108:cfg_value = %02x ret: %d\n", cfg_val & ~TMP108_CONF_OP_MODE, ret);
    }
  else
    {
      snerr(LOG_ERR, "TMP108 close read config failed: %d\n", ret);
    }


  return ret;
}

/****************************************************************************
 * Name: tmp108_read
 ****************************************************************************/

static ssize_t tmp108_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct tmp108_dev_s *priv   = inode->i_private;
  FAR b16_t             *ptr;
  ssize_t                nsamples;
  int                    i;
  int                    ret;


  /* How many samples were requested to get? */
  nsamples = buflen / sizeof(b16_t);
  ptr      = (FAR b16_t *)buffer;

  sninfo("buflen: %d nsamples: %d\n", buflen, nsamples);

  for (i = 0; i < nsamples; i++)
    {
      b16_t temp = 0;

      /* Read the next b16_t temperature value */

      ret = tmp108_readtemp(priv, &temp);
      if (ret < 0)
        {
          snerr("ERROR: tmp108_readtemp failed: %d\n", ret);
          return (ssize_t)ret;
        }

      /* Save the temperature value in the user buffer */

      *ptr++ = temp;
    }

  return nsamples * sizeof(b16_t);
}

/****************************************************************************
 * Name: tmp108_write
 ****************************************************************************/

static ssize_t tmp108_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: tmp108_ioctl
 ****************************************************************************/

static int tmp108_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct tmp108_dev_s *priv  = inode->i_private;
  int                    ret   = OK;
  uint16_t               conf = 0;

  switch (cmd)
    {
      /* Read from the configuration register. Arg: uint8_t* pointer */

      case SNIOC_READCONF:
        {
          FAR uint16_t *ptr = (FAR uint16_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          ret = tmp108_readconf(priv, ptr);
          sninfo("conf: %02x ret: %d\n", *ptr, ret);
        }
        break;

      /* Wrtie to the configuration register. Arg:  uint8_t value */
      case SNIOC_WRITECONF:
        ret = tmp108_writeconf(priv, (uint16_t)arg);
        sninfo("conf: %02x ret: %d\n", *(FAR uint16_t *)arg, ret);
        break;

      /* Shutdown the TMP108, Arg: None */
      case SNIOC_SHUTDOWN:
        {
          ret = tmp108_readconf(priv, &conf);
          if (ret == OK)
            {
              ret = tmp108_writeconf(priv, conf & ~TMP108_CONF_OP_MODE);
            }

          sninfo("conf: %02x ret: %d\n", conf & ~TMP108_CONF_OP_MODE, ret);
        }
        break;

      /* Powerup the TMP108, Arg: None */
      case SNIOC_POWERUP:
        {
          ret = tmp108_readconf(priv, &conf);
          if (ret == OK)
            {
              ret = tmp108_writeconf(priv, conf | TMP108_CONF_ONE_SHOT_MODE);
            }

          sninfo("conf: %02x ret: %d\n", conf | TMP108_CONF_ONE_SHOT_MODE, ret);
        }
        break;

      /* Report samples in Fahrenheit */

      case SNIOC_FAHRENHEIT:
        priv->fahrenheit = true;
        sninfo("Fahrenheit\n");
        break;

      /* Report Samples in Centigrade */

      case SNIOC_CENTIGRADE:
        priv->fahrenheit = false;
        sninfo("Centigrade\n");
        break;

      /* Read THYS temperature register.  Arg: b16_t* pointer */

      case SNIOC_READTHYS:
        {
          FAR b16_t *ptr = (FAR b16_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          ret = tmp108_readb16(priv, TMP108_CONF_REG, ptr);
          sninfo("THYS: %08x ret: %d\n", *ptr, ret);
        }
        break;

      /* Write THYS temperature register. Arg: b16_t value */

      case SNIOC_WRITETHYS:
        ret = tmp108_readconf(priv, &conf);
        if (ret == OK)
          {
            ret = tmp108_writeconf(priv, conf | TMP108_CONF_ONE_SHOT_MODE);
          }
        ret = tmp108_writeb16(priv, TMP108_CONF_REG, (b16_t)arg);
        sninfo("THYS: %08x ret: %d\n", (b16_t)arg, ret);
        break;

      /* Read TLOW (Lower limit temp) Register. Arg: b16_t* pointer */
      case SNIOC_READTLOW:
        {
          FAR b16_t *ptr = (FAR b16_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          ret = tmp108_readb16(priv, TMP108_TLOW_REG, ptr);
          sninfo("TOS: %08x ret: %d\n", *ptr, ret);
        }
        break;

      /* Write TLOW (Lower limit temp) Register. Arg: b16_t* pointer */
      case SNIOC_WRITETLOW:
        ret = tmp108_writeb16(priv, TMP108_TLOW_REG, (b16_t)arg);
        sninfo("TOS: %08x ret: %d\n", (b16_t)arg, ret);
        break;

      /* Read THIGH (Higher limit temp) Register. Arg: b16_t* pointer */
      case SNIOC_READTHIGH:
        {
          FAR b16_t *ptr = (FAR b16_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          ret = tmp108_readb16(priv, TMP108_THIGH_REG, ptr);
          sninfo("TOS: %08x ret: %d\n", *ptr, ret);
        }
        break;

      /* Write THIGH (Higher limit temp) Register. Arg: b16_t* pointer */
      case SNIOC_WRITETHIGH:
        ret = tmp108_writeb16(priv, TMP108_THIGH_REG, (b16_t)arg);
        sninfo("TOS: %08x ret: %d\n", (b16_t)arg, ret);
        break;

      case SNIOC_GA_REGISTER_INT:
        {
          DEBUGASSERT(GOOD_SIGNO(arg));
          priv->receive_pid   = getpid();
          priv->signo = (uint8_t)arg;
        }
        break;

      case SNIOC_GA_UNREGISTER_INT:
        {
          priv->receive_pid   = 0;
          priv->signo = 0;
        }
        break;

      default:
        sninfo("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

static void tmp108_alert_handler(FAR struct tmp108_dev_s *priv)
{
  union sigval value;

  if (priv->receive_pid != 0)
    {
      (void)sigqueue(priv->receive_pid, priv->signo, value);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tmp108_register
 *
 * Description:
 *   Register the tmp108  character device as 'devpath'
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int tmp108_register(FAR const char *devpath, FAR struct i2c_master_s *i2c, uint8_t addr, uint8_t pin)
{
  FAR struct tmp108_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);
  DEBUGASSERT(addr == CONFIG_TMP108_ADDR0 || addr == CONFIG_TMP108_ADDR1 ||
              addr == CONFIG_TMP108_ADDR2 || addr == CONFIG_TMP108_ADDR3);


  /* Initialize the tmp108 device structure */

  priv = (FAR struct tmp108_dev_s *)kmm_malloc(sizeof(struct tmp108_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c        = i2c;
  priv->addr       = addr;
  priv->alert_pin  = pin;
  priv->alert_cb   = tmp108_alert_handler;
  priv->fahrenheit = false;

  priv->receive_pid = 0;
  priv->signo = 0;

  /* Register the character driver */

  /* Register device information */
  tmp108_dev_register(priv);

  ret = register_driver(devpath, &g_tmp108fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}
#endif /* CONFIG_I2C && CONFIG_TMP108_I2C */
