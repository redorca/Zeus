/****************************************************************************
 * drivers/sensors/max86140.c
 * Character driver for the max86140 heart rate
 *
 *   Copyright (C) 2018 Zglue
 *   Author: Min Yang <min.yang@zglue.com>
 *
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
#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>
#include <string.h>


#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/ioctl.h>
#include <nuttx/sensors/max86140.h>
#include <nrf52_gpio.h>
#include "chip.h"

#if defined(CONFIG_MAX86140)

/* MAX86140 Registers */
#define HRAFE_PARTID      0xFF
#define HRAFE_SYSTEM_CTL      0x0D
#define HRAFE_LED_SEQ_REG1    0x20
#define HRAFE_LED_SEQ_REG2    0x21
#define HRAFE_LED_SEQ_REG3    0x22
#define HRAFE_PPG_CONFIG0    0x10
#define HRAFE_PPG_CONFIG1    0x11
#define HRAFE_PPG_CONFIG2    0x12
#define HRAFE_PPG_CONFIG3    0x13
#define HRAFE_PHOTO_DIODE_BIAS    0x15
#define HRAFE_LED_RNG1    0x2A
#define HRAFE_LED1_PA    0x23
#define HRAFE_LED2_PA    0x24
#define HRAFE_LED3_PA    0x25
#define HRAFE_FIFO_CONFIG1    0x09
#define HRAFE_FIFO_CONFIG2    0x0A

/* MAX86140 SPI READ/WRITE value */
#define HRAFE_REG_READ    0x80
#define HRAFE_REG_WRITE   0x00

#define HRAFE_INT1_STATUS    0x00
#define HRAFE_INT2_STATUS    0x01
#define HRAFE_INT1_ENABLE    0x02
#define HRAFE_INT2_ENABLE    0x03

#define HRAFE_MAX86140_PARTID     0x24
#define HRAFE_MAX86141_PARTID     0x25

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
extern int max86140_int_initialize(FAR struct max86140_dev_s *priv);
extern int max86140_dev_register(FAR struct max86140_dev_s *priv);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int max86140_open(FAR struct file *filep);
static int max86140_close(FAR struct file *filep);
static ssize_t max86140_read(FAR struct file *filep, FAR char *buffer, size_t buflen);
static ssize_t max86140_write(FAR struct file *filep, FAR const char *buffer, size_t buflen);
static ssize_t max86140_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_max86140_ops =
{
  max86140_open,                  /* open */
  max86140_close,                 /* close */
  max86140_read,                  /* read */
  max86140_write,                 /* write */
  0,                            /* seek */
  max86140_ioctl,                 /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  0,                            /* poll */
#endif
  0                             /* unlink */
};

/****************************************************************************
 * Name: max86140_read_register
 *
 * Description:
 *   Read an 8 bit register from the MAX86140 sensor.
 *
 * Return: one byte read data
 *
 ****************************************************************************/
static uint8_t max86140_read_register(FAR struct max86140_dev_s *priv, uint8_t address)
{
  uint8_t data_s[2];
  data_s[0] = address;
  data_s[1] = HRAFE_REG_READ;

  uint8_t data_r;

  /* If SPI bus is shared then lock and configure it */
  (void)SPI_LOCK(priv->spi, true);

  /* set the SPI frequency */
  SPI_SETFREQUENCY(priv->spi, priv->spi_frequency);

  /* Select the Heart rate device */
  SPI_SELECT(priv->spi, priv->config.spi_devid, true);

  /*Send*/
  SPI_SNDBLOCK(priv->spi, &data_s, 2);

  /*Receive*/
  SPI_RECVBLOCK(priv->spi, &data_r, 1);

  /* Deselect the Heart rate device */
  SPI_SELECT(priv->spi, priv->config.spi_devid, false);

  /* Unlock bus */
  (void)SPI_LOCK(priv->spi, false);

  return data_r;
}

/****************************************************************************
 * Name: max86140_write_register
 *
 * Description:
 *   Write an 8 bit register to the MAX86140 sensor.
 *
 ****************************************************************************/
static void max86140_write_register(FAR struct max86140_dev_s *priv, uint8_t address, uint8_t data)
{
  /*The format of write register for max86140 is address,write,data*/
  uint8_t data_s[3];
  data_s[0] = address;
  data_s[1] = HRAFE_REG_WRITE;
  data_s[2] = data;

  /* If SPI bus is shared then lock and configure it */
  (void)SPI_LOCK(priv->spi, true);

  /* set the SPI frequency */
  SPI_SETFREQUENCY(priv->spi, priv->spi_frequency);

  /* Select the Heart rate device */
  SPI_SELECT(priv->spi, priv->config.spi_devid, true);

  /*Send*/
  SPI_SNDBLOCK(priv->spi, &data_s, 3);

  /* Deselect the Heart rate device */
  SPI_SELECT(priv->spi, priv->config.spi_devid, false);

  /* Unlock bus */
  (void)SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: max86140_read_fifo
 *
 * Description:
 *   Read 3 bytes of data from MAX86140 FIFO.
 *
 ****************************************************************************/
static void max86140_read_fifo(FAR struct max86140_dev_s *priv, uint8_t address, uint8_t *data_r)
{
  uint8_t data_s[2];
  data_s[0] = address;
  data_s[1] = HRAFE_REG_READ;

  /* If SPI bus is shared then lock and configure it */
  (void)SPI_LOCK(priv->spi, true);

  /* set the SPI frequency */
  SPI_SETFREQUENCY(priv->spi, priv->spi_frequency);

  /* Select the Heart rate device */
  SPI_SELECT(priv->spi, priv->config.spi_devid, true);

  /*Send*/
  SPI_SNDBLOCK(priv->spi, &data_s, 2);

  /*Receive*/
  SPI_RECVBLOCK(priv->spi, data_r, 3);

  /* Deselect the Heart rate device */
  SPI_SELECT(priv->spi, priv->config.spi_devid, false);

  /* Unlock bus */
  (void)SPI_LOCK(priv->spi, false);

}

/****************************************************************************
 * Name: max86140_init
 *
 * Description:
 *   Initialize the max86140 sensor
 *
 ****************************************************************************/
int max86140_init(FAR struct max86140_dev_s *priv)
{
  /*Read chip id*/
  uint8_t ret = 0;
  ret = max86140_read_register(priv, HRAFE_PARTID);
  if (ret != HRAFE_MAX86140_PARTID)
    {
      snerr("ERROR: Chip Max86140 part id[%x] not equal to [%x] \r\n", ret, HRAFE_MAX86140_PARTID);
      return -1;
    }

  /*Reset the device*/
  max86140_write_register(priv, HRAFE_SYSTEM_CTL, 0x01);
  up_udelay(1000); //delay 1ms

  /*Set LED sequence*/
  max86140_write_register(priv, HRAFE_LED_SEQ_REG1, 0x21);
  max86140_write_register(priv, HRAFE_LED_SEQ_REG2, 0x03);
  max86140_write_register(priv, HRAFE_LED_SEQ_REG3, 0x00);

  max86140_write_register(priv, HRAFE_PPG_CONFIG0, 0x04);

  /*set adc range 16uA, pulse width 128.3 us*/
  max86140_write_register(priv, HRAFE_PPG_CONFIG1, 0x0F);

  /*set sps, averaging
   *PPG Configuration 2 (0x12)
   *Field PPG_SR[4:0] SMP_AVE[2:0]
   *Reset       0x11          0x0
   */
  max86140_write_register(priv, HRAFE_PPG_CONFIG2, 0x88);

  /*set settling time 12us
   *PPG Configuration 3 (0x13)
   *Field LED_SETLNG[1:0] DIG_FILT_SEL â€â€BURST_RATE[1:0] BURST_EN
   *Reset            0x03       0x0     â€â€           0x0    0x0
   *Access Type Write, Read Write, Read â€â€Write, Read Write, Read
  */
  max86140_write_register(priv, HRAFE_PPG_CONFIG3, 0xC0);

  /*set photdiode capacitance*/
  max86140_write_register(priv, HRAFE_PHOTO_DIODE_BIAS, 0x05);

  /*set LEDs range to 124mA range*/
  max86140_write_register(priv, HRAFE_LED_RNG1, 0x3F);

  /*set LEDs current*/
  max86140_write_register(priv, HRAFE_LED1_PA, 0x09);
  max86140_write_register(priv, HRAFE_LED2_PA, 0x09);
  max86140_write_register(priv, HRAFE_LED3_PA, 0x28);

  /*set to read fifo interupt status when its at half full (128-64(0x34))*/
  /*register (0x09) sets the watermark for the FIFO and determines when the A_FULL bit in the Interrupt_Status
  register (0x00) gets asserted. The A_FULL bit will be set when the FIFO contains 128 minus FIFO_A_FULL[6:0] items */
  max86140_write_register(priv, HRAFE_FIFO_CONFIG1, 0x34);

  /*clear fifo*/
  max86140_write_register(priv, HRAFE_FIFO_CONFIG2, 0x12);

  /*Enable interrupt register1
   * BIT7    BIT6      BIT5     BIT4      BIT3       BIT2          BIT1     BIT0
   * A_FULL  DATA_RDY  ALC_OVF  PROX_INT  LED_COMPB  DIE_TEMP_RDY  VDD_OOR  PWR_RDY
   *
   */
  max86140_write_register(priv, HRAFE_INT1_ENABLE, 0x00);

  /*Enable interrupt register1
   * BIT7    BIT6      BIT5     BIT4      BIT3       BIT2        BIT1     BIT0
   * 0       0         0        0         0          0           0        SHA_DONE
   *
   */
  max86140_write_register(priv, HRAFE_INT2_ENABLE, 0x00);

  /*Clear INT*/
  ret = max86140_read_register(priv, HRAFE_INT1_STATUS);
  sninfo("INTI_CLEAR_INT1 = %x\r\n", ret);

  ret = max86140_read_register(priv, HRAFE_INT2_STATUS);
  sninfo("INTI_CLEAR_INT2 = %x\r\n", ret);

  max86140_int_initialize(priv);

  up_udelay(1000);

  return OK;
}

/****************************************************************************
 * Name: max86140_readdata
 *
 * Description:
 *   Read data from max86140 FIFO
 *
 ****************************************************************************/
static int max86140_readdata(FAR struct max86140_dev_s *priv, uint8_t *fifodata)
{
  uint8_t buf[3] = {0};
  uint8_t fifo_counter = 0;
  fifo_counter = max86140_read_register(priv, 0x07);
  if (fifo_counter > 0x0A)
    {
      /*Read FIFO data*/
      max86140_read_fifo(priv, 0x08, buf);
      fifodata[0] = buf[0];
      fifodata[1] = buf[1];
      fifodata[2] = buf[2];
    }
  else
    {
      fifodata[0] = 0;
      fifodata[1] = 0;
      fifodata[2] = 0;
    }
  return OK;
}

/****************************************************************************
 * Name: max86140_open
 *
 * Description:
 *   This function is called whenever the MAX86140 device is opened.
 *
 ****************************************************************************/
static int max86140_open(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct max86140_dev_s *priv  = inode->i_private;
  int ret = 0;
  ret = max86140_init(priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to open driver: %d\n", ret);
      return ret;
    }
  return OK;
}

/****************************************************************************
 * Name: max86140_close
 *
 * Description:
 *   This routine is called when the max86140 device is closed.
 *
 ****************************************************************************/
static int max86140_close(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct max86140_dev_s *priv  = inode->i_private;

  max86140_write_register(priv, HRAFE_INT1_ENABLE, 0);
  max86140_write_register(priv, HRAFE_INT2_ENABLE, 0);

  max86140_read_register(priv, HRAFE_INT1_STATUS);
  max86140_read_register(priv, HRAFE_INT2_STATUS);

  max86140_write_register(priv, HRAFE_SYSTEM_CTL, 0x02);
  return OK;
}

/****************************************************************************
 * Name: max86140_write
 ****************************************************************************/
static ssize_t max86140_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: max86140_write
 ****************************************************************************/
static ssize_t max86140_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: max86140_ioctl
 ****************************************************************************/
static ssize_t max86140_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct max86140_dev_s *priv  = inode->i_private;
  uint8_t *raw;
  if (NULL == (void *)arg)
    {
      snerr("[%s]: parameter is NULL.\n", __FUNCTION__);
      return -EINVAL;
    }
  switch (cmd)
    {
      case SNIOC_A_READ_FIFO:
        raw = (uint8_t *) arg;
        max86140_readdata(priv, raw);
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
      case SNIOC_ENABLE_INT1:
        {
          max86140_write_register(priv, HRAFE_INT1_ENABLE, (uint8_t)arg);
          snerr("Enable_INT1 = %x\r\n", (uint8_t)arg);
        }
        break;
      case SNIOC_DISABLE_INT1:
        {
          max86140_write_register(priv, HRAFE_INT1_ENABLE, 0);
          snerr("Disable_INT1...\n");
        }
        break;
      case SNIOC_CLEAR_INT1:
        {
          uint8_t reg;
          reg = max86140_read_register(priv, HRAFE_INT1_STATUS);
          sninfo("CLear_INT1_by_reading = %x\r\n", reg);
          *(uint8_t *)arg = reg;
        }
        break;
      case SNIOC_GPIO_1_2_TEST:
        {
          /*1. Make GIPO 1  in tristate ro mux control config*/
          max86140_write_register(priv, HRAFE_PPG_CONFIG0, 0x00);
          up_udelay(100000);

          max86140_write_register(priv, HRAFE_LED_SEQ_REG1, 0x00);
          max86140_write_register(priv, HRAFE_LED_SEQ_REG2, 0x00);
          up_udelay(100000);

          max86140_write_register(priv, HRAFE_LED_SEQ_REG1, 0x21);
          max86140_write_register(priv, HRAFE_LED_SEQ_REG2, 0x03);
          up_udelay(100000);


          /*1. Make GIPO 2  in tristate ro mux control config*/
          max86140_write_register(priv, HRAFE_PPG_CONFIG0, 0x04);
          up_udelay(100000);

          max86140_write_register(priv, HRAFE_LED_SEQ_REG1, 0x00);
          max86140_write_register(priv, HRAFE_LED_SEQ_REG2, 0x00);
          up_udelay(100000);

          max86140_write_register(priv, HRAFE_LED_SEQ_REG1, 0x21);
          max86140_write_register(priv, HRAFE_LED_SEQ_REG2, 0x03);
          up_udelay(100000);

          *(uint32_t *) arg = true;
        }
        break;
      default:
        syslog(LOG_ERR, "NO CMD matched: %d\r\n", cmd);
        break;
    }
  return OK;
}

static void max86140_int_cb(FAR struct max86140_dev_s *priv)
{
  union sigval value;

  if (priv->receive_pid != 0)
    {
      (void)sigqueue(priv->receive_pid, priv->signo, value);
    }
}

/****************************************************************************
 * Name: max86140_register
 *
 * Description:
 *   Register the max86140 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/heartrate0"
 *   spi     - An instance of the SPI interface to use to communicate with
 *             max86140
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/
int max86140_register(FAR const char *devpath, FAR struct spi_dev_s *spi, FAR struct max86140_config_s *config)
{
  FAR struct max86140_dev_s *priv;
  int ret;

  /* Sanity check */
  DEBUGASSERT(spi != NULL);
  DEBUGASSERT(config != NULL);

  /* Initialize the max86140 device structure */
  priv = (FAR struct max86140_dev_s *)kmm_malloc(sizeof(struct max86140_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  /* Initialize the max86140 device structure */
  priv->spi = spi;
  memcpy(&priv->config, config, sizeof(struct max86140_config_s));

  priv->int_cb = max86140_int_cb;
  priv->receive_pid = 0;
  priv->signo = 0;

  /* Setup SPI frequency and mode */
#ifdef CONFIG_MAX86140_125K
  priv->spi_frequency = 125000;
#elif defined(CONFIG_MAX86140_250K)
  priv->spi_frequency = 250000;
#elif defined(CONFIG_MAX86140_500K)
  priv->spi_frequency = 500000;
#elif defined(CONFIG_MAX86140_1M)
  priv->spi_frequency = 1000000;
#elif defined(CONFIG_MAX86140_2M)
  priv->spi_frequency = 2000000;
#elif defined(CONFIG_MAX86140_4M)
  priv->spi_frequency = 4000000;
#elif defined(CONFIG_MAX86140_8M)
  priv->spi_frequency = 8000000;
#else
#  error No SPI frequency for the MAX86140 is defined.
#endif

  /* Deselect the Heart rate device */
  SPI_SELECT(priv->spi, priv->config.spi_devid, false);

  /* Setup SPI mode */
  SPI_SETMODE(spi, SPIDEV_MODE0);

  /* Register device information */
  max86140_dev_register(priv);

  /* Register the character driver */
  ret = register_driver(devpath, &g_max86140_ops, 0666, priv);

  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  sninfo("MAX86140 driver loaded successfully!\n");
  return ret;
}

#endif /*CONFIG_MAX86140*/
