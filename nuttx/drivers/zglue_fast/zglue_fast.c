/****************************************************************************
 *   drivers/zglue_fast/fast_zeus.c
 *
 *   Copyright (C) 2018 zGlue Inc. All rights reserved.
 *   Author: Bill Rees <bill@zglue.com>
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
#include <stdint.h>
#include <sys/types.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/progmem.h>
#include <arch/board/board.h>
#include <nuttx/spi/spi_transfer.h>
#include <nuttx/drivers/zglue_fast.h>
#include <nuttx/zglue_fast/fast_api.h>
#include <nuttx/zglue_fast/fast_debug_api.h>
#include "nrf52_jtag.h"
#include "nrf52_gpio.h"
#if defined(CONFIG_FAST_SPI)
#include <nuttx/spi/spi.h>
#include "nrf52_spi.h"
#endif
#if defined(CONFIG_FAST_I2C)
#include <nuttx/i2c/i2c_master.h>
#endif

#ifndef CONFIG_FAT_MAXFNAME
#  define CONFIG_FAT_MAXFNAME 32
#endif

#ifndef CONFIG_FAST_FILE_DIRECTORY
#  define CONFIG_FAST_FILE_DIRECTORY "/mnt"
#endif

#ifdef CONFIG_FAST_CONFIG_FILE
#include "zglue_fast_config_file.h"
#endif

#define CONFIG_FILE_NAME "fast_config_file.bin"

/*
 * Holds flags and values for manipulating debug
 * features in the fast driver, fast commands subsys
 * and anyone else who shows interest.
 */
int32_t g_debug_fast;
int8_t zcad_program_status = -1;

extern unsigned char fast_config_file_bin[];

static int fast_open(FAR struct file *filep);
static int fast_close(FAR struct file *filep);
static ssize_t fast_read(FAR struct file *filep, FAR char *buffer, size_t buflen);
static ssize_t fast_write(FAR struct file *filep, FAR const char *buffer, size_t buflen);
static int fast_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
static int fast_poll(FAR struct file *filep, struct pollfd *fds, bool setup);
#endif
const char *fast_dev_path = FAST_DEV_PATH;
uint16_t *fast_config_file_ptr;

extern uint32_t __fast_config_file_start__;
extern uint32_t __fast_config_file_end__;
extern uint32_t __fast_config_section_start__;
extern uint32_t __fast_config_section_end__;
extern uint8_t fast_debug_level;
extern uint8_t fast_api_init_done;
extern fast_power_state_t fast_current_power_state;



/* Internal data blob holding semaphore, buf pointer, buflen, count
 * and anything else needed for transfering data over spi.
 */
static struct zg_fast_dev_s g_fastdev;
static struct zg_fast_dev_s *g_fastdev_p = &g_fastdev;

/* Most of these ops are not supported (basically all but open(), ioctl())
 */
static const struct file_operations g_fast_ops =
{
  fast_open,       /* open   */
  fast_close,      /* close  */
  fast_read,       /* read   */
  fast_write,      /* write  */
  0,               /* seek   */
  fast_ioctl,      /* ioctl  */
#ifndef CONFIG_DISABLE_POLL
  fast_poll,       /* poll   */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  0,               /* unlink */
#endif
};




/****************************************************************************************
 * @brief byte swap function
 * @return Status
 ****************************************************************************************
 */
uint32_t _bswap32(uint32_t a)
{
  a = ((a & 0x000000FF) << 24) |
      ((a & 0x0000FF00) <<  8) |
      ((a & 0x00FF0000) >>  8) |
      ((a & 0xFF000000) >> 24);
  return a;
}


/************************************************************************************
 * Name: fast_open
 *
 * Description:
 *   This function is called whenever the fast device is opened.
 *
 ************************************************************************************/
static int fast_open(FAR struct file *filep)
{
  FAST_ENTER();

  while (sem_trywait(&g_fastdev_p->dev_sem) != OK)
    {
      if (filep->f_oflags & O_NONBLOCK)
        {
          set_errno(-EAGAIN);
          fasterr("FAST device open failed:\n");
          return !OK;
        }
    }

  FAST_EXIT();
  return OK;
}

/************************************************************************************
 * Name: fast_close
 *
 * Description:
 *   This function is called whenever the fast device is closed.
 *
 ************************************************************************************/
static int fast_close(FAR struct file *filep)
{
  FAST_ENTER();
  sem_post(&g_fastdev_p->dev_sem);

  FAST_EXIT();
  return OK;
}

/******************************************************************************
 * @brief fast_jtag_read writes 8 bits to the instrution register followed by
 *                       32 bits  read from the data register. This is  simple
 *                       jtag read command
 * @param instruction  - 8 bit JTAG instruction code
 *        data         - pointer that contains the data
 *
 *****************************************************************************/

fast_status_e fast_jtag_read(uint8_t jtag_instruction, uint32_t *data)
{
  FAST_ENTER();
#ifdef CONFIG_NRF52_JTAG
  jtag_32b_read(jtag_instruction, data);
#endif
  FAST_EXIT();
  return OK;
}




/******************************************************************************
 *
 * @brief fast_jtag_write writes 8 bits to the instrution register followed by
 *                        32 bits to the data register. This is  simple jtag
 *                        command with its data.
 * @param instruction   - 8 bit JTAG instruction code
 *        data          - 32 bit data to the JTAG instruction
 *
 *****************************************************************************/

fast_status_e fast_jtag_write(uint8_t jtag_instruction, uint32_t data)
{
  FAST_ENTER();
#ifdef CONFIG_NRF52_JTAG
  jtag_32b_write(jtag_instruction, data);
#endif
  FAST_EXIT();
  return OK;
}

/****************************************************************************************
 * @brief fast read register
 * @return Status
 ****************************************************************************************
 */
#if defined(CONFIG_ZEUS2)
fast_status_e fast_read_register(uint8_t dev_id, uint32_t reg_addr, uint8_t *data_r, uint16_t len)
{
  uint8_t data[ZEUS2_REG_LEN] = {0, 0, 0, 0};
  reg_addr = _bswap32(reg_addr);
  /*SPI interface*/
#if defined(CONFIG_FAST_SPI)
  if (g_fastdev_p->interface == FAST_SPI_INTERFACE)
    {
      uint8_t spi_address[ZEUS2_REG_LEN] = FORMAT_ADDR(reg_addr);

      /* If SPI bus is shared then lock and configure it */
      (void)SPI_LOCK(g_fastdev_p->spi, true);

      /* set the SPI frequency */
      SPI_SETFREQUENCY(g_fastdev_p->spi, g_fastdev_p->spi_frequency);

      /* Select the FAST device */
      SPI_SELECT(g_fastdev_p->spi, g_fastdev_p->config.spi_devid, true);

      if (g_fastdev_p->spi_bit_mode == FAST_SPI_32_BIT_MODE)
        {
          SPI_SNDBLOCK(g_fastdev_p->spi, spi_address, 4);
          SPI_RECVBLOCK(g_fastdev_p->spi, data, 4);
        }
      else if (g_fastdev_p->spi_bit_mode == FAST_SPI_16_BIT_MODE)
        {
          SPI_SNDBLOCK(g_fastdev_p->spi, &(spi_address[0]), 2);
          SPI_SNDBLOCK(g_fastdev_p->spi, &(spi_address[2]), 2);
          SPI_RECVBLOCK(g_fastdev_p->spi, &(data[0]), 2);
          SPI_RECVBLOCK(g_fastdev_p->spi, &(data[2]), 2);
        }
      else if (g_fastdev_p->spi_bit_mode == FAST_SPI_8_BIT_MODE)
        {
          SPI_SNDBLOCK(g_fastdev_p->spi, &(spi_address[0]), 1);
          SPI_SNDBLOCK(g_fastdev_p->spi, &(spi_address[1]), 1);
          SPI_SNDBLOCK(g_fastdev_p->spi, &(spi_address[2]), 1);
          SPI_SNDBLOCK(g_fastdev_p->spi, &(spi_address[3]), 1);

          SPI_RECVBLOCK(g_fastdev_p->spi, &(data[0]), 1);
          SPI_RECVBLOCK(g_fastdev_p->spi, &(data[1]), 1);
          SPI_RECVBLOCK(g_fastdev_p->spi, &(data[2]), 1);
          SPI_RECVBLOCK(g_fastdev_p->spi, &(data[3]), 1);
        }

      /* Deselect the FAST device */
      SPI_SELECT(g_fastdev_p->spi, g_fastdev_p->config.spi_devid, false);

      /* Unlock bus */
      (void)SPI_LOCK(g_fastdev_p->spi, false);
    }
#endif

  /*i2c interface*/
#if defined(CONFIG_FAST_I2C)
  if (g_fastdev_p->interface == FAST_I2C_INTERFACE)
    {
      struct i2c_config_s config;
      memset(&config, 0, sizeof(struct i2c_config_s));
      int ret = FAST_OK;

      data[0] = reg_addr & 0xff;
      data[1] = (reg_addr & 0xff00) >> 8;
      data[2] = (reg_addr & 0xff0000) >> 16;
      data[3] = (reg_addr & 0xff000000) >> 24;
      /* Set up the I2C configuration */
      config.address   = dev_id;
      config.addrlen   = 7;
      config.frequency = CONFIG_FAST_I2C_FREQUENCY;
      ret = i2c_write(g_fastdev_p->i2c, &config, data, ZEUS2_REG_LEN);
      if (ret < 0)
        {
          snerr("ERROR: i2c_write failed: %d\n", ret);
          return FAST_REG_READ_FAILURE;
        }
      for (int i = 0; i < ZEUS2_REG_LEN; i++)
        {
          data[i] = 0;
        }
      ret = i2c_read(g_fastdev_p->i2c, &config, data, ZEUS2_REG_LEN);
      if (ret < 0)
        {
          snerr("ERROR: i2c_write failed: %d\n", ret);
          return FAST_REG_READ_FAILURE;
        }
    }

#endif


  for (int i = 0; i < ZEUS2_REG_LEN; i++)
    {
      data_r[i] = data[ZEUS2_REG_LEN - 1 - i];
    }

  return FAST_OK;
}

/****************************************************************************************
 * @brief fast write register
 * @return Status
 ****************************************************************************************
 */
fast_status_e fast_write_register(uint8_t dev_id, uint32_t reg_addr, uint8_t *data_w, uint16_t len)
{
#if defined(CONFIG_FAST_SPI) || defined(CONFIG_FAST_I2C)
  uint8_t write_data[ZEUS2_REG_LEN] = {0, 0, 0, 0};
  reg_addr = _bswap32(reg_addr);
  for (int i = 0; i < ZEUS2_REG_LEN; i++)
    {
      write_data[i] = data_w[ZEUS2_REG_LEN - 1 - i];
    }
  /*SPI interface*/
#if defined(CONFIG_FAST_SPI)
  if (g_fastdev_p->interface == FAST_SPI_INTERFACE)
    {
      uint8_t spi_address[ZEUS2_REG_LEN] = FORMAT_ADDR(reg_addr);


      /* If SPI bus is shared then lock and configure it */
      (void)SPI_LOCK(g_fastdev_p->spi, true);

      /* set the SPI frequency */
      SPI_SETFREQUENCY(g_fastdev_p->spi, g_fastdev_p->spi_frequency);

      /* Select the FAST device */
      SPI_SELECT(g_fastdev_p->spi, g_fastdev_p->config.spi_devid, true);

      if (g_fastdev_p->spi_bit_mode == FAST_SPI_32_BIT_MODE)
        {
          SPI_SNDBLOCK(g_fastdev_p->spi, spi_address, 4);
          SPI_SNDBLOCK(g_fastdev_p->spi, write_data, 4);
        }
      else if (g_fastdev_p->spi_bit_mode == FAST_SPI_16_BIT_MODE)
        {
          SPI_SNDBLOCK(g_fastdev_p->spi, &(spi_address[0]), 2);
          SPI_SNDBLOCK(g_fastdev_p->spi, &(spi_address[2]), 2);
          SPI_SNDBLOCK(g_fastdev_p->spi, &(write_data[0]), 2);
          SPI_SNDBLOCK(g_fastdev_p->spi, &(write_data[2]), 2);
        }
      else if (g_fastdev_p->spi_bit_mode == FAST_SPI_8_BIT_MODE)
        {
          SPI_SNDBLOCK(g_fastdev_p->spi, &(spi_address[0]), 1);
          SPI_SNDBLOCK(g_fastdev_p->spi, &(spi_address[1]), 1);
          SPI_SNDBLOCK(g_fastdev_p->spi, &(spi_address[2]), 1);
          SPI_SNDBLOCK(g_fastdev_p->spi, &(spi_address[3]), 1);
          SPI_SNDBLOCK(g_fastdev_p->spi, &(write_data[0]), 1);
          SPI_SNDBLOCK(g_fastdev_p->spi, &(write_data[1]), 1);
          SPI_SNDBLOCK(g_fastdev_p->spi, &(write_data[2]), 1);
          SPI_SNDBLOCK(g_fastdev_p->spi, &(write_data[3]), 1);
        }

      /* Deselect the FAST device */
      SPI_SELECT(g_fastdev_p->spi, g_fastdev_p->config.spi_devid, false);

      /* Unlock bus */
      (void)SPI_LOCK(g_fastdev_p->spi, false);
    }
#endif

  /*i2c interface*/

#if defined(CONFIG_FAST_I2C)
  if (g_fastdev_p->interface == FAST_I2C_INTERFACE)
    {
      struct i2c_config_s config;
      memset(&config, 0, sizeof(struct i2c_config_s));
      int ret = FAST_OK;

      /* Set up the I2C configuration */
      config.address   = dev_id;
      config.addrlen   = 7;
      config.frequency = 400000;
      uint8_t data[ZEUS2_REG_LEN * 2];
      data[0] = reg_addr & 0xff;
      data[1] = (reg_addr & 0xff00) >> 8;
      data[2] = (reg_addr & 0xff0000) >> 16;
      data[3] = (reg_addr & 0xff000000) >> 24;
      data[4] = write_data[0];
      data[5] = write_data[1];
      data[6] = write_data[2];
      data[7] = write_data[3];

      /* Write the register address and value */
      ret = i2c_write(g_fastdev_p->i2c, &config, data, ZEUS2_REG_LEN * 2);
      if (ret < 0)
        {
          snerr("ERROR: i2c_write failed: %d\n", ret);
          return FAST_REG_WRITE_FAILURE;
        }
    }

#endif
#endif
  return FAST_OK;
}
#endif //CONFIG_ZEUS2
/******************************************************************************
 * @brief fast_config_file_read locate the fast config data in flash and
 *                               pass it back to the caller (the API)
 *
 *****************************************************************************/
fast_status_e fast_config_file_read(uint32_t offset, uint32_t length,
                                    uint16_t *buffer)
{
  FAST_ENTER();
#ifdef CONFIG_FAST_CONFIG_FILE
  uint32_t fast_config_file_start_address;

  fast_config_file_ptr = (uint16_t *)fast_config_file_bin;
  fast_config_file_start_address = (uint32_t) & (__fast_config_file_start__);
  if (up_progmem_ispageerased(up_progmem_getpage(fast_config_file_start_address))  == 0)
    {
      return FAST_FILE_OPEN_FAIL;
    }
  uint32_t config_file_length = fast_config_file_bin_len;
  if (length > config_file_length)
    {
      length = config_file_length;
    }
  uint32_t fast_start_address = (uint32_t) & (__fast_config_file_start__);

  for (uint32_t i = 0 ; i < length ; i++)
    {
      buffer[i] = *((uint32_t *)(fast_start_address + offset * 2 + i * 2));
    }

#else
  return -1;
#endif

  FAST_EXIT();
  return OK;
}

/**
 ****************************************************************************************
 * @brief FAST realign tile grids
 * @param  filename: input realignment-correction file
 * @return Status
 ****************************************************************************************
 */
fast_status_e fast_config_file_update(char *config_filename)
{
  FAST_ENTER();
  FILE *config_file = fopen(config_filename, "rb");
  char line[256] = {0};
  uint16_t index = 0;
  uint32_t filesize = 0;
  size_t bytes_read = 0;
  uint32_t fast_start_address = (uint32_t) & (__fast_config_section_start__);
  uint32_t fast_config_section_length = (((uint32_t) & (__fast_config_section_end__)) - ((uint32_t) &
                                         (__fast_config_section_start__)));

  if (config_file == NULL)
    {
      printf("fast config file update: error opening file \r\n");
      return FAST_FILE_OPEN_FAIL;
    }

  fseek(config_file, 0, SEEK_END); // seek to end of file
  filesize = ftell(config_file); // get current file pointer
  if (filesize > fast_config_section_length)
    {
      printf("fast config file update: file too big \r\n");
      return FAST_FILE_OPEN_FAIL;
    }

  /* Seek to the beginning of the file */
  fseek(config_file, 0, SEEK_SET);
  while (1)
    {
      bytes_read = fread(line, 1, 256, config_file);
      if (bytes_read == 0)
        {
          break;
        }
      if (bytes_read > 0 && bytes_read < 256)
        {
          /* pad the rest of the 256 with 0 */
          memset(&line[bytes_read], 0x00, (sizeof(uint8_t) * (256 - bytes_read)));
        }
      printf("Copying to address : 0x%X bytes : %d\r\n", fast_start_address + index, 256);
      up_progmem_write(fast_start_address + index, (void *)line, 256);
      index += bytes_read;
    }

  FAST_EXIT();
  return OK;
}


/******************************************************************************
 * @brief fast_config_file_read locate the fast config data in flash and
 *                               pass it back to the caller (the API)
 *
 *****************************************************************************/
fast_status_e fast_config_file_erase(void)
{
  FAST_ENTER();
#ifdef CONFIG_FAST_CONFIG_FILE
  size_t temp = 0;
  size_t pagesize = up_progmem_pagesize(temp);
  uint32_t fast_config_file_start_address = (uint32_t) & (__fast_config_section_start__);
  uint32_t config_file_length = (((uint32_t) & (__fast_config_section_end__)) - ((uint32_t) & (__fast_config_section_start__)));
  uint32_t numpages = config_file_length / pagesize;
  if (config_file_length % pagesize > 0)
    {
      numpages++;
    }
  printf("Erased num pages : %d, page size : %d file size : %d \r\n", numpages, pagesize, config_file_length);
  for (int i = 0; i < numpages; i++)
    {
      uint32_t pagenumber =  up_progmem_getpage(fast_config_file_start_address + (i * (up_progmem_pagesize(temp))));
      printf("Erased page number : %d\r\n", pagenumber);
      if (up_progmem_erasepage(pagenumber) < 0)
        {
          return -EIO;
        }
    }

  /* Reset the fast api init done to 0 */
  fast_api_init_done = 0;
#endif
  FAST_EXIT();
  return OK;
}

/************************************************************************************
 * Name: fast_sleep
 *
 *
 ************************************************************************************/
void fast_sleep(uint32_t time_us)
{
  up_udelay(time_us);
}

/************************************************************************************
 * Name: fast zeus1 reset
 *
 * Description:
 *  Toggles the RESETB and RESETIOB to bring the zeus1 out of reset
 *
 ************************************************************************************/
#if defined(CONFIG_ARCH_BOARD_ZGLUE_ZEUS1_REMORA_BOARD) && defined(CONFIG_ZEUS1)
int16_t fast_zeus1_reset(void)
{
  gpio_pin_write(BOARD_FAST_EXT_RESETIOB_PIN, 1);
  up_udelay(1000);  /*Delay 1ms*/
  gpio_pin_write(BOARD_FAST_EXT_RESETB_PIN, 1);
  return OK;
}
#endif

/************************************************************************************
 * Name: fast zeus2 status
 *
 * Description:
 *  Bit 0 - PWR_CFG1_STATUS
 *  Bit 1 - PWR_CFG2_STATUS
 *  Bit 2 - PWR_CFG3_STATUS
 *  Bit 3 - PWR_CFG4_STATUS
 *  Bit 4 - PWR_CLK_RDY
 *  Bit 5 - FAST_RDY
 *
 ************************************************************************************/
#if defined(CONFIG_ZEUS2_DEVKIT_FEATURES) && defined(CONFIG_ZEUS2)
uint16_t fast_zeus2_status(void)
{
  uint16_t fast_status = 0;
  fast_status = (gpio_pin_read(BOARD_FAST_PWRCFG_4) << 3) | (gpio_pin_read(BOARD_FAST_PWRCFG_3) << 2) | (gpio_pin_read(
                  BOARD_FAST_PWRCFG_2) << 1) | gpio_pin_read(BOARD_FAST_PWRCFG_1);
  fast_status |= (gpio_pin_read(BOARD_FAST_PWR_CLK_RDY) << 4);
  fast_status |= (gpio_pin_read(BOARD_FAST_FAST_RDY) << 5);
  printf("pwr clk rdy : %d fast rdy : %d\r\n", gpio_pin_read(BOARD_FAST_PWR_CLK_RDY), gpio_pin_read(BOARD_FAST_FAST_RDY));
  printf("pwr cfg 1 : %d 2 : %d 3 : %d 4 : %d\r\n", gpio_pin_read(BOARD_FAST_PWRCFG_1), gpio_pin_read(BOARD_FAST_PWRCFG_2),
         gpio_pin_read(BOARD_FAST_PWRCFG_3), gpio_pin_read(BOARD_FAST_PWRCFG_4));

  return fast_status;
}
#endif

/************************************************************************************
 * Name: fast zeus1 reset
 *
 * Description:
 *  Toggles the RESETB and RESETIOB to bring the zeus1 out of reset
 *
 ************************************************************************************/
#if defined(CONFIG_ZEUS2_DEVKIT_FEATURES) && defined(CONFIG_ZEUS2)
fast_status_e fast_zeus2_poweron(void)
{

  uint16_t fast_status = fast_zeus2_status();
  printf("fast status : 0x%x\n", fast_status);
  if (((fast_status & 0x4) == 0x4) || ((fast_status & 0x8) == 0x8))
    {
      gpio_pin_write(BOARD_FAST_ZIP_EN_H, 1);
      up_mdelay(1500);  /*Delay 1 second */
      gpio_pin_write(BOARD_FAST_ZIP_EN_H, 0);
    }
  else if (((fast_status & 0x1) == 0x1) || ((fast_status & 0x2) == 0x2))
    {
      gpio_pin_write(BOARD_FAST_ZIP_EN_L, 1);
      up_mdelay(1500);  /*Delay 1 second */
      gpio_pin_write(BOARD_FAST_ZIP_EN_L, 0);
    }
  return OK;
}
#endif

/************************************************************************************
 * Name: fast zeus1 reset
 *
 * Description:
 *  Toggles the RESETB and RESETIOB to bring the zeus1 out of reset
 *
 ************************************************************************************/
#if defined(CONFIG_ZEUS2_DEVKIT_FEATURES) && defined(CONFIG_ZEUS2)
fast_status_e fast_zeus2_poweroff(void)
{
  return fast_mcu_poweroff();
}
#endif


/************************************************************************************
 * Name: fast_toggle_ulpm_wake
 *
 * Description:
 *  Toggles the ULPM wake pin. This brings the zeus2 chip out of reset
 *
 ************************************************************************************/
int16_t fast_toggle_ulpm_wake(void)
{
#ifdef CONFIG_FAST_ULPM_WAKEUP_PIN
  gpio_pin_write(BOARD_ULPM_WAKEUP_PIN, 1);
  up_udelay(4000);  /*Delay 4ms*/
  gpio_pin_write(BOARD_ULPM_WAKEUP_PIN, 0);
#endif
  return OK;
}

/************************************************************************************
 * Name: fast_read
 *
 * Description:
 *   A dummy read method.  This is provided only to satisfy the VFS layer.
 *
 ************************************************************************************/
static ssize_t fast_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  FAST_ENTER();
  set_errno(-ENOSYS);
  FAST_EXIT();
  return !OK;
}

/************************************************************************************
 * Name: fast_write
 *
 * Description:
 *   A dummy write method.  This is provided only to satisfy the VFS layer.
 *
 ************************************************************************************/
static ssize_t fast_write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
{
  FAST_ENTER();
  set_errno(-ENOSYS);
  FAST_EXIT();
  return !OK;
}

/************************************************************************************
 * Name: fast_ioctl
 *
 * Description:
 *   The standard ioctl method.  This is where ALL of the fast api work is
 *   done.
 *
 ************************************************************************************/
static int fast_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  int ret = OK;

  FAST_ENTER();

  switch (cmd)
    {
      case FAST_IOCTL_API_INIT:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          uint8_t config_file_id = (uint8_t)((uint32_t *)arg)[0];
          ret = fast_api_init((uint8_t)config_file_id);
        }
        break;
      case FAST_IOCTL_API_CLOSE:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          uint8_t config_file_id = (uint8_t)((uint32_t *)arg)[0];
          ret = fast_api_close(config_file_id);
        }
        break;
      case FAST_IOCTL_GET_SYSTEM_INFORMATION:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          fast_system_information_t *sysinfo;
          sysinfo = (fast_system_information_t *)(((uint32_t *)arg)[0]);
          ret = fast_system_get_information(sysinfo);
        }
        break;
      case FAST_IOCTL_GET_SYSTEM_CHIPS_INFORMATION:
        {
          fast_system_chips_information_t *chipinfo;
          chipinfo = (fast_system_chips_information_t *)(((uint32_t *)arg)[0]);
          ret = fast_system_get_chips_information(chipinfo);
        }
        break;
      case FAST_IOCTL_CONNECT_SYSTEM:
        {
          uint8_t config_file_id = (uint8_t)((uint32_t *)arg)[0];
          fastinfo("\t%s arg0 : %d\t arg1:%d\n", str_cmd(cmd), config_file_id);
          ret = fast_system_connect(config_file_id);
        }
        break;
      case FAST_IOCTL_DISCONNECT_SYSTEM:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          uint8_t config_file_id = (uint8_t)((uint32_t *)arg)[0];
          ret = fast_system_disconnect(config_file_id);
        }
        break;
      case FAST_IOCTL_CONNECT_CHIP:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          uint8_t config_file_id = (uint8_t)((uint32_t *)arg)[0];
          uint16_t chip_id = (uint16_t)((uint32_t *)arg)[1];
          ret = fast_chip_connect(config_file_id, chip_id);
        }
        break;
      case FAST_IOCTL_DISCONNECT_CHIP:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          uint8_t config_file_id = (uint8_t)((uint32_t *)arg)[0];
          uint16_t chip_id = (uint16_t)((uint32_t *)arg)[1];
          ret = fast_chip_disconnect(config_file_id, chip_id);
        }
        break;
      case FAST_IOCTL_READ_ID:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          ret = fast_read_id((uint8_t *)arg, FAST_CHIP_ID_SIZE);
        }
        break;
      case FAST_IOCTL_ENTER_POWER_STATE:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          fast_power_state_t power_state = ((uint32_t *)arg)[0];
          ret = fast_power_state_enter(power_state);
        }
        break;
      case FAST_IOCTL_SET_TIMEOUT:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          uint16_t time_ms;
          time_ms = ((uint32_t *)arg)[0];;
          ret = fast_set_timeout(time_ms);
        }
        break;
      case FAST_IOCTL_GET_CURRENT_POWER_STATE:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          fast_power_state_t *power_state = (fast_power_state_t *) & (((uint32_t *)arg)[0]);
          ret = fast_power_state_get(power_state);
        }
        break;
      case FAST_IOCTL_SET_DEBUG_LEVEL:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          fast_debug_level_t fast_debug_level_local;
          fast_debug_level_local = ((fast_debug_level_t)((uint32_t *)arg)[0]);
          ret = fast_set_debug_level(fast_debug_level_local);
        }
        break;
      case FAST_IOCTL_GET_DEBUG_LEVEL:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          ((uint8_t *)arg)[0] = fast_debug_level;
          ret = OK;
        }
        break;
      case FAST_IOCTL_READ_TILE:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          uint8_t row = 0, col = 0;
          row = ((uint8_t)((uint32_t *)arg)[0]);
          col = ((uint8_t)((uint32_t *)arg)[1]);
          ret = fast_tile_read(row, col, &(((uint32_t *)arg)[2]));
        }
        break;
      case FAST_IOCTL_WRITE_TILE:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          uint8_t row = 0, col = 0;
          row = ((uint8_t)((uint32_t *)arg)[0]);
          col = ((uint8_t)((uint32_t *)arg)[1]);
          ret = fast_tile_write(row, col, ((uint32_t *)arg)[2]);
        }
        break;
      case FAST_IOCTL_SCANTILE:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          uint8_t start_row = 0, start_col = 0, end_row = 0, end_col;
          uint32_t *scantile_data;
          start_row = ((uint8_t)((uint32_t *)arg)[0]);
          start_col = ((uint8_t)((uint32_t *)arg)[1]);
          end_row = ((uint8_t)((uint32_t *)arg)[2]);
          end_col = ((uint8_t)((uint32_t *)arg)[3]);
          scantile_data = ((uint32_t *)((uint32_t *)arg)[4]);
          ret = fast_scantile(start_row, start_col, end_row, end_col, scantile_data);
        }
        break;
      case FAST_IOCTL_CFG_FILE_ERASE:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          ret = fast_config_file_erase();
        }
        break;
      case FAST_IOCTL_CFG_FILE_UPDATE:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          char src[CONFIG_FAT_MAXFNAME], fw_config_file[CONFIG_FAT_MAXFNAME * 2];
          strcpy(src,  (char *)((uint32_t *)arg)[0]);
          strcpy(fw_config_file, CONFIG_FAST_FILE_DIRECTORY);
          strcat(fw_config_file, "/");
          strcat(fw_config_file, src);
          ret = fast_config_file_update(fw_config_file);
        }
        break;
#if defined(CONFIG_ARCH_BOARD_ZGLUE_ZEUS1_REMORA_BOARD) && defined(CONFIG_ZEUS1)
      case FAST_IOCTL_ZEUS1_RESET:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          ret = fast_zeus1_reset();
        }
        break;
#endif
#if defined(CONFIG_ZEUS2_DEVKIT_FEATURES) && defined(CONFIG_ZEUS2)
      case FAST_IOCTL_DEVKIT_ZEUS2_STATUS:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          ((uint16_t *)arg)[0] = fast_zeus2_status();
          ret = OK;
        }
        break;
      case FAST_IOCTL_DEVKIT_ZEUS2_PWRON:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          ret = fast_zeus2_poweron();
        }
        break;
      case FAST_IOCTL_DEVKIT_ZEUS2_PWROFF:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          ret = fast_zeus2_poweroff();
        }
        break;
      case FAST_IOCTL_DEVKIT_ZEUS2_BOOTCFG_BYP:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          uint8_t byp = 0;
          byp = ((uint8_t)((uint32_t *)arg)[0]);
          gpio_pin_write(BOARD_FAST_BOOTCFG_BYP, byp);
        }
        break;
      case FAST_IOCTL_DEVKIT_ZEUS2_nRF_EN_SW:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          uint8_t NRF_EN_SW_pin = 0;
          NRF_EN_SW_pin = ((uint8_t)((uint32_t *)arg)[0]);
          gpio_pin_write(BOARD_NRF_EN_SW, NRF_EN_SW_pin);
        }
        break;
#endif
#if defined(CONFIG_ZEUS2)
      case FAST_IOCTL_CLEAR_TILE:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          ret = fast_clear_tiles();
        }
        break;
      case FAST_IOCTL_PROGRAM:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          char src[CONFIG_FAT_MAXFNAME], dest[CONFIG_FAT_MAXFNAME * 2];
          uint8_t connections_action = 0;
          strcpy(src,  (char *)((uint32_t *)arg)[0]);
          strcpy(dest, CONFIG_FAST_FILE_DIRECTORY);
          strcat(dest, "/");
          strcat(dest, src);
          connections_action = (uint8_t)(((uint32_t *)arg)[1]);
          ret = fast_program(dest, connections_action);
        }
        break;
      case FAST_IOCTL_ZCAD_PROGRAM:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          char src[CONFIG_FAT_MAXFNAME], dest[CONFIG_FAT_MAXFNAME * 2];
          strcpy(src,  (char *)((uint32_t *)arg)[0]);
          strcpy(dest, CONFIG_FAST_FILE_DIRECTORY);
          strcat(dest, "/");
          strcat(dest, src);
          zcad_program_status = fast_zcad_program(dest);
        }
        break;
      case FAST_IOCTL_ZCAD_VERSION:
        {
          char src[CONFIG_FAT_MAXFNAME], dest[CONFIG_FAT_MAXFNAME * 2];
          char *version;
          version = (char *)(((uint32_t *)arg)[1]);
          char buf[32] = {0};
          strcpy(src,  (char *)((uint32_t *)arg)[0]);
          strcpy(dest, CONFIG_FAST_FILE_DIRECTORY);
          strcat(dest, "/");
          strcat(dest, src);

          FILE *fp;
          fp = fopen(dest, "r");
          if (fp == NULL)
            {
              printf("\tNo such file in the file system\r\n");
              return FAST_FILE_OPEN_FAIL;
            }
          /* Seek to the beginning of the file */
          fseek(fp, 0, SEEK_SET);

          if (zcad_program_status == 0)
            {
              if (fgets (buf, 32, fp) != NULL)
                {
                  for (int i = 0; i < strlen(buf); i++)
                    {
                      version[i] = buf[i];
                    }
                }
            }
          else if (zcad_program_status == -1)
            {
              version[0] = 'E';
              version[1] = 'M';
              version[2] = 'P';
              version[3] = 'T';
              version[4] = 'Y';
              version[5] = '\0';
            }
          else
            {
              version[0] = 'I';
              version[1] = 'N';
              version[2] = 'V';
              version[3] = 'A';
              version[4] = 'L';
              version[5] = 'I';
              version[6] = 'D';
              version[7] = '\0';
            }

          fclose(fp);


          return 0;
        }
        break;
      case FAST_IOCTL_REALIGNMENT:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          char src[CONFIG_FAT_MAXFNAME], dest[CONFIG_FAT_MAXFNAME], input_file[CONFIG_FAT_MAXFNAME * 2], output_file[CONFIG_FAT_MAXFNAME * 2];
          strcpy(src,  (char *)((uint32_t *)arg)[0]);
          strcpy(dest,  (char *)((uint32_t *)arg)[1]);
          strcpy(input_file, CONFIG_FAST_FILE_DIRECTORY);
          strcat(input_file, "/");
          strcat(input_file, src);
          strcpy(output_file, CONFIG_FAST_FILE_DIRECTORY);
          strcat(output_file, "/");
          strcat(output_file, dest);
          ret = fast_realign_grid(input_file, output_file);
        }
        break;
      case FAST_IOCTL_WRITE_REG:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          uint32_t    reg_addr;
          uint32_t    reg_data;
          reg_addr = ((uint32_t *)arg)[0];
          reg_data = ((uint32_t *)arg)[1];
          ret = fast_reg_write(reg_addr, reg_data);
        }
        break;
      case FAST_IOCTL_READ_REG:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          uint32_t    reg_addr;
          uint16_t    *reg_data;
          reg_addr = ((uint32_t *)arg)[0];
          reg_data = (uint16_t *) & ((uint32_t *)arg)[1];
          ret = fast_reg_read(reg_addr, reg_data);
        }
        break;
      case FAST_IOCTL_READ_PERIPHERAL:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          fast_peripheral_id_t peri_id;
          uint8_t peri_index;
          uint32_t *peri_data;
          peri_id = (fast_peripheral_id_t)((uint32_t *)arg)[0];
          peri_index = (uint8_t)((uint32_t *)arg)[1];
          peri_data =  &((uint32_t *)arg)[2];
          ret = fast_peripheral_read(peri_id, peri_index, peri_data);
        }
        break;
      case FAST_IOCTL_WRITE_PERIPHERAL:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          fast_peripheral_id_t peri_id;
          uint8_t peri_index = 0;
          uint32_t peri_data = 0;
          peri_id = (fast_peripheral_id_t)((uint32_t *)arg)[0];
          peri_index = (uint8_t)((uint32_t *)arg)[1];
          peri_data  = (uint32_t)((uint32_t *)arg)[2];
          ret = fast_peripheral_write(peri_id, peri_index, peri_data);
        }
        break;
      case FAST_IOCTL_CLEAR_PERIPHERAL:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          fast_peripheral_id_t peri_id;
          peri_id = (fast_peripheral_id_t)((uint32_t *)arg)[0];
          ret = fast_peripheral_clear(peri_id);
        }
        break;
      case FAST_IOCTL_SCAN_PERIPHERAL:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          fast_peripheral_id_t peri_id;
          uint8_t peri_start_index = 0, peri_count = 0;
          uint32_t *peri_data;
          peri_id = (fast_peripheral_id_t)((uint32_t *)arg)[0];
          peri_start_index = (uint8_t)((uint32_t *)arg)[1];
          peri_count = (uint8_t)((uint32_t *)arg)[2];
          peri_data =  (uint32_t *)(((uint32_t *)arg)[3]);
          ret = fast_scan_peripheral(peri_id, peri_start_index, peri_count, peri_data);
        }
        break;
      case FAST_IOCTL_SCAN_REGS:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          ret = fast_scan_regs();
        }
        break;
      case FAST_IOCTL_CONFIGURE_LED:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          bool                   invert_pwm;
          fast_led_id_t          led_id;
          fast_led_duty_cycle_t  duty_cycle;
          fast_led_period_t      led_period;
          fast_led_intensity_control_t         intensity;
          fast_led_brightness_range_control_t  brightness;
          led_id = ((fast_led_id_t)((uint32_t *)arg)[0]);
          duty_cycle = ((fast_led_duty_cycle_t)((uint32_t *)arg)[1]);
          led_period = ((fast_led_period_t)((uint32_t *)arg)[2]);
          intensity = ((fast_led_intensity_control_t)((uint32_t *)arg)[3]);
          brightness = ((fast_led_brightness_range_control_t)((uint32_t *)arg)[4]);
          invert_pwm = (bool)(((uint32_t *)arg)[5]);
          ret = fast_led_configure(led_id, duty_cycle, led_period, intensity,
                                   brightness, invert_pwm);
        }
        break;
      case FAST_IOCTL_ENABLE_LED:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          fast_led_mask_u led_id;
          led_id.data_uint8_t = ((uint8_t)((uint32_t *)arg)[0]);
          ret = fast_led_enable(led_id);
        }
        break;
      case FAST_IOCTL_DISABLE_LED:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          fast_led_mask_u led_id;
          led_id.data_uint8_t = ((uint8_t)((uint32_t *)arg)[0]);
          ret =  fast_led_disable(led_id);
        }
        break;
      case FAST_IOCTL_READ_CUSTOMER_OTP:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          uint32_t start_addr;
          uint16_t *data_ptr;
          uint32_t number_of_bytes;
          start_addr = ((uint32_t *)arg)[1];
          number_of_bytes = ((uint32_t *)arg)[2];
          data_ptr = (uint16_t *)(((uint32_t *)arg)[3]);
          ret = fast_read_customer_otp(start_addr, data_ptr, number_of_bytes);
        }
        break;
      case FAST_IOCTL_WRITE_CUSTOMER_OTP:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          uint32_t start_addr = ((uint32_t *)arg)[1];
          uint32_t number_of_bytes = ((uint32_t *)arg)[2];
          uint16_t *data_ptr = (uint16_t *)(((uint32_t *)arg)[3]);
          ret = fast_write_customer_otp(start_addr, data_ptr, number_of_bytes);
        }
        break;
      case FAST_IOCTL_READ_FAST_OTP:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          uint32_t start_addr;
          uint16_t *data_ptr;
          uint32_t number_of_bytes;
          start_addr = ((uint32_t *)arg)[1];
          number_of_bytes = ((uint32_t *)arg)[2];
          data_ptr = (uint16_t *)(((uint32_t *)arg)[3]);
          ret = fast_read_fast_otp(start_addr, data_ptr, number_of_bytes);
        }
        break;
      case FAST_IOCTL_WRITE_FAST_OTP:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          uint32_t start_addr = ((uint32_t *)arg)[1];
          uint32_t number_of_bytes = ((uint32_t *)arg)[2];
          uint16_t *data_ptr = (uint16_t *)(((uint32_t *)arg)[3]);
          ret = fast_write_fast_otp(start_addr, data_ptr, number_of_bytes);
        }
        break;
      case FAST_IOCTL_PMIC_BOOST_CONFIGURE:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          fast_pmic_boost_voltage_out_t boost_voltage;
          fast_pmic_bypass_mode_t bypass_mode;
          fast_pmic_boost_current_limit_t current_limit;
          boost_voltage  = (fast_pmic_boost_voltage_out_t) ((uint32_t *)arg)[0];
          bypass_mode  = (fast_pmic_bypass_mode_t )((uint32_t *)arg)[1];
          current_limit  = (fast_pmic_boost_current_limit_t )((uint32_t *)arg)[2];
          ret = fast_pmic_boost_configure(boost_voltage, bypass_mode, current_limit);
        }
        break;
      case FAST_IOCTL_PMIC_HVLDO_CONFIGURE:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          fast_pmic_voltage_out_t voltage_out;
          fast_pmic_bypass_mode_t bypass_mode;
          bool current_limit_enable;
          voltage_out = (fast_pmic_voltage_out_t) ((uint32_t *)arg)[0];
          bypass_mode = (fast_pmic_bypass_mode_t) ((uint32_t *)arg)[1];
          current_limit_enable = (bool) ((uint32_t *)arg)[2];
          ret = fast_pmic_sysldo_configure(voltage_out, bypass_mode, current_limit_enable);
        }
        break;
      case FAST_IOCTL_PMIC_VRAIL_CONFIGURE:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          fast_pmic_ldo_id_t ldo_number;
          bool bypass_enable;
          bool current_limit_enable;
          ldo_number = (fast_pmic_ldo_id_t)((uint32_t *)arg)[0];
          bypass_enable = (bool)((uint32_t *)arg)[1];
          current_limit_enable = (bool)((uint32_t *)arg)[2];
          ret = fast_pmic_ldo_configure(ldo_number, bypass_enable, current_limit_enable);
        }
        break;
      case FAST_IOCTL_PMIC_LDO_ENABLE:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          uint8_t ldo_mask;
          ldo_mask = (uint8_t)((uint32_t *)arg)[0];
          ret = fast_pmic_ldo_enable((fast_ldo_mask_u)ldo_mask);
        }
        break;
      case FAST_IOCTL_PMIC_LDO_DISABLE:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          uint8_t ldo_mask;
          ldo_mask = (uint8_t)((uint32_t *)arg)[0];
          ret = fast_pmic_ldo_disable((fast_ldo_mask_u)ldo_mask);
        }
        break;
      case FAST_IOCTL_PMIC_VRAIL_VOUT:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          fast_pmic_ldo_id_t ldo_number;
          fast_pmic_voltage_out_t voltage_out;
          ldo_number = (fast_pmic_ldo_id_t)((uint32_t *)arg)[0];
          voltage_out = (fast_pmic_voltage_out_t)((uint32_t *)arg)[1];
          ret = fast_pmic_ldo_vout(ldo_number, voltage_out);
        }
        break;
      case FAST_IOCTL_PMIC_VRAIL_VOUT_GET:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          fast_pmic_ldo_id_t ldo_number;
          ldo_number = (fast_pmic_ldo_id_t)((uint32_t *)arg)[0];
          ret = fast_pmic_ldo_vout_get(ldo_number, (fast_pmic_voltage_out_t *) & ((uint32_t *)arg)[1]);
        }
        break;
      case FAST_IOCTL_PMIC_THERMAL_MONITOR_CONFIGURE:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          fast_pmic_thermal_alarm_temp_t alarm_temperature;
          fast_pmic_thermal_shutdown_temp_t shutdown_temperature;
          alarm_temperature = (fast_pmic_thermal_alarm_temp_t)((uint32_t *)arg)[0];
          shutdown_temperature = (fast_pmic_thermal_shutdown_temp_t)((uint32_t *)arg)[1];
          ret = fast_pmic_thermal_monitor_configure(alarm_temperature, shutdown_temperature);
        }
        break;
      case FAST_IOCTL_PMIC_THERMAL_MONITOR_ENABLE:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          ret = fast_pmic_thermal_monitor_enable();
        }
        break;
      case FAST_IOCTL_PMIC_THERMAL_MONITOR_DISABLE:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          ret = fast_pmic_thermal_monitor_disable();
        }
        break;
      case FAST_IOCTL_CLEAR_FAULT_INTERRUPT:
        {
          fast_fault_status_mask_u *data_ptr;
          data_ptr = (fast_fault_status_mask_u *)arg;
          fastinfo("\t%s\n", str_cmd(cmd));
          ret = fast_fault_clear_interrupt(data_ptr);
        }
        break;
      case FAST_IOCTL_GET_FAULT_STATUS:
        {
          fast_fault_status_mask_u *data_ptr;
          data_ptr = (fast_fault_status_mask_u *)arg;
          fastinfo("\t%s\n", str_cmd(cmd));
          ret = fast_fault_get_status(data_ptr);
        }
        break;
      case FAST_IOCTL_ENABLE_FAULT_INTERRUPT:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          fast_fault_status_mask_u fault_status_mask;
          fault_status_mask = ((fast_fault_status_mask_u)((uint16_t)((uint32_t *)arg)[0]));
          ret = fast_fault_enable_interrupt(fault_status_mask);
        }
        break;
      case FAST_IOCTL_GPIO_CONFIGURE_PIN:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          fast_gpio_port_t gpio_port;
          fast_gpio_pin_t gpio_pin;
          fast_gpio_functions_t gpio_function;
          fast_gpio_pin_level_t gpio_level;
          gpio_port  = ((fast_gpio_port_t)((uint32_t *)arg)[0]);
          gpio_pin  = ((fast_gpio_pin_t)((uint32_t *)arg)[1]);
          gpio_function = ((fast_gpio_functions_t)((uint32_t *)arg)[2]);
          gpio_level = ((fast_gpio_pin_level_t)((uint32_t *)arg)[3]);
          ret = fast_gpio_exp_configure_pin(gpio_port, gpio_pin, gpio_function, gpio_level);
        }
        break;
      case FAST_IOCTL_GPIO_SET_PIN_LEVEL:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          fast_gpio_port_t gpio_port;
          fast_gpio_pin_t gpio_pin;
          fast_gpio_pin_level_t gpio_level;
          gpio_port  = ((fast_gpio_port_t)((uint32_t *)arg)[0]);
          gpio_pin  = ((fast_gpio_pin_t)((uint32_t *)arg)[1]);
          gpio_level = ((fast_gpio_pin_level_t)((uint32_t *)arg)[2]);
          ret = fast_gpio_exp_set_pin(gpio_port, gpio_pin, gpio_level);
        }
        break;
      case FAST_IOCTL_GPIO_GET_PIN_LEVEL:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          fast_gpio_port_t gpio_port;
          fast_gpio_pin_t gpio_pin;
          fast_gpio_pin_level_t *gpio_level;
          gpio_port  = ((fast_gpio_port_t)((uint32_t *)arg)[0]);
          gpio_pin  = ((fast_gpio_pin_t)((uint32_t *)arg)[1]);
          gpio_level = ((fast_gpio_pin_level_t *) & ((uint32_t *)arg)[2]);
          ret = fast_gpio_exp_get_pin(gpio_port, gpio_pin, gpio_level);
        }
        break;
      case FAST_IOCTL_GPIO_DISABLE_PIN:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          fast_gpio_port_t gpio_port;
          fast_gpio_pin_t gpio_pin;
          gpio_port  = ((fast_gpio_port_t)((uint32_t *)arg)[0]);
          gpio_pin  = ((fast_gpio_pin_t)((uint32_t *)arg)[1]);
          ret = fast_gpio_exp_disable_pin(gpio_port, gpio_pin);
        }
        break;
      case FAST_IOCTL_GPIO_CLEAR_IRQ:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          fast_gpio_interrupt_status_u *int_status ;
          int_status =  ((fast_gpio_interrupt_status_u *) & ((uint16_t *)arg)[0]);
          ret = fast_gpio_exp_clear_irq(int_status);
        }
        break;
      case FAST_IOCTL_SPI_CONFIGURE:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          fast_spi_bit_order_t bit_order;
          fast_spi_bit_mode_t bit_mode;
          fast_spi_cpol_t cpol;
          fast_spi_cpha_t cpha;
          bit_order = ((fast_spi_bit_order_t)((uint32_t *)arg)[0]);
          bit_mode = ((fast_spi_bit_mode_t)((uint32_t *)arg)[1]);
          cpol = ((fast_spi_cpol_t)((uint32_t *)arg)[2]);
          cpha = ((fast_spi_cpha_t)((uint32_t *)arg)[3]);
          ret = fast_spi_configure(bit_order, bit_mode, cpol, cpha);
          // store configuration locally
          g_fastdev_p->spi_bit_mode = bit_mode;
        }
        break;
      case FAST_IOCTL_I2C_CONFIGURE:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          bool i2c_enable;
          uint8_t i2c_address;
          i2c_enable = ((bool)((uint32_t *)arg)[0]);
          i2c_address = ((uint8_t)((uint32_t *)arg)[1]);
          ret = fast_i2c_config(i2c_enable, i2c_address);
        }
        break;
      case FAST_IOCTL_INTERFACE_SET:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          uint8_t fast_interface = ((uint8_t)((uint32_t *)arg)[0]);
          /* Check if the interface is valid */
          if ((fast_interface < FAST_SPI_INTERFACE) || (fast_interface > FAST_JTAG_INTERFACE))
            {
              return -1;
            }
          g_fastdev_p->interface = fast_interface;
        }
        break;
      case FAST_IOCTL_INTERFACE_GET:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          *(uint32_t *)arg = (uint32_t)g_fastdev_p->interface;
        }
        break;
      case FAST_IOCTL_LPM_FSM_CONTROL:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          bool enable;
          enable = ((bool)((uint32_t *)arg)[0]);
          ret = fast_pm_lpm_fsm_control(enable);
        }
        break;
      case FAST_IOCTL_LPM_FSM_TRIGGER:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          ret = fast_pm_trigger_lpm_fsm();
        }
        break;
      case FAST_IOCTL_GROUP_MODE_CONFIG:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          uint16_t group_mode_config = 0;
          group_mode_config = ((uint32_t *)arg)[0];
          ret = fast_config_groupmode(group_mode_config);
        }
        break;
      case FAST_IOCTL_CONNECT_PERIPHERAL:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          uint8_t config_file_id = (uint8_t)((uint32_t *)arg)[0];
          uint16_t peri_id = (uint16_t)((uint32_t *)arg)[1];
          ret = fast_connect_peripheral(config_file_id, peri_id);
        }
        break;
      case FAST_IOCTL_DISCONNECT_PERIPHERAL:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          uint8_t config_file_id = (uint8_t)((uint32_t *)arg)[0];
          uint16_t peri_id = (uint16_t)((uint32_t *)arg)[1];
          ret = fast_disconnect_peripheral(config_file_id, peri_id);
        }
        break;
      case FAST_IOCTL_CONNECT_CHIP_VRAIL:
        {
          fastinfo("\t%s\n", str_cmd(cmd));
          uint8_t config_file_id = (uint8_t)((uint32_t *)arg)[0];
          uint16_t chip_id = (uint16_t)((uint32_t *)arg)[1];
          uint16_t vrail_id = (uint16_t)((uint32_t *)arg)[2];
          ret = fast_chip_vrail_connect(config_file_id, chip_id, vrail_id);
        }
        break;
#endif //CONFIG_ZEUS2
      default:
        {
          fasterr("Unknown command: %d\n", cmd);
          return -1;
        }
        break;
    }

  FAST_EXIT();
  return ret;
}

#ifndef CONFIG_DISABLE_POLL
static int fast_poll(FAR struct file *filep, struct pollfd *fds, bool setup)
{
  FAST_ENTER();
  set_errno(-ENOSYS);
  FAST_EXIT();
  return !OK;
}
#endif



/******************************************************************************
 * @brief register fast api driver
 *
 *****************************************************************************/
#if defined(CONFIG_SYSTEM_FAST_DRIVER)
int32_t fast_register(FAR struct zg_fast_driver_s *fastapi_driver, FAR struct zg_fast_config_s *config)
{
  int status = 0;
  memset(&g_fastdev, 0, sizeof(struct zg_fast_dev_s));
  sem_init(&g_fastdev_p->dev_sem, 0, 1);
  sem_setprotocol(&g_fastdev_p->dev_sem, SEM_PRIO_NONE);
  memcpy(&(g_fastdev.config), config, sizeof(struct zg_fast_config_s));
  g_fastdev_p->interface = FAST_INVALID_INTERFACE;

#if defined(CONFIG_FAST_SPI)
  FAR struct spi_dev_s *spi = fastapi_driver->spi;

  /* Setup SPI frequency and mode */
#ifdef CONFIG_FASTAPI_125KHZ
  g_fastdev_p->spi_frequency = 125000;
#elif defined(CONFIG_FASTAPI_250KHZ)
  g_fastdev_p->spi_frequency = 250000;
#elif defined(CONFIG_FASTAPI_500KHZ)
  g_fastdev_p->spi_frequency = 500000;
#elif defined(CONFIG_FASTAPI_1MHZ)
  g_fastdev_p->spi_frequency = 1000000;
#elif defined(CONFIG_FASTAPI_2MHZ)
  g_fastdev_p->spi_frequency = 2000000;
#elif defined(CONFIG_FASTAPI_4MHZ)
  g_fastdev_p->spi_frequency = 4000000;
#elif defined(CONFIG_FASTAPI_8MHZ)
  g_fastdev_p->spi_frequency = 8000000;
#else
#  error No SPI frequency for the FAST is defined.
#endif
  /* Setup SPI mode */
  SPI_SETMODE(spi, SPIDEV_MODE0);

  g_fastdev_p->spi = spi;
  g_fastdev_p->spi_bit_mode = FAST_SPI_8_BIT_MODE;
  g_fastdev_p->interface = FAST_SPI_INTERFACE;

#endif /* endif of CONFIG_FAST_SPI*/

#if defined(CONFIG_FAST_I2C)
  FAR struct i2c_master_s *i2c = fastapi_driver->i2c;
  g_fastdev_p->i2c = i2c;
  g_fastdev_p->interface = FAST_I2C_INTERFACE;

#endif /* endif of CONFIG_FAST_I2C*/

#ifdef CONFIG_FAST_ULPM_WAKEUP_PIN
  nrf_gpio_cfg_output(BOARD_ULPM_WAKEUP_PIN);
  gpio_pin_write(BOARD_ULPM_WAKEUP_PIN, 0);
#endif

#ifdef CONFIG_ZEUS2_DEVKIT_FEATURES

  nrf_gpio_cfg_input(BOARD_FAST_PWR_CLK_RDY, NRF_GPIO_PIN_PULLDOWN);
  nrf_gpio_cfg_input(BOARD_FAST_FAST_RDY, NRF_GPIO_PIN_PULLDOWN);
  nrf_gpio_cfg_output(BOARD_FAST_BOOTCFG_BYP);
  gpio_pin_write(BOARD_FAST_BOOTCFG_BYP, 0);
  nrf_gpio_cfg_output(BOARD_NRF_EN_SW);
  nrf_gpio_cfg_output(BOARD_FAST_ZIP_EN_L);
  gpio_pin_write(BOARD_FAST_ZIP_EN_L, 0);
  nrf_gpio_cfg_output(BOARD_FAST_ZIP_EN_H);
  gpio_pin_write(BOARD_FAST_ZIP_EN_H, 0);
  gpio_pin_write(BOARD_NRF_EN_SW, 1);
  nrf_gpio_cfg_input(BOARD_FAST_PWRCFG_1, NRF_GPIO_PIN_PULLDOWN);
  nrf_gpio_cfg_input(BOARD_FAST_PWRCFG_2, NRF_GPIO_PIN_PULLDOWN);
  nrf_gpio_cfg_input(BOARD_FAST_PWRCFG_3, NRF_GPIO_PIN_PULLDOWN);
  nrf_gpio_cfg_input(BOARD_FAST_PWRCFG_4, NRF_GPIO_PIN_PULLDOWN);
#endif

  status = register_driver(fast_dev_path, FAR & g_fast_ops, 0644, FAR (void *) &g_fastdev);

  return status;
}
#endif


