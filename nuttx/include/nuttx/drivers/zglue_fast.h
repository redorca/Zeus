/****************************************************************************
 * drivers/fast_debug/zeus_fast.h
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

#ifndef __INCLUDE_NUTTX_DRIVERS_ZGLUE_FAST_H
#define __INCLUDE_NUTTX_DRIVERS_ZGLUE_FAST_H

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#include <nuttx/zglue_fast/fast_api.h>

#if defined(CONFIG_FAST_SPI)
#include <nuttx/spi/spi.h>
#endif

#if defined(CONFIG_FAST_I2C)
#include <nuttx/i2c/i2c_master.h>
#endif
/*
 * To simplify code and flow setup general defines for
 * these debug helpers in a default inactive state and
 * enable only those features configured in.
 */
#define fasterr(x...)
#define fastwarn(format, ...)
#define fastinfo(format, ...)
#define FAST_ENTER()
#define FAST_EXIT()

#ifdef CONFIG_DEBUG_FEATURES
#  ifdef CONFIG_DEBUG_FAST_DRIVER
#    undef fasterr
#    undef fastwarn
#    undef fastinfo
#    define fasterr(format, ...)     _err(format, ##__VA_ARGS__)
#    define fastwarn(format, ...)    _warn(format, ##__VA_ARGS__)
#    define fastinfo(format, ...)    _info(format, ##__VA_ARGS__)
#  endif ## DEBUG_FAST_DRIVER
#  ifdef CONFIG_DEBUG_FAST_INOUTRO
#    undef FAST_ENTER
#    undef FAST_EXIT
#    define FAST_ENTER()             _err("%s\t::Enter ...\n", __func__)
#    define FAST_EXIT()              _warn("%s\t::Exit.\n", __func__)
#  endif ## DEBUG_FAST_INOUTRO
#endif ## DEBUG_FEATURES

#define FAST_DEV_PATH  "/dev/fast"
#define FAST_IDLEN   32
#define ZEUS2_REG_LEN 4  /* 4 bytes for the data*/

/* FAST driver ioctl definitions *********************************************/
#define _FASTIOCVALID(c)                        (_IOC_TYPE(c)==_FASTBASE)
#define _FASTIOC(nr)                            _IOC(_FASTBASE,nr)

#define FAST_IOCTL_CMD_BASE         0   /* 20 general commands */
#define FAST_IOCTL_PMIC_CMD_BASE      20  /* 20 PMIC commands */
#define FAST_IOCTL_LED_CMD_BASE       40  /* 10 LED commands */
#define FAST_IOCTL_GPIO_EXP_CMD_BASE    50  /* 10 GPIO exp commands */
#define FAST_IOCTL_OTP_CMD_BASE       60  /* 10 OTP commands */
#define FAST_IOCTL_INTERFACE_CMD_BASE     70  /* 10 FAST interface commands */
#define FAST_IOCTL_READ_CMD_BASE      80  /* 20 FAST read commands */
#define FAST_IOCTL_WRITE_CMD_BASE       100  /* 20 FAST write commands */
#define FAST_IOCTL_CLEAR_CMD_BASE       120  /* 20 FAST clear commands */
#define FAST_IOCTL_CFG_FILE_BASE        130  /* 10 FAST cfg file commands */
#if defined(CONFIG_ZEUS2_DEVKIT_FEATURES) && defined(CONFIG_ZEUS2)
#define FAST_IOCTL_DEVKIT_FEATURE_BASE  140  /* 10 FAST devkit feature commands */
#endif

#define  FAST_IOCTL_API_INIT                      _FASTIOC(FAST_IOCTL_CMD_BASE+1)
#define  FAST_IOCTL_API_CLOSE                     _FASTIOC(FAST_IOCTL_CMD_BASE+2)
#define  FAST_IOCTL_GET_SYSTEM_INFORMATION        _FASTIOC(FAST_IOCTL_CMD_BASE+3)
#define  FAST_IOCTL_GET_SYSTEM_CHIPS_INFORMATION  _FASTIOC(FAST_IOCTL_CMD_BASE+4)
#define  FAST_IOCTL_CONNECT_SYSTEM                _FASTIOC(FAST_IOCTL_CMD_BASE+5)
#define  FAST_IOCTL_DISCONNECT_SYSTEM             _FASTIOC(FAST_IOCTL_CMD_BASE+6)
#define  FAST_IOCTL_CONNECT_CHIP                  _FASTIOC(FAST_IOCTL_CMD_BASE+7)
#define  FAST_IOCTL_DISCONNECT_CHIP               _FASTIOC(FAST_IOCTL_CMD_BASE+8)
#define  FAST_IOCTL_READ_ID                       _FASTIOC(FAST_IOCTL_CMD_BASE+9)
#define  FAST_IOCTL_ENTER_POWER_STATE             _FASTIOC(FAST_IOCTL_CMD_BASE+10)
#define  FAST_IOCTL_GET_CURRENT_POWER_STATE       _FASTIOC(FAST_IOCTL_CMD_BASE+11)
#define  FAST_IOCTL_SET_TIMEOUT                   _FASTIOC(FAST_IOCTL_CMD_BASE+12)
#define  FAST_IOCTL_LPM_FSM_CONTROL               _FASTIOC(FAST_IOCTL_CMD_BASE+13)
#define  FAST_IOCTL_LPM_FSM_TRIGGER               _FASTIOC(FAST_IOCTL_CMD_BASE+14)
#define  FAST_IOCTL_GROUP_MODE_CONFIG             _FASTIOC(FAST_IOCTL_CMD_BASE+15)
#ifdef CONFIG_ZEUS1
#define  FAST_IOCTL_ZEUS1_RESET                   _FASTIOC(FAST_IOCTL_CMD_BASE+16)
#endif
#define  FAST_IOCTL_CONNECT_PERIPHERAL            _FASTIOC(FAST_IOCTL_CMD_BASE+17)
#define  FAST_IOCTL_DISCONNECT_PERIPHERAL         _FASTIOC(FAST_IOCTL_CMD_BASE+18)
#define  FAST_IOCTL_REALIGNMENT                   _FASTIOC(FAST_IOCTL_CMD_BASE+19)
#define  FAST_IOCTL_CONNECT_CHIP_VRAIL            _FASTIOC(FAST_IOCTL_CMD_BASE+20)


/* FAST read commands */
#define  FAST_IOCTL_READ_REG                      _FASTIOC(FAST_IOCTL_READ_CMD_BASE+1)
#define  FAST_IOCTL_READ_TILE                     _FASTIOC(FAST_IOCTL_READ_CMD_BASE+2)
#define  FAST_IOCTL_READ_PERIPHERAL               _FASTIOC(FAST_IOCTL_READ_CMD_BASE+3)
#define  FAST_IOCTL_READ_CUSTOMER_OTP             _FASTIOC(FAST_IOCTL_READ_CMD_BASE+4)
#define  FAST_IOCTL_READ_FAST_OTP                 _FASTIOC(FAST_IOCTL_READ_CMD_BASE+5)
#define  FAST_IOCTL_SCANTILE                      _FASTIOC(FAST_IOCTL_READ_CMD_BASE+6)
#define  FAST_IOCTL_SCAN_PERIPHERAL               _FASTIOC(FAST_IOCTL_READ_CMD_BASE+7)
#define  FAST_IOCTL_SCAN_REGS                     _FASTIOC(FAST_IOCTL_READ_CMD_BASE+8)

/* FAST write commands */
#define  FAST_IOCTL_WRITE_REG                     _FASTIOC(FAST_IOCTL_WRITE_CMD_BASE+1)
#define  FAST_IOCTL_WRITE_TILE                    _FASTIOC(FAST_IOCTL_WRITE_CMD_BASE+2)
#define  FAST_IOCTL_WRITE_PERIPHERAL              _FASTIOC(FAST_IOCTL_WRITE_CMD_BASE+3)
#define  FAST_IOCTL_WRITE_CUSTOMER_OTP            _FASTIOC(FAST_IOCTL_WRITE_CMD_BASE+4)
#define  FAST_IOCTL_WRITE_FAST_OTP                _FASTIOC(FAST_IOCTL_WRITE_CMD_BASE+5)
#define  FAST_IOCTL_PROGRAM                       _FASTIOC(FAST_IOCTL_WRITE_CMD_BASE+6)
#define  FAST_IOCTL_ZCAD_PROGRAM                  _FASTIOC(FAST_IOCTL_WRITE_CMD_BASE+7)
#define  FAST_IOCTL_ZCAD_VERSION                  _FASTIOC(FAST_IOCTL_WRITE_CMD_BASE+8)

/* FAST clear commands */
#define  FAST_IOCTL_CLEAR_TILE                    _FASTIOC(FAST_IOCTL_CLEAR_CMD_BASE+1)
#define  FAST_IOCTL_CLEAR_PERIPHERAL              _FASTIOC(FAST_IOCTL_CLEAR_CMD_BASE+2)

/*LED IOCTL calls */
#define  FAST_IOCTL_CONFIGURE_LED                 _FASTIOC(FAST_IOCTL_LED_CMD_BASE+1)
#define  FAST_IOCTL_ENABLE_LED                    _FASTIOC(FAST_IOCTL_LED_CMD_BASE+2)
#define  FAST_IOCTL_DISABLE_LED                   _FASTIOC(FAST_IOCTL_LED_CMD_BASE+3)

/* PMIC IOCTL calls */
#define  FAST_IOCTL_PMIC_BOOST_CONFIGURE          _FASTIOC(FAST_IOCTL_PMIC_CMD_BASE+1)
#define  FAST_IOCTL_PMIC_HVLDO_CONFIGURE          _FASTIOC(FAST_IOCTL_PMIC_CMD_BASE+2)
#define  FAST_IOCTL_PMIC_VRAIL_CONFIGURE          _FASTIOC(FAST_IOCTL_PMIC_CMD_BASE+3)
#define  FAST_IOCTL_PMIC_THERMAL_MONITOR_CONFIGURE   _FASTIOC(FAST_IOCTL_PMIC_CMD_BASE+4)
#define  FAST_IOCTL_PMIC_THERMAL_MONITOR_ENABLE   _FASTIOC(FAST_IOCTL_PMIC_CMD_BASE+5)
#define  FAST_IOCTL_PMIC_THERMAL_MONITOR_DISABLE  _FASTIOC(FAST_IOCTL_PMIC_CMD_BASE+6)
#define  FAST_IOCTL_CLEAR_FAULT_INTERRUPT         _FASTIOC(FAST_IOCTL_PMIC_CMD_BASE+7)
#define  FAST_IOCTL_GET_FAULT_STATUS              _FASTIOC(FAST_IOCTL_PMIC_CMD_BASE+8)
#define  FAST_IOCTL_ENABLE_FAULT_INTERRUPT        _FASTIOC(FAST_IOCTL_PMIC_CMD_BASE+9)
#define  FAST_IOCTL_PMIC_VRAIL_VOUT               _FASTIOC(FAST_IOCTL_PMIC_CMD_BASE+10)
#define  FAST_IOCTL_PMIC_LDO_ENABLE               _FASTIOC(FAST_IOCTL_PMIC_CMD_BASE+11)
#define  FAST_IOCTL_PMIC_LDO_DISABLE              _FASTIOC(FAST_IOCTL_PMIC_CMD_BASE+12)
#define  FAST_IOCTL_PMIC_VRAIL_VOUT_GET           _FASTIOC(FAST_IOCTL_PMIC_CMD_BASE+13)


/* GPIO exp IOCTL calls */
#define  FAST_IOCTL_GPIO_CONFIGURE_PIN            _FASTIOC(FAST_IOCTL_GPIO_EXP_CMD_BASE+1)
#define  FAST_IOCTL_GPIO_SET_PIN_LEVEL            _FASTIOC(FAST_IOCTL_GPIO_EXP_CMD_BASE+2)
#define  FAST_IOCTL_GPIO_GET_PIN_LEVEL            _FASTIOC(FAST_IOCTL_GPIO_EXP_CMD_BASE+3)
#define  FAST_IOCTL_GPIO_CLEAR_IRQ                _FASTIOC(FAST_IOCTL_GPIO_EXP_CMD_BASE+4)
#define  FAST_IOCTL_GPIO_DISABLE_PIN              _FASTIOC(FAST_IOCTL_GPIO_EXP_CMD_BASE+5)
#define  FAST_IOCTL_SET_DEBUG_LEVEL               _FASTIOC(FAST_IOCTL_GPIO_EXP_CMD_BASE+6)
#define  FAST_IOCTL_GET_DEBUG_LEVEL               _FASTIOC(FAST_IOCTL_GPIO_EXP_CMD_BASE+7)

/* FAST interface commands */
#define  FAST_IOCTL_SPI_CONFIGURE                 _FASTIOC(FAST_IOCTL_INTERFACE_CMD_BASE+1)
#define  FAST_IOCTL_I2C_CONFIGURE                 _FASTIOC(FAST_IOCTL_INTERFACE_CMD_BASE+2)
#define  FAST_IOCTL_INTERFACE_SET                 _FASTIOC(FAST_IOCTL_INTERFACE_CMD_BASE+3)
#define  FAST_IOCTL_INTERFACE_GET                 _FASTIOC(FAST_IOCTL_INTERFACE_CMD_BASE+4)

/* FAST config file commands */
#define  FAST_IOCTL_CFG_FILE_ERASE                 _FASTIOC(FAST_IOCTL_CFG_FILE_BASE+1)
#define  FAST_IOCTL_CFG_FILE_UPDATE                _FASTIOC(FAST_IOCTL_CFG_FILE_BASE+2)

#if defined(CONFIG_ZEUS2_DEVKIT_FEATURES) && defined(CONFIG_ZEUS2)
/* FAST dev kit feature commands */
#define  FAST_IOCTL_DEVKIT_ZEUS2_STATUS            _FASTIOC(FAST_IOCTL_DEVKIT_FEATURE_BASE+1)
#define  FAST_IOCTL_DEVKIT_ZEUS2_PWRON             _FASTIOC(FAST_IOCTL_DEVKIT_FEATURE_BASE+2)
#define  FAST_IOCTL_DEVKIT_ZEUS2_PWROFF            _FASTIOC(FAST_IOCTL_DEVKIT_FEATURE_BASE+3)
#define  FAST_IOCTL_DEVKIT_ZEUS2_BOOTCFG_BYP       _FASTIOC(FAST_IOCTL_DEVKIT_FEATURE_BASE+4)
#define  FAST_IOCTL_DEVKIT_ZEUS2_nRF_EN_SW         _FASTIOC(FAST_IOCTL_DEVKIT_FEATURE_BASE+5)
#endif

typedef struct __attribute__((packed)) fast_data_t
{
  uint32_t  system_config_file_id;
  uint16_t  chip_id;
} fast_data_t;

typedef struct __attribute__((packed)) fast_register_param_t
{
  uint8_t dev_id;
  uint32_t reg_addr;
  uint8_t data[ZEUS2_REG_LEN];
  uint16_t len;
} fast_register_param_t;

/* FAST PMIC thermal shutdown temperature */
typedef enum
{
  FAST_SPI_INTERFACE  = 0,
  FAST_I2C_INTERFACE,
  FAST_JTAG_INTERFACE,
  FAST_INVALID_INTERFACE,
} fast_interface_t;

struct zg_fast_config_s
{
  /* Since multiple device can be connected to the same SPI bus we need
   * to use multiple spi device ids which are employed by NuttX to select/
   * deselect the desired device via their chip select inputs.
   */
  int spi_devid;
};

struct zg_fast_driver_s
{
  /* Since multiple device can be connected to the same SPI bus we need
   * to use multiple spi device ids which are employed by NuttX to select/
   * deselect the desired device via their chip select inputs.
   */
  struct spi_dev_s *spi;
  struct i2c_master_s *i2c;
};

struct zg_fast_dev_s
{
  int     id;
  char    *buf;
  ssize_t  count;
  ssize_t  buflen;
  sem_t    dev_sem;
  fast_spi_bit_mode_t spi_bit_mode;
  struct spi_dev_s *spi;          /* Pointer to the SPI instance */
  struct i2c_master_s *i2c;          /* Pointer to the I2C instance */
  struct zg_fast_config_s config; /* configuration of the fastapi device */
  fast_interface_t interface;
  uint32_t i2c_frequency;         /* i2c frequency */
  uint32_t spi_frequency;         /* spi frequency */
};


#if defined(CONFIG_DEBUG_FEATURES)
#define str_cmd(a) strings_ioctl_cmd[a]
#else
#define str_cmd(a)
#endif /* CONFIG_DEBUG_FEATURES */

/* Breaks down a uint32_t into 4 uint8_t */
#define FORMAT_ADDR(a)  {(a & 0xFF), ((a >> 8) & 0xFF), ((a >> 16) & 0xFF), ((a >> 24) & 0xFF)}

int32_t fast_register(FAR struct zg_fast_driver_s *spi, FAR struct zg_fast_config_s *config);

fast_status_e fast_config_file_erase(void);




#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_DRIVERS_ZGLUE_FAST_H */

