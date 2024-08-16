/****************************************************************************
 * include/nuttx/sensors/internal_mc3672.h
 * low level driver for the mc3672 accelerometer.
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


#ifndef NUTTX_SENSORS_INTERNAL_MC3672_H__
#define NUTTX_SENSORS_INTERNAL_MC3672_H__

#include <nuttx/config.h>

#include <stdint.h>
#include <stddef.h>

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/*******************************************************************************
 *** CONSTANT / DEFINE
 *******************************************************************************/
#define MC3672_RETCODE_SUCCESS                 (0)
#define MC3672_RETCODE_ERROR_BUS               (-1)
#define MC3672_RETCODE_ERROR_NULL_POINTER      (-2)
#define MC3672_RETCODE_ERROR_STATUS            (-3)
#define MC3672_RETCODE_ERROR_SETUP             (-4)
#define MC3672_RETCODE_ERROR_GET_DATA          (-5)
#define MC3672_RETCODE_ERROR_IDENTIFICATION    (-6)
#define MC3672_RETCODE_ERROR_WRONG_ARGUMENT    (-7)


#define MC3672_FIFO_DEPTH                      32

#define MC3672_FEATURE_DISABLE                 (0X00)
#define MC3672_FEATURE_ENABLE                  (0X01)

/*******************************************************************************
 *** Register Map
 *******************************************************************************/
//=============================================
#define MC3672_REG_EXT_STAT_1       (0x00)
#define MC3672_REG_EXT_STAT_2       (0x01)
#define MC3672_REG_XOUT_LSB         (0x02)
#define MC3672_REG_XOUT_MSB         (0x03)
#define MC3672_REG_YOUT_LSB         (0x04)
#define MC3672_REG_YOUT_MSB         (0x05)
#define MC3672_REG_ZOUT_LSB         (0x06)
#define MC3672_REG_ZOUT_MSB         (0x07)
#define MC3672_REG_STATUS_1         (0x08)
#define MC3672_REG_STATUS_2         (0x09)

#define MC3672_REG_FREG_1           (0x0D)
#define MC3672_REG_FREG_2           (0x0E)
#define MC3672_REG_INIT_1           (0x0F)
#define MC3672_REG_MODE_C           (0x10)
#define MC3672_REG_RATE_1           (0x11)
#define MC3672_REG_SNIFF_C          (0x12)
#define MC3672_REG_SNIFFTH_C        (0x13)
#define MC3672_REG_SNIFFCF_C        (0x14)
#define MC3672_REG_RANGE_C          (0x15)
#define MC3672_REG_FIFO_C           (0x16)
#define MC3672_REG_INTR_C           (0x17)

#define MC3672_REG_INIT_3           (0x1A)
#define MC3672_REG_SCRATCH          (0x1B)
#define MC3672_REG_PMCR             (0x1C)

#define MC3672_REG_DMX              (0x20)
#define MC3672_REG_DMY              (0x21)
#define MC3672_REG_DMZ              (0x22)

#define MC3672_REG_RESET            (0x24)

#define MC3672_REG_INIT_2           (0x28)
#define MC3672_REG_TRIGC            (0x29)

#define MC3672_REG_XOFFL            (0x2A)
#define MC3672_REG_XOFFH            (0x2B)
#define MC3672_REG_YOFFL            (0x2C)
#define MC3672_REG_YOFFH            (0x2D)
#define MC3672_REG_ZOFFL            (0x2E)
#define MC3672_REG_ZOFFH            (0x2F)
#define MC3672_REG_XGAIN            (0x30)
#define MC3672_REG_YGAIN            (0x31)
#define MC3672_REG_ZGAIN            (0x32)


#define MC3672_REG_MAP_SIZE         (64)

/*MODE control register*/
#define MC3672_MODE_C_MCTRL_POS     (0x00)
#define MC3672_MODE_C_MCTRL_MSK     (0x07 << MC3672_MODE_C_MCTRL_POS)

/*RANGE control register*/
#define MC3672_RANGE_C_RANGE_POS     (0x04)
#define MC3672_RANGE_C_RANGE_MSK     (0x07 << MC3672_RANGE_C_RANGE_POS)

#define MC3672_RANGE_C_RES_POS     (0x00)
#define MC3672_RANGE_C_RES_MSK     (0x07 << MC3672_RANGE_C_RES_POS)

/*PMCR register*/
#define MC3672_PMCR_CSPM_POS (0X00)
#define MC3672_PMCR_CSPM_MSK (0X07 << MC3672_PMCR_CSPM_POS)

#define MC3672_PMCR_SPM_POS (0X04)
#define MC3672_PMCR_SPM_MSK (0X07 << MC3672_PMCR_SPM_POS)

/*RATE 1 register*/
#define MC3672_RATE_1_RR_POS (0X00)
#define MC3672_RATE_1_RR_MSK (0X0F << MC3672_RATE_1_RR_POS)

/*feature 2 register*/
#define MC3672_FREG_2_BURST_POS (0x01)
#define MC3672_FREG_2_BURST_MSK (0x01<<MC3672_FREG_2_BURST_POS)

#define MC3672_FREG_2_FSTAT_POS (0x03)
#define MC3672_FREG_2_FSTAT_MSK (0x01<<MC3672_FREG_2_FSTAT_POS)

#define MC3672_FREG_2_STREAM_POS (0x05)
#define MC3672_FREG_2_STREAM_MSK (0x01<<MC3672_FREG_2_STREAM_POS)


/*FIFO control register*/
#define MC3672_FIFO_C_TH_POS (0X00)
#define MC3672_FIFO_C_TH_MSK (0X1F<<MC3672_FIFO_C_TH_POS)

#define MC3672_FIFO_C_MODE_POS (0X05)
#define MC3672_FIFO_C_MODE_MSK (0X01<<MC3672_FIFO_C_MODE_POS)

#define MC3672_FIFO_C_EN_POS (0X06)
#define MC3672_FIFO_C_EN_MSK (0X01<<MC3672_FIFO_C_EN_POS)

#define MC3672_FIFO_C_RESET_POS (0X07)
#define MC3672_FIFO_C_RESET_MSK (0X01<<MC3672_FIFO_C_RESET_POS)

/*interrupt control register*/
#define MC3672_INTR_C_IPP_POS (0X00)
#define MC3672_INTR_C_IPP_MSK (0X01<<MC3672_INTR_C_IPP_POS)

#define MC3672_INTR_C_IAH_POS (0X01)
#define MC3672_INTR_C_IAH_MSK (0X01<<MC3672_INTR_C_IAH_POS)

#define MC3672_INTR_C_WAKE_POS (0X02)
#define MC3672_INTR_C_WAKE_MSK (0X01<<MC3672_INTR_C_WAKE_POS)

#define MC3672_INTR_C_ACQ_POS (0X03)
#define MC3672_INTR_C_ACQ_MSK (0X01<<MC3672_INTR_C_ACQ_POS)

#define MC3672_INTR_C_EMPTY_POS (0X04)
#define MC3672_INTR_C_EMPTY_MSK (0X01<<MC3672_INTR_C_EMPTY_POS)

#define MC3672_INTR_C_FULL_POS (0X05)
#define MC3672_INTR_C_FULL_MSK (0X01<<MC3672_INTR_C_FULL_POS)

#define MC3672_INTR_C_THRESH_POS (0X06)
#define MC3672_INTR_C_THRESH_MSK (0X01<<MC3672_INTR_C_THRESH_POS)

#define MC3672_INTR_C_SWAKE_POS (0X07)
#define MC3672_INTR_C_SWAKE_MSK (0X01<<MC3672_INTR_C_SWAKE_POS)

/*status 2 register*/
#define MC3672_STATUS_2_WAKE_POS (0X02)
#define MC3672_STATUS_2_WAKE_MSK (0X01<<MC3672_STATUS_2_WAKE_POS)

#define MC3672_STATUS_2_ACQ_POS (0X03)
#define MC3672_STATUS_2_ACQ_MSK (0X01<<MC3672_STATUS_2_ACQ_POS)

#define MC3672_STATUS_2_EMPTY_POS (0X04)
#define MC3672_STATUS_2_EMPTY_MSK (0X01<<MC3672_STATUS_2_EMPTY_POS)

#define MC3672_STATUS_2_FULL_POS (0X05)
#define MC3672_STATUS_2_FULL_MSK (0X01<<MC3672_STATUS_2_FULL_POS)

#define MC3672_STATUS_2_THRESH_POS (0X06)
#define MC3672_STATUS_2_THRESH_MSK (0X01<<MC3672_STATUS_2_THRESH_POS)

#define MC3672_STATUS_2_SWAKE_POS (0X07)
#define MC3672_STATUS_2_SWAKE_MSK (0X01<<MC3672_STATUS_2_SWAKE_POS)

/*ZOUT_MSB register*/
#define MC3672_ZOUT_MSB_TH_POS (0X06)
#define MC3672_ZOUT_MSB_TH_MSK (0X01<<MC3672_ZOUT_MSB_TH_POS)

#define MC3672_ZOUT_MSB_FULL_POS (0X05)
#define MC3672_ZOUT_MSB_FULL_MSK (0X01<<MC3672_ZOUT_MSB_FULL_POS)


#define MC3672_ZOUT_MSB_EMPTY_POS (0X04)
#define MC3672_ZOUT_MSB_EMPTY_MSK (0X01<<MC3672_ZOUT_MSB_EMPTY_POS)

#define MC3672_ZOUT_MSB_SIG_POS (0X03)
#define MC3672_ZOUT_MSB_SIG_MSK (0X01<<MC3672_ZOUT_MSB_SIG_POS)

#define MC3672_ZOUT_MSB_FSTATUS_POS (0X04)
#define MC3672_ZOUT_MSB_FSTATUS_MSK (0X0F<<MC3672_ZOUT_MSB_FSTATUS_POS)
#define MC3672_ZOUT_MSB_SIG_EXPAND_POS 0x00
#define MC3672_ZOUT_MSB_SIG_EXPAND_NEG 0x0F



/********************************************************************************************
 * Public Types
 ********************************************************************************************/
typedef enum
{
  MC3672_MODE_SLEEP      = 0,
  MC3672_MODE_STANDBY    = 1,
  MC3672_MODE_SNIFF      = 2,
  MC3672_MODE_CWAKE      = 5,
  MC3672_MODE_SWAKE      = 6,
  MC3672_MODE_TRIG       = 7,
} MC3672_mode_t;

typedef enum
{
  MC3672_RANGE_2G    = 0,
  MC3672_RANGE_4G    = 1,
  MC3672_RANGE_8G    = 2,
  MC3672_RANGE_16G   = 3,
  MC3672_RANGE_12G   = 4,
  MC3672_RANGE_END,
}   MC3672_range_t;

typedef enum
{
  MC3672_RESOLUTION_6BIT    = 0,
  MC3672_RESOLUTION_7BIT    = 1,
  MC3672_RESOLUTION_8BIT    = 2,
  MC3672_RESOLUTION_10BIT   = 3,
  MC3672_RESOLUTION_12BIT   = 4,
  MC3672_RESOLUTION_14BIT   = 5,  //(Do not select if FIFO enabled)
  MC3672_RESOLUTION_END,
}   MC3672_resolution_t;

typedef enum
{
  MC3672_CWAKE_SR_HPM_START        = 4,
  MC3672_CWAKE_SR_HPM_14HZ,
  MC3672_CWAKE_SR_HPM_28Hz,
  MC3672_CWAKE_SR_HPM_55Hz,
  MC3672_CWAKE_SR_HPM_80Hz,
  MC3672_CWAKE_SR_HPM_END,
}   MC3672_cwake_sr_hpm_t;


typedef enum
{
  MC3672_CWAKE_SR_ULPM_START        = 5,
  MC3672_CWAKE_SR_ULPM_25Hz,
  MC3672_CWAKE_SR_ULPM_50Hz,
  MC3672_CWAKE_SR_ULPM_100Hz,
  MC3672_CWAKE_SR_ULPM_190Hz,
  MC3672_CWAKE_SR_ULPM_380Hz,
  MC3672_CWAKE_SR_ULPM_750Hz,
  MC3672_CWAKE_SR_ULPM_1100Hz,
  MC3672_CWAKE_SR_ULPM_END,
}   MC3672_cwake_sr_ulpm_t;


typedef enum
{
  MC3672_CWAKE_SR_LPM_START        = 4,
  MC3672_CWAKE_SR_LPM_14Hz,
  MC3672_CWAKE_SR_LPM_28Hz,
  MC3672_CWAKE_SR_LPM_54Hz,
  MC3672_CWAKE_SR_LPM_105Hz,
  MC3672_CWAKE_SR_LPM_210Hz,
  MC3672_CWAKE_SR_LPM_400Hz,
  MC3672_CWAKE_SR_LPM_600Hz,
  MC3672_CWAKE_SR_LPM_END,
}   MC3672_cwake_sr_lpm_t;


typedef enum
{
  MC3672_SNIFF_SR_0p4Hz       = 1,
  MC3672_SNIFF_SR_0p8Hz       = 2,
  MC3672_SNIFF_SR_1p5Hz       = 3,
  MC3672_SNIFF_SR_7Hz         = 4,
  MC3672_SNIFF_SR_14Hz        = 5,
  MC3672_SNIFF_SR_28Hz        = 6,
  MC3672_SNIFF_SR_54Hz        = 7,
  MC3672_SNIFF_SR_105Hz       = 8,
  MC3672_SNIFF_SR_210Hz       = 9,
  MC3672_SNIFF_SR_400Hz       = 10,
  MC3672_SNIFF_SR_600Hz       = 11,
  MC3672_SNIFF_SR_END,
}   MC3672_sniff_sr_t;

typedef enum
{
  MC3672_POWER_MODE_LOW = 0,
  MC3672_POWER_MODE_ULTRA_LOW = 3,
  MC3672_POWER_MODE_PRECISION = 4,
}   MC3672_wake_sniff_PM_t;

typedef enum
{
  MC3672_FIFO_CONTROL_DISABLE = 0,
  MC3672_FIFO_CONTROL_ENABLE,
  MC3672_FIFO_CONTROL_END,
}   MC3672_fifo_control_t;

typedef enum
{
  MC3672_FIFO_WRAP_02_07,
  MC3672_FIFO_WRAP_02_09,
  MC3672_FIFO_WRAP_END,
}   MC3672_fifo_warp_t;


typedef enum
{
  MC3672_FIFO_MODE_NORMAL = 0,
  MC3672_FIFO_MODE_WATERMARK,
  MC3672_FIFO_MODE_END,
}   MC3672_fifo_mode_t;

typedef enum
{
  MC3672_INTR_C_IPP_MODE_OPEN_DRAIN = 0,
  MC3672_INTR_C_IPP_MODE_PUSH_PULL,
  MC3672_INTR_C_IPP_MODE_END,
}   MC3672_INTR_C_IPP_t;

typedef enum
{
  MC3672_INTR_C_IAH_ACTIVE_LOW = 0,
  MC3672_INTR_C_IAH_ACTIVE_HIGH,
  MC3672_INTR_C_IAH_END,
}   MC3672_INTR_C_IAH_t;


/*!
 * @brief Interface selection Enums
 */
typedef enum mc3672_intf
{
  /*! SPI interface */
  MC3672_SPI_INTF,
  /*! I2C interface */
  MC3672_I2C_INTF
}   MC3672_intf_t;

struct mc3672_acc_t
{
  int16_t XAxis;
  int16_t YAxis;
  int16_t ZAxis;
};

struct mc3672_acc_reg_t
{
  uint8_t XOUT_LSB;
  uint8_t XOUT_MSB;
  uint8_t YOUT_LSB;
  uint8_t YOUT_MSB;
  uint8_t ZOUT_LSB;
  uint8_t ZOUT_MSB;
};


/*!
 * @brief cwake mode setting
 */
struct mc3672_cwake_settings
{
  MC3672_range_t range;
  MC3672_resolution_t resolution;
  uint8_t wake_sr;
  MC3672_wake_sniff_PM_t wake_pm;
};

struct mc3672_fifo_settings
{
  uint8_t fifo_freeze_en;/*not implement for now*/
  uint8_t fifo_stream_en;
  MC3672_fifo_mode_t fifo_mode;
  uint8_t fifo_threshold;
};

struct mc3672_int_settings
{
  MC3672_INTR_C_IPP_t INTN_PIN_IPP;
  MC3672_INTR_C_IAH_t INTN_PIN_IAH;
  uint8_t wake_int_en;
  uint8_t acq_int_en;
  uint8_t fifo_empty_int_en;
  uint8_t fifo_full_int_en;
  uint8_t fifo_thresh_int_en;
  uint8_t swake_int_en;
};


/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read and write functions of the user
 */
struct mc3672_dev;
typedef int8_t (*mc3672_com_fptr_t)(const struct mc3672_dev *dev,
                                    uint8_t reg_addr,
                                    uint8_t *data,
                                    uint16_t len);

/*! delay function pointer */
typedef void (*mc3672_delay_fptr_t)(const struct mc3672_dev *dev, uint32_t ms);

/*!
 * @brief mc3672 device structure
 */
struct mc3672_dev
{
  /*! Chip Id */
  uint8_t chip_id;
  /*! Device Id */
  uint8_t dev_id;
  /*! SPI/I2C Interface */
  MC3672_intf_t intf;
  /*! Bus read function pointer */
  mc3672_com_fptr_t read;
  /*! Bus write function pointer */
  mc3672_com_fptr_t write;
  /*! delay(in ms) function pointer */
  mc3672_delay_fptr_t delay_ms;
};

/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/
int8_t mc3672_init(struct mc3672_dev *dev);
int8_t mc3672_reset(struct mc3672_dev *dev);
int8_t mc3672_get_mode(struct mc3672_dev *dev, MC3672_mode_t *mode);
int8_t mc3672_set_mode(struct mc3672_dev *dev, MC3672_mode_t mode);
int8_t mc3672_cfg_cwake(struct mc3672_dev *dev, struct mc3672_cwake_settings *setting);
int8_t mc3672_cfg_fifo(struct mc3672_dev *dev, struct mc3672_fifo_settings *setting);
int8_t mc3672_cfg_int(struct mc3672_dev *dev, struct mc3672_int_settings *setting);
int8_t mc3672_get_int_status(struct mc3672_dev *dev, uint8_t *events);
int8_t mc3672_enable_fifo(struct mc3672_dev *dev, bool enable);
int8_t mc3672_reset_fifo(struct mc3672_dev *dev);
int8_t mc3672_read_fifo(struct mc3672_dev *dev,
                        struct mc3672_acc_t *sample,
                        uint8_t target_sample_count,
                        uint8_t *gotted_sample_count);
int8_t mc3672_read_raw(struct mc3672_dev *dev, struct mc3672_acc_t *raw);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* NUTTX_SENSORS_INTERNAL_MC3672_H__ */

