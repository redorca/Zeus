/****************************************************************************
 * drivers/sensors/internal_mc3672.c
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


#include <nuttx/config.h>

#include <string.h>
#include <stddef.h>
#include <stdio.h>

#include <nuttx/sensors/internal_mc3672.h>



#if (defined CONFIG_MC3672)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define MC3672_DEFAULT_MODE     MC3672_MODE_STANDBY
#define MC3672_DEFAULT_SAMPLE_RATE_CWAKE    MC3672_CWAKE_SR_54Hz
#define MC3672_DEFAULT_POWER_MODE_CWAKE     MC3672_POWER_MODE_LOW
#define MC3672_DEFAULT_RANGE    MC3672_RANGE_8G
#define MC3672_DEFAULT_RESOLUTION   MC3672_RESOLUTION_14BIT

#define MC3672_SCRATCH_VALUE    (0X7E)

/**\name Macro to SET and GET BITS of a register*/
#define MC3672_SET_BITS(reg_data, bitname, data) \
        ((reg_data & ~(bitname##_MSK)) | \
        ((data << bitname##_POS) & bitname##_MSK))

#define MC3672_GET_BITS(reg_data, bitname)  ((reg_data & (bitname##_MSK)) >> \
              (bitname##_POS))

#define MC3672_SET_BITS_POS_0(reg_data, bitname, data) \
        ((reg_data & ~(bitname##_MSK)) | \
        (data & bitname##_MSK))

#define MC3672_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))

/****************************************************************************
 * Private type
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/


/****************************************************************************
 * Private Functions
 ****************************************************************************/
/*!
 * @brief This internal API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t null_ptr_check(const struct mc3672_dev *dev)
{
  int8_t rslt;

  if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL))
    {
      /* Device structure pointer is not valid */
      rslt = MC3672_RETCODE_ERROR_NULL_POINTER;
    }
  else
    {
      /* Device structure is fine */
      rslt = MC3672_RETCODE_SUCCESS;
    }

  return rslt;
}
/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 */
int8_t mc3672_set_regs(uint8_t reg_addr, uint8_t *reg_data, uint8_t len, const struct mc3672_dev *dev)
{
  int8_t rslt;

  /* Check for null pointer in the device structure*/
  rslt = null_ptr_check(dev);
  /* Proceed if null check is fine */
  if ((rslt ==  MC3672_RETCODE_SUCCESS) && (reg_data != NULL) && (len != 0))
    {
      if (dev->intf != MC3672_I2C_INTF)
        {
          /* If interface selected is SPI */
          reg_addr = reg_addr | 0x40;
        }
      /* Read the data from the reg_addr */
      rslt = dev->write(dev, reg_addr, reg_data, len);
    }
  else
    {
      rslt = MC3672_RETCODE_ERROR_NULL_POINTER;
    }

  return rslt;
}

/*!
 * @brief This API reads the data from the given register address of the sensor.
 */
int8_t mc3672_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint8_t len, const struct mc3672_dev *dev)
{
  int8_t rslt;

  /* Check for null pointer in the device structure*/
  rslt = null_ptr_check(dev);
  /* Proceed if null check is fine */
  if ((rslt ==  MC3672_RETCODE_SUCCESS) && (reg_data != NULL))
    {
      if (dev->intf != MC3672_I2C_INTF)
        {
          /* If interface selected is SPI */
          reg_addr = reg_addr | 0xC0;
        }
      /* Read the data from the reg_addr */
      rslt = dev->read(dev, reg_addr, reg_data, len);
    }
  else
    {
      rslt = MC3672_RETCODE_ERROR_NULL_POINTER;
    }

  return rslt;
}

int8_t mc3672_get_reg8(uint8_t reg_addr, uint8_t *reg_data, const struct mc3672_dev *dev)
{
  return mc3672_get_regs(reg_addr, reg_data, 1, dev);
}

int8_t mc3672_set_reg8(uint8_t reg_addr, uint8_t reg_data, const struct mc3672_dev *dev)
{
  return mc3672_set_regs(reg_addr, &reg_data, 1, dev);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
int8_t mc3672_reset(struct mc3672_dev *dev)
{
  int8_t rslt;

  /* Check for null pointer in the device structure*/
  rslt = null_ptr_check(dev);
  /* Proceed if null check is fine */
  if (rslt ==  MC3672_RETCODE_SUCCESS)
    {
      rslt = mc3672_set_reg8(0x10, 0x01, dev);

      dev->delay_ms(dev, 10);

      rslt += mc3672_set_reg8(0x24, 0x40, dev);

      dev->delay_ms(dev, 10);

      rslt += mc3672_set_reg8(0x09, 0x00, dev);

      if (dev->intf == MC3672_I2C_INTF)
        {
          rslt += mc3672_set_reg8(0x0D, 0x40, dev);
        }
      else
        {
          rslt += mc3672_set_reg8(0x0D, 0x80, dev);
        }

      rslt += mc3672_set_reg8(0x0F, 0x42, dev);

      rslt += mc3672_set_reg8(0x20, 0x01, dev);

      rslt += mc3672_set_reg8(0x21, 0x80, dev);

      rslt += mc3672_set_reg8(0x28, 0x00, dev);

      rslt += mc3672_set_reg8(0x1a, 0x00, dev);
    }

  return rslt;
}


int8_t mc3672_init(struct mc3672_dev *dev )
{
  int8_t rslt;
  uint8_t chip_id;

  /* Check for null pointer in the device structure*/
  rslt = null_ptr_check(dev);
  /* Proceed if null check is fine */
  if (rslt ==  MC3672_RETCODE_SUCCESS)
    {
      /* Power up the sensor from suspend to sleep mode */
      rslt = mc3672_reset(dev);
      /* Start-up time delay of 1ms*/
      dev->delay_ms(dev, 10);
      if (rslt ==  MC3672_RETCODE_SUCCESS)
        {
          /*set scratch register value*/
          rslt = mc3672_set_reg8(MC3672_REG_SCRATCH, MC3672_SCRATCH_VALUE, dev);

          if (rslt ==  MC3672_RETCODE_SUCCESS)
            {
              /*read it back*/
              rslt = mc3672_get_reg8(MC3672_REG_SCRATCH, &chip_id, dev);
              if (rslt ==  MC3672_RETCODE_SUCCESS)
                {
                  /*chech retrived value*/
                  if (chip_id != MC3672_SCRATCH_VALUE)
                    {
                      rslt = MC3672_RETCODE_ERROR_IDENTIFICATION;
                    }
                  else
                    {
                      /*put the device into standby mode*/
                      rslt = mc3672_set_mode(dev, MC3672_MODE_STANDBY);
                    }
                }
            }
        }
    }

  return rslt;
}

int8_t mc3672_get_mode(struct mc3672_dev *dev, MC3672_mode_t *mode)
{
  int8_t rslt;
  uint8_t reg_value;
  MC3672_mode_t cur_mode;


  rslt = mc3672_get_reg8(MC3672_REG_MODE_C, &reg_value, dev);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  cur_mode = (MC3672_mode_t)MC3672_GET_BITS(reg_value, MC3672_MODE_C_MCTRL);

  *mode = cur_mode;

  return MC3672_RETCODE_SUCCESS;
}


int8_t mc3672_set_mode(struct mc3672_dev *dev, MC3672_mode_t mode)
{
  int8_t rslt;
  uint8_t reg_value;
  MC3672_mode_t cur_mode;

  rslt = mc3672_get_reg8(MC3672_REG_MODE_C, &reg_value, dev);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  cur_mode = (MC3672_mode_t)MC3672_GET_BITS(reg_value, MC3672_MODE_C_MCTRL);

  /*If we are working on the wanted mode now*/
  if (cur_mode == mode)
    {
      return MC3672_RETCODE_SUCCESS;
    }

  /*check current mode, if not standby mode, set it to this mode*/
  if (cur_mode != MC3672_MODE_STANDBY)
    {
      reg_value = MC3672_SET_BITS(reg_value, MC3672_MODE_C_MCTRL, MC3672_MODE_STANDBY);

      rslt = mc3672_set_reg8(MC3672_REG_MODE_C, reg_value, dev);

      if (rslt != MC3672_RETCODE_SUCCESS)
        {
          return rslt;
        }

      dev->delay_ms(dev, 10);
    }

  /*if the target mode is standby, we can stop*/
  if (mode == MC3672_MODE_STANDBY)
    {
      return MC3672_RETCODE_SUCCESS;
    }

  reg_value = MC3672_SET_BITS(reg_value, MC3672_MODE_C_MCTRL, mode);

  rslt = mc3672_set_reg8(MC3672_REG_MODE_C, reg_value, dev);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  dev->delay_ms(dev, 10);

  return MC3672_RETCODE_SUCCESS;
}

int8_t mc3672_cfg_cwake(struct mc3672_dev *dev, struct mc3672_cwake_settings *setting)
{
  int8_t rslt;
  uint8_t reg_value;
  uint8_t high_limit;
  uint8_t low_limit;
  MC3672_mode_t pre_mode;

  /*save the previous mode*/
  rslt = mc3672_get_mode(dev, &pre_mode);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  /*we can only write to registers in standby mode*/
  rslt = mc3672_set_mode(dev, MC3672_MODE_STANDBY);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  /*set range and resolution*/
  rslt = mc3672_get_reg8(MC3672_REG_RANGE_C, &reg_value, dev);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  reg_value = MC3672_SET_BITS(reg_value, MC3672_RANGE_C_RANGE, setting->range);
  reg_value = MC3672_SET_BITS(reg_value, MC3672_RANGE_C_RES, setting->resolution);

  rslt = mc3672_set_reg8(MC3672_REG_RANGE_C, reg_value, dev);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  /*set cwake power mode*/
  rslt = mc3672_get_reg8(MC3672_REG_PMCR, &reg_value, dev);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  reg_value = MC3672_SET_BITS(reg_value, MC3672_PMCR_CSPM, setting->wake_pm);

  rslt = mc3672_set_reg8(MC3672_REG_PMCR, reg_value, dev);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  /*set cwake sample rate*/
  switch (setting->wake_pm)
    {
      case MC3672_POWER_MODE_LOW:
        high_limit = MC3672_CWAKE_SR_LPM_END;
        low_limit = MC3672_CWAKE_SR_LPM_START;
        break;
      case MC3672_POWER_MODE_ULTRA_LOW:
        high_limit = MC3672_CWAKE_SR_ULPM_END;
        low_limit = MC3672_CWAKE_SR_ULPM_START;
        break;
      case MC3672_POWER_MODE_PRECISION:
        high_limit = MC3672_CWAKE_SR_HPM_END;
        low_limit = MC3672_CWAKE_SR_HPM_START;
        break;
      default:
        high_limit = 0;
        low_limit =  0;
        break;
    }

  /*check sample rate limitation*/
  if ((setting->wake_sr <=  low_limit) || (setting->wake_sr >=  high_limit))
    {
      return MC3672_RETCODE_ERROR_WRONG_ARGUMENT;
    }

  rslt = mc3672_get_reg8(MC3672_REG_RATE_1, &reg_value, dev);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  reg_value = MC3672_SET_BITS(reg_value, MC3672_RATE_1_RR, setting->wake_sr);

  rslt = mc3672_set_reg8(MC3672_REG_RATE_1, reg_value, dev);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  /*back to previous mode*/
  rslt = mc3672_set_mode(dev, pre_mode);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  return MC3672_RETCODE_SUCCESS;
}

int8_t mc3672_cfg_fifo(struct mc3672_dev *dev, struct mc3672_fifo_settings *setting)
{
  int8_t rslt;
  uint8_t reg_value;
  MC3672_mode_t pre_mode;


  /*save the previous mode*/
  rslt = mc3672_get_mode(dev, &pre_mode);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  /*we can only write to registers in standby mode*/
  rslt = mc3672_set_mode(dev, MC3672_MODE_STANDBY);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  /*reset the fifo at first*/
  reg_value = 0;
  reg_value = MC3672_SET_BITS(reg_value, MC3672_FIFO_C_RESET, MC3672_FEATURE_ENABLE);

  rslt = mc3672_set_reg8(MC3672_REG_FIFO_C, reg_value, dev);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  /*set stream feature and burst feature*/
  rslt = mc3672_get_reg8(MC3672_REG_FREG_2, &reg_value, dev);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  /*enable fifo status and fifo burst features, we need these two features to simplify fifo read operation*/
  reg_value = MC3672_SET_BITS(reg_value, MC3672_FREG_2_FSTAT, MC3672_FEATURE_ENABLE);
  reg_value = MC3672_SET_BITS(reg_value, MC3672_FREG_2_BURST, MC3672_FEATURE_ENABLE);

  reg_value = MC3672_SET_BITS(reg_value, MC3672_FREG_2_STREAM, setting->fifo_stream_en);

  rslt = mc3672_set_reg8(MC3672_REG_FREG_2, reg_value, dev);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  /*set fifo_mode and fifo_threshold, this will enable fifo, as well*/
  if (setting->fifo_threshold >= MC3672_FIFO_DEPTH)
    {
      return MC3672_RETCODE_ERROR_WRONG_ARGUMENT;
    }

  reg_value = 0;
  reg_value = MC3672_SET_BITS(reg_value, MC3672_FIFO_C_EN, MC3672_FEATURE_ENABLE);
  reg_value = MC3672_SET_BITS(reg_value, MC3672_FIFO_C_MODE, setting->fifo_mode);
  reg_value = MC3672_SET_BITS(reg_value, MC3672_FIFO_C_TH, setting->fifo_threshold);

  rslt = mc3672_set_reg8(MC3672_REG_FIFO_C, reg_value, dev);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  /*back to previous mode*/
  rslt = mc3672_set_mode(dev, pre_mode);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  return MC3672_RETCODE_SUCCESS;
}

int8_t mc3672_enable_fifo(struct mc3672_dev *dev, bool enable)
{
  int8_t rslt;
  uint8_t reg_value;
  MC3672_mode_t pre_mode;
  uint8_t fifo_en;


  /*save the previous mode*/
  rslt = mc3672_get_mode(dev, &pre_mode);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }


  /*we can only write to registers in standby mode*/
  rslt = mc3672_set_mode(dev, MC3672_MODE_STANDBY);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  /*set fifo enable*/
  rslt = mc3672_get_reg8(MC3672_REG_FIFO_C, &reg_value, dev);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  fifo_en = enable ? MC3672_FEATURE_ENABLE : MC3672_FEATURE_DISABLE;
  reg_value = MC3672_SET_BITS(reg_value, MC3672_FIFO_C_EN, fifo_en);

  rslt = mc3672_set_reg8(MC3672_REG_FIFO_C, reg_value, dev);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  /*back to previous mode*/
  rslt = mc3672_set_mode(dev, pre_mode);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  return MC3672_RETCODE_SUCCESS;
}


int8_t mc3672_reset_fifo(struct mc3672_dev *dev)
{
  int8_t rslt;
  uint8_t reg_value;
  MC3672_mode_t pre_mode;


  /*save the previous mode*/
  rslt = mc3672_get_mode(dev, &pre_mode);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }


  /*we can only write to registers in standby mode*/
  rslt = mc3672_set_mode(dev, MC3672_MODE_STANDBY);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  /*set fifo reset*/
  rslt = mc3672_get_reg8(MC3672_REG_FIFO_C, &reg_value, dev);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  reg_value = MC3672_SET_BITS(reg_value, MC3672_FIFO_C_RESET, MC3672_FEATURE_ENABLE);

  rslt = mc3672_set_reg8(MC3672_REG_FIFO_C, reg_value, dev);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  /*back to previous mode*/
  rslt = mc3672_set_mode(dev, pre_mode);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  return MC3672_RETCODE_SUCCESS;

}

int8_t mc3672_cfg_int(struct mc3672_dev *dev, struct mc3672_int_settings *setting)
{
  int8_t rslt;
  uint8_t reg_value;
  MC3672_mode_t pre_mode;
  uint8_t int_en;

  /*save the previous mode*/
  rslt = mc3672_get_mode(dev, &pre_mode);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  /*we can only write to registers in standby mode*/
  rslt = mc3672_set_mode(dev, MC3672_MODE_STANDBY);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  /*set bits*/
  reg_value = 0;

  int_en = setting->acq_int_en ? MC3672_FEATURE_ENABLE : MC3672_FEATURE_DISABLE;
  reg_value = MC3672_SET_BITS(reg_value, MC3672_INTR_C_ACQ, int_en);

  int_en = setting->fifo_empty_int_en ? MC3672_FEATURE_ENABLE : MC3672_FEATURE_DISABLE;
  reg_value = MC3672_SET_BITS(reg_value, MC3672_INTR_C_EMPTY, int_en);

  int_en = setting->fifo_full_int_en ? MC3672_FEATURE_ENABLE : MC3672_FEATURE_DISABLE;
  reg_value = MC3672_SET_BITS(reg_value, MC3672_INTR_C_FULL, int_en);

  int_en = setting->fifo_thresh_int_en ? MC3672_FEATURE_ENABLE : MC3672_FEATURE_DISABLE;
  reg_value = MC3672_SET_BITS(reg_value, MC3672_INTR_C_THRESH, int_en);

  int_en = setting->swake_int_en ? MC3672_FEATURE_ENABLE : MC3672_FEATURE_DISABLE;
  reg_value = MC3672_SET_BITS(reg_value, MC3672_INTR_C_SWAKE, int_en);

  int_en = setting->wake_int_en ? MC3672_FEATURE_ENABLE : MC3672_FEATURE_DISABLE;
  reg_value = MC3672_SET_BITS(reg_value, MC3672_INTR_C_WAKE, int_en);

  reg_value = MC3672_SET_BITS(reg_value, MC3672_INTR_C_IAH, setting->INTN_PIN_IAH);
  reg_value = MC3672_SET_BITS(reg_value, MC3672_INTR_C_IPP, setting->INTN_PIN_IPP);

  rslt = mc3672_set_reg8(MC3672_REG_INTR_C, reg_value, dev);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  /*back to previous mode*/
  rslt = mc3672_set_mode(dev, pre_mode);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  return MC3672_RETCODE_SUCCESS;
}

int8_t mc3672_get_int_status(struct mc3672_dev *dev, uint8_t *events)
{
  int8_t rslt;
  uint8_t reg_value;

  rslt = mc3672_get_reg8(MC3672_REG_STATUS_2, &reg_value, dev);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  *events = reg_value;

  /*if we using SPI interface, we need to write STATUS_2 register to clean interrupt flags*/
  if (dev->intf == MC3672_SPI_INTF)
    {
      rslt = mc3672_set_reg8(MC3672_REG_STATUS_2, 0, dev);

      if (rslt != MC3672_RETCODE_SUCCESS)
        {
          return rslt;
        }
    }

  return MC3672_RETCODE_SUCCESS;
}

int8_t mc3672_read_fifo(struct mc3672_dev *dev,
                        struct mc3672_acc_t *sample,
                        uint8_t target_sample_count,
                        uint8_t *gotted_sample_count)
{
  int8_t rslt;
  struct mc3672_acc_reg_t raw[MC3672_FIFO_DEPTH];
  int i;

  /*check input parameter*/
  if (target_sample_count > MC3672_FIFO_DEPTH)
    {
      return MC3672_RETCODE_ERROR_WRONG_ARGUMENT;
    }

  /*fetch data*/
  rslt = mc3672_get_regs(MC3672_REG_XOUT_LSB,
                         (uint8_t *)raw,
                         target_sample_count * sizeof(struct mc3672_acc_reg_t),
                         dev);

  if (rslt != MC3672_RETCODE_SUCCESS)
    {
      return rslt;
    }

  for (i = 0; i < target_sample_count; i++)
    {

      /*As we enabled fifo status bit in feature 2 register, we can got the fifo
        empty indication by analyzing bits appended at the ZOUT_MSB register*/
      if (!MC3672_GET_BITS(raw[i].ZOUT_MSB, MC3672_ZOUT_MSB_TH) \
          && !MC3672_GET_BITS(raw[i].ZOUT_MSB, MC3672_ZOUT_MSB_FULL)
          && MC3672_GET_BITS(raw[i].ZOUT_MSB, MC3672_ZOUT_MSB_EMPTY))
        {
          /*chech fifo empty bits in higher 4 bits of Z axis value, if it's set, discard following sample*/
          break;
        }
      else
        {
          sample[i].XAxis = (int16_t)((((uint16_t)raw[i].XOUT_MSB) << 8) | raw[i].XOUT_LSB);
          sample[i].YAxis = (int16_t)((((uint16_t)raw[i].YOUT_MSB) << 8) | raw[i].YOUT_LSB);

          /*repair z axis value, discard fifo status indication bits*/
          if (MC3672_GET_BITS(raw[i].ZOUT_MSB, MC3672_ZOUT_MSB_SIG))
            {
              raw[i].ZOUT_MSB = MC3672_SET_BITS(raw[i].ZOUT_MSB, MC3672_ZOUT_MSB_FSTATUS, MC3672_ZOUT_MSB_SIG_EXPAND_NEG);
            }
          else
            {
              raw[i].ZOUT_MSB = MC3672_SET_BITS(raw[i].ZOUT_MSB, MC3672_ZOUT_MSB_FSTATUS, MC3672_ZOUT_MSB_SIG_EXPAND_POS);
            }

          sample[i].ZAxis = (int16_t)((((uint16_t)raw[i].ZOUT_MSB) << 8) | raw[i].ZOUT_LSB);

        }

    }

  *gotted_sample_count = i;

  return MC3672_RETCODE_SUCCESS;
}



#endif /*defined CONFIG_MC3672*/
