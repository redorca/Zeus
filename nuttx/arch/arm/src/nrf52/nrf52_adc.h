/************************************************************************************
 * arch/arm/src/nrf52/nrf52_adc.h
 *
 *   Copyright (C) 2009, 2011, 2017 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2017   zGlue Inc. All Rights Reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Levin Li  <zhiqiang@zglue.com>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_NRF52_NRF52_ADC_H
#define __ARCH_ARM_SRC_NRF52_NRF52_ADC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#include <nuttx/analog/adc.h>


/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/


/************************************************************************************
 * Public Types
 ************************************************************************************/
typedef enum adc_gain_s
{
  ADC_GAIN_1_6 = 0,
  ADC_GAIN_1_5,
  ADC_GAIN_1_4,
  ADC_GAIN_1_3,
  ADC_GAIN_1_2,
  ADC_GAIN_1_1,
  ADC_GAIN_1X,
  ADC_GAIN_2X,
  ADC_GAIN_3X,
  ADC_GAIN_4X,
} adc_gain_t;

typedef enum adc_reference_s
{
  ADC_REFERENCE_INTERNAL = 0,
  ADC_REFERENCE_VDD1_4
} adc_reference_t;

typedef enum adc_resistor_s
{
  ADC_RESISTOR_BYPASS = 0,
  ADC_RESISTER_PULL_DOWN,
  ADC_RESISTER_PULL_UP,
  ADC_RESISTER_VDD1_2,
} adc_resistor_t;

typedef enum adc_time_s
{
  /* adc convert time for one sample */
  ADC_TIME_3US = 0,
  ADC_TIME_5US,
  ADC_TIME_10US,
  ADC_TIME_15US,
  ADC_TIME_20US,
} adc_time_t;

typedef enum adc_input_pin_s
{
  /* the input pin related with analog input actual input pin_0...7 */
  ADC_INPUT_PIN_DISCONNECT = 0,
  ADC_INPUT_PIN_1 = 1,
  ADC_INPUT_PIN_2 = 2,
  ADC_INPUT_PIN_3 = 3,
  ADC_INPUT_PIN_4 = 4,
  ADC_INPUT_PIN_5 = 5,
  ADC_INPUT_PIN_6 = 6,
  ADC_INPUT_PIN_7 = 7,
  ADC_INPUT_PIN_8 = 8,
  ADC_INPUT_PIN_VDD = 9,
} adc_input_pin_t;

typedef enum adc_channel_mode_s
{
  ADC_CHANNEL_MODE_SINGLE_END = 0,
  ADC_CHANNEL_MODE_DIFFERENTIAL = 1
} adc_channel_mode_t;

typedef struct adc_channel_config_s
{
  uint8_t         channel;  /* channel number : 0---7 */

  adc_channel_mode_t mode;

  /* input range :  range = ref / gain : etc ref = VDD1_4 , gain = GAIN_1_6
   * example :  range = VDD1_4 / GAIN_1_6 = (3.3/4) / (1/6) = 4.95 V
   */
  adc_reference_t ref;
  adc_gain_t      gain;
  adc_time_t      time;
  adc_resistor_t  resistor_p;
  adc_resistor_t  resistor_n;
  adc_input_pin_t pin_p;  /*adc input pin: 1 --8 for analog input, 0 is disconnect, 9 is VDD */
  adc_input_pin_t pin_n;  /*adc input pin: 1 --8 for analog input, 0 is disconnect, 9 is VDD */
} adc_channel_config_t;

typedef enum adc_resolution_s
{
  ADC_RESOLUTION_8BIT = 0,
  ADC_RESOLUTION_10BIT,
  ADC_RESOLUTION_12BIT,
  ADC_RESOLUTION_14BIT,
  ADC_RESOLUTION_16BIT
} adc_resolution_t;

typedef enum adc_oversample_s
{
  ADC_OVERSAMPLE_BYPASS  = 0,
  ADC_OVERSAMPLE_2X,
  ADC_OVERSAMPLE_4X,
  ADC_OVERSAMPLE_8X,
  ADC_OVERSAMPLE_16X,
  ADC_OVERSAMPLE_32X,
  ADC_OVERSAMPLE_64X,
  ADC_OVERSAMPLE_128X,
  ADC_OVERSAMPLE_256X,
} adc_oversample_t;

typedef enum adc_mode_s
{
  ADC_MODE_ONE_SHOT = 0,
  ADC_MODE_CONTINUOUS = 1,
} adc_mode_t;

typedef struct adc_config_s
{
  adc_resolution_t resolution;
  adc_oversample_t oversample;
  adc_mode_t      mode;       /* mode should be ADC global attribute */
} adc_config_t;


/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: nrf52_adcinitialize
 *
 * Description:
 *   Initialize the ADC.
 *
 * Input Parameters:
 *   config      - ADC config
 *   ch_config  - The list of channels
 *   nchannels - Number of channels
 *
 * Returned Value:
 *   Success for OK
 *
 ****************************************************************************/

int nrf52_adc_initialize(adc_config_t *config,
                         adc_channel_config_t *ch_config, int nchannels);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_NRF52_NRF52_ADC_H */

