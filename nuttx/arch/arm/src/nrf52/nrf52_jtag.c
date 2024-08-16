/****************************************************************************
 * arch/arm/src/nrf52/nrf52_jtag.c
 *
 *   Copyright (C) 2012, 2014-2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Copyright (C) 2011 Li Zhuoyi. All rights reserved.
 *   Author: Li Zhuoyi <lzyy.cn@gmail.com> (Original author)
 *
 * Derived from arch/arm/src/nrf52/nrf52_i2c.c
 *
 *   Author: David Hewson
 *
 *   Copyright (C) 2010-2011 Gregory Nutt. All rights reserved.
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

/******************************************************************************
 * INCLUDE FILES
 *****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_NRF52_JTAG
#include <arch/board/board.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/types.h>
#include <nuttx/arch.h>
#include "nrf52_gpio.h"
#include "nrf52_jtag.h"

#define JTAG_CLK_DELAY 40

static const struct tms_sequences short_tms_seqs[7][7] =   /* [from_state_ndx][to_state_ndx] */
{
  /* to state: */
  /*  RESET        IDLE            DRSHIFT         DRPAUSE         IRSHIFT         IRPAUSE         CAPTUREDR*/              /* from state: */
  {B8(1111111, 7), B8(0000000, 7), B8(0010111, 7), B8(0001010, 7), B8(0011011, 7), B8(0010110, 7), B8(010111, 6)     },     /* RESET */
  {B8(1111111, 7), B8(0000000, 7), B8(001, 3),     B8(0101, 4),    B8(0011, 4),    B8(01011, 5),   B8(01, 2)         },     /* IDLE */
  {B8(1111111, 7), B8(011, 3),     B8(00111, 5),   B8(01, 2),      B8(001111, 6),  B8(0101111, 7), B8(0111, 4),      },     /* DRSHIFT */
  {B8(1111111, 7), B8(011, 3),     B8(01, 2),      B8(0, 1),       B8(001111, 6),  B8(0101111, 7), B8(0111, 4),      },     /* DRPAUSE */
  {B8(1111111, 7), B8(011, 3),     B8(00111, 5),   B8(010111, 6),  B8(001111, 6),  B8(01, 2),      B8(0111, 4),      },     /* IRSHIFT */
  {B8(1111111, 7), B8(011, 3),     B8(00111, 5),   B8(010111, 6),  B8(01, 2),      B8(0, 1),       B8(0111, 4),      },     /* IRPAUSE */
  {B8(1111111, 7), B8(01, 2),      B8(0011, 4),    B8(01011, 5),   B8(00111, 5),   B8(0, 1),       B8(011, 3),       },     /* IR1EXIT */
};



/******************************************************************************
 * @brief jtag_pins_init. We need 4 JTAG pins. TDI,TDO,TMS and TCK
 *                        Be caureful and make sure that the above
 *                        pins are not being used for any other
 *                        purpose like I2C, SPI, UART or GPIOs
 * @param None
 *
 *****************************************************************************/

void jtag_pins_init(void)
{
  nrf_gpio_cfg_output(BOARD_JTAG_TCK_PIN);
  nrf_gpio_cfg_output(BOARD_JTAG_TMS_PIN);
  nrf_gpio_cfg_output(BOARD_JTAG_TDI_PIN);
  nrf_gpio_cfg_input(BOARD_JTAG_TDO_PIN, NRF_GPIO_PIN_NOPULL);
}


/******************************************************************************
 * @brief send_jtag_instruction. sends an 4 bit JTAG instruction by bit
 *                               banging the TMS, TCK and TDI pins.
 * @param jtag_instruction - 4 bit instruction to send to the FAST
 *        tms_sequence     - TMS sequence to use. since the JTAG needs to be
 *                           correct to the exact clock cycle
 *
 *****************************************************************************/
void send_jtag_instruction(uint32_t jtag_instruction, uint32_t tms_sequence)
{
  int count = 0;
  uint32_t jtag_local_instruction = jtag_instruction;

  for (count = 0; count < JTAG_INSTRUCTION_SIZE; count++)
    {
      gpio_pin_write(BOARD_JTAG_TCK_PIN, 0);
      (jtag_local_instruction & 0x01) ? (gpio_pin_write(BOARD_JTAG_TDI_PIN, 1)) : \
      (gpio_pin_write(BOARD_JTAG_TDI_PIN, 0));
      (tms_sequence & 0x01) ? (gpio_pin_write(BOARD_JTAG_TMS_PIN, 1)) : \
      (gpio_pin_write(BOARD_JTAG_TMS_PIN, 0));
      up_udelay(JTAG_CLK_DELAY);
      gpio_pin_write(BOARD_JTAG_TCK_PIN, 1);
      jtag_local_instruction = jtag_local_instruction >> 1;
      tms_sequence = tms_sequence >> 1;
      up_udelay(JTAG_CLK_DELAY);
    }
}

void get_jtag_data(uint32_t *data)
{
  int count = 0;
  unsigned int local_data = 0;
  for (count = 0; count < JTAG_DATA_SIZE; count++)
    {
      gpio_pin_write(BOARD_JTAG_TCK_PIN, 0);
      local_data |= (nrf_gpio_pin_read(BOARD_JTAG_TDO_PIN) << count);
      up_udelay(JTAG_CLK_DELAY);
      gpio_pin_write(BOARD_JTAG_TCK_PIN, 1);
      up_udelay(JTAG_CLK_DELAY);
    }
  *data = local_data;
}


/******************************************************************************
 * @brief send_jtag_instruction. sends an 4 bit JTAG instruction by bit banging
 *                               the TMS, TCK and TDI pins.
 * @param jtag_instruction - 4 bit instruction to send to the FAST
 *        tms_sequence     - TMS sequence to use. since the JTAG needs to be
 *                           correct to exact clock cycle
 *
 *****************************************************************************/
void send_jtag_data(uint32_t data, uint32_t tms_sequence)
{
  int count = 0;
  for (count = 0; count < JTAG_DATA_SIZE; count++)
    {
      gpio_pin_write(BOARD_JTAG_TCK_PIN, 0);
      (data & 0x01) ? (gpio_pin_write(BOARD_JTAG_TDI_PIN, 1)) : \
      (gpio_pin_write(BOARD_JTAG_TDI_PIN, 0));
      (tms_sequence & 0x01) ? (gpio_pin_write(BOARD_JTAG_TMS_PIN, 1)) : \
      (gpio_pin_write(BOARD_JTAG_TMS_PIN, 0));
      data >>= 1;
      tms_sequence = tms_sequence >> 1;
      up_udelay(JTAG_CLK_DELAY);
      gpio_pin_write(BOARD_JTAG_TCK_PIN, 1);
      up_udelay(JTAG_CLK_DELAY);
    }
}


/******************************************************************************
 * @brief switch_tap_state from current_state to next_state. The TMS sequence
 *                         to be used is defined in the global variable
 *                         short_tms_seqs. We then bit bang manually through
 *                         the GPIO PINS JTAG_TMS and JTAG_TCK
 * @param current_state - current state of the JTAG state machine
 *        next_state    - state to which the  JTAG state machine should go to
 *
 *****************************************************************************/
void switch_tap_state(uint8_t current_state, uint8_t next_state)
{
  int count = 0;
  uint8_t tms_sequence = short_tms_seqs[current_state][next_state].bits;
  for (count = 0; count < short_tms_seqs[current_state][next_state].bit_count; count++)
    {
      gpio_pin_write(BOARD_JTAG_TCK_PIN, 0);
      (tms_sequence & 0x01) ? (gpio_pin_write(BOARD_JTAG_TMS_PIN, 1)) : \
      (gpio_pin_write(BOARD_JTAG_TMS_PIN, 0));
      up_udelay(JTAG_CLK_DELAY);
      gpio_pin_write(BOARD_JTAG_TCK_PIN, 1);
      tms_sequence = tms_sequence >> 1;
      up_udelay(JTAG_CLK_DELAY);
    }
}

/******************************************************************************
 * @brief jtag_32b_read writes 8 bits to the instrution register followed by
 *                       32 bits  read from the data register. This is  simple
 *                       jtag read command
 * @param instruction  - 8 bit JTAG instruction code
 *        data         - pointer that contains the data
 *
 *****************************************************************************/
int16_t jtag_32b_read(uint8_t jtag_instruction, uint32_t *data)
{

  switch_tap_state(TAP_RESET, TAP_RESET);
  switch_tap_state(TAP_RESET, TAP_IRSHIFT);
  send_jtag_instruction(jtag_instruction, JTAG_INSTRUCTION_EXIT_IR_TMS_SEQUENCE);
  switch_tap_state(TAP_IREXIT1, TAP_DRSHIFT);
  get_jtag_data(data);
  switch_tap_state(TAP_DRSHIFT, TAP_RESET);

  return OK;
}

/******************************************************************************
 *
 * @brief jtag_32b_write writes 8 bits to the instrution register followed by
 *                        32 bits to the data register. This is  simple jtag
 *                        command with its data.
 * @param instruction   - 8 bit JTAG instruction code
 *        data          - 32 bit data to the JTAG instruction
 *
 *****************************************************************************/
int16_t jtag_32b_write(uint8_t jtag_instruction, uint32_t data)
{

  switch_tap_state(TAP_RESET, TAP_RESET);
  switch_tap_state(TAP_RESET, TAP_IRSHIFT);
  send_jtag_instruction(jtag_instruction, JTAG_INSTRUCTION_EXIT_IR_TMS_SEQUENCE);
  switch_tap_state(TAP_IREXIT1, TAP_DRSHIFT);
  send_jtag_data(data, JTAG_DATA_EXIT_DR_TMS_SEQUENCE);
  switch_tap_state(TAP_DRSHIFT, TAP_RESET);

  return OK;
}
#endif /*CONFIG_NRF52_JTAG */
