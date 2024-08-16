/****************************************************************************
 * arch/arm/src/nrf52/nrf52_jtag.h
 *
 *   Copyright (C) 2010, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_NRF52_JTAG_H
#define __ARCH_ARM_SRC_NRF52_JTAG_H

/******************************************************************************
 * INCLUDE FILES
 *****************************************************************************/


/******************************************************************************
 * DEFINES
 *****************************************************************************/

#define JTAG_INSTRUCTION_SIZE 4
#define JTAG_DATA_SIZE                                 32

#define JTAG_INSTRUCTION_EXIT_IR_TMS_SEQUENCE 0x08
#define JTAG_DATA_EXIT_DR_TMS_SEQUENCE 0x80000000

/* JTAG COMMANDS  */
#define JTAG_IDCODE_INSTRUCTION         0x02

#define HEX__(n) 0x##n##LU

#define B8__(x) \
       ((((x) & 0x0000000FLU) ? (1 << 0) : 0) \
       +(((x) & 0x000000F0LU) ? (1 << 1) : 0) \
       +(((x) & 0x00000F00LU) ? (1 << 2) : 0) \
       +(((x) & 0x0000F000LU) ? (1 << 3) : 0) \
       +(((x) & 0x000F0000LU) ? (1 << 4) : 0) \
       +(((x) & 0x00F00000LU) ? (1 << 5) : 0) \
       +(((x) & 0x0F000000LU) ? (1 << 6) : 0) \
       +(((x) & 0xF0000000LU) ? (1 << 7) : 0))

#define B8(bits, count) {((uint8_t)B8__(HEX__(bits))), (count)}

/******************************************************************************
 * TYPE DEFINITIONS
 *****************************************************************************/

typedef enum tap_state
{
  TAP_INVALID     = -1,
  TAP_RESET       = 0,
  TAP_IDLE        = 1,
  TAP_DRSHIFT     = 2,
  TAP_DRPAUSE     = 3,
  TAP_IRSHIFT     = 4,
  TAP_IRPAUSE     = 5,
  TAP_IREXIT1     = 6,
} tap_state_t;

struct tms_sequences
{
  uint8_t bits;
  uint8_t bit_count;
};

/******************************************************************************
 * FUNCTION DECLARATIONS
 *****************************************************************************/

void jtag_pins_init(void);
int16_t jtag_32b_read(uint8_t jtag_instruction, uint32_t *data);
int16_t jtag_32b_write(uint8_t jtag_instruction, uint32_t data);

#endif /* __ARCH_ARM_SRC_NRF52_JTAG_H */
