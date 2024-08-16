/****************************************************************************
 * This file contains information definitons of the FAST API
 *
 *   Copyright (C) 2017 Zglue  Inc. All rights reserved.
 *   Author: Arjun Hary
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
 *****************************************************************************/
#ifndef FAST_DEBUG_API_H
#define FAST_DEBUG_API_H

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* FAST tile program mode configuration */
typedef enum
{
  FAST_PERIPHERAL_LEFT_VRAIL = 1,
  FAST_PERIPHERAL_RIGHT_VRAIL,
  FAST_PERIPHERAL_LEFT_TILEIO,
  FAST_PERIPHERAL_RIGHT_TILEIO,
  FAST_PERIPHERAL_LEFT_RDAC,
  FAST_PERIPHERAL_RIGHT_RDAC,
  FAST_PERIPHERAL_CDAC,
  FAST_PERIPHERAL_MAX,
} fast_peripheral_id_t;

/* Register operations API */
fast_status_e fast_reg_write(uint32_t reg_addr, uint16_t reg_data);
fast_status_e fast_reg_read(uint32_t reg_addr, uint16_t *reg_data);

/* OTP */
fast_status_e fast_read_fast_otp(uint32_t start_addr, uint16_t *data_ptr, uint32_t number_of_bytes);
fast_status_e fast_write_customer_otp(uint32_t start_addr, uint16_t *data_ptr, uint32_t number_of_bytes);
fast_status_e fast_write_fast_otp(uint32_t start_addr, uint16_t *data_ptr, uint32_t number_of_bytes);

/* Change I2C Address*/
fast_status_e fast_i2c_config(bool enable, uint8_t address);

/* zeus2 LPM FSM functions */
fast_status_e fast_pm_trigger_lpm_fsm(void);
fast_status_e fast_pm_lpm_fsm_control(bool enable);

/* Tile and router read and write */
fast_status_e fast_tile_write(uint8_t row_addr, uint8_t col_addr, uint32_t tile_data);
fast_status_e fast_tile_read(uint8_t row_addr, uint8_t col_addr, uint32_t *tile_data);
fast_status_e fast_peripheral_write(fast_peripheral_id_t peripheral_id, uint8_t index, uint32_t data);
fast_status_e fast_peripheral_read(fast_peripheral_id_t peripheral_id, uint8_t index, uint32_t *data);
fast_status_e fast_peripheral_clear(fast_peripheral_id_t peripheral_id);
fast_status_e fast_connect_peripheral(uint8_t config_file_id, uint16_t peri_id);
fast_status_e fast_disconnect_peripheral(uint8_t config_file_id, uint16_t peri_id);

/* scan functions */
fast_status_e fast_scan_peripheral(fast_peripheral_id_t peripheral_id, uint8_t start_index, uint8_t count, uint32_t *data_ptr);
fast_status_e fast_scantile(uint8_t start_row, uint8_t start_column, uint8_t end_row, uint8_t end_column, uint32_t *data_ptr);
fast_status_e fast_scan_regs(void);

/* clears the whole tilegrid */
fast_status_e fast_clear_tiles(void);

/* FAST program registers/peripherals according to an input file */
fast_status_e fast_program(char *filename, uint8_t arg);

/* FAST zcad program registers according to zcad input file */
fast_status_e fast_zcad_program(char *filename);

/* FAST run realignment according to an input file */
fast_status_e fast_realign_grid(char *input_filename, char *output_filename);

/* Support the zeus2 mcu poweroff */
fast_status_e fast_mcu_poweroff(void);


/* FAST configure group  mode */
fast_status_e fast_config_groupmode(uint16_t group_mode_enable);
#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /*FAST_DEBUG_API_H*/
