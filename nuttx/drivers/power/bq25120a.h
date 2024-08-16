/****************************************************************************
 * drivers/power/bq2425x.h
 * Lower half driver for BQ2425x battery charger
 *
 *   Copyright (C) 2015 Alan Carvalho de Assis. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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

#ifndef __DRIVERS_POWER_BQ25120A_H
#define __DRIVERS_POWER_BQ25120A_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Auxiliary Definitions */

#define BQ2512X_VOLT_MIN  3600
#define BQ2512X_VOLT_MAX  4650

#define BQ2512X_CURR_MIN  5
#define BQ2512X_CURR_RANGE1  35
#define BQ2512X_CURR_RANGE2  40
#define BQ2512X_CURR_MAX  300

/* BQ2425x Register Definitions ********************************************/

#define BQ2512X_STATUS_REG     0x00
#define BQ2512X_FAULT_REG      0x01
#define BQ2512X_TS_REG         0x02
#define BQ2512X_FAST_CHG_REG   0x03
#define BQ2512X_TERM_REG       0x04
#define BQ2512X_VBREG          0x05
#define BQ2512X_VOUT_REG       0x06
#define BQ2512X_LDO_CTRL_REG   0x07
#define BD2512X_BUTTON_CTRL_REG 0X08
#define BQ2512X_ILIM_UVLO_REG  0x09
#define BQ2512X_VBMON_REG      0x0A

/* REG 0 */
#define BQ2512X_STATUS_SYS_EN          (1 << 0) /* RO SW enabled/disabled */
#define BQ2512X_STATUS_CD_STAT         (1 << 1) /* RO CD low/high */
#define BQ2512X_STATUS_VINDPM_STAT     (1 << 2) /* RO VIN_DPM active/not active */
#define BQ2512X_STATUS_TIMER           (1 << 3) /* RO safety timer fault */
#define BQ2512X_STATUS_RESET_FAULT     (1 << 4) /* RO reset fault */
#define BQ2512X_STATUS_SHIPMODE_EN     (1 << 5) /* RO ship mode enabled/disabled */
#define BQ2512X_STATUS_SHIFT           6        /* Battery charger status */
#define BQ2512X_STATUS_MASK            (3 << BQ2512X_STATUS_SHIFT)
#  define BQ2512X_STATUS_READY         (0 << BQ2512X_STATUS_SHIFT)
#  define BQ2512X_STATUS_CHG_PROGRESS  (1 << BQ2512X_STATUS_SHIFT)
#  define BQ2512X_STATUS_CHG_DONE      (2 << BQ2512X_STATUS_SHIFT)
#  define BQ2512X_STATUS_FAULT         (3 << BQ2512X_STATUS_SHIFT)

/* REG 1 */
#define BQ2512X_FAULT_BAT_OCP_M          (1 << 0) /* R/W Battery OCP Fault mask */
#define BQ2512X_FAULT_BAT_UVLO_M         (1 << 1) /* R/W Battery UVLO Fault mask */
#define BQ2512X_FAULT_VIN_UV_M           (1 << 2) /* R/W Vin under voltage fault mask */
#define BQ2512X_FAULT_VIN_OV_M           (1 << 3) /* R/W Vin over voltage fault mask */
#define BQ2512X_FAULT_BAT_OCP            (1 << 4) /* RO Battery OCP Fault */
#define BQ2512X_FAULT_BAT_UVLO           (1 << 5) /* RO Battery UVLO Fault */
#define BQ2512X_FAULT_VIN_UV             (1 << 6) /* RO Vin under voltage fault */
#define BQ2512X_FAULT_VIN_OV             (1 << 7) /* RO Vin over voltage fault */

/* REG 2 */
#define BQ2512X_TS_TIMER_M               (1 << 0) /* R/W mask timer fault int */
#define BQ2512X_TS_RESET_M               (1 << 1) /* R/W mask reset int */
#define BQ2512X_TS_WAKE_M                (1 << 2) /* R/W mask int from wake */
#define BQ2512X_TS_EN_INT                (1 << 3) /* R/W enb/dis int function */
#define BQ2512X_TS_FAULT_OPEN                   (1 << 4) /* reserved */

#define BQ2512X_TS_FAULT_SHIFT            5       /* TS fault status */
#define BQ2512X_TS_FAULT_MASK            (3 << BQ2512X_TS_FAULT_SHIFT)
#define BQ2512X_TS_MORMAL                (0 << BQ2512X_TS_FAULT_SHIFT)
#define BQ2512X_TS_COLD_HOT              (1 << BQ2512X_TS_FAULT_SHIFT)
#define BQ2512X_TS_COOL                  (2 << BQ2512X_TS_FAULT_SHIFT)
#define BQ2512X_TS_WARM                  (3 << BQ2512X_TS_FAULT_SHIFT)


#define BQ2512X_TS_FAULT_EN              (1 << 7) /* TS function enable/disable */

/* REG 3 */
#define BQ2512X_FAST_CHG_HZ_MODE        (1 << 0) /* R/W High impedance mode */
#define BQ2512X_FAST_CHG_EN             (1 << 1) /* R/W charger enable */
#define BQ2512X_FAST_CHG_ICHG_SHIFT      2        /* Battery charge current setting */
#define BQ2512X_FAST_CHG_ICHG_MASK      (0x1F << BQ2512X_FAST_CHG_ICHG_SHIFT)
#define BQ2512X_FAST_CHG_ICHG_RANGE     (1 << 7) /* Setting the range for current charge */


/* REG 4 */
#define BQ2512X_PRE_TERM_RES                (1 << 0) /* R/W Reserved */
#define BQ2512X_PRE_TERM_TE                 (1 << 1) /* R/W charge current termination enable/disable */
#define BQ2512X_PRE_TERM_IPRETERM_0         (1 << 2) /* R/W termination current 500uA or 1mA */
#define BQ2512X_PRE_TERM_IPRETERM_1         (1 << 3) /* R/W termination current 1 or 2mA */
#define BQ2512X_PRE_TERM_IPRETERM_2         (1 << 4) /* R/W termination current 2 or 4mA */
#define BQ2512X_PRE_TERM_IPRETERM_3         (1 << 5) /* R/W termination current 4 or 8mA */
#define BQ2512X_PRE_TERM_IPRETERM_4         (1 << 6) /* R/W termination current 8 or 16mA */
#define BQ2512X_PRE_TERM_IPRETERM_RANGE     (1 << 7) /* R/W termination range 0 - 500uA-5mA range 1 - 6mA-37mA */

/* REG 5 */
#define BQ2512X_VBREG_RES       (1 << 0) /* R/W Reserved */
#define BQ2512X_VBREG_SHIFT     1        /* VBREG shift value */
#define BQ2512X_VBREG_MASK      (0x7F << BQ2512X_VBREG_SHIFT)

/* REG 6 */
#define BQ2512X_SYS_VOUT_01_MIN     1300
#define BQ2512X_SYS_VOUT_01_MAX     2800
#define BQ2512X_SYS_VOUT_11_MIN     1800
#define BQ2512X_SYS_VOUT_11_MAX     3300


#define BQ2512X_SYS_VOUT_RES        (1 << 0) /* R/W Reserved */
#define BQ2512X_SYS_VOUT_SHIFT       1       /* SYS vout shift value */
#define BQ2512X_SYS_VOUT_MASK       (0xF << BQ2512X_SYS_VOUT_SHIFT)
#define BQ2512X_SYS_VOUT_0          (1 << 1) /* R/W OUT Voltage: 100 mV step if SYS_SEL is 01 or 11 */
#define BQ2512X_SYS_VOUT_1          (1 << 2) /* R/W OUT Voltage: 200 mV step if SYS_SEL is 01 or 11 */
#define BQ2512X_SYS_VOUT_2          (1 << 3) /* R/W OUT Voltage: 400 mV step if SYS_SEL is 01 or 11 */
#define BQ2512X_SYS_VOUT_3          (1 << 4) /* R/W OUT Voltage: 800 mV step if SYS_SEL is 01 or 11 */
#define BQ2512X_SYS_SEL_SHIFT       5        /* SYS vout shift value */
#define BQ2512X_SYS_SEL_MASK        (3 << BQ2512X_SYS_SEL_SHIFT)
#  define BQ2512X_SYS_SEL_0         (0 << BQ2512X_SYS_SEL_SHIFT) /* R/W OUT Voltage: 1.1 and 1.2V */
#  define BQ2512X_SYS_SEL_1         (1 << BQ2512X_SYS_SEL_SHIFT) /* R/W OUT Voltage: 1.3 - 2.8V */
#  define BQ2512X_SYS_SEL_2         (2 << BQ2512X_SYS_SEL_SHIFT) /* R/W OUT Voltage: 1.5 - 2.75V */
#  define BQ2512X_SYS_SEL_3         (3 << BQ2512X_SYS_SEL_SHIFT) /* R/W OUT Voltage: 1.8 - 3.3V */
#define BQ2512X_SYS_VOUT_EN         (1 << 7) /* R/W 1 - enable sw, 0 - disable sw */

/*REG 7*/
#define BQ2512X_LDO_VOUT_MIN        800
#define BQ2512X_LDO_VOUT_MAX        3300

#define BQ2512X_LDO_VOUT_SHIFT       2       /* SYS vout shift value */
#define BQ2512X_LDO_VOUT_MASK       (0x1F << BQ2512X_LDO_VOUT_SHIFT)
#define BQ2512X_LDO_VOUT_EN         (1 << 7) /* R/W 1 - enable LDO, 0 - disable LDO */

/*REG 8*/
#define BQ2512X_PGB_MR_SHIFT        2 /*PGB_MR bit shift*/
#define BQ2512X_PGB_MR_MASK         (0X01<<BQ2512X_PGB_MR_SHIFT)

#define BQ2512X_PGB_RESET_SHIFT     3
#define BQ2512X_PGB_RESET_MASK      (0X03<<BQ2512X_PGB_RESET_SHIFT)

enum bq25120_reset_time
{
  BQ2512X_RESET_TIME_4S = 0x00,
  BQ2512X_RESET_TIME_8S = 0x01,
  BQ2512X_RESET_TIME_10S = 0x02,
  BQ2512X_RESET_TIME_14S = 0x03,
};




/* REG 9 */
#define BQ2512X_BUVLO_SHIFT          0        /* SYS vout shift value */
#define BQ2512X_BUVLO_MASK          (7 << BQ2512X_BUVLO_SHIFT)
#  define BQ2512X_BUVLO_0           (0 << BQ2512X_BUVLO_SHIFT) /* 0 - reserved */
#  define BQ2512X_BUVLO_1           (1 << BQ2512X_BUVLO_SHIFT) /* 1 - reserved */
#  define BQ2512X_BUVLO_2           (2 << BQ2512X_BUVLO_SHIFT) /* 010: BUVLO = 3.0 V */
#  define BQ2512X_BUVLO_3           (3 << BQ2512X_BUVLO_SHIFT) /* 011: BUVLO = 2.8 V */
#  define BQ2512X_BUVLO_4           (4 << BQ2512X_BUVLO_SHIFT) /* 100: BUVLO = 2.6 V */
#  define BQ2512X_BUVLO_5           (5 << BQ2512X_BUVLO_SHIFT) /* 101: BULVO = 2.4 V */
#  define BQ2512X_BUVLO_6           (6 << BQ2512X_BUVLO_SHIFT) /* 110: BUVLO = 2.2 V */
#  define BQ2512X_BUVLO_7           (7 << BQ2512X_BUVLO_SHIFT) /* 110: BUVLO = 2.2 V */
#define BQ2512X_INLIM_SHIFT          3        /* SYS vout shift value */
#define BQ2512X_INLIM_MASK          (7 << BQ2512X_INLIM_SHIFT)
#  define BQ2512X_INLIM_50MA        (0 << BQ2512X_INLIM_SHIFT) /* 0 - 50mA */
#  define BQ2512X_INLIM_100MA       (1 << BQ2512X_INLIM_SHIFT) /* 1 - 100mA */
#  define BQ2512X_INLIM_150MA       (2 << BQ2512X_INLIM_SHIFT) /* 2 - 150mA */
#  define BQ2512X_INLIM_200MA       (3 << BQ2512X_INLIM_SHIFT) /* 3 - 200mA */
#  define BQ2512X_INLIM_250MA       (4 << BQ2512X_INLIM_SHIFT) /* 4 - 250mA */
#  define BQ2512X_INLIM_300MA       (5 << BQ2512X_INLIM_SHIFT) /* 5 - 300mA */
#  define BQ2512X_INLIM_350MA       (6 << BQ2512X_INLIM_SHIFT) /* 6 - 350mA */
#  define BQ2512X_INLIM_400MA       (7 << BQ2512X_INLIM_SHIFT) /* 7 - 400mA */
#define BQ2512X_INLIM_RES           (1 << 6) /* reserved */
#define BQ2512X_INLIM_RESET         (1 << 7) /* 1 - Reset all regs to default values */


/* REG 10 */
#define BQ2512X_VBMON_RES            (3 << 0) /* R/W Reserved */
#define BQ2512X_VBMON_TH_SHIFT        2        /* SYS vout shift value */
#define BQ2512X_VBMON_TH_MASK        (7 << BQ2512X_VBMON_TH_SHIFT)
#  define BQ2512X_VBMON_TH_0         (1 << BQ2512X_VBMON_TH_SHIFT) /* Above 0% of VBMON_RANGE */
#  define BQ2512X_VBMON_TH_1         (2 << BQ2512X_VBMON_TH_SHIFT) /* Above 2% of VBMON_RANGE */
#  define BQ2512X_VBMON_TH_2         (3 << BQ2512X_VBMON_TH_SHIFT) /* Above 4% of VBMON_RANGE */
#  define BQ2512X_VBMON_TH_3         (6 << BQ2512X_VBMON_TH_SHIFT) /* Above 6% of VBMON_RANGE */
#  define BQ2512X_VBMON_TH_4         (7 << BQ2512X_VBMON_TH_SHIFT) /* Above 8% of VBMON_RANGE */
#define BQ2512X_VBMON_RANGE_SHIFT    5     /* VBMON range shift value */
#define BQ2512X_VBMON_RANGE_MASK     (3 << BQ2512X_VBMON_RANGE_SHIFT)
#  define BQ2512X_VBMON_RANGE_0      (1 << BQ2512X_VBMON_RANGE_SHIFT) /* 00 – 60% to 70% of VBATREG */
#  define BQ2512X_VBMON_RANGE_1      (2 << BQ2512X_VBMON_RANGE_SHIFT) /* 01 – 70% to 80% of VBATREG */
#  define BQ2512X_VBMON_RANGE_2      (3 << BQ2512X_VBMON_RANGE_SHIFT) /* 10 – 80% to 90% of VBATREG */
#  define BQ2512X_VBMON_RANGE_3      (6 << BQ2512X_VBMON_RANGE_SHIFT) /* 11 – 90% to 100% of VBATREG */
#define BQ2512X_VBMON_READ_EN        (1 << 7) /* R/W 1 - enable vbmon */

#endif /* __DRIVERS_POWER_BQ2425X_H */
