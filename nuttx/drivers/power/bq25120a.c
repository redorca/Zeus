/****************************************************************************
 * drivers/power/bq2512x.c
 * Lower half driver for BQ2512x battery charger
 *
 *   Copyright (C) 2015 Alan Carvalho de Assis. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

/* The BQ25120A is a low Iq highly integrated battery charge management
 * solution for wearables and IoT systems
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/power/battery_charger.h>
#include <nuttx/power/battery_ioctl.h>
#include <nuttx/power/bq2512x.h>
#include "bq25120a.h"

/* This driver requires:
 *
 * CONFIG_BATTERY_CHARGER - Upper half battery driver support
 * CONFIG_I2C - I2C support
 * CONFIG_I2C_BQ2512X - And the driver must be explictly selected.
 */

#if defined(CONFIG_BATTERY_CHARGER) && defined(CONFIG_I2C) && \
    defined(CONFIG_I2C_BQ2512X)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_BQ2512X
#  define baterr  _err
#  define batdbg  _info
#  define batinfo _info
#else
#  ifdef CONFIG_CPP_HAVE_VARARGS
#    define baterr(x...)
#    define batdbg(x...)
#    define batinfo(x...)
#  else
#    define baterr (void)
#    define batdbg (void)
#    define batinfo(void)
#  endif
#endif


/****************************************************************************
 * Private
 ****************************************************************************/

struct bq2512x_dev_s
{
  /* The common part of the battery driver visible to the upper-half driver */

  FAR const struct battery_charger_operations_s *ops; /* Battery operations */
  sem_t batsem;                /* Enforce mutually exclusive access */

  /* Data fields specific to the lower half BQ2512x driver follow */

  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
  uint32_t frequency;           /* I2C frequency */
  const struct bq25120_low_level_operations_s *low_level_op; /*gpio operation*/

#ifdef CONFIG_BQ2512X_NRESET_TO_GPIO
  uint8_t        reset_pin_signo; /* signo to use when signaling a interrupt */
  pid_t          reset_pin_receive_pid; /* The task to be signalled */
#endif
  uint8_t        int_pin_signo; /* signo to use when signaling a interrupt */
  pid_t          int_pin_receive_pid; /* The task to be signalled */

};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* I2C support */

static int bq2512x_getreg8(FAR struct bq2512x_dev_s *priv, uint8_t regaddr,
                           FAR uint8_t *regval);
static int bq2512x_putreg8(FAR struct bq2512x_dev_s *priv, uint8_t regaddr,
                           uint8_t regval);

static inline int bq2512x_getreport(FAR struct bq2512x_dev_s *priv,
                                    uint8_t *report);
static inline int bq2512x_getfault(FAR struct bq2512x_dev_s *priv,
                                   uint8_t *fault);
static inline int bq2512x_reset(FAR struct bq2512x_dev_s *priv);
static inline int bq2512x_powersupply(FAR struct bq2512x_dev_s *priv, int current);

/* Battery driver lower half methods */

static int bq2512x_state(struct battery_charger_dev_s *dev, int *status);
static int bq2512x_health(struct battery_charger_dev_s *dev, int *health);
static int bq2512x_online(struct battery_charger_dev_s *dev, bool *status);
static int bq2512x_voltage(struct battery_charger_dev_s *dev, int value);
static int bq2512x_current(struct battery_charger_dev_s *dev, int value);
static int bq2512x_input_current(struct battery_charger_dev_s *dev, int value);
static int bq2512x_operate(struct battery_charger_dev_s *dev, uintptr_t param);
static int bq2512x_set_buck_vout(FAR struct bq2512x_dev_s *priv, uint32_t volts);
static int bq2512x_get_buck_vout(FAR struct bq2512x_dev_s *priv, uint32_t *volts);


/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct battery_charger_operations_s g_bq2512xops =
{
  bq2512x_state,
  bq2512x_health,
  bq2512x_online,
  bq2512x_voltage,
  bq2512x_current,
  bq2512x_input_current,
  bq2512x_operate
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bq2512x_getreg8
 *
 * Description:
 *   Read a 8-bit value from a BQ2512x register pair.
 *
 *   START <I2C write address> ACK <Reg address> ACK
 *   REPEATED-START <I2C read address> ACK Data0 ACK Data1 NO-ACK STOP
 *
 ****************************************************************************/

static int bq2512x_getreg8(FAR struct bq2512x_dev_s *priv, uint8_t regaddr,
                           FAR uint8_t *regval)
{
  struct i2c_config_s config;
  uint8_t val = 0;;
  int ret;

  /* Set up the I2C configuration */

  config.frequency = priv->frequency;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address */

  ret = i2c_writeread(priv->i2c,
                      &config,
                      &regaddr, 1,
                      &val, 1);
  if (ret < 0)
    {
      baterr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Copy 8-bit value to be returned */

  *regval = val;
  return OK;
}

/****************************************************************************
 * Name: bq2512x_putreg8
 *
 * Description:
 *   Write a 8-bit value to a BQ2512x register pair.
 *
 *   START <I2C write address> ACK <Reg address> ACK Data0 ACK Data1 ACK STOP
 *
 ****************************************************************************/

static int bq2512x_putreg8(FAR struct bq2512x_dev_s *priv, uint8_t regaddr,
                           uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t buffer[2];

  /* Set up the I2C configuration */

  config.frequency = priv->frequency;
  config.address   = priv->addr;
  config.addrlen   = 7;

  batinfo("addr: %02x regval: %08x\n", regaddr, regval);

  /* Set up a 3 byte message to send */

  buffer[0] = regaddr;
  buffer[1] = regval;

  /* Write the register address followed by the data (no RESTART) */

  return i2c_write(priv->i2c, &config, buffer, 2);
}

/****************************************************************************
 * Name: bq2512x_getreport
 *
 * Description:
 *   Read the BQ2425X Register #0 (status)
 *
 ****************************************************************************/

static inline int bq2512x_getreport(FAR struct bq2512x_dev_s *priv,
                                    uint8_t *report)
{
  uint8_t regval = 0;
  int ret;

  ret = bq2512x_getreg8(priv, BQ2512X_STATUS_REG, &regval);
  if (ret == OK)
    {
      *report = regval;
    }

  return ret;
}

/****************************************************************************
 * Name: bq2512x_getfault
 *
 * Description:
 *   Read the BQ2425X Register #1 (fault)
 *
 ****************************************************************************/

static inline int bq2512x_getfault(FAR struct bq2512x_dev_s *priv,
                                   uint8_t *fault)
{
  uint8_t regval = 0;
  int ret;

  ret = bq2512x_getreg8(priv, BQ2512X_FAULT_REG, &regval);
  if (ret == OK)
    {
      *fault = regval;
    }

  return ret;
}

/****************************************************************************
 * Name: bq2512x_reset
 *
 * Description:
 *   Reset the BQ2512x
 *
 ****************************************************************************/

static inline int bq2512x_reset(FAR struct bq2512x_dev_s *priv)
{
  int ret;
  uint8_t regval;

  /* Read current register value */

  ret = bq2512x_getreg8(priv, BQ2512X_ILIM_UVLO_REG, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ2425X! Error = %d\n", ret);
      return ret;
    }

  /* Send reset command */

  regval |= BQ2512X_INLIM_RESET;
  ret = bq2512x_putreg8(priv, BQ2512X_ILIM_UVLO_REG, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ2425X! Error = %d\n", ret);
      return ret;
    }

  /* Wait a little bit to clear registers */

  nxsig_usleep(500);

  return OK;
}

/****************************************************************************
 * Name: bq2512x_state
 *
 * Description:
 *   Return the current battery state
 *
 ****************************************************************************/

static int bq2512x_state(struct battery_charger_dev_s *dev, int *status)
{
  FAR struct bq2512x_dev_s *priv = (FAR struct bq2512x_dev_s *)dev;
  uint8_t regval = 0;
  int ret;

  ret = bq2512x_getreport(priv, &regval);
  if (ret < 0)
    {
      *status = BATTERY_UNKNOWN;
      return ret;
    }

  regval &= BQ2512X_STATUS_MASK;

  switch (regval)
    {
      case BQ2512X_STATUS_FAULT:
        *status = BATTERY_FAULT;
        break;
      case BQ2512X_STATUS_CHG_DONE:
        *status = BATTERY_FULL;
        break;
      case BQ2512X_STATUS_CHG_PROGRESS:
        *status = BATTERY_CHARGING;
        break;
      case BQ2512X_STATUS_READY:
        *status = BATTERY_IDLE;
        break;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2512x_health
 *
 * Description:
 *   Return the current battery health state
 *
 * Note: if more than one fault happened the user needs to call this ioctl
 * again to read a new fault, repeat until receive a BATTERY_HEALTH_GOOD.
 *
 ****************************************************************************/

static int bq2512x_health(struct battery_charger_dev_s *dev, int *health)
{
  FAR struct bq2512x_dev_s *priv = (FAR struct bq2512x_dev_s *)dev;
  uint8_t regfault = 0;
  uint8_t regstatus = 0;
  uint8_t regval = 0;
  int ret;

  ret = bq2512x_getreport(priv, &regstatus);
  if (ret < 0)
    {
      *health = BATTERY_HEALTH_UNKNOWN;
      return ret;
    }

  ret = bq2512x_getfault(priv, &regfault);
  if (ret < 0)
    {
      *health = BATTERY_HEALTH_UNKNOWN;
      return ret;
    }

  if ((regstatus & BQ2512X_STATUS_TIMER) == BQ2512X_STATUS_TIMER)
    {
      *health = BATTERY_HEALTH_SAFE_TMR_EXP;
      return OK;
    }

  if ((regfault & BQ2512X_FAULT_VIN_OV) == BQ2512X_FAULT_VIN_OV)
    {
      *health = BATTERY_HEALTH_OVERVOLTAGE;
      return OK;
    }

  if ((regfault & BQ2512X_FAULT_BAT_UVLO) == BQ2512X_FAULT_BAT_UVLO)
    {
      *health = BATTERY_HEALTH_UNDERVOLTAGE;
      return OK;
    }

  if (((regfault & BQ2512X_FAULT_VIN_UV) == BQ2512X_FAULT_VIN_UV) || ((regfault & BQ2512X_FAULT_BAT_OCP) == BQ2512X_FAULT_BAT_OCP))
    {
      *health = BATTERY_HEALTH_UNSPEC_FAIL;
      return OK;
    }

  /* check for temperature faults */
  ret = bq2512x_getreg8(priv, BQ2512X_TS_REG, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ2425X! Error = %d\n", ret);
      return ret;
    }

  if ((regval & BQ2512X_TS_FAULT_MASK) != 0)
    {
      *health = BATTERY_HEALTH_OVERHEAT;
      return ret;
    }

  *health = BATTERY_HEALTH_GOOD;

  return OK;
}

/****************************************************************************
 * Name: bq2512x_online
 *
 * Description:
 *   Return true if the battery is online
 *
 ****************************************************************************/

static int bq2512x_online(struct battery_charger_dev_s *dev, bool *status)
{
  /* There is no concept of online/offline in this driver */

  *status = true;
  return OK;
}

/****************************************************************************
 * Name: bq2512x_powersupply
 *
 * Description:
 *   Set the Power Supply Current Limit.
 *
 ****************************************************************************/

static inline int bq2512x_powersupply(FAR struct bq2512x_dev_s *priv, int current)
{
  uint8_t regval;
  int ret, idx;

  switch (current)
    {
      case 50:
        idx = BQ2512X_INLIM_50MA;
        break;

      case 100:
        idx = BQ2512X_INLIM_100MA;
        break;

      case 150:
        idx = BQ2512X_INLIM_150MA;
        break;

      case 200:
        idx = BQ2512X_INLIM_200MA;
        break;

      case 250:
        idx = BQ2512X_INLIM_250MA;
        break;

      case 300:
        idx = BQ2512X_INLIM_300MA;
        break;

      case 350:
        idx = BQ2512X_INLIM_350MA;
        break;

      case 400:
        idx = BQ2512X_INLIM_400MA;
        break;

      default:
        baterr("ERROR: Current not supported, setting default to 100mA!\n");
        idx = BQ2512X_INLIM_100MA;
        break;
    }

  /* Read current register */

  ret = bq2512x_getreg8(priv, BQ2512X_ILIM_UVLO_REG, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ2425X! Error = %d\n", ret);
      return ret;
    }

  /* Clear previous current and set new value */

  regval &= ~(BQ2512X_INLIM_MASK);
  regval |= idx;

  /* Also clear Reset bit to avoid the resetting BUG */

  regval &= ~(BQ2512X_INLIM_RESET);

  ret = bq2512x_putreg8(priv, BQ2512X_ILIM_UVLO_REG, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ2425X! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2512x_set_buck_vout
 *
 * Description:
 *   Set the voltage output of the PMIC BUCK. Voltage value in mV.
 *
 ****************************************************************************/

static int bq2512x_set_buck_vout(FAR struct bq2512x_dev_s *priv, uint32_t volts)
{
  uint8_t regval = 0;
  int ret;
  uint32_t vout_min = 0;
  uint32_t vout = 0;
  uint8_t vout_sel = 0;

  /* Verify if voltage is in the acceptable range */
  if ((volts < BQ2512X_SYS_VOUT_01_MIN) || (volts > BQ2512X_SYS_VOUT_11_MAX))
    {
      baterr("ERROR: Voltage %d mV is out of range.\n", volts);
      return -EINVAL;
    }

  if ((volts >= BQ2512X_SYS_VOUT_01_MIN) && (volts <= BQ2512X_SYS_VOUT_01_MAX))
    {
      vout_min = BQ2512X_SYS_VOUT_01_MIN;
      vout_sel = BQ2512X_SYS_SEL_1;
    }
  else if ((volts >= BQ2512X_SYS_VOUT_11_MIN) && (volts <= BQ2512X_SYS_VOUT_11_MAX))
    {
      vout_min = BQ2512X_SYS_VOUT_11_MIN;
      vout_sel = BQ2512X_SYS_SEL_3;
    }

  vout = ((volts - vout_min) / 100);

  /* read the VOUT register */
  ret = bq2512x_getreg8(priv, BQ2512X_VOUT_REG, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ2512X! Error = %d\n", ret);
      return ret;
    }

  regval = 0;
  regval |= (vout_sel | BQ2512X_SYS_VOUT_EN);
  regval |= (vout << BQ2512X_SYS_VOUT_SHIFT);

  batinfo("New reg val : 0x%x\n", regval);

  /* write the new value to VOUT register */
  /*ret = bq2512x_putreg8(priv, BQ2512X_VOUT_REG, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ2512X! Error = %d\n", ret);
      return ret;
    }*/


  return OK;
}

/****************************************************************************
 * Name: bq2512x_get_buck_vout
 *
 * Description:
 *   Get the voltage output of the PMIC BUCK. Voltage value in mV.
 *
 ****************************************************************************/

static int bq2512x_get_buck_vout(FAR struct bq2512x_dev_s *priv, uint32_t *volts)
{
  uint8_t regval = 0;
  int ret;
  uint32_t vout = 0;

  ret = bq2512x_getreg8(priv, BQ2512X_VOUT_REG, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ2512X! Error = %d\n", ret);
      return ret;
    }

  vout = ((regval & BQ2512X_SYS_VOUT_MASK) >> BQ2512X_SYS_VOUT_SHIFT);
  switch (regval & BQ2512X_SYS_SEL_MASK)
    {
      case BQ2512X_SYS_SEL_1:
        *volts = BQ2512X_SYS_VOUT_01_MIN + (vout * 100) ;
        break;
      case BQ2512X_SYS_SEL_3:
        *volts = BQ2512X_SYS_VOUT_11_MIN + (vout * 100) ;
        break;
      default:
        *volts = 0;
        break;
    }

  return OK;
}

static int bq2512x_push_time(FAR struct bq2512x_dev_s *priv, enum bq25120_reset_time time)
{
  int ret;
  uint8_t regval;

  /* Read current register value */

  ret = bq2512x_getreg8(priv, BD2512X_BUTTON_CTRL_REG, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ2425X! Error = %d\n", ret);
      return ret;
    }

  /* Send reset command */

  regval &= (~BQ2512X_PGB_RESET_MASK);
  regval |= (time << BQ2512X_PGB_RESET_SHIFT);
  ret = bq2512x_putreg8(priv, BD2512X_BUTTON_CTRL_REG, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ2425X! Error = %d\n", ret);
      return ret;
    }

  return OK;
}


/****************************************************************************
 * Name: bq2512x_get_LDO_vout
 *
 * Description:
 *   Get the voltage output of the PMIC LDO. Voltage value in mV.
 *
 ****************************************************************************/

static int bq2512x_get_LDO_vout(FAR struct bq2512x_dev_s *priv, uint32_t *volts)
{
  uint8_t regval = 0;
  int ret;
  uint32_t vout = 0;

  ret = bq2512x_getreg8(priv, BQ2512X_LDO_CTRL_REG, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ2512X! Error = %d\n", ret);
      return ret;
    }

  vout = ((regval & BQ2512X_LDO_VOUT_MASK) >> BQ2512X_LDO_VOUT_SHIFT);
  *volts = BQ2512X_LDO_VOUT_MIN + (vout * 100) ;

  return OK;
}

/****************************************************************************
 * Name: bq2512x_set_LDO_vout
 *
 * Description:
 *   Set the voltage output of the PMIC LDO. Voltage value in mV.
 *
 ****************************************************************************/

static int bq2512x_set_LDO_vout(FAR struct bq2512x_dev_s *priv, uint32_t volts)
{
  uint8_t regval = 0;
  int ret;
  uint32_t vout = 0;


  /* Verify if voltage is in the acceptable range */
  if ((volts < BQ2512X_LDO_VOUT_MIN) || (volts > BQ2512X_LDO_VOUT_MAX))
    {
      baterr("ERROR: Voltage %d mV is out of range.\n", volts);
      return -EINVAL;
    }

  vout = ((volts - BQ2512X_LDO_VOUT_MIN) / 100);

  /* read the VOUT register */
  ret = bq2512x_getreg8(priv, BQ2512X_LDO_CTRL_REG, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ2512X! Error = %d\n", ret);
      return ret;
    }

  /*We cann't change the LDO output voltage while the LDO is on*/
  if (regval & BQ2512X_LDO_VOUT_EN)
    {
      baterr("ERROR: Can't change LDO output voltage while it's ON!\n");
      return -EPERM;
    }

  regval &= (~BQ2512X_LDO_VOUT_MASK);
  regval |= (vout << BQ2512X_LDO_VOUT_SHIFT);

  batinfo("New reg val : 0x%x\n", regval);

  /* write the new value to VOUT register */
  ret = bq2512x_putreg8(priv, BQ2512X_LDO_CTRL_REG, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ2512X! Error = %d\n", ret);
      return ret;
    }

  return OK;

}

/****************************************************************************
 * Name: bq2512x_LDO_enable
 *
 * Description:
 *   enable or disable the LDO output, if LSCTRL pin is connected,
 *   this will pull up or pull down LSCTRL pin, if not, we will oprate the register.
 *
 ****************************************************************************/
static int bq2512x_LDO_enable(FAR struct bq2512x_dev_s *priv, bool ON_OFF)
{
  int ret = OK;

#ifdef CONFIG_BQ2512X_LSCTRL_TO_GPIO

  ret = priv->low_level_op->bq25120_LSCTRL_write(ON_OFF);

#else
//#error need to be implement!
#endif

  return ret;
}

/****************************************************************************
 * Name: bq2512x_read_PG_pin
 *
 * Input Parameters:
 *   priv      - device private data
 *   MR_shift  - true if PG works as voltage shifted push-button (MR) input
 *               false if works as PG
 *   level     - location to hold the obtained value
 *
 * Description:
 *   Get the PG pin status, true if high, false if low.
 *
 ****************************************************************************/
#ifdef CONFIG_BQ2512X_NPG_TO_GPIO
static int bq2512x_read_MR_pin(FAR struct bq2512x_dev_s *priv,
                               bool MR_shift,
                               uint32_t *level)
{
  uint8_t regval;
  int ret;
  bool pin_level;

  ret = bq2512x_getreg8(priv, BD2512X_BUTTON_CTRL_REG, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ2512X! Error = %d\n", ret);
      return ret;
    }

  if (MR_shift)
    {
      regval |= BQ2512X_PGB_MR_MASK;
    }
  else
    {
      regval &= (~BQ2512X_PGB_MR_MASK);
    }

  ret = bq2512x_putreg8(priv, BD2512X_BUTTON_CTRL_REG, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ2512X! Error = %d\n", ret);
      return ret;
    }

  ret = priv->low_level_op->bq25120_NPG_read(&pin_level);

  if (ret != OK)
    {
      baterr("ERROR: Error reading PG pin of BQ2512X! Error = %d\n", ret);
      return ret;
    }

  *level = (pin_level == true) ? 1 : 0;


  return OK;
}
#endif

/****************************************************************************
 * Name: bq2512x_get_int_status
 *
 * Input Parameters:
 *   priv      - device private data
 *   int       - location to hold interrupt source flag
 *
 * Description:
 *   Get the interrupt source,you can use BQ2512X_CHECK_XXX_INT to check the
 *   interrupt is triggerd by which source
 *
 ****************************************************************************/

static int bq2512x_get_int_status(FAR struct bq2512x_dev_s *priv,
                                  uint32_t *value)
{
  uint8_t fault_regval;
  uint8_t TS_regval;
  int ret;

  ret = bq2512x_getreg8(priv, BQ2512X_FAULT_REG, &fault_regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ2512X! Error = %d\n", ret);
      return ret;
    }

  ret = bq2512x_getreg8(priv, BQ2512X_TS_REG, &TS_regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ2512X! Error = %d\n", ret);
      return ret;
    }

  *value = 0;
  *value |= ((fault_regval & BQ2512X_FAULT_BAT_OCP) ? BQ2512X_INT_BAT_OCP : 0);
  *value |= ((fault_regval & BQ2512X_FAULT_BAT_UVLO) ? BQ2512X_INT_BAT_UVLO : 0);
  *value |= ((fault_regval & BQ2512X_FAULT_VIN_UV) ? BQ2512X_INT_VIN_UV : 0);
  *value |= ((fault_regval & BQ2512X_FAULT_VIN_OV) ? BQ2512X_INT_VIN_OV : 0);
  *value |= ((TS_regval & BQ2512X_TS_FAULT_MASK) == BQ2512X_TS_COLD_HOT ? BQ2512X_INT_TS_COLD_HOT : 0);
  *value |= ((TS_regval & BQ2512X_TS_FAULT_MASK) == BQ2512X_TS_COOL ? BQ2512X_INT_TS_COOL : 0);
  *value |= ((TS_regval & BQ2512X_TS_FAULT_MASK) == BQ2512X_TS_WARM ? BQ2512X_INT_TS_WARM : 0);
  *value |= ((TS_regval & BQ2512X_TS_FAULT_OPEN) ? BQ2512X_INT_TS_OFF : 0);

  return OK;
}

static int bq2512x_get_bat_temp(FAR struct bq2512x_dev_s *priv,
                                uint32_t *temp)
{
  uint8_t TS_regval;
  int ret;

  ret = bq2512x_getreg8(priv, BQ2512X_TS_REG, &TS_regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ2512X! Error = %d\n", ret);
      return ret;
    }

  switch (TS_regval & BQ2512X_TS_FAULT_MASK)
    {
      case BQ2512X_TS_MORMAL:
        *temp = BQ2512X_BAT_NORMAL;
        break;
      case BQ2512X_TS_COLD_HOT:
        *temp = BQ2512X_BAT_COLD_HOT;
        break;
      case BQ2512X_TS_COOL:
        *temp = BQ2512X_BAT_COOL;
        break;
      case BQ2512X_TS_WARM:
        *temp = BQ2512X_BAT_WARM;
        break;
    }

  return OK;
}


void bq2512x_int_pin_handler(void            *param)
{
  FAR struct bq2512x_dev_s *priv = (struct bq2512x_dev_s *)param;

  kill(priv->int_pin_receive_pid, priv->int_pin_signo);
}

#ifdef CONFIG_BQ2512X_NRESET_TO_GPIO
void bq2512x_reset_pin_handler(void *param)
{
  FAR struct bq2512x_dev_s *priv = (struct bq2512x_dev_s *)param;

  kill(priv->reset_pin_receive_pid, priv->reset_pin_signo);
}
#endif


/****************************************************************************
 * Name: bq2512x_voltage
 *
 * Description:
 *   Set battery charger voltage
 *
 ****************************************************************************/

static int bq2512x_voltage(struct battery_charger_dev_s *dev, int value)
{
  FAR struct bq2512x_dev_s *priv = (FAR struct bq2512x_dev_s *)dev;
  uint8_t regval;
  int ret, idx;

  /* Verify if voltage is in the acceptable range */

  if ((value < BQ2512X_VOLT_MIN) || (value > BQ2512X_VOLT_MAX))
    {
      baterr("ERROR: Voltage %d mV is out of range.\n", value);
      return -EINVAL;
    }

  ret = bq2512x_getreg8(priv, BQ2512X_VBREG, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ2512X! Error = %d\n", ret);
      return ret;
    }

  /* Voltage starts at 3600mV and goes to 4650mV */

  idx = value - BQ2512X_VOLT_MIN;
  idx = idx / 10;

  /* Clear previous voltage */

  regval &= ~(BQ2512X_VBREG_MASK);
  regval |= (idx << BQ2512X_VBREG_SHIFT);

  ret = bq2512x_putreg8(priv, BQ2512X_VBREG, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ2425X! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2512x_current
 *
 * Description:
 *   Set the battery charger current rate for charging
 *
 ****************************************************************************/

static int bq2512x_current(struct battery_charger_dev_s *dev, int value)
{
  FAR struct bq2512x_dev_s *priv = (FAR struct bq2512x_dev_s *)dev;
  int ret;
  uint8_t regval = 0;
  int idx = 0;

  /* Set current to battery charger */

  /* Verify if current is in the acceptable range */

  if (value < BQ2512X_CURR_MIN || value > BQ2512X_CURR_MAX)
    {
      baterr("ERROR: Current %d mA is out of range.\n", value);
      return -EINVAL;
    }

  ret = bq2512x_getreg8(priv, BQ2512X_FAST_CHG_REG, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ2425X! Error = %d\n", ret);
      return ret;
    }

  /* Current is in 2 ranges 5-35mA and 40-300mA */

  if ((value >= BQ2512X_CURR_MIN) && (value <= BQ2512X_CURR_RANGE1))
    {
      /* In this range you can go in steps of 1mA */
      idx = value - BQ2512X_CURR_MIN;
      idx = idx / 1;
      idx = idx << BQ2512X_FAST_CHG_ICHG_SHIFT;
      idx &= (~BQ2512X_FAST_CHG_ICHG_RANGE);
    }
  else if ((value >= BQ2512X_CURR_RANGE2) && (value <= BQ2512X_CURR_MAX))
    {
      /* In this range you can go in steps of 10mA */
      idx = value - BQ2512X_CURR_RANGE2;
      idx = idx / 10;
      idx = ((idx << BQ2512X_FAST_CHG_ICHG_SHIFT) | BQ2512X_FAST_CHG_ICHG_RANGE);
    }

  /* Clear previous current and set new value */

  regval &= ~(BQ2512X_FAST_CHG_ICHG_MASK | BQ2512X_FAST_CHG_ICHG_RANGE);
  regval |= idx;

  ret = bq2512x_putreg8(priv, BQ2512X_FAST_CHG_REG, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ2425X! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2512x_input_current
 *
 * Description:
 *   Set the power-supply input current limit
 *
 ****************************************************************************/

static int bq2512x_input_current(struct battery_charger_dev_s *dev, int value)
{
  FAR struct bq2512x_dev_s *priv = (FAR struct bq2512x_dev_s *)dev;
  int ret;

  ret = bq2512x_powersupply(priv, value);
  if (ret < 0)
    {
      baterr("ERROR: Failed to set BQ2512x power supply input limit: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2512x_operate
 *
 * Description:
 *   Do miscellaneous battery ioctl()
 *
 ****************************************************************************/

static int bq2512x_operate(struct battery_charger_dev_s *dev, uintptr_t param)
{
  FAR struct bq2512x_dev_s *priv = (FAR struct bq2512x_dev_s *)dev;
  struct batio_operate_msg_s *msg = (struct batio_operate_msg_s *)param;
  uint32_t op, value;
  int ret = OK;


  op = msg->operate_type;
  value = (int)msg->u32;
  switch (op)
    {
      case BATIO_OPRTN_BUCK_VOUT_GET:
        ret = bq2512x_get_buck_vout(priv, &(msg->u32));
        break;
      case BATIO_OPRTN_BUCK_VOUT_SET:
        ret = bq2512x_set_buck_vout(priv, value);
        break;
      case BATIO_OPRTN_LDO_VOUT_GET:
        ret = bq2512x_get_LDO_vout(priv, &(msg->u32));
        break;
      case BATIO_OPRTN_LDO_VOUT_SET:
        ret = bq2512x_set_LDO_vout(priv, value);
        break;
      case BATIO_OPRTN_LDO_ENABLE:
        ret = bq2512x_LDO_enable(priv, true);
        break;
      case BATIO_OPRTN_LDO_DISABLE:
        ret = bq2512x_LDO_enable(priv, false);
        break;
#ifdef CONFIG_BQ2512X_NPG_TO_GPIO
      case BATIO_OPRTN_READ_PG:
        ret = bq2512x_read_MR_pin(priv, false, &(msg->u32));;
        break;
      case BATIO_OPRTN_READ_MR:
        ret = bq2512x_read_MR_pin(priv, true, &(msg->u32));
        break;
#endif
      case BATIO_OPRTN_GET_INT_STA:
        ret = bq2512x_get_int_status(priv, &(msg->u32));
        break;
      case BATIO_OPRTN_GET_BAT_TEMP:
        ret = bq2512x_get_bat_temp(priv, &(msg->u32));
        break;
      case BATIO_OPRTN_REGISTER_INT:
        priv->int_pin_receive_pid   = getpid();
        priv->int_pin_signo = (uint8_t)msg->u32;
        break;
      case BATIO_OPRTN_UNREGISTER_INT:
        priv->int_pin_receive_pid   = 0;
        priv->int_pin_signo = 0;
        break;
#ifdef CONFIG_BQ2512X_NRESET_TO_GPIO
      case BATIO_OPRTN_REGISTER_RESET:
        priv->reset_pin_receive_pid   = getpid();
        priv->reset_pin_signo = (uint8_t)msg->u32;
        break;
      case BATIO_OPRTN_UNREGISTER_RESET:
        priv->reset_pin_receive_pid   = 0;
        priv->reset_pin_signo = 0;
        break;
#endif
      case BATIO_OPRTN_ENABLE_INT:
        ret = priv->low_level_op->bq25120_INT_enable(true);
        break;
      case BATIO_OPRTN_DISABLE_INT:
        ret = priv->low_level_op->bq25120_INT_enable(false);
        break;
#ifdef CONFIG_BQ2512X_NRESET_TO_GPIO
      case BATIO_OPRTN_ENABLE_RESET:
        ret = priv->low_level_op->bq25120_RESET_enable(true);
        break;
      case BATIO_OPRTN_DISABLE_RESET:
        ret = priv->low_level_op->bq25120_RESET_enable(false);
        break;
#endif
      default:
        baterr("not implement command:%d!\n", op);
        ret = -EINVAL;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bq2512x_initialize
 *
 * Description:
 *   Initialize the BQ2512X battery driver and return an instance of the
 *   lower_half interface that may be used with battery_charger_register();
 *
 *   This driver requires:
 *
 *   CONFIG_BATTERY_CHARGER - Upper half battery driver support
 *   CONFIG_I2C - I2C support
 *   CONFIG_I2C_BQ2512X - And the driver must be explictly selected.
 *
 * Input Parameters:
 *   i2c       - An instance of the I2C interface to use to communicate with
 *               the BQ2512X
 *   addr      - The I2C address of the BQ2512X (Better be 0x6A)
 *   frequency - The I2C frequency
 *   current   - The input current our power-supply can offer to charger
 *
 * Returned Value:
 *   A pointer to the initialized lower-half driver instance.  A NULL pointer
 *   is returned on a failure to initialize the BQ2512X lower half.
 *
 ****************************************************************************/

FAR struct battery_charger_dev_s *
bq2512x_initialize(FAR struct i2c_master_s *i2c,
                   uint8_t addr,
                   uint32_t frequency,
                   const struct bq25120_low_level_operations_s *ll_op)
{
  FAR struct bq2512x_dev_s *priv;
  int ret;
  uint8_t regval = 0;

  /* Initialize the BQ2512X device structure */

  priv = (FAR struct bq2512x_dev_s *)kmm_zalloc(sizeof(struct bq2512x_dev_s));
  if (priv)
    {
      /* Initialize the BQ2512X device structure */

      nxsem_init(&priv->batsem, 0, 1);
      priv->ops       = &g_bq2512xops;
      priv->i2c       = i2c;
      priv->addr      = addr;
      priv->frequency = frequency;
      priv->int_pin_receive_pid = 0;
      priv->int_pin_signo = 0;
#ifdef CONFIG_BQ2512X_NRESET_TO_GPIO
      priv->reset_pin_receive_pid = 0;
      priv->reset_pin_signo = 0;
#endif

      priv->low_level_op = ll_op;



      /*config INT pin as interrupt source*/
      if (priv->low_level_op && priv->low_level_op->bq25120_INT_cfg_interrupt)
        {
          ret = priv->low_level_op->bq25120_INT_cfg_interrupt(bq2512x_int_pin_handler, priv);
        }
      else
        {
          ret = -EINVAL;
        }

      if (ret < 0)
        {
          baterr("ERROR: Failed to config INT pin\n");
          goto err_out;
        }

#ifdef CONFIG_BQ2512X_NRESET_TO_GPIO

      if (priv->low_level_op && priv->low_level_op->bq25120_RESET_cfg_interrupt)
        {
          ret = priv->low_level_op->bq25120_RESET_cfg_interrupt(bq2512x_reset_pin_handler, priv);
        }
      else
        {
          ret = -EINVAL;
        }

      if (ret < 0)
        {
          baterr("ERROR: Failed to config RESET pin\n");
          goto err_out;
        }
#endif

#ifdef CONFIG_BQ2512X_LSCTRL_TO_GPIO

      if (priv->low_level_op && priv->low_level_op->bq25120_LSCTRL_cfg_output)
        {
          ret = priv->low_level_op->bq25120_LSCTRL_cfg_output();
        }
      else
        {
          ret = -EINVAL;
        }

      if (ret < 0)
        {
          baterr("ERROR: Failed to config LSCTRL pin\n");
          goto err_out;
        }
#endif

#ifdef CONFIG_BQ2512X_NPG_TO_GPIO

      if (priv->low_level_op && priv->low_level_op->bq25120_NPG_cfg_input)
        {
          ret = priv->low_level_op->bq25120_NPG_cfg_input();
        }
      else
        {
          ret = -EINVAL;
        }

      if (ret < 0)
        {
          baterr("ERROR: Failed to config NPG pin\n");
          goto err_out;
        }
#endif


      /*do a dumb read to wake up its IIC block */
      bq2512x_getreport(priv, &regval);

      /* check if the BQ is on. status reg should be 0x01 */
      ret = bq2512x_getreport(priv, &regval);
      if (ret < 0)
        {
          baterr("ERROR: Failed to read the status of BQ2512x: %d\n", ret);
          goto err_out;
        }

      if ((regval & BQ2512X_STATUS_SYS_EN) != BQ2512X_STATUS_SYS_EN)
        {
          baterr("ERROR: status is not correct for BQ2512x: %d\n", ret);
          goto err_out;
        }

      /* Reset the BQ2512X */
      ret = bq2512x_reset(priv);
      if (ret < 0)
        {
          baterr("ERROR: Failed to reset the BQ2512X: %d\n", ret);
          goto err_out;
        }

      /*set push button reset time */
      ret = bq2512x_push_time(priv, BQ2512X_RESET_TIME_4S);
      if (ret < 0)
        {
          baterr("ERROR: Failed to set BQ2512X reset time: %d\n", ret);
          goto err_out;
        }

    }

  return (FAR struct battery_charger_dev_s *)priv;

err_out:
  sem_destroy(&priv->batsem);
  kmm_free(priv);
  return NULL;
}

#endif /* CONFIG_BATTERY_CHARGER && CONFIG_I2C && CONFIG_I2C_BQ2512X */
