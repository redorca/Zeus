/****************************************************************************
 * include/nuttx/sensors/ioctl.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_SENSORS_IOCTL_H
#define __INCLUDE_NUTTX_SENSORS_IOCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* common IOCTL commands*/
#define SNIOC_READID        _SNIOC(0x001b) /* Arg: uint32_t* pointer */

/* IOCTL commands unique to the BH1750FVI */

#define SNIOC_CHRM          _SNIOC(0x0001) /* Contin. H-Res Mode Arg: None   */
#define SNIOC_CHRM2         _SNIOC(0x0002) /* Contin. H-Res Mode2 Arg: None  */
#define SNIOC_CLRM          _SNIOC(0x0003) /* Contin. L-Res Mode Arg: None   */
#define SNIOC_OTHRM         _SNIOC(0x0004) /* One Time H-Res Mode Arg: None  */
#define SNIOC_OTHRM2        _SNIOC(0x0005) /* One Time H-Res Mode2 Arg: None */
#define SNIOC_OTLRM         _SNIOC(0x0006) /* One Time L-Res Mode Arg: None  */
#define SNIOC_CHMEATIME     _SNIOC(0x0007) /* Change Meas. Time Arg: uint8_t */

/* IOCTL commands unique to the KXTJ9 */

#define SNIOC_ENABLE        _SNIOC(0x0008) /* Arg: None */
#define SNIOC_DISABLE       _SNIOC(0x0009) /* Arg: None */
#define SNIOC_CONFIGURE     _SNIOC(0x000a) /* Arg: enum kxtj9_odr_e value */

/* IOCTL commands common to the LM75, LM92 (and compatible parts) */

#define SNIOC_READCONF      _SNIOC(0x000b) /* Arg: uint8_t* pointer */
#define SNIOC_WRITECONF     _SNIOC(0x000c) /* Arg: uint8_t value */
#define SNIOC_SHUTDOWN      _SNIOC(0x000d) /* Arg: None */
#define SNIOC_POWERUP       _SNIOC(0x000e) /* Arg: None */
#define SNIOC_FAHRENHEIT    _SNIOC(0x000f) /* Arg: None */
#define SNIOC_CENTIGRADE    _SNIOC(0x0010) /* Arg: None */
#define SNIOC_READTHYS      _SNIOC(0x0011) /* Arg: b16_t* pointer */
#define SNIOC_WRITETHYS     _SNIOC(0x0012) /* Arg: b16_t value */

/* IOCTL commands unique to the LM75 */

#define SNIOC_READTOS       _SNIOC(0x0013) /* Arg: b16_t* pointer */
#define SNIOC_WRITETOS      _SNIOC(0x0014) /* Arg: b16_t value */

/* IOCTL commands for LM92 and TMP108 */

#define SNIOC_READTCRIT     _SNIOC(0x0015) /* Arg: b16_t* pointer */
#define SNIOC_WRITETCRIT    _SNIOC(0x0016) /* Arg: b16_t value */
#define SNIOC_READTLOW      _SNIOC(0x0017) /* Arg: b16_t* pointer */
#define SNIOC_WRITETLOW     _SNIOC(0x0018) /* Arg: b16_t value */
#define SNIOC_READTHIGH     _SNIOC(0x0019) /* Arg: b16_t* pointer */
#define SNIOC_WRITETHIGH    _SNIOC(0x001a) /* Arg: b16_t value */

/* IOCTL commands unique to the LSM9DS1 */

#define SNIOC_START         _SNIOC(0x001c) /* Arg: None */
#define SNIOC_STOP          _SNIOC(0x001d) /* Arg: None */
#define SNIOC_SETSAMPLERATE _SNIOC(0x001e) /* Arg: uint32_t value */
#define SNIOC_SETFULLSCALE  _SNIOC(0x001f) /* Arg: uint32_t value */

/* IOCTL commands unique to the MB7040 */

#define SNIOC_MEASURE       _SNIOC(0x0020) /* Arg: None */
#define SNIOC_RANGE         _SNIOC(0x0021) /* Arg: int32_t* pointer */
#define SNIOC_CHANGEADDR    _SNIOC(0x0022) /* Arg: uint8_t value */

/* IOCTL commands unique to the MCP9844 */

#define SNIOC_READTEMP      _SNIOC(0x0023)  /* Arg: mcp9844_temp_arg_s* pointer */
#define SNIOC_SETRESOLUTION _SNIOC(0x0024)  /* Arg: uint16_t value */

/* IOCTL commands unique to the MS58XX */

#define SNIOC_MEASURE       _SNIOC(0x0025) /* Arg: None */
#define SNIOC_TEMPERATURE   _SNIOC(0x0026) /* Arg: int32_t* pointer */
#define SNIOC_PRESSURE      _SNIOC(0x0027) /* Arg: int32_t* pointer */
#define SNIOC_RESET         _SNIOC(0x0028) /* Arg: None */
#define SNIOC_OVERSAMPLING  _SNIOC(0x0029) /* Arg: uint16_t value */

/* IOCTL command for accelerometer & Gyro */
typedef enum
{
  GA_MODE_SUSPEND,/*suspend mode, lowest power consumption, long wake-up time*/
  GA_MODE_LOWPOWER,/*low power mode, medium power consumption, quick wake-up*/
  GA_MODE_NORMAL,/*mormal mode, highest power consumption, don't need to wake*/
  GA_MODE_FASTSTARTUP,/*special for gyro, consume a bit more power than suspend mode
                       *and reduce the startup time sharply*/
} sn_ga_mode_s;

typedef struct _sn_ga_param
{
  int    range;/*for accelerometer, the unit of this value is g, for example 4 means 4g*/
  /*for gyro, the unit of this value is dps, for example 1000 means 1000dps*/
  int    resolution;/*the unit of this value is bit, for example 8 means 8bits*/
  int    odr;/*the unit of this value is 0.1HZ, for example 500 means 500*0.1HZ=50HZ*/
  sn_ga_mode_s     power_mode;
} sn_ga_param_s;


/* Command:     SNIOC_G_SPARAM
 * Description: set gyro param
 * Argument:    pointer to the sn_ga_param_s struct*/

#define SNIOC_G_SPARAM   _SNIOC(0x002a)

/* Command:     SNIOC_A_SPARAM
 * Description: set accel param
 * Argument:    pointer to the sn_ga_param_s struct*/

#define SNIOC_A_SPARAM   _SNIOC(0x002b)

/* Command:     SNIOC_G_SPARAM
 * Description: get gyro param
 * Argument:    pointer to the sn_ga_param_s struct*/

#define SNIOC_G_GPARAM   _SNIOC(0x002c)

/* Command:     SNIOC_A_SPARAM
 * Description: get accel param
 * Argument:    pointer to the sn_ga_param_s struct*/

#define SNIOC_A_GPARAM   _SNIOC(0x002d)


/* Gyro & Accelerometer sensor RAW data */

typedef struct _sn_ga_raw
{
  int16_t x_axis;
  int16_t y_axis;
  int16_t z_axis;
} sn_ga_raw_s;

/* Command:     SNIOC_G_GRAW
 * Description: get gyro raw data
 * Argument:    pointer to the sn_ga_raw_s struct*/

#define SNIOC_G_GRAW     _SNIOC(0x002e) /* get Gyro raw data */

/* Command:     SNIOC_A_GRAW
 * Description: get accelerometer raw data
 * Argument:    pointer to the sn_ga_raw_s struct*/

#define SNIOC_A_GRAW     _SNIOC(0x002f) /* get accelerometer raw data */

/* Command:     SNIOC_GA_CONFIG_ISR
 * Description: Configuring sensor interrupt source
 * Argument:    pointer to the interrupt configration struct,
                different seneor may have different data type*/

#define SNIOC_GA_CONFIG_ISR       _SNIOC(0x0030)

/* Command:     SNIOC_GA_GINTSTATUS
 * Description: get the interrupt status.
 * Argument:    uint32_t*
 *              Pointer to the variable which will hold the interrupt status.
 *              If a interrupt is triggered, the corresponding bits will be set to 1.
 */

#define SNIOC_GA_GINTSTATUS       _SNIOC(0x0031)


/* Command:     SNIOC_GA_REGISTER_INT
 * Description: Register to receive a signal whenever an interrupt
 *              is received.
 * Argument:    The number of signal to be generated when the interrupt
 *              occurs.*/

#define SNIOC_GA_REGISTER_INT     _SNIOC(0x0032)

/* Command:     SNIOC_GA_UNREGISTER_INT
 * Description: Stop receiving signals.
 * Argument:    None.*/

#define SNIOC_GA_UNREGISTER_INT   _SNIOC(0x0033)


/* IOCTL commands unique to the bmi160 */

/* Command:     SNIOC_ACCEL_SETBW
 * Description: set accelormeter bandwidth
 * Argument:    value of the bandwidth */

#define SNIOC_ACCEL_SETBW         _SNIOC(0x0034)

/* Command:     SNIOC_ACCEL_GETBW
 * Description: get accelormeter bandwidth
 * Argument:    pointer to the bandwidth */

#define SNIOC_ACCEL_GETBW         _SNIOC(0x0035)

/* Command:     SNIOC_GYRO_SETBW
 * Description: set gyro bandwidth
 * Argument:    value of the bandwidth */

#define SNIOC_GYRO_SETBW          _SNIOC(0x0036)

/* Command:     SNIOC_GYRO_GETBW
 * Description: get gyro bandwidth
 * Argument:    pointer to the bandwidth */

#define SNIOC_GYRO_GETBW          _SNIOC(0x0037)

/* Command:     SNIOC_A_START_FIFO
 * Description: start fifo data collection
 * Argument:    None */

#define SNIOC_A_START_FIFO          _SNIOC(0x0038)

/* Command:     SNIOC_A_STOP_FIFO
 * Description: stop fifo data collection
 * Argument:    None */

#define SNIOC_A_STOP_FIFO          _SNIOC(0x0039)

/* Command:     SNIOC_A_READ_FIFO
 * Description: read fifo data collected
 * Argument:    None */
#define SNIOC_A_READ_FIFO          _SNIOC(0x003A)

/* IOCTL commands to the HTS221 & LPS25H */

#define SNIOC_CFGR              _SNIOC(0x003B)
#define SNIOC_GET_DEV_ID        _SNIOC(0x003C)

/* IOCTL commands unique to the HTS221 */

#define SNIOC_START_CONVERSION  _SNIOC(0x003D)
#define SNIOC_CHECK_STATUS_REG  _SNIOC(0x003E)
#define SNIOC_READ_RAW_DATA     _SNIOC(0x003F)
#define SNIOC_READ_CONVERT_DATA _SNIOC(0x0040)
#define SNIOC_DUMP_REGS         _SNIOC(0x0041)

/* IOCTL commands unique to the LPS25H */

#define SNIOC_TEMPERATURE_OUT   _SNIOC(0x0042)
#define SNIOC_PRESSURE_OUT      _SNIOC(0x0043)
#define SNIOC_SENSOR_OFF        _SNIOC(0x0044)

/* IOCTL commands unique to the LIS2DH */

#define SNIOC_WRITESETUP            _SNIOC(0x0045) /* Arg: uint8_t value */
#define SNIOC_WRITE_INT1THRESHOLD   _SNIOC(0x0046) /* Arg: uint8_t value */
#define SNIOC_WRITE_INT2THRESHOLD   _SNIOC(0x0047) /* Arg: uint8_t value */
#define SNIOC_RESET_HPFILTER        _SNIOC(0x0048) /* Arg: uint8_t value */
#define SNIOC_START_SELFTEST        _SNIOC(0x0049) /* Arg: uint8_t value */
#define SNIOC_WHO_AM_I              _SNIOC(0x004A)
#define SNIOC_READ_TEMP             _SNIOC(0x004B) /* Arg: int16_t value */

/* IOCTL commands unique to the MAX44009 */

#define SNIOC_INIT              _SNIOC(0x004C)
#define SNIOC_THRESHOLD         _SNIOC(0x004D)

/* IOCTL commands unique to LIS3DH */

#define SNIOC_SET_POWER_MODE    _SNIOC(0x004F) /* Arg: LIS3DH_POWER_xxx */
#define SNIOC_SET_DATA_RATE     _SNIOC(0x0050) /* Arg: LIS3DH_ODR_xxx */
#define SNIOC_SET_DATA_FORMAT   _SNIOC(0x0051) /* Arg: LIS3DH_FORMAT_xxx */

/* IOCTL commands unique to the MAX86140 */

#define SNIOC_ENABLE_INT1        _SNIOC(0x0052)
#define SNIOC_DISABLE_INT1       _SNIOC(0x0053)
#define SNIOC_CLEAR_INT1         _SNIOC(0x0054)
#define SNIOC_GPIO_1_2_TEST      _SNIOC(0x0055)

/* IOCTL commands unique to BMM150 */

/*set high value threshold for BMM150*/
#define SNIOC_HIGH_THRESHOLD    _SNIOC(0x0052) /*Arg: pointer to uint8_t value*/
/*set low value threshold for BMM150*/
#define SNIOC_LOW_THRESHOLD     _SNIOC(0x0053) /*Arg: pointer to uint8_t value*/
/*Activates mapping of differnent kinds of interrupt to the INT pin*/
#define SNIOC_INT_ENABLE        _SNIOC(0x0054) /* Arg: pointer to bmm150_int_type */
/*Deactives mapping of differnent kinds of interrupt to the INT pin*/
#define SNIOC_INT_DISABLE       _SNIOC(0x0055) /* Arg: pointer to bmm150_int_type */
/*fetch interrupt status*/
#define SNIOC_INT_STATUS        _SNIOC(0x0056) /* Arg: pointer to uint16_t type */

#endif /* __INCLUDE_NUTTX_SENSORS_IOCTL_H */
