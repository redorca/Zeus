/****************************************************************************
 * include/nuttx/qdecoder.h
 *
 *   Copyright (C) 2012, 2015, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Author: Levin Li <zhiqiang@zglue.com>
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

#ifndef __INCLUDE_NUTTX_SENSORS_QDECODER_H
#define __INCLUDE_NUTTX_SENSORS_QDECODER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_QDECODER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************
 * CONFIG_QDECODER - Enables support for the quadrature decoder upper half
 */

/* IOCTL Commands ***********************************************************/
/* The Quadrature Encode module uses a standard character driver framework.
 * However, since the driver is a device control interface rather than a
 * data transfer interface, the majority of the functionality is implemented
 * in driver ioctl calls.  The  ioctl commands are listed below:
 *
 * QDIOG_DATA - Get the current position from the decoder.
 *   Argument: qdec_data_s pointer to the location to return the position.
 *             up to support 3 channel qdecoder
 * QDIOC_DEC_CNT - wait until to decoder the cnt sample .
 *   Argument: init32_t
 */

typedef enum _qdec_sampleperiod
{
  QDEC_SAMPLE_PERIOD_128US = 0,
  QDEC_SAMPLE_PERIOD_256US,
  QDEC_SAMPLE_PERIOD_512US,
  QDEC_SAMPLE_PERIOD_1024US,
  QDEC_SAMPLE_PERIOD_2048US,
  QDEC_SAMPLE_PERIOD_4096US,
  QDEC_SAMPLE_PERIOD_8192US,
  QDEC_SAMPLE_PERIOD_16384US,
  QDEC_SAMPLE_PERIOD_32768US,
  QDEC_SAMPLE_PERIOD_65536US,
  QDEC_SAMPLE_PERIOD_131072US = 10,
} qdec_sample_period_s;

typedef struct _qdec_setting
{
  qdec_sample_period_s sample_period;
  bool                 dfilter_enable;
} qdec_setting_s;

#define QDIOC_SETTING      _QDIOC(0x0001) /* Arg: qdec_setting_s* pointer */
#define QDIOC_START        _QDIOC(0x0002) /* Arg: None */
#define QDIOC_STOP         _QDIOC(0x0003) /* Arg: None */
#define QDIOC_RESET        _QDIOC(0x0004) /* Arg: None */

/*
typedef enum _qdec_direction
{
  QDEC_DIRECTION_N = 0,
  QDEC_DIRECTION_FORWARD = 1,
  QDEC_DIRECTION_BACK = -1,
} qdec_direction_s;

 */
typedef struct _qdec_data
{
  int32_t x;
  int32_t y;
  int32_t z;
  uint8_t xflag;      /* indicate if X is valid data */
  uint8_t yflag;      /* indicate if Y is valid data */
  uint8_t zflag;      /* indicate if Z is valid data */
} qdec_data_s;

/* Arg: qdec_data_s* pointer */

#define QDIOG_POSITION     _QDIOC(0x0005)


/* wait until cnt count decoder generate
 * Arg:  int32_t  cnt
 */

#define QDIOC_DEC_CNT  _QDIOC(0x0006)

#define QD_FIRST           0x0001         /* First required command */
#define QD_NCMDS           6              /* Two required commands */

/* User defined ioctl commands are also supported. These will be forwarded
 * by the upper-half QD driver to the lower-half QD driver via the ioctl()
 * method fo the QD lower-half interface.  However, the lower-half driver
 * must reserve a block of commands as follows in order prevent IOCTL
 * command numbers from overlapping.
 */



/****************************************************************************
 * Public Types
 ****************************************************************************/
/* This is the vtable that is used to by the upper half quadrature decoder
 * to call back into the lower half quadrature decoder.
 */

struct qd_lowerhalf_s;
struct qd_ops_s
{
  /* This method is called when the driver is opened.  The lower half driver
   * should configure and initialize the device so that it is ready for use.
   * The initial position value should be zero.
   */

  CODE int (*setup)(FAR struct qd_lowerhalf_s *lower);

  /* This method is called when the driver is closed.  The lower half driver
   * should stop data collection, free any resources, disable timer hardware,
   * and put the system into the lowest possible power usage state
   */

  CODE int (*shutdown)(FAR struct qd_lowerhalf_s *lower);


  /* starting to measure data */

  CODE int (*start)(FAR struct qd_lowerhalf_s *lower);

  /* stop to measure data */

  CODE int (*stop)(FAR struct qd_lowerhalf_s *lower);

  /* Reset the position measurement to zero. */

  CODE int (*reset)(FAR struct qd_lowerhalf_s *lower);


  /* Lower-half logic may support platform-specific ioctl commands */

  CODE int (*ioctl)(FAR struct qd_lowerhalf_s *lower,
                    int cmd, unsigned long arg);
};

/* This is the interface between the lower half quadrature decoder driver
 * and the upper half quadrature decoder driver.  A (device-specific)
 * instance of this structure is passed to the upper-half driver when the
 * quadrature decoder driver is registered.
 *
 * Normally that lower half logic will have its own, custom state structure
 * that is simply cast to struct qd_lowerhalf_s.  In order to perform such
 * casts, the initial fields of the custom state structure match the initial
 * fields of the following generic lower half state structure.
 */

struct qd_lowerhalf_s
{
  /* The first field of this state structure must be a pointer to the lower-
   * half callback structure:
   */

  FAR const struct qd_ops_s *ops;

  /* The custom timer state structure may include additional fields after
   * the pointer to the callback structure.
   */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: qd_register
 *
 * Description:
 *   Register the Quadrature Encoder lower half device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/qd0"
 *   lower - An instance of the lower half interface
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int qd_register(FAR const char *devpath, FAR struct qd_lowerhalf_s *lower);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_QDECODER */
#endif /* __INCLUDE_NUTTX_SENSORS_QDECODER_H */
