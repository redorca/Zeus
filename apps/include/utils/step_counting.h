/****************************************************************************
 * include/utils/step_counting.h
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

#ifndef __INCLUDE_UTILS_STEP_COUNTING_H
#define __INCLUDE_UTILS_STEP_COUNTING_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
typedef FAR void *stepCNT_t;

/****************************************************************************
 * Public Types
 ****************************************************************************/
/*motion pattern*/
typedef enum
{
  STEP_PATT_UNKNOWN,
  STEP_PATT_WALK,
  STEP_PATT_RUN,
  STEP_PATT_STATIC,
} step_patt;



/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
/****************************************************************************
 * Name: stepCNT_start
 *
 * Description:
 *   create step counting thread and start it
 *
 * Input Parameters:
 *   ptr_handle : pointer to hold the handle
 *   file_path : file path of the accelerometer device, "/dev/accel0", for example.
 *
 * Returned Value:
 *   0 on success, negative value on fail.
 *
 ****************************************************************************/
int stepCNT_start(stepCNT_t *ptr_handle, const char *file_path);

/****************************************************************************
 * Name: stepCNT_stop
 *
 * Description:
 *   stop the step counting thread and release all the allocated resource.
 *
 * Input Parameters:
 *   handle -- handle of the step counting thread.
 *
 * Returned Value:
 *   0 on success, negative value on fail.
 *
 ****************************************************************************/
int stepCNT_stop(stepCNT_t handle);


/****************************************************************************
 * Name: stepCNT_reset
 *
 * Description:
 *   reset the step counting algorithm, total step value will be cleard.
 *
 * Input Parameters:
 *   handle -- handle of the step counting thread.
 *
 * Returned Value:
 *   0 on success, negative value on fail.
 *
 ****************************************************************************/
int stepCNT_reset(stepCNT_t handle);


/****************************************************************************
 * Name: stepCNT_total
 *
 * Description:
 *   fetch the total steps since stepCNT_start was called.
 *
 * Input Parameters:
 *   handle -- handle of the step counting thread.
 *   total  -- pointer to the total step value.
 *
 * Returned Value:
 *   0 on success, negative value on fail.
 *
 ****************************************************************************/

int stepCNT_total(stepCNT_t handle, uint32_t *total);

/****************************************************************************
 * Name: stepCNT_motion_patt
 *
 * Description:
 *   fetch current motion pattern.
 *
 * Input Parameters:
 *   handle -- handle of the step counting thread.
 *   patt -- pointer to the motion pattern value.
 *
 * Returned Value:
 *   0 on success, negative value on fail.
 *
 ****************************************************************************/

int stepCNT_motion_patt(stepCNT_t handle, step_patt *patt);




#endif /*INCLUDE_UTILS_STEP_COUNTING_H */


