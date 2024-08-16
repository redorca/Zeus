/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup app_timer Application Timer
 * @{
 * @ingroup app_common
 *
 * @brief Application timer functionality.
 *
 * @details This module enables the application to create multiple timer instances based on the RTC1
 *          peripheral. Checking for time-outs and invocation of user time-out handlers is performed
 *          in the RTC1 interrupt handler. List handling is done using a software interrupt (SWI0).
 *          Both interrupt handlers are running in APP_LOW priority level.
 *
 * @details When calling app_timer_start() or app_timer_stop(), the timer operation is just queued,
 *          and the software interrupt is triggered. The actual timer start/stop operation is
 *          executed by the SWI0 interrupt handler. Since the SWI0 interrupt is running in APP_LOW,
 *          if the application code calling the timer function is running in APP_LOW or APP_HIGH,
 *          the timer operation will not be performed until the application handler has returned.
 *          This will be the case, for example, when stopping a timer from a time-out handler when not using
 *          the scheduler.
 *
 * @details Use the USE_SCHEDULER parameter of the APP_TIMER_INIT() macro to select if the
 *          @ref app_scheduler should be used or not. Even if the scheduler is
 *          not used, app_timer.h will include app_scheduler.h, so when
 *          compiling, app_scheduler.h must be available in one of the compiler include paths.
 */

#ifndef APP_TIMER_H__
#define APP_TIMER_H__
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  APP_TIMER_MODE_SINGLE_SHOT,                 /**< The timer will expire only once. */
  APP_TIMER_MODE_REPEATED                     /**< The timer will restart each time it expires. */
} app_timer_mode_t;

/****************************************************************************
 * Name: app_timer_create
 *
 * Description:
 *   Create a timer with timer call back function.
 *
 * Input parameters:
 *   timerid - The timer id, should be defined by user and valued by this
 *             API.
 *
 *   timer_cb - Timer callback function, defined by user.
 *
 * Returned Value:
 *   On success, a 0(OK) is returned to the caller.
 *   If an error occurs, the function will return a value of -1 (ERROR)
 *   and set errno to indicate the error.
 *
 ****************************************************************************/
int32_t app_timer_create(timer_t *timerid, void *timer_cb);

/****************************************************************************
 * Name: app_timer_start
 *
 * Description:
 *   Start a timer with timer delay.
 *
 * Input parameters:
 *   timerid - The timer id.
 *
 *   msec - Timer callback will be called after delay time. Measured in milliseconds.
 *
 *   mode: choose from  APP_TIMER_MODE_SINGLE_SHOT or APP_TIMER_MODE_REPEATED
 *
 * Returned Value:
 *   On success, a 0(OK) is returned to the caller.
 *   If an error occurs, the function will return a value of -1 (ERROR)
 *   and set errno to indicate the error.
 *
 ****************************************************************************/
int32_t app_timer_start(timer_t timerid, int msec, app_timer_mode_t mode);

/****************************************************************************
 * Name: app_timer_stop
 *
 * Description:
 *   Stop a timer.
 *
 * Input parameters:
 *   timerid - The timer id.
 *
 * Returned Value:
 *   On success, a 0(OK) is returned to the caller.
 *   If an error occurs, the function will return a value of -1 (ERROR)
 *   and set errno to indicate the error.
 *
 ****************************************************************************/
int32_t app_timer_stop(timer_t timerid);

/****************************************************************************
 * Name: app_timer_delete
 *
 * Description:
 *   Delete a timer.
 *
 * Input parameters:
 *   timerid - The timer id.
 *
 * Returned Value:
 *   On success, a 0(OK) is returned to the caller.
 *   If an error occurs, the function will return a value of -1 (ERROR)
 *   and set errno to indicate the error.
 *
 ****************************************************************************/
int32_t app_timer_delete(timer_t timerid);

#endif // APP_TIMER_H__

/** @} */
