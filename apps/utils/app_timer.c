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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <semaphore.h>
#include <sched.h>
#include <utils/app_timer.h>
#include "nrf.h"


/****************************************************************************
 * Private Definitions
 ****************************************************************************/
#define SIGVALUE_INT  42
#define TOTAL_SIGNUM 32

pthread_mutex_t mut_sig_en = PTHREAD_MUTEX_INITIALIZER;
struct DataItem
{
  timer_t timerid;
  int signalno;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/
volatile uint32_t sig_en = 0; //sig_en contains 32 bits to keep tracking the available signals
struct DataItem *hashArray[TOTAL_SIGNUM];

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static int search(timer_t timerid)
{
  int hashIndex = 0;

  while (hashArray[hashIndex] != NULL)
    {

      if (hashArray[hashIndex]->timerid == timerid)
        {
          return hashArray[hashIndex]->signalno;
        }

      hashIndex++;
    }
  return -1;
}

static void insert(timer_t timerid, int signalno)
{
  int hashIndex = signalno;

  struct DataItem *item = (struct DataItem *) malloc(sizeof(struct DataItem));
  item->timerid = timerid;
  item->signalno = signalno;

  hashArray[hashIndex] = item;
}

/**@brief Function for adding timer id and signal number as a pair to the list .
 */
static void addsigtolist(int32_t signal_no, timer_t *timerid)
{
  insert(*timerid, signal_no);
  sig_en = sig_en | (1 << signal_no);
}

/**@brief Function for removing timer id and signal number as a pair from the list .
 */
static int32_t removesigfromlist(timer_t timerid)
{
  int status;
  status = pthread_mutex_lock(&mut_sig_en);
  if (status != 0)
    {
      _err("pthread_mutex_lock failed\n");
    }
  if (timerid != NULL)
    {
      int32_t signal_no;
      signal_no = search(timerid);
      sig_en = sig_en & (~(1 << signal_no));
      return OK;
    }
  else
    {
      _err("Not able to remove from list");
      return -1;
    }
  status = pthread_mutex_unlock(&mut_sig_en);
  if (status != 0)
    {
      _err("pthread_mutex_unlock failed\n");
    }
}

/**@brief Function for finding available signals.
 *
 * @details There are 32 signals(0~31) for application usage. sig_en contains 32 bits to keep tracking the available signals.
 */
static int32_t findavailablesig(void)
{
  int temp;
  for (int i = 0; i < TOTAL_SIGNUM; i++)
    {
      temp = (sig_en >> i) & 1;
      if (temp == 0)
        {
          return i;
        }
    }
  return -1;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/**@brief Function for creating timer and the corresponding callback function.
 */
int32_t app_timer_create(timer_t *timerid, void *timer_cb)
{
  struct sigaction   act;
  struct sigaction   oact;
  struct sigevent notify;
  int32_t status;
  int32_t signal_no;

  status = pthread_mutex_lock(&mut_sig_en);
  if (status != 0)
    {
      _err("pthread_mutex_lock failed");
    }

  signal_no = findavailablesig();
  if (signal_no < 0)
    {
      _err("There is no available signal\r\n");
      return -1;
    }

  act.sa_sigaction = timer_cb;
  act.sa_flags  = SA_SIGINFO;

  (void)sigfillset(&act.sa_mask);
  (void)sigdelset(&act.sa_mask, signal_no);

  status = sigaction(signal_no, &act, &oact);
  if (status != OK)
    {
      _err("timer_test: ERROR sigaction failed, status=%d\n", status);
    }

  notify.sigev_notify            = SIGEV_SIGNAL;
  notify.sigev_signo             = signal_no;
  notify.sigev_value.sival_int   = SIGVALUE_INT;
#ifdef CONFIG_SIG_EVTHREAD
  notify.sigev_notify_function   = NULL;
  notify.sigev_notify_attributes = NULL;
#endif
  status = timer_create(CLOCK_REALTIME, &notify, timerid);
  if (status != OK)
    {
      _err("sigev_thread_test: timer_create failed, errno=%d\n", errno);
    }

  addsigtolist(signal_no, timerid);
  status = pthread_mutex_unlock(&mut_sig_en);
  if (status != 0)
    {
      _err("pthread_mutex_lock failed");
    }
  return status;
}

/**@brief Function for starting timer.
 *
 * @details There are 32 signals(0~31) for application usage. sig_en contains 32 bits to keep tracking the available signals.
 * *
 * @param timerid: timer id
 *        msec: timer interval measured in millisecond
 *        mode: choose from  APP_TIMER_MODE_SINGLE_SHOT or APP_TIMER_MODE_REPEATED
 */
int32_t app_timer_start(timer_t timerid, int msec, app_timer_mode_t mode)
{
  struct itimerspec timer;
  int status;
  if (msec == 0)
    {
      _err("Set timer inverval 0. It means timer stops");
    }
  timer.it_value.tv_sec     = msec / 1000;
  timer.it_value.tv_nsec    = (msec % 1000) * 1000000;
  timer.it_interval.tv_sec = (mode == APP_TIMER_MODE_REPEATED) ? (msec / 1000) : 0;
  timer.it_interval.tv_nsec = (mode == APP_TIMER_MODE_REPEATED) ? (msec % 1000) * 1000000 : 0;

  status = timer_settime(timerid, 0, &timer, NULL);
  if (status != OK)
    {
      _err("timer_test: timer_settime failed, errno=%d\n", errno);
    }
  return status;
}

/**@brief Function for stopping timer.
 */
int32_t app_timer_stop(timer_t timerid)
{
  struct itimerspec timer;
  int status;
  timer.it_value.tv_sec     = 0;
  timer.it_value.tv_nsec    = 0;
  timer.it_interval.tv_sec = 0;
  timer.it_interval.tv_nsec = 0;

  status = timer_settime(timerid, 0, &timer, NULL);
  if (status != OK)
    {
      _err("timer_test: timer_settime failed, errno=%d\n", errno);
    }
  return status;
}

/**@brief Function for deleting timer.
 */
int32_t app_timer_delete(timer_t timerid)
{
  int32_t status;
  status = removesigfromlist(timerid);
  if (status != OK)
    {
      _err("Unable to delete signal");
      return status;
    }
  status = timer_delete(timerid);

  if (status != OK)
    {
      _err("sigev_thread_test: timer_settime failed, errno=%d\n", errno);
    }

  return status;
}

