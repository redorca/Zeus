/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */
#include "syscfg/syscfg.h"

#include <assert.h>
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include "os/os.h"
#include "os/queue.h"
#include "nrf_rtc.h"
#include "chip.h"

/**
 * @addtogroup OSKernel
 * @{
 *   @defgroup OSTime Time
 *   @{
 */

CTASSERT(sizeof(os_time_t) == 4);

#define OS_USEC_PER_TICK   (1000*1000 / OS_TICKS_PER_SEC)
#define OS_TICK_PRIORITY   (NVIC_SYSH_PRIORITY_DEFAULT)

#define BLE_OS_CALLOUT_STACK_SIZE   (1024)

os_time_t g_os_time;

static sem_t os_callout_sem;

static NRF_RTC_Type *g_os_tick = NRF_RTC1;

struct os_task os_callout_task_handle;
/*
 * Time-of-day collateral.
 */
static struct {
    os_time_t ostime;
    struct os_timeval uptime;
    struct os_timeval utctime;
    struct os_timezone timezone;
} basetod;

static void
os_deltatime(os_time_t delta, const struct os_timeval *base,
    struct os_timeval *result)
{
    struct os_timeval tvdelta;

    tvdelta.tv_sec = delta / OS_TICKS_PER_SEC;
    tvdelta.tv_usec = (delta % OS_TICKS_PER_SEC) * OS_USEC_PER_TICK;
    os_timeradd(base, &tvdelta, result);
}

/**
 * Get the current OS time in ticks
 *
 * @return OS time in ticks
 */
os_time_t
os_time_get(void)
{
    return (g_os_time);
}

#if MYNEWT_VAL(OS_SCHEDULING)
static void
os_callout_task(void *arg)
{
  while(1) {
    sem_wait(&os_callout_sem);
    os_callout_tick();
  }

  return;
}

static int os_tick_event_irq(int irq, FAR void *context, FAR void *arg)
{
  /* clear event firstly , then call os_tick event */
  nrf_rtc_event_clear(g_os_tick, NRF_RTC_EVENT_TICK);

  os_time_advance(1);

  return 0;
}

static void os_tick_init(void)
{
  uint32_t prescale;

  /* caculate prescale according to os_tick */
  prescale = 32768 / OS_TICKS_PER_SEC - 1;
  nrf_rtc_prescaler_set(g_os_tick, prescale);

  /* enable tick event inerrupt */
  nrf_rtc_event_clear(g_os_tick, NRF_RTC_EVENT_TICK);
  nrf_rtc_event_enable(g_os_tick, NRF_RTC_INT_TICK_MASK);
  nrf_rtc_int_enable(g_os_tick, NRF_RTC_INT_TICK_MASK);

  /* enable os_tick irq and module */
  hal_NVIC_SetVector(OS_TICK_IRQ, (uint32_t)os_tick_event_irq);
  hal_NVIC_SetPriority(OS_TICK_IRQ, OS_TICK_PRIORITY);
  hal_NVIC_EnableIRQ(OS_TICK_IRQ);

  nrf_rtc_task_trigger(g_os_tick, NRF_RTC_TASK_START);
}

void
os_time_init(void)
{
  sem_init(&os_callout_sem, 0, 0);

  /* Initialize the host task */
  os_task_init(&os_callout_task_handle, "ble_os_callout", os_callout_task, NULL,
               MYNEWT_VAL(BLE_OS_CALLOUT), OS_WAIT_FOREVER, NULL,
               BLE_OS_CALLOUT_STACK_SIZE);

  os_tick_init();

}

static void
os_time_tick(int ticks)
{
    os_sr_t sr;
    os_time_t delta, prev_os_time;

    assert(ticks >= 0);

    OS_ENTER_CRITICAL(sr);
    prev_os_time = g_os_time;
    g_os_time += ticks;

    /*
     * Update 'basetod' when 'g_os_time' crosses the 0x00000000 and
     * 0x80000000 thresholds.
     */
    if ((prev_os_time ^ g_os_time) >> 31) {
        delta = g_os_time - basetod.ostime;
        os_deltatime(delta, &basetod.uptime, &basetod.uptime);
        os_deltatime(delta, &basetod.utctime, &basetod.utctime);
        basetod.ostime = g_os_time;
    }
    OS_EXIT_CRITICAL(sr);
}

/**
 * Move OS time forward ticks.
 *
 * @param ticks The number of ticks to move time forward.
 */
void
os_time_advance(uint32_t ticks)
{
    assert(ticks >= 0);

    if (ticks > 0) {
        if (!os_started()) {
            g_os_time += ticks;
        } else {
            os_time_tick(ticks);
            /* This timet advance will run in INT, put the callout task out fo INT.
             * os_callout_tick();
             */
            sem_post(&os_callout_sem);
        }
    }
}
#else
void
os_time_advance(uint32_t ticks)
{
    g_os_time += ticks;
}
#endif

/**
 * Puts the current task to sleep for the specified number of os ticks. There
 * is no delay if ticks is <= 0.
 *
 * @param osticks Number of ticks to delay (<= 0 means no delay).
 */
void
os_time_delay(uint32_t osticks)
{
    if (osticks > 0) {

        /* First convert timeout tick into nuttx time unit */

        useconds_t timeout = osticks*1000*1000/OS_TICKS_PER_SEC;

        usleep(timeout);
    }
}

/**
 * Set the time of day.  This does not modify os time, but rather just modifies
 * the offset by which we are tracking real time against os time.
 *
 * @param utctime A timeval representing the UTC time we are setting
 * @param tz The time-zone to apply against the utctime being set.
 *
 * @return 0 on success, non-zero on failure.
 */
int
os_settimeofday(struct os_timeval *utctime, struct os_timezone *tz)
{
    os_sr_t sr;
    os_time_t delta;

    OS_ENTER_CRITICAL(sr);
    if (utctime != NULL) {
        /*
         * Update all time-of-day base values.
         */
        delta = os_time_get() - basetod.ostime;
        os_deltatime(delta, &basetod.uptime, &basetod.uptime);
        basetod.utctime = *utctime;
        basetod.ostime += delta;
    }

    if (tz != NULL) {
        basetod.timezone = *tz;
    }
    OS_EXIT_CRITICAL(sr);

    return (0);
}

/**
 * Get the current time of day.  Returns the time of day in UTC
 * into the tv argument, and returns the timezone (if set) into
 * tz.
 *
 * @param tv The structure to put the UTC time of day into
 * @param tz The structure to put the timezone information into
 *
 * @return 0 on success, non-zero on failure
 */
int
os_gettimeofday(struct os_timeval *tv, struct os_timezone *tz)
{
    os_sr_t sr;
    os_time_t delta;

    OS_ENTER_CRITICAL(sr);
    if (tv != NULL) {
        delta = os_time_get() - basetod.ostime;
        os_deltatime(delta, &basetod.utctime, tv);
    }

    if (tz != NULL) {
        *tz = basetod.timezone;
    }
    OS_EXIT_CRITICAL(sr);

    return (0);
}

/**
 * Get time since boot in microseconds.
 *
 * @return time since boot in microseconds
 */
int64_t
os_get_uptime_usec(void)
{
  struct os_timeval tv;
  os_time_t delta;
  os_sr_t sr;
  os_time_t ostime;


  OS_ENTER_CRITICAL(sr);
  tv = basetod.uptime;
  ostime = basetod.ostime;
  delta = os_time_get() - ostime;
  OS_EXIT_CRITICAL(sr);

  os_deltatime(delta, &tv, &tv);

  return(tv.tv_sec * 1000000 + tv.tv_usec);
}

/**
 * Converts milliseconds to OS ticks.
 *
 * @param ms                    The milliseconds input.
 * @param out_ticks             The OS ticks output.
 *
 * @return                      0 on success; OS_EINVAL if the result is too
 *                                  large to fit in a uint32_t.
 */
int
os_time_ms_to_ticks(uint32_t ms, uint32_t *out_ticks)
{
    uint64_t ticks;

#if OS_TICKS_PER_SEC == 1000
    *out_ticks = ms;
    return 0;
#endif

    _Static_assert(OS_TICKS_PER_SEC <= UINT32_MAX,
                   "OS_TICKS_PER_SEC must be <= UINT32_MAX");

    ticks = (uint64_t)ms * OS_TICKS_PER_SEC / 1000;
    if (ticks > UINT32_MAX) {
        return OS_EINVAL;
    }

    *out_ticks = (uint32_t)ticks;
    return 0;
}

int
os_time_ticks_to_ms(uint32_t ticks, uint32_t *out_ms)
{
    uint64_t ms;

#if OS_TICKS_PER_SEC == 1000
    *out_ms = ticks;
    return 0;
#endif

    ms = ((uint64_t)ticks * 1000) / OS_TICKS_PER_SEC;
    if (ms > UINT32_MAX) {
        return OS_EINVAL;
    }

    *out_ms = ms;

    return 0;
}

uint32_t
os_time_ms_to_ticks32(uint32_t ms)
{
#if OS_TICKS_PER_SEC == 1000
    return ms;
#else
    return ((uint64_t)ms * OS_TICKS_PER_SEC) / 1000;
#endif
}

uint32_t
os_time_ticks_to_ms32(uint32_t ticks)
{
#if OS_TICKS_PER_SEC == 1000
    return ticks;
#else
    return ((uint64_t)ticks * 1000) / OS_TICKS_PER_SEC;
#endif
}
