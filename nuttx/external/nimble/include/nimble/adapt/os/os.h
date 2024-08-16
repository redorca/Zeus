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

#ifndef _OS_H
#define _OS_H

#include <stdlib.h>
#include <inttypes.h>

#include <syscfg/syscfg.h>
#include <syscfg/mesh_cfg.h>

#include "nuttx/irq.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MYNEWT (0)

#ifndef min
#define min(a, b) ((a)<(b)?(a):(b))
#endif

#ifndef max
#define max(a, b) ((a)>(b)?(a):(b))
#endif

#define os_get_return_addr() (__builtin_return_address(0))

#define OS_ALIGN(__n, __a) (                             \
        (((__n) & ((__a) - 1)) == 0)                   ? \
            (__n)                                      : \
            ((__n) + ((__a) - ((__n) & ((__a) - 1))))    \
        )


#define CTASSERT(x) typedef int __ctasssert ## __LINE__[(x) ? 1 : -1]


/*GPIO debug for nimble os*/
/*This GPIO setting is nrf related because the os  is still under developing*/
/*Future all macro should nuttx related.*/
#define NIMBLE_GPIO_DEBUG (0)
#if (NIMBLE_GPIO_DEBUG)
#include <nrf52_gpio.h>
#include "chip.h"

__STATIC_INLINE void debug_gpio_init(uint32_t start,uint32_t end)
{
  for (int i=start; i<end+1; i++)
  {
    nrf_gpio_cfg_output(i);
    nrf_gpio_pin_clear(i);
  }
}

__STATIC_INLINE void debug_gpio_toggle(uint32_t pin,uint32_t cnt)
{
  for(int i=0; i<cnt; i++)
  {
    nrf_gpio_pin_set(pin);
    nrf_gpio_pin_clear(pin);
  }
}

#define DEBUG_GPIO_INIT(start,end) debug_gpio_init(start,end)
#define DEBUG_GPIO_TOGGLE(pin,cnt)  debug_gpio_toggle(pin,cnt)
#define DEBUG_GPIO_UP(pin)    nrf_gpio_pin_set(pin)
#define DEBUG_GPIO_LOW(pin)   nrf_gpio_pin_clear(pin)
#else
#define DEBUG_GPIO_INIT(start,end)
#define DEBUG_GPIO_TOGGLE(pin,cnt)
#define DEBUG_GPIO_UP(pin)
#define DEBUG_GPIO_LOW(pin)
#endif


/**
 * Whether or not the operating system has been started.  Set to
 * 1 right before first task is run.
 */
extern int g_os_started;

int os_info_init(void);

uint8_t sysinit_active;

/**
 * Returns 1 if the OS has been started, 0 if it has not yet been
 * been started.
 */
int os_started(void);


#define OS_WAIT_FOREVER (-1)

#define OS_IDLE_PRIO (0xff)
#define OS_MAIN_TASK_PRIO       MYNEWT_VAL(OS_MAIN_TASK_PRIO)
#define OS_MAIN_STACK_SIZE      MYNEWT_VAL(OS_MAIN_STACK_SIZE)

void os_init(int (*fn)(int argc, char **argv));
void os_start(void);

void hal_NVIC_DisableIRQ(IRQn_Type IRQn);
void hal_NVIC_EnableIRQ(IRQn_Type IRQn);
void hal_NVIC_SetPendingIRQ(IRQn_Type IRQn);
void hal_NVIC_ClearPendingIRQ(IRQn_Type IRQn);
void hal_NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority);
void hal_NVIC_SetVector(IRQn_Type IRQn, uint32_t vector);


/* XXX: Not sure if this should go here; I want to differentiate API that
 * should be called by application developers as those that should not. */
void os_init_idle_task(void);

#include "os/endian.h"
#include "os/os_arch.h"
#include "os/os_cfg.h"
#include "os/os_cputime.h"
#include "os/os_dev.h"
#include "os/os_error.h"
#include "os/os_heap.h"
#include "os/os_fault.h"
#include "os/os_mutex.h"
#include "os/os_sched.h"
#include "os/os_sem.h"
#include "os/os_task.h"
#include "os/os_time.h"
#include "os/os_eventq.h"
#include "os/os_callout.h"
#include "nimble/nimble_npl.h"
#include "os/os_mempool.h"
#include "os/os_mbuf.h"
#include "sysinit/sysinit.h"
#include "log/log.h"

#ifdef __cplusplus
}
#endif

#endif /* _OS_H */
