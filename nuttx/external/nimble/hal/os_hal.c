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

#include <nuttx/config.h>
#include <stdint.h>
#include <assert.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include "os/os.h"

/**
 * @brief This section contains documentation Convert Nuttx OS api to mynewt OS api for nimble in Nuttx.
 * 
 */


/* IRQ offset for OS */

/*The extern interrupt register start with 0xe000e000, bit 0 means interrupt #16.*/
#define ARMV7M_NVIC_BASE                0xe000e000
#define NVIC_IRQ_PEND_OFFSET(n)         (0x0200 + 4*((n) >> 5))
#define NVIC_IRQ_PEND(n)                (ARMV7M_NVIC_BASE + NVIC_IRQ_PEND_OFFSET(n))

#define NVIC_IRQ_CLRPEND_OFFSET(n)      (0x0280 + 4*((n) >> 5))
#define NVIC_IRQ_CLRPEND(n)             (ARMV7M_NVIC_BASE + NVIC_IRQ_CLRPEND_OFFSET(n))

extern void os_msys_init(void);

extern void ble_hci_ram_init(void);
extern void log_init(void);
extern void ble_hs_init(void);
extern void ble_ll_init(void);
extern void ble_svc_gap_init(void);
extern void ble_svc_gatt_init(void);
extern void ble_svc_ans_init(void);
extern void ble_store_ram_init(void);
extern void ble_store_config_init(void);
extern void newtmgr_ble_pkg_init(void);
extern void os_time_init(void);

extern struct os_callout_list g_callout_list;
extern struct os_task g_ble_hs_task;
extern struct os_task g_ble_ll_task;
extern struct os_task os_callout_task_handle;

/*
 * Initializes systick for Nimble; Nimble only support 32768 clock.
 */
void
os_bsp_systick_init(void)
{
    uint32_t mask;

    /* Turn on the LFCLK */
    NRF_CLOCK->TASKS_LFCLKSTOP = 1;
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->LFCLKSRC = (1UL);
    NRF_CLOCK->TASKS_LFCLKSTART = 1;

    /* Wait here till started! */
    mask = (0x1UL << (16UL)) | (1UL);
    while (1) {
        if (NRF_CLOCK->EVENTS_LFCLKSTARTED) {
            if ((NRF_CLOCK->LFCLKSTAT & mask) == mask) {
                break;
            }
        }
    }    
}

/*
 * Initializes BSP; registers flash_map with the system.
 */
void
os_bsp_init(void)
{
  int rc;
  
  (void)rc;

  os_bsp_systick_init();

#if MYNEWT_VAL(TIMER_5)
  rc = hal_timer_init(5, NULL);
  assert(rc == 0);
#endif
  
#if (MYNEWT_VAL(OS_CPUTIME_TIMER_NUM) >= 0)
  rc = os_cputime_init(MYNEWT_VAL(OS_CPUTIME_FREQ));
  assert(rc == 0);
#endif

  os_time_init();
}

void nimble_init(void)
{

  /* ble_hci_ram_init (net/nimble/transport/ram) */
  ble_hci_ram_init();

  /* ble_hs_init (net/nimble/host) */
  ble_hs_init();

  /* ble_ll_init (net/nimble/controller) */
  ble_ll_init();

  /* ble_store_ram_init (net/nimble/host/store/ram) */
  //ble_store_ram_init();
  ble_store_config_init();

}

void
nimble_sysinit_start(void)
{
    sysinit_active = 1;
}

void
nimble_sysinit_end(void)
{
    sysinit_active = 0;
}

/*
 * Initialize the hal layer module.
 */
void
nimble_hal_init(void)
{
  os_msys_init();
  os_bsp_init();
  TAILQ_INIT(&g_callout_list);
  os_eventq_init(os_eventq_dflt_get());
  nimble_init();
}

void
nimble_hal_destory(void)
{
  hal_NVIC_DisableIRQ(RTC0_IRQn);
  hal_NVIC_DisableIRQ(OS_TICK_IRQ);
  hal_NVIC_DisableIRQ(RNG_IRQn);
  hal_NVIC_DisableIRQ(RADIO_IRQn);
  hal_timer_deinit(5);
  
  /*End the host task.*/
  pthread_cancel(g_ble_hs_task.thread);
  /*End the controller task.*/
  pthread_cancel(g_ble_ll_task.thread);
  /*End the callout task.*/
  pthread_cancel(os_callout_task_handle.thread);

}

/**
 * Convert disable External Interrupt API to nuttx.
 * Disables a device-specific interrupt in the NVIC interrupt controller.
 *
 * @param IRQn               External interrupt number. Value cannot be negative.
 *
 * @return                   NONE.
 */
void
hal_NVIC_DisableIRQ(IRQn_Type IRQn)
{
    up_disable_irq(IRQn);
}

/**
 * Convert enable External Interrupt API to nuttx.
 * Enables a device-specific interrupt in the NVIC interrupt controller.
 *
 * @param IRQn               External interrupt number. Value cannot be negative.
 *
 * @return                   NONE.
 */
void
hal_NVIC_EnableIRQ(IRQn_Type IRQn)
{
    up_enable_irq(IRQn);
}

/**
 * Convert set Pending Interrupt API to nuttx.
 * Sets the pending bit of an external interrupt.
 *
 * @param IRQn               External interrupt number. Value cannot be negative.
 *
 * @return                   NONE.
 */
void
hal_NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
    /*Internal OS will use the 0~15 IRQ number, this IRQn is extern INT number,
    so this IRQn number should no less than 16.*/
    uint32_t external_irq ;
    if(IRQn < NRF52_IRQ_EXTINT)
    {
        assert(0);
    }
    external_irq = IRQn - NRF52_IRQ_EXTINT;
    *(volatile uint32_t *)NVIC_IRQ_PEND(external_irq) = (uint32_t)(1UL << (external_irq & 0x1FUL));
}

/**
 * Convert Clear Pending Interrupt API to nuttx.
 * Sets the pending bit of an external interrupt.
 *
 * @param IRQn               External interrupt number. Value cannot be negative.
 *
 * @return                   NONE.
 */
void
hal_NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
    /*Internal OS will use the 0~15 IRQ number, this IRQn is extern INT number,
    so this IRQn number should no less than 16.*/
    uint32_t external_irq ;
    if(IRQn < NRF52_IRQ_EXTINT)
    {
        assert(0);
    }
    external_irq = IRQn - NRF52_IRQ_EXTINT;
    *(volatile uint32_t *)NVIC_IRQ_CLRPEND(external_irq) = (uint32_t)(1UL << (external_irq & 0x1FUL));
}

/**
 * Convert set Interrupt Priority API to nuttx.
 * Sets the Interrupt Priority.
 *
 * @param IRQn               External interrupt number. Value cannot be negative.
 * @param priority           Priority of the interrupt number.
 *
 * @return                   NONE.
 */
void
hal_NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
#ifdef CONFIG_ARCH_IRQPRIO
    up_prioritize_irq(IRQn, priority);
#else
   /*
    * In right way, we need enable CONFIG_ARCH_IRQPRIO option for priority irq,
    * But current nuttx os have a critical issue,if irq prioriyt is different,
    * it will case unexpected context switch, lead to os schedule under disorder status.
    * so we will disable this feature untill we find the root cause of this issue.
    *
    *  #info "You should enable CONFIG_ARCH_IRQPRIO option for priority irq"
    */
    (void)IRQn;
    (void)priority;
#endif
}

/**
 * Convert set vector API to nuttx.
 * Set the vector of vector table.
 *
 * @param IRQn               External interrupt number. Value cannot be negative.
 * @param IRQn               IRQ handler address of the IRQn.
 *
 * @return                   NONE.
 */
void
hal_NVIC_SetVector(IRQn_Type IRQn, uint32_t vector)
{
    irq_attach(IRQn, (xcpt_t)vector, 0);
}


