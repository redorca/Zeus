/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
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

/**@file
 *
 * @defgroup app_util_platform Utility Functions and Definitions (Platform)
 * @{
 * @ingroup app_common
 *
 * @brief Various types and definitions available to all applications when using SoftDevice.
 */

#ifndef APP_UTIL_PLATFORM_H__
#define APP_UTIL_PLATFORM_H__

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <stdint.h>
#include "compiler_abstraction.h"
#include "nrf.h"
#include "app_util.h"
#ifdef SOFTDEVICE_PRESENT
#include "nrf_soc.h"
#include "nrf_nvic.h"
#endif
#include "nrf_assert.h"
#include "app_error.h"


#ifdef __cplusplus
extern "C" {
#endif

#if __CORTEX_M == (0x00U)
#define _PRIO_SD_HIGH       0
#define _PRIO_APP_HIGH      1
#define _PRIO_APP_MID       1
#define _PRIO_SD_LOW        2
#define _PRIO_APP_LOW       3
#define _PRIO_APP_LOWEST    3
#define _PRIO_THREAD        4
#elif __CORTEX_M == (0x04U)
#define _PRIO_SD_HIGH       0
#define _PRIO_SD_MID        1
#define _PRIO_APP_HIGH      2
#define _PRIO_APP_MID       3
#define _PRIO_SD_LOW        4
#define _PRIO_SD_LOWEST     5
#define _PRIO_APP_LOW       6
#define _PRIO_APP_LOWEST    7
#define _PRIO_THREAD        15
#else
#error "No platform defined"
#endif


//lint -save -e113 -e452
/**@brief The interrupt priorities available to the application while the SoftDevice is active. */
typedef enum
{
#ifndef SOFTDEVICE_PRESENT
  APP_IRQ_PRIORITY_HIGHEST = _PRIO_SD_HIGH,
#else
  APP_IRQ_PRIORITY_HIGHEST = _PRIO_APP_HIGH,
#endif
  APP_IRQ_PRIORITY_HIGH    = _PRIO_APP_HIGH,
#ifndef SOFTDEVICE_PRESENT
  APP_IRQ_PRIORITY_MID     = _PRIO_SD_LOW,
#else
  APP_IRQ_PRIORITY_MID     = _PRIO_APP_MID,
#endif
  APP_IRQ_PRIORITY_LOW     = _PRIO_APP_LOW,
  APP_IRQ_PRIORITY_LOWEST  = _PRIO_APP_LOWEST,
  APP_IRQ_PRIORITY_THREAD  = _PRIO_THREAD     /**< "Interrupt level" when running in Thread Mode. */
} app_irq_priority_t;
//lint -restore


/*@brief The privilege levels available to applications in Thread Mode */
typedef enum
{
  APP_LEVEL_UNPRIVILEGED,
  APP_LEVEL_PRIVILEGED
} app_level_t;

/**@cond NO_DOXYGEN */
#define EXTERNAL_INT_VECTOR_OFFSET 16
/**@endcond */

/**@brief Macro for setting a breakpoint.
 */
#if defined(__GNUC__)
#define NRF_BREAKPOINT __builtin_trap()
#else
#define NRF_BREAKPOINT __BKPT(0)
#endif

/** @brief Macro for setting a breakpoint.
 *
 * If it is possible to detect debugger presence then it is set only in that case.
 *
 */
#if __CORTEX_M == 0x04
#define NRF_BREAKPOINT_COND do {                            \
    /* C_DEBUGEN == 1 -> Debugger Connected */              \
    if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk)   \
    {                                                       \
       /* Generate breakpoint if debugger is connected */   \
            NRF_BREAKPOINT;                                 \
    } \
    }while (0)
#else
#define NRF_BREAKPOINT_COND NRF_BREAKPOINT
#endif // __CORTEX_M == 0x04

#if defined ( __CC_ARM )
#define PACKED(TYPE) __packed TYPE
#define PACKED_STRUCT PACKED(struct)
#elif defined   ( __GNUC__ )
#define PACKED __attribute__((packed))
#define PACKED_STRUCT struct PACKED
#endif

void app_util_critical_region_enter (uint8_t *p_nested);
void app_util_critical_region_exit (uint8_t nested);

/**@brief Macro for entering a critical region.
 *
 * @note Due to implementation details, there must exist one and only one call to
 *       CRITICAL_REGION_EXIT() for each call to CRITICAL_REGION_ENTER(), and they must be located
 *       in the same scope.
 */
#ifdef SOFTDEVICE_PRESENT
#define CRITICAL_REGION_ENTER()                                                             \
    {                                                                                       \
        uint8_t __CR_NESTED = 0;                                                            \
        app_util_critical_region_enter(&__CR_NESTED);
#else
#define CRITICAL_REGION_ENTER()             \
  {                                         \
    irqstate_t flags;                       \
    flags = enter_critical_section();

#endif

/**@brief Macro for leaving a critical region.
 *
 * @note Due to implementation details, there must exist one and only one call to
 *       CRITICAL_REGION_EXIT() for each call to CRITICAL_REGION_ENTER(), and they must be located
 *       in the same scope.
 */
#ifdef SOFTDEVICE_PRESENT
#define CRITICAL_REGION_EXIT()                                                              \
        app_util_critical_region_exit(__CR_NESTED);                                         \
    }
#else
#define CRITICAL_REGION_EXIT()             \
    leave_critical_section(flags);         \
  }
#endif

/* Workaround for Keil 4 */
#ifndef IPSR_ISR_Msk
#define IPSR_ISR_Msk                       (0x1FFUL /*<< IPSR_ISR_Pos*/)                  /*!< IPSR: ISR Mask */
#endif



/**@brief Macro to enable anonymous unions from a certain point in the code.
 */
#if defined(__CC_ARM)
#define ANON_UNIONS_ENABLE _Pragma("push") \
                               _Pragma("anon_unions")
#elif defined(__ICCARM__)
#define ANON_UNIONS_ENABLE _Pragma("language=extended")
#else
#define ANON_UNIONS_ENABLE
// No action will be taken.
// For GCC anonymous unions are enabled by default.
#endif

/**@brief Macro to disable anonymous unions from a certain point in the code.
 * @note Call only after first calling @ref ANON_UNIONS_ENABLE.
 */
#if defined(__CC_ARM)
#define ANON_UNIONS_DISABLE _Pragma("pop")
#elif defined(__ICCARM__)
#define ANON_UNIONS_DISABLE
// for IAR leave anonymous unions enabled
#else
#define ANON_UNIONS_DISABLE
// No action will be taken.
// For GCC anonymous unions are enabled by default.
#endif

/* Workaround for Keil 4 */
#ifndef CONTROL_nPRIV_Msk
#define CONTROL_nPRIV_Msk                  (1UL /*<< CONTROL_nPRIV_Pos*/)                 /*!< CONTROL: nPRIV Mask */
#endif


/**@brief Function for finding the current interrupt level.
 *
 * @return   Current interrupt level.
 * @retval   APP_IRQ_PRIORITY_HIGH    We are running in Application High interrupt level.
 * @retval   APP_IRQ_PRIORITY_LOW     We are running in Application Low interrupt level.
 * @retval   APP_IRQ_PRIORITY_THREAD  We are running in Thread Mode.
 */
uint8_t current_int_priority_get(void);


/**@brief Function for finding out the current privilege level.
 *
 * @return   Current privilege level.
 * @retval   APP_LEVEL_UNPRIVILEGED    We are running in unprivileged level.
 * @retval   APP_LEVEL_PRIVILEGED    We are running in privileged level.
 */
uint8_t privilege_level_get(void);


#ifdef __cplusplus
}
#endif

#endif // APP_UTIL_PLATFORM_H__

/** @} */
