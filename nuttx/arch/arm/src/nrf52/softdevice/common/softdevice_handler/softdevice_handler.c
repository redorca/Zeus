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
#include "softdevice_handler.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "nrf_assert.h"
#include "nrf_soc.h"
//#include "core_cm7.h" /* temp solution for adjust */
#include <nuttx/arch.h>
#include "cache.h"
#include "up_internal.h"
#include "nrf_nvic.h"

#define NRF_LOG_MODULE_NAME "SDH"
#if defined(ANT_STACK_SUPPORT_REQD) && defined(BLE_STACK_SUPPORT_REQD)
#include "ant_interface.h"
#elif defined(ANT_STACK_SUPPORT_REQD)
#include "ant_interface.h"
#elif defined(BLE_STACK_SUPPORT_REQD)
#include "ble.h"
#endif

#define RAM_START_ADDRESS         0x20000000
#define SOFTDEVICE_EVT_IRQ        SD_EVT_IRQn       /**< SoftDevice Event IRQ number. Used for both protocol events and SoC events. */
//#define SOFTDEVICE_EVT_IRQHandler SD_EVT_IRQHandler
#define RAM_TOTAL_SIZE            ((NRF_FICR->INFO.RAM) * 1024)
#define RAM_END_ADDRESS           (RAM_START_ADDRESS + RAM_TOTAL_SIZE)

#define NRF_LOG_ERROR(...)                     _err(__VA_ARGS__)
#define NRF_LOG_WARNING(...)                   _warn( __VA_ARGS__)
#define NRF_LOG_DEBUG(...)                     _info( __VA_ARGS__)

#ifdef DEBUG
#define APP_ERROR_HANDLER(ERR_CODE)                                    \
      do                                                                 \
      {                                                                  \
          _err("ERR_CODE: %d, LINE: %d, FILE: %s\n", (ERR_CODE), __LINE__, (uint8_t*) __FILE__);  \
      } while (0)
#else
#define APP_ERROR_HANDLER(ERR_CODE)                                    \
      do                                                                 \
      {                                                                  \
          _err("ERR_CODE: %d\n",(ERR_CODE));                            \
          up_systemreset();                                           \
      } while (0)
#endif
/**@brief Macro for calling error handler function if supplied error code any other than NRF_SUCCESS.
 *
 * @param[in] ERR_CODE Error code supplied to the error handler.
 */
#define APP_ERROR_CHECK(ERR_CODE)                           \
      do                                                      \
      {                                                       \
          uint32_t LOCAL_ERR_CODE = (ERR_CODE);         \
          if (LOCAL_ERR_CODE != NRF_SUCCESS)                  \
          {                                                   \
              APP_ERROR_HANDLER(LOCAL_ERR_CODE);              \
          }                                                   \
      } while (0)

#define SOFTDEVICE_VS_UUID_COUNT       0
#define SOFTDEVICE_GATTS_ATTR_TAB_SIZE BLE_GATTS_ATTR_TAB_SIZE_DEFAULT
#define SOFTDEVICE_GATTS_SRV_CHANGED   0
#define SOFTDEVICE_PERIPH_CONN_COUNT   1
#define SOFTDEVICE_CENTRAL_CONN_COUNT  4
#define SOFTDEVICE_CENTRAL_SEC_COUNT   1

static void SOFTDEVICE_EVT_IRQHandler(void);

static softdevice_evt_schedule_func_t
m_evt_schedule_func;              /**< Pointer to function for propagating SoftDevice events to the scheduler. */

static volatile bool                  m_softdevice_enabled =
  false;     /**< Variable to indicate whether the SoftDevice is enabled. */
static volatile bool                  m_suspended;                      /**< Current state of the event handler. */
#ifdef BLE_STACK_SUPPORT_REQD
// The following three definitions is needed only if BLE events are needed to be pulled from the stack.
static uint8_t                       *mp_ble_evt_buffer;                /**< Buffer for receiving BLE events from the SoftDevice. */
static uint16_t                       m_ble_evt_buffer_size;            /**< Size of BLE event buffer. */
static ble_evt_handler_t              m_ble_evt_handler;                /**< Application event handler for handling BLE events. */
#endif

#ifdef ANT_STACK_SUPPORT_REQD
// The following two definitions are needed only if ANT events are needed to be pulled from the stack.
static ant_evt_t                      m_ant_evt_buffer;                 /**< Buffer for receiving ANT events from the SoftDevice. */
static ant_evt_handler_t              m_ant_evt_handler;                /**< Application event handler for handling ANT events.  */
#endif

static sys_evt_handler_t
m_sys_evt_handler;                /**< Application event handler for handling System (SOC) events.  */

static __INLINE uint32_t __REV(uint32_t value)
{
#if (__GNUC__ > 4) || (__GNUC__ == 4 && __GNUC_MINOR__ >= 5)
  return __builtin_bswap32(value);
#else
  uint32_t result;

  __ASM volatile ("rev %0, %1" : __CMSIS_GCC_OUT_REG (result) : __CMSIS_GCC_USE_REG (value) );
  return (result);
#endif
}

static __INLINE bool is_word_aligned(void const *p)
{
  return (((uintptr_t)p & 0x03) == 0);
}

/**@brief       Callback function for asserts in the SoftDevice.
 *
 * @details     A pointer to this function will be passed to the SoftDevice. This function will be
 *              called by the SoftDevice if certain unrecoverable errors occur within the
 *              application or SoftDevice.
 *
 *              See @ref nrf_fault_handler_t for more details.
 *
 * @param[in] id    Fault identifier. See @ref NRF_FAULT_IDS.
 * @param[in] pc    The program counter of the instruction that triggered the fault.
 * @param[in] info  Optional additional information regarding the fault. Refer to each fault
 *                  identifier for details.
 */
void softdevice_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
  _err("ID: %d, PC: %d, INFO: %d.\n", id, pc, info);
  up_systemreset();
}

void intern_softdevice_events_execute(void)
{
  if (!m_softdevice_enabled)
    {
      // SoftDevice not enabled. This can be possible if the SoftDevice was enabled by the
      // application without using this module's API (i.e softdevice_handler_init)

      return;
    }

  bool no_more_soc_evts = (m_sys_evt_handler == NULL);
#ifdef BLE_STACK_SUPPORT_REQD
  bool no_more_ble_evts = (m_ble_evt_handler == NULL);
#endif
#ifdef ANT_STACK_SUPPORT_REQD
  bool no_more_ant_evts = (m_ant_evt_handler == NULL);
#endif

  for (;;)
    {
      uint32_t err_code;

      if (!no_more_soc_evts)
        {
          if (m_suspended)
            {
              // Cancel pulling next event if event handler was suspended by user.
              return;
            }

          uint32_t evt_id;

          // Pull event from SOC.
          err_code = sd_evt_get(&evt_id);

          if (err_code == NRF_ERROR_NOT_FOUND)
            {
              no_more_soc_evts = true;
            }
          else if (err_code != NRF_SUCCESS)
            {
              APP_ERROR_HANDLER(err_code);
            }
          else
            {
              // Call application's SOC event handler.
#if defined(SOFTDEVICE_PRESENT)
              /*
               *nrf_drv_clock_on_soc_event(evt_id);
               */
              if (m_sys_evt_handler)
                {
                  m_sys_evt_handler(evt_id);
                }
#else
              m_sys_evt_handler(evt_id);
#endif
            }
        }

#ifdef BLE_STACK_SUPPORT_REQD
      // Fetch BLE Events.
      if (!no_more_ble_evts)
        {
          if (m_suspended)
            {
              // Cancel pulling next event if event handler was suspended by user.
              return;
            }

          // Pull event from stack
          uint16_t evt_len = m_ble_evt_buffer_size;

          err_code = sd_ble_evt_get(mp_ble_evt_buffer, &evt_len);
          if (err_code == NRF_ERROR_NOT_FOUND)
            {
              no_more_ble_evts = true;
            }
          else if (err_code != NRF_SUCCESS)
            {
              APP_ERROR_HANDLER(err_code);
            }
          else
            {
              // Call application's BLE stack event handler.
              m_ble_evt_handler((ble_evt_t *)mp_ble_evt_buffer);
            }
        }
#endif

#ifdef ANT_STACK_SUPPORT_REQD
      // Fetch ANT Events.
      if (!no_more_ant_evts)
        {
          if (m_suspended)
            {
              // Cancel pulling next event if event handler was suspended by user.
              return;
            }

          // Pull event from stack
          err_code = sd_ant_event_get(&m_ant_evt_buffer.channel,
                                      &m_ant_evt_buffer.event,
                                      m_ant_evt_buffer.msg.evt_buffer);
          if (err_code == NRF_ERROR_NOT_FOUND)
            {
              no_more_ant_evts = true;
            }
          else if (err_code != NRF_SUCCESS)
            {
              APP_ERROR_HANDLER(err_code);
            }
          else
            {
              // Call application's ANT stack event handler.
              m_ant_evt_handler(&m_ant_evt_buffer);
            }
        }
#endif

      if (no_more_soc_evts)
        {
          // There are no remaining System (SOC) events to be fetched from the SoftDevice.
#if defined(ANT_STACK_SUPPORT_REQD) && defined(BLE_STACK_SUPPORT_REQD)
          // Check if there are any remaining BLE and ANT events.
          if (no_more_ble_evts && no_more_ant_evts)
            {
              break;
            }
#elif defined(BLE_STACK_SUPPORT_REQD)
          // Check if there are any remaining BLE events.
          if (no_more_ble_evts)
            {
              break;
            }
#elif defined(ANT_STACK_SUPPORT_REQD)
          // Check if there are any remaining ANT events.
          if (no_more_ant_evts)
            {
              break;
            }
#else
          // No need to check for BLE or ANT events since there is no support for BLE and ANT
          // required.
          break;
#endif
        }
    }
}

bool softdevice_handler_is_enabled(void)
{
  return m_softdevice_enabled;
}

uint32_t softdevice_handler_init(nrf_clock_lf_cfg_t            *p_clock_lf_cfg,
                                 void                          *p_ble_evt_buffer,
                                 uint16_t                       ble_evt_buffer_size,
                                 softdevice_evt_schedule_func_t evt_schedule_func)
{
  uint32_t err_code;

  // Save configuration.
#if defined (BLE_STACK_SUPPORT_REQD)
  // Check that buffer is not NULL.
  if (p_ble_evt_buffer == NULL)
    {
      return NRF_ERROR_INVALID_PARAM;
    }

  // Check that buffer is correctly aligned.
  if (!is_word_aligned(p_ble_evt_buffer))
    {
      return NRF_ERROR_INVALID_PARAM;
    }

  mp_ble_evt_buffer     = (uint8_t *)p_ble_evt_buffer;
  m_ble_evt_buffer_size = ble_evt_buffer_size;
#else
  // The variables p_ble_evt_buffer and ble_evt_buffer_size is not needed if BLE Stack support
  // is not required.
  UNUSED_PARAMETER(p_ble_evt_buffer);
  UNUSED_PARAMETER(ble_evt_buffer_size);
#endif

  m_evt_schedule_func = evt_schedule_func;

  // Initialize SoftDevice.

#if defined(CONFIG_NRF52_RNG) && defined(SOFTDEVICE_PRESENT)
  bool rng_isr_enabled = nrf_drv_common_irq_enable_check(RNG_IRQn);
  if (rng_isr_enabled)
    {
      up_disable_irq(RNG_IRQn);
    }
#endif

//Disable interrupt before sd_softdevice_enable
  up_disable_irq(POWER_CLOCK_IRQn);
  up_disable_irq(UARTE0_UART0_IRQn);
  up_disable_irq(RNG_IRQn);
  up_disable_irq(GPIOTE_IRQn);
  up_disable_irq(RTC1_IRQn);


#if defined(S212) || defined(S332)
  err_code = sd_softdevice_enable(p_clock_lf_cfg, softdevice_fault_handler, ANT_LICENSE_KEY);
#else
  err_code = sd_softdevice_enable(p_clock_lf_cfg, softdevice_fault_handler);
#endif


//Enable irq again.
  up_enable_irq(POWER_CLOCK_IRQn);
  up_enable_irq(UARTE0_UART0_IRQn);
  up_enable_irq(RNG_IRQn);
  up_enable_irq(GPIOTE_IRQn);
  up_enable_irq(RTC1_IRQn);

  if (err_code != NRF_SUCCESS)
    {
#if defined(CONFIG_NRF52_RNG) && defined(SOFTDEVICE_PRESENT)
      if (rng_isr_enabled)
        {
          up_enable_irq(RNG_IRQn);
        }
#endif
      return err_code;
    }

  m_softdevice_enabled = true;


  // Enable BLE event interrupt (interrupt priority has already been set by the stack).
#ifdef SOFTDEVICE_PRESENT
  irq_attach(SOFTDEVICE_EVT_IRQ, (xcpt_t)SOFTDEVICE_EVT_IRQHandler, NULL);
  up_enable_irq(SOFTDEVICE_EVT_IRQ); //Enable irq

  return err_code;

#else
  //In case of Serialization NVIC must be accessed directly.
  up_enable_irq(SOFTDEVICE_EVT_IRQ);
  return NRF_SUCCESS;
#endif

}


uint32_t softdevice_handler_sd_disable(void)
{
  uint32_t err_code = sd_softdevice_disable();
  if (err_code == NRF_SUCCESS)
    {
      m_softdevice_enabled = false;

#if defined(CONFIG_NRF52_RNG) && defined(SOFTDEVICE_PRESENT)
      nrf_drv_rng_on_sd_disable();
#endif
    }
  return err_code;
}

#ifdef BLE_STACK_SUPPORT_REQD
uint32_t softdevice_ble_evt_handler_set(ble_evt_handler_t ble_evt_handler)
{
  ASSERT(ble_evt_handler != NULL);

  m_ble_evt_handler = ble_evt_handler;

  return NRF_SUCCESS;
}
#endif


#ifdef ANT_STACK_SUPPORT_REQD
uint32_t softdevice_ant_evt_handler_set(ant_evt_handler_t ant_evt_handler)
{
  ASSERT(ant_evt_handler != NULL);

  m_ant_evt_handler = ant_evt_handler;

  return NRF_SUCCESS;
}
#endif


uint32_t softdevice_sys_evt_handler_set(sys_evt_handler_t sys_evt_handler)
{
  ASSERT(sys_evt_handler != NULL);

  m_sys_evt_handler = sys_evt_handler;

  return NRF_SUCCESS;
}


/**@brief   Function for handling the Application's BLE Stack events interrupt.
 *
 * @details This function is called whenever an event is ready to be pulled.
 */
static void SOFTDEVICE_EVT_IRQHandler(void)
{

  if (m_evt_schedule_func != NULL)
    {
      uint32_t err_code = m_evt_schedule_func();
      APP_ERROR_CHECK(err_code);
    }
  else
    {
      intern_softdevice_events_execute();
    }

}

void softdevice_handler_suspend()
{
#ifdef SOFTDEVICE_PRESENT
  ret_code_t err_code = sd_nvic_DisableIRQ((IRQn_Type)SOFTDEVICE_EVT_IRQ);
  APP_ERROR_CHECK(err_code);
#else
  NVIC_DisableIRQ(SOFTDEVICE_EVT_IRQ);
#endif
  m_suspended = true;
  return;
}

void softdevice_handler_resume()
{
  if (!m_suspended)
    {
      return;
    }
  m_suspended = false;

#ifdef SOFTDEVICE_PRESENT
  ret_code_t err_code;

  // Force calling ISR again to make sure that events not pulled previously
  // has been processed.
  err_code = sd_nvic_SetPendingIRQ((IRQn_Type)SOFTDEVICE_EVT_IRQ);
  APP_ERROR_CHECK(err_code);
  err_code = sd_nvic_EnableIRQ((IRQn_Type)SOFTDEVICE_EVT_IRQ);
  APP_ERROR_CHECK(err_code);
#else
  NVIC_SetPendingIRQ((IRQn_Type)SOFTDEVICE_EVT_IRQ);
  up_enable_irq(SOFTDEVICE_EVT_IRQ);
#endif

  return;
}

bool softdevice_handler_is_suspended()
{
  return m_suspended;
}

#if defined(BLE_STACK_SUPPORT_REQD)
uint32_t softdevice_enable_get_default_config(uint8_t central_links_count,
                                              uint8_t periph_links_count,
                                              ble_enable_params_t *p_ble_enable_params)
{
  memset(p_ble_enable_params, 0, sizeof(ble_enable_params_t));
  p_ble_enable_params->common_enable_params.vs_uuid_count   = 1;
  p_ble_enable_params->gatts_enable_params.attr_tab_size    = SOFTDEVICE_GATTS_ATTR_TAB_SIZE;
  p_ble_enable_params->gatts_enable_params.service_changed  = SOFTDEVICE_GATTS_SRV_CHANGED;
  p_ble_enable_params->gap_enable_params.periph_conn_count  = periph_links_count;
  p_ble_enable_params->gap_enable_params.central_conn_count = central_links_count;
  if (p_ble_enable_params->gap_enable_params.central_conn_count != 0)
    {
      p_ble_enable_params->gap_enable_params.central_sec_count  = SOFTDEVICE_CENTRAL_SEC_COUNT;
    }

  return NRF_SUCCESS;
}


static inline uint32_t ram_total_size_get(void)
{
#ifdef NRF51
  uint32_t size_ram_blocks = (uint32_t)NRF_FICR->SIZERAMBLOCKS;
  uint32_t total_ram_size = size_ram_blocks;
  total_ram_size = total_ram_size * (NRF_FICR->NUMRAMBLOCK);
  return total_ram_size;
#elif (defined (NRF52) || defined(NRF52840_XXAA))
  return RAM_TOTAL_SIZE;
#endif /* NRF51 */
}


/*lint --e{528} -save suppress 528: symbol not referenced */
/**@brief   Function for finding the end address of the RAM.
 *
 * @retval  ram_end_address  Address of the end of the RAM.
 */
static inline uint32_t ram_end_address_get(void)
{
  uint32_t ram_end_address = (uint32_t)RAM_START_ADDRESS;
  ram_end_address += ram_total_size_get();
  return ram_end_address;
}
/*lint -restore*/

/*lint --e{10} --e{27} --e{40} --e{529} -save */
uint32_t softdevice_enable(ble_enable_params_t *p_ble_enable_params)
{
#if (defined(S130) || defined(S132) || defined(S332) || defined(S140))
  uint32_t err_code;
  uint32_t app_ram_base;

#if defined ( __CC_ARM )
  extern uint32_t Image$$RW_IRAM1$$Base;
  const volatile uint32_t ram_start = (uint32_t) &Image$$RW_IRAM1$$Base;
#elif defined ( __ICCARM__ )
  extern uint32_t __ICFEDIT_region_RAM_start__;
  volatile uint32_t ram_start = (uint32_t) &__ICFEDIT_region_RAM_start__;
#elif defined   ( __GNUC__ )
  extern uint32_t __data_start__;
  volatile uint32_t ram_start = (uint32_t) &__data_start__;
#endif

  app_ram_base = ram_start;
  NRF_LOG_DEBUG("sd_ble_enable: RAM start at 0x%x\r\n",
                app_ram_base);
  NRF_LOG_DEBUG("softdevice version: %s\r\n", CONFIG_NRF_SOFTDEVICE);

  err_code = sd_ble_enable(p_ble_enable_params, &app_ram_base);

  if (app_ram_base != ram_start)
    {
      NRF_LOG_WARNING("sd_ble_enable: RAM start should be adjusted to 0x%x\r\n",
                      app_ram_base);
      NRF_LOG_WARNING("RAM size should be adjusted to 0x%x \r\n",
                      ram_end_address_get() - app_ram_base);
    }
  else if (err_code != NRF_SUCCESS)
    {
      NRF_LOG_ERROR("sd_ble_enable: error 0x%x\r\n", err_code);
    }
  return err_code;
#else
  return NRF_SUCCESS;
#endif   //defined(S130) || defined(S132) || defined(S332) || defined(S140)
}
/*lint -restore*/

#endif //BLE_STACK_SUPPORT_REQD
