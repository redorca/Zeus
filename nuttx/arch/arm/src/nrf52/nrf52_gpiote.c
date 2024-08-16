/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
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
#include <string.h>
#include <stddef.h>
#include <errno.h>
#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/semaphore.h>
#include "chip.h"
#include "nrf_error.h"
#include "nrf52_gpiote.h"
#include "nrf52_gpio.h"
#include "nrf_assert.h"

#ifdef CONFIG_NRF52_GPIOTE

#define FORBIDDEN_HANDLER_ADDRESS ((nrf_drv_gpiote_evt_handler_t)UINT32_MAX)
#define PIN_NOT_USED              (-1)
#define PIN_USED                  (-2)
#define NO_CHANNELS               (-1)
#define SENSE_FIELD_POS           (6)
#define SENSE_FIELD_MASK          (0xC0)

#define GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS 4
#define GPIOTE_CONFIG_IRQ_PRIORITY 7

/**
 * @brief Macro for converting task-event index to an address of an event register.
 *
 * Macro utilizes the fact that registers are grouped together in ascending order.
 */
#define TE_IDX_TO_EVENT_ADDR(idx)    (nrf_gpiote_events_t)((uint32_t)NRF_GPIOTE_EVENTS_IN_0 + \
                                                           (sizeof(uint32_t) * (idx)))

/**
 * @brief Macro for converting task-event index of OUT task to an address of a task register.
 *
 * Macro utilizes the fact that registers are grouped together in ascending order.
 */
#define TE_OUT_IDX_TO_TASK_ADDR(idx) (nrf_gpiote_tasks_t)((uint32_t)NRF_GPIOTE_TASKS_OUT_0 + \
                                                          (sizeof(uint32_t) * (idx)))

#if defined(GPIOTE_FEATURE_SET_PRESENT) || defined(__SDK_DOXYGEN__)
/**
 * @brief Macro for converting task-event index of SET task to an address of a task register.
 *
 * Macro utilizes the fact that registers are grouped together in ascending order.
 */
#define TE_SET_IDX_TO_TASK_ADDR(idx) (nrf_gpiote_tasks_t)((uint32_t)NRF_GPIOTE_TASKS_SET_0 + \
                                                          (sizeof(uint32_t) * (idx)))

#endif // defined(GPIOTE_FEATURE_SET_PRESENT) || defined(__SDK_DOXYGEN__)

#if defined(GPIOTE_FEATURE_CLR_PRESENT) || defined(__SDK_DOXYGEN__)
/**
 * @brief Macro for converting task-event index of CLR task to an address of a task register.
 *
 * Macro utilizes the fact that registers are grouped together in ascending order.
 */
#define TE_CLR_IDX_TO_TASK_ADDR(idx) (nrf_gpiote_tasks_t)((uint32_t)NRF_GPIOTE_TASKS_CLR_0 + \
                                                          (sizeof(uint32_t) * (idx)))

#endif // defined(GPIOTE_FEATURE_CLR_PRESENT) || defined(__SDK_DOXYGEN__)

/*lint -save -e661*/

/****************************************************************************
 * Private Data
 ****************************************************************************/
typedef struct
{
  nrf_drv_gpiote_evt_handler_t handlers[GPIOTE_CH_NUM + GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS];
  int8_t                       pin_assignments[NUMBER_OF_PINS];
  int8_t                       port_handlers_pins[GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS];
  nrfx_drv_state_t              state;
} nrf52_gpiote_dev_s;

static nrf52_gpiote_dev_s g_gpiotedev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static inline bool pin_in_use(uint32_t pin)
{
  return (g_gpiotedev.pin_assignments[pin] != PIN_NOT_USED);
}


static inline bool pin_in_use_as_non_task_out(uint32_t pin)
{
  return (g_gpiotedev.pin_assignments[pin] == PIN_USED);
}


static inline bool pin_in_use_by_te(uint32_t pin)
{
  return (g_gpiotedev.pin_assignments[pin] >= 0 && g_gpiotedev.pin_assignments[pin] <
          GPIOTE_CH_NUM) ? true : false;
}


static inline bool pin_in_use_by_port(uint32_t pin)
{
  return (g_gpiotedev.pin_assignments[pin] >= GPIOTE_CH_NUM);
}


static inline bool pin_in_use_by_gpiote(uint32_t pin)
{
  return (g_gpiotedev.pin_assignments[pin] >= 0);
}


static inline void pin_in_use_by_te_set(uint32_t                     pin,
                                        uint32_t                     channel_id,
                                        nrf_drv_gpiote_evt_handler_t handler,
                                        bool                         is_channel)
{
  g_gpiotedev.pin_assignments[pin] = channel_id;
  g_gpiotedev.handlers[channel_id] = handler;
  if (!is_channel)
    {
      g_gpiotedev.port_handlers_pins[channel_id - GPIOTE_CH_NUM] = (int8_t)pin;
    }
}


static inline void pin_in_use_set(uint32_t pin)
{
  g_gpiotedev.pin_assignments[pin] = PIN_USED;
}


static inline void pin_in_use_clear(uint32_t pin)
{
  g_gpiotedev.pin_assignments[pin] = PIN_NOT_USED;
}


static inline int8_t channel_port_get(uint32_t pin)
{
  return g_gpiotedev.pin_assignments[pin];
}


static inline nrf_drv_gpiote_evt_handler_t channel_handler_get(uint32_t channel)
{
  return g_gpiotedev.handlers[channel];
}

/****************************************************************************
 * Name: channel_port_alloc
 *
 * Description:
 *
 ****************************************************************************/
static int8_t channel_port_alloc(uint32_t pin, nrf_drv_gpiote_evt_handler_t handler, bool channel)
{
  int8_t   channel_id = NO_CHANNELS;
  uint32_t i;
  irqstate_t flags;


  uint32_t start_idx = channel ? 0 : GPIOTE_CH_NUM;
  uint32_t end_idx   =
    channel ? GPIOTE_CH_NUM : (GPIOTE_CH_NUM + GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS);

  flags = enter_critical_section();

  for (i = start_idx; i < end_idx; i++)
    {
      if (g_gpiotedev.handlers[i] == FORBIDDEN_HANDLER_ADDRESS)
        {
          pin_in_use_by_te_set(pin, i, handler, channel);
          channel_id = i;
          break;
        }
    }

  leave_critical_section(flags);

  return channel_id;
}

/****************************************************************************
 * Name: channel_free
 *
 * Description:
 *
 ****************************************************************************/
static void channel_free(uint8_t channel_id)
{
  g_gpiotedev.handlers[channel_id] = FORBIDDEN_HANDLER_ADDRESS;
  if (channel_id >= GPIOTE_CH_NUM)
    {
      g_gpiotedev.port_handlers_pins[channel_id - GPIOTE_CH_NUM] = (int8_t)PIN_NOT_USED;
    }
}

static uint32_t nrf52_bitmask_bit_is_set(uint32_t bit, void const *p_mask)
{
  uint8_t *pdata = (uint8_t *)p_mask;

  return pdata[bit / 8] & (1 << (bit % 8));
}

static uint32_t nrf52_bitmask_bit_set(uint32_t bit, void const *p_mask)
{
  uint8_t *pdata = (uint8_t *)p_mask;

  return pdata[bit / 8] |= (1 << (bit % 8));
}

/****************************************************************************
 * Name: GPIOTE_IRQHandler
 *
 * Description:
 *
 ****************************************************************************/
int GPIOTE_IRQHandler(int irq, void *context, FAR void *arg)
{
  uint32_t status            = 0;
  uint32_t input[GPIO_COUNT] = {0};

  /* collect status of all GPIOTE pin events. Processing is done once all are collected and cleared.*/
  uint32_t            i;
  nrf_gpiote_events_t event = NRF_GPIOTE_EVENTS_IN_0;
  uint32_t            mask  = (uint32_t)NRF_GPIOTE_INT_IN0_MASK;

  for (i = 0; i < GPIOTE_CH_NUM; i++)
    {
      if (nrf_gpiote_event_is_set(event) && nrf_gpiote_int_is_enabled(mask))
        {
          nrf_gpiote_event_clear(event);

          status |= mask;
        }
      mask <<= 1;
      /* Incrementing to next event, utilizing the fact that events are grouped together
       * in ascending order. */
      event = (nrf_gpiote_events_t)((uint32_t)event + sizeof(uint32_t));
    }

  /* collect PORT status event, if event is set read pins state. Processing is postponed to the
   * end of interrupt. */
  if (nrf_gpiote_event_is_set(NRF_GPIOTE_EVENTS_PORT))
    {
      nrf_gpiote_event_clear(NRF_GPIOTE_EVENTS_PORT);
      status |= (uint32_t)NRF_GPIOTE_INT_PORT_MASK;
      nrf_gpio_ports_read(0, GPIO_COUNT, input);
    }

  /* Process pin events. */
  if (status & NRF_GPIOTE_INT_IN_MASK)
    {
      mask = (uint32_t)NRF_GPIOTE_INT_IN0_MASK;

      for (i = 0; i < GPIOTE_CH_NUM; i++)
        {
          if (mask & status)
            {
              nrf_drv_gpiote_pin_t pin = nrf_gpiote_event_pin_get(i);
              nrf_gpiote_polarity_t        polarity = nrf_gpiote_event_polarity_get(i);
              nrf_drv_gpiote_evt_handler_t handler  = channel_handler_get(i);
              if (handler)
                {
                  handler(pin, polarity);
                }
            }
          mask <<= 1;
        }
    }

  if (status & (uint32_t)NRF_GPIOTE_INT_PORT_MASK)
    {
      /* Process port event. */
      uint32_t port_idx;
      uint8_t  repeat                  = 0;
      uint32_t toggle_mask[GPIO_COUNT] = {0};
      uint32_t pins_to_check[GPIO_COUNT];

      // Faster way of doing memset because in interrupt context.
      for (port_idx = 0; port_idx < GPIO_COUNT; port_idx++)
        {
          pins_to_check[port_idx] = 0xFFFFFFFF;
        }

      do
        {
          repeat = 0;

          for (i = 0; i < GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS; i++)
            {
              uint8_t              pin_and_sense = g_gpiotedev.port_handlers_pins[i];
              nrf_drv_gpiote_pin_t pin           = (pin_and_sense & ~SENSE_FIELD_MASK);

              if ((g_gpiotedev.port_handlers_pins[i] != PIN_NOT_USED)
                  && nrf52_bitmask_bit_is_set(pin, pins_to_check))
                {
                  nrf_gpiote_polarity_t polarity =
                    (nrf_gpiote_polarity_t)((pin_and_sense &
                                             SENSE_FIELD_MASK) >> SENSE_FIELD_POS);
                  nrf_drv_gpiote_evt_handler_t handler =
                    channel_handler_get(channel_port_get(pin));
                  if (handler || (polarity == NRF_GPIOTE_POLARITY_TOGGLE))
                    {
                      if (polarity == NRF_GPIOTE_POLARITY_TOGGLE)
                        {
                          nrf52_bitmask_bit_set(pin, toggle_mask);
                        }
                      nrf_gpio_pin_sense_t sense     = nrf_gpio_pin_sense_get(pin);
                      uint32_t             pin_state = nrf52_bitmask_bit_is_set(pin, input);
                      if ((pin_state && (sense == NRF_GPIO_PIN_SENSE_HIGH)) ||
                          (!pin_state && (sense == NRF_GPIO_PIN_SENSE_LOW))  )
                        {
                          if (polarity == NRF_GPIOTE_POLARITY_TOGGLE)
                            {
                              nrf_gpio_pin_sense_t next_sense =
                                (sense == NRF_GPIO_PIN_SENSE_HIGH) ?
                                NRF_GPIO_PIN_SENSE_LOW :
                                NRF_GPIO_PIN_SENSE_HIGH;
                              nrf_gpio_cfg_sense_set(pin, next_sense);
                              ++repeat;

                            }
                          if (handler)
                            {
                              handler(pin, polarity);
                            }
                        }
                    }
                }
            }

          if (repeat)
            {
              // When one of the pins in low-accuracy and toggle mode becomes active,
              // it's sense mode is inverted to clear the internal SENSE signal.
              // State of any other enabled low-accuracy input in toggle mode must be checked
              // explicitly, because it does not trigger the interrput when SENSE signal is active.
              // For more information about SENSE functionality, refer to Product Specification.

              uint32_t new_input[GPIO_COUNT];
              bool     input_unchanged = true;
              nrf_gpio_ports_read(0, GPIO_COUNT, new_input);

              // Faster way of doing memcmp because in interrupt context.
              for (port_idx = 0; port_idx < GPIO_COUNT; port_idx++)
                {
                  if (new_input[port_idx] != input[port_idx])
                    {
                      input_unchanged = false;
                      break;
                    }
                }

              if (input_unchanged)
                {
                  // No change.
                  repeat = 0;
                }
              else
                {
                  // Faster way of doing memcpy because in interrupt context.
                  for (port_idx = 0; port_idx < GPIO_COUNT; port_idx++)
                    {
                      input[port_idx]         = new_input[port_idx];
                      pins_to_check[port_idx] = toggle_mask[port_idx];
                    }
                }
            }
        }
      while (repeat);
    }
  return OK;
}


/****************************************************************************
 * Name: nrf_drv_gpiote_init
 *
 * Description:
 *
 ****************************************************************************/
ret_code_t nrf_drv_gpiote_init(void)
{
  ret_code_t err_code;

  if (g_gpiotedev.state != NRF_DRV_STATE_UNINITIALIZED)
    {
      err_code = NRF_ERROR_INVALID_STATE;
      return err_code;
    }

  uint8_t i;

  for (i = 0; i < NUMBER_OF_PINS; i++)
    {
      pin_in_use_clear(i);
    }

  for (i = 0; i < (GPIOTE_CH_NUM + GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS); i++)
    {
      channel_free(i);
    }

  irq_attach(GPIOTE_IRQn, GPIOTE_IRQHandler, NULL);
  nrf_drv_common_irq_enable(GPIOTE_IRQn, GPIOTE_CONFIG_IRQ_PRIORITY);
  nrf_gpiote_event_clear(NRF_GPIOTE_EVENTS_PORT);
  nrf_gpiote_int_enable(GPIOTE_INTENSET_PORT_Msk);
  g_gpiotedev.state = NRF_DRV_STATE_INITIALIZED;

  err_code = NRF_SUCCESS;
  return err_code;
}

/****************************************************************************
 * Name: nrf_drv_gpiote_is_init
 *
 * Description:
 *
 ****************************************************************************/
bool nrf_drv_gpiote_is_init(void)
{
  return (g_gpiotedev.state != NRF_DRV_STATE_UNINITIALIZED) ? true : false;
}

/****************************************************************************
 * Name: nrf_drv_gpiote_uninit
 *
 * Description:
 *
 ****************************************************************************/
void nrf_drv_gpiote_uninit(void)
{
  ASSERT(g_gpiotedev.state != NRF_DRV_STATE_UNINITIALIZED);

  uint32_t i;

  for (i = 0; i < NUMBER_OF_PINS; i++)
    {
      if (pin_in_use_as_non_task_out(i))
        {
          nrf_drv_gpiote_out_uninit(i);
        }
      else if ( pin_in_use_by_gpiote(i))
        {
          /* Disable gpiote_in is having the same effect on out pin as gpiote_out_uninit on
           * so it can be called on all pins used by GPIOTE.
           */
          nrf_drv_gpiote_in_uninit(i);
        }
    }
  g_gpiotedev.state = NRF_DRV_STATE_UNINITIALIZED;
}

/****************************************************************************
 * Name: nrf_drv_gpiote_out_init
 *
 * Description:
 *
 ****************************************************************************/
ret_code_t nrf_drv_gpiote_out_init(nrf_drv_gpiote_pin_t                pin,
                                   nrf_drv_gpiote_out_config_t const *p_config)
{
  ASSERT(pin < NUMBER_OF_PINS);
  ASSERT(g_gpiotedev.state == NRF_DRV_STATE_INITIALIZED);
  ASSERT(p_config);

  ret_code_t err_code = NRF_SUCCESS;

  if (pin_in_use(pin))
    {
      err_code = NRF_ERROR_INVALID_STATE;
    }
  else
    {
      if (p_config->task_pin)
        {
          int8_t channel = channel_port_alloc(pin, NULL, true);

          if (channel != NO_CHANNELS)
            {
              nrf_gpiote_task_configure(channel, pin, p_config->action, p_config->init_state);
            }
          else
            {
              err_code = NRF_ERROR_NO_MEM;
            }
        }
      else
        {
          pin_in_use_set(pin);
        }

      if (err_code == NRF_SUCCESS)
        {
          if (p_config->init_state == NRF_GPIOTE_INITIAL_VALUE_HIGH)
            {
              nrf_gpio_pin_set(pin);
            }
          else
            {
              nrf_gpio_pin_clear(pin);
            }

          nrf_gpio_cfg_output(pin);
        }
    }

  return err_code;
}

/****************************************************************************
 * Name: nrf_drv_gpiote_out_uninit
 *
 * Description:
 *
 ****************************************************************************/
void nrf_drv_gpiote_out_uninit(nrf_drv_gpiote_pin_t pin)
{
  ASSERT(pin < NUMBER_OF_PINS);
  ASSERT(pin_in_use(pin));

  if (pin_in_use_by_te(pin))
    {
      channel_free((uint8_t)channel_port_get(pin));
      nrf_gpiote_te_default(channel_port_get(pin));
    }
  pin_in_use_clear(pin);

  nrf_gpio_cfg_default(pin);
}

/****************************************************************************
 * Name: nrf_drv_gpiote_out_set
 *
 * Description:
 *
 ****************************************************************************/
void nrf_drv_gpiote_out_set(nrf_drv_gpiote_pin_t pin)
{
  ASSERT(pin < NUMBER_OF_PINS);
  ASSERT(pin_in_use(pin));
  ASSERT(!pin_in_use_by_te(pin));

  nrf_gpio_pin_set(pin);
}

/****************************************************************************
 * Name: nrf_drv_gpiote_out_clear
 *
 * Description:
 *
 ****************************************************************************/
void nrf_drv_gpiote_out_clear(nrf_drv_gpiote_pin_t pin)
{
  ASSERT(pin < NUMBER_OF_PINS);
  ASSERT(pin_in_use(pin));
  ASSERT(!pin_in_use_by_te(pin));

  nrf_gpio_pin_clear(pin);
}

/****************************************************************************
 * Name: nrf_drv_gpiote_out_toggle
 *
 * Description:
 *
 ****************************************************************************/
void nrf_drv_gpiote_out_toggle(nrf_drv_gpiote_pin_t pin)
{
  ASSERT(pin < NUMBER_OF_PINS);
  ASSERT(pin_in_use(pin));
  ASSERT(!pin_in_use_by_te(pin));

  nrf_gpio_pin_toggle(pin);
}

/****************************************************************************
 * Name: nrf_drv_gpiote_out_task_enable
 *
 * Description:
 *
 ****************************************************************************/
void nrf_drv_gpiote_out_task_enable(nrf_drv_gpiote_pin_t pin)
{
  ASSERT(pin < NUMBER_OF_PINS);
  ASSERT(pin_in_use(pin));
  ASSERT(pin_in_use_by_te(pin));

  nrf_gpiote_task_enable(g_gpiotedev.pin_assignments[pin]);
}

/****************************************************************************
 * Name: nrf_drv_gpiote_out_task_disable
 *
 * Description:
 *
 ****************************************************************************/
void nrf_drv_gpiote_out_task_disable(nrf_drv_gpiote_pin_t pin)
{
  ASSERT(pin < NUMBER_OF_PINS);
  ASSERT(pin_in_use(pin));
  ASSERT(pin_in_use_by_te(pin));

  nrf_gpiote_task_disable(g_gpiotedev.pin_assignments[pin]);
}

/****************************************************************************
 * Name: nrf_drv_gpiote_out_task_addr_get
 *
 * Description:
 *
 ****************************************************************************/
uint32_t nrf_drv_gpiote_out_task_addr_get(nrf_drv_gpiote_pin_t pin)
{
  ASSERT(pin < NUMBER_OF_PINS);
  ASSERT(pin_in_use_by_te(pin));

  nrf_gpiote_tasks_t task = TE_OUT_IDX_TO_TASK_ADDR(channel_port_get(pin));
  return nrf_gpiote_task_addr_get(task);
}


#if defined(GPIOTE_FEATURE_SET_PRESENT)
uint32_t nrf_drv_gpiote_set_task_addr_get(nrf_drv_gpiote_pin_t pin)
{
  ASSERT(pin < NUMBER_OF_PINS);
  ASSERT(pin_in_use_by_te(pin));

  nrf_gpiote_tasks_t task = TE_SET_IDX_TO_TASK_ADDR(channel_port_get(pin));
  return nrf_gpiote_task_addr_get(task);
}


#endif // defined(GPIOTE_FEATURE_SET_PRESENT)

#if defined(GPIOTE_FEATURE_CLR_PRESENT)
uint32_t nrf_drv_gpiote_clr_task_addr_get(nrf_drv_gpiote_pin_t pin)
{
  ASSERT(pin < NUMBER_OF_PINS);
  ASSERT(pin_in_use_by_te(pin));

  nrf_gpiote_tasks_t task = TE_CLR_IDX_TO_TASK_ADDR(channel_port_get(pin));
  return nrf_gpiote_task_addr_get(task);
}


#endif // defined(GPIOTE_FEATURE_CLR_PRESENT)

/****************************************************************************
 * Name: nrf_drv_gpiote_out_task_force
 *
 * Description:
 *
 ****************************************************************************/
void nrf_drv_gpiote_out_task_force(nrf_drv_gpiote_pin_t pin, uint8_t state)
{
  ASSERT(pin < NUMBER_OF_PINS);
  ASSERT(pin_in_use(pin));
  ASSERT(pin_in_use_by_te(pin));

  nrf_gpiote_outinit_t init_val =
    state ? NRF_GPIOTE_INITIAL_VALUE_HIGH : NRF_GPIOTE_INITIAL_VALUE_LOW;
  nrf_gpiote_task_force(g_gpiotedev.pin_assignments[pin], init_val);
}

/****************************************************************************
 * Name: nrf_drv_gpiote_out_task_trigger
 *
 * Description:
 *
 ****************************************************************************/
void nrf_drv_gpiote_out_task_trigger(nrf_drv_gpiote_pin_t pin)
{
  ASSERT(pin < NUMBER_OF_PINS);
  ASSERT(pin_in_use(pin));
  ASSERT(pin_in_use_by_te(pin));

  nrf_gpiote_tasks_t task = TE_OUT_IDX_TO_TASK_ADDR(channel_port_get(pin));
  nrf_gpiote_task_set(task);
}


#if defined(GPIOTE_FEATURE_SET_PRESENT)
void nrf_drv_gpiote_set_task_trigger(nrf_drv_gpiote_pin_t pin)
{
  ASSERT(pin < NUMBER_OF_PINS);
  ASSERT(pin_in_use(pin));
  ASSERT(pin_in_use_by_te(pin));

  nrf_gpiote_tasks_t task = TE_SET_IDX_TO_TASK_ADDR(channel_port_get(pin));
  nrf_gpiote_task_set(task);
}


#endif // defined(GPIOTE_FEATURE_SET_PRESENT)

#if  defined(GPIOTE_FEATURE_CLR_PRESENT)
void nrf_drv_gpiote_clr_task_trigger(nrf_drv_gpiote_pin_t pin)
{
  ASSERT(pin < NUMBER_OF_PINS);
  ASSERT(pin_in_use(pin));
  ASSERT(pin_in_use_by_te(pin));

  nrf_gpiote_tasks_t task = TE_CLR_IDX_TO_TASK_ADDR(channel_port_get(pin));
  nrf_gpiote_task_set(task);
}


#endif // defined(GPIOTE_FEATURE_CLR_PRESENT)

/****************************************************************************
 * Name: nrf_drv_gpiote_in_init
 *
 * Description:
 *
 ****************************************************************************/
ret_code_t nrf_drv_gpiote_in_init(nrf_drv_gpiote_pin_t               pin,
                                  nrf_drv_gpiote_in_config_t const *p_config,
                                  nrf_drv_gpiote_evt_handler_t       evt_handler)
{
  ASSERT(pin < NUMBER_OF_PINS);
  ret_code_t err_code = NRF_SUCCESS;

  /* Only one GPIOTE channel can be assigned to one physical pin. */
  if (pin_in_use_by_gpiote(pin))
    {
      err_code = NRF_ERROR_INVALID_STATE;
    }
  else
    {
      int8_t channel = channel_port_alloc(pin, evt_handler, p_config->hi_accuracy);
      if (channel != NO_CHANNELS)
        {
          if (p_config->is_watcher)
            {
              nrf_gpio_cfg_watcher(pin);
            }
          else
            {
              nrf_gpio_cfg_input(pin, p_config->pull);
            }

          if (p_config->hi_accuracy)
            {
              nrf_gpiote_event_configure(channel, pin, p_config->sense);
            }
          else
            {
              g_gpiotedev.port_handlers_pins[channel -
                                                     GPIOTE_CH_NUM] |= (p_config->sense) << SENSE_FIELD_POS;
            }
        }
      else
        {
          err_code = NRF_ERROR_NO_MEM;
        }
    }

  return err_code;
}

/****************************************************************************
 * Name: nrf_drv_gpiote_in_event_enable
 *
 * Description:
 *
 ****************************************************************************/
void nrf_drv_gpiote_in_event_enable(nrf_drv_gpiote_pin_t pin, bool int_enable)
{
  ASSERT(pin < NUMBER_OF_PINS);
  ASSERT(pin_in_use_by_gpiote(pin));
  if (pin_in_use_by_port(pin))
    {
      uint8_t pin_and_sense =
        g_gpiotedev.port_handlers_pins[channel_port_get(pin) - GPIOTE_CH_NUM];
      nrf_gpiote_polarity_t polarity =
        (nrf_gpiote_polarity_t)(pin_and_sense >> SENSE_FIELD_POS);
      nrf_gpio_pin_sense_t sense;
      if (polarity == NRF_GPIOTE_POLARITY_TOGGLE)
        {
          /* read current pin state and set for next sense to oposit */
          sense = (nrf_gpio_pin_read(pin)) ?
                  NRF_GPIO_PIN_SENSE_LOW : NRF_GPIO_PIN_SENSE_HIGH;
        }
      else
        {
          sense = (polarity == NRF_GPIOTE_POLARITY_LOTOHI) ?
                  NRF_GPIO_PIN_SENSE_HIGH : NRF_GPIO_PIN_SENSE_LOW;
        }
      nrf_gpio_cfg_sense_set(pin, sense);
    }
  else if (pin_in_use_by_te(pin))
    {
      int32_t             channel = (int32_t)channel_port_get(pin);
      nrf_gpiote_events_t event   = TE_IDX_TO_EVENT_ADDR(channel);

      nrf_gpiote_event_enable(channel);

      nrf_gpiote_event_clear(event);
      if (int_enable)
        {
          nrf_drv_gpiote_evt_handler_t handler = channel_handler_get(channel_port_get(pin));
          // Enable the interrupt only if event handler was provided.
          if (handler)
            {
              nrf_gpiote_int_enable(1 << channel);
            }
        }
    }
}

/****************************************************************************
 * Name: nrf_drv_gpiote_in_event_disable
 *
 * Description:
 *
 ****************************************************************************/
void nrf_drv_gpiote_in_event_disable(nrf_drv_gpiote_pin_t pin)
{
  ASSERT(pin < NUMBER_OF_PINS);
  ASSERT(pin_in_use_by_gpiote(pin));
  if (pin_in_use_by_port(pin))
    {
      nrf_gpio_cfg_sense_set(pin, NRF_GPIO_PIN_NOSENSE);
    }
  else if (pin_in_use_by_te(pin))
    {
      int32_t channel = (int32_t)channel_port_get(pin);
      nrf_gpiote_event_disable(channel);
      nrf_gpiote_int_disable(1 << channel);
    }
}

/****************************************************************************
 * Name: nrf_drv_gpiote_in_uninit
 *
 * Description:
 *
 ****************************************************************************/
void nrf_drv_gpiote_in_uninit(nrf_drv_gpiote_pin_t pin)
{
  ASSERT(pin < NUMBER_OF_PINS);
  ASSERT(pin_in_use_by_gpiote(pin));
  nrf_drv_gpiote_in_event_disable(pin);
  if (pin_in_use_by_te(pin))
    {
      nrf_gpiote_te_default(channel_port_get(pin));
    }
  nrf_gpio_cfg_default(pin);
  channel_free((uint8_t)channel_port_get(pin));
  pin_in_use_clear(pin);
}

/****************************************************************************
 * Name: nrf_drv_gpiote_in_is_set
 *
 * Description:
 *
 ****************************************************************************/
bool nrf_drv_gpiote_in_is_set(nrf_drv_gpiote_pin_t pin)
{
  ASSERT(pin < NUMBER_OF_PINS);
  return nrf_gpio_pin_read(pin) ? true : false;
}

/****************************************************************************
 * Name: nrf_drv_gpiote_in_event_addr_get
 *
 * Description:
 *
 ****************************************************************************/
uint32_t nrf_drv_gpiote_in_event_addr_get(nrf_drv_gpiote_pin_t pin)
{
  ASSERT(pin < NUMBER_OF_PINS);
  ASSERT(pin_in_use_by_port(pin) || pin_in_use_by_te(pin));

  nrf_gpiote_events_t event = NRF_GPIOTE_EVENTS_PORT;

  if (pin_in_use_by_te(pin))
    {
      event = TE_IDX_TO_EVENT_ADDR(channel_port_get(pin));
    }
  return nrf_gpiote_event_addr_get(event);
}


int nrf_sdk_retcode_to_nuttx(ret_code_t nrf_ret_code)
{
  int ret;

  switch (nrf_ret_code)
    {
      case NRF_SUCCESS:
        ret = OK;
        break;
      case NRF_ERROR_NO_MEM:
        ret = -ENOMEM;
        break;
      case NRF_ERROR_INVALID_PARAM:
        ret = -EINVAL;
        break;
      case NRF_ERROR_BUSY:
        ret = -EBUSY;
        break;
      case NRF_ERROR_INVALID_ADDR:
        ret = -EFAULT;
        break;
      default:
        ret = -EIO;
        break;
    }

  return ret;
}

/*lint -restore*/
#endif // CONFIG_NRF52_GPIOTE
