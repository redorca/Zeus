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


#include <stddef.h>
#include <stdio.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf52_gpio.h"
#include "nrf_error.h"
#include "bsp.h"
#include "bsp_config.h"
#include "boards.h"

#ifndef BSP_SIMPLE
#include <utils/app_timer.h>
#endif // BSP_SIMPLE

#if defined(LEDS_NUMBER) && LEDS_NUMBER > 0 && !(defined BSP_SIMPLE)
static bsp_indication_t m_stable_state        = BSP_INDICATE_IDLE;
static bool             m_leds_clear          = false;
static uint32_t         m_indication_type     = 0;
timer_t m_leds_timer_id;
#endif // LEDS_NUMBER > 0 && !(defined BSP_SIMPLE)


#if LEDS_NUMBER > 0 && !(defined BSP_SIMPLE)
/**@brief       Configure leds to indicate required state.
 * @param[in]   indicate   State to be indicated.
 */
static uint32_t bsp_led_indication(bsp_indication_t indicate)
{
  uint32_t err_code   = NRF_SUCCESS;
  uint32_t next_delay = 0;

  if (m_leds_clear)
    {
      m_leds_clear = false;
      bsp_board_leds_off();
    }

  switch (indicate)
    {
      case BSP_INDICATE_IDLE:
        bsp_board_leds_off();
        m_stable_state = indicate;
        break;

      case BSP_INDICATE_SCANNING:
      case BSP_INDICATE_ADVERTISING:
        // in advertising blink LED_0
        if (bsp_board_led_state_get(BSP_LED_INDICATE_INDICATE_ADVERTISING))
          {
            bsp_board_led_off(BSP_LED_INDICATE_INDICATE_ADVERTISING);
            next_delay = indicate ==
                         BSP_INDICATE_ADVERTISING ? ADVERTISING_LED_OFF_INTERVAL :
                         ADVERTISING_SLOW_LED_OFF_INTERVAL;
          }
        else
          {
            bsp_board_led_on(BSP_LED_INDICATE_INDICATE_ADVERTISING);
            next_delay = indicate ==
                         BSP_INDICATE_ADVERTISING ? ADVERTISING_LED_ON_INTERVAL :
                         ADVERTISING_SLOW_LED_ON_INTERVAL;
          }

        m_stable_state = indicate;
        err_code = app_timer_start(m_leds_timer_id, next_delay, APP_TIMER_MODE_SINGLE_SHOT);
        break;

      case BSP_INDICATE_ADVERTISING_WHITELIST:
        // in advertising quickly blink LED_0
        if (bsp_board_led_state_get(BSP_LED_INDICATE_ADVERTISING_WHITELIST))
          {
            bsp_board_led_off(BSP_LED_INDICATE_ADVERTISING_WHITELIST);
            next_delay = indicate ==
                         BSP_INDICATE_ADVERTISING_WHITELIST ?
                         ADVERTISING_WHITELIST_LED_OFF_INTERVAL :
                         ADVERTISING_SLOW_LED_OFF_INTERVAL;
          }
        else
          {
            bsp_board_led_on(BSP_LED_INDICATE_ADVERTISING_WHITELIST);
            next_delay = indicate ==
                         BSP_INDICATE_ADVERTISING_WHITELIST ?
                         ADVERTISING_WHITELIST_LED_ON_INTERVAL :
                         ADVERTISING_SLOW_LED_ON_INTERVAL;
          }
        m_stable_state = indicate;
        err_code = app_timer_start(m_leds_timer_id, next_delay, APP_TIMER_MODE_SINGLE_SHOT);
        break;

      case BSP_INDICATE_ADVERTISING_SLOW:
        // in advertising slowly blink LED_0
        if (bsp_board_led_state_get(BSP_LED_INDICATE_ADVERTISING_SLOW))
          {
            bsp_board_led_off(BSP_LED_INDICATE_ADVERTISING_SLOW);
            next_delay = indicate ==
                         BSP_INDICATE_ADVERTISING_SLOW ? ADVERTISING_SLOW_LED_OFF_INTERVAL :
                         ADVERTISING_SLOW_LED_OFF_INTERVAL;
          }
        else
          {
            bsp_board_led_on(BSP_LED_INDICATE_ADVERTISING_SLOW);
            next_delay = indicate ==
                         BSP_INDICATE_ADVERTISING_SLOW ? ADVERTISING_SLOW_LED_ON_INTERVAL :
                         ADVERTISING_SLOW_LED_ON_INTERVAL;
          }
        m_stable_state = indicate;
        err_code = app_timer_start(m_leds_timer_id, next_delay, APP_TIMER_MODE_SINGLE_SHOT);
        break;

      case BSP_INDICATE_ADVERTISING_DIRECTED:
        // in advertising very quickly blink LED_0
        if (bsp_board_led_state_get(BSP_LED_INDICATE_ADVERTISING_DIRECTED))
          {
            bsp_board_led_off(BSP_LED_INDICATE_ADVERTISING_DIRECTED);
            next_delay = indicate ==
                         BSP_INDICATE_ADVERTISING_DIRECTED ?
                         ADVERTISING_DIRECTED_LED_OFF_INTERVAL :
                         ADVERTISING_SLOW_LED_OFF_INTERVAL;
          }
        else
          {
            bsp_board_led_on(BSP_LED_INDICATE_ADVERTISING_DIRECTED);
            next_delay = indicate ==
                         BSP_INDICATE_ADVERTISING_DIRECTED ?
                         ADVERTISING_DIRECTED_LED_ON_INTERVAL :
                         ADVERTISING_SLOW_LED_ON_INTERVAL;
          }
        m_stable_state = indicate;
        err_code = app_timer_start(m_leds_timer_id, next_delay, APP_TIMER_MODE_SINGLE_SHOT);
        break;

      case BSP_INDICATE_BONDING:
        // in bonding fast blink LED_0
        bsp_board_led_invert(BSP_LED_INDICATE_BONDING);

        m_stable_state = indicate;
        err_code = app_timer_start(m_leds_timer_id, BONDING_INTERVAL, APP_TIMER_MODE_SINGLE_SHOT);
        break;

      case BSP_INDICATE_CONNECTED:
        bsp_board_led_on(BSP_LED_INDICATE_CONNECTED);
        m_stable_state = indicate;
        break;

      case BSP_INDICATE_SENT_OK:
        // when sending shortly invert LED_1
        m_leds_clear = true;
        bsp_board_led_invert(BSP_LED_INDICATE_SENT_OK);
        err_code = app_timer_start(m_leds_timer_id, SENT_OK_INTERVAL, APP_TIMER_MODE_SINGLE_SHOT);
        break;

      case BSP_INDICATE_SEND_ERROR:
        // on receving error invert LED_1 for long time
        m_leds_clear = true;
        bsp_board_led_invert(BSP_LED_INDICATE_SEND_ERROR);
        err_code = app_timer_start(m_leds_timer_id, SEND_ERROR_INTERVAL, APP_TIMER_MODE_SINGLE_SHOT);
        break;

      case BSP_INDICATE_RCV_OK:
        // when receving shortly invert LED_1
        m_leds_clear = true;
        bsp_board_led_invert(BSP_LED_INDICATE_RCV_OK);
        err_code = app_timer_start(m_leds_timer_id, RCV_OK_INTERVAL, APP_TIMER_MODE_SINGLE_SHOT);
        break;

      case BSP_INDICATE_RCV_ERROR:
        // on receving error invert LED_1 for long time
        m_leds_clear = true;
        bsp_board_led_invert(BSP_LED_INDICATE_RCV_ERROR);
        err_code = app_timer_start(m_leds_timer_id, RCV_ERROR_INTERVAL, APP_TIMER_MODE_SINGLE_SHOT);
        break;

      case BSP_INDICATE_FATAL_ERROR:
        // on fatal error turn on all leds
        bsp_board_leds_on();
        m_stable_state = indicate;
        break;

      case BSP_INDICATE_ALERT_0:
      case BSP_INDICATE_ALERT_1:
      case BSP_INDICATE_ALERT_2:
      case BSP_INDICATE_ALERT_3:
      case BSP_INDICATE_ALERT_OFF:
        next_delay = (uint32_t)BSP_INDICATE_ALERT_OFF - (uint32_t)indicate;

        // a little trick to find out that if it did not fall through ALERT_OFF
        if (next_delay && (err_code == NRF_SUCCESS))
          {
            if (next_delay > 1)
              {

              }
            bsp_board_led_on(BSP_LED_ALERT);
          }
        else
          {
            bsp_board_led_off(BSP_LED_ALERT);
          }
        break;

      case BSP_INDICATE_USER_STATE_OFF:
        bsp_board_leds_off();
        m_stable_state = indicate;
        break;

      case BSP_INDICATE_USER_STATE_0:
        bsp_board_leds_off();
        bsp_board_led_on(BSP_LED_INDICATE_USER_LED1);
        m_stable_state = indicate;
        break;

      case BSP_INDICATE_USER_STATE_1:
        bsp_board_leds_off();
        bsp_board_led_on(BSP_LED_INDICATE_USER_LED2);
        m_stable_state = indicate;
        break;

      case BSP_INDICATE_USER_STATE_2:
        bsp_board_leds_off();
        bsp_board_led_on(BSP_LED_INDICATE_USER_LED1);
        bsp_board_led_on(BSP_LED_INDICATE_USER_LED2);
        m_stable_state = indicate;
        break;

      case BSP_INDICATE_USER_STATE_3:

      case BSP_INDICATE_USER_STATE_ON:
        bsp_board_leds_on();
        m_stable_state = indicate;
        break;

      default:
        break;
    }

  return err_code;
}


/**@brief Handle events from leds timer.
 *
 * @note Timer handler does not support returning an error code.
 * Errors from bsp_led_indication() are not propagated.
 *
 * @param[in]   p_context   parameter registered in timer start function.
 */
void leds_timer_handler()
{

  if (m_indication_type & BSP_INIT_LED)
    {
      UNUSED_VARIABLE(bsp_led_indication(m_stable_state));
    }
}

#endif // #if LEDS_NUMBER > 0 && !(defined BSP_SIMPLE)


/**@brief Configure indicators to required state.
 */
uint32_t bsp_indication_set(bsp_indication_t indicate)
{
  uint32_t err_code = NRF_SUCCESS;

#if LEDS_NUMBER > 0 && !(defined BSP_SIMPLE)
  if (m_indication_type & BSP_INIT_LED)
    {
      err_code = bsp_led_indication(indicate);
    }

#endif // LEDS_NUMBER > 0 && !(defined BSP_SIMPLE)
  return err_code;
}


uint32_t bsp_init(uint32_t type, bsp_event_callback_t callback)
{
  uint32_t err_code = NRF_SUCCESS;


#if LEDS_NUMBER > 0 && !(defined BSP_SIMPLE)
  m_indication_type     = type;
#endif // LEDS_NUMBER > 0 && !(defined BSP_SIMPLE)


#if LEDS_NUMBER > 0 && !(defined BSP_SIMPLE)

  if (type & BSP_INIT_LED)
    {
      bsp_board_leds_init();
    }

#endif // LEDS_NUMBER > 0 && !(defined BSP_SIMPLE)

  return err_code;
}

