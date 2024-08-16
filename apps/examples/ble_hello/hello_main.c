/****************************************************************************
 * examples/hello/hello_main.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include <nuttx/board.h>
#include <errno.h>
#include <stdio.h>
#include <pthread.h>
#include <wireless/bluetooth/ble_app_api.h>
#include "app_error.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "../wireless/bluetooth/nrf/ble_bsp/bsp.h"
#include <utils/app_timer.h>
#include "sensorsim.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

#define DEVICE_NAME                      "Zglue_HRM"                     /**< Name of device. Will be included in the advertising data. */

#define APP_TIMER_PRESCALER              0                                /**< Value of the RTC1 PRESCALER register. */

#define BATTERY_LEVEL_MEAS_INTERVAL      2000                             /**< Battery level measurement interval (ms). */

#define HEART_RATE_MEAS_INTERVAL         1000                             /**< Heart rate measurement interval (ms). */

#define RR_INTERVAL_INTERVAL             300                              /**< RR interval interval (ms). */

#define SENSOR_CONTACT_DETECTED_INTERVAL 5000                             /**< Sensor Contact Detected toggle interval (ms). */

/****************************************************************************
 * Global variables
 ****************************************************************************/
extern pthread_mutex_t mut_sig_en;

static bool   m_rr_interval_enabled =
  true;            /**< Flag for enabling and disabling the registration of new RR interval measurements. */

static sensorsim_cfg_t   m_battery_sim_cfg;               /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t m_battery_sim_state;             /**< Battery Level sensor simulator state. */
static sensorsim_cfg_t   m_heart_rate_sim_cfg;            /**< Heart Rate sensor simulator configuration. */
static sensorsim_state_t m_heart_rate_sim_state;          /**< Heart Rate sensor simulator state. */
static sensorsim_cfg_t   m_rr_interval_sim_cfg;           /**< RR Interval sensor simulator configuration. */
static sensorsim_state_t m_rr_interval_sim_state;         /**< RR Interval sensor simulator state. */

static ble_uuid_16_t m_adv_uuids[] =                         /**< Universally unique service identifiers. */
{
  {BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE, BLE_UUID_HEART_RATE_SERVICE },
  {BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE, BLE_UUID_BATTERY_SERVICE },
  {BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE, BLE_UUID_DEVICE_INFORMATION_SERVICE }
};

sem_t m_ble_event_ready;

timer_t m_heart_rate_timer_id = NULL, m_battery_timer_id = NULL, m_rr_interval_timer_id = NULL, m_sensor_contact_timer_id = NULL;
extern timer_t m_detection_delay_timer_id, m_leds_timer_id;

/****************************************************************************
 * Public Functions
 ****************************************************************************/


/****************************************************************************
 * Private Functions
 ****************************************************************************/


/**@brief Function for handling the Battery measurement timer time-out.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 */
static void battery_level_meas_timeout_handler(void)
{
  uint8_t  battery_level;

  battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);
  battery_level_update(battery_level);
}


/**@brief Function for handling the Heart rate measurement timer timeout.
 *
 * @details This function will be called each time the heart rate measurement timer expires.
 *          It will exclude RR Interval data from every third measurement.
 */
static void heart_rate_meas_timeout_handler(void)
{
  static uint32_t cnt = 0;
  uint16_t        heart_rate;

  heart_rate = (uint16_t)sensorsim_measure(&m_heart_rate_sim_state, &m_heart_rate_sim_cfg);
  heart_rate_update(heart_rate);

  cnt++;
  // Disable RR Interval recording every third heart rate measurement.
  // NOTE: An application will normally not do this. It is done here just for testing generation
  // of messages without RR Interval measurements.
  m_rr_interval_enabled = ((cnt % 3) != 0);
}

/**@brief Function for handling the RR interval timer time-out.
 *
 * @details This function will be called each time the RR interval timer expires.
 */
static void rr_interval_timeout_handler(void)
{
  if (m_rr_interval_enabled)
    {
      uint16_t rr_interval;

      rr_interval = (uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                                                &m_rr_interval_sim_cfg);

      rr_interval_update(rr_interval);

    }
}

/**@brief Function for handling the Sensor Contact Detected timer time-out.
 *
 * @details This function will be called each time the Sensor Contact Detected timer expires.
 */
static void sensor_contact_detected_timeout_handler(void)
{
  static bool sensor_contact_detected = false;

  sensor_contact_detected = !sensor_contact_detected;

  sensor_contact_detected_update(sensor_contact_detected);

}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates application timers.
 */
static void timers_init(void)
{
  int status;
  status = app_timer_create(&m_heart_rate_timer_id, heart_rate_meas_timeout_handler);
  if (status < 0)
    {
      printf("app_timer_create fail\r\n");
    }
  status = app_timer_create(&m_battery_timer_id, battery_level_meas_timeout_handler);
  if (status < 0)
    {
      printf("app_timer_create fail\r\n");
    }
  status = app_timer_create(&m_rr_interval_timer_id, rr_interval_timeout_handler);
  if (status < 0)
    {
      printf("app_timer_create fail\r\n");
    }
  status = app_timer_create(&m_sensor_contact_timer_id, sensor_contact_detected_timeout_handler);
  if (status < 0)
    {
      printf("app_timer_create fail\r\n");
    }
  status = app_timer_create(&m_leds_timer_id, leds_timer_handler);
  if (status < 0)
    {
      printf("app_timer_create fail\r\n");
    }
}

/**@brief Function for the Timer Start.
 *
 * @details Start the timer module. This starts application timers.
 */
static void application_timers_start(void)
{
  app_timer_start(m_heart_rate_timer_id, HEART_RATE_MEAS_INTERVAL, APP_TIMER_MODE_REPEATED);
  app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, APP_TIMER_MODE_REPEATED);
  app_timer_start(m_rr_interval_timer_id, RR_INTERVAL_INTERVAL, APP_TIMER_MODE_REPEATED);
  app_timer_start(m_sensor_contact_timer_id, SENSOR_CONTACT_DETECTED_INTERVAL, APP_TIMER_MODE_REPEATED);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void leds_init(bool *p_erase_bonds)
{

  uint32_t err_code = bsp_init(BSP_INIT_LED,
                               NULL);

  APP_ERROR_CHECK(err_code);

}


#ifdef CONFIG_NRF52_PROCFS_DEVID
void set_chip_ID_to_BD_address(void)
{
  char dev_id[BLE_CHIP_ID_LEN] = {0};
  ble_app_addr_t addr;

  /*1. Open device ID file and read id value.*/
  FILE *devid_file = fopen(BLE_CHIP_ID, "r");
  fread(dev_id, 1, BLE_CHIP_ID_LEN, devid_file);

  /*2. Set staic random address*/
  addr.addr_type = BLE_APP_ADDR_TYPE_RANDOM_STATIC;
  for (int i = 0; i < BLE_APP_ADDR_LEN; i++)
    {
      addr.addr[i] = dev_id[i];
    }
  addr.addr[BLE_APP_ADDR_LEN - 1] |= BLE_APP_ADDR_TYPE_RANDOM_STATIC_MSB_BIT;
  ble_app_set_address(&addr);

  /*3. Close device id file. */
  fclose(devid_file);
}
#endif

/**@brief Timer thread.
 *
 * @details This application timer thread.
 *
 */
void *timer_thread(void *unused)
{
  pthread_mutex_init(&mut_sig_en, NULL);
  timers_init();
  application_timers_start();
  while (1)
    {
    }
}

/****************************************************************************
 * hello_main
 ****************************************************************************/
#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char *argv[])
#endif
{

  printf("BLE Heartrate example\r\n");

  uint32_t err_code;
  pthread_t m_timer_thread;
  ble_adv_params_t advpara;
  ble_app_advdata_t advdata;
  bool     erase_bonds;

  pthread_create(&m_timer_thread, NULL, timer_thread, NULL);

  leds_init(&erase_bonds);

  /*BLE core initialization.*********************************************/
  ble_init();

  /*Set device name*/
#ifndef _BLE_DEV_NAME
  ble_app_set_device_name((const uint8_t *)DEVICE_NAME, sizeof(DEVICE_NAME) - 1);
#endif

  /*Add chip ID as defaut staic random BD address, so every chip have unique BD address.*/
#ifdef CONFIG_NRF52_PROCFS_DEVID
  set_chip_ID_to_BD_address();
#endif

  /*BLE service initialization.*************************************/
  ble_srv_bas_init();
  ble_srv_dis_init();
  ble_srv_hrs_init();

  /*BLE ADV data initialization..*****************************************/
  memset(&advdata, 0, sizeof(ble_app_advdata_t));
  advdata.name_type = BLE_ADV_DATA_FULL_NAME;
  advdata.uuid16_list.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
  advdata.uuid16_list.p_uuids  = m_adv_uuids;
  err_code = ble_adv_set_data(&advdata, NULL);
  APP_ERROR_CHECK(err_code);

  /*BLE ADV paramter initialization..*************************************/
  memset(&advpara, 0, sizeof(ble_adv_params_t));
  advpara.timeout = BLE_APP_ADV_MAX_TIMEOUT;
  err_code = ble_adv_start(&advpara);
  APP_ERROR_CHECK(err_code);



  while (1)
    {
      sleep(1);
    }
  return 0;
}
