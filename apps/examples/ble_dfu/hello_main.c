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
#include <fcntl.h>
#include <pthread.h>
#include <nuttx/kmalloc.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_mbr.h"
#include "../wireless/bluetooth/nrf/bootloader/nrf_bootloader.h"
#include "../wireless/bluetooth/nrf/bootloader/nrf_bootloader_app_start.h"
#include "../wireless/bluetooth/nrf/bootloader/nrf_bootloader_info.h"
#include "../wireless/bluetooth/nrf/bootloader/dfu/nrf_dfu.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "app_error.h"
#include "app_error_weak.h"

#include "../wireless/bluetooth/nrf/ble_bsp/bsp.h"
#include <utils/app_timer.h>
#include "nrf52_gpio.h"
#include "../wireless/bluetooth/nrf/common/ble_conn_params.h"


extern timer_t m_conn_params_timer_id, m_leds_timer_id;

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
  int status;
  status = app_timer_create(&m_conn_params_timer_id, update_timeout_handler);
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

/**@brief Function for initializing the button module.
 */
static void buttons_init(void)
{
  nrf_gpio_cfg_sense_input(BOOTLOADER_BUTTON,
                           BUTTON_PULL,
                           NRF_GPIO_PIN_SENSE_LOW);
}

void *ble_stack_thread(void *unused)
{
  printf("ble_stack_thread\r\n");
  uint32_t ret_val;
  buttons_init();
  ret_val = nrf_bootloader_init();
  APP_ERROR_CHECK(ret_val);

  // Either there was no DFU functionality enabled in this project or the DFU module detected
  // no ongoing DFU operation and found a valid main application.
  // Boot the main application.
  nrf_bootloader_app_start(MAIN_APPLICATION_START_ADDR);

  // Should never be reached.
  NRF_LOG_INFO("After main\r\n");
  while (1)
    {

    }
}

void *timer_thread(void *unused)
{
  timers_init();
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
  printf("BLE DFU\r\n");

  pthread_t m_ble_stack_thread, m_timer_thread;
  pthread_create(&m_timer_thread, NULL, timer_thread, NULL);
  pthread_create(&m_ble_stack_thread, NULL, ble_stack_thread, NULL);
  pthread_join(m_ble_stack_thread, NULL);
  pthread_join(m_timer_thread, NULL);

  while (1)
    {

    }

  return 0;
}
