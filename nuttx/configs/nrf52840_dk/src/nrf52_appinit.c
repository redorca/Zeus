/****************************************************************************
 * configs/nrf52840_dk/src/nrf52_appinit.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Copyright (C) 2016-2018 Zglue. All rights reserved.
 *   Author: Levin Li <zhiqiang@zglue.com>
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

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mount.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <stdlib.h>
#include <sys/boardctl.h>

#include <nuttx/version.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/fs/nxffs.h>
#include <nuttx/i2c/i2c_master.h>


#include "nrf.h"
#include <arch/board/board.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>
#include <nuttx/drivers/drivers.h>
#include "nrf52_i2c.h"
#include "nrf52_spi.h"
#include "nrf52_tim.h"
#include "nrf52_rtc.h"
#include "nrf52_wdg.h"
#include "nrf52_pwm.h"
#include "nrf52_qdec.h"
#include "chip/nrf52_tim.h"
#include "nrf52_ppi.h"
#include "nrf52_gpiote.h"
#include "boards.h"

#ifdef CONFIG_MC3672
#include <nuttx/sensors/mc3672.h>
#endif

#ifdef CONFIG_SENSORS_TMP108
#include <nuttx/sensors/tmp108.h>
#endif

#if defined(CONFIG_SYSTEM_FAST_DRIVER)
#include <nuttx/drivers/zglue_fast.h>
#endif

#ifdef CONFIG_NRF52_ADC
#include "nrf52_adc.h"
#endif

#ifdef CONFIG_NRF52_PROCFS_DEVID
#include "nrf52_procfs.h"
#endif

#include <nuttx/timers/rtc.h>

#ifdef CONFIG_MCUBOOT
#include "bootutil/image.h"
#include "bootutil/bootutil.h"
#endif

#ifdef CONFIG_USBMONITOR
#include <nuttx/usb/usbmonitor.h>
#endif


/****************************************************************************
 * Public Functions
 ****************************************************************************/
#ifdef CONFIG_NRF52_SPI
extern void customer_spi_cs_select(void);
extern int nrf52_spi_flash_fs_initialize(void);
#endif

#ifdef CONFIG_CODE_ANALYSIS
extern int zdk_code_analysis(void);
#endif

extern int nrf52_internal_flash_fs_initialize(void);
extern int nrf52_qspi_flash_fs_init(void);
/****************************************************************************
 * Name: nrf52_i2c_register
 *
 * Description:
 *   Register one I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#ifdef CONFIG_SYSTEM_I2CTOOL
#if defined(CONFIG_NRF52_I2C0)||defined(CONFIG_NRF52_I2C1)
static void nrf52_i2c_register(int bus)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  i2c = nrf52_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
                 bus, ret);
          nrf52_i2cbus_uninitialize(i2c);
        }
    }
}
#endif
#endif


/****************************************************************************
 * Name: nrf52_i2ctool
 *
 * Description:
 *   Register I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#ifdef CONFIG_SYSTEM_I2CTOOL
static void nrf52_i2ctool(void)
{
  return;
#ifdef CONFIG_NRF52_I2C0
  nrf52_i2c_register(0);
#endif
#ifdef CONFIG_NRF52_I2C1
  nrf52_i2c_register(1);
#endif
}
#else
#  define nrf52_i2ctool()
#endif

int nrf52_proc_fs_initialize(void)
{
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = mount(NULL, PROCFS_MOUNT, PROCFS_TYPE, 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at  %s: %d\n",
             PROCFS_MOUNT, ret);
    }

#ifdef CONFIG_NRF52_PROCFS_DEVID
  ret |= devid_procfs_register();
#endif

#endif

#ifdef CONFIG_FS_TMPFS
  /* Mount the procfs file system */

  ret |= mount(NULL, TMPFS_MOUNT, TMPFS_TYPE, 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount tmpfs at  %s: %d\n",
             TMPFS_MOUNT, ret);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initalization logic and the the
 *         matching application logic.  The value cold be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
  int ret = OK;

  nrf52_internal_flash_fs_initialize();


#ifdef CONFIG_CODE_ANALYSIS
  zdk_code_analysis();
#endif

  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to do nrf clock init: %d\n",
             ret);
      return ret;
    }

#ifdef CONFIG_NRF52_GPIOTE
  ret = nrf_drv_gpiote_init();
  if (ret != 0)
    {
      snerr("ERROR: GPIOTE Init error\n");
      return -ENODEV;

    }
#endif

#ifdef CONFIG_COUNTER
#ifdef CONFIG_NRF52_RTC0
  ret = nrf52_rtc_initialize("/dev/rtc0", 0);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the rtc driver: %d\n",
             ret);
      return ret;
    }
#endif
#ifdef CONFIG_NRF52_RTC1
  ret = nrf52_rtc_initialize("/dev/rtc1", 1);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the rtc driver: %d\n",
             ret);
      return ret;
    }
#endif
#ifdef CONFIG_NRF52_RTC2
  ret = nrf52_rtc_initialize("/dev/rtc2", 2);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the rtc driver: %d\n",
             ret);
      return ret;
    }
#endif
#endif

#ifdef CONFIG_TIMER
  /* Initialize and register the timer driver */

#ifdef CONFIG_NRF52_TIM0
  FAR struct nrf52_tim_dev_s *tim0;
  tim0 = nrf52_timer_initialize("/dev/timer0", 0);
  if (tim0 == NULL)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the timer driver: %d\n",
             -ENODEV);
      return -ENODEV;
    }
#endif

#ifdef CONFIG_NRF52_TIM1
  FAR struct nrf52_tim_dev_s *tim1;
  tim1 = nrf52_timer_initialize("/dev/timer1", 1);
  if (tim1 == NULL)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the timer driver: %d\n",
             -ENODEV);
      return -ENODEV;
    }
#endif

#endif

#if defined(CONFIG_SYSTEM_FAST_DRIVER)
  struct zg_fast_driver_s fastapi_driver;
  struct zg_fast_config_s fastapi_config;
  memset(&fastapi_driver, 0, sizeof(struct zg_fast_driver_s));
  memset(&fastapi_config, 0, sizeof(struct zg_fast_config_s));
#endif

#ifdef CONFIG_I2C

#ifdef CONFIG_NRF52_I2C0
  FAR struct i2c_master_s *i2c0;
  /* Initialize I2C */
  i2c0 = nrf52_i2cbus_initialize(0);
  if (!i2c0)
    {
      syslog(LOG_ERR, "ERROR: Failed to init the i2c0 driver\n");
      return -ENODEV;
    }
#ifdef CONFIG_SYSTEM_I2CTOOL
  /* Register I2C drivers on behalf of the I2C tool */
  i2c_register(i2c0, 0);
#endif
#endif//CONFIG_NRF52_I2C0

#ifdef CONFIG_NRF52_I2C1
  FAR struct i2c_master_s *i2c1;
  /* Initialize I2C */
  i2c1 = nrf52_i2cbus_initialize(1);
  if (!i2c1)
    {
      syslog(LOG_ERR, "ERROR: Failed to init the i2c1 driver\n");
      return -ENODEV;
    }

#ifdef CONFIG_SYSTEM_I2CTOOL
  /* Register I2C drivers on behalf of the I2C tool */
  i2c_register(i2c1, 1);
#endif

#if defined(CONFIG_SENSORS_TMP108)
  /* Then register the accelerometer sensor */
  ret = tmp108_register("/dev/temp0", i2c1, CONFIG_TMP108_I2C_ADDR);
  if (ret < 0)
    {
      snerr("ERROR: Error registering tmp108\n");
      return -ENODEV;
    }
#endif
#if defined(CONFIG_FAST_I2C)
  fastapi_driver.i2c = i2c1;
#endif


#endif//CONFIG_NRF52_I2C1
#endif//CONFIG_I2C

#ifdef CONFIG_SPI

#ifdef CONFIG_NRF52_SPI
  customer_spi_cs_select();
#endif

#if defined(CONFIG_NRF52_SPI0)
  FAR struct spi_dev_s *spi0;

#ifdef CONFIG_NRF52_SPI0
  /* Initialize SPI0 */
  spi0 = nrf52_spibus_initialize(0, true);
  if (!spi0)
    {
      return -ENODEV;
    }
#endif

#endif /* CONFIG_NRF52_SPI0 */
#endif /* CONFIG_SPI */

#if defined(CONFIG_SYSTEM_FAST_DRIVER)
  /* Then register the FAST API device */
  ret = fast_register(&fastapi_driver, &fastapi_config);
  if (ret < 0)
    {
      snerr("ERROR: Error registering the FASTAPI DEVICE\n");
      return -ENODEV;
    }
#endif


#ifdef CONFIG_NRF52_PPI
  ret = nrf_drv_ppi_init();
  if (ret != 0)
    {
      snerr("ERROR: PPI Init error\n");
      return -ENODEV;

    }
#endif

#ifdef CONFIG_NRF52_EXAMPLES_GPIOTE_PPI
  uint32_t compare_evt_addr_0 = 0, compare_evt_addr_1 = 0, compare_evt_addr_2 = 0;
  compare_evt_addr_0 = nrf52_timer_event_address_get(tim0, NRF_TIMER_EVENT_COMPARE0);
  compare_evt_addr_1 = nrf52_timer_event_address_get(tim0, NRF_TIMER_EVENT_COMPARE1);
  compare_evt_addr_2 = nrf52_timer_event_address_get(tim0, NRF_TIMER_EVENT_COMPARE2);
#endif

#ifdef CONFIG_NRF52_EXAMPLES_GPIOTE_PPI
  nrf_ppi_channel_t ppi_channel_0 = NRF_PPI_CHANNEL0, ppi_channel_1 = NRF_PPI_CHANNEL0, ppi_channel_2 = NRF_PPI_CHANNEL0;
  uint32_t gpiote_task_addr_0 = 0;
  nrf_drv_gpiote_out_config_t config = GPIOTE_CONFIG_OUT_TASK_TOGGLE(false);
  nrf_drv_gpiote_out_init(LED_1, &config);
  gpiote_task_addr_0 = nrf_drv_gpiote_out_task_addr_get(LED_1);
  nrf_drv_ppi_channel_alloc(&ppi_channel_0);
  nrf_drv_ppi_channel_alloc(&ppi_channel_1);
  nrf_drv_ppi_channel_alloc(&ppi_channel_2);
  nrf_drv_ppi_channel_assign(ppi_channel_0, compare_evt_addr_0, gpiote_task_addr_0);
  nrf_drv_ppi_channel_enable(ppi_channel_0);
  nrf_drv_ppi_channel_assign(ppi_channel_1, compare_evt_addr_1, gpiote_task_addr_0);
  nrf_drv_ppi_channel_enable(ppi_channel_1);
  nrf_drv_ppi_channel_assign(ppi_channel_2, compare_evt_addr_2, gpiote_task_addr_0);
  nrf_drv_ppi_channel_enable(ppi_channel_2);

  nrf_drv_gpiote_out_task_enable(LED_1);
#endif



#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Configure CPU load estimation */

  cpuload_initialize_once();
#endif

#ifdef CONFIG_SYSTEM_I2CTOOL
  /* Register I2C drivers on behalf of the I2C tool */
  nrf52_i2ctool();
#endif

#ifdef CONFIG_WATCHDOG
#ifdef CONFIG_NRF52_WDG

  nrf52_wdg_initialize("/dev/watchdog0", WDG_BEHAVIOUR_Run, WDG_BEHAVIOUR_Pause);

#endif
#endif

#ifdef CONFIG_PWM
#ifdef CONFIG_NRF52_PWM
  struct pwm_pinmux_t pwm_pinmux[] =
  {
    {BOARD_PWM0_CHAN_1_PIN, 1},
    {BOARD_PWM0_CHAN_2_PIN, 2},
    {BOARD_PWM0_CHAN_3_PIN, 3},
    {BOARD_PWM0_CHAN_4_PIN, 4},
  };
  ret = nrf52_pwm_initialize(0, pwm_pinmux,
                             sizeof(pwm_pinmux) / sizeof(struct pwm_pinmux_t));
  if (ret < 0)
    {
      serr("ERROR: PWM initialize Fail\n");
    }
#endif
#endif

#ifdef CONFIG_QDECODER
#ifdef CONFIG_NRF52_QDECODER
  ret = nrf52_qdec_initialize("/dev/qdec0");
  if (ret < 0)
    {
      serr("ERROR: PWM initialize Fail\n");
    }
#endif
#endif

  nrf52_proc_fs_initialize();

#ifdef CONFIG_NRF52_SPI
  nrf52_spi_flash_fs_initialize();
#endif

#ifdef CONFIG_NRF52_QSPI
  nrf52_qspi_flash_fs_init();
#endif

#ifdef CONFIG_NRF52_ADC
  adc_config_t adc_config;
  adc_channel_config_t channel_config[CONFIG_NRF52_ADC_CHANNEL];

  /*setup adc channel config */

  for (int i = 0; i < CONFIG_NRF52_ADC_CHANNEL; i++)
    {
      channel_config[i].channel    = i;
      channel_config[i].resistor_p = ADC_RESISTOR_BYPASS;
      channel_config[i].resistor_n = ADC_RESISTOR_BYPASS;
      channel_config[i].gain       = ADC_GAIN_1_4;
      channel_config[i].ref        = ADC_REFERENCE_VDD1_4;
      channel_config[i].time       = ADC_TIME_20US;
      channel_config[i].mode       = ADC_CHANNEL_MODE_SINGLE_END;

      /* analog input pin : Nodirc DK board , analog ping from AIN2(GPIO03) start */

      channel_config[i].pin_p = i + ADC_INPUT_PIN_2;
      channel_config[i].pin_n = 0;
    }

  adc_config.oversample = ADC_OVERSAMPLE_BYPASS;
  adc_config.resolution = ADC_RESOLUTION_14BIT;
  adc_config.mode       = ADC_MODE_ONE_SHOT;

  ret = nrf52_adc_initialize(&adc_config, channel_config, CONFIG_NRF52_ADC_CHANNEL);

  if (ret < 0)
    {
      serr("ERROR: ADC initialize Fail\n");
    }
#endif

#ifdef CONFIG_COMP
  ret = nrf52_comp_initialize();

  if (ret < 0)
    {
      serr("ERROR: comparator initialize Fail\n");
    }

#endif

#if (CONFIG_FLASH_ORIGIN != 0)
  /* confirm slot 0 as permanet */
  ret = boot_set_confirmed();
  _warn("Confirm Slot-0 as Permanet Image : %d.\n", ret);
#endif

#if (defined CONFIG_AUDIO_PDM_MIC) && (defined CONFIG_NRF52_PDM)
  ret = nrf52_pdm_microphone_initialize();
  if (ret < 0)
    {
      serr("ERROR: pdm microphone initialize Fail\n");
    }
#endif


#if (defined CONFIG_AUDIO_PCM510X) && (defined CONFIG_NRF52_I2S)
  ret = nrf52_pcm510x_initialize();
  if (ret < 0)
    {
      serr("ERROR: pcm510x initialize Fail\n");
    }
#endif

#ifdef CONFIG_BMM150
  ret = nrf52_bmm150_initialize();
  if (ret < 0)
    {
      serr("ERROR: bmm150 initialize Fail\n");
    }

#endif

#ifdef CONFIG_BMI160
  ret = nrf52_bmi160_initialize();
  if (ret < 0)
    {
      serr("ERROR: bmi160 initialize Fail\n");
    }
#endif

#ifdef CONFIG_LSM6DS3
  ret = nrf52_lsm6ds3_initialize();
  if (ret < 0)
    {
      serr("ERROR: lsm6ds3 initialize Fail\n");
    }
#endif

#ifdef CONFIG_MC3672
  ret = nrf52_mc3672_initialize();
  if (ret < 0)
    {
      serr("ERROR: mc3672 initialize Fail\n");
    }

#endif

#ifdef  CONFIG_USBMONITOR
  usbmonitor_start();
#endif

#ifdef CONFIG_BOARDCTL_IOCTL
  char data[BOARDIOC_MAX_STRLEN];
  boardctl(BOARDIOC_G_VERSION, (uintptr_t)data);
  syslog(LOG_INFO, "Firmware V: %s\n", data);

  boardctl(BOARDIOC_G_MODEL, (uintptr_t)data);
  syslog(LOG_INFO, "Hardware M: %s\n", data);

  boardctl(BOARDIOC_G_DEVNAME, (uintptr_t)data);
  syslog(LOG_INFO, "Dev S/N: %s\n", data);

#endif

  return OK;
}

#ifdef CONFIG_BOARDCTL_IOCTL
int board_ioctl(unsigned int cmd, uintptr_t arg)
{
  int ret = OK;
  uint8_t uniqid[8];
  switch (cmd)
    {
      case BOARDIOC_G_VERSION:
        strncpy((char *)arg, CONFIG_VERSION_BUILD, BOARDIOC_MAX_STRLEN);
        break;
      case BOARDIOC_G_MODEL:
        strncpy((char *)arg, CONFIG_ARCH_BOARD, BOARDIOC_MAX_STRLEN);
        break;
      case BOARDIOC_G_DEVNAME:
        board_uniqueid(uniqid);
        snprintf((char *)arg, BOARDIOC_MAX_STRLEN, "%s_%02x%02x%02x%02x:%02x%02x%02x%02x", \
                 uniqid[0], uniqid[1], uniqid[2], uniqid[3], \
                 uniqid[4], uniqid[5], uniqid[6], uniqid[7]);
        break;
      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}
#endif

