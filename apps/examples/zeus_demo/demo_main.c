/****************************************************************************
 * examples/nimble/nimble_demo_main.c
 *
 *   Copyright (C) 2007-2013, 2017 Gregory Nutt. All rights reserved.
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

#include <sys/stat.h>
#include <stdint.h>
#include <stdio.h>
#include <sched.h>
#include <errno.h>
#include <utils/easy_timer.h>
#include <hal/hal_bsp.h>
#include <hal/hal_timer.h>
#include "host/ble_gap.h"
#include "host/ble_att.h"
#include <sys/boardctl.h>
#include <nuttx/board.h>
#include <arch/board/board.h>
#include <fcntl.h>

#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "services/cus_inc/ble_svc_demo.h"

#include "wireless/bluetooth/ble_app_api.h"

#include <utils/zglue_fast_app_api.h>

#include <nuttx/sensors/tmp108.h>

#include "utils/step_counting.h"
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/*BLE************************************************************************/
#define BLE_ADV_MIN_ITVL           (160)               /**Uint of 0.625ms . */
#define BLE_ADV_MAX_ITVL           (200)               /**Uint of 0.625ms . */
#define BLE_INVALID_HANDLE         (0xff)

#define BLE_COMMAND_LEN            (2)

#define BLE_CMD_LED_BLINK          (0x01)
#define BLE_CMD_LED_ON             (0x02)
#define BLE_CMD_LED_OFF            (0x03)
#define BLE_CMD_MAGNETIC_ENABLE    (0x04)
#define BLE_CMD_MAGNETIC_DISABLE   (0x05)
#define BLE_CMD_HR_ENABLE          (0x06)
#define BLE_CMD_HR_DISABLE         (0x07)
#define BLE_CMD_STEP_COUNT_ENABLE  (0x08)
#define BLE_CMD_STEP_COUNT_DISABLE (0x09)
#define BLE_CMD_TEMP_ENABLE        (0x0A)
#define BLE_CMD_TEMP_DISABLE       (0x0B)


/*TMP108*********************************************************************/
#define CONFIG_SYSTEM_TMP108_DEVNAME "/dev/temp0"

/*Set defualt config, compare mode, CR is 1Hz
 *
 *BYTE 1
 *D7  D6   D5   D4   D3   D2   D1   D0
 *ID  CR1  CR0  FH   FL   TM   M1   M0
 * 0    1    1   0    0    0    1    0
 *
 *BYTE 2
 *POL --   HYS1 HYS0  --   --   --   --
 * 0   0      0    1  0     0    0    0
 *
 *
 *  CR1 CR0: 00[0.25HZ], 01[1HZ(default)], 02[4HZ], 03[16HZ]
 *
 *  HYS1 HYS0: 00[0], 01[1(default)], 02[2], 03[4]
 */
#define TMP108_CONFIG_VALUE       (0x1062)


/*Step counter***************************************************************/
#define STEP_CONNTER_DEV_PATH "/dev/accel0"

/*DEMO**********************************************************************/
#define DEMO_TASK_PRIORIT         (100)
#define DEMO_TASK_SIZE            (1024)
#define DEMO_TASK_SLEEP_TIME      (1000*500)                      /**Uint of us*/

#define DEMO_MAGNETIC_ENABLE      (0x01)
#define DEMO_TEMPERATURE_ENABLE   (0x02)
#define DEMO_STEP_COUNTER_ENABLE  (0x04)
#define DEMO_BLE_NOTIFY_ENABLE    (0x80)


#define DEMO_BOARD_INFO_LEN       (20)

#define DEMO_DATA_SIZE_MAX        (32)

/****************************************************************************
 * Private Types
 ****************************************************************************/
enum demo_data_type
{
  DEMO_BLE_DATA_TEMP = 0,
  DEMO_BLE_DATA_MAG,
  DEMO_BLE_DATA_SC,
  DEMO_BLE_DATA_HR,
  DEMO_BLE_DATA_TYPE_MAX,
};
/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void ble_read_handler(uint8_t *data, uint16_t *len);
static void ble_write_handler(uint8_t *data, uint16_t len);
static void ble_reset_handler(void);

struct os_task demo_task_handle;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/*BLE************************************************************************/
static void demo_sync_cb(void);
static uint16_t demo_conn_handle = BLE_INVALID_HANDLE;
static ble_demo_handler demo_handler =
{
  .read = &ble_read_handler,
  .write = &ble_write_handler,
  .reset = &ble_reset_handler,
};


/*TMP108*********************************************************************/
static int tmp108_fd;


/*Step counter***************************************************************/
#ifdef CONFIG_STEPCNT_ALGO
static stepCNT_t sc_handle = NULL;
#endif

/*DEMO**********************************************************************/
static uint8_t demo_features_flag = 0;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
#if (BLETEST_THROUGHPUT_TEST == 1)
void bletest_completed_bytes(uint16_t handle, uint8_t len, bool retrans)
{
  return;
}
#endif

#ifdef CONFIG_EXAMPLES_DEMO_MAGNETIC
extern void mag_dev_open(void);
extern void mag_dev_close(void);
extern void mag_dev_enable(void);
extern void mag_dev_disable(void);
extern void mag_dev_read(void);
extern void mag_angel_caculate(uint8_t *data);
extern void mag_calibrate(void);
#endif
/****************************************************************************
 * Private Functions
 ****************************************************************************/
#if 0
static void easy_timer_cb(union sigval value)
{
  ble_svc_trans_ntf_val((uint8_t *)&ntf_val, ntf_bytes);
}
#endif

static void print_conn_info(uint16_t conn_handle)
{
  int rc;
  struct ble_gap_conn_desc out_desc;
  rc = ble_gap_conn_find(conn_handle, &out_desc);
  if (rc != 0)
    {
      APP_LOG_WARNING("Find conn failed... = %x\n", rc);
    }

  APP_LOG_WARNING("Connection info: handle=%x,itvl=%x,latency=%x,mca=%x,role=%x,state=%x,timeout=%x\n",
                  out_desc.conn_handle,
                  out_desc.conn_itvl,
                  out_desc.conn_latency,
                  out_desc.master_clock_accuracy,
                  out_desc.role,
                  out_desc.sec_state,
                  out_desc.supervision_timeout
                 );

}

static void ble_read_handler(uint8_t *data, uint16_t *len)
{

#ifdef CONFIG_BOARDCTL_IOCTL
  boardctl(BOARDIOC_G_VERSION, (uintptr_t)data);
  APP_LOG_INFO("FW_VERSION: %s, len = %d\n", &data[0], DEMO_BOARD_INFO_LEN);
  *len += DEMO_BOARD_INFO_LEN;

  boardctl(BOARDIOC_G_MODEL, (uintptr_t)&data[DEMO_BOARD_INFO_LEN]);
  APP_LOG_INFO("MODEL: %s, len = %d\n", &data[DEMO_BOARD_INFO_LEN], DEMO_BOARD_INFO_LEN);
  *len += DEMO_BOARD_INFO_LEN;

  boardctl(BOARDIOC_G_DEVNAME, (uintptr_t)&data[2 * DEMO_BOARD_INFO_LEN]);
  APP_LOG_INFO("DEV: %s, len = %d\n", &data[2 * DEMO_BOARD_INFO_LEN], DEMO_BOARD_INFO_LEN);
  *len += DEMO_BOARD_INFO_LEN;
#endif

}


void ble_write_handler(uint8_t *data, uint16_t len)
{
  uint8_t *cmd;
  uint8_t led_num = FAST_LED1;
  assert(data != NULL);
  if (len <= 0 || len > BLE_COMMAND_LEN)
    {
      APP_LOG_WARNING("Demo command is not valid lenght...\n");
    }

  cmd = data;
  APP_LOG_WARNING("CMD = %x,%x,%x,%x\n", cmd[0], cmd[1], cmd[2], cmd[3]);

  switch (cmd[0])
    {
      case BLE_CMD_LED_BLINK:
        if (cmd[1] == FAST_LED1 || cmd[1] == FAST_LED2 || cmd[1] == FAST_LED3)
          {
            led_num = cmd[1];
            fast_led_blinky(led_num);
          }
        break;
      case BLE_CMD_LED_ON:
        if (cmd[1] == FAST_LED1 || cmd[1] == FAST_LED2 || cmd[1] == FAST_LED3)
          {
            led_num = cmd[1];
            fast_led_on(1, led_num);
          }
        break;
      case BLE_CMD_LED_OFF:
        if (cmd[1] == FAST_LED1 || cmd[1] == FAST_LED2 || cmd[1] == FAST_LED3)
          {
            led_num = cmd[1];
            fast_led_on(0, led_num);
          }
        break;
      case BLE_CMD_MAGNETIC_ENABLE:

#ifdef CONFIG_EXAMPLES_DEMO_MAGNETIC
        mag_dev_enable();
        demo_features_flag |= DEMO_MAGNETIC_ENABLE;
        break;
      case BLE_CMD_MAGNETIC_DISABLE:
        mag_dev_disable();
        demo_features_flag &= ~DEMO_MAGNETIC_ENABLE;
#endif
        break;
      case BLE_CMD_HR_ENABLE:
        break;
      case BLE_CMD_HR_DISABLE:
        break;

#ifdef CONFIG_STEPCNT_ALGO
      case BLE_CMD_STEP_COUNT_ENABLE:
        demo_features_flag |= DEMO_STEP_COUNTER_ENABLE;
        break;
      case BLE_CMD_STEP_COUNT_DISABLE:
        demo_features_flag &= ~DEMO_STEP_COUNTER_ENABLE;
        break;
#endif

      case BLE_CMD_TEMP_ENABLE:
        demo_features_flag |= DEMO_TEMPERATURE_ENABLE;
        break;
      case BLE_CMD_TEMP_DISABLE:
        demo_features_flag &= ~DEMO_TEMPERATURE_ENABLE;
        break;
      default:
        break;
    }

}

void ble_disconnect_handler(void)
{
  demo_features_flag = 0;
  demo_conn_handle = BLE_INVALID_HANDLE;
  ble_svc_demo_set_conn_handle(BLE_INVALID_HANDLE);
  /* Connection terminated; resume advertising. */
  demo_sync_cb();
}

void ble_reset_handler(void)
{
  /*TO DO */
}
/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that forms.
 * bleprph uses the same callback for all connections.
 *
 * @param event                 The type of event being signalled.
 * @param ctxt                  Various information pertaining to the event.
 * @param arg                   Application-specified argument; unuesd by
 *                                  bleprph.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
static int
bleprph_gap_event(struct ble_gap_event *event, void *arg)
{
  int32_t rc = 0;
  (void)rc;

  switch (event->type)
    {
      case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        APP_LOG_INFO("connection %s; status=%x ",
                     event->connect.status == 0 ? "established" : "failed",
                     event->connect.status);
        if (event->connect.status == 0)
          {
            demo_conn_handle = event->connect.conn_handle;
            ble_svc_demo_set_conn_handle(demo_conn_handle);
            print_conn_info(event->connect.conn_handle);
          }
        else
          {
            /* Connection failed; resume advertising. */
            BLE_HS_LOG(ERROR, "\nconnection wrong status\n");
            demo_sync_cb();
          }
        return 0;

      case BLE_GAP_EVENT_DISCONNECT:
        APP_LOG_WARNING("disconnect; reason=%x \n", event->disconnect.reason);
        ble_disconnect_handler();
        return 0;

      case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        APP_LOG_WARNING("connection updated; status=%x \n",
                        event->conn_update.status);
        print_conn_info(event->connect.conn_handle);
        return 0;

      case BLE_GAP_EVENT_ADV_COMPLETE:
        APP_LOG_WARNING("advertise complete; reason=%x \n",
                        event->adv_complete.reason);
        demo_sync_cb();
        return 0;

      case BLE_GAP_EVENT_ENC_CHANGE:
        /* Encryption has been enabled or disabled for this connection. */
        APP_LOG_WARNING("encryption change event; status=%d \n",
                        event->enc_change.status);
        return 0;

      case BLE_GAP_EVENT_NOTIFY_TX:
        return 0;

      case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.cur_notify == 1)
          {
            demo_features_flag |= DEMO_BLE_NOTIFY_ENABLE;
            APP_LOG_WARNING("Enable notify...\n");
          }
        else
          {
            demo_features_flag &= ~DEMO_TEMPERATURE_ENABLE;
            APP_LOG_WARNING("Disable notify...[%d]\n", event->subscribe.cur_notify);
          }
        return 0;
    }

  return 0;
}

static void demo_sync_cb(void)
{
  int rc;
  struct ble_gap_adv_params adv_params;
  struct ble_hs_adv_fields fields;
  const char *name = "zGlue_China_Sensor";
  /*This could be random or public address.*/
  /*Random address must be set before use.*/
  ble_addr_t out_addr;

  (void)name;
  (void)rc;
  memset(&fields, 0, sizeof fields);

  /* Advertise two flags:
  *     o Discoverability in forthcoming advertisement (general)
  *     o BLE-only (BR/EDR unsupported).
  */
  fields.flags = BLE_HS_ADV_F_DISC_GEN |
                 BLE_HS_ADV_F_BREDR_UNSUP;

  /* Indicate that the TX power level field should be included; have the
  * stack fill this value automatically.  This is done by assiging the
  * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
  */
#ifdef _BLE_DEV_NAME
  fields.name = (uint8_t *)_BLE_DEV_NAME;
  fields.name_len = sizeof(_BLE_DEV_NAME) - 1;
#else
  fields.name = (uint8_t *)name;
  fields.name_len = strlen(name);
#endif

  ble_svc_gap_device_name_set((const char *)fields.name);
  fields.name_is_complete = 1;

  rc = ble_gap_adv_set_fields(&fields);
  if (rc != 0)
    {
      assert(rc == 0);
      return;
    }

  memset(&adv_params, 0, sizeof adv_params);
  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  adv_params.itvl_min = BLE_ADV_MIN_ITVL;
  adv_params.itvl_max = BLE_ADV_MAX_ITVL;


  ble_hs_id_gen_rnd(false, &out_addr);
  ble_hs_id_set_rnd(out_addr.val);

  rc = ble_gap_adv_start(BLE_OWN_ADDR_RANDOM, NULL, BLE_HS_FOREVER, &adv_params, bleprph_gap_event, NULL);
  if (rc != 0)
    {
      APP_LOG_ERROR("\n BLE start ADV failed... [%d]\n", rc);
    }

  APP_LOG_WARNING("zGlue sensor ADV... \n");

  if (rc != 0)
    {
      assert(rc == 0);
    }
}


__attribute__((__noreturn__)) void
demo_task(void *arg)
{
  uint8_t temp_data[DEMO_DATA_SIZE_MAX];
  uint8_t mag_data[DEMO_DATA_SIZE_MAX];
  uint8_t sc_data[DEMO_DATA_SIZE_MAX];
  uint16_t config_val = 0;
  uint32_t steps;
  step_patt patt;
  int32_t ret = 0;

  /*Open and inicialize the tmp108 sensor.*/
  tmp108_fd  = open(CONFIG_SYSTEM_TMP108_DEVNAME, O_RDONLY);
  if (tmp108_fd < 0)
    {
      APP_LOG_ERROR("\n Open tmp108 failed... [%d]\n", tmp108_fd);
    }
  config_val = TMP108_CONFIG_VALUE;
  ret = ioctl(tmp108_fd, SNIOC_WRITECONF, (long unsigned int)config_val);
  if (ret < 0)
    {
      APP_LOG_ERROR("\n Tmp108 IOCTL SNIOC_WRITECONF failed... [%d]\n", ret);
    }

#ifdef CONFIG_EXAMPLES_DEMO_MAGNETIC
  mag_dev_open();
#endif


#ifdef CONFIG_STEPCNT_ALGO
  ret = stepCNT_start(&sc_handle, STEP_CONNTER_DEV_PATH);
  if (ret != OK)
    {
      APP_LOG_ERROR("Step counting thread start failed:%d\n", ret);
    }
#endif

  while (1)
    {
      usleep(DEMO_TASK_SLEEP_TIME);

      if (BLE_INVALID_HANDLE == demo_conn_handle)
        {
          continue;
        }
      /*Start get data and send to remote device.*/
      if (demo_features_flag & DEMO_BLE_NOTIFY_ENABLE)
        {
          if ((tmp108_fd >= 0) && (demo_features_flag & DEMO_TEMPERATURE_ENABLE))
            {
              /*Ready temperature Value.*/
              ioctl(tmp108_fd, SNIOC_GA_REGISTER_INT, (long unsigned int)SIGUSR1);
              read(tmp108_fd, &temp_data[1], sizeof(int32_t));
              /*The temp uint is 0.0625 centi-grade*/
              temp_data[0] = DEMO_BLE_DATA_TEMP;
              ble_svc_demo_send_data(temp_data, sizeof(uint32_t) + 1);
            }
#ifdef CONFIG_EXAMPLES_DEMO_MAGNETIC
          if (demo_features_flag & DEMO_MAGNETIC_ENABLE)
            {
              mag_data[0] = DEMO_BLE_DATA_MAG;
              mag_dev_read();
              mag_angel_caculate(&mag_data[1]);
              ble_svc_demo_send_data(mag_data, sizeof(double) + 1);
            }
#endif

#ifdef CONFIG_STEPCNT_ALGO
          if (demo_features_flag & DEMO_STEP_COUNTER_ENABLE)
            {
              stepCNT_total(sc_handle, &steps);
              stepCNT_motion_patt(sc_handle, &patt);
              sc_data[0] = DEMO_BLE_DATA_SC;
              sc_data[1] = patt;
              *(uint32_t *)(&sc_data[2]) = steps;
              ble_svc_demo_send_data(sc_data, sizeof(uint8_t) + sizeof(uint32_t) + 1);
            }
#endif
        }
    }
}

/****************************************************************************
 * Name: nimble_demo_main
 ****************************************************************************/

__attribute__((__noreturn__))
#ifdef CONFIG_BUILD_KERNEL
void main(int argc, FAR char *argv[])
#else
void demo_main(int argc, char *argv[])
#endif
{
  int rc = 0;
  (void)rc;

  APP_LOG_INFO("NIMBLE_DEMO_APP_START...\n");

  /*Nimble stack init.**************************************/
  ble_hs_cfg.reset_cb = NULL;
  ble_hs_cfg.sync_cb = &demo_sync_cb;
  ble_hs_cfg.gatts_register_cb = NULL;
  ble_hs_cfg.store_status_cb = NULL;
  nimble_sysinit_start();
  nimble_hal_init();
  /*Default services of nimble*/
  ble_svc_gap_init();
  ble_svc_gatt_init();
  nimble_sysinit_end();

  ble_svc_demo_init(&demo_handler);
  ble_att_set_preferred_mtu(BLE_SVC_DEMO_MAX_MTU);

  /* Initialize the host task */
  os_task_init(&demo_task_handle, "demo_task", demo_task, NULL, DEMO_TASK_PRIORIT, OS_WAIT_FOREVER, NULL, DEMO_TASK_SIZE);

  APP_LOG_WARNING("Demo APP initialized done...\n");

  while (1)
    {
      os_eventq_run(os_eventq_dflt_get());
    }
}

