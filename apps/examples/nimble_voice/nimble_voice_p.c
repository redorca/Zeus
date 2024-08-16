/****************************************************************************
 * examples/nimble/nimble_voice.c
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
#include "services/bleuart/bleuart.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "services/cus_inc/ble_svc_voice.h"


#include <stdlib.h>
#include <fcntl.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/pcm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define BLE_ADV_MIN_ITVL                    (160)/**Uint of 0.625ms . */
#define BLE_ADV_MAX_ITVL                    (200)/**Uint of 0.625ms . */
#define BLE_ADV_MFG_DATA_LEN                (4)

#define BLE_VOICE_REC_TASK_PRIO             (100)
#define BLE_VOICE_REC_TASK_STACK_SIZE       (2048)

#define BLE_VOICE_REV_BUFFER_BYTES          (480)

#define BLE_VOICE_REV_BUFFER_NUM            (8)

#define BLE_AUDIO_MSG_Q_MAX                 (16)

#define BLE_AUDIO_REV_PATH                   "/dev/audio/pdm_microphone"

#define BLE_VOICE_CMD_ENABLE_TRANS            (1)
#define BLE_VOICE_CMD_DISABLE_TRANS           (0)
#define BLE_VOICE_CMD_TRANS_MASK              (1)

#define BLE_VOICE_IS_MTU_TO_MAX               (2)
#define BLE_VOICE_IS_MTU_TO_MAX_MASK          (0x10)

#define BLE_VOICE_ATTR_FEATURE_ALL_MASK       (0x11)
/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static void ble_voice_p_cb(void);

static void ble_voice_p_read_handler(uint8_t *data, uint16_t *len);
static void ble_voice_p_write_handler(uint8_t *data, uint16_t len);
static void ble_voice_p_reset_handler(void);
static void ble_voice_p_disconn_handler(void);
/****************************************************************************
 * Private Data
 ****************************************************************************/
static ble_app_handler ble_voice_p_app_handler =
{
  .read             = ble_voice_p_read_handler,
  .write            = ble_voice_p_write_handler,
  .notify           = NULL,
  .indicate         = NULL,
  .set_conn_handle  = NULL,
  .reset            = ble_voice_p_reset_handler,
  .disconn          = ble_voice_p_disconn_handler,
};
static uint16_t                  ble_voice_p_conn_handle = BLE_APP_INVALID_CONN_HANDLE;
static uint16_t                  attr_flag = 0;

static uint16_t                  record_start = 0;

static mqd_t                     ble_voice_rev;
static int32_t                   ble_voice_p_dev_fd;
static struct os_task            nimble_voice_p_task;
static FAR struct ap_buffer_s    *ble_voice_p_pBuffers[BLE_VOICE_REV_BUFFER_NUM];
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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void ble_voice_p_read_handler(uint8_t *data, uint16_t *len)
{
  uint8_t tx_phy, rx_phy;

  ble_gap_read_le_phy(ble_voice_p_conn_handle, &tx_phy, &rx_phy);
  APP_LOG_WARNING( "Before After, MTU is[%d], PHY[tx = %d, rx = %d]...\n", ble_att_mtu(ble_voice_p_conn_handle), tx_phy, rx_phy);
  data[0] = ble_att_mtu(ble_voice_p_conn_handle);
  data[1] = tx_phy;
  data[2] = rx_phy;
  *len = 3;
}


void ble_voice_p_write_handler(uint8_t *data, uint16_t len)
{
  int32_t ret;

  assert(data != NULL);
  if (len <= 0 || len > BLE_APP_COMMAND_LEN)
    {
      APP_LOG_WARNING("Command is not valid length[%d].\n", len);
    }

  switch (data[0])
    {
      case BLE_VOICE_CMD_ENABLE_TRANS:
        attr_flag |= BLE_VOICE_CMD_TRANS_MASK;
        break;

      case BLE_VOICE_CMD_DISABLE_TRANS:
        attr_flag &= (~BLE_VOICE_CMD_TRANS_MASK);
        break;
    }
}

void ble_voice_p_reset_handler(void)
{
  /*TO DO */
}

void ble_voice_p_disconn_handler(void)
{
  assert(ble_voice_p_app_handler.set_conn_handle);
  ble_voice_p_app_handler.set_conn_handle(BLE_APP_INVALID_CONN_HANDLE);
  ble_voice_p_conn_handle = BLE_APP_INVALID_CONN_HANDLE;
  attr_flag = 0;
  /* Connection terminated; resume advertising. */
  ble_voice_p_cb();
}

static void print_conn_info(uint16_t conn_handle)
{
  int32_t ret;
  struct ble_gap_conn_desc out_desc;
  ret = ble_gap_conn_find(conn_handle, &out_desc);
  if (ret != 0)
    {
      APP_LOG_WARNING("Find conn failed... = %x\n", ret);
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
static int32_t
ble_voice_p_gap_event(struct ble_gap_event *event, void *arg)
{
  int32_t ret = 0;
  (void)ret;

  switch (event->type)
    {
      case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        APP_LOG_INFO("connection %s; status=%x ",
                     event->connect.status == 0 ? "established" : "failed",
                     event->connect.status);
        if (event->connect.status == 0)
          {
            ble_voice_p_conn_handle = event->connect.conn_handle;
            ble_voice_p_app_handler.set_conn_handle(ble_voice_p_conn_handle);
            print_conn_info(event->connect.conn_handle);
            uint8_t tx_phy, rx_phy;
            /*For big data transfer, we need support:
             *
             *1. Use 512B MTU
             *
             *2. Use 2M bit rate
             *
             *3. Use DLE
             */

            /*Read default MTU, PHY setting.*/
            ret = ble_gap_read_le_phy(ble_voice_p_conn_handle, &tx_phy, &rx_phy);
            if (0 != ret)
              {
                APP_LOG_ERROR( "BLE PHY setting read failed[%x]...\n", ret);
              }

            ret = ble_gap_set_prefered_le_phy(ble_voice_p_conn_handle, BLE_GAP_LE_PHY_2M_MASK,
                                              BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_CODED_ANY);
            if (0 != ret)
              {
                APP_LOG_ERROR( "BLE set phy rate failed[%x]...\n", ret);
              }

            ret = ble_hs_hci_util_set_data_len(ble_voice_p_conn_handle, BLE_HCI_SET_DATALEN_TX_OCTETS_MAX, BLE_HCI_SET_DATALEN_TX_TIME_MAX);
            if (0 != ret)
              {
                APP_LOG_ERROR( "BLE set data len failed[%x]...\n", ret);
              }

          }
        else
          {
            /* Connection failed; resume advertising. */
            BLE_HS_LOG(ERROR, "\nconnection wrong status\n");
            ble_voice_p_cb();
          }
        return 0;

      case BLE_GAP_EVENT_DISCONNECT:
        APP_LOG_WARNING("disconnect; reason=%x \n", event->disconnect.reason);
        ble_voice_p_app_handler.disconn();
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
        ble_voice_p_cb();
        return 0;

      case BLE_GAP_EVENT_ENC_CHANGE:
        /* Encryption has been enabled or disabled for this connection. */
        APP_LOG_WARNING("encryption change event; status=%d \n",
                        event->enc_change.status);
        return 0;

      case BLE_GAP_EVENT_NOTIFY_TX:
        APP_LOG_INFO("mtu update event; ind=%d\n", event->notify_tx.indication);
        return 0;

      case BLE_GAP_EVENT_NOTIFY_RX:
        return 0;

      case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.cur_notify == 1)
          {
            APP_LOG_WARNING("Enable notify...\n");
          }
        else
          {
            APP_LOG_WARNING("Disable notify...[%d]\n", event->subscribe.cur_notify);
          }
        return 0;

      case BLE_GAP_EVENT_MTU:
        APP_LOG_WARNING("mtu update event; conn_handle=%d cid=%d mtu=%d\n",
                        event->mtu.conn_handle,
                        event->mtu.channel_id,
                        event->mtu.value);
        if (event->mtu.value > BLE_VOICE_REV_BUFFER_BYTES)
          {
            attr_flag |= BLE_VOICE_IS_MTU_TO_MAX_MASK;
          }
        else
          {
            attr_flag &= (~BLE_VOICE_IS_MTU_TO_MAX_MASK);
          }
        return 0;

      case BLE_GAP_EVENT_PHY_UPDATE_COMPLETE:
        APP_LOG_WARNING("PHY MODE update event; status=%d tx=%d rx=%d\n",
                        event->phy_updated.status,
                        event->phy_updated.tx_phy,
                        event->phy_updated.rx_phy);

        return 0;
    }

  return 0;
}

static void ble_voice_p_cb(void)
{
  int32_t ret;
  struct ble_gap_adv_params adv_params;
  struct ble_hs_adv_fields fields;
  const char *name = "Z_ble_voice_p";
  static uint8_t mfg_data[BLE_ADV_MFG_DATA_LEN] = {0};
  /*This could be random or public address.*/
  /*Random address must be set before use.*/
  ble_addr_t out_addr;

  (void)name;
  (void)ret;
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

  *((uint16_t *)&mfg_data[0]) = BLE_APP_MFG_Z_COMPANY_ID;
  *((uint16_t *)&mfg_data[2]) = BLE_APP_MFG_VOICE_ID;
  fields.mfg_data = mfg_data;
  fields.mfg_data_len = sizeof(mfg_data);

  ret = ble_gap_adv_set_fields(&fields);
  if (ret != 0)
    {
      assert(ret == 0);
      return;
    }

  memset(&adv_params, 0, sizeof adv_params);
  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  adv_params.itvl_min = BLE_ADV_MIN_ITVL;
  adv_params.itvl_max = BLE_ADV_MAX_ITVL;


  ble_hs_id_gen_rnd(false, &out_addr);
  ble_hs_id_set_rnd(out_addr.val);

  ret = ble_gap_adv_start(BLE_OWN_ADDR_RANDOM, NULL, BLE_HS_FOREVER, &adv_params, ble_voice_p_gap_event, NULL);
  if (ret != 0)
    {
      BLE_HS_LOG(ERROR, "\nBLE VOICE ADV Start failed, err = [%d]\n", ret);
    }
  else
    {
      APP_LOG_WARNING("\nBLE VOICE ADV Start...\n");
    }
}

static void ble_voice_p_dev_init(void)
{
  int32_t errcode;

  /*1. open audio devices*/
  ble_voice_p_dev_fd = open(BLE_AUDIO_REV_PATH, O_RDWR);
  if (ble_voice_p_dev_fd == BLE_APP_INVALID_FD)
    {
      errcode = errno;
      DEBUGASSERT(errcode > 0);
      APP_LOG_ERROR("ERROR: Failed to open %s: %d\n", -errcode);
      UNUSED(errcode);
    }
}

static void ble_voice_p_buffer_init(void)
{
  int32_t i, ret;
  static struct audio_buf_desc_s   buf_desc;

  for (i = 0; i < BLE_VOICE_REV_BUFFER_NUM; i++)
    {
      ble_voice_p_pBuffers[i] = NULL;
    }

  /*Init audio ble_voice_p_pBuffers from audio.*/
  for (i = 0; i < BLE_VOICE_REV_BUFFER_NUM; i++)
    {
      memset(&buf_desc, 0, sizeof(struct audio_buf_desc_s));
      /* Fill in the buffer descriptor struct to issue an alloc request */
      buf_desc.numbytes = BLE_VOICE_REV_BUFFER_BYTES;
      buf_desc.u.ppBuffer = &ble_voice_p_pBuffers[i];
      ret = ioctl(ble_voice_p_dev_fd, AUDIOIOC_ALLOCBUFFER, (unsigned long) &buf_desc);
      if (ret != sizeof(buf_desc))
        {
          /* Buffer alloc Operation not supported or error allocating! */
          APP_LOG_ERROR("ERROR: Could not allocate buffer %d, ret = %d\n", i, ret);
        }
    }
  /*Put buffer into audio, make audio continue output.*/
  for (i = 0; i < BLE_VOICE_REV_BUFFER_NUM; i++)
    {
      memset(&buf_desc, 0, sizeof(struct audio_buf_desc_s));
      /* Fill in the buffer descriptor struct to issue an alloc request */
      buf_desc.numbytes = BLE_VOICE_REV_BUFFER_BYTES;
      buf_desc.u.pBuffer = ble_voice_p_pBuffers[i];
      ret = ioctl(ble_voice_p_dev_fd, AUDIOIOC_ENQUEUEBUFFER, (unsigned long) &buf_desc);
      if (ret < 0)
        {
          /* Buffer alloc Operation not supported or error allocating! */
          int errcode = errno;
          assert(errcode > 0);

          APP_LOG_ERROR("ERROR: AUDIOIOC_ENQUEUEBUFFER ioctl failed: %d\n", errcode);
        }
    }

}

static void ble_voice_p_buffer_dequeue(void *data)
{
  int32_t ret;
  FAR struct ap_buffer_s *apb;
  struct audio_buf_desc_s  buf_desc;

  apb = (FAR struct ap_buffer_s * )data;

  if (!(ble_voice_p_app_handler.notify &&
        (attr_flag & BLE_VOICE_ATTR_FEATURE_ALL_MASK) &&
        BLE_APP_IS_CONNECTED(ble_voice_p_conn_handle))
     )
    {
      assert(0);
    }

#if 0
  if ((apb->curbyte != 0) || (apb->nbytes != BLE_VOICE_REV_BUFFER_BYTES))
    {
      APP_LOG_ERROR("ERROR: Voice received data not integrity [%d,%d]\n", apb->curbyte, apb->nbytes);
    }
#endif

  ble_voice_p_app_handler.notify(apb->samp, apb->nbytes);


  memset(apb->samp, 0, apb->nmaxbytes);
  apb->nbytes = 0;
  apb->curbyte = 0;

  memset(&buf_desc, 0, sizeof(struct audio_buf_desc_s));
  buf_desc.numbytes = BLE_VOICE_REV_BUFFER_BYTES;
  buf_desc.u.pBuffer = apb;

  DEBUG_GPIO_TOGGLE(36, 2);
  ret = ioctl(ble_voice_p_dev_fd, AUDIOIOC_ENQUEUEBUFFER, (unsigned long)&buf_desc);
  if (ret < 0)
    {
      int errcode = errno;
      DEBUGASSERT(errcode > 0);

      APP_LOG_ERROR("ERROR: AUDIOIOC_ENQUEUEBUFFER ioctl failed: %d\n", errcode);
    }
}

__attribute__((__noreturn__)) void
ble_voice_p_receive_task(void *arg)
{
  int32_t ret, prio;
  uint32_t size = 0;
  struct mq_attr attr;
  struct audio_msg_s msg;

  /*Init audio device*/
  ble_voice_p_dev_init();
  ble_voice_p_buffer_init();

  if (ble_voice_p_dev_fd == BLE_APP_INVALID_FD)
    {
      APP_LOG_ERROR("Audio dev open failed...\n");
    }

  attr.mq_maxmsg  = BLE_AUDIO_MSG_Q_MAX;
  attr.mq_msgsize = sizeof(struct audio_msg_s);
  attr.mq_curmsgs = 0;
  attr.mq_flags   = 0;

  ble_voice_rev = mq_open("ble_voice_rev", O_RDWR | O_CREAT, 0644, &attr);
  if (ble_voice_rev == NULL)
    {
      /* Unable to open message queue! */
      int32_t errcode;
      errcode = -errno;
      APP_LOG_ERROR("ERROR: mq_open failed: %d\n", errcode);
    }
  ret = ioctl(ble_voice_p_dev_fd, AUDIOIOC_REGISTERMQ, (unsigned long) ble_voice_rev);
  if (ret < 0)
    {
      int32_t errcode;
      errcode = errno;
      DEBUGASSERT(errcode > 0);
      syslog(LOG_ERR, "AUDIOIOC_ENQUEUEBUFFER ioctl failed: %d\n", errcode);
    }

  ret = ioctl(ble_voice_p_dev_fd, AUDIOIOC_RESERVE, 0);
  if (ret < 0)
    {
      APP_LOG_ERROR("ERROR: Audio IOCTRL AUDIOIOC_RESERVE Failed %d. \n", AUDIOIOC_RESERVE);
    }

  while (!(attr_flag & BLE_VOICE_ATTR_FEATURE_ALL_MASK))
    {
      APP_LOG_ERROR("Waiting for record... \n");
      sleep(2);
    }

  ret = ioctl(ble_voice_p_dev_fd, AUDIOIOC_START, 0);
  if (ret < 0)
    {
      int32_t errcode;
      errcode = errno;
      DEBUGASSERT(errcode > 0);
      syslog(LOG_ERR, " AUDIOIOC_ENQUEUEBUFFER ioctl failed: %d\n", errcode);
    }

  while (1)
    {
      size = mq_receive(ble_voice_rev, (FAR char *)&msg, sizeof(msg), &prio);

      if (size != sizeof(msg))
        {
          APP_LOG_ERROR("Unexcept MSG[%d] received, ignore this and continune loop...\n", msg.msgId);
          continue;
        }

      switch (msg.msgId)
        {
          case AUDIO_MSG_DEQUEUE:
            if ((attr_flag & BLE_VOICE_ATTR_FEATURE_ALL_MASK) && BLE_APP_IS_CONNECTED(ble_voice_p_conn_handle))
              {
                ble_voice_p_buffer_dequeue(msg.u.pPtr);
              }
            else
              {
                APP_LOG_WARNING("Recording in unexcept status...\n");
              }
            break;

          case AUDIO_MSG_STOP:
            APP_LOG_WARNING("AUDIO_MSG_STOP...\n");
            break;

          case AUDIO_MSG_COMPLETE:
            APP_LOG_WARNING("AUDIO_MSG_COMPLETE...\n");
            break;

          default:
            APP_LOG_WARNING("AUDIO_MSG_UNKNOWN...\n");
            break;
        }
    }
}

/****************************************************************************
 * Name: nimble_uart_main
 ****************************************************************************/

__attribute__((__noreturn__))
#ifdef CONFIG_BUILD_KERNEL
void main(int argc, FAR char *argv[])
#else
void nimble_voice_p_main(int argc, char *argv[])
#endif
{
  int32_t ret = 0;
  (void)ret;

  DEBUG_GPIO_INIT(36, 36);
  DEBUG_GPIO_TOGGLE(36, 6);

  DEBUG_GPIO_INIT(39, 42);

  DEBUG_GPIO_TOGGLE(39, 1);
  DEBUG_GPIO_TOGGLE(40, 2);
  DEBUG_GPIO_TOGGLE(42, 3);

  /*Nimble stack init.*/
  ble_hs_cfg.reset_cb = NULL;
  ble_hs_cfg.sync_cb = &ble_voice_p_cb;
  ble_hs_cfg.gatts_register_cb = NULL;
  ble_hs_cfg.store_status_cb = NULL;
  nimble_sysinit_start();
  nimble_hal_init();
  /*Default services of nimble*/
  ble_svc_gap_init();
  ble_svc_gatt_init();
  nimble_sysinit_end();

  ble_svc_voice_init(&ble_voice_p_app_handler);
  ble_att_set_preferred_mtu(BLE_APP_MAX_DATA_LEN);

  /* Initialize the host task */
  os_task_init(&nimble_voice_p_task, "BLE_VOICE_RECEIVE", ble_voice_p_receive_task, NULL,
               BLE_VOICE_REC_TASK_PRIO, OS_WAIT_FOREVER, NULL,
               BLE_VOICE_REC_TASK_STACK_SIZE);

  APP_LOG_WARNING("BLE VOICE peripheral initialized done...\n");

  while (1)
    {
      os_eventq_run(os_eventq_dflt_get());
    }
}
