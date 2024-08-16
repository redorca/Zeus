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
#define BLE_ADV_MIN_ITVL                      (160)/**Uint of 0.625ms . */
#define BLE_ADV_MAX_ITVL                      (200)/**Uint of 0.625ms . */

#define BLE_VOICE_REC_TASK_PRIO               (100)
#define BLE_VOICE_REC_TASK_STACK_SIZE         (2048)

#define BLE_VOICE_SEND_BUFFER_BYTES           (480)

#define BLE_VOICE_SEND_BUFFER_NUM             (4)

#define BLE_VOICE_HEADER_LEN                  ((BLE_VOICE_SEND_BUFFER_BYTES-44)/2+44)

#define BLE_VOICE_SEND_BUFFER_READY           (0xAB)

#define BLE_VOICE_SEND_BUFFER_DELAY           (1000*2)

#define BLE_AUDIO_MSG_Q_MAX                   (16)

#define BLE_AUDIO_SEND_PATH                   "/dev/audio/pcm510x"

#define BLE_VOICE_EVET_STATUS_SCAN_REPORT     (1)
#define BLE_VOICE_EVET_STATUS_CONNECTED       (2)
#define BLE_VOICE_EVET_STATUS_CHR_QUERY       (3)

#define BLE_VOICE_CMD_ENABLE_TRANS            (0)
#define BLE_VOICE_CMD_DISABLE_TRANS           (1)
#define BLE_VOICE_CMD_TRANS_MASK              (1)
/****************************************************************************
 * Private Types
 ****************************************************************************/
typedef struct ble_voice_c_attr_handles
{
  uint16_t read_handle;
  uint16_t write_handle;
  uint16_t ntf_handle;
} ble_voice_c_attr_handles;

typedef enum ble_voice_c_ev_status_type
{
  BLE_VOICE_C_STATUS_IDLE = 0,
  BLE_VOICE_C_STATUS_ADV_REPORTED,
  BLE_VOICE_C_STATUS_CONNECTED,
  BLE_VOICE_C_STATUS_STORE_CHAR_HANDLE,
  BLE_VOICE_C_STATUS_MAX
} ble_voice_c_ev_status_type;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static void ble_voice_c_cb(void);
static int32_t ble_voice_c_gap_event(struct ble_gap_event *event, void *arg);
static void ble_voice_c_scan_reported_handler(void);
static void ble_voice_c_connected_handler(void);
static void ble_voice_c_char_query_handler(void);
static void ble_app_event_trigger(uint8_t event_id);
static void ble_voice_c_buffer_enqueue(uint8_t *data, uint16_t len, uint8_t num);

/****************************************************************************
 * Private Data
 ****************************************************************************/
static ble_app_ev_handler voice_ev_handle_table[BLE_VOICE_C_STATUS_MAX] =
{
  NULL,
  ble_voice_c_scan_reported_handler,
  ble_voice_c_connected_handler,
  ble_voice_c_char_query_handler,
};

static ble_addr_t                remote_addr;
static ble_voice_c_attr_handles  attr_handles = {0};
static uint16_t                  ble_voice_c_conn_handle = BLE_APP_INVALID_CONN_HANDLE;
static ble_app_event_type        ble_voice_event = {0};

static mqd_t                     ble_voice_send;
static int32_t                   ble_voice_c_dev_fd;
static struct os_task            nimble_voice_c_task;
static FAR struct ap_buffer_s    *ble_voice_c_pBuffers[BLE_VOICE_SEND_BUFFER_NUM];
static uint8_t                    ble_voice_buffer_num = 0;
static uint8_t                    ble_voice_first_play = 0;
/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
#if (BLETEST_THROUGHPUT_TEST == 1)
void bletest_completed_bytes(uint16_t handle, uint8_t len, bool retrans)
{
}
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void ble_voice_c_print_conn_info(uint16_t conn_handle)
{
  int32_t rc;
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

static void ble_voice_c_cb(void)
{
  int ret;
  struct ble_gap_disc_params disc_params;

  memset(&disc_params, 0, sizeof(disc_params));

  disc_params.filter_duplicates = 0;
  disc_params.filter_policy = 0;
  disc_params.itvl = BLE_APP_SCAN_GEN_INTERVAL;
  disc_params.limited = 0;
  disc_params.passive = 1;
  disc_params.window = BLE_APP_SCAN_GEN_WINDOW;

  ret = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &disc_params, ble_voice_c_gap_event, NULL);
  ASSERT(0 == ret);
  APP_LOG_WARNING("BLE_DISCOVING....[%d]\n", ret);
}

int ble_voice_c_mtu_fn(uint16_t conn_handle,
                       const struct ble_gatt_error *error,
                       uint16_t mtu, void *arg)
{
  return 0;
}

static int ble_voice_c_chr_fn(uint16_t conn_handle,
                              const struct ble_gatt_error *error,
                              const struct ble_gatt_chr *chr, void *arg)
{
  if (chr == 0)
    {
      return 0;
    }

  APP_LOG_INFO(" CHAR_CB: conn_handle[%x]--char[%x]=[def_h=%x, val_h=%x, prop=%x]\n",
               conn_handle,
               chr,
               chr->def_handle,
               chr->val_handle,
               chr->properties);

  if (0 == ble_uuid_cmp((ble_uuid_t *)&chr->uuid, (ble_uuid_t *)&gatt_svr_chr_voice_read_uuid))
    {
      attr_handles.read_handle = chr->val_handle;
    }
  else if (0 == ble_uuid_cmp((ble_uuid_t *)&chr->uuid, (ble_uuid_t *)&gatt_svr_chr_voice_write_uuid))
    {
      attr_handles.write_handle = chr->val_handle;
    }
  else if (0 == ble_uuid_cmp((ble_uuid_t *)&chr->uuid, (ble_uuid_t *)&gatt_svr_chr_voice_ntf_uuid))
    {
      attr_handles.ntf_handle = chr->val_handle;
    }
  else
    {
    }

  return 0;
}

int ble_voice_c_gatt_read_periph_info_fn(uint16_t conn_handle,
                                         const struct ble_gatt_error *error,
                                         struct ble_gatt_attr *attr,
                                         void *arg)
{
  int len;
  uint8_t data[BLE_VOICE_SEND_BUFFER_BYTES];

  len = OS_MBUF_PKTLEN(attr->om);

  os_mbuf_copydata(attr->om, 0, len, data);

  APP_LOG_INFO("\n data[%s] len[%d] \n", data, len);
  return 0;
}

static void ble_voice_c_connected_handler(void)
{
  int ret;
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
  ret = ble_gap_read_le_phy(ble_voice_c_conn_handle, &tx_phy, &rx_phy);
  if (0 != ret)
    {
      APP_LOG_ERROR( "BLE PHY setting read failed[%x]...\n", ret);
    }

  ret = ble_gattc_exchange_mtu(ble_voice_c_conn_handle, ble_voice_c_mtu_fn, NULL);
  if (0 != ret)
    {
      APP_LOG_ERROR( "BLE set exchange MTU failed[%x]...\n", ret);
    }

  ret = ble_gap_set_prefered_le_phy(ble_voice_c_conn_handle, BLE_GAP_LE_PHY_2M_MASK,
                                    BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_CODED_ANY);
  if (0 != ret)
    {
      APP_LOG_ERROR( "BLE set phy rate failed[%x]...\n", ret);
    }

  ret = ble_hs_hci_util_set_data_len(ble_voice_c_conn_handle, BLE_HCI_SET_DATALEN_TX_OCTETS_MAX, BLE_HCI_SET_DATALEN_TX_TIME_MAX);
  if (0 != ret)
    {
      APP_LOG_ERROR( "BLE set data len failed[%x]...\n", ret);
    }

  /*Go through all the CHARs and save it for next operations. */
  ret = ble_gattc_disc_all_chrs(ble_voice_c_conn_handle,
                                BLE_APP_GATT_HANDLE_START,
                                BLE_APP_GATT_HANDLE_END,
                                ble_voice_c_chr_fn,
                                NULL);
  if (0 != ret)
    {
      APP_LOG_ERROR("\n BLE GE char ERR [%d]\n");
      ble_gap_terminate(ble_voice_c_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
  else
    {
      sleep(BLE_APP_GATT_GET_ALL_CHAR_DEAY);
      ble_app_event_trigger(BLE_VOICE_EVET_STATUS_CHR_QUERY);
    }
}

static void ble_voice_c_scan_reported_handler(void)
{
  int ret;
  struct ble_gap_conn_params conn_params;
  conn_params.scan_itvl = BLE_APP_SCAN_GEN_INTERVAL;
  conn_params.scan_window = BLE_APP_SCAN_GEN_WINDOW;
  conn_params.itvl_min = BLE_APP_CONN_MIN_INTERVAL;
  conn_params.itvl_max = BLE_APP_CONN_MIN_INTERVAL;
  conn_params.latency = BLE_GAP_INITIAL_CONN_LATENCY;
  conn_params.supervision_timeout = BLE_GAP_INITIAL_SUPERVISION_TIMEOUT;
  conn_params.min_ce_len = BLE_GAP_INITIAL_CONN_MIN_CE_LEN;
  conn_params.max_ce_len = BLE_GAP_INITIAL_CONN_MAX_CE_LEN;
  ble_gap_disc_cancel();
  ret = ble_gap_connect(BLE_OWN_ADDR_PUBLIC,
                        &remote_addr,
                        BLE_HS_FOREVER,
                        &conn_params,
                        ble_voice_c_gap_event, NULL);

  if (0 != ret)
    {
      APP_LOG_ERROR( "BLE Connect failed, err = %d\n", ret);
      ble_voice_c_cb();
    }
}


static void ble_voice_c_char_query_handler(void)
{
  int32_t ret;

  uint8_t data[2];

  if (attr_handles.ntf_handle == 0 || attr_handles.read_handle == 0 || attr_handles.ntf_handle == 0)
    {
      APP_LOG_ERROR("Ble didn't get the right char handle...\n");
      assert(0);
    }

#if 0
  ret = ble_gattc_read(ble_voice_c_conn_handle, attr_handles.read_handle, ble_voice_c_gatt_read_periph_info_fn, NULL);
  if (0 != ret)
    {
      APP_LOG_ERROR("\n BLE read handle failed, err = [%d]\n", ret);
      ble_gap_terminate(ble_voice_c_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
#endif
  data[0] = 0x01;
  data[1] = 0x00;
  ret = ble_gattc_write_no_rsp_flat(ble_voice_c_conn_handle, attr_handles.write_handle, data, 2);

  if (0 != ret)
    {
      APP_LOG_ERROR("BLE write handle failed, err = [%d]\n", ret);
    }
  else
    {
      APP_LOG_ERROR("BLE write handle OK\n");
    }

}

void ble_app_event_trigger(uint8_t event_id)
{
  //Ensure we will not miss any or overwrite any event.
  if (ble_voice_event.event_id != BLE_APP_EVET_STATUS_IDLE)
    {
      assert(ble_voice_event.event_id == BLE_APP_EVET_STATUS_IDLE);
    }
  ble_voice_event.event_id = event_id;
  ble_npl_eventq_put((struct ble_npl_eventq *)os_eventq_dflt_get(), &ble_voice_event.event_handle);
}

void ble_app_event_handler(struct ble_npl_event *ev)
{
  uint8_t event_id = 0;

  if (ble_voice_event.event_id == 0)
    {
      assert(ble_voice_event.event_id != 0);
    }
  event_id = ble_voice_event.event_id;

  //Clear event before excute hanler.
  ble_voice_event.event_id = BLE_APP_EVET_STATUS_IDLE;
  if (NULL != voice_ev_handle_table[event_id])
    {
      voice_ev_handle_table[event_id]();
    }
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
static int32_t ble_voice_c_gap_event(struct ble_gap_event *event, void *arg)
{
  int ret;

  switch (event->type)
    {
      case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        APP_LOG_INFO( "\nConnection EVENT...\n");
        APP_LOG_INFO( "\n handle %d; status=%x \n", event->connect.conn_handle, event->connect.status);

        if (event->connect.status == 0)
          {
            ble_voice_c_conn_handle = event->connect.conn_handle;
            ble_voice_c_print_conn_info(event->connect.conn_handle);
            ble_app_event_trigger(BLE_VOICE_EVET_STATUS_CONNECTED);
          }

        return 0;

      case BLE_GAP_EVENT_DISCONNECT:
        APP_LOG_INFO( "disconnect; reason=%x \n", event->disconnect.reason);
        ble_voice_c_cb();
        return 0;

      case BLE_GAP_EVENT_CONN_UPDATE:
        APP_LOG_INFO( "connection updated; status=%x \n",
                      event->conn_update.status);
        ble_voice_c_print_conn_info(event->connect.conn_handle);
        return 0;

      case BLE_GAP_EVENT_NOTIFY_RX:
        {
          uint16_t len;
          uint8_t *data;
          len = OS_MBUF_PKTLEN(event->notify_rx.om);
          if (BLE_VOICE_HEADER_LEN == len)
            {
              APP_LOG_WARNING( "Receive voice header len=%d\n", len);
              len = len - 2;
            }
          if ((len != BLE_VOICE_SEND_BUFFER_BYTES / 2) && (len != BLE_VOICE_HEADER_LEN))
            {
              APP_LOG_ERROR( "NOTIFY error len=%d\n", len);
            }
          data = malloc(BLE_VOICE_SEND_BUFFER_BYTES);
          if (data == NULL)
            {
              APP_LOG_ERROR( "Voice buffer Malloc data failed\n");
              return 0;
            }
          ret = os_mbuf_copydata(event->notify_rx.om, 0, len, data);
          if (ble_voice_first_play < BLE_VOICE_SEND_BUFFER_NUM)
            {
              static struct audio_buf_desc_s   buf_desc;
              memcpy(ble_voice_c_pBuffers[ble_voice_first_play]->samp, data, len);
              ble_voice_c_pBuffers[ble_voice_first_play]->nmaxbytes = BLE_VOICE_SEND_BUFFER_BYTES;
              ble_voice_c_pBuffers[ble_voice_first_play]->nbytes = len;
              ble_voice_c_pBuffers[ble_voice_first_play]->curbyte = 0;
              ble_voice_c_pBuffers[ble_voice_first_play]->flags   = 0;

              buf_desc.numbytes = BLE_VOICE_SEND_BUFFER_BYTES;
              buf_desc.u.pBuffer = ble_voice_c_pBuffers[ble_voice_first_play];
              ret = ioctl(ble_voice_c_dev_fd, AUDIOIOC_ENQUEUEBUFFER, (unsigned long) &buf_desc);
              if (ret < 0)
                {
                  int errcode = errno;
                  assert(errcode > 0);

                  APP_LOG_ERROR("ERROR: AUDIOIOC_ENQUEUEBUFFER ioctl failed... %d\n", errcode);
                }
              up_putc('0' + ble_voice_first_play);
              ble_voice_first_play++;
            }
          else
            {
              ble_voice_c_buffer_enqueue(data, len, ble_voice_buffer_num);
              ble_voice_buffer_num = (ble_voice_buffer_num + 1) % BLE_VOICE_SEND_BUFFER_NUM;
            }
          free(data);
        }
        return 0;
      case BLE_GAP_EVENT_DISC:
        {
          const struct ble_hs_adv_field *field;

          ret = ble_hs_adv_find_field(BLE_HS_ADV_TYPE_MFG_DATA, event->disc.data,
                                      event->disc.length_data, &field);
          if (ret == 0)
            {
              uint16_t company_id;
              uint16_t val;
              company_id = *((uint16_t *)&field->value[0]);
              val = *((uint16_t *)&field->value[2]);
              APP_LOG_WARNING( "\nCompany id = %x, val = %x\n", company_id, val);
              if ((company_id == BLE_APP_MFG_Z_COMPANY_ID) && (val == BLE_APP_MFG_VOICE_ID))
                {
                  remote_addr = event->disc.addr;
                  ble_app_event_trigger(BLE_VOICE_EVET_STATUS_SCAN_REPORT);
                }
            }
        }
        return 0;

      case BLE_GAP_EVENT_DISC_COMPLETE:
        APP_LOG_INFO( "\nBLE_GAP_EVENT_DISC_COMPLETE_______________\n");
        return 0;

      case BLE_GAP_EVENT_REPEAT_PAIRING:
        APP_LOG_INFO( "\nBLE_GAP_EVENT_REPEAT_PAIRING_______________\n");
        return 0;
      case BLE_GAP_EVENT_MTU:
        APP_LOG_WARNING("mtu update event; conn_handle=%d cid=%d mtu=%d\n",
                        event->mtu.conn_handle,
                        event->mtu.channel_id,
                        event->mtu.value);
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


static void ble_voice_c_dev_init(void)
{
  int32_t errcode;

  /*1. open audio devices*/
  ble_voice_c_dev_fd = open(BLE_AUDIO_SEND_PATH, O_RDWR);
  if (ble_voice_c_dev_fd == BLE_APP_INVALID_FD)
    {
      errcode = errno;
      DEBUGASSERT(errcode > 0);
      APP_LOG_ERROR("ERROR: Failed to open %s: %d\n", -errcode);
      UNUSED(errcode);
    }
}

static void ble_voice_c_buffer_init(void)
{
  int32_t i, ret;
  static struct audio_buf_desc_s   buf_desc;

  for (i = 0; i < BLE_VOICE_SEND_BUFFER_NUM; i++)
    {
      ble_voice_c_pBuffers[i] = NULL;
    }

  /*Init audio ble_voice_c_pBuffers from audio.*/
  for (i = 0; i < BLE_VOICE_SEND_BUFFER_NUM; i++)
    {
      memset(&buf_desc, 0, sizeof(struct audio_buf_desc_s));
      /* Fill in the buffer descriptor struct to issue an alloc request */
      buf_desc.numbytes = BLE_VOICE_SEND_BUFFER_BYTES;
      buf_desc.u.ppBuffer = &ble_voice_c_pBuffers[i];
      ret = ioctl(ble_voice_c_dev_fd, AUDIOIOC_ALLOCBUFFER, (unsigned long) &buf_desc);
      if (ret != sizeof(buf_desc))
        {
          /* Buffer alloc Operation not supported or error allocating! */
          APP_LOG_ERROR("ERROR: Could not allocate buffer %d, ret = %d\n", i, ret);
        }
    }
  /*Put buffer into audio, make audio continue output.*/
  for (i = 0; i < BLE_VOICE_SEND_BUFFER_NUM; i++)
    {
      memset(&buf_desc, 0, sizeof(struct audio_buf_desc_s));
      /* Fill in the buffer descriptor struct to issue an alloc request */
      buf_desc.numbytes = BLE_VOICE_SEND_BUFFER_BYTES;
      buf_desc.u.pBuffer = ble_voice_c_pBuffers[i];

      memset(ble_voice_c_pBuffers[i]->samp, 0, ble_voice_c_pBuffers[i]->nmaxbytes);
      ble_voice_c_pBuffers[i]->nmaxbytes = BLE_VOICE_SEND_BUFFER_BYTES;
      ble_voice_c_pBuffers[i]->nbytes = 0;
      ble_voice_c_pBuffers[i]->curbyte = 0;
      ble_voice_c_pBuffers[i]->flags = BLE_VOICE_SEND_BUFFER_READY;
    }

}

static void ble_voice_c_buffer_enqueue(uint8_t *data, uint16_t len, uint8_t num)
{
  int ret;
  static struct audio_buf_desc_s   buf_desc;

  assert(num <= BLE_VOICE_SEND_BUFFER_NUM);
  memset(&buf_desc, 0, sizeof(struct audio_buf_desc_s));
  DEBUG_GPIO_TOGGLE(36, 2);
  while (ble_voice_c_pBuffers[num]->flags != BLE_VOICE_SEND_BUFFER_READY)
    {
      usleep(BLE_VOICE_SEND_BUFFER_DELAY);
      DEBUG_GPIO_TOGGLE(36, 3);
      //up_putc('w');
    }

  buf_desc.numbytes = BLE_VOICE_SEND_BUFFER_BYTES;
  buf_desc.u.pBuffer = ble_voice_c_pBuffers[num];

  memcpy(ble_voice_c_pBuffers[num]->samp, data, len);
  ble_voice_c_pBuffers[num]->nmaxbytes = BLE_VOICE_SEND_BUFFER_BYTES;
  ble_voice_c_pBuffers[num]->nbytes = len;
  ble_voice_c_pBuffers[num]->curbyte = 0;
  ble_voice_c_pBuffers[num]->flags   = 0;
  ret = ioctl(ble_voice_c_dev_fd, AUDIOIOC_ENQUEUEBUFFER, (unsigned long) &buf_desc);
  if (ret < 0)
    {
      int errcode = errno;
      assert(errcode > 0);

      APP_LOG_ERROR("ERROR: AUDIOIOC_ENQUEUEBUFFER ioctl failed... %d\n", errcode);
    }
}

static void ble_voice_c_buffer_dequeue(void *data)
{
  FAR struct ap_buffer_s *apb;
  apb = (FAR struct ap_buffer_s * )data;
  memcpy(apb->samp, 0, BLE_VOICE_SEND_BUFFER_BYTES);
  apb->nbytes = 0;
  apb->curbyte = 0;
  apb->flags = BLE_VOICE_SEND_BUFFER_READY;
  DEBUG_GPIO_TOGGLE(38, 3);
}

__attribute__((__noreturn__)) void
ble_voice_c_receive_task(void *arg)
{
  int32_t ret, prio;
  uint32_t size = 0;
  struct mq_attr attr;
  struct audio_msg_s msg;

  /*Init audio device*/
  ble_voice_c_dev_init();
  ble_voice_c_buffer_init();

  if (ble_voice_c_dev_fd == BLE_APP_INVALID_FD)
    {
      APP_LOG_ERROR("Audio dev open failed...\n");
    }

  attr.mq_maxmsg  = BLE_AUDIO_MSG_Q_MAX;
  attr.mq_msgsize = sizeof(struct audio_msg_s);
  attr.mq_curmsgs = 0;
  attr.mq_flags   = 0;

  ble_voice_send = mq_open("ble_voice_send", O_RDWR | O_CREAT, 0644, &attr);
  if (ble_voice_send == NULL)
    {
      /* Unable to open message queue! */
      int32_t errcode;
      errcode = -errno;
      APP_LOG_ERROR("ERROR: mq_open failed: %d\n", errcode);
    }
  ret = ioctl(ble_voice_c_dev_fd, AUDIOIOC_REGISTERMQ, (unsigned long) ble_voice_send);
  if (ret < 0)
    {
      int32_t errcode;
      errcode = errno;
      DEBUGASSERT(errcode > 0);
      syslog(LOG_ERR, "AUDIOIOC_ENQUEUEBUFFER ioctl failed: %d\n", errcode);
    }

  ret = ioctl(ble_voice_c_dev_fd, AUDIOIOC_RESERVE, 0);
  if (ret < 0)
    {
      APP_LOG_ERROR("ERROR: Audio IOCTRL AUDIOIOC_RESERVE Failed %d. \n", AUDIOIOC_RESERVE);
    }

  APP_LOG_ERROR("Waiting for player MSG... \n");

  while (ble_voice_first_play < BLE_VOICE_SEND_BUFFER_NUM)
    {
      usleep(1000 * 2);
    }


  up_putc('S');
  ret = ioctl(ble_voice_c_dev_fd, AUDIOIOC_START, 0);
  if (ret < 0)
    {
      int32_t errcode;
      errcode = errno;
      DEBUGASSERT(errcode > 0);
      syslog(LOG_ERR, " AUDIOIOC_ENQUEUEBUFFER ioctl failed: %d\n", errcode);
    }


  while (1)
    {
      size = mq_receive(ble_voice_send, (FAR char *)&msg, sizeof(msg), &prio);

      if (size != sizeof(msg))
        {
          APP_LOG_ERROR("Unexcept MSG[%d] send, ignore this and continune loop...\n", msg.msgId);
          continue;
        }

      switch (msg.msgId)
        {
          case AUDIO_MSG_DEQUEUE:
            ble_voice_c_buffer_dequeue(msg.u.pPtr);
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
void nimble_voice_c_main(int argc, char *argv[])
#endif
{
  int32_t rc = 0;
  (void)rc;

  DEBUG_GPIO_INIT(34, 42);

  DEBUG_GPIO_TOGGLE(36, 2);
  DEBUG_GPIO_TOGGLE(38, 3);
  DEBUG_GPIO_TOGGLE(39, 4);
  DEBUG_GPIO_TOGGLE(40, 7);
  DEBUG_GPIO_TOGGLE(42, 8);

  /*Nimble stack init.*/
  ble_hs_cfg.reset_cb = NULL;
  ble_hs_cfg.sync_cb = &ble_voice_c_cb;
  ble_hs_cfg.gatts_register_cb = NULL;
  ble_hs_cfg.store_status_cb = NULL;
  nimble_sysinit_start();
  nimble_hal_init();
  /*Default services of nimble*/
  ble_svc_gap_init();
  ble_svc_gatt_init();
  nimble_sysinit_end();

  ble_att_set_preferred_mtu(BLE_APP_MAX_DATA_LEN);

  ble_npl_event_init(&ble_voice_event.event_handle, ble_app_event_handler, NULL);

  /* Initialize the host task */
  os_task_init(&nimble_voice_c_task, "BLE_VOICE_SEND", ble_voice_c_receive_task, NULL,
               BLE_VOICE_REC_TASK_PRIO, OS_WAIT_FOREVER, NULL,
               BLE_VOICE_REC_TASK_STACK_SIZE);

  APP_LOG_WARNING("BLE VOICE central initialized done...\n");

  while (1)
    {
      os_eventq_run(os_eventq_dflt_get());
    }
}
