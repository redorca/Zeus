/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <nuttx/config.h>

#include <sys/stat.h>
#include <stdint.h>
#include <stdio.h>
#include <sched.h>
#include <errno.h>

/* BLE */
#include <hal/hal_bsp.h>
#include "host/ble_gap.h"
#include "host/ble_att.h"
#include "nimble/ble.h"
#include "host/ble_hs.h"
#include "mesh/glue.h"
#include "mesh/mesh.h"
#include "services/gap/ble_svc_gap.h"
#include <syscfg/mesh_cfg.h>
#include <utils/easy_timer.h>
#include "wireless/bluetooth/ble_app_api.h"
#include "nrf_gpio.h"


#define BT_DBG_ENABLED (MYNEWT_VAL(BLE_MESH_DEBUG))

/* Company ID */
#define CID_VENDOR 0x0800 //company id
#define PID_VENDOR 0x0001 //product id
#define VID_VENDOR 0x0100 //produckt version id
#define STANDARD_TEST_ID 0x00
#define TEST_ID 0x01
static int recent_test_id = STANDARD_TEST_ID;

#define BT_MESH_MODEL_ID_VND_SRV                0x2000
#define BT_MESH_MODEL_ID_VND_CLI                0x2001

#define BT_MESH_MODEL_LIGHT1                    (13)
#define BT_MESH_MODEL_LIGHT2                    (14)
#define BT_MESH_MODEL_LIGHT3                    (15)
#define BT_MESH_MODEL_LIGHT4                    (16)

static uint8_t vnd_fea1_state = 0xff;

#define FAULT_ARR_SIZE 2

static bool has_reg_fault = true;

#define INPUT_NUM_MIN '0'
#define INPUT_NUM_MAX '9'

#define INPUT_FILE "/dev/console"

#define BLE_MESH_TIMER 0

#if(BLE_MESH_TIMER)
static timer_t timerid = NULL;
#endif

static struct os_task blemesh_task;

static struct bt_mesh_cfg_srv cfg_srv =
{
  .relay = BT_MESH_RELAY_DISABLED,
  .beacon = BT_MESH_BEACON_ENABLED,
#if MYNEWT_VAL(BLE_MESH_FRIEND)
  .frnd = BT_MESH_FRIEND_ENABLED,
#else
  .gatt_proxy = BT_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
#if MYNEWT_VAL(BLE_MESH_GATT_PROXY)
  .gatt_proxy = BT_MESH_GATT_PROXY_ENABLED,
#else
  .gatt_proxy = BT_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
  .default_ttl = 7,

  /* 3 transmissions with 20ms interval */
  .net_transmit = BT_MESH_TRANSMIT(2, 20),
  .relay_retransmit = BT_MESH_TRANSMIT(2, 20),
};

static int
fault_get_cur(struct bt_mesh_model *model,
              uint8_t *test_id,
              uint16_t *company_id,
              uint8_t *faults,
              uint8_t *fault_count)
{
  uint8_t reg_faults[FAULT_ARR_SIZE] = { [0 ... FAULT_ARR_SIZE - 1] = 0xff };

  APP_LOG_INFO("fault_get_cur() has_reg_fault %u\n", has_reg_fault);

  *test_id = recent_test_id;
  *company_id = CID_VENDOR;

  *fault_count = min(*fault_count, sizeof(reg_faults));
  memcpy(faults, reg_faults, *fault_count);

  return 0;
}

static int
fault_get_reg(struct bt_mesh_model *model,
              uint16_t company_id,
              uint8_t *test_id,
              uint8_t *faults,
              uint8_t *fault_count)
{
  if (company_id != CID_VENDOR)
    {
      return -BLE_HS_EINVAL;
    }

  APP_LOG_INFO("fault_get_reg() has_reg_fault %u\n", has_reg_fault);

  *test_id = recent_test_id;

  if (has_reg_fault)
    {
      uint8_t reg_faults[FAULT_ARR_SIZE] = { [0 ... FAULT_ARR_SIZE - 1] = 0xff };

      *fault_count = min(*fault_count, sizeof(reg_faults));
      memcpy(faults, reg_faults, *fault_count);
    }
  else
    {
      *fault_count = 0;
    }

  return 0;
}

static int
fault_clear(struct bt_mesh_model *model, uint16_t company_id)
{
  if (company_id != CID_VENDOR)
    {
      return -BLE_HS_EINVAL;
    }

  has_reg_fault = false;

  return 0;
}

static int
fault_test(struct bt_mesh_model *model, uint8_t test_id, uint16_t company_id)
{
  if (company_id != CID_VENDOR)
    {
      return -BLE_HS_EINVAL;
    }

  if (test_id != STANDARD_TEST_ID && test_id != TEST_ID)
    {
      return -BLE_HS_EINVAL;
    }

  recent_test_id = test_id;
  has_reg_fault = true;
  bt_mesh_fault_update(bt_mesh_model_elem(model));

  return 0;
}

static void fea1_set_operation(uint8_t state)
{
  /*In this demo we try to light on/off a LED.
   * 0: light off all leds;
   * 1: light on led 1;
   * 2: light on led 2;
   * 3: light on led 3;
   * 4: light on led 4;
   */
  switch (state)
    {
      case 0:
        nrf_gpio_pin_write(BT_MESH_MODEL_LIGHT1, 1);
        nrf_gpio_pin_write(BT_MESH_MODEL_LIGHT2, 1);
        nrf_gpio_pin_write(BT_MESH_MODEL_LIGHT3, 1);
        nrf_gpio_pin_write(BT_MESH_MODEL_LIGHT4, 1);
        break;
      case 1:
        nrf_gpio_pin_write(BT_MESH_MODEL_LIGHT1, 0);
        break;
      case 2:
        nrf_gpio_pin_write(BT_MESH_MODEL_LIGHT2, 0);
        break;
      case 3:
        nrf_gpio_pin_write(BT_MESH_MODEL_LIGHT3, 0);
        break;
      case 4:
        nrf_gpio_pin_write(BT_MESH_MODEL_LIGHT4, 0);
        break;

      default:
        break;
    }
}

static const struct bt_mesh_health_srv_cb health_srv_cb =
{
  .fault_get_cur = &fault_get_cur,
  .fault_get_reg = &fault_get_reg,
  .fault_clear = &fault_clear,
  .fault_test = &fault_test,
};

static struct bt_mesh_health_srv health_srv =
{
  .cb = &health_srv_cb,
};

static struct bt_mesh_model_pub health_pub;

static void
health_pub_init(void)
{
  health_pub.msg  = BT_MESH_HEALTH_FAULT_MSG(0);
}

static struct bt_mesh_model_pub gen_level_pub;
static struct bt_mesh_model_pub gen_onoff_pub;

static struct bt_mesh_model_pub vendor_fea1_pub;
static struct bt_mesh_model_pub vnd_fea1_cli_pub;

int vendor_fea1_pub_fun(struct bt_mesh_model *mod)
{
  struct os_mbuf *msg = NET_BUF_SIMPLE(3);
  uint8_t *status;

  APP_LOG_INFO("#mesh-onoff STATUS\n");

  bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_3(0x14, CID_VENDOR));
  status = net_buf_simple_add(msg, 1);
  *status = vnd_fea1_state;

  APP_LOG_INFO("vnd_cli_pub__len %u: %s\n", mod->pub->msg->om_len, bt_hex(mod->pub->msg->om_data, mod->pub->msg->om_len));

  return 0;
}

int vnd_fea1_cli_pub_fun(struct bt_mesh_model *mod)
{
  uint8_t *status;

  /* Feature1 client publication to write feature1 state of 1 byte.*/
  bt_mesh_model_msg_init(mod->pub->msg, BT_MESH_MODEL_OP_3(0x12, CID_VENDOR));

  status = net_buf_simple_add(mod->pub->msg, 1);
  *status = vnd_fea1_state;

  APP_LOG_INFO("pub to [%x] data-> %u: %s\n", mod->pub->addr, mod->pub->msg->om_len, bt_hex(mod->pub->msg->om_data,
               mod->pub->msg->om_len));
  return 0;
}

static void
vendor_fea1_pub_init(void)
{
  vendor_fea1_pub.count = 0;
  vendor_fea1_pub.retransmit = 0;

  /* bit 0~5 step number;
   * bit 6~7 step resolution: 00: 100ms; 01: 1s; 10: 10s; q1: 10min;
   */
  vendor_fea1_pub.period = 0x42;

  vendor_fea1_pub.msg = NET_BUF_SIMPLE(3);
  vendor_fea1_pub.update = &vendor_fea1_pub_fun;
}

static void
vnd_fea1_cli_pub_init(void)
{
  vnd_fea1_cli_pub.count = 0;
  vnd_fea1_cli_pub.retransmit = 0;

  /* bit 0~5 step number;
   * bit 6~7 step resolution: 00: 100ms; 01: 1s; 10: 10s; q1: 10min;
   */
  vnd_fea1_cli_pub.period = 0x42;

  vnd_fea1_cli_pub.msg = NET_BUF_SIMPLE(3);
  vnd_fea1_cli_pub.update = &vnd_fea1_cli_pub_fun;
}

static uint8_t gen_on_off_state;
static int16_t gen_level_state;

static void gen_onoff_status(struct bt_mesh_model *model,
                             struct bt_mesh_msg_ctx *ctx)
{
  struct os_mbuf *msg = NET_BUF_SIMPLE(3);
  uint8_t *status;

  APP_LOG_INFO("#mesh-onoff STATUS\n");

  bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_2(0x82, 0x04));
  status = net_buf_simple_add(msg, 1);
  *status = gen_on_off_state;

  if (bt_mesh_model_send(model, ctx, msg, NULL, NULL))
    {
      APP_LOG_INFO("#mesh-onoff STATUS: send status failed\n");
    }

  os_mbuf_free_chain(msg);
}

static void gen_onoff_get(struct bt_mesh_model *model,
                          struct bt_mesh_msg_ctx *ctx,
                          struct os_mbuf *buf)
{
  gen_onoff_status(model, ctx);
}

static void gen_onoff_set(struct bt_mesh_model *model,
                          struct bt_mesh_msg_ctx *ctx,
                          struct os_mbuf *buf)
{
  APP_LOG_INFO("#mesh-onoff SET\n");

  gen_on_off_state = buf->om_data[0];
  //hal_gpio_write(LED_2, !gen_on_off_state);

  gen_onoff_status(model, ctx);
}

static void gen_onoff_set_unack(struct bt_mesh_model *model,
                                struct bt_mesh_msg_ctx *ctx,
                                struct os_mbuf *buf)
{
  APP_LOG_INFO("#mesh-onoff SET-UNACK\n");

  gen_on_off_state = buf->om_data[0];
  //hal_gpio_write(LED_2, !gen_on_off_state);
}

static const struct bt_mesh_model_op gen_onoff_op[] =
{
  { BT_MESH_MODEL_OP_2(0x82, 0x01), 0, gen_onoff_get },
  { BT_MESH_MODEL_OP_2(0x82, 0x02), 2, gen_onoff_set },
  { BT_MESH_MODEL_OP_2(0x82, 0x03), 2, gen_onoff_set_unack },
  BT_MESH_MODEL_OP_END,
};

static void vendor_fea1_status(struct bt_mesh_model *model,
                               struct bt_mesh_msg_ctx *ctx)
{
  struct os_mbuf *msg = NET_BUF_SIMPLE(3);
  uint8_t *status;

  APP_LOG_INFO("#mesh-onoff STATUS\n");

  bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_3(0x14, CID_VENDOR));
  status = net_buf_simple_add(msg, 1);
  *status = vnd_fea1_state;

  if (bt_mesh_model_send(model, ctx, msg, NULL, NULL))
    {
      APP_LOG_INFO("#mesh-onoff STATUS: send status failed\n");
    }

  os_mbuf_free_chain(msg);
}

static void vendor_fea1_get(struct bt_mesh_model *model,
                            struct bt_mesh_msg_ctx *ctx,
                            struct os_mbuf *buf)
{

  APP_LOG_INFO("Vendor feature1 received get MSG\n");
  APP_LOG_INFO("MSG addr[%x] len[%d]: %x\n", model->pub->addr, buf->om_len, buf->om_data[0]);
  vendor_fea1_status(model, ctx);
}

static void vendor_fea1_set(struct bt_mesh_model *model,
                            struct bt_mesh_msg_ctx *ctx,
                            struct os_mbuf *buf)
{
  APP_LOG_INFO("Vendor feature1 received set MSG...\n");
  vnd_fea1_state = buf->om_data[0];
  vendor_fea1_status(model, ctx);
  fea1_set_operation(vnd_fea1_state);
}

static void vendor_fea1_set_unack(struct bt_mesh_model *model,
                                  struct bt_mesh_msg_ctx *ctx,
                                  struct os_mbuf *buf)
{
  APP_LOG_INFO("Vendor feature1 SET with unack\n");
  vnd_fea1_state = buf->om_data[0];
}


static void vnd_fea1_cli_get(struct bt_mesh_model *model,
                             struct bt_mesh_msg_ctx *ctx,
                             struct os_mbuf *buf)
{
  APP_LOG_INFO("Vendor fea1 client GET\n");
  APP_LOG_INFO("MSG addr[%x] len[%d]: %x\n", model->pub->addr, buf->om_len, buf->om_data[0]);
  vendor_fea1_status(model, ctx);

}

static void vnd_fea1_cli_set(struct bt_mesh_model *model,
                             struct bt_mesh_msg_ctx *ctx,
                             struct os_mbuf *buf)
{
  APP_LOG_INFO("Vendor feature1 Client SET \n");
}

static const struct bt_mesh_model_op vendor_fea1_op[] =
{
  { BT_MESH_MODEL_OP_3(0x11, CID_VENDOR), 1, vendor_fea1_get },
  { BT_MESH_MODEL_OP_3(0x12, CID_VENDOR), 1, vendor_fea1_set },
  { BT_MESH_MODEL_OP_3(0x13, CID_VENDOR), 1, vendor_fea1_set_unack },
  BT_MESH_MODEL_OP_END,
};


static const struct bt_mesh_model_op vnd_fea1_cli_op[] =
{
  { BT_MESH_MODEL_OP_3(0x14, CID_VENDOR), 1, vnd_fea1_cli_get },
  { BT_MESH_MODEL_OP_3(0x15, CID_VENDOR), 1, vnd_fea1_cli_set },
  BT_MESH_MODEL_OP_END,
};

static void gen_level_status(struct bt_mesh_model *model,
                             struct bt_mesh_msg_ctx *ctx)
{
  struct os_mbuf *msg = NET_BUF_SIMPLE(4);

  APP_LOG_INFO("#mesh-level STATUS\n");

  bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_2(0x82, 0x08));
  net_buf_simple_add_le16(msg, gen_level_state);

  if (bt_mesh_model_send(model, ctx, msg, NULL, NULL))
    {
      APP_LOG_INFO("#mesh-level STATUS: send status failed\n");
    }

  os_mbuf_free_chain(msg);
}

static void gen_level_get(struct bt_mesh_model *model,
                          struct bt_mesh_msg_ctx *ctx,
                          struct os_mbuf *buf)
{
  APP_LOG_INFO("#mesh-level GET\n");

  gen_level_status(model, ctx);
}

static void gen_level_set(struct bt_mesh_model *model,
                          struct bt_mesh_msg_ctx *ctx,
                          struct os_mbuf *buf)
{
  int16_t level;

  level = (int16_t) net_buf_simple_pull_le16(buf);
  APP_LOG_INFO("#mesh-level SET: level=%d\n", level);

  gen_level_status(model, ctx);

  gen_level_state = level;
  APP_LOG_INFO("#mesh-level: level=%d\n", gen_level_state);
}

static void gen_level_set_unack(struct bt_mesh_model *model,
                                struct bt_mesh_msg_ctx *ctx,
                                struct os_mbuf *buf)
{
  int16_t level;

  level = (int16_t) net_buf_simple_pull_le16(buf);
  APP_LOG_INFO("#mesh-level SET-UNACK: level=%d\n", level);

  gen_level_state = level;
  APP_LOG_INFO("#mesh-level: level=%d\n", gen_level_state);
}

static void gen_delta_set(struct bt_mesh_model *model,
                          struct bt_mesh_msg_ctx *ctx,
                          struct os_mbuf *buf)
{
  int16_t delta_level;

  delta_level = (int16_t) net_buf_simple_pull_le16(buf);
  APP_LOG_INFO("#mesh-level DELTA-SET: delta_level=%d\n", delta_level);

  gen_level_status(model, ctx);

  gen_level_state += delta_level;
  APP_LOG_INFO("#mesh-level: level=%d\n", gen_level_state);
}

static void gen_delta_set_unack(struct bt_mesh_model *model,
                                struct bt_mesh_msg_ctx *ctx,
                                struct os_mbuf *buf)
{
  int16_t delta_level;

  delta_level = (int16_t) net_buf_simple_pull_le16(buf);
  APP_LOG_INFO("#mesh-level DELTA-SET: delta_level=%d\n", delta_level);

  gen_level_state += delta_level;
  APP_LOG_INFO("#mesh-level: level=%d\n", gen_level_state);
}

static void gen_move_set(struct bt_mesh_model *model,
                         struct bt_mesh_msg_ctx *ctx,
                         struct os_mbuf *buf)
{
  APP_LOG_INFO("#gen_move_set_unack\n");
}

static void gen_move_set_unack(struct bt_mesh_model *model,
                               struct bt_mesh_msg_ctx *ctx,
                               struct os_mbuf *buf)
{
  APP_LOG_INFO("#gen_move_set_unack\n");
}

static const struct bt_mesh_model_op gen_level_op[] =
{
  { BT_MESH_MODEL_OP_2(0x82, 0x05), 0, gen_level_get },
  { BT_MESH_MODEL_OP_2(0x82, 0x06), 3, gen_level_set },
  { BT_MESH_MODEL_OP_2(0x82, 0x07), 3, gen_level_set_unack },
  { BT_MESH_MODEL_OP_2(0x82, 0x09), 5, gen_delta_set },
  { BT_MESH_MODEL_OP_2(0x82, 0x0a), 5, gen_delta_set_unack },
  { BT_MESH_MODEL_OP_2(0x82, 0x0b), 3, gen_move_set },
  { BT_MESH_MODEL_OP_2(0x82, 0x0c), 3, gen_move_set_unack },
  BT_MESH_MODEL_OP_END,
};

static struct bt_mesh_model root_models[] =
{
  BT_MESH_MODEL_CFG_SRV(&cfg_srv),
  BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
  BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_onoff_op,
                &gen_onoff_pub, NULL),
  BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_LEVEL_SRV, gen_level_op,
                &gen_level_pub, NULL),
};

/*
 * Server id 0x2000, subsrciption msg from client, control the fea1 state.
 * Client id 0x2001, publication the msg to server, read and write the fea1 state depend on condition.
 */
static struct bt_mesh_model vnd_models[] =
{
  BT_MESH_MODEL_VND(CID_VENDOR + 1, BT_MESH_MODEL_ID_VND_SRV, vendor_fea1_op,
                    &vendor_fea1_pub, NULL),
  BT_MESH_MODEL_VND(CID_VENDOR + 2, BT_MESH_MODEL_ID_VND_CLI, vnd_fea1_cli_op,
                    &vnd_fea1_cli_pub, NULL),
};

static struct bt_mesh_elem elements[] =
{
  BT_MESH_ELEM(0, root_models, vnd_models),
};

static const struct bt_mesh_comp comp =
{
  .cid = CID_VENDOR,
  .pid = PID_VENDOR,
  .vid = VID_VENDOR,
  .elem = elements,
  .elem_count = ARRAY_SIZE(elements),
};

static int output_number(bt_mesh_output_action_t action, uint32_t number)
{
  APP_LOG_WARNING("\n----------------OOB Number: %lu----------------\n", number);

  return 0;
}

static void prov_complete(u16_t net_idx, u16_t addr)
{
#if(BLE_MESH_TIMER)
  APP_LOG_INFO("Timer start...\n", addr);
  et_delay_t delay;
  delay.interval.nsec = 0;
  delay.interval.sec = 5;
  delay.start.nsec = 0;
  delay.start.sec = 1;
  easy_timer_start(timerid, delay);
#endif
}

static const uint8_t dev_uuid[16] = MYNEWT_VAL(BLE_MESH_DEV_UUID);

static const struct bt_mesh_prov prov =
{
  .uuid = dev_uuid,
  .output_size = 1,
  .output_actions = BT_MESH_DISPLAY_NUMBER | BT_MESH_BEEP | BT_MESH_VIBRATE | BT_MESH_BLINK,
  .output_number = output_number,
  .complete = prov_complete,
};

static void
blemesh_on_reset(int reason)
{
  BLE_HS_LOG(ERROR, "Resetting state; reason=%d\n", reason);
}

static void
blemesh_on_sync(void)
{
  int err;
  ble_addr_t addr;

  APP_LOG_INFO("Bluetooth initialized\n");

  /* Use NRPA */
  err = ble_hs_id_gen_rnd(1, &addr);
  assert(err == 0);
  err = ble_hs_id_set_rnd(addr.val);
  assert(err == 0);

  err = bt_mesh_init(addr.type, &prov, &comp);
  if (err)
    {
      APP_LOG_INFO("Initializing mesh failed (err %d)\n", err);
      return;
    }

  APP_LOG_INFO("Mesh initialized\n");
}

#if(BLE_MESH_TIMER)
static void mesh_timer_cb(union sigval value)
{
}
#endif

/*This task used for getting user input information*/
__attribute__((__noreturn__)) void
blemesh_thread(void *arg)
{
  uint32_t len;
  uint8_t data = 0;
  static FILE *m_input_file = NULL;

  m_input_file = fopen(INPUT_FILE, "r+");
  if (NULL == m_input_file)
    {
      APP_LOG_ERROR("UART file open [%s] failed...\n", INPUT_FILE);
    }
  else
    {
      APP_LOG_INFO("UART file open [%s] successed...\n", INPUT_FILE);
    }

  while (1)
    {
      if (NULL == m_input_file)
        {
          APP_LOG_ERROR("\nUART file closed abnormal...\n");
        }
      len = fread(&data, 1, 1, m_input_file);
      if (1 != len)
        {
          APP_LOG_ERROR("\nInput wrong data lend...\n");
        }
      else
        {
          if (data >= INPUT_NUM_MIN && data <= INPUT_NUM_MAX)
            {
              APP_LOG_WARNING("\nGet Command input number [%d].\n", data - INPUT_NUM_MIN);
              vnd_fea1_state = data - INPUT_NUM_MIN;
            }
        }

    }
}

__attribute__((__noreturn__))
void
mesh_main(int argc, char **argv)
{
  int rc = 0;

  (void)rc;
  nimble_sysinit_start();
  nimble_hal_init();
  bt_mesh_register_gatt();
  ble_svc_gap_init();
  nimble_sysinit_end();

#if(BLE_MESH_TIMER)
  easy_timer_create(&timerid, &mesh_timer_cb);
#endif

  ble_hs_cfg.reset_cb = blemesh_on_reset;
  ble_hs_cfg.sync_cb = blemesh_on_sync;
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;


  health_pub_init();
  vendor_fea1_pub_init();
  vnd_fea1_cli_pub_init();

  /* Initialize the host task */
  os_task_init(&blemesh_task, "nimble_uart", blemesh_thread, NULL,
               100, OS_WAIT_FOREVER, NULL,
               128);

  while (1)
    {
      os_eventq_run(os_eventq_dflt_get());
    }
}
