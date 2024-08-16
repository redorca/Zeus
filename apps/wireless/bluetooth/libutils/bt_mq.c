/****************************************************************************
 *   wireless/bluetooth/libutils/bt_mq.c
 *
 *   Copyright (C) 2007-2017. All rights reserved.
 *   Author: taohan <taohan@zglue.org>
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

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <errno.h>

#include <wireless/bluetooth/bt_mq.h>
/****************************************************************************
 * Private Definitions
 ****************************************************************************/
#define BT_MSG_Q_STACKSIZE   (1024)
#define BT_MSG_Q_MSG_DATA_POSI (sizeof(bt_mq_recv_handler)+sizeof(uint16_t))
#define BT_MSG_Q_NAME        "bluetooth_mqueue"

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/
typedef struct
{
  mqd_t recv_mqfd;       /*BT MSG Q received handle*/
  mqd_t send_mqfd;       /*BT MSG Q send handle*/
  uint8_t msg_count;     /*Current message number*/
} bt_mq_param_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

bt_mq_param_t bt_mq_para = {0};


/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void *bt_receiver_thread(void *arg)
{
  int8_t msg_buffer[MAX_BT_MSG_SIZE];
  uint16_t nbytes;
  bt_mq_msg_t *bt_mq_p;

  _info("BT_MSG_Q thread starting...\n");

  /* Perform the receive loop... */

  for (; ;)
    {
      memset(msg_buffer, 0, MAX_BT_MSG_SIZE);
      nbytes = mq_receive(bt_mq_para.recv_mqfd, (char *)msg_buffer, MAX_BT_MSG_SIZE, 0);
      if (nbytes < 0)
        {
          /* The queue was empty, and the O_NONBLOCK flag was set,
          *  it will return EAGAIN.
          *  But for default mode, it is BLOCK mode, when queue is
          *  empty, it will be suspended on mq_recive...
          */
          _err("BT_MSG_Q get message failure: ERRORNO = %d\n", errno);
        }
      else
        {
          bt_mq_para.msg_count--;
          bt_mq_p = (bt_mq_msg_t *)msg_buffer;
          (*bt_mq_p->handler)((uint8_t *)(&msg_buffer[BT_MSG_Q_MSG_DATA_POSI]), bt_mq_p->len);
        }
    }

  /* Destroy the message queue */
  if (mq_unlink(BT_MSG_Q_NAME) < 0)
    {
      _err("BT_MSG_Q: ERROR mq_unlink failed\n");
    }

  /* Close the queue and return success */
  if (mq_close(bt_mq_para.recv_mqfd) < 0)
    {
      _err("ERROR mq_close failed\n");
    }
  else
    {
      bt_mq_para.recv_mqfd = NULL;
    }
  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
uint32_t bt_mq_send_message(bt_mq_msg_t *msg, int16_t priority)
{
  uint32_t ret = 0;
  uint8_t data[MAX_BT_MSG_SIZE] = {0};
  irqstate_t flags;

  assert(msg != NULL);
  assert(bt_mq_para.send_mqfd != NULL);

  if ((BT_MSG_Q_MSG_DATA_POSI + msg->len) > MAX_BT_MSG_SIZE)
    {
      _err("ERROR: Message length[%d] is exceed the MAX...\n", (BT_MSG_Q_MSG_DATA_POSI + msg->len));
      return EMSGSIZE;
    }

  flags = enter_critical_section();
  if (bt_mq_para.msg_count >= CONFIG_BT_MSG_Q_MAX_MSG_NUMBER)
    {
      _err("ERROR: Message number[%d] is excced the MAX...\n", bt_mq_para.msg_count);
      return EMSGSIZE;
    }
  bt_mq_para.msg_count++;
  leave_critical_section(flags);

  memcpy(data, (uint8_t *)msg, BT_MSG_Q_MSG_DATA_POSI);
  memcpy(data + BT_MSG_Q_MSG_DATA_POSI, (uint8_t *)msg->data, msg->len);

  ret = mq_send(bt_mq_para.send_mqfd, (char *)data, (BT_MSG_Q_MSG_DATA_POSI + msg->len), priority);

  if (ret != OK)
    {
      ret = get_errno();
      _err("ERROR BT_mq_send failure=%d on msg\n", ret);
      bt_mq_para.msg_count--;
    }

  return ret;

}

void bt_mq_init(void)
{
  struct mq_attr msg_attr;
  pthread_t receiver;
  pthread_attr_t attr;
  struct sched_param sparam;
  int8_t status;

  _info("\n BT MSG Q INIT...\n");

  /* Create message queue files*/

  msg_attr.mq_maxmsg  = CONFIG_BT_MSG_Q_MAX_MSG_NUMBER;
  msg_attr.mq_msgsize = MAX_BT_MSG_SIZE;
  msg_attr.mq_flags   = 0;

  bt_mq_para.recv_mqfd = mq_open(BT_MSG_Q_NAME, O_RDONLY | O_CREAT, 0666, &msg_attr);
  if (bt_mq_para.recv_mqfd < 0)
    {
      _err("receiver_thread: ERROR mq_open failed\n");
    }

  if (bt_mq_para.send_mqfd == 0)
    {

      bt_mq_para.send_mqfd = mq_open(BT_MSG_Q_NAME, O_WRONLY | O_NONBLOCK);
      if (bt_mq_para.send_mqfd < 0)
        {
          _err("ERROR BT_mq_open failed\n");
        }
    }

  bt_mq_para.msg_count = 0;

  status = pthread_attr_init(&attr);
  if (status != 0)
    {
      _err("BT_MSG_Q: pthread_attr_init failed, status=%d\n", status);
    }

  status = pthread_attr_setstacksize(&attr, CONFIG_BT_MSG_Q_STACKSIZE);
  if (status != 0)
    {
      _err("BT_MSG_Q: pthread_attr_setstacksize failed, status=%d\n", status);
    }

  sparam.sched_priority = CONFIG_BT_MSG_Q_PRIORITY;
  status = pthread_attr_setschedparam(&attr, &sparam);
  if (status != OK)
    {
      _err("BT_MSG_Q: pthread_attr_setschedparam failed, status=%d\n", status);
    }
  else
    {
      printf("BT_MSG_Q: Thread priority is %d\n", sparam.sched_priority);
    }

  status = pthread_create(&receiver, &attr, bt_receiver_thread, NULL);
  if (status != 0)
    {
      printf("BT_MSG_Q: pthread_create failed, status=%d\n", status);
    }

}


