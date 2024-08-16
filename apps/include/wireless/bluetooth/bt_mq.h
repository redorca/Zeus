/****************************************************************************
 *  wireless/bt_mq.h
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

#ifndef __WIRELESS_BT_MQ_H
#define __WIRELESS_BT_MQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/mqueue.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <mqueue.h>
#include <sched.h>
#include <signal.h>

#if CONFIG_MQ_MAXMSGSIZE > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_BT_MSG_SIZE       (32)      /* Max message size */

#define MQ_PRIO_MAX    _POSIX_MQ_PRIO_MAX

#define MQ_BT_MAX_DATA_LEN   (MAX_BT_MSG_SIZE-sizeof(bt_mq_recv_handler)-sizeof(uint16_t))

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

typedef void (* bt_mq_recv_handler)(uint8_t *data, uint16_t len);

typedef struct
{
  bt_mq_recv_handler  handler;    /* The message handler*/
  uint16_t            len;        /* Length in bytes to be written*/
  uint8_t             *data;      /* Data to be transfered*/
} bt_mq_msg_t;


/* This structure describes one buffered POSIX message. */

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
extern void bt_mq_init(void);

/****************************************************************************
 * Name: bt_mq_send_message
 *
 * Description:
 *   This function adds the specified message (msg) to BT message queue
 *   (mqdes).
 *
 *
 * Parameters:
 *   msg - Message to send
 *   prio - The priority of the message
 *
 * Return Value:
 *   On success, bt_mq_send_message() returns 0 (OK);
 *
 *   EMSGSIZE 'msglen' was greater than the maxmsgsize attribute of the
 *            message queue.
 *   EINTR    The call was interrupted by a signal handler.
 *
 * Assumptions/restrictions:
 *
 ****************************************************************************/
extern uint32_t bt_mq_send_message(bt_mq_msg_t *msg, int16_t priority);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_MQ_MAXMSGSIZE > 0 */
#endif /* __SCHED_MQUEUE_MQUEUE_H */

