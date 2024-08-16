/****************************************************************************
 * include/utils/ymodem_receiver.h
 *
 *   Copyright (C) 2018 zGlue Inc. All rights reserved.
 *   Author: Zhengwei Wang <zhengwei@zglue.com>
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

#ifndef __YMODEM_RECEIVER_H
#define __YMODEM_RECEIVER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>

#include <stdint.h>


#include <nuttx/mqueue.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
typedef FAR void *ymodem_receiver_id;

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* ymodem receiver buffer structure*/

begin_packed_struct struct ymodem_buffer_s
{
  struct dq_entry_s     dq_entry;   /* Double linked queue entry */
  int            maxbytes;  /* The maximum number of bytes */
  int            nbytes;     /* The number of bytes used */
  int            curbyte;    /* Next byte to be processed */
  uint8_t               samp[0];    /* Offset of the first sample */
} end_packed_struct;

/*ymodem exit reason*/
typedef enum
{
  YMODEM_EXIT_COMPLETE,/*complete*/
  YMODEM_EXIT_ERROR_IO,/*read/write error*/
  YMODEM_EXIT_ERROR_SYN,/*out of synchronism*/
  YMODEM_EXIT_SENDER_QUIT,/*remote sender quit*/
  YMODEM_EXIT_MASTER_QUIT,/*we received a stop command and quit*/
} ymodem_exit_reason;

/*ymodem message type*/
typedef enum
{
  YMODEM_MSG_START,/*transfer start message*/
  YMODEM_MSG_FILE_HEADER,/*file header message*/
  YMODEM_MSG_FILE_DATA,/*file data message*/
  YMODEM_MSG_EXIT,/*exit message*/
} ymodem_message_type;

/*file header message content*/
struct ymodem_header_message
{
  uint32_t file_size;/*file size*/
  char *file_name;
};

/*file data message content*/
struct ymodem_data_message
{
  struct ymodem_buffer_s *buf;
};

/*exit message content*/
struct ymodem_exit_message
{
  ymodem_exit_reason reason;
};

/*message content*/
union ymodem_message_info
{
  struct ymodem_header_message header_msg;
  struct ymodem_data_message  data_msg;
  struct ymodem_exit_message  exit_msg;
};

/*ymodem receiver message */
struct ymodem_message
{
  ymodem_message_type msg_type;
  union ymodem_message_info msg_info;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
/****************************************************************************
 * Name: ymodem_receiver_create
 *
 * Description:
 *   create a ymodem receiver thread
 *
 * Input Parameters:
 *   id -- place to hold the receiver id.
 *   fd -- transfer interface, this may be UART,USB or other interface
 *   mq -- receiver thread use this message queue to communicate with user thread
 *
 * Returned Value:
 *   0 on success, negative value on fail.
 *
 ****************************************************************************/
int ymodem_receiver_create(ymodem_receiver_id *id, int fd, mqd_t mq);

/****************************************************************************
 * Name: ymodem_receiver_start
 *
 * Description:
 *   start the receiver thread specified by the receiver id
 *
 * Input Parameters:
 *   id -- receiver id.
 *
 * Returned Value:
 *   0 on success, negative value on fail.
 *
 ****************************************************************************/
int ymodem_receiver_start(ymodem_receiver_id id);

/****************************************************************************
 * Name: ymodem_receiver_stop
 *
 * Description:
 *   stop the receiver thread specified by the receiver id
 *
 * Input Parameters:
 *   id -- receiver id.
 *
 * Returned Value:
 *   0 on success, negative value on fail.
 *
 ****************************************************************************/
int ymodem_receiver_stop(ymodem_receiver_id id);

/****************************************************************************
 * Name: ymodem_receiver_suspend
 *
 * Description:
 *   suspend the receiver thread specified by the receiver id, you can call
 *   ymodem_receiver_resume or ymodem_receiver_stop to resume/stop it lately.
 *
 * Input Parameters:
 *   id -- receiver id.
 *
 * Returned Value:
 *   0 on success, negative value on fail.
 *
 ****************************************************************************/
int ymodem_receiver_suspend(ymodem_receiver_id id);

/****************************************************************************
 * Name: ymodem_receiver_resume
 *
 * Description:
 *   resume the receiver thread specified by the receiver id.
 *
 * Input Parameters:
 *   id -- receiver id.
 *
 * Returned Value:
 *   0 on success, negative value on fail.
 *
 ****************************************************************************/

int ymodem_receiver_resume(ymodem_receiver_id id);

/****************************************************************************
 * Name: ymodem_receiver_delete
 *
 * Description:
 *   delete the receiver thread specified by the receiver id, this will release
 *   all the resource used by the receiver thread.
 *
 * Input Parameters:
 *   id -- receiver id.
 *
 * Returned Value:
 *   0 on success, negative value on fail.
 *
 ****************************************************************************/

int ymodem_receiver_delete(ymodem_receiver_id id);

/****************************************************************************
 * Name: ymodem_buf_alloc
 *
 * Description:
 *   malloc a buffer and initialize it.
 *
 * Input Parameters:
 *   ppBuffer -- place to hold the beffer info.
 *   numbytes -- buffer length
 *
 * Returned Value:
 *   0 on success, negative value on fail.
 *
 ****************************************************************************/

int ymodem_buf_alloc(FAR struct ymodem_buffer_s  **ppBuffer, int numbytes);

/****************************************************************************
 * Name: ymodem_buf_free
 *
 * Description:
 *   free a buffer.
 *
 * Input Parameters:
 *   buf -- pointer to the buffer info.
 *
 * Returned Value:
 *
 *
 ****************************************************************************/

void ymodem_buf_free(FAR struct ymodem_buffer_s *buf);

/****************************************************************************
 * Name: ymodem_buf_enqueue
 *
 * Description:
 *   enqueue a buffer to the receiver's buffer queue.
 *
 * Input Parameters:
 *   id  -- receiver id.
 *   buf -- pointer to the buf to be enqueued.
 *
 * Returned Value:
 *   0 on success, negative value on fail.
 *
 ****************************************************************************/

int ymodem_buf_enqueue(ymodem_receiver_id id, struct ymodem_buffer_s *buf);



#endif /*__YMODEM_RECEIVER_H */


