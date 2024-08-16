/****************************************************************************
 * apps/utils/ymodem_receiver.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/select.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <dirent.h>
#include <debug.h>
#include <unistd.h>
#include <assert.h>
#include <crc16.h>

#include <utils/ymodem_receiver.h>

#include <nuttx/mqueue.h>
#include <nuttx/signal.h>

#ifdef CONFIG_YMODEM_RECEIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define CONFIG_YMODEM_RECEIVER_POLLING


/* ASCII control codes: */
#define SOH (0x01)      /* start of 128-byte data packet */
#define STX (0x02)      /* start of 1024-byte data packet */
#define EOT (0x04)      /* end of transmission */
#define ACK (0x06)      /* receive OK */
#define NAK (0x15)      /* receiver error; retry */
#define CAN (0x18)      /* two of these in succession aborts transfer */
#define CNC (0x43)      /* character 'C' */

/*retry times*/
#define RETRY_SEND_C  (3)  /*retry three times if the sender doesn't response to C character*/
#define RETRY_SEND_NACK  (15)  /*retry several times if the sender doesn't response to nack character*/
#define RETRY_RECEIVE  (2)   /*retry several times if we got no data in read operation*/

/*buffer size*/
#define YMODEM_BUFFER_SIZE 1030

#define YMODEM_PACKAGE_START_CHAR_POS (0)
#define YMODEM_PACKAGE_START_CHAR_SIZE (1)

#define YMODEM_PACKAGE_NO_POS (YMODEM_PACKAGE_START_CHAR_POS + YMODEM_PACKAGE_START_CHAR_SIZE)
#define YMODEM_PACKAGE_RVS_NO_POS  (YMODEM_PACKAGE_NO_POS + 1)
#define YMODEM_PACKAGE_NO_SIZE (2)

#define YMODEM_PACKAGE_HEAD_SIZE (YMODEM_PACKAGE_START_CHAR_SIZE + YMODEM_PACKAGE_NO_SIZE)

#define YMODEM_PACKAGE_DATA_POS (YMODEM_PACKAGE_NO_POS + YMODEM_PACKAGE_NO_SIZE)
#define YMODEM_PACKAGE_SOH_DATA_SIZE (128)
#define YMODEM_PACKAGE_STX_DATA_SIZE (1024)

#define YMODEM_PACKAGE_CRC_SIZE (2)
#define YMODEM_PACKAGE_CHECKSUM_SIZE (1)

/*Maximum wait time when we are clean the trash data,
 *if timeout, we will start transfer.
 */
#define YMODEM_TIMEOUT_CLEAN_LINE_SEC 0 /*seconds*/
#define YMODEM_TIMEOUT_CLEAN_LINE_MSEC 500 /*micro seconds*/


/*Maximum wait time when we are receiving a package,
 *if timeout, we will stop receiving and start data processing.
 */
#define YMODEM_TIMEOUT_CONTINUOUS_PAC_SEC 0 /*seconds*/
#define YMODEM_TIMEOUT_CONTINUOUS_PAC_MSEC 10 /*micro seconds*/

/*Maximum wait time after we issue an ACK/NACK,
 *if timeout, we will send NACK to ask the sender resend previouse package.
 */
#define YMODEM_TIMEOUT_NEW_PAC_SEC 1 /*seconds*/
#define YMODEM_TIMEOUT_NEW_PAC_MSEC 0 /*micro seconds*/

/*wait time after we issue stop command*/
#define YMODEM_RECEIVER_STOP_LATENCY (1000) /*unit ms*/
#define YMODEM_RECEIVER_STOP_TEST_INTERVEL (10)

#define RSP_TO_STR(response)  (response == ACK ? "ACK" : \
                                (response == NAK ? "NAK" : \
                                (response  == CNC ? "CNC" : \
                                (response  == CAN ? "CAN" : "BS"))))


/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/
typedef enum
{
  YMODEM_IDLE,
  YMODEM_SUSPEND,
  YMODEM_CLEAN_LINE,
  YMODEM_START,
  YMODEM_START_C,
  YMODEM_START_NACK,
  YMODEM_RECEIVE,
  YMODEM_DATA_PROCESS,
  YMODEM_NACK,
  YMODEM_ACK,
  YMODEM_ACK_CNC,
  YMODEM_SEND_HEADER_MSG,
  YMODEM_SEND_DATA_MSG,
  YMODEM_QUIT,
  YMODEM_EXIT,
  YMODEM_DELETE,
} ymodem_status;

typedef enum
{
  YMODEM_COMMAND_START,
  YMODEM_COMMAND_STOP,
  YMODEM_COMMAND_SUSPEND,
  YMODEM_COMMAND_RESUME,
  YMODEM_COMMAND_DELETE,
} ymodem_command_type;


typedef enum
{
  YMODEM_PAC_FILEPATH,
  YMODEM_PAC_DATA,
  YMODEM_PAC_STOP,
} ymodem_pac_type;

typedef struct ymodem_receiver_type
{
  ymodem_status status;
  ymodem_status previous_status;
  ymodem_status next_status;
  ymodem_command_type command;
  int retry_counter;
  int fd;
  mqd_t mq;
  pthread_t myId;         /* Thread ID of the receiver thread */
  sem_t pendsem;          /* Protect pendq */
  struct dq_queue_s pendq; /* Queue of pending buffers to be sent */
} ymodem_receiver_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static void ymodem_signal_action(int signo, siginfo_t *siginfo, void *context);
static int ymodem_receiver_send(ymodem_receiver_t *rec, char response);
static int ymodem_receiver_check_package(uint8_t *buffer, int len, bool crc_check);
static int extract_file_size(uint8_t *buffer);
static void *ymodem_receiver_thread(pthread_addr_t pvarg);



/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: ymodem_signal_action
 *
 * Description:
 *   signal handler.
 *
 ****************************************************************************/

static void ymodem_signal_action(int signo, siginfo_t *siginfo, void *context)
{
  ymodem_receiver_t *receiver = (ymodem_receiver_t *)siginfo->si_value.sival_ptr;

  if (signo == SIGUSR1)
    {

      switch (receiver->command)
        {
          case YMODEM_COMMAND_START:
            syslog(LOG_INFO, "YMRCV: got start command\n");
            /*we only handle start command in idle state*/
            if (receiver->status == YMODEM_IDLE)
              {
                receiver->next_status =   YMODEM_CLEAN_LINE;
              }
            break;


          case YMODEM_COMMAND_STOP:
            syslog(LOG_INFO, "YMRCV: got stop command\n");
            /*we only handle start command when we are not in idle state*/
            if (receiver->status != YMODEM_IDLE)
              {
                /*user want to stop*/
                receiver->next_status = YMODEM_QUIT;
              }
            break;

          case YMODEM_COMMAND_SUSPEND:
            syslog(LOG_INFO, "YMRCV: got suspend command\n");
            /*we don't need to handle suspend command when we are in idle state or SUSPEND state*/
            if ((receiver->status != YMODEM_IDLE) && (receiver->status != YMODEM_SUSPEND))
              {
                receiver->next_status = YMODEM_SUSPEND;
              }
            break;

          case YMODEM_COMMAND_RESUME:
            syslog(LOG_INFO, "YMRCV: got resume command\n");
            /*we only handle resume command when we are in suspend state*/
            if (receiver->status == YMODEM_SUSPEND)
              {
                receiver->next_status = receiver->previous_status;
              }
            break;

          case YMODEM_COMMAND_DELETE:
            if (receiver->status == YMODEM_IDLE)
              {
                receiver->next_status = YMODEM_DELETE;
              }
            break;

          default:
            syslog(LOG_ERR, "YMRCV: got unexpected command\n");
            break;
        }

    }
  else
    {
      syslog(LOG_ERR, "YMRCV: received unexpected signal!\n");
    }
}


/****************************************************************************
 * Name: ymodem_receiver_send
 *
 * Description:
 *   send out response char
 *
 * Input Parameters:
 *   rec -- receiver.
 *   response -- char to be send
 *
 * Returned Value:
 *   0 on success, negative value on fail.
 *
 ****************************************************************************/


static int ymodem_receiver_send(ymodem_receiver_t *rec, char response)
{
  int txlen;

  txlen = write(rec->fd, &response, sizeof(char));

  if (txlen != sizeof(char))
    {
      syslog(LOG_WARNING, "YMRCV: send response failed:%d\n", errno);
      return txlen;
    }

  syslog(LOG_INFO, "YMRCV:send char:%s\n", RSP_TO_STR(response));

  return OK;
}

/****************************************************************************
 * Name: ymodem_receiver_check_package
 *
 * Description:
 *   check package
 *
 * Input Parameters:
 *   buffer -- buffer pointer.
 *   len -- buffer length
 *   crc_check -- true:use crc check, false:use checksum check
 *
 * Returned Value:
 *   0 on success, negative value on fail.
 *
 ****************************************************************************/

static int ymodem_receiver_check_package(uint8_t *buffer, int len, bool crc_check)
{
  int size_hope = YMODEM_PACKAGE_HEAD_SIZE;
  uint16_t result;
  int ret = OK;
  int i;

  /*check first charactor*/
  if (buffer[0] == SOH)
    {
      size_hope += YMODEM_PACKAGE_SOH_DATA_SIZE;
    }
  else if (buffer[0] == STX)
    {
      size_hope += YMODEM_PACKAGE_STX_DATA_SIZE;
    }
  else if ((buffer[0] == EOT) || (buffer[0] == CAN))
    {
      /*should we check length here?*/
      return OK;
    }
  else
    {
      syslog(LOG_ERR, "YMRCV:package start with a illegal charactor:%c!\n", buffer[0]);
      return -ECOMM;
    }

  if (crc_check)
    {
      size_hope += YMODEM_PACKAGE_CRC_SIZE;
    }
  else
    {
      size_hope += YMODEM_PACKAGE_CHECKSUM_SIZE;
    }

  /*check length*/
  if (len != size_hope)
    {
      syslog(LOG_ERR, "YMRCV:package length error,expected=%d,received=%d\n", size_hope, len);
      return -ECOMM;
    }

  /*check packet number*/
  if ((uint8_t)buffer[YMODEM_PACKAGE_NO_POS] != (uint8_t)(~buffer[YMODEM_PACKAGE_RVS_NO_POS]))
    {
      syslog(LOG_ERR, "YMRCV:package serial number error,NO=%d,RVS=%d\n", buffer[YMODEM_PACKAGE_NO_POS],
             buffer[YMODEM_PACKAGE_RVS_NO_POS]);
      return -ECOMM;
    }


  if (crc_check)
    {
      /*check crc*/
      result = crc16((const uint8_t *)&buffer[YMODEM_PACKAGE_DATA_POS], len - YMODEM_PACKAGE_HEAD_SIZE);
      if (result)
        {
          syslog(LOG_ERR, "YMRCV:package CRC check error,CRC=%d\n", result);
          ret = -ECOMM;
        }
    }
  else
    {
      result = 0;
      /*check checksum*/
      for (i = YMODEM_PACKAGE_DATA_POS; i < len; i++)
        {
          result ^= buffer[i];
        }

      if (result)
        {
          syslog(LOG_ERR, "YMRCV:package checksum error,checksum=%d\n", result);
          ret = -ECOMM;
        }
    }

  return ret;

}

/****************************************************************************
 * Name: extract_file_size
 *
 * Description:
 *   extract file size from the file header package
 *
 * Input Parameters:
 *   buffer -- buffer pointer.
 *
 * Returned Value:
 *   0 on success, negative value on fail.
 *
 ****************************************************************************/

static int extract_file_size(uint8_t *buffer)
{
  int name_len;
  name_len = strlen((const char *)buffer);
  return atoi((const char *)&buffer[name_len + 1]);
}

/****************************************************************************
 * Name: ymodem_receiver_thread
 *
 * Description:
 *   receiver thread.
 *
 ****************************************************************************/

static void *ymodem_receiver_thread(pthread_addr_t pvarg)
{
  ymodem_receiver_t *receiver = (ymodem_receiver_t *) pvarg;
  FAR struct timeval timeout;

  /*the package number and package type we are waiting*/
  ymodem_pac_type expect_pac_type;
  uint8_t expect_pac_number;

  /*current file size*/
  int current_pac_total_size;
  int current_pac_left_size;
  int current_file_total_size;
  int current_file_left_size;

  /*use crc or checksum*/
  bool crc_check;

  /*buffer for the data*/
  uint8_t *buffer;
  int buffer_len;
  uint8_t *write_ptr;
  uint8_t *read_ptr;
  struct ymodem_buffer_s *current_data_buf = NULL;


  /*buffer for the message*/
  uint8_t *buffer_msg;
  struct mq_attr attr_mq;
  struct ymodem_message *ymodem_msg;


  /*exit reason*/
  ymodem_exit_reason exit_reason;

  int ret;
  fd_set readset;
  bool running = true;
  struct sigaction act;
  sigset_t set;

  /*preprae the recceiver buffer*/

  buffer = malloc(YMODEM_BUFFER_SIZE);
  if (buffer == NULL)
    {
      syslog(LOG_ERR, "YMRCV:Failed to malloc buffer, errno=%d\n",
             errno);
      return NULL;
    }

  mq_getattr(receiver->mq, &attr_mq);

  buffer_msg = malloc(attr_mq.mq_msgsize);
  if (buffer_msg == NULL)
    {
      syslog(LOG_ERR, "YMRCV:Failed to malloc message buffer, errno=%d\n",
             errno);
      goto error_msg;
    }

  ymodem_msg = (struct ymodem_message *)buffer_msg;


  /* Set up so that siguser_action will respond to SIGUSR1 */

  memset(&act, 0, sizeof(struct sigaction));
  act.sa_sigaction =  ymodem_signal_action;
  act.sa_flags     = SA_SIGINFO;

  ret = sigaction(SIGUSR1, &act, NULL);
  if (ret != 0)
    {
      syslog(LOG_ERR, "YMRCV:Failed to install SIGUSR1 handler, errno=%d\n",
             errno);
      goto errout;
    }

  /**/
  while (running)
    {

      if (receiver->status != receiver->next_status)
        {
          switch (receiver->next_status)
            {
              case YMODEM_IDLE:
                receiver->status = receiver->next_status;
                break;
              case YMODEM_SUSPEND:
                receiver->previous_status = receiver->status;
                receiver->status = receiver->next_status;
                break;

              case YMODEM_CLEAN_LINE:
                syslog(LOG_INFO, "YMRCV:clean...\n");
                receiver->status = receiver->next_status;
                break;


              case YMODEM_START:
                /*in case we receive a stop command,
                 *we cann't set this value in signal handler*/
                exit_reason = YMODEM_EXIT_MASTER_QUIT;
                receiver->status = receiver->next_status;
                ymodem_msg->msg_type = YMODEM_MSG_START;
                break;

              case YMODEM_START_C:
                ymodem_receiver_send(receiver, CNC);
                receiver->retry_counter = 0;
                receiver->status = receiver->next_status;
                break;

              case YMODEM_START_NACK:
                ymodem_receiver_send(receiver, NAK);
                receiver->retry_counter = 0;
                receiver->status = receiver->next_status;
                break;

              case YMODEM_RECEIVE:

                syslog(LOG_INFO, "YMRCV:receiving...\n");
                write_ptr = buffer;
                buffer_len = 0;
                receiver->retry_counter = 0;
                receiver->status = receiver->next_status;
                break;

              case YMODEM_DATA_PROCESS:
                syslog(LOG_INFO, "YMRCV:processing...\n");
                receiver->status = receiver->next_status;
                break;

              case YMODEM_NACK:
                ymodem_receiver_send(receiver, NAK);
                receiver->retry_counter = 0;
                receiver->status = receiver->next_status;
                break;

              case YMODEM_ACK:
                ymodem_receiver_send(receiver, ACK);
                receiver->status = receiver->next_status;
                break;

              case YMODEM_ACK_CNC:
                ymodem_receiver_send(receiver, ACK);
                ymodem_receiver_send(receiver, CNC);
                receiver->status = receiver->next_status;
                break;


              case YMODEM_SEND_HEADER_MSG:
                ymodem_msg->msg_type = YMODEM_MSG_FILE_HEADER;
                ymodem_msg->msg_info.header_msg.file_size = current_file_total_size;
                receiver->status = receiver->next_status;
                break;

              case YMODEM_SEND_DATA_MSG:
                read_ptr = &buffer[YMODEM_PACKAGE_DATA_POS];
                receiver->status = receiver->next_status;
                break;

              case YMODEM_QUIT:
                receiver->status = receiver->next_status;
                break;

              case YMODEM_EXIT:
                ymodem_msg->msg_type = YMODEM_MSG_EXIT;
                ymodem_msg->msg_info.exit_msg.reason = exit_reason;
                receiver->status = receiver->next_status;
                break;

              case YMODEM_DELETE:
                receiver->status = receiver->next_status;
                break;

              default:
                break;
            }
        }

      switch (receiver->status)
        {
          case YMODEM_IDLE:
          case YMODEM_SUSPEND:
            /* prepare the signal set */
            (void)sigemptyset(&set);

            /*we do nothing but waiting for a start/resume signal*/
            ret = sigsuspend(&set);

            break;

          case YMODEM_CLEAN_LINE:
            {
              int len;
              int errcode;

              /*there may have some trash data left in the input buffer, we should clean them at first*/
              len = read(receiver->fd, buffer, YMODEM_BUFFER_SIZE);
              if (len < 0)
                {
                  errcode = errno;
                  if ((errcode != EINTR) && (errcode != EAGAIN))
                    {
                      syslog(LOG_ERR, "YMRCV: read failed:%d\n", errcode);
                      exit_reason = YMODEM_EXIT_ERROR_IO;
                      receiver->next_status = YMODEM_QUIT;
                      continue;
                    }
                }
              /*discard the trash data*/

              /*if len==0 or errcode == -EINTR or errcode == -EAGAIN*/
              /*we wait for several seconds to make sure this line is clean enough*/
              timeout.tv_sec = YMODEM_TIMEOUT_CLEAN_LINE_SEC;
              timeout.tv_usec = YMODEM_TIMEOUT_CLEAN_LINE_MSEC * USEC_PER_MSEC;

              /*prepare the read fd set*/
              FD_ZERO(&readset);
              FD_SET(receiver->fd, &readset);
              ret = select(receiver->fd + 1, (FAR fd_set *)&readset, (FAR fd_set *)NULL, (FAR fd_set *)NULL, &timeout);

              if (ret > 0)
                {
                  continue;
                }
              else if (ret == 0)
                {
                  /*no more trash data, tell the master we are starting*/
                  receiver->next_status = YMODEM_START;
                }
              else
                {
                  /*If we encounter any error*/
                  if (errno != EINTR)
                    {
                      syslog(LOG_ERR, "YMRCV:encounter an error in select operation:%d\n", errno);
                    }
                }
            }

            break;

          case YMODEM_START_C:

            timeout.tv_sec = YMODEM_TIMEOUT_NEW_PAC_SEC;
            timeout.tv_usec = YMODEM_TIMEOUT_NEW_PAC_MSEC * USEC_PER_MSEC;

            /*wait for data packet from ymodem sender*/
            /*prepare the read fd set*/
            FD_ZERO(&readset);
            FD_SET(receiver->fd, &readset);
            ret = select(receiver->fd + 1, (FAR fd_set *)&readset, (FAR fd_set *)NULL, (FAR fd_set *)NULL, &timeout);

            if (ret > 0)
              {
                /*if we received data packet*/
                crc_check = true;
                expect_pac_type = YMODEM_PAC_FILEPATH;
                expect_pac_number = 0;
                receiver->next_status = YMODEM_RECEIVE;
              }
            else if (ret == 0)
              {
                /*if timeout, try again*/
                receiver->retry_counter ++;

                if (receiver->retry_counter >= RETRY_SEND_C)
                  {
                    /*If we tried three times but didn't got any response, try NACK*/
                    receiver->next_status = YMODEM_START_NACK;
                    syslog(LOG_INFO, "YMRCV:send C and get no response, try NACK\n");
                  }
                else
                  {
                    ymodem_receiver_send(receiver, CNC);
                  }
              }
            else
              {
                /*If we encounter any error*/
                if (errno != EINTR)
                  {
                    syslog(LOG_ERR, "YMRCV:encounter an error in select operation:%d\n", errno);
                  }
              }

            break;

          case YMODEM_START_NACK:
          case YMODEM_NACK:
          case YMODEM_ACK:
          case YMODEM_ACK_CNC:

            timeout.tv_sec = YMODEM_TIMEOUT_NEW_PAC_SEC;
            timeout.tv_usec = YMODEM_TIMEOUT_NEW_PAC_MSEC * USEC_PER_MSEC;

            /*wait for data packet from ymodem sender*/
            /*prepare the read fd set*/
            FD_ZERO(&readset);
            FD_SET(receiver->fd, &readset);
            ret = select(receiver->fd + 1, (FAR fd_set *)&readset, (FAR fd_set *)NULL, (FAR fd_set *)NULL, &timeout);

            if (ret > 0)
              {
                if (receiver->status == YMODEM_START_NACK)
                  {
                    /*if we received data packet*/
                    crc_check = false;
                    expect_pac_type = YMODEM_PAC_FILEPATH;
                    expect_pac_number = 0;
                  }
                receiver->next_status = YMODEM_RECEIVE;
              }
            else if (ret == 0)
              {
                if ((receiver->status == YMODEM_START_NACK) || (receiver->status == YMODEM_NACK))
                  {
                    syslog(LOG_ERR, "YMRCV:retry...\n");
                    /*if timeout, try again*/
                    receiver->retry_counter ++;
                    if (receiver->retry_counter >= RETRY_SEND_NACK)
                      {
                        /*If we tried 15 times but didn't got any response, we should exit*/
                        receiver->next_status = YMODEM_QUIT;
                        exit_reason = YMODEM_EXIT_ERROR_SYN;
                        syslog(LOG_ERR, "YMRCV:exit, please start the sender firstly\n");
                      }
                    else
                      {
                        ymodem_receiver_send(receiver, NAK);
                      }
                  }
                else
                  {
                    receiver->next_status = YMODEM_NACK;
                  }
              }
            else
              {
                /*If we encounter any error*/
                if (errno != EINTR)
                  {
                    syslog(LOG_ERR, "YMRCV:encounter an error in select operation:%d\n", errno);
                  }
              }
            break;

          case YMODEM_RECEIVE:
            {
#ifdef CONFIG_YMODEM_RECEIVER_POLLING

              int len;
              int errcode;

              len = read(receiver->fd, write_ptr, YMODEM_BUFFER_SIZE - buffer_len);
              if (len < 0)
                {
                  errcode = errno;
                  if ((errcode != EINTR) && (errcode != EAGAIN))
                    {
                      syslog(LOG_ERR, "YMRCV: read failed:%d\n", errcode);
                      exit_reason = YMODEM_EXIT_ERROR_IO;
                      receiver->next_status = YMODEM_QUIT;
                      continue;
                    }

                  if (errcode == EINTR)
                    {
                      continue;
                    }
                }
              else if (len > 0)
                {
                  write_ptr += len;
                  buffer_len += len;
                  if (buffer_len >= YMODEM_BUFFER_SIZE)
                    {
                      syslog(LOG_ERR, "YMRCV:data package size exceed 1k byte! \n");
                      /*roll over to the start, this error will be handled later*/
                      write_ptr = buffer;
                      buffer_len = 0;
                    }
                  receiver->retry_counter = 0;
                  continue;
                }

              /*if len==0 or  errcode == -EAGAIN*/
              /*we retry several times before we go to select operation, this will reduce the wake up time*/
              if (receiver->retry_counter < RETRY_RECEIVE)
                {
                  receiver->retry_counter ++;
                  continue;
                }
              else
                {
                  /*this means we have got no data in read operation for 20 times continuously*/
                  receiver->retry_counter = 0;
                }


              /*if len==0 or errcode == -EINTR or errcode == -EAGAIN*/
              /*we wait for several microseconds to check whether sender stop sending or just taking a break*/
              timeout.tv_sec = YMODEM_TIMEOUT_CONTINUOUS_PAC_SEC;
              timeout.tv_usec = YMODEM_TIMEOUT_CONTINUOUS_PAC_MSEC * USEC_PER_MSEC;

              /*prepare the read fd set*/
              FD_ZERO(&readset);
              FD_SET(receiver->fd, &readset);
              ret = select(receiver->fd + 1, (FAR fd_set *)&readset, (FAR fd_set *)NULL, (FAR fd_set *)NULL, &timeout);

              if (ret > 0)
                {
                  continue;
                }
              else if (ret == 0)
                {
                  receiver->next_status = YMODEM_DATA_PROCESS;
                }
              else
                {
                  /*If we encounter any error*/
                  if (errno != EINTR)
                    {
                      syslog(LOG_ERR, "YMRCV:encounter an error in select operation:%d\n", errno);
                    }
                }


#else
              int len;
              int errcode;

              len = read(receiver->fd, write_ptr, YMODEM_BUFFER_SIZE - buffer_len);
              if (len < 0)
                {
                  errcode = errno;

                  /*if we receive a signal*/
                  if (errcode == EINTR)
                    {
                      continue;
                    }
                  /*if we receive no data*/
                  else if (errcode == EAGAIN)
                    {
                      receiver->retry_counter ++;
                    }
                  /*if an unexcepted error happend*/
                  else
                    {
                      syslog(LOG_ERR, "YMRCV: read failed:%d\n", errcode);
                      exit_reason = YMODEM_EXIT_ERROR_IO;
                      receiver->next_status = YMODEM_QUIT;
                      continue;
                    }
                }
              else if (len > 0)
                {
                  write_ptr += len;
                  buffer_len += len;
                  if (buffer_len >= YMODEM_BUFFER_SIZE)
                    {
                      syslog(LOG_ERR, "YMRCV:data package size exceed 1k byte! \n");
                      /*roll over to the start, this error will be handled later*/
                      write_ptr = buffer;
                      buffer_len = 0;
                    }

                  /*clear the counter*/
                  receiver->retry_counter = 0;
                }
              else
                {
                  receiver->retry_counter ++;
                }

              /*if this is the second time we receive 0 bytes*/
              if (receiver->retry_counter == RETRY_RECEIVE)
                {
                  receiver->next_status = YMODEM_DATA_PROCESS;
                  continue;
                }

              usleep(10 * 1000);

#endif

            }
            break;

          case YMODEM_DATA_PROCESS:
            {
              int check_result;
              /*check formate and crc(checksum)*/
              check_result = ymodem_receiver_check_package(buffer, buffer_len, crc_check);


              if (check_result != OK)
                {
                  /*send NACK*/
                  receiver->next_status = YMODEM_NACK;
                  syslog(LOG_INFO, "YMRCV:got an error package,send NACK!\n");
                  continue;
                }

              syslog(LOG_INFO, "YMRCV:got a valid package\n");

              /*process the package*/

              if (buffer[YMODEM_PACKAGE_START_CHAR_POS] == EOT)
                {
                  syslog(LOG_INFO, "YMRCV:got EOT.\n");
                  /*if this is the first time we receive EOT*/
                  if (expect_pac_type == YMODEM_PAC_DATA)
                    {
                      /*expect the second EOT charactor*/
                      expect_pac_type = YMODEM_PAC_STOP;
                      /*send NACK*/
                      receiver->next_status = YMODEM_NACK;
                    }
                  else if (expect_pac_type == YMODEM_PAC_STOP)
                    {
                      /*current file transmittion complete.*/
                      expect_pac_type = YMODEM_PAC_FILEPATH;
                      expect_pac_number = 0;
                      /*send something special to the user */
                      /*todo:*/

                      /*send ACK and CNC*/
                      receiver->next_status = YMODEM_ACK_CNC;
                    }
                  else
                    {
                      /*we should not receive EOT when we are expecting the FILEPATH package*/
                      syslog(LOG_ERR, "YMRCV:lost synchronization, abording!\n");
                      receiver->next_status = YMODEM_QUIT;
                      exit_reason = YMODEM_EXIT_ERROR_SYN;
                    }
                }
              else if (buffer[YMODEM_PACKAGE_START_CHAR_POS] == CAN)
                {
                  syslog(LOG_INFO, "YMRCV:got CAN.\n");
                  syslog(LOG_ERR, "YMRCV:sender quit!\n");
                  receiver->next_status = YMODEM_EXIT;
                  exit_reason = YMODEM_EXIT_SENDER_QUIT;
                }
              else
                {
                  /*if this is the previouse package*/
                  if (buffer[YMODEM_PACKAGE_NO_POS] == (expect_pac_number - 1))
                    {
                      /*send ack*/
                      receiver->next_status = YMODEM_ACK;
                    }
                  /*if this is the package we expect*/
                  else if (buffer[YMODEM_PACKAGE_NO_POS] == expect_pac_number)
                    {

                      if (expect_pac_type == YMODEM_PAC_FILEPATH)
                        {
                          current_file_total_size = extract_file_size(&buffer[YMODEM_PACKAGE_DATA_POS]);
                          if (current_file_total_size == 0)
                            {
                              /*all files are transmitted*/
                              syslog(LOG_INFO, "YMRCV: mission accomplished!\n");
                              receiver->next_status = YMODEM_EXIT;
                              exit_reason = YMODEM_EXIT_COMPLETE;
                              ymodem_receiver_send(receiver, ACK);
                            }
                          else
                            {
                              /*record how many data left to be receive*/
                              current_file_left_size = current_file_total_size;

                              /*start transfer data section*/
                              expect_pac_type = YMODEM_PAC_DATA;
                              expect_pac_number += 1;

                              /*tell the master thread we will start transfer a new file*/
                              receiver->next_status = YMODEM_SEND_HEADER_MSG;
                            }
                        }
                      else if (expect_pac_type == YMODEM_PAC_DATA)
                        {
                          expect_pac_number += 1;

                          if (buffer[YMODEM_PACKAGE_START_CHAR_POS] == SOH)
                            {
                              current_pac_total_size = YMODEM_PACKAGE_SOH_DATA_SIZE;
                            }
                          else if (buffer[YMODEM_PACKAGE_START_CHAR_POS] == STX)
                            {
                              current_pac_total_size = YMODEM_PACKAGE_STX_DATA_SIZE;
                            }
                          else
                            {
                              /*impossible to happen*/
                              current_pac_total_size = 0;
                            }


                          if (current_pac_total_size < current_file_left_size)
                            {
                              current_file_left_size -= current_pac_total_size;
                            }
                          else
                            {
                              current_pac_total_size = current_file_left_size;
                              current_file_left_size = 0;
                            }

                          current_pac_left_size = current_pac_total_size;

                          /*tell the master thread we received a data package*/
                          receiver->next_status = YMODEM_SEND_DATA_MSG;
                        }
                      else
                        {
                          /*we should never receive a package start with SOH(STX) when we are expecting EOT*/
                          syslog(LOG_ERR, "YMRCV: lost synchronization, abording!\n");
                          receiver->next_status = YMODEM_QUIT;
                          exit_reason = YMODEM_EXIT_ERROR_SYN;
                        }
                    }
                  else
                    {
                      syslog(LOG_ERR, "YMRCV: lost synchronization, abording!\n");
                      receiver->next_status = YMODEM_QUIT;
                      exit_reason = YMODEM_EXIT_ERROR_SYN;
                    }
                }
            }

            break;

          case YMODEM_SEND_DATA_MSG:
            {

              int buffer_length;
              int copy_length;
              irqstate_t flags;

              if (current_data_buf == NULL)
                {
                  ret = sem_wait(&receiver->pendsem);
                  DEBUGASSERT(ret == 0 || errno == EINTR);

                  if (ret < 0 && errno == EINTR)
                    {
                      continue;
                    }

                  flags = enter_critical_section();

                  if (dq_empty(&receiver->pendq))
                    {
                      leave_critical_section(flags);

                      /*if the pend queue is empty, we suspend*/
                      receiver->next_status = YMODEM_SUSPEND;

                      continue;
                    }
                  else
                    {
                      /*take the first buffer in the pend queue*/
                      current_data_buf = (struct ymodem_buffer_s *)dq_remfirst(&receiver->pendq);

                      leave_critical_section(flags);

                      syslog(LOG_INFO, "Dequeueing: buf=%p\n", current_data_buf);
                    }

                }

              current_data_buf->nbytes = 0;
              current_data_buf->curbyte = 0;

              buffer_length = current_data_buf->maxbytes - current_data_buf->nbytes;
              copy_length = (current_pac_left_size > buffer_length) ? buffer_length : current_pac_left_size;

              memcpy(&current_data_buf->samp[current_data_buf->nbytes], read_ptr, copy_length);

              current_data_buf->nbytes += copy_length;

              /*we send out this buffer*/
              ymodem_msg->msg_type = YMODEM_MSG_FILE_DATA;
              ymodem_msg->msg_info.data_msg.buf = current_data_buf;

              ret = mq_send(receiver->mq, (const char *)buffer_msg,
                            (sizeof(struct ymodem_message)), 42);
              if (ret == OK)
                {
                  current_pac_left_size -= copy_length;
                  read_ptr += copy_length;

                  /*we will try to get another buffer in the queue*/
                  current_data_buf = NULL;

                  if (current_pac_left_size == 0)
                    {
                      /*after this, we go to send ACK*/
                      receiver->next_status = YMODEM_ACK;
                      syslog(LOG_INFO, "YMRCV: send out the received package\n");
                    }
                }
              else
                {
                  if (errno != EINTR)
                    {
                      syslog(LOG_INFO, "YMRCV: ERROR mq_send failure=%d\n", errno);
                    }
                }

            }

            break;

          case YMODEM_QUIT:
            ymodem_receiver_send(receiver, CAN);
            ymodem_receiver_send(receiver, CAN);
            ymodem_receiver_send(receiver, CAN);
            ymodem_receiver_send(receiver, CAN);
            ymodem_receiver_send(receiver, 0x08);
            ymodem_receiver_send(receiver, 0x08);
            ymodem_receiver_send(receiver, 0x08);
            ymodem_receiver_send(receiver, 0x08);

            receiver->next_status = YMODEM_EXIT;
            syslog(LOG_INFO, "YMRCV: quit from current transfer,try again!\n");

            break;

          case YMODEM_START:
          case YMODEM_SEND_HEADER_MSG:
          case YMODEM_EXIT:
            ret = mq_send(receiver->mq, (const char *)buffer_msg,
                          (sizeof(struct ymodem_message)), 42);
            if (ret == OK)
              {
                switch (receiver->status)
                  {
                    case YMODEM_START:
                      receiver->next_status = YMODEM_START_C;
                      syslog(LOG_INFO, "YMRCV: start...\n");
                      break;
                    case YMODEM_SEND_HEADER_MSG:
                      receiver->next_status = YMODEM_ACK_CNC;
                      syslog(LOG_INFO, "YMRCV: start transfer a new file\n");
                      break;
                    case YMODEM_EXIT:
                      receiver->next_status = YMODEM_IDLE;
                      syslog(LOG_INFO, "YMRCV: exit\n");
                      break;
                    default:
                      break;

                  }

              }
            else
              {
                if (errno != EINTR)
                  {
                    printf("YMRCV: ERROR mq_send failure=%d\n", errno);
                  }
              }
            break;

          case YMODEM_DELETE:
            running = false;
            break;
          default:
            break;

        }


    }

errout:
  free(buffer_msg);
error_msg:
  free(buffer);
  return NULL;

}


/****************************************************************************
 * Public Functions
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

int ymodem_receiver_create(ymodem_receiver_id *id, int fd, mqd_t mq)
{
  int ret;
  ymodem_receiver_t *receiver;
  struct sched_param  sparam;
  pthread_attr_t      tattr;
  int flags;

  /* Initialize the ymodem receiver data structure */

  receiver = (ymodem_receiver_t *)malloc(sizeof(ymodem_receiver_t));
  if (receiver == NULL)
    {
      syslog(LOG_ERR, "YMRCV: Failed to allocate instance\n");
      return -ENOMEM;
    }

  /*fill in the structure*/
  receiver->status = YMODEM_IDLE;
  receiver->next_status = YMODEM_IDLE;
  receiver->retry_counter = 0;
  receiver->fd = fd;
  receiver->mq = mq;

  /* Initialize buffer queue */

  dq_init(&receiver->pendq);

  /* Initialize buffer semphone */

  sem_init(&receiver->pendsem, 0, 0);


  /*set the NONBLOCK flag*/
  flags = fcntl(receiver->fd, F_GETFL, 0);
  if (flags < 0)
    {
      syslog(LOG_ERR, "YMRCV: failed get fd flag:%d\n", flags);
      ret = flags;
      goto err_out;
    }

  flags = fcntl(receiver->fd, F_SETFL, flags | O_NONBLOCK);
  if (flags < 0)
    {
      syslog(LOG_ERR, "YMRCV: failed set fd flag:%d\n", flags);
      ret = flags;
      goto err_out;
    }

  /*create receiving thread*/
  pthread_attr_init(&tattr);
  sparam.sched_priority = sched_get_priority_max(SCHED_FIFO) - 9;
  (void)pthread_attr_setschedparam(&tattr, &sparam);
  (void)pthread_attr_setstacksize(&tattr, 2048);

  ret = pthread_create(&receiver->myId, &tattr, ymodem_receiver_thread,
                       (pthread_addr_t) receiver);
  if (ret != OK)
    {
      syslog(LOG_ERR, "YMRCV:Failed to create receiver thread: %d\n", ret);
      goto err_out;
    }

  /* Name the thread */

  pthread_setname_np(receiver->myId, "ymodemReceiver");
  *id = (ymodem_receiver_id)receiver;
  return OK;

err_out:
  free(receiver);
  return ret;


}

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

int ymodem_receiver_start(ymodem_receiver_id id)
{
  ymodem_receiver_t *receiver = (ymodem_receiver_t *)id;
  union sigval value;

  /*set command type*/
  receiver->command = YMODEM_COMMAND_START;

  /*send signal*/

  value.sival_ptr = (void *)receiver;
  (void)sigqueue(receiver->myId, SIGUSR1, value);

  return OK;
}

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

int ymodem_receiver_stop(ymodem_receiver_id id)
{
  ymodem_receiver_t *receiver = (ymodem_receiver_t *)id;
  union sigval value;

  /*set command type*/
  receiver->command = YMODEM_COMMAND_STOP;

  /*send signal*/

  value.sival_ptr = (void *)receiver;
  (void)sigqueue(receiver->myId, SIGUSR1, value);


  return OK;
}

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

int ymodem_receiver_suspend(ymodem_receiver_id id)
{
  ymodem_receiver_t *receiver = (ymodem_receiver_t *)id;
  union sigval value;

  /*set command type*/
  receiver->command = YMODEM_COMMAND_SUSPEND;

  /*send signal*/

  value.sival_ptr = (void *)receiver;
  (void)sigqueue(receiver->myId, SIGUSR1, value);


  return OK;
}

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

int ymodem_receiver_resume(ymodem_receiver_id id)
{
  ymodem_receiver_t *receiver = (ymodem_receiver_t *)id;
  union sigval value;

  /*set command type*/
  receiver->command = YMODEM_COMMAND_RESUME;

  /*send signal*/

  value.sival_ptr = (void *)receiver;
  (void)sigqueue(receiver->myId, SIGUSR1, value);

  return OK;

}

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

int ymodem_receiver_delete(ymodem_receiver_id id)
{
  ymodem_receiver_t *receiver = (ymodem_receiver_t *)id;
  union sigval value;
  FAR void *p_return_value;
  int ret;
  int count = 0;

  /*if we are running, we should stop at first*/
  if (receiver->status != YMODEM_IDLE)
    {
      /*set command type*/
      receiver->command = YMODEM_COMMAND_STOP;

      /*send signal*/

      value.sival_ptr = (void *)receiver;
      (void)sigqueue(receiver->myId, SIGUSR1, value);

    }

  /*wait the state machine come to idle status*/
  while ((receiver->status != YMODEM_IDLE) &&
         (count < (YMODEM_RECEIVER_STOP_LATENCY / YMODEM_RECEIVER_STOP_TEST_INTERVEL)))
    {
      usleep(YMODEM_RECEIVER_STOP_TEST_INTERVEL * USEC_PER_MSEC); /*wait for 10ms and test again*/
      count ++;
    }

  if (receiver->status != YMODEM_IDLE)
    {
      return -ETIME;
    }

  /*set command type*/
  receiver->command = YMODEM_COMMAND_DELETE;

  /*send signal*/

  value.sival_ptr = (void *)receiver;
  (void)sigqueue(receiver->myId, SIGUSR1, value);


  ret = pthread_join(receiver->myId, &p_return_value);
  if (ret == OK)
    {
      free(receiver);
    }

  return ret;

}

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

int ymodem_buf_alloc(FAR struct ymodem_buffer_s  **ppBuffer, int numbytes)
{
  uint32_t            bufsize;
  int                 ret;
  struct ymodem_buffer_s  *buf;

  DEBUGASSERT(ppBuffer != NULL);

  /* Perform a user mode allocation */

  bufsize = sizeof(struct ymodem_buffer_s) + numbytes;
  buf = malloc(bufsize);
  *ppBuffer = buf;

  /* Test if the allocation was successful or not */

  if (*ppBuffer == NULL)
    {
      ret = -ENOMEM;
    }
  else
    {
      /* Populate the buffer contents */
      buf->maxbytes  = numbytes;
      buf->nbytes     = 0;
      buf->curbyte     = 0;

      ret = bufsize;
    }

  return ret;
}

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

void ymodem_buf_free(FAR struct ymodem_buffer_s *buf)
{
  free(buf);
}

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

int ymodem_buf_enqueue(ymodem_receiver_id id,
                       struct ymodem_buffer_s *buf)
{
  ymodem_receiver_t *receiver = (ymodem_receiver_t *)id;
  irqstate_t flags;

  syslog(LOG_INFO, "Enqueueing: buf=%p curbyte=%d nbytes=%d\n",
         buf, buf->curbyte, buf->nbytes);


  /* Add the new buffer to the tail of pending audio buffers */

  flags = enter_critical_section();
  dq_addlast(&buf->dq_entry, &receiver->pendq);
  leave_critical_section(flags);

  /*Post the semphone. If the receiver thread is waiting for avaliable buffer,
  this will wake it up.*/
  sem_post(&receiver->pendsem);


  return OK;
}



#endif /*CONFIG_YMODEM_RECEIVER*/
