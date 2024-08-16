/****************************************************************************
 * apps/system/nxplayer/nxplayer_main.c
 *
 *   Copyright (C) 2013 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <assert.h>
#include <fcntl.h>

#include "system/readline.h"
#include "utils/ymodem_receiver.h"

#ifdef CONFIG_SYSTEM_YMODEM_RECEIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#ifndef CONFIG_YMODEM_RECEIVER
#  error "YMODEM_RECEIVER module is not enabled (CONFIG_YMODEM_RECEIVER)"
#endif

#ifndef CONFIG_YMODEM_RECEIVER_MAX_MSG
#  define CONFIG_YMODEM_RECEIVER_MAX_MSG (20)
#endif

#ifndef CONFIG_YMODEM_RECEIVER_MSG_SIZE
#  define CONFIG_YMODEM_RECEIVER_MSG_SIZE (32)
#endif

#ifndef CONFIG_YMODEM_RECEIVER_SOURCE_DEVICE
#  error "YMODEM receiver data source device is not specified"
#endif

#ifndef CONFIG_YMODEM_RECEIVER_DEFAULT_DEST_FILE
#  error "YMODEM receiver default destination file is not specified"
#endif

#ifndef CONFIG_MSG_HANDLE_THREAD_STACK_SIZE
#  define CONFIG_MSG_HANDLE_THREAD_STACK_SIZE (2048)
#endif

#define EXIT_REASON_TO_STR(reason)  (reason == YMODEM_EXIT_COMPLETE ? "YMODEM_EXIT_COMPLETE" :                      \
                                (reason == YMODEM_EXIT_ERROR_IO ? "YMODEM_EXIT_ERROR_IO" :                          \
                                (reason  == YMODEM_EXIT_ERROR_SYN ? "YMODEM_EXIT_ERROR_SYN" :                  \
                                (reason == YMODEM_EXIT_SENDER_QUIT ? "YMODEM_EXIT_SENDER_QUIT" :                  \
                                (reason == YMODEM_EXIT_MASTER_QUIT ? "YMODEM_EXIT_MASTER_QUIT": "UNKNOWN REASON")))))

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

struct ymodem_message_handler_parameter_s
{
  mqd_t mq;
  bool *working;
  char *path;
  bool test;
  ymodem_receiver_id slave;
};


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/



/****************************************************************************
 * Private Data
 ****************************************************************************/



/****************************************************************************
 * Private Functions
 ****************************************************************************/
static int get_time_interval(struct timeval start_time, struct timeval end_time)
{

  time_t seconds;
  long microseconds;
  uint32_t millseconds;

  seconds = end_time.tv_sec - start_time.tv_sec;
  if (seconds < 0)
    {
      return 0;
    }

  if (start_time.tv_usec > end_time.tv_usec)
    {
      if (seconds == 0)
        {
          return 0;
        }

      seconds --;
      microseconds = end_time.tv_usec + (USEC_PER_SEC - start_time.tv_usec);
    }
  else
    {
      microseconds = end_time.tv_usec - start_time.tv_usec;
    }

  millseconds = seconds * MSEC_PER_SEC + (microseconds / USEC_PER_MSEC);

  return millseconds;

}




static void *ymodem_receiver_message_handler(pthread_addr_t pvarg)
{
  struct ymodem_message_handler_parameter_s *param =
    (struct ymodem_message_handler_parameter_s *)pvarg;

  /*buffer for the message*/
  char *buffer_msg;
  struct mq_attr attr_mq;
  struct ymodem_message *ymodem_msg;

  int msg_bytes;/*received message size*/
  int received_bytes;/*how many data we received until now*/
  int file_size;/*current file size*/
  int percent;
  struct ymodem_buffer_s *data_buffer;

  int current_fd = 0;

  bool started = false;
  bool running = true;
  bool write_enable = true;
  int file_counter = 0;

  int buffer_length;
  int write_bytes;

  struct timeval start_time;
  struct timeval end_time;

  int millseconds;
  int speed = 0;

  *param->working = true;

  mq_getattr(param->mq, &attr_mq);

  buffer_msg = malloc(attr_mq.mq_msgsize);
  if (buffer_msg == NULL)
    {
      printf("MSG_handler: Failed to malloc message buffer, errno=%d\n", errno);
      goto errout_working;
    }

  while (running)
    {
      msg_bytes = mq_receive(param->mq, buffer_msg, attr_mq.mq_msgsize, 0);
      if (msg_bytes < 0)
        {
          if (errno != EINTR)
            {
              printf("MSG_handler: ERROR mq_receive failed: %d\n", errno);
            }

          continue;
        }

      ymodem_msg = (struct ymodem_message *)buffer_msg;

      switch (ymodem_msg->msg_type)
        {
          case YMODEM_MSG_START:
            if (!started)
              {
                started = true;
                file_counter = 0;
                /*we will discard all the received data if we are executing test command*/
                if (param->test)
                  {
                    write_enable = false;
                  }
                else
                  {
                    write_enable = true;
                  }
                gettimeofday(&start_time, NULL);
              }

            break;

          case YMODEM_MSG_FILE_HEADER:
            if (started)
              {

                file_counter ++;
                received_bytes = 0;
                file_size = ymodem_msg->msg_info.header_msg.file_size;

                printf("MSG_handler: Remote sender start sending a new file, filesize = %d\n", file_size);

                /*If this is not the first file, we discard this file*/
                if (file_counter > 1)
                  {
                    write_enable = false;
                    printf("MSG_handler: Discard this file, we currently don't support batch file transfer mode!\n");
                  }
                else
                  {
                    /*If this is the first file, we open it*/
                    current_fd = open(param->path, O_CREAT | O_WRONLY);
                    if (current_fd < 0)
                      {
                        printf("MSG_handler: open %s file failed: %d\n", current_fd, errno);
                        write_enable = false;
                        printf("MSG_handler: Discard this file\n");
                        continue;
                      }
                  }


              }
            break;


          case YMODEM_MSG_FILE_DATA:
            /*There may have some trash message left by perviouse file transfer,
             *so we start receiving DATA message until we got a START message.
             */
            if (started)
              {
                data_buffer = ymodem_msg->msg_info.data_msg.buf;
                buffer_length = data_buffer->nbytes  - data_buffer->curbyte;

                received_bytes += buffer_length;

                if (received_bytes > file_size)
                  {
                    printf("MSG_handler: Oops! we received more bytes then file size.\n");
                    write_enable = false;
                    continue;
                  }

                if (write_enable)
                  {

                    write_bytes = write(current_fd, &data_buffer->samp[data_buffer->curbyte], buffer_length);
                    if (write_bytes < buffer_length)
                      {
                        printf("MSG_handler: Encounter an error in write operation, errcode=%d\n", errno);
                      }

                    percent = (received_bytes * 100) / file_size;
                    printf("\nReceiving file, %d%% complete.\n", percent);
                  }
                else
                  {
                    if (!param->test)
                      {
                        printf("Discard a data package!\n");
                      }
                  }

                ymodem_buf_enqueue(param->slave, data_buffer);
              }
            break;
          case YMODEM_MSG_EXIT:
            if (started)
              {

                if (current_fd != 0)
                  {
                    close(current_fd);
                    current_fd = 0;
                  }

                printf("MSG_handler: ymodem receiver thread exit, reason:%s\n",
                       EXIT_REASON_TO_STR(ymodem_msg->msg_info.exit_msg.reason));

                gettimeofday(&end_time, NULL);
                millseconds = get_time_interval(start_time, end_time);
                speed = (received_bytes * 1000) / millseconds;
                printf("MSG_handler: Transfer %d file(s) in %d millseconds, speed %d Bps\n", file_counter, millseconds, speed);

                running = false;
              }
            break;
          default:
            printf("MSG_handler: Unrecognized message type!\n");
            break;
        }

    }


  printf("MSG_handler:exit!\n");
  free(buffer_msg);

errout_working:
  *param->working = false;

  return NULL;
}



/****************************************************************************
 * Name: ymodem_receiver_master_help
 *
 ****************************************************************************/

static void ymodem_receiver_master_help(void)
{
  printf("\n");
  printf("[start filepath], start the file transfer and put it in the specified filepath\n");
  printf("[stop], stop the transfer\n");
  printf("[pause/p], pause the current transfer\n");
  printf("[resume/r], resume the current transfer\n");
  printf("[quit/q], quit from this app\n");
  printf("\n");
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ymodem_receiver_main
 *
 *   ymodem_receiver_main() reads in commands from the console using the readline
 *   system add-in and implemets a command-line based ymodem receiver that
 *   uses the ymodem protocol to receiver files qnd put it in user specified file path.
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int ymodem_receiver_main(int argc, char *argv[])
#endif
{
  char                    buffer[64];
  int                     len, running;
  char                    *cmd, *arg;
  struct mq_attr attr;
  pthread_attr_t      tattr;
  mqd_t mq;
  int ret = OK;
  struct sched_param  sparam;
  pthread_t msg_thread_id;
  bool child_running = false;
  int source_fd;
  ymodem_receiver_id ymodem_salve;
  char *msq_name = "mqueue";
  char *user_path;
  int user_path_length;
  FAR void *p_return_value;
  int i;
  struct ymodem_buffer_s *buf_ptr[CONFIG_YMODEM_RECEIVER_BUF_NUM];
  struct ymodem_message_handler_parameter_s msg_handler_parameter;
  char des_path[CONFIG_PATH_MAX];


  printf("h for commands, q to exit\n");
  printf("\n");

  setlogmask(LOG_UPTO(LOG_ERR));

  /*create the msssage queue used for communication with ymodem slave pthread*/

  /* Fill in attributes for message queue */

  attr.mq_maxmsg  = CONFIG_YMODEM_RECEIVER_MAX_MSG;
  attr.mq_msgsize = CONFIG_YMODEM_RECEIVER_MSG_SIZE;
  attr.mq_flags   = 0;

  /*open a message queue*/
  mq = mq_open(msq_name, O_RDWR | O_CREAT, 0666, &attr);
  if (mq == (mqd_t) - 1)
    {
      printf("YMODEM_main: ERROR mq_open failed, errno=%d\n", errno);
      ret = errno;
      return ret;
    }

  /*malloc data buffers, receiver thread will load data to these buffers*/
  for (i = 0; i < CONFIG_YMODEM_RECEIVER_BUF_NUM; i++)
    {
      ret = ymodem_buf_alloc(&buf_ptr[i], CONFIG_YMODEM_RECEIVER_BUF_SIZE);
      if (ret < 0)
        {
          printf("YMODEM_main: Failed to malloc buffer, errno=%d\n", ret);
          goto   errout_mq;
        }
    }


  /*open source file*/
  source_fd = open(CONFIG_YMODEM_RECEIVER_SOURCE_DEVICE, O_RDWR | O_NONBLOCK);
  if (source_fd < 0)
    {
      printf("YMODEM_main: open %s device failed: %d\n", CONFIG_YMODEM_RECEIVER_SOURCE_DEVICE, errno);
      ret = errno;
      goto errout_buf;
    }

  /* Initialize ymodem slave thread */

  ret = ymodem_receiver_create(&ymodem_salve, source_fd, mq);
  if (ret < 0)
    {
      printf("YMODEM_main: create ymodem slave thread failed: %d\n", ret);
      goto errout;
    }

  /*enqueue all buffer*/
  for (i = 0; i < CONFIG_YMODEM_RECEIVER_BUF_NUM; i++)
    {
      ymodem_buf_enqueue(ymodem_salve, buf_ptr[i]);
    }


  /* Loop until the user exits */

  running = TRUE;
  while (running)
    {
      /* Print a prompt */

      printf("ymodem_receiver> ");
      fflush(stdout);

      /* Read a line from the terminal */

      len = readline(buffer, sizeof(buffer), stdin, stdout);
      buffer[len] = '\0';
      if (len > 0)
        {
          if (buffer[len - 1] == '\n')
            {
              buffer[len - 1] = '\0';
            }

          /* Parse the command from the argument */

          cmd = strtok_r(buffer, " \n", &arg);
          if (cmd == NULL)
            {
              continue;
            }

          /* Remove leading spaces from arg */

          while (*arg == ' ')
            {
              arg++;
            }

          if ((strcmp(cmd, "start") == 0) || (strcmp(cmd, "test") == 0))
            {
              if (!child_running)
                {
                  user_path = strtok_r(NULL, " \n", &arg);


                  /*set the destination file path, */
                  if (user_path == NULL)
                    {
                      strcpy(des_path, CONFIG_YMODEM_RECEIVER_DEFAULT_DEST_FILE);
                      printf("YMODEM_main: use default destination file path: %s\n", des_path);
                    }
                  else
                    {
                      user_path_length = strlen(user_path);
                      if (user_path_length < CONFIG_PATH_MAX)
                        {
                          strcpy(des_path, user_path);
                          printf("YMODEM_main: set destination file path: %s\n", des_path);
                        }
                      else
                        {
                          strcpy(des_path, CONFIG_YMODEM_RECEIVER_DEFAULT_DEST_FILE);
                          printf("YMODEM_main: user specified destination file path too long!\n");
                          printf("YMODEM_main: use default destination file path: %s\n", des_path);
                        }
                    }

                  /*prepare the parameters passed to the message handler thread*/
                  msg_handler_parameter.path = des_path;
                  msg_handler_parameter.mq = mq;
                  msg_handler_parameter.working = &child_running;
                  msg_handler_parameter.slave = ymodem_salve;

                  /*we will discard all the received data if we receive a test command*/
                  if (strcmp(cmd, "test") == 0)
                    {
                      msg_handler_parameter.test = true;
                    }
                  else
                    {
                      msg_handler_parameter.test = false;
                    }

                  /*create message handler thread*/
                  pthread_attr_init(&tattr);
                  sparam.sched_priority = SCHED_PRIORITY_DEFAULT;//sched_get_priority_max(SCHED_FIFO) - 9;
                  (void)pthread_attr_setschedparam(&tattr, &sparam);
                  (void)pthread_attr_setstacksize(&tattr, CONFIG_MSG_HANDLE_THREAD_STACK_SIZE);

                  ret = pthread_create(&msg_thread_id, &tattr, ymodem_receiver_message_handler,
                                       (pthread_addr_t) &msg_handler_parameter);
                  if (ret != OK)
                    {
                      printf("Failed to create msg_handler thread: %d\n", ret);
                      goto errout;
                    }

                  /* Name the thread */

                  pthread_setname_np(msg_thread_id, "ymodemMsgHandler");

                }
              else
                {
                  printf("YMODEM_main: ymodem receiver is running, stop it firstly!\n");
                }

              /*start the receiver thread*/
              ret = ymodem_receiver_start(ymodem_salve);

              if (ret != OK)
                {
                  printf("YMODEM_main:an error happend in start process:%d\n", ret);
                  goto errout;
                }
            }
          else if (strcmp(cmd, "stop") == 0)
            {
              ret = ymodem_receiver_stop(ymodem_salve);
              if (ret == OK)
                {
                  /*wait the message handler thread to exit*/
                  if (child_running)
                    {
                      pthread_join(msg_thread_id, &p_return_value);
                    }
                }
              else
                {
                  printf("YMODEM_main:an error happend in stop process:%d\n", ret);
                  goto errout;
                }
            }
          else if ((strcmp(cmd, "pause") == 0) || (strcmp(cmd, "p") == 0))
            {
              ret = ymodem_receiver_suspend(ymodem_salve);
              if (ret != OK)
                {
                  printf("YMODEM_main:an error happend in suspend process:%d\n", ret);
                  goto errout;
                }

            }
          else if ((strcmp(cmd, "resume") == 0) || (strcmp(cmd, "r") == 0))
            {
              ymodem_receiver_resume(ymodem_salve);
              if (ret != OK)
                {
                  printf("YMODEM_main:an error happend in resume process:%d\n", ret);
                  goto errout;
                }
            }
          else if ((strcmp(cmd, "q") == 0) || (strcmp(cmd, "quit") == 0))
            {
              ymodem_receiver_delete(ymodem_salve);

              /*wait the message handler thread to exit*/
              if (child_running)
                {
                  pthread_join(msg_thread_id, &p_return_value);
                }

              if (ret == OK)
                {
                  running = FALSE;
                }
              else
                {
                  printf("YMODEM_main:an error happend in delete process:%d\n", ret);
                  goto errout;
                }

            }
          else if ((strcmp(cmd, "h") == 0) || (strcmp(cmd, "help") == 0))
            {
              ymodem_receiver_master_help();
            }
          else
            {
              printf("%s:  unknown command\n", buffer);

              ymodem_receiver_master_help();
            }
        }
    }

errout:
  close(source_fd);             /* Close the device */

errout_buf:
  for (i = 0; i < CONFIG_YMODEM_RECEIVER_BUF_NUM; i++)
    {
      ymodem_buf_free(buf_ptr[i]);
    }

errout_mq:
  mq_close(mq);                  /* Close the message queue */
  mq_unlink(msq_name);           /* Unlink the message queue */

  return ret;
}

#endif /*CONFIG_SYSTEM_YMODEM_RECEIVER*/

