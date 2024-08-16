/****************************************************************************
 * arch/arm/src/armv7-m/up_segger_rtt_console.c
 *
 *   Copyright (C) 2018, 2019 Gregory Nutt. All rights reserved.
 *   Author: Zhiqiang Li <zhiqiang@zglue.com>
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
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/serial/serial.h>
#include <stdint.h>
#include "up_internal.h"
#include "up_arch.h"

#include <sysview/SEGGER_RTT.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/
typedef struct
{

  pthread_t       thread_tx;
  pthread_t       thread_rx;
  sem_t           sem_tx;
  sem_t           sem_rx;

  uint8_t         rx_status;
  uint8_t         tx_status;

} rtt_config;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/



/****************************************************************************
 * Private Variables
 ****************************************************************************/
static int  rtt_setup(struct uart_dev_s *dev);
static void rtt_shutdown(struct uart_dev_s *dev);
static int  rtt_attach(struct uart_dev_s *dev);
static void rtt_detach(struct uart_dev_s *dev);

static int  rtt_receive(struct uart_dev_s *dev, unsigned int *status);
static void rtt_rxint(struct uart_dev_s *dev, bool enable);
static bool rtt_rxavailable(struct uart_dev_s *dev);

static void rtt_txint(struct uart_dev_s *dev, bool enable);
static bool rtt_txready(struct uart_dev_s *dev);
static void rtt_send(struct uart_dev_s *dev, int ch);


/****************************************************************************
 * Private Functions
 ****************************************************************************/

static char rtt_rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char rtt_txbuffer[CONFIG_UART0_TXBUFSIZE];
static rtt_config g_rtt;
static struct uart_ops_s g_rtt_ops =
{
  .setup          = rtt_setup,
  .shutdown       = rtt_shutdown,
  .attach         = rtt_attach,
  .detach         = rtt_detach,
  .ioctl          = NULL,

  .receive        = rtt_receive,
  .rxint          = rtt_rxint,
  .rxavailable    = rtt_rxavailable,

  .send           = rtt_send,
  .txint          = rtt_txint,
  .txready        = rtt_txready,
  .txempty        = rtt_txready,
};

static uart_dev_t rtt_console =
{

  .isconsole = true,
  .recv     =
  {
    .size   = CONFIG_UART0_RXBUFSIZE,
    .buffer = rtt_rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART0_TXBUFSIZE,
    .buffer = rtt_txbuffer,
  },
  .ops      = &g_rtt_ops,
  .priv     = &g_rtt,
};
/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtt_txready
 *
 * Description:
 *   Return true if the transmitter is ready. This function has special case
 *    handled with the tx_enable_firstime flag. In the nordic nrf52 there is
 *    no way to find if the TX is ready for the first time wihtout writing
 *    data. Writing data generates the TXDRDY signal . This is resolved with
 *    the help of the flag "tx_enable_firstime"
 *
 ****************************************************************************/
static bool rtt_txready(FAR struct uart_dev_s *dev)
{
  return true;
}

/****************************************************************************
 * Name: rtt_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/
static void rtt_send(FAR struct uart_dev_s *dev, int ch)
{
  SEGGER_RTT_PutChar(0, (char)ch);
  return;
}

/****************************************************************************
 * Name: rtt_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool rtt_rxavailable(struct uart_dev_s *dev)
{
  return (bool)SEGGER_RTT_HasKey();
}

/****************************************************************************
 * Name: rtt_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the USART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int rtt_receive(FAR struct uart_dev_s *dev, uint32_t *status)
{
  return SEGGER_RTT_GetKey();
}

/****************************************************************************
 * Name: rtt_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static int rtt_serial_rx_thread(int argc, char **args)
{
  rtt_config *config = &g_rtt;
  struct timespec timeout;

  timeout.tv_sec  = 0;
  timeout.tv_nsec = 10 * 1000000;
  while (1)
    {
      sem_timedwait(&config->sem_rx, &timeout);

      do
        {
          if (config->rx_status)
            {
              uart_recvchars((FAR uart_dev_t *)&rtt_console);
            }
          else
            {
              break;
            }
        }
      while (1);
    }

  return 0;
}

static void rtt_rxint(struct uart_dev_s *dev, bool enable)
{
  rtt_config *config = (rtt_config *)dev->priv;

  if (enable)
    {
      sem_post(&config->sem_rx);
    }

  config->rx_status = enable;

  return;
}

/****************************************************************************
 * Name: rtt_serial_tx_thread
 *
 * Description:
 *   This thread to receive data from RTT and put it into serial HAL buffer
 *
 ****************************************************************************/

static int rtt_serial_tx_thread(int argc, char **args)
{
  rtt_config *config = &g_rtt;

  if (0 == config->thread_rx)
    {
#ifdef CONFIG_BUILD_PROTECTED
      config->thread_rx = kthread_create("RTT_RX", 128, 1024, rtt_serial_rx_thread, NULL);
#else
      pthread_create(&config->thread_rx, NULL, (pthread_startroutine_t)rtt_serial_rx_thread, NULL);
#endif
    }

  while (1)
    {
      sem_wait(&config->sem_tx);

      do
        {
          if (config->tx_status)
            {
              uart_xmitchars((FAR uart_dev_t *)&rtt_console);
            }
          else
            {
              break;
            }
        }
      while (1);
    }

  return 0;
}


static void rtt_txint(struct uart_dev_s *dev, bool enable)
{
  rtt_config *config = (rtt_config *)dev->priv;

  if (enable)
    {
      if (0 == config->thread_tx)
        {
#ifdef CONFIG_BUILD_PROTECTED
          config->thread_tx = kthread_create("RTT_TX", 128, 1024, rtt_serial_tx_thread, NULL);
#else
          pthread_create(&config->thread_tx, NULL, (pthread_startroutine_t)rtt_serial_tx_thread, NULL);
#endif

        }

      sem_post(&config->sem_tx);
    }
  config->tx_status = enable;

  return;
}

/****************************************************************************
 * Name: rtt_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int rtt_setup(FAR struct uart_dev_s *dev)
{
  int ret = OK;

  return ret;
}

/****************************************************************************
 * Name: rtt_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void rtt_shutdown(FAR struct uart_dev_s *p_dev)
{
  return;
}

/****************************************************************************
 * Name: rtt_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled then by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() methods are called.
 *
 ****************************************************************************/

static int rtt_attach(FAR struct uart_dev_s *dev)
{
  int ret = OK;

  return ret;
}

/****************************************************************************
 * Name: rtt_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void rtt_detach(struct uart_dev_s *dev)
{
  return;
}


void up_lowputc(char ch)
{
  SEGGER_RTT_PutChar(0, (char)ch);

  return;
}

/****************************************************************************
 * Name: segger_rtt_lowsetup
 *
 * Description:
 *   This performs basic initialization of the USART used for the serial
 *   console.  Its purpose is to get the console output availabe as soon
 *   as possible.
 *
 ****************************************************************************/

void segger_rtt_lowsetup(void)
{
}

void segger_rtt_register_console(void)
{

  sem_init(&g_rtt.sem_tx, 0, 0);
  sem_init(&g_rtt.sem_rx, 0, 0);

  (void)uart_register("/dev/console", (uart_dev_t *)&rtt_console);

  return;
}

