/****************************************************************************
 * arch/arm/src/nrf52/nrf52_serial.c
 *
 *   Copyright (C) 2017, 2018 Zglue Inc. All rights reserved.
 *   Author: Bill Rees <bill@zglue.com>
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
#include <nuttx/clock.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>
#include <nuttx/power/pm.h>
#include <nuttx/kthread.h>
#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <arch/serial.h>
#include <arch/board/board.h>

#include "chip.h"
#include "nvic.h"
#include "up_arch.h"
#include "up_internal.h"
#include "nrf.h"
#include "nrf52_uart.h"
#include "nrf52_gpio.h"
#include "nrf_uarte.h"
#include "nrf_uart.h"
#include "up_arch.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
typedef struct
{
  uint32_t  baudrate;
  uint32_t  config;
} nrf_baudrate_config_t;

static const nrf_baudrate_config_t  nrf_baudrate[] =
{
  {1200,     0x0004F000},
  {2400,     0x0009D000},
  {4800,     0x0013B000},
  {9600,     0x00275000},
  {14400,    0x003AF000},
  {19200,    0x004EA000},
  {28800,    0x0075C000},
  {31250,    0x00800000},
  {38400,    0x009D0000},
  {56000,    0x00E50000},
  {57600,    0x00EB0000},
  {76800,    0x013A9000},
  {115200,   0x01D60000},
  {230400,   0x03B00000},
  {250000,   0x04000000},
  {460800,   0x07400000},
  {921600,   0x0F000000},
  {1000000,  0x10000000},
};

#ifdef USE_SERIALDRIVER
/* always define uart count as 2 */
#define UART_ENABLED_COUNT 2

#ifndef UART_DEFAULT_CONFIG_IRQ_PRIORITY
#define UART_DEFAULT_CONFIG_IRQ_PRIORITY NVIC_SYSH_MAXNORMAL_PRIORITY
#endif

#define CONFIG_UART_MONITOR_PRIORITY 50

#define NRF52_XFER_STATUS_INIT            0  /* : not been fill up */
#define NRF52_XFER_STATUS_FULL            1  /* : BUFFER is full */
#define NRF52_XFER_STATUS_TRANSFERRING    2  /* had been set into hardware */

typedef struct tx_config_s
{
  struct tx_config_s *next;
  uint8_t  *buf;
  uint8_t  index;   /*actual write index */
  uint8_t  len;     /*total lentgh buffer */
  uint8_t  status;
} tx_config_t;

typedef struct rx_config_s
{
  uint8_t  *buf;
  uint8_t  index;  /* read index */
  uint8_t  bytes;  /* atcual receive bytes in buf */
  uint8_t  len;    /*total lentgh of buffer */
  uint8_t  status;
} rx_config_t;

#define NRF52_UART_TX_BUF_CNT  2

struct nrf52_uart_s
{
  struct uart_dev_s   dev;       /* Generic UART device */
  uart_config_t       *config;    /* serial atrribute config */

  uint32_t          rx_intmask;   /* Set of receive interrupts to manage. */
  uint32_t          tx_intmask;   /* Set of trasmit interrupts to manage. */
  uint8_t           irq_prio;  /* Priority. */
  uint8_t           irq;       /* IRQ associated with this USART */

  /* If termios are supported, then the following fields may vary at
   * runtime.
   */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  bool                iflow;     /* input flow control (RTS) enabled */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  bool                oflow;     /* output flow control (CTS) enabled */
#endif
  uint8_t         thread_rdy;
  uint8_t         legacy_status; /* use for legacy driver */
  pthread_t       thread;        /* thread for legacy send */
  sem_t           sem;           /* sem for legacy thread */

  uint8_t         rx_status; /* init; transfering */
  uint8_t         tx_status; /* init; transfering */

  tx_config_t     tx_xfer[NRF52_UART_TX_BUF_CNT];
  rx_config_t     rx_xfer;

  sq_queue_t      tx_queue_empty;
  sq_queue_t      tx_queue_full;

};

/* we will always use UARTE interface and it can act as tranditional behaviour */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
/* static int  up_interrupt(int irq, void *context, void *arg); */
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);

static int  up_receive(struct uart_dev_s *dev, unsigned int *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);

static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);

#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool up_rxflowcontrol(struct uart_dev_s *dev, unsigned int nbuffered,
                             bool upper);
#endif

#ifdef CONFIG_SERIAL_DMA
static void up_dma_receive(struct uart_dev_s *dev);
static void up_dma_send(struct uart_dev_s *dev);
static void up_dma_rxfree(struct uart_dev_s *dev);
static void up_dma_txavail(struct uart_dev_s *dev);

#endif


static uint32_t nrf52_uart_set_baudrate(uart_config_t *p_config);

/****************************************************************************
 * Private Variables
 ****************************************************************************/
#define NRF52_UART_EASYDMA_BUF_LEN  8

#ifdef CONFIG_NRF52_UART0
/* buffer for middle layer serial usage */
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];

/* buffer for easyDMA usage */
static char g_uart0_dma_rxbuffer[NRF52_UART_EASYDMA_BUF_LEN];
static char g_uart0_dma_txbuffer[NRF52_UART_EASYDMA_BUF_LEN];

static const uart_config_t g_uart0 =
{
  .uartbase       = NRF_UARTE0_BASE,
#ifdef CONFIG_UART0_DMA
#if CONSOLE_UART == 1
  .easydma     = false,
#else
  .easydma     = true,
#endif
#else
  .easydma     = false,
#endif
  .rxd_pin     = BOARD_UART0_RX_PIN,
  .txd_pin     = BOARD_UART0_TX_PIN,
  .cts_pin     = BOARD_UART0_CTS_PIN,
  .rts_pin     = BOARD_UART0_RTS_PIN,

  .parity      = CONFIG_UART0_PARITY,
  .bits        = CONFIG_UART0_BITS,
  .stopbits2   = CONFIG_UART0_2STOP,
  .baud_rate   = CONFIG_UART0_BAUD,
  .rx_len      = NRF52_UART_EASYDMA_BUF_LEN,
  .rx_buf      = (uint8_t *)g_uart0_dma_rxbuffer,
  .tx_len      = NRF52_UART_EASYDMA_BUF_LEN,
  .tx_buf      = (uint8_t *)g_uart0_dma_txbuffer,
};
#endif

#ifdef CONFIG_NRF52_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];

static char g_uart1_dma_rxbuffer[NRF52_UART_EASYDMA_BUF_LEN];
static char g_uart1_dma_txbuffer[NRF52_UART_EASYDMA_BUF_LEN];

static const uart_config_t g_uart1 =
{
  .uartbase       = NRF_UARTE1_BASE,
#ifdef CONFIG_UART1_DMA
#if CONSOLE_UART == 2
  .easydma     = false,
#else
  .easydma     = true,
#endif
#else
  .easydma     = false,
#endif
  .rxd_pin     = BOARD_UART1_RX_PIN,
  .txd_pin     = BOARD_UART1_TX_PIN,
  .cts_pin     = BOARD_UART1_CTS_PIN,
  .rts_pin     = BOARD_UART1_RTS_PIN,

  .parity      = CONFIG_UART1_PARITY,
  .bits        = CONFIG_UART1_BITS,
  .stopbits2   = CONFIG_UART1_2STOP,
  .baud_rate   = CONFIG_UART1_BAUD,
  .rx_len      = NRF52_UART_EASYDMA_BUF_LEN,
  .rx_buf      = (uint8_t *)g_uart1_dma_rxbuffer,
  .tx_len      = NRF52_UART_EASYDMA_BUF_LEN,
  .tx_buf      = (uint8_t *)g_uart1_dma_txbuffer,
};

#endif


#define WAIT_FOR_UART_EVENT(a, b) \
      while (!nrf_uarte_event_check(a, b)) {}

static const struct uart_ops_s g_uart_ops =
{
  .setup          = up_setup,
  .shutdown       = up_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .ioctl          = up_ioctl,

  .receive        = up_receive,
  .rxint          = up_rxint,
  .rxavailable    = up_rxavailable,

  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txready,

#ifdef CONFIG_SERIAL_DMA
  .dmasend        = up_dma_send,
  .dmareceive     = up_dma_receive,
  .dmarxfree      = up_dma_rxfree,
  .dmatxavail     = up_dma_txavail,
#endif

#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = up_rxflowcontrol,
#endif
};

/* This describes the state of the UART port. */
#ifdef CONFIG_NRF52_UART0
static struct nrf52_uart_s g_uart0priv =
{
  .dev =
  {
#if CONSOLE_UART == 1
    .isconsole = true,
#endif
    .recv     =
    {
      .size   = CONFIG_UART0_RXBUFSIZE,
      .buffer = g_uart0rxbuffer,
    },
    .xmit     =
    {
      .size   = CONFIG_UART0_TXBUFSIZE,
      .buffer = g_uart0txbuffer,
    },
    .ops      = &g_uart_ops,
    .priv     = &g_uart0priv,
  },
  .config     = (uart_config_t *) &g_uart0,

  .tx_intmask     = NRF_UARTE_INT_ENDTX_MASK,
  .rx_intmask     = NRF_UARTE_INT_ENDRX_MASK,
  .irq            = UARTE0_UART0_IRQn,
  .irq_prio       = UART_DEFAULT_CONFIG_IRQ_PRIORITY,
#if defined(CONFIG_SERIAL_IFLOWCONTROL)
  .iflow          = true,
#endif
#if defined(CONFIG_SERIAL_OFLOWCONTROL)
  .oflow          = true,
#endif
};
#endif

#ifdef CONFIG_NRF52_UART1
static struct nrf52_uart_s g_uart1priv =
{
  .dev =
  {
#if CONSOLE_UART == 2
    .isconsole = true,
#endif
    .recv     =
    {
      .size   = CONFIG_UART1_RXBUFSIZE,
      .buffer = g_uart1rxbuffer,
    },
    .xmit     =
    {
      .size   = CONFIG_UART1_TXBUFSIZE,
      .buffer = g_uart1txbuffer,
    },
    .ops      = &g_uart_ops,
    .priv     = &g_uart1priv,
  },
  .config     = (uart_config_t *) &g_uart1,

  .tx_intmask     = NRF_UARTE_INT_ENDTX_MASK,
  .rx_intmask     = NRF_UARTE_INT_ENDRX_MASK,
  .irq            = UARTE1_IRQn,
  .irq_prio       = UART_DEFAULT_CONFIG_IRQ_PRIORITY,
#if defined(CONFIG_SERIAL_IFLOWCONTROL)
  .iflow          = true,
#endif
#if defined(CONFIG_SERIAL_OFLOWCONTROL)
  .oflow          = true,
#endif
};
#endif

/* This table lets us iterate over the configured UARTs */
FAR static const struct uart_dev_s * const g_uart_devs[UART_ENABLED_COUNT] =
{
#ifdef CONFIG_NRF52_UART0
  [0] = (struct uart_dev_s *) &g_uart0priv,
#else
  [0] = NULL,
#endif

#ifdef CONFIG_NRF52_UART1
  [1] = (struct uart_dev_s *) &g_uart1priv,
#else
  [1] = NULL,
#endif

};


/****************************************************************************
 * Name: up_enableuart
 *
 * Description:
 *   Disables the Nordic NRF52 UART
 *
 ****************************************************************************/
static inline void up_enableuart(const struct nrf52_uart_s *p_dev)
{
  NRF_UARTE_Type *p_uart;
  p_uart = (NRF_UARTE_Type *)(p_dev->config->uartbase);

  if (p_dev->config->easydma)
    {
      nrf_uarte_enable(p_uart);
    }
  else
    {
      nrf_uart_enable((NRF_UART_Type *)p_uart);
    }
}

/****************************************************************************
 * Name: up_disableuart
 *
 * Description:
 *   Disables the Nordic NRF52 UART
 *
 ****************************************************************************/
static inline void up_disableuart(const struct nrf52_uart_s *p_dev)
{
  NRF_UARTE_Type *p_uart;
  p_uart = (NRF_UARTE_Type *)(p_dev->config->uartbase);

  nrf_uarte_int_disable(p_uart, 0xffffffff);

  nrf_uarte_disable(p_uart);
}

/****************************************************************************
 * Name: nrf_drv_uart_errorsrc_get
 *
 * Description:
 *   Nordic nrf52 function to get the error source when the UART hits an error
 *
 ****************************************************************************/
uint32_t nrf52_uart_errorsrc_get(const struct nrf52_uart_s *p_dev)
{
  NRF_UARTE_Type *p_uart;
  uint32_t status;

  p_uart = (NRF_UARTE_Type *)p_dev->config->uartbase;
  status = nrf_uarte_errorsrc_get_and_clear(p_uart);
  nrf_uarte_event_clear(p_uart, NRF_UARTE_EVENT_ERROR);

  return status;
}

static inline void nrf52_uart_setup_tx(struct nrf52_uart_s *priv, tx_config_t *tx)
{
  NRF_UARTE_Type *p_uart;

  p_uart = (NRF_UARTE_Type *)priv->config->uartbase;

  tx->status = NRF52_XFER_STATUS_TRANSFERRING;
  /* use index instead of len */
  nrf_uarte_tx_buffer_set(p_uart, tx->buf, tx->index);
  nrf_uarte_task_trigger(p_uart, NRF_UARTE_TASK_STARTTX);
  priv->tx_status = NRF52_XFER_STATUS_TRANSFERRING;
}

static inline void nrf52_uart_setup_rx(struct nrf52_uart_s *priv, rx_config_t *rx)
{
  NRF_UARTE_Type *p_uart;

  p_uart = (NRF_UARTE_Type *)priv->config->uartbase;

  rx->status = NRF52_XFER_STATUS_TRANSFERRING;
  rx->index = 0;
  rx->bytes = 0;

  nrf_uarte_int_enable(p_uart, NRF_UARTE_INT_RXDRDY_MASK);
  nrf_uarte_rx_buffer_set(p_uart, rx->buf, rx->len);
  nrf_uarte_task_trigger(p_uart, NRF_UARTE_TASK_STARTRX);
  priv->rx_status = NRF52_XFER_STATUS_TRANSFERRING;
}

static inline tx_config_t *nrf52_uart_find_txavail(FAR struct nrf52_uart_s *dev)
{

  if (sq_peek(&dev->tx_queue_empty))
    {
      return (tx_config_t *)sq_peek(&dev->tx_queue_empty);
    }

  return NULL;
}

static inline tx_config_t *nrf52_uart_find_txfull(FAR struct nrf52_uart_s *dev)
{

  if (sq_peek(&dev->tx_queue_full))
    {
      return (tx_config_t *)sq_peek(&dev->tx_queue_full);
    }

  return NULL;
}

int nrf52_uart_xmit_done(struct nrf52_uart_s *priv, NRF_UARTE_Type *uart)
{
  uint8_t *p_buf = (uint8_t *)uart->TXD.PTR;
  tx_config_t *tx = (tx_config_t *)sq_peek(&priv->tx_queue_full);

  priv->tx_status = NRF52_XFER_STATUS_INIT;

  if (p_buf == tx->buf)
    {
      sq_remfirst(&priv->tx_queue_full);
      tx->status = NRF52_XFER_STATUS_INIT;
      tx->index = 0;
      sq_addlast((FAR sq_entry_t *)tx, &priv->tx_queue_empty);
      return true;
    }

#ifdef CONFIG_SERIAL_DMA
  /* handle DMA transfer */
  else
    {
      uint32_t  xmit_bytes = uart->TXD.AMOUNT;

      ASSERT(p_buf == (uint8_t *)priv->dev.dmatx.buffer);
      priv->dev.dmatx.nbytes = xmit_bytes;
      uart_xmitchars_done(&priv->dev);
    }
#endif

  return false;
}

static inline rx_config_t *nrf52_uart_find_rxavail(FAR struct nrf52_uart_s *priv)
{

  if (NRF52_XFER_STATUS_INIT == priv->rx_xfer.status )
    {
      return &priv->rx_xfer;
    }

  return NULL;
}
static inline rx_config_t *nrf52_uart_find_rxfull(struct nrf52_uart_s *priv )
{

  if ((NRF52_XFER_STATUS_FULL == priv->rx_xfer.status ||
       NRF52_XFER_STATUS_TRANSFERRING == priv->rx_xfer.status) &&
      (priv->rx_xfer.index != priv->rx_xfer.bytes))
    {
      return &priv->rx_xfer;
    }

  return NULL;

}

static int nrf52_uart_recv_done(struct nrf52_uart_s *priv, NRF_UARTE_Type *uart)
{
  uint32_t  recv_bytes = uart->RXD.AMOUNT;
  uint8_t *p_buf = (uint8_t *)uart->RXD.PTR;
  rx_config_t *rx = &priv->rx_xfer;

  priv->rx_status = NRF52_XFER_STATUS_INIT;

  if (p_buf == priv->rx_xfer.buf)
    {
      /*if there is data and read index is less than buffer data */
      if (recv_bytes && recv_bytes < rx->index)
        {
          rx->status = NRF52_XFER_STATUS_FULL;
          rx->bytes = recv_bytes;
          /* if this is internal buffer, put data to up-layer serial driver */
          uart_recvchars(&priv->dev);
          return true;
        }
      else
        {
          rx->status = NRF52_XFER_STATUS_INIT;
        }
      return false;
    }
#ifdef CONFIG_SERIAL_DMA
  /* handle DMA transfer */
  else
    {
      ASSERT(p_buf == (uint8_t *)priv->dev.dmarx.buffer );
      priv->dev.dmarx.nbytes = recv_bytes;
      uart_recvchars_done(&priv->dev);
    }
#endif

  return false;
}

/****************************************************************************
 * Name: nrf52_uart_irqhandler
 *
 * Description:
 *   Main IRQ handler for the UART. Handles TX, RX, Error and Timeout cases
 *
 ****************************************************************************/

static int nrf52_uart_irqhandler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct nrf52_uart_s *p_dev = (FAR struct nrf52_uart_s *)arg;
  NRF_UARTE_Type *p_uart;

  p_uart = (NRF_UARTE_Type *)p_dev->config->uartbase;

  /* Handle case where there is an error in UART */
  if (nrf_uarte_event_check(p_uart, NRF_UARTE_EVENT_ERROR))
    {
      /* Get the error info. */
      uint32_t status = nrf52_uart_errorsrc_get(p_dev);
      UNUSED(status);
    }
  else if (nrf_uarte_int_enable_check(p_uart, NRF_UARTE_INT_RXDRDY_MASK) &&
           nrf_uarte_event_check(p_uart, NRF_UARTE_EVENT_RXDRDY))
    {

      /* This handler the UART receive case. uart_recvchars is a nuttx function
        that takes characters and puts it in the fifo  */
#if CONFIG_NFILE_DESCRIPTORS > 0
      if (p_dev->config->easydma && p_uart->RXD.PTR == (uint32_t)p_dev->rx_xfer.buf)
        {
          nrf_uarte_event_clear(p_uart, NRF_UARTE_EVENT_RXDRDY);
          p_dev->rx_xfer.bytes++;
        }

      uart_recvchars(&p_dev->dev);
#endif
    }

  if (nrf_uarte_int_enable_check(p_uart, NRF_UARTE_INT_ENDRX_MASK) &&
      nrf_uarte_event_check(p_uart, NRF_UARTE_EVENT_ENDRX))
    {

      nrf_uarte_event_clear(p_uart, NRF_UARTE_EVENT_ENDRX);

      nrf52_uart_recv_done(p_dev, p_uart);

      /* if dma is enable , first try DMA */
#ifdef CONFIG_SERIAL_DMA
      uart_recvchars_dma(&p_dev->dev);
#endif

      if (NRF52_XFER_STATUS_INIT == p_dev->rx_status)
        {
          rx_config_t *p_rx_new = nrf52_uart_find_rxavail(p_dev);

          if (p_rx_new)
            {
              nrf52_uart_setup_rx(p_dev, p_rx_new);
            }
        }

    }

  if (nrf_uarte_int_enable_check(p_uart, NRF_UARTE_INT_TXDRDY_MASK) &&
      nrf_uarte_event_check(p_uart, NRF_UARTE_EVENT_TXDRDY))
    {
      nrf_uarte_event_clear(p_uart, NRF_UARTE_EVENT_TXDRDY);

      /* This handler the UART transmit case. uart_xmitchars is a nuttx function
      that send characters from the fifo  */
#if CONFIG_NFILE_DESCRIPTORS > 0
//     uart_xmitchars(&p_dev->dev);
#endif
    }

  if (nrf_uarte_int_enable_check(p_uart, NRF_UARTE_INT_ENDTX_MASK) && \
      nrf_uarte_event_check(p_uart, NRF_UARTE_EVENT_ENDTX))
    {

      nrf_uarte_event_clear(p_uart, NRF_UARTE_EVENT_ENDTX);

      /* handle xmit done firstly */
      nrf52_uart_xmit_done(p_dev, p_uart);

      /* if dma is enable , first try DMA */
#ifdef CONFIG_SERIAL_DMA
      uart_xmitchars_dma(&p_dev->dev);
#endif

      if (NRF52_XFER_STATUS_INIT == p_dev->tx_status)
        {
          uart_xmitchars(&p_dev->dev);

          tx_config_t *p_tx_new = nrf52_uart_find_txfull(p_dev);

          if (p_tx_new)
            {
              nrf52_uart_setup_tx(p_dev, p_tx_new);
            }
        }

    }

  /* Handle the RXTO event */
  if (nrf_uarte_event_check(p_uart, NRF_UARTE_EVENT_RXTO))
    {
      nrf_uarte_event_clear(p_uart, NRF_UARTE_EVENT_RXTO);
    }


  return OK;
}

#ifdef CONFIG_SERIAL_DMA
static inline void nrf52_uart_setup_rx_dma(struct nrf52_uart_s *priv)
{
  NRF_UARTE_Type *p_uart = (NRF_UARTE_Type *)priv->config->uartbase;

  /* disable every RXD ready interrupt for buffer recv mode */
  nrf_uarte_int_disable(p_uart, NRF_UARTE_INT_RXDRDY_MASK);

  nrf_uarte_rx_buffer_set(p_uart, (uint8_t *)priv->dev.dmarx.buffer, priv->dev.dmarx.length);
  nrf_uarte_task_trigger(p_uart, NRF_UARTE_TASK_STARTRX);
  priv->rx_status = NRF52_XFER_STATUS_TRANSFERRING;
}

static void up_dma_receive(struct uart_dev_s *dev)
{
#if 0
  /* issue here: hal serial will recv all data by recvchars_dma,
   * but it mis-match with application layer requirement like:
   * app_read(buf, 1) , this will cause easyDMA can NOT feedback
   * data immediately . So , disable RX_DMA here
   */

  irqstate_t flag;
  flag = enter_critical_section();
  struct nrf52_uart_s *priv = (struct nrf52_uart_s *)dev;

  if (NRF52_XFER_STATUS_INIT == priv->tx_status)
    {
      nrf52_uart_setup_rx_dma(priv);
    }
  leave_critical_section(flag);
#endif
}

static void up_dma_rxfree(struct uart_dev_s *dev)
{
#if 0
  /* issue here: hal serial will recv all data by recvchars_dma,
   * but it mis-match with application layer requirement like:
   * app_read(buf, 1) , this will cause easyDMA can NOT feedback
   * data immediately . So , disable RX_DMA here
   */
  irqstate_t flag;
  flag = enter_critical_section();

  uart_recvchars_dma(dev);

  leave_critical_section(flag);
#endif
}

static inline void nrf52_uart_setup_tx_dma(struct nrf52_uart_s *priv)
{
  NRF_UARTE_Type *p_uart = (NRF_UARTE_Type *)priv->config->uartbase;

  nrf_uarte_tx_buffer_set(p_uart, (uint8_t *)priv->dev.dmatx.buffer, priv->dev.dmatx.length);
  nrf_uarte_task_trigger(p_uart, NRF_UARTE_TASK_STARTTX);
  priv->tx_status = NRF52_XFER_STATUS_TRANSFERRING;
}


static void up_dma_send(struct uart_dev_s *dev)
{
  struct nrf52_uart_s *priv = (struct nrf52_uart_s *)dev;

  irqstate_t flag;
  flag = enter_critical_section();

  if (NRF52_XFER_STATUS_INIT == priv->tx_status)
    {
      nrf52_uart_setup_tx_dma(priv);
    }
  leave_critical_section(flag);
}

static void up_dma_txavail(struct uart_dev_s *dev)
{
  struct nrf52_uart_s *priv = (struct nrf52_uart_s *)dev;

  if (priv->config->easydma)
    {
      irqstate_t flag;
      flag = enter_critical_section();

      uart_xmitchars_dma(dev);

      leave_critical_section(flag);
    }
}

#endif

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the transmitter is ready. This function has special case
 *    handled with the tx_enable_firstime flag. In the nordic nrf52 there is
 *    no way to find if the TX is ready for the first time wihtout writing
 *    data. Writing data generates the TXDRDY signal . This is resolved with
 *    the help of the flag "tx_enable_firstime"
 *
 ****************************************************************************/
static bool up_txready(FAR struct uart_dev_s *dev)
{
  struct nrf52_uart_s *priv = (struct nrf52_uart_s *)dev;

  if (priv->config->easydma)
    {
      if (nrf52_uart_find_txavail(priv))
        {
          return true;
        }
      else
        {
          return false;
        }
    }
  else
    {
      /* legacy driver , it always ready */
      return true;
    }
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/
static void up_send(FAR struct uart_dev_s *dev, int ch)
{
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
  struct nrf52_uart_s *priv = (struct nrf52_uart_s *)dev;

  if (priv->config->easydma)
    {
      irqstate_t flag = enter_critical_section();

      tx_config_t *tx = nrf52_uart_find_txavail(priv);

      if (tx)
        {
          tx->buf[tx->index++] = (uint8_t)ch;
          if (tx->index == tx->len)
            {
              tx->status = NRF52_XFER_STATUS_FULL;
              sq_remfirst(&priv->tx_queue_empty);
              sq_addlast((FAR sq_entry_t *)tx, &priv->tx_queue_full);
            }
        }

      leave_critical_section(flag);
    }
  else
    {
      nrf52_uart_legacy_send(priv->config->uartbase, (uint8_t)ch);
    }
#endif
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct nrf52_uart_s *priv = (struct nrf52_uart_s *)dev;

  if (priv->config->easydma)
    {
      if (nrf52_uart_find_rxfull(priv))
        {
          return true;
        }
      else
        {
          return false;
        }
    }
  else
    {
      return nrf_uarte_event_check((NRF_UARTE_Type *)priv->config->uartbase,
                                   NRF_UARTE_EVENT_RXDRDY);
    }
}

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the USART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int up_receive(FAR struct uart_dev_s *dev, uint32_t *status)
{
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
  int ch = ' ';
  struct nrf52_uart_s *priv = (struct nrf52_uart_s *)dev;

  if (priv->config->easydma)
    {
      rx_config_t *rx = nrf52_uart_find_rxfull(priv);

      if (rx)
        {
          ch = (int)rx->buf[rx->index++];
          if (rx->index >= rx->bytes && NRF52_XFER_STATUS_FULL == rx->status)
            {
              rx->status = NRF52_XFER_STATUS_INIT;
              rx->index = 0;
              rx->bytes = 0;
            }
        }
    }
  else
    {
      NRF_UART_Type *p_uart = (NRF_UART_Type *)priv->config->uartbase;
      nrf_uarte_event_clear((NRF_UARTE_Type *)p_uart, NRF_UARTE_EVENT_RXDRDY);
      ch = (int)p_uart->RXD;
    }
  return ch;

#endif
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void up_rxint(struct uart_dev_s *dev, bool enable)
{
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
  struct nrf52_uart_s *p_dev = (struct nrf52_uart_s *)dev;
  irqstate_t flags;
  NRF_UARTE_Type *p_uart;
  p_uart = (NRF_UARTE_Type *)p_dev->config->uartbase;

  flags = enter_critical_section();

  if (p_dev->config->easydma)
    {
      if (enable)
        {
          // Make sure the UART's rx int bit is enabled so the event will fire.
          if (!nrf_uarte_int_enable_check(p_uart, p_dev->rx_intmask))
            {
              nrf_uarte_int_enable(p_uart, (p_dev->rx_intmask | NRF_UARTE_INT_ERROR_MASK));
            }

          if (NRF52_XFER_STATUS_INIT == p_dev->rx_status)
            {
              rx_config_t *rx = nrf52_uart_find_rxavail(p_dev);
              if (rx)
                {
                  nrf52_uart_setup_rx( p_dev, rx);
                }
            }
        }
      else
        {

          if (NRF52_XFER_STATUS_TRANSFERRING == p_dev->tx_status)
            {
              nrf_uarte_event_clear(p_uart, NRF_UARTE_EVENT_RXTO);

              nrf_uarte_task_trigger(p_uart, NRF_UARTE_TASK_STOPRX);
              p_dev->rx_status = NRF52_XFER_STATUS_INIT;

              WAIT_FOR_UART_EVENT(p_uart, NRF_UARTE_EVENT_RXTO);
              nrf_uarte_event_clear(p_uart, NRF_UARTE_EVENT_RXTO);
              nrf_uarte_event_clear(p_uart, NRF_UARTE_EVENT_ENDRX);

              nrf52_uart_recv_done(p_dev, p_uart);

            }

        }
    }
  else
    {
      p_dev->rx_intmask = NRF_UARTE_INT_RXDRDY_MASK;
      if (enable)
        {
          nrf_uarte_int_enable(p_uart, p_dev->rx_intmask);
          nrf_uarte_task_trigger(p_uart, NRF_UARTE_TASK_STARTRX);
        }
      else
        {
          nrf_uarte_task_trigger(p_uart, NRF_UARTE_TASK_STOPRX);
          WAIT_FOR_UART_EVENT(p_uart, NRF_UARTE_EVENT_RXTO);
          nrf_uarte_event_clear(p_uart, NRF_UARTE_EVENT_RXTO);
        }
    }
  leave_critical_section(flags);
#endif
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts it also initiates a
 *    transmission sequence. This function has special case handled with
 *    the tx_enable_firstime flag. In the nordic nrf52 there is
 *    no way to find if the TX is ready for the first time wihtout writing
 *    data. Writing data generates the TXDRDY signal . This is resolved with
 *    the help of the flag "tx_enable_firstime"
 *
 ****************************************************************************/

static int nrf52_serial_thread(int argc, char **args)
{
  struct nrf52_uart_s *p_dev = NULL;

  for (int i = 0; i < UART_ENABLED_COUNT ; i++)
    {
      p_dev = (struct nrf52_uart_s *)g_uart_devs[i];
      if (false == p_dev->config->easydma)
        {
          break;
        }
    }

  while (p_dev)
    {
      sem_wait(&p_dev->sem);

      do
        {
          if (p_dev->legacy_status)
            {
              uart_xmitchars((FAR uart_dev_t *)p_dev);
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


static void up_txint(struct uart_dev_s *dev, bool enable)
{
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
  struct nrf52_uart_s *p_dev = (struct nrf52_uart_s *)dev;
  irqstate_t flags;
  NRF_UARTE_Type *p_uart;
  p_uart = (NRF_UARTE_Type *)p_dev->config->uartbase;

  if (false == p_dev->config->easydma)
    {
      /* NOT easyDMA , will send data under  non-interrupt mode
       * because interrupt will introduce latency for every one character
       */

      if (0 == p_dev->thread)
        {
          sem_init(&p_dev->sem, 0, 0);
#ifdef CONFIG_BUILD_PROTECTED
          p_dev->thread = kthread_create("Serial", 128, 1024, nrf52_serial_thread, NULL);
#else
          pthread_create(&p_dev->thread, NULL, (pthread_startroutine_t)nrf52_serial_thread, NULL);
#endif
        }

      flags = enter_critical_section();
      p_dev->legacy_status = enable;
      leave_critical_section(flags);

      sem_post(&p_dev->sem);
      return;
    }

  flags = enter_critical_section();

  if (enable)
    {
      /* Make sure the UART's tx int bit is enabled so the event will fire. */
      if (!nrf_uarte_int_enable_check(p_uart, p_dev->tx_intmask))
        {
          nrf_uarte_int_enable(p_uart, (p_dev->tx_intmask | NRF_UARTE_INT_ERROR_MASK));
        }

      if (NRF52_XFER_STATUS_INIT == p_dev->tx_status)
        {
          tx_config_t *tx ;

          /*put all into easyDMA buffer */
          uart_xmitchars(dev);

          tx = nrf52_uart_find_txfull(p_dev);
          if (tx)
            {
              nrf52_uart_setup_tx( p_dev, tx);
            }
          else
            {
              /* anyway start TX */
              tx = (tx_config_t *)sq_peek(&p_dev->tx_queue_empty);
              if (tx && tx->index)
                {
                  sq_remfirst(&p_dev->tx_queue_empty);
                  sq_addlast((FAR sq_entry_t *)tx, &p_dev->tx_queue_full);

                  tx->status = NRF52_XFER_STATUS_TRANSFERRING;
                  nrf52_uart_setup_tx( p_dev, tx);
                }
            }
        }
    }
  else
    {
#if 0
      if (NRF52_XFER_STATUS_TRANSFERRING == p_dev->tx_status)
        {
          nrf_uarte_task_trigger(p_uart, NRF_UARTE_TASK_STOPTX);

          p_dev->tx_status = NRF52_XFER_STATUS_INIT;

          WAIT_FOR_UART_EVENT(p_uart, NRF_UARTE_EVENT_TXSTOPPED);
          nrf_uarte_event_clear(p_uart, NRF_UARTE_EVENT_TXSTOPPED);
          nrf_uarte_event_clear(p_uart, NRF_UARTE_EVENT_ENDTX);
          nrf52_uart_xmit_done(p_dev, p_uart);
        }
#endif
    }
  leave_critical_section(flags);
#endif
}

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int up_setup(FAR struct uart_dev_s *dev)
{
  struct nrf52_uart_s *priv = (struct nrf52_uart_s *)dev;
  /* uninitialize the UART */

  /* Initialize the UART again */
  nrf52_uart_apply_config(priv->config);

  up_enableuart(priv);

  return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void up_shutdown(FAR struct uart_dev_s *p_dev)
{

  /* Reset hardware and disable Rx and Tx */
  up_disableuart((struct nrf52_uart_s *)p_dev);
}

/****************************************************************************
 * Name: up_attach
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

static int up_attach(FAR struct uart_dev_s *dev)
{
  int ret = OK;
  struct nrf52_uart_s *priv = (struct nrf52_uart_s *)dev;

  /* Attach and enable the IRQ(s). */

  ret = irq_attach(priv->irq, nrf52_uart_irqhandler, priv);

  if (ret == OK)
    {
      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct nrf52_uart_s *priv = (struct nrf52_uart_s *)dev;

  /* Disable interrupts */

//  up_disableuart(priv);
  up_disable_irq(priv->irq);

  /* Detach from the interrupt(s) */

  irq_detach(priv->irq);
}


/****************************************************************************
 * Name: up_rxflowcontrol
 *
 * Description:
 *   Called when Rx buffer is full (or exceeds configured watermark levels
 *   if CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS is defined).
 *   Return true if UART activated RX flow control to block more incoming
 *   data
 *
 * Input parameters:
 *   dev       - UART device instance
 *   nbuffered - the number of characters currently buffered
 *               (if CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS is
 *               not defined the value will be 0 for an empty buffer or the
 *               defined buffer size for a full buffer)
 *   upper     - true indicates the upper watermark was crossed where
 *               false indicates the lower watermark has been crossed
 *
 * Returned Value:
 *   true if RX flow control activated.
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool up_rxflowcontrol(struct uart_dev_s *dev,
                             unsigned int nbuffered, bool upper)
{
  struct nrf52_uart_s *priv = (struct nrf52_uart_s *)dev->priv;

  if (priv->iflow)
    {
      /* Is the RX buffer full? */

      if (upper)
        {
          /* Disable Rx interrupt to prevent more data being from
           * peripheral.  When hardware RTS is enabled, this will
           * prevent more data from coming in.
           *
           * This function is only called when UART recv buffer is full,
           * that is: "dev->recv.head + 1 == dev->recv.tail".
           *
           * Logic in "uart_read" will automatically toggle Rx interrupts
           * when buffer is read empty and thus we do not have to re-
           * enable Rx interrupts.
           */

          up_rxint(dev, false);
          return true;
        }

      /* No.. The RX buffer is empty */

      else
        {
          /* We might leave Rx interrupt disabled if full recv buffer was
           * read empty.  Enable Rx interrupt to make sure that more input is
           * received.
           */
          up_rxint(dev, true);
        }
    }

  return false;
}
#endif

/****************************************************************************
 * Name: up_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be avaiable during bootup.  This must be called
 *   before up_serialinit() .
 ****************************************************************************/
#ifdef USE_EARLYSERIALINIT
void up_earlyserialinit(void)
{
#if CONSOLE_UART > 0
  struct nrf52_uart_s *priv = (struct nrf52_uart_s *)g_uart_devs[CONSOLE_UART - 1];

  /* anyway: disable uart hw firstly , let up_setup do pin config again */

  nrf_uarte_disable((NRF_UARTE_Type *)priv->config->uartbase);

  up_setup(&priv->dev);

#endif
  return;
}
#endif /* USE_EARLYSERIALINIT */

/****************************************************************************
 * Name: up_serialinit(void)
 *
 * Description:
 *   Register serial console and other serial ports.  This assumes that
 *   up_earlyserialinit() was called previously.
 *
 ****************************************************************************/

static inline void nrf52_uart_easydma_buf_setup(struct nrf52_uart_s *uart_dev,
                                                uint8_t *txbuf, uint8_t txlen,
                                                uint8_t *rxbuf, uint8_t rxlen)
{

  uart_dev->tx_xfer[0].buf = txbuf;
  uart_dev->tx_xfer[0].index = 0;
  uart_dev->tx_xfer[0].len = txlen / 2;
  uart_dev->tx_xfer[0].status = NRF52_XFER_STATUS_INIT;

  sq_addlast((FAR sq_entry_t *)&uart_dev->tx_xfer[0], &uart_dev->tx_queue_empty);
  sq_addlast((FAR sq_entry_t *)&uart_dev->tx_xfer[1], &uart_dev->tx_queue_empty);

  uart_dev->tx_xfer[1].buf = txbuf + txlen / 2;
  uart_dev->tx_xfer[1].index = 0;
  uart_dev->tx_xfer[1].len = txlen / 2;
  uart_dev->tx_xfer[1].status = NRF52_XFER_STATUS_INIT;

  uart_dev->rx_xfer.buf = rxbuf;
  uart_dev->rx_xfer.index = 0;
  uart_dev->rx_xfer.len = rxlen;
  uart_dev->rx_xfer.bytes = 0;
  uart_dev->rx_xfer.status = NRF52_XFER_STATUS_INIT;

}

void up_serialinit(void)
{
  char devname[16];
  unsigned i;
  unsigned minor = 0;

  struct nrf52_uart_s *p_dev;

#ifdef CONFIG_SEGGER_RTT_CONSOLE
  extern  void segger_rtt_register_console(void);
  segger_rtt_register_console();
#endif

  /* Register the console */
  strcpy(devname, "/dev/ttySx");
  devname[9] = '0' + minor++;
#if CONSOLE_UART > 0
  p_dev = (struct nrf52_uart_s *)g_uart_devs[CONSOLE_UART - 1];
  (void)uart_register("/dev/console", (uart_dev_t *)p_dev);
  up_putc('L');

#ifndef CONFIG_SERIAL_DISABLE_REORDERING
  /* If not disabled, register the console UART to ttyS0 and exclude
   * it from initializing it further down
   */

  (void)uart_register(devname, (uart_dev_t *)g_uart_devs[CONSOLE_UART - 1]);
#endif
#endif /* CONSOLE_UART > 0 */

  /* Register all remaining USARTs */
  for (i = 0; i < UART_ENABLED_COUNT ; i++)
    {

      p_dev = (struct nrf52_uart_s *)g_uart_devs[i];
      /* Don't create a device for non-configured ports. */

      if (p_dev == 0)
        {
          continue;
        }

      if (p_dev->thread_rdy == 0 && p_dev->config->easydma)
        {
          nrf52_uart_easydma_buf_setup(p_dev, p_dev->config->tx_buf, p_dev->config->tx_len,
                                       p_dev->config->rx_buf, p_dev->config->rx_len);

          p_dev->thread_rdy = 1;
        }

#ifndef CONFIG_SERIAL_DISABLE_REORDERING
      /* Don't create a device for the console - we did that above */

      if (p_dev->dev.isconsole)
        {
          continue;
        }
#endif

      /* Register USARTs as devices in increasing order */

      devname[9] = '0' + minor++;
#if CONFIG_NFILE_DESCRIPTORS > 0
      (void)uart_register(devname, &p_dev->dev);
#endif
    }
}

/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if defined(CONFIG_SERIAL_TERMIOS) || defined(CONFIG_SERIAL_TIOCSERGSTRUCT)
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
  struct nrf52_uart_s   *priv  = (struct nrf52_uart_s *)dev;
#endif
  int                ret    = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
      case TIOCSERGSTRUCT:
        {
          struct nrf52_uart_s *user = (struct nrf52_uart_s *)arg;
          if (!user)
            {
              ret = -EINVAL;
            }
          else
            {
              memcpy(user, dev, sizeof(struct nrf52_uart_s));
            }
        }
        break;
#endif

      case TIOCSBRK:   /* BSD compatibility: Turn break on, unconditionally */
      case TIOCCBRK:   /* BSD compatibility: Turn break off, unconditionally */
        ret = -ENOTTY; /* Not supported */
        break;

#ifdef CONFIG_SERIAL_TERMIOS
      case TCGETS:
        {
          struct termios *termiosp = (struct termios *)arg;

          if (!termiosp)
            {
              ret = -EINVAL;
              break;
            }

          /* TODO:  Other termios fields are not yet returned.
           * Note that cfsetospeed is not necessary because we have
           * knowledge that only one speed is supported.
           * Both cfset(i|o)speed() translate to cfsetspeed.
           */

          cfsetispeed(termiosp, priv->config->baud_rate);
        }
        break;

      case TCSETS:
        {
          struct termios *termiosp = (struct termios *)arg;
          /* uint32_t           lcr;  * Holds current values of line control register */
          /* uint16_t           dl;   * Divisor latch */

          if (!termiosp)
            {
              ret = -EINVAL;
              break;
            }

          /* TODO:  Handle other termios settings.
           * Note that only cfgetispeed is used because we have knowledge
           * that only one speed is supported.
           */

          /* Get the c_speed field in the termios struct */

          priv->config->baud_rate = cfgetispeed(termiosp);

          /* Reset the baud */

          nrf52_uart_set_baudrate(priv->config);
        }
        break;

#endif /* CONFIG_SERIAL_TERMIOS */

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#if CONSOLE_UART > 0
  struct nrf52_uart_s *p_dev = (struct nrf52_uart_s *)g_uart_devs[CONSOLE_UART - 1];

  /* Check for LF */

  if ('\n' == ch)
    {
      int return_ch = '\r';
      if (p_dev->thread_rdy)
        {
          up_send((uart_dev_t *)g_uart_devs[CONSOLE_UART - 1], return_ch);
        }
      else
        {
          up_lowputc((char) return_ch);
        }
    }

  if (p_dev->thread_rdy)
    {
      up_send((uart_dev_t *)g_uart_devs[CONSOLE_UART - 1], ch);
    }
  else
    {
      up_lowputc((char) ch);
    }
#else
  up_lowputc((char) ch);
#endif
  return ch;
}

#else
/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{

  /* Check for LF */

  if ('\n' == ch)
    {
      up_lowputc((char) '\r');
    }

  up_lowputc((char) ch);

  return ch;
}

#endif

/****************************************************************************
 * Name: nrf_apply_config
 *
 * Description:
 *   Applies HW pin configurations ,baud rate settings and flow control
 *    settings
 *
 ****************************************************************************/

static uint32_t nrf52_uart_set_baudrate(uart_config_t *p_config)
{
  int i;

  for (i = 0; i < sizeof(nrf_baudrate) / sizeof(nrf_baudrate_config_t) ; i++)
    {
      if (p_config->baud_rate == nrf_baudrate[i].baudrate)
        {
          break;
        }
    }

  ASSERT(i < sizeof(nrf_baudrate) / sizeof(nrf_baudrate_config_t));

  nrf_uarte_baudrate_set((NRF_UARTE_Type *)p_config->uartbase, nrf_baudrate[i].config);

  return OK;
}

int nrf52_uart_apply_config(uart_config_t *p_config)
{
  NRF_UARTE_Type *p_uart = (NRF_UARTE_Type *)p_config->uartbase;
  nrf_uarte_hwfc_t hwfc;

  ASSERT(p_config->txd_pin != NRF52_INVALID_GPIO_PIN);
  ASSERT(p_config->rxd_pin != NRF52_INVALID_GPIO_PIN);

  /* intitialize tx and rx pins */
  nrf_gpio_pin_set(p_config->txd_pin);
  nrf_gpio_cfg_output(p_config->txd_pin);

  nrf_gpio_cfg_input(p_config->rxd_pin, NRF_GPIO_PIN_NOPULL);

  /* configure HW flow control */
  if ((p_config->cts_pin != NRF52_INVALID_GPIO_PIN) && \
      (p_config->rts_pin != NRF52_INVALID_GPIO_PIN))
    {
      nrf_gpio_cfg_input(p_config->cts_pin, NRF_GPIO_PIN_PULLDOWN);
      nrf_gpio_cfg_output(p_config->rts_pin);
      nrf_gpio_pin_set(p_config->rts_pin);
      nrf_uarte_hwfc_pins_set(p_uart, p_config->rts_pin, p_config->cts_pin);

      hwfc = NRF_UARTE_HWFC_ENABLED;
    }
  else
    {
      hwfc = NRF_UARTE_HWFC_DISABLED;
    }

  /* set the UART baud rate */
  nrf52_uart_set_baudrate(p_config);

  nrf_uarte_txrx_pins_set(p_uart, p_config->txd_pin, p_config->rxd_pin);

  nrf_uarte_configure(p_uart, p_config->parity, p_config->stopbits2, hwfc);

  return OK;
}

void nrf52_uart_legacy_send(uint32_t reg_base, uint8_t ch)
{
  NRF_UARTE_Type *uart = (NRF_UARTE_Type *)reg_base;

  nrf_uarte_event_clear(uart, NRF_UARTE_EVENT_TXDRDY);
  ((NRF_UART_Type *)uart)->TXD =  ch;
  nrf_uarte_task_trigger(uart, NRF_UARTE_TASK_STARTTX);
  while (!nrf_uarte_event_check(uart, NRF_UARTE_EVENT_TXDRDY));
}

