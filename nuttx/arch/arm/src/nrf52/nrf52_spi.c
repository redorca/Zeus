/****************************************************************************
 * arch/arm/src/nrf52/nrf52_spi.c
 *
 *   Copyright (C) 2009-2013, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Copyright (C) 2017 Zglue  Inc. All rights reserved.
 *           Arjun Hary   <arjun@zglue.com>
 *           Levin Li     <zhiqiang@zglue.com>
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
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "nrf.h"
#include "nrf_spim.h"
#include "nrf_spi.h"

#include "nrf52_gpio.h"
#include "nrf52_spi.h"


#if defined(CONFIG_NRF52_SPI0) || defined(CONFIG_NRF52_SPI1) || defined(CONFIG_NRF52_SPI2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define NRF_DRV_SPI_FLAG_TX_POSTINC          (1UL << 0) /**< TX buffer address incremented after transfer. */
#define NRF_DRV_SPI_FLAG_RX_POSTINC          (1UL << 1) /**< RX buffer address incremented after transfer. */
#define NRF_DRV_SPI_FLAG_NO_XFER_EVT_HANDLER (1UL << 2) /**< Interrupt after each transfer is suppressed, and the event handler is not called. */
#define NRF_DRV_SPI_FLAG_HOLD_XFER           (1UL << 3) /**< Set up the transfer but do not start it. */
#define NRF_DRV_SPI_FLAG_REPEATED_XFER       (1UL << 4) /**< Flag indicating that the transfer will be executed multiple times. */
#define NRF_DRV_SPI_PIN_NOT_USED  0xFF

#ifndef NRF_SPI_PIN_NOT_CONNECTED
#define NRF_SPI_PIN_NOT_CONNECTED  0xFFFFFFFF
#endif

#define NRF52_SPI0_CLK_FREQUENCY  NRF_SPIM_FREQ_8M

#define NRF52_SPI1_CLK_FREQUENCY  NRF_SPIM_FREQ_8M

#define NRF52_SPI2_CLK_FREQUENCY  NRF_SPIM_FREQ_8M

// All interrupt flags
#define DISABLE_ALL_INT_SHORT  0xFFFFFFFF

#define NRF52_SPI_MAX_TRANSFER_LEN  0xFF
#define NRF52_SPI_ONCE_TRANSFER_LEN 0xF0 /* 240 for one time */

/* Note:
 *    Nordic tx / rx buffer accept the max transfer data is 255 byte.
 *    You can check datasheet by 31.6.14 TXD.MAXCNT Address offset: 0x548
 *    they support double buffer feature , but now this driver not use double buffer
 *    feature
 *    another feature is DMA list ,  now we don't use it
 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/**
 * @brief Single transfer descriptor structure.
 */
typedef struct
{
  uint8_t const *p_tx_buffer;  ///< Pointer to TX buffer.
  uint8_t        *p_rx_buffer; ///< Pointer to RX buffer.
  uint8_t         tx_length;   ///< TX buffer length.
  uint8_t         rx_length;   ///< RX buffer length.
} nrf_drv_spi_xfer_desc_t;

struct nrf52_spidev_s
{
  struct spi_dev_s          spidev;     /* Generic SPI device */
  unsigned int              base;       /* Base address of registers */
  uint16_t                  irqid;      /* IRQ for this device */
  uint8_t                   busid;
  int8_t                    refs;       /* the reference count by devices */
  sem_t                     mutex;      /* Only one thread can access at a time */
#ifdef CONFIG_NRF52_SPI_INTERRUPTS
  int (*isr)(int, void *, void *);      /* Interrupt handler */
  sem_t                     wait;       /* Interrupt wait semaphore */
#endif
  WDOG_ID                   timeout;    /* Watchdog to timeout when bus hung */
  nrf_spim_frequency_t      frequency;  /* Current SPI frequency */
  uint32_t                  mode;       /* Current SPI frequency */
  nrf_spim_bit_order_t      bit_order;  /* Current SPI frequency */
  uint8_t                   orc;        /* < Over-run character.
                                         **< This character is used when all bytes from the TX buffer are sent,
                                         *but the transfer continues due to RX. */

  volatile uint32_t         int_mask;
  nrf_drv_spi_xfer_desc_t   xfer_desc;
  nrfx_drv_state_t           state;
  uint32_t                  flags;
  volatile bool             busy;

  volatile bool             transfer_in_progress;

  struct spi_pinmux_t       *pinmux;
  bool                      easydma;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

ret_code_t nrf52_spi_init(struct nrf52_spidev_s *dev, bool reset_cs_pin_flag);

static uint32_t nrf52_spi_setfrequency(FAR struct spi_dev_s *dev,
                                       uint32_t frequency);
void nrf52_spi_select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected);
/* SPI device operations */
static int nrf52_spi_lock(FAR struct spi_dev_s *dev, bool lock);
static void nrf52_spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
static uint16_t nrf52_spi_send(FAR struct spi_dev_s *dev, uint16_t wd);

#ifdef CONFIG_SPI_EXCHANGE
static void nrf52_spi_exchange(FAR struct spi_dev_s *dev,
                               FAR const void *txbuffer, FAR void *rxbuffer,
                               size_t nwords);
#else
static void nrf52_spi_sndblock(FAR struct spi_dev_s *dev,
                               FAR const void *txbuffer, size_t nwords);
static void nrf52_spi_recvblock(FAR struct spi_dev_s *dev,
                                FAR void *buffer, size_t nwords);
#endif

#ifdef CONFIG_SPI_RESET
static int  nrf52_spi_reset(FAR struct spi_dev_s *dev);
#endif

#ifdef CONFIG_NRF52_SPI_INTERRUPTS
static int  nrf52_spi_interrupt_handler(int irq, void *context, FAR void *arg);
static void spim_int_enable(NRF_SPIM_Type *p_spim, bool enable);

#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/
const struct spi_ops_s nrf52_spi_ops =
{
  .lock              = nrf52_spi_lock,
  .select            = nrf52_spi_select,
  .setfrequency      = nrf52_spi_setfrequency,
  .setmode           = nrf52_spi_setmode,
  .setbits           = NULL,
  .status            = NULL,
  .send              = nrf52_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = nrf52_spi_exchange,
#else
  .sndblock          = nrf52_spi_sndblock,
  .recvblock         = nrf52_spi_recvblock,
#endif
#ifdef CONFIG_SPI_RESET
  .reset             = nrf52_spi_reset,
#endif
  .registercallback  = NULL,
};

#ifdef CONFIG_NRF52_SPI0
static const struct spi_pinmux_t spi0_pinmux =
{
  .miso_pin   = BOARD_SPI0_MISO_PIN,
  .mosi_pin   = BOARD_SPI0_MOSI_PIN,
  .sck_pin    = BOARD_SPI0_SCL_PIN,
};

static struct nrf52_spidev_s g_spi0dev =
{
  .spidev     = { &nrf52_spi_ops },
  .base       = NRF_SPIM0_BASE,
  .irqid      = SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn,
  .busid      = SPI_NRF52_BUS_0,
  .frequency  = NRF52_SPI0_CLK_FREQUENCY,
  .mode       = NRF_SPIM_MODE_0,
  .orc        = 0xA5,
  .bit_order  = NRF_SPIM_BIT_ORDER_MSB_FIRST,

#ifdef CONFIG_NRF52_SPI_INTERRUPTS
  .isr        = nrf52_spi_interrupt_handler,
#endif
  .pinmux     = (struct spi_pinmux_t *) &spi0_pinmux,
  .easydma    = true
};
#endif

#ifdef CONFIG_NRF52_SPI1

static const struct spi_pinmux_t spi1_pinmux =
{
  .miso_pin   = BOARD_SPI1_MISO_PIN,
  .mosi_pin   = BOARD_SPI1_MOSI_PIN,
  .sck_pin    = BOARD_SPI1_SCL_PIN,
};

static struct nrf52_spidev_s g_spi1dev =
{
  .spidev     = { &nrf52_spi_ops },
  .base       = NRF_SPIM1_BASE,
  .irqid      = SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn,
  .busid      = SPI_NRF52_BUS_1,
  .frequency  = NRF52_SPI1_CLK_FREQUENCY,
  .mode       = NRF_SPIM_MODE_0,
  .orc        = 0xA5,
  .bit_order  = NRF_SPIM_BIT_ORDER_MSB_FIRST,

#ifdef CONFIG_NRF52_SPI_INTERRUPTS
  .isr        = nrf52_spi_interrupt_handler,
#endif
  .pinmux     = (struct spi_pinmux_t *) &spi1_pinmux,
  .easydma    = true
};
#endif

#ifdef CONFIG_NRF52_SPI2

static const struct spi_pinmux_t spi2_pinmux =
{
  .miso_pin   = BOARD_SPI2_MISO_PIN,
  .mosi_pin   = BOARD_SPI2_MOSI_PIN,
  .sck_pin    = BOARD_SPI2_SCL_PIN,
};

static struct nrf52_spidev_s g_spi2dev =
{
  .spidev     = { &nrf52_spi_ops },
  .base       = NRF_SPIM2_BASE,
  .irqid      = SPIM2_SPIS2_SPI2_IRQn,
  .busid      = SPI_NRF52_BUS_2,
  .frequency  = NRF52_SPI2_CLK_FREQUENCY,
  .mode       = NRF_SPIM_MODE_0,
  .orc        = 0xA5,
  .bit_order  = NRF_SPIM_BIT_ORDER_MSB_FIRST,

#ifdef CONFIG_NRF52_SPI_INTERRUPTS
  .isr        = nrf52_spi_interrupt_handler,
#endif
  .pinmux     = (struct spi_pinmux_t *) &spi2_pinmux,
  .easydma    = true
};
#endif

static SPI_CS_CallBack g_cs_select;

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/****************************************************************************
 * Name: nrf52_spi_setfrequency
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 ****************************************************************************/
static uint32_t nrf52_spi_setfrequency(FAR struct spi_dev_s *dev,
                                       uint32_t frequency)
{
  struct nrf52_spidev_s *priv = (struct nrf52_spidev_s *) dev;
  NRF_SPIM_Type *p_spim = (NRF_SPIM_Type *)priv->base;
  uint32_t freq_k ;

  freq_k = frequency / 1000;

  switch (freq_k)
    {
      case 125:
        priv->frequency = NRF_SPIM_FREQ_125K;
        break;
      case 250:
        priv->frequency = NRF_SPIM_FREQ_250K;
        break;
      case 500:
        priv->frequency = NRF_SPIM_FREQ_500K;
        break;
      case 1000:
        priv->frequency = NRF_SPIM_FREQ_1M;
        break;
      case 2000:
        priv->frequency = NRF_SPIM_FREQ_2M;
        break;
      case 4000:
        priv->frequency = NRF_SPIM_FREQ_4M;
        break;
      case 8000:
        priv->frequency = NRF_SPIM_FREQ_8M;
        break;
      default:
        spierr("Invalid Parameter for SPI frequency , %d\n", frequency);
        return -EINVAL;
        break;
    }
  nrf_spim_frequency_set(p_spim, (nrf_spim_frequency_t)priv->frequency);

  return frequency;
}

/****************************************************************************
 * Name: nrf52_spi_select
 *
 * Description:
 *   Set the chip select signal
 *
 ****************************************************************************/
void nrf52_spi_select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  struct nrf52_spidev_s *priv = (struct nrf52_spidev_s *) dev;

  if (NULL == g_cs_select)
    {
      spierr("Please register customized  CS select API.\n");
      return;
    }

  g_cs_select(priv->busid, devid, selected);

}

/****************************************************************************
 * Name: nrf52_spi_setmode
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 ****************************************************************************/
static void nrf52_spi_setmode(FAR struct spi_dev_s *dev,
                              enum spi_mode_e mode)
{
  struct nrf52_spidev_s *priv = (struct nrf52_spidev_s *) dev;
  NRF_SPIM_Type *p_spim = (NRF_SPIM_Type *)priv->base;
  uint32_t config = p_spim->CONFIG;
  switch (mode)
    {
      case SPIDEV_MODE0:
        config |= (SPIM_CONFIG_CPOL_ActiveHigh << SPIM_CONFIG_CPOL_Pos) |
                  (SPIM_CONFIG_CPHA_Leading    << SPIM_CONFIG_CPHA_Pos);
        break;
      case SPIDEV_MODE1:
        config |= (SPIM_CONFIG_CPOL_ActiveHigh << SPIM_CONFIG_CPOL_Pos) |
                  (SPIM_CONFIG_CPHA_Trailing   << SPIM_CONFIG_CPHA_Pos);
        break;
      case SPIDEV_MODE2:
        config |= (SPIM_CONFIG_CPOL_ActiveLow  << SPIM_CONFIG_CPOL_Pos) |
                  (SPIM_CONFIG_CPHA_Leading    << SPIM_CONFIG_CPHA_Pos);
        break;
      case SPIDEV_MODE3:
      default:
        config |= (SPIM_CONFIG_CPOL_ActiveLow  << SPIM_CONFIG_CPOL_Pos) |
                  (SPIM_CONFIG_CPHA_Trailing   << SPIM_CONFIG_CPHA_Pos);
        break;
    }
  p_spim->CONFIG = config;
}

/************************************************************************************
 * Name: spi_lock
 *
 * Description:
 *   On SPI busses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the busses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI buss is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static int nrf52_spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct nrf52_spidev_s *priv = (FAR struct nrf52_spidev_s *)dev;

  if (lock == true)
    {
      /* Take the semaphore (perhaps waiting) */
      while (sem_wait(&priv->mutex) != 0)
        {
          /* The only case that an error should occur here is if the wait was awakened
           * by a signal.
           */

          ASSERT(errno == EINTR);
        }
    }
  else
    {
      (void)sem_post(&priv->mutex);
    }
  return OK;
}


/****************************************************************************
 * Name: spim_list_enable_handle
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 ****************************************************************************/
static void spim_list_enable_handle(NRF_SPIM_Type *p_spim, uint32_t flags)
{
  if (NRF_DRV_SPI_FLAG_TX_POSTINC & flags)
    {
      nrf_spim_tx_list_enable(p_spim);
    }
  else
    {
      nrf_spim_tx_list_disable(p_spim);
    }

  if (NRF_DRV_SPI_FLAG_RX_POSTINC & flags)
    {
      nrf_spim_rx_list_enable(p_spim);
    }
  else
    {
      nrf_spim_rx_list_disable(p_spim);
    }
}

/************************************************************************************
 * Name: nrf_spim_xfer
 *
 * Description:
 *   Take the exclusive access, waiting as necessary
 *
 ************************************************************************************/
static ret_code_t nrf_spim_xfer(FAR struct nrf52_spidev_s *priv,
                                NRF_SPIM_Type                  *p_spim,
                                nrf_drv_spi_xfer_desc_t const *p_xfer_desc,
                                uint32_t                        flags)
{
  ret_code_t err_code;

  /* EasyDMA requires that transfer buffers are placed in Data RAM region;
   * signal error if they are not. */
  if ((p_xfer_desc->p_tx_buffer && !nrf_is_in_ram(p_xfer_desc->p_tx_buffer)) ||
      (p_xfer_desc->p_rx_buffer && !nrf_is_in_ram(p_xfer_desc->p_rx_buffer)))
    {
      err_code = EINVAL;
      return err_code;
    }

  nrf_spim_event_clear(p_spim, NRF_SPIM_EVENT_END);

  nrf_spim_tx_buffer_set(p_spim, p_xfer_desc->p_tx_buffer, p_xfer_desc->tx_length);
  nrf_spim_rx_buffer_set(p_spim, p_xfer_desc->p_rx_buffer, p_xfer_desc->rx_length);

  spim_list_enable_handle(p_spim, flags);

  nrf_spim_task_trigger(p_spim, NRF_SPIM_TASK_START);

#ifdef CONFIG_NRF52_SPI_INTERRUPTS

  /* enable interrupt and start task */

  spim_int_enable(p_spim, true);

  /* if there is interrupt , just wait the semephore
   * potential issue: if interrupt missing , no post action
   */

  sem_wait(&priv->wait);

#else

  while (!nrf_spim_event_check(p_spim, NRF_SPIM_EVENT_END))
    {
      spiinfo("Sleep to let other task to run\n");
    }
#endif

  err_code = OK;
  return err_code;
}


/************************************************************************************
 * Name: nrf_legacy_8_bit_transfer
 *
 * Description:
 *   Transfers (read + write) 1 byte using the legacy driver
 *
 ************************************************************************************/
ret_code_t nrf_legacy_8_bit_transfer(FAR struct nrf52_spidev_s *priv,
                                     nrf_drv_spi_xfer_desc_t const *p_xfer_desc)
{

  nrf_spi_event_clear((NRF_SPI_Type *)priv->base, NRF_SPI_EVENT_READY);

  /* start spi transaction */
  nrf_spi_txd_set((NRF_SPI_Type *)priv->base, (p_xfer_desc->tx_length > 0 ?  p_xfer_desc->p_tx_buffer[0] : priv->orc));

  /* wait for transaction to complete */
  while (!nrf_spi_event_check((NRF_SPI_Type *)priv->base, NRF_SPI_EVENT_READY));

  /* get read data */
  p_xfer_desc->p_rx_buffer[0] = nrf_spi_rxd_get((NRF_SPI_Type *)priv->base);

  return OK;

}

/************************************************************************************
 * Name: nrf_drv_spi_transfer
 *
 * Description:
 *   Take the exclusive access, waiting as necessary
 *
 ************************************************************************************/
static ret_code_t nrf_drv_spi_transfer(FAR struct nrf52_spidev_s *priv,
                                       uint8_t const *p_tx_buffer,
                                       uint8_t         tx_buffer_length,
                                       uint8_t        *p_rx_buffer,
                                       uint8_t         rx_buffer_length)
{
  ASSERT(priv->state != NRF_DRV_STATE_UNINITIALIZED);

  priv->xfer_desc.p_tx_buffer = p_tx_buffer;
  priv->xfer_desc.p_rx_buffer = p_rx_buffer;
  priv->xfer_desc.tx_length   = tx_buffer_length;
  priv->xfer_desc.rx_length   = rx_buffer_length;

  if (rx_buffer_length == 1)
    {
      if (priv->easydma)
        {
          priv->easydma = false;
          /* reconfigure from easy_dma -> legacy */
          nrf_spi_disable((NRF_SPI_Type *)priv->base);

          nrf_spi_pins_set((NRF_SPI_Type *)priv->base,
                           priv->pinmux->sck_pin,
                           priv->pinmux->mosi_pin,
                           priv->pinmux->miso_pin);
          nrf_spi_frequency_set((NRF_SPI_Type *)priv->base, priv->frequency);
          nrf_spi_configure((NRF_SPI_Type *)priv->base, priv->mode, priv->bit_order);
          nrf_spi_enable((NRF_SPI_Type *)priv->base);

          /* clear flags */
          nrf_spi_int_disable((NRF_SPI_Type *)priv->base, NRF_SPI_INT_READY_MASK);
          nrf_spi_event_clear((NRF_SPI_Type *)priv->base, NRF_SPI_EVENT_READY);
        }

      return nrf_legacy_8_bit_transfer(priv, &priv->xfer_desc);
    }
  else
    {
      if (priv->easydma == false)
        {
          priv->easydma = true;
          /* reconfigure from legacy -> easy_dma */
          nrf_spi_disable((NRF_SPI_Type *)priv->base);
          priv->state = NRF_DRV_STATE_UNINITIALIZED;

          nrf52_spi_init(priv, false);
        }

      return nrf_spim_xfer(priv, (NRF_SPIM_Type *)priv->base, &priv->xfer_desc, 0);
    }
}

#ifdef CONFIG_NRF52_SPI_INTERRUPTS
/****************************************************************************
 * Name: spim_int_enable
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 ****************************************************************************/
static void spim_int_enable(NRF_SPIM_Type *p_spim, bool enable)
{
  if (!enable)
    {
      nrf_spim_int_disable(p_spim, NRF_SPIM_INT_END_MASK);
    }
  else
    {
      nrf_spim_int_enable(p_spim, NRF_SPIM_INT_END_MASK);
    }
}

/****************************************************************************
 * Name: nrf52_spi_interrupt
 *
 * Description:
 *   The SPI Interrupt Handler
 *
 ****************************************************************************/
static int nrf52_spi_interrupt_handler(int irq, void *context, FAR void *arg)
{
  struct nrf52_spidev_s *dev = (struct nrf52_spidev_s *)arg;

  NRF_SPIM_Type *p_spim = (NRF_SPIM_Type *)dev->base;

  if (nrf_spim_event_check(p_spim, NRF_SPIM_EVENT_END))
    {
      irqinfo("SPI: END Event.\n");

      nrf_spim_event_clear(p_spim, NRF_SPIM_EVENT_END);

      /* Signal that the transfer is complete */

      sem_post(&(dev->wait));
    }

  if (nrf_spim_event_check(p_spim, NRF_SPIM_EVENT_STARTED))
    {
      irqinfo("SPI: Start Event.\n");
      nrf_spim_event_clear(p_spim, NRF_SPIM_EVENT_STARTED);
      /* not use start event for now , will be used under dual buffer mode
       * ToDo in further
       * nrf_spim_event_clear(p_spim, NRF_SPIM_EVENT_END);
       */
    }

  if (nrf_spim_event_check(p_spim, NRF_SPIM_EVENT_STOPPED))
    {
      irqinfo("SPI: STOP Event");
      nrf_spim_event_clear(p_spim, NRF_SPIM_EVENT_STOPPED);
    }

  return OK;
}

#endif

/************************************************************************************
 * Name: nrf52_spi_reset
 *
 * Description:
 *   Perform an SPI bus reset in an attempt to break loose stuck SPI devices.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ************************************************************************************/

#ifdef CONFIG_SPI_RESET
static int nrf52_spi_reset(struct spi_dev_s *dev)
{
  return OK;
}
#endif /* CONFIG_SPI_RESET */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_spi_easy_dma_mode
 *
 * Description:
 *   selects and reconfigures spi driver used
 *   - easy_dma
 *   - legacy [used to rx exactly 1 byte]
 *
 ****************************************************************************/

int32_t nrf52_spi_easy_dma_mode(FAR struct spi_dev_s *dev, bool easydma)
{

  FAR struct nrf52_spidev_s *priv = (struct nrf52_spidev_s *)dev;
  priv->easydma = easydma;

  return OK;
}

/****************************************************************************
 * Name: nrf52_spi_init
 *
 * Description:
 *   init an spi device
 *
 ****************************************************************************/

ret_code_t nrf52_spi_init(struct nrf52_spidev_s *dev, bool reset_cs_pin_flag)
{
  uint32_t mosi_pin;
  uint32_t miso_pin;
  ret_code_t err_code;
  NRF_SPIM_Type *p_spim = (NRF_SPIM_Type *)dev->base;
  struct spi_pinmux_t *p_pinmux = dev->pinmux;

  ASSERT(dev);

  if (dev->state != NRF_DRV_STATE_UNINITIALIZED)
    {
      err_code = EPERM;
      return err_code;
    }

  dev->int_mask = 0;

  /* Configure pins used by the peripheral:
   * - SCK - output with initial value corresponding with the SPI mode used:
   *   0 - for modes 0 and 1 (CPOL = 0), 1 - for modes 2 and 3 (CPOL = 1);
   *   according to the reference manual guidelines this pin and its input
   *   buffer must always be connected for the SPI to work.
   */

  if (dev->mode <= NRF_SPIM_MODE_1)
    {
      nrf_gpio_pin_clear(p_pinmux->sck_pin);
    }
  else
    {
      nrf_gpio_pin_set(p_pinmux->sck_pin);
    }

  nrf_gpio_cfg_output(p_pinmux->sck_pin);

  /* - MOSI (optional) - output with initial value 0 */

  if (p_pinmux->mosi_pin != NRF_SPI_PIN_NOT_CONNECTED)
    {
      mosi_pin = p_pinmux->mosi_pin;
      nrf_gpio_pin_clear(mosi_pin);
      nrf_gpio_cfg_output(mosi_pin);
    }
  else
    {
      mosi_pin = NRF_SPI_PIN_NOT_CONNECTED;
    }

  /* - MISO (optional) - input */

  if (p_pinmux->miso_pin != NRF_SPI_PIN_NOT_CONNECTED)
    {
      miso_pin = p_pinmux->miso_pin;
      nrf_gpio_cfg_input(miso_pin, GPIO_PIN_CNF_PULL_Pulldown);
    }
  else
    {
      miso_pin = NRF_SPI_PIN_NOT_CONNECTED;
    }

  nrf_spim_pins_set(p_spim, p_pinmux->sck_pin, mosi_pin, miso_pin);
  nrf_spim_frequency_set(p_spim, (nrf_spim_frequency_t)dev->frequency);
  nrf_spim_configure(p_spim,
                     (nrf_spim_mode_t)dev->mode,
                     (nrf_spim_bit_order_t)dev->bit_order);

  nrf_spim_orc_set(p_spim, dev->orc);

  nrf_spim_enable(p_spim);

  dev->state = NRF_DRV_STATE_INITIALIZED;

  return OK;
}

FAR int nrf52_spibus_register(SPI_CS_CallBack cs_select)
{
  g_cs_select = cs_select;
  return OK;
}

/****************************************************************************
 * Name: nrf52_spibus_initialize
 *
 * Description:
 *   Initialise an SPI device
 *
 ****************************************************************************/

struct spi_dev_s *nrf52_spibus_initialize(int port, bool reset_cs_pin_flag)
{
  struct nrf52_spidev_s *priv = NULL;
  if (port > 2)
    {
      spierr("ERROR: NRF52 SPI Only supports ports 0 and 1 & 2\n");
      return NULL;
    }

  irqstate_t flags;
  uint32_t err_code;

#ifdef CONFIG_NRF52_SPI0
  if (port == 0)
    {
      priv   = &g_spi0dev;
    }
#endif

#ifdef CONFIG_NRF52_SPI1
  if (port == 1)
    {
      priv   = &g_spi1dev ;
    }
#endif

#ifdef CONFIG_NRF52_SPI2
  if (port == 2)
    {
      priv   = &g_spi2dev ;
    }
#endif

  if (NULL == priv)
    {
      return NULL;
    }

  flags = enter_critical_section();

  if (priv->refs == 0)
    {
      err_code = nrf52_spi_init(priv, reset_cs_pin_flag);

      if (err_code != OK)
        {
          return NULL;
        }

      /* Initialize semaphores */

      sem_init(&priv->mutex, 0, 1);

#ifdef CONFIG_NRF52_SPI_INTERRUPTS
      sem_init(&priv->wait, 0, 0);

      /* The wait semaphore is used for signaling and, hence, should not have
       * priority inheritance enabled.
      */
      sem_setprotocol(&priv->wait, SEM_PRIO_NONE);
#endif

#ifdef CONFIG_NRF52_SPI_INTERRUPTS

      /* Attach Interrupt Handler */
      irq_attach(priv->irqid, priv->isr, priv);

      /* Enable Interrupt Handler */
      up_enable_irq(priv->irqid);
#endif
    }

  priv->refs++;
  leave_critical_section(flags);

  return &(priv->spidev);
}



/****************************************************************************
 * Name: nrf52_spibus_uninitialize
 *
 * Description:
 *   Uninitialise an SPI device
 *
 ****************************************************************************/

int nrf52_spibus_uninitialize(FAR struct spi_dev_s *dev)
{
  struct nrf52_spidev_s *priv = (struct nrf52_spidev_s *) dev;
  irqstate_t flags;

  ASSERT(priv->state != NRF_DRV_STATE_UNINITIALIZED);

  flags = enter_critical_section();

  priv->refs--;
  if (priv->refs <= 0)
    {
      /* Disable SPI */
      nrf_spim_disable((NRF_SPIM_Type *)priv->base);

      /* Reset data structures */

      sem_destroy(&priv->mutex);
#ifdef CONFIG_NRF52_SPI_INTERRUPTS
      sem_destroy(&priv->wait);
#endif


      /* Disable interrupts */

      up_disable_irq(priv->irqid);

      /* Detach Interrupt Handler */

      irq_detach(priv->irqid);

      priv->state = NRF_DRV_STATE_UNINITIALIZED;
    }
  leave_critical_section(flags);


  return OK;
}

/****************************************************************************
 * Name: nrf52_spi_send
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 ****************************************************************************/
static uint16_t nrf52_spi_send(FAR struct spi_dev_s *dev, uint16_t wd)
{
  FAR struct nrf52_spidev_s *priv = (struct nrf52_spidev_s *)dev;
  uint8_t response = 0;
  uint8_t cmd;

  cmd = (uint8_t)wd;

  response = 0;

  /* issue here:
   *    spi_send should return the response data , but there is bug
   *    if use nrf_drv_spi_transfer(priv, &cmd, 1, &response, 1) to get response
   *    nordic will send out two command, bus data is wrong.
   *    Should get workaround in further
   */

  nrf_drv_spi_transfer(priv, &cmd, 1, &response, 1);

  spiinfo("Cmd: %02#x. Response: %02#x\n", cmd, response);

  return (uint16_t)response;
}

#ifdef CONFIG_SPI_EXCHANGE
/************************************************************************************
 * Name: nrf52_spi_exchange
 *
 * Description:
 *   Release exclusive access
 *
 ************************************************************************************/
static void nrf52_spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer, FAR void *rxbuffer, size_t nwords)
{
  FAR struct nrf52_spidev_s *priv = (struct nrf52_spidev_s *)dev;

  spiinfo("Exchange: size %d.\n", nwords);

  uint8_t leaf_size;
  uint32_t total_cycle;
  uint32_t i = 0;
  uint8_t *p_tx = (uint8_t *)txbuffer;
  uint8_t *p_rx = (uint8_t *)rxbuffer;

  /* split send to small size for multi send */

  total_cycle = nwords / NRF52_SPI_ONCE_TRANSFER_LEN;
  leaf_size = nwords % NRF52_SPI_ONCE_TRANSFER_LEN;

  while ( i < total_cycle)
    {
      if (txbuffer && rxbuffer)
        {
          nrf_drv_spi_transfer(priv, p_tx + i * NRF52_SPI_ONCE_TRANSFER_LEN, NRF52_SPI_ONCE_TRANSFER_LEN,
                               p_rx + i * NRF52_SPI_ONCE_TRANSFER_LEN, NRF52_SPI_ONCE_TRANSFER_LEN);
        }

      /* read only operation */
      if (NULL == txbuffer)
        {
          nrf_drv_spi_transfer(priv, NULL, 0,
                               p_rx + i * NRF52_SPI_ONCE_TRANSFER_LEN, NRF52_SPI_ONCE_TRANSFER_LEN);
        }

      /* write only operation */
      if ( NULL == rxbuffer)
        {
          nrf_drv_spi_transfer(priv, p_tx + i * NRF52_SPI_ONCE_TRANSFER_LEN, NRF52_SPI_ONCE_TRANSFER_LEN,
                               NULL, 0);
        }
      i++;
    }

  /* read/write the leave data */

  if (leaf_size)
    {
      if (txbuffer && rxbuffer)
        {
          nrf_drv_spi_transfer(priv, p_tx + i * NRF52_SPI_ONCE_TRANSFER_LEN, leaf_size,
                               p_rx + i * NRF52_SPI_ONCE_TRANSFER_LEN, leaf_size);
        }

      if (NULL == txbuffer)
        {
          nrf_drv_spi_transfer(priv, NULL, 0,
                               p_rx + i * NRF52_SPI_ONCE_TRANSFER_LEN, leaf_size);
        }

      if (NULL == rxbuffer)
        {
          nrf_drv_spi_transfer(priv, p_tx + i * NRF52_SPI_ONCE_TRANSFER_LEN, leaf_size,
                               NULL, 0);
        }
    }
}

#else
/****************************************************************************
 * Name: nrf52_spi_sndblock
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 ****************************************************************************/
static void nrf52_spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *txbuffer, size_t nwords)
{
  FAR struct nrf52_spidev_s *priv = (struct nrf52_spidev_s *)dev;

  spiinfo("Sndblock: size %d.\n", nwords);

  uint8_t leaf_size;
  uint32_t total_cycle;
  uint32_t i = 0;
  uint8_t *p_uint8 = (uint8_t *)txbuffer;

  /* split send to small size for multi send */

  total_cycle = nwords / NRF52_SPI_ONCE_TRANSFER_LEN;
  leaf_size = nwords % NRF52_SPI_ONCE_TRANSFER_LEN;

  while ( i < total_cycle)
    {
      nrf_drv_spi_transfer(priv, p_uint8 + i * NRF52_SPI_ONCE_TRANSFER_LEN, NRF52_SPI_ONCE_TRANSFER_LEN, NULL, 0);
      i++;
    }

  /* write the leave data */

  if (leaf_size)
    {
      nrf_drv_spi_transfer(priv, p_uint8 + i * NRF52_SPI_ONCE_TRANSFER_LEN, leaf_size, NULL, 0);
    }

}

/****************************************************************************
 * Name: nrf52_spi_recvblock
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 ****************************************************************************/
static void nrf52_spi_recvblock(FAR struct spi_dev_s *dev, FAR void *rxbuffer, size_t nwords)
{
  FAR struct nrf52_spidev_s *priv = (struct nrf52_spidev_s *)dev;

  spiinfo("Recvblock: size %d.\n", nwords);

  uint8_t leaf_size;
  uint32_t total_cycle;
  uint32_t i = 0;
  uint8_t *p_uint8 = (uint8_t *)rxbuffer;

  /* split send to small size for multi send */

  total_cycle = nwords / NRF52_SPI_ONCE_TRANSFER_LEN;
  leaf_size = nwords % NRF52_SPI_ONCE_TRANSFER_LEN;

  while ( i < total_cycle)
    {
      nrf_drv_spi_transfer(priv, NULL, 0, p_uint8 + i * NRF52_SPI_ONCE_TRANSFER_LEN, NRF52_SPI_ONCE_TRANSFER_LEN);
      i++;
    }

  /* read the leave data */

  if (leaf_size)
    {
      nrf_drv_spi_transfer(priv, NULL, 0, p_uint8 + i * NRF52_SPI_ONCE_TRANSFER_LEN, leaf_size);
    }
}

#endif /* CONFIG_SPI_EXCHANGE */

#endif /*CONFIG_NRF52_SPI0 CONFIG_NRF52_SPI1 CONFIG_NRF52_SPI2 */
