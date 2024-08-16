/****************************************************************************
 * arch/arm/src/nrf52/nrf52_i2c.c
 *
 *   Copyright (C) 2018 zGlue Inc. All rights reserved.
 *   Author: Zhengwei Wang <zhengwei@zglue.com>
 *
 * References:
 *
 * -  The framework for this driver is based on stm32 I2C driver
 *    written by Gregory Nutt.
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
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

#include "chip.h"
#include "nrf.h"
#include "nrf_error.h"
#include "nrf_twim.h"
#include "nrf52_gpio.h"
#include "nrf52_i2c.h"
#include "chip/nrf52_i2c.h"

#if defined(CONFIG_NRF52_I2C0) || defined(CONFIG_NRF52_I2C1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ********************************************************************/
/* CONFIG_I2C_POLLED may be set so that I2C interrupts will not be used.  Instead,
 * CPU-intensive polling will be used.
 */

/* Interrupt wait timeout in seconds and milliseconds */

#if !defined(CONFIG_NRF52_I2CTIMEOSEC) && !defined(CONFIG_NRF52_I2CTIMEOMS)
#  define CONFIG_NRF52_I2CTIMEOSEC 0
#  define CONFIG_NRF52_I2CTIMEOMS  500   /* Default is 500 milliseconds */
#elif !defined(CONFIG_NRF52_I2CTIMEOSEC)
#  define CONFIG_NRF52_I2CTIMEOSEC 0     /* User provided milliseconds */
#elif !defined(CONFIG_NRF52_I2CTIMEOMS)
#  define CONFIG_NRF52_I2CTIMEOMS  0     /* User provided seconds */
#endif

/* Interrupt wait time timeout in system timer ticks */

#ifndef CONFIG_NRF52_I2CTIMEOTICKS
#  define CONFIG_NRF52_I2CTIMEOTICKS \
    (SEC2TICK(CONFIG_NRF52_I2CTIMEOSEC) + MSEC2TICK(CONFIG_NRF52_I2CTIMEOMS))
#endif

#ifndef CONFIG_NRF52_I2C_DYNTIMEO_STARTSTOP
#  define CONFIG_NRF52_I2C_DYNTIMEO_STARTSTOP TICK2USEC(CONFIG_NRF52_I2CTIMEOTICKS)
#endif


// All interrupt flags
#define DISABLE_ALL_INT_SHORT  0xFFFFFFFF

#define SCL_PIN_INIT_CONF     ( (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) \
                              | (GPIO_PIN_CNF_DRIVE_H0D1     << GPIO_PIN_CNF_DRIVE_Pos) \
                              | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)  \
                              | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos) \
                              | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos))
#define SDA_PIN_INIT_CONF        SCL_PIN_INIT_CONF

#define SDA_PIN_UNINIT_CONF   ( (GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos) \
                              | (GPIO_PIN_CNF_DRIVE_H0H1       << GPIO_PIN_CNF_DRIVE_Pos) \
                              | (GPIO_PIN_CNF_PULL_Disabled    << GPIO_PIN_CNF_PULL_Pos)  \
                              | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) \
                              | (GPIO_PIN_CNF_DIR_Input        << GPIO_PIN_CNF_DIR_Pos))
#define SCL_PIN_UNINIT_CONF      SDA_PIN_UNINIT_CONF

#define SCL_PIN_INIT_CONF_CLR ( (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) \
                              | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos) \
                              | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)  \
                              | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos) \
                              | (GPIO_PIN_CNF_DIR_Output     << GPIO_PIN_CNF_DIR_Pos))
#define SDA_PIN_INIT_CONF_CLR    SCL_PIN_INIT_CONF_CLR

#define NRF52_TWIM_TXRX_MAXLENGTH 0xff       /*max buffer length in one transfer*/

#define EVT_TO_STR_TWIM(event)  (event == NRF_TWIM_EVENT_STOPPED ? "NRF_TWIM_EVENT_STOPPED" :                      \
                                (event == NRF_TWIM_EVENT_ERROR ? "NRF_TWIM_EVENT_ERROR" :                          \
                                (event == NRF_TWIM_EVENT_SUSPENDED ? "NRF_TWIM_EVENT_SUSPENDED" :                  \
                                (event == NRF_TWIM_EVENT_RXSTARTED ? "NRF_TWIM_EVENT_RXSTARTED" :                  \
                                (event == NRF_TWIM_EVENT_TXSTARTED ? "NRF_TWIM_EVENT_TXSTARTED" :                  \
                                (event == NRF_TWIM_EVENT_LASTRX ? "NRF_TWIM_EVENT_LASTRX" :                        \
                                (event == NRF_TWIM_EVENT_LASTTX ? "NRF_TWIM_EVENT_LASTTX" : "UNKNOWN ERROR")))))))


/****************************************************************************
 * Private Types
 ****************************************************************************/
/* Interrupt state */

typedef enum
{
  INTSTATE_IDLE = 0,      /* No I2C activity */
  INTSTATE_WAITING,       /* Waiting for completion of interrupt activity */
  INTSTATE_DONE,          /* Interrupt activity complete */
} nrf52_tmim_intstate_t;

/* I2C Device hardware configuration */

struct nrf52_i2c_config_s
{
  uint32_t base;              /* I2C base address */
  uint32_t scl_pin;           /* GPIO configuration for SCL as SCL */
  uint32_t sda_pin;           /* GPIO configuration for SDA as SDA */
#ifndef CONFIG_I2C_POLLED
  uint32_t irqno;            /* IRQ number*/
#endif
};

/* I2C Device Private Data */

struct nrf52_i2c_priv_s
{
  const struct i2c_ops_s *ops; /* Standard I2C operations */
  const struct nrf52_i2c_config_s *config; /* Port configuration */
  uint32_t frequency;          /* Current I2C frequency */
  uint8_t address;             /* Current I2C address */
  int refs;                    /* Referernce count */
  sem_t sem_excl;              /* Mutual exclusion semaphore */
#ifndef CONFIG_I2C_POLLED
  sem_t sem_isr;               /* Interrupt wait semaphore */
#endif
  volatile nrf52_tmim_intstate_t intstate;/* Interrupt handshake (see enum nrf52_intstate_e) */

  uint8_t msgc;                /* Message count */
  struct i2c_msg_s *msgv;      /* Message list */
  uint8_t *ptr;                /* Current message buffer */
  int dcnt;                    /* Current message length */
  uint16_t flags;              /* Current message flags */
  bool error;             /*set True when error happend during a transfer */
};


/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

static inline void nrf52_i2c_sem_wait(FAR struct nrf52_i2c_priv_s *priv);

#ifdef CONFIG_nrf52_I2C_DYNTIMEO
static useconds_t nrf52_i2c_tousecs(int msgc, FAR struct i2c_msg_s *msgs);
#endif /* CONFIG_NRF52_I2C_DYNTIMEO */

static int  nrf52_i2c_sem_waitdone(FAR struct nrf52_i2c_priv_s *priv);
static inline void nrf52_i2c_sem_post(FAR struct nrf52_i2c_priv_s *priv);
static inline void nrf52_i2c_sem_init(FAR struct nrf52_i2c_priv_s *priv);
static inline void nrf52_i2c_sem_destroy(FAR struct nrf52_i2c_priv_s *priv);

static void nrf52_i2c_isr_handle_next(struct nrf52_i2c_priv_s *priv);
static void nrf52_i2c_isr_handle_over(struct nrf52_i2c_priv_s *priv);
static void nrf52_i2c_isr_handle_tx(struct nrf52_i2c_priv_s *priv, uint32_t *int_mask, uint32_t *shorts);
static void nrf52_i2c_isr_handle_rx(struct nrf52_i2c_priv_s *priv, uint32_t *int_mask, uint32_t *shorts);
static int nrf52_i2c_handle_timeout(struct nrf52_i2c_priv_s *priv);


#ifndef CONFIG_I2C_POLLED
static int nrf52_i2c_isr(int irq, void *context, FAR void *arg);
#endif /* !CONFIG_I2C_POLLED */

static int nrf52_i2c_set_frequency(FAR struct nrf52_i2c_priv_s *priv, uint32_t frequency);
static int nrf52_i2c_init(FAR struct nrf52_i2c_priv_s *priv);
static int nrf52_i2c_deinit(FAR struct nrf52_i2c_priv_s *priv);
static int nrf52_i2c_transfer(FAR struct i2c_master_s *dev, FAR struct i2c_msg_s *msgs,
                              int count);
#ifdef CONFIG_I2C_RESET
static int nrf52_i2c_reset(FAR struct i2c_master_s *dev);
#endif


/****************************************************************************
 * Private Data
 ****************************************************************************/
struct i2c_ops_s nrf52_i2c_ops =
{
  .transfer = nrf52_i2c_transfer,
#ifdef CONFIG_I2C_RESET
  .reset  = nrf52_i2c_reset
#endif
};

#ifdef CONFIG_NRF52_I2C0
static const struct nrf52_i2c_config_s nrf52_i2c0_config =
{
  .base       = NRF_TWIM0_BASE,
  .scl_pin    = BOARD_I2C0_SCL_PIN,
  .sda_pin    = BOARD_I2C0_SDA_PIN,

#ifndef CONFIG_I2C_POLLED
  .irqno      = SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn,
#endif
};

static struct nrf52_i2c_priv_s nrf52_i2c0_priv =
{
  .ops        = &nrf52_i2c_ops,
  .config     = &nrf52_i2c0_config,
  .frequency  = BOARD_I2C0_FREQ,
  .refs       = 0,
  .intstate   = INTSTATE_IDLE,
  .msgc       = 0,
  .msgv       = NULL,
  .ptr        = NULL,
  .dcnt       = 0,
  .flags      = 0,
  .error      = false,
};
#endif

#ifdef CONFIG_NRF52_I2C1
static const struct nrf52_i2c_config_s nrf52_i2c1_config =
{
  .base       = NRF_TWIM1_BASE,
  .scl_pin    = BOARD_I2C1_SCL_PIN,
  .sda_pin    = BOARD_I2C1_SDA_PIN,

#ifndef CONFIG_I2C_POLLED
  .irqno      = SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn,
#endif

};

static struct nrf52_i2c_priv_s nrf52_i2c1_priv =
{
  .ops        = &nrf52_i2c_ops,
  .config     = &nrf52_i2c1_config,
  .frequency  = BOARD_I2C1_FREQ,
  .refs       = 0,
  .intstate   = INTSTATE_IDLE,
  .msgc       = 0,
  .msgv       = NULL,
  .ptr        = NULL,
  .dcnt       = 0,
  .flags      = 0,
  .error      = false,
};
#endif

/************************************************************************************
 * Name: nrf52_i2c_sem_wait
 *
 * Description:
 *   Take the exclusive access, waiting as necessary
 *
 ************************************************************************************/

static inline void nrf52_i2c_sem_wait(FAR struct nrf52_i2c_priv_s *priv)
{
  int ret;

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_wait(&priv->sem_excl);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);
}

/************************************************************************************
 * Name: nrf52_i2c_tousecs
 *
 * Description:
 *   Return a micro-second delay based on the number of bytes left to be processed.
 *
 ************************************************************************************/

#ifdef CONFIG_NRF52_I2C_DYNTIMEO
static useconds_t nrf52_i2c_tousecs(int msgc, FAR struct i2c_msg_s *msgs)
{
  size_t bytecount = 0;
  int i;

  /* Count the number of bytes left to process */

  for (i = 0; i < msgc; i++)
    {
      bytecount += msgs[i].length;
    }

  /* Then return a number of microseconds based on a user provided scaling
   * factor.
   */

  return (useconds_t)(CONFIG_NRF52_I2C_DYNTIMEO_USECPERBYTE * bytecount);
}
#endif

/************************************************************************************
 * Name: nrf52_i2c_sem_waitdone
 *
 * Description:
 *   Wait for a transfer to complete
 *
 ************************************************************************************/

#ifndef CONFIG_I2C_POLLED
static int nrf52_i2c_sem_waitdone(FAR struct nrf52_i2c_priv_s *priv)
{
  struct timespec abstime;
  int ret;

  /* Signal the interrupt handler that we are waiting.  NOTE:  Interrupts
   * are currently disabled but will be temporarily re-enabled below when
   * nxsem_timedwait() sleeps.
   */

  priv->intstate = INTSTATE_WAITING;

  /* Enable I2C interrupts */

  up_enable_irq(priv->config->irqno);


  do
    {
      /* Get the current time */

      (void)clock_gettime(CLOCK_REALTIME, &abstime);

      /* Calculate a time in the future */

#if CONFIG_NRF52_I2CTIMEOSEC > 0
      abstime.tv_sec += CONFIG_NRF52_I2CTIMEOSEC;
#endif

      /* Add a value proportional to the number of bytes in the transfer */

#ifdef CONFIG_NRF52_I2C_DYNTIMEO
      abstime.tv_nsec += 1000 * nrf52_i2c_tousecs(priv->msgc, priv->msgv);
      if (abstime.tv_nsec >= 1000 * 1000 * 1000)
        {
          abstime.tv_sec++;
          abstime.tv_nsec -= 1000 * 1000 * 1000;
        }

#elif CONFIG_NRF52_I2CTIMEOMS > 0
      abstime.tv_nsec += CONFIG_NRF52_I2CTIMEOMS * 1000 * 1000;
      if (abstime.tv_nsec >= 1000 * 1000 * 1000)
        {
          abstime.tv_sec++;
          abstime.tv_nsec -= 1000 * 1000 * 1000;
        }
#endif

      /* Wait until either the transfer is complete or the timeout expires */

      ret = nxsem_timedwait(&priv->sem_isr, &abstime);
      if (ret < 0 && ret != -EINTR)
        {
          /* Break out of the loop on irrecoverable errors.  This would
           * include timeouts and mystery errors reported by nxsem_timedwait.
           * NOTE that we try again if we are awakened by a signal (EINTR).
           */

          break;
        }
    }

  /* Loop until the interrupt level transfer is complete. */

  while (priv->intstate != INTSTATE_DONE);

  /* Set the interrupt state back to IDLE */

  priv->intstate = INTSTATE_IDLE;

  /* Disable I2C interrupts */

  up_disable_irq(priv->config->irqno);

  return ret;
}
#else
static int nrf52_i2c_sem_waitdone(FAR struct nrf52_i2c_priv_s *priv)
{
  /*need to be implimented*/
  return OK;
}
#endif

/************************************************************************************
 * Name: nrf52_i2c_sem_post
 *
 * Description:
 *   Release the mutual exclusion semaphore
 *
 ************************************************************************************/

static inline void nrf52_i2c_sem_post(struct nrf52_i2c_priv_s *priv)
{
  nxsem_post(&priv->sem_excl);
}

/************************************************************************************
 * Name: nrf52_i2c_sem_init
 *
 * Description:
 *   Initialize semaphores
 *
 ************************************************************************************/

static inline void nrf52_i2c_sem_init(FAR struct nrf52_i2c_priv_s *priv)
{
  nxsem_init(&priv->sem_excl, 0, 1);

#ifndef CONFIG_I2C_POLLED
  /* This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_init(&priv->sem_isr, 0, 0);
  nxsem_setprotocol(&priv->sem_isr, SEM_PRIO_NONE);
#endif
}

/************************************************************************************
 * Name: nrf52_i2c_sem_destroy
 *
 * Description:
 *   Destroy semaphores.
 *
 ************************************************************************************/

static inline void nrf52_i2c_sem_destroy(FAR struct nrf52_i2c_priv_s *priv)
{
  nxsem_destroy(&priv->sem_excl);
#ifndef CONFIG_I2C_POLLED
  nxsem_destroy(&priv->sem_isr);
#endif
}


/************************************************************************************
 * Name: nrf52_i2c_start_transfer
 *
 * Description:
 *   start a i2c transfer
 *
 ************************************************************************************/
static int nrf52_i2c_start_transfer(FAR struct nrf52_i2c_priv_s *priv)
{
  struct i2c_msg_s *msg;
  int ret;
  NRF_TWIM_Type *twim = (NRF_TWIM_Type *)priv->config->base;

  msg = priv->msgv;

  if (msg)
    {

      /*set frequency*/

      if (msg->frequency != priv->frequency)
        {
          nrf52_i2c_set_frequency(priv, msg->frequency);
        }

      /*set address*/

      if (msg->addr != priv->address)
        {
          if (msg->flags & I2C_M_TEN)
            {
              i2cerr("unsupported address %d, nrf52 only support 8bits address.\n", msg->addr);
              ret = -EINVAL;
              goto errorout;
            }
          else
            {
              nrf_twim_address_set(twim, msg->addr);
              priv->address = msg->addr;
            }
        }

      priv->ptr = msg->buffer;
      priv->dcnt = msg->length;
      priv->flags = msg->flags;

      DEBUGASSERT(nrf_is_in_ram(priv->ptr));

      nrf52_i2c_isr_handle_next(priv);

      ret = OK;

    }
  else
    {
      /*we should never come here*/
      i2cerr("empty msg queue!\n");
      ret = -EINVAL;
    }

errorout:
  return ret;

}

/************************************************************************************
 * Name: nrf52_i2c_handle_timeout
 *
 * Description:
 *   Handle the timeout situation, if the i2c peripheral is enabled, we try to resume
 *   and stop the transfer. This may be useless, because the cause of timeout may be
 *   complicated, in case this function failed to handle the timeout, user may need to
 *   reset the i2c bus manually.
 *
 ************************************************************************************/
static int nrf52_i2c_handle_timeout(FAR struct nrf52_i2c_priv_s *priv)
{
  /*I donnot know which situation will cause a timeout in current solution.
   *In case it happend, the following steps may be taken to solve it(I hope).
   */
  systime_t start;
  systime_t elapsed;
  systime_t timeout;
  bool stopped;
  NRF_TWIM_Type *twim = (NRF_TWIM_Type *)priv->config->base;


  if (twim->ENABLE)
    {
      nrf_twim_task_trigger(twim, NRF_TWIM_TASK_RESUME);
      nrf_twim_task_trigger(twim, NRF_TWIM_TASK_STOP);


      /* Select a timeout */

#ifdef CONFIG_NRF52_I2C_DYNTIMEO
      timeout = USEC2TICK(CONFIG_NRF52_I2C_DYNTIMEO_STARTSTOP);
#else
      timeout = CONFIG_NRF52_I2CTIMEOTICKS;
#endif

      /* Wait as stop might still be in progress
       */

      start = clock_systimer();
      do
        {
          /* Calculate the elapsed time */

          elapsed = clock_systimer() - start;

          /* Check for STOP condition */

          stopped = nrf_twim_event_check(twim, NRF_TWIM_EVENT_STOPPED);
          if (stopped)
            {
              nrf_twim_event_clear(twim, NRF_TWIM_EVENT_STOPPED);
              nrf_twim_disable(twim);
              return OK;
            }

        }

      /* Loop until the stop is complete or a timeout occurs. */

      while (elapsed < timeout);

      /* If we get here then a timeout occurred with the STOP condition
       * still pending.
       */

      i2cerr("Handle timeout situation failed!\n");

      return -EIO;

    }

  return OK;
}


/************************************************************************************
 * Name: nrf52_i2c_isr_handle_over
 *
 * Description:
 *   This function is called in isr to handle the situation when a queue of tasks
 *   is completed.
 *
 ************************************************************************************/
void nrf52_i2c_isr_handle_over(struct nrf52_i2c_priv_s *priv)
{
  NRF_TWIM_Type *p_twim = (NRF_TWIM_Type *)priv->config->base;

  nrf_twim_int_disable(p_twim, DISABLE_ALL_INT_SHORT);
  nrf_twim_disable(p_twim);
#ifndef CONFIG_I2C_POLLED
  if (priv->intstate == INTSTATE_WAITING)
    {
      /* Yes.. inform the thread that the transfer is complete
       * and wake it up.
       */

      priv->intstate = INTSTATE_DONE;
      nxsem_post(&priv->sem_isr);

    }
#else
  priv->intstate = INTSTATE_DONE;
#endif

}

/************************************************************************************
 * Name: nrf52_i2c_isr_handle_tx
 *
 * Description:
 *   This function is called in isr to handle the situation when a TX task
 *   is completed.It will start the next transmission.
 *
 ************************************************************************************/
void nrf52_i2c_isr_handle_tx(struct nrf52_i2c_priv_s *priv,
                             uint32_t *int_mask, uint32_t *shorts)
{
  struct i2c_msg_s *msg = priv->msgv;
  struct i2c_msg_s *next_msg;
  uint8_t msgc = priv->msgc;

  NRF_TWIM_Type *twim = (NRF_TWIM_Type *)priv->config->base;

  if (msgc > 1)
    {
      next_msg = msg + 1;
    }
  else
    {
      next_msg = NULL;
    }

  if (priv->dcnt > NRF52_TWIM_TXRX_MAXLENGTH)
    {
      /*continue with current TX*/
      nrf_twim_tx_buffer_set(twim, priv->ptr, NRF52_TWIM_TXRX_MAXLENGTH);
      priv->dcnt -= NRF52_TWIM_TXRX_MAXLENGTH;
      priv->ptr  += NRF52_TWIM_TXRX_MAXLENGTH;
      *int_mask = NRF_TWIM_INT_SUSPENDED_MASK | NRF_TWIM_INT_ERROR_MASK;
      *shorts = NRF_TWIM_SHORT_LASTTX_SUSPEND_MASK;
    }
  else
    {
      /*current TX will stop, try to set up the next one*/
      nrf_twim_tx_buffer_set(twim, priv->ptr, priv->dcnt);
      priv->dcnt = 0;
      priv->ptr  = NULL;

      if (next_msg == NULL)
        {
          /*current TX is the last message, we can stop after this transfer*/
          *int_mask = NRF_TWIM_INT_STOPPED_MASK | NRF_TWIM_INT_ERROR_MASK;
          *shorts = NRF_TWIM_SHORT_LASTTX_STOP_MASK;
        }
      else
        {
          /*current TX is followed by a RX and it requires restart, we can set the rx buffer
          *and enable the NRF_TWIM_SHORT_LASTTX_STARTRX_MASK,so we will not be wake up
          *until the rx transfer is also completed.
          */
          if (((next_msg->flags & I2C_M_READ) != 0) && ((next_msg->flags & I2C_M_NORESTART) == 0))
            {
              /*prepare the next buffer, we should make the priv->ptr points to the rx buffer*/
              priv->msgv ++;
              priv->msgc --;

              priv->ptr = priv->msgv->buffer;
              priv->dcnt = priv->msgv->length;
              priv->flags = priv->msgv->flags;

              msg = priv->msgv;
              msgc = priv->msgc;

              DEBUGASSERT(nrf_is_in_ram(priv->ptr));

              if (priv->dcnt > NRF52_TWIM_TXRX_MAXLENGTH)
                {
                  /*if we cann't finish the rx task in one shot, we should try it again in the next run*/
                  nrf_twim_rx_buffer_set(twim, priv->ptr, NRF52_TWIM_TXRX_MAXLENGTH);
                  priv->dcnt -= NRF52_TWIM_TXRX_MAXLENGTH;
                  priv->ptr  += NRF52_TWIM_TXRX_MAXLENGTH;

#ifdef CONFIG_ARCH_CHIP_NRF52840
                  *int_mask = NRF_TWIM_INT_SUSPENDED_MASK | NRF_TWIM_INT_ERROR_MASK;
                  *shorts = NRF_TWIM_SHORT_LASTTX_STARTRX_MASK | NRF_TWIM_SHORT_LASTRX_SUSPEND_MASK;
#else
                  *int_mask = NRF_TWIM_INT_LASTRX_MASK | NRF_TWIM_INT_ERROR_MASK;
                  *shorts = NRF_TWIM_SHORT_LASTTX_STARTRX_MASK;
#endif



                }
              else
                {
                  /*if we can, we should see the third message*/
                  nrf_twim_rx_buffer_set(twim, priv->ptr, priv->dcnt);
                  priv->dcnt = 0;
                  priv->ptr  = NULL;

                  /*get the third message pointer*/
                  if (msgc > 1)
                    {
                      next_msg = msg + 1;
                    }
                  else
                    {
                      next_msg = NULL;
                    }

                  /*if it's null, we can stop the device after the rx task*/
                  if (next_msg == NULL)
                    {
                      *shorts = NRF_TWIM_SHORT_LASTTX_STARTRX_MASK | NRF_TWIM_SHORT_LASTRX_STOP_MASK;
                      *int_mask = NRF_TWIM_INT_STOPPED_MASK | NRF_TWIM_INT_ERROR_MASK;
                    }
                  else
                    {
                      /*If the third task is a rx task and it requires no stop and start,
                       *or it is a TX task and it reqires restart, we should update the priv->ptr
                       *again, making it points to the third task.
                      */
                      if ((((next_msg->flags & I2C_M_READ) != 0) && ((next_msg->flags & I2C_M_NORESTART) != 0)) || \
                          (((next_msg->flags & I2C_M_READ) == 0) && ((next_msg->flags & I2C_M_NORESTART) == 0)))
                        {
                          priv->msgv ++;
                          priv->msgc --;

                          priv->ptr = priv->msgv->buffer;
                          priv->dcnt = priv->msgv->length;
                          priv->flags = priv->msgv->flags;

                          DEBUGASSERT(nrf_is_in_ram(priv->ptr));

#ifdef CONFIG_ARCH_CHIP_NRF52840
                          *int_mask = NRF_TWIM_INT_SUSPENDED_MASK | NRF_TWIM_INT_ERROR_MASK;
                          *shorts = NRF_TWIM_SHORT_LASTTX_STARTRX_MASK | NRF_TWIM_SHORT_LASTRX_SUSPEND_MASK;
#else
                          *int_mask = NRF_TWIM_INT_LASTRX_MASK | NRF_TWIM_INT_ERROR_MASK;
                          *shorts = NRF_TWIM_SHORT_LASTTX_STARTRX_MASK;
#endif


                        }
                      else
                        {
                          /*we should generate a stop after the RX task*/
                          *shorts = NRF_TWIM_SHORT_LASTTX_STARTRX_MASK | NRF_TWIM_SHORT_LASTRX_STOP_MASK;
                          *int_mask = NRF_TWIM_INT_STOPPED_MASK | NRF_TWIM_INT_ERROR_MASK;
                        }
                    }
                }
            }
          else if (((next_msg->flags & I2C_M_READ) == 0) && ((next_msg->flags & I2C_M_NORESTART) != 0))
            {
              /*current TX is followed by another TX and it requires no stop and start*/
              priv->msgv ++;
              priv->msgc --;

              priv->ptr = priv->msgv->buffer;
              priv->dcnt = priv->msgv->length;
              priv->flags = priv->msgv->flags;

              DEBUGASSERT(nrf_is_in_ram(priv->ptr));

              *int_mask = NRF_TWIM_INT_SUSPENDED_MASK | NRF_TWIM_INT_ERROR_MASK;
              *shorts = NRF_TWIM_SHORT_LASTTX_SUSPEND_MASK;
            }
          else
            {
              /*we should generate a stop after current TX*/
              *int_mask = NRF_TWIM_INT_STOPPED_MASK | NRF_TWIM_INT_ERROR_MASK;
              *shorts = NRF_TWIM_SHORT_LASTTX_STOP_MASK;
            }
        }
    }

}

/************************************************************************************
 * Name: nrf52_i2c_isr_handle_rx
 *
 * Description:
 *   This function is called in isr to handle the situation when a RX task
 *   is completed.It will start the next transmission.
 *
 ************************************************************************************/
void nrf52_i2c_isr_handle_rx(struct nrf52_i2c_priv_s *priv,
                             uint32_t *int_mask, uint32_t *shorts)
{
  struct i2c_msg_s *msg = priv->msgv;
  struct i2c_msg_s *next_msg;
  uint8_t msgc = priv->msgc;

  NRF_TWIM_Type *twim = (NRF_TWIM_Type *)priv->config->base;

  if (msgc > 1)
    {
      next_msg = msg + 1;
    }
  else
    {
      next_msg = NULL;
    }

  if (priv->dcnt > NRF52_TWIM_TXRX_MAXLENGTH)
    {
      /*continue with current RX*/
      nrf_twim_rx_buffer_set(twim, priv->ptr, NRF52_TWIM_TXRX_MAXLENGTH);
      priv->dcnt -= NRF52_TWIM_TXRX_MAXLENGTH;
      priv->ptr  += NRF52_TWIM_TXRX_MAXLENGTH;

#ifdef CONFIG_ARCH_CHIP_NRF52840
      *int_mask = NRF_TWIM_INT_SUSPENDED_MASK | NRF_TWIM_INT_ERROR_MASK;
      *shorts =  NRF_TWIM_SHORT_LASTRX_SUSPEND_MASK;
#else
      *int_mask = NRF_TWIM_INT_LASTRX_MASK | NRF_TWIM_INT_ERROR_MASK;
      *shorts = 0;
#endif

    }
  else
    {
      /*current RX will stop, try to set up the next one*/
      nrf_twim_rx_buffer_set(twim, priv->ptr, priv->dcnt);
      priv->dcnt = 0;
      priv->ptr  = NULL;

      if (next_msg == NULL)
        {
          /*current RX is the last message, we can stop after this transfer*/
          *int_mask = NRF_TWIM_INT_STOPPED_MASK | NRF_TWIM_INT_ERROR_MASK;
          *shorts = NRF_TWIM_SHORT_LASTRX_STOP_MASK;
        }
      else
        {
          /*current RX is followed by a TX and it requires restart, we can set the Tx buffer
          *and enable the NRF_TWIM_SHORT_LASTRX_STARTTX_MASK,so we will not be wake up
          *until the Tx transfer is also completed.
          */
          if (((next_msg->flags & I2C_M_READ) == 0) && ((next_msg->flags & I2C_M_NORESTART) == 0))
            {
              /*prepare the next buffer, we should make the priv->ptr points to the Tx buffer*/
              priv->msgv ++;
              priv->msgc --;

              priv->ptr = priv->msgv->buffer;
              priv->dcnt = priv->msgv->length;
              priv->flags = priv->msgv->flags;

              msg = priv->msgv;
              msgc = priv->msgc;

              DEBUGASSERT(nrf_is_in_ram(priv->ptr));

              if (priv->dcnt > NRF52_TWIM_TXRX_MAXLENGTH)
                {
                  /*if we cann't finish the Tx task in one shot, we should try it again in the next run*/
                  nrf_twim_tx_buffer_set(twim, priv->ptr, NRF52_TWIM_TXRX_MAXLENGTH);
                  priv->dcnt -= NRF52_TWIM_TXRX_MAXLENGTH;
                  priv->ptr  += NRF52_TWIM_TXRX_MAXLENGTH;

                  *shorts = NRF_TWIM_SHORT_LASTRX_STARTTX_MASK | NRF_TWIM_SHORT_LASTTX_SUSPEND_MASK;
                  *int_mask = NRF_TWIM_INT_SUSPENDED_MASK | NRF_TWIM_INT_ERROR_MASK;
                }
              else
                {
                  /*if we can, we should see the third message*/
                  nrf_twim_tx_buffer_set(twim, priv->ptr, priv->dcnt);
                  priv->dcnt = 0;
                  priv->ptr  = NULL;

                  /*get the third message pointer*/
                  if (msgc > 1)
                    {
                      next_msg = msg + 1;
                    }
                  else
                    {
                      next_msg = NULL;
                    }

                  /*if it's null, we can stop the device after the tx task*/
                  if (next_msg == NULL)
                    {
                      *shorts = NRF_TWIM_SHORT_LASTRX_STARTTX_MASK | NRF_TWIM_SHORT_LASTTX_STOP_MASK;
                      *int_mask = NRF_TWIM_INT_STOPPED_MASK | NRF_TWIM_INT_ERROR_MASK;
                    }
                  else
                    {
                      /*If the third task is a Tx task and it requires no stop and start,
                       *or it is a RX task and it reqires restart, we should update the priv->ptr
                       *again, making it points to the third task, and wait for the next run.
                      */
                      if ((((next_msg->flags & I2C_M_READ) == 0) && ((next_msg->flags & I2C_M_NORESTART) != 0)) || \
                          (((next_msg->flags & I2C_M_READ) != 0) && ((next_msg->flags & I2C_M_NORESTART) == 0)))
                        {
                          priv->msgv ++;
                          priv->msgc --;

                          priv->ptr = priv->msgv->buffer;
                          priv->dcnt = priv->msgv->length;
                          priv->flags = priv->msgv->flags;

                          DEBUGASSERT(nrf_is_in_ram(priv->ptr));

                          *shorts = NRF_TWIM_SHORT_LASTRX_STARTTX_MASK | NRF_TWIM_SHORT_LASTTX_SUSPEND_MASK;
                          *int_mask = NRF_TWIM_INT_SUSPENDED_MASK | NRF_TWIM_INT_ERROR_MASK;

                        }
                      else
                        {
                          /*we should generate a stop after the TX task*/
                          *shorts = NRF_TWIM_SHORT_LASTRX_STARTTX_MASK | NRF_TWIM_SHORT_LASTTX_STOP_MASK;
                          *int_mask = NRF_TWIM_INT_STOPPED_MASK | NRF_TWIM_INT_ERROR_MASK;
                        }
                    }
                }
            }
          else if (((next_msg->flags & I2C_M_READ) != 0) && ((next_msg->flags & I2C_M_NORESTART) != 0))
            {
              /*current RX is followed by another RX and it requires no stop and start*/
              priv->msgv ++;
              priv->msgc --;

              priv->ptr = priv->msgv->buffer;
              priv->dcnt = priv->msgv->length;
              priv->flags = priv->msgv->flags;

              DEBUGASSERT(nrf_is_in_ram(priv->ptr));

#ifdef CONFIG_ARCH_CHIP_NRF52840
              *int_mask = NRF_TWIM_INT_SUSPENDED_MASK | NRF_TWIM_INT_ERROR_MASK;
              *shorts =  NRF_TWIM_SHORT_LASTRX_SUSPEND_MASK;
#else
              *int_mask = NRF_TWIM_INT_LASTRX_MASK | NRF_TWIM_INT_ERROR_MASK;
              *shorts = 0;
#endif


            }
          else
            {
              /*we should generate a stop after current RX*/
              *int_mask = NRF_TWIM_INT_STOPPED_MASK | NRF_TWIM_INT_ERROR_MASK;
              *shorts = NRF_TWIM_SHORT_LASTRX_STOP_MASK;
            }
        }
    }

}

/************************************************************************************
 * Name: nrf52_i2c_isr_handle_next
 *
 * Description:
 *   This function is called in isr to handle the situation when a task
 *   is completed.It will call nrf52_i2c_isr_handle_tx or nrf52_i2c_isr_handle_rx
 *   to start the next transmission task.
 *
 ************************************************************************************/
void nrf52_i2c_isr_handle_next(struct nrf52_i2c_priv_s *priv)
{
  uint32_t int_mask;
  uint32_t shorts;
  NRF_TWIM_Type *p_twim = (NRF_TWIM_Type *)priv->config->base;
  nrf_twim_task_t task;

  if (priv->ptr == NULL)
    {
      i2cerr("logic error!\n");/*we should never come here*/
      int_mask = NRF_TWIM_INT_STOPPED_MASK | NRF_TWIM_INT_ERROR_MASK;
      shorts = 0;
      task = NRF_TWIM_TASK_STOP;
    }
  else if ((priv->msgv->flags & I2C_M_READ) == 0)
    {
      nrf52_i2c_isr_handle_tx(priv, &int_mask, &shorts);
      task = NRF_TWIM_TASK_STARTTX;
    }
  else
    {
      nrf52_i2c_isr_handle_rx(priv, &int_mask, &shorts);
      task = NRF_TWIM_TASK_STARTRX;
    }

  nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_LASTTX);
  nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_LASTRX);
  nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_STOPPED);
  nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_SUSPENDED);
  nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_ERROR);

  nrf_twim_shorts_disable(p_twim, DISABLE_ALL_INT_SHORT);
  nrf_twim_shorts_set(p_twim, shorts);
  nrf_twim_int_disable(p_twim, DISABLE_ALL_INT_SHORT);
  nrf_twim_int_enable(p_twim, int_mask);

  nrf_twim_task_trigger(p_twim, task);
  nrf_twim_task_trigger(p_twim, NRF_TWIM_TASK_RESUME);
}


/************************************************************************************
 * Name: nrf52_i2c_isr
 *
 * Description:
 *   Common I2C interrupt service routine
 *
 ************************************************************************************/

#ifndef CONFIG_I2C_POLLED
static int nrf52_i2c_isr(int irq, void *context, FAR void *arg)
{
  struct nrf52_i2c_priv_s *priv = (struct nrf52_i2c_priv_s *)arg;
  NRF_TWIM_Type *p_twim = (NRF_TWIM_Type *)priv->config->base;

  DEBUGASSERT(priv != NULL);

  if (nrf_twim_event_check(p_twim, NRF_TWIM_EVENT_ERROR))
    {
      nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_ERROR);
      i2cinfo("TWIM: Event: %s.\r\n", (uint32_t)EVT_TO_STR_TWIM(NRF_TWIM_EVENT_ERROR));
      priv->error = true;
      if (!nrf_twim_event_check(p_twim, NRF_TWIM_EVENT_STOPPED))
        {
          nrf_twim_int_disable(p_twim, DISABLE_ALL_INT_SHORT);
          nrf_twim_int_enable(p_twim, NRF_TWIM_INT_STOPPED_MASK);

          nrf_twim_task_trigger(p_twim, NRF_TWIM_TASK_RESUME);
          nrf_twim_task_trigger(p_twim, NRF_TWIM_TASK_STOP);
          return OK;
        }
    }

  if (nrf_twim_event_check(p_twim, NRF_TWIM_EVENT_STOPPED))
    {
      nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_STOPPED);
      i2cinfo("TWIM: Event: %s.\r\n", (uint32_t)EVT_TO_STR_TWIM(NRF_TWIM_EVENT_STOPPED));
      if (priv->error == true)
        {
          nrf52_i2c_isr_handle_over(priv);
          return OK;
        }
      else
        {

          if (priv->ptr == NULL)
            {
              priv->msgc --;
              if (priv->msgc == 0)
                {
                  priv->msgv = NULL;
                }
              else
                {
                  priv->msgv++;
                }
            }
          else
            {
              i2cerr("logic error!\n");
            }

          if ( priv->ptr == NULL && priv->msgv == NULL)
            {
              nrf52_i2c_isr_handle_over(priv);
            }
          else
            {
              priv->ptr = priv->msgv->buffer;
              priv->dcnt = priv->msgv->length;
              priv->flags = priv->msgv->flags;

              DEBUGASSERT(nrf_is_in_ram(priv->ptr));

              nrf52_i2c_isr_handle_next(priv);
            }

        }

    }
  else if (nrf_twim_event_check(p_twim, NRF_TWIM_EVENT_SUSPENDED))
    {
      nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_SUSPENDED);
      i2cinfo("TWIM: Event: %s.\r\n", (uint32_t)EVT_TO_STR_TWIM(NRF_TWIM_EVENT_SUSPENDED));

      nrf52_i2c_isr_handle_next(priv);
    }
  else if (nrf_twim_event_check(p_twim, NRF_TWIM_EVENT_LASTRX))
    {
      nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_LASTRX);
      i2cinfo("TWIM: Event: %s.\r\n", (uint32_t)EVT_TO_STR_TWIM(NRF_TWIM_EVENT_LASTRX));

      nrf52_i2c_isr_handle_next(priv);

    }


  return OK;
}
#endif

/************************************************************************************
 * Name: nrf52_i2c_set_frequency
 *
 * Description:
 *   set i2c peripheral frequency.
 *
 ************************************************************************************/
static int nrf52_i2c_set_frequency(FAR struct nrf52_i2c_priv_s *priv, uint32_t frequency)
{
  int ret = OK;
  NRF_TWIM_Type *twim = (NRF_TWIM_Type *)priv->config->base;

  switch (frequency)
    {
      case 100000:/*100KHZ*/
        nrf_twim_frequency_set(twim, NRF_TWIM_FREQ_100K);
        priv->frequency = frequency;
        break;
      case 250000:/*250KHZ*/
        nrf_twim_frequency_set(twim, NRF_TWIM_FREQ_250K);
        priv->frequency = frequency;
        break;
      case 400000:
        nrf_twim_frequency_set(twim, NRF_TWIM_FREQ_400K);
        priv->frequency = frequency;
        break;
      default:
        nrf_twim_frequency_set(twim, NRF_TWIM_FREQ_100K);
        priv->frequency = 100000;
        i2cwarn("unsupported frequency %d,use default frequency %d\n", frequency, 100000);
        ret = -EINVAL;
        break;
    }
  return ret;
}

/************************************************************************************
 * Name: nrf52_i2c_init
 *
 * Description:
 *   Setup the I2C hardware, ready for operation with defaults
 *
 ************************************************************************************/

static int nrf52_i2c_init(FAR struct nrf52_i2c_priv_s *priv)
{
  NRF_TWIM_Type *twim = (NRF_TWIM_Type *)priv->config->base;

  /* Power-up and configure GPIOs */

  /* Configure pins */

  NRF_GPIO->PIN_CNF[priv->config->scl_pin] = SCL_PIN_INIT_CONF;
  NRF_GPIO->PIN_CNF[priv->config->sda_pin] = SDA_PIN_INIT_CONF;
  nrf_twim_pins_set(twim, priv->config->scl_pin, priv->config->sda_pin);

  /* Attach ISRs */

#ifndef CONFIG_I2C_POLLED
  irq_attach(priv->config->irqno, nrf52_i2c_isr, priv);
#endif

  /* set default frequency */

  nrf52_i2c_set_frequency(priv, priv->frequency);

  return OK;
}

/************************************************************************************
 * Name: nrf52_i2c_deinit
 *
 * Description:
 *   Shutdown the I2C hardware
 *
 ************************************************************************************/

static int nrf52_i2c_deinit(FAR struct nrf52_i2c_priv_s *priv)
{
  NRF_TWIM_Type *twim = (NRF_TWIM_Type *)priv->config->base;

  /* Disable I2C */

  nrf_twim_int_disable(twim, DISABLE_ALL_INT_SHORT);
  nrf_twim_shorts_disable(twim, DISABLE_ALL_INT_SHORT);
  nrf_twim_disable(twim);

  /* Unconfigure GPIO pins */
  NRF_GPIO->PIN_CNF[priv->config->scl_pin] = SCL_PIN_UNINIT_CONF;
  NRF_GPIO->PIN_CNF[priv->config->sda_pin] = SDA_PIN_UNINIT_CONF;

  /* Disable and detach interrupts */

#ifndef CONFIG_I2C_POLLED
  up_disable_irq(priv->config->irqno);
  irq_detach(priv->config->irqno);
#endif

  return OK;
}

/************************************************************************************
 * Device Driver Operations
 ************************************************************************************/

/************************************************************************************
 * Name: nrf52_i2c_transfer
 *
 * Description:
 *   Generic I2C transfer function
 *
 ************************************************************************************/

static int nrf52_i2c_transfer(FAR struct i2c_master_s *dev, FAR struct i2c_msg_s *msgs,
                              int count)
{
  FAR struct nrf52_i2c_priv_s *priv = (struct nrf52_i2c_priv_s *)dev;
  uint32_t errorsrc;
  int ret = OK;
  NRF_TWIM_Type *twim = (NRF_TWIM_Type *)priv->config->base;

  DEBUGASSERT(count > 0);

  /* Ensure that address or flags don't change meanwhile */

  nrf52_i2c_sem_wait(priv);

  /*The TWI peripheral should have been disabled at the end of last transfer*/

  DEBUGASSERT(!twim->ENABLE);

  /* Old transfers are done */

  /* Reset ptr and dcnt to ensure an unexpected data interrupt doesn't
   * overwrite stale data.
   */

  priv->dcnt = 0;
  priv->ptr = NULL;

  priv->msgv = msgs;
  priv->msgc = count;

  priv->error = false;

  nrf_twim_int_disable(twim, DISABLE_ALL_INT_SHORT);
  nrf_twim_shorts_disable(twim, DISABLE_ALL_INT_SHORT);

  nrf_twim_enable(twim);

  /* Trigger start condition, then the process moves into the ISR..
   */

  ret = nrf52_i2c_start_transfer(priv);

  if (ret < 0)
    {
      nrf_twim_disable(twim);
      goto error_out_sem;
    }

  /* Wait for an ISR, if there was a timeout, fetch latest status to get
   * the BUSY flag.
   */

  if (nrf52_i2c_sem_waitdone(priv) < 0)
    {
      ret = -ETIMEDOUT;

      i2cerr("ERROR: Time out\n");

      /*handle the timeout error*/

      if (nrf52_i2c_handle_timeout(priv) < 0)
        {
          /*If we fail to handle this problem, user may need to reset the i2c bus.*/
          ret = -EBUSY;
        }
    }
  else
    {
      errorsrc =  nrf_twim_errorsrc_get_and_clear(twim);

      /*handle the error source*/

      if (errorsrc & NRF_TWI_ERROR_OVERRUN)
        {
          ret = -EIO;
        }

      if (errorsrc & NRF_TWI_ERROR_ADDRESS_NACK)
        {
          ret = -ENXIO;
        }

      if (errorsrc & NRF_TWI_ERROR_DATA_NACK)
        {
          ret = -EIO;
        }

    }

error_out_sem:

  /* Ensure that any ISR happening after we finish can't overwrite any user data */

  priv->dcnt = 0;
  priv->ptr = NULL;

  nrf52_i2c_sem_post(priv);
  return ret;
}

/************************************************************************************
 * Name: nrf52_i2c_reset
 *
 * Description:
 *   Perform an I2C bus reset in an attempt to break loose stuck I2C devices.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ************************************************************************************/

#ifdef CONFIG_I2C_RESET
static int nrf52_i2c_reset(FAR struct i2c_master_s *dev)
{
  FAR struct nrf52_i2c_priv_s *priv = (struct nrf52_i2c_priv_s *)dev;
  unsigned int clock_count;
  unsigned int stretch_count;
  uint32_t scl_gpio;
  uint32_t sda_gpio;
  uint32_t frequency;
  int ret = ERROR;

  ASSERT(dev);

  /* Our caller must own a ref */

  ASSERT(priv->refs > 0);

  /* Lock out other clients */

  nrf52_i2c_sem_wait(priv);

  /* De-init the port */

  nrf52_i2c_deinit(priv);

  /* Use GPIO configuration to un-wedge the bus */

  NRF_GPIO->PIN_CNF[priv->config->scl_pin] = SCL_PIN_INIT_CONF_CLR;
  NRF_GPIO->PIN_CNF[priv->config->sda_pin] = SDA_PIN_INIT_CONF_CLR;


  /* Let SDA go high */

  nrf_gpio_pin_set(priv->config->sda_pin);
  nrf_gpio_pin_set(priv->config->scl_pin);


  /* Clock the bus until any slaves currently driving it let it go. */

  clock_count = 0;
  while (!nrf_gpio_pin_read(priv->config->sda_pin))
    {
      /* Give up if we have tried too hard */

      if (clock_count++ > 10)
        {
          goto out;
        }

      /* Sniff to make sure that clock stretching has finished.
       *
       * If the bus never relaxes, the reset has failed.
       */

      stretch_count = 0;
      while (!nrf_gpio_pin_read(priv->config->scl_pin))
        {
          /* Give up if we have tried too hard */

          if (stretch_count++ > 10)
            {
              goto out;
            }

          up_udelay(10);
        }

      /* Drive SCL low */

      nrf_gpio_pin_write(priv->config->scl_pin, 0);
      up_udelay(10);

      /* Drive SCL high again */

      nrf_gpio_pin_write(priv->config->scl_pin, 1);
      up_udelay(10);
    }

  /* Generate a start followed by a stop to reset slave
   * state machines.
   */

  nrf_gpio_pin_write(priv->config->sda_pin, 0);
  up_udelay(10);
  nrf_gpio_pin_write(priv->config->scl_pin, 0);
  up_udelay(10);
  nrf_gpio_pin_write(priv->config->scl_pin, 1);
  up_udelay(10);
  nrf_gpio_pin_write(priv->config->sda_pin, 1);
  up_udelay(10);

  /* Re-init the port */

  nrf52_i2c_init(priv);

  ret = OK;

out:

  /* Release the port for re-use by other clients */

  nrf52_i2c_sem_post(priv);
  return ret;
}
#endif /* CONFIG_I2C_RESET */


/************************************************************************************
 * Public Functions
 ************************************************************************************/



/************************************************************************************
 * Name: nrf52_i2cbus_initialize
 *
 * Description:
 *   Initialize one I2C bus
 *
 ************************************************************************************/

FAR struct i2c_master_s *nrf52_i2cbus_initialize(int port)
{
  struct nrf52_i2c_priv_s *priv = NULL;
  irqstate_t flags;

  /* Get I2C private structure */

  switch (port)
    {
#ifdef CONFIG_NRF52_I2C0
      case 0:
        priv = (struct nrf52_i2c_priv_s *)&nrf52_i2c0_priv;
        break;
#endif
#ifdef CONFIG_NRF52_I2C1
      case 1:
        priv = (struct nrf52_i2c_priv_s *)&nrf52_i2c1_priv;
        break;
#endif
      default:
        return NULL;
    }

  /* Initialize private data for the first time, increment reference count,
   * power-up hardware and configure GPIOs.
   */

  flags = enter_critical_section();

  if ((volatile int)priv->refs++ == 0)
    {
      leave_critical_section(flags);

      nrf52_i2c_sem_init(priv);
      nrf52_i2c_init(priv);

      flags = enter_critical_section();

    }

  leave_critical_section(flags);
  return (struct i2c_master_s *)priv;
}


/****************************************************************************
 * Name: nrf52_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialise an I2C device
 *
 ****************************************************************************/
int nrf52_i2cbus_uninitialize(FAR struct i2c_master_s *dev)
{
  FAR struct nrf52_i2c_priv_s *priv = (struct nrf52_i2c_priv_s *)dev;
  irqstate_t flags;

  ASSERT(dev);

  /* Decrement reference count and check for underflow */

  if (priv->refs == 0)
    {
      return ERROR;
    }

  flags = enter_critical_section();

  if (--priv->refs)
    {
      leave_critical_section(flags);
      return OK;
    }

  leave_critical_section(flags);

  /* Disable power and other HW resource (GPIO's) */

  nrf52_i2c_deinit(priv);

  /* Release unused resources */

  nrf52_i2c_sem_destroy(priv);
  return OK;
}


#endif /*CONFIG_NRF52_I2C0 CONFIG_NRF52_I2C1 */
