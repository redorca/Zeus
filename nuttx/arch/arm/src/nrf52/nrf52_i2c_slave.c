/****************************************************************************
 * arch/arm/src/nrf52/nrf52_i2c_slave.c
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
#include <nuttx/wqueue.h>
#include <nuttx/i2c/i2c_slave.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

#include "chip.h"
#include "nrf.h"
#include "nrf_error.h"
#include "nrf_twis.h"
#include "nrf52_gpio.h"

#if defined(CONFIG_NRF52_I2CS0) || defined(CONFIG_NRF52_I2CS1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ********************************************************************/
/* CONFIG_I2C_POLLED may be set so that I2C interrupts will not be used.  Instead,
 * CPU-intensive polling will be used.
 */

/* Interrupt wait timeout in seconds and milliseconds */




// All interrupt flags
#define DISABLE_ALL_INT_SHORT  0xFFFFFFFF

#define NRF52_TWIM_TXRX_MAXLENGTH 0xff       /*max buffer length in one transfer*/

#define EVT_TO_STR_TWIS(event)  (event == NRF_TWIS_EVENT_STOPPED ? "NRF_TWIS_EVENT_STOPPED" :                      \
                                (event == NRF_TWIS_EVENT_ERROR ? "NRF_TWIS_EVENT_ERROR" :                          \
                                (event == NRF_TWIS_EVENT_RXSTARTED ? "NRF_TWIS_EVENT_RXSTARTED" :                  \
                                (event == NRF_TWIS_EVENT_TXSTARTED ? "NRF_TWIS_EVENT_TXSTARTED" :                  \
                                (event == NRF_TWIS_EVENT_WRITE ? "NRF_TWIS_EVENT_WRITE" :                  \
                                (event == NRF_TWIS_EVENT_READ ? "NRF_TWIS_EVENT_READ" : "NRF_TWIS_EVENT_UNKNOWN"))))))

#define STATUS_TO_STR_TWIM(status)  (status == NRF52_TWIS_STATE_IDLE ? "NRF52_TWIS_STATE_IDLE" :                      \
                                (status == NRF52_TWIS_STATE_READ_WAITING ? "NRF52_TWIS_STATE_READ_WAITING" :                          \
                                (status == NRF52_TWIS_STATE_READING ? "NRF52_TWIS_STATE_READING" :                  \
                                (status == NRF52_TWIS_STATE_WRITE_WAITING ? "NRF52_TWIS_STATE_WRITE_WAITING" :                  \
                                (status == NRF52_TWIS_STATE_WRITING ? "NRF52_TWIS_STATE_WRITING" : "NRF52_TWIS_STATE_UNKNOWN")))))

#define NRF_DRV_COMMON_EVREGS_OFFSET 0x100U


/****************************************************************************
 * Private Types
 ****************************************************************************/
/* Interrupt state */

typedef enum
{
  INTSTATE_IDLE = 0,      /* No I2C activity */
  INTSTATE_WAITING,       /* Waiting for completion of interrupt activity */
  INTSTATE_DONE,          /* Interrupt activity complete */
} nrf52_tmis_intstate_t;

/**
 * @brief Actual state of internal state machine
 */
typedef enum
{
  NRF52_TWIS_STATE_IDLE,          ///< No ongoing transmission
  NRF52_TWIS_STATE_READ_WAITING,  ///< Read request received, waiting for data
  NRF52_TWIS_STATE_READING,  ///< Reading is actually pending (data sending)
  NRF52_TWIS_STATE_WRITE_WAITING, ///< Write request received, waiting for data buffer
  NRF52_TWIS_STATE_WRITING, ///< Writing is actually pending (data receiving)
} nrf52_twis_state_t;


/* I2C Device hardware configuration */

struct nrf52_i2cs_config_s
{
  uint32_t base;              /* I2C base address */
  uint32_t scl_pin;           /* GPIO configuration for SCL as SCL */
  uint32_t sda_pin;           /* GPIO configuration for SDA as SDA */
  nrf_gpio_pin_pull_t sda_pullup;
  nrf_gpio_pin_pull_t scl_pullup;
  uint32_t irqno;            /* IRQ number*/
};

/* I2C Device Private Data */

struct nrf52_i2cs_priv_s
{
  const struct i2c_slaveops_s *ops; /* Standard I2C operations */
  const struct nrf52_i2cs_config_s *config; /* Port configuration */
  int refs;                    /* Referernce count */
  int (*callback)(FAR void *, i2c_slave_flag );/*user callback function*/
  void *callback_args; /*args for the callback function*/
  sem_t sem_excl;              /* Mutual exclusion semaphore */
  sem_t sem_pend;              /*user are waitting for the read/write results*/
  struct work_s work;          /* this work will clear all apb buffers in the done queue */
  nrf52_twis_state_t status; /*current status*/
  bool flag_tx_buffer_set;  /*tx buffer set*/
  bool flag_rx_buffer_set;  /*tx buffer set*/
  bool tx_error;           /*set True when error happend during a tx transfer */
  bool rx_error;           /*set True when error happend during a rx transfer */
  int sended_length;       /*data length we sended*/
  int received_length;     /*data length we received*/
};


/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

static inline void nrf52_i2cs_sem_wait(FAR struct nrf52_i2cs_priv_s *priv);
static inline void nrf52_i2cs_sem_post(FAR struct nrf52_i2cs_priv_s *priv);
static inline void nrf52_i2cs_sem_init(FAR struct nrf52_i2cs_priv_s *priv);
static inline void nrf52_i2cs_sem_destroy(FAR struct nrf52_i2cs_priv_s *priv);
int nrf52_i2cs_enable(FAR struct i2c_slave_s *dev, int addr, int nbits);
int nrf52_i2cs_write(FAR struct i2c_slave_s *dev, FAR const uint8_t *buffer, int buflen);
int nrf52_i2cs_read(FAR struct i2c_slave_s *dev, FAR uint8_t *buffer, int buflen);
int nrf52_i2cs_registercb(FAR struct i2c_slave_s *dev,
                          int (*callback)(FAR void *arg, i2c_slave_flag flag),
                          FAR void *arg);




/****************************************************************************
 * Private Data
 ****************************************************************************/
struct i2c_slaveops_s nrf52_i2cs_ops =
{
  .setownaddress = nrf52_i2cs_enable,
  .write = nrf52_i2cs_write,
  .read = nrf52_i2cs_read,
  .registercallback = nrf52_i2cs_registercb,
};

#ifdef CONFIG_NRF52_I2CS0
static const struct nrf52_i2cs_config_s nrf52_i2cs0_config =
{
  .base       = NRF_TWIS0_BASE,
  .scl_pin    = BOARD_I2CS0_SCL_PIN,
  .sda_pin    = BOARD_I2CS0_SDA_PIN,

#ifdef BOADR_I2CS0_SCL_PULLUP_ENABLE
  .scl_pullup = NRF_GPIO_PIN_PULLUP,
#else
  .scl_pullup = NRF_GPIO_PIN_NOPULL,
#endif

#ifdef BOADR_I2CS0_SDA_PULLUP_ENABLE
  .sda_pullup = NRF_GPIO_PIN_PULLUP,
#else
  .sda_pullup = NRF_GPIO_PIN_NOPULL,
#endif

  .irqno      = SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn,
};

static struct nrf52_i2cs_priv_s nrf52_i2cs0_priv =
{
  .ops        = &nrf52_i2cs_ops,
  .config     = &nrf52_i2cs0_config,
  .refs       = 0,
  .callback   = NULL,
  .callback_args = NULL,
  .status = NRF52_TWIS_STATE_IDLE,
  .flag_tx_buffer_set = false,
  .flag_rx_buffer_set = false,
  .tx_error      = false,
  .rx_error      = false,
  .sended_length = 0,
  .received_length = 0,
};
#endif

#ifdef CONFIG_NRF52_I2CS1
static const struct nrf52_i2cs_config_s nrf52_i2cs1_config =
{
  .base       = NRF_TWIS1_BASE,
  .scl_pin    = BOARD_I2CS1_SCL_PIN,
  .sda_pin    = BOARD_I2CS1_SDA_PIN,

#ifdef BOADR_I2CS1_SCL_PULLUP_ENABLE
  .scl_pullup = NRF_GPIO_PIN_PULLUP,
#else
  .scl_pullup = NRF_GPIO_PIN_NOPULL,
#endif

#ifdef BOADR_I2CS1_SDA_PULLUP_ENABLE
  .sda_pullup = NRF_GPIO_PIN_PULLUP,
#else
  .sda_pullup = NRF_GPIO_PIN_NOPULL,
#endif

  .irqno      = SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn,
};

static struct nrf52_i2cs_priv_s nrf52_i2cs1_priv =
{
  .ops        = &nrf52_i2cs_ops,
  .config     = &nrf52_i2cs1_config,
  .refs       = 0,
  .callback   = NULL,
  .callback_args = NULL,
  .status = NRF52_TWIS_STATE_IDLE,
  .flag_tx_buffer_set = false,
  .flag_rx_buffer_set = false,
  .tx_error      = false,
  .rx_error      = false,
  .sended_length = 0,
  .received_length = 0,
};
#endif

/************************************************************************************
 * Name: nrf52_i2cs_sem_wait
 *
 * Description:
 *   Take the exclusive access, waiting as necessary
 *
 ************************************************************************************/

static inline void nrf52_i2cs_sem_wait(FAR struct nrf52_i2cs_priv_s *priv)
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
 * Name: nrf52_i2c_sem_post
 *
 * Description:
 *   Release the mutual exclusion semaphore
 *
 ************************************************************************************/

static inline void nrf52_i2cs_sem_post(struct nrf52_i2cs_priv_s *priv)
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

static inline void nrf52_i2cs_sem_init(FAR struct nrf52_i2cs_priv_s *priv)
{
  nxsem_init(&priv->sem_excl, 0, 1);
  nxsem_init(&priv->sem_pend, 0, 0);
}

/************************************************************************************
 * Name: nrf52_i2c_sem_destroy
 *
 * Description:
 *   Destroy semaphores.
 *
 ************************************************************************************/

static inline void nrf52_i2cs_sem_destroy(FAR struct nrf52_i2cs_priv_s *priv)
{
  nxsem_destroy(&priv->sem_excl);
  nxsem_destroy(&priv->sem_pend);
}

static inline uint32_t nrf52_i2cs_event_to_bitpos(uint32_t event)
{
  return (event - NRF_DRV_COMMON_EVREGS_OFFSET) / sizeof(uint32_t);
}


/**
 * @brief Auxiliary function for getting event state on right bit possition
 *
 * This function calls @ref nrf_twis_event_get function but the the result
 * is shifted to match INTEN register scheme.
 *
 * @param[in,out] p_reg TWIS to read  event from
 * @param ev  Event code
 *
 * @return Selected event state shifted by @ref nrf52_i2cs_event_to_bitpos
 *
 * @sa nrf_twis_event_get
 * @sa nrf52_i2cs_event_to_bitpos
 */
static inline uint32_t nrf52_i2cs_event_bit_get(NRF_TWIS_Type * const p_reg, nrf_twis_event_t ev)
{
  return (uint32_t)nrf_twis_event_get_and_clear(p_reg, ev) << nrf52_i2cs_event_to_bitpos(ev);
}

/**
 * @brief Auxiliary function for checking event bit inside given flags value
 *
 * Function used here to check presence of the event inside given flags value.
 * It transforms given event to bit possition and then checks if in given variable it is cleared.
 *
 * @param flags Flags to test
 * @param ev Event code
 *
 * @retval true Flag for selected event is set
 * @retval false Flag for selected event is cleared
 */
static inline bool nrf52_i2cs_check_bit(uint32_t flags, nrf_twis_event_t ev)
{
  return 0 != (flags & (1U << nrf52_i2cs_event_to_bitpos(ev)));
}

/**
 * @brief Auxiliary function for clearing event bit in given flags value
 *
 * Function used to clear selected event bit.
 *
 * @param flags Flags to process
 * @param ev    Event code to clear
 *
 * @return Value @em flags with cleared event bit that matches given @em ev
 */
static inline uint32_t nrf52_i2cs_clear_bit(uint32_t flags, nrf_twis_event_t ev)
{
  return flags & ~(1U << nrf52_i2cs_event_to_bitpos(ev));
}

static void nrf52_i2cs_worker(void *arg)
{
  struct nrf52_i2cs_priv_s *priv = (struct nrf52_i2cs_priv_s *)arg;
  NRF_TWIS_Type *twis = (NRF_TWIS_Type *)priv->config->base;
  int (*callback)(FAR void *, i2c_slave_flag );
  void *callback_args;
  irqstate_t flags;


  flags = enter_critical_section();
  callback = priv->callback;
  callback_args = priv->callback_args;
  leave_critical_section(flags);

  if (priv->status == NRF52_TWIS_STATE_READ_WAITING)
    {
      if (callback != NULL)
        {
          callback(callback_args, I2CS_R);
        }

      /*if the user doesn't set callback function or
       *the user doesn't set tx buffer in the callback function
       */
      if (!priv->flag_tx_buffer_set)
        {
          nrf_twis_task_trigger(twis, NRF_TWIS_TASK_STOP);
        }
    }
  else if (priv->status == NRF52_TWIS_STATE_WRITE_WAITING)
    {
      if (callback != NULL)
        {
          callback(callback_args, I2CS_W);
        }

      /*if the user doesn't set callback function or
       *the user doesn't set rx buffer in the callback function
       */
      if (!priv->flag_rx_buffer_set)
        {
          nrf_twis_task_trigger(twis, NRF_TWIS_TASK_STOP);
        }
    }
  else
    {
      /*if something happend when we are waitting user to set TRX buffer*/
      nrf_twis_task_trigger(twis, NRF_TWIS_TASK_STOP);
    }

}


/**
 * @brief State machine main function
 *
 * State machine function that reacts on events.
 * This function gets all events and reacts on them only if there is any event detected.
 * It makes it possible to use it either in interrupt or in polling mode.
 * @param instNr Driver instance number that has called this runtime.
 */
static int nrf52_i2cs_isr(int irq, void *context, FAR void *arg)
{
  struct nrf52_i2cs_priv_s *priv = (struct nrf52_i2cs_priv_s *)arg;
  NRF_TWIS_Type *twis = (NRF_TWIS_Type *)priv->config->base;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Current state copy  */
  nrf52_twis_state_t state = priv->status;

  /* Event flags */
  uint32_t ev = 0;

  /* Get all events */
  ev |= nrf52_i2cs_event_bit_get(twis, NRF_TWIS_EVENT_STOPPED);
  ev |= nrf52_i2cs_event_bit_get(twis, NRF_TWIS_EVENT_ERROR);
  ev |= nrf52_i2cs_event_bit_get(twis, NRF_TWIS_EVENT_RXSTARTED);
  ev |= nrf52_i2cs_event_bit_get(twis, NRF_TWIS_EVENT_TXSTARTED);
  ev |= nrf52_i2cs_event_bit_get(twis, NRF_TWIS_EVENT_WRITE);
  ev |= nrf52_i2cs_event_bit_get(twis, NRF_TWIS_EVENT_READ);

  /* State machine */
  while (0 != ev)
    {
      switch (state)
        {
          case NRF52_TWIS_STATE_IDLE:
            if (nrf52_i2cs_check_bit(ev, NRF_TWIS_EVENT_STOPPED))
              {
                /* Stopped event is always allowed in IDLE state - just ignore */
                ev = nrf52_i2cs_clear_bit(ev, NRF_TWIS_EVENT_STOPPED);
              }
            else if (nrf52_i2cs_check_bit(ev, NRF_TWIS_EVENT_READ))
              {

                if (priv->flag_rx_buffer_set)
                  {
                    priv->rx_error = true;
                    nxsem_post(&priv->sem_pend);
                  }
                priv->flag_rx_buffer_set = false;

                if (priv->flag_tx_buffer_set)
                  {
                    nrf_twis_task_trigger(twis, NRF_TWIS_TASK_RESUME);
                    state = NRF52_TWIS_STATE_READING;
                  }
                else
                  {
                    state = NRF52_TWIS_STATE_READ_WAITING;
                    /*schedule worker*/
                    if (work_available(&priv->work))
                      {
                        /* call the user callback on the worker thread. */

                        ret = work_queue(LPWORK, &priv->work, nrf52_i2cs_worker, priv, 0);
                        if (ret != 0)
                          {
                            i2cerr("ERROR: Failed to queue work: %d\n", ret);
                          }
                      }

                  }
                ev = nrf52_i2cs_clear_bit(ev, NRF_TWIS_EVENT_READ);
                ev = nrf52_i2cs_clear_bit(ev, NRF_TWIS_EVENT_TXSTARTED);
                ev = nrf52_i2cs_clear_bit(ev, NRF_TWIS_EVENT_WRITE);
                ev = nrf52_i2cs_clear_bit(ev, NRF_TWIS_EVENT_RXSTARTED);
              }
            else if (nrf52_i2cs_check_bit(ev, NRF_TWIS_EVENT_WRITE))
              {
                if (priv->flag_tx_buffer_set)
                  {
                    priv->tx_error = true;
                    nxsem_post(&priv->sem_pend);
                  }
                priv->flag_tx_buffer_set = false;

                if (priv->flag_rx_buffer_set)
                  {
                    nrf_twis_task_trigger(twis, NRF_TWIS_TASK_RESUME);
                    state = NRF52_TWIS_STATE_WRITING;
                  }
                else
                  {
                    state = NRF52_TWIS_STATE_WRITE_WAITING;
                    /*schedule worker*/
                    if (work_available(&priv->work))
                      {
                        /* call the user callback on the worker thread. */

                        ret = work_queue(LPWORK, &priv->work, nrf52_i2cs_worker, priv, 0);
                        if (ret != 0)
                          {
                            i2cerr("ERROR: Failed to queue work: %d\n", ret);
                          }
                      }

                  }

                ev = nrf52_i2cs_clear_bit(ev, NRF_TWIS_EVENT_READ);
                ev = nrf52_i2cs_clear_bit(ev, NRF_TWIS_EVENT_TXSTARTED);
                ev = nrf52_i2cs_clear_bit(ev, NRF_TWIS_EVENT_WRITE);
                ev = nrf52_i2cs_clear_bit(ev, NRF_TWIS_EVENT_RXSTARTED);
              }
            else
              {
                /*error happend, I suppose it will never happen*/
                ev = 0;
              }
            break;
          case NRF52_TWIS_STATE_READ_WAITING:
            if (nrf52_i2cs_check_bit(ev, NRF_TWIS_EVENT_TXSTARTED))
              {
                state = NRF52_TWIS_STATE_READING;
                /* Any other bits requires further processing in PENDING substate */
                ev = nrf52_i2cs_clear_bit(ev, NRF_TWIS_EVENT_TXSTARTED);
              }
            else
              {
                priv->tx_error = true;
                if (priv->flag_tx_buffer_set)
                  {
                    nxsem_post(&priv->sem_pend);
                  }
                priv->flag_tx_buffer_set = false;
                state = NRF52_TWIS_STATE_IDLE;
                ev = 0;
              }
            break;
          case NRF52_TWIS_STATE_READING:
            if (nrf52_i2cs_check_bit(ev, NRF_TWIS_EVENT_WRITE) ||
                nrf52_i2cs_check_bit(ev, NRF_TWIS_EVENT_READ) ||
                nrf52_i2cs_check_bit(ev, NRF_TWIS_EVENT_STOPPED))
              {

                priv->sended_length = nrf_twis_tx_amount_get(twis);
                if (priv->flag_tx_buffer_set)
                  {
                    nxsem_post(&priv->sem_pend);
                  }
                priv->flag_tx_buffer_set = false;

                /* Go to idle and repeat the state machine if READ or WRITE events detected.
                 * This time READ or WRITE would be started */
                state = NRF52_TWIS_STATE_IDLE;
                ev = nrf52_i2cs_clear_bit(ev, NRF_TWIS_EVENT_STOPPED);
              }
            else
              {
                priv->tx_error = true;
                if (priv->flag_tx_buffer_set)
                  {
                    nxsem_post(&priv->sem_pend);
                  }
                priv->flag_tx_buffer_set = false;

                state = NRF52_TWIS_STATE_IDLE;
                ev = 0;
              }
            break;
          case NRF52_TWIS_STATE_WRITE_WAITING:
            if (nrf52_i2cs_check_bit(ev, NRF_TWIS_EVENT_RXSTARTED))
              {
                state = NRF52_TWIS_STATE_WRITING;
                /* Any other bits requires further processing in PENDING substate */
                ev = nrf52_i2cs_clear_bit(ev, NRF_TWIS_EVENT_RXSTARTED);
              }
            else
              {
                priv->rx_error = true;
                if (priv->flag_rx_buffer_set)
                  {
                    nxsem_post(&priv->sem_pend);
                  }
                priv->flag_rx_buffer_set = false;

                state = NRF52_TWIS_STATE_IDLE;
                ev = 0;
              }
            break;
          case NRF52_TWIS_STATE_WRITING:
            if (nrf52_i2cs_check_bit(ev, NRF_TWIS_EVENT_WRITE) ||
                nrf52_i2cs_check_bit(ev, NRF_TWIS_EVENT_READ) ||
                nrf52_i2cs_check_bit(ev, NRF_TWIS_EVENT_STOPPED))
              {
                priv->received_length = nrf_twis_rx_amount_get(twis);
                if (priv->flag_rx_buffer_set)
                  {
                    nxsem_post(&priv->sem_pend);
                  }
                priv->flag_rx_buffer_set = false;

                /* Go to idle and repeat the state machine if READ or WRITE events detected.
                 * This time READ or WRITE would be started */
                state = NRF52_TWIS_STATE_IDLE;
                ev = nrf52_i2cs_clear_bit(ev, NRF_TWIS_EVENT_STOPPED);
              }
            else
              {
                priv->rx_error = true;
                if (priv->flag_rx_buffer_set)
                  {
                    nxsem_post(&priv->sem_pend);
                  }
                priv->flag_rx_buffer_set = false;

                state = NRF52_TWIS_STATE_IDLE;
                ev = 0;
              }
            break;
          default:
            state = NRF52_TWIS_STATE_IDLE;
            /* Do not clear any events and repeat the machine */
            break;
        }
    }

  priv->status = state;

  return OK;
}



/*************************************************************************************
 * Name: nrf52_i2cs_reset
 *
 * Description:
 *   Reset all the registers to known state
 *
 *   This function clears all registers that requires it to known state.
 *   TWIS is left disabled after this function.
 *   All events are cleared.
 *
 * Input Parameters:
 *   p_reg -- TWIS to reset register address
 ************************************************************************************/
static inline void nrf52_i2cs_reset(FAR struct nrf52_i2cs_priv_s *priv)
{
  NRF_TWIS_Type *twis = (NRF_TWIS_Type *)priv->config->base;

  /* Disable TWIS */
  nrf_twis_disable(twis);

  /* Disable interrupt global for the instance */
  up_disable_irq(priv->config->irqno);

  /*disable all shorts*/
  nrf_twis_shorts_disable(twis, NRF_TWIS_SHORT_WRITE_SUSPEND_MASK | NRF_TWIS_SHORT_READ_SUSPEND_MASK);

  /*clear all events*/
  nrf_twis_event_clear(twis, NRF_TWIS_EVENT_STOPPED);
  nrf_twis_event_clear(twis, NRF_TWIS_EVENT_ERROR);
  nrf_twis_event_clear(twis, NRF_TWIS_EVENT_RXSTARTED);
  nrf_twis_event_clear(twis, NRF_TWIS_EVENT_TXSTARTED);
  nrf_twis_event_clear(twis, NRF_TWIS_EVENT_WRITE);
  nrf_twis_event_clear(twis, NRF_TWIS_EVENT_READ);

  /* Disable interrupts */
  nrf_twis_int_disable(twis, NRF_TWIS_INT_STOPPED_MASK);
  nrf_twis_int_disable(twis, NRF_TWIS_INT_ERROR_MASK);
  nrf_twis_int_disable(twis, NRF_TWIS_INT_RXSTARTED_MASK);
  nrf_twis_int_disable(twis, NRF_TWIS_INT_TXSTARTED_MASK);
  nrf_twis_int_disable(twis, NRF_TWIS_INT_WRITE_MASK);
  nrf_twis_int_disable(twis, NRF_TWIS_INT_READ_MASK);

}


/*************************************************************************************
 * Name: nrf52_i2cs_config_pin
 *
 * Description:
 *   Configure pin
 *
 *   Function configures selected for work as SDA or SCL.
 *
 * Input Parameters:
 *   pin -- Pin number to configure
 ************************************************************************************/

static inline void nrf52_i2cs_config_pin(uint32_t pin, nrf_gpio_pin_pull_t pull)
{
  nrf_gpio_cfg(pin,
               NRF_GPIO_PIN_DIR_INPUT,
               NRF_GPIO_PIN_INPUT_DISCONNECT,
               pull,
               NRF_GPIO_PIN_S0D1,
               NRF_GPIO_PIN_NOSENSE);
}


/************************************************************************************
 * Name: nrf52_i2cs_init
 *
 * Description:
 *   Setup the I2C slave hardware, ready for operation with defaults
 *
 ************************************************************************************/

static int nrf52_i2cs_init(FAR struct nrf52_i2cs_priv_s *priv)
{
  NRF_TWIS_Type *twis = (NRF_TWIS_Type *)priv->config->base;

  /*software reset*/
  nrf52_i2cs_reset(priv);

  /* Power-up and configure GPIOs */

  /* Configure pins */
  nrf52_i2cs_config_pin(priv->config->scl_pin, priv->config->scl_pullup);
  nrf52_i2cs_config_pin(priv->config->sda_pin, priv->config->sda_pullup);
  nrf_twis_pins_set(twis, priv->config->scl_pin, priv->config->sda_pin);

  /*disable all address*/
  nrf_twis_config_address_set(twis, 0);

  /*enable all shorts*/
  nrf_twis_shorts_enable(twis, NRF_TWIS_SHORT_WRITE_SUSPEND_MASK | NRF_TWIS_SHORT_READ_SUSPEND_MASK);

  /*set interrupt mask*/
  nrf_twis_int_enable(twis, NRF_TWIS_INT_STOPPED_MASK);
  nrf_twis_int_enable(twis, NRF_TWIS_INT_ERROR_MASK);
  nrf_twis_int_enable(twis, NRF_TWIS_INT_RXSTARTED_MASK);
  nrf_twis_int_enable(twis, NRF_TWIS_INT_TXSTARTED_MASK);
  nrf_twis_int_enable(twis, NRF_TWIS_INT_WRITE_MASK);
  nrf_twis_int_enable(twis, NRF_TWIS_INT_READ_MASK);

  /* Attach ISRs */
  irq_attach(priv->config->irqno, nrf52_i2cs_isr, priv);


  return OK;
}

/************************************************************************************
 * Name: nrf52_i2cs_deinit
 *
 * Description:
 *   Shutdown the I2C hardware
 *
 ************************************************************************************/

static int nrf52_i2cs_deinit(FAR struct nrf52_i2cs_priv_s *priv)
{
  NRF_TWIS_Type *twis = (NRF_TWIS_Type *)priv->config->base;

  /* software reset */
  nrf52_i2cs_reset(priv);

  /* Clear pins configration */
  nrf_gpio_cfg_default(priv->config->scl_pin);
  nrf_gpio_cfg_default(priv->config->sda_pin);

  /* Disconnect pins */
  nrf_twis_pins_set(twis, ~0U, ~0U);

  /* Disable and detach interrupts */
  irq_detach(priv->config->irqno);

  return OK;
}

/************************************************************************************
 * Device Driver Operations
 ************************************************************************************/

/****************************************************************************
 * Name: I2CS_SETOWNADDRESS
 *
 * Description:
 *   Set our own I2C address. Calling this function enables Slave mode and
 *   disables Master mode on the I2C bus (note that I2C is a bus, where
 *   multiple masters and slave may be handled by one device driver).
 *
 *   One may register a callback to be notified about reception. During the
 *   slave mode reception, the methods READ and WRITE must be used to
 *   to handle reads and writes from a master.
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   address - Our own slave address; If it is 0x00, then the device driver
 *             listens to general call
 *   nbits   - The number of address bits provided (7 or 10)
 *
 * Returned Value:
 *   OK on valid address and if the same address has not been assigned
 *   to another instance sharing the same port. Otherwise ERROR is returned.
 *
 ****************************************************************************/
int nrf52_i2cs_enable(FAR struct i2c_slave_s *dev, int addr, int nbits)
{
  FAR struct nrf52_i2cs_priv_s *priv = (struct nrf52_i2cs_priv_s *)dev;
  int ret = OK;
  NRF_TWIS_Type *twis = (NRF_TWIS_Type *)priv->config->base;

  /* Ensure that address or flags don't change meanwhile */

  nrf52_i2cs_sem_wait(priv);

  /*if we are reading or writting*/
  if (priv->status != NRF52_TWIS_STATE_IDLE)
    {
      return ERROR;
    }

  /*we only support 7bit address*/
  if (nbits != 7)
    {
      return ERROR;
    }

  /*if the address is bigger than 128*/
  if ((nbits == 7) && ((addr & (~0x0000007f)) != 0))
    {
      return ERROR;
    }

  /*reset the device*/
  nrf52_i2cs_reset(priv);

  /*set address*/
  nrf_twis_address_set(twis, 0, addr);
  nrf_twis_config_address_set(twis, NRF_TWIS_CONFIG_ADDRESS0_MASK);

  /*set interrupt mask*/
  nrf_twis_int_enable(twis, NRF_TWIS_INT_STOPPED_MASK);
  nrf_twis_int_enable(twis, NRF_TWIS_INT_ERROR_MASK);
  nrf_twis_int_enable(twis, NRF_TWIS_INT_RXSTARTED_MASK);
  nrf_twis_int_enable(twis, NRF_TWIS_INT_TXSTARTED_MASK);
  nrf_twis_int_enable(twis, NRF_TWIS_INT_WRITE_MASK);
  nrf_twis_int_enable(twis, NRF_TWIS_INT_READ_MASK);

  /*enable all shorts*/
  nrf_twis_shorts_enable(twis, NRF_TWIS_SHORT_WRITE_SUSPEND_MASK | NRF_TWIS_SHORT_READ_SUSPEND_MASK);


  /* enable irq */
  up_enable_irq(priv->config->irqno);

  /* enable TWIS */
  nrf_twis_enable(twis);

  nrf52_i2cs_sem_post(priv);
  return ret;


}

/****************************************************************************
 * Name: I2CS_WRITE
 *
 * Description:
 *   Send a block of data on I2C using the previously selected I2C
 *   frequency and slave address. Each write operational will be an 'atomic'
 *   operation in the sense that any other I2C actions will be serialized
 *   and pend until this write completes. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buffer - A pointer to the read-only buffer of data to be written to device
 *   buflen - The number of bytes to send from the buffer
 *
 * Returned Value:
 *   return the tranferd length on success;
 *   return a negated errno on fail.
 *
 ****************************************************************************/
int nrf52_i2cs_write(FAR struct i2c_slave_s *dev, FAR const uint8_t *buffer,
                     int buflen)
{
  FAR struct nrf52_i2cs_priv_s *priv = (struct nrf52_i2cs_priv_s *)dev;
  int ret = OK;
  NRF_TWIS_Type *twis = (NRF_TWIS_Type *)priv->config->base;
  irqstate_t flags;

  /* Ensure that address or flags don't change meanwhile */

  nrf52_i2cs_sem_wait(priv);

  /*we can do this only in idle status or master are waiting to read*/
  if ((priv->status != NRF52_TWIS_STATE_IDLE) && (priv->status != NRF52_TWIS_STATE_READ_WAITING))
    {
      i2cerr("write at a wrong occation! current:%s \n", STATUS_TO_STR_TWIM(priv->status));
      ret = ERROR;
      goto errout;
    }

  DEBUGASSERT(nrf_is_in_ram(buffer));

  /* Check data size */
  if ((buflen & TWIS_TXD_MAXCNT_MAXCNT_Msk) != buflen)
    {
      i2cerr("write too long! length:%d, MAX:%d \n", buflen, TWIS_TXD_MAXCNT_MAXCNT_Msk);
      ret = ERROR;
      goto errout;
    }

  flags = enter_critical_section();

  /*set tx buffer*/
  nrf_twis_tx_prepare(twis, (uint8_t const *)buffer, (nrf_twis_amount_t)buflen);
  priv->flag_tx_buffer_set = true;

  if (priv->status == NRF52_TWIS_STATE_READ_WAITING)
    {
      nrf_twis_task_trigger(twis, NRF_TWIS_TASK_RESUME);
    }

  leave_critical_section(flags);

  /*wait until the transfer is done*/
  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_wait(&priv->sem_pend);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);

  /*get transfer reuslt*/
  if (!priv->tx_error)
    {
      ret = priv->sended_length;
    }
  else
    {
      priv->tx_error = false;
      ret = ERROR;
    }

errout:
  nrf52_i2cs_sem_post(priv);
  return ret;


}

/****************************************************************************
 * Name: I2CS_READ
 *
 * Description:
 *   Receive a block of data from I2C using the previously selected I2C
 *   frequency and slave address. Each read operational will be an 'atomic'
 *   operation in the sense that any other I2C actions will be serialized
 *   and pend until this read completes. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buffer - A pointer to a buffer of data to receive the data from the device
 *   buflen - The requested number of bytes to be read
 *
 * Returned Value:
 *   return the tranferd length on success;
 *   return a negated errno on fail.
 *
 ****************************************************************************/
int nrf52_i2cs_read(FAR struct i2c_slave_s *dev, FAR uint8_t *buffer,
                    int buflen)
{
  FAR struct nrf52_i2cs_priv_s *priv = (struct nrf52_i2cs_priv_s *)dev;
  int ret = OK;
  NRF_TWIS_Type *twis = (NRF_TWIS_Type *)priv->config->base;
  irqstate_t flags;

  /* Ensure that address or flags don't change meanwhile */

  nrf52_i2cs_sem_wait(priv);

  /*we can do this only in idle status or master are waiting to read*/
  if ((priv->status != NRF52_TWIS_STATE_IDLE) && (priv->status != NRF52_TWIS_STATE_WRITE_WAITING))
    {
      i2cerr("read at a wrong occation! current:%s \n", STATUS_TO_STR_TWIM(priv->status));
      ret = ERROR;
      goto errout;
    }

  DEBUGASSERT(nrf_is_in_ram(buffer));

  /* Check data size */
  if ((buflen & TWIS_TXD_MAXCNT_MAXCNT_Msk) != buflen)
    {
      i2cerr("read too long! length:%d, MAX:%d \n", buflen, TWIS_TXD_MAXCNT_MAXCNT_Msk);
      ret = ERROR;
      goto errout;
    }

  flags = enter_critical_section();

  /*set tx buffer*/
  nrf_twis_rx_prepare(twis, buffer, (nrf_twis_amount_t)buflen);
  priv->flag_rx_buffer_set = true;

  if (priv->status == NRF52_TWIS_STATE_WRITE_WAITING)
    {
      nrf_twis_task_trigger(twis, NRF_TWIS_TASK_RESUME);
    }

  leave_critical_section(flags);


  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_wait(&priv->sem_pend);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);

  /*get transfer reuslt*/
  if (!priv->rx_error)
    {
      ret = priv->received_length;
    }
  else
    {
      priv->rx_error = false;
      ret = ERROR;
    }

errout:
  nrf52_i2cs_sem_post(priv);
  return ret;


}

/****************************************************************************
 * Name: I2CS_REGISTERCALLBACK
 *
 * Description:
 *   Register to receive a callback when something is received on I2C.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   callback - The function to be called when something has been received.
 *   arg      - User provided argument to be used with the callback
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/
int nrf52_i2cs_registercb(FAR struct i2c_slave_s *dev,
                          int (*callback)(FAR void *arg, i2c_slave_flag flag), FAR void *arg)
{
  FAR struct nrf52_i2cs_priv_s *priv = (struct nrf52_i2cs_priv_s *)dev;
  irqstate_t flags;


  flags = enter_critical_section();
  priv->callback = callback;
  priv->callback_args = arg;
  leave_critical_section(flags);

  return OK;
}



/************************************************************************************
 * Public Functions
 ************************************************************************************/
/************************************************************************************
 * Name: nrf52_i2cslave_initialize
 *
 * Description:
 *   Initialize one I2C slave
 *
 ************************************************************************************/

FAR struct i2c_slave_s *nrf52_i2cslave_initialize(int port)
{
  struct nrf52_i2cs_priv_s *priv = NULL;
  irqstate_t flags;

  /* Get I2C private structure */

  switch (port)
    {
#ifdef CONFIG_NRF52_I2CS0
      case 0:
        priv = (struct nrf52_i2cs_priv_s *)&nrf52_i2cs0_priv;
        break;
#endif
#ifdef CONFIG_NRF52_I2CS1
      case 1:
        priv = (struct nrf52_i2cs_priv_s *)&nrf52_i2cs1_priv;
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

      nrf52_i2cs_sem_init(priv);
      nrf52_i2cs_init(priv);

      flags = enter_critical_section();

    }

  leave_critical_section(flags);
  return (struct i2c_slave_s *)priv;
}


/****************************************************************************
 * Name: nrf52_i2cslave_uninitialize
 *
 * Description:
 *   Uninitialise an I2C slave device
 *
 ****************************************************************************/
int nrf52_i2cslave_uninitialize(FAR struct i2c_slave_s *dev)
{
  FAR struct nrf52_i2cs_priv_s *priv = (struct nrf52_i2cs_priv_s *)dev;
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

  nrf52_i2cs_deinit(priv);

  /* Release unused resources */

  nrf52_i2cs_sem_destroy(priv);
  return OK;
}


#endif /*CONFIG_NRF52_I2CS0 CONFIG_NRF52_I2CS1 */
