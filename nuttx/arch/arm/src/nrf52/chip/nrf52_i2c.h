/************************************************************************************
 * arch/arm/src/nrf52/chip/nrf52_i2c.h
 *
 *   Copyright (C) 2009, 2011, 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_NRF52_CHIP_NRF52_I2C_H
#define __ARCH_ARM_SRC_NRF52_CHIP_NRF52_I2C_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define NRF_DRV_TWI_FLAG_TX_POSTINC          (1UL << 0) /**< TX buffer address incremented after transfer. */
#define NRF_DRV_TWI_FLAG_RX_POSTINC          (1UL << 1) /**< RX buffer address incremented after transfer. */
#define NRF_DRV_TWI_FLAG_NO_XFER_EVT_HANDLER (1UL << 2) /**< Interrupt after each transfer is suppressed, and the event handler is not called. */
#define NRF_DRV_TWI_FLAG_HOLD_XFER           (1UL << 3) /**< Set up the transfer but do not start it. */
#define NRF_DRV_TWI_FLAG_REPEATED_XFER       (1UL << 4) /**< Flag indicating that the transfer will be executed multiple times. */
#define NRF_DRV_TWI_FLAG_TX_NO_STOP          (1UL << 5) /**< Flag indicating that the TX transfer will not end with a stop condition. */

/**
 * @defgroup drv_specific_errors Error / status codes specific to drivers.
 * @{
 */
#define NRF_ERROR_PERIPH_DRIVERS_ERR_BASE   (0x8200)
#define NRF_ERROR_DRV_TWI_ERR_OVERRUN              (NRF_ERROR_PERIPH_DRIVERS_ERR_BASE + 0x0000)
#define NRF_ERROR_DRV_TWI_ERR_ANACK                (NRF_ERROR_PERIPH_DRIVERS_ERR_BASE + 0x0001)
#define NRF_ERROR_DRV_TWI_ERR_DNACK                (NRF_ERROR_PERIPH_DRIVERS_ERR_BASE + 0x0002)

/**@brief Macro for setting the TX transfer descriptor. */
#define NRF_DRV_TWI_XFER_DESC_TX(addr, p_data, length)                 \
    {                                                                  \
        .type = NRF_DRV_TWI_XFER_TX,                                   \
        .address = addr,                                               \
        .primary_length = length,                                      \
        .p_primary_buf  = p_data,                                      \
    }

/**@brief Macro for setting the RX transfer descriptor. */
#define NRF_DRV_TWI_XFER_DESC_RX(addr, p_data, length)                 \
    {                                                                  \
        .type = NRF_DRV_TWI_XFER_RX,                                   \
        .address = addr,                                               \
        .primary_length = length,                                      \
        .p_primary_buf  = p_data,                                      \
    }

/**@brief Macro for setting the TXRX transfer descriptor. */
#define NRF_DRV_TWI_XFER_DESC_TXRX(addr, p_tx, tx_len, p_rx, rx_len)   \
    {                                                                  \
        .type = NRF_DRV_TWI_XFER_TXRX,                                 \
        .address = addr,                                               \
        .primary_length   = tx_len,                                    \
        .secondary_length = rx_len,                                    \
        .p_primary_buf    = p_tx,                                      \
        .p_secondary_buf  = p_rx,                                      \
    }

/**@brief Macro for setting the TXTX transfer descriptor. */
#define NRF_DRV_TWI_XFER_DESC_TXTX(addr, p_tx, tx_len, p_tx2, tx_len2) \
    {                                                                  \
        .type = NRF_DRV_TWI_XFER_TXTX,                                 \
        .address = addr,                                               \
        .primary_length   = tx_len,                                    \
        .secondary_length = tx_len2,                                   \
        .p_primary_buf    = p_tx,                                      \
        .p_secondary_buf  = p_tx2,                                     \
    }

/**
 * @enum nrf_i2c_frequency_t
 * @brief i2c frequency supported by nrf52.
 */
typedef enum
{
  NRF_I2C_FREQUENCY_100K   =  26738688,
  NRF_I2C_FREQUENCY_250K   =  67108864,
  NRF_I2C_FREQUENCY_400K   =  104857600,
} nrf_i2c_frequency_t;


/**
 * @brief Structure for the TWI master driver instance configuration.
 */
typedef struct
{
  uint32_t            scl;                 ///< SCL pin number.
  uint32_t            sda;                 ///< SDA pin number.
  nrf_i2c_frequency_t frequency;           ///< TWI frequency.
  uint8_t             interrupt_priority;  ///< Interrupt priority.
  bool                clear_bus_init;      ///< Clear bus during init.
  bool                hold_bus_uninit;     ///< Hold pull up state on gpio pins after uninit.
} nrf_drv_twi_config_t;

typedef enum
{
  NRF_TWI_ERROR_ADDRESS_NACK = TWI_ERRORSRC_ANACK_Msk,  ///< NACK received after sending the address.
  NRF_TWI_ERROR_DATA_NACK    = TWI_ERRORSRC_DNACK_Msk,  ///< NACK received after sending a data byte.
  NRF_TWI_ERROR_OVERRUN      = TWI_ERRORSRC_OVERRUN_Msk ///< Overrun error.
                               /**< A new byte was received before the previous byte was read
                                *   from the RXD register (previous data is lost). */
} nrf_twi_error_t;

/**
 * @brief TWI master driver event types.
 */
typedef enum
{
  NRF_DRV_TWI_EVT_DONE,         ///< Transfer completed event.
  NRF_DRV_TWI_EVT_ADDRESS_NACK, ///< Error event: NACK received after sending the address.
  NRF_DRV_TWI_EVT_DATA_NACK     ///< Error event: NACK received after sending a data byte.
} nrf_drv_twi_evt_type_t;

/**
 * @brief TWI master driver transfer types.
 */
typedef enum
{
  NRF_DRV_TWI_XFER_TX,          ///< TX transfer.
  NRF_DRV_TWI_XFER_RX,          ///< RX transfer.
  NRF_DRV_TWI_XFER_TXRX,        ///< TX transfer followed by RX transfer with repeated start.
  NRF_DRV_TWI_XFER_TXTX         ///< TX transfer followed by TX transfer with repeated start.
} nrf_drv_twi_xfer_type_t;

/**
 * @brief Structure for a TWI transfer descriptor.
 */
typedef struct
{
  nrf_drv_twi_xfer_type_t type;             ///< Type of transfer.
  uint8_t                 address;          ///< Slave address.
  uint8_t                 primary_length;   ///< Number of bytes transferred.
  uint8_t                 secondary_length; ///< Number of bytes transferred.
  uint8_t                *p_primary_buf;    ///< Pointer to transferred data.
  uint8_t                *p_secondary_buf;  ///< Pointer to transferred data.
} nrf_drv_twi_xfer_desc_t;

/**
 * @brief Structure for a TWI event.
 */
typedef struct
{
  nrf_drv_twi_evt_type_t  type;      ///< Event type.
  nrf_drv_twi_xfer_desc_t xfer_desc; ///< Transfer details.
} nrf_drv_twi_evt_t;

/**
 * @brief TWI event handler prototype.
 */
typedef void (* nrf_drv_twi_evt_handler_t)(nrf_drv_twi_evt_t const *p_event,
                                           void                     *p_context);

#endif /* __ARCH_ARM_SRC_NRF52DK_CHIP_NRF52_I2C_H */

