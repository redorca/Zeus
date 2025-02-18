/*
 * Copyright (c) Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 *   3. Neither the name of Nordic Semiconductor ASA nor the names of other
 *   contributors to this software may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 *   4. This software must only be used in a processor manufactured by Nordic
 *   Semiconductor ASA, or in a processor manufactured by a third party that
 *   is used in combination with a processor manufactured by Nordic Semiconductor.
 *
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**
  @defgroup nrf_sdm_api SoftDevice Manager API
  @{

  @brief APIs for SoftDevice management.

 */

#ifndef NRF_SDM_H__
#define NRF_SDM_H__

#include "nrf_svc.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "nrf_error_sdm.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup NRF_SDM_DEFINES Defines
 * @{ */
#ifdef NRFSOC_DOXYGEN
/// Declared in nrf_mbr.h
#define MBR_SIZE 0
#warning test
#endif

/** @brief The major version for the SoftDevice binary distributed with this header file. */
#define SD_MAJOR_VERSION  (5)

/** @brief The minor version for the SoftDevice binary distributed with this header file. */
#define SD_MINOR_VERSION  (0)

/** @brief The bugfix version for the SoftDevice binary distributed with this header file. */
#define SD_BUGFIX_VERSION (0)

/** @brief The full version number for the SoftDevice binary this header file was distributed
 *         with, as a decimal number in the form Mmmmbbb, where:
 *           - M is major version (one or more digits)
 *           - mmm is minor version (three digits)
 *           - bbb is bugfix version (three digits). */
#define SD_VERSION (SD_MAJOR_VERSION * 1000000 + SD_MINOR_VERSION * 1000 + SD_BUGFIX_VERSION)

/** @brief SoftDevice Manager SVC Base number. */
#define SDM_SVC_BASE 0x10

/** @brief Invalid info field. Returned when an info field does not exist. */
#define SDM_INFO_FIELD_INVALID (0)

/** @brief Defines the SoftDevice Information Structure location (address) as an offset from
the start of the softdevice (without MBR)*/
#define SOFTDEVICE_INFO_STRUCT_OFFSET (0x2000)

/** @brief Defines the absolute SoftDevice Information Structure location (address) when the
 *         SoftDevice is installed just above the MBR (the usual case). */
#define SOFTDEVICE_INFO_STRUCT_ADDRESS (SOFTDEVICE_INFO_STRUCT_OFFSET + MBR_SIZE)

/** @brief Defines the offset for the SoftDevice Information Structure size value relative to the
 *         SoftDevice base address. The size value is of type uint8_t. */
#define SD_INFO_STRUCT_SIZE_OFFSET (SOFTDEVICE_INFO_STRUCT_OFFSET)

/** @brief Defines the offset for the SoftDevice size value relative to the SoftDevice base address.
 *         The size value is of type uint32_t. */
#define SD_SIZE_OFFSET (SOFTDEVICE_INFO_STRUCT_OFFSET + 0x08)

/** @brief Defines the offset for FWID value relative to the SoftDevice base address. The FWID value
 *         is of type uint16_t.  */
#define SD_FWID_OFFSET (SOFTDEVICE_INFO_STRUCT_OFFSET + 0x0C)

/** @brief Defines the offset for the SoftDevice ID relative to the SoftDevice base address. The ID
 *         is of type uint32_t. */
#define SD_ID_OFFSET (SOFTDEVICE_INFO_STRUCT_OFFSET + 0x10)

/** @brief Defines the offset for the SoftDevice version relative to the SoftDevice base address in
 *         the same format as @ref SD_VERSION, stored as an uint32_t. */
#define SD_VERSION_OFFSET (SOFTDEVICE_INFO_STRUCT_OFFSET + 0x14)

/** @brief Defines a macro for retreiving the actual SoftDevice Information Structure size value
 *         from a given base address. Use @ref MBR_SIZE as the argument when the SoftDevice is
 *         installed just above the MBR (the usual case). */
#define SD_INFO_STRUCT_SIZE_GET(baseaddr) (*((uint8_t *) ((baseaddr) + SD_INFO_STRUCT_SIZE_OFFSET)))

/** @brief Defines a macro for retreiving the actual SoftDevice size value from a given base
 *         address. Use @ref MBR_SIZE as the argument when the SoftDevice is installed just above
 *         the MBR (the usual case). */
#define SD_SIZE_GET(baseaddr) (*((uint32_t *) ((baseaddr) + SD_SIZE_OFFSET)))

/** @brief Defines a macro for retreiving the actual FWID value from a given base address. Use @ref
 *         MBR_SIZE as the argument when the SoftDevice is installed just above the MBR (the usual
 *         case). */
#define SD_FWID_GET(baseaddr) (*((uint16_t *) ((baseaddr) + SD_FWID_OFFSET)))

/** @brief Defines a macro for retreiving the actual SoftDevice ID from a given base address. Use
 *         @ref MBR_SIZE as the argument when the SoftDevice is installed just above the MBR (the
 *         usual case). */
#define SD_ID_GET(baseaddr) ((SD_INFO_STRUCT_SIZE_GET(baseaddr) > (SD_ID_OFFSET - SOFTDEVICE_INFO_STRUCT_OFFSET)) \
        ? (*((uint32_t *) ((baseaddr) + SD_ID_OFFSET))) : SDM_INFO_FIELD_INVALID)

/** @brief Defines a macro for retreiving the actual FWID value from a given base address. Use @ref
 *         MBR_SIZE as the argument when the SoftDevice is installed just above the MBR (the usual
 *         case). */
#define SD_VERSION_GET(baseaddr) ((SD_INFO_STRUCT_SIZE_GET(baseaddr) > (SD_VERSION_OFFSET - SOFTDEVICE_INFO_STRUCT_OFFSET)) \
        ? (*((uint32_t *) ((baseaddr) + SD_VERSION_OFFSET))) : SDM_INFO_FIELD_INVALID)

/**@defgroup NRF_FAULT_ID_RANGES Fault ID ranges
 * @{ */
#define NRF_FAULT_ID_SD_RANGE_START     0x00000000            /**< SoftDevice ID range start. */
#define NRF_FAULT_ID_APP_RANGE_START    0x00001000            /**< Application ID range start. */
/**@} */

/**@defgroup NRF_FAULT_IDS Fault ID types
 * @{ */
#define NRF_FAULT_ID_SD_ASSERT    (NRF_FAULT_ID_SD_RANGE_START  + 1)          /**< SoftDevice assertion. The info parameter is reserved for future used. */
#define NRF_FAULT_ID_APP_MEMACC   (NRF_FAULT_ID_APP_RANGE_START + 1)          /**< Application invalid memory access (nRF52 only). The info parameter will contain 0x00000000, in case of SoftDevice RAM
                                                                              access violation. In case of SoftDevice peripheral register violation the info parameter will contain the sub-region number of PREGION[0], on whose address range the unallowed
                                                                              write access caused the memory access fault. */
/**@} */

/** @} */

/** @addtogroup NRF_SDM_ENUMS Enumerations
 * @{ */

/**@brief nRF SoftDevice Manager API SVC numbers. */
enum NRF_SD_SVCS
{
  SD_SOFTDEVICE_ENABLE = SDM_SVC_BASE, /**< ::sd_softdevice_enable */
  SD_SOFTDEVICE_DISABLE,               /**< ::sd_softdevice_disable */
  SD_SOFTDEVICE_IS_ENABLED,            /**< ::sd_softdevice_is_enabled */
  SD_SOFTDEVICE_VECTOR_TABLE_BASE_SET, /**< ::sd_softdevice_vector_table_base_set */
  SVC_SDM_LAST                         /**< Placeholder for last SDM SVC */
};

/** @} */

/** @addtogroup NRF_SDM_DEFINES Defines
 * @{ */

/**@defgroup NRF_CLOCK_LF_XTAL_ACCURACY Clock accuracy
 * @{ */

#define NRF_CLOCK_LF_XTAL_ACCURACY_250_PPM (0) /**< Default: 250 ppm */
#define NRF_CLOCK_LF_XTAL_ACCURACY_500_PPM (1) /**< 500 ppm */
#define NRF_CLOCK_LF_XTAL_ACCURACY_150_PPM (2) /**< 150 ppm */
#define NRF_CLOCK_LF_XTAL_ACCURACY_100_PPM (3) /**< 100 ppm */
#define NRF_CLOCK_LF_XTAL_ACCURACY_75_PPM  (4) /**< 75 ppm */
#define NRF_CLOCK_LF_XTAL_ACCURACY_50_PPM  (5) /**< 50 ppm */
#define NRF_CLOCK_LF_XTAL_ACCURACY_30_PPM  (6) /**< 30 ppm */
#define NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM  (7) /**< 20 ppm */

/** @} */

/**@defgroup NRF_CLOCK_LF_SRC Possible lfclk oscillator sources
 * @{ */

#define NRF_CLOCK_LF_SRC_RC      (0)                        /**< LFCLK RC oscillator. */
#define NRF_CLOCK_LF_SRC_XTAL    (1)                        /**< LFCLK crystal oscillator. */
#define NRF_CLOCK_LF_SRC_SYNTH   (2)                        /**< LFCLK Synthesized from HFCLK. */

/** @} */

/** @} */

/** @addtogroup NRF_SDM_TYPES Types
 * @{ */

/**@brief Type representing lfclk oscillator source. */
typedef struct
{
  uint8_t source;        /**< LF oscillator clock source, see @ref NRF_CLOCK_LF_SRC. */
  uint8_t rc_ctiv;      /**< Only for NRF_CLOCK_LF_SRC_RC: Calibration timer interval in 1/4 second
                              units (nRF51: 1-64, nRF52: 1-32).
                              @note To avoid excessive clock drift, 0.5 degrees Celsius is the
                                    maximum temperature change allowed in one calibration timer
                                    interval. The interval should be selected to ensure this.

                              @note Must be 0 if source is not NRF_CLOCK_LF_SRC_RC.  */
  uint8_t rc_temp_ctiv; /**<  Only for NRF_CLOCK_LF_SRC_RC: How often (in number of calibration
                              intervals) the RC oscillator shall be calibrated if the temperature
                              hasn't changed.
                                  0: Always calibrate even if the temperature hasn't changed.
                                  1: Only calibrate if the temperature has changed (nRF51 only).
                                  2-33: Check the temperature and only calibrate if it has changed,
                                        however calibration will take place every rc_temp_ctiv
                                        intervals in any case.

                              @note Must be 0 if source is not NRF_CLOCK_LF_SRC_RC.

                              @note For nRF52, the application must ensure calibration at least once
                                    every 8 seconds to ensure +/-250ppm clock stability. The
                                    recommended configuration for NRF_CLOCK_LF_SRC_RC on nRF52 is
                                    rc_ctiv=16 and rc_temp_ctiv=2. This will ensure calibration at
                                    least once every 8 seconds and for temperature changes of 0.5
                                    degrees Celsius every 4 seconds. See the Product Specification
                                    for the nRF52 device being used for more information.*/
  uint8_t xtal_accuracy; /**< External crystal clock accuracy used in the LL to compute timing windows.

                              @note For the NRF_CLOCK_LF_SRC_RC clock source this parameter is ignored. */
} nrf_clock_lf_cfg_t;

/**@brief Fault Handler type.
 *
 * When certain unrecoverable errors occur within the application or SoftDevice the fault handler will be called back.
 * The protocol stack will be in an undefined state when this happens and the only way to recover will be to
 * perform a reset, using e.g. CMSIS NVIC_SystemReset().
 * If the application returns from the fault handler the SoftDevice will call NVIC_SystemReset().
 *
 * @note This callback is executed in HardFault context, thus SVC functions cannot be called from the fault callback.
 *
 * @param[in] id Fault identifier. See @ref NRF_FAULT_IDS.
 * @param[in] pc The program counter of the instruction that triggered the fault.
 * @param[in] info Optional additional information regarding the fault. Refer to each Fault identifier for details.
 *
 * @note When id is set to NRF_FAULT_ID_APP_MEMACC, pc will contain the address of the instruction being executed at the time when
 * the fault is detected by the CPU. The CPU program counter may have advanced up to 2 instructions (no branching) after the one that triggered the fault.
 */
typedef void (*nrf_fault_handler_t)(uint32_t id, uint32_t pc, uint32_t info);

/** @} */

/** @addtogroup NRF_SDM_FUNCTIONS Functions
 * @{ */

/**@brief Enables the SoftDevice and by extension the protocol stack.
 *
 * @note Some care must be taken if a low frequency clock source is already running when calling this function:
 *       If the LF clock has a different source then the one currently running, it will be stopped. Then, the new
 *       clock source will be started.
 *
 * @note This function has no effect when returning with an error.
 *
 * @post If return code is ::NRF_SUCCESS
 *       - SoC library and protocol stack APIs are made available.
 *       - A portion of RAM will be unavailable (see relevant SDS documentation).
 *       - Some peripherals will be unavailable or available only through the SoC API (see relevant SDS documentation).
 *       - Interrupts will not arrive from protected peripherals or interrupts.
 *       - nrf_nvic_ functions must be used instead of CMSIS NVIC_ functions for reliable usage of the SoftDevice.
 *       - Interrupt latency may be affected by the SoftDevice  (see relevant SDS documentation).
 *       - Chosen low frequency clock source will be running.
 *
 * @param p_clock_lf_cfg Low frequency clock source and accuracy.
                         If NULL the clock will be configured as an rc source with rc_ctiv = 16 and .rc_temp_ctiv = 2
                         In the case of XTAL source, the PPM accuracy of the chosen clock source must be greater than or equal to the actual characteristics of your XTAL clock.
 * @param fault_handler Callback to be invoked in case of fault, cannot be NULL.
 *
 * @retval ::NRF_SUCCESS
 * @retval ::NRF_ERROR_INVALID_ADDR  Invalid or NULL pointer supplied.
 * @retval ::NRF_ERROR_INVALID_STATE SoftDevice is already enabled, and the clock source and fault handler cannot be updated.
 * @retval ::NRF_ERROR_SDM_INCORRECT_INTERRUPT_CONFIGURATION SoftDevice interrupt is already enabled, or an enabled interrupt has an illegal priority level.
 * @retval ::NRF_ERROR_SDM_LFCLK_SOURCE_UNKNOWN Unknown low frequency clock source selected.
 */
SVCALL(SD_SOFTDEVICE_ENABLE, uint32_t, sd_softdevice_enable(nrf_clock_lf_cfg_t const *p_clock_lf_cfg,
                                                            nrf_fault_handler_t fault_handler));


/**@brief Disables the SoftDevice and by extension the protocol stack.
 *
 * Idempotent function to disable the SoftDevice.
 *
 * @post SoC library and protocol stack APIs are made unavailable.
 * @post All interrupts that was protected by the SoftDevice will be disabled and initialized to priority 0 (highest).
 * @post All peripherals used by the SoftDevice will be reset to default values.
 * @post All of RAM become available.
 * @post All interrupts are forwarded to the application.
 * @post LFCLK source chosen in ::sd_softdevice_enable will be left running.
 *
 * @retval ::NRF_SUCCESS
 */
SVCALL(SD_SOFTDEVICE_DISABLE, uint32_t, sd_softdevice_disable(void));

/**@brief Check if the SoftDevice is enabled.
 *
 * @param[out]  p_softdevice_enabled If the SoftDevice is enabled: 1 else 0.
 *
 * @retval ::NRF_SUCCESS
 */
SVCALL(SD_SOFTDEVICE_IS_ENABLED, uint32_t, sd_softdevice_is_enabled(uint8_t *p_softdevice_enabled));

/**@brief Sets the base address of the interrupt vector table for interrupts forwarded from the SoftDevice
 *
 * This function is only intended to be called when a bootloader is enabled.
 *
 * @param[in] address The base address of the interrupt vector table for forwarded interrupts.

 * @retval ::NRF_SUCCESS
 */
SVCALL(SD_SOFTDEVICE_VECTOR_TABLE_BASE_SET, uint32_t, sd_softdevice_vector_table_base_set(uint32_t address));

/** @} */

#ifdef __cplusplus
}
#endif
#endif // NRF_SDM_H__

/**
  @}
 */
