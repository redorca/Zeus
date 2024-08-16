/* Copyright (c) 2008 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 */

/** @file
 * @brief Common defines and macros for firmware developed by Nordic Semiconductor.
 */

#ifndef NORDIC_COMMON_H__
#define NORDIC_COMMON_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Check if selected module is enabled
 *
 * This is save function for driver enable checking.
 * Correct from Lint point of view (not using default of undefined value).
 *
 * Usage:
 * @code
#   if NRF_MODULE_ENABLED(UART)
   ...
#   endif
 * @endcode
 *
 * @param module The module name.
 *
 * @retval 1 The macro <module>_ENABLE is defined and is non-zero.
 * @retval 0 The macro <module>_ENABLE is not defined or it equals zero.
 *
 * @note
 * This macro intentionally does not implement second expansion level.
 * The name of the module to be checked has to be given directly as a parameter.
 * And given parameter would be connected with @c _ENABLED postfix directly
 * without evaluating its value.
 */
//lint -e491 // Suppers warning 491 "non-standard use of 'defined' preprocessor operator"
#define NRF_MODULE_ENABLED(module) \
    ((defined(module ## _ENABLED) && (module ## _ENABLED)) ? 1 : 0)

/** The upper 8 bits of a 32 bit value */
//lint -emacro(572,MSB) // Suppress warning 572 "Excessive shift value"
#define MSB_32(a) (((a) & 0xFF000000) >> 24)
/** The lower 8 bits (of a 32 bit value) */
#define LSB_32(a) ((a) & 0x000000FF)

/** The upper 8 bits of a 16 bit value */
//lint -emacro(572,MSB_16) // Suppress warning 572 "Excessive shift value"
#define MSB_16(a) (((a) & 0xFF00) >> 8)
/** The lower 8 bits (of a 16 bit value) */
#define LSB_16(a) ((a) & 0x00FF)

/** Leaves the minimum of the two 32-bit arguments */
/*lint -emacro(506, MIN) */ /* Suppress "Constant value Boolean */
#define MIN(a, b) ((a) < (b) ? (a) : (b))
/** Leaves the maximum of the two 32-bit arguments */
/*lint -emacro(506, MAX) */ /* Suppress "Constant value Boolean */
#define MAX(a, b) ((a) < (b) ? (b) : (a))

/**@brief Concatenates two parameters.
 *
 * It realizes two level expansion to make it sure that all the parameters
 * are actually expanded before gluing them together.
 *
 * @param p1 First parameter to concatenating
 * @param p2 Second parameter to concatenating
 *
 * @return Two parameters glued together.
 *         They have to create correct C mnemonic in other case
 *         preprocessor error would be generated.
 *
 * @sa CONCAT_3
 */
#define CONCAT_2(p1, p2)      CONCAT_2_(p1, p2)
/** Auxiliary macro used by @ref CONCAT_2 */
#define CONCAT_2_(p1, p2)     p1##p2

/**@brief Concatenates three parameters.
 *
 * It realizes two level expansion to make it sure that all the parameters
 * are actually expanded before gluing them together.
 *
 * @param p1 First parameter to concatenating
 * @param p2 Second parameter to concatenating
 * @param p3 Third parameter to concatenating
 *
 * @return Three parameters glued together.
 *         They have to create correct C mnemonic in other case
 *         preprocessor error would be generated.
 *
 * @sa CONCAT_2
 */
#define CONCAT_3(p1, p2, p3)  CONCAT_3_(p1, p2, p3)
/** Auxiliary macro used by @ref CONCAT_3 */
#define CONCAT_3_(p1, p2, p3) p1##p2##p3

#define NUM_TO_STR_INTERNAL(val) #val
/** Converts numeric value to string.
 */
#define NUM_TO_STR(val) NUM_TO_STR_INTERNAL(val)

/** Counts number of elements inside the array
 */
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

/**@brief Set a bit in the uint32 word.
 *
 * @param[in] W  Word whose bit is being set.
 * @param[in] B  Bit number in the word to be set.
 */
#define SET_BIT(W,B)  ((W) |= (uint32_t)(1U << (B)))


/**@brief Clears a bit in the uint32 word.
 *
 * @param[in] W   Word whose bit is to be cleared.
 * @param[in] B   Bit number in the word to be cleared.
 */
#define CLR_BIT(W, B) ((W) &= (~((uint32_t)1 << (B))))


/**@brief Checks if a bit is set.
 *
 * @param[in] W   Word whose bit is to be checked.
 * @param[in] B   Bit number in the word to be checked.
 *
 * @retval 1 if bit is set.
 * @retval 0 if bit is not set.
 */



#ifdef __cplusplus
}
#endif

#endif // NORDIC_COMMON_H__
