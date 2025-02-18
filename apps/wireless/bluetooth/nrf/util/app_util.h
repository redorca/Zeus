/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup app_util Utility Functions and Definitions
 * @{
 * @ingroup app_common
 *
 * @brief Various types and definitions available to all applications.
 */

#ifndef APP_UTIL_H__
#define APP_UTIL_H__

#include <stdint.h>
#include <stdbool.h>
#include "compiler_abstraction.h"
#include "nrf.h"

#ifdef __cplusplus
extern "C" {
#endif

enum
{
  UNIT_0_625_MS = 625,                                /**< Number of microseconds in 0.625 milliseconds. */
  UNIT_1_25_MS  = 1250,                               /**< Number of microseconds in 1.25 milliseconds. */
  UNIT_10_MS    = 10000                               /**< Number of microseconds in 10 milliseconds. */
};


/**@brief Implementation specific macro for delayed macro expansion used in string concatenation
 *
 * @param[in]   lhs   Left hand side in concatenation
 * @param[in]   rhs   Right hand side in concatenation
 */
#define STRING_CONCATENATE_IMPL(lhs, rhs) lhs ## rhs


/**@brief Macro used to concatenate string using delayed macro expansion
 *
 * @note This macro will delay concatenation until the expressions have been resolved
 *
 * @param[in]   lhs   Left hand side in concatenation
 * @param[in]   rhs   Right hand side in concatenation
 */
#define STRING_CONCATENATE(lhs, rhs) STRING_CONCATENATE_IMPL(lhs, rhs)


// Disable lint-warnings/errors for STATIC_ASSERT_MSG
//lint --emacro(10, STATIC_ASSERT_MSG)
//lint --emacro(18, STATIC_ASSERT_MSG)
//lint --emacro(19, STATIC_ASSERT_MSG)
//lint --emacro(30, STATIC_ASSERT_MSG)
//lint --emacro(37, STATIC_ASSERT_MSG)
//lint --emacro(42, STATIC_ASSERT_MSG)
//lint --emacro(26, STATIC_ASSERT_MSG)
//lint --emacro(102,STATIC_ASSERT_MSG)
//lint --emacro(533,STATIC_ASSERT_MSG)
//lint --emacro(534,STATIC_ASSERT_MSG)
//lint --emacro(132,STATIC_ASSERT_MSG)
//lint --emacro(414,STATIC_ASSERT_MSG)
//lint --emacro(578,STATIC_ASSERT_MSG)
//lint --emacro(628,STATIC_ASSERT_MSG)
//lint --emacro(648,STATIC_ASSERT_MSG)
//lint --emacro(830,STATIC_ASSERT_MSG)


/**@brief Macro for doing static (i.e. compile time) assertion.
 *
 * @note If the EXPR isn't resolvable, then the error message won't be shown.
 *
 * @note The output of STATIC_ASSERT_MSG will be different across different compilers.
 *
 * @param[in] EXPR Constant expression to be verified.
 * @param[in] MSG  Name of the static assert.
 */
#if defined(__COUNTER__)

#define STATIC_ASSERT_MSG(EXPR, MSG) \
        ;enum { STRING_CONCATENATE(MSG, __COUNTER__) = 1 / (!!(EXPR)) }

#else

#define STATIC_ASSERT_MSG(EXPR, MSG) \
        ;enum { STRING_CONCATENATE(MSG, __LINE__) = 1 / (!!(EXPR)) }

#endif


/**@brief Macro for doing static (i.e. compile time) assertion.
 *
 * @note If the EXPR isn't resolvable, then the error message won't be shown.
 *
 * @note The output of STATIC_ASSERT will be different across different compilers.
 *
 * @param[in] EXPR Constant expression to be verified.
 */
#define STATIC_ASSERT(EXPR) STATIC_ASSERT_MSG((EXPR), static_assert_)


/**@brief Implementation details for NUM_VAR_ARGS */
#define NUM_VA_ARGS_IMPL(                              \
    _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10,       \
    _11, _12, _13, _14, _15, _16, _17, _18, _19, _20,  \
    _21, _22, _23, _24, _25, _26, _27, _28, _29, _30,  \
    _31, _32, _33, _34, _35, _36, _37, _38, _39, _40,  \
    _41, _42, _43, _44, _45, _46, _47, _48, _49, _50,  \
    _51, _52, _53, _54, _55, _56, _57, _58, _59, _60,  \
    _61, _62, N, ...) N


/**@brief Macro to get the number of arguments in a call variadic macro call
 *
 * param[in]    ...     List of arguments
 *
 * @retval  Number of variadic arguments in the argument list
 */
#define NUM_VA_ARGS(...) NUM_VA_ARGS_IMPL(__VA_ARGS__, 63, 62, 61,  \
    60, 59, 58, 57, 56, 55, 54, 53, 52, 51,                         \
    50, 49, 48, 47, 46, 45, 44, 43, 42, 41,                         \
    40, 39, 38, 37, 36, 35, 34, 33, 32, 31,                         \
    30, 29, 28, 27, 26, 25, 24, 23, 22, 21,                         \
    20, 19, 18, 17, 16, 15, 14, 13, 12, 11,                         \
    10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0)

/**@brief Implementation details for NUM_VAR_ARGS */
#define NUM_VA_ARGS_LESS_1_IMPL(                       \
    _ignored,                                          \
    _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10,       \
    _11, _12, _13, _14, _15, _16, _17, _18, _19, _20,  \
    _21, _22, _23, _24, _25, _26, _27, _28, _29, _30,  \
    _31, _32, _33, _34, _35, _36, _37, _38, _39, _40,  \
    _41, _42, _43, _44, _45, _46, _47, _48, _49, _50,  \
    _51, _52, _53, _54, _55, _56, _57, _58, _59, _60,  \
    _61, _62, N, ...) N

/**@brief Macro to get the number of arguments in a call variadic macro call.
 * First argument is not counted.
 *
 * param[in]    ...     List of arguments
 *
 * @retval  Number of variadic arguments in the argument list
 */
#define NUM_VA_ARGS_LESS_1(...) NUM_VA_ARGS_LESS_1_IMPL(__VA_ARGS__, 63, 62, 61,  \
    60, 59, 58, 57, 56, 55, 54, 53, 52, 51,                         \
    50, 49, 48, 47, 46, 45, 44, 43, 42, 41,                         \
    40, 39, 38, 37, 36, 35, 34, 33, 32, 31,                         \
    30, 29, 28, 27, 26, 25, 24, 23, 22, 21,                         \
    20, 19, 18, 17, 16, 15, 14, 13, 12, 11,                         \
    10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, ~)


/**@brief type for holding an encoded (i.e. little endian) 16 bit unsigned integer. */
typedef uint8_t uint16_le_t[2];

/**@brief Type for holding an encoded (i.e. little endian) 32 bit unsigned integer. */
typedef uint8_t uint32_le_t[4];

/**@brief Byte array type. */
typedef struct
{
  uint16_t  size;                 /**< Number of array entries. */
  uint8_t *p_data;                /**< Pointer to array entries. */
} uint8_array_t;


/**@brief Macro for performing rounded integer division (as opposed to truncating the result).
 *
 * @param[in]   A   Numerator.
 * @param[in]   B   Denominator.
 *
 * @return      Rounded (integer) result of dividing A by B.
 */
#define ROUNDED_DIV(A, B) (((A) + ((B) / 2)) / (B))


/**@brief Macro for checking if an integer is a power of two.
 *
 * @param[in]   A   Number to be tested.
 *
 * @return      true if value is power of two.
 * @return      false if value not power of two.
 */
#define IS_POWER_OF_TWO(A) ( ((A) != 0) && ((((A) - 1) & (A)) == 0) )


/**@brief Macro for converting milliseconds to ticks.
 *
 * @param[in] TIME          Number of milliseconds to convert.
 * @param[in] RESOLUTION    Unit to be converted to in [us/ticks].
 */
#define MSEC_TO_UNITS(TIME, RESOLUTION) (((TIME) * 1000) / (RESOLUTION))


/**@brief Macro for performing integer division, making sure the result is rounded up.
 *
 * @details One typical use for this is to compute the number of objects with size B is needed to
 *          hold A number of bytes.
 *
 * @param[in]   A   Numerator.
 * @param[in]   B   Denominator.
 *
 * @return      Integer result of dividing A by B, rounded up.
 */
#define CEIL_DIV(A, B)      \
    (((A) + (B) - 1) / (B))


/**@brief Macro for creating a buffer aligned to 4 bytes.
 *
 * @param[in]   NAME        Name of the buffor.
 * @param[in]   MIN_SIZE    Size of this buffor (it will be rounded up to multiples of 4 bytes).
 */
#define WORD_ALIGNED_MEM_BUFF(NAME, MIN_SIZE) static uint32_t NAME[CEIL_DIV(MIN_SIZE, sizeof(uint32_t))]


/**@brief Macro for calculating the number of words that are needed to hold a number of bytes.
 *
 * @details Adds 3 and divides by 4.
 *
 * @param[in]  n_bytes  The number of bytes.
 *
 * @return The number of words that @p n_bytes take up (rounded up).
 */
#define BYTES_TO_WORDS(n_bytes) (((n_bytes) + 3) >> 2)


/**@brief The number of bytes in a word.
 */
#define BYTES_PER_WORD (4)


/**@brief Macro for increasing a number to the nearest (larger) multiple of another number.
 *
 * @param[in]  alignment  The number to align to.
 * @param[in]  number     The number to align (increase).
 *
 * @return The aligned (increased) @p number.
 */
#define ALIGN_NUM(alignment, number) ((number - 1) + alignment - ((number - 1) % alignment))

/**@brief Macro for getting first of 2 parameters.
 *
 * @param[in] a1    First parameter.
 * @param[in] a2    Second parameter.
 */
#define GET_ARG_1(a1, a2) a1

/**@brief Macro for getting second of 2 parameters.
 *
 * @param[in] a1    First parameter.
 * @param[in] a2    Second parameter.
 */
#define GET_ARG_2(a1, a2) a2


/**@brief Container of macro (borrowed from Linux kernel).
 *
 * This macro returns parent structure address basing on child member address.
 *
 * @param ptr       Address of child type.
 * @param type      Type of parent structure.
 * @param member    Name of child field in parent structure.
 *
 * @return Parent structure address.
 * */
#define CONTAINER_OF(ptr, type, member)                 \
        (type *)((char *)ptr - offsetof(type, member))


/**
 * @brief Define Bit-field mask
 *
 * Macro that defined the mask with selected number of bits set, starting from
 * provided bit number.
 *
 * @param[in] bcnt Number of bits in the bit-field
 * @param[in] boff Lowest bit number
 */
#define BF_MASK(bcnt, boff) ( ((1U << (bcnt)) - 1U) << (boff) )

/**
 * @brief Get bit-field
 *
 * Macro that extracts selected bit-field from provided value
 *
 * @param[in] val  Value from witch selected bit-field would be extracted
 * @param[in] bcnt Number of bits in the bit-field
 * @param[in] boff Lowest bit number
 *
 * @return Value of the selected bits
 */
#define BF_GET(val, bcnt, boff) ( ( (val) & BF_MASK((bcnt), (boff)) ) >> (boff) )

/**
 * @brief Create bit-field value
 *
 * Value is masked and shifted to match given bit-field
 *
 * @param[in] val  Value to set on bit-field
 * @param[in] bcnt Number of bits for bit-field
 * @param[in] boff Offset of bit-field
 *
 * @return Value positioned of given bit-field.
 */
#define BF_VAL(val, bcnt, boff) ( (((uint32_t)(val)) << (boff)) & BF_MASK(bcnt, boff) )

/**
 * @name Configuration of complex bit-field
 *
 * @sa BF_CX
 * @{
 */
/** @brief Position of bit count in complex bit-field value */
#define BF_CX_BCNT_POS  0U
/** @brief Mask of bit count in complex bit-field value */
#define BF_CX_BCNT_MASK (0xffU << BF_CX_BCNT_POS)
/** @brief Position of bit position in complex bit-field value */
#define BF_CX_BOFF_POS  8U
/** @brief Mask of bit position in complex bit-field value */
#define BF_CX_BOFF_MASK (0xffU << BF_CX_BOFF_POS)
/** @} */

/**
 * @brief Define complex bit-field
 *
 * Complex bit-field would contain its position and size in one number.
 * @sa BF_CX_MASK
 * @sa BF_CX_POS
 * @sa BF_CX_GET
 *
 * @param[in] bcnt Number of bits in the bit-field
 * @param[in] boff Lowest bit number
 *
 * @return The single number that describes the bit-field completely.
 */
#define BF_CX(bcnt, boff) ( ((((uint32_t)(bcnt)) << BF_CX_BCNT_POS) & BF_CX_BCNT_MASK) | ((((uint32_t)(boff)) << BF_CX_BOFF_POS) & BF_CX_BOFF_MASK) )

/**
 * @brief Get number of bits in bit-field
 *
 * @sa BF_CX
 *
 * @param bf_cx Complex bit-field
 *
 * @return Number of bits in given bit-field
 */
#define BF_CX_BCNT(bf_cx) ( ((bf_cx) & BF_CX_BCNT_MASK) >> BF_CX_BCNT_POS )

/**
 * @brief Get lowest bit number in the field
 *
 * @sa BF_CX
 *
 * @param[in] bf_cx Complex bit-field
 *
 * @return Lowest bit number in given bit-field
 */
#define BF_CX_BOFF(bf_cx) ( ((bf_cx) & BF_CX_BOFF_MASK) >> BF_CX_BOFF_POS )

/**
 * @brief Get bit mask of the selected field
 *
 * @sa BF_CX
 *
 * @param[in] bf_cx Complex bit-field
 *
 * @return Mask of given bit-field
 */
#define BF_CX_MASK(bf_cx) BF_MASK(BF_CX_BCNT(bf_cx), BF_CX_BOFF(bf_cx))

/**
 * @brief Get bit-field
 *
 * Macro that extracts selected bit-field from provided value.
 * Bit-field is given as a complex value.
 *
 * @sa BF_CX
 * @sa BF_GET
 *
 * @param[in] val   Value from witch selected bit-field would be extracted
 * @param[in] bf_cx Complex bit-field
 *
 * @return Value of the selected bits.
 */
#define BF_CX_GET(val, bf_cx) BF_GET(val, BF_CX_BCNT(bf_cx), BF_CX_BOFF(bf_cx))

/**
 * @brief Create bit-field value
 *
 * Value is masked and shifted to match given bit-field.
 *
 * @param[in] val  Value to set on bit-field
 * @param[in] bf_cx Complex bit-field
 *
 * @return Value positioned of given bit-field.
 */
#define BF_CX_VAL(val, bf_cx) BF_VAL(val, BF_CX_BCNT(bf_cx), BF_CX_BOFF(bf_cx))

/**
 * @brief Extracting data from the brackets
 *
 * This macro get rid of brackets around the argument.
 * It can be used to pass multiple arguments in logical one argument to a macro.
 * Call it with arguments inside brackets:
 * @code
 * #define ARGUMENTS (a, b, c)
 * BRACKET_EXTRACT(ARGUMENTS)
 * @endcode
 * It would produce:
 * @code
 * a, b, c
 * @endcode
 *
 * @param a Argument with anything inside brackets
 * @return Anything that appears inside the brackets of the argument
 *
 * @note
 * The argument of the macro have to be inside brackets.
 * In other case the compilation would fail.
 */
#define BRACKET_EXTRACT(a)  BRACKET_EXTRACT_(a)
#define BRACKET_EXTRACT_(a) BRACKET_EXTRACT__ a
#define BRACKET_EXTRACT__(...) __VA_ARGS__


/**
 * @brief Check if number of parameters is more than 1
 *
 * @param ... Arguments to count
 *
 * @return 0 If argument count is <= 1
 * @return 1 If argument count is > 1
 *
 * @sa NUM_VA_ARGS
 * @sa NUM_IS_MORE_THAN_1
 */
#define NUM_VA_ARGS_IS_MORE_THAN_1(...) NUM_IS_MORE_THAN_1(NUM_VA_ARGS(__VA_ARGS__))

/**
 * @brief Check if given numeric value is bigger than 1
 *
 * This macro accepts numeric value, that may be the result of argument expansion.
 * This numeric value is then converted to 0 if it is lover than 1 or to 1 if
 * its value is higher than 1.
 * The generated result can be used to glue it into other macro mnemonic name.
 *
 * @param N Numeric value to check
 *
 * @return 0 If argument is <= 1
 * @return 1 If argument is > 1
 *
 * @note Any existing definition of a form NUM_IS_MORE_THAN_1_PROBE_[N] can
 *       broke the result of this macro
 */
#define NUM_IS_MORE_THAN_1(N) NUM_IS_MORE_THAN_1_(N)
#define NUM_IS_MORE_THAN_1_(N)  NUM_IS_MORE_THAN_1_PROBE_(NUM_IS_MORE_THAN_1_PROBE_ ## N, 1)
#define NUM_IS_MORE_THAN_1_PROBE_(...) GET_VA_ARG_1(GET_ARGS_AFTER_1(__VA_ARGS__))
#define NUM_IS_MORE_THAN_1_PROBE_0 ~, 0
#define NUM_IS_MORE_THAN_1_PROBE_1 ~, 0

/**
 * @brief Get the first argument
 *
 * @param ... Arguments to select
 *
 * @return First argument or empty if no arguments are provided
 */
#define GET_VA_ARG_1(...) GET_VA_ARG_1_(__VA_ARGS__, ) // Make sure that also for 1 argument it works
#define GET_VA_ARG_1_(a1, ...) a1

/**
 * @brief Get all the arguments but the first one
 *
 * @param ... Arguments to select
 *
 * @return All arguments after the first one or empty if less than 2 arguments are provided
 */
#define GET_ARGS_AFTER_1(...) GET_ARGS_AFTER_1_(__VA_ARGS__, ) // Make sure that also for 1 argument it works
#define GET_ARGS_AFTER_1_(a1, ...) __VA_ARGS__

/**
 * @brief Size of a field in declared structure
 *
 * Macro that returns the size of the structure field.
 * @param struct_type Variable type to get the field size from
 * @param field Field name to analyze. It can be even field inside field (field.somethingelse.and_another).
 *
 * @return Size of the field
 */
#define FIELD_SIZE(struct_type, field) sizeof(((struct struct_type*)NULL)->field)

/**
 * @brief Number of elements in field array in declared structure
 *
 * Macro that returns number of elementy in structure field.
 * @param struct_type Variable type to get the field size from
 * @param field Field name to analyze.
 *
 * @return Number of elements in field array
 *
 * @sa FIELD_SIZE
 */
#define FIELD_ARRAY_SIZE(struct_type, field) (FIELD_SIZE(struct_type, field) / FIELD_SIZE(struct_type, field[0]))

/**
 * @brief Mapping macro
 *
 * Macro that process all arguments using given macro
 *
 * @param ... Macro name to be used for argument processing followed by arguments to process.
 *            Macro should have following form: MACRO(argument)
 *
 * @return All arguments processed by given macro
 */
#define MACRO_MAP(...) MACRO_MAP_(__VA_ARGS__)
#define MACRO_MAP_(...) MACRO_MAP_N(NUM_VA_ARGS_LESS_1(__VA_ARGS__), __VA_ARGS__) // To make sure it works also for 2 arguments in total

/**
 * @brief Mapping macro, recursive version
 *
 *  Can be used in @ref MACRO_MAP macro
 */
#define MACRO_MAP_REC(...) MACRO_MAP_REC_(__VA_ARGS__)
#define MACRO_MAP_REC_(...) MACRO_MAP_REC_N(NUM_VA_ARGS_LESS_1(__VA_ARGS__), __VA_ARGS__) // To make sure it works also for 2 arguments in total
/**
 * @brief Mapping N arguments macro
 *
 * Macro similar to @ref MACRO_MAP but maps exact number of arguments.
 * If there is more arguments given, the rest would be ignored.
 *
 * @param N   Number of arguments to map
 * @param ... Macro name to be used for argument processing followed by arguments to process.
 *            Macro should have following form: MACRO(argument)
 *
 * @return Selected number of arguments processed by given macro
 */
#define MACRO_MAP_N(N, ...) MACRO_MAP_N_(N, __VA_ARGS__)
#define MACRO_MAP_N_(N, ...) CONCAT_2(MACRO_MAP_, N)(__VA_ARGS__, )

/**
 * @brief Mapping N arguments macro, recursive version
 *
 *  Can be used in @ref MACRO_MAP_N macro
 */
#define MACRO_MAP_REC_N(N, ...) MACRO_MAP_REC_N_(N, __VA_ARGS__)
#define MACRO_MAP_REC_N_(N, ...) CONCAT_2(MACRO_MAP_REC_, N)(__VA_ARGS__, )

#define MACRO_MAP_0(           ...)
#define MACRO_MAP_1( macro, a, ...) macro(a)
#define MACRO_MAP_2( macro, a, ...) macro(a) MACRO_MAP_1 (macro, __VA_ARGS__, )
#define MACRO_MAP_3( macro, a, ...) macro(a) MACRO_MAP_2 (macro, __VA_ARGS__, )
#define MACRO_MAP_4( macro, a, ...) macro(a) MACRO_MAP_3 (macro, __VA_ARGS__, )
#define MACRO_MAP_5( macro, a, ...) macro(a) MACRO_MAP_4 (macro, __VA_ARGS__, )
#define MACRO_MAP_6( macro, a, ...) macro(a) MACRO_MAP_5 (macro, __VA_ARGS__, )
#define MACRO_MAP_7( macro, a, ...) macro(a) MACRO_MAP_6 (macro, __VA_ARGS__, )
#define MACRO_MAP_8( macro, a, ...) macro(a) MACRO_MAP_7 (macro, __VA_ARGS__, )
#define MACRO_MAP_9( macro, a, ...) macro(a) MACRO_MAP_8 (macro, __VA_ARGS__, )
#define MACRO_MAP_10(macro, a, ...) macro(a) MACRO_MAP_9 (macro, __VA_ARGS__, )
#define MACRO_MAP_11(macro, a, ...) macro(a) MACRO_MAP_10(macro, __VA_ARGS__, )
#define MACRO_MAP_12(macro, a, ...) macro(a) MACRO_MAP_11(macro, __VA_ARGS__, )
#define MACRO_MAP_13(macro, a, ...) macro(a) MACRO_MAP_12(macro, __VA_ARGS__, )
#define MACRO_MAP_14(macro, a, ...) macro(a) MACRO_MAP_13(macro, __VA_ARGS__, )
#define MACRO_MAP_15(macro, a, ...) macro(a) MACRO_MAP_14(macro, __VA_ARGS__, )

#define MACRO_MAP_REC_0(           ...)
#define MACRO_MAP_REC_1( macro, a, ...) macro(a)
#define MACRO_MAP_REC_2( macro, a, ...) macro(a) MACRO_MAP_REC_1 (macro, __VA_ARGS__, )
#define MACRO_MAP_REC_3( macro, a, ...) macro(a) MACRO_MAP_REC_2 (macro, __VA_ARGS__, )
#define MACRO_MAP_REC_4( macro, a, ...) macro(a) MACRO_MAP_REC_3 (macro, __VA_ARGS__, )
#define MACRO_MAP_REC_5( macro, a, ...) macro(a) MACRO_MAP_REC_4 (macro, __VA_ARGS__, )
#define MACRO_MAP_REC_6( macro, a, ...) macro(a) MACRO_MAP_REC_5 (macro, __VA_ARGS__, )
#define MACRO_MAP_REC_7( macro, a, ...) macro(a) MACRO_MAP_REC_6 (macro, __VA_ARGS__, )
#define MACRO_MAP_REC_8( macro, a, ...) macro(a) MACRO_MAP_REC_7 (macro, __VA_ARGS__, )
#define MACRO_MAP_REC_9( macro, a, ...) macro(a) MACRO_MAP_REC_8 (macro, __VA_ARGS__, )
#define MACRO_MAP_REC_10(macro, a, ...) macro(a) MACRO_MAP_REC_9 (macro, __VA_ARGS__, )
#define MACRO_MAP_REC_11(macro, a, ...) macro(a) MACRO_MAP_REC_10(macro, __VA_ARGS__, )
#define MACRO_MAP_REC_12(macro, a, ...) macro(a) MACRO_MAP_REC_11(macro, __VA_ARGS__, )
#define MACRO_MAP_REC_13(macro, a, ...) macro(a) MACRO_MAP_REC_12(macro, __VA_ARGS__, )
#define MACRO_MAP_REC_14(macro, a, ...) macro(a) MACRO_MAP_REC_13(macro, __VA_ARGS__, )
#define MACRO_MAP_REC_15(macro, a, ...) macro(a) MACRO_MAP_REC_14(macro, __VA_ARGS__, )

/**
 * @brief Mapping macro with current index
 *
 * Basically macro similar to @ref MACRO_MAP, but the processing function would get an argument
 * and current argument index (beginning from 0).
 *
 * @param ... Macro name to be used for argument processing followed by arguments to process.
 *            Macro should have following form: MACRO(argument, index)
 * @return All arguments processed by given macro
 */
#define MACRO_MAP_FOR(...) MACRO_MAP_FOR_(__VA_ARGS__)
#define MACRO_MAP_FOR_N_LIST 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15
#define MACRO_MAP_FOR_(...) MACRO_MAP_FOR_N(NUM_VA_ARGS_LESS_1(__VA_ARGS__), __VA_ARGS__)

/**
 * @brief Mapping N arguments macro with current index
 *
 * Macro is similar to @ref MACRO_MAP_FOR but maps exact number of arguments.
 * If there is more arguments given, the rest would be ignored.
 *
 * @param N   Number of arguments to map
 * @param ... Macro name to be used for argument processing followed by arguments to process.
 *            Macro should have following form: MACRO(argument, index)
 *
 * @return Selected number of arguments processed by given macro
 */
#define MACRO_MAP_FOR_N(N, ...) MACRO_MAP_FOR_N_(N, __VA_ARGS__)
#define MACRO_MAP_FOR_N_(N, ...) CONCAT_2(MACRO_MAP_FOR_, N)((MACRO_MAP_FOR_N_LIST), __VA_ARGS__, )

#define MACRO_MAP_FOR_0( n_list,           ...)
#define MACRO_MAP_FOR_1( n_list, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list)))
#define MACRO_MAP_FOR_2( n_list, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list))) MACRO_MAP_FOR_1 ((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__, )
#define MACRO_MAP_FOR_3( n_list, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list))) MACRO_MAP_FOR_2 ((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__, )
#define MACRO_MAP_FOR_4( n_list, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list))) MACRO_MAP_FOR_3 ((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__, )
#define MACRO_MAP_FOR_5( n_list, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list))) MACRO_MAP_FOR_4 ((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__, )
#define MACRO_MAP_FOR_6( n_list, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list))) MACRO_MAP_FOR_5 ((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__, )
#define MACRO_MAP_FOR_7( n_list, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list))) MACRO_MAP_FOR_6 ((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__, )
#define MACRO_MAP_FOR_8( n_list, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list))) MACRO_MAP_FOR_7 ((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__, )
#define MACRO_MAP_FOR_9( n_list, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list))) MACRO_MAP_FOR_8 ((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__, )
#define MACRO_MAP_FOR_10(n_list, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list))) MACRO_MAP_FOR_9 ((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__, )
#define MACRO_MAP_FOR_11(n_list, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list))) MACRO_MAP_FOR_10((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__, )
#define MACRO_MAP_FOR_12(n_list, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list))) MACRO_MAP_FOR_11((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__, )
#define MACRO_MAP_FOR_13(n_list, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list))) MACRO_MAP_FOR_12((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__, )
#define MACRO_MAP_FOR_14(n_list, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list))) MACRO_MAP_FOR_13((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__, )
#define MACRO_MAP_FOR_15(n_list, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list))) MACRO_MAP_FOR_14((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), macro, __VA_ARGS__, )


/**
 * @brief Mapping macro with current index and parameter
 *
 * Version of @ref MACRO_MAP_FOR that passes also the same parameter to all macros.
 *
 * @param param Parameter that would be passed to each macro call during mapping.
 * @param ...   Macro name to be used for argument processing followed by arguments to process.
 *              Macro should have following form: MACRO(argument, index, param)
 *
 * @return All arguments processed by given macro
 */
#define MACRO_MAP_FOR_PARAM(param, ...) MACRO_MAP_FOR_PARAM_(param, __VA_ARGS__)
#define MACRO_MAP_FOR_PARAM_(param, ...) MACRO_MAP_FOR_PARAM_N(NUM_VA_ARGS_LESS_1(__VA_ARGS__), param, __VA_ARGS__)

/**
 * @brief Mapping N arguments macro with with current index and parameter
 *
 * @param N     Number of arguments to map
 * @param param Parameter that would be passed to each macro call during mapping.
 * @param ...   Macro name to be used for argument processing followed by arguments to process.
 *              Macro should have following form: MACRO(argument, index, param)
 *
 * @return All arguments processed by given macro
 */
#define MACRO_MAP_FOR_PARAM_N(N, param, ...) MACRO_MAP_FOR_PARAM_N_(N, param, __VA_ARGS__)
#define MACRO_MAP_FOR_PARAM_N_(N, param, ...) CONCAT_2(MACRO_MAP_FOR_PARAM_, N)((MACRO_MAP_FOR_N_LIST), param, __VA_ARGS__, )


#define MACRO_MAP_FOR_PARAM_0( n_list, param, ...)
#define MACRO_MAP_FOR_PARAM_1( n_list, param, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), param)
#define MACRO_MAP_FOR_PARAM_2( n_list, param, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), param) MACRO_MAP_FOR_PARAM_1 ((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), param, macro, __VA_ARGS__, )
#define MACRO_MAP_FOR_PARAM_3( n_list, param, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), param) MACRO_MAP_FOR_PARAM_2 ((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), param, macro, __VA_ARGS__, )
#define MACRO_MAP_FOR_PARAM_4( n_list, param, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), param) MACRO_MAP_FOR_PARAM_3 ((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), param, macro, __VA_ARGS__, )
#define MACRO_MAP_FOR_PARAM_5( n_list, param, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), param) MACRO_MAP_FOR_PARAM_4 ((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), param, macro, __VA_ARGS__, )
#define MACRO_MAP_FOR_PARAM_6( n_list, param, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), param) MACRO_MAP_FOR_PARAM_5 ((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), param, macro, __VA_ARGS__, )
#define MACRO_MAP_FOR_PARAM_7( n_list, param, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), param) MACRO_MAP_FOR_PARAM_6 ((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), param, macro, __VA_ARGS__, )
#define MACRO_MAP_FOR_PARAM_8( n_list, param, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), param) MACRO_MAP_FOR_PARAM_7 ((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), param, macro, __VA_ARGS__, )
#define MACRO_MAP_FOR_PARAM_9( n_list, param, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), param) MACRO_MAP_FOR_PARAM_8 ((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), param, macro, __VA_ARGS__, )
#define MACRO_MAP_FOR_PARAM_10(n_list, param, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), param) MACRO_MAP_FOR_PARAM_9 ((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), param, macro, __VA_ARGS__, )
#define MACRO_MAP_FOR_PARAM_11(n_list, param, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), param) MACRO_MAP_FOR_PARAM_10((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), param, macro, __VA_ARGS__, )
#define MACRO_MAP_FOR_PARAM_12(n_list, param, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), param) MACRO_MAP_FOR_PARAM_11((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), param, macro, __VA_ARGS__, )
#define MACRO_MAP_FOR_PARAM_13(n_list, param, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), param) MACRO_MAP_FOR_PARAM_12((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), param, macro, __VA_ARGS__, )
#define MACRO_MAP_FOR_PARAM_14(n_list, param, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), param) MACRO_MAP_FOR_PARAM_13((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), param, macro, __VA_ARGS__, )
#define MACRO_MAP_FOR_PARAM_15(n_list, param, macro, a, ...) macro(a, GET_VA_ARG_1(BRACKET_EXTRACT(n_list)), param) MACRO_MAP_FOR_PARAM_14((GET_ARGS_AFTER_1(BRACKET_EXTRACT(n_list))), param, macro, __VA_ARGS__, )


/**@brief Adding curly brace to macro parameter
 *
 * Useful in array of structures initialization.
 *
 * @param p parameter to put into the curly brace*/
#define PARAM_CBRACE(p) { p },


/**@brief Function for changing the value unit.
 *
 * @param[in]   value               Value to be rescaled.
 * @param[in]   old_unit_reversal   Reversal of the incoming unit.
 * @param[in]   new_unit_reversal   Reversal of the desired unit.
 *
 * @return      Number of bytes written.
 */
static __INLINE uint64_t value_rescale(uint32_t value, uint32_t old_unit_reversal, uint16_t new_unit_reversal)
{
  return (uint64_t)ROUNDED_DIV((uint64_t)value * new_unit_reversal, old_unit_reversal);
}

/**@brief Function for encoding a uint16 value.
 *
 * @param[in]   value            Value to be encoded.
 * @param[out]  p_encoded_data   Buffer where the encoded data is to be written.
 *
 * @return      Number of bytes written.
 */
static __INLINE uint8_t uint16_encode(uint16_t value, uint8_t *p_encoded_data)
{
  p_encoded_data[0] = (uint8_t) ((value & 0x00FF) >> 0);
  p_encoded_data[1] = (uint8_t) ((value & 0xFF00) >> 8);
  return sizeof(uint16_t);
}

/**@brief Function for encoding a three-byte value.
 *
 * @param[in]   value            Value to be encoded.
 * @param[out]  p_encoded_data   Buffer where the encoded data is to be written.
 *
 * @return      Number of bytes written.
 */
static __INLINE uint8_t uint24_encode(uint32_t value, uint8_t *p_encoded_data)
{
  p_encoded_data[0] = (uint8_t) ((value & 0x000000FF) >> 0);
  p_encoded_data[1] = (uint8_t) ((value & 0x0000FF00) >> 8);
  p_encoded_data[2] = (uint8_t) ((value & 0x00FF0000) >> 16);
  return 3;
}

/**@brief Function for encoding a uint32 value.
 *
 * @param[in]   value            Value to be encoded.
 * @param[out]  p_encoded_data   Buffer where the encoded data is to be written.
 *
 * @return      Number of bytes written.
 */
static __INLINE uint8_t uint32_encode(uint32_t value, uint8_t *p_encoded_data)
{
  p_encoded_data[0] = (uint8_t) ((value & 0x000000FF) >> 0);
  p_encoded_data[1] = (uint8_t) ((value & 0x0000FF00) >> 8);
  p_encoded_data[2] = (uint8_t) ((value & 0x00FF0000) >> 16);
  p_encoded_data[3] = (uint8_t) ((value & 0xFF000000) >> 24);
  return sizeof(uint32_t);
}

/**@brief Function for encoding a uint48 value.
 *
 * @param[in]   value            Value to be encoded.
 * @param[out]  p_encoded_data   Buffer where the encoded data is to be written.
 *
 * @return      Number of bytes written.
 */
static __INLINE uint8_t uint48_encode(uint64_t value, uint8_t *p_encoded_data)
{
  p_encoded_data[0] = (uint8_t) ((value & 0x0000000000FF) >> 0);
  p_encoded_data[1] = (uint8_t) ((value & 0x00000000FF00) >> 8);
  p_encoded_data[2] = (uint8_t) ((value & 0x000000FF0000) >> 16);
  p_encoded_data[3] = (uint8_t) ((value & 0x0000FF000000) >> 24);
  p_encoded_data[4] = (uint8_t) ((value & 0x00FF00000000) >> 32);
  p_encoded_data[5] = (uint8_t) ((value & 0xFF0000000000) >> 40);
  return 6;
}

/**@brief Function for decoding a uint16 value.
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 *
 * @return      Decoded value.
 */
static __INLINE uint16_t uint16_decode(const uint8_t *p_encoded_data)
{
  return ( (((uint16_t)((uint8_t *)p_encoded_data)[0])) |
           (((uint16_t)((uint8_t *)p_encoded_data)[1]) << 8 ));
}

/**@brief Function for decoding a uint16 value in big-endian format.
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 *
 * @return      Decoded value.
 */
static __INLINE uint16_t uint16_big_decode(const uint8_t *p_encoded_data)
{
  return ( (((uint16_t)((uint8_t *)p_encoded_data)[0]) << 8 ) |
           (((uint16_t)((uint8_t *)p_encoded_data)[1])) );
}

/**@brief Function for decoding a three-byte value.
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 *
 * @return      Decoded value (uint32_t).
 */
static __INLINE uint32_t uint24_decode(const uint8_t *p_encoded_data)
{
  return ( (((uint32_t)((uint8_t *)p_encoded_data)[0]) << 0)  |
           (((uint32_t)((uint8_t *)p_encoded_data)[1]) << 8)  |
           (((uint32_t)((uint8_t *)p_encoded_data)[2]) << 16));
}

/**@brief Function for decoding a uint32 value.
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 *
 * @return      Decoded value.
 */
static __INLINE uint32_t uint32_decode(const uint8_t *p_encoded_data)
{
  return ( (((uint32_t)((uint8_t *)p_encoded_data)[0]) << 0)  |
           (((uint32_t)((uint8_t *)p_encoded_data)[1]) << 8)  |
           (((uint32_t)((uint8_t *)p_encoded_data)[2]) << 16) |
           (((uint32_t)((uint8_t *)p_encoded_data)[3]) << 24 ));
}

/**@brief Function for decoding a uint32 value in big-endian format.
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 *
 * @return      Decoded value.
 */
static __INLINE uint32_t uint32_big_decode(const uint8_t *p_encoded_data)
{
  return ( (((uint32_t)((uint8_t *)p_encoded_data)[0]) << 24) |
           (((uint32_t)((uint8_t *)p_encoded_data)[1]) << 16) |
           (((uint32_t)((uint8_t *)p_encoded_data)[2]) << 8)  |
           (((uint32_t)((uint8_t *)p_encoded_data)[3]) << 0) );
}

/**
 * @brief Function for encoding an uint16 value in big-endian format.
 *
 * @param[in]   value            Value to be encoded.
 * @param[out]  p_encoded_data   Buffer where the encoded data will be written.
 *
 * @return      Number of bytes written.
 */
static __INLINE uint8_t uint16_big_encode(uint16_t value, uint8_t *p_encoded_data)
{
  p_encoded_data[0] = (uint8_t) (value >> 8);
  p_encoded_data[1] = (uint8_t) (value & 0xFF);

  return sizeof(uint16_t);
}

/**@brief Function for encoding a uint32 value in big-endian format.
 *
 * @param[in]   value            Value to be encoded.
 * @param[out]  p_encoded_data   Buffer where the encoded data will be written.
 *
 * @return      Number of bytes written.
 */

static __INLINE uint32_t __REV(uint32_t value)
{
#if (__GNUC__ > 4) || (__GNUC__ == 4 && __GNUC_MINOR__ >= 5)
  return __builtin_bswap32(value);
#else
  uint32_t result;

  __ASM volatile ("rev %0, %1" : __CMSIS_GCC_OUT_REG (result) : __CMSIS_GCC_USE_REG (value) );
  return (result);
#endif
}
static __INLINE uint8_t uint32_big_encode(uint32_t value, uint8_t *p_encoded_data)
{
  *(uint32_t *)p_encoded_data = __REV(value);
  return sizeof(uint32_t);
}

/**@brief Function for decoding a uint48 value.
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 *
 * @return      Decoded value. (uint64_t)
 */
static __INLINE uint64_t uint48_decode(const uint8_t *p_encoded_data)
{
  return ( (((uint64_t)((uint8_t *)p_encoded_data)[0]) << 0)  |
           (((uint64_t)((uint8_t *)p_encoded_data)[1]) << 8)  |
           (((uint64_t)((uint8_t *)p_encoded_data)[2]) << 16) |
           (((uint64_t)((uint8_t *)p_encoded_data)[3]) << 24) |
           (((uint64_t)((uint8_t *)p_encoded_data)[4]) << 32) |
           (((uint64_t)((uint8_t *)p_encoded_data)[5]) << 40 ));
}

/** @brief Function for converting the input voltage (in milli volts) into percentage of 3.0 Volts.
 *
 *  @details The calculation is based on a linearized version of the battery's discharge
 *           curve. 3.0V returns 100% battery level. The limit for power failure is 2.1V and
 *           is considered to be the lower boundary.
 *
 *           The discharge curve for CR2032 is non-linear. In this model it is split into
 *           4 linear sections:
 *           - Section 1: 3.0V - 2.9V = 100% - 42% (58% drop on 100 mV)
 *           - Section 2: 2.9V - 2.74V = 42% - 18% (24% drop on 160 mV)
 *           - Section 3: 2.74V - 2.44V = 18% - 6% (12% drop on 300 mV)
 *           - Section 4: 2.44V - 2.1V = 6% - 0% (6% drop on 340 mV)
 *
 *           These numbers are by no means accurate. Temperature and
 *           load in the actual application is not accounted for!
 *
 *  @param[in] mvolts The voltage in mV
 *
 *  @return    Battery level in percent.
 */
static __INLINE uint8_t battery_level_in_percent(const uint16_t mvolts)
{
  uint8_t battery_level;

  if (mvolts >= 3000)
    {
      battery_level = 100;
    }
  else if (mvolts > 2900)
    {
      battery_level = 100 - ((3000 - mvolts) * 58) / 100;
    }
  else if (mvolts > 2740)
    {
      battery_level = 42 - ((2900 - mvolts) * 24) / 160;
    }
  else if (mvolts > 2440)
    {
      battery_level = 18 - ((2740 - mvolts) * 12) / 300;
    }
  else if (mvolts > 2100)
    {
      battery_level = 6 - ((2440 - mvolts) * 6) / 340;
    }
  else
    {
      battery_level = 0;
    }

  return battery_level;
}

/**@brief Function for checking if a pointer value is aligned to a 4 byte boundary.
 *
 * @param[in]   p   Pointer value to be checked.
 *
 * @return      TRUE if pointer is aligned to a 4 byte boundary, FALSE otherwise.
 */
static __INLINE bool is_word_aligned(void const *p)
{
  return (((uintptr_t)p & 0x03) == 0);
}

/**@brief Macro for calling error handler function.
 *
 * @param[in] ERR_CODE Error code supplied to the error handler.
 */
#if 0
#ifdef DEBUG
#define APP_ERROR_HANDLER(ERR_CODE)                                    \
      do                                                                 \
      {                                                                  \
          _err("ERR_CODE: %d, LINE: %d, FILE: %s\n", (ERR_CODE), __LINE__, (uint8_t*) __FILE__);  \
      } while (0)
#else
#define APP_ERROR_HANDLER(ERR_CODE)                                    \
      do                                                                 \
      {                                                                  \
          _err("ERR_CODE: %d\n",(ERR_CODE));                            \
          up_systemreset();                                           \
      } while (0)
#endif
/**@brief Macro for calling error handler function if supplied error code any other than NRF_SUCCESS.
 *
 * @param[in] ERR_CODE Error code supplied to the error handler.
 */
#define APP_ERROR_CHECK(ERR_CODE)                           \
      do                                                      \
      {                                                       \
          uint32_t LOCAL_ERR_CODE = (ERR_CODE);         \
          if (LOCAL_ERR_CODE != NRF_SUCCESS)                  \
          {                                                   \
              APP_ERROR_HANDLER(LOCAL_ERR_CODE);              \
          }                                                   \
      } while (0)

/**@brief Macro for calling error handler function if supplied boolean value is false.
 *
 * @param[in] BOOLEAN_VALUE Boolean value to be evaluated.
 */
#define APP_ERROR_CHECK_BOOL(BOOLEAN_VALUE)                   \
      do                                                        \
      {                                                         \
          uint32_t LOCAL_BOOLEAN_VALUE = (BOOLEAN_VALUE); \
          if (!LOCAL_BOOLEAN_VALUE)                             \
          {                                                     \
              APP_ERROR_HANDLER(0);                             \
          }                                                     \
      } while (0)

#endif

#ifdef __cplusplus
}
#endif

#endif // APP_UTIL_H__

/** @} */
