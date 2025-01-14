/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
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

#ifndef SDK_MAPPED_FLAGS_H__
#define SDK_MAPPED_FLAGS_H__

#include <stdint.h>
#include <stdbool.h>
#include "app_util.h"
#include "compiler_abstraction.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file
 * @defgroup sdk_mapped_flags Mapped flags
 * @ingroup app_common
 * @{
 * @brief Module for writing and reading flags that are associated
 *        with keys.
 *
 * @details The flags are represented as bits in a bitmap called a <i>flag collection</i>. The keys
 *          are uint16_t. Each flag collection contains all flags of the same type, one flag for
 *          each key.
 *
 *          The mapped flags module does not keep the flag states, nor the list of keys. These are
 *          provided in the API calls. A key's index in the key list determines which bit in the
 *          flag collection is associated with it. This module does not ever edit the key list, and
 *          does not edit flags except in function calls that take the flag collection as a pointer.
 *
 */

#define SDK_MAPPED_FLAGS_N_KEYS          8       /**< The number of keys to keep flags for. This is also the number of flags in a flag collection. If changing this value, you might also need change the width of the sdk_mapped_flags_t type. */
#define SDK_MAPPED_FLAGS_N_KEYS_PER_BYTE 8       /**< The number of flags that fit in one byte. */
#define SDK_MAPPED_FLAGS_INVALID_INDEX   0xFFFF  /**< A flag index guaranteed to be invalid. */

typedef uint8_t
sdk_mapped_flags_t; /**< The bitmap to hold flags. Each flag is one bit, and each bit represents the flag state associated with one key. */


// Test whether the flag collection type is large enough to hold all the flags. If this fails,
// reduce SDK_MAPPED_FLAGS_N_KEYS or increase the size of sdk_mapped_flags_t.
STATIC_ASSERT((
                sizeof(sdk_mapped_flags_t) * SDK_MAPPED_FLAGS_N_KEYS_PER_BYTE) >= SDK_MAPPED_FLAGS_N_KEYS);


/**@brief Type used to present a subset of the registered keys.
 */
typedef struct
{
  uint32_t len;                                 /**< The length of the list. */
  uint16_t flag_keys[SDK_MAPPED_FLAGS_N_KEYS];  /**< The list of keys. */
} sdk_mapped_flags_key_list_t;


/**@brief Function for getting the first index at which the flag is true in the provided
 *        collection.
 *
 * @param[in]  flags   The flag collection to search for a flag set to true.
 *
 * @return  The first index that has its flag set to true. If none were found, the
 *          function returns @ref SDK_MAPPED_FLAGS_INVALID_INDEX.
 */
uint16_t sdk_mapped_flags_first_key_index_get(sdk_mapped_flags_t flags);


/**@brief Function for updating the state of a flag.
 *
 * @param[in]  p_keys   The list of associated keys (assumed to have a length of
 *                      @ref SDK_MAPPED_FLAGS_N_KEYS).
 * @param[out] p_flags  The flag collection to modify.
 * @param[in]  key      The key to modify the flag of.
 * @param[in]  value    The state to set the flag to.
 */
void sdk_mapped_flags_update_by_key(uint16_t            *p_keys,
                                    sdk_mapped_flags_t *p_flags,
                                    uint16_t             key,
                                    bool                 value);


/**@brief Function for updating the state of the same flag in multiple flag collections.
 *
 * @details The key and value are the same for all flag collections in the p_flags array.
 *
 * @param[in]  p_keys              The list of associated keys (assumed to have a length of
 *                                 @ref SDK_MAPPED_FLAGS_N_KEYS).
 * @param[out] p_flags             The flag collections to modify.
 * @param[out] n_flag_collections  The number of flag collections in p_flags.
 * @param[in]  key                 The key to modify the flag of.
 * @param[in]  value               The state to set the flag to.
 */
void sdk_mapped_flags_bulk_update_by_key(uint16_t            *p_keys,
                                         sdk_mapped_flags_t *p_flags,
                                         uint32_t             n_flag_collections,
                                         uint16_t             key,
                                         bool                 value);


/**@brief Function for getting the state of a specific flag.
 *
 * @param[in]  p_keys  The list of associated keys (assumed to have a length of
 *                     @ref SDK_MAPPED_FLAGS_N_KEYS).
 * @param[in]  flags   The flag collection to read from.
 * @param[in]  key     The key to get the flag for.
 *
 * @return  The state of the flag.
 */
bool sdk_mapped_flags_get_by_key(uint16_t *p_keys, sdk_mapped_flags_t flags, uint16_t key);


/**@brief Function for getting a list of all keys that have a specific flag set to true.
 *
 * @param[in]  p_keys  The list of associated keys (assumed to have a length of
 *                     @ref SDK_MAPPED_FLAGS_N_KEYS).
 * @param[in]  flags   The flag collection to search.
 *
 * @return  The list of keys.
 */
sdk_mapped_flags_key_list_t sdk_mapped_flags_key_list_get(uint16_t            *p_keys,
                                                          sdk_mapped_flags_t   flags);


/**@brief Function for getting the number of keys that have a specific flag set to true.
 *
 * @param[in]  flags  The flag collection to search.
 *
 * @return  The number of keys.
 */
uint32_t sdk_mapped_flags_n_flags_set(sdk_mapped_flags_t flags);


/**@brief Function for querying whether any flags in the collection are set.
 *
 * @param[in]  flags  The flag collection to query.
 *
 * @retval  true If one or more flags are set to true.
 * @retval  false Otherwise.
 */
static __INLINE bool sdk_mapped_flags_any_set(sdk_mapped_flags_t flags)
{
  return (flags != 0);
}


/** @} */


#ifdef __cplusplus
}
#endif

#endif /* SDK_MAPPED_FLAGS_H__ */
