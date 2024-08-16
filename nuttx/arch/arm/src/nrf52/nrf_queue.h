/*****************************************************************************
 * @file     nrf_queue.h
 *
 * @brief    zGlue created header file to contain queue handing macros and declarations
 *           for which NuttX does not provide (namely circular queues)
 *
 * @version  V1
 * @date     18. December 2017
 *
 * @note
 *
 * @par      Copyright (c) 2017, zGlue Inc.
 *           All rights reserved.
 *
 *           Redistribution and use in source and binary forms, with or without
 *           modification, are permitted provided that the following conditions are met:
 *
 *           * Redistributions of source code must retain the above copyright notice, this
 *           list of conditions and the following disclaimer.
 *
 *           * Redistributions in binary form must reproduce the above copyright notice,
 *           this list of conditions and the following disclaimer in the documentation
 *           and/or other materials provided with the distribution.
 *
 *           * Neither the name of Nordic Semiconductor ASA nor the names of its
 *           contributors may be used to endorse or promote products derived from
 *           this software without specific prior written permission.
 *
 *           THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *           AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *           IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *           DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *           FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *           DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *           SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *           CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *           OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *           OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *****************************************************************************/


#ifndef __ARCH_ARM_SRC_NRF52_NRF_QUEUE_H
#define __ARCH_ARM_SRC_NRF52_NRF_QUEUE_H

#include <stdint.h>
#include <stdbool.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>

/*****************************************************************************
 * Public Types
 *****************************************************************************/
typedef struct
{
  sem_t q_sem;
  size_t trailing;        /* Simple counter */
  size_t leading;         /* Simple counter */
  size_t size;            /* Ideally, should be a multiple of 2 */
  size_t count;           /* The number of entries occupied. */
  uint8_t *buf;
} nrf_circ_q_t;

/*****************************************************************************
 * Public Functions Prototypes
 *****************************************************************************/
#if !defined(SUPPRESS_INLINE_IMPLEMENTATION)
#define __STATIC_INLINE static inline
#else
#define __STATIC_INLINE
#endif

/**
 * @brief Function for initializing a circular q (ring buffer)
 *
 * @param[in] *cirq  Pointer to the circular queue structure
 */
__STATIC_INLINE FAR nrf_circ_q_t *nrf_cirq_alloc(size_t size);

/**
 * @brief Function for checking the full state of the given circular queue
 *
 * @param[in] *cirq  Pointer to the circular queue structure
 */
__STATIC_INLINE bool nrf_cirq_is_full(FAR nrf_circ_q_t *cirq);

/**
 * @brief Function for checking the empty state of the given circular queue
 *
 * @param[in] *cirq  Pointer to the circular queue structure
 */
__STATIC_INLINE bool nrf_cirq_is_empty(FAR nrf_circ_q_t *cirq);

/**
 * @brief Function for adding  an element to the circular queue
 *
 * @param[in] *cirq  Pointer to the circular queue structure
 */
__STATIC_INLINE bool nrf_cirq_put(FAR nrf_circ_q_t *cirq, uint8_t data);

/**
 * @brief Function for removing an element of the circular queue
 *
 * @param[in] *cirq  Pointer to the circular queue structure
 */
__STATIC_INLINE bool nrf_cirq_get(FAR nrf_circ_q_t *cirq, FAR uint8_t *data);


/**
 * @brief Function for calculating the amount of data in the queue
 *
 * @param[in] *cirq  Pointer to the circular queue structure
 * @param[out] word representing the numbere of bytes in the queue
 */
__STATIC_INLINE uint32_t nrf_cirq_bytes(FAR nrf_circ_q_t *cirq);

#if !defined(SUPPRESS_INLINE_IMPLEMENTATION)
/**
 * @brief Function for allocating a circular q (ring buffer)
 *
 * @param[in] *cirq  Pointer to the circular queue structure
 */
__STATIC_INLINE FAR nrf_circ_q_t *nrf_cirq_alloc(size_t size)
{
  FAR nrf_circ_q_t *tmpq;

  tmpq = (nrf_circ_q_t *) kmm_malloc(sizeof(nrf_circ_q_t));
  if (tmpq == NULL)
    {
      goto err_B;
    }
  tmpq->buf = (uint8_t *) kmm_malloc(size);
  if (tmpq->buf == NULL)
    {
      goto err_A;
    }

  tmpq->size = size;
  tmpq->count = 0;
  tmpq->trailing = 0;
  tmpq->leading = 0;

  if (sem_init(&(tmpq->q_sem), 0, 0) == 0)
    {
      return tmpq;
    };

err_A:
  kmm_free(tmpq->buf);
err_B:
  kmm_free(tmpq);

  return NULL;
}

__STATIC_INLINE bool nrf_cirq_is_empty(FAR nrf_circ_q_t *cirq)
{
  bool flag;

  flag = cirq->trailing == cirq->leading ? true : false;

  if (flag)
    {
      cirq->trailing = 0;
      cirq->leading = 0;
    }
  return flag;
}

__STATIC_INLINE bool nrf_cirq_is_full(FAR nrf_circ_q_t *cirq)
{
  return (cirq->trailing + cirq->size - 1) == cirq->leading;
}

__STATIC_INLINE bool nrf_cirq_put(FAR nrf_circ_q_t *cirq, uint8_t data)
{
  bool empty;

  empty = nrf_cirq_is_full(cirq);
  if (!empty)
    {
      cirq->buf[cirq->leading++ % cirq->size] = data;
    }

  return !empty;
}

__STATIC_INLINE bool nrf_cirq_get(FAR nrf_circ_q_t *cirq, FAR uint8_t *data)
{
  bool full;

  full = nrf_cirq_is_empty(cirq);
  if (!full)
    {
      *data = cirq->buf[cirq->trailing++ % cirq->size];
    }

  return !full;
}

uint32_t nrf_cirq_bytes(FAR nrf_circ_q_t *cirq)
{
  return cirq->leading - cirq->trailing;
}
#endif /* SUPPRESS_INLINE_IMPLEMENTATION */

#endif /*  __ARCH_ARM_SRC_NRF52_NRF_QUEUE_H */
