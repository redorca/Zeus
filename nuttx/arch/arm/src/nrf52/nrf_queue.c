/*****************************************************************************
 * @file     nrf52_queue.c
 *
 * @brief    zGlue created source file to contain queue handing routines
 *           for which NuttX does not provide (namely circular queues)
 *
 * @version  V1
 * @date     02. January 2018
 *
 * @note
 *
 * @par      Copyright (c) 2017,2018 zGlue Inc.
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

#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "nrf_queue.h"

/*****************************************************************************
 * Public Functions
 *****************************************************************************/
#if defined(SUPPRESS_INLINE_IMPLEMENTATION)
bool nrf_cirq_is_empty(FAR nrf_circ_q_t *cirq)
{
  bool flag;

  flag = cirq->trailing == cirq->leading ? true : false;

  if (flag)
    {
      cirq->trailing = 0;
      cirq->leading = 0;
    }
  return flag
}

bool nrf_cirq_is_full(FAR nrf_circ_q_t *cirq)
{
  return (cirq->trailing + cirq->size - 1) == cirq->leading;
}

bool nrf_cirq_put(FAR nrf_circ_q_t *cirq, uint8_t data)
{
  bool empty;

  empty = nrf_cirq_is_full(cirq);
  if (!empty)
    {
      cirq->buf[cirq->leading++ % cirq->size] = data;
    }

  return !empty;
}

bool nrf_cirq_get(FAR nrf_circ_q_t *cirq, FAR uint8_t *data)
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

FAR nrf_circ_q_t *nrf_cirq_alloc(size_t size)
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

#endif /* SUPPRESS_INLINE_IMPLEMENTATION */
