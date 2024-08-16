/*
 * Copyright (c) 2017 Simon Goldschmidt
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 * 
 * Author: Simon Goldschmidt
 *
 */


#include <lwip/opt.h>
#include <lwip/arch.h>
#if !NO_SYS
#include "arch/sys_arch.h"
#endif
#include <lwip/stats.h>
#include <lwip/debug.h>
#include <lwip/sys.h>

#include <string.h>

u32_t lwip_sys_now;

u32_t
sys_jiffies(void)
{
  return lwip_sys_now;
}

u32_t
sys_now(void)
{
  return lwip_sys_now;
}

void
sys_init(void)
{
}

#if !NO_SYS

test_sys_arch_waiting_fn the_waiting_fn;

void
test_sys_arch_wait_callback(test_sys_arch_waiting_fn waiting_fn)
{
  the_waiting_fn = waiting_fn;
}

err_t
sys_sem_new(sys_sem_t *sem, u8_t count)
{
    assert(sem != NULL);

    if (!sem) {
        return ERR_ARG;
    }

    if(OK == sem_init(sem, 0, count)){
        return ERR_OK;
    }

    return ERR_VAL;
}


void
sys_sem_free(sys_sem_t *sem)
{
    assert(sem != NULL);

    /* Check for valid semaphore */
    if (!sem) {
        return;
    }
    sem_destroy(sem);

    return;
}



void
sys_sem_set_invalid(sys_sem_t *sem)
{
  LWIP_ASSERT("sem != NULL", sem != NULL);
  sem = NULL;
}

/* semaphores are 1-based because RAM is initialized as 0, which would be valid */
u32_t
sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
    int ret = 0;

    struct timespec abstime;
    struct timespec curtime;

    assert(sem != NULL);

    /* Check for valid semaphore */
    if (!sem) {
        return ERR_ARG;
    }

    ret = clock_gettime(CLOCK_REALTIME, &curtime);
    if (ret != OK) {
        return ERR_VAL;
    }
    else if (curtime.tv_sec < 0 || curtime.tv_nsec < 0 || curtime.tv_nsec >= NSEC_PER_SEC) {
        return ERR_VAL;
    }

    /* First convert timeout tick into nuttx time unit */


    timeout = TICK2USEC(timeout);

    abstime.tv_nsec = ((timeout*NSEC_PER_USEC + curtime.tv_nsec) % NSEC_PER_SEC);
    abstime.tv_sec = ((timeout*NSEC_PER_USEC + curtime.tv_nsec) / NSEC_PER_SEC) + curtime.tv_sec;

    ret = sem_timedwait(sem, &abstime);

    if(OK == ret) {
        return ERR_OK;
    }

    if(ETIMEDOUT == ret) {
        return ERR_TIMEOUT;
    }

    return ERR_VAL;
}


void
sys_sem_signal(sys_sem_t *sem)
{
    assert(sem != NULL);

    /* Check for valid semaphore */
    if (!sem) {
        return;
    }
    sem_post(sem);

    return;
}


err_t
sys_mutex_new(sys_mutex_t *mutex)
{
    assert(NULL != mutex);

    if (!mutex) {
        return ERR_ARG;
    }

    sem_init(mutex, 0, 1);

    return ERR_OK;
}


void
sys_mutex_free(sys_mutex_t *mutex)
{
    assert(mutex != NULL);

    /* Check for valid semaphore */
    if (!mutex) {
        return;
    }
    sem_destroy(mutex);

    return;
}


void
sys_mutex_set_invalid(sys_mutex_t *mutex)
{
  LWIP_ASSERT("mutex != NULL", mutex != NULL);
  mutex = NULL;
}

void
sys_mutex_lock(sys_mutex_t *mutex)
{
    assert(mutex != NULL);

    /* Check for valid semaphore */
    if (!mutex) {
        return;
    }
    sem_post(mutex);

    return;
}


void
sys_mutex_unlock(sys_mutex_t *mutex)
{
    assert(mutex != NULL);

    /* Check for valid semaphore */
    if (!mutex) {
        return;
    }
    sem_wait(mutex);

    return;
}

sys_thread_t
sys_thread_new(const char *name, lwip_thread_fn function, void *arg, int stacksize, int prio)
{
  LWIP_UNUSED_ARG(name);
  LWIP_UNUSED_ARG(function);
  LWIP_UNUSED_ARG(arg);
  LWIP_UNUSED_ARG(stacksize);
  LWIP_UNUSED_ARG(prio);
  /* threads not supported */
  return 0;
}

err_t
sys_mbox_new(sys_mbox_t *mbox, int size)
{
  int mboxsize = size;
  LWIP_ASSERT("mbox != NULL", mbox != NULL);
  LWIP_ASSERT("size >= 0", size >= 0);
  if (size == 0) {
    mboxsize = 1024;
  }
  mbox->head = mbox->tail = 0;
  mbox->sem = mbox; /* just point to something for sys_mbox_valid() */
  mbox->q_mem = (void**)malloc(sizeof(void*)*mboxsize);
  mbox->size = mboxsize;
  mbox->used = 0;

  memset(mbox->q_mem, 0, sizeof(void*)*mboxsize);
  return ERR_OK;
}

void
sys_mbox_free(sys_mbox_t *mbox)
{
  /* parameter check */
  LWIP_ASSERT("mbox != NULL", mbox != NULL);
  LWIP_ASSERT("mbox->sem != NULL", mbox->sem != NULL);
  LWIP_ASSERT("mbox->sem == mbox", mbox->sem == mbox);
  LWIP_ASSERT("mbox->q_mem != NULL", mbox->q_mem != NULL);
  mbox->sem = NULL;
  free(mbox->q_mem);
  mbox->q_mem = NULL;
}

void
sys_mbox_set_invalid(sys_mbox_t *mbox)

{
  LWIP_ASSERT("mbox != NULL", mbox != NULL);
  LWIP_ASSERT("mbox->q_mem == NULL", mbox->q_mem == NULL);
  mbox->sem = NULL;
  mbox->q_mem = NULL;
}

void
sys_mbox_post(sys_mbox_t *q, void *msg)
{
  LWIP_ASSERT("q != SYS_MBOX_NULL", q != SYS_MBOX_NULL);
  LWIP_ASSERT("q->sem == q", q->sem == q);
  LWIP_ASSERT("q->q_mem != NULL", q->q_mem != NULL);
  LWIP_ASSERT("q->used >= 0", q->used >= 0);
  LWIP_ASSERT("q->size > 0", q->size > 0);

  LWIP_ASSERT("mbox already full", q->used < q->size);

  q->q_mem[q->head] = msg;
  q->head++;
  if (q->head >= (unsigned int)q->size) {
    q->head = 0;
  }
  LWIP_ASSERT("mbox is full!", q->head != q->tail);
  q->used++;
}

err_t
sys_mbox_trypost(sys_mbox_t *q, void *msg)
{
  LWIP_ASSERT("q != SYS_MBOX_NULL", q != SYS_MBOX_NULL);
  LWIP_ASSERT("q->sem == q", q->sem == q);
  LWIP_ASSERT("q->q_mem != NULL", q->q_mem != NULL);
  LWIP_ASSERT("q->used >= 0", q->used >= 0);
  LWIP_ASSERT("q->size > 0", q->size > 0);
  LWIP_ASSERT("q->used <= q->size", q->used <= q->size);

  if (q->used == q->size) {
    return ERR_MEM;
  }
  sys_mbox_post(q, msg);
  return ERR_OK;
}

err_t
sys_mbox_trypost_fromisr(sys_mbox_t *q, void *msg)
{
  return sys_mbox_trypost(q, msg);
}

u32_t
sys_arch_mbox_fetch(sys_mbox_t *q, void **msg, u32_t timeout)
{
  u32_t ret = 0;
  u32_t ret2;
  LWIP_ASSERT("q != SYS_MBOX_NULL", q != SYS_MBOX_NULL);
  LWIP_ASSERT("q->sem == q", q->sem == q);
  LWIP_ASSERT("q->q_mem != NULL", q->q_mem != NULL);
  LWIP_ASSERT("q->used >= 0", q->used >= 0);
  LWIP_ASSERT("q->size > 0", q->size > 0);

  if (q->used == 0) {
    /* need to wait */
    /* need to wait */
    if(!timeout)
    {
      /* wait infinite */
      LWIP_ASSERT("cannot wait without waiting callback", the_waiting_fn != NULL);
      do {
        int expectSomething = the_waiting_fn(NULL, q);
        LWIP_ASSERT("q->used >= 0", q->used >= 0);
        LWIP_ASSERT("expecting item available but it's 0", !expectSomething || (q->used > 0));
        ret++;
        if (ret == SYS_ARCH_TIMEOUT) {
          ret--;
        }
      } while(q->used == 0);
    }
    else
    {
      if (the_waiting_fn) {
        int expectSomething = the_waiting_fn(NULL, q);
        LWIP_ASSERT("expecting item available count but it's 0", !expectSomething || (q->used > 0));
      }
      LWIP_ASSERT("q->used >= 0", q->used >= 0);
      if (q->used == 0) {
        if(msg) {
          *msg = NULL;
        }
        return SYS_ARCH_TIMEOUT;
      }
      ret = 1;
    }
  }
  LWIP_ASSERT("q->used > 0", q->used > 0);
  ret2 = sys_arch_mbox_tryfetch(q, msg);
  LWIP_ASSERT("got no message", ret2 == 0);
  return ret;
}

u32_t
sys_arch_mbox_tryfetch(sys_mbox_t *q, void **msg)
{
  LWIP_ASSERT("q != SYS_MBOX_NULL", q != SYS_MBOX_NULL);
  LWIP_ASSERT("q->sem == q", q->sem == q);
  LWIP_ASSERT("q->q_mem != NULL", q->q_mem != NULL);
  LWIP_ASSERT("q->used >= 0", q->used >= 0);
  LWIP_ASSERT("q->size > 0", q->size > 0);

  if (!q->used) {
    return SYS_ARCH_TIMEOUT;
  }
  if(msg) {
    *msg = q->q_mem[q->tail];
  }

  q->tail++;
  if (q->tail >= (unsigned int)q->size) {
    q->tail = 0;
  }
  q->used--;
  LWIP_ASSERT("q->used >= 0", q->used >= 0);
  return 0;
}

#if LWIP_NETCONN_SEM_PER_THREAD
#error LWIP_NETCONN_SEM_PER_THREAD==1 not supported
#endif /* LWIP_NETCONN_SEM_PER_THREAD */

#endif /* !NO_SYS */
