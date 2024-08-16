/****************************************************************************
 * arch/arm/src/nrf52/nrf52_rng.c
 *
 *   Copyright (C) 2012 Max Holtzberg. All rights reserved.
 *   Author: Max Holtzberg <mh@uvc.de>
 *   mods for STL32L4 port by dev@ziggurat29.com
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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/drivers/drivers.h>

#include <arch/board/board.h>
#include "nrf.h"
#include "up_arch.h"
#include "nrf.h"
#include "chip.h"
#include "nrf52_rng.h"
#include "up_internal.h"

#if defined(CONFIG_NRF52_RNG)

#ifdef SOFTDEVICE_PRESENT
#include "softdevice_handler.h"
#include "nrf_soc.h"
#include "app_util_platform.h"
#endif /* SOFTDEVICE_PRESENT */
#if defined(COLLIDE_WITH_HEADER)
#ifdef SOFTDEVICE_PRESENT
#define SD_RAND_POOL_SIZE           (32)
STATIC_ASSERT(RNG_CONFIG_POOL_SIZE == SD_RAND_POOL_SIZE);
#define NRF_DRV_RNG_SD_IS_ENABLED() softdevice_handler_is_enabled()
#define NRF_DRV_RNG_LOCK()        \
    {                                  \
        uint8_t __CR_NESTED = 0;       \
        sd_nvic_critical_region_enter(&__CR_NESTED);

#define NRF_DRV_RNG_RELEASE() \
        sd_nvic_critical_region_exit(__CR_NESTED); \
    }
#else /* ! SOFTDEVICE_PRESENT */
#define NRF_DRV_RNG_LOCK() \
    {                           \
        irq_state_t f;          \
        f = enter_critical_section();

#define NRF_DRV_RNG_RELEASE() \
        leave_critical_section(f); \
    }

#define NRF_DRV_RNG_SD_IS_ENABLED() false

#endif /* SOFTDEVICE_PRESENT */
#endif /* COLLIDE_WITH_HEADER */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int nrf52_rng_initialize(void);
static int nrf52_rng_irqhandler(int irq, void *context, FAR void *arg);
static void nrf52_rng_enable(void);
static void nrf52_rng_disable(void);
static ssize_t nrf52_rng_read(FAR struct file *filep, FAR char *buffer, size_t);
static ssize_t nrf52_rng_write(FAR struct file *filep, FAR const char *buffer, size_t);
static int nrf52_rng_open(FAR struct file *filep);
static int nrf52_rng_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rng_dev_s
{
  char     id[8];
  uint8_t *rd_buf;
  size_t   rd_count;
  size_t   buflen;
  sem_t    rd_sem;         /* eclusive gate for read RNG */
  sem_t    excl_devsem;    /* eclusive gate for device RNG */
  uint8_t  state;
};

#define BLOCK 0x01
#define RNG_DEVSTRUCT_ID  "FACEECAF"
/****************************************************************************
 * Private Data
 ****************************************************************************/

const char *devpath_urandom = "/dev/urandom";
const char *devpath_random = "/dev/random";

static struct rng_dev_s g_rngdev;

static const struct file_operations g_rngops =
{
  nrf52_rng_open,       /* open   */
  0,      /* close  */
  nrf52_rng_read,       /* read   */
  nrf52_rng_write,      /* write  */
  0,                   /* seek   */
  nrf52_rng_ioctl,      /* ioctl  */
#ifndef CONFIG_DISABLE_POLL
  0,                   /* poll   */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  0,                   /* unlink */
#endif
};

/****************************************************************************
 * Private functions
 ****************************************************************************/
__STATIC_ uint32_t rng_reg_write32(uint32_t reg_offset, uint32_t data)
{
  /* return putreg32(data, ((uint32_t *)NRF_RNG_BASE + ((uint32_t) reg_offset >> 2))); */
  return putreg32(data, RNG_REG_ADDR(reg_offset));
}

__STATIC_ uint32_t rng_reg_read32(uint32_t reg_offset)
{
  /* return getreg32((uint32_t *)NRF_RNG_BASE + ((uint32_t) reg_offset >> 2)); */
  return getreg32(RNG_REG_ADDR(reg_offset));
}

static void nrf52_rng_start(void)
{
  ASSERT(!NRF_DRV_RNG_SD_IS_ENABLED());
  NRF52_RNG_EVENT_CLR(RESET);
  NRF52_RNG_CONFIG_BIAS_SET(SET);
  NRF52_RNG_INTENSET(SET);
  NRF52_RNG_TASK_START(TRIGGER);
}

static void nrf52_rng_stop(void)
{
  ASSERT(!NRF_DRV_RNG_SD_IS_ENABLED());
  NRF52_RNG_TASK_STOP(TRIGGER);
  NRF52_RNG_INTENCLR(CLEAR);
  NRF52_RNG_EVENT_CLR(RESET);
  if (NRF52_RNG_EVENT_GET() != 0)
    {
      _warn("RNG event did not clear.\n");
    }
}

static int nrf52_rng_initialize(void)
{
  _info("Initializing RNG\n");
  int rv = 0;

  memset(&g_rngdev, 0, sizeof(struct rng_dev_s));

  snprintf(g_rngdev.id, 8, RNG_DEVSTRUCT_ID);
  rv += sem_init(&g_rngdev.rd_sem, 0, 1);

  if (rv != 0)
    {
      _err("Failed to initialize at least one of the semaphores.\n");
      return -ENOMEM;
    }
  sem_setprotocol(&g_rngdev.rd_sem, SEM_PRIO_NONE);

  _info("Ready to stop\n");
  nrf52_rng_stop();

  if (irq_attach(RNG_IRQn, nrf52_rng_irqhandler, &g_rngdev) != 0)
    {
      /* We could not attach the ISR to the interrupt */
      _warn("Could not attach IRQ.\n");

      return -EAGAIN;
    }

  return OK;
}

static void nrf52_rng_enable(void)
{
  nrf52_rng_start();

  /* Enable generation and interrupts */
  up_enable_irq(RNG_IRQn);
}

static void nrf52_rng_disable()
{
  up_disable_irq(RNG_IRQn);
  nrf52_rng_stop();
}


static int nrf52_rng_irqhandler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct rng_dev_s *priv = (struct rng_dev_s *) arg;
  uint8_t *addr;
  int32_t rv;

  rv = OK;
  if (NRF52_RNG_EVENT_GET() != 0)
    {
      NRF52_RNG_EVENT_CLR(RESET);
    }

  if (priv->rd_count == priv->buflen)
    {
      nrf52_rng_stop();
      sem_post(&priv->rd_sem);
      return OK;
    }

  if (priv->rd_count < priv->buflen)
    {
      addr = priv->rd_buf + priv->rd_count++;
      *addr = NRF52_RNG_READ_VALUE();
    }

  return rv;
}

/****************************************************************************
 * Name: nrf52_rng_open
 ****************************************************************************/

static int nrf52_rng_open(FAR struct file *filep)
{
  /* O_NONBLOCK is not supported */

  if (filep->f_oflags & O_NONBLOCK)
    {
      _err("nRF52 rng didn't support O_NONBLOCK mode.\n");
      return -EPERM;
    }

  return OK;
}


/****************************************************************************
 * Name: nrf52_rng_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *
 * Input Parameters:
 *   dev - pointer to device structure used by the driver
 *   cmd - command
 *   arg - arguments passed with command
 *
 * Returned Value:
 *
 ****************************************************************************/

static int nrf52_rng_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  int ret = OK;

  switch (cmd)
    {
      case RNG_SETFILTER:
        if (NRF52_RNG_CONFIG_READ() == 1)
          {
            /* Bias filter already enabled. */
            break;
          }
        NRF52_RNG_CONFIG_BIAS_SET(SET);
        break;
      case RNG_CLRFILTER:
        NRF52_RNG_CONFIG_BIAS_CLR(RESET);
        break;
      case RNG_START_CONV:
        NRF52_RNG_TASK_START(TRIGGER);
        break;
      case RNG_STOP_CONV:
        NRF52_RNG_TASK_STOP(TRIGGER);
        break;
      default:
        _err("ERROR: Unknown cmd: %d\n", cmd);
        ret = -EPERM;
        break;
    }

  return ret;

}

/****************************************************************************
 * Name: nrf52_rng_write
 ****************************************************************************/

static ssize_t nrf52_rng_write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
{
  return -EPERM;
}

/****************************************************************************
 * Name: nrf52_rn_gread
 ****************************************************************************/

static ssize_t nrf52_rng_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rng_dev_s *priv = (struct rng_dev_s *) inode->i_private;

  if (sem_wait(&priv->rd_sem) != OK)
    {
      errno = EBUSY;
      return -errno;
    }

  /* We've got the read semaphore, proceed with reading */

  /* Initialize semaphore with 0 for blocking until the buffer is filled from
   * interrupts.
   */

  sem_init(&priv->rd_sem, 0, 0);

  priv->rd_buf = (uint8_t *) buffer;
  priv->buflen = buflen;
  priv->rd_count = 0;

  /* Wait until the buffer is filled */

  nrf52_rng_enable();

  sem_wait(&priv->rd_sem);

  /* Free RNG for next use */

  nrf52_rng_disable();

  sem_post(&priv->rd_sem);

  /* Initialize the operation semaphore with 0 for blocking until the
   * buffer is filled from interrupts.  The waitsem semaphore is used
   * for signaling and, hence, should not have priority inheritance
   * enabled.
   */
  if (priv->rd_count > priv->buflen)
    {
      _err("Bad rd_count: Too much data, exceeds buffer size: %d\n", priv->rd_count);
    }

  return priv->rd_count;
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devrandom_register
 *
 * Description:
 *   Initialize the RNG hardware and register the /dev/random driver.
 *   Must be called BEFORE devurandom_register.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_RANDOM
void devrandom_register(void)
{
  nrf52_rng_initialize();
  (void)register_driver(devpath_random, FAR & g_rngops, 0444, FAR (void *) &g_rngdev);
}
#endif

/****************************************************************************
 * Name: devurandom_register
 *
 * Description:
 *   Register /dev/urandom
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_URANDOM_ARCH
void devurandom_register(void)
{
#ifndef CONFIG_DEV_RANDOM
  nrf52_rng_initialize();
#endif
  (void)register_driver(devpath_urandom, FAR & g_rngops, 0444, FAR (void *) &g_rngdev);
}
#endif

#endif /* CONFIG_NRF52_RNG */

