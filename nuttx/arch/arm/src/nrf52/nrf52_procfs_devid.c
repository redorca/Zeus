/****************************************************************************
 * arch/arm/src/nrf52/nrf52_procfs_devid.c
 *
 *   Copyright (C) 2017 Zglue  Inc. All rights reserved.
 *   Authors: Levin Li     <zhiqiang@zglue.com>
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
#include <sys/statfs.h>
#include <sys/stat.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/procfs.h>
#include <nuttx/fs/dirent.h>

#include <arch/irq.h>

#include "nrf_nvmc.h"

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_FS_PROCFS) && \
     defined(CONFIG_FS_PROCFS_REGISTER) && defined(CONFIG_NRF52_PROCFS_DEVID)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CCM_LINELEN     64

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes one open "file" */
struct devid_file_s
{
  struct procfs_file_s  base;   /* Base open file structure */
};


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* File system methods */

static int     devid_open(FAR struct file *filep, FAR const char *relpath,
                          int oflags, mode_t mode);
static int     devid_close(FAR struct file *filep);
static ssize_t devid_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static int     devid_stat(FAR const char *relpath, FAR struct stat *buf);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* See include/nutts/fs/procfs.h
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

static const struct procfs_operations devid_procfsoperations =
{
  devid_open,   /* open */
  devid_close,  /* close */
  devid_read,   /* read */
  NULL,         /* write */
  NULL,         /* dup */
  NULL,         /* opendir */
  NULL,         /* closedir */
  NULL,         /* readdir */
  NULL,         /* rewinddir */
  devid_stat    /* stat */
};

static const struct procfs_entry_s g_procfs_devid =
{
  "devid",
  &devid_procfsoperations
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devid_open
 ****************************************************************************/

static int devid_open(FAR struct file *filep, FAR const char *relpath,
                      int oflags, mode_t mode)
{
  FAR struct devid_file_s *attr;

  finfo("Open '%s'\n", relpath);

  /* PROCFS is read-only.  Any attempt to open with any kind of write
   * access is not permitted.
   *
   * REVISIT:  Write-able proc files could be quite useful.
   */

  if ((oflags & O_WRONLY) != 0 || (oflags & O_RDONLY) == 0)
    {
      ferr("ERROR: Only O_RDONLY supported\n");
      return -EACCES;
    }

  /* "devid" is the only acceptable value for the relpath */

  if (strcmp(relpath, "devid") != 0)
    {
      ferr("ERROR: relpath is '%s'\n", relpath);
      return -ENOENT;
    }

  /* Allocate a container to hold the file attributes */

  attr = (FAR struct devid_file_s *)kmm_zalloc(sizeof(struct devid_file_s));
  if (!attr)
    {
      ferr("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  /* Save the attributes as the open-specific state in filep->f_priv */

  filep->f_priv = (FAR void *)attr;

  return OK;
}

/****************************************************************************
 * Name: devid_close
 ****************************************************************************/

static int devid_close(FAR struct file *filep)
{
  FAR struct devid_file_s *attr;

  /* Recover our private data from the struct file instance */

  attr = (FAR struct devid_file_s *)filep->f_priv;
  DEBUGASSERT(attr);

  /* Release the file attributes structure */

  kmm_free(attr);
  filep->f_priv = NULL;
  return OK;
}


/****************************************************************************
 * Name: devid_read
 ****************************************************************************/

static ssize_t devid_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{

  uint32_t devid[2];
  ssize_t  ret;
  off_t offset;

  finfo("buffer=%p buflen=%d\n", buffer, (int)buflen);

  devid[0] = nrf_nvmc_read_dev_id0();
  devid[1] = nrf_nvmc_read_dev_id1();

  finfo("devid0=%#x, devid1=%#x\n", devid[0], devid[1]);

  /* Transfer the system up time to user receive buffer */

  offset = filep->f_pos;
  ret    = procfs_memcpy((const char *)devid, 8, buffer, buflen, &offset);

  filep->f_pos += ret;

  return ret;
}

static int devid_stat(const char *relpath, struct stat *buf)
{
  if (strcmp(relpath, "devid") != 0)
    {
      ferr("ERROR: relpath is '%s'\n", relpath);
      return -ENOENT;
    }

  buf->st_mode    = S_IFREG | S_IROTH | S_IRGRP | S_IRUSR;
  buf->st_size    = 8;
  buf->st_blksize = 0;
  buf->st_blocks  = 0;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devid_procfs_register
 *
 * Description:
 *   Register the CCM procfs file system entry
 *
 ****************************************************************************/

int devid_procfs_register(void)
{
  return procfs_register(&g_procfs_devid);
}

#endif /* !CONFIG_DISABLE_MOUNTPOINT && CONFIG_FS_PROCFS &&
        * CONFIG_FS_PROCFS_REGISTER && CONFIG_NRF52_DEVID_PROCFS */

