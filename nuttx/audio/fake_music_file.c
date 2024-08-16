/****************************************************************************
 * drivers/audio/fake_music_file.c
 *
 * This is a simple character driver for testing I2S.It pretend to be
 * a wav formate music file.
 *
 *   Copyright (C) 2018 zGlue Inc. All rights reserved.
 *   Author: Zhengwei Wang <zhengwei@zglue.com>
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

#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/i2s.h>

#ifdef CONFIG_AUDIO_FAKE_MUSIC_FILE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
#define END_ADDRESS   0xEBB40
#define START_ADDRESS 0x80000

/* Device naming ************************************************************/


/****************************************************************************
 * Private Types
 ****************************************************************************/
struct bufferchar_dev_s
{
  uint32_t offset;              /* Assures mutually exclusive access */
};


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static ssize_t bufferchar_read(FAR struct file *filep, FAR char *buffer,
                               size_t buflen);
static int bufferchar_open(FAR struct file *filep);


/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations bufferchar_fops =
{
  bufferchar_open,                 /* open  */
  NULL,                 /* close */
  bufferchar_read,         /* read  */
  NULL,        /* write */
  NULL,                 /* seek  */
  NULL,                 /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  NULL,                 /* poll  */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static int bufferchar_open(FAR struct file *filep)
{
  FAR struct inode     *inode = filep->f_inode;
  FAR struct bufferchar_dev_s *priv   = inode->i_private;

  priv->offset = START_ADDRESS;

  return OK;
}



/****************************************************************************
 * Name: bufferchar_read
 *
 * Description:
 *   Standard character driver read method
 *
 ****************************************************************************/

static ssize_t bufferchar_read(FAR struct file *filep, FAR char *buffer,
                               size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bufferchar_dev_s *priv;
  uint32_t len;

  /* Get our private data structure */

  DEBUGASSERT(filep && buffer);

  inode = filep->f_inode;
  DEBUGASSERT(inode);

  priv = (FAR struct bufferchar_dev_s *)inode->i_private;
  DEBUGASSERT(priv);

  if (priv->offset + buflen > END_ADDRESS)
    {
      len = END_ADDRESS - priv->offset;
    }
  else
    {
      len = buflen;
    }

  memcpy(buffer, (char *)priv->offset, len);

  priv->offset += len;

  /* Lie to the caller and tell them that all of the bytes have been
   * received
   */

  return len;

}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bufferchar_register
 *
 * Description:
 *   Create and register the character driver.It will add a audio file to /music
 *   directory, so we can use this file to test the playback features.
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int fake_music_file_register(void)
{
  FAR struct bufferchar_dev_s *priv;
  int ret;

  /* Allocate a I2S character device structure */

  priv = (FAR struct bufferchar_dev_s *)kmm_zalloc(sizeof(struct bufferchar_dev_s));
  if (priv)
    {
      /* Create the character device name */

      ret = register_driver("/music/audiobuffer.wav", &bufferchar_fops, 0666, priv);
      if (ret < 0)
        {
          /* Free the device structure if we failed to create the character
           * device.
           */

          kmm_free(priv);
        }

      /* Return the result of the registration */

      return OK;
    }


  return -ENOMEM;
}

#endif /*CONFIG_AUDIO_FAKE_MUSIC_FILE*/
