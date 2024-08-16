/****************************************************************************
 * examplex/mtdblockrw/mtdblockrw_main.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Copyright (C) 2017 Zglue  Inc. All rights reserved.
 *           Levin Li     <zhiqiang@zglue.com>
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
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/mtd/mtd.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_EXAMPLES_MTDBLOCKRW

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#define MAX_MTD_DEV_LEN  32

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mtdweb_filedesc_s
{
  FAR char *name;
  bool deleted;
  size_t len;
  uint32_t crc;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/
static uint32_t start_addr;
static uint32_t rw_len;
static char mtd_name[MAX_MTD_DEV_LEN];

/****************************************************************************
 * External Functions
 ****************************************************************************/


/****************************************************************************
 * Public Functions
 ****************************************************************************/
static void show_usage(void)
{
  printf("Usage: mtdblock_rw [OPTIONS]\n");
  printf("  [-d device] selects the MTD device\n");
  printf("  [-s start address] sepcial the RW Start Address\n");
  printf("  [-n size]  special the RW data Size\n");
  printf("  [-h] shows this message and exits\n");

  return;
}

static int arg_string(FAR char **arg, FAR char **value)
{
  FAR char *ptr = *arg;

  if (ptr[2] == '\0')
    {
      *value = arg[1];
      return 2;
    }
  else
    {
      *value = &ptr[2];
      return 1;
    }
}

static int arg_decimal(FAR char **arg, FAR long *value)
{
  FAR char *string;
  int ret;

  ret = arg_string(arg, &string);
  *value = strtol(string, NULL, 10);
  return ret;
}


static void parse_args(int argc, FAR char **argv)
{
  FAR char *ptr;
  FAR char *str;
  long value;
  int index;
  int nargs;
  int32_t str_len;

  for (index = 1; index < argc; )
    {
      ptr = argv[index];
      if (ptr[0] != '-')
        {
          printf("Invalid options format: %s\n", ptr);
          exit(0);
        }

      switch (ptr[1])
        {
          case 'n':
            nargs = arg_decimal(&argv[index], &value);
            if (value < 0)
              {
                printf("Count must be non-negative: %ld\n", value);
                exit(1);
              }

            rw_len = (uint32_t)value;
            index += nargs;
            break;

          case 's':
            nargs = arg_decimal(&argv[index], &value);
            if (value < 0)
              {
                printf("Count must be non-negative: %ld\n", value);
                exit(1);
              }

            rw_len = (uint32_t)value;
            index += nargs;
            break;

          case 'd':
            nargs = arg_string(&argv[index], &str);
            str_len = strlen(str);

            if (str_len < MAX_MTD_DEV_LEN)
              {
                memcpy(mtd_name, str, str_len + 1);
              }
            else
              {
                printf("the mtd device name length is out MAX LEN %d.\n", MAX_MTD_DEV_LEN);
              }
            index += nargs;
            break;

          case 'h':
            show_usage();
            exit(0);

          default:
            printf("Unsupported option: %s\n", ptr);
            show_usage();
            exit(1);
        }
    }
}

/****************************************************************************
 * Name: mtdrwb_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int mtdblock_rw_main(int argc, char *argv[])
#endif
{
  FAR struct mtd_geometry_s geo;
  FAR uint32_t *buffer = NULL;
  ssize_t nbytes;
  off_t nblocks;
  off_t offset;
  off_t seekpos;
  unsigned int blkpererase;
  int fd = -1;
  int i;
  int j;
  int k;
  int ret;

  mtd_name[0] = 0;
  start_addr = 0;

  parse_args(argc, argv);

  if (0 == mtd_name[0])
    {
      printf("Please special the mtd device name.\n");
      show_usage();
      return 1;
    }

  fd = open(mtd_name, O_RDWR);
  if (fd < 0)
    {
      printf("Can't open mtd device [%s].\n", mtd_name);
      exit(EXIT_FAILURE);
    }

  ret = ioctl(fd, MTDIOC_GEOMETRY, (unsigned long)&geo);
  if (ret < 0)
    {
      ferr("ERROR: ioctl MTDIOC_GEOMETRY failed: %d\n", ret);
      ret = 5;
      goto EXIT;
    }
  printf("Flash Geometry:\n");
  printf("  blocksize:      %lu\n", (unsigned long)geo.blocksize);
  printf("  erasesize:      %lu\n", (unsigned long)geo.erasesize);
  printf("  neraseblocks:   %lu\n", (unsigned long)geo.neraseblocks);

  blkpererase = geo.erasesize / geo.blocksize;
  printf("  blkpererase:    %u\n", blkpererase);

  nblocks = geo.neraseblocks * blkpererase;
  printf("  nblocks:        %lu\n", (unsigned long)nblocks);

  printf("\nStarting to erase All Flash....\n");
  ret = ioctl(fd, MTDIOC_BULKERASE, 0);
  if (ret < 0)
    {
      printf("ERROR: MTDIOC_BULKERASE ioctl failed: %d\n", ret);
      ret = 5;
      goto EXIT;
    }
  printf("\nErasing All Flash Done....\n");

  /* Allocate a buffer */

  buffer = (FAR uint32_t *)malloc(geo.blocksize);
  if (!buffer)
    {
      printf("ERROR: failed to allocate a sector buffer\n");
      fflush(stdout);
      ret = 6;
      goto EXIT;
    }


  /* Now write the offset into every block */

  printf("Initializing media:\n");

  offset = start_addr;
  seekpos = lseek(fd, offset, SEEK_SET);
  if (seekpos != offset)
    {
      printf("Can't Seek to Start Address %#x .\n", start_addr);
    }

  for (i = 0, j = 0; i < rw_len; i += geo.blocksize, j++)
    {
      /* Fill the block with the offset */

      for (k = 0; k < geo.blocksize / sizeof(uint32_t); k++)
        {
          buffer[k] = offset;
          offset += sizeof(uint32_t);
        }

      /* And write it using the character driver */

      nbytes = write(fd, buffer, geo.blocksize);
      if (nbytes < 0)
        {
          printf("ERROR: write to /dev/mtd0 failed: %d\n", errno);
          fflush(stdout);
          ret = 8;
          goto EXIT;
        }
      printf("Wrote Block %d  Done.\n", j);
    }

  /* Now verify the offset in every block */

  printf("Starting to verify writing data\n");

  /* Seek to start read/write init position */

  offset  = start_addr;
  seekpos = lseek(fd, offset, SEEK_SET);
  if (seekpos != offset)
    {
      printf("ERROR: lseek to offset %ld failed: %d\n",
             (unsigned long)offset, errno);
      fflush(stdout);
      ret = 10;
      goto EXIT;
    }

  for (j = 0; j < rw_len; j += nbytes)
    {
      /* Read the next block into memory */

      nbytes = read(fd, buffer, geo.blocksize);
      if (nbytes < 0)
        {
          printf("ERROR: read from %s failed: %d\n", mtd_name, errno);
          fflush(stdout);
          ret = 11;
          goto EXIT;
        }
      else if (nbytes == 0)
        {
          printf("ERROR: Unexpected end-of file in %s\n", mtd_name);
          fflush(stdout);
          ret = 12;
          goto EXIT;
        }
      else if (nbytes != geo.blocksize)
        {
          printf("ERROR: Unexpected read size from %s : %ld\n",
                 mtd_name, (unsigned long)nbytes);
          fflush(stdout);
          ret = 13;
          goto EXIT;
        }

      /* This is not expected at all */

      else if (nbytes != geo.blocksize)
        {
          printf("ERROR: Short read from /dev/mtd0 failed: %lu\n",
                 (unsigned long)nbytes);
          fflush(stdout);
          ret = 15;
          goto EXIT;
        }

      /* Verify the offsets in the block */

      printf("Starint to verify Block %d ...\n", j);

      for (k = 0; k < geo.blocksize / sizeof(uint32_t); k++)
        {
          if (buffer[k] != offset)
            {
              printf("ERROR: index %d, Bad offset %lu, expected %lu\n",
                     k, (long)buffer[k], (long)offset);
              fflush(stdout);
              ret = 16;
              goto EXIT;
            }

          offset += sizeof(uint32_t);
        }
    }


  /* And exit without bothering to clean up */
  printf("PASS: Everything looks good\n");

EXIT:

  if (fd >= 0)
    {
      close(fd);
    }

  if (buffer)
    {
      free(buffer);
    }

  fflush(stdout);
  return ret;
}

#endif /* CONFIG_EXAMPLES_MTDBLOCKRW */

