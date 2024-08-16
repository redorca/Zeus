/****************************************************************************
 *   system/zglue_fast/fast_reg.c
 *
 *   Copyright (C) 2011, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <fast_cmd.h>
#include <nuttx/zglue_fast/fast_debug_api.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* help functions */
static int32_t do_read_help(int32_t argc, char **argv);
static int32_t do_write_help(int32_t argc, char **argv);
static int32_t do_scan_help(int32_t argc, char **argv);
static int32_t do_clear_help(int32_t argc, char **argv);
/* tile functions */
static int32_t do_read_tile(int32_t argc, char **argv);
static int32_t do_write_tile(int32_t argc, char **argv);
static int32_t do_scan_tile(int32_t argc, char **argv);
static int32_t do_clear_tile(int32_t argc, char **argv);
static int32_t do_clear_all(int32_t argc, char **argv);

#if defined(CONFIG_ZEUS2)
static int32_t do_scan_otp(int32_t argc, char **argv);
static int32_t do_write_otp(int32_t argc, char **argv);
static int32_t do_scan_regs(int32_t argc, char **argv);
static int32_t do_read_reg(int32_t argc, char **argv);
static int32_t do_write_reg(int32_t argc, char **argv);
static int32_t do_read_peripheral(int32_t argc, char **argv);
static int32_t do_write_peripheral(int32_t argc, char **argv);
static int32_t do_clear_peripheral(int32_t argc, char **argv);
static int32_t do_scan_peripheral(int32_t argc, char **argv);

const char zeus2_tileio_str[] = {'t', 'i', 'l', 'e', 'i', 'o', '\0'};
const char zeus2_vrail_str[] = {'v', 'r', 'a', 'i', 'l', '\0'};
const char zeus2_rdac_str[] = {'r', 'd', 'a', 'c', '\0'};
const char zeus2_cdac_str[] = {'c', 'd', 'a', 'c', '\0'};

#define ZEUS2_LEFT_PERIPHERAL 1
#define ZEUS2_RIGHT_PERIPHERAL 2

#endif

#if defined(CONFIG_ZEUS1)
#define TILE_ROUTER_DATA_BIT_MASK 0xFFFF
#elif defined(CONFIG_ZEUS2)
#define TILE_ROUTER_DATA_BIT_MASK 0xFFFFFF
#else
#define TILE_ROUTER_DATA_BIT_MASK 0xFFFFFFFF
#endif

static const struct fast_cmdmap_s g_fast_read[] =
{
  { "help",     do_read_help,           "",  NULL},
  { "tile",     do_read_tile,           "fast read tile",   "\n\t [row]\n\t [column]"},
#if defined(CONFIG_ZEUS2)
  { "reg",      do_read_reg,            "fast read reg",    "\n\t [addr<hex>]"},
  { zeus2_tileio_str,   do_read_peripheral,       "fast read tileio",     "\n\t [(1=left, 2=right)]\n\t [index]"},
  { zeus2_vrail_str,    do_read_peripheral,       "fast read vrail",  "\n\t [(1=left, 2=right)]\n\t [index]"},
  { zeus2_rdac_str,     do_read_peripheral,       "fast read rdac",   "\n\t [(1=left, 2=right)]\n\t [index]"},
  { zeus2_cdac_str,     do_read_peripheral,       "fast read cdac",   "\n\t [index]"},
#endif
  { NULL,   NULL,        NULL,             NULL },
};

static const struct fast_cmdmap_s g_fast_scan[] =
{
  { "help",      do_scan_help,        "",  NULL},
  { "tile",     do_scan_tile,       "fast scan tile", "\n\t [start row]\n\t [start column]\n\t [end row]\n\t [end column]"},
#if defined(CONFIG_ZEUS2)
  { "reg",     do_scan_regs,       "fast scan reg", NULL},
  { "otp",     do_scan_otp,       "fast scan otp", "\n\t [(1=fast otp, 2=customer otp)]\n\t [start addr<hex>]\n\t [count in bytes]"},
  { zeus2_tileio_str,      do_scan_peripheral,   "fast scan tileio",  "\n\t [(1=left, 2=right)]\n\t [start index]\n\t [count]"},
  { zeus2_vrail_str,    do_scan_peripheral,       "fast scan vrail",  "\n\t [(1=left, 2=right)]\n\t [start index]\n\t [count]"},
  { zeus2_rdac_str,     do_scan_peripheral,       "fast scan rdac",   "\n\t [(1=left, 2=right)]\n\t [start index]\n\t [count]"},
#endif
  { NULL,   NULL,        NULL,             NULL },
};

static const struct fast_cmdmap_s g_fast_write[] =
{
  { "help",     do_write_help,        "",  NULL},
  { "tile",     do_write_tile,       "fast write tile", "\n\t [row]\n\t [column]\n\t [data<hex>]"},
#if defined(CONFIG_ZEUS2)
  { "reg",      do_write_reg,        "fast write reg",  "\n\t [addr<hex>]\n\t [data<hex>]"},
  { "otp",      do_write_otp,        "fast write otp",  "\n\t [(1=fast otp, 2=customer otp)]\n\t [start addr<hex>]\n\t [16-bit data<hex>]"},
  { zeus2_tileio_str,       do_write_peripheral,       "fast write tileio", "\n\t [(1=left, 2=right)]\n\t [index]\n\t [data<hex>]"},
  { zeus2_vrail_str,    do_write_peripheral,       "fast write vrail", "\n\t [(1=left, 2=right)]\n\t [index]\n\t [data<hex>]"},
  { zeus2_rdac_str,     do_write_peripheral,       "fast write rdac", "\n\t [(1=left, 2=right)]\n\t [index]\n\t [data<hex>]"},
  { zeus2_cdac_str,     do_write_peripheral,       "fast write cdac", "\n\t [index]\n\t [data<hex>]"},
#endif
  { NULL,   NULL,        NULL,             NULL },
};

static const struct fast_cmdmap_s g_fast_clear[] =
{
  { "help",     do_clear_help,        "",  NULL},
  { "all",     do_clear_all,       "fast clear all", NULL},
  { "tile",     do_clear_tile,       "fast clear tile", NULL},
#if defined(CONFIG_ZEUS2)
  { zeus2_tileio_str,       do_clear_peripheral,       "fast clear tileio", "\n\t [(1=left, 2=right)]"},
  { zeus2_vrail_str,    do_clear_peripheral,       "fast clear vrail", "\n\t [(1=left, 2=right)]"},
  { zeus2_rdac_str,     do_clear_peripheral,       "fast clear rdac", "\n\t [(1=left, 2=right)]"},
#endif
  { NULL,   NULL,        NULL,             NULL },
};

extern uint32_t fast_ioctl_arg[FAST_IOCTL_ARG_COUNT];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: do_read_help
 ****************************************************************************/
static int32_t do_read_help(int32_t argc, char **argv)
{
  const struct fast_cmdmap_s *ptr;
  printf("Help: read tile, register, logic translator(lt), Row mux(rmux), Voltage rails(vrails),\n prog. resistors(RDAC) and prog.capacitors(CDAC)\n");
  printf("Usage: fast read <cmd> [arguments]\n");
  printf("Where <cmd> is one of:\n\n");
  for (ptr = (g_fast_read + 1); ptr->cmd; ptr++)
    {
      if (ptr->usage)
        {
          printf("  %s: %s %s\n\n", ptr->cmd, ptr->desc,  ptr->usage);
        }
      else
        {
          printf("  %s: %s\n\n", ptr->cmd, ptr->desc);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: do_scan_help
 ****************************************************************************/
static int32_t do_scan_help(int32_t argc, char **argv)
{
  const struct fast_cmdmap_s *ptr;
  printf("Help: Reads tiles, registers, logic translator(lts), Row muxes(rmux), Voltage rails(vrails),\n prog. resistors(RDAC) and prog.capacitors(CDAC)\n");
  printf("Usage: fast scan <cmd> [arguments]\n");
  printf("Where <cmd> is one of:\n\n");
  for (ptr = (g_fast_scan + 1); ptr->cmd; ptr++)
    {
      if (ptr->usage)
        {
          printf("  %s: %s %s\n\n", ptr->cmd, ptr->desc,  ptr->usage);
        }
      else
        {
          printf("  %s: %s\n\n", ptr->cmd, ptr->desc);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: do_write_help
 ****************************************************************************/
static int32_t do_write_help(int32_t argc, char **argv)
{
  const struct fast_cmdmap_s *ptr;

  printf("Help: Writes tile, register, logic translator(lt), Row mux(rmux), Voltage rails(vrails),\n prog. resistors(RDAC) and prog.capacitors(CDAC)\n");
  printf("Usage: fast write <cmd> [arguments]\n");
  printf("Where <cmd> is one of:\n\n");
  for (ptr = (g_fast_write + 1); ptr->cmd; ptr++)
    {
      if (ptr->usage)
        {
          printf("  %s: %s %s\n\n", ptr->cmd, ptr->desc,  ptr->usage);
        }
      else
        {
          printf("  %s: %s\n\n", ptr->cmd, ptr->desc);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: do_clear_help
 ****************************************************************************/
static int32_t do_clear_help(int32_t argc, char **argv)
{
  const struct fast_cmdmap_s *ptr;

  printf("Help: Clears tiles, logic translators(lt), Row muxes(rmux), Voltage rails(vrails),\n prog. resistors(RDAC) and prog.capacitors(CDAC)\n");
  printf("Usage: fast clear <cmd> [arguments]\n");
  printf("Where <cmd> is one of:\n\n");
  for (ptr = (g_fast_clear + 1); ptr->cmd; ptr++)
    {
      if (ptr->usage)
        {
          printf("  %s: %s %s\n\n", ptr->cmd, ptr->desc,  ptr->usage);
        }
      else
        {
          printf("  %s: %s\n\n", ptr->cmd, ptr->desc);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: do_write_tile
 ****************************************************************************/
static int32_t do_write_tile(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc < 3)
    {
      do_write_help(argc, argv);
      return -EINVAL;
    }
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  fast_ioctl_arg[1] = (uint32_t) strtol(argv[2], NULL, 10);
  fast_ioctl_arg[2] = (uint32_t) strtol(argv[3], NULL, 16);
  ret = send_fast_ioctl(FAST_IOCTL_WRITE_TILE, (ulong_t)fast_ioctl_arg);
  return ret;
}

/****************************************************************************
 * Name: do_read_tile
 ****************************************************************************/
static int32_t do_read_tile(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc < 3)
    {
      do_read_help(argc, argv);
      return -EINVAL;
    }
  memset(fast_ioctl_arg, 0x00, FAST_IOCTL_ARG_COUNT * sizeof(uint32_t));
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  fast_ioctl_arg[1] = (uint32_t) strtol(argv[2], NULL, 10);
  ret = send_fast_ioctl(FAST_IOCTL_READ_TILE, (ulong_t)fast_ioctl_arg);
  if (ret == OK)
    {
      printf("Tile data row: %d column : %d data : 0x%06x\r\n", fast_ioctl_arg[0], fast_ioctl_arg[1],
             (uint32_t)fast_ioctl_arg[2] & TILE_ROUTER_DATA_BIT_MASK);
    }
  return ret;
}

/****************************************************************************
 * Name: do_clear_tile
 ****************************************************************************/
static int32_t do_clear_tile(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc < 1)
    {
      do_clear_help(argc, argv);
      return -EINVAL;
    }
  ret = send_fast_ioctl(FAST_IOCTL_CLEAR_TILE, (ulong_t)fast_ioctl_arg);
  return ret;
}

/****************************************************************************
 * Name: do_clear_all
 ****************************************************************************/
static int32_t do_clear_all(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc < 1)
    {
      do_clear_help(argc, argv);
      return -EINVAL;
    }
  /* clear tiles */
  ret = send_fast_ioctl(FAST_IOCTL_CLEAR_TILE, (ulong_t)fast_ioctl_arg);
  /* clear tileio */
  fast_ioctl_arg[0] = FAST_PERIPHERAL_LEFT_TILEIO;
  ret = send_fast_ioctl(FAST_IOCTL_CLEAR_PERIPHERAL, (ulong_t)fast_ioctl_arg);
  fast_ioctl_arg[0] = FAST_PERIPHERAL_RIGHT_TILEIO;
  ret = send_fast_ioctl(FAST_IOCTL_CLEAR_PERIPHERAL, (ulong_t)fast_ioctl_arg);
  /* clear vrails */
  fast_ioctl_arg[0] = FAST_PERIPHERAL_LEFT_VRAIL;
  ret = send_fast_ioctl(FAST_IOCTL_CLEAR_PERIPHERAL, (ulong_t)fast_ioctl_arg);
  fast_ioctl_arg[0] = FAST_PERIPHERAL_RIGHT_VRAIL;
  ret = send_fast_ioctl(FAST_IOCTL_CLEAR_PERIPHERAL, (ulong_t)fast_ioctl_arg);
  /* clear rdacs */
  fast_ioctl_arg[0] = FAST_PERIPHERAL_LEFT_RDAC;
  ret = send_fast_ioctl(FAST_IOCTL_CLEAR_PERIPHERAL, (ulong_t)fast_ioctl_arg);
  fast_ioctl_arg[0] = FAST_PERIPHERAL_RIGHT_RDAC;
  ret = send_fast_ioctl(FAST_IOCTL_CLEAR_PERIPHERAL, (ulong_t)fast_ioctl_arg);
  /* clear cdac */
  fast_ioctl_arg[0] = FAST_PERIPHERAL_CDAC;
  ret = send_fast_ioctl(FAST_IOCTL_CLEAR_PERIPHERAL, (ulong_t)fast_ioctl_arg);
  return ret;
}

#if defined(CONFIG_ZEUS2)
/****************************************************************************
 * Name: do_read_reg
 ****************************************************************************/
static int32_t do_read_reg(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc < 2)
    {
      do_read_help(argc, argv);
      return -EINVAL;
    }
  memset(fast_ioctl_arg, 0x00, FAST_IOCTL_ARG_COUNT * sizeof(uint32_t));
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 16);
  ret = send_fast_ioctl(FAST_IOCTL_READ_REG, (ulong_t) fast_ioctl_arg);
  if (ret == 0)
    {
      printf("\tFAST Reg read Addr : 0x%05x\t Data : 0x%04x\r\n", (uint32_t)fast_ioctl_arg[0], (uint16_t)fast_ioctl_arg[1]);
    }
  return ret;
}

/****************************************************************************
 * Name: do_write_reg
 ****************************************************************************/
static int32_t do_write_reg(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc < 3)
    {
      do_write_help(argc, argv);
      return -EINVAL;
    }
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 16);
  fast_ioctl_arg[1] = (uint32_t) strtol(argv[2], NULL, 16);
  ret = send_fast_ioctl(FAST_IOCTL_WRITE_REG, (ulong_t)fast_ioctl_arg);
  return ret;
}


/****************************************************************************
 * Name: calc_peripheral_id
 ****************************************************************************/
static fast_peripheral_id_t calc_peripheral_id(char *peripheral_string, uint32_t left_peripheral)
{
  if (strcmp(peripheral_string, zeus2_tileio_str) == 0)
    {
      if (left_peripheral == ZEUS2_LEFT_PERIPHERAL)
        {
          return FAST_PERIPHERAL_LEFT_TILEIO;
        }
      else if (left_peripheral == ZEUS2_RIGHT_PERIPHERAL)
        {
          return FAST_PERIPHERAL_RIGHT_TILEIO;
        }
      else
        {
          return FAST_PERIPHERAL_MAX;
        }
    }
  else if (strcmp(peripheral_string, zeus2_vrail_str) == 0)
    {
      if (left_peripheral == ZEUS2_LEFT_PERIPHERAL)
        {
          return FAST_PERIPHERAL_LEFT_VRAIL;
        }
      else if (left_peripheral == ZEUS2_RIGHT_PERIPHERAL)
        {
          return FAST_PERIPHERAL_RIGHT_VRAIL;
        }
      else
        {
          return FAST_PERIPHERAL_MAX;
        }
    }
  else if (strcmp(peripheral_string, zeus2_rdac_str) == 0)
    {
      if (left_peripheral == ZEUS2_LEFT_PERIPHERAL)
        {
          return FAST_PERIPHERAL_LEFT_RDAC;
        }
      else if (left_peripheral == ZEUS2_RIGHT_PERIPHERAL)
        {
          return FAST_PERIPHERAL_RIGHT_RDAC;
        }
      else
        {
          return FAST_PERIPHERAL_MAX;
        }
    }
  else if (strcmp(peripheral_string, zeus2_cdac_str) == 0)
    {
      return FAST_PERIPHERAL_CDAC;
    }
  return FAST_PERIPHERAL_MAX;
}

/****************************************************************************
 * Name: do_write_peripheral
 ****************************************************************************/
static int32_t do_write_peripheral(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc < 4)
    {
      do_write_help(argc, argv);
      return -EINVAL;
    }
  uint32_t left_peripheral = (uint32_t) strtol(argv[1], NULL, 10);
  fast_ioctl_arg[1] = (uint32_t) strtol(argv[2], NULL, 10);
  fast_ioctl_arg[2] = (uint32_t) strtol(argv[3], NULL, 16);
  char *peripheral = argv[0];
  fast_ioctl_arg[0] = calc_peripheral_id(peripheral, left_peripheral);
  ret = send_fast_ioctl(FAST_IOCTL_WRITE_PERIPHERAL, (ulong_t)fast_ioctl_arg);
  return ret;
}

/****************************************************************************
 * Name: do_clear_peripheral
 ****************************************************************************/
static int32_t do_clear_peripheral(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc < 1)
    {
      do_clear_help(argc, argv);
      return -EINVAL;
    }
  uint32_t left_peripheral = (uint32_t) strtol(argv[1], NULL, 10);
  char *peripheral = argv[0];
  fast_ioctl_arg[0] = calc_peripheral_id(peripheral, left_peripheral);
  ret = send_fast_ioctl(FAST_IOCTL_CLEAR_PERIPHERAL, (ulong_t)fast_ioctl_arg);
  return ret;
}

/****************************************************************************
 * Name: do_read_peripheral
 ****************************************************************************/
static int32_t do_read_peripheral(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc < 3)
    {
      do_read_help(argc, argv);
      return -EINVAL;
    }
  memset(fast_ioctl_arg, 0x00, FAST_IOCTL_ARG_COUNT * sizeof(uint32_t));
  char *peripheral = argv[0];
  uint32_t left_peripheral = (uint32_t) strtol(argv[1], NULL, 10);
  fast_ioctl_arg[1] = (uint32_t) strtol(argv[2], NULL, 10);
  fast_ioctl_arg[0] = calc_peripheral_id(peripheral, left_peripheral);
  ret = send_fast_ioctl(FAST_IOCTL_READ_PERIPHERAL, (ulong_t)fast_ioctl_arg);
  if (ret == 0)
    {
      printf("\tFAST peripheral : %s index : %d data : 0x%02x\r\n", peripheral, fast_ioctl_arg[1], (uint8_t)fast_ioctl_arg[2]);
    }
  return ret;
}

/****************************************************************************
 * Name: do_scan_peripheral
 ****************************************************************************/
static int32_t do_scan_peripheral(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc < 3)
    {
      do_scan_help(argc, argv);
      return -EINVAL;
    }
  char *peripheral = argv[0];
  uint8_t peri_start_index = 0, peri_count = 0, index = 0, i = 0;
  uint32_t *peri_data;
  uint32_t left_peripheral = (uint32_t) strtol(argv[1], NULL, 10);
  fast_ioctl_arg[1] = (uint32_t) strtol(argv[2], NULL, 10);
  fast_ioctl_arg[2] = (uint32_t) strtol(argv[3], NULL, 10);
  peri_start_index = fast_ioctl_arg[1];
  peri_count = fast_ioctl_arg[2];
  if (peri_count == 0)
    {
      return FAST_INVALID_ARGUMENT;
    }
  peri_data = (uint32_t *)malloc(peri_count * sizeof(uint32_t));
  memset(peri_data, 0x00, peri_count * sizeof(uint32_t));
  fast_ioctl_arg[3] = (uint32_t)peri_data;
  fast_ioctl_arg[0] = calc_peripheral_id(peripheral, left_peripheral);
  ret = send_fast_ioctl(FAST_IOCTL_SCAN_PERIPHERAL, (ulong_t)fast_ioctl_arg);
  for (i = peri_start_index ; i < peri_count; i++)
    {
      printf("\tPeripheral data index : %d data : 0x%02x\r\n", i, (uint8_t)peri_data[index]);
      index++;
    }
  free(peri_data);
  return ret;
}


/****************************************************************************
 * Name: DumpHex
 ****************************************************************************/
void dumphex(const void *data, uint32_t start_addr, size_t size)
{
  char ascii[17];
  size_t i, j;
  ascii[16] = '\0';
  for (i = 0; i < size; ++i)
    {
      if ( i == 0)
        {
          printf("\n%05x : ", start_addr + i);
        }
      printf("%02X ", ((unsigned char *)data)[i]);

      if (((unsigned char *)data)[i] >= ' ' && ((unsigned char *)data)[i] <= '~')
        {
          ascii[i % 16] = ((unsigned char *)data)[i];
        }
      else
        {
          ascii[i % 16] = '.';
        }
      if ((i + 1) % 8 == 0 || i + 1 == size)
        {
          printf(" ");
          if ((i + 1) % 16 == 0)
            {
              printf("|  %s \n", ascii);
              if ((i + 1) != size)
                {
                  printf("%05x : ", start_addr + i + 1);
                }
            }
          else if (i + 1 == size)
            {
              ascii[(i + 1) % 16] = '\0';
              if ((i + 1) % 16 <= 8)
                {
                  printf(" ");
                }
              for (j = (i + 1) % 16; j < 16; ++j)
                {
                  printf("   ");
                }
              printf("|  %s \n", ascii);
            }
        }
    }
}


/****************************************************************************
 * Name: do_scan_otp
 ****************************************************************************/
static int32_t do_scan_otp(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc < 4)
    {
      do_scan_help(argc, argv);
      return -EINVAL;
    }
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  fast_ioctl_arg[1] = (uint32_t) strtol(argv[2], NULL, 16);
  fast_ioctl_arg[2] = (uint32_t) strtol(argv[3], NULL, 10);
  uint16_t *otp_data = (uint16_t *)malloc(fast_ioctl_arg[2]);
  memset((uint8_t *)otp_data, 0x00, fast_ioctl_arg[2]);
  fast_ioctl_arg[3] = (uint32_t)otp_data;
  if (fast_ioctl_arg[0] == 1)
    {
      ret = send_fast_ioctl(FAST_IOCTL_READ_FAST_OTP, (ulong_t)fast_ioctl_arg);
    }
  else if (fast_ioctl_arg[0] == 2)
    {
      ret = send_fast_ioctl(FAST_IOCTL_READ_CUSTOMER_OTP, (ulong_t)fast_ioctl_arg);
    }
  dumphex(otp_data, fast_ioctl_arg[1], fast_ioctl_arg[2]);
  free(otp_data);
  return ret;
}

/****************************************************************************
 * Name: do_write_otp
 ****************************************************************************/
static int32_t do_write_otp(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc != 4)
    {
      do_scan_help(argc, argv);
      return -EINVAL;
    }
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  fast_ioctl_arg[1] = (uint32_t) strtol(argv[2], NULL, 16); //start_addr
  fast_ioctl_arg[2] = 1;                                    //number_of_bytes
  uint16_t *otp_data = (uint16_t *)malloc(fast_ioctl_arg[2]);
  *otp_data = (uint32_t) strtol(argv[3], NULL, 16);
  fast_ioctl_arg[3] = (uint32_t)otp_data;                   //data_ptr
  if (fast_ioctl_arg[0] == 1)
    {
      ret = send_fast_ioctl(FAST_IOCTL_WRITE_FAST_OTP, (ulong_t)fast_ioctl_arg);
    }
  else if (fast_ioctl_arg[0] == 2)
    {
      ret = send_fast_ioctl(FAST_IOCTL_WRITE_CUSTOMER_OTP, (ulong_t)fast_ioctl_arg);
    }
  free(otp_data);
  return ret;
}

/****************************************************************************
 * Name: do_scan_regs
 ****************************************************************************/
static int32_t do_scan_regs(int32_t argc, char **argv)
{
  int32_t ret = -1;
  ret = send_fast_ioctl(FAST_IOCTL_SCAN_REGS, (ulong_t)fast_ioctl_arg);
  return ret;
}

#endif //#if defined(CONFIG_ZEUS2)

/****************************************************************************
 * Name: do_scan_tile
 ****************************************************************************/
static int32_t do_scan_tile(int32_t argc, char **argv)
{
  int32_t ret = -1;
  if (argc < 5)
    {
      do_scan_help(argc, argv);
      return -EINVAL;
    }
  fast_ioctl_arg[0] = (uint32_t) strtol(argv[1], NULL, 10);
  fast_ioctl_arg[1] = (uint32_t) strtol(argv[2], NULL, 10);
  fast_ioctl_arg[2] = (uint32_t) strtol(argv[3], NULL, 10);
  fast_ioctl_arg[3] = (uint32_t) strtol(argv[4], NULL, 10);
  uint8_t start_row = 0, start_col = 0, end_row = 0, end_col = 0, i = 0, j = 0;
  uint32_t index = 0;
  start_row = fast_ioctl_arg[0];
  start_col = fast_ioctl_arg[1];
  end_row = fast_ioctl_arg[2];
  end_col = fast_ioctl_arg[3];
  uint32_t size = ((end_row - start_row) * (end_col - start_col));
  if (size <= 0)
    {
      return FAST_INVALID_ARGUMENT;
    }
  uint32_t *scantile_data = (uint32_t *)malloc(size * sizeof(uint32_t));
  memset(scantile_data, 0x00, size * sizeof(uint32_t));
  fast_ioctl_arg[4] = (uint32_t)scantile_data;
  ret = send_fast_ioctl(FAST_IOCTL_SCANTILE, (ulong_t)fast_ioctl_arg);
  if (ret == OK)
    {
      for (i = start_row; i < end_row; i++)
        {
          for (j = start_col; j < end_col; j++)
            {
              if (scantile_data[index] != 0)
                {
                  printf("Tile data row: %d column : %d data : 0x%06x\r\n", i, j, (scantile_data[index] & TILE_ROUTER_DATA_BIT_MASK));
                }
              index++;
            }
        }
    }
  free(scantile_data);
  return ret;
}


/****************************************************************************
 * Name: fast_read
 ****************************************************************************/
int32_t fast_read(int32_t argc, FAR char **argv)
{
  int32_t ret = -1;
  set_errno(0);
  dbg_tag();
  if (argc == 1)
    {
      do_read_help(argc, argv);
      return -EINVAL;
    }
  ret = fast_parse(g_fast_read, argc, argv);
  return ret;
}

/****************************************************************************
 * Name: fast_write
 ****************************************************************************/
int32_t fast_write(int32_t argc, FAR char **argv)
{
  int32_t ret = -1;
  set_errno(0);
  dbg_tag();
  if (argc == 1)
    {
      do_write_help(argc, argv);
      return -EINVAL;
    }
  ret = fast_parse(g_fast_write, argc, argv);
  return ret;
}

/****************************************************************************
 * Name: fast_scan
 ****************************************************************************/
int32_t fast_scan(int32_t argc, FAR char **argv)
{
  int32_t ret = -1;
  set_errno(0);
  dbg_tag();
  if (argc == 1)
    {
      do_scan_help(argc, argv);
      return -EINVAL;
    }
  ret = fast_parse(g_fast_scan, argc, argv);
  return ret;
}

/****************************************************************************
 * Name: fast_clear
 ****************************************************************************/
int32_t fast_clear(int32_t argc, FAR char **argv)
{
  int32_t ret = -1;
  set_errno(0);
  dbg_tag();
  if (argc == 1)
    {
      do_clear_help(argc, argv);
      return -EINVAL;
    }
  ret = fast_parse(g_fast_clear, argc, argv);
  return ret;
}


