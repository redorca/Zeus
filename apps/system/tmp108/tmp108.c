/****************************************************************************
 * apps/system/tmp108/tmp108.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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
#include <fcntl.h>
#include <errno.h>
#include <fixedmath.h>

#include <nuttx/sensors/tmp108.h>

#include "nrf52_i2c.h"
#include "nrf52_gpio.h"
#include "nrf52_gpiote.h"
#include "nrf_gpio.h"
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SYSTEM_TMP108_DEVNAME
#  warning CONFIG_SYSTEM_TMP108_DEVNAME is not defined
#  define CONFIG_SYSTEM_TMP108_DEVNAME "/dev/temp0"
#endif

#if !defined(CONFIG_SYSTEM_TMP108_FAHRENHEIT) && !defined(CONFIG_SYSTEM_TMP108_CELSIUS)
#  warning one of CONFIG_SYSTEM_TMP108_FAHRENHEIT or CONFIG_SYSTEM_TMP108_CELSIUS must be defined
#  defeind CONFIG_SYSTEM_TMP108_FAHRENHEIT 1
#endif

#if defined(CONFIG_SYSTEM_TMP108_FAHRENHEIT) && defined(CONFIG_SYSTEM_TMP108_CELSIUS)
#  warning both of CONFIG_SYSTEM_TMP108_FAHRENHEIT and CONFIG_SYSTEM_TMP108_CELSIUS defined
#  undef CONFIG_SYSTEM_TMP108_CELSIUS
#endif

/*Set defualt config, compare mode, CR is 1Hz
 *
 *BYTE 1
 *D7  D6   D5   D4   D3   D2   D1   D0
 *ID  CR1  CR0  FH   FL   TM   M1   M0
 * 0    1    1   0    0    0    1    0
 *
 *BYTE 2
 *POL --   HYS1 HYS0  --   --   --   --
 * 0   0      0    1  0     0    0    0
 */
#define TMP108_CONFIG_DEFAULT     (0x1062)

#define TMP108_FL_BIT             (0x08)
#define TMP108_FL_SHIF            (3)
#define TMP108_FL_TIMEOUT         (10)
#define TMP108_SAMPLE_DELAY       (100000) /*Uint us*/

#define TMP108_CR_BIT             (0x60)
#define TMP108_CR_SHIFT           (5)
#define TMP108_CR_MIN_LEVEL       (0)
#define TMP108_CR_MAX_LEVEL       (3)


/****************************************************************************
 * Public Definitions
 ****************************************************************************/
extern int up_putc(int ch);

extern ret_code_t nrf_drv_gpiote_in_init(nrf_drv_gpiote_pin_t               pin,
                                         nrf_drv_gpiote_in_config_t const *p_config,
                                         nrf_drv_gpiote_evt_handler_t       evt_handler);

extern void nrf_drv_gpiote_in_event_enable(nrf_drv_gpiote_pin_t pin, bool int_enable);
/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_samples = 1;

static int temp_fd;

static int temp_STL_status = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tmp108_help
 ****************************************************************************/

#ifdef CONFIG_NSH_BUILTIN_APPS
static void tmp108_help(void)
{
  syslog(LOG_INFO, "Usage: temp [OPTIONS]\n");
  syslog(LOG_INFO, "  [-n count] selects the samples to collect.  "
         "Default: 1 Current: %d\n", g_samples);
  syslog(LOG_INFO, "  [-c value] Set conversion rate value 0~3: 0 0.25HZ, 1 1HZ, 2 4HZ, 3 16HZ.\n");
  syslog(LOG_INFO, "  [-s] Do system level test.\n");
  syslog(LOG_INFO, "  [-r] Read TMP108 config vaule.\n");
  syslog(LOG_INFO, "  [-h] shows this message and exits\n");
}
#endif

/****************************************************************************
 * Name: arg_string
 ****************************************************************************/

#ifdef CONFIG_NSH_BUILTIN_APPS
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
#endif

/****************************************************************************
 * Name: arg_decimal
 ****************************************************************************/

#ifdef CONFIG_NSH_BUILTIN_APPS
static int arg_decimal(FAR char **arg, FAR long *value)
{
  FAR char *string;
  int ret;

  ret = arg_string(arg, &string);
  *value = strtol(string, NULL, 10);
  return ret;
}
#endif

void tmp108_INT_signo(int signo, FAR siginfo_t *siginfo, FAR void *context)
{
  temp_STL_status = true;
}

static int temp_SLT_test(void)
{
  uint16_t config_val = 0;
  uint32_t reg = 0, i = 0;
  int32_t threshold_val;
  int32_t ret;

  struct sigaction   act;

  act.sa_sigaction = tmp108_INT_signo;
  act.sa_flags  = SA_SIGINFO;

  (void)sigemptyset(&act.sa_mask);
  (void)sigaddset(&act.sa_mask, SIGUSR1);

  ret = sigaction(SIGUSR1, &act, NULL);
  if (ret != OK)
    {
      syslog(LOG_ERR, "Tmp108 SLT test: ERROR sigaction failed, ret=%d\n", ret);
    }

  temp_STL_status = 0;

  syslog(LOG_INFO, "TMP108 SLT TEST START-----\n\n");

  ioctl(temp_fd, SNIOC_GA_REGISTER_INT, (long unsigned int)SIGUSR1);

  read(temp_fd, &reg, sizeof(int32_t));
  syslog(LOG_INFO, "1. Current TEMP is %.02f Celsius degree.\n\n", ((double)reg / 16.0));
  reg = 0;

  config_val = TMP108_CONFIG_DEFAULT;
  ioctl(temp_fd, SNIOC_WRITECONF, (long unsigned int)config_val);
  ioctl(temp_fd, SNIOC_READCONF, (long unsigned int)&reg);
  syslog(LOG_INFO, "2. Set compare mode to enable interrupt: Val = [0x%04x].\n\n", reg);
  reg = 0;

  /*2. Set LOW TEMP Threshold to 100 to trigger a INT*/
  threshold_val = 0x640000;
  ioctl(temp_fd, SNIOC_WRITETLOW, (long unsigned int)threshold_val);
  ioctl(temp_fd, SNIOC_READTLOW, (long unsigned int)&reg);
  syslog(LOG_INFO, "3. Set LOW temperature threshold as 100 to generate interruptz: Val = [0x%08x].\n\n", reg);
  usleep(TMP108_SAMPLE_DELAY);
  reg = 0;

  for (i = 0; i < TMP108_FL_TIMEOUT ; i++)
    {
      ioctl(temp_fd, SNIOC_READCONF, (long unsigned int)&reg);
      if (reg & TMP108_FL_BIT)
        {
          break;
        }
      else
        {
          if (i == (TMP108_FL_TIMEOUT - 1))
            {
              syslog(LOG_INFO, "Wait for INT %d times, timeout...\n", i);
            }
        }
    };

  ioctl(temp_fd, SNIOC_READCONF, (long unsigned int)&reg);
  syslog(LOG_INFO, "4. Read INT register status: Val = [0x%04x].\n\n", reg);
  reg = 0;

  if (temp_STL_status)
    {
      syslog(LOG_WARNING, "5. TMP108 SLT INT test passed...\n\n");
    }
  else
    {
      syslog(LOG_ERR, "5. TMP108 SLT INT test failed, alert pin not connected...\n\n");
    }

  /*3. Set LOW TEMP Threshold to 0 to clear INT*/
  threshold_val = 0;
  ioctl(temp_fd, SNIOC_WRITETLOW, (long unsigned int)threshold_val);
  usleep(TMP108_SAMPLE_DELAY);

  for (i = 0; i < TMP108_FL_TIMEOUT ; i++)
    {
      ioctl(temp_fd, SNIOC_READCONF, (long unsigned int)&reg);
      if (reg & TMP108_FL_BIT)
        {
          if (i == (TMP108_FL_TIMEOUT - 1))
            {
              syslog(LOG_INFO, "Wait for clear INT %d times, timeout...\n", i);
            }
        }
      else
        {
          break;
        }
    };

  syslog(LOG_INFO, "\nTMP108 SLT TEST COMPLETE-----\n\n");
  return 0;
}
/****************************************************************************
 * Name: parse_args
 ****************************************************************************/

#ifdef CONFIG_NSH_BUILTIN_APPS
static void parse_args(int argc, FAR char **argv)
{
  FAR char *ptr;
  long value;
  int index, nargs, ret;
  uint16_t reg = 0;

  for (index = 1; index < argc; )
    {
      ptr = argv[index];
      if (ptr[0] != '-')
        {
          syslog(LOG_INFO, "Invalid options format: %s\n", ptr);
          exit(0);
        }

      switch (ptr[1])
        {
          case 'n':
            nargs = arg_decimal(&argv[index], &value);
            if (value < 0)
              {
                syslog(LOG_INFO, "Count must be non-negative: %ld\n", value);
                exit(1);
              }

            g_samples = (int)value;
            index += nargs;
            break;

          case 'h':
            tmp108_help();
            exit(0);

          case 's':
            temp_SLT_test();
            exit(1);

          case 'r':
            {
              reg = 0;
              ret = ioctl(temp_fd, SNIOC_READCONF, (long unsigned int)&reg);
              if (ret < 0)
                {
                  syslog(LOG_ERR, "TMP108: SNIOC_READCONF ioctl failed: %d\n", errno);
                  exit(0);
                }
              syslog(LOG_INFO, "TMP108: Config value = %4x \n", reg);
              exit(1);
            }
          case 'c':
            {
              uint16_t c_rate = 0;
              reg = 0;
              arg_decimal(&argv[index], &value);
              if (value < 0 || value > 3)
                {
                  syslog(LOG_WARNING, "Conversion rate only support 0~3 level, %ld\n", value);
                  exit(0);
                }
              else
                {
                  syslog(LOG_INFO, "Conversion rate value is %ld\n", value);
                }

              ret = ioctl(temp_fd, SNIOC_READCONF, (long unsigned int)&reg);
              if (ret < 0)
                {
                  syslog(LOG_ERR, "TMP108: SNIOC_READCONF ioctl failed: %d\n", errno);
                  exit(0);
                }
              c_rate = (uint16_t)reg;
              c_rate = (reg & (~TMP108_CR_BIT)) | (((uint16_t)value) << TMP108_CR_SHIFT);
              syslog(LOG_INFO, "TMP108: New configvalue = %4x \n", c_rate);

              ret = ioctl(temp_fd, SNIOC_WRITECONF, (long unsigned int)c_rate);
              if (ret < 0)
                {
                  syslog(LOG_ERR, "TMP108: SNIOC_WRITECONF ioctl failed: %d\n", errno);
                  exit(0);
                }
              exit(1);
            }

          default:
            syslog(LOG_INFO, "Unsupported option: %s\n", ptr);
            tmp108_help();
            exit(1);
        }
    }
}
#endif


/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int tmp108_main(int argc, char *argv[])
#endif
{
#ifdef CONFIG_LIBC_FLOATINGPOINT
  double temp;
#endif
  b16_t temp16;
  ssize_t nbytes;
  int ret;
  int i;

  /* Open the temperature sensor device */

  temp_fd  = open(CONFIG_SYSTEM_TMP108_DEVNAME, O_RDONLY);
  if (temp_fd < 0)
    {
      fprintf(stderr, "ERROR: Failed to open %s: %d\n",
              CONFIG_SYSTEM_TMP108_DEVNAME, errno);
      return EXIT_FAILURE;
    }

#ifdef CONFIG_SYSTEM_TMP108_FAHRENHEIT
  /* Select Fahrenheit scaling */

  ret = ioctl(temp_fd, SNIOC_FAHRENHEIT, 0);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: ioctl(SNIOC_FAHRENHEIT) failed: %d\n",
              errno);
      return EXIT_FAILURE;
    }
#else
  /* Select Fahrenheit scaling */

  ret = ioctl(temp_fd, SNIOC_CENTIGRADE, 0);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: ioctl(SNIOC_CENTIGRADE) failed: %d\n",
              errno);
      return EXIT_FAILURE;
    }
#endif

  /* Parse the command line */

#ifdef CONFIG_NSH_BUILTIN_APPS
  parse_args(argc, argv);
#endif

  for (i = 0; i < g_samples; i++)
    {
      /* Read the current temperature as a fixed precision number */

      nbytes = read(temp_fd, &temp16, sizeof(b16_t));

      if (nbytes < 0)
        {
          fprintf(stderr, "ERROR: read(%d) failed: %d\n",
                  sizeof(b16_t), errno);
          return EXIT_FAILURE;
        }

      if (nbytes != sizeof(b16_t))
        {
          fprintf(stderr, "ERROR: Unexpected read size: %Ld vs %d\n",
                  (long)nbytes, sizeof(b16_t));
          return EXIT_FAILURE;
        }

      /* Print the current temperature on stdout */

#ifdef CONFIG_LIBC_FLOATINGPOINT
      temp = (double)temp16 / 16.0;
#  ifdef CONFIG_SYSTEM_TMP108_FAHRENHEIT
      temp = ((temp * 1.8) + 32);
      syslog(LOG_INFO, "%3.2f degrees Fahrenheit\n", temp);
#  else
      syslog(LOG_INFO, "%3.2f degrees Celsius\n", temp);
#  endif
#else
#  ifdef CONFIG_SYSTEM_TMP108_FAHRENHEIT
      syslog(LOG_INFO, "0x%04x.%04x degrees Fahrenheit\n", temp16 >> 16, temp16 & 0xffff);
#  else
      syslog(LOG_INFO, "0x%04x.%04x degrees Celsius\n", temp16 >> 16, temp16 & 0xffff);
#  endif
#endif

      /* Force it to print out the lines */

      fflush(stdout);

      /* Wait 100 ms */

      usleep(100000);
    }

  close(temp_fd);

  return EXIT_SUCCESS;
}
