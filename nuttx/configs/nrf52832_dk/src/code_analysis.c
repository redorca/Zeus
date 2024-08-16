
/****************************************************************************
 * configs/nrf52832_dk/src/code_analysis.c
 *
 *   Copyright (C) 2017 Zglue  Inc. All rights reserved.
 *   Author: Levin Li <zhiqiang@zglue.com>
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
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <debug.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/
struct code_analysis
{
  char      *module;
  uint32_t  type;
  uint32_t  start;
  uint32_t  end;
};

enum
{
  CODE_INFO_TEXT = 0,
  CODE_INFO_DATA,
  CODE_INFO_BSS
} CODE_INFO;

static const char *name_section[] =
{
  "Text",
  "Data",
  "Bss"
};

#define SECTION_TEXT_VARS_END_SYMBOL(section_name)           _end_ ## section_name
#define SECTION_TEXT_VARS_END_ADDR(section_name)         (uint32_t)&SECTION_TEXT_VARS_END_SYMBOL(section_name)

#define SECTION_TEXT_VARS_START_SYMBOL(section_name)           _start_ ## section_name
#define SECTION_TEXT_VARS_START_ADDR(section_name)         (uint32_t)&SECTION_TEXT_VARS_START_SYMBOL(section_name)

#define SECTION_DATA_VARS_END_SYMBOL(section_name)           _end_ ## section_name ## _data
#define SECTION_DATA_VARS_END_ADDR(section_name)         (uint32_t)&SECTION_DATA_VARS_END_SYMBOL(section_name)

#define SECTION_DATA_VARS_START_SYMBOL(section_name)           _start_ ## section_name ## _data
#define SECTION_DATA_VARS_START_ADDR(section_name)         (uint32_t)&SECTION_DATA_VARS_START_SYMBOL(section_name)

#define SECTION_BSS_VARS_END_SYMBOL(section_name)           _end_ ## section_name ## _bss
#define SECTION_BSS_VARS_END_ADDR(section_name)         (uint32_t)&SECTION_BSS_VARS_END_SYMBOL(section_name)

#define SECTION_BSS_VARS_START_SYMBOL(section_name)           _start_ ## section_name ## _bss
#define SECTION_BSS_VARS_START_ADDR(section_name)         (uint32_t)&SECTION_BSS_VARS_START_SYMBOL(section_name)

#define SECTION_VARS_CREATE_SECTION_SYMBOL(section_name) \
  extern uint32_t   SECTION_TEXT_VARS_START_SYMBOL(section_name); \
  extern uint32_t   SECTION_TEXT_VARS_END_SYMBOL(section_name); \
  \
  extern uint32_t   SECTION_DATA_VARS_START_SYMBOL(section_name); \
  extern uint32_t   SECTION_DATA_VARS_END_SYMBOL(section_name); \
  \
  extern uint32_t   SECTION_BSS_VARS_START_SYMBOL(section_name); \
  extern uint32_t   SECTION_BSS_VARS_END_SYMBOL(section_name)

/* define each module symbol variable */

SECTION_VARS_CREATE_SECTION_SYMBOL(sched);
SECTION_VARS_CREATE_SECTION_SYMBOL(mm);
SECTION_VARS_CREATE_SECTION_SYMBOL(c_cxx);
SECTION_VARS_CREATE_SECTION_SYMBOL(fs);
SECTION_VARS_CREATE_SECTION_SYMBOL(drivers);
SECTION_VARS_CREATE_SECTION_SYMBOL(soc);
SECTION_VARS_CREATE_SECTION_SYMBOL(apps);
SECTION_VARS_CREATE_SECTION_SYMBOL(others);

static const struct code_analysis code_info[] =
{
  {
    .module = "Sched",
    .type   = CODE_INFO_TEXT,
    .start  = SECTION_TEXT_VARS_START_ADDR(sched),
    .end    = SECTION_TEXT_VARS_END_ADDR(sched),
  },
  {
    .module = "Sched",
    .type   = CODE_INFO_DATA,
    .start  = SECTION_DATA_VARS_START_ADDR(sched),
    .end    = SECTION_DATA_VARS_END_ADDR(sched),
  },
  {
    .module = "Sched",
    .type   = CODE_INFO_BSS,
    .start  = SECTION_BSS_VARS_START_ADDR(sched),
    .end    = SECTION_BSS_VARS_END_ADDR(sched),
  },

  {
    .module = "MM",
    .type   = CODE_INFO_TEXT,
    .start  = SECTION_TEXT_VARS_START_ADDR(mm),
    .end    = SECTION_TEXT_VARS_END_ADDR(mm),
  },
  {
    .module = "MM",
    .type   = CODE_INFO_DATA,
    .start  = SECTION_DATA_VARS_START_ADDR(mm),
    .end    = SECTION_DATA_VARS_END_ADDR(mm),
  },
  {
    .module = "MM",
    .type   = CODE_INFO_BSS,
    .start  = SECTION_BSS_VARS_START_ADDR(mm),
    .end    = SECTION_BSS_VARS_END_ADDR(mm),
  },

  {
    .module = "Libc",
    .type   = CODE_INFO_TEXT,
    .start  = SECTION_TEXT_VARS_START_ADDR(c_cxx),
    .end    = SECTION_TEXT_VARS_END_ADDR(c_cxx),
  },
  {
    .module = "Libc",
    .type   = CODE_INFO_DATA,
    .start  = SECTION_DATA_VARS_START_ADDR(c_cxx),
    .end    = SECTION_DATA_VARS_END_ADDR(c_cxx),
  },
  {
    .module = "Libc",
    .type   = CODE_INFO_BSS,
    .start  = SECTION_BSS_VARS_START_ADDR(c_cxx),
    .end    = SECTION_BSS_VARS_END_ADDR(c_cxx),
  },

  {
    .module = "FS",
    .type   = CODE_INFO_TEXT,
    .start  = SECTION_TEXT_VARS_START_ADDR(fs),
    .end    = SECTION_TEXT_VARS_END_ADDR(fs),
  },
  {
    .module = "FS",
    .type   = CODE_INFO_DATA,
    .start  = SECTION_DATA_VARS_START_ADDR(fs),
    .end    = SECTION_DATA_VARS_END_ADDR(fs),
  },
  {
    .module = "FS",
    .type   = CODE_INFO_BSS,
    .start  = SECTION_BSS_VARS_START_ADDR(fs),
    .end    = SECTION_BSS_VARS_END_ADDR(fs),
  },

  {
    .module = "Drivers",
    .type   = CODE_INFO_TEXT,
    .start  = SECTION_TEXT_VARS_START_ADDR(drivers),
    .end    = SECTION_TEXT_VARS_END_ADDR(drivers),
  },
  {
    .module = "Drivers",
    .type   = CODE_INFO_DATA,
    .start  = SECTION_DATA_VARS_START_ADDR(drivers),
    .end    = SECTION_DATA_VARS_END_ADDR(drivers),
  },
  {
    .module = "Drivers",
    .type   = CODE_INFO_BSS,
    .start  = SECTION_BSS_VARS_START_ADDR(drivers),
    .end    = SECTION_BSS_VARS_END_ADDR(drivers),
  },

  {
    .module = "SoC",
    .type   = CODE_INFO_TEXT,
    .start  = SECTION_TEXT_VARS_START_ADDR(soc),
    .end    = SECTION_TEXT_VARS_END_ADDR(soc),
  },
  {
    .module = "SoC",
    .type   = CODE_INFO_DATA,
    .start  = SECTION_DATA_VARS_START_ADDR(soc),
    .end    = SECTION_DATA_VARS_END_ADDR(soc),
  },
  {
    .module = "SoC",
    .type   = CODE_INFO_BSS,
    .start  = SECTION_BSS_VARS_START_ADDR(soc),
    .end    = SECTION_BSS_VARS_END_ADDR(soc),
  },

  {
    .module = "Apps",
    .type   = CODE_INFO_TEXT,
    .start  = SECTION_TEXT_VARS_START_ADDR(apps),
    .end    = SECTION_TEXT_VARS_END_ADDR(apps),
  },
  {
    .module = "Apps",
    .type   = CODE_INFO_DATA,
    .start  = SECTION_DATA_VARS_START_ADDR(apps),
    .end    = SECTION_DATA_VARS_END_ADDR(apps),
  },
  {
    .module = "Apps",
    .type   = CODE_INFO_BSS,
    .start  = SECTION_BSS_VARS_START_ADDR(apps),
    .end    = SECTION_BSS_VARS_END_ADDR(apps),
  },

  {
    .module = "Others",
    .type   = CODE_INFO_TEXT,
    .start  = SECTION_TEXT_VARS_START_ADDR(others),
    .end    = SECTION_TEXT_VARS_END_ADDR(others),
  },
  {
    .module = "Others",
    .type   = CODE_INFO_DATA,
    .start  = SECTION_DATA_VARS_START_ADDR(others),
    .end    = SECTION_DATA_VARS_END_ADDR(others),
  },
  {
    .module = "Others",
    .type   = CODE_INFO_BSS,
    .start  = SECTION_BSS_VARS_START_ADDR(others),
    .end    = SECTION_BSS_VARS_END_ADDR(others),
  },

};

int zdk_code_analysis(void)
{

  /* output the text ,data, bss information for each module */
  for (int i = 0; i < sizeof(code_info) / sizeof(struct code_analysis); i++)
    {

      _err("[%s] module, [%s] info: start:0x%08x, end:0x%08x. Size:%08d.\n",
           code_info[i].module, name_section[code_info[i].type],
           code_info[i].start, code_info[i].end, code_info[i].end - code_info[i].start);
    }

  return OK;
}

