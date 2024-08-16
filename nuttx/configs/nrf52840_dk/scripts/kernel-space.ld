/****************************************************************************
 * configs/nrf52840_dk/scripts/kernel-space.ld
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

/* NOTE:  This depends on the memory.ld script having been included prior to
 * this script.
 */

#include <nuttx/config.h>
#include "memory.ld"

OUTPUT_ARCH(arm)
ENTRY(_stext)
SECTIONS
{
    .text : {
        _stext = ABSOLUTE(.);
        *(.vectors)
        PROVIDE(_start_sched = .);
          KEEP(*libsched.a:*(.text .text.*))
          KEEP(*libsched.a:*(.rodata .rodata.*))
        PROVIDE(_end_sched = .);

        PROVIDE(_start_mm = .);
          KEEP(*libmm.a:*(.text .text.*))
          KEEP(*libmm.a:*(.rodata .rodata.*))
        PROVIDE(_end_mm = .);

        PROVIDE(_start_c_cxx = .);
          KEEP(*libc.a:*(.text .text.*))
          KEEP(*libc.a:*(.rodata .rodata.*))
          KEEP(*libcxx.a:*(.text .text.*))
          KEEP(*libcxx.a:*(.rodata .rodata.*))
        PROVIDE(_end_c_cxx = .);

        PROVIDE(_start_fs = .);
          KEEP(*libfs.a:*(.text .text.*))
          KEEP(*libfs.a:*(.rodata .rodata.*))
        PROVIDE(_end_fs = .);

        PROVIDE(_start_drivers = .);
          KEEP(*libdrivers.a:*(.text .text.*))
          KEEP(*libdrivers.a:*(.rodata .rodata.*))
          KEEP(*libbinfmt.a:*(.text .text.*))
          KEEP(*libbinfmt.a:*(.rodata .rodata.*))
        PROVIDE(_end_drivers = .);

        PROVIDE(_start_soc = .);
          KEEP(*libarch.a:*(.text .text.*))
          KEEP(*libarch.a:*(.rodata .rodata.*))
          KEEP(*libconfigs.a:*(.text .text.*))
          KEEP(*libconfigs.a:*(.rodata .rodata.*))
          KEEP(*libboard.a:*(.text .text.*))
          KEEP(*libboard.a:*(.rodata .rodata.*))
        PROVIDE(_end_soc = .);

        PROVIDE(_start_apps = .);
          KEEP(*libapps.a:*(.text .text.*))
          KEEP(*libapps.a:*(.rodata .rodata.*))
        PROVIDE(_end_apps = .);

        PROVIDE(_start_others = .);
          *(.text .text.*)
          *(.rodata .rodata.*)
        PROVIDE(_end_others = .);

        *(.fixup)
        *(.gnu.warning)
        *(.gnu.linkonce.t.*)
        *(.glue_7)
        *(.glue_7t)
        *(.got)
        *(.gcc_except_table)
        *(.gnu.linkonce.r.*)
        _etext = ABSOLUTE(.);
    } > kflash

    .init_section : {
        _sinit = ABSOLUTE(.);
        KEEP(*(.init_array .init_array.*))
        _einit = ABSOLUTE(.);
    } > kflash

    .ARM.extab : {
        *(.ARM.extab*)
    } > kflash

    __exidx_start = ABSOLUTE(.);
    .ARM.exidx : {
        *(.ARM.exidx*)
    } > kflash

    __exidx_end = ABSOLUTE(.);

    _eronly = ABSOLUTE(.);

    .data : {
        __data_start__ = .;
        _sdata = ABSOLUTE(.);

        PROVIDE(_start_sched_data = .);
            KEEP(*libsched.a:*(.data .data.*))
        PROVIDE(_end_sched_data = .);

        PROVIDE(_start_mm_data = .);
          KEEP(*libmm.a:*(.data .data.*))
        PROVIDE(_end_mm_data = .);

        PROVIDE(_start_drivers_data = .);
          KEEP(*libdrivers.a:*(.data .data.*))
          KEEP(*libbinfmt.a:*(.data .data.*))
        PROVIDE(_end_drivers_data = .);

        PROVIDE(_start_c_cxx_data = .);
          KEEP(*libc.a:*(.data .data.*))
          KEEP(*libcxx.a:*(.data .data.*))
        PROVIDE(_end_c_cxx_data = .);

        PROVIDE(_start_fs_data = .);
          KEEP(*libfs.a:*(.data .data.*))
        PROVIDE(_end_fs_data = .);

        PROVIDE(_start_soc_data = .);
          KEEP(*libarch.a:*(.data .data.*))
          KEEP(*libconfigs.a:*(.data .data.*))
          KEEP(*libboard.a:*(.data .data.*))
        PROVIDE(_end_soc_data = .);

        PROVIDE(_start_apps_data = .);
          KEEP(*libapps.a:*(.data .data.*))
        PROVIDE(_end_apps_data = .);

        PROVIDE(_start_others_data = .);
          *(.data .data.*)
        PROVIDE(_end_others_data = .);

        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        _edata = ABSOLUTE(.);
        __data_end__ = .;
    } > ksram AT > kflash

    .bss : {
        _sbss = ABSOLUTE(.);

        PROVIDE(_start_sched_bss = .);
          KEEP(*libsched.a:*(.bss .bss.*))
        PROVIDE(_end_sched_bss = .);

        PROVIDE(_start_mm_bss = .);
          KEEP(*libmm.a:*(.bss .bss.*))
        PROVIDE(_end_mm_bss = .);

        PROVIDE(_start_drivers_bss = .);
          KEEP(*libdrivers.a:*(.bss .bss.*))
          KEEP(*libbinfmt.a:*(.bss .bss.*))
        PROVIDE(_end_drivers_bss = .);

        PROVIDE(_start_c_cxx_bss = .);
          KEEP(*libc.a:*(.bss .bss.*))
          KEEP(*libcxx.a:*(.bss .bss.*))
        PROVIDE(_end_c_cxx_bss = .);

        PROVIDE(_start_fs_bss = .);
          KEEP(*libfs.a:*(.bss .bss.*))
        PROVIDE(_end_fs_bss = .);

        PROVIDE(_start_soc_bss = .);
          KEEP(*libarch.a:*(.bss .bss.*))
          KEEP(*libconfigs.a:*(.bss .bss.*))
        PROVIDE(_end_soc_bss = .);

        PROVIDE(_start_apps_bss = .);
          KEEP(*libapps.a:*(.bss .bss.*))
        PROVIDE(_end_apps_bss = .);

        PROVIDE(_start_others_bss = .);
          *(.bss .bss.*)
        PROVIDE(_end_others_bss = .);

        *(COMMON)
        *(.gnu.linkonce.b.*)
        _ebss = ABSOLUTE(.);
    } > ksram
        /* make sure the kernel data sectio is enough for data + idle stack */

    ASSERT(LENGTH(ksram) >= (SIZEOF(.data) + SIZEOF(.bss) + CONFIG_IDLETHREAD_STACKSIZE + 16),
          "Kernel Ram is Not enough for idle stack")

    /* Stabs debugging sections */

    .stab 0 : { *(.stab) }
    .stabstr 0 : { *(.stabstr) }
    .stab.excl 0 : { *(.stab.excl) }
    .stab.exclstr 0 : { *(.stab.exclstr) }
    .stab.index 0 : { *(.stab.index) }
    .stab.indexstr 0 : { *(.stab.indexstr) }
    .comment 0 : { *(.comment) }
    .debug_abbrev 0 : { *(.debug_abbrev) }
    .debug_info 0 : { *(.debug_info) }
    .debug_line 0 : { *(.debug_line) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    .debug_aranges 0 : { *(.debug_aranges) }
}

