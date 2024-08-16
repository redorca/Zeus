############################################################################
#   configs/fast_nrf52832_dk/defaults.mk
#
#   Copyright (C) 2016 Gregory Nutt. All rights reserved.
#   Copyright (C) 2018 Zglue Inc. All rights reserved.
#   Author: bill@zglue.com
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

include $(TOPDIR)/arch/arm/src/nrf52/Make.vars

SCRIPTSDIR = $(TOPDIR)/configs/$(CONFIG_ARCH_BOARD)/scripts
ifeq ($(CONFIG_NRF52_BOOTLOADER),y)
  LDTEMPLATE = ld_ota.template
else
  LDTEMPLATE = ld.template
endif

ifeq ($(CONFIG_FAST_CONFIG_FILE),y)
	LDTEMPLATE = ld.fast_config_file
	SCRIPT_FAST_CONFIG_FILE_LENGTH = $(CONFIG_FLASH_FAST_CONFIG_FILE_LENGTH)
else
	SCRIPT_FAST_CONFIG_FILE_LENGTH = 0
endif

#
# Provide a variable to disable warnings as errors in the compile.
# The default is to enable -Werror, warnings are errors, and disable
# them if NO_ERRORS is defined: make NO_ERRORS=xxx (just give it value)
#
WERROR = -Werror
ifdef NO_ERRORS
        WERROR =
endif

ifeq ($(CONFIG_NRF52_BLUETOOTH),y)
LDSCRIPT = ld.softd.script
else
LDSCRIPT = ld.standard.script
#$(shell sed -e '/ FS_DATA/,/END_FS_DATA/d' \
#	-e '/__data_start__/d'  \
#	$(SCRIPTSDIR)/$(LDTEMPLATE) > $(SCRIPTSDIR)/tmp)
#LDTEMPLATE = tmp
endif

$(shell eval sed -e 's/NRF_FLASH_START/$(CONFIG_FLASH_ORIGIN)/' \
	-e 's/NRF_FLASH_LENGTH/$(CONFIG_FLASH_LENGTH)/'  \
	-e 's/NRF_FLASH_FAST_CONFIG_FILE_LENGTH/$(SCRIPT_FAST_CONFIG_FILE_LENGTH)/'  \
	-e 's/NRF_SRAM_START/$(CONFIG_SRAM_ORIGIN)/' \
	-e 's/NRF_SRAM_LENGTH/$(CONFIG_SRAM_LENGTH)/' \
	$(SCRIPTSDIR)/$(LDTEMPLATE) > $(SCRIPTSDIR)/$(LDSCRIPT))

ifeq ($(WINTOOL),y)
  # Windows-native toolchains
  DIRLINK = $(TOPDIR)/tools/copydir.sh
  DIRUNLINK = $(TOPDIR)/tools/unlink.sh
  MKDEP = $(TOPDIR)/tools/mknulldeps.sh
  ARCHINCLUDES = -I. -isystem "${shell cygpath -w $(TOPDIR)/include}"
  ARCHXXINCLUDES = -I. -isystem "${shell cygpath -w $(TOPDIR)/include}" -isystem "${shell cygpath -w $(TOPDIR)/include/cxx}"
  ARCHSCRIPT = -T "${shell cygpath -w $(TOPDIR)/configs/$(CONFIG_ARCH_BOARD)/scripts/$(LDSCRIPT)}"
else
  # Linux/Cygwin-native toolchain
  MKDEP = $(TOPDIR)/tools/mkdeps$(HOSTEXEEXT)

  ARCHSCRIPT = -T$(TOPDIR)/configs/$(CONFIG_ARCH_BOARD)/scripts/$(LDSCRIPT)

  ARCHXXINCLUDES = -I. -isystem $(TOPDIR)/include -isystem $(TOPDIR)/include/cxx

  ARCHINCLUDES  = -I$(CHIP_DIR) -I$(TOPDIR)/arch/arm/src/board -I. \
	          -isystem $(TOPDIR)/include                       \
		  -I$(HAL_HEADERS) -I$(DEVICE_HEADERS)             \


ifeq ($(CONFIG_NRF52_BLUETOOTH),y)
  ARCHINCLUDES += -I$(SD_HANDLER) -I$(SD_HEADERS) -I$(SD_HEADERS_NRF52)
endif

endif # WINTOOL

CC = $(CROSSDEV)gcc
CXX = $(CROSSDEV)g++
CPP = $(CROSSDEV)gcc -E
LD = $(CROSSDEV)ld
AR = $(ARCROSSDEV)ar rcs
NM = $(ARCROSSDEV)nm
OBJCOPY = $(CROSSDEV)objcopy
OBJDUMP = $(CROSSDEV)objdump
RM = rm -f
CP = cp
RMDIR = rm -rf

ARCHCCVERSION = ${shell $(CC) -v 2>&1 | sed -n '/^gcc version/p' | sed -e 's/^gcc version \([0-9\.]\)/\1/g' -e 's/[-\ ].*//g' -e '1q'}
ARCHCCMAJOR = ${shell echo $(ARCHCCVERSION) | cut -d'.' -f1}

ifeq ($(CONFIG_DEBUG_SYMBOLS),y)
  ARCHOPTIMIZATION = -g
endif

ifneq ($(CONFIG_DEBUG_NOOPT),y)
  ARCHOPTIMIZATION += $(MAXOPTIMIZATION) -fno-strict-aliasing -fno-strength-reduce -fomit-frame-pointer
endif

ARCHCFLAGS = -fno-builtin -std=c99
ARCHCXXFLAGS = -fno-builtin -fno-exceptions -fno-rtti

#
#  See above for details on WERROR
#
ARCHWARNINGS = $(WERROR) -Wall -Wstrict-prototypes -Wshadow -Wundef

ARCHWARNINGSXX = -Wall -Wshadow -Wundef
ARCHDEFINES =
ARCHPICFLAGS = -fpic -msingle-pic-base -mpic-register=r10


CFLAGS = $(ARCHCFLAGS) $(ARCHWARNINGS) $(ARCHOPTIMIZATION) $(ARCHCPUFLAGS) $(ARCHINCLUDES) $(ARCHDEFINES) $(EXTRADEFINES) -pipe
CPICFLAGS = $(ARCHPICFLAGS) $(CFLAGS)
CXXFLAGS = $(ARCHCXXFLAGS) $(ARCHWARNINGSXX) $(ARCHOPTIMIZATION) $(ARCHCPUFLAGS) $(ARCHXXINCLUDES) $(ARCHDEFINES) $(EXTRADEFINES) -pipe
CXXPICFLAGS = $(ARCHPICFLAGS) $(CXXFLAGS)
CPPFLAGS = $(ARCHINCLUDES) $(ARCHDEFINES) $(EXTRADEFINES)
AFLAGS = $(CFLAGS) -D__ASSEMBLY__

NXFLATLDFLAGS1 = -r -d -warn-common
NXFLATLDFLAGS2 = $(NXFLATLDFLAGS1) -T$(TOPDIR)/binfmt/libnxflat/gnu-nxflat-pcrel.ld -no-check-sections
LDNXFLATFLAGS = -e main -s 2048

ASMEXT = .S
OBJEXT = .o
LIBEXT = .a
EXEEXT =

# for zeus1 + zeus2 board macros
ifeq ($(CONFIG_FAST_API_ZEUS2),y)
CFLAGS += -DZEUS2_BOARD
endif
ifeq ($(CONFIG_FAST_API_ZEUS1),y)
CFLAGS += -DZEUS1_BOARD
endif

# for pre-define the ble device name  macro
ifneq ($(_BLE_DEV_NAME),)
CFLAGS += -D_BLE_DEV_NAME=\"$(_BLE_DEV_NAME)\"
endif

ifneq ($(CROSSDEV),arm-nuttx-elf-)
  LDFLAGS += -nostartfiles -nodefaultlibs
endif
ifeq ($(CONFIG_DEBUG_SYMBOLS),y)
  LDFLAGS += -g
endif

# Use this flag only for emergencies ....
# LD_WARNS_MISMATCH = --no-warn-mismatch
LDFLAGS += -Map=${TOPDIR}/nuttx.map

CFLAGS += -ffunction-sections -fdata-sections

CFLAGS += -I${TOPDIR}/${APPDIR}/include
HOSTCC = gcc
HOSTINCLUDES = -I.
HOSTCFLAGS = -Wall -Wstrict-prototypes -Wshadow -Wundef -g -pipe
HOSTLDFLAGS =

FLASHER=nrfjprog
FLASHER_IDENTS= $(FLASHER) -i
FLASHER_ERASE = $(FLASHER) -e
FLASHER_FLASH = $(FLASHER) --program
FLASHER_RESET = $(FLASHER) -r
FLASHER_VERIFY = $(FLASHER) --verify

#
# With this macro defined making the target "download" will build
# nuttx and then call this define to program the flashdevice.
# (see Makefile.unix, target download:)
#
define DOWNLOAD
	$(Q)pwd
        $(Q)if [ -n "$(shell $(FLASHER_IDENTS))" ] ; then \
                $(FLASHER_ERASE)  ;\
                $(FLASHER_FLASH)  $(NUTTXNAME).hex ;\
                $(FLASHER_RESET)  ;\
                $(FLASHER_VERIFY) ; $(NUTTXNAME).hex ;\
        else                       \
                echo "\nNo flash identified for programming.\n" ;\
        fi
endef

