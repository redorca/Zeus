############################################################################
# configs/nrf52840_dk/defaults.mk
#
#   Copyright (C) 2016 Gregory Nutt. All rights reserved.
#   Copyright (C) 2018 Zglue Inc. All rights reserved.
#   Author: Levin Li  <zhiqiang@zglue.com>
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

#
# Handle build situations where the app uses code from submodules that may not be
# present in the tree: external/{mbedtls,mcuboot} so rather than fail due to a header
# not found fail right away as soon as you know you will need that source.
#
ifeq ($(MAKECMDGOALS),)

ifdef CONFIG_MCUBOOT
MCUBOOT_DIR=$(shell if [ -d $(TOPDIR)/external/mcuboot ] ; then echo y; else echo n; fi)
ifeq ($(MCUBOOT_DIR), n)
$(error :: CONFIG_MCUBOOT is enabled but those sources appear to be missing.)
endif # (shell -d ... )
endif # CONFIG_MCUBOOT

ifdef CONFIG_MBEDTLS
MBEDTLS_DIR=$(shell if [ -d $(TOPDIR)/external/mbedtls ] ; then echo y; else echo n; fi)
ifeq ($(MBEDTLS_DIR), n)
$(error :: CONFIG_MBEDTLS is enabled but those sources appear to be missing.)
endif # (shell -d ... )
endif # CONFIG_MBEDTLS

ifdef CONFIG_BLE_NIMBLE
NIMBLE_DIR=$(shell if [ -d $(TOPDIR)/external/nimble ] ; then echo y; else echo n; fi)

ifeq ($(NIMBLE_DIR), n)
$(error :: CONFIG_BLE_NIMBLE is enabled but those sources appear to be missing.)
endif # external/nimble
endif # CONFIG_BLE_NIMBLE


ifdef CONFIG_LWIP
LWIP_DIR=$(shell if [ -d $(TOPDIR)/external/lwip ] ; then echo y; else echo n; fi)

ifeq ($(LWIP_DIR), n)
$(error :: CONFIG_LWIP is enabled but those sources appear to be missing.)
endif # external/lwip
endif # CONFIG_LWIP

endif # MAKECMDGOALS


$(shell [ ! -d $(TOPDIR)/external ] && mkdir -p $(TOPDIR)/external)

#
# Provide a variable to disable warnings as errors in the compile.
# The default is to enable -Werror, warnings are errors, and disable
# them if NO_ERRORS is defined: make NO_ERRORS=xxx (just give it value)
# user -Wall instead of -Werror
WERROR ?= -Wall
ifdef NO_ERRORS
        WERROR =
endif

SCRIPTSDIR = $(TOPDIR)/configs/$(CONFIG_ARCH_BOARD)/scripts

LINKERLD = zdk_flat.ld

ifeq ($(CONFIG_FAST_CONFIG_FILE),y)
LINKERLD = ld.fast_config_file
SCRIPT_FAST_CONFIG_FILE_LENGTH = $(CONFIG_FLASH_FAST_CONFIG_FILE_LENGTH)
endif

ifeq ($(CONFIG_NRF52_BLUETOOTH),y)
LDSCRIPT = generated.softd.script
else
LDSCRIPT = generated.standard.ld
endif

$(shell [ -d $(TOPDIR)/external ] && [ ! -f $(TOPDIR)/external/Kconfig ] && touch $(TOPDIR)/external/Kconfig)

ifeq ($(CONFIG_BUILD_PROTECTED),y)
LINKERLD = kernel-space.ld
LDSCRIPT = generated-k-space.ld
LDFLAGS += -L$(TOPDIR)$(DELIM)configs$(DELIM)$(CONFIG_ARCH_BOARD)$(DELIM)scripts
endif

ifeq ($(WINTOOL),y)
  # Windows-native toolchains
  DIRLINK = $(TOPDIR)/tools/copydir.sh
  DIRUNLINK = $(TOPDIR)/tools/unlink.sh
  MKDEP = $(TOPDIR)/tools/mknulldeps.sh
  ARCHINCLUDES = -I. -isystem "${shell cygpath -w $(TOPDIR)$(DELIM)include}"
  ARCHXXINCLUDES = -I. -isystem "${shell cygpath -w $(TOPDIR)$(DELIM)include}" -isystem "${shell cygpath -w $(TOPDIR)$(DELIM)include$(DELIM)cxx}"
  ARCHSCRIPT = -T "${shell cygpath -w $(TOPDIR)$(DELIM)configs$(DELIM)$(CONFIG_ARCH_BOARD)$(DELIM)scripts$(DELIM)$(LDSCRIPT)}"
else
  # Linux/Cygwin-native toolchain
  MKDEP = $(TOPDIR)/tools/mkdeps$(HOSTEXEEXT)
  ARCHINCLUDES = -I. -isystem $(TOPDIR)$(DELIM)include
  ARCHXXINCLUDES = -I. -isystem $(TOPDIR)$(DELIM)include -isystem $(TOPDIR)$(DELIM)include$(DELIM)cxx
  ARCHSCRIPT = -T$(TOPDIR)/configs/$(CONFIG_ARCH_BOARD)/scripts/$(LDSCRIPT)
endif # WINTOOL

ifeq ($(CONFIG_NRF52_BLUETOOTH),y)
  ARCHINCLUDES += -I$(SD_HANDLER) -I$(SD_HEADERS) -I$(SD_HEADERS_NRF52)
endif

ifeq ($(CONFIG_UCLIBCXX),y)
  ARCHXXINCLUDES += -isystem $(TOPDIR)/include/uClibc++
endif

# added SoC driver header directory
ARCHINCLUDES  += -I$(CHIP_DIR) -I$(TOPDIR)/arch/arm/src/board  \
		  -I$(HAL_HEADERS) -I$(DEVICE_HEADERS)         \

CC = $(CROSSDEV)gcc
CXX = $(CROSSDEV)g++
CPP = $(CROSSDEV)gcc -E
LD = $(CROSSDEV)ld
STRIP = $(CROSSDEV)strip --strip-unneeded
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
#ARCHCXXFLAGS = -fno-builtin -fno-exceptions -fno-rtti
ifeq ($(CONFIG_UCLIBCXX_EXCEPTION),y)
  ARCHCXXFLAGS = -fno-builtin -fpermissive
else
  ARCHCXXFLAGS = -fno-builtin -fno-exceptions -fpermissive
endif

#
#  See above for details on WERROR
#
ARCHWARNINGS = $(WERROR) -Wall -Wstrict-prototypes -Wshadow -Wundef

ARCHWARNINGSXX = -Wall -Wshadow -Wundef
ARCHDEFINES =
ARCHPICFLAGS = -fpic -msingle-pic-base -mpic-register=r10


CFLAGS = $(ARCHCFLAGS) $(ARCHWARNINGS) $(ARCHOPTIMIZATION) $(ARCHCPUFLAGS) $(ARCHINCLUDES) $(ARCHDEFINES) $(EXTRADEFINES) -pipe

# enable stack-information collecting
CFLAGS += -fstack-usage -Wsuggest-attribute=noreturn
CPICFLAGS = $(ARCHPICFLAGS) $(CFLAGS)
CXXFLAGS = $(ARCHCXXFLAGS) $(ARCHWARNINGSXX) $(ARCHOPTIMIZATION) $(ARCHCPUFLAGS) $(ARCHXXINCLUDES) $(ARCHDEFINES) $(EXTRADEFINES) -pipe
CXXPICFLAGS = $(ARCHPICFLAGS) $(CXXFLAGS)
CPPFLAGS = $(ARCHINCLUDES) $(ARCHDEFINES) $(EXTRADEFINES)
AFLAGS = $(CFLAGS) -D__ASSEMBLY__

NXFLATLDFLAGS1 = -r -d -warn-common
NXFLATLDFLAGS2 = $(NXFLATLDFLAGS1) -T$(TOPDIR)$(DELIM)binfmt$(DELIM)libnxflat$(DELIM)gnu-nxflat-pcrel.ld -no-check-sections
LDNXFLATFLAGS = -e main -s 2048

ASMEXT = .S
OBJEXT = .o
LIBEXT = .a
EXEEXT =

# for pre-define the ble device name  macro
ifneq ($(_BLE_DEV_NAME),)
CFLAGS += -D_BLE_DEV_NAME=\"$(_BLE_DEV_NAME)\"
endif

# added stack check flag
ifeq ($(CONFIG_ARMV7M_STACKCHECK),y)
CFLAGS += -finstrument-functions
endif

ifneq ($(CROSSDEV),arm-nuttx-elf-)
  LDFLAGS += -nostartfiles -nodefaultlibs
endif
ifeq ($(CONFIG_DEBUG_SYMBOLS),y)
  LDFLAGS += -g
endif

LDFLAGS += -Map=${TOPDIR}/nuttx.map
CFLAGS += -ffunction-sections -fdata-sections
#LDFLAGS += --gc-sections

CFLAGS += -I${TOPDIR}/${APPDIR}/include
HOSTCC = gcc
HOSTINCLUDES = -I.
HOSTCFLAGS = -Wall -Wstrict-prototypes -Wshadow -Wundef -g -pipe
HOSTLDFLAGS =

ifeq ($(DO_CLEAN),y)
EXTRA_CLEAN = rm -f $(TOPDIR)/external/Kconfig
EXTRA_DISTCLEAN = rm -rf $(TOPDIR)/external
endif

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
define CLEAN
        $(Q) if exist *$(OBJEXT) (del /f /q *$(OBJEXT))
        $(Q) if exist *$(LIBEXT) (del /f /q *$(LIBEXT))
        $(Q) if exist *~ (del /f /q *~)
        $(Q) if exist (del /f /q  .*.swp *.su)
endef
else
define CLEAN
       $(Q) rm -f *$(OBJEXT) *$(LIBEXT) *~ .*.swp *.su
endef
endif

