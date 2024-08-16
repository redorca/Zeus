############################################################################
# external/mcuboot/Makefile
#
#   Copyright (C) 2017, 2018 Zglue Inc. All rights reserved.
#   Author: Levin Li <zhiqiang@zglue.com>
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

-include $(TOPDIR)/Make.defs

# Basic

CUR_DIR=$(TOPDIR)/external/mcuboot

VPATH += : bootutil/src
VPATH += : nuttx/src

DEPPATH += --dep-path .
DEPPATH += --dep-path bootutil/src
DEPPATH += --dep-path nuttx/src


ASRCS =
AOBJS = $(ASRCS:.S=$(OBJEXT))

CSRCS = loader.c bootutil_misc.c image_validate.c image_rsa.c image_ec256.c caps.c

CSRCS += flash_map.c hal_flash.c hal_os.c nuttx_keys.c

ifeq ($(CONFIG_MCUBOOT_SECOND_IMAGE_QSPI),y)
CSRCS += flash_map_qspi.c
endif

COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

BIN = libmcuboot$(LIBEXT)

${info CSRCS $(CSRCS)}

CFLAGS += -I$(CUR_DIR)/nuttx/include

CFLAGS += -DMCUBOOT_USE_FLASH_AREA_GET_SECTORS

# pass macro mbed_tls or tinycrypt to library

ifeq ($(CONFIG_USE_MBED_TLS),y)
CFLAGS += -DMCUBOOT_USE_MBED_TLS
CFLAGS += -DMBEDTLS_CONFIG_FILE=\"$(CONFIG_MBEDTLS_CFG_FILE)\"
endif

ifeq ($(CONFIG_USE_TINYCRYPT),y)
CFLAGS += -DMCUBOOT_USE_TINYCRYPT
endif


ifeq ($(CONFIG_MCUBOOT_VALIDATE_SLOT0),y)
CFLAGS += -DMCUBOOT_VALIDATE_SLOT0
endif

ifeq ($(CONFIG_MCUBOOT_SIGN_RSA),y)
CFLAGS += -DMCUBOOT_SIGN_RSA
endif

ifeq ($(CONFIG_MCUBOOT_SIGN_EC),y)
CFLAGS += -DMCUBOOT_SIGN_EC
endif

ifeq ($(CONFIG_MCUBOOT_SIGN_EC256),y)
CFLAGS += -DMCUBOOT_SIGN_EC256
endif

all: $(BIN)
.PHONY: depend clean distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

context:
	$(Q) (cd ../../include && if [ ! -L bootutil ] ; then \
	ln -s ../external/mcuboot/bootutil/include/bootutil bootutil; fi)

.depend: Makefile $(SRCS)
ifneq ($(CONFIG_MBEDTLS_CFG_FILE),"")
# due to mkdeps issue : added mbedtls_cfg_file  macro here with double backslash
	$(MKDEP) $(DEPPATH) "$(CC)" -- $(CFLAGS) -DMBEDTLS_CONFIG_FILE=\\\"$(CONFIG_MBEDTLS_CFG_FILE)\\\" -- $(SRCS) >Make.dep
else
	$(MKDEP) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
endif
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)
	$(Q) rm -f ../../include/bootutil

-include Make.dep
