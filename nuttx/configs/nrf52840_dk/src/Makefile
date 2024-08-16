############################################################################
# configs/nrf52840_dk/src/Makefile
#
#   Copyright (C) 2014 Gregory Nutt. All rights reserved.
#   Copyright (C) 2016-2018 Zglue Inc. All rights reserved.
#   Author: Levin Li   <zhiqiang@zglue.com>
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

ASRCS =
CSRCS = boards.c nrf52_boot.c  nrf52_autoleds.c

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS +=  nrf52_buttons.c
endif

CSRCS += nrf52_appinit.c
CSRCS += nrf52_internal_flash.c
CSRCS += nrf52_uid.c

ifeq ($(CONFIG_EXAMPLES_OSTEST),y)
CSRCS += nrf52_ostest.c
endif

ifeq ($(CONFIG_NRF52_SPI),y)
CSRCS += nrf52_user_spi.c
CSRCS += nrf52_spi_w25.c
endif

ifeq ($(CONFIG_CODE_ANALYSIS),y)
CSRCS += code_analysis.c
endif

ifeq ($(CONFIG_AUDIO_PCM510X),y)
CSRCS += nrf52_pcm510x.c
endif

ifeq ($(CONFIG_AUDIO_PDM_MIC),y)
CSRCS += nrf52_pdm_microphone.c
endif

ifeq ($(CONFIG_NRF52_QSPI),y)
CSRCS += nrf52_qspi_mx25.c
endif

ifeq ($(CONFIG_BMM150),y)
CSRCS += nrf52_bmm150.c
endif

ifeq ($(CONFIG_MC3672),y)
CSRCS += nrf52_mc3672.c
endif
ifeq ($(CONFIG_BMI160),y)
CSRCS += nrf52_bmi160.c
endif

ifeq ($(CONFIG_LSM6DS3),y)
CSRCS += nrf52_lsm6ds3.c
endif

ifeq ($(CONFIG_COMP),y)
CSRCS += nrf52_comp.c
endif

include $(TOPDIR)/configs/Board.mk
