##############################################################################
#                                                                            #
#   configs/nrf52840_dk/Makefile                                             #
#                                                                            #
#   Copyright (C) 2017,2018, zGlue inc.  All rights reserved.                #
#                                                                            #
#                                                                            #
#   This makefile builds Nuttx for the zglue_zeus2_chicago board specific    #
#   config, providing shortcuts for managing, building and mods.             #
#                                                                            #
##############################################################################

V ?= 0
Q ?= @
PLATFORM ?= ${notdir $(shell pwd)}
DEPTH = ../..
CHIP_SRC = arch/arm/src/nrf52
CHIP_SRC_DEPTH = $(shell echo $(CHIP_SRC) | sed -e 's/[^/]*/\.\./g')
CSCOPE_OUT = cscope.out
APPS = $(shell ls */defconfig | sed -e 's/\/.*//')
ARCH ?= arm

ifeq ($(shell [ -f $(DEPTH)/.config ] && echo y),y)
ARCH = (${CONFIG_ARCH:"%"=%})
include $(DEPTH)/.config
TARFILE = /tmp/tar.$(ZEUS)_for_nuttx.tgz

endif


CD = cd
LN = ln -sf
CP = cp
RM = rm -f
LD_SCRIPTS = ld.standard.script ld.softd.script tmp
TMPDIR = /tmp

#
# Copy arg 1 to back-track-to-root/arg2. Uses
# localy defined value DEPTH which is the
# backtrack path to the root of this repo.
#
define COPY_FROM_ROOT
	@echo "Copy FROM root $1 to $2"
	$(Q) $(CP)  $(DEPTH)/$1 $2
endef

#
# Copy from back-track-to-root/arg1 arg2 locally.
# Uses localy defined value DEPTH which is the
# backtrack path to the root of this repo.
#
define COPY_TO_ROOT
	@echo "Copy TO root $1 to $2"
	$(Q) $(CP)  $1 $(DEPTH)/$2
endef

#
# Given a path return the return path (e.g. a/b/c/d ==> ../../../..).
# If the path is '.' then the return path is '.'.
#
define BACK_TRACK
        $(shell [ -n "$1" ] && echo "$1" | sed -e 's/[^/]*/\.\./g' || echo ".")
endef

#
# Given a path, cd to that path and run cscope from the directory root
# of that path.  E.g. if the path is arch/arm/src then cd to src and
# run cscope -s../../..
#
define REDO_CSCOPE
	@echo "Run cscope from $(DEPTH)/$1"
        $(Q) cd $(DEPTH)/$1 && cscope -R -k -b -s $(call BACK_TRACK,$1)
endef

#
define UNDO_CSCOPE
	@echo "Remove cscope.out from $(DEPTH)/$1"
        $(Q) cd $(DEPTH)/$1 && $(RM) cscope.out
endef

#
# Given a path from root to a target dir remove any cscope.out file
# there.
#
#	cd $(DEPTH)/configs/$(PLATFORM) &&  rm -f $(CSCOPE_OUT)

.PHONY: $(APPS)

help:
	@echo "\nTargets:" >&2
	@count=0 && for name in config $(APPS) ; do \
		[ "$$count" -eq 0 ] && /bin/echo -n -e "\t   " >&2 ;\
		/bin/echo -n -e "$$name\t" >&2; \
		count=$$(( count + 1 )); \
		[ $${#name} -lt 5 ] && /bin/echo -n -e "\t" >&2 ;\
		[ "$$count" -eq 3 ] && count=0 && echo "" >&2 || true; \
	done
	@echo "\n" >&2
	@echo "Parameter: _MAJOR_ , _MINOR_ , _BUILD_VERSION_" >&2
	@echo "\t   _MAJOR_ is for Nuttx major version" >&2
	@echo "\t   _MINOR_ is for Nuttx minor version" >&2
	@echo "\t   _BUILD_VERSION_ is for custimizing build version" >&2
	@echo "\t   example:" >&2
	@echo "\t        make nsh _MAJOR_=7 _MINOR_=21 \\" >&2
	@echo "\t\t\t_BUILD_VERSION_=201711111111_12345" >&2
	@echo "\nParameter: _BLE_DEV_NAME, ZEUS2_BOARD" >&2
	@echo "\t   Define your own ble device name. Please export the name as" >&2
	@echo "\t   _BLE_DEV_NAME=“your_own_name”. If building for ZEUS2," >&2
	@echo "\t   please export ZEUS2_BOARD=1" >&2
	@echo "\nParameter: NO_ERRORS" >&2
	@echo "\t   NO_ERRORS is for building without -Werror.  Pass NO_ERRORS=1" >&2
	@echo >&2

config: menuconfig
reconfig: olddefconfig
all-reconfig: ${APPS:%=%_config}
${APPS:%=%_config}:
	$(Q)$(call COPY_TO_ROOT,${@:%_config=%}/defconfig,.config)
	$(Q)$(MAKE) -s olddefconfig
	$(Q)$(call COPY_FROM_ROOT,.config,${@:%_config=%}/defconfig)

distclean download menuconfig olddefconfig: $(DEPTH)/.config $(DEPTH)/Make.defs
	$(CD) $(DEPTH) && $(MAKE) $@

#
# Target compares the specified examples module listed in the
# config file with what it finds in the apps defconfig file.
# If the example module chosen in .config matches the one
# specified in the defconfig then the .config is copied ontop
# of the defconfig.
#
defconfig:
	$(Q) if [ -f $(DEPTH)/.config ] ; then \
		APP=$$(grep ^C.*AMPLES_.*=y $(DEPTH)/.config); \
		if [ -n "$$APP" ] ; then MOD=$$(echo $$APP | \
			sed -e s'/^C.*AMPLES_//' -e 's/=.*//' | tr A-Z a-z); \
			cp $(DEPTH)/.config $$MOD/defconfig; \
		fi \
	fi

#
# For the nsh module compile verbosely
#
nsh: V=2

top:
	$(Q) cd /tmp && ln -sf $(SWD)/$(DEPTH)/nuttx nuttx
	$(Q) cd /tmp && ln -sf $(SWD)/$(DEPTH)/apps apps

#
# This test is the opposite for the defconfig: target above.  If the
# specified modules differ between the .config and defconfig then
# a distclean is run and the appropriate defconfig is copied to .config.
#
$(APPS):
	$(Q) if [ ! -f $(DEPTH)/.config ] || \
		[ "$$(grep ^C.*XAMPLES_.*=y $(DEPTH)/.config)" != \
			"$$(grep ^C.*XAMPLES_.*=y $(@)/defconfig)" ] ; then \
			cd $(DEPTH) && $(MAKE) distclean;   \
		\
	fi
	$(Q)$(call COPY_TO_ROOT,$@/defconfig,.config)
	$(Q)$(call COPY_TO_ROOT,scripts/Make.defs,Make.defs)
	$(Q) $(MAKE) Q=$(Q) V=$(V) -C $(DEPTH)

$(DEPTH)/.config:
	$(Q) @echo "CONFIG_ARCH_BOARD=\"$(PLATFORM)\"" > $(DEPTH)/.config
	$(Q) @echo "CONFIG_ARCH=\"$(ARCH)\"" >> $(DEPTH)/.config

$(DEPTH)/Make.defs: $(DEPTH)/.config
	$(Q) @echo 'include $${TOPDIR}/.config' > $@
	$(Q) @echo 'include $${TOPDIR}/tools/Config.mk' >> $@
	$(Q) @echo 'include $${TOPDIR}/arch/arm/src/armv7-m/Toolchain.defs' >> $@
	$(Q) @echo 'include $${TOPDIR}/configs/$$(CONFIG_ARCH_BOARD)/defaults.mk' >> $@

clean_cscope:
	$(Q)$(call UNDO_CSCOPE,)
	$(Q)$(call UNDO_CSCOPE,$(CHIP_SRC))
	$(Q)$(call UNDO_CSCOPE,configs/$(PLATFORM))

cscope: clean_cscope
	$(Q)$(call REDO_CSCOPE,)
	$(Q)$(call REDO_CSCOPE,$(CHIP_SRC))
	$(Q)$(call REDO_CSCOPE,configs/$(PLATFORM))

clean:
	$(Q) if [ "$(MAKECMDGOALS)" = "clean" ] ; then \
		echo "Clean found." ;\
		$(MAKE) -C $(DEPTH) $@ ; \
	fi
	$(Q) $(RM) $(LD_SCRIPTS:%=scripts/%)

clobber: distclean

#
# Clean out all of the stuff strewn about the tree as a result of coding.
#
clean_env: clean_cscope clean_cruft

#
# Remove all those pesky files resulting from side effects of running
# some commands namely patch, merge, me.
#
clean_cruft:
	$(Q) $(RM) /tmp/out.*
	$(Q) cd $(DEPTH) && find . -path ./.git -prune -o              \
			-type f -name  "*.orig"  -exec rm {} \; -o     \
			-type f -name  "*.rej"   -exec rm {} \; -o     \
			-type f -name  "tmp*"    -exec rm -rf {} \; -o \
			-type f -name  "files.*" -exec rm {} \; -o     \
			-type f -name  "diffs.*" -exec rm {} \; -o     \
			-type f -iname "[a-h][a-h]" -exec rm {} \; -o  \
			-type f -iname "[a-h][a-h][a-h]" -exec rm {} \;

import: $(DEPTH)/.config $(DEPTH)/Make.defs $(TARFILE)
	@echo "Importing Zeus files into Nuttx ..."
	$(Q) cd $(DEPTH) && tar -zxf $(TARFILE)

list_apps:
	$(Q) echo "$(APPS)"

