Q  = @
CP      = cp
ZIP     = zip
RM      = rm -f
MV      = mv -f

DIR_BASE                = /home/zglue/src/zglue
LOCAL_ROOT              = $(shell pwd)
CONFIG_ROOT             = $(LOCAL_ROOT)/configs
DOXYGEN_DOCDIR          = doxy
DOXYGEN_ZIPFILE         = sdk_documentation.zip
DOXYGEN                ?= $(shell which doxygen)
DOXY_CONFIG_FILE        = configs/Doxyfile.nuttx.v0.1.SDK
DOXYVERSION             =  $(shell $(DOXYGEN) --version)
ZGLUE_LOGO              = images/zglue_logo_mini_color.png
DOXY_ZGLUE_LOGO         = doxy/html/zglue_logo_mini_color.png
OUTFILE                 = /tmp/.doxy.${USER}.out

INCLUDED_DIRS           = /home/zglue/src/zglue/three-devs/nuttx/external/nimble        \
                          /home/zglue/src/zglue/doxy/fast_api                           \
                          /home/zglue/src/zglue/three-devs/nuttx/external/SDK-Documentation

INCLUDED_TARGETS        = ${notdir $(INCLUDED_DIRS)}
$(info included targets <$(INCLUDED_TARGETS)>)

NUTTX_ROOT              = $(DIR_BASE)/three-devs/nuttx/external/SDK-Documentation
FASTAPI_ROOT            = $(DIR_BASE)/doxy/fast_api
NIMBLE_ROOT             = $(DIR_BASE)/three-devs/nuttx/external/nimble
WARNING_LOGFILES        = ${INCLUDED_TARGETS:%=log.%.warn}

.PHONY: dox help
include $(DOXY_CONFIG_FILE)
help:
	@echo "\tRun make dox for a set of doxygen generated documents."
	@echo "\tVersoion $(DOXYVERSION) of doxygen is available and if"
	@echo "\tthe version is less than 1.8.15 docs will not build"
	@echo "\tproperly."
	@echo

define DOXY
	@cd $1 && echo "Running in $$(pwd) for $3"
	$(Q) cd $1 && $(DOXYGEN) $2 >> $(OUTFILE) 2>&1
	$(Q) cd $(LOCAL_ROOT) && ([ ! -d doxy/html/html.${notdir $1} ] || rm -rf doxy/html/html.${notdir $1})
	$(Q) cd $(LOCAL_ROOT) && ([ -d doxy/html ]|| mkdir doxy/html) && $(MV) doxy/${notdir $1}/html doxy/html/html.${notdir $1}
endef

$(DOXYGEN_DOCDIR):
	$(Q) mkdir -p $@

${INCLUDED_DIRS:%=%-dox}: Foo = ${notdir $@}
${INCLUDED_DIRS:%=%-dox}:
	$(Q) echo "Foo $(Foo)"

dox-nuttx: DOXY_CONFIG_FILE = $(CONFIG_ROOT)/Doxyfile.nuttx.v0.1.SDK
dox-fastapi: DOXY_CONFIG_FILE = $(CONFIG_ROOT)/Doxyfile.fastapi.v0.1.SDK
dox-nimble: DOXY_CONFIG_FILE = $(CONFIG_ROOT)/Doxyfile.nimble.v0.1.SDK

dox-nuttx: DOC_SRCDIR = $(NUTTX_ROOT)
dox-fastapi: DOC_SRCDIR = $(FASTAPI_ROOT)
dox-nimble: DOC_SRCDIR = $(NIMBLE_ROOT)

dox-fastapi     \
dox-nimble:
	$(Q) $(call DOXY,$(DOC_SRCDIR),$(DOXY_CONFIG_FILE),$@)

dox-nuttx:
	$(Q) cd $(DOC_SRCDIR) && $(DOXYGEN) $(DOXY_CONFIG_FILE) >> $(OUTFILE) 2>&1

dox-zip: dox
	$(Q) $(RM) $(DOXYGEN_DOCDIR)/html/$(DOXYGEN_ZIPFILE)
	$(Q) cd $(DOXYGEN_DOCDIR) && zip -ru ${DOXYGEN_ZIPFILE:%.zip=%} html
	$(Q) $(MV) $(DOXYGEN_DOCDIR)/$(DOXYGEN_ZIPFILE) $(DOXYGEN_DOCDIR)/html

dox: $(DOXYGEN_DOCDIR) dox-fastapi dox-nimble dox-nuttx
	@echo "Assembling the document."
	$(Q) pwd
#	$(Q) $(MV) doxy/fastapi/html doxy/html/html.fastapi
#	$(Q) $(MV) doxy/nimble/html doxy/html/html.nimble

$(INCLUDED_DIRS):
	$(Q) cd $@ && $(DOXYGEN) $(DOXY_CONFIG_FILE)

dox-clean \
dox_clean \
clean:
	@echo "$@ ::"
	$(Q) $(RM) -r doxy/**/* 2>/dev/null || true
	$(Q) $(RM) $(OUTFILE) $(WARNING_LOGFILES)

dox-distclean \
dox_distclean \
distclean: clean
	@echo "$@ ::"
	@$(RM) -r $(DOXYGEN_DOCDIR)

