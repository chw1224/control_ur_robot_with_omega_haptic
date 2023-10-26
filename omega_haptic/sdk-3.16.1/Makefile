################################################################################
#
#  Copyright (C) 2001-2023 Force Dimension, Switzerland.
#  All Rights Reserved.
#
#  Force Dimension SDK 3.16.1
#
################################################################################

# project structure
TOP_DIR = .
BIN_DIR = $(TOP_DIR)/bin
include Makefile.common

# build dependencies
SUBDIRS = examples
ifneq (,$(wildcard externals/Makefile))
    SUBDIRS += externals
    examples: externals
endif

# build rules

all: $(SUBDIRS)

.PHONY: $(SUBDIRS)
$(SUBDIRS):
	$(MAKE) -C $@

clean:
	@for T in $(SUBDIRS); do make -C $$T $@; done

