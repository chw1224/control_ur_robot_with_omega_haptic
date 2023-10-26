################################################################################
#
#  Copyright (C) 2001-2023 Force Dimension, Switzerland.
#  All Rights Reserved.
#
#  Force Dimension SDK 3.16.1
#
################################################################################

# DO NOT MODIFY THIS FILE
# Project-specific changes should go in Makefile.project(.*).

# Default target
all: project

# Common settings
TOP_DIR = ../../..
ifneq (,$(wildcard $(TOP_DIR)/Makefile.common))
    include $(TOP_DIR)/Makefile.common
endif

# Directories
SRC_DIR = .
OBJ_DIR = ./obj/$(CFG)/$(OS)-$(ARCH)-$(COMPILER)

# Files
SOURCES  = $(wildcard $(SRC_DIR)/*.cpp)
INCLUDES = $(wildcard $(SRC_DIR)/*.h)
OBJECTS  = $(patsubst %.cpp, $(OBJ_DIR)/%.o, $(notdir $(SOURCES)))
NAME     = $(notdir $(shell pwd)) 
TARGET   = $(BIN_DIR)/$(NAME)

# Import project settings.
ifneq (,$(wildcard ./Makefile.project))
    include ./Makefile.project
endif

# Import platform-specific project settings.
ifneq (,$(wildcard ./Makefile.project.$(OS)))
    include ./Makefile.project.$(OS)
endif

# Build rules

project: $(TARGET)

$(OBJECTS): $(INCLUDES) $(LIB_INCLUDES) | $(OBJ_DIR)

$(TARGET): $(LIB_STATIC) $(OBJECTS) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(OBJECTS) $(LDFLAGS) $(LDLIBS) -o $(TARGET)

$(OBJ_DIR):
	mkdir -p $@

$(OBJ_DIR)/%.o : $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

.PHONY: project

clean:
	rm -f $(TARGET) $(OBJECTS) *~ TAGS core *.bak #*#
	-rmdir $(OBJ_DIR)
