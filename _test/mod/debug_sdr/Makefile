################################################################
#
# debug_sdr.c unit tests (based on gcc)
#
################################################################

TARGET = debug_sdr

################################################################
# build variables
################################################################

DEBUG ?= 0
OPT   = -Og

################################################################
# paths
################################################################

BUILD_DIR = build
ROOT_DIR  = ../../../..
FRAMEWORK_DIR = ../../framework

################################################################
# source
################################################################

TEST_SOURCES = \
test_debug_sdr.c \
mock/test_debug_sdr_stubs.c

COV_C_SOURCES = \
$(ROOT_DIR)/mod/debug_sdr/debug_sdr.c

FRAMEWORK_SOURCES = \
$(FRAMEWORK_DIR)/src/test_assert.c \
$(FRAMEWORK_DIR)/src/test_runner.c

C_SOURCES = $(TEST_SOURCES) $(COV_C_SOURCES) $(FRAMEWORK_SOURCES)

################################################################
# compiler
################################################################

CC = gcc

################################################################
# C flags
################################################################

C_INCLUDES = \
-I. \
-I./mock \
-I$(ROOT_DIR)/mod/debug_sdr \
-I$(ROOT_DIR)/mod/error_sdr \
-I$(FRAMEWORK_DIR)/src

C_DEFS = \
-DUNIT_TEST \
-DDEBUG_INTF_BUF_SIZE=64

CFLAGS = $(C_INCLUDES) $(C_DEFS) $(OPT) -Wall -g
CFLAGS += -Wno-unused-function
CFLAGS += -Wno-unused-variable
CFLAGS += -Wno-implicit-function-declaration
CFLAGS += -ftest-coverage
CFLAGS += -fprofile-arcs
CFLAGS += -ffile-prefix-map=$(shell pwd)/=

ifeq ($(DEBUG), 1)
CFLAGS += -DDEBUG
else
CFLAGS += -DRELBLD
endif

################################################################
# build all
################################################################

all: clean $(BUILD_DIR)/$(TARGET)

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

$(BUILD_DIR)/%.o: %.c $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ -lgcov -lm

$(BUILD_DIR):
	mkdir $@

################################################################
# build test
################################################################

test:
	@echo THIS TEST MUST BE EXECUTED IN A BASH TERMINAL. CMD/PS do not work.
	-rm -fR $(BUILD_DIR)
	$(MAKE) all
	$(BUILD_DIR)/$(TARGET)
	tail -n 7 "results.txt"
	mkdir -p coverage
	gcovr $(BUILD_DIR) \
		--filter '.*/debug_sdr[.]c$$' \
		--html-details coverage/coverage.html \
		--json coverage/coverage.json

################################################################
# clean up
################################################################

clean:
	-rm -fR $(BUILD_DIR)
