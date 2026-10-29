################################################################
#
# nav_pof.c unit tests (based on gcc)
#
################################################################

TARGET = nav_pof

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

MOD_DIR = $(ROOT_DIR)/mod
STUBS_DIR = $(MOD_DIR)/_test/stubs

################################################################
# source
################################################################

TEST_SOURCES = \
test_pof.c \
$(wildcard $(STUBS_DIR)/nav_contract_stubs/*.c)

COV_C_SOURCES = \
$(wildcard $(MOD_DIR)/nav/nav*.c) \
$(MOD_DIR)/math_sdr/math_sdr.c

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
-I$(MOD_DIR)/ \
-I$(MOD_DIR)/_test/stubs \
-I$(MOD_DIR)/_test/stubs/nav_contract_stubs \
-I$(MOD_DIR)/math_sdr \
-I$(MOD_DIR)/error_sdr \
-I$(MOD_DIR)/debug_sdr \
-I$(MOD_DIR)/nav	   \
-I$(FRAMEWORK_DIR)/src

C_DEFS = \
-DUSE_HAL_DRIVER \
-DSTM32H750xx \
-DFLIGHT_COMPUTER \
-DA0002_REV2 \
-DUNIT_TEST

CFLAGS = $(C_INCLUDES) $(C_DEFS) $(OPT) -Wall -g
CFLAGS += -Wno-pointer-to-int-cast
CFLAGS += -Wno-int-to-pointer-cast
CFLAGS += -Wno-unused-function
CFLAGS += -Wno-unused-variable
CFLAGS += -Wno-implicit-function-declaration
CFLAGS += -ffunction-sections
CFLAGS += -fdata-sections
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
	$(CC) $(OBJECTS) -o $@ -Wl,--gc-sections -lgcov -lm

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
		--filter '.*/(nav|math_sdr)[^/]*[.]c$$' \
		--html-details coverage/coverage.html \
		--json coverage/coverage.json

################################################################
# clean up
################################################################

clean:
	-rm -fR $(BUILD_DIR)
