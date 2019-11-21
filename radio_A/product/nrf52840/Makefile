# MIT License
# 
# Copyright (c) 2019 Martin Aalien
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

PROJECT_NAME := radio_001
TARGETS      := nrf52840_xxaa

export OUTPUT_FILENAME
#MAKEFILE_NAME := $(CURDIR)/$(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST))
MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) ) 

TEMPLATE_PATH = ../../../
PROJ_DIR := ../../../

ifeq ($(OS),Windows_NT)
include $(PROJ_DIR)/lib_sdk/components/toolchain/gcc/Makefile.windows
else
include $(PROJ_DIR)/lib_sdk/components/toolchain/gcc/Makefile.posix
endif

MK := mkdir
RM := rm -rf

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO := 
else
NO_ECHO := @
endif

# Toolchain commands
CC              := '$(GNU_INSTALL_ROOT)/$(GNU_PREFIX)-gcc'
AS              := '$(GNU_INSTALL_ROOT)/$(GNU_PREFIX)-as'
AR              := '$(GNU_INSTALL_ROOT)/$(GNU_PREFIX)-ar' -r
LD              := '$(GNU_INSTALL_ROOT)/$(GNU_PREFIX)-ld'
NM              := '$(GNU_INSTALL_ROOT)/$(GNU_PREFIX)-nm'
OBJDUMP         := '$(GNU_INSTALL_ROOT)/$(GNU_PREFIX)-objdump'
OBJCOPY         := '$(GNU_INSTALL_ROOT)/$(GNU_PREFIX)-objcopy'
SIZE            := '$(GNU_INSTALL_ROOT)/$(GNU_PREFIX)-size'

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

#source common to all targets
C_SOURCE_FILES += \
  $(PROJ_DIR)/radio_A/src/radio_001.c \
  $(PROJ_DIR)/lib_sdk/modules/nrfx/mdk/system_nrf52840.c \
  $(PROJ_DIR)/lib_sdk/components/libraries/log/src/nrf_log_backend_rtt.c \
  $(PROJ_DIR)/lib_sdk/components/libraries/log/src/nrf_log_backend_serial.c \
  $(PROJ_DIR)/lib_sdk/components/libraries/log/src/nrf_log_backend_uart.c \
  $(PROJ_DIR)/lib_sdk/components/libraries/log/src/nrf_log_default_backends.c \
  $(PROJ_DIR)/lib_sdk/components/libraries/log/src/nrf_log_frontend.c \
  $(PROJ_DIR)/lib_sdk/components/libraries/log/src/nrf_log_str_formatter.c \
  $(PROJ_DIR)/lib_sdk/components/boards/boards.c \
  $(PROJ_DIR)/lib_sdk/components/libraries/util/app_error.c \
  $(PROJ_DIR)/lib_sdk/components/libraries/util/app_error_handler_gcc.c \
  $(PROJ_DIR)/lib_sdk/components/libraries/util/app_error_weak.c \
  $(PROJ_DIR)/lib_sdk/components/libraries/util/app_util_platform.c \
  $(PROJ_DIR)/lib_sdk/components/libraries/util/nrf_assert.c \
  $(PROJ_DIR)/lib_sdk/components/libraries/atomic/nrf_atomic.c \
  $(PROJ_DIR)/lib_sdk/components/libraries/balloc/nrf_balloc.c \
  $(PROJ_DIR)/lib_sdk/external/fprintf/nrf_fprintf.c \
  $(PROJ_DIR)/lib_sdk/external/fprintf/nrf_fprintf_format.c \
  $(PROJ_DIR)/lib_sdk/components/libraries/gfx/nrf_gfx.c \
  $(PROJ_DIR)/lib_sdk/components/libraries/memobj/nrf_memobj.c \
  $(PROJ_DIR)/lib_sdk/components/libraries/ringbuf/nrf_ringbuf.c \
  $(PROJ_DIR)/lib_sdk/components/libraries/strerror/nrf_strerror.c \
  $(PROJ_DIR)/lib_sdk/integration/nrfx/legacy/nrf_drv_clock.c \
  $(PROJ_DIR)/lib_sdk/integration/nrfx/legacy/nrf_drv_spi.c \
  $(PROJ_DIR)/lib_sdk/integration/nrfx/legacy/nrf_drv_uart.c \
  $(PROJ_DIR)/lib_sdk/components/drivers_nrf/nrf_soc_nosd/nrf_nvic.c \
  $(PROJ_DIR)/lib_sdk/components/drivers_nrf/nrf_soc_nosd/nrf_soc.c \
  $(PROJ_DIR)/lib_sdk/modules/nrfx/soc/nrfx_atomic.c \
  $(PROJ_DIR)/lib_sdk/modules/nrfx/drivers/src/nrfx_clock.c \
  $(PROJ_DIR)/lib_sdk/modules/nrfx/drivers/src/prs/nrfx_prs.c \
  $(PROJ_DIR)/lib_sdk/modules/nrfx/drivers/src/nrfx_spi.c \
  $(PROJ_DIR)/lib_sdk/modules/nrfx/drivers/src/nrfx_spim.c \
  $(PROJ_DIR)/lib_sdk/modules/nrfx/drivers/src/nrfx_uart.c \
  $(PROJ_DIR)/lib_sdk/modules/nrfx/drivers/src/nrfx_uarte.c \
  $(PROJ_DIR)/lib_sdk/external/thedotfactory_fonts//orkney24pts.c \
  $(PROJ_DIR)/lib_sdk/external/thedotfactory_fonts/orkney8pts.c \
  $(PROJ_DIR)/lib_sdk/components/drivers_ext/ili9341/ili9341.c \
  $(PROJ_DIR)/lib_sdk/components/drivers_ext/st7735/st7735.c \
  $(PROJ_DIR)/lib_sdk/external/segger_rtt/SEGGER_RTT_Syscalls_GCC.c \
  $(PROJ_DIR)/lib_sdk/external/segger_rtt/SEGGER_RTT_printf.c \
  $(PROJ_DIR)/lib_sdk/external/segger_rtt/SEGGER_RTT.c \
  $(PROJ_DIR)/lib_project/timer.c \

#assembly files common to all targets
ASM_SOURCE_FILES  += \
  $(PROJ_DIR)/lib_sdk/modules/nrfx/mdk/gcc_startup_nrf52840.s \

#includes common to all targets
INC_PATHS += \
  -I$(PROJ_DIR)/radio_A/product/nrf52840 \
  -I$(PROJ_DIR)/lib_sdk/components/libraries/strerror \
  -I$(PROJ_DIR)/lib_sdk/components/libraries/util \
  -I$(PROJ_DIR)/lib_sdk/components/libraries/balloc \
  -I$(PROJ_DIR)/lib_sdk/components/libraries/ringbuf \
  -I$(PROJ_DIR)/lib_sdk/components/libraries/log \
  -I$(PROJ_DIR)/lib_sdk/components/libraries/gfx \
  -I$(PROJ_DIR)/lib_sdk/components/libraries/delay \
  -I$(PROJ_DIR)/lib_sdk/components/libraries/log/src \
  -I$(PROJ_DIR)/lib_sdk/components/libraries/atomic \
  -I$(PROJ_DIR)/lib_sdk/components/libraries/experimental_section_vars \
  -I$(PROJ_DIR)/lib_sdk/components/libraries/memobj \
  -I$(PROJ_DIR)/lib_sdk/components/toolchain/cmsis/include \
  -I$(PROJ_DIR)/lib_sdk/components/boards \
  -I$(PROJ_DIR)/lib_sdk/components/drivers_ext/ili9341 \
  -I$(PROJ_DIR)/lib_sdk/components/drivers_ext/st7735 \
  -I$(PROJ_DIR)/lib_sdk/components/drivers_nrf/nrf_soc_nosd \
  -I$(PROJ_DIR)/lib_sdk/modules/nrfx/hal \
  -I$(PROJ_DIR)/lib_sdk/modules/nrfx \
  -I$(PROJ_DIR)/lib_sdk/modules/nrfx/drivers/include \
  -I$(PROJ_DIR)/lib_sdk/modules/nrfx/drivers \
  -I$(PROJ_DIR)/lib_sdk/modules/nrfx/mdk \
  -I$(PROJ_DIR)/lib_sdk/integration/nrfx/legacy \
  -I$(PROJ_DIR)/lib_sdk/integration/nrfx \
  -I$(PROJ_DIR)/lib_sdk/external/segger_rtt \
  -I$(PROJ_DIR)/lib_sdk/external/fprintf \
  -I$(PROJ_DIR)/lib_sdk/external/thedotfactory_fonts \
  -I$(PROJ_DIR)/lib_project \


OBJECT_DIRECTORY = _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

#flags common to all targets
CFLAGS += -DNRF52840_XXAA
CFLAGS += -DBOARD_PCA10056
# Enable the following for FPGA device
#CFLAGS += -DFPGA_DEVICE 
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall -O3 -g3
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums 
# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys

# Assembler flags
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DNRF52840_XXAA
ASMFLAGS += -DBOARD_PCA10056

#default target - first one defined
default: clean generate_hex

#building all targets
all: clean
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e cleanobj
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e generate_hex

#target for printing all targets
help:
	@echo following targets are available:
	@echo 	generate_hex

C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

vpath %.c $(C_PATHS)
vpath %.s $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

generate_hex: OUTPUT_FILENAME := radio_001
generate_hex: LINKER_SCRIPT = radio_001.ld

generate_hex: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -lm -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

## Create build directories
$(BUILD_DIRECTORIES):
	echo $(MAKEFILE_NAME)
	$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.s
	@echo Assembly file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<
# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -lm -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

finalize: genbin genhex echosize

genbin:
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
genhex: 
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex
echosize:
	-@echo ''
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ''

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o
flash: generate_hex
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/radio_001.hex
	nrfjprog --program $(OUTPUT_BINARY_DIRECTORY)/radio_001.hex -f nrf52  --sectorerase
	nrfjprog --reset -f nrf52
