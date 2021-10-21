#
# Copyright (c) 2016 - 2019 - InvnSense Inc
#

SDK_VERSION := 1.0.0

# Windows
ifeq ($(OS),Windows_NT)
	RM = del /Q
	MKDIR = mkdir
	FixPath = $(subst /,\,$1)
endif

# 
# Basic configuration
#
ifndef INVN_TOOLCHAIN_PATH
	INVN_TOOLCHAIN_PATH="%ProgramFiles(x86)%"\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin
endif
TOOLCHAIN_PATH := $(addsuffix \,$(INVN_TOOLCHAIN_PATH))

PREFIX  ?= $(TOOLCHAIN_PATH)arm-none-eabi-
GCC     := $(PREFIX)gcc
OBJCOPY := $(PREFIX)objcopy
SIZE    := $(PREFIX)size

OUTPUT_DIR  ?= output
OUTPUT      ?= libEMDCore.a
EXTRA_FLAGS ?=
FLAGS   ?= -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 $(EXTRA_FLAGS)
CFLAGS  ?= $(FLAGS) -pipe -std=c99 -fdata-sections -ffunction-sections -Wall -Wextra -Wno-unused-parameter -fsigned-char \
	-mlong-calls -fno-strict-aliasing -Wmissing-prototypes -Werror-implicit-function-declaration -Wpointer-arith -Wchar-subscripts \
	-Wcomment -Wformat=2 -Wimplicit-int -Wmain -Wparentheses -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs -Wunused -Wuninitialized -Wunknown-pragmas \
	-Wundef -Wbad-function-cast -Wwrite-strings -Wsign-compare -Waggregate-return -Wmissing-declarations -Wformat -Wmissing-format-attribute \
	-Wno-deprecated-declarations -Wpacked -Wunreachable-code --param max-inline-insns-single=500	
VERSION ?= $(SDK_VERSION)

ifeq ($(DEBUG),yes)
	CFLAGS += -O0 -g3
else
	CFLAGS += -O2
endif

EXTRA_IDIRS ?=
EXTRA_DEPS  ?=
EXTRA_CSRCS ?=
EXTRA_DEFS  ?=
EXTRA_LIBS  ?=
EXTRA_LDIRS ?=

# List of files and dependencies (automatically-generated)
IDIRS   +=  \
	sources/board-hal \
	sources/Invn/Devices/Drivers/Icm207xx \
	sources/Invn/Devices \
	sources/Invn/DynamicProtocol \
	sources/Invn/EmbUtils \
	sources/Invn \
	sources/Invn/LibAlgo
LDIRS   +=  \
	prebuilt/lib
DEPS    +=  \
	sources/EMDFwVersion.h \
	sources/Invn/Devices/Drivers/Icm207xx/Icm207xx.h \
	sources/Invn/Devices/Drivers/Icm207xx/Icm207xxAuxTransport.h \
	sources/Invn/Devices/Drivers/Icm207xx/Icm207xxCtrl.h \
	sources/Invn/Devices/Drivers/Icm207xx/Icm207xxDefs.h \
	sources/Invn/Devices/Drivers/Icm207xx/Icm207xxSelfTest.h \
	sources/Invn/Devices/Drivers/Icm207xx/Icm207xxSerif.h \
	sources/Invn/Devices/Drivers/Icm207xx/Icm207xxSetup.h \
	sources/Invn/Devices/Drivers/Icm207xx/Icm207xxTransport.h \
	sources/Invn/Devices/SensorConfig.h \
	sources/Invn/Devices/SensorTypes.h \
	sources/Invn/DynamicProtocol/DynProtocol.h \
	sources/Invn/DynamicProtocol/DynProtocolTransport.h \
	sources/Invn/DynamicProtocol/DynProtocolTransportUart.h \
	sources/Invn/EmbUtils/DataConverter.h \
	sources/Invn/EmbUtils/ErrorHelper.h \
	sources/Invn/EmbUtils/InvAssert.h \
	sources/Invn/EmbUtils/InvBool.h \
	sources/Invn/EmbUtils/InvError.h \
	sources/Invn/EmbUtils/InvExport.h \
	sources/Invn/EmbUtils/InvProtocol.h \
	sources/Invn/EmbUtils/InvQueue.h \
	sources/Invn/EmbUtils/InvScheduler.h \
	sources/Invn/EmbUtils/Message.h \
	sources/Invn/EmbUtils/RingBuffer.h \
	sources/Invn/EmbUtils/RingByteBuffer.h \
	sources/Invn/IDDVersion.h \
	sources/Invn/LibAlgo/LibAlgo.h \
	sources/Invn/VSensor/VSensorConfig.h \
	sources/Invn/VSensor/VSensorData.h
CSRCS   +=  \
	sources/Invn/Devices/Drivers/Icm207xx/Icm207xxAuxTransport.c \
	sources/Invn/Devices/Drivers/Icm207xx/Icm207xxCtrl.c \
	sources/Invn/Devices/Drivers/Icm207xx/Icm207xxSelfTest.c \
	sources/Invn/Devices/Drivers/Icm207xx/Icm207xxSetup.c \
	sources/Invn/Devices/Drivers/Icm207xx/Icm207xxTransport.c \
	sources/Invn/Devices/Sensor.c \
	sources/Invn/DynamicProtocol/DynProtocol.c \
	sources/Invn/DynamicProtocol/DynProtocolTransportUart.c \
	sources/Invn/EmbUtils/DataConverter.c \
	sources/Invn/EmbUtils/ErrorHelper.c \
	sources/Invn/EmbUtils/InvProtocol.c \
	sources/Invn/EmbUtils/InvQueue.c \
	sources/Invn/EmbUtils/InvScheduler.c \
	sources/Invn/EmbUtils/Message.c \
	sources/Invn/EmbUtils/RingByteBuffer.c
DEFS    +=  \
	-DINV_MSG_ENABLE=INV_MSG_LEVEL_VERBOSE \
	-DASSERT \
	-DNDEBUG \
	-DARM_MATH_CM4
LIBS    +=  \
	-lAlgoInvn \
	-lMLMath
LDSCRIPT ?= 

ifdef INVN_EMD_SDK_PATH
# fixup path according to SDK root
	CSRCS := $(addprefix $(INVN_EMD_SDK_PATH)/,$(CSRCS))
	DEPS := $(addprefix $(INVN_EMD_SDK_PATH)/,$(DEPS))
	IDIRS := $(addprefix $(INVN_EMD_SDK_PATH)/,$(IDIRS))
	LDIRS := $(addprefix $(INVN_EMD_SDK_PATH)/,$(LDIRS))
ifneq ($(LDSCRIPT),)
	LDSCRIPT := $(addprefix $(INVN_EMD_SDK_PATH)/,$(LDSCRIPT))
endif
endif

ifneq ($(LDSCRIPT),)
	LDSCRIPT := -T $(LDSCRIPT)
endif
DEFS += -DVERSION=$(VERSION)
IDIRS := $(addprefix -I,$(IDIRS))
LDIRS := $(addprefix -L,$(LDIRS))
CSRCS += $(EXTRA_CSRCS)
DEPS  += $(EXTRA_DEPS)
DEFS  += $(EXTRA_DEFS)
IDIRS += $(EXTRA_IDIRS)
LDIRS += $(EXTRA_LDIRS)
LIBS  += $(EXTRA_LIBS)

# Get objects lists according to sources list
OBJS := $(notdir $(CSRCS))
OBJS := $(addsuffix .o,$(OBJS))
OBJS := $(addprefix $(OUTPUT_DIR)/objs/,$(OBJS))

#
# Rules
#
all: $(OUTPUT)
	@echo "SUCCESS: output available under '$(OUTPUT_DIR)/$(OUTPUT)'"

# Generic rule for compiling 
define CRule
$(OUTPUT_DIR)/objs/$(notdir $(1).o): $(1) $(DEPS)
	$(GCC) -c $(CFLAGS) $(IDIRS) $(DEFS) $(1) -o $(OUTPUT_DIR)/objs/$(notdir $(1)).o
endef
$(foreach _,$(CSRCS), $(eval $(call CRule,$_)))

$(OUTPUT): $(OUTPUT_DIR)/$(OUTPUT)

$(OUTPUT_DIR)/objs:
	@$(MKDIR) $(call FixPath, $(OUTPUT_DIR)/objs)

$(OUTPUT_DIR)/$(OUTPUT): $(OUTPUT_DIR)/objs $(OBJS)
	$(PREFIX)ar rcs $(OUTPUT_DIR)/$(OUTPUT) $(OBJS)

clean:
	$(RM) $(call FixPath,$(OBJS))
	$(RM) $(call FixPath, $(OUTPUT_DIR)/$(OUTPUT))

# for Makefile debug
print-%  : ; @echo $* = $($*)

.phony: all clean size 
