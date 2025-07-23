CMSIS_ROOT = ..

CC := arm-none-eabi-gcc
AR := arm-none-eabi-ar

CFLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
 -Os -ffunction-sections -fdata-sections -DNDEBUG -Wall -Wextra -Werror \
 -fshort-enums -Wno-unused-parameter

# Includes
CMSIS_CORE := $(CMSIS_ROOT)/CMSIS_5/CMSIS/Core/Include
DSP_INCLUDES := \
  $(CMSIS_ROOT)/Include \
  $(CMSIS_ROOT)/PrivateInclude \
  $(CMSIS_ROOT)/Core/Include/

INC_FLAGS := $(addprefix -I, $(DSP_INCLUDES) $(CMSIS_CORE))
CFLAGS += $(INC_FLAGS)

BUILDDIR := build

SRCS := \
  TransformFunctions/arm_rfft_fast_f32.c \
  ComplexMathFunctions/arm_cmplx_mag_f32.c \
  StatisticsFunctions/arm_max_f32.c \
  CommonTables/arm_common_tables.c \
  TransformFunctions/arm_cfft_init_f32.c \
  TransformFunctions/arm_cfft_f32.c \
  CommonTables/arm_const_structs.c \
  TransformFunctions/arm_bitreversal2.c \
  TransformFunctions/arm_cfft_radix8_f32.c \
  TransformFunctions/arm_rfft_fast_init_f32.c \
  FilteringFunctions/arm_fir_f32.c \
  FilteringFunctions/arm_fir_init_f32.c

OBJS := $(SRCS:%.c=$(BUILDDIR)/%.o)

all: $(BUILDDIR)/libCMSISDSP.a

$(BUILDDIR)/libCMSISDSP.a: $(OBJS)
	$(AR) rcs $@ $^

$(BUILDDIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILDDIR)

.PHONY: all clean
