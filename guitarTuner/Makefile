PROJECT = main
BUILD_DIR = bin

CFILES = teste.c							\
libs/cmsis_dsp/Source/TransformFunctions/arm_cfft_radix4_f32.c		\
libs/cmsis_dsp/Source/TransformFunctions/arm_cfft_radix4_init_f32.c	\
libs/cmsis_dsp/Source/TransformFunctions/arm_rfft_fast_f32.c		\
libs/cmsis_dsp/Source/ComplexMathFunctions/arm_cmplx_mag_f32.c		\
libs/cmsis_dsp/Source/StatisticsFunctions/arm_max_f32.c			\
libs/cmsis_dsp/Source/CommonTables/arm_common_tables.c			\
libs/cmsis_dsp/Source/TransformFunctions/arm_cfft_init_f32.c		\
libs/cmsis_dsp/Source/TransformFunctions/arm_cfft_f32.c			\
libs/cmsis_dsp/Source/TransformFunctions/arm_bitreversal.c		\
libs/cmsis_dsp/Source/FastMathFunctions/arm_sin_f32.c			\
libs/cmsis_dsp/Source/CommonTables/arm_const_structs.c			\
libs/cmsis_dsp/Source/TransformFunctions/arm_bitreversal2.c		\
libs/cmsis_dsp/Source/TransformFunctions/arm_cfft_radix8_f32.c 		\
libs/cmsis_dsp/Source/TransformFunctions/arm_rfft_fast_init_f32.c \
libs/cmsis_dsp/Source/FastMathFunctions/arm_cos_f32.c

# TODO - you will need to edit these two lines!
DEVICE=stm32f411ce
OOCD_FILE = board/stm32f4discovery.cfg

# You shouldn't have to edit anything below here.
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
INCLUDES += -I libs/cmsis_dsp/Include/
INCLUDES += -I libs/cmsis_dsp/Include/dsp/
INCLUDES += -I libs/cmsis_dsp/Core/Include/
OPENCM3_DIR=../libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include ../rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk

flashbin:
	st-flash write main.bin 0x8000000

