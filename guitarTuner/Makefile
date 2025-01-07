PROJECT = main
BUILD_DIR = bin

CFILES = my-project.c \
				 libs/cmsis_dsp/Source/TransformFunctions/arm_cfft_radix4_f32.c \
				 libs/cmsis_dsp/Source/TransformFunctions/arm_cfft_radix4_q31.c \
				 libs/cmsis_dsp/Source/TransformFunctions/arm_cfft_radix4_q15.c \
				 libs/cmsis_dsp/Source/StatisticsFunctions/arm_rms_f32.c

 

# TODO - you will need to edit these two lines!
DEVICE=stm32f411ce
OOCD_FILE = board/stm32f4discovery.cfg

# You shouldn't have to edit anything below here.
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
INCLUDES += -I./libs/cmsis_dsp/Include/
INCLUDES += -I ./libs/cmsis_dsp/Core/Include/
OPENCM3_DIR=../libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include ../rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk
