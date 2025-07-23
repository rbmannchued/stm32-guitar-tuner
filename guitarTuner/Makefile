
PROJECT = main
BUILD_DIR = bin

OPENCM3_DIR = ../libopencm3
DEVICE = stm32f411ce

CFILES = guitarTuner.c \
         libs/ssd1306_libopencm3/ssd1306.c \
         libs/ssd1306_libopencm3/ssd1306_fonts.c

CMSISDSP_LIB = libs/cmsis_dsp/Source/build/libCMSISDSP.a

# Compila CMSIS-DSP antes do projeto
$(PROJECT).elf: $(CMSISDSP_LIB)

$(CMSISDSP_LIB):
	$(MAKE) -C libs/cmsis_dsp/Source

INCLUDES += -Ilibs/cmsis_dsp/Include
INCLUDES += -Ilibs/cmsis_dsp/Include/dsp
INCLUDES += -Ilibs/cmsis_dsp/Core/Include
INCLUDES += -Ilibs/ssd1306_libopencm3
INCLUDES += -I.

LDLIBS += $(CMSISDSP_LIB)

include $(OPENCM3_DIR)/mk/genlink-config.mk
include ../rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk

# Regra para gerar .bin a partir do .elf
$(PROJECT).bin: $(PROJECT).elf
	arm-none-eabi-objcopy -Obinary $< $@


bin: $(PROJECT).bin

#Flash
flashbin:
	st-flash write $(PROJECT).bin 0x8000000
