PROJECT = usbhs
BUILD_DIR = bin

ARCH_FLAGS = -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard

CFILES += cdcacm.c st7789_stm32_spi.c qspi_main2.c i2c.c adctest.c

DEVICE=stm32h747


# You shouldn't have to edit anything below here.
VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
OPENCM3_DIR=../libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include ../rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk
