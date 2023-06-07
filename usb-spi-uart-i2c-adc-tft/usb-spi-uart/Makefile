PROJECT = usbhs
BUILD_DIR = bin

ARCH_FLAGS = -Wl,-Map=mapfile -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard
LDLIBS += -lm
CFILES += cdcacm.c bulkspi.c usb-2-i2c/util.c usb-2-i2c/bitops.c usb-2-i2c/regdump.c usb-2-i2c/i2c_ctx.c tft_stm32_spi.c debugen.c adctest.c xpt2046.c console/console.c
CFLAGS +=-std=c99 -g3 -MD -fno-aggressive-loop-optimizations
LDFLAGS += -Wl,--print-memory-usage -nostartfiles -lc -lnosys \
	-Wl,-Map=mapfile -mthumb
DEVICE=stm32h747


# You shouldn't have to edit anything below here.
VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
OPENCM3_DIR=../libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include ../rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk
