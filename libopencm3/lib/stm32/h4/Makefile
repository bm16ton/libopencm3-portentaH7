##
## This file is part of the libopencm3 project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
## Copyright (C) 2013 Alexandru Gagniuc <mr.nuke.me@gmail.com>
##
## This library is free software: you can redistribute it and/or modify
## it under the terms of the GNU Lesser General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This library is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU Lesser General Public License for more details.
##
## You should have received a copy of the GNU Lesser General Public License
## along with this library.  If not, see <http://www.gnu.org/licenses/>.
##

LIBNAME		= libopencm3_stm32h4
SRCLIBDIR	?= ../..

FP_FLAGS	?= -mfloat-abi=hard -mfpu=fpv4-sp-d16
CC		= $(PREFIX)gcc
AR		= $(PREFIX)ar
TGT_CFLAGS	= -Os \
		  -Wall -Wextra -Wimplicit-function-declaration \
		  -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes \
		  -Wundef -Wshadow \
		  -I../../../include -fno-common \
		  -mcpu=cortex-m4 -mthumb $(FP_FLAGS) \
		  -Wstrict-prototypes \
		  -ffunction-sections -fdata-sections -MD -DSTM32H4
TGT_CFLAGS      += $(DEBUG_FLAGS)
TGT_CFLAGS	+= $(STANDARD_FLAGS)
# ARFLAGS	= rcsv
ARFLAGS		= rcs

OBJS += adc.o adc_common_v2.o adc_common_v2_multi.o
OBJS += dac_common_all.o dac_common_v2.o
OBJS += dmamux.o
OBJS += exti_common_all.o
OBJS += fdcan.o fdcan_common.o
OBJS += flash_common_all.o flash_common_f.o flash_common_f24.o
OBJS += fmc_common_f47.o
OBJS += gpio_common_all.o gpio_common_f0234.o
OBJS += pwr.o rcc.o
OBJS += rcc_common_all.o
OBJS += rng_common_v1.o
OBJS += spi_common_v3.o
OBJS += swpmi.o
OBJS += syscfg.o
OBJS += i2c_common_v2.o
OBJS += timer_common_all.o
OBJS += usart_common_all.o usart_common_v2.o usart_common_fifos.o
OBJS += quadspi_common_v1.o
OBJS += usb.o usb_control.o usb_standard.o
OBJS += usb_cdc.o
OBJS += usb_hid.o
OBJS += usb_audio.o usb_cdc.o usb_midi.o
OBJS += usb_dwc_common.o usb_f207.o

VPATH += ../../usb:../:../../cm3:../common



include ../../Makefile.include
