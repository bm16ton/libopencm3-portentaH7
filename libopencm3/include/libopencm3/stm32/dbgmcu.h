/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2011 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIBOPENCM3_STM32_DBGMCU_H
#define LIBOPENCM3_STM32_DBGMCU_H

#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/memorymap.h>

/* --- DBGMCU registers ---------------------------------------------------- */

/* Debug MCU IDCODE register (DBGMCU_IDCODE) */
#define DBGMCU_IDCODE			MMIO32(DBGMCU_BASE + 0x00)

/* Debug MCU configuration register (DBGMCU_CR) */
/* Note: Only 32bit access supported. */
#define DBGMCU_CR			MMIO32(DBGMCU_BASE + 0x04)

/* --- DBGMCU_IDCODE values ------------------------------------------------ */

#define DBGMCU_IDCODE_DEV_ID_MASK	0x00000fff
#define DBGMCU_IDCODE_REV_ID_MASK	0xffff0000

/** Known MCUs */
enum dbgmcu_idcode_dev_id {
	DBGMCU_IDCODE_DEV_ID_STM32H742 = 0x450,
	DBGMCU_IDCODE_DEV_ID_STM32H743 = 0x450,
	DBGMCU_IDCODE_DEV_ID_STM32H750 = 0x450,
	DBGMCU_IDCODE_DEV_ID_STM32H753 = 0x450,
};

/** Known device revisions */
enum dbgmu_idcode_rev_id {
#if defined(STM32H7) || defined(STM32H4)
	DBGMCU_IDCODE_REV_ID_Z = 0x1001 << 16,
	DBGMCU_IDCODE_REV_ID_Y = 0x1003 << 16,
	DBGMCU_IDCODE_REV_ID_X = 0x2001 << 16,
	DBGMCU_IDCODE_REV_ID_V = 0x2003 << 16,
#endif
};

/* --- DBGMCU_CR values ---------------------------------------------------- */

/* Bit 31: Reserved. */

/* Bits [24:22]: Reserved, must be kept cleared. */

/* Bits [4:3]: Reserved. */

#define DBGMCU_CR_SLEEP			0x00000001
#define DBGMCU_CR_STOP			0x00000002
#define DBGMCU_CR_STANDBY		0x00000004
#define DBGMCU_CR_TRACE_IOEN		0x00000020
#define DBGMCU_CR_TRACE_MODE_MASK	0x000000C0
#define DBGMCU_CR_TRACE_MODE_ASYNC	0x00000000
#define DBGMCU_CR_TRACE_MODE_SYNC_1	0x00000040
#define DBGMCU_CR_TRACE_MODE_SYNC_2	0x00000080
#define DBGMCU_CR_TRACE_MODE_SYNC_4	0x000000C0
#define DBGMCU_CR_IWDG_STOP		0x00000100
#define DBGMCU_CR_WWDG_STOP		0x00000200
#define DBGMCU_CR_TIM1_STOP		0x00000400
#define DBGMCU_CR_TIM2_STOP		0x00000800
#define DBGMCU_CR_TIM3_STOP		0x00001000
#define DBGMCU_CR_TIM4_STOP		0x00002000
#define DBGMCU_CR_CAN1_STOP		0x00004000
#define DBGMCU_CR_I2C1_SMBUS_TIMEOUT	0x00008000
#define DBGMCU_CR_I2C2_SMBUS_TIMEOUT	0x00010000
#define DBGMCU_CR_TIM8_STOP		0x00020000
#define DBGMCU_CR_TIM5_STOP		0x00040000
#define DBGMCU_CR_TIM6_STOP		0x00080000
#define DBGMCU_CR_TIM7_STOP		0x00100000
#define DBGMCU_CR_CAN2_STOP		0x00200000

#endif
