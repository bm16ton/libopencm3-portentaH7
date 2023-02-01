/** @defgroup exti_defines EXTI Defines
 *
 * @ingroup STM32H7xx_defines
 *
 * @brief <b>Defined Constants and Types for the STM32H7xx EXTI Control</b>
 *
 * @version 1.0.0
 *
 * LGPL License Terms @ref lgpl_license
 *  */
/*
 * This file is part of the libopencm3 project.
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
/**@{*/
#ifndef LIBOPENCM3_EXTI_H
#define LIBOPENCM3_EXTI_H

#include <libopencm3/stm32/common/exti_common_all.h>
#include <libopencm3/stm32/common/exti_common_v2.h>

/** EXTI CPU Event Pending Register 1 */
#define EXTI_CPUPR1	MMIO32(EXTI_BASE + 0xC8)
#define EXTI_PR 	EXTI_CPUPR1
#define EXTI_C2CPUPR1	EXTI_CPUPR1
#define EXTI_C2PR 	EXTI_C2CPUPR1

#define EXTI_C2IMR1			MMIO32(EXTI_BASE + 0xC0)
#define EXTI_C2IMR			EXTI_C2IMR1

/** EXTI Event Mask Registers 1 */
#define EXTI_C2EMR1			MMIO32(EXTI_BASE + 0xC4)
#define EXTI_C2EMR			EXTI_C2EMR1

/** EXTI Interrupt Mask Registers C2 M4 */
#define EXTI_C2IMR2			MMIO32(EXTI_BASE + 0xD0)
/** EXTI Event Mask Registers C2 M4 */
#define EXTI_C2EMR2			MMIO32(EXTI_BASE + 0xD4)
/** EXTI CPU Event Pending Register 2 C2 M4 */
#define EXTI_C2CPUPR2	MMIO32(EXTI_BASE + 0xD8)

/** EXTI Interrupt Mask Registers 3 C2 M4 */
#define EXTI_C2IMR3			MMIO32(EXTI_BASE + 0xE0)
/** EXTI Event Mask Registers C2 M4 */
#define EXTI_C2EMR3			MMIO32(EXTI_BASE + 0xE4)
/** EXTI CPU Event Pending Register 3 C2 M4 */
#define EXTI_C2CPUPR3	MMIO32(EXTI_BASE + 0xE8)
BEGIN_DECLS

END_DECLS

#endif  /* LIBOPENCM3_EXTI_H */
/**@}*/
