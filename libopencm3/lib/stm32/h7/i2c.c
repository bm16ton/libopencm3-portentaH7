/** @defgroup i2c_file I2C
 *
 * @ingroup STM32F3xx
 *
 * @brief <b>libopencm3 STM32F3xx I2C</b>
 *
 * @version 1.0.0
 *
 * @date 15 October 2012
 *
 * LGPL License Terms @ref lgpl_license
 */

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

#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/syscfg.h>

void enable_i2c1_fmp(bool enable)
{
    if (enable == 1) {
        SYSCFG_PMCR |= SYSCFG_PMCR_I2C1_FMP;
    }
    if (enable == 0) {
        SYSCFG_PMCR &= ~SYSCFG_PMCR_I2C1_FMP;
    }
}

void enable_i2c2_fmp(bool enable)
{
    if (enable == 1) {
        SYSCFG_PMCR |= SYSCFG_PMCR_I2C2_FMP;
    }
    if (enable == 0) {
        SYSCFG_PMCR &= ~SYSCFG_PMCR_I2C2_FMP;
    }
}

void enable_i2c3_fmp(bool enable)
{
    if (enable == 1) {
        SYSCFG_PMCR |= SYSCFG_PMCR_I2C3_FMP;
    }
    if (enable == 0) {
        SYSCFG_PMCR &= ~SYSCFG_PMCR_I2C3_FMP;
    }
}

void enable_i2c4_fmp(bool enable)
{
    if (enable == 1) {
        SYSCFG_PMCR |= SYSCFG_PMCR_I2C4_FMP;
    }
    if (enable == 0) {
        SYSCFG_PMCR &= ~SYSCFG_PMCR_I2C4_FMP;
    }
}

void enable_pb6_fmp(bool enable)
{
    if (enable == 1) {
        SYSCFG_PMCR |= SYSCFG_PMCR_I2C_PB6_FMP;
    }
    if (enable == 0) {
        SYSCFG_PMCR &= ~SYSCFG_PMCR_I2C_PB6_FMP;
    }
}

void enable_pb7_fmp(bool enable)
{
    if (enable == 1) {
        SYSCFG_PMCR |= SYSCFG_PMCR_I2C_PB7_FMP;
    }
    if (enable == 0) {
        SYSCFG_PMCR &= ~SYSCFG_PMCR_I2C_PB7_FMP;
    }
}

void enable_pb8_fmp(bool enable)
{
    if (enable == 1) {
        SYSCFG_PMCR |= SYSCFG_PMCR_I2C_PB8_FMP;
    }
    if (enable == 0) {
        SYSCFG_PMCR &= ~SYSCFG_PMCR_I2C_PB8_FMP;
    }
}

void enable_pb9_fmp(bool enable)
{
    if (enable == 1) {
        SYSCFG_PMCR |= SYSCFG_PMCR_I2C_PB9_FMP;
    }
    if (enable == 0) {
        SYSCFG_PMCR &= ~SYSCFG_PMCR_I2C_PB9_FMP;
    }
}

void enable_i2c_fmp_port(uint32_t i2c)
{
		switch (i2c) {
	case I2C1:
		enable_i2c1_fmp(true);
		break;
#if defined(I2C2_BASE)
	case I2C2:
		enable_i2c2_fmp(true);
		break;
#endif
#if defined(I2C3_BASE)
	case I2C3:
		enable_i2c3_fmp(true);
		break;
#endif
#if defined(I2C4_BASE)
	case I2C4:
		enable_i2c4_fmp(true);
		break;
#endif
	default:
		break;
	}
}
