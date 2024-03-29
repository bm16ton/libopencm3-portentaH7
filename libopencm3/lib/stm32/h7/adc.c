/** @addtogroup adc_file ADC peripheral API
 * @ingroup peripheral_apis
 * LGPL License Terms @ref lgpl_license
 */
/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2020 Matt Walker <m@allocor.tech>
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

#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dbgmcu.h>
#include <libopencm3/stm32/syscfg.h>
/** @brief Set ADC clock source, mode, and boost
 *
 * The H7 ADC clock can be sourced either from the peripheral bus clock or from
 * a clock configured in the RCC. This is set via \p mode. For silicon revisions
 * of X and greater a clock division by 2 occurs. Then for all revisions the
 * clock is further divided by \p prescale.
 *
 * The ADC may need to be boosted depending on the resulting frequency and
 * silicon revision. @sa adc_set_boost
 *
 * @param adc Peripheral of choice @ref adc_reg_base
 * @param mode ADC kernel clock source
 * @param prescale Divider for the ADC clock
 *
 * @returns The total clock divisor, including revision specific division.
 */
void adc_enable_dual_mode(uint32_t adc) {
    ADC_CCR(adc) |= (ADC_CCR_DUAL_MASK << 0x00);
}

void adc_set_dual_mode_type(uint32_t adc, uint32_t type) {
    ADC_CCR(adc) |= type;
}

void adc_set_dual_mode_data_format(uint32_t adc, uint32_t format) {
    ADC_CCR(adc) |= format;
}

uint32_t adc_read_multi_master(uint32_t adc)
{
	return ADC_CDR(adc);
}

uint32_t adc_read_multi_slave(uint32_t adc)
{
	return ADC_CDR(adc) & ADC_CDR_SLAVE_OFFSET;
}

uint32_t adc_read_multi_32b(uint32_t adc)
{
	return ADC_CDR2(adc);
}


uint32_t adc_set_clock_param(uint32_t adc, enum adc_ccr_ckmode mode,
		         enum adc_ccr_presc prescale)
{
	uint32_t reg32 = ADC_CCR(adc);
	reg32 &= ~(ADC_CCR_CKMODE_MASK | ADC_CCR_PRESC_MASK);
	reg32 |= mode | prescale;
	ADC_CCR(adc) = reg32;

	uint32_t div = (prescale >> 17);
	div *= ((mode >> 16) | 0x1);

	if ((DBGMCU_IDCODE & DBGMCU_IDCODE_REV_ID_MASK) >= DBGMCU_IDCODE_REV_ID_X)
	{
		div *= 2;
	}

	return div;
}

void adc_set_dma_dmngt_circ(uint32_t adc)
{
	ADC_CFGR1(adc) |= ADC_CFGR1_DMNGT_DMA_CIRC;
//	ADC_CFGR1(adc) |= (0x1 << 0xb);

}
/** Set the boost parameter of the ADC which depends on the clock frequency and
 * silicon revision.
 * @param adc Peripheral of choice @ref adc_reg_base
 * @param adc_clock_freq ADC kernel clock frequency in Hz
 * @sa adc_set_clock_param
 */
void adc_set_boost(uint32_t adc, uint32_t adc_clock_freq)
{
	if ((DBGMCU_IDCODE & DBGMCU_IDCODE_REV_ID_MASK) >= DBGMCU_IDCODE_REV_ID_X)
	{
		uint32_t adc_cr_reg = ADC_CR(adc) & (~ADC_CR_BOOST_V_MASK);
		if (adc_clock_freq > 25000000)
		{
			adc_cr_reg |= ADC_CR_BOOST_V_25000_50000_KHZ;
		}
		else if (adc_clock_freq > 12500000)
		{
			adc_cr_reg |= ADC_CR_BOOST_V_12500_25000_KHZ;
		}
		else if (adc_clock_freq > 6250000)
		{
			adc_cr_reg |= ADC_CR_BOOST_V_6250_12500_KHZ;
		}
		else
		{
			adc_cr_reg |= ADC_CR_BOOST_V_0_6250_KHZ;
		}
	}
	else
	{
		if (adc_clock_freq > 20000000)
		{
			ADC_CR(adc) |= ADC_CR_BOOST_Y_GTE_20_MHZ;
		}
		else
		{
			ADC_CR(adc) &= (~ADC_CR_BOOST_Y_GTE_20_MHZ);
		}
	}
}

/** For each channel selected through SQRx or JSQRx, the corresponding ADC_PCSEL bit
 * must be configured.
 */
void adc_set_pcsel(uint32_t adc, uint8_t length, const uint8_t channel[])
{
	uint32_t pcsel_reg = 0;
	for (uint8_t i = 0; i < length; ++i)
	{
		pcsel_reg |= 1 << (0x1F & channel[i]);
	}
	ADC_PCSEL(adc) = pcsel_reg;
}

/**
 * Enable the ADC Voltage regulator
 * Before any use of the ADC, the ADC Voltage regulator must be enabled.
 * You must wait up to 10uSecs afterwards before trying anything else.
 * @param[in] adc ADC block register address base
 * @sa adc_disable_regulator
 */
void adc_enable_regulator(uint32_t adc)
{
	ADC_CR(adc) &= ~ADC_CR_DEEPPWD;
	ADC_CR(adc) |= ADC_CR_ADVREGEN;
}

/**
 * Disable the ADC Voltage regulator
 * You can disable the adc vreg when not in use to save power
 * @param[in] adc ADC block register address base
 * @sa adc_enable_regulator
 */
void adc_disable_regulator(uint32_t adc)
{
	ADC_CR(adc) &= ~ADC_CR_ADVREGEN;
}

/* Connects switch between PA0 and PA0_C which has
direct connection to ADC12_INN1/ADC12_INP0 not sure what isuues
could arise from using PA0(ADC1_INP16)or
PA0_C(ADC12_INN1/ADC12_INP0) as analog in with switch connected */
void enable_pa0so(bool enable)
{
    if (enable == 1) {
        SYSCFG_PMCR |= SYSCFG_PMCR_PA0SO;
    }
    if (enable == 0) {
        SYSCFG_PMCR &= ~SYSCFG_PMCR_PA0SO;
    }
}

/* Connects switch between PA1 and PA1_C which has
direct connection to ADC12_INP1 not sure what isuues
could arise from using PA1(ADC1_INN16/ADC1_INP17)or
PA1_C(ADC12_INP1) as analog in with switch connected */
void enable_pa1so(bool enable)
{
    if (enable == 1) {
        SYSCFG_PMCR |= SYSCFG_PMCR_PA1SO;
    }
    if (enable == 0) {
        SYSCFG_PMCR &= ~SYSCFG_PMCR_PA1SO;
    }
}

/* Connects switch between PC2 and PC2_C which has
direct connection to ADC3_INN1/ADC3_INP0 not sure what isuues
could arise from using PC2(ADC123_INN11/ADC123_INP12 )or
PC2_C(ADC3_INN1/ADC3_INP0) as analog in with switch connected */
void enable_pc2so(bool enable)
{
    if (enable == 1) {
        SYSCFG_PMCR |= SYSCFG_PMCR_PC2SO;
    }
    if (enable == 0) {
        SYSCFG_PMCR &= ~SYSCFG_PMCR_PC2SO;
    }
}

/* Connects switch between PC3 and PC3_C which has
direct connection to ADC3_INP1 not sure what isuues
could arise from using PC3(ADC12_INN12/ADC12_INP13 )or
PC3_C(ADC3_INP1) as analog in with switch connected */
void enable_pc3so(bool enable)
{
    if (enable == 1) {
        SYSCFG_PMCR |= SYSCFG_PMCR_PC3SO;
    }
    if (enable == 0) {
        SYSCFG_PMCR &= ~SYSCFG_PMCR_PC3SO;
    }
}

