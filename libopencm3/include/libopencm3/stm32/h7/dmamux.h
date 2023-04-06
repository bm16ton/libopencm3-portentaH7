/** @defgroup dmamux_defines DMAMUX Defines

@ingroup STM32G4xx_defines

@brief Defined Constants and Types for the STM32G4xx DMAMUX

@version 1.0.0

LGPL License Terms @ref lgpl_license
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

#pragma once
#include <libopencm3/stm32/memorymap.h>
/**@{*/

//#include <libopencm3/stm32/common/dmamux_common_all.h>

 /** @defgroup dmamux_reg_base DMAMUX register base addresses
  * @{
  */
#define DMAMUX1				DMAMUX1_BASE
#define DMAMUX2				DMAMUX2_BASE
/**@}*/


#define DMAMUX2                   DMAMUX2_BASE
#define DMAMUX2_CHANNEL0_BASE    (DMAMUX2_BASE)
#define DMAMUX2_CHANNEL1_BASE    (DMAMUX2_BASE + 0x0004UL)
#define DMAMUX2_CHANNEL2_BASE    (DMAMUX2_BASE + 0x0008UL)
#define DMAMUX2_CHANNEL3_BASE    (DMAMUX2_BASE + 0x000CUL)
#define DMAMUX2_CHANNEL4_BASE    (DMAMUX2_BASE + 0x0010UL)
#define DMAMUX2_CHANNEL5_BASE    (DMAMUX2_BASE + 0x0014UL)
#define DMAMUX2_CHANNEL6_BASE    (DMAMUX2_BASE + 0x0018UL)
#define DMAMUX2_CHANNEL7_BASE    (DMAMUX2_BASE + 0x001CUL)

#define DMAMUX1 DMAMUX1_BASE
#define DMAMUX1_CHANNEL0_BASE    (DMAMUX1_BASE)
#define DMAMUX1_CHANNEL1_BASE    (DMAMUX1_BASE + 0x0004UL)
#define DMAMUX1_CHANNEL2_BASE    (DMAMUX1_BASE + 0x0008UL)
#define DMAMUX1_CHANNEL3_BASE    (DMAMUX1_BASE + 0x000CUL)
#define DMAMUX1_CHANNEL4_BASE    (DMAMUX1_BASE + 0x0010UL)
#define DMAMUX1_CHANNEL5_BASE    (DMAMUX1_BASE + 0x0014UL)
#define DMAMUX1_CHANNEL6_BASE    (DMAMUX1_BASE + 0x0018UL)
#define DMAMUX1_CHANNEL7_BASE    (DMAMUX1_BASE + 0x001CUL)
#define DMAMUX1_CHANNEL8_BASE    (DMAMUX1_BASE + 0x0020UL)
#define DMAMUX1_CHANNEL9_BASE    (DMAMUX1_BASE + 0x0024UL)
#define DMAMUX1_CHANNEL10_BASE   (DMAMUX1_BASE + 0x0028UL)
#define DMAMUX1_CHANNEL11_BASE   (DMAMUX1_BASE + 0x002CUL)
#define DMAMUX1_CHANNEL12_BASE   (DMAMUX1_BASE + 0x0030UL)
#define DMAMUX1_CHANNEL12        DMAMUX1_CHANNEL12_BASE
#define DMAMUX1_CHANNEL13_BASE   (DMAMUX1_BASE + 0x0034UL)
#define DMAMUX1_CHANNEL14_BASE   (DMAMUX1_BASE + 0x0038UL)
#define DMAMUX1_CHANNEL15_BASE   (DMAMUX1_BASE + 0x003CUL)

/* --- H7DMAMUX_CxCR values ------------------------------------ */

/** @defgroup dmamux_cxcr_sync_id SYNCID Synchronization input selected
@{*/
#define H7DMAMUX_CxCR_SYNC_ID_EXTI_LINE0       0
#define H7DMAMUX_CxCR_SYNC_ID_EXTI_LINE1       1
#define H7DMAMUX_CxCR_SYNC_ID_EXTI_LINE2       2
#define H7DMAMUX_CxCR_SYNC_ID_EXTI_LINE3       3
#define H7DMAMUX_CxCR_SYNC_ID_EXTI_LINE4       4
#define H7DMAMUX_CxCR_SYNC_ID_EXTI_LINE5       5
#define H7DMAMUX_CxCR_SYNC_ID_EXTI_LINE6       6
#define H7DMAMUX_CxCR_SYNC_ID_EXTI_LINE7       7
#define H7DMAMUX_CxCR_SYNC_ID_EXTI_LINE8       8
#define H7DMAMUX_CxCR_SYNC_ID_EXTI_LINE9       9
#define H7DMAMUX_CxCR_SYNC_ID_EXTI_LINE10      10
#define H7DMAMUX_CxCR_SYNC_ID_EXTI_LINE11      11
#define H7DMAMUX_CxCR_SYNC_ID_EXTI_LINE12      12
#define H7DMAMUX_CxCR_SYNC_ID_EXTI_LINE13      13
#define H7DMAMUX_CxCR_SYNC_ID_EXTI_LINE14      14
#define H7DMAMUX_CxCR_SYNC_ID_EXTI_LINE15      15
#define H7DMAMUX_CxCR_SYNC_ID_DMAMUX_CH0_EVT   16
#define H7DMAMUX_CxCR_SYNC_ID_DMAMUX_CH1_EVT   17
#define H7DMAMUX_CxCR_SYNC_ID_DMAMUX_CH2_EVT   18
#define H7DMAMUX_CxCR_SYNC_ID_DMAMUX_CH3_EVT   19
#define H7DMAMUX_CxCR_SYNC_ID_LPTIM1_OUT       20
/**@}*/


/** @defgroup dmamux_cxcr_dmareq_id DMAREQID DMA request line selected
@{*/
/* DMA IDs dmamux1*/

#define H7DMAMUX_CxCR_DMAREQ_ID_dmamux1_req_gen0	1
#define H7DMAMUX_CxCR_DMAREQ_ID_dmamux1_req_gen1	2
#define H7DMAMUX_CxCR_DMAREQ_ID_dmamux1_req_gen2	3
#define H7DMAMUX_CxCR_DMAREQ_ID_dmamux1_req_gen3	4
#define H7DMAMUX_CxCR_DMAREQ_ID_dmamux1_req_gen4	5
#define H7DMAMUX_CxCR_DMAREQ_ID_dmamux1_req_gen5	6
#define H7DMAMUX_CxCR_DMAREQ_ID_dmamux1_req_gen6	7
#define H7DMAMUX_CxCR_DMAREQ_ID_dmamux1_req_gen7	8
#define H7DMAMUX_CxCR_DMAREQ_ID_adc1_dma                    9
#define H7DMAMUX_CxCR_DMAREQ_ID_adc2_dma					10
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM1_CH1                  11
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM1_CH2                  12
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM1_CH3                  13
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM1_CH4                  14
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM1_UP                    15
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM1_TRIG                 16
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM1_COM                 17
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM2_CH1                  18
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM2_CH2                  19
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM2_CH3                  20
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM2_CH4                  21
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM2_UP                    22
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM3_CH1                  23
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM3_CH2                  24
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM3_CH3                  25
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM3_CH4                  26
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM3_UP                    27
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM3_TRIG                 28
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM4_CH1                  29
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM4_CH2                  30
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM4_CH3                  31
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM4_UP                    32
#define H7DMAMUX_CxCR_DMAREQ_ID_i2c1_rx_dma               33
#define H7DMAMUX_CxCR_DMAREQ_ID_i2c1_tx_dma                34
#define H7DMAMUX_CxCR_DMAREQ_ID_i2c2_rx_dma               35
#define H7DMAMUX_CxCR_DMAREQ_ID_i2c2_tx_dma                36
#define H7DMAMUX_CxCR_DMAREQ_ID_spi1_rx_dma               37
#define H7DMAMUX_CxCR_DMAREQ_ID_spi1_tx_dma                38
#define H7DMAMUX_CxCR_DMAREQ_ID_spi2_rx_dma               39
#define H7DMAMUX_CxCR_DMAREQ_ID_spi2_tx_dma                40
#define H7DMAMUX_CxCR_DMAREQ_ID_usart1_rx_dma            41
#define H7DMAMUX_CxCR_DMAREQ_ID_usart1_tx_dma            42
#define H7DMAMUX_CxCR_DMAREQ_ID_usart2_rx_dma            43
#define H7DMAMUX_CxCR_DMAREQ_ID_usart2_tx_dma            44
#define H7DMAMUX_CxCR_DMAREQ_ID_usart3_rx_dma            45
#define H7DMAMUX_CxCR_DMAREQ_ID_usart3_tx_dma            46
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM8_CH1                  47
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM8_CH2                  48
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM8_CH3                  49
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM8_CH4                  50
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM8_UP                    51
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM8_TRIG                 52
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM8_COM                 53
#define H7DMAMUX_CxCR_DMAREQ_ID_Reserved                     54
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM5_CH1                  55
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM5_CH2                  56
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM5_CH3                  57
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM5_CH4                  58
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM5_UP                    59
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM5_TRIG                 60
#define H7DMAMUX_CxCR_DMAREQ_ID_spi3_rx_dma               61
#define H7DMAMUX_CxCR_DMAREQ_ID_spi3_tx_dma                62
#define H7DMAMUX_CxCR_DMAREQ_ID_uart4_rx_dma              63
#define H7DMAMUX_CxCR_DMAREQ_ID_uart4_tx_dma              64
#define H7DMAMUX_CxCR_DMAREQ_ID_uart5_rx_dma              65
#define H7DMAMUX_CxCR_DMAREQ_ID_uart5_tx_dma              66
#define H7DMAMUX_CxCR_DMAREQ_ID_dac_ch1_dma              67
#define H7DMAMUX_CxCR_DMAREQ_ID_dac_ch2_dma              68
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM6_UP                    69
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM7_UP                    70
#define H7DMAMUX_CxCR_DMAREQ_ID_usart6_rx_dma            71
#define H7DMAMUX_CxCR_DMAREQ_ID_usart6_tx_dma            72
#define H7DMAMUX_CxCR_DMAREQ_ID_i2c3_rx_dma               73
#define H7DMAMUX_CxCR_DMAREQ_ID_i2c3_tx_dma                74
#define H7DMAMUX_CxCR_DMAREQ_ID_dcmi_dma                   75
#define H7DMAMUX_CxCR_DMAREQ_ID_cryp_in_dma               76
#define H7DMAMUX_CxCR_DMAREQ_ID_cryp_out_dma             77
#define H7DMAMUX_CxCR_DMAREQ_ID_hash_in_dma               78
#define H7DMAMUX_CxCR_DMAREQ_ID_uart7_rx_dma              79
#define H7DMAMUX_CxCR_DMAREQ_ID_uart7_tx_dma              80
#define H7DMAMUX_CxCR_DMAREQ_ID_uart8_rx_dma              81
#define H7DMAMUX_CxCR_DMAREQ_ID_uart8_tx_dma              82
#define H7DMAMUX_CxCR_DMAREQ_ID_spi4_rx_dma               83
#define H7DMAMUX_CxCR_DMAREQ_ID_spi4_tx_dma                84
#define H7DMAMUX_CxCR_DMAREQ_ID_spi5_rx_dma               85
#define H7DMAMUX_CxCR_DMAREQ_ID_spi5_tx_dma                86
#define H7DMAMUX_CxCR_DMAREQ_ID_sai1a_dma                   87
#define H7DMAMUX_CxCR_DMAREQ_ID_sai1b_dma                   88
#define H7DMAMUX_CxCR_DMAREQ_ID_sai2a_dma                   89
#define H7DMAMUX_CxCR_DMAREQ_ID_sai2b_dma                   90
#define H7DMAMUX_CxCR_DMAREQ_ID_swpmi_rx_dma           91
#define H7DMAMUX_CxCR_DMAREQ_ID_swpmi_tx_dma            92
#define H7DMAMUX_CxCR_DMAREQ_ID_spdifrx_dat_dma         93
#define H7DMAMUX_CxCR_DMAREQ_ID_spdifrx_ctrl_dma         94
#define H7DMAMUX_CxCR_DMAREQ_ID_HR_REQ1                 95
#define H7DMAMUX_CxCR_DMAREQ_ID_HR_REQ2                96
#define H7DMAMUX_CxCR_DMAREQ_ID_HR_REQ3               97
#define H7DMAMUX_CxCR_DMAREQ_ID_HR_REQ4                 98
#define H7DMAMUX_CxCR_DMAREQ_ID_HR_REQ5                99
#define H7DMAMUX_CxCR_DMAREQ_ID_ HR_REQ6				100
#define H7DMAMUX_CxCR_DMAREQ_ID_dfsdm1_dma0              101
#define H7DMAMUX_CxCR_DMAREQ_ID_dfsdm1_dma1              102
#define H7DMAMUX_CxCR_DMAREQ_ID_dfsdm1_dma2              103
#define H7DMAMUX_CxCR_DMAREQ_ID_dfsdm1_dma3              104
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM15_CH1                105
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM15_UP                   106
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM15_TRIG               107
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM15_COM               108
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM16_CH1                109
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM16_UP                   110
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM17_CH1                111
#define H7DMAMUX_CxCR_DMAREQ_ID_TIM17_UP                   112
#define H7DMAMUX_CxCR_DMAREQ_ID_sai3_a_dma                 113
#define H7DMAMUX_CxCR_DMAREQ_ID_sai3_b_dma                 114
#define H7DMAMUX_CxCR_DMAREQ_ID_adc3_dma                    115

/**@}*/

/* --- DMAMUX_RGxCR values ----------------------------------- */

/** @defgroup dmamux_rgxcr_sig_id SIGID DMA request trigger input selected
@{*/
#define H7DMAMUX_CxCR(dmamux_base, channel)	MMIO32((dmamux_base) + 0x000 + \
					       (0x04 * ((channel))))

#define H7DMAMUX1_CxCR(channel)		H7DMAMUX_CxCR(DMAMUX1, channel)
#define H7DMAMUX2_CxCR(channel)		H7DMAMUX_CxCR(DMAMUX2, channel)

#define DMAMUX_CxCR_TRIGGER_ID_dmamux1_evt0       0
#define DMAMUX_CxCR_TRIGGER_ID_dmamux1_evt1       1
#define DMAMUX_CxCR_TRIGGER_ID_dmamux1_evt2      2
#define DMAMUX_CxCR_TRIGGER_ID_lptim1_out      		 3
#define DMAMUX_CxCR_TRIGGER_ID_lptim2_out       		4
#define DMAMUX_CxCR_TRIGGER_ID_lptim3_out       		5
#define DMAMUX_CxCR_TRIGGER_ID_extit0      				 6
#define DMAMUX_CxCR_TRIGGER_ID_TIM12_TRGO       7

//sync id's
#define DMAMUX_CxCR_SYNC_ID_dmamux1_evt0       0
#define DMAMUX_CxCR_SYNC_ID_dmamux1_evt1       1
#define DMAMUX_CxCR_SYNC_ID_dmamux1_evt2      2
#define DMAMUX_CxCR_SYNC_ID_lptim1_out      		 3
#define DMAMUX_CxCR_SYNC_ID_lptim2_out       		4
#define DMAMUX_CxCR_SYNC_ID_lptim3_out       		5
#define DMAMUX_CxCR_SYNC_ID_extit0      				 6
#define DMAMUX_CxCR_SYNC_ID_TIM12_TRGO       7
#define dmamux1_evt0    DMAMUX_CxCR_SYNC_ID_dmamux1_evt0
#define dmamux1_evt1    DMAMUX_CxCR_SYNC_ID_dmamux1_evt1
#define dmamux1_evt2    DMAMUX_CxCR_SYNC_ID_dmamux1_evt2
#define lptim1_out           DMAMUX_CxCR_SYNC_ID_lptim1_out
#define lptim2_out           DMAMUX_CxCR_SYNC_ID_lptim2_out
#define lptim3_out           DMAMUX_CxCR_SYNC_ID_lptim3_out
#define extit0                   DMAMUX_CxCR_SYNC_ID_extit0
#define TIM12_TRGO      DMAMUX_CxCR_SYNC_ID_TIM12_TRGO

/**@}*/

/* --- H7DMAMUX1_CxCR values ----------------------------------------------------- */
/** @defgroup dmamux1_cxcr_values H7DMAMUX1_CxCR values
@{*/

/** SYNC_ID[2:0]: Synchronization identification */
#define H7DMAMUX1_CxCR_SYNC_ID  (0x7 << 24)
#define H7DMAMUX1_CxCR_SYNC_ID_MASK	    0x7
#define H7DMAMUX1_CxCR_SYNC_ID_OFFSET	24U
/** NBREQ[4:0]: Number of DMA requests minus 1 to forward */
#define H7DMAMUX1_CxCR_NBREQ            (0x1F << 19)
#define H7DMAMUX1_CxCR_NBREQ_MASK		0x1F
#define H7DMAMUX1_CxCR_SHIFT            19U
/** SPOL[1:0]: Synchronization polarity */
#define H7DMAMUX1_CxCR_SPOL_MASK		0x3
#define H7DMAMUX1_CxCR_SPOL_SHIFT       17U
#define H7DMAMUX1_CxCR_SPOL             (0x3 << 17)
/** SE: Synchronization enable */
#define H7DMAMUX1_CxCR_SE			(1 << 16)

/** EGE: Event generation enable */
#define H7DMAMUX1_CxCR_EGE		(1 << 9)

/** SOIE: Synchronization overrun interrupt enable */
#define H7DMAMUX1_CxCR_SOIE		(1 << 8)

/** DMAREQ_ID[6:0]: DMA request identification */
#define H7DMAMUX1_CxCR_DMAREQ_ID_MASK	0x7F
#define H7DMAMUX1_CxCR_DMAREQ_ID_OFFSET	0x0
/**@}*/

/* --- H7DMAMUX2_CxCR values ----------------------------------------------------- */
/** @defgroup dmamux2_cxcr_values H7DMAMUX2_CxCR values
@{*/

/** SYNC_ID[4:0]: Synchronization identification */
#define H7DMAMUX2_CxCR_SYNC_ID_MASK	(0x1F << 24)
#define H7DMAMUX2_CxCR_SYNC_ID_OFFSET	24U
/** NBREQ[4:0]: Number of DMA requests minus 1 to forward */
#define H7DMAMUX2_CxCR_NBREQ_MASK		0x1F
#define H7DMAMUX2_CxCR_NBREQ            (0x1F << 19)
#define H7DMAMUX2_CxCR_NBREQ_SHIFT      19U
/** SPOL[1:0]: Synchronization polarity */
#define H7DMAMUX2_CxCR_SPOL_MASK		0x3
#define H7DMAMUX2_CxCR_SPOL_SHIFT       17
#define H7DMAMUX2_CxCR_SPOL             (0x3 << 17)
/** SE: Synchronization enable */
#define H7DMAMUX2_CxCR_SE			(1 << 16)

/** EGE: Event generation enable */
#define H7DMAMUX2_CxCR_EGE		(1 << 9)

/** SOIE: Synchronization overrun interrupt enable */
#define H7DMAMUX2_CxCR_SOIE		(1 << 8)

/** DMAREQ_ID[4:0]: DMA request identification */
#define H7DMAMUX2_CxCR_DMAREQ_ID_MASK	0x1F
#define H7DMAMUX2_CxCR_DMAREQ_ID_OFFSET	0
#define H7DMAMUX2_CxCR_DMAREQ_ID        (0x1F << 0)
/**@}*/

/* --- DMAMUX1_CSR values ----------------------------------------------------- */
/** @defgroup dmamux1_csr_values DMAMUX1_CSR values
@{*/

/** SOF[15:0]: Synchronization overrun event flag */
#define DMAMUX1_CSR_SOF             (0xFFFF << 0)
#define DMAMUX1_CSR_SOF_MASK		0xFFFF
#define DMAMUX1_CSR_SOF_SHIFT       0
/**@}*/

/* --- DMAMUX2_CSR values ----------------------------------------------------- */
/** @defgroup dmamux2_csr_values DMAMUX2_CSR values
@{*/

/** SOF[7:0]: Synchronization overrun event flag */
#define DMAMUX2_CSR_SOF_MASK        0xFF
#define DMAMUX2_CSR_SOF             (0xFF << 0)
#define DMAMUX2_CSR_SOF_SHIFT       0
/**@}*/

/* --- DMAMUX1_CFR values ----------------------------------------------------- */
/** @defgroup dmamux1_cfr_values DMAMUX1_CFR values
@{*/
#define H7DMAMUX_CFR(dmamux_base)	MMIO32((dmamux_base) + 0x084)

#define H7DMAMUX1_CFR		H7DMAMUX_CFR(DMAMUX1)
#define H7DMAMUX2_CFR		H7DMAMUX_CFR(DMAMUX2)
/** CSOF[15:0]: Clear synchronization overrun event flag */
#define DMAMUX1_CFR_CSOF_MASK	0xFFFF
#define DMAMUX1_CFR_CSOF_SHIFT  0
#define DMAMUX1_CFR_CSOF        (0xFFFF << 0)
/**@}*/

/* --- DMAMUX2_CFR values ----------------------------------------------------- */
/** @defgroup dmamux2_cfr_values DMAMUX2_CFR values
@{*/

/** CSOF[7:0]: Clear synchronization overrun event flag */
#define H7DMAMUX_CFR_CSOF_MASK		(0xFF << 0)
#define H7DMAMUX_CFR_CSOF(dma_channel)	(1 << ((dma_channel)))
/**@}*/

/* --- DMAMUX1_RGxCR values ----------------------------------------------------- */
/** @defgroup dmamux1_rgxcr_values DMAMUX1_RGxCR values
@{*/
#define DMAMUX1_RGxCR(channel)	MMIO32((DMAMUX1_BASE) + 0x100 + \
					       (0x04 * ((channel))))

#define DMAMUX1_RGxCR_OFFSET         (DMAMUX1_RGxCR);

/** GNBREQ[4:0]: Number of DMA requests to be generated (minus 1) */
#define DMAMUX1_RGxCR_GNBREQ_MASK	(0x1F << 19)
#define DMAMUX1_RGxCR_GNBREQ_OFFSET 19U
/** GPOL[1:0]: DMA request generator trigger polarity */
#define DMAMUX1_RGxCR_GPOL_MASK		(0x3 << 17)

/** GE: DMA request generator channel x enable */
#define DMAMUX1_RGxCR_GE		(1 << 16)

/** OIE: Trigger overrun interrupt enable */
#define DMAMUX1_RGxCR_OIE		(1 << 8)

/** SIG_ID[2:0]: Signal identification */
#define DMAMUX1_RGxCR_SIG_ID_MASK	(0x7 << 0)

/**@}*/

/* --- DMAMUX2_RGxCR values ----------------------------------------------------- */
/** @defgroup dmamux2_rgxcr_values DMAMUX2_RGxCR values
@{*/
#define DMAMUX2_RGxCR(channel)	MMIO32((DMAMUX2_BASE) + 0x100 + \
					       (0x04 * ((channel))))
/** GNBREQ[4:0]: Number of DMA requests to be generated (minus 1) */
#define DMAMUX2_RGxCR_GNBREQ_MASK	(0x1F << 19)

/** GPOL[1:0]: DMA request generator trigger polarity */
#define DMAMUX2_RGxCR_GPOL_MASK		(0x3 << 17)

/** GE: DMA request generator channel x enable */
#define DMAMUX2_RGxCR_GE		(1 << 16)

/** OIE: Trigger overrun interrupt enable */
#define DMAMUX2_RGxCR_OIE		(1 << 8)

/** SIG_ID[4:0]: Signal identification */
#define DMAMUX2_RGxCR_SIG_ID_MASK	0x1F
#define DMAMUX2_RGxCR_SIG_ID_SHIFT  0
/**@}*/

/* --- DMAMUX1_RGSR values ----------------------------------------------------- */
/** @defgroup dmamux1_rgsr_values DMAMUX1_RGSR values
@{*/

/** OF[7:0]: Trigger overrun event flag */
#define DMAMUX1_RGSR_OF_MASK		0xFF
#define DMAMUX1_RGSR_OFSHIFT        0
#define DMAMUX1_RGSR_OF(rg_channel)	(1 << ((rg_channel)))
/**@}*/

/* --- DMAMUX2_RGSR values ----------------------------------------------------- */
/** @defgroup dmamux2_rgsr_values DMAMUX2_RGSR values
@{*/

/** OF[7:0]: Trigger overrun event flag */
#define DMAMUX2_RGSR_OF_MASK		0xFF
#define DMAMUX2_RGSR_OF_SHIFT       0
#define DMAMUX2_RGSR_OF(rg_channel)	(1 << ((rg_channel)))
/**@}*/

/* --- DMAMUX1_RGCFR values ----------------------------------------------------- */
/** @defgroup dmamux1_rgcfr_values DMAMUX1_RGCFR values
@{*/

#define H7DMAMUX_RGCFR(dmamux_base)	MMIO32((dmamux_base) + 0x144

#define DMAMUX1_RGCFR		H7DMAMUX_RGCFR(DMAMUX1)
#define DMAMUX2_RGCFR		H7DMAMUX_RGCFR(DMAMUX2)

//#define DMAMUX1_RGCFR MMIO32((DMAMUX1_BASE) + 0x144
/** COF[7:0]: Clear trigger overrun event flag */
#define DMAMUX1_RGCFR_COF_MASK		0xFF
#define DMAMUX1_RGCFR_COF_SHIFT     0
#define DMAMUX1_RGCFR_COF_OFFSET    (DMAMUX1_RGCFR + 0x0);
#define DMAMUX1_RGCFR_COF(rg_channel)	(1 << ((rg_channel)))

/**@}*/

/* --- DMAMUX2_RGCFR values ----------------------------------------------------- */
/** @defgroup dmamux2_rgcfr_values DMAMUX2_RGCFR values
@{*/
//#define DMAMUX2_RGCFR_COF MMIO32((DMAMUX2_BASE) + 0x144
/** COF[7:0]: Clear trigger overrun event flag */
#define DMAMUX2_RGCFR_COF_MASK		0xFF
#define DMAMUX2_RGCFR_COF(rg_channel)	(1 << ((rg_channel)))
#define DMAMUX2_RGCFR_COF_OFFSET    (DMAMUX2_RGCFR)
#define DMAMUX2_RGCFR_COF_SHIFT 0
/**@}*/
uint8_t dmamux_get_dma_channel_request(uint32_t dmamux, uint8_t channel);
void dmamux_set_dma_channel_request(uint32_t dmamux, uint8_t channel, uint8_t request_id);
void dmamux_reset_dma_channel(uint32_t dmamux, uint8_t channel);
void dmamux_clear_dma_request_sync_overrun(uint32_t dmamux, uint8_t channel);


/**@}*/
