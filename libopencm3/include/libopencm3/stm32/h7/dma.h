/** @defgroup dma_defines DMA Defines

@ingroup STM32H7xx_defines

@brief Defined Constants and Types for the STM32F4xx DMA Controller

@version 1.0.0

@date 30 November 2012

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



/* --- DMA_SxCR values ----------------------------------------------------- */
/** @defgroup dma_sxcr_values DMA_SxCR values
@{*/
#ifndef LIBOPENCM3_DMA_H
#define LIBOPENCM3_DMA_H

#define DMA1    DMA1_BASE
#define DMA2    DMA2_BASE
#define DMA_CHANNEL1			1
#define DMA_CHANNEL2			2
#define DMA_CHANNEL3			3
#define DMA_CHANNEL4			4
#define DMA_CHANNEL5			5
#define DMA_CHANNEL6			6
#define DMA_CHANNEL7			7
/* DMA Stream x peripheral address register (DMA_SxPAR) */

#define DMA_SPAR(dma_base, n)		MMIO32((dma_base) + 0x18 + 0x18 * ((n)))

#define DMA1_SPAR(n)			DMA1_SPAR(DMA1, (n))
#define DMA2_SPAR(n)			DMA2_SPAR(DMA2, (n))
#define DMA1_S0PAR			DMA1_SPAR(0)
#define DMA1_S1PAR			DMA1_SPAR(1)
#define DMA1_S2PAR			DMA1_SPAR(2)
#define DMA1_S3PAR			DMA1_SPAR(3)
#define DMA1_S4PAR			DMA1_SPAR(4)
#define DMA1_S5PAR			DMA1_SPAR(5)
#define DMA1_S6PAR			DMA1_SPAR(6)
#define DMA1_S7PAR			DMA1_SPAR(7)

#define DMA2_S0PAR			DMA2_SPAR(0)
#define DMA2_S1PAR			DMA2_SPAR(1)
#define DMA2_S2PAR			DMA2_SPAR(2)
#define DMA2_S3PAR			DMA2_SPAR(3)
#define DMA2_S4PAR			DMA2_SPAR(4)
#define DMA2_S5PAR			DMA2_SPAR(5)
#define DMA2_S6PAR			DMA2_SPAR(6)
#define DMA2_S7PAR			DMA2_SPAR(7)

/* --- DMA stream registers ------------------------------------------------ */

/* DMA Stream x configuration register (DMA_SxCR) */
#define DMA_SxCR(dma_base, n)		MMIO32((dma_base) + 0x10 + (0x18 * ((n))))

#define DMA1_SxCR(n)			DMA_SxCR(DMA1, (n))
#define DMA2_SxCR(n)			DMA_SxCR(DMA2, (n))

#define DMA1_S0CR			DMA1_SxCR(0)
#define DMA1_S1CR			DMA1_SxCR(1)
#define DMA1_S2CR			DMA1_SxCR(2)
#define DMA1_S3CR			DMA1_SxCR(3)
#define DMA1_S4CR			DMA1_SxCR(4)
#define DMA1_S5CR			DMA1_SxCR(5)
#define DMA1_S6CR			DMA1_SxCR(6)
#define DMA1_S7CR			DMA1_SxCR(7)

#define DMA2_S0CR			DMA2_SxCR(0)
#define DMA2_S1CR			DMA2_SxCR(1)
#define DMA2_S2CR			DMA2_SxCR(2)
#define DMA2_S3CR			DMA2_SxCR(3)
#define DMA2_S4CR			DMA2_SxCR(4)
#define DMA2_S5CR			DMA2_SxCR(5)
#define DMA2_S6CR			DMA2_SxCR(6)
#define DMA2_S7CR			DMA2_SxCR(7)

/* DMA Stream x memory address 0 register (DMA_SxM0AR) */


#define DMA_SM0AR(dma_base, n)		MMIO32((dma_base) + 0x1C +  0x18 * ((n)))
#define DMA1_SM0AR(n)			DMA_SM0AR(DMA1, (n))
#define DMA2_SM0AR(n)			DMA_SM0AR(DMA2, (n))

#define DMA1_S0M0AR			DMA1_SM0AR(0)
#define DMA1_S1M0AR			DMA1_SM0AR(1)
#define DMA1_S2M0AR			DMA1_SM0AR(2)
#define DMA1_S3M0AR			DMA1_SM0AR(3)
#define DMA1_S4M0AR			DMA1_SM0AR(4)
#define DMA1_S5M0AR			DMA1_SM0AR(5)
#define DMA1_S6M0AR			DMA1_SM0AR(6)
#define DMA1_S7M0AR			DMA1_SM0AR(7)

#define DMA2_S0M0AR			DMA2_SM0AR(0)
#define DMA2_S1M0AR			DMA2_SM0AR(1)
#define DMA2_S2M0AR			DMA2_SM0AR(2)
#define DMA2_S3M0AR			DMA2_SM0AR(3)
#define DMA2_S4M0AR			DMA2_SM0AR(4)
#define DMA2_S5M0AR			DMA2_SM0AR(5)
#define DMA2_S6M0AR			DMA2_SM0AR(6)
#define DMA2_S7M0AR			DMA2_SM0AR(7)

#define DMA_SM1AR(dma_base, n)		MMIO32((dma_base) + 0x20 +  0x18 * ((n)))
#define DMA1_SM1AR(n)			DMA_SM1AR(DMA1, (n))
#define DMA2_SM1AR(n)			DMA_SM1AR(DMA2, (n))

#define DMA1_S0M1AR			DMA1_SM1AR(0)
#define DMA1_S1M1AR			DMA1_SM1AR(1)
#define DMA1_S2M1AR			DMA1_SM1AR(2)
#define DMA1_S3M1AR			DMA1_SM1AR(3)
#define DMA1_S4M1AR			DMA1_SM1AR(4)
#define DMA1_S5M1AR			DMA1_SM1AR(5)
#define DMA1_S6M1AR			DMA1_SM1AR(6)
#define DMA1_S7M1AR			DMA1_SM1AR(7)

#define DMA2_S0M1AR			DMA2_SM1AR(0)
#define DMA2_S1M1AR			DMA2_SM1AR(1)
#define DMA2_S2M1AR			DMA2_SM1AR(2)
#define DMA2_S3M1AR			DMA2_SM1AR(3)
#define DMA2_S4M1AR			DMA2_SM1AR(4)
#define DMA2_S5M1AR			DMA2_SM1AR(5)
#define DMA2_S6M1AR			DMA2_SM1AR(6)
#define DMA2_S7M1AR			DMA2_SM1AR(7)

#define DMA_STREAM0			0
#define DMA_STREAM1			1
#define DMA_STREAM2			2
#define DMA_STREAM3			3
#define DMA_STREAM4			4
#define DMA_STREAM5			5
#define DMA_STREAM6			6
#define DMA_STREAM7			7
/**@}*/


#define DMA_STREAM(dma_base, n)		MMIO32((dma_base) + 0x10 + (24 * ((n)))
#define DMA1_STREAM(n)			DMA_STREAM(DMA1, (n))
#define DMA2_STREAM(n)			DMA_STREAM(DMA2, (n))

#define DMA1_STREAM0			DMA1_STREAM(0)
#define DMA1_STREAM1			DMA1_STREAM(1)
#define DMA1_STREAM2			DMA1_STREAM(2)
#define DMA1_STREAM3			DMA1_STREAM(3)
#define DMA1_STREAM4			DMA1_STREAM(4)
#define DMA1_STREAM5			DMA1_STREAM(5)
#define DMA1_STREAM6			DMA1_STREAM(6)
#define DMA1_STREAM7			DMA1_STREAM(7)

#define DMA2_STREAM0			DMA2_STREAM(0)
#define DMA2_STREAM1			DMA2_STREAM(1)
#define DMA2_STREAM2			DMA2_STREAM(2)
#define DMA2_STREAM3			DMA2_STREAM(3)
#define DMA2_STREAM4			DMA2_STREAM(4)
#define DMA2_STREAM5			DMA2_STREAM(5)
#define DMA2_STREAM6			DMA2_STREAM(6)
#define DMA2_STREAM7			DMA2_STREAM(7)

#define DMA_CHANNEL8			8
#define DMA_CHANNEL9			9
#define DMA_CHANNEL10			10
#define DMA_CHANNEL11			11
#define DMA_CHANNEL12			12
#define DMA_CHANNEL13			13
#define DMA_CHANNEL14			14
#define DMA_CHANNEL15			15

#define DMA1_CCR8			DMA1_CCR(DMA_CHANNEL8)
#define DMA1_CCR9		    DMA1_CCR(DMA_CHANNEL9)
#define DMA1_CCR10			DMA1_CCR(DMA_CHANNEL10)
#define DMA1_CCR11			DMA1_CCR(DMA_CHANNEL11)
#define DMA1_CCR12			DMA1_CCR(DMA_CHANNEL12)
#define DMA1_CCR13			DMA1_CCR(DMA_CHANNEL13)
#define DMA1_CCR14			DMA1_CCR(DMA_CHANNEL14)
#define DMA1_CCR15			DMA1_CCR(DMA_CHANNEL15)

#define DMA_LISR3_TEIF            (1 << 3)
#define DMA_LISR9_TEIF            (1 << 9)
#define DMA_LISR19_TEIF           (1 << 19)
#define DMA_LISR25_TEIF           (1 << 25)

#define DMA_LISR0_CFEIF           (1 << 0)
#define DMA_LISR6_CFEIF          (1 << 6)
#define DMA_LISR16_CFEIF         (1 << 16)
#define DMA_LISR22_CFEIF         (1 << 22)
#define DMA_LISR(dma_base)     MMIO32((dma_base) + 0x0)
#define DMA_LISR5_TCIE			(1 << 5)
#define DMA_LISR5_TCIE_OFFSET        6U

#define DMA_LISR11_TCIE			(1 << 11)
#define DMA_LIFC511_TCIE_OFFSET        11U

#define DMA_LISR21_TCIE			(1 << 21)
#define DMA_LIFC521_TCIE_OFFSET        21U

#define DMA_LISR27_TCIE			(1 << 27)
#define DMA_LIFC527_TCIE_OFFSET        27U

#define DMA_LISRC3_TEIF            (1 << 3)
#define DMA_LISRC9_TEIF            (1 << 9)
#define DMA_LISRC19_TEIF           (1 << 19)
#define DMA_LISRC25_TEIF           (1 << 25)

#define DMA_LISRC0_CFEIF           (1 << 0)
#define DMA_LISRC6_CFEIF          (1 << 6)
#define DMA_LISRC16_CFEIF         (1 << 16)
#define DMA_LISRC22_CFEIF         (1 << 22)

#define DMA_LIFCR(dma_base)     MMIO32((dma_base) + 0x08)
#define DMA_LIFCR5_TCIE			(1 << 5)
#define DMA_LIFC5R_TCIE_OFFSET        6U

#define DMA_LIFCR11_TCIE			(1 << 11)
#define DMA_LIFC511_TCIE_OFFSET        11U

#define DMA_LIFCR21_TCIE			(1 << 21)
#define DMA_LIFC521_TCIE_OFFSET        21U

#define DMA_LIFCR27_TCIE			(1 << 27)
#define DMA_LIFC527_TCIE_OFFSET        27U

#define DMA_LIFCRC3_TEIF            (1 << 3)
#define DMA_LIFCRC9_TEIF            (1 << 9)
#define DMA_LIFCRC19_TEIF           (1 << 19)
#define DMA_LIFCRC25_TEIF           (1 << 25)

#define DMA_LIFCRC0_CFEIF           (1 << 0)
#define DMA_LIFCRC6_CFEIF          (1 << 6)
#define DMA_LIFCRC16_CFEIF         (1 << 16)
#define DMA_LIFCRC22_CFEIF         (1 << 22)

#define DMA_HISR(dma_base)     MMIO32((dma_base) + 0x04)
#define DMA_HISR5_TCIE			(1 << 5)
#define DMA_HISR5_TCIE_OFFSET        6U

#define DMA_HISR11_TCIE			(1 << 11)
#define DMA_HISR11_TCIE_OFFSET        11U

#define DMA_HISR21_TCIE			(1 << 21)
#define DMA_HISR21_TCIE_OFFSET        21U

#define DMA_HISR27_TCIE			(1 << 27)
#define DMA_HISR27_TCIE_OFFSET        27U

#define DMA_HISR3_TEIF            (1 << 3)
#define DMA_HISR9_TEIF            (1 << 9)
#define DMA_HISR19_TEIF           (1 << 19)
#define DMA_HISR25_TEIF           (1 << 25)

#define DMA_HISR0_CFEIF           (1 << 0)
#define DMA_HISR6_CFEIF          (1 << 6)
#define DMA_HISR16_CFEIF         (1 << 16)
#define DMA_HISR22_CFEIF         (1 << 22)

//#define DMA_HIFCR(dma_base)     MMIO32((dma_base) + 0x0c)
#define DMA_HIFCR5_TCIE			(1 << 5)
#define DMA_HIFCR5_TCIE_OFFSET        6U

#define DMA_HIFCR11_TCIE			(1 << 11)
#define DMA_HIFC511_TCIE_OFFSET        11U

#define DMA_HIFCR21_TCIE			(1 << 21)
#define DMA_HIFC521_TCIE_OFFSET        21U

#define DMA_HIFCR27_TCIE			(1 << 27)
#define DMA_HIFC527_TCIE_OFFSET        27U

#define DMA_HIFCRC3_TEIF            (1 << 3)
#define DMA_HIFCRC9_TEIF            (1 << 9)
#define DMA_HIFCRC19_TEIF           (1 << 19)
#define DMA_HIFCRC25_TEIF           (1 << 25)

#define DMA_HIFCRC0_CFEIF           (1 << 0)
#define DMA_HIFCRC6_CFEIF          (1 << 6)
#define DMA_HIFCRC16_CFEIF         (1 << 16)
#define DMA_HIFCRC22_CFEIF         (1 << 22)

#define DMA_SxNDTR(dma_base, channel)	MMIO32((dma_base) + 0x14 + \
					       (0x18 * ((channel))))

#define DMA1_SxNDTR(channel)		DMA_SxNDTR(DMA1, channel)
#define DMA1_SxNDTR1			DMA1_SxNDTR(DMA_CHANNEL1)
#define DMA1_SxNDTR2			DMA1_SxNDTR(DMA_CHANNEL2)
#define DMA1_SxNDTR3			DMA1_SxNDTR(DMA_CHANNEL3)
#define DMA1_SxNDTR4			DMA1_SxNDTR(DMA_CHANNEL4)
#define DMA1_SxNDTR5			DMA1_SxNDTR(DMA_CHANNEL5)
#define DMA1_SxNDTR6			DMA1_SxNDTR(DMA_CHANNEL6)
#define DMA1_SxNDTR7			DMA1_SxNDTR(DMA_CHANNEL7)
#define DMA1_SxNDTR8			    DMA1_SxNDTR(DMA_CHANNEL8)
#define DMA1_SxNDTR9			    DMA1_SxNDTR(DMA_CHANNEL9)
#define DMA1_SxNDTR10			DMA1_SxNDTR(DMA_CHANNEL10)
#define DMA1_SxNDTR11			DMA1_SxNDTR(DMA_CHANNEL11)
#define DMA1_SxNDTR12			DMA1_SxNDTR(DMA_CHANNEL12)
#define DMA1_SxNDTR13			DMA1_SxNDTR(DMA_CHANNEL13)
#define DMA1_SxNDTR14			DMA1_SxNDTR(DMA_CHANNEL14)
#define DMA1_SxNDTR15			DMA1_SxNDTR(DMA_CHANNEL15)

#define DMA2_SxNDTR8			    DMA2_SxNDTR(DMA_CHANNEL8)
#define DMA2_SxNDTR9			    DMA2_SxNDTR(DMA_CHANNEL9)
#define DMA2_SxNDTR10			DMA2_SxNDTR(DMA_CHANNEL10)
#define DMA2_SxNDTR11			DMA2_SxNDTR(DMA_CHANNEL11)
#define DMA2_SxNDTR12			DMA2_SxNDTR(DMA_CHANNEL12)
#define DMA2_SxNDTR13			DMA2_SxNDTR(DMA_CHANNEL13)
#define DMA2_SxNDTR14			DMA2_SxNDTR(DMA_CHANNEL14)
#define DMA2_SxNDTR15			DMA2_SxNDTR(DMA_CHANNEL15)

#define DMA_CPAR(dma_base, channel)	MMIO32((dma_base) + 0x18 + \
					       ( 0x18 * ((channel))))
/*
#define DMA1_CPAR(channel)		DMA_CPAR(DMA1, channel)
#define DMA1_CPAR1			DMA1_CPAR(DMA_CHANNEL1)
#define DMA1_CPAR2			DMA1_CPAR(DMA_CHANNEL2)
#define DMA1_CPAR3			DMA1_CPAR(DMA_CHANNEL3)
#define DMA1_CPAR4			DMA1_CPAR(DMA_CHANNEL4)
#define DMA1_CPAR5			DMA1_CPAR(DMA_CHANNEL5)
#define DMA1_CPAR6			DMA1_CPAR(DMA_CHANNEL6)
#define DMA1_CPAR7			DMA1_CPAR(DMA_CHANNEL7)
#define DMA1_CPAR8			    DMA1_CPAR(DMA_CHANNEL8)
#define DMA1_CPAR9			    DMA1_CPAR(DMA_CHANNEL9)
#define DMA1_CPAR10			    DMA1_CPAR(DMA_CHANNEL10)
#define DMA1_CPAR11			    DMA1_CPAR(DMA_CHANNEL11)
#define DMA1_CPAR12			    DMA1_CPAR(DMA_CHANNEL12)
#define DMA1_CPAR13			    DMA1_CPAR(DMA_CHANNEL13)
#define DMA1_CPAR14			    DMA1_CPAR(DMA_CHANNEL14)
#define DMA1_CPAR15			    DMA1_CPAR(DMA_CHANNEL15)


#define DMA1_CMAR(channel)		DMA_CMAR(DMA1, channel)
#define DMA1_CMAR1			    DMA1_CMAR(DMA_CHANNEL1)
#define DMA1_CMAR2			    DMA1_CMAR(DMA_CHANNEL2)
#define DMA1_CMAR3			    DMA1_CMAR(DMA_CHANNEL3)
#define DMA1_CMAR4			    DMA1_CMAR(DMA_CHANNEL4)
#define DMA1_CMAR5			    DMA1_CMAR(DMA_CHANNEL5)
#define DMA1_CMAR6			    DMA1_CMAR(DMA_CHANNEL6)
#define DMA1_CMAR7			    DMA1_CMAR(DMA_CHANNEL7)
#define DMA1_CMAR8			    DMA1_CMAR(DMA_CHANNEL8)
#define DMA1_CMAR9			    DMA1_CMAR(DMA_CHANNEL9)
#define DMA1_CMAR10			    DMA1_CMAR(DMA_CHANNEL10)
#define DMA1_CMAR11			    DMA1_CMAR(DMA_CHANNEL11)
#define DMA1_CMAR12			    DMA1_CMAR(DMA_CHANNEL12)
#define DMA1_CMAR13			    DMA1_CMAR(DMA_CHANNEL13)
#define DMA1_CMAR14			    DMA1_CMAR(DMA_CHANNEL14)
#define DMA1_CMAR15			    DMA1_CMAR(DMA_CHANNEL15)

#define DMA2_CPAR(channel)		DMA_CPAR(DMA2, channel)
#define DMA2_CPAR1			    DMA2_CPAR(DMA_CHANNEL1)
#define DMA2_CPAR2			    DMA2_CPAR(DMA_CHANNEL2)
#define DMA2_CPAR3			    DMA2_CPAR(DMA_CHANNEL3)
#define DMA2_CPAR4			    DMA2_CPAR(DMA_CHANNEL4)
#define DMA2_CPAR5			    DMA2_CPAR(DMA_CHANNEL5)
#define DMA2_CPAR6			    DMA2_CPAR(DMA_CHANNEL6)
#define DMA2_CPAR7			    DMA1_CPAR(DMA_CHANNEL7)
#define DMA2_CPAR8			    DMA2_CPAR(DMA_CHANNEL8)
#define DMA2_CPAR9			    DMA2_CPAR(DMA_CHANNEL9)
#define DMA2_CPAR10			    DMA2_CPAR(DMA_CHANNEL10)
#define DMA2_CPAR11			    DMA2_CPAR(DMA_CHANNEL11)
#define DMA2_CPAR12			    DMA2_CPAR(DMA_CHANNEL12)
#define DMA2_CPAR13			    DMA2_CPAR(DMA_CHANNEL13)
#define DMA2_CPAR14			    DMA2_CPAR(DMA_CHANNEL14)
#define DMA2_CPAR15			    DMA2_CPAR(DMA_CHANNEL15)

#define DMA2_CMAR(channel)		DMA_CMAR(DMA2, channel)
#define DMA2_CMAR1			    DMA2_CMAR(DMA_CHANNEL1)
#define DMA2_CMAR2			    DMA2_CMAR(DMA_CHANNEL2)
#define DMA2_CMAR3			    DMA2_CMAR(DMA_CHANNEL3)
#define DMA2_CMAR4			    DMA2_CMAR(DMA_CHANNEL4)
#define DMA2_CMAR5			    DMA2_CMAR(DMA_CHANNEL5)
#define DMA2_CMAR6			    DMA2_CMAR(DMA_CHANNEL6)
#define DMA2_CMAR7			    DMA2_CMAR(DMA_CHANNEL7)
#define DMA2_CMAR8			    DMA2_CMAR(DMA_CHANNEL8)
#define DMA2_CMAR9			    DMA2_CMAR(DMA_CHANNEL9)
#define DMA2_CMAR10			    DMA2_CMAR(DMA_CHANNEL10)
#define DMA2_CMAR11			    DMA2_CMAR(DMA_CHANNEL11)
#define DMA2_CMAR12			    DMA2_CMAR(DMA_CHANNEL12)
#define DMA2_CMAR13			    DMA2_CMAR(DMA_CHANNEL13)
#define DMA2_CMAR14			    DMA2_CMAR(DMA_CHANNEL14)
#define DMA2_CMAR15			    DMA2_CMAR(DMA_CHANNEL15)
*/
#define DMA_ISR_TEIF8			    DMA_ISR_TEIF(DMA_CHANNEL8)
#define DMA_ISR_TEIF9			    DMA_ISR_TEIF(DMA_CHANNEL9)
#define DMA_ISR_TEIF10			    DMA_ISR_TEIF(DMA_CHANNEL10)
#define DMA_ISR_TEIF11			    DMA_ISR_TEIF(DMA_CHANNEL11)
#define DMA_ISR_TEIF12			    DMA_ISR_TEIF(DMA_CHANNEL12)
#define DMA_ISR_TEIF13			    DMA_ISR_TEIF(DMA_CHANNEL13)
#define DMA_ISR_TEIF14			    DMA_ISR_TEIF(DMA_CHANNEL14)
#define DMA_ISR_TEIF15			    DMA_ISR_TEIF(DMA_CHANNEL15)

#define DMA_ISR_HTIF8			    DMA_ISR_HTIF(DMA_CHANNEL8)
#define DMA_ISR_HTIF9			    DMA_ISR_HTIF(DMA_CHANNEL9)
#define DMA_ISR_HTIF10			    DMA_ISR_HTIF(DMA_CHANNEL10)
#define DMA_ISR_HTIF11			    DMA_ISR_HTIF(DMA_CHANNEL11)
#define DMA_ISR_HTIF12			    DMA_ISR_HTIF(DMA_CHANNEL12)
#define DMA_ISR_HTIF13			    DMA_ISR_HTIF(DMA_CHANNEL13)
#define DMA_ISR_HTIF14			    DMA_ISR_HTIF(DMA_CHANNEL14)
#define DMA_ISR_HTIF15			    DMA_ISR_HTIF(DMA_CHANNEL15)

#define DMA_ISR_TCIF8			    DMA_ISR_TCIF(DMA_CHANNEL8)
#define DMA_ISR_TCIF9			    DMA_ISR_TCIF(DMA_CHANNEL9)
#define DMA_ISR_TCIF10			    DMA_ISR_TCIF(DMA_CHANNEL10)
#define DMA_ISR_TCIF11			    DMA_ISR_TCIF(DMA_CHANNEL11)
#define DMA_ISR_TCIF12			    DMA_ISR_TCIF(DMA_CHANNEL12)
#define DMA_ISR_TCIF13			    DMA_ISR_TCIF(DMA_CHANNEL13)
#define DMA_ISR_TCIF14			    DMA_ISR_TCIF(DMA_CHANNEL14)
#define DMA_ISR_TCIF15			    DMA_ISR_TCIF(DMA_CHANNEL15)

#define DMA_ISR_GIF8			    DMA_ISR_GIF(DMA_CHANNEL8)
#define DMA_ISR_GIF9			    DMA_ISR_GIF(DMA_CHANNEL9)
#define DMA_ISR_GIF10			    DMA_ISR_GIF(DMA_CHANNEL10)
#define DMA_ISR_GIF11			    DMA_ISR_GIF(DMA_CHANNEL11)
#define DMA_ISR_GIF12			    DMA_ISR_GIF(DMA_CHANNEL12)
#define DMA_ISR_GIF13			    DMA_ISR_GIF(DMA_CHANNEL13)
#define DMA_ISR_GIF14			    DMA_ISR_GIF(DMA_CHANNEL14)
#define DMA_ISR_GIF15			    DMA_ISR_GIF(DMA_CHANNEL15)

//#define DMA_FLAG_OFFSET(channel)	(4*((channel) - 1))             //where did these numbers come from? not in g4 reference man
//#define DMA_FLAGS			(DMA_TEIF | DMA_TCIF | DMA_HTIF |  DMA_GIF)
//#define DMA_ISR_MASK(channel)		(DMA_FLAGS << DMA_FLAG_OFFSET(channel))
/* CTEIF: Transfer error clear */
#define DMA_IFCR_CTEIF_BIT		DMA_TEIF
#define DMA_IFCR_CTEIF(channel)		(DMA_IFCR_CTEIF_BIT << \
					 (DMA_FLAG_OFFSET(channel)))

#define DMA_IFCR_CTEIF1			DMA_IFCR_CTEIF(DMA_CHANNEL1)
#define DMA_IFCR_CTEIF2			DMA_IFCR_CTEIF(DMA_CHANNEL2)
#define DMA_IFCR_CTEIF3			DMA_IFCR_CTEIF(DMA_CHANNEL3)
#define DMA_IFCR_CTEIF4			DMA_IFCR_CTEIF(DMA_CHANNEL4)
#define DMA_IFCR_CTEIF5			DMA_IFCR_CTEIF(DMA_CHANNEL5)
#define DMA_IFCR_CTEIF6			DMA_IFCR_CTEIF(DMA_CHANNEL6)
#define DMA_IFCR_CTEIF7			DMA_IFCR_CTEIF(DMA_CHANNEL7)
#define DMA_IFCR_CTEIF8			    DMA_IFCR_CTEIF(DMA_CHANNEL8)
#define DMA_IFCR_CTEIF9			    DMA_IFCR_CTEIF(DMA_CHANNEL9)
#define DMA_IFCR_CTEIF10			DMA_IFCR_CTEIF(DMA_CHANNEL10)
#define DMA_IFCR_CTEIF11			DMA_IFCR_CTEIF(DMA_CHANNEL11)
#define DMA_IFCR_CTEIF12			DMA_IFCR_CTEIF(DMA_CHANNEL12)
#define DMA_IFCR_CTEIF13			DMA_IFCR_CTEIF(DMA_CHANNEL13)
#define DMA_IFCR_CTEIF14			DMA_IFCR_CTEIF(DMA_CHANNEL14)
#define DMA_IFCR_CTEIF15			DMA_IFCR_CTEIF(DMA_CHANNEL15)

#define DMA_IFCR_CHTIF8			    DMA_IFCR_CHTIF(DMA_CHANNEL8)
#define DMA_IFCR_CHTIF9			    DMA_IFCR_CHTIF(DMA_CHANNEL9)
#define DMA_IFCR_CHTIF10			DMA_IFCR_CHTIF(DMA_CHANNEL10)
#define DMA_IFCR_CHTIF11			DMA_IFCR_CHTIF(DMA_CHANNEL11)
#define DMA_IFCR_CHTIF12			DMA_IFCR_CHTIF(DMA_CHANNEL12)
#define DMA_IFCR_CHTIF13			DMA_IFCR_CHTIF(DMA_CHANNEL13)
#define DMA_IFCR_CHTIF14			DMA_IFCR_CHTIF(DMA_CHANNEL14)
#define DMA_IFCR_CHTIF15			DMA_IFCR_CHTIF(DMA_CHANNEL15)

#define DMA_IFCR_CTCIF8			    DMA_IFCR_CTCIF(DMA_CHANNEL8)
#define DMA_IFCR_CTCIF9			    DMA_IFCR_CTCIF(DMA_CHANNEL9)
#define DMA_IFCR_CTCIF10			DMA_IFCR_CTCIF(DMA_CHANNEL10)
#define DMA_IFCR_CTCIF11			DMA_IFCR_CTCIF(DMA_CHANNEL11)
#define DMA_IFCR_CTCIF12			DMA_IFCR_CTCIF(DMA_CHANNEL12)
#define DMA_IFCR_CTCIF13			DMA_IFCR_CTCIF(DMA_CHANNEL13)
#define DMA_IFCR_CTCIF14			DMA_IFCR_CTCIF(DMA_CHANNEL14)
#define DMA_IFCR_CTCIF15			DMA_IFCR_CTCIF(DMA_CHANNEL15)

#define DMA_IFCR_CGIF8			DMA_IFCR_CGIF(DMA_CHANNEL8)
#define DMA_IFCR_CGIF9			DMA_IFCR_CGIF(DMA_CHANNEL9)
#define DMA_IFCR_CGIF10			DMA_IFCR_CGIF(DMA_CHANNEL10)
#define DMA_IFCR_CGIF11			DMA_IFCR_CGIF(DMA_CHANNEL11)
#define DMA_IFCR_CGIF12			DMA_IFCR_CGIF(DMA_CHANNEL12)
#define DMA_IFCR_CGIF13			DMA_IFCR_CGIF(DMA_CHANNEL13)
#define DMA_IFCR_CGIF14			DMA_IFCR_CGIF(DMA_CHANNEL14)
#define DMA_IFCR_CGIF15			DMA_IFCR_CGIF(DMA_CHANNEL15)

#define DMA_IFCR_CIF8			DMA_IFCR_CIF(DMA_CHANNEL8)
#define DMA_IFCR_CIF9			DMA_IFCR_CIF(DMA_CHANNEL9)
#define DMA_IFCR_CIF10			DMA_IFCR_CIF(DMA_CHANNEL10)
#define DMA_IFCR_CIF11			DMA_IFCR_CIF(DMA_CHANNEL11)
#define DMA_IFCR_CIF12			DMA_IFCR_CIF(DMA_CHANNEL12)
#define DMA_IFCR_CIF13			DMA_IFCR_CIF(DMA_CHANNEL13)
#define DMA_IFCR_CIF14			DMA_IFCR_CIF(DMA_CHANNEL14)
#define DMA_IFCR_CIF15			DMA_IFCR_CIF(DMA_CHANNEL15)

/**@}*/

/* DMA high interrupt flag clear register (DMAx_HIFCR) */
#define DMA_HIFCR(port)			MMIO32((port) + 0x0C)
#define DMA1_HIFCR			DMA_HIFCR(DMA1)
#define DMA2_HIFCR			DMA_HIFCR(DMA2)
/* --- DMA_HIFCR values ----------------------------------------------------- */

#define DMA_HIFCR_CFEIF4		(1 << 0)
#define DMA_HIFCR_CDMEIF4		(1 << 2)
#define DMA_HIFCR_CTEIF4		(1 << 3)
#define DMA_HIFCR_CHTIF4		(1 << 4)
#define DMA_HIFCR_CTCIF4		(1 << 5)

#define DMA_HIFCR_CFEIF5		(1 << 6)
#define DMA_HIFCR_CDMEIF5		(1 << 8)
#define DMA_HIFCR_CTEIF5		(1 << 9)
#define DMA_HIFCR_CHTIF5		(1 << 10)
#define DMA_HIFCR_CTCIF5		(1 << 11)

#define DMA_HIFCR_CFEIF6		(1 << 16)
#define DMA_HIFCR_CDMEIF6		(1 << 18)
#define DMA_HIFCR_CTEIF6		(1 << 19)
#define DMA_HIFCR_CHTIF6		(1 << 20)
#define DMA_HIFCR_CTCIF6		(1 << 21)

#define DMA_HIFCR_CFEIF7		(1 << 22)
#define DMA_HIFCR_CDMEIF7		(1 << 24)
#define DMA_HIFCR_CTEIF7		(1 << 25)
#define DMA_HIFCR_CHTIF7		(1 << 26)
#define DMA_HIFCR_CTCIF7		(1 << 27)
/**@}*/

#define DMA_SxCR_DIR_OFFSET         6U
#define DMA_SxCR_DIR_PERIPHERAL_TO_MEM	(0 << 6)
#define DMA_SxCR_DIR_MEM_TO_PERIPHERAL	(1 << 6)
#define DMA_SxCR_DIR_MEM_TO_MEM		(2 << 6)
/** MBURST[1:0]: memory burst transfer configuration */
#define DMA_SxCR_MBURST_MASK		(0x3 << 23)
#define DMA_SxCR_MBURST_OFFSET      23U
/** PBURST[1:0]: peripheral burst transfer configuration */
#define DMA_SxCR_PBURST_MASK		(0x3 << 21)
#define DMA_SxCR_PBURST_OFFSET      21U
/** TRBUFF: Enable the DMA to handle bufferable transfers. */
#define DMA_SxCR_TRBUFF			(1 << 20)
#define DMA_SxCR_TRBUFF_OFFSET  20U
/** CT: current target (only in double-buffer mode) */
#define DMA_SxCR_CT			(1 << 19)
#define DMA_SxCR_CT_OFFSET  19U
/** DBM: double-buffer mode */
#define DMA_SxCR_DBM			(1 << 18)
#define DMA_SxCR_DBM_OFFSET     18U
/** PL[1:0]: priority level */
#define DMA_SxCR_PL_LOW			(0x0 << 16)
#define DMA_SxCR_PL_MEDIUM		(0x1 << 16)
#define DMA_SxCR_PL_HIGH			(0x2 << 16)
#define DMA_SxCR_PL_VERY_HIGH		(0x3 << 16)
#define DMA_SxCR_PL_MASK		(0x3 << 16)
#define DMA_SxCR_PL_OFFSET      16U
/** PINCOS: peripheral increment offset size */
#define DMA_SxCR_PINCOS			(1 << 15)
#define DMA_SxCR_PINCOS_OFFSET  15U
/** MSIZE[1:0]: memory data size */
#define DMA_SxCR_MSIZE_MASK		(0x3 << 13)
#define DMA_SxCR_MSIZE_OFFSET   13U
#define DMA_SxCR_MSIZE_8BIT		(0x0 << 13)
#define DMA_SxCR_MSIZE_16BIT		(0x1 << 13)
#define DMA_SxCR_MSIZE_32BIT		(0x2 << 13)
/** PSIZE[1:0]: peripheral data size */
#define DMA_SxCR_PSIZE_MASK		(0x3 << 11)
#define DMA_SxCR_PSIZE_OFFSET 11
/** MINC: memory increment mode */
#define DMA_SxCR_PSIZE_8BIT		(0x0 << 11)
#define DMA_SxCR_PSIZE_16BIT		(0x1 << 11)
#define DMA_SxCR_PSIZE_32BIT		(0x2 << 11)
#define DMA_SxCR_MINC			(1 << 10)
#define DMA_SxCR_MINC_OFFSET     10
/** PINC: peripheral increment mode */
#define DMA_SxCR_PINC			(1 << 9)
#define DMA_SxCR_PINC_OFFSET    9U
/** CIRC: circular mode */
#define DMA_SxCR_CIRC			(1 << 8)
#define DMA_SxCR_CIRC_OFFSET    8U
/** DIR[1:0]: data transfer direction */
#define DMA_SxCR_DIR_MASK		(0x3 << 6)
#define DMA_SxCR_DIR_SHIFT		6
#define DMA_SxCR_DIR            DMA_SxCR_DIR_MASK

/** PFCTRL: peripheral flow controller */
#define DMA_SxCR_PFCTRL			(1 << 5)
#define DMA_SxCR_PFCTRL_OFFSET  5U
/** TCIE: transfer complete interrupt enable */
#define DMA_SxCR_TCIE			(1 << 4)
#define DMA_SxCR_TCIE_OFFSET        4U
/** HTIE: half transfer interrupt enable */
#define DMA_SxCR_HTIE			(1 << 3)
#define DMA_SxCR_HTIE_OFFSET    3U
/** TEIE: transfer error interrupt enable */
#define DMA_SxCR_TEIE			(1 << 2)
#define DMA_SxCR_TEIE_OFFSET    2U
/** DMEIE: direct mode error interrupt enable */
#define DMA_SxCR_DMEIE			(1 << 1)
#define DMA_SxCR_DMEIE_OFFSET   1U
/** EN: stream enable / flag stream ready when read low */
#define DMA_SxCR_EN			(1 << 0)
#define DMA_SxCR_EN_OFFSET  1U
/**@}*/

/* --- DMA_SxNDTR values ----------------------------------------------------- */
/** @defgroup dma_sxndtr_values DMA_SxNDTR values
@{*/

/** NDT[15:0]: number of data items to transfer (0 up to 65535) */
#define DMA_SxNDTR_NDT(dma, channel)		(0xFFFF << 0)
#define DMA_SxNDTR_NDT_OFFSET   0x0

/**@}*/

/* --- DMA_SxPAR values ----------------------------------------------------- */
/** @defgroup dma_sxpar_values DMA_SxPAR values
@{*/

/** PAR[31:0]: peripheral address */
#define DMA_SxPAR_PAR_MASK		(0xFFFFFFFF << 0)

/**@}*/

/* --- DMA_SxM0AR values ----------------------------------------------------- */
/** @defgroup dma_sxm0ar_values DMA_SxM0AR values
@{*/

/** M0A[31:0]: memory 0 address */
#define DMA_SxM0AR_M0A_MASK		(0xFFFFFFFF << 0)

/**@}*/

/* --- DMA_SxM1AR values ----------------------------------------------------- */
/** @defgroup dma_sxm1ar_values DMA_SxM1AR values
@{*/

/** M1A[31:0]: memory 1 address (used in case of double-buffer mode) */
#define DMA_SxM1AR_M1A_MASK		(0xFFFFFFFF << 0)

/**@}*/




#define DMA_SxFCR(dma_base, channel)	MMIO32((dma_base) + 0x24 + \
					       (0x18 * ((channel))))

#define DMA1_SxFCR			DMA_SxFCR(DMA1)
#define DMA2_SxFCR			DMA_SxFCR(DMA2)
/* --- DMA_SxFCR values ----------------------------------------------------- */
/** @defgroup dma_sxfcr_values DMA_SxFCR values
@{*/

/** FEIE: FIFO error interrupt enable */
#define DMA_SxFCR_FEIE			(1 << 7)

/** FS[2:0]: FIFO status */
#define DMA_SxFCR_FS_MASK		(0x7 << 3)

/** DMDIS: direct mode disable */
#define DMA_SxFCR_DMDIS			(1 << 2)

/** FTH[1:0]: FIFO threshold selection */
#define DMA_SxFCR_FTH_MASK		(0x3 << 0)
#define DMA_THRESHOLD_QUARTER        00
#define DMA_THRESHOLD_HALF           01
#define DMA_THRESHOLD_THREEQUARTER   00
#define DMA_THRESHOLD_FULL           11
/**@}*/

/* DMA IDs dmamux1

 DMAMUX_CxCR_DMAREQ_ID_dmamux1_req_gen0	1
 DMAMUX_CxCR_DMAREQ_ID_dmamux1_req_gen1	2
 DMAMUX_CxCR_DMAREQ_ID_dmamux1_req_gen2	3
 DMAMUX_CxCR_DMAREQ_ID_dmamux1_req_gen3	4
 DMAMUX_CxCR_DMAREQ_ID_dmamux1_req_gen4	5
 DMAMUX_CxCR_DMAREQ_ID_dmamux1_req_gen5	6
 DMAMUX_CxCR_DMAREQ_ID_dmamux1_req_gen6	7
 DMAMUX_CxCR_DMAREQ_ID_dmamux1_req_gen7	8
 DMAMUX_CxCR_DMAREQ_ID_adc1_dma                    9
 DMAMUX_CxCR_DMAREQ_ID_adc2_dma					10
 DMAMUX_CxCR_DMAREQ_ID_TIM1_CH1                  11
 DMAMUX_CxCR_DMAREQ_ID_TIM1_CH2                  12
 DMAMUX_CxCR_DMAREQ_ID_TIM1_CH3                  13
 DMAMUX_CxCR_DMAREQ_ID_TIM1_CH4                  14
 DMAMUX_CxCR_DMAREQ_ID_TIM1_UP                    15
 DMAMUX_CxCR_DMAREQ_ID_TIM1_TRIG                 16
 DMAMUX_CxCR_DMAREQ_ID_TIM1_COM                 17
 DMAMUX_CxCR_DMAREQ_ID_TIM2_CH1                  18
 DMAMUX_CxCR_DMAREQ_ID_TIM2_CH2                  19
 DMAMUX_CxCR_DMAREQ_ID_TIM2_CH3                  20
 DMAMUX_CxCR_DMAREQ_ID_TIM2_CH4                  21
 DMAMUX_CxCR_DMAREQ_ID_TIM2_UP                    22
 DMAMUX_CxCR_DMAREQ_ID_TIM3_CH1                  23
 DMAMUX_CxCR_DMAREQ_ID_TIM3_CH2                  24
 DMAMUX_CxCR_DMAREQ_ID_TIM3_CH3                  25
 DMAMUX_CxCR_DMAREQ_ID_TIM3_CH4                  26
 DMAMUX_CxCR_DMAREQ_ID_TIM3_UP                    27
 DMAMUX_CxCR_DMAREQ_ID_TIM3_TRIG                 28
 DMAMUX_CxCR_DMAREQ_ID_TIM4_CH1                  29
 DMAMUX_CxCR_DMAREQ_ID_TIM4_CH2                  30
 DMAMUX_CxCR_DMAREQ_ID_TIM4_CH3                  31
 DMAMUX_CxCR_DMAREQ_ID_TIM4_UP                    32
 DMAMUX_CxCR_DMAREQ_ID_i2c1_rx_dma               33
 DMAMUX_CxCR_DMAREQ_ID_i2c1_tx_dma                34
 DMAMUX_CxCR_DMAREQ_ID_i2c2_rx_dma               35
 DMAMUX_CxCR_DMAREQ_ID_i2c2_tx_dma                36
 DMAMUX_CxCR_DMAREQ_ID_spi1_rx_dma               37
 DMAMUX_CxCR_DMAREQ_ID_spi1_tx_dma                38
 DMAMUX_CxCR_DMAREQ_ID_spi2_rx_dma               39
 DMAMUX_CxCR_DMAREQ_ID_spi2_tx_dma                40
 DMAMUX_CxCR_DMAREQ_ID_usart1_rx_dma            41
 DMAMUX_CxCR_DMAREQ_ID_usart1_tx_dma            42
 DMAMUX_CxCR_DMAREQ_ID_usart2_rx_dma            43
 DMAMUX_CxCR_DMAREQ_ID_usart2_tx_dma            44
 DMAMUX_CxCR_DMAREQ_ID_usart3_rx_dma            45
 DMAMUX_CxCR_DMAREQ_ID_usart3_tx_dma            46
 DMAMUX_CxCR_DMAREQ_ID_TIM8_CH1                  47
 DMAMUX_CxCR_DMAREQ_ID_TIM8_CH2                  48
 DMAMUX_CxCR_DMAREQ_ID_TIM8_CH3                  49
 DMAMUX_CxCR_DMAREQ_ID_TIM8_CH4                  50
 DMAMUX_CxCR_DMAREQ_ID_TIM8_UP                    51
 DMAMUX_CxCR_DMAREQ_ID_TIM8_TRIG                 52
 DMAMUX_CxCR_DMAREQ_ID_TIM8_COM                 53
 DMAMUX_CxCR_DMAREQ_ID_Reserved                     54
 DMAMUX_CxCR_DMAREQ_ID_TIM5_CH1                  55
 DMAMUX_CxCR_DMAREQ_ID_TIM5_CH2                  56
 DMAMUX_CxCR_DMAREQ_ID_TIM5_CH3                  57
 DMAMUX_CxCR_DMAREQ_ID_TIM5_CH4                  58
 DMAMUX_CxCR_DMAREQ_ID_TIM5_UP                    59
 DMAMUX_CxCR_DMAREQ_ID_TIM5_TRIG                 60
 DMAMUX_CxCR_DMAREQ_ID_spi3_rx_dma               61
 DMAMUX_CxCR_DMAREQ_ID_spi3_tx_dma                62
 DMAMUX_CxCR_DMAREQ_ID_uart4_rx_dma              63
 DMAMUX_CxCR_DMAREQ_ID_uart4_tx_dma              64
 DMAMUX_CxCR_DMAREQ_ID_uart5_rx_dma              65
 DMAMUX_CxCR_DMAREQ_ID_uart5_tx_dma              66
 DMAMUX_CxCR_DMAREQ_ID_dac_ch1_dma              67
 DMAMUX_CxCR_DMAREQ_ID_dac_ch2_dma              68
 DMAMUX_CxCR_DMAREQ_ID_TIM6_UP                    69
 DMAMUX_CxCR_DMAREQ_ID_TIM7_UP                    70
 DMAMUX_CxCR_DMAREQ_ID_usart6_rx_dma            71
 DMAMUX_CxCR_DMAREQ_ID_usart6_tx_dma            72
 DMAMUX_CxCR_DMAREQ_ID_i2c3_rx_dma               73
 DMAMUX_CxCR_DMAREQ_ID_i2c3_tx_dma                74
 DMAMUX_CxCR_DMAREQ_ID_dcmi_dma                   75
 DMAMUX_CxCR_DMAREQ_ID_cryp_in_dma               76
 DMAMUX_CxCR_DMAREQ_ID_cryp_out_dma             77
 DMAMUX_CxCR_DMAREQ_ID_hash_in_dma               78
 DMAMUX_CxCR_DMAREQ_ID_uart7_rx_dma              79
 DMAMUX_CxCR_DMAREQ_ID_uart7_tx_dma              80
 DMAMUX_CxCR_DMAREQ_ID_uart8_rx_dma              81
 DMAMUX_CxCR_DMAREQ_ID_uart8_tx_dma              82
 DMAMUX_CxCR_DMAREQ_ID_spi4_rx_dma               83
 DMAMUX_CxCR_DMAREQ_ID_spi4_tx_dma                84
 DMAMUX_CxCR_DMAREQ_ID_spi5_rx_dma               85
 DMAMUX_CxCR_DMAREQ_ID_spi5_tx_dma                86
 DMAMUX_CxCR_DMAREQ_ID_sai1a_dma                   87
 DMAMUX_CxCR_DMAREQ_ID_sai1b_dma                   88
 DMAMUX_CxCR_DMAREQ_ID_sai2a_dma                   89
 DMAMUX_CxCR_DMAREQ_ID_sai2b_dma                   90
 DMAMUX_CxCR_DMAREQ_ID_swpmi_rx_dma           91
 DMAMUX_CxCR_DMAREQ_ID_swpmi_tx_dma            92
 DMAMUX_CxCR_DMAREQ_ID_spdifrx_dat_dma         93
 DMAMUX_CxCR_DMAREQ_ID_spdifrx_ctrl_dma         94
 DMAMUX_CxCR_DMAREQ_ID_HR_REQ(1)                 95
 DMAMUX_CxCR_DMAREQ_ID_HR_REQ(2)                 96
 DMAMUX_CxCR_DMAREQ_ID_HR_REQ(3)                 97
 DMAMUX_CxCR_DMAREQ_ID_HR_REQ(4)                 98
 DMAMUX_CxCR_DMAREQ_ID_HR_REQ(5)                 99
 DMAMUX_CxCR_DMAREQ_ID_ HR_REQ(6)				100
 DMAMUX_CxCR_DMAREQ_ID_dfsdm1_dma0              101
 DMAMUX_CxCR_DMAREQ_ID_dfsdm1_dma1              102
 DMAMUX_CxCR_DMAREQ_ID_dfsdm1_dma2              103
 DMAMUX_CxCR_DMAREQ_ID_dfsdm1_dma3              104
 DMAMUX_CxCR_DMAREQ_ID_TIM15_CH1                105
 DMAMUX_CxCR_DMAREQ_ID_TIM15_UP                   106
 DMAMUX_CxCR_DMAREQ_ID_TIM15_TRIG               107
 DMAMUX_CxCR_DMAREQ_ID_TIM15_COM               108
 DMAMUX_CxCR_DMAREQ_ID_TIM16_CH1                109
 DMAMUX_CxCR_DMAREQ_ID_TIM16_UP                   110
 DMAMUX_CxCR_DMAREQ_ID_TIM17_CH1                111
 DMAMUX_CxCR_DMAREQ_ID_TIM17_UP                   112
 DMAMUX_CxCR_DMAREQ_ID_sai3_a_dma                 113
 DMAMUX_CxCR_DMAREQ_ID_sai3_b_dma                 114
 DMAMUX_CxCR_DMAREQ_ID_adc3_dma                    115
*/

/* dmamux2 id's
 dmamux2_req_gen0	1
 dmamux2_req_gen1    2
 dmamux2_req_gen2    3
 dmamux2_req_gen3    4
 dmamux2_req_gen4    5
 dmamux2_req_gen5    6
 dmamux2_req_gen6    7
 dmamux2_req_gen7    8
 lpuart1_rx_dma           9
 lpuart1_tx_dma		10
 spi6_rx_dma            11
 spi6_tx_dma            12
 i2c4_rx_dma            13
 i2c4_tx_dma            14
 sai4_a_dma              15
 sai4_b_dma             16
 adc3_dma                17
 Reserved                  18
 Reserved                  19
 Reserved                  20
 Reserved                  21
 Reserved                  22
 Reserved                  23
 Reserved                  24
 Reserved                  25
 Reserved                  26
 Reserved                  27
 Reserved                  28
 Reserved                  29
 Reserved                  30
 Reserved                  31
 Reserved                  32
*/

/* --- DMA_SxFCR values ----------------------------------------------------- */
/** @defgroup dma_sxfcr_values DMA_SxFCR values
@{*/

/** FEIE: FIFO error interrupt enable */
#define DMA_SxFCR_FEIE			(1 << 7)

/** FS[2:0]: FIFO status */
#define DMA_SxFCR_FS_MASK		(0x7 << 3)

/** DMDIS: direct mode disable */
#define DMA_SxFCR_DMDIS			(1 << 2)



/**@}*/

void dma_enable(int dma, int channel);
void dma_disable(int dma, int channel);
void dma_dblbuf_enable(int dma, int channel);
void dma_set_dir_memory_to_memory(int dma, int channel);
void dma_clear_dir_memory_to_memory(int dma, int channel);
void dma_set_dir_memory_to_peripheral(int dma, int channel);
void dma_set_dir_peripheral_to_memory(int dma, int channel);
void dma_set_priority_level(int dma, int channel, int priority);
void dma_set_msize(int dma, int channel, int msize);
void dma_set_psize(int dma, int channel, int psize);
void dma_minc_enable(int dma, int channel);
void dma_pinc_enable(int dma, int channel);
void dma_circ_disable(int dma, int stream);
void dma_circ_enable(int dma, int stream, uint32_t direction);
void dma_set_dir_read_from_memory(int dma, int channel);
void dma_set_dir_read_from_peripheral(int dma, int channel);
void dma_tx_error_interrupt_enable(int dma, int channel);
void dma_tx_error_interrupt_disable(int dma, int channel);
void dma_half_transfer_interrupt_enable(int dma, int channel);
void dma_half_transfer_interrupt_disable(int dma, int channel);
void dma_transfer_complete_interrupt_enable(int dma, int channel);
void dma_transfer_complete_interrupt_disable(int dma, int channel);
void dma_set_sndtr(int dma, int channel, uint16_t num);
void dma_set_sparx(int dma, int channel, uint32_t addr);
void dma_set_sm0ar(int dma, int channel, uint32_t addr);
void dma_set_sm1ar(int dma, int channel, uint16_t addr);
void dma_pinc_disable(int dma, int channel);
void dma_set_memory_size(uint32_t dma, uint8_t channel, uint32_t mem_size);
void dma_set_peripheral_size(uint32_t dma, uint8_t channel, uint32_t peripheral_size);
void dma_set_transfer_mode(uint32_t dma, uint8_t stream, uint32_t direction);
void dma_stream_reset(uint32_t dma, uint8_t channel);
void dma_enable_memory_increment_mode(uint32_t dma, uint8_t channel);
void dma_disable_peripheral_increment_mode(uint32_t dma, uint8_t channel);
void dma_enable_peripheral_increment_mode(uint32_t dma, uint8_t channel);
void dma_set_peripheral_address(uint32_t dma, uint8_t channel, uint32_t address);
void dma_set_memory_address(uint32_t dma, uint8_t channel, uint32_t address);
void dma_set_memory_address2(uint32_t dma, uint8_t channel, uint32_t address);
uint16_t dma_get_number_of_data(uint32_t dma, uint8_t channel);
void dma_enable_stream(uint32_t dma, uint8_t channel);
void dma_disable_stream(uint32_t dma, uint8_t channel);
void dma_set_number_of_data(uint32_t dma, uint8_t channel, uint32_t number);
void dma_clear_interrupt_flags(uint32_t dma, uint8_t channel);
void dma_enable_transfer_complete_interrupt(uint32_t dma, uint8_t channel);
void dma_disable_transfer_complete_interrupt(uint32_t dma, uint8_t channel);
void dma_enable_bufferable_transfers(uint32_t dma, uint8_t channel);
void dma_disable_mburst(uint32_t dma, uint8_t channel);
void dma_disable_pburst(uint32_t dma, uint8_t channel);
void dma_enable_pburst(uint32_t dma, uint8_t channel);
void dma_enable_mburst(uint32_t dma, uint8_t channel);
void dma_set_as_flow_controller(uint32_t dma, uint8_t channel);
void dma_peripheral_set_as_flow_controller(uint32_t dma, uint8_t channel);
void dma_enable_direct_mode(uint32_t dma, uint8_t channel);
void dma_disable_direct_mode(uint32_t dma, uint8_t channel);
void dma_set_fifo_threshold(uint32_t dma, uint8_t channel, uint8_t thresh);
void dma_disable_memory_increment_mode(uint32_t dma, uint8_t channel);
uint16_t dma_get_current_target(uint32_t dma, uint8_t channel);
#endif
/**@}*/

