#ifndef LIBOPENCM3_FMPI2C_H
#define LIBOPENCM3_FMPI2C_H

#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/stm32/common/i2c_common_v1.h>
#include <stddef.h>
#include <stdint.h>

#define I2C_FMPI2C		FMPI2C1_BASE
#define I2C_FMPI2C1		I2C_FMPI2C

#define FMPI2C_CR1		FMPI2C1_BASE + 0x00
#define FMPI2C1_CR1			FMPI2C_CR1

#define FMPI2C_CR2		FMPI2C1_BASE + 0x4
#define FMPI2C1_CR2			FMPI2C_CR2

#define FMPI2C_OAR1		FMPI2C1_BASE + 0x8
#define FMPI2C1_OAR1			FMPI2C_OAR1

#define FMPI2C_OAR2		FMPI2C1_BASE + 0xC
#define FMPI2C1_OAR2			FMPI2C_OAR2

#define FMPI2C_TIMINGR		FMPI2C1_BASE + 0x10
#define FMPI2C1_TIMINGR			FMPI2C_TIMINGR

#define FMPI2C_TIMEOUTR		FMPI2C1_BASE + 0x14
#define FMPI2C1_TIMEOUTR			FMPI2C_TIMEOUTR

#define FMPI2C_ISR		FMPI2C_ISR + 0x18
#define FMPI2C1_ISR			FMPI2C_ISR

#define FMPI2C_ICR		FMPI2C_ISR + 0x1C
#define FMPI2C1_ICR			FMPI2C_ICR

#define FMPI2C_PECR		FMPI2C_PECR + 0x20
#define FMPI2C1_PECR			FMPI2C_PECR

#define FMPI2C_RXDR		FMPI2C_PECR + 0x24
#define FMPI2C1_RXDR			FMPI2C_RXDR

#define FMPI2C_TXDR		FMPI2C_PECR + 0x28
#define FMPI2C1_TXDR			FMPI2C_TXDR

/* --- FMPI2C_CR1 values ----------------------------------------------------- */
/** @defgroup fmpi2c_cr1_values FMPI2C_CR1 values
@{*/

/** PECEN: PEC enable */
#define FMPI2C_CR1_PECEN		(1 << 23)

/** ALERTEN: SMBus alert enable */
#define FMPI2C_CR1_ALERTEN		(1 << 22)

/** SMBDEN: SMBus Device Default address enable */
#define FMPI2C_CR1_SMBDEN		(1 << 21)

/** SMBHEN: SMBus Host address enable */
#define FMPI2C_CR1_SMBHEN		(1 << 20)

/** GCEN: General call enable */
#define FMPI2C_CR1_GCEN			(1 << 19)

/** NOSTRETCH: Clock stretching disable */
#define FMPI2C_CR1_NOSTRETCH		(1 << 17)

/** SBC: Slave byte control */
#define FMPI2C_CR1_SBC			(1 << 16)

/** RXDMAEN: DMA reception requests enable */
#define FMPI2C_CR1_RXDMAEN		(1 << 15)

/** TXDMAEN: DMA transmission requests enable */
#define FMPI2C_CR1_TXDMAEN		(1 << 14)

/** ANFOFF: Analog noise filter OFF */
#define FMPI2C_CR1_ANFOFF		(1 << 12)

/** DNF[3:0]: Digital noise filter */
#define FMPI2C_CR1_DNF_MASK		(0xF << 8)

/** ERRIE: Error interrupts enable */
#define FMPI2C_CR1_ERRIE		(1 << 7)

/** TCIE: Transfer Complete interrupt enable */
#define FMPI2C_CR1_TCIE			(1 << 6)

/** STOPIE: Stop detection Interrupt enable */
#define FMPI2C_CR1_STOPIE		(1 << 5)

/** NACKIE: Not acknowledge received Interrupt enable */
#define FMPI2C_CR1_NACKIE		(1 << 4)

/** ADDRIE: Address match Interrupt enable (slave only) */
#define FMPI2C_CR1_ADDRIE		(1 << 3)

/** RXIE: RX Interrupt enable */
#define FMPI2C_CR1_RXIE			(1 << 2)

/** TXIE: TX Interrupt enable */
#define FMPI2C_CR1_TXIE			(1 << 1)

/** PE: Peripheral enable */
#define FMPI2C_CR1_PE			(1 << 0)

/**@}*/

/* --- FMPI2C_CR2 values ----------------------------------------------------- */
/** @defgroup fmpi2c_cr2_values FMPI2C_CR2 values
@{*/

/** PECBYTE: Packet error checking byte */
#define FMPI2C_CR2_PECBYTE		(1 << 26)

/** AUTOEND: Automatic end mode (master mode) */
#define FMPI2C_CR2_AUTOEND		(1 << 25)

/** RELOAD: NBYTES reload mode */
#define FMPI2C_CR2_RELOAD		(1 << 24)

/** NBYTES[7:0]: Number of bytes */
#define FMPI2C_CR2_NBYTES_MASK		(0xFF << 16)

/** NACK: NACK generation (slave mode) */
#define FMPI2C_CR2_NACK			(1 << 15)

/** STOP: Stop generation (master mode) */
#define FMPI2C_CR2_STOP			(1 << 14)

/** START: Start generation */
#define FMPI2C_CR2_START		(1 << 13)

/** HEAD10R: 10-bit address header only read direction (master receiver mode) */
#define FMPI2C_CR2_HEAD10R		(1 << 12)

/** ADD10: 10-bit addressing mode (master mode) */
#define FMPI2C_CR2_ADD10		(1 << 11)

/** RD_WRN: Transfer direction (master mode) */
#define FMPI2C_CR2_RD_WRN		(1 << 10)

/** SADD[9:8]: 8 (10bit master mode) */
#define FMPI2C_CR2_SADD10_MASK		(0x3 << 8)

/** SADD[7:1]: 1 (master mode) */
#define FMPI2C_CR2_SADD_MASK		(0x7F << 1)

/** SADD0: Slave address bit 0 (master mode) */
#define FMPI2C_CR2_SADD0		(1 << 0)

/**@}*/

/* --- FMPI2C_OAR1 values ----------------------------------------------------- */
/** @defgroup fmpi2c_oar1_values FMPI2C_OAR1 values
@{*/

/** OA1EN: Own Address 1 enable */
#define FMPI2C_OAR1_OA1EN		(1 << 15)

/** OA1MODE: Own Address 1 10-bit mode */
#define FMPI2C_OAR1_OA1MODE		(1 << 10)

/** OA1[9:8]: 10bit Interface address */
#define FMPI2C_OAR1_OA1_10B_MASK		(0x3 << 8)

/** OA1[7:1]: 7/10bit Interface address */
#define FMPI2C_OAR1_OA1_MASK		(0x7F << 1)

/** OA1: 7bit Interface address */
#define FMPI2C_OAR1_7B_OA1_MASK			(1 << 0)

/**@}*/

/* --- FMPI2C_OAR2 values ----------------------------------------------------- */
/** @defgroup fmpi2c_oar2_values FMPI2C_OAR2 values
@{*/

/** OA2EN: Own Address 2 enable */
#define FMPI2C_OAR2_OA2EN		(1 << 15)

/** OA2MSK[2:0]: Own Address 2 masks */
#define FMPI2C_OAR2_OA2MSK_MASK		(0x7 << 8)

/** OA2[7:1]: Interface address */
#define FMPI2C_OAR2_OA2_MASK		(0x7F << 1)

/**@}*/

/* --- FMPI2C_TIMINGR values ----------------------------------------------------- */
/** @defgroup fmpi2c_timingr_values FMPI2C_TIMINGR values
@{*/

/** PRESC[3:0]: Timing prescaler */
#define FMPI2C_TIMINGR_PRESC_MASK	(0xF << 28)

/** SCLDEL[3:0]: Data setup time */
#define FMPI2C_TIMINGR_SCLDEL_MASK	(0xF << 20)

/** SDADEL[3:0]: Data hold time */
#define FMPI2C_TIMINGR_SDADEL_MASK	(0xF << 16)

/** SCLH[7:0]: SCL high period (master mode) */
#define FMPI2C_TIMINGR_SCLH_MASK	(0xFF << 8)

/** SCLL[7:0]: SCL low period (master mode) */
#define FMPI2C_TIMINGR_SCLL_MASK	(0xFF << 0)

/**@}*/

/* --- FMPI2C_TIMEOUTR values ----------------------------------------------------- */
/** @defgroup fmpi2c_timeoutr_values FMPI2C_TIMEOUTR values
@{*/

/** TEXTEN: Extended clock timeout enable */
#define FMPI2C_TIMEOUTR_TEXTEN		(1 << 31)

/** TIMEOUTB[11:0]: Bus timeout B */
#define FMPI2C_TIMEOUTR_TIMEOUTB_MASK	(0xFFF << 16)

/** TIMOUTEN: Clock timeout enable */
#define FMPI2C_TIMEOUTR_TIMOUTEN	(1 << 15)

/** TIDLE: Idle clock timeout detection */
#define FMPI2C_TIMEOUTR_TIDLE		(1 << 12)

/** TIMEOUTA[11:0]: Bus Timeout A */
#define FMPI2C_TIMEOUTR_TIMEOUTA_MASK	(0xFFF << 0)

/**@}*/

/* --- FMPI2C_ISR values ----------------------------------------------------- */
/** @defgroup fmpi2c_isr_values FMPI2C_ISR values
@{*/

/** ADDCODE[6:0]: Address match code (Slave mode) */
#define FMPI2C_ISR_ADDCODE_MASK		(0x7F << 17)

/** DIR: Transfer direction (Slave mode) */
#define FMPI2C_ISR_DIR			(1 << 16)

/** BUSY: Bus busy */
#define FMPI2C_ISR_BUSY			(1 << 15)

/** ALERT: SMBus alert */
#define FMPI2C_ISR_ALERT		(1 << 13)

/** TIMEOUT: Timeout or tLOW detection flag */
#define FMPI2C_ISR_TIMEOUT		(1 << 12)

/**@}*/

/* --- FMPI2C_ISR values ----------------------------------------------------- */
/** @defgroup fmpi2c_isr_values FMPI2C_ISR values
@{*/

/** ADDCODE[6:0]: Address match code (Slave mode) */
#define FMPI2C_ISR_ADDCODE_MASK		(0x7F << 17)

/** DIR: Transfer direction (Slave mode) */
#define FMPI2C_ISR_DIR			(1 << 16)

/** BUSY: Bus busy */
#define FMPI2C_ISR_BUSY			(1 << 15)

/** ALERT: SMBus alert */
#define FMPI2C_ISR_ALERT		(1 << 13)

/** TIMEOUT: Timeout or tLOW detection flag */
#define FMPI2C_ISR_TIMEOUT		(1 << 12)

/** PECERR: PEC Error in reception */
#define FMPI2C_ISR_PECERR		(1 << 11)

/** OVR: Overrun/Underrun (slave mode) */
#define FMPI2C_ISR_OVR			(1 << 10)

/** ARLO: Arbitration lost */
#define FMPI2C_ISR_ARLO			(1 << 9)

/** BERR: Bus error */
#define FMPI2C_ISR_BERR			(1 << 8)

/** TCR: Transfer Complete Reload */
#define FMPI2C_ISR_TCR			(1 << 7)

/** TC: Transfer Complete (master mode) */
#define FMPI2C_ISR_TC			(1 << 6)

/** STOPF: Stop detection flag */
#define FMPI2C_ISR_STOPF		(1 << 5)

/** NACKF: Not Acknowledge received flag */
#define FMPI2C_ISR_NACKF		(1 << 4)

/** ADDR: Address matched (slave mode) */
#define FMPI2C_ISR_ADDR			(1 << 3)

/** RXNE: Receive data register not empty (receivers) */
#define FMPI2C_ISR_RXNE			(1 << 2)

/** TXIS: Transmit interrupt status (transmitters) */
#define FMPI2C_ISR_TXIS			(1 << 1)

/** TXE: Transmit data register empty (transmitters) */
#define FMPI2C_ISR_TXE			(1 << 0)

/**@}*/

/* --- FMPI2C_ICR values ----------------------------------------------------- */
/** @defgroup fmpi2c_icr_values FMPI2C_ICR values
@{*/

/** RXNE: Receive data register not empty (receivers) */
#define FMPI2C_ICR_RXNE			(1 << 2)

/** TXIS: Transmit interrupt status (transmitters) */
#define FMPI2C_ICR_TXIS			(1 << 1)

/** TXE: Transmit data register empty (transmitters) */
#define FMPI2C_ICR_TXE			(1 << 0)

/** ALERTCF: Alert flag clear */
#define FMPI2C_ICR_ALERTCF		(1 << 13)

/** TIMOUTCF: Timeout detection flag clear */
#define FMPI2C_ICR_TIMOUTCF		(1 << 12)

/** PECCF: PEC Error flag clear */
#define FMPI2C_ICR_PECCF		(1 << 11)

/** OVRCF: Overrun/Underrun flag clear */
#define FMPI2C_ICR_OVRCF		(1 << 10)

/** ARLOCF: Arbitration lost flag clear */
#define FMPI2C_ICR_ARLOCF		(1 << 9)

/** BERRCF: Bus error flag clear */
#define FMPI2C_ICR_BERRCF		(1 << 8)

/** STOPCF: STOP detection flag clear */
#define FMPI2C_ICR_STOPCF		(1 << 5)

/** NACKCF: Not Acknowledge flag clear */
#define FMPI2C_ICR_NACKCF		(1 << 4)

/** ADDRCF: Address matched flag clear */
#define FMPI2C_ICR_ADDRCF		(1 << 3)

/**@}*/

/* --- FMPI2C_PECR values ----------------------------------------------------- */
/** @defgroup fmpi2c_pecr_values FMPI2C_PECR values
@{*/

/** OVRCF: Overrun/Underrun flag clear */
#define FMPI2C_PECR_OVRCF		(1 << 10)

/** ARLOCF: Arbitration lost flag clear */
#define FMPI2C_PECR_ARLOCF		(1 << 9)

/** BERRCF: Bus error flag clear */
#define FMPI2C_PECR_BERRCF		(1 << 8)

/** STOPCF: STOP detection flag clear */
#define FMPI2C_PECR_STOPCF		(1 << 5)

/** NACKCF: Not Acknowledge flag clear */
#define FMPI2C_PECR_NACKCF		(1 << 4)

/** ADDRCF: Address matched flag clear */
#define FMPI2C_PECR_ADDRCF		(1 << 3)

/** PEC[7:0]: 0] Packet error checking register */
#define FMPI2C_PECR_PEC_MASK		(0xFF << 0)

/**@}*/

/* --- FMPI2C_RXDR values ----------------------------------------------------- */
/** @defgroup fmpi2c_rxdr_values FMPI2C_RXDR values
@{*/

/** RXDATA[7:0]: 0] 8-bit receive data */
#define FMPI2C_RXDR_RXDATA_MASK		(0xFF << 0)

/**@}*/

/* --- FMPI2C_TXDR values ----------------------------------------------------- */
/** @defgroup fmpi2c_txdr_values FMPI2C_TXDR values
@{*/

/** TXDATA[7:0]: 0] 8-bit transmit data */
#define FMPI2C_TXDR_TXDATA_MASK		(0xFF << 0)

/**@}*/


#endif
