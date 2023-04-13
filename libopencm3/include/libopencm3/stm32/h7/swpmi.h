#ifndef LIBOPENCM3_SWPMI_H
#define LIBOPENCM3_SWPMI_H

#define SWPMI_CR   MMIO32((SWPMI_BASE) + 0x0)
#define SWPMI_BRR   MMIO32((SWPMI_BASE) + 0x04)
#define SWPMI_ISR   MMIO32((SWPMI_BASE) + 0x0C)
#define SWPMI_ICR   MMIO32((SWPMI_BASE) + 0x10)
#define SMPMI_IER   MMIO32((SWPMI_BASE) + 0x14)
#define SWPMI_RFL   MMIO32((SWPMI_BASE) + 0x18)
#define SWPMI_TDR   MMIO32((SWPMI_BASE) + 0x1C)
#define SWPMI_RDR   MMIO32((SWPMI_BASE) + 0x20)
#define SWPMI_OR   MMIO32((SWPMI_BASE) + 0x24)


/* --- SWPMI_CR values ----------------------------------------------------- */
/** @defgroup swpmi_cr_values SWPMI_CR values
@{*/

/** SWPTEN: Single wire protocol master transceiver enable */
#define SWPMI_CR_SWPTEN			(1 << 11)

/** DEACT: Single wire protocol master interface deactivate */
#define SWPMI_CR_DEACT			(1 << 10)

/** SWPACT: Single wire protocol master interface activate */
#define SWPMI_CR_SWPACT			(1 << 5)

/** LPBK: Loopback mode enable */
#define SWPMI_CR_LPBK			(1 << 4)

/** TXMODE: Transmission buffering mode */
#define SWPMI_CR_TXMODE			(1 << 3)

/** RXMODE: Reception buffering mode */
#define SWPMI_CR_RXMODE			(1 << 2)

/** TXDMA: Transmission DMA enable */
#define SWPMI_CR_TXDMA			(1 << 1)

/** RXDMA: Reception DMA enable */
#define SWPMI_CR_RXDMA			(1 << 0)

/**@}*/

/* --- SWPMI_BRR values ----------------------------------------------------- */
/** @defgroup swpmi_brr_values SWPMI_BRR values
@{*/

/** BR[7:0]: Bitrate prescaler */
#define SWPMI_BRR_BR_MASK		(0xFF << 0)
#define SWPMI_BRR_BR_OFFSET		7U

/**@}*/

/* --- SWPMI_ISR values ----------------------------------------------------- */
/** @defgroup swpmi_isr_values SWPMI_ISR values
@{*/

/** RDYF: transceiver ready flag */
#define SWPMI_ISR_RDYF			(1 << 11)
#define SWPMI_ISR_RDYF_OFFSET	11u
/** DEACTF: DEACTIVATED flag */
#define SWPMI_ISR_DEACTF		(1 << 10)
#define SWPMI_ISR_DEACTF_OFFSET 10u
/** SUSP: SUSPEND flag */
#define SWPMI_ISR_SUSP			(1 << 9)
#define SWPMI_ISR_SUSP_OFFSET   9u
/** SRF: Slave resume flag */
#define SWPMI_ISR_SRF			(1 << 8)
#define SWPMI_ISR_SRF_OFFSET    8u
/** TCF: Transfer complete flag */
#define SWPMI_ISR_TCF			(1 << 7)
#define SWPMI_ISR_TCF_OFFSET    7U
/** TXE: Transmit data register empty */
#define SWPMI_ISR_TXE			(1 << 6)
#define SWPMI_ISR_TXE_OFFSET    6U
/** RXNE: Receive data register not empty */
#define SWPMI_ISR_RXNE			(1 << 5)
#define SWPMI_ISR_RXNE_OFFSET   5U
/** TXUNRF: Transmit underrun error flag */
#define SWPMI_ISR_TXUNRF		(1 << 4)
#define SWPMI_ISR_TXUNRF_OFFSET 4U
/** RXOVRF: Receive overrun error flag */
#define SWPMI_ISR_RXOVRF		(1 << 3)
#define SWPMI_ISR_RXOVRF_OFFSET 3U
/** RXBERF: Receive CRC error flag */
#define SWPMI_ISR_RXBERF		(1 << 2)
#define SWPMI_ISR_RXBERF_OFFSET 2U
/** TXBEF: Transmit buffer empty flag */
#define SWPMI_ISR_TXBEF			(1 << 1)
#define SWPMI_ISR_TXBEF_OFFSET  1U
/** RXBFF: Receive buffer full flag */
#define SWPMI_ISR_RXBFF			(1 << 0)
#define SWPMI_ISR_RXBFF_OFFSET 0U
/**@}*/

/* --- SWPMI_ICR values ----------------------------------------------------- */
/** @defgroup swpmi_icr_values SWPMI_ICR values
@{*/

/** CRDYF: Bit 11 CRDYF Clear transceiver ready flag */
#define SWPMI_ICR_CRDYF			(1 << 11)
#define SWPMI_ICR_CRDYF_OFFSET  11U
/** CSRF: Clear slave resume flag */
#define SWPMI_ICR_CSRF			(1 << 8)
#define SWPMI_ICR_CSRF_OFFSET   8U
/** CTCF: Clear transfer complete flag */
#define SWPMI_ICR_CTCF			(1 << 7)
#define SWPMI_ICR_CTCF_OFFSET   7U
/** CTXUNRF: Clear transmit underrun error flag */
#define SWPMI_ICR_CTXUNRF		(1 << 4)
#define SWPMI_ICR_CTXUNRF_OFFSET    4U
/** CRXOVRF: Clear receive overrun error flag */
#define SWPMI_ICR_CRXOVRF		(1 << 3)
#define SWPMI_ICR_CRXOVRF_OFFSET    3U
/** CRXBERF: Clear receive CRC error flag */
#define SWPMI_ICR_CRXBERF		(1 << 2)
#define SWPMI_ICR_CRXBERF_OFFSET    2U
/** CTXBEF: Clear transmit buffer empty flag */
#define SWPMI_ICR_CTXBEF		(1 << 1)
#define SWPMI_ICR_CTXBEF_OFFSET 1U
/** CRXBFF: Clear receive buffer full flag */
#define SWPMI_ICR_CRXBFF		(1 << 0)
#define SWPMI_ICR_CRXBFF_OFFSET 0U
/**@}*/

/* --- SMPMI_IER values ----------------------------------------------------- */
/** @defgroup smpmi_ier_values SMPMI_IER values
@{*/

/** RDYIE: Transceiver ready interrupt enable */
#define SMPMI_IER_RDYIE			(1 << 11)

/** SRIE: Slave resume interrupt enable */
#define SMPMI_IER_SRIE			(1 << 8)

/** TCIE: Transmit complete interrupt enable */
#define SMPMI_IER_TCIE			(1 << 7)

/** TIE: Transmit interrupt enable */
#define SMPMI_IER_TIE			(1 << 6)

/** RIE: Receive interrupt enable */
#define SMPMI_IER_RIE			(1 << 5)

/** TXUNRIE: Transmit underrun error interrupt enable */
#define SMPMI_IER_TXUNRIE		(1 << 4)

/** RXOVRIE: Receive overrun error interrupt enable */
#define SMPMI_IER_RXOVRIE		(1 << 3)

/** RXBERIE: Receive CRC error interrupt enable */
#define SMPMI_IER_RXBERIE		(1 << 2)

/** TXBEIE: Transmit buffer empty interrupt enable */
#define SMPMI_IER_TXBEIE		(1 << 1)

/** RXBFIE: Receive buffer full interrupt enable */
#define SMPMI_IER_RXBFIE		(1 << 0)

/**@}*/


/* --- SWPMI_RFL values ----------------------------------------------------- */
/** @defgroup swpmi_rfl_values SWPMI_RFL values
@{*/

/** RFL[4:0]: Receive frame length */
#define SWPMI_RFL_RFL_MASK		(0x1F << 0)

/**@}*/


/* --- SWPMI_TDR values ----------------------------------------------------- */
/** @defgroup swpmi_tdr_values SWPMI_TDR values
@{*/

/** TD[31:0]: Transmit data */
#define SWPMI_TDR_TD_MASK		(0xFFFFFFFF << 0)

/**@}*/

/* --- SWPMI_RDR values ----------------------------------------------------- */
/** @defgroup swpmi_rdr_values SWPMI_RDR values
@{*/

/** RD[31:0]: received data */
#define SWPMI_RDR_RD_MASK		(0xFFFFFFFF << 0)

/**@}*/

/* --- SWPMI_OR values ----------------------------------------------------- */
/** @defgroup swpmi_or_values SWPMI_OR values
@{*/

/** SWP_CLASS: SWP class selection */
#define SWPMI_OR_SWP_CLASS		(1 << 1)
#define SWPMI_OR_SWP_CLASS_OFFSET   1U
/** SWP_TBYP: SWP transceiver bypass */
#define SWPMI_OR_SWP_TBYP		(1 << 0)
#define SWPMI_OR_SWP_TBYP_OFFSET 0U
/**@}*/

void swpmi_enable(void);
void swpmi_disable(void);
void swpmi_deactivate(void);
void swpmi_activate(void);
void swpmi_loopback(bool onoff);
void swpmi_set_tx_single_buffer(void);
void swpmi_set_tx_multi_buffer(void);
void swpmi_set_rx_single_buffer(void);
void swpmi_set_rx_multi_buffer(void);
void swpmi_enable_tx_dma(void);
void swpmi_disable_tx_dma(void);
void swpmi_enable_rx_dma(void);
void swpmi_disable_rx_dma(void);
void swpmi_set_bitrate(uint16_t bitr);
int swpmi_get_transfer_ready_flag(void);
int swpmi_get_deactivated_flag(void);
int swpmi_get_suspended_flag(void);
int swpmi_get_slave_resume_flag(void);
int swpmi_get_transfer_complete_flag(void);
int swpmi_get_tx_register_empty_flag(void);
int swpmi_get_rx_register_not_empty_flag(void);
int swpmi_get_tx_underrun_flag(void);
int swpmi_get_rx_overrun_flag(void);
int swpmi_get_rx_crc_error_flag(void);
int swpmi_get_tx_buffer_empty_flag(void);
int swpmi_get_rx_buffer_full_flag(void);
void swpmi_clear_transceiver_ready_flag(void);
void swpmi_clear_slave_resume_flag(void);
void swpmi_clear_transfer_complete_flag(void);
void swpmi_clear_tx_underrun_flag(void);
void swpmi_clear_rx_overrun_flag(void);
void swpmi_clear_rx_crc_error_flag(void);
void swpmi_clear_tx_buffer_empty_flag(void);
void swpmi_clear_rx_buffer_full_flag(void);
void swpmi_enable_transceiver_ready_interrupt(void);
void swpmi_enable_slave_resume_interrupt(void);
void swpmi_enable_tx_complete_interrupt(void);
void swpmi_enable_tx_interrupt(void);
void swpmi_enable_rx_interrupt(void);
void swpmi_enable_tx_underrun_interrupt(void);
void swpmi_enable_rx_overrun_interrupt(void);
void swpmi_enable_rx_crc_error_interrupt(void);
void swpmi_enable_tx_buffer_empty_interrupt(void);
void swpmi_enable_rx_buffer_full_interrupt(void);
void swpmi_disable_transceiver_ready_interrupt(void);
void swpmi_disable_slave_resume_interrupt(void);
void swpmi_disable_tx_complete_interrupt(void);
void swpmi_disable_tx_interrupt(void);
void swpmi_disable_rx_interrupt(void);
void swpmi_disable_tx_underrun_interrupt(void);
void swpmi_disable_rx_overrun_interrupt(void);
void swpmi_disable_rx_crc_error_interrupt(void);
void swpmi_disable_tx_buffer_empty_interrupt(void);
void swpmi_disable_rx_buffer_full_interrupt(void);
void swpmi_set_rx_frame_length(uint16_t flen);
void swpmi_write(uint32_t data);
uint32_t swpmi_read(void);
void swpmi_set_class_c(void);
void swpmi_set_class_b(void);
void swpmi_enable_internal_transceiver(void);
void swpmi_disable_internal_transceiver(void);
void swpmi_setup_pin(void);


#endif
