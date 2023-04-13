#include <libopencm3/stm32/swpmi.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
void swpmi_enable(void)
{
    SWPMI_CR  |= SWPMI_CR_SWPTEN;
}

void swpmi_disable(void)
{
    SWPMI_CR  &= ~SWPMI_CR_SWPTEN;
}

void swpmi_deactivate(void)
{
    SWPMI_CR  |= SWPMI_CR_DEACT;
}


/* switches to suspend state from deactivated*/
void swpmi_activate(void)
{
    SWPMI_CR  &= ~SWPMI_CR_DEACT;
    SWPMI_CR  |= SWPMI_CR_SWPACT;
}

void swpmi_loopback(bool onoff)
{
    if (onoff == 0) {
    SWPMI_CR  &= ~SWPMI_CR_LPBK;
    } else {
    SWPMI_CR  |= SWPMI_CR_LPBK;
    }
}

void swpmi_set_tx_single_buffer(void)
{
    SWPMI_CR  &= ~SWPMI_CR_SWPACT;
    SWPMI_CR  &= ~SWPMI_CR_TXMODE;
}

void swpmi_set_tx_multi_buffer(void)
{
    SWPMI_CR  &= ~SWPMI_CR_SWPACT;
    SWPMI_CR  |= SWPMI_CR_TXMODE;
}

void swpmi_set_rx_single_buffer(void)
{
    SWPMI_CR  &= ~SWPMI_CR_SWPACT;
    SWPMI_CR  &= ~SWPMI_CR_RXMODE;
}

void swpmi_set_rx_multi_buffer(void)
{
    SWPMI_CR  &= ~SWPMI_CR_SWPACT;
    SWPMI_CR  |= SWPMI_CR_RXMODE;
}

void swpmi_enable_tx_dma(void)
{
    SWPMI_CR  &= ~SWPMI_CR_SWPACT;
    SWPMI_CR  |= SWPMI_CR_TXDMA;
}

void swpmi_disable_tx_dma(void)
{
    SWPMI_CR  &= ~SWPMI_CR_SWPACT;
    SWPMI_CR  &= ~SWPMI_CR_TXDMA;
}

void swpmi_enable_rx_dma(void)
{
    SWPMI_CR  &= ~SWPMI_CR_SWPACT;
    SWPMI_CR  |= SWPMI_CR_RXDMA;
}

void swpmi_disable_rx_dma(void)
{
    SWPMI_CR  &= ~SWPMI_CR_SWPACT;
    SWPMI_CR  &= ~SWPMI_CR_RXDMA;
}

void swpmi_set_bitrate(uint16_t bitr)
{
    SWPMI_CR  &= ~SWPMI_CR_SWPACT;
    SWPMI_BRR  |= (bitr << SWPMI_BRR_BR_OFFSET);
}

int swpmi_get_transfer_ready_flag(void)
{
    return (SWPMI_ISR & SWPMI_ISR_RDYF_OFFSET);
}

int swpmi_get_deactivated_flag(void)
{
    return (SWPMI_ISR & SWPMI_ISR_DEACTF_OFFSET);
}

int swpmi_get_suspended_flag(void)
{
    return (SWPMI_ISR & SWPMI_ISR_SUSP_OFFSET);
}

int swpmi_get_slave_resume_flag(void)
{
    return (SWPMI_ISR & SWPMI_ISR_SRF_OFFSET);
}

int swpmi_get_transfer_complete_flag(void)
{
    return (SWPMI_ISR & SWPMI_ISR_TCF_OFFSET);
}

int swpmi_get_tx_register_empty_flag(void)
{
    return (SWPMI_ISR & SWPMI_ISR_TXE_OFFSET);
}

int swpmi_get_rx_register_not_empty_flag(void)
{
    return (SWPMI_ISR & SWPMI_ISR_RXNE_OFFSET);
}

int swpmi_get_tx_underrun_flag(void)
{
    return (SWPMI_ISR & SWPMI_ISR_TXUNRF_OFFSET);
}

int swpmi_get_rx_overrun_flag(void)
{
    return (SWPMI_ISR & SWPMI_ISR_RXOVRF_OFFSET);
}

int swpmi_get_rx_crc_error_flag(void)
{
    return (SWPMI_ISR & SWPMI_ISR_RXBERF_OFFSET);
}

int swpmi_get_tx_buffer_empty_flag(void)
{
    return (SWPMI_ISR & SWPMI_ISR_TXBEF_OFFSET);
}

int swpmi_get_rx_buffer_full_flag(void)
{
    return (SWPMI_ISR & SWPMI_ISR_RXBFF_OFFSET);
}

void swpmi_clear_transceiver_ready_flag(void)
{
   SWPMI_ICR |= SWPMI_ICR_CRDYF;
}

void swpmi_clear_slave_resume_flag(void)
{
   SWPMI_ICR |= SWPMI_ICR_CSRF;
}

void swpmi_clear_transfer_complete_flag(void)
{
   SWPMI_ICR |= SWPMI_ICR_CTCF;
}

void swpmi_clear_tx_underrun_flag(void)
{
   SWPMI_ICR |= SWPMI_ICR_CTXUNRF;
}

void swpmi_clear_rx_overrun_flag(void)
{
   SWPMI_ICR |= SWPMI_ICR_CRXOVRF;
}

void swpmi_clear_rx_crc_error_flag(void)
{
   SWPMI_ICR |= SWPMI_ICR_CRXBERF;
}

void swpmi_clear_tx_buffer_empty_flag(void)
{
   SWPMI_ICR |= SWPMI_ICR_CTXBEF;
}

void swpmi_clear_rx_buffer_full_flag(void)
{
   SWPMI_ICR |= SWPMI_ICR_CRXBFF;
}

void swpmi_enable_transceiver_ready_interrupt(void)
{
   SMPMI_IER |= SMPMI_IER_RDYIE;
}

void swpmi_enable_slave_resume_interrupt(void)
{
   SMPMI_IER |= SMPMI_IER_SRIE;
}

void swpmi_enable_tx_complete_interrupt(void)
{
   SMPMI_IER |= SMPMI_IER_TCIE;
}

void swpmi_enable_tx_interrupt(void)
{
   SMPMI_IER |= SMPMI_IER_TIE;
}

void swpmi_enable_rx_interrupt(void)
{
   SMPMI_IER |= SMPMI_IER_RIE;
}

void swpmi_enable_tx_underrun_interrupt(void)
{
   SMPMI_IER |= SMPMI_IER_TXUNRIE;
}

void swpmi_enable_rx_overrun_interrupt(void)
{
   SMPMI_IER |= SMPMI_IER_RXOVRIE;
}

void swpmi_enable_rx_crc_error_interrupt(void)
{
   SMPMI_IER |= SMPMI_IER_RXBERIE;
}

void swpmi_enable_tx_buffer_empty_interrupt(void)
{
   SMPMI_IER |= SMPMI_IER_TXBEIE;
}

void swpmi_enable_rx_buffer_full_interrupt(void)
{
   SMPMI_IER |= SMPMI_IER_RXBFIE;
}

void swpmi_disable_transceiver_ready_interrupt(void)
{
   SMPMI_IER &= ~SMPMI_IER_RDYIE;
}

void swpmi_disable_slave_resume_interrupt(void)
{
   SMPMI_IER &= ~SMPMI_IER_SRIE;
}

void swpmi_disable_tx_complete_interrupt(void)
{
   SMPMI_IER &= ~SMPMI_IER_TCIE;
}

void swpmi_disable_tx_interrupt(void)
{
   SMPMI_IER &= ~SMPMI_IER_TIE;
}

void swpmi_disable_rx_interrupt(void)
{
   SMPMI_IER &= ~SMPMI_IER_RIE;
}

void swpmi_disable_tx_underrun_interrupt(void)
{
   SMPMI_IER &= ~SMPMI_IER_TXUNRIE;
}

void swpmi_disable_rx_overrun_interrupt(void)
{
   SMPMI_IER &= ~SMPMI_IER_RXOVRIE;
}

void swpmi_disable_rx_crc_error_interrupt(void)
{
   SMPMI_IER &= ~SMPMI_IER_RXBERIE;
}

void swpmi_disable_tx_buffer_empty_interrupt(void)
{
   SMPMI_IER &= ~SMPMI_IER_TXBEIE;
}

void swpmi_disable_rx_buffer_full_interrupt(void)
{
   SMPMI_IER &= ~SMPMI_IER_RXBFIE;
}

void swpmi_set_rx_frame_length(uint16_t flen)
{
    SWPMI_RFL |= (flen << SWPMI_RFL_RFL_MASK);
}

void swpmi_write(uint32_t data)
{
	/* Write data MUST be 32bit so pad if needed */
	SWPMI_TDR = data;
}

uint32_t swpmi_read(void)
{
	/* Read the data 32bit. */
	return SWPMI_RDR;
}

void swpmi_set_class_c(void)
{
    SWPMI_OR |= (0 << SWPMI_OR_SWP_CLASS_OFFSET);
}

void swpmi_set_class_b(void)
{
    SWPMI_OR |= (1 << SWPMI_OR_SWP_CLASS_OFFSET);
}

void swpmi_enable_internal_transceiver(void)
{
    SWPMI_OR |= (0 << SWPMI_OR_SWP_TBYP_OFFSET);
}

void swpmi_disable_internal_transceiver(void)
{
    SWPMI_OR |= (1 << SWPMI_OR_SWP_TBYP_OFFSET);
}

void swpmi_setup_pin(void)
{
    rcc_periph_clock_enable(RCC_GPIOC);
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE,
        GPIO6
    );

    gpio_set_output_options(GPIOC, GPIO_OTYPE_PP,
				GPIO_OSPEED_100MHZ, GPIO6);

	gpio_set_af(GPIOC, GPIO_AF11,
       GPIO6
    );
}






