#include <libopencm3/stm32/usart.h>

void usart_set_oversample_8(uint32_t usart)
{
	USART_CR1(usart) |= USART_CR1_OVER8;
}

void usart_set_oversample_16(uint32_t usart)
{
	USART_CR1(usart) &= ~USART_CR1_OVER8;
}


/*
#define USART_CR2_ABRMOD_STARTBIT	(0x0 << USART_CR2_ABRMOD_SHIFT)
#define USART_CR2_ABRMOD_FALL_EDGE	(0x1 << USART_CR2_ABRMOD_SHIFT)
#define USART_CR2_ABRMOD_FRAME_0x7F	(0x2 << USART_CR2_ABRMOD_SHIFT)
#define USART_CR2_ABRMOD_FRAME_0x55	(0x3 << USART_CR2_ABRMOD_SHIFT)
*/

void usart_set_autobaud_mode(uint32_t usart, uint32_t abmode)
{

	USART_CR2(usart) |= abmode;
}

void usart_enable_autobaud(uint32_t usart)
{

	USART_CR2(usart) |= USART_CR2_ABREN;
}

void usart_disable_autobaud(uint32_t usart)
{

	USART_CR2(usart) &= ~USART_CR2_ABREN;
}

void usart_enable_msb(uint32_t usart)
{

	USART_CR2(usart) |= USART_CR2_MSBFIRST;
}

void usart_enable_lsb(uint32_t usart)
{

	USART_CR2(usart) &= ~USART_CR2_MSBFIRST;
}

void usart_enable_binary_inversion(uint32_t usart)
{

	USART_CR2(usart) |= USART_CR2_DATAINV;
}

void usart_disable_binary_inversion(uint32_t usart)
{

	USART_CR2(usart) &= ~USART_CR2_DATAINV;
}

void usart_enable_tx_level_inversion(uint32_t usart)
{

	USART_CR2(usart) |= USART_CR2_TXINV;
}

void usart_disable_tx_level_inversion(uint32_t usart)
{

	USART_CR2(usart) &= ~USART_CR2_TXINV;
}

void usart_enable_rx_level_inversion(uint32_t usart)
{

	USART_CR2(usart) |= USART_CR2_RXINV;
}

void usart_disable_rx_level_inversion(uint32_t usart)
{

	USART_CR2(usart) &= ~USART_CR2_RXINV;
}

void usart_enable_swap_io_pins(uint32_t usart)
{

	USART_CR2(usart) |= USART_CR2_SWAP;
}

void usart_disable_swap_io_pins(uint32_t usart)
{

	USART_CR2(usart) &= ~USART_CR2_SWAP;
}

void usart_enable_lin_mode(uint32_t usart)
{

	USART_CR2(usart) |= USART_CR2_LINEN;
}

void usart_disable_lin_mode(uint32_t usart)
{

	USART_CR2(usart) &= ~USART_CR2_LINEN;
}

void usart_enable_lin_break_detection_int(uint32_t usart)
{

	USART_CR2(usart) |= USART_CR2_LBDIE;
}

void usart_disable_lin_break_detection_int(uint32_t usart)
{

	USART_CR2(usart) &= ~USART_CR2_LBDIE;
}

void usart_set_lin_break_detection_length_10(uint32_t usart)
{

	USART_CR2(usart) &= ~USART_CR2_LBDL;
}

void usart_set_lin_break_detection_length_11(uint32_t usart)
{

	USART_CR2(usart) |= USART_CR2_LBDL;
}

void usart_enable_irda_mode(uint32_t usart)
{

	USART_CR3(usart) |= USART_CR3_IREN;
}

void usart_disable_irda_mode(uint32_t usart)
{

	USART_CR3(usart) &= ~USART_CR3_IREN;
}

void usart_request_autobaud_next_rx(uint32_t usart)
{

	USART_RQR(usart) |= USART_RQR_ABRRQ;
}
