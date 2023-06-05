/** @defgroup usart_defines USART Defines
 *
 * @brief <b>Defined Constants and Types for the STM32H7xx USART</b>
 *
 * @ingroup STM32H7xx_defines
 *
 * @version 1.0.0
 *
 * @date 6 November 2019
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

#ifndef LIBOPENCM3_USART_H
#define LIBOPENCM3_USART_H

#include <libopencm3/stm32/common/usart_common_fifos.h>
#include <libopencm3/stm32/dma.h>
/**@{*/
/** @defgroup usart_reg_base USART register base addresses
 * Holds all the U(S)ART peripherals supported.
 * @{
 */
#define USART1          USART1_BASE
#define USART2          USART2_BASE
#define USART3          USART3_BASE
#define UART4           UART4_BASE
#define UART5           UART5_BASE
#define USART6          USART6_BASE
#define UART7           UART7_BASE
#define UART8           UART8_BASE
/**@}*/

#define USART_ICR_TCBGTCF			(1 << 7)

BEGIN_DECLS
void usart_set_oversample_8(uint32_t usart);
void usart_set_oversample_16(uint32_t usart);
void usart_set_autobaud_mode(uint32_t usart, uint32_t abmode);
void usart_enable_autobaud(uint32_t usart);
void usart_disable_autobaud(uint32_t usart);
void usart_enable_msb(uint32_t usart);
void usart_enable_lsb(uint32_t usart);
void usart_enable_binary_inversion(uint32_t usart);
void usart_disable_binary_inversion(uint32_t usart);
void usart_enable_tx_level_inversion(uint32_t usart);
void usart_disable_tx_level_inversion(uint32_t usart);
void usart_enable_rx_level_inversion(uint32_t usart);
void usart_disable_rx_level_inversion(uint32_t usart);
void usart_enable_swap_io_pins(uint32_t usart);
void usart_disable_swap_io_pins(uint32_t usart);
void usart_enable_lin_mode(uint32_t usart);
void usart_disable_lin_mode(uint32_t usart);
void usart_enable_lin_break_detection_int(uint32_t usart);
void usart_disable_lin_break_detection_int(uint32_t usart);
void usart_set_lin_break_detection_length_10(uint32_t usart);
void usart_set_lin_break_detection_length_11(uint32_t usart);
void usart_enable_irda_mode(uint32_t usart);
void usart_disable_irda_mode(uint32_t usart);
void usart_request_autobaud_next_rx(uint32_t usart);
//void usart_enable_idle_interrupt(uint32_t usart);
void usart_clear_idle_interrupt(uint32_t usart);
void usart_clear_interrupt_flags_all(uint32_t usart);
END_DECLS


/**@}*/

#endif

