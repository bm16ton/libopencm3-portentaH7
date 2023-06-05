
#ifndef CDC_H
#define CDC_H

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
//#include <libopencm3/stm32/dma.h>
#include <setjmp.h>

#define BOOTMAGIC0 0xb007da7a
#define BOOTMAGIC1 0xbaadfeed
#define BOOTMAGIC2 0xbaadfeed
#define BOOTMAGIC3 0xb007da7a
#define BOOTMAGIC4 0xbeedfaad
#define BOOTMAGIC5 0xb007da7a
#define BOOTMAGIC6 0xb006faad
#define BOOTMAGIC7 0xb007d07a

//#define DMA_CGIF DMA_IFCR_CGIF1

extern volatile uint32_t poopsielst;
extern volatile uint32_t poopsiettl;
extern volatile uint32_t poopsielst2;
extern uint8_t b4usb[768];
extern volatile uint32_t currentold;
extern volatile int lcddma;
extern volatile uint16_t lastct;
extern volatile uint16_t ct;

typedef struct usb_cdc_line_coding usb_cdc_line_coding_s;



#define USBUSART_ISR_TEMPLATE(USART, DMA_IRQ) do { \
const char num3[] = {"NUM3"}; \
const char num4[] = {"NUM4"}; \
const char num5[] = {"NUM5"}; \
const char num1[] = {"NUM1"}; \
/* dma_status("yup"); */ \
/* uart_status(USART1, "ser");  */ \
ct = dma_get_current_target(DMA1, DMA_STREAM3); \
volatile uint32_t current = (504 - dma_get_number_of_data(DMA1, DMA_STREAM3)); \
const bool isIdle = usart_get_flag(USART1, USART_FLAG_IDLE); \
/* printf("ct = %d lastct = %d current = %ld poopsielst %ld \r\n", ct, lastct, current, poopsielst); */ \
usart_recv(USART1); \
	if (isIdle) { \
/*	printf("inside  usart1_isr uart is idle\r\n");  */ \
	USART_ICR(USART1) = USART_ICR_IDLECF;  \
/*	usbuart_run();  */ \
/*	usart_disable(USART1);   */ \
	} \
/* uart_status(USART1, "ser"); */ \
/* dma_status("yup"); */ \
if (((current == 0) && (poopsielst == 0)) || ((current == poopsielst) && (current != 252))) { \
printf("returning\r\n"); \
return; \
}; \
if (USART_ISR(USART1) & USART_ISR_TC) { \
USART_ICR(USART1) |= USART_ICR_TCCF; \
} \
if (poopsielst > current) { \
if (ct == 0) { \
while (usbd_ep_write_packet(g_usbd_dev, 0x85, &buf_rx2[poopsielst], (504 - poopsielst)) == 0);  \
while (usbd_ep_write_packet(g_usbd_dev, 0x85, &buf_rx[0], current) == 0);  \
} else { \
while (usbd_ep_write_packet(g_usbd_dev, 0x85, &buf_rx[poopsielst], (504 - poopsielst)) == 0);  \
while (usbd_ep_write_packet(g_usbd_dev, 0x85, &buf_rx2[0], current) == 0); \
	} \
poopsielst = current; \
lastct = ct; \
return; \
	} \
if (ct == lastct) { \
volatile uint32_t poopsie = (current - poopsielst); \
if (dma_get_current_target(DMA1, DMA_STREAM3) == 0) { \
while (usbd_ep_write_packet(g_usbd_dev, 0x85, &buf_rx[poopsielst], poopsie) == 0); \
} else { \
while (usbd_ep_write_packet(g_usbd_dev, 0x85, &buf_rx2[poopsielst], poopsie) == 0); \
	} \
} \
if (ct != lastct) { \
if (ct == 0) { \
/* while (usbd_ep_write_packet(g_usbd_dev, 0x85, &buf_rx2[poopsielst], (504 - poopsielst)) == 0); */ \
while (usbd_ep_write_packet(g_usbd_dev, 0x85, &buf_rx[0], current) == 0);  \
} else { \
/* while (usbd_ep_write_packet(g_usbd_dev, 0x85, &buf_rx[poopsielst], (504 - poopsielst)) == 0); */ \
while (usbd_ep_write_packet(g_usbd_dev, 0x85, &buf_rx2[0], current) == 0); \
	} \
} \
lastct = ct; \
poopsielst = current; \
} while(0)





#define USBUSART_DMA_RX_ISR_TEMPLATE(USART_IRQ, DMA_RX_CHAN) do {		\
volatile uint16_t ct2 = dma_get_current_target(DMA1, DMA_STREAM3); \
volatile uint32_t current2 = (504 - dma_get_number_of_data(DMA1, DMA_STREAM3)); \
const char num6[] = {"NUM6"}; \
const char num2[] = {"NUM2"}; \
/* printf("ct2 = %d lastct = %d current2 = %ld poopsielst %ld \r\n", ct2, lastct, current2, poopsielst); */ \
if (DMA_LISR(DMA1) & DMA_LISR27_TCIE) { \
DMA_LIFCR(DMA1) |= DMA_LIFCR27_TCIE; \
/* USBUSART_ISR_TEMPLATE(USART1, DMA_STREAM3); */ \
		} \
if ((current2 == 0) && (poopsielst == 0)) { \
if (ct2 == 1) { \
while (usbd_ep_write_packet(g_usbd_dev, 0x85, &buf_rx[0], 504) == 0);  \
} else { \
while (usbd_ep_write_packet(g_usbd_dev, 0x85, &buf_rx2[0], 504) == 0);  \
} \
 lastct = ct2;  \
 poopsielst = 0;  \
return; \
} \
if (ct2) { \
while (usbd_ep_write_packet(g_usbd_dev, 0x85, &buf_rx[poopsielst], (504 - poopsielst)) == 0);  \
__DSB(); \
/* while (usbd_ep_write_packet(g_usbd_dev, 0x85, buf_rx2, current2) == 0); */ \
} else { \
current2 = (504 - dma_get_number_of_data(DMA1, DMA_STREAM3)); \
while (usbd_ep_write_packet(g_usbd_dev, 0x85, &buf_rx2[poopsielst], (504 - poopsielst)) == 0);  \
__DSB(); \
/* while (usbd_ep_write_packet(g_usbd_dev, 0x85, buf_rx, current2) == 0); */\
	} \
if ((current2 == 0) && (poopsielst == 0)) { \
while (usbd_ep_write_packet(g_usbd_dev, 0x85, &buf_rx[0], 504) == 0);  \
 lastct = 1;  \
 poopsielst = 0;  \
return; \
} \
if (USART_ISR(USART1) & USART_ISR_TC) { \
USART_ICR(USART1) |= USART_ICR_TCCF; \
} \
 lastct = ct2;  \
 poopsielst = 0;  \
/* USBUSART_ISR_TEMPLATE(USART1, DMA_STREAM3); */ \
} while(0)


void SysTick_IRQn_handler( void );
void delay_ms( uint32_t ms );
int sdramsetup( void );
extern volatile uint32_t systick;
int _write(int file, char *ptr, int len);
//void i2c_init(void);
uint32_t SysTick_Config(uint32_t ticks);

#endif
