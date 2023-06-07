

#include <libopencm3/stm32/syscfg.h>
#define EXTICR_SELECTION_FIELDSIZE	4
#define EXTICR_SELECTION_REG(x)	SYSCFG_EXTICR(x)

void h7_exti_select_source(uint32_t exti, uint32_t gpioport);

void h7_exti_select_source(uint32_t exti, uint32_t gpioport)
{
	uint32_t line;
	for (line = 0; line < 16; line++) {
		if (!(exti & (1 << line))) {
			continue;
		}

		uint32_t bits = 0;

		switch (gpioport) {
		case GPIOA:
			bits = 0;
			break;
		case GPIOB:
			bits = 1;
			break;
		case GPIOC:
			bits = 2;
			break;
		case GPIOD:
			bits = 3;
			break;
		case GPIOE:
			bits = 4;
			break;
		case GPIOF:
			bits = 5;
			break;
		case GPIOG:
			bits = 6;
			break;
		case GPIOH:
			bits = 7;
			break;
		case GPIOI:
			bits = 8;
			break;
		case GPIOJ:
			bits = 9;
			break;
		case GPIOK:
			bits = 10;
			break;
		}

		uint8_t shift = (uint8_t)(EXTICR_SELECTION_FIELDSIZE * (line % 4));
		uint32_t mask = ((1 << EXTICR_SELECTION_FIELDSIZE) - 1) << shift;
		uint32_t reg = line / 4;

		SYSCFG_EXTICR2 = (SYSCFG_EXTICR2 & ~mask) | (bits << shift);
	};
}
