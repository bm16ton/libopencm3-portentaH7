

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/stm32/rcc.h>

void setcm4bootadd0(uint16_t bootadd) {
SYSCFG_UR(3) = (SYSCFG_UR(3) & ~(SYSCFG_UR3_BCM4_ADD0_MASK << SYSCFG_UR3_BCM4_ADD0_SHIFT)) | ((uint32_t)bootadd << SYSCFG_UR3_BCM4_ADD0_SHIFT);
}

void setcm4bootadd1(uint16_t bootadd) {
SYSCFG_UR(4) = (SYSCFG_UR(4) & ~(SYSCFG_UR4_BCM4_ADD1_MASK << SYSCFG_UR4_BCM4_ADD1_SHIFT)) | ((uint32_t)bootadd << SYSCFG_UR4_BCM4_ADD1_SHIFT);
}

void bootboth(void) {

    SYSCFG_UR(1) = (SYSCFG_UR(1) & ~(SYSCFG_UR1_BCM4_MASK << SYSCFG_UR1_BCM4_SHIFT)) | (0x1 << SYSCFG_UR1_BCM4_SHIFT);
    SYSCFG_UR(1) = (SYSCFG_UR(1) & ~(SYSCFG_UR1_BCM7_MASK << SYSCFG_UR1_BCM7_SHIFT)) | (0x1 << SYSCFG_UR1_BCM7_SHIFT);
}

void forcem4boot(void)
{

//  RCC_GCR = (RCC_GCR_BOOT_C2_MASK << RCC_GCR_BOOT_C2_SHIFT);
  RCC_GCR = (RCC_GCR & ~(RCC_GCR_BOOT_C2_MASK << RCC_GCR_BOOT_C2_SHIFT)) | (0x1 << RCC_GCR_BOOT_C2_SHIFT);
}

uint16_t getcm4bootadd0(void) {
uint16_t read;

read = ((SYSCFG_UR(3) >> SYSCFG_UR3_BCM4_ADD0_SHIFT) & SYSCFG_UR3_BCM4_ADD0_MASK);
return read;
}
