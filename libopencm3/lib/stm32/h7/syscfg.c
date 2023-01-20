

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/stm32/syscfg.h>

void setcm4bootadd0(uint16_t bootadd) {
SYSCFG_UR(3) = (SYSCFG_UR(3) & ~(SYSCFG_UR3_BCM4_ADD0_MASK << SYSCFG_UR3_BCM4_ADD0_SHIFT)) | ((uint32_t)bootadd << SYSCFG_UR3_BCM4_ADD0_SHIFT);
}

void setcm4bootadd1(uint16_t bootadd) {
SYSCFG_UR(4) = (SYSCFG_UR(4) & ~(SYSCFG_UR4_BCM4_ADD1_MASK << SYSCFG_UR4_BCM4_ADD1_SHIFT)) | ((uint32_t)bootadd << SYSCFG_UR4_BCM4_ADD1_SHIFT);
}
