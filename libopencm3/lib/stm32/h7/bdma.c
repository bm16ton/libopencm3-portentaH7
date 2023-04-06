
#include <libopencm3/stm32/bdma.h>
void bdma_enable(int channel) {
BDMA_CCR(channel) |= BDMA_CCRx_EN;
}

void bdma_disable(int channel) {
BDMA_CCR(channel) |= (0 << BDMA_CCRx_EN_OFFSET);
}

void bdma_dblbuf_enable(int channel) {
BDMA_CCR(channel) |= BDMA_CCRx_DBM;
}

void bdma_mem2mem_enable(int channel) {
BDMA_CCR(channel) |= BDMA_CCRx_MEM2MEM;
}

void bdma_mem2mem_disable(int channel) {
BDMA_CCR(channel) &= ~BDMA_CCRx_MEM2MEM_OFFSET;
}

void bdma_set_priority_level(int channel, int priority) {
BDMA_CCR(channel) |= (priority << BDMA_CCRx_PL_OFFSET);
}

void bdma_set_msize(int channel, int msize) {
BDMA_CCR(channel) |=  (msize << BDMA_CCRx_MSIZE_OFFSET);
}

void bdma_set_psize(int channel, int psize) {
BDMA_CCR(channel) |=  (psize << BDMA_CCRx_PSIZE_OFFSET);
}

void bdma_minc_enable(int channel) {
BDMA_CCR(channel) |= BDMA_CCRx_MINC;
}

void bdma_pinc_enable(int channel) {
BDMA_CCR(channel) |= BDMA_CCRx_PINC;
}

void bdma_circ_disable(int channel) {
BDMA_CCR(channel) &= ~BDMA_CCRx_CIRC_OFFSET;
}

void bdma_circ_enable(int channel) {
BDMA_CCR(channel) |= BDMA_CCRx_CIRC;
}

void bdma_set_dir_read_from_memory(int channel) {
BDMA_CCR(channel) |= BDMA_CCRx_DIR;
}

void bdma_set_dir_read_from_peripheral(int channel) {
BDMA_CCR(channel) &= ~BDMA_CCRx_DIR;
}

void bdma_tx_error_interrupt_enable(int channel) {
BDMA_CCR(channel) |= BDMA_CCRx_TEIE;
}

void bdma_tx_error_interrupt_disable(int channel) {
BDMA_CCR(channel) |= (0 << BDMA_CCRx_TEIE_OFFSET);
}

void bdma_half_transfer_interrupt_enable(int channel) {
BDMA_CCR(channel) |= BDMA_CCRx_HTIE;
}

void bdma_half_transfer_interrupt_disable(int channel) {
BDMA_CCR(channel) |= (0 << BDMA_CCRx_HTIE_OFFSET);
}

void bdma_transfer_complete_interrupt_enable(int channel) {
BDMA_CCR(channel) |= BDMA_CCRx_TCIE;
}

void bdma_transfer_complete_interrupt_disable(int channel) {
BDMA_CCR(channel) |= (0 << BDMA_CCRx_TCIE_OFFSET);
}

void bdma_set_cndtrx(int channel, uint16_t num) {
BDMA_CNDTRx_NDT(channel) = (num << BDMA_CNDTRx_NDT_OFFSET);
}

void bdma_set_cparx(int channel, uint32_t addr) {
BDMA_CPARx(channel) = (addr << BDMA_CPARx_OFFSET);
}

void bdma_set_cm0ar(int channel, uint16_t addr) {
BDMA_CM0ARx(channel) = (addr << BDMA_CM0ARx_OFFSET);
}

void bdma_set_cm1ar(int channel, uint16_t addr) {
BDMA_CM1ARx(channel) = (addr << BDMA_CM1ARx_OFFSET);
}

