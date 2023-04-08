
#include <libopencm3/stm32/dma.h>

void dma_stream_reset(uint32_t dma, uint8_t channel)
{
	/* Disable channel and reset config bits. */
	DMA_SxCR(dma, channel) = 0;
	/* Reset data transfer number. */
	DMA_SxNDTR(dma, channel) = 0;
	/* Reset peripheral address. */
	DMA_SPAR(dma, channel) = 0;
	/* Reset memory address. */
	DMA_SM0AR(dma, channel) = 0;
	/* Reset interrupt flags. */
//	DMA_IFCR(dma) |= DMA_IFCR_CIF(channel);
}

void dma_enable_transfer_complete_interrupt(uint32_t dma, uint8_t channel)
{
	DMA_SxCR(dma, channel) |= DMA_SxCR_TCIE;
}

void dma_disable_transfer_complete_interrupt(uint32_t dma, uint8_t channel)
{
    DMA_SxCR(dma, channel) &= ~DMA_SxCR_TCIE;
}

void dma_disable_mburst(uint32_t dma, uint8_t channel)
{
    DMA_SxCR(dma, channel) &= ~DMA_SxCR_MBURST_MASK;
}

void dma_enable_mburst(uint32_t dma, uint8_t channel)
{
    DMA_SxCR(dma, channel) |= DMA_SxCR_MBURST_MASK;
}

void dma_disable_direct_mode(uint32_t dma, uint8_t channel)
{
    DMA_SxCR(dma, channel) |= DMA_SxFCR_DMDIS;
}

void dma_enable_direct_mode(uint32_t dma, uint8_t channel)
{
    DMA_SxCR(dma, channel) &= ~DMA_SxFCR_DMDIS;
}

void dma_set_as_flow_controller(uint32_t dma, uint8_t channel)
{
    DMA_SxCR(dma, channel)  &= ~DMA_SxCR_PFCTRL;
}

void peripheral_set_as_flow_controller(uint32_t dma, uint8_t channel)
{
    DMA_SxCR(dma, channel)  |= DMA_SxCR_PFCTRL;
}

void dma_disable_pburst(uint32_t dma, uint8_t channel)
{
    DMA_SxCR(dma, channel) &= ~DMA_SxCR_PBURST_MASK;
}

void dma_enable_pburst(uint32_t dma, uint8_t channel)
{
    DMA_SxCR(dma, channel) &= ~DMA_SxCR_PBURST_MASK;
}

void dma_clear_interrupt_flags(uint32_t dma, uint8_t channel)
{
(void)channel;
	DMA_LIFCR(dma) |= DMA_LIFCR5_TCIE;
	DMA_LIFCR(dma) |= DMA_LIFCR11_TCIE;
	DMA_LIFCR(dma) |= DMA_LIFCR21_TCIE;
	DMA_LIFCR(dma) |= DMA_LIFCR27_TCIE;
	DMA_LIFCR(dma) |= DMA_LIFCRC3_TEIF;
	DMA_LIFCR(dma) |= DMA_LIFCRC9_TEIF;
	DMA_LIFCR(dma) |= DMA_LIFCRC19_TEIF;
	DMA_LIFCR(dma) |= DMA_LIFCRC25_TEIF;
	DMA_LIFCR(dma) |= DMA_LIFCRC0_CFEIF;
	DMA_LIFCR(dma) |= DMA_LIFCRC6_CFEIF;
	DMA_LIFCR(dma) |= DMA_LIFCRC16_CFEIF;
	DMA_LIFCR(dma) |= DMA_LIFCRC22_CFEIF;
	DMA_HIFCR(dma) |= DMA_HIFCR5_TCIE;
	DMA_HIFCR(dma) |= DMA_HIFCR11_TCIE;
	DMA_HIFCR(dma) |= DMA_HIFCR21_TCIE;
	DMA_HIFCR(dma) |= DMA_HIFCR27_TCIE;
	DMA_HIFCR(dma) |= DMA_HIFCRC3_TEIF;
	DMA_HIFCR(dma) |= DMA_HIFCRC9_TEIF;
	DMA_HIFCR(dma) |= DMA_HIFCRC19_TEIF;
	DMA_HIFCR(dma) |= DMA_HIFCRC25_TEIF;
	DMA_HIFCR(dma) |= DMA_HIFCRC0_CFEIF;
	DMA_HIFCR(dma) |= DMA_HIFCRC6_CFEIF;
	DMA_HIFCR(dma) |= DMA_HIFCRC16_CFEIF;
	DMA_HIFCR(dma) |= DMA_HIFCRC22_CFEIF;
}

void dma_set_number_of_data(uint32_t dma, uint8_t channel, uint32_t number)
{
	DMA_SxNDTR(dma, channel) = number;
}

void dma_enable_bufferable_transfers(uint32_t dma, uint8_t channel)
{
	DMA_SxCR(dma, channel) |=  DMA_SxCR_TRBUFF;
}

void dma_enable_stream(uint32_t dma, uint8_t channel)
{
	DMA_SxCR(dma, channel) |= DMA_SxCR_EN;
}

uint16_t dma_get_number_of_data(uint32_t dma, uint8_t channel)
{
	return DMA_SxNDTR(dma, channel);
}

void dma_disable_stream(uint32_t dma, uint8_t channel)
{
	DMA_SxCR(dma, channel) &= ~DMA_SxCR_EN;
}

void dma_set_memory_address(uint32_t dma, uint8_t channel, uint32_t address)
{
	if (!(DMA_SxCR(dma, channel) & DMA_SxCR_EN)) {
		DMA_SM0AR(dma, channel) = (uint32_t) address;
	}
}


void dma_set_peripheral_address(uint32_t dma, uint8_t channel, uint32_t address)
{
	if (!(DMA_SxCR(dma, channel) & DMA_SxCR_EN)) {
		DMA_SPAR(dma, channel) = (uint32_t) address;
	}
}

void dma_set_priority(uint32_t dma, uint8_t channel, uint32_t prio)
{
	DMA_SxCR(dma, channel) &= ~(DMA_SxCR_PL_MASK);
	DMA_SxCR(dma, channel) |= prio;
}

void dma_set_fifo_threshold(uint32_t dma, uint8_t channel, uint8_t thresh)
{
	DMA_SxFCR(dma, channel) |= thresh;
}

void dma_enable_memory_increment_mode(uint32_t dma, uint8_t channel)
{
	DMA_SxCR(dma, channel) |= DMA_SxCR_MINC;
}

void dma_disable_memory_increment_mode(uint32_t dma, uint8_t channel)
{
	DMA_SxCR(dma, channel) |= DMA_SxCR_MINC;
}

void dma_disable_peripheral_increment_mode(uint32_t dma, uint8_t channel)
{
	DMA_SxCR(dma, channel) &= ~DMA_SxCR_PINC;
}

void dma_enable_peripheral_increment_mode(uint32_t dma, uint8_t channel)
{
	DMA_SxCR(dma, channel) &= ~DMA_SxCR_PINC;
}

void dma_set_transfer_mode(uint32_t dma, uint8_t stream, uint32_t direction)
{
	uint32_t reg32 = (DMA_SxCR(dma, stream) & ~DMA_SxCR_DIR_MASK);
	/* Disable circular and double buffer modes if memory to memory
	 * transfers are in effect. (Direct Mode is automatically disabled by
	 * hardware)
	 */
	if (direction == DMA_SxCR_DIR_MEM_TO_MEM) {
		reg32 &= ~(DMA_SxCR_CIRC | DMA_SxCR_DBM);
	}

	DMA_SxCR(dma, stream) = (reg32 | direction);
}

void dma_set_memory_size(uint32_t dma, uint8_t channel, uint32_t mem_size)
{

	DMA_SxCR(dma, channel) &= ~(DMA_SxCR_MSIZE_MASK);
	DMA_SxCR(dma, channel) |= mem_size;
}

void dma_set_peripheral_size(uint32_t dma, uint8_t channel,
			     uint32_t peripheral_size)
{
	DMA_SxCR(dma, channel) &= ~(DMA_SxCR_PSIZE_MASK);
	DMA_SxCR(dma, channel) |= peripheral_size;
}

void dma_enable(int dma, int channel) {
DMA_SxCR(dma, channel) |= DMA_SxCR_EN;
}

void dma_disable(int dma, int channel) {
DMA_SxCR(dma, channel) |= (0 << DMA_SxCR_EN_OFFSET);
}

void dma_dblbuf_enable(int dma, int channel) {
DMA_SxCR(dma, channel) |= DMA_SxCR_DBM;
}

void dma_set_priority_level(int dma, int channel, int priority) {
DMA_SxCR(dma, channel) |= (priority << DMA_SxCR_PL_OFFSET);
}

void dma_set_msize(int dma, int channel, int msize) {
DMA_SxCR(dma, channel) |=  (msize << DMA_SxCR_MSIZE_OFFSET);
}

void dma_set_psize(int dma, int channel, int psize) {
DMA_SxCR(dma, channel) |=  (psize << DMA_SxCR_PSIZE_OFFSET);
}

void dma_minc_enable(int dma, int channel) {
DMA_SxCR(dma, channel) |= DMA_SxCR_MINC;
}

void dma_pinc_enable(int dma, int channel) {
DMA_SxCR(dma, channel) |= DMA_SxCR_PINC;
}

void dma_pinc_disable(int dma, int channel) {
DMA_SxCR(dma, channel) &= ~DMA_SxCR_PINC_OFFSET;
}

void dma_circ_disable(int dma, int channel) {
DMA_SxCR(dma, channel) &= ~DMA_SxCR_CIRC_OFFSET;
}

void dma_circ_enable(int dma, int channel) {
DMA_SxCR(dma, channel) |= DMA_SxCR_CIRC;
}

// sourrce = sxpar   dest = moar
void dma_set_dir_memory_to_memory(int dma, int channel) {
DMA_SxCR(dma, channel) |= DMA_SxCR_DIR_MEM_TO_MEM;
}

// ends up sett periph to mem
void dma_clear_dir_memory_to_memory(int dma, int channel) {
DMA_SxCR(dma, channel) &= ~DMA_SxCR_DIR_OFFSET;
}


// sourrce = moar  dest = sxpar
void dma_set_dir_memory_to_peripheral(int dma, int channel) {
DMA_SxCR(dma, channel) |= DMA_SxCR_DIR_MEM_TO_PERIPHERAL;
}

// sourrce = sxpar   dest = moar
void dma_set_dir_peripheral_to_memory(int dma, int channel) {
DMA_SxCR(dma, channel) |= DMA_SxCR_DIR_PERIPHERAL_TO_MEM;
}

void dma_tx_error_interrupt_enable(int dma, int channel) {
DMA_SxCR(dma, channel) |= DMA_SxCR_TEIE;
}

void dma_tx_error_interrupt_disable(int dma, int channel) {
DMA_SxCR(dma, channel) |= (0 << DMA_SxCR_TEIE_OFFSET);
}

void dma_half_transfer_interrupt_enable(int dma, int channel) {
DMA_SxCR(dma, channel) |= DMA_SxCR_HTIE;
}

void dma_half_transfer_interrupt_disable(int dma, int channel) {
DMA_SxCR(dma, channel) |= (0 << DMA_SxCR_HTIE_OFFSET);
}

void dma_transfer_complete_interrupt_enable(int dma, int channel) {
DMA_SxCR(dma, channel) |= DMA_SxCR_TCIE;
}

void dma_transfer_complete_interrupt_disable(int dma, int channel) {
DMA_SxCR(dma, channel) |= (0 << DMA_SxCR_TCIE_OFFSET);
}

void dma_set_sndtr(int dma, int channel, uint16_t num) {
DMA_SxCR(dma, channel) |= num << DMA_SxNDTR_NDT_OFFSET;
}

void dma_set_sparx(int dma, int channel, uint32_t addr) {
DMA_SPAR(dma, channel) = addr;
}

void dma_set_sm0ar(int dma, int channel, uint32_t addr) {
DMA_SM0AR(dma, channel) = addr;
}

void dma_set_sm1ar(int dma, int channel, uint16_t addr) {
DMA_SM1AR(dma, channel) = addr;
}

