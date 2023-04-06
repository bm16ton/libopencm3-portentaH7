#include <libopencm3/stm32/dmamux.h>
#include <libopencm3/stm32/memorymap.h>

void dmamux_set_dma_channel_request(uint32_t dmamux, uint8_t channel, uint8_t request_id)
{
	uint32_t reg32 = H7DMAMUX_CxCR(dmamux, channel);
	reg32 &= ~(H7DMAMUX1_CxCR_DMAREQ_ID_MASK << H7DMAMUX1_CxCR_DMAREQ_ID_OFFSET);
	reg32 |= ((request_id & H7DMAMUX1_CxCR_DMAREQ_ID_MASK) << H7DMAMUX1_CxCR_DMAREQ_ID_OFFSET);
	H7DMAMUX_CxCR(dmamux, channel) = reg32;
}

void dmamux_reset_dma_channel(uint32_t dmamux, uint8_t channel)
{
	H7DMAMUX_CxCR(dmamux, channel) = 0;
	dmamux_clear_dma_request_sync_overrun(dmamux, channel);
}


void dmamux_clear_dma_request_sync_overrun(uint32_t dmamux, uint8_t channel)
{
	H7DMAMUX_CFR(dmamux) = H7DMAMUX_CFR_CSOF(channel);
}

uint8_t dmamux_get_dma_channel_request(uint32_t dmamux, uint8_t channel)
{
	return (H7DMAMUX_CxCR(dmamux, channel) >> H7DMAMUX1_CxCR_DMAREQ_ID_OFFSET) & H7DMAMUX1_CxCR_DMAREQ_ID_MASK;
}

