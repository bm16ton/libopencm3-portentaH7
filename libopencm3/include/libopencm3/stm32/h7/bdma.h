



#define BDMA_CCR(channel)	MMIO32((BDMA_BASE) + 0x08 + \
					       (0x14 * ((channel))))


/* --- BDMA_CCRx values ----------------------------------------------------- */
/** @defgroup bdma_ccrx_values BDMA_CCRx values
@{*/

/** CT: current target memory of DMA transfer in double-buffer mode */
#define BDMA_CCRx_CT			(1 << 16)

/** DBM: double-buffer mode */
#define BDMA_CCRx_DBM			(1 << 15)

/** MEM2MEM: memory-to-memory mode */
#define BDMA_CCRx_MEM2MEM		(1 << 14)
#define BDMA_CCRx_MEM2MEM_OFFSET   14U
/** PL[1:0]: priority level */
#define BDMA_CCRx_PL_MASK		(0x3 << 12)
#define BDMA_CCRx_PL_OFFSET		12U
/** MSIZE[1:0]: memory size */
#define BDMA_CCRx_MSIZE_MASK		(0x3 << 10)
#define BDMA_CCRx_MSIZE_OFFSET		10U
/** PSIZE[1:0]: peripheral size */
#define BDMA_CCRx_PSIZE_MASK		(0x3 << 8)
#define BDMA_CCRx_PSIZE_OFFSET		8U
/** MINC: memory increment mode */
#define BDMA_CCRx_MINC			(1 << 7)

/** PINC: peripheral increment mode */
#define BDMA_CCRx_PINC			(1 << 6)

/** CIRC: circular mode */
#define BDMA_CCRx_CIRC			(1 << 5)
#define BDMA_CCRx_CIRC_OFFSET   5
/** DIR: data transfer direction */
#define BDMA_CCRx_DIR			(1 << 4)

/** TEIE: transfer error interrupt enable */
#define BDMA_CCRx_TEIE			(1 << 3)
#define BDMA_CCRx_TEIE_OFFSET   3U
/** HTIE: half transfer interrupt enable */
#define BDMA_CCRx_HTIE			(1 << 2)
#define BDMA_CCRx_HTIE_OFFSET			2U
/** TCIE: transfer complete interrupt enable */
#define BDMA_CCRx_TCIE			(1 << 1)
#define BDMA_CCRx_TCIE_OFFSET   1U
/** EN: channel enable */
#define BDMA_CCRx_EN			(1 << 0)
#define BDMA_CCRx_EN_OFFSET	 0x0
/**@}*/


/*
This field is updated by hardware when the channel is enabled:
–
It is decremented after each single BDMA ‘read followed by write’ transfer, indicating
the remaining amount of data items to transfer.
–
It is kept at zero when the programmed amount of data to transfer is reached, if the
channel is not in circular mode (CIRC = 0 in the BDMA_CCRx register).
–
It is reloaded automatically by the previously programmed value, when the transfer
is complete, if the channel is in circular mode (CIRC = 1).
If this field is zero, no transfer can be served whatever the channel status (enabled or not).
Note: this field is set and cleared by software.
It must not be written when the channel is enabled (EN = 1).
It is read-only when the channel is enabled (EN = 1).
*/
/* --- BDMA_CNDTRx values ----------------------------------------------------- */
/** @defgroup bdma_cndtrx_values BDMA_CNDTRx values
@{*/

#define BDMA_CNDTRx_NDT(channel)	MMIO32((BDMA_BASE) + 0x0C + \
					       (0x14 * ((channel))))
/** NDT[15:0]: number of data to transfer (0 to 2 16 - 1) */
#define BDMA_CNDTRx_NDT_MASK		(0xFFFF << 0)
#define BDMA_CNDTRx_NDT_OFFSET      0x0
/**@}*/

/*peripheral address
It contains the base address of the peripheral data register from/to which the data is
read/written.
When PSIZE[1:0] = 01 (16 bits), bit 0 of PA[31:0] is ignored. Access is automatically aligned
to a half-word address.
When PSIZE = 10 (32 bits), bits 1 and 0 of PA[31:0] are ignored. Access is automatically
aligned to a word address.
In memory-to-memory mode, this register identifies the memory destination address if
DIR = 1 and the memory source address if DIR = 0.
In peripheral-to-peripheral mode, this register identifies the peripheral destination address
DIR = 1 and the peripheral source address if DIR = 0.
Note: this register is set and cleared by software.
It must not be written when the channel is enabled (EN = 1).
It is not read-only when the channel is enabled (EN = 1).
*/
#define BDMA_CPARx(channel)  MMIO32((BDMA_BASE) + 0x10 + \
					       (0x14 * ((channel))))
/* --- BDMA_CPARx values ----------------------------------------------------- */
/** @defgroup bdma_cparx_values BDMA_CPARx values
@{*/

/** PA[31:0]: peripheral address */
#define BDMA_CPARx_MASK		(0xFFFFFFFF << 0)
#define BDMA_CPARx_OFFSET   0x0
/**@}*/


#define BDMA_CM0ARx(channel) MMIO32((BDMA_BASE) + 0x14 + \
					       (0x14 * ((channel))))
/* --- BDMA_CM0ARx values ----------------------------------------------------- */
/** @defgroup bdma_cm0arx_values BDMA_CM0ARx values
@{*/

/** MA[31:0]: peripheral address */
#define BDMA_CM0ARx_MASK		(0xFFFFFFFF << 0)
#define BDMA_CM0ARx_OFFSET  0
/**@}*/

/*
peripheral address
It contains the base address of the memory from/to which the data is read/written.
When MSIZE[1:0] = 01 (16 bits), bit 0 of MA[31:0] is ignored. Access is automatically aligned
to a half-word address.
When MSIZE = 10 (32 bits), bits 1 and 0 of MA[31:0] are ignored. Access is automatically
aligned to a word address.
In memory-to-memory mode, this register identifies the memory source address if DIR = 1
and the memory destination address if DIR = 0.
In peripheral-to-peripheral mode, this register identifies the peripheral source address
DIR = 1 and the peripheral destination address if DIR = 0.
Note: this register is set and cleared by software.
It must not be written when the channel is enabled (EN = 1).
It is not read-only when the channel is enabled (EN = 1).*/
#define BDMA_CM1ARx(channel)  MMIO32((BDMA_BASE) + 0x18 + \
					       (0x14 * ((channel))))
/* --- BDMA_CM1ARx values ----------------------------------------------------- */
/** @defgroup bdma_cm1arx_values BDMA_CM1ARx values
@{*/

/** MA[31:0]: peripheral address */
#define BDMA_CM1ARx_MASK		(0xFFFFFFFF << 0)
#define BDMA_CM1ARx_OFFSET  0
/**@}*/

#define BCHANNEL1 1
#define BCHANNEL2 2
#define BCHANNEL3 3
#define BCHANNEL4 4
#define BCHANNEL5 5
#define BCHANNEL6 6
#define BCHANNEL7 7

void bdma_enable(int channel);
void bdma_disable(int channel);
void bdma_dblbuf_enable(int channel);
void bdma_mem2mem_enable(int channel);
void bdma_mem2mem_disable(int channel);
void bdma_set_priority_level(int channel, int priority);
void bdma_set_msize(int channel, int msize);
void bdma_set_psize(int channel, int psize);
void bdma_minc_enable(int channel);
void bdma_pinc_enable(int channel);
void bdma_circ_enable(int channel);
void bdma_circ_disable(int channel);
void bdma_set_dir_read_from_memory(int channel);
void bdma_set_dir_read_from_peripheral(int channel);
void bdma_tx_error_interrupt_enable(int channel);
void bdma_tx_error_interrupt_disable(int channel);
void bdma_half_transfer_interrupt_enable(int channel);
void bdma_half_transfer_interrupt_disable(int channel);
void bdma_transfer_complete_interrupt_enable(int channel);
void bdma_transfer_complete_interrupt_disable(int channel);
void bdma_set_cndtrx(int channel, uint16_t num);
void bdma_set_cparx(int channel, uint32_t addr);
void bdma_set_cm0ar(int channel, uint16_t addr);
void bdma_set_cm1ar(int channel, uint16_t addr);






