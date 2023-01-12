// =================================================================================================
#include <stddef.h>
#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

int AddCs(uint32_t cs, const SpiCsConfig &config, SpiSetCs cs_func) noexcept {
  RET_USR_IF(cs >= num_cs_, ErrorReturn::Invalid);

//  std::lock_guard<std::mutex> lock(mutex_);
  chip_selects_[cs].config   = config;
  chip_selects_[cs].set_func = cs_func;

  // Ensure the chip select is deasserted.
  chip_selects_[cs].set_func(false);

  // Calculatate the divisor value for this chip select to store in the CFG1 setting.
  uint32_t br          = 0;
  uint32_t base_clk_hz = rcc_get_peripheral_clk_freq(config_.base);
  while ((base_clk_hz >> (br + 1)) > config.clock_hz) {
    br++;
    RET_USR_IF(br > SPI_CFG1_MBR_MCLK_DIV_256, ErrorReturn::Invalid);
  }
  chip_selects_[cs].actual_clock_hz = base_clk_hz >> (br + 1);

  // Set CFG1 with calculated divider and the following defaults.
  // Dafaults:
  //   - DMA Disabled
  //   - CRC Disabled
  //   - FIFO is 4-bytes
  //   - Datasize 8-bits
  chip_selects_[cs].cfg1 =
      (br << SPI_CFG1_MBR_SHIFT) | SPI_CFG1_DSIZE(8) | (SPI_CFG1_FTHLV(4) << SPI_CFG1_FTHLV_SHIFT);
  packet_sz_ = 4U;  // For now, this is 4-bytes, and that's it!.

  // Set CFG2 based on user inputs and the following defaults.
  // Dafaults:
  //   - Master Mode, MSB first, Full-Duples
  //   - Software slave select control (use GPIO and user callback).
  //   - No inter-data delays.
  chip_selects_[cs].cfg2 = SPI_CFG2_MASTER | SPI_CFG2_SSOM | SPI_CFG2_SSM | SPI_CFG2_AFCNTR |
                           (SPI_CFG2_COMM_FULLDUPLEX << SPI_CFG2_COMM_SHIFT);
  if (config.mode == Mode::Mode2 || config.mode == Mode::Mode3) {
    chip_selects_[cs].cfg2 |= SPI_CFG2_CPOL;
  }
  if (config.mode == Mode::Mode1 || config.mode == Mode::Mode3) {
    chip_selects_[cs].cfg2 |= SPI_CFG2_CPHA;
  }

  chip_selects_[cs].assigned = true;
  return ErrorReturn::Success;
}

// =================================================================================================
int StartTransaction(void) {
  // Setup the in_process transaction to this one.
  in_process_.bytes_sent     = 0;
  in_process_.bytes_received = 0;

  SPI_CFG1(config_.base) = chip_selects_[in_process_.transaction.cs].cfg1;
  SPI_CFG2(config_.base) = chip_selects_[in_process_.transaction.cs].cfg2;
  SPI_CR2(config_.base) = in_process_.transaction.len;
  SPI2S_CR1(config_.base) |= SPI2S_CR1_SPE;
  SPI2S_IER(config_.base) |= SPI2S_IER_RXPIE | SPI2S_IER_EOTIE;

  // Set the chip select and start the transfer.
  chip_selects_[in_process_.transaction.cs].set_func(true);
  SPI2S_CR1(config_.base) |= SPI2S_CR1_CSTART;

  // Prime the jets, put some data in the TX FIFO.
  ProcessTxp();

  // Yes, we're polling completion of our interrupt for now until we get events working.
  while (in_process_.bytes_received < in_process_.transaction.len)
    ;

  // Disable the user's chip select.
  if (!in_process_.transaction.hold_cs) {
    chip_selects_[in_process_.transaction.cs].set_func(false);
  }

  return ErrorReturn::Success;
}

// =================================================================================================
void ProcessTxp(void) {
  while (BITMASK_IS_SET(SPI2S_SR(config_.base), SPI2S_SR_TXP) &&
         in_process_.bytes_sent < in_process_.transaction.len) {
    const uint8_t *tx_buf   = reinterpret_cast<const uint8_t *>(in_process_.transaction.tx_buf);
    const uint32_t *u32_buf = reinterpret_cast<const uint32_t *>(&tx_buf[in_process_.bytes_sent]);
    uint32_t bytes = std::min(in_process_.transaction.len - in_process_.bytes_sent, packet_sz_);

    size_t reg_writes = IDivCeil(bytes, sizeof(uint32_t));
    for (size_t i = 0; i < reg_writes; i++) {
      SPI2S_TXDR(config_.base) = (tx_buf != nullptr) ? u32_buf[i] : 0xffffffff;
    }
    in_process_.bytes_sent += bytes;
  }
}
