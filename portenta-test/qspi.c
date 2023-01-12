#include "qspi.h"
#include <stdio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/quadspi.h>
#include <libopencm3/cm3/scb.h>

void qspi_wen( uint8_t dw ) {
  // Disable the peripheral.
  QUADSPI_CR   &= ~( QUADSPI_CR_EN );
  // Clear the instruction, mode, and transaction phases.
  QUADSPI_CCR  &= ~( QUADSPI_CCR_INST_MASK | QUADSPI_CCR_FMODE_MASK |
                      QUADSPI_CCR_IMODE_MASK | QUADSPI_CCR_DMODE_MASK |
                      QUADSPI_CCR_ABMODE_MASK );
  // Set N-wire instruction mode.
  QUADSPI_CCR  |=  ( dw << QUADSPI_CCR_IMODE_SHIFT );
  // Enable the peripheral and send the 'write enable' command.
  QUADSPI_CR   |=  ( QUADSPI_CR_EN );
  QUADSPI_CCR  |=  ( 0x06 << QUADSPI_CCR_INST_SHIFT );
  // Wait for the transaction to finish.
//  while ( QUADSPI_SR & QUADSPI_SR_BUSY ) {};
  // Disable the peripheral.
  QUADSPI_CR   &= ~( QUADSPI_CR_EN );
  // Wait until 'writes enabled' is set in the config register.
  qspi_reg_wait( 0x05, 0x03, 0x02, dw );
}

void qspi_reg_wait( uint8_t reg, uint32_t msk,
                    uint32_t mat, uint8_t dw ) {
  // Set the 'mask', 'match', and 'polling interval' values.
  QUADSPI_PSMKR = msk;
  QUADSPI_PSMAR = mat;
  QUADSPI_PIR   = 0x100;
  // Set the 'auto-stop' bit to end the transaction after a match.
  QUADSPI_CR   |=  ( QUADSPI_CR_APMS );
  // Clear instruction, mode and transaction phases.
  QUADSPI_CCR  &= ~( QUADSPI_CCR_INST_MASK | QUADSPI_CCR_FMODE_MASK |
                      QUADSPI_CCR_IMODE_MASK | QUADSPI_CCR_DMODE_MASK |
                      QUADSPI_CCR_ADMODE_MASK );
  // Set N-wire instruction and data modes, and auto-polling mode.
  QUADSPI_CCR  |=  ( ( dw << QUADSPI_CCR_IMODE_SHIFT ) |
                      ( dw << QUADSPI_CCR_DMODE_SHIFT ) |
                      ( 2 << QUADSPI_CCR_FMODE_SHIFT ) );
  // Enable the peripheral.
  QUADSPI_CR   |=  ( QUADSPI_CR_EN );
  // Set the given 'read register' instruction to start polling.
  QUADSPI_CCR  |=  ( reg << QUADSPI_CCR_INST_SHIFT );
  // Wait for a match.
//  while ( QUADSPI_SR & QUADSPI_SR_BUSY ) {};
  // Acknowledge the 'status match flag.'
  QUADSPI_FCR |=  ( QUADSPI_FCR_CSMF );
  // Disable the peripheral.
  QUADSPI_CR   &= ~( QUADSPI_CR_EN );
}

void qspi_erase_sector( uint32_t snum ) {
  // Send 'enable writes' command.
  qspi_wen( 1 );
  // Erase the sector, and wait for the operation to complete.
  while ( QUADSPI_SR & QUADSPI_SR_BUSY ) {};
  QUADSPI_CCR  &= ~( QUADSPI_CCR_INST_MASK | QUADSPI_CCR_FMODE_MASK |
                      QUADSPI_CCR_IMODE_MASK | QUADSPI_CCR_DMODE_MASK |
                      QUADSPI_CCR_ADMODE_MASK );
  QUADSPI_CCR |=  ( ( 1 << QUADSPI_CCR_IMODE_SHIFT ) |
                     ( 1 << QUADSPI_CCR_ADMODE_SHIFT ) );
  QUADSPI_CR  |=  ( QUADSPI_CR_EN );
  // 0x20 is the "sector erase" command.
  QUADSPI_CCR |=  ( 0x20 << QUADSPI_CCR_INST_SHIFT );
  // The address is equal to the sector number * 4KB.
  QUADSPI_AR   =  ( snum * 0x1000 );
  while ( QUADSPI_SR & QUADSPI_SR_BUSY ) {};
  // Disable the peripheral once the transaction is complete.
  QUADSPI_CR  &= ~( QUADSPI_CR_EN );
  // Wait for the 'write in progress' bit to clear.
  qspi_reg_wait( 0x05, 0x01, 0x00, 1 );
}

void qspi_write_word( uint32_t addr, uint32_t data ) {
  // Send 'enable writes' command.
  qspi_wen( 1 );
  // Write the word of data.
  while ( QUADSPI_SR & QUADSPI_SR_BUSY ) {};
  QUADSPI_CCR  &= ~( QUADSPI_CCR_INST_MASK |
                      QUADSPI_CCR_FMODE_MASK |
                      QUADSPI_CCR_IMODE_MASK |
                      QUADSPI_CCR_DMODE_MASK |
                      QUADSPI_CCR_ADMODE_MASK );
  QUADSPI_CCR |=  ( ( 1 << QUADSPI_CCR_IMODE_SHIFT ) |
                     ( 1 << QUADSPI_CCR_ADMODE_SHIFT ) |
                     ( 3 << QUADSPI_CCR_DMODE_SHIFT ) );
  // Set data length (3 + 1 = 4 bytes).
  QUADSPI_DLR = 3;
  // Enable the peripheral and set instruction, address, data.
  QUADSPI_CR  |=  ( QUADSPI_CR_EN );
  QUADSPI_CCR |=  ( 0x32 << QUADSPI_CCR_INST_SHIFT );
  QUADSPI_AR   =  ( addr );
  QUADSPI_DR   =  ( data );
  // Wait for the transaction to complete, and disable the peripheral.
//  while ( QUADSPI_SR & QUADSPI_SR_BUSY ) {};
  QUADSPI_CR  &= ~( QUADSPI_CR_EN );
  // Clear the data length register.
  QUADSPI_DLR = 0;
  // Wait for the 'write in progress' bit to clear.
  qspi_reg_wait( 0x05, 0x01, 0x00, 1 );
}
