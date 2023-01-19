/** @defgroup sdcard_defines SDCARD Defines

@brief <b>Defined Constants and Types for the SDCARD</b>

@ingroup SDCARD_defines

@version 1.0.0

@date 19 April 2017

LGPL License Terms @ref lgpl_license
 */

/*
 * This file is part of the libopencm3 project.
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SDCARD_H
#define SDCARD_H

#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/sdio.h>

typedef enum
{
/** 
  * @brief  SDIO specific error defines  
  */   
  SD_CMD_CRC_FAIL                    = (1), /*!< Command response received (but CRC check failed) */
  SD_DATA_CRC_FAIL                   = (2), /*!< Data bock sent/received (CRC check Failed) */
  SD_CMD_RSP_TIMEOUT                 = (3), /*!< Command response timeout */
  SD_DATA_TIMEOUT                    = (4), /*!< Data time out */
  SD_TX_UNDERRUN                     = (5), /*!< Transmit FIFO under-run */
  SD_RX_OVERRUN                      = (6), /*!< Receive FIFO over-run */
  SD_START_BIT_ERR                   = (7), /*!< Start bit not detected on all data signals in widE bus mode */
  SD_CMD_OUT_OF_RANGE                = (8), /*!< CMD's argument was out of range.*/
  SD_ADDR_MISALIGNED                 = (9), /*!< Misaligned address */
  SD_BLOCK_LEN_ERR                   = (10), /*!< Transferred block length is not allowed for the card or the number of transferred bytes does not match the block length */
  SD_ERASE_SEQ_ERR                   = (11), /*!< An error in the sequence of erase command occurs.*/
  SD_BAD_ERASE_PARAM                 = (12), /*!< An Invalid selection for erase groups */
  SD_WRITE_PROT_VIOLATION            = (13), /*!< Attempt to program a write protect block */
  SD_LOCK_UNLOCK_FAILED              = (14), /*!< Sequence or password error has been detected in unlock command or if there was an attempt to access a locked card */
  SD_COM_CRC_FAILED                  = (15), /*!< CRC check of the previous command failed */
  SD_ILLEGAL_CMD                     = (16), /*!< Command is not legal for the card state */
  SD_CARD_ECC_FAILED                 = (17), /*!< Card internal ECC was applied but failed to correct the data */
  SD_CC_ERROR                        = (18), /*!< Internal card controller error */
  SD_GENERAL_UNKNOWN_ERROR           = (19), /*!< General or Unknown error */
  SD_STREAM_READ_UNDERRUN            = (20), /*!< The card could not sustain data transfer in stream read operation. */
  SD_STREAM_WRITE_OVERRUN            = (21), /*!< The card could not sustain data programming in stream mode */
  SD_CID_CSD_OVERWRITE               = (22), /*!< CID/CSD overwrite error */
  SD_WP_ERASE_SKIP                   = (23), /*!< only partial address space was erased */
  SD_CARD_ECC_DISABLED               = (24), /*!< Command has been executed without using internal ECC */
  SD_ERASE_RESET                     = (25), /*!< Erase sequence was cleared before executing because an out of erase sequence command was received */
  SD_AKE_SEQ_ERROR                   = (26), /*!< Error in sequence of authentication. */
  SD_INVALID_VOLTRANGE               = (27),
  SD_ADDR_OUT_OF_RANGE               = (28),
  SD_SWITCH_ERROR                    = (29),
  SD_SDIO_DISABLED                   = (30),
  SD_SDIO_FUNCTION_BUSY              = (31),
  SD_SDIO_FUNCTION_FAILED            = (32),
  SD_SDIO_UNKNOWN_FUNCTION           = (33),

/** 
  * @brief  Standard error defines   
  */ 
  SD_INTERNAL_ERROR, 
  SD_NOT_CONFIGURED,
  SD_REQUEST_PENDING, 
  SD_REQUEST_NOT_APPLICABLE, 
  SD_INVALID_PARAMETER,  
  SD_UNSUPPORTED_FEATURE,  
  SD_UNSUPPORTED_HW,  
  SD_ERROR,  
  SD_OK = 0 
} SDCARD_Result;

/** 
 * @brief  SDIO Transfer state  
 */   
typedef enum
{
  SD_TRANSFER_OK  = 0,
  SD_TRANSFER_BUSY = 1,
  SD_TRANSFER_ERROR
} SDTransferState;

/** 
 * @brief  SD Card States 
 */   
typedef enum
{
  SD_CARD_READY                  = ((uint32_t)0x00000001),
  SD_CARD_IDENTIFICATION         = ((uint32_t)0x00000002),
  SD_CARD_STANDBY                = ((uint32_t)0x00000003),
  SD_CARD_TRANSFER               = ((uint32_t)0x00000004),
  SD_CARD_SENDING                = ((uint32_t)0x00000005),
  SD_CARD_RECEIVING              = ((uint32_t)0x00000006),
  SD_CARD_PROGRAMMING            = ((uint32_t)0x00000007),
  SD_CARD_DISCONNECTED           = ((uint32_t)0x00000008),
  SD_CARD_ERROR                  = ((uint32_t)0x000000FF)
}SDCardState;


/** 
 * @brief  Card Specific Data: CSD Register   
 */ 
typedef struct
{
  uint8_t  CSDStruct;            /*!< CSD structure */
  uint8_t  SysSpecVersion;       /*!< System specification version */
  uint8_t  Reserved1;            /*!< Reserved */
  uint8_t  TAAC;                 /*!< Data read access-time 1 */
  uint8_t  NSAC;                 /*!< Data read access-time 2 in CLK cycles */
  uint8_t  MaxBusClkFrec;        /*!< Max. bus clock frequency */
  uint16_t CardComdClasses;      /*!< Card command classes */
  uint8_t  RdBlockLen;           /*!< Max. read data block length */
  uint8_t  PartBlockRead;        /*!< Partial blocks for read allowed */
  uint8_t  WrBlockMisalign;      /*!< Write block misalignment */
  uint8_t  RdBlockMisalign;      /*!< Read block misalignment */
  uint8_t  DSRImpl;              /*!< DSR implemented */
  uint8_t  Reserved2;            /*!< Reserved */
  uint32_t DeviceSize;           /*!< Device Size */
  uint8_t  MaxRdCurrentVDDMin;   /*!< Max. read current @ VDD min */
  uint8_t  MaxRdCurrentVDDMax;   /*!< Max. read current @ VDD max */
  uint8_t  MaxWrCurrentVDDMin;   /*!< Max. write current @ VDD min */
  uint8_t  MaxWrCurrentVDDMax;   /*!< Max. write current @ VDD max */
  uint8_t  DeviceSizeMul;        /*!< Device size multiplier */
  uint8_t  EraseGrSize;          /*!< Erase group size */
  uint8_t  EraseGrMul;           /*!< Erase group size multiplier */
  uint8_t  WrProtectGrSize;      /*!< Write protect group size */
  uint8_t  WrProtectGrEnable;    /*!< Write protect group enable */
  uint8_t  ManDeflECC;           /*!< Manufacturer default ECC */
  uint8_t  WrSpeedFact;          /*!< Write speed factor */
  uint8_t  MaxWrBlockLen;        /*!< Max. write data block length */
  uint8_t  WriteBlockPaPartial;  /*!< Partial blocks for write allowed */
  uint8_t  Reserved3;            /*!< Reserded */
  uint8_t  ContentProtectAppli;  /*!< Content protection application */
  uint8_t  FileFormatGrouop;     /*!< File format group */
  uint8_t  CopyFlag;             /*!< Copy flag (OTP) */
  uint8_t  PermWrProtect;        /*!< Permanent write protection */
  uint8_t  TempWrProtect;        /*!< Temporary write protection */
  uint8_t  FileFormat;           /*!< File Format */
  uint8_t  ECC;                  /*!< ECC code */
  uint8_t  CSD_CRC;              /*!< CSD CRC */
  uint8_t  Reserved4;            /*!< always 1*/
} SD_CSD;

/** 
 * @brief  Card Identification Data: CID Register   
 */
typedef struct
{
   uint8_t  ManufacturerID;       /*!< ManufacturerID */
   uint16_t OEM_AppliID;          /*!< OEM/Application ID */
   uint32_t ProdName1;            /*!< Product Name part1 */
   uint8_t  ProdName2;            /*!< Product Name part2*/
   uint8_t  ProdRev;              /*!< Product Revision */
   uint32_t ProdSN;               /*!< Product Serial Number */
   uint8_t  Reserved1;            /*!< Reserved1 */
   uint16_t ManufactDate;         /*!< Manufacturing Date */
   uint8_t  CID_CRC;              /*!< CID CRC */
   uint8_t  Reserved2;            /*!< always 1 */
} SD_CID;

/** 
* @brief SD Card Status 
*/
typedef struct
{
   uint8_t DAT_BUS_WIDTH;
   uint8_t SECURED_MODE;
   uint16_t SD_CARD_TYPE;
   uint32_t SIZE_OF_PROTECTED_AREA;
   uint8_t SPEED_CLASS;
   uint8_t PERFORMANCE_MOVE;
   uint8_t AU_SIZE;
   uint16_t ERASE_SIZE;
   uint8_t ERASE_TIMEOUT;
   uint8_t ERASE_OFFSET;
} SD_CardStatus;

/** 
* @brief SD Card information 
*/
typedef struct
{
  SD_CSD SD_csd;
  SD_CID SD_cid;
  uint64_t CardCapacity;  /*!< Card Capacity */
  uint32_t CardBlockSize; /*!< Card Block Size */
  uint16_t RCA;
  uint8_t CardType;
} SD_CardInfo;

/** 
* @brief SDIO Commands  Index 
*/
#define SD_CMD_GO_IDLE_STATE                       ((uint8_t)0)
#define SD_CMD_SEND_OP_COND                        ((uint8_t)1)
#define SD_CMD_ALL_SEND_CID                        ((uint8_t)2)
#define SD_CMD_SET_REL_ADDR                        ((uint8_t)3) /*!< SDIO_SEND_REL_ADDR for SD Card */
#define SD_CMD_SET_DSR                             ((uint8_t)4)
#define SD_CMD_SDIO_SEN_OP_COND                    ((uint8_t)5)
#define SD_CMD_HS_SWITCH                           ((uint8_t)6)
#define SD_CMD_SEL_DESEL_CARD                      ((uint8_t)7)
#define SD_CMD_HS_SEND_EXT_CSD                     ((uint8_t)8)
#define SD_CMD_SEND_CSD                            ((uint8_t)9)
#define SD_CMD_SEND_CID                            ((uint8_t)10)
#define SD_CMD_READ_DAT_UNTIL_STOP                 ((uint8_t)11) /*!< SD Card doesn't support it */
#define SD_CMD_STOP_TRANSMISSION                   ((uint8_t)12)
#define SD_CMD_SEND_STATUS                         ((uint8_t)13)
#define SD_CMD_HS_BUSTEST_READ                     ((uint8_t)14)
#define SD_CMD_GO_INACTIVE_STATE                   ((uint8_t)15)
#define SD_CMD_SET_BLOCKLEN                        ((uint8_t)16)
#define SD_CMD_READ_SINGLE_BLOCK                   ((uint8_t)17)
#define SD_CMD_READ_MULT_BLOCK                     ((uint8_t)18)
#define SD_CMD_HS_BUSTEST_WRITE                    ((uint8_t)19)
#define SD_CMD_WRITE_DAT_UNTIL_STOP                ((uint8_t)20) /*!< SD Card doesn't support it */
#define SD_CMD_SET_BLOCK_COUNT                     ((uint8_t)23) /*!< SD Card doesn't support it */
#define SD_CMD_WRITE_SINGLE_BLOCK                  ((uint8_t)24)
#define SD_CMD_WRITE_MULT_BLOCK                    ((uint8_t)25)
#define SD_CMD_PROG_CID                            ((uint8_t)26) /*!< reserved for manufacturers */
#define SD_CMD_PROG_CSD                            ((uint8_t)27)
#define SD_CMD_SET_WRITE_PROT                      ((uint8_t)28)
#define SD_CMD_CLR_WRITE_PROT                      ((uint8_t)29)
#define SD_CMD_SEND_WRITE_PROT                     ((uint8_t)30)
#define SD_CMD_SD_ERASE_GRP_START                  ((uint8_t)32) /*!< To set the address of the first write block to be erased. (For SD card only) */
#define SD_CMD_SD_ERASE_GRP_END                    ((uint8_t)33) /*!< To set the address of the last write block of the continuous range to be erased. (For SD card only) */
#define SD_CMD_ERASE_GRP_START                     ((uint8_t)35) /*!< To set the address of the first write block to be erased. (For MMC card only spec 3.31) */
#define SD_CMD_ERASE_GRP_END                       ((uint8_t)36) /*!< To set the address of the last write block of the continuous range to be erased. (For MMC card only spec 3.31) */
#define SD_CMD_ERASE                               ((uint8_t)38)
#define SD_CMD_FAST_IO                             ((uint8_t)39) /*!< SD Card doesn't support it */
#define SD_CMD_GO_IRQ_STATE                        ((uint8_t)40) /*!< SD Card doesn't support it */
#define SD_CMD_LOCK_UNLOCK                         ((uint8_t)42)
#define SD_CMD_APP_CMD                             ((uint8_t)55)
#define SD_CMD_GEN_CMD                             ((uint8_t)56)
#define SD_CMD_NO_CMD                              ((uint8_t)64)

/** 
* @brief Following commands are SD Card Specific commands.
*        SDIO_APP_CMD should be sent before sending these commands. 
*/
#define SD_CMD_APP_SD_SET_BUSWIDTH                 ((uint8_t)6)  /*!< For SD Card only */
#define SD_CMD_SD_APP_STAUS                        ((uint8_t)13) /*!< For SD Card only */
#define SD_CMD_SD_APP_SEND_NUM_WRITE_BLOCKS        ((uint8_t)22) /*!< For SD Card only */
#define SD_CMD_SD_APP_OP_COND                      ((uint8_t)41) /*!< For SD Card only */
#define SD_CMD_SD_APP_SET_CLR_CARD_DETECT          ((uint8_t)42) /*!< For SD Card only */
#define SD_CMD_SD_APP_SEND_SCR                     ((uint8_t)51) /*!< For SD Card only */
#define SD_CMD_SDIO_RW_DIRECT                      ((uint8_t)52) /*!< For SD I/O Card only */
#define SD_CMD_SDIO_RW_EXTENDED                    ((uint8_t)53) /*!< For SD I/O Card only */

/** 
* @brief Following commands are SD Card Specific security commands.
*        SDIO_APP_CMD should be sent before sending these commands. 
*/
#define SD_CMD_SD_APP_GET_MKB                      ((uint8_t)43) /*!< For SD Card only */
#define SD_CMD_SD_APP_GET_MID                      ((uint8_t)44) /*!< For SD Card only */
#define SD_CMD_SD_APP_SET_CER_RN1                  ((uint8_t)45) /*!< For SD Card only */
#define SD_CMD_SD_APP_GET_CER_RN2                  ((uint8_t)46) /*!< For SD Card only */
#define SD_CMD_SD_APP_SET_CER_RES2                 ((uint8_t)47) /*!< For SD Card only */
#define SD_CMD_SD_APP_GET_CER_RES1                 ((uint8_t)48) /*!< For SD Card only */
#define SD_CMD_SD_APP_SECURE_READ_MULTIPLE_BLOCK   ((uint8_t)18) /*!< For SD Card only */
#define SD_CMD_SD_APP_SECURE_WRITE_MULTIPLE_BLOCK  ((uint8_t)25) /*!< For SD Card only */
#define SD_CMD_SD_APP_SECURE_ERASE                 ((uint8_t)38) /*!< For SD Card only */
#define SD_CMD_SD_APP_CHANGE_SECURE_AREA           ((uint8_t)49) /*!< For SD Card only */
#define SD_CMD_SD_APP_SECURE_WRITE_MKB             ((uint8_t)48) /*!< For SD Card only */

/**
* @brief  SD detection on its memory slot
*/
#define SD_PRESENT                                 ((uint8_t)0x01)
#define SD_NOT_PRESENT                             ((uint8_t)0x00)

/** 
* @brief Supported SD Memory Cards 
*/
#define SDIO_STD_CAPACITY_SD_CARD_V1_1             ((uint32_t)0x00000000)
#define SDIO_STD_CAPACITY_SD_CARD_V2_0             ((uint32_t)0x00000001)
#define SDIO_HIGH_CAPACITY_SD_CARD                 ((uint32_t)0x00000002)
#define SDIO_MULTIMEDIA_CARD                       ((uint32_t)0x00000003)
#define SDIO_SECURE_DIGITAL_IO_CARD                ((uint32_t)0x00000004)
#define SDIO_HIGH_SPEED_MULTIMEDIA_CARD            ((uint32_t)0x00000005)
#define SDIO_SECURE_DIGITAL_IO_COMBO_CARD          ((uint32_t)0x00000006)
#define SDIO_HIGH_CAPACITY_MMC_CARD                ((uint32_t)0x00000007)

#define SDCARD_INIT_FREQ                           (uint32_t)400000
#define GET_INIT_DIVIDER(input_freq)               (uint8_t)((input_freq / SDCARD_INIT_FREQ) - 2)
#define GET_TRANSFER_DIVIDER(input_freq, freq)     (uint8_t)((input_freq / freq) - 2)

BEGIN_DECLS

SDCARD_Result sdcard_init(uint32_t apb_freq, uint32_t sdcard_freq);
SDCARD_Result sdcard_power_on(void);
SDCARD_Result sdcard_power_off(void);
SDCARD_Result sdcard_cards_init(void);
SDCARD_Result sdcard_get_card_info(SD_CardInfo *cardinfo);
SDCARD_Result sdcard_select_deselect(uint64_t addr);
SDCARD_Result sdcard_enable_wide_bus_operation(SDIO_WideBusMode mode);
SDCARD_Result sdcard_read_block(uint8_t *readbuff, uint64_t ReadAddr, uint16_t BlockSize);
SDCARD_Result sdcard_write_block(uint8_t *writebuff, uint64_t WriteAddr, uint16_t BlockSize);
uint8_t sdcard_detect(void);
SDCARD_Result sdcard_get_card_status(SD_CardStatus *cardstatus);
SDCARD_Result sdcard_send_status(uint32_t *psdstatus);
SDCARD_Result sdcard_erase(uint64_t startaddr, uint64_t endaddr);
SDTransferState sdcard_get_status(void);
// SDCARD_Result sdcard_wait_read_operation(void);
// SDCARD_Result sdcard_wait_write_operation(void);
SDCardState sdcard_get_state(void);

//SDCARD_Result sdcard_read_multi_blocks(uint8_t *readbuff, uint64_t ReadAddr, uint16_t BlockSize, uint32_t NumberOfBlocks);
// SD_Error SD_WriteMultiBlocks(uint8_t *writebuff, uint64_t WriteAddr, uint16_t BlockSize, uint32_t NumberOfBlocks);
// SDTransferState SD_GetTransferState(void);
// SD_Error SD_StopTransfer(void);
// SD_Error SD_SendStatus(uint32_t *pcardstatus);
// SD_Error SD_ProcessIRQSrc(void);
// void SD_ProcessDMAIRQ(void);
// SD_Error SD_HighSpeed(void);

END_DECLS

#endif