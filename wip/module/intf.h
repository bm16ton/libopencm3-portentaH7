/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Common definitions for FTDI FT232H interface device
 *
 * Copyright (C) 2017 - 2018 DENX Software Engineering
 * Anatolij Gustschin <agust@denx.de>
 */

#ifndef __LINUX_INTF_H
#define __LINUX_INTF_H


/*
 * struct dev_io_desc_data - Descriptor of FT232H pin used by attached device
 * @con_id: Name of the GPIO pin
 * @idx: Index of the pin
 * @flags: GPIOD flags of the pin
 *
 * Description of a GPIO used by device connected to FT232H
 */
struct dev_io_desc_data {
	const char *con_id;
	unsigned int idx;
	unsigned int flags;
};


#define USBSPI_IO_DESC_MAGIC	0x5345494F
/*
 * struct mpsse_spi_dev_data - MPSSE SPI device platform data
 * @magic: Special # indicating that this is a I/O descriptor struct
 * @io_data: Array with descriptors of used I/O pins
 * @io_data_len: Length of io_data array
 *
 * MPSSE SPI slave specific platform data describing additional
 * I/O pins (if any) of attached SPI slave. It is supposed to be
 * passed via .platform_data of spi_board_info struct.
 * To differentiate between MPSSE I/O descriptor data and other
 * driver-specific platform data we use FTDI_MPSSE_IO_DESC_MAGIC
 * in the header of this struct
 */
struct tinyusb_spi_dev_data {
	u32 magic;
	struct dev_io_desc_data *desc;
	size_t desc_len;
};


/*
 * struct mpsse_spi_platform_data - MPSSE SPI bus platform data
 * @ops: USB interface operations used in MPSSE SPI controller driver
 * @spi_info: Array with spi_board_info structures of attached SPI slaves
 * @spi_info_len: Length of spi_info array
 * @io_data: Array with descriptors of used I/O pins
 * @io_data_len: Length of io_data array
 *
 * MPSSE SPI specific platform data describing attached SPI slaves and
 * their additional I/O pins
 */
struct tinyusb_spi_platform_data {
	const struct tinyusbh_intf_ops *ops;
	struct spi_board_info *spi_info;
	size_t spi_info_len;
	u32 magic;
	struct dev_io_desc_data *desc;
	size_t desc_len;
//	struct dev_info_desc_data *data;
//	size_t data_len;
//	struct dev_io_desc_data *io_data;
//	size_t io_data_len;
//	int dc;
};

#endif /* __LINUX_INTF_H */
