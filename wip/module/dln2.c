// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for the Diolan DLN-2 USB adapter
 *
 * Copyright (c) 2014 Intel Corporation
 *
 * Derived from:
 *  i2c-diolan-u2c.c
 *  Copyright (c) 2010-2011 Ericsson AB
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/mfd/core.h>
#include <linux/mfd/dln2.h>
#include <linux/rculist.h>
#include <linux/spi/spi.h>

struct dln2_dev {
	struct usb_device *usb_dev;
	struct usb_interface *interface;
    struct platform_device		*spi_pdev;
};

enum dln2_handle {
	DLN2_HANDLE_SPI,
	DLN2_HANDLES
};

struct dev_io_desc_data {
	const char *con_id;
	unsigned int idx;
	unsigned int flags;
};

#define SPI_INTF_DEVNAME	"spi-tiny-usb"
#define FTDI_MPSSE_IO_DESC_MAGIC	0x5345494F
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
struct usbspi_spi_dev_data {
	u32 magic;
	struct dev_io_desc_data *desc;
	size_t desc_len;
};

struct usbspi_spi_platform_data {
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

static struct dev_io_desc_data tinyusb_spi_bus_dev_io[] = {
 //      { "ce", 1, GPIO_ACTIVE_HIGH },
//       { "csn", 2, GPIO_ACTIVE_LOW },
};

static const struct usbspi_spi_dev_data tinyusb_spi_dev_data[] = {
	{
	.magic		= FTDI_MPSSE_IO_DESC_MAGIC,
	.desc		= tinyusb_spi_bus_dev_io,
	.desc_len	= ARRAY_SIZE(tinyusb_spi_bus_dev_io),
	},
};


static struct spi_board_info tinyusb_spi_bus_info[] = {
    {
//    .modalias	= "yx240qv29",
//	.modalias	= "ili9341",
    .modalias	= "w25q32",
//	.modalias	= "mcp2515",
//    .modalias	= "spi-petra",
//    .modalias	= "nrf24",
//	.modalias	= "ili9341",
    .mode		= SPI_MODE_0,
//    .mode		= SPI_MODE_0 | SPI_LSB_FIRST | SPI_CS_HIGH,
//    .max_speed_hz	= 4000000,
    .max_speed_hz	= 30000000,
    .bus_num	= 0,
    .chip_select	= 0,
   .platform_data	= tinyusb_spi_dev_data,
// 	.swnode	= &nrf24_node,    //changed from properties to swnode i dunno aroun kernel 5.15ish
//    .properties	= mcp2515_properties,
//	.swnode  =  &mcp2515_node,
//	.irq     = 0,
    },
/*
   {
    .modalias	= "spi-petra",    //use instead of spidev for spidev no-longer enumerates
//    .modalias	= "w25q32",
//	  .modalias	= "spidev",
    .mode		= SPI_MODE_0,
//    .mode		= SPI_MODE_0 | SPI_LSB_FIRST | SPI_CS_HIGH,
    .max_speed_hz	= 30000000,
    .bus_num	= 0,
    .chip_select	= 5, // GPIOH0 at ACBUS0
    },
*/
};

static const struct usbspi_spi_platform_data tinyusb_spi_bus_plat_data = {
    .spi_info	= tinyusb_spi_bus_info,
    .spi_info_len	= ARRAY_SIZE(tinyusb_spi_bus_info),
};


/* Only one SPI port supported */
static struct dln2_platform_data dln2_pdata_spi = {
	.handle = DLN2_HANDLE_SPI,
	.port = 0,
};



static const struct mfd_cell dln2_devs[] = {
	{
		.name = "dln2-spi",
		.platform_data = &tinyusb_spi_bus_plat_data,
		.pdata_size = sizeof(tinyusb_spi_bus_plat_data),
	},
};

static int dln2_probe(struct usb_interface *interface,
		      const struct usb_device_id *usb_id)
{
	struct usb_host_interface *hostif = interface->cur_altsetting;
	struct usb_endpoint_descriptor *epin;
	struct usb_endpoint_descriptor *epout;
	struct device *dev = &interface->dev;
	struct device *parent = &interface->dev;
	struct platform_device *pdev;
	struct dln2_dev *dln2;
	const struct usbspi_spi_platform_data *pd;
	size_t lookup_size, tbl_size;
	int ret;
	int i, j;


	dln2 = kzalloc(sizeof(*dln2), GFP_KERNEL);
	if (!dln2)
		return -ENOMEM;


	dln2->usb_dev = usb_get_dev(interface_to_usbdev(interface));
	dln2->interface = interface;
	usb_set_intfdata(interface, dln2);

pdev = platform_device_alloc(SPI_INTF_DEVNAME, 0);
/*	ret = mfd_add_hotplug_devices(dev, dln2_devs, ARRAY_SIZE(dln2_devs));
	if (ret != 0) {
		dev_err(dev, "failed to add mfd devices to core\n");
		goto out_stop_rx;
	}
*/

	pdev->dev.parent = parent;
	pdev->dev.fwnode = NULL;
	dln2->spi_pdev = pdev;
	tbl_size = pd->spi_info_len + 1;


	for (i = 0; i < pd->spi_info_len; i++) {
		dev_dbg(parent, "INFO: %s cs %d\n",
			pd->spi_info[i].modalias);
	}


	ret = platform_device_add_data(pdev, pd, sizeof(*pd));
    if (ret) {
    ;
    }
    
    ret = platform_device_add(pdev);
	if (ret < 0)
		;
	return 0;

out_stop_rx:


out_free:


	return ret;
}

static void dln2_disconnect(struct usb_interface *interface)
{
;
}

#define VID 0x0403
#define PID 0xc633


static const struct usb_device_id dln2_table[] = {
	{USB_DEVICE(VID, PID)},
	{}
};



MODULE_DEVICE_TABLE(usb, dln2_table);

static struct usb_driver dln2_driver = {
	.name = "dln2",
	.probe = dln2_probe,
	.disconnect = dln2_disconnect,
	.id_table = dln2_table,
};

module_usb_driver(dln2_driver);

MODULE_AUTHOR("Octavian Purdila <octavian.purdila@intel.com>");
MODULE_DESCRIPTION("Core driver for the Diolan DLN2 interface adapter");
MODULE_LICENSE("GPL v2");
