#ifndef __LINUX_SPI_TINY_USB_H
#define __LINUX_SPI_TINY_USB_H

#define USBSPI_IO_DESC_MAGIC	0x5345494F

struct usb_spi_platform_data {
	const struct usbspi_intf_ops *ops;
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

struct ctrl_desc {
	unsigned int dir_out;
	u8 request;
	u8 requesttype;
	u16 value;
	u16 index;
	u16 size;
	void *data;
	int timeout;
};

struct bulk_desc {
	unsigned int dir_out;
	void *data;
	int len;
	int act_len;
	int timeout;
};


struct usbspi_intf_ops {
	int (*bulk_xfer)(struct usb_interface *interface, struct bulk_desc *);
	int (*ctrl_xfer)(struct usb_interface *intf, struct ctrl_desc *desc);
	int (*read_data)(struct usb_interface *interface, void *buf, size_t len);
	int (*write_data)(struct usb_interface *interface, const char *buf, size_t len);
	void (*setup_data)(struct usb_interface *interface, int cmd, int value, int index);
	void (*lock)(struct usb_interface *intf);
	void (*unlock)(struct usb_interface *intf);
	void (*chcs)(struct usb_interface *interface, bool enable);
};

struct usbspi_dev_data {
	u32 magic;
	struct dev_io_desc_data *desc;
	size_t desc_len;
};

struct dev_io_desc_data {
	const char *con_id;
	unsigned int idx;
	unsigned int flags;
};




int spi_tiny_usb_xfer_one_two(struct spi_master *master, struct spi_message *m);
#endif
