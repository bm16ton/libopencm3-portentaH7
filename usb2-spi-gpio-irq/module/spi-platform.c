// SPDX-License-Identifier: GPL-2.0
/*
 * FTDI FT232H interface driver for ARRI FPGA configuration
 *
 * Copyright (C) 2017 - 2018 DENX Software Engineering
 * Anatolij Gustschin <agust@denx.de>
 *
 *    Figure: FT232H, FPGA Devices and Drivers Relationship
 *
 *      +-------------+
 *      |             |
 *      |  STRATIX V  |PS-SPI         FT245 FIFO & GPIO
 *      |             +-----+    +-------------------+
 *      |  on Board 1 |     +    +                   |
 *      |             |                         +----+---+
 *      |  PCIe       |   ADBUS&ACBUS           |  CPLD  |
 *      +---+---------+ Connection Options      +----+---+
 *          ^          (MPSSE or FIFO&GPIO)          |
 *          +                  +              +------+-------+
 *     altera-cvp  +-----------+----------+   |     FPP      |
 *                 |        FT232H        |   |              |
 *                 |     0x0403:0x7148    |   |   ARRIA 10   |
 *                 |     0x0403:0x7149    |   |              |
 *                 +----------+-----------+   |  on Board 2  |
 *                            |               |              |
 *                +-----------+------------+  |        PCIe  |
 *        creates | htd-intf (USB misc) |  +----------+---+
 *       platform |     bulk/ctrl xfer     |             ^
 *        devices |ACBUS GPIO Ctrl (0x7148)|             |
 *         below  |MPSSE GPIO Ctrl (0x7149)|             |
 *                +-------+-------+--------+             |
 *                        |       |                      |
 *           for     +----+       +------+    for        |
 *        PID 0x7149 |                   | PID 0x7148    |
 *         +---------+--------+  +-------+---------+     |
 *         |  htd-stm32h7-spi  |  |                 |     |
 *         | altera-ps-spi in |  |htd-fifo-fpp-mgr|     |
 *         |   spi_board_info |  |                 |     |
 *         +---------+--------+  +--------+--------+     |
 *                   ^                    ^              |
 *        Drivers:   |                    |              |
 *                   +                    |              |
 *      MPSSE SPI master(spi-htd-stm32h7)  |              +
 *                   ^                    |              |
 *                   |                    +              +
 *             altera-ps-spi        htd-fifo-fpp    altera-cvp
 *              FPGA Manager         FPGA Manager   FPGA Manager
 *
 *
 * When using your custom USB product ID, this FT232H interface driver
 * also allows to register the GPIO controller for CBUS pins or for
 * MPSSE GPIO pins. Below are examples how to use the driver as CBUS-
 * or MPSSE-GPIO controller.
 *
 * For CBUS-GPIOs add new entry with your PID to htd_intf_table[]:
 * static const struct htd_intf_info htd_cbus_gpio_intf_info = {
 *	.use_cbus_gpio_ctrl = true,
 * };
 * { USB_DEVICE(FTDI_VID, PID),
 *   .driver_info = (kernel_ulong_t)&htd_cbus_gpio_intf_info },
 *
 * For MPSSE-GPIO add new entry with your PID to htd_intf_table[]:
 * static const struct htd_intf_info htd_stm32h7_gpio_intf_info = {
 *	.use_stm32h7_gpio_ctrl = true,
 * };
 * { USB_DEVICE(FTDI_VID, PID),
 *   .driver_info = (kernel_ulong_t)&htd_stm32h7_gpio_intf_info },
 *
 * With custom USB product IDs it is also possible to use FT232H SPI bus
 * with different SPI slave devices attached (e.g. SPI-NOR flash chips,
 * spidev, etc.). Example below shows how to add a bus with two SPI slave
 * devices for your USB PID:
 *
 * static struct spi_board_info htd_spi_bus_info[] = {
 *	{
 *	.modalias	= "w25q32",
 *	.mode		= SPI_MODE_0,
 *	.max_speed_hz	= 60000000,
 *	.bus_num	= 0,
 *	.chip_select	= 0, // TCK/SK at ADBUS0
 *	},
 *	{
 *	.modalias	= "spidev",
 *	.mode		= SPI_MODE_0 | SPI_LSB_FIRST | SPI_CS_HIGH,
 *	.max_speed_hz	= 30000000,
 *	.bus_num	= 0,
 *	.chip_select	= 5, // GPIOH0 at ACBUS0
 *	},
 * };
 *
 * static const struct stm32h7_spi_platform_data htd_spi_bus_plat_data = {
 *	.ops		= &htd_intf_ops,
 *	.spi_info	= htd_spi_bus_info,
 *	.spi_info_len	= ARRAY_SIZE(htd_spi_bus_info),
 * };
 *
 * static const struct htd_intf_info htd_spi_bus_intf_info = {
 *	.probe  = htd_intf_spi_probe,
 *	.remove  = htd_intf_spi_remove,
 *	.plat_data  = &htd_spi_bus_plat_data,
 * };
 * { USB_DEVICE(FTDI_VID, YOUR_PID),
 *	.driver_info = (kernel_ulong_t)&htd_spi_bus_intf_info },
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/printk.h>
#include <linux/gpio.h> //16ton
#include <linux/gpio/driver.h>
#include <linux/gpio/machine.h>
#include <linux/idr.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/usb/ch9.h>
#include <linux/usb.h>
#include <linux/usb/htd-intf.h>
//16ton
#include <linux/version.h>
#include <linux/property.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/kobject.h>
#include <linux/kdev_t.h>

#include <linux/fs.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/workqueue.h>

#define IRQPIN 3
#define POLL_PERIOD_MS 10;

int usb_wait_msec = 0;
module_param(usb_wait_msec, int, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(usb_wait_msec, "Wait after USB transfer in msec");


struct htd_intf_priv {
    char *name;
	struct usb_interface	*intf;
	struct usb_device	*udev;
	struct mutex		io_mutex; /* sync I/O with disconnect */
	struct mutex		ops_mutex;
	int			bitbang_enabled;
	int			id;
	int			index;
	u8			bulk_in;
	u8			bulk_out;
	size_t			bulk_in_sz;
	void			*bulk_in_buf;

	const struct usb_device_id	*usb_dev_id;
	struct htd_intf_info		*info;
	struct platform_device		*fifo_pdev;
	struct platform_device		*spi_pdev;
	struct gpiod_lookup_table	*lookup_fifo;
	struct gpiod_lookup_table	*lookup_cs;
	struct gpiod_lookup_table	*lookup_irq;
   

	struct gpio_chip	cbus_gpio;
	const char		*cbus_gpio_names[4];
	u8			cbus_pin_offsets[4];
	u8			cbus_mask;
	u8			pinbuf[4];

	struct gpio_chip	stm32h7_gpio;

    struct gpio_desc *ce_gpio;
	struct gpio_desc *interrupt_gpio;
	char *interrupt_name;
	int old_value;
	u8			gpiol_mask;
	u8			gpioh_mask;
	u8			gpiol_dir;
	u8			gpioh_dir;
	u8			tx_buf[4];
	unsigned int offset;
     bool    hwirq;
     int                      gpio_irq_map[4]; // GPIO to IRQ map (gpio_num elements)

     struct irq_chip   irq;                                // chip descriptor for IRQs
     int               num;
     uint8_t           irq_num;                            // number of pins with IRQs
     int               irq_base;                           // base IRQ allocated
     const struct cpumask *aff_mask;
     int               irq_types    [5]; // IRQ types (irq_num elements)
     bool              irq_enabled  [5]; // IRQ enabled flag (irq_num elements)
     int               irq_gpio_map [5]; // IRQ to GPIO pin map (irq_num elements)
     int               irq_hw;                             // IRQ for GPIO with hardware IRQ (default -1)
     int irq_poll_interval;
    struct work_struct irq_work;

	int		model;
	int		numgpio;
	u8		eeprom[FTDI_MAX_EEPROM_SIZE];
};


/* Device info struct used for device specific init. */
struct htd_intf_info {
	int (*probe)(struct usb_interface *intf, const void *plat_data);
	int (*remove)(struct usb_interface *intf);
	const void *plat_data; /* optional, passed to probe() */
	int			model;
	int 		numgpio;
};

struct htd_intf_device {
	struct htd_intf_priv irq_chip;

};

dev_t dev =0;
//static struct class *dev_class;
struct kobject *kobj_ref;



unsigned int gpio_no = 3;

int irqon;

int irqt = 2;

int poll_interval;

static DEFINE_IDA(htd_devid_ida);

static int htd_read_eeprom(struct htd_intf_priv *priv);

static int htd_stm32h7_gpio_to_irq(struct gpio_chip *chip, unsigned offset);

static void usb_gpio_irq_enable(struct irq_data *irqd);

static void usb_gpio_irq_disable(struct irq_data *irqd);

static int usbirq_irq_set_type(struct irq_data *irqd, unsigned type);

static uint poll_period = POLL_PERIOD_MS;       // module parameter poll period

unsigned int GPIO_irqNumber;

/*
static ssize_t eeprom_show(struct kobject *kobj,
				  struct kobj_attribute *attr, char *buf)
{
 //  struct htd_intf_priv *priv = container_of(*intf, struct htd_intf_priv,
 //                                     stm32h7_gpio);
//	struct htd_intf_priv *priv = dev_get_drvdata(dev);
//    htd_read_eeprom(priv);
//    return sysfs_emit(buf, "%hh \n", priv->eeprom);
    return 0;
}


static ssize_t eeprom_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *valbuf, size_t count)
{
        return count;
}
struct kobj_attribute eeprom = __ATTR(eeprom, 0660, eeprom_show, eeprom_store);
//static DEVICE_ATTR_RW(eeprom);


static int create_sysfs_attrs(struct usb_interface *intf)
{
	int retval = 0;

    kobj_ref = kobject_create_and_add("stm32h7",NULL); 
    

//			retval = device_create_file(&intf->dev,
//						    &dev_attr_eeprom);

            retval = sysfs_create_file(kobj_ref,&eeprom.attr);
						    
	return retval;
}

static void remove_sysfs_attrs(struct usb_interface *intf)
{

//			device_remove_file(&intf->dev, &dev_attr_eeprom);
            sysfs_remove_file(kobj_ref,&eeprom.attr);
}
*/
/* Use baudrate calculation borrowed from libhtd */
static int htd_to_clkbits(int baudrate, unsigned int clk, int clk_div,
			   unsigned long *encoded_divisor)
{
	static const char frac_code[8] = { 0, 3, 2, 4, 1, 5, 6, 7 };
	int best_baud = 0;
	int div, best_div;

	if (baudrate >= clk / clk_div) {
		*encoded_divisor = 0;
		best_baud = clk / clk_div;
	} else if (baudrate >= clk / (clk_div + clk_div / 2)) {
		*encoded_divisor = 1;
		best_baud = clk / (clk_div + clk_div / 2);
	} else if (baudrate >= clk / (2 * clk_div)) {
		*encoded_divisor = 2;
		best_baud = clk / (2 * clk_div);
	} else {
		/*
		 * Divide by 16 to have 3 fractional bits and
		 * one bit for rounding
		 */
		div = clk * 16 / clk_div / baudrate;
		if (div & 1)	/* Decide if to round up or down */
			best_div = div / 2 + 1;
		else
			best_div = div / 2;
		if (best_div > 0x20000)
			best_div = 0x1ffff;
		best_baud = clk * 16 / clk_div / best_div;
		if (best_baud & 1)	/* Decide if to round up or down */
			best_baud = best_baud / 2 + 1;
		else
			best_baud = best_baud / 2;
		*encoded_divisor = (best_div >> 3) |
				   (frac_code[best_div & 0x7] << 14);
	}
	return best_baud;
}

#define H_CLK	120000000
#define C_CLK	48000000
static int htd_convert_baudrate(struct htd_intf_priv *priv, int baud,
				 u16 *value, u16 *index)
{
	unsigned long encoded_divisor = 0;
	int best_baud = 0;

	if (baud <= 0)
		return -EINVAL;

	/*
	 * On H Devices, use 12000000 baudrate when possible.
	 * We have a 14 bit divisor, a 1 bit divisor switch (10 or 16),
	 * three fractional bits and a 120 MHz clock. Assume AN_120
	 * "Sub-integer divisors between 0 and 2 are not allowed" holds
	 * for DIV/10 CLK too, so /1, /1.5 and /2 can be handled the same
	 */
	if (baud * 10 > H_CLK / 0x3fff) {
		best_baud = htd_to_clkbits(baud, H_CLK, 10, &encoded_divisor);
		encoded_divisor |= 0x20000;	/* switch on CLK/10 */
	} else {
		best_baud = htd_to_clkbits(baud, C_CLK, 16, &encoded_divisor);
	}

	if (best_baud <= 0) {
		pr_err("Invalid baudrate: %d\n", best_baud);
		return -EINVAL;
	}

	/* Check within tolerance (about 5%) */
	if ((best_baud * 2 < baud) ||
	    (best_baud < baud
		? (best_baud * 21 < baud * 20)
		: (baud * 21 < best_baud * 20))) {
		pr_err("Unsupported baudrate.\n");
		return -EINVAL;
	}

	/* Split into "value" and "index" values */
	*value = (u16)(encoded_divisor & 0xffff);
	*index = (u16)(((encoded_divisor >> 8) & 0xff00) | priv->index);

	dev_dbg(&priv->intf->dev, "best baud %d, v/i: %d, %d\n",
		best_baud, *value, *index);
	return best_baud;
}

/*
 * htd_ctrl_xfer - FTDI control endpoint transfer
 * @intf: USB interface pointer
 * @desc: pointer to descriptor struct for control transfer
 *
 * Return:
 * Return: If successful, the number of bytes transferred. Otherwise,
 * a negative error number.
 */
static int htd_ctrl_xfer(struct usb_interface *intf, struct ctrl_desc *desc)
{
	struct htd_intf_priv *priv = usb_get_intfdata(intf);
	struct usb_device *udev = priv->udev;
	unsigned int pipe;
	int ret;

	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		ret = -ENODEV;
		goto exit;
	}

	if (!desc->data && desc->size)
		desc->data = priv->bulk_in_buf;

	if (desc->dir_out)
		pipe = usb_sndctrlpipe(udev, 0);
	else
		pipe = usb_rcvctrlpipe(udev, 0);

	ret = usb_control_msg(udev, pipe, desc->request, desc->requesttype,
			      desc->value, desc->index, desc->data, desc->size,
			      desc->timeout);
	if (ret < 0)
		dev_dbg(&udev->dev, "ctrl msg failed: %d\n", ret);
exit:
	mutex_unlock(&priv->io_mutex);
	return ret;
}

/*
 * htd_bulk_xfer - FTDI bulk endpoint transfer
 * @intf: USB interface pointer
 * @desc: pointer to descriptor struct for bulk-in or bulk-out transfer
 *
 * Return:
 * If successful, 0. Otherwise a negative error number. The number of
 * actual bytes transferred will be stored in the @desc->act_len field
 * of the descriptor struct.
 */
static int htd_bulk_xfer(struct usb_interface *intf, struct bulk_desc *desc)
{
	struct htd_intf_priv *priv = usb_get_intfdata(intf);
	struct usb_device *udev = priv->udev;
	unsigned int pipe;
	int ret;

	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		ret = -ENODEV;
		goto exit;
	}

	if (desc->dir_out)
		pipe = usb_sndbulkpipe(udev, priv->bulk_out);
	else
		pipe = usb_rcvbulkpipe(udev, priv->bulk_in);

	ret = usb_bulk_msg(udev, pipe, desc->data, desc->len,
			   &desc->act_len, desc->timeout);
	if (ret)
		dev_dbg(&udev->dev, "bulk msg failed: %d\n", ret);

exit:
	mutex_unlock(&priv->io_mutex);
	if (usb_wait_msec > 0) {
		usleep_range(usb_wait_msec * 1000, usb_wait_msec * 1000 + 1000);
	}
	return ret;
}

/*
 * htd_set_baudrate - set the device baud rate
 * @intf: USB interface pointer
 * @baudrate: baud rate value to set
 *
 * Return: If successful, 0. Otherwise a negative error number.
 */
static int htd_set_baudrate(struct usb_interface *intf, int baudrate)
{
	struct htd_intf_priv *priv = usb_get_intfdata(intf);
	struct ctrl_desc desc;
	u16 index, value;
	int ret;

	if (priv->bitbang_enabled)
		baudrate *= 4;

	ret = htd_convert_baudrate(priv, baudrate, &value, &index);
	if (ret < 0)
		return ret;

	desc.dir_out = true;
	desc.request = FTDI_SIO_SET_BAUDRATE_REQUEST;
	desc.requesttype = USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT;
	desc.value = value;
	desc.index = index;
	desc.data = NULL;
	desc.size = 0;
	desc.timeout = USB_CTRL_SET_TIMEOUT;

	ret = htd_ctrl_xfer(intf, &desc);
	if (ret < 0) {
		dev_dbg(&intf->dev, "failed to set baudrate: %d\n", ret);
		return ret;
	}

	return 0;
}

static int htd_set_clock(struct usb_interface *intf, int clock_freq_hz)
{
	struct htd_intf_priv *priv = usb_get_intfdata(intf);
	struct bulk_desc desc;
	uint8_t *buf = priv->tx_buf;
	uint32_t value = 0;
	int ret;

	desc.act_len = 0;
	desc.dir_out = true;
	desc.data = (char *)buf;
	desc.timeout = FTDI_USB_WRITE_TIMEOUT;

	switch (priv->usb_dev_id->idProduct) {
	case 0x6001: /* FT232 */
		if (clock_freq_hz >= FTDI_CLK_6MHZ) {
			value = (FTDI_CLK_6MHZ/clock_freq_hz) - 1;
		}
		break;

	case 0x6010: /* FT2232 */
	case 0x6011: /* FT4232 */
	case 0x6041: /* FT4233 */
	case 0x6014: /* FT232H */
	case 0x0146: /* GW16146 */
		desc.len = 1;
		if (clock_freq_hz <= (FTDI_CLK_30MHZ/65535)) {
			buf[0] = EN_DIV_5;
			ret = htd_bulk_xfer(intf, &desc);
			if (ret) {
				return ret;
			}
			value = (FTDI_CLK_6MHZ/clock_freq_hz) - 1;
		}
		else {
			buf[0] = DIS_DIV_5;
			ret = htd_bulk_xfer(intf, &desc);
			if (ret) {
				return ret;
			}
			value = (FTDI_CLK_30MHZ/clock_freq_hz) - 1;
		}

		break;
	}

	buf[0] = TCK_DIVISOR;
	buf[1] = (uint8_t)(value & 0xff);
	buf[2] = (uint8_t)(value >> 8);
	desc.act_len = 0;
	desc.len = 3;
	ret = htd_bulk_xfer(intf, &desc);

	return ret;
}


/*
 * htd_read_data - read from FTDI bulk-in endpoint
 * @intf: USB interface pointer
 * @buf:  pointer to data buffer
 * @len:  length in bytes of the data to read
 *
 * The two modem status bytes transferred in every read will
 * be removed and will not appear in the data buffer.
 *
 * Return:
 * If successful, the number of data bytes received (can be 0).
 * Otherwise, a negative error number.
 */
static int htd_read_data(struct usb_interface *intf, void *buf, size_t len)
{
	struct htd_intf_priv *priv = usb_get_intfdata(intf);
	struct bulk_desc desc;
	int ret;

	desc.act_len = 0;
	desc.dir_out = false;
	desc.data = priv->bulk_in_buf;
	/* Device sends 2 additional status bytes, read at least len + 2 */
	desc.len = min_t(size_t, len + 2, priv->bulk_in_sz);
	desc.timeout = FTDI_USB_READ_TIMEOUT;

	ret = htd_bulk_xfer(intf, &desc);
	if (ret)
		return ret;

	/* Only status bytes and no data? */
	if (desc.act_len <= 2)
		return 0;

	/* Skip first two status bytes */
	ret = desc.act_len - 2;
	if (ret > len)
		ret = len;
	memcpy(buf, desc.data + 2, ret);
	return ret;
}

/*
 * htd_write_data - write to FTDI bulk-out endpoint
 * @intf: USB interface pointer
 * @buf:  pointer to data buffer
 * @len:  length in bytes of the data to send
 *
 * Return:
 * If successful, the number of bytes transferred. Otherwise a negative
 * error number.
 */
static int htd_write_data(struct usb_interface *intf,
			   const char *buf, size_t len)
{
	struct bulk_desc desc;
	int ret;

	desc.act_len = 0;
	desc.dir_out = true;
	desc.data = (char *)buf;
	desc.len = len;
	desc.timeout = FTDI_USB_WRITE_TIMEOUT;

	ret = htd_bulk_xfer(intf, &desc);
	if (ret < 0)
		return ret;

	return desc.act_len;
}

/*
 * htd_set_bitmode - configure bitbang mode
 * @intf: USB interface pointer
 * @bitmask: line configuration bitmask
 * @mode: bitbang mode to set
 *
 * Return:
 * If successful, 0. Otherwise a negative error number.
 */
static int htd_set_bitmode(struct usb_interface *intf, unsigned char bitmask,
			    unsigned char mode)
{
	struct htd_intf_priv *priv = usb_get_intfdata(intf);
	struct ctrl_desc desc;
	int ret;

	desc.dir_out = true;
	desc.data = NULL;
	desc.request = FTDI_SIO_SET_BITMODE_REQUEST;
	desc.requesttype = USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT;
	desc.index = 1;
	desc.value = (mode << 8) | bitmask;
	desc.size = 0;
	desc.timeout = USB_CTRL_SET_TIMEOUT;

	ret = htd_ctrl_xfer(intf, &desc);
	if (ret < 0)
		return ret;

	switch (mode) {
	case BITMODE_BITBANG:
	case BITMODE_CBUS:
	case BITMODE_SYNCBB:
	case BITMODE_SYNCFF:
		priv->bitbang_enabled = 1;
		break;
	case BITMODE_MPSSE:
	case BITMODE_RESET:
	default:
		priv->bitbang_enabled = 0;
		break;
	}

	return 0;
}



static void htd_lock(struct usb_interface *intf)
{
	struct htd_intf_priv *priv = usb_get_intfdata(intf);

	mutex_lock(&priv->ops_mutex);
}

static void htd_unlock(struct usb_interface *intf)
{
	struct htd_intf_priv *priv = usb_get_intfdata(intf);

	mutex_unlock(&priv->ops_mutex);
}

static const struct htd_intf_ops htd_intf_ops = {
	.ctrl_xfer = htd_ctrl_xfer,
	.bulk_xfer = htd_bulk_xfer,
	.read_data = htd_read_data,
	.write_data = htd_write_data,
	.lock = htd_lock,
	.unlock = htd_unlock,
	.set_bitmode = htd_set_bitmode,
	.set_baudrate = htd_set_baudrate,
	.disable_bitbang = htd_disable_bitbang,
	.init_pins = htd_stm32h7_init_pins,
	.cfg_bus_pins = htd_stm32h7_cfg_bus_pins,
	.set_clock = htd_set_clock,
	.set_latency = htd_set_latency,
};

#define SPI_INTF_DEVNAME	"spi-htd-stm32h7"

// csn is actually on CS0 but ce is correct on 1 IE AD4 and IRQ on AD6
static struct dev_io_desc_data htd_spi_bus_dev_io[] = {
//       { "rst", 1, GPIO_ACTIVE_HIGH },
//       { "csn", 2, GPIO_ACTIVE_LOW },
};

static const struct stm32h7_spi_dev_data htd_spi_dev_data[] = {
	{
	.magic		= FTDI_MPSSE_IO_DESC_MAGIC,
	.desc		= htd_spi_bus_dev_io,
	.desc_len	= ARRAY_SIZE(htd_spi_bus_dev_io),
	},
};

static const struct property_entry mcp2515_properties[] = {
	PROPERTY_ENTRY_U32("clock-frequency", 8000000),
//	PROPERTY_ENTRY_U32("xceiver", 1),
//	PROPERTY_ENTRY_U32("gpio-controller", 3),
	{}
};

static const struct property_entry nrf24_properties[] = {
	PROPERTY_ENTRY_U32("interrupts", 3),
	{}
};

static const struct property_entry eeprom_93xx46_properties[] = {
	PROPERTY_ENTRY_U32("spi-max-frequency", 1000000),
	PROPERTY_ENTRY_U32("data-size", 16),
//	PROPERTY_ENTRY_U32("xceiver", 1),
//	PROPERTY_ENTRY_U32("gpio-controller", 3),
	{}
};

static const struct property_entry ili9341_properties[] = {
	PROPERTY_ENTRY_U32("rotate", 270),
	PROPERTY_ENTRY_BOOL("bgr"),
	PROPERTY_ENTRY_U32("fps", 60),
	PROPERTY_ENTRY_U32("speed", 30000000),
	PROPERTY_ENTRY_U32("buswidth", 8),
	PROPERTY_ENTRY_U32("regwidth", 8),
	{}
};

static const struct property_entry w25q32_properties[] = {
    PROPERTY_ENTRY_STRING("partitions", "poop"),
    PROPERTY_ENTRY_STRING(".label", "poop"),
    PROPERTY_ENTRY_STRING(".name", "poop"),
	{}
};

#include <dt-bindings/leds/common.h>

static const struct property_entry worldsemi_properties[] = {
    PROPERTY_ENTRY_U32("reg", 0),
    PROPERTY_ENTRY_U32("color", LED_COLOR_ID_RGB),
    PROPERTY_ENTRY_U32_ARRAY("function", LED_FUNCTION_STATUS),
	{}
};

static const struct property_entry sx1276_properties[] = {
    PROPERTY_ENTRY_U32("clock-frequency", 40000000),
    PROPERTY_ENTRY_U32("center-carrier-frq", 915000000),
//    PROPERTY_ENTRY_U32("minimal-RF-channel", 11),
//    PROPERTY_ENTRY_U32("maximum-RF-channel", 11),
	{}
};

static const struct software_node nrf24_node = {
	.properties = nrf24_properties,
};

static const struct software_node mcp2515_node = {
	.properties = mcp2515_properties,
};

static const struct software_node w25q32_node = {
	.properties = w25q32_properties,
};

static const struct software_node sx1276_node = {
	.properties = sx1276_properties,
};

static const struct software_node worldsemi_node = {
	.properties = worldsemi_properties,
};

static struct spi_board_info htd_spi_bus_info[] = {
    {
//    .modalias	= "yx240qv29",
//	.modalias	= "ili9341",
    .modalias	= "spi-petra",
//     .modalias	= "sx1278",
//    .modalias	= "spi-nor",
//	.modalias	= "mcp2515",
//    .modalias	= "spi-petra",
//    .modalias	= "nrf24",
//	.modalias	= "ili9341",
    .mode		= SPI_MODE_0,
//    .mode		= SPI_MODE_0 | SPI_LSB_FIRST | SPI_CS_HIGH,
//    .max_speed_hz	= 4000000,
    .max_speed_hz	= 12500000,
    .bus_num	= 0,
    .chip_select	= 0,
    .platform_data	= htd_spi_dev_data,
// 	.swnode	= &nrf24_node,    //changed from properties to swnode i dunno aroun kernel 5.15ish
//    .properties	= mcp2515_properties,
	.swnode  =  &worldsemi_node,
//    .swnode = &sx1276_node,
 //   .swnode = &w25q32_node,
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

static const struct stm32h7_spi_platform_data htd_spi_bus_plat_data = {
    .ops		= &htd_intf_ops,
    .spi_info	= htd_spi_bus_info,
    .spi_info_len	= ARRAY_SIZE(htd_spi_bus_info),
};


static const struct stm32h7_spi_platform_data usb_cfg_spi_plat_data = {
	.ops		= &htd_intf_ops,
	.spi_info	= usb_cfg_spi_info,
	.spi_info_len	= ARRAY_SIZE(usb_cfg_spi_info),
};

static struct platform_device *stm32h7_dev_register(struct htd_intf_priv *priv,
				const struct stm32h7_spi_platform_data *pd)
{
	struct device *parent = &priv->intf->dev;
	struct platform_device *pdev;
	struct gpiod_lookup_table *lookup;
	size_t lookup_size, tbl_size;
	int i, ret;
    
	ret = htd_intf_add_stm32h7_gpio(priv);
	if (ret < 0) {
		goto err;
	}
		
		if (irqpoll) {
		
        pd->spi_info[0].irq = GPIO_irqNumber;
          
        priv->ce_gpio = gpiochip_request_own_desc(&priv->stm32h7_gpio, 
                                                1,
                                                "ce",
                                                GPIOD_OUT_LOW,
                                                GPIO_ACTIVE_HIGH);
                                                
        gpiod_direction_output(priv->ce_gpio, true);
    
        gpiochip_free_own_desc(priv->ce_gpio);
    

//irq	irq_set_irq_type(GPIO_irqNumber, irqt);   
	} 
	pdev = platform_device_alloc(SPI_INTF_DEVNAME, 0);
	if (!pdev)
		return NULL;

	pdev->dev.parent = parent;
	pdev->dev.fwnode = NULL;
	

	priv->spi_pdev = pdev;
    
	tbl_size = pd->spi_info_len + 1;
	lookup_size = sizeof(*lookup) + tbl_size * sizeof(struct gpiod_lookup);
	lookup = devm_kzalloc(parent, lookup_size, GFP_KERNEL);
	if (!lookup) {
		ret = -ENOMEM;
		goto err;
	}

	for (i = 0; i < pd->spi_info_len; i++) {
		dev_dbg(parent, "INFO: %s cs %d\n",
			pd->spi_info[i].modalias, pd->spi_info[i].chip_select);
	}


	ret = platform_device_add_data(pdev, pd, sizeof(*pd));
	if (ret)
		goto err;

	pdev->id = priv->id;


	lookup->dev_id = devm_kasprintf(parent, GFP_KERNEL, "%s.%d",
					pdev->name, pdev->id);
	if (!lookup->dev_id) {
		ret = -ENOMEM;
		goto err;
	}

	for (i = 0; i < pd->spi_info_len; i++) {
		lookup->table[i].key = priv->stm32h7_gpio.label;
		lookup->table[i].chip_hwnum = pd->spi_info[i].chip_select;
		lookup->table[i].idx = i;
		lookup->table[i].con_id = NULL;
		if (pd->spi_info[i].mode & SPI_CS_HIGH)
			lookup->table[i].flags = GPIO_ACTIVE_HIGH;
		else
			lookup->table[i].flags = GPIO_ACTIVE_LOW;
	}

	priv->lookup_cs = lookup;

	gpiod_add_lookup_table(priv->lookup_cs);


	ret = platform_device_add(pdev);
	if (ret < 0)
		goto err_add;

	dev_dbg(&pdev->dev, "%s done\n", __func__);
	return pdev;

err_add:
	gpiod_remove_lookup_table(priv->lookup_cs);
err:
	platform_device_put(pdev);
	return ERR_PTR(ret);
}

static int htd_intf_spi_probe(struct usb_interface *intf,
				 const void *plat_data)
{
	struct htd_intf_priv *priv = usb_get_intfdata(intf);
	struct device *dev = &intf->dev;
	struct platform_device *pdev;

	pdev = stm32h7_dev_register(priv, plat_data);
	if (IS_ERR(pdev)) {
		dev_err(dev, "%s: Can't create MPSSE SPI device %ld\n",
			__func__, PTR_ERR(pdev));
		return PTR_ERR(pdev);
	}

	priv->spi_pdev = pdev;

	return 0;
}

static int htd_intf_spi_remove(struct usb_interface *intf)
{
	struct htd_intf_priv *priv = usb_get_intfdata(intf);
	struct device *dev = &intf->dev;

	dev_dbg(dev, "%s: spi pdev %p\n", __func__, priv->spi_pdev);
	gpiod_remove_lookup_table(priv->lookup_cs);
    priv->irq_enabled[3] = false;
	platform_device_unregister(priv->spi_pdev);
	return 0;
}

static const struct htd_intf_info htd_spi_bus_intf_info = {
    .probe  = htd_intf_spi_probe,
    .remove  = htd_intf_spi_remove,
    .plat_data  = &htd_spi_bus_plat_data,
};

static const struct htd_intf_info usb_cfg_spi_intf_info = {
	.probe  = htd_intf_spi_probe,
	.remove  = htd_intf_spi_remove,
	.plat_data  = &usb_cfg_spi_plat_data,
};


static int htd_intf_probe(struct usb_interface *intf,
			     const struct usb_device_id *id)
{
	struct htd_intf_priv *priv;
	struct device *dev = &intf->dev;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	const struct htd_intf_info *info;
	unsigned int i;
	int ret = 0;
	int inf;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->udev = usb_get_dev(interface_to_usbdev(intf));
	inf = intf->cur_altsetting->desc.bInterfaceNumber;


	iface_desc = intf->cur_altsetting;

	for (i = 0; i < iface_desc->desc.bNumEndpoints; i++) {
		endpoint = &iface_desc->endpoint[i].desc;

		if (usb_endpoint_is_bulk_out(endpoint))
			priv->bulk_out = endpoint->bEndpointAddress;

		if (usb_endpoint_is_bulk_in(endpoint)) {
			priv->bulk_in = endpoint->bEndpointAddress;
			priv->bulk_in_sz = usb_endpoint_maxp(endpoint);
		}
	}

	priv->usb_dev_id = id;
	priv->index = 1;
	priv->intf = intf;
	priv->info = (struct htd_intf_info *)id->driver_info;

	info = priv->info;
	if (!info) {
		dev_err(dev, "Missing device specific driver info...\n");
		return -ENODEV;
	}

	mutex_init(&priv->io_mutex);
	mutex_init(&priv->ops_mutex);
	usb_set_intfdata(intf, priv);

	priv->bulk_in_buf = devm_kmalloc(dev, priv->bulk_in_sz, GFP_KERNEL);
	if (!priv->bulk_in_buf)
		return -ENOMEM;

	priv->udev = usb_get_dev(interface_to_usbdev(intf));

	priv->id = ida_simple_get(&htd_devid_ida, 0, 0, GFP_KERNEL);
	if (priv->id < 0)
		return priv->id;

	if (info->probe) {
		ret = info->probe(intf, info->plat_data);
		if (ret < 0)
			goto err;
		return 0;
	}

	/* for simple GPIO-only devices */
//	ret = -ENODEV;

	if (info->use_cbus_gpio_ctrl)
		ret = htd_intf_add_cbus_gpio(priv);
	else if (info->use_stm32h7_gpio_ctrl)
		ret = htd_intf_add_stm32h7_gpio(priv);
	if (!ret)
		return 0;

err:
	ida_simple_remove(&htd_devid_ida, priv->id);
	return ret;
}

static void htd_intf_disconnect(struct usb_interface *intf)
{
	struct htd_intf_priv *priv = usb_get_intfdata(intf);
	const struct htd_intf_info *info;
    
	if (irqon == 1) {            
		irqon = 0;
		cancel_work_sync(&priv->irq_work);
	}
    
    usleep_range(7000, 7200);
    
//    kobject_put(kobj_ref);
//	sysfs_remove_file(kernel_kobj, &eeprom.attr);
    
    
	info = (struct htd_intf_info *)priv->usb_dev_id->driver_info;
	if (info && info->remove)
		info->remove(intf);


   	if (info->use_stm32h7_gpio_ctrl) {
          gpiochip_remove(&priv->stm32h7_gpio);
    }
    
	if (info->use_cbus_gpio_ctrl)
		gpiochip_remove(&priv->cbus_gpio);

 
	mutex_lock(&priv->io_mutex);
 	priv->intf = NULL;
	usb_set_intfdata(intf, NULL);
	mutex_unlock(&priv->io_mutex);

	usb_put_dev(priv->udev);
	ida_simple_remove(&htd_devid_ida, priv->id);
//	mutex_destroy(&priv->io_mutex);
//	mutex_destroy(&priv->ops_mutex);
//	kfree (priv);
}

#define FTDI_VID			0x0403
#define ARRI_FPP_INTF_PRODUCT_ID	0x7148
#define ARRI_SPI_INTF_PRODUCT_ID	0x7149

static struct usb_device_id htd_intf_table[] = {
//	{ USB_DEVICE(FTDI_VID, 0x6010),
//		.driver_info = (kernel_ulong_t)&usb_cfg_fifo_intf_info },
//	{ USB_DEVICE(FTDI_VID, ARRI_SPI_INTF_PRODUCT_ID),
//		.driver_info = (kernel_ulong_t)&usb_cfg_spi_intf_info },
	{ USB_DEVICE(FTDI_VID, 0x6014),
        .driver_info = (kernel_ulong_t)&htd_spi_bus_intf_info },
	{ USB_DEVICE(FTDI_VID, 0x6010),
        .driver_info = (kernel_ulong_t)&htd_spi_bus_intf_info },
	{ USB_DEVICE(FTDI_VID, 0x6011),
        .driver_info = (kernel_ulong_t)&htd_spi_bus_intf_info },
	{ USB_DEVICE(FTDI_VID, 0x6041),
        .driver_info = (kernel_ulong_t)&htd_spi_bus_intf_info },
	{}
};
MODULE_DEVICE_TABLE(usb, htd_intf_table);

static struct usb_driver htd_intf_driver = {
	.name		= KBUILD_MODNAME,
	.id_table	= htd_intf_table,
	.probe		= htd_intf_probe,
	.disconnect	= htd_intf_disconnect,
};

module_usb_driver(htd_intf_driver);

MODULE_ALIAS("htd-intf");
MODULE_AUTHOR("Anatolij Gustschin <agust@denx.de>");
MODULE_DESCRIPTION("FT232H to FPGA interface driver");
MODULE_LICENSE("GPL v2");

module_param(poll_period, uint, 0644);
MODULE_PARM_DESC(poll_period, "GPIO polling period in ms (default 10 ms)");
