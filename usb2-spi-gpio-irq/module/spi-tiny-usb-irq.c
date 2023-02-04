/*
 * driver for the spi-tiny-usb adapter - 1.0
 *
 * Copyright (C) 2014 Krystian Duzynski (krystian.duzynski@gmail.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2.
 *
 * Initially based on i2c-tiny-usb project of Till Harbaum (Till@Harbaum.org)
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>

#include <linux/usb.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/uio_driver.h>

//16ton
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/interrupt.h>
#include <linux/fs.h>

#include "../config.h"

#define USB_CMD_WRITE       0
#define USB_CMD_READ        1
#define USB_CMD_GPIO_OUTPUT 2
#define USB_CMD_GPIO_INPUT  3
#define USB_CMD_GPIO_SET    4
#define USB_CMD_GPIO_GET    5

#define FLAGS_BEGIN 1
#define FLAGS_END   2
//16ton
//#define UIO_IRQ_CUSTOM 329
#define GPIO_326_IN  (326)
unsigned int GPIO_irqNumber;

const char *gpio_names[] = { "led", "GP", "EXT", "RESET", "DC" };

/* Structure to hold all of our device specific stuff */
struct spi_tiny_usb {
	struct spi_master *master;	/* spi master related things */
	struct spi_device *fb_ili9341;	/* spi device related things */
	struct spi_board_info info;	/* board info for spidev module */
	struct usb_device *udev;
	struct gpio_chip gpio_chip;	/* gpio related things */
};


static void spi_tiny_usb_free(struct spi_tiny_usb *priv)
{
   usb_put_dev(data->udev);

   kfree(data); //deallocate, allocated by kzmalloc()

}

static int spi_tiny_usb_freqtodiv(int freq)
{
	int div = 48 * 1000 * 1000 / freq;
	int i, divVal = 0;
	for (i = 2; i <= 256; i *= 2, divVal++)
		if (i >= div)
			break;
	return divVal;
}

static int
usb_read(struct spi_tiny_usb *dev, int cmd, int value, int index, void *data, int len);
static int
usb_write(struct spi_tiny_usb *dev, int cmd, int value, int index, void *data, int len);

/* ----- begin of spi layer ---------------------------------------------- */

static int spi_tiny_usb_xfer_one(struct spi_master *master, struct spi_message *m)
{
	struct spi_tiny_usb *priv = spi_master_get_devdata(master);
	struct spi_transfer *t;
	int spi_flags;
	int ret = 0;

	m->actual_length = 0;

	spi_flags = FLAGS_BEGIN;
	list_for_each_entry(t, &m->transfers, transfer_list) {
		if (list_is_last(&t->transfer_list, &m->transfers))
			spi_flags |= FLAGS_END;

		spi_flags |= spi_tiny_usb_freqtodiv(t->speed_hz) << 2;

		dev_dbg(&master->dev,
			"tx: %p rx: %p len: %d speed: %d flags: %d delay: %d\n", t->tx_buf,
			t->rx_buf, t->len, t->speed_hz, spi_flags, t->delay.value);

		if (t->cs_change)
			spi_flags |= FLAGS_END;

		if (t->tx_buf) {
			ret = usb_write(priv, 0, 0, spi_flags, (void *)t->tx_buf, t->len);
			if (ret < 0)
				break;
		} else {
			void *txbuf = kmalloc(t->len, GFP_KERNEL);
			memset(txbuf, 0x00, t->len);
			if (!txbuf) {
				ret = -ENOMEM;
				break;
			}
			ret = usb_write(priv, 0, 0, spi_flags, txbuf, t->len);
			kfree(txbuf);
			if (ret < 0)
				break;
		}

		if (t->rx_buf) {
			ret = usb_read(priv, 1, 0, 0, t->rx_buf, t->len);
			if (ret < 0)
				break;
		}
		// spin_lock_irqsave(&ebu_lock, flags);
		// ret = spi_tiny_usb_xfer(m->spi, t, spi_flags);
		// spin_unlock_irqrestore(&ebu_lock, flags);

		m->actual_length += t->len;

		if (t->delay.value)
			udelay(t->delay.value);

		spi_flags = 0;

		if (t->cs_change)
			spi_flags |= FLAGS_BEGIN;
	}

	m->status = ret;
	spi_finalize_current_message(master);

	return 0;
}

static irqreturn_t gpio_irq_handler(int irq,void *dev_id)
{
  static unsigned long flags = 0;

  local_irq_save(flags);
  pr_info("Interrupt Occurred : ");
  local_irq_restore(flags);
  return IRQ_HANDLED;
}

/* ----- end of spi layer ------------------------------------------------ */

/* ----- begin of gpio layer ---------------------------------------------- */

static inline struct spi_tiny_usb *spi_tiny_usb_gc_to_priv(struct gpio_chip *chip)
{
	return container_of(chip, struct spi_tiny_usb, gpio_chip);
}

static int spi_tiny_usb_gpio_input(struct gpio_chip *chip, unsigned offset)
{
	struct spi_tiny_usb *priv = spi_tiny_usb_gc_to_priv(chip);
	int ret;

	if (offset == 0)
		return -ENXIO;

	ret = usb_read(priv, USB_CMD_GPIO_INPUT, 0, offset, 0, 0);
	if (ret < 0)
		return ret;

	return 0;
}

static int spi_tiny_usb_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct spi_tiny_usb *priv = spi_tiny_usb_gc_to_priv(chip);
	int ret, retval;

	char *rxbuf = kmalloc(1, GFP_KERNEL);
	if (!rxbuf)
		return -ENOMEM;
	ret = usb_read(priv, USB_CMD_GPIO_GET, 0, offset, rxbuf, 1);
	retval = rxbuf[0] ? 1 : 0;
	kfree(rxbuf);
	if (ret < 0)
		return ret;

	return retval;
}

static int spi_tiny_usb_gpio_output(struct gpio_chip *chip, unsigned offset, int val)
{
	struct spi_tiny_usb *priv = spi_tiny_usb_gc_to_priv(chip);
	int ret;

	ret = usb_read(priv, USB_CMD_GPIO_OUTPUT, 0, offset, 0, 0);
	if (ret < 0)
		return ret;

	return 0;
}

static void spi_tiny_usb_gpio_set(struct gpio_chip *chip, unsigned offset, int val)
{
	struct spi_tiny_usb *priv = spi_tiny_usb_gc_to_priv(chip);

	usb_read(priv, USB_CMD_GPIO_SET, val, offset, 0, 0);
}

/* ----- end of gpio layer ------------------------------------------------ */

/* ----- begin of usb layer ---------------------------------------------- */

static const struct usb_device_id spi_tiny_usb_table[] = {
	{USB_DEVICE(VID, PID)},
	{}
};

MODULE_DEVICE_TABLE(usb, spi_tiny_usb_table);

static int
usb_read(struct spi_tiny_usb *dev, int cmd, int value, int index, void *data, int len)
{
	/* do control transfer */
	return usb_control_msg(dev->usb_dev, usb_rcvctrlpipe(dev->usb_dev, 0),
			       cmd,
			       USB_TYPE_VENDOR | USB_RECIP_INTERFACE |
			       USB_DIR_IN, value, index, data, len, 2000);
}

static int
usb_write(struct spi_tiny_usb *dev, int cmd, int value, int index, void *data, int len)
{
	/* do control transfer */
	return usb_control_msg(dev->usb_dev, usb_sndctrlpipe(dev->usb_dev, 0),
			       cmd, USB_TYPE_VENDOR | USB_RECIP_INTERFACE,
			       value, index, data, len, 2000);
}

static int spi_tiny_usb_probe(struct usb_interface *interface,
			      const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(interface);
	struct spi_tiny_usb *data;
	int ret = -ENOMEM;
	u16 version;

	dev_dbg(&interface->dev, "probing usb device\n");

	/* allocate memory for our device state and initialize it */
	data = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	data->usb_dev = usb_get_dev(interface_to_usbdev(interface));
	data->interface = interface;

	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, data);

	version = le16_to_cpu(data->usb_dev->descriptor.bcdDevice);
	dev_info(&interface->dev,
		 "version %x.%02x found at bus %03d address %03d\n",
		 version >> 8, version & 0xff, data->usb_dev->bus->busnum,
		 data->usb_dev->devnum);

	dev_info(&interface->dev, "connected spi-tiny-usb device\n");

	// SPI master
	data->master = spi_alloc_master(&interface->dev, sizeof(*priv));
	if (!data->master)
		goto error;
	data->master->mode_bits = SPI_MODE_0;
	data->master->flags = 0;
	// data->master->setup = spi_tiny_usb_setup;
	// data->master->prepare_transfer_hardware = spi_tiny_usb_prepare_xfer;
	data->master->transfer_one_message = spi_tiny_usb_xfer_one;
	// data->master->unprepare_transfer_hardware = spi_tiny_usb_unprepare_xfer;
	data->master->dev.of_node = interface->dev.of_node;
	data->master->num_chipselect = 1;
	data->master->max_speed_hz = 48 * 1000 * 1000 / 2;
	data->master->min_speed_hz = 48 * 1000 * 1000 / 256;
	// data->master->dev.platform_data = priv;
	spi_master_set_devdata(data->master, priv);

	ret = spi_register_master(data->master);
	if (ret)
		goto error2;

	strcpy(data->info.modalias, "fb_ili9341");
	data->info.max_speed_hz = 48 * 1000 * 1000 / 2;
	data->info.chip_select = 0;
	data->info.mode = SPI_MODE_0;

	data->info.controller_data = priv;
	data->fb_ili9341 = spi_new_device(data->master, &data->info);
	if (!data->fb_ili9341)
		goto error2;
	dev_info(&interface->dev, "added new SPI device\n");

    GPIO_irqNumber = gpio_to_irq(GPIO_326_IN);
    pr_info("GPIO_irqNumber = %d\n", GPIO_irqNumber);

    if (request_irq(GPIO_irqNumber,             //IRQ number
                  (void *)gpio_irq_handler,   //IRQ handler
                  IRQF_TRIGGER_RISING,        //Handler will be called in raising edge
                  "spi-tiny-usb",               //used to identify the device name using this IRQ
                  NULL)) {                    //device id for shared IRQ
    pr_err("my_device: cannot register IRQ ");
    goto error2;
  }

	// USB interrupt
	data->urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!data->urb) {
	    dev_info(&interface->dev, "spi-tiny-usb no usb irq!\n");
		goto error2;
        };
    //16ton


	data->urbBuffer = kmalloc(64, GFP_KERNEL);

	usb_fill_int_urb(data->urb, data->usb_dev,
			 usb_rcvintpipe(data->usb_dev, 1), data->urbBuffer, 64,
			 spi_tiny_usb_urb_complete, priv, 10);


	ret = usb_submit_urb(data->urb, GFP_KERNEL);
	if (ret) {
	    dev_info(&interface->dev, "spi-tiny-usb priv urb gfp_kernel failedr\n");
// 16ton  goto error2 was commented out but didnt have { }; so hopefully all it was
		goto error2;
		};
	dev_info(&interface->dev, "started USB interrupts handler\n");

	// GPIOs
	memset(&data->gpio_chip, 0x00, sizeof(data->gpio_chip));
	data->gpio_chip.owner = THIS_MODULE;
	data->gpio_chip.parent = &interface->dev;
	data->gpio_chip.label = dev_name(data->gpio_chip.parent);
	data->gpio_chip.direction_input = spi_tiny_usb_gpio_input;
	data->gpio_chip.direction_output = spi_tiny_usb_gpio_output;
	data->gpio_chip.get = spi_tiny_usb_gpio_get;
	data->gpio_chip.set = spi_tiny_usb_gpio_set;
	data->gpio_chip.base = -1;
	data->gpio_chip.ngpio = 5;
	data->gpio_chip.names = gpio_names;

	dev_dbg(&interface->dev, "adding GPIO interface\n");
	ret = gpiochip_add(&data->gpio_chip);
	if (ret) {
		printk(KERN_DEBUG "err %d\n", ret);
		goto error2;
	}
	dev_info(&interface->dev, "added GPIO interface\n");

	return 0;

 error2:
	printk(KERN_DEBUG "spi-tiny-usb error2\n");
	spi_master_put(data->master);

 error:
	printk(KERN_DEBUG "spi-tiny-usb error\n");
	if (priv)
		spi_tiny_usb_free(priv);

	return ret;
}

static void spi_tiny_usb_disconnect(struct usb_interface *interface)
{
	struct spi_tiny_usb *priv = usb_get_intfdata(interface);
//	int i, ret;
    int i;

	for (i = data->gpio_chip.base; i < data->gpio_chip.base + data->gpio_chip.ngpio;
	     i++) {
		gpio_free(i);
	}

	dev_dbg(&interface->dev, "gpiochip_remove\n");
	free_irq(GPIO_irqNumber,NULL);
//	ret = gpiochip_remove(&data->gpio_chip);
    gpiochip_remove(&data->gpio_chip);

	dev_dbg(&interface->dev, "usb_kill_urb\n");
	usb_kill_urb(data->urb);

	dev_dbg(&interface->dev, "urbBuffer\n");
	if (data->urbBuffer)
		kfree(data->urbBuffer);

	dev_dbg(&interface->dev, "uio\n");
	if (data->uio) {
		uio_unregister_device(data->uio);
		kfree(data->uio);
	}

	dev_dbg(&interface->dev, "usb_free_urb\n");
	if (data->urb)
		usb_free_urb(data->urb);

	dev_dbg(&interface->dev, "spi_unregister_master\n");
	spi_unregister_master(data->master);

	dev_dbg(&interface->dev, "spi_tiny_usb_free\n");
	usb_set_intfdata(interface, NULL);
	spi_tiny_usb_free(priv);

	dev_dbg(&interface->dev, "disconnected\n");
}

static struct usb_driver spi_tiny_usb_driver = {
	.name = "spi-tiny-usb",
	.probe = spi_tiny_usb_probe,
	.disconnect = spi_tiny_usb_disconnect,
	.id_table = spi_tiny_usb_table,
};

module_usb_driver(spi_tiny_usb_driver);

/* ----- end of usb layer ------------------------------------------------ */

MODULE_AUTHOR("Krystian Duzynski <krystian.duzynski@gmail.com>");
MODULE_DESCRIPTION("spi-tiny-usb driver v1.0");
MODULE_LICENSE("GPL");
