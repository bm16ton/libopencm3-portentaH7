#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/printk.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/machine.h>
#include <linux/idr.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/usb/ch9.h>
#include <linux/usb.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/uio_driver.h>
#include <linux/iopoll.h>

#define USB_CMD_WRITE       0
#define USB_CMD_READ        1
#define USB_CMD_GPIO_OUTPUT 2
#define USB_CMD_GPIO_INPUT  3
#define USB_CMD_GPIO_SET    4
#define USB_CMD_GPIO_GET    5

#define FLAGS_BEGIN 1
#define FLAGS_END   2

const char *gpio_names[] = { "led", "GP", "EXT", "RESET", "DC" };

struct spi_tiny_plat_priv {
	struct usb_device *usb_dev;	/* the usb device for this device */
	struct usb_interface *interface;	/* the interface for this device */
	struct urb *urb;	/* urb for usb interrupt transfer */
	char *urbBuffer;	/* urb incoming data buffer */
	const struct usb_device_id	*usb_dev_id;
	struct platform_device		*spi_pdev;
	struct gpiod_lookup_table	*lookup_cs;
	struct gpiod_lookup_table	*lookup_dc;
	struct gpiod_lookup_table	*lookup_reset
	;
	struct gpio_chip gpio_chip;	/* gpio related things */
	const char		*cbus_gpio_names[4];
	u8			cbus_pin_offsets[4];
	u8			cbus_mask;
	u8			pinbuf[4];
};

static void spi_tiny_usb_free(struct spi_tiny_usb *priv)
{
	usb_put_dev(priv->usb_dev);
	kfree(priv);
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

static const struct usb_device_id spi_tiny_usb_table[] = {
	{USB_DEVICE(VID, PID)},
	{}
};

MODULE_DEVICE_TABLE(usb, spi_tiny_usb_table);

static void spi_tiny_usb_urb_complete(struct urb *urb)
{
	struct spi_tiny_usb *priv = (struct spi_tiny_usb *)urb->context;
	int ret;

	if (urb->status == 0) {
		uio_event_notify(priv->uio);
		dev_info(&priv->interface->dev,
			"spi_tiny_usb_urb_complete (%d) %d %d %d %d\n",
			urb->status, priv->urbBuffer[0], priv->urbBuffer[1],
			priv->urbBuffer[2], priv->urbBuffer[3]);
	}

	ret = usb_submit_urb(priv->urb, GFP_KERNEL);
}

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

static struct platform_device *spi_tiny_dev_register(struct spi_tiny_plat *priv;,
				const struct spi_tiny_spi_platform_data *pd)
{
	struct device *parent = &priv->intf->dev;
	struct platform_device *pdev;
	struct gpiod_lookup_table *lookup;
	size_t lookup_size, tbl_size;
	int i, ret;
	int ret = -ENOMEM;
	u16 version;

	dev_dbg(&interface->dev, "probing usb device\n");

	/* allocate memory for our device state and initialize it
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;
*/

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

		ret = spi_tiny_plat_add_spi_tiny_gpio(priv);
	if (ret < 0)
		goto err;

	lookup->dev_id = devm_kasprintf(parent, GFP_KERNEL, "%s.%d",
					pdev->name, pdev->id);
	if (!lookup->dev_id) {
		ret = -ENOMEM;
		goto err;
	}

	for (i = 0; i < pd->spi_info_len; i++) {
		lookup->table[i].key = priv->spi_tiny_gpio.label;
		lookup->table[i].chip_hwnum = pd->spi_info[i].chip_select;
		lookup->table[i].idx = i;
		lookup->table[i].con_id = NULL;
		if (pd->spi_info[i].mode & SPI_CS_HIGH)
			lookup->table[i].flags = GPIO_ACTIVE_HIGH;
		else
			lookup->table[i].flags = GPIO_ACTIVE_LOW;
	}

	priv->lookup_cs = lookup;
	priv->lookup_dc = lookup;
	priv->lookup_reset = lookup;
	gpiod_add_lookup_table(priv->lookup_cs);
	gpiod_add_lookup_table(priv->lookup_dc);
	gpiod_add_lookup_table(priv->lookup_reset);

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

static int ft232h_intf_spi_probe(struct usb_interface *interface,
				 const void *plat_data)
{
	struct spi_tiny_plat_priv *priv = usb_get_intfdata(interface);
	struct device *dev = &interface->dev;
	struct platform_device *pdev;

	pdev = spi_tiny_dev_register(priv, plat_data);
	if (IS_ERR(pdev)) {
		dev_err(dev, "%s: Can't create TINY-SPI device %ld\n",
			__func__, PTR_ERR(pdev));
		return PTR_ERR(pdev);
	}

	priv->spi_pdev = pdev;
	return 0;
}

static int spi_tiny_plat_spi_remove(struct usb_interface *intferface)
{
	struct spi_tiny_plat_priv *priv = usb_get_intfdata(interface);
	struct device *dev = &intf->dev;

	dev_dbg(dev, "%s: spi pdev %p\n", __func__, priv->spi_pdev);
	gpiod_remove_lookup_table(priv->lookup_cs);
	platform_device_unregister(priv->spi_pdev);
	return 0;
}

static const struct spi_tiny_plat_info tiny_spi_bus_intf_info = {
    .probe  = spi_tiny_plat_spi_probe,
    .remove  = spi_tiny_plat_spi_remove,
    .plat_data  = &fspi_tiny_plat_spi_bus_plat_data,
};

static int spi_tiny_plat_probe(struct usb_interface *intf,
			     const struct usb_device_id *id)
{
	struct spi_tiny_plat_priv *priv;
	struct device *dev = &intf->dev;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	const struct spi_tiny_plat_info *info;
	unsigned int i;
	int ret = 0;

	priv->usb_dev = usb_get_dev(interface_to_usbdev(interface));
	priv->interface = interface;

	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, priv);

	version = le16_to_cpu(priv->usb_dev->descriptor.bcdDevice);
	dev_info(&interface->dev,
		 "version %x.%02x found at bus %03d address %03d\n",
		 version >> 8, version & 0xff, priv->usb_dev->bus->busnum,
		 priv->usb_dev->devnum);

	dev_info(&interface->dev, "connected spi-tiny-usb device\n");

	// SPI master
	priv->master = spi_alloc_master(&interface->dev, sizeof(*priv));
	if (!priv->master)
		goto error;
	priv->master->mode_bits = SPI_MODE_0;
	priv->master->flags = 0;
	// priv->master->setup = spi_tiny_usb_setup;
	// priv->master->prepare_transfer_hardware = spi_tiny_usb_prepare_xfer;
	priv->master->transfer_one_message = spi_tiny_usb_xfer_one;
	// priv->master->unprepare_transfer_hardware = spi_tiny_usb_unprepare_xfer;
	priv->master->dev.of_node = interface->dev.of_node;
	priv->master->num_chipselect = 1;
	priv->master->max_speed_hz = 48 * 1000 * 1000 / 2;
	priv->master->min_speed_hz = 48 * 1000 * 1000 / 256;
	// priv->master->dev.platform_data = priv;
	spi_master_set_devdata(priv->master, priv);

	ret = spi_register_master(priv->master);
	if (ret)
		goto error2;

	strcpy(priv->info.modalias, "fb_ili9341");
	priv->info.max_speed_hz = 48 * 1000 * 1000 / 2;
	priv->info.chip_select = 0;
	priv->info.mode = SPI_MODE_0;

	priv->info.controller_data = priv;
	priv->fb_ili9341 = spi_new_device(priv->master, &priv->info);
	if (!priv->fb_ili9341)
		goto error2;
	dev_info(&interface->dev, "added new SPI device\n");

	// UIO
	priv->uio = kzalloc(sizeof(struct uio_info), GFP_KERNEL);
	if (!priv->uio)
		goto error2;
	priv->uio->priv = priv;
	priv->uio->name = "spi-tiny-usb";
	priv->uio->version = "1.0.16ton";

	priv->uio->mem[0].size = 0;
	priv->uio->port[0].size = 0;

	priv->uio->irq = UIO_IRQ_CUSTOM; //UIO_IRQ_CUSTOM;
	priv->uio->irq_flags = IRQF_SHARED;
	priv->uio->irqcontrol = spi_tiny_usb_irqcontrol;

	if (uio_register_device(&interface->dev, priv->uio))
		goto error2;
	dev_info(&interface->dev, "registered new UIO device\n");

	// USB interrupt
	priv->urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!priv->urb) {
	    dev_info(&interface->dev, "spi-tiny-usb no usb irq!\n");
		goto error2;
        };
    //16ton


	priv->urbBuffer = kmalloc(64, GFP_KERNEL);

	usb_fill_int_urb(priv->urb, priv->usb_dev,
			 usb_rcvintpipe(priv->usb_dev, 1), priv->urbBuffer, 64,
			 spi_tiny_usb_urb_complete, priv, 10);


	ret = usb_submit_urb(priv->urb, GFP_KERNEL);
	if (ret)
	    dev_info(&interface->dev, "spi-tiny-usb priv urb gfp_kernel failedr\n");
//		goto error2;
	dev_info(&interface->dev, "started USB interrupts handler\n");

	// GPIOs
	memset(&priv->gpio_chip, 0x00, sizeof(priv->gpio_chip));
	priv->gpio_chip.owner = THIS_MODULE;
	priv->gpio_chip.parent = &interface->dev;
	priv->gpio_chip.label = dev_name(priv->gpio_chip.parent);
	priv->gpio_chip.direction_input = spi_tiny_usb_gpio_input;
	priv->gpio_chip.direction_output = spi_tiny_usb_gpio_output;
	priv->gpio_chip.get = spi_tiny_usb_gpio_get;
	priv->gpio_chip.set = spi_tiny_usb_gpio_set;
	priv->gpio_chip.base = -1;
	priv->gpio_chip.ngpio = 5;
	priv->gpio_chip.names = gpio_names;

	dev_dbg(&interface->dev, "adding GPIO interface\n");
	ret = gpiochip_add(&priv->gpio_chip);
	if (ret) {
		printk(KERN_DEBUG "err %d\n", ret);
		goto error2;
	}
	dev_info(&interface->dev, "added GPIO interface\n");

	return 0;

 error2:
	printk(KERN_DEBUG "spi-tiny-usb error2\n");
	spi_master_put(priv->master);

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

	for (i = priv->gpio_chip.base; i < priv->gpio_chip.base + priv->gpio_chip.ngpio;
	     i++) {
		gpio_free(i);
	}

	dev_dbg(&interface->dev, "gpiochip_remove\n");
//	ret = gpiochip_remove(&priv->gpio_chip);
    gpiochip_remove(&priv->gpio_chip);

	dev_dbg(&interface->dev, "usb_kill_urb\n");
	usb_kill_urb(priv->urb);

	dev_dbg(&interface->dev, "urbBuffer\n");
	if (priv->urbBuffer)
		kfree(priv->urbBuffer);

	dev_dbg(&interface->dev, "uio\n");
	if (priv->uio) {
		uio_unregister_device(priv->uio);
		kfree(priv->uio);
	}

	dev_dbg(&interface->dev, "usb_free_urb\n");
	if (priv->urb)
		usb_free_urb(priv->urb);

	dev_dbg(&interface->dev, "spi_unregister_master\n");
	spi_unregister_master(priv->master);

	dev_dbg(&interface->dev, "spi_tiny_usb_free\n");
	usb_set_intfdata(interface, NULL);
	spi_tiny_usb_free(priv);

	dev_dbg(&interface->dev, "disconnected\n");
}

