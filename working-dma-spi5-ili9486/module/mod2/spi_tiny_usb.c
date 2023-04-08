#include<linux/cdev.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/usb.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/kref.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/mutex.h>
#include <linux/version.h>
#include <linux/workqueue.h>	//for work_struct
#include <linux/gpio.h>		//for led
#include <linux/gpio/driver.h>
#include <linux/gpio/machine.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/pwm.h>
#include <linux/property.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include "../inc/spi-tiny-usb.h"
#include <linux/usb/ch9.h>
#include <linux/kobject.h>
#include <linux/kdev_t.h>
#include <linux/sysfs.h>
#include <uapi/linux/stat.h>
#define USB_SPI_MINOR_BASE	192
#define MAX_TRANSFER		(PAGE_SIZE - 512)
#define WRITES_IN_FLIGHT	8

#define FLAGS_BEGIN 1
#define FLAGS_END   2

#define VID 0x0403
#define PID 0xc631

static DEFINE_IDA (usbspi_devid_ida);

#define CLASS_NAME	"spi_tiny_usb"
#define NAME		"usbio"

static char *name;
module_param (name, charp, 0000);
MODULE_PARM_DESC (name, "Devicename modalias");

static int noirq = 0;
module_param (noirq, int, 0000);
MODULE_PARM_DESC (noirq, "disable gpio irq support");
//static char poop[] = "spi-petra";

struct spi_tiny_usb
{
	struct usb_interface *interface;
	struct usb_device *usb_dev;
	struct mutex io_mutex;
	unsigned long disconnected:1;
	struct mutex ops_mutex;
	struct mutex spi_buf_lock;
	struct mutex spi_lock;
	int id;
	int index;
	u8 bulk_in;
	u8 bulk_out;
	size_t bulk_in_sz;
	void *bulk_in_buf;
	struct gpio_chip chip;
	bool gpio_init;
	bool hwirq;
	int gpio_irq_map[5];
	struct gpiod_lookup_table *lookup[5];
	struct gpio_desc *ce_gpiod;

	struct gpio_desc *interrupt_gpio;

	struct usb_endpoint_descriptor *int_in_endpoint;
	struct urb *int_in_urb;	/* gpio irq urb */
	unsigned char *int_in_buf;	/* gpio irq rec buffer */
	__u8 int_in_endpointAddr;	/* gpio irq endpoint address */
	struct irq_chip irq;	// chip descriptor for IRQs
	int num;
	uint8_t irq_num;	// todo either enable 3 more interrupts or keep other 3 from reporting irq capabilities
	int irq_base;		// base IRQ allocated
//     struct irq_desc*  irq_descs    [5]; // IRQ descriptors used (irq_num elements)
//    const struct cpumask *aff_mask;   // not currently used but for irq_to_desc'ish like behavior
	int irq_types[5];	// IRQ types (irq_num elements)
	bool irq_enabled[5];	// IRQ enabled flag (irq_num elements)
	int irq_gpio_map[5];	// IRQ to GPIO pin map (irq_num elements)
	int irq_hw;
	const struct usb_device_id *usb_dev_id;
	struct usbspi_intf_info *info;

	struct platform_device *spi_pdev;
	struct gpiod_lookup_table *lookup_cs;

	struct work_struct work;	// todo  remove both work's in favor of either currently implmented  usb-send/rec functions or struct based option
	struct work_struct work2;
	struct usb_anchor submitted;	// todo use as replacement for kill urb etc.
	int errors;
	spinlock_t err_lock;
	struct kref kref;

	u16 last_mode;		//used to compare old/new spi modes if diff update controller
	u32 last_speed_hz;	// same as above but for spi speed
	u16 last_bpw;		// same as two above but for bits per word
	u16 oldlsb;		// same same for msb/lsb



	int oldspihz;
	u16 gbase;
	u8 bufr[4];
};
#define to_usbspi_dev(d) container_of(d, struct spi_tiny_usb, kref)

#define USBSPI_READ_TIMEOUT	5000
#define USBSPI_WRITE_TIMEOUT	5000

struct usbspi_intf_info
{
	int (*probe) (struct usb_interface * interface,
		      const void *plat_data);
	int (*remove) (struct usb_interface * interface);
	const void *plat_data;	/* optional, passed to probe() */
};

static int i2c_gpio_to_irq (struct gpio_chip *chip, unsigned offset);
static struct usb_driver spi_tiny_usb_driver;
static void usbspi_draw_down (struct spi_tiny_usb *dev);



unsigned int GPIO_irqNumber;
EXPORT_SYMBOL (GPIO_irqNumber);

static uint8_t gpio_val = 0;	// brequest
static uint8_t offs = 0;	// windex? I didnt know the actuall names of these when I started, been cp'ing the same code around since
static uint8_t usbval = 0;	// wvalue

// TODO PUT THE FOLLOWING 3 VARIABLES IN spi_tiny_usb
int irqt = 2;
int irqyup = 0;
int edget = GPIO_ACTIVE_LOW;
int gbase = 0;

static void
_gpio_work_job (struct work_struct *work)
{
	struct spi_tiny_usb *sd =
		container_of (work, struct spi_tiny_usb, work);
	mutex_lock (&sd->io_mutex);
//   printk(KERN_ALERT "gpioval i/o: %d \n", gpio_val);
//   printk(KERN_ALERT "usbval i/o: %d \n", usbval);
//   printk(KERN_ALERT "offset i/o: %d \n",offs);
	usb_control_msg (sd->usb_dev,
			 usb_sndctrlpipe (sd->usb_dev, 0),
			 gpio_val, USB_TYPE_VENDOR | USB_DIR_OUT,
			 usbval, offs, NULL, 0, 3000);
	mutex_unlock (&sd->io_mutex);
}

static void
_gpio_work_job2 (struct work_struct *work2)
{
	struct spi_tiny_usb *sd =
		container_of (work2, struct spi_tiny_usb, work2);
	mutex_lock (&sd->io_mutex);
//   printk(KERN_ALERT "Read port i/o: %d \n", offs);
	usb_control_msg (sd->usb_dev,
			 usb_rcvctrlpipe (sd->usb_dev, 0),
			 gpio_val, USB_TYPE_VENDOR | USB_DIR_IN,
			 usbval, offs, (u8 *) sd->bufr, 4, 100);
	mutex_unlock (&sd->io_mutex);
}




static ssize_t i2cspeed_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
struct spi_tiny_usb *data = dev_get_drvdata(dev);
    u16 value;
	u8 ret2;
	u8 ret3;
	ret2 = kstrtou16(buf, 10, &value);
//	printk("value %d\n", value);
	if (value == 100) {
	ret3 = 1;
	} else if (value == 400) {
	ret3 = 2;
	} else if (value == 1000) {
	ret3 = 3;
	} else {
	return -EINVAL;
	}
	mutex_lock (&data->io_mutex);
    usb_control_msg(data->usb_dev,
                   usb_sndctrlpipe(data->usb_dev, 0),
                   81, USB_TYPE_VENDOR | USB_DIR_OUT,
                ret3, ret3,
                  NULL, 0,
                   1000);

	mutex_unlock (&data->io_mutex);

    return count;
}

static ssize_t i2cspeed_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{

struct spi_tiny_usb *data = dev_get_drvdata(dev);
int retval;
int retval2;
u16 ret2 = 0;
char *rxbuf = kmalloc (2, GFP_KERNEL);
if (!rxbuf) {
	return -ENOMEM;
}

	usb_control_msg (data->usb_dev,
			 usb_rcvctrlpipe (data->usb_dev, 0),
			 82, USB_TYPE_VENDOR | USB_DIR_IN,
			 4, 4, rxbuf, 2, 500);

//memcpy (data->bufr, rxbuf, 2);

retval = rxbuf[0];
retval2 = rxbuf[1];
printk ("retval = %d\n", retval);
printk ("retval2 = %d\n", retval2);
if (retval2 == 1) {
	ret2 = 100;
} else if (retval2 == 2) {
	ret2 = 400;
} else if (retval2 == 3) {
	ret2 = 1000;
}
printk ("ret2 = %d\n", ret2);
kfree (rxbuf);

return scnprintf(buf, PAGE_SIZE, "%d\n", ret2);

}
static DEVICE_ATTR_RW(i2cspeed);

//struct class *class_usbio;
//dev_t device_number;
//struct device *device_usbio;

static struct attribute *spi_tiny_usb_attrs[] = {
&dev_attr_i2cspeed.attr,
    NULL
};

struct class *class_usbio;
dev_t device_number;
struct device *device_usbio;

static int create_sysfs_attrs(struct usb_interface *intf)
{
class_usbio= class_create(THIS_MODULE,"usbio");
device_usbio= device_create(class_usbio, NULL, device_number, NULL,"usbi2c");

return 0;
}

static void remove_sysfs_attrs(struct usb_interface *intf)
{
device_remove_file(&intf->dev, &dev_attr_i2cspeed);
}



static void
int_cb (struct urb *urb)
{
	struct spi_tiny_usb *sd = urb->context;
	unsigned long flags;
	char *intrxbuf = kmalloc (4, GFP_KERNEL);
	if (!intrxbuf)
		printk (KERN_ALERT "Failed to create intrxbuf \n");

//   printk(KERN_ALERT "urb interrupt is called \n");
	memcpy (intrxbuf, sd->int_in_buf, 4);
//   printk(KERN_ALERT "received data 0: %d \n", sd->int_in_buf[0]);  // TODO MAKE 4/5 GPIO IRQ COMPAT AND SEPERATE/CALL CORRECTLY THE IRQ FROM HERE
//   printk(KERN_ALERT "received data 1: %d \n", sd->int_in_buf[1]);
//   printk(KERN_ALERT "received data 2: %d \n", sd->int_in_buf[2]);
//   printk(KERN_ALERT "received data 3: %d \n", sd->int_in_buf[3]);

	if (irqyup == 1)
	  {
//       printk(KERN_ALERT "irq alive and fired\n");
		  local_irq_save (flags);
		  generic_handle_irq (GPIO_irqNumber);
		  local_irq_disable ();
		  local_irq_restore (flags);
	  }

	usb_submit_urb (sd->int_in_urb, GFP_KERNEL);
	kfree (intrxbuf);
}

static void
_gpioa_set (struct gpio_chip *chip, unsigned offset, int value)
{
	struct spi_tiny_usb *data = container_of (chip, struct spi_tiny_usb,
						  chip);
//   printk(KERN_INFO "GPIO SET INFO for pin: %d \n", offset);

	usbval = 0;

	offs = offset;
	gpio_val = value;
	schedule_work (&data->work);
}

static int
_gpioa_get (struct gpio_chip *chip, unsigned offset)
{
	struct spi_tiny_usb *data = container_of (chip, struct spi_tiny_usb,
						  chip);

	int retval, retval1, retval2, retval3;
	char *rxbuf = kmalloc (4, GFP_KERNEL);	//4 for 4 gpio for todo grab multi pin states, also seems the usb rx buffer needs to be same size or issues
	if (!rxbuf)
		return -ENOMEM;

//    printk(KERN_INFO "GPIO GET INFO: %d \n", offset);

	usbval = 3;
	offs = offset;
	gpio_val = 1;

	usb_control_msg (data->usb_dev,
			 usb_rcvctrlpipe (data->usb_dev, 0),
			 gpio_val, USB_TYPE_VENDOR | USB_DIR_IN,
			 usbval, offs, rxbuf, 4, 100);


	//THE FOLLOWING IS OVERKILL BUT WHEN YOU FINALLY FIGURE OUT HOW TO ACTUALLY RECEIVE SOME DATA.....
	retval = rxbuf[0];
	retval1 = rxbuf[1];
	retval2 = rxbuf[2];
	retval3 = rxbuf[3];
	/*   printk("buf0 =  %d \n", retval);
	   printk("buf1 =  %d \n", retval1);
	   printk("buf2 =  %d \n", retval2);
	   printk("buf3 =  %d \n", retval3);
	 */
	kfree (rxbuf);

	return retval1 - 3;
}

static int
_direction_output (struct gpio_chip *chip, unsigned offset, int value)
{
	struct spi_tiny_usb *data = container_of (chip, struct spi_tiny_usb,
						  chip);
	//  printk("Setting pin to OUTPUT gppio-offset %d\n", offset);

	usbval = 2;
	offs = offset;

	schedule_work (&data->work);

	return 0;
}

static int
_direction_input (struct gpio_chip *chip, unsigned offset)
{
	struct spi_tiny_usb *data = container_of (chip, struct spi_tiny_usb,
						  chip);

//   printk("Setting pin to INPUT gpio-offset %d\n", offset);

	usbval = 1;
	offs = offset;

	schedule_work (&data->work);

	return 0;
}

static int
i2c_gpio_to_irq (struct gpio_chip *chip, unsigned offset)
{
	struct spi_tiny_usb *data = container_of (chip, struct spi_tiny_usb,
						  chip);
	GPIO_irqNumber = irq_create_mapping (data->chip.irq.domain, offset);
	pr_info ("GPIO_irqNumber = %d\n", GPIO_irqNumber);

	return GPIO_irqNumber;
}

static void
usb_gpio_irq_enable (struct irq_data *irqd)
{
	struct spi_tiny_usb *dev = irq_data_get_irq_chip_data (irqd);
	irqyup = 1;
	/* Is that needed? */
	if (dev->irq_enabled[4])
		return;

	dev->irq_enabled[4] = true;
}

void
set_irq_disabled (struct irq_data *irqd)
{
	struct spi_tiny_usb *dev = irq_data_get_irq_chip_data (irqd);
	irqyup = 0;
	if (!dev->irq_enabled[4])
		return;

	dev->irq_enabled[4] = false;
}

static void
usb_gpio_irq_disable (struct irq_data *irqd)
{
//    struct gpio_chip *chip = irq_data_get_irq_chip_data(irqd);
//    struct spi_tiny_usb *data = container_of(chip, struct spi_tiny_usb, chip);
	/* Is that needed? */
//      if (!chip->irq_enabled[4])
//              return;

	usbval = 9;
	offs = 1;
	gpio_val = 9;
//   schedule_work(&data->work);
	set_irq_disabled (irqd);
}

static int
usbirq_irq_set_type (struct irq_data *irqd, unsigned type)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data (irqd);
	struct spi_tiny_usb *data = container_of (chip, struct spi_tiny_usb,
						  chip);
//    int pin = irqd_to_hwirq(irqd);
//    pr_info("irq pin = %d\n", pin);
	irqt = type;
	irqyup = 1;
	usb_gpio_irq_enable (irqd);
//    pr_info("setting irq type GPIO_irqNumber = %d\n", GPIO_irqNumber);
	switch (type)
	  {
	  case IRQ_TYPE_NONE:
		  usbval = 9;
		  offs = 1;
		  gpio_val = 9;
		  schedule_work (&data->work);
		  break;
	  case IRQ_TYPE_LEVEL_HIGH:
		  usbval = 9;
		  offs = 2;
		  gpio_val = 9;
		  schedule_work (&data->work);
		  break;
	  case IRQ_TYPE_LEVEL_LOW:
		  usbval = 9;
		  offs = 3;
		  gpio_val = 9;
		  schedule_work (&data->work);
		  break;
	  case IRQ_TYPE_EDGE_BOTH:
		  usbval = 9;
		  offs = 4;
		  gpio_val = 9;
		  schedule_work (&data->work);
		  break;
	  case IRQ_TYPE_EDGE_RISING:
		  usbval = 9;
		  offs = 5;
		  gpio_val = 9;
		  edget = GPIO_ACTIVE_LOW;
		  schedule_work (&data->work);
		  break;
	  case IRQ_TYPE_EDGE_FALLING:
		  usbval = 9;
		  offs = 6;
		  gpio_val = 9;
		  schedule_work (&data->work);
		  edget = GPIO_ACTIVE_HIGH;
		  break;
	  default:
		  return -EINVAL;
	  }

	return 0;
}

//TODO dynamically append to gpionames for multi-instance
// const char *gpio_names[] = { "CSpi1", "PC7", "PG7", "PJ11", "IRQpk1" };
const char *gpio_names[] = { "spi-cs", "ce", "csn", "PJ11", "interrupt" };

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,18,0)	//dunno actual kernel ver when it switched but somewhere around then
static const struct irq_chip usb_gpio_irqchip = {
	.name = "usbgpio-irq",
	.irq_enable = usb_gpio_irq_enable,
	.irq_disable = usb_gpio_irq_disable,
	.irq_set_type = usbirq_irq_set_type,
	.flags = IRQCHIP_IMMUTABLE, GPIOCHIP_IRQ_RESOURCE_HELPERS,
};
#endif

static void
spi_tiny_usb_free (struct spi_tiny_usb *priv)
{
	usb_put_dev (priv->usb_dev);
	kfree (priv);
}

// static int usb_read(struct spi_tiny_usb *dev, int cmd, int value, int index, void *data, int len);  //WILL NEED/USE AGAIN IM SURE
static int usb_write (struct spi_tiny_usb *dev, int cmd, int value, int index,
		      void *data, int len);
static int usbspi_bulk_xfer (struct usb_interface *interface,
			     struct bulk_desc *desc);
static int usbspi_write_data (struct usb_interface *interface,
			      const char *buf, size_t len);
static int usbspi_read_data (struct usb_interface *interface, void *buf,
			     size_t len);
//void csset(struct spi_device *spi, bool enable);


static int
usbspi_write_data (struct usb_interface *interface,
		   const char *buf, size_t len)
{
	struct bulk_desc desc;
	int ret;
	desc.act_len = 0;
	desc.dir_out = true;
	desc.data = (char *) buf;
	desc.len = len;
	desc.timeout = USBSPI_WRITE_TIMEOUT;
	ret = usbspi_bulk_xfer (interface, &desc);
	if (ret < 0)
		return ret;
	return desc.act_len;
}



static int
usbspi_read_data (struct usb_interface *interface, void *buf, size_t len)
{
	struct spi_tiny_usb *priv = usb_get_intfdata (interface);
	struct bulk_desc desc;
	int ret;

	desc.act_len = 0;
	desc.dir_out = false;
	desc.data = priv->bulk_in_buf;

	desc.len = min_t (size_t, len, priv->bulk_in_sz);
	desc.timeout = USBSPI_READ_TIMEOUT;

	ret = usbspi_bulk_xfer (interface, &desc);
	if (ret)
		return ret;

	ret = desc.act_len;
	if (ret > len)
		ret = len;
	memcpy (buf, desc.data, ret);
	return ret;
}


static int
usbspi_bulk_xfer (struct usb_interface *interface, struct bulk_desc *desc)
{
	struct spi_tiny_usb *priv = usb_get_intfdata (interface);
	struct usb_device *usb_dev = priv->usb_dev;
	unsigned int pipe;
	int ret;
	mutex_lock (&priv->io_mutex);
	if (!priv->interface)
	  {
		  printk ("failed to get interface \r\n");
		  ret = -ENODEV;
		  goto exit;
	  }

	if (desc->dir_out)
		pipe = usb_sndbulkpipe (usb_dev, priv->bulk_out);
	else
		pipe = usb_rcvbulkpipe (usb_dev, priv->bulk_in);
	ret = usb_bulk_msg (usb_dev, pipe, desc->data, desc->len,
			    &desc->act_len, desc->timeout);
	if (ret)
		dev_dbg (&usb_dev->dev, "bulk msg failed: %d\n", ret);

      exit:
	mutex_unlock (&priv->io_mutex);
	return ret;
}

static void
usbspi_lock (struct usb_interface *intf)
{
	struct spi_tiny_usb *priv = usb_get_intfdata (intf);

	mutex_lock (&priv->ops_mutex);
}

static void
usbspi_unlock (struct usb_interface *intf)
{
	struct spi_tiny_usb *priv = usb_get_intfdata (intf);

	mutex_unlock (&priv->ops_mutex);
}

/*
static void setcs(struct spi_device *spi, bool enable) {
struct spi_tiny_usb *priv = spi_master_get_devdata(spi->master);
//struct spi_device *spidev = to_spi_device(priv);
if (enable == false) {
usb_write(priv, 72, 0, 0, NULL, 0);
printk("setting cs high\n");
  }
//csset(priv->spidev, enable);

if (enable == true) {
usb_write(priv, 71, 0, 0, NULL, 0);
printk("setting cs low\n");
}
}
*/

static void
usbspi_config (struct usb_interface *dev, int cmd, int value, int index,
	       void *data, int len)
{
	struct spi_tiny_usb *priv = usb_get_intfdata (dev);
	struct usb_device *usb_dev = priv->usb_dev;
	int ret;
	mutex_lock (&priv->io_mutex);
	ret = usb_control_msg (usb_dev, usb_sndctrlpipe (usb_dev, 0),
			       cmd, USB_TYPE_VENDOR | USB_RECIP_INTERFACE,
			       value, index, data, len, 2000);

	if (ret < 0)
		;

	mutex_unlock (&priv->io_mutex);
	return;
}

/*
static void usbspi_config(struct spi_tiny_usb *dev, int cmd, int value, int index)
{
	struct spi_tiny_usb *priv = usb_get_intfdata(interface);
	struct usb_device *usb_dev = priv->usb_dev;
	unsigned int pipe;
	int ret;

	mutex_lock(&priv->io_mutex);
	if (!priv->interface) {
		ret = -ENODEV;
		goto exit;
	}

	if (!desc->data && desc->size)
		desc->data = priv->bulk_in_buf;

	if (desc->dir_out)
		pipe = usb_sndctrlpipe(usb_dev, 0);
	else
		pipe = usb_rcvctrlpipe(usb_dev, 0);

	ret = usb_control_msg(usb_dev, pipe, desc->request, desc->requesttype,
			      desc->value, desc->index, desc->data, desc->size,
			      desc->timeout);
	if (ret < 0)
		dev_dbg(&usb_dev->dev, "ctrl msg failed: %d\n", ret);
exit:
	mutex_unlock(&priv->io_mutex);

}
*/

static const struct usbspi_intf_ops usbspi_intf_ops = {
//      .ctrl_xfer = usbspi_bulk_xfer,
	.bulk_xfer = usbspi_bulk_xfer,
	.read_data = usbspi_read_data,
	.write_data = usbspi_write_data,
	.lock = usbspi_lock,
	.unlock = usbspi_unlock,
	.setup_data = usbspi_config,
};


/* ----- begin of spi layer ---------------------------------------------- */

static int
usbspi_intf_spi_remove (struct usb_interface *intf)
{
	struct spi_tiny_usb *priv = usb_get_intfdata (intf);
	struct device *dev = &intf->dev;

	dev_dbg (dev, "%s: spi pdev %p\n", __func__, priv->spi_pdev);
	gpiod_remove_lookup_table (priv->lookup_cs);
	priv->irq_enabled[4] = false;
	platform_device_unregister (priv->spi_pdev);
	return 0;
}

static const struct usb_spi_platform_data usbspi_bus_plat_data;

static int usbspi_intf_spi_probe (struct usb_interface *interface,
				  const void *plat_data);

static const struct usbspi_intf_info usbspi_bus_intf_info = {
	.probe = usbspi_intf_spi_probe,
	.remove = usbspi_intf_spi_remove,
	.plat_data = &usbspi_bus_plat_data,
};

static struct usb_device_id spi_tiny_usb_table[] = {
	{USB_DEVICE (VID, PID),
	 .driver_info = (kernel_ulong_t) & usbspi_bus_intf_info},
	{}
};

MODULE_DEVICE_TABLE (usb, spi_tiny_usb_table);

static int
usb_write (struct spi_tiny_usb *dev, int cmd, int value, int index,
	   void *data, int len)
{
	int ret;
	mutex_lock (&dev->io_mutex);
	ret = usb_control_msg (dev->usb_dev,
			       usb_sndctrlpipe (dev->usb_dev, 0), cmd,
			       USB_TYPE_VENDOR | USB_RECIP_INTERFACE, value,
			       index, data, len, 2000);
	mutex_unlock (&dev->io_mutex);
	return ret;
}

struct property_entry mcp2515_properties[] = {
	PROPERTY_ENTRY_U32 ("clock-frequency", 0xF42400),
//      PROPERTY_ENTRY_U32("xceiver", 1),
//      PROPERTY_ENTRY_U32("gpio-controller", 3),
	{}
};

struct property_entry w25q32_properties[] = {
//      PROPERTY_ENTRY_U32("spi-max-frequency", 10000000),
//      PROPERTY_ENTRY_BOOL("m25p,read"),
//      PROPERTY_ENTRY_U32("gpio-controller", 3),
	{}
};

static const struct software_node mcp2515_node = {
	.properties = mcp2515_properties,
};


static const struct software_node w25q32_node = {
	.properties = w25q32_properties,
};

/*
static int usb_spi_setup(struct spi_device *spi)
{
	struct spi_tiny_usb *priv = spi_master_get_devdata(spi->master);

	dev_info(&priv->usb_dev->dev, "usb_spi_setup() for %s on CS %d",
	         spi->modalias, spi->chip_select);

	usb_write(priv, 74, 6, 6, NULL, 0);

	return 0;
}
*/

static const struct property_entry nrf24_properties[] = {
	PROPERTY_ENTRY_U32 ("interrupts", 4),
	{}
};

static const struct software_node nrf24_node = {
	.properties = nrf24_properties,
};

static struct dev_io_desc_data usb_spi_bus_dev_io[] = {
	{"interrupts", 4, GPIO_ACTIVE_HIGH},
	{"ce", 1, GPIOD_OUT_HIGH},
	{"csn", 2, GPIO_ACTIVE_LOW},
};

static const struct usbspi_dev_data usb_spi_dev_data[] = {
	{
	 .magic = USBSPI_IO_DESC_MAGIC,
	 .desc = usb_spi_bus_dev_io,
	 .desc_len = ARRAY_SIZE (usb_spi_bus_dev_io),
	 },
};

static struct spi_board_info usb_spi_bus_info[] = {
	{
//    .modalias = "yx240qv29",
//      .modalias       = "ili9341",
//    .modalias = "w25q32",
	 .modalias = "mcp2515",
//    .modalias = "spi-petra",
//    .modalias = "nrf24",
//      .modalias       = "ili9341",
	 .mode = SPI_MODE_0,
//    .mode             = SPI_MODE_0 | SPI_LSB_FIRST | SPI_CS_HIGH,
	 .max_speed_hz = 4000000,
//    .max_speed_hz     = 30000000,
	 .bus_num = 0,
	 .chip_select = 0,
	 .platform_data = usb_spi_dev_data,
	 //   .swnode   = &nrf24_node,
	 .swnode = &mcp2515_node,
//      .s ,    //changed from properties to swnode i dunno aroun kernel 5.15ish
//    .properties       = mcp2515_properties,
//      .swnode  =  &mcp2515_node,
//      .irq     = 0,
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

#define SPI_INTF_DEVNAME	"dtbspi_plat_usb"

static const struct usb_spi_platform_data usbspi_bus_plat_data = {
	.ops = &usbspi_intf_ops,
	.spi_info = usb_spi_bus_info,
	.spi_info_len = ARRAY_SIZE (usb_spi_bus_info),
};

static struct platform_device *
usbspi_dev_register (struct spi_tiny_usb *priv,
		     const struct usb_spi_platform_data *pd)
{
	struct device *parent = &priv->interface->dev;
	struct platform_device *pdev;
	struct gpiod_lookup_table *lookup;
//	struct device *dev = &pd->dev;
	size_t lookup_size, tbl_size;
	int i, ret;

//    printk("spi_tiny_usb Start of platform probe\n");

	pd->spi_info[0].irq = GPIO_irqNumber;
	irq_set_irq_type (GPIO_irqNumber, irqt);

	pdev = platform_device_alloc (SPI_INTF_DEVNAME, 0);
	if (!pdev)
		printk ("spi_tiny_usb failed to add platform device\n");

	pdev->dev.parent = parent;
	pdev->dev.fwnode = NULL;

	priv->spi_pdev = pdev;

	tbl_size = pd->spi_info_len + 1;
	lookup_size =
		sizeof (*lookup) + tbl_size * sizeof (struct gpiod_lookup);
	lookup = devm_kzalloc (parent, lookup_size, GFP_KERNEL);
	if (!lookup)
	  {
		  printk ("failed lookuo_size\n");
		  ret = -ENOMEM;
		  goto err;
	  }

	for (i = 0; i < pd->spi_info_len; i++)
	  {
		  printk ("INFO: %s cs %d\n",
			  pd->spi_info[i].modalias,
			  pd->spi_info[i].chip_select);
	  }

	ret = platform_device_add_data (pdev, pd, sizeof (*pd));
	if (ret)
		goto err;

	pdev->id = priv->id;

	lookup->dev_id = devm_kasprintf (parent, GFP_KERNEL, "%s.%d",
					 pdev->name, pdev->id);
	if (!lookup->dev_id)
	  {
		  ret = -ENOMEM;
		  goto err;
	  }

	for (i = 0; i < pd->spi_info_len; i++)
	  {
		  lookup->table[i].key = priv->chip.label;
		  lookup->table[i].chip_hwnum = pd->spi_info[i].chip_select;
		  lookup->table[i].idx = i;
		  lookup->table[i].con_id = NULL;
		  if (pd->spi_info[i].mode & SPI_CS_HIGH)
			  lookup->table[i].flags = GPIO_ACTIVE_HIGH;
		  else
			  lookup->table[i].flags = GPIO_ACTIVE_LOW;
	  }

	priv->lookup_cs = lookup;

	gpiod_add_lookup_table (priv->lookup_cs);
	ret = platform_device_add (pdev);
	if (ret < 0)
		printk ("error onn platform_device_add \n");

	dev_dbg (&pdev->dev, "%s done\n", __func__);
	return pdev;

      err:
	platform_device_put (pdev);
	return ERR_PTR (ret);
}

static int
usbspi_intf_spi_probe (struct usb_interface *interface, const void *plat_data)
{
	struct spi_tiny_usb *priv = usb_get_intfdata (interface);
	struct device *dev = &interface->dev;
	struct platform_device *pdev;
	int inf;

	inf = priv->interface->cur_altsetting->desc.bInterfaceNumber;
	pdev = usbspi_dev_register (priv, plat_data);
	if (IS_ERR (pdev))
	  {
		  dev_err (dev, "%s: Can't create USBSPI device %ld\n",
			   __func__, PTR_ERR (pdev));
		  return PTR_ERR (pdev);
	  }
	priv->spi_pdev = pdev;
	return 0;
}

static int
spi_tiny_usb_probe (struct usb_interface *interface,
		    const struct usb_device_id *id)
{
	struct device *dev = &interface->dev;
	struct spi_tiny_usb *priv;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint, *bulk_in, *bulk_out;
	struct usbspi_intf_info *info;
	struct gpio_irq_chip *girq;
	char **names, *label;
	int ret = 0;
	u16 version;
	int retval;
	int inf;
	int i;
	int rc;

	dev_dbg (&interface->dev, "probing usb device\n");

	priv = kzalloc (sizeof (*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	priv->usb_dev = usb_get_dev (interface_to_usbdev (interface));
	inf = interface->cur_altsetting->desc.bInterfaceNumber;
	priv->interface = interface;
	iface_desc = interface->cur_altsetting;

	if (inf == 0)
	  {
		  return -ENODEV;
	  }

	if (inf == 1)
	  {
		  priv->usb_dev =
			  usb_get_dev (interface_to_usbdev (interface));
		  endpoint = &iface_desc->endpoint[1].desc;
		  retval = usb_find_int_in_endpoint (interface->
						     cur_altsetting,
						     &endpoint);
		  if (retval)
		    {
			    printk ("USBSPI Could not find int-in endpoint\n");
			    return retval;
		    }
		  priv->int_in_endpoint = endpoint;
		  priv->int_in_endpointAddr = endpoint->bEndpointAddress;
		  priv->int_in_endpoint->wMaxPacketSize = 0x10;
		  if (noirq == 0)
		    {
			    priv->int_in_urb = usb_alloc_urb (0, GFP_KERNEL);
			    priv->int_in_buf =
				    kmalloc (le16_to_cpu
					     (priv->int_in_endpoint->
					      wMaxPacketSize), GFP_KERNEL);
			    usb_fill_int_urb (priv->int_in_urb, priv->usb_dev,
					      usb_rcvintpipe (priv->usb_dev,
							      priv->
							      int_in_endpoint->
							      bEndpointAddress),
					      priv->int_in_buf,
					      le16_to_cpu (priv->
							   int_in_endpoint->
							   wMaxPacketSize),
					      int_cb, priv,
					      (priv->int_in_endpoint->
					       bInterval));

			    usb_set_intfdata (interface, priv);
			    printk (KERN_INFO "usb gpio irq is connected \n");
			    i = usb_submit_urb (priv->int_in_urb, GFP_KERNEL);
			    if (i)
			      {
				      printk (KERN_ALERT
					      "Failed to submit urb \n");
			      }
		    }		// noirq

		  INIT_WORK (&priv->work, _gpio_work_job);
		  INIT_WORK (&priv->work2, _gpio_work_job2);

		  label = devm_kasprintf (dev, GFP_KERNEL, "usb-tiny-usb.%d",
					  priv->id);
		  if (!label)
			  return -ENOMEM;


		  priv->chip.label = label;
		  priv->chip.parent = dev;
		  priv->chip.owner = THIS_MODULE;
		  priv->chip.base = -1;
		  priv->chip.ngpio = 5;
		  priv->chip.can_sleep = true;

		  priv->chip.set = _gpioa_set;
		  priv->chip.get = _gpioa_get;
		  priv->chip.direction_input = _direction_input;
		  priv->chip.direction_output = _direction_output;
		  priv->chip.to_irq = i2c_gpio_to_irq;
		  priv->chip.names = gpio_names;
		  if (noirq == 0)
		    {
#if LINUX_VERSION_CODE <= KERNEL_VERSION(5,18,0)	//yeah sure well call it 5.18
			    priv->irq.name = "usbgpio-irq";
			    priv->irq.irq_set_type = usbirq_irq_set_type;
			    priv->irq.irq_enable = usb_gpio_irq_enable;
			    priv->irq.irq_disable = usb_gpio_irq_disable;
			    .flags = IRQCHIP_IMMUTABLE,
				    GPIOCHIP_IRQ_RESOURCE_HELPERS, girq =
				    &priv->chip.irq;
			    girq->chip = &priv->irq;
#else
			    girq = &priv->chip.irq;
			    gpio_irq_chip_set_chip (girq, &usb_gpio_irqchip);
#endif
			    girq->parent_handler = NULL;
			    girq->num_parents = 0;
			    girq->parents = NULL;
			    girq->default_type = IRQ_TYPE_NONE;
			    girq->handler = handle_simple_irq;
			    rc = irq_alloc_desc (0);
			    if (rc < 0)
			      {
				      ;
				      return rc;
			      }

			    priv->irq_base = rc;

		    }		//end of noirq

		  names = devm_kcalloc (dev, priv->chip.ngpio,
					sizeof (char *), GFP_KERNEL);
		  if (!names)
			  return -ENOMEM;


		  priv->chip.names = gpio_names;

		  mutex_init (&priv->io_mutex);
		  mutex_init (&priv->ops_mutex);
		  mutex_init (&priv->spi_buf_lock);



		  ret = devm_gpiochip_add_data (dev, &priv->chip, priv);
		  if (ret < 0)
		    {
			    printk ("Failed to add USB GPIO chip: %d\n", ret);
			    return ret;
		    }
		  priv->usb_dev_id = id;
		  priv->index = 1;
		  priv->interface = interface;
		  priv->info = (struct usbspi_intf_info *) id->driver_info;
		  info = priv->info;

		  if (noirq == 0)
		    {

			    gbase = (cpu_to_le16 (priv->chip.base));

// Some sorta thing where interrupt irq gpio seems to want to be claimed and released before
// working with external drivers. This can also be done via /sys and userland. Probly something im missing forcing this
			    priv->interrupt_gpio =
				    gpiochip_request_own_desc (&priv->chip, 4,
							       "interrupt",
							       GPIOD_IN,
							       edget);

			    i2c_gpio_to_irq (&priv->chip, 4);
			    irq_set_irq_type (GPIO_irqNumber, irqt);
			    gpiochip_free_own_desc (priv->interrupt_gpio);

		    }
// the nrf24 driver im using seemed to have issues setting gpio to output needs retesting
/*
        priv->ce_gpiod = gpiochip_request_own_desc(&priv->chip,
                                                1,
                                                "ce",
                                                GPIO_LOOKUP_FLAGS_DEFAULT,
                                                GPIO_ACTIVE_HIGH);

        gpiod_direction_output(priv->ce_gpiod, true);
        gpiochip_free_own_desc(priv->ce_gpiod);
*/
		  retval = usb_find_common_endpoints (interface->
						      cur_altsetting,
						      &bulk_in, &bulk_out,
						      NULL, NULL);
		  if (retval)
		    {
			    dev_err (&interface->dev,
				     "Could not find both bulk-in and bulk-out endpoints\n");
			    goto error;
		    }
		  priv->bulk_in_sz = usb_endpoint_maxp (bulk_in);
		  priv->bulk_in = bulk_in->bEndpointAddress;
		  priv->bulk_out = bulk_out->bEndpointAddress;
		  device_create_file(&interface->dev, &dev_attr_i2cspeed);
		  usb_set_intfdata (interface, priv);
		  priv->bulk_in_buf =
			  devm_kmalloc (dev, priv->bulk_in_sz, GFP_KERNEL);
		  if (!priv->bulk_in_buf)
			  return -ENOMEM;

		  priv->usb_dev =
			  usb_get_dev (interface_to_usbdev (interface));
		  priv->id =
			  ida_simple_get (&usbspi_devid_ida, 0, 0,
					  GFP_KERNEL);
		  if (priv->id < 0)
			  return priv->id;

		  version = le16_to_cpu (priv->usb_dev->descriptor.bcdDevice);
		  dev_info (&interface->dev,
			    "version %x.%02x found at bus %03d address %03d\n",
			    version >> 8, version & 0xff,
			    priv->usb_dev->bus->busnum,
			    priv->usb_dev->devnum);

		  dev_info (&interface->dev,
			    "connected spi-tiny-usb device\n");

		  if (noirq == 0)
		    {
			    irq_set_irq_type (GPIO_irqNumber, irqt);
		    }

		  if (info->probe)
		    {
			    ret = info->probe (interface, info->plat_data);
			    if (ret < 0)
				    printk ("info->probe fail\n");
		    }

     create_sysfs_attrs(interface);
 //      kernfs_create_link(device_usbio, i2cspeed, i2cspeed);
	  }

	return 0;


      error:
	printk (KERN_DEBUG "spi-tiny-usb error\n");
	if (priv)
		spi_tiny_usb_free (priv);

	return ret;
}

void spi_tiny_usb_remove (struct spi_tiny_usb *priv);
void spi_tiny_usb_gpio_remove (struct spi_tiny_usb *priv);

void
spi_tiny_usb_remove (struct spi_tiny_usb *priv)
{
	usb_kill_urb (priv->int_in_urb);
	usb_free_urb (priv->int_in_urb);
}

void
spi_tiny_usb_gpio_remove (struct spi_tiny_usb *priv)
{
	if (!priv->gpio_init)
		return;


	gpiochip_remove (&priv->chip);
//      if (noirq == 0) {
//      irq_free_desc(&priv->chip.irq);
//      }
}


static void
spi_tiny_usb_disconnect (struct usb_interface *interface)
{
	struct spi_tiny_usb *priv = usb_get_intfdata (interface);
	const struct usbspi_intf_info *info;
	info = (struct usbspi_intf_info *) priv->usb_dev_id->driver_info;
	if (info && info->remove)
		info->remove (interface);
	spi_tiny_usb_remove (priv);
	spi_tiny_usb_gpio_remove (priv);
	mutex_lock (&priv->io_mutex);
	remove_sysfs_attrs(interface);

	device_destroy(class_usbio, device_number);
    class_destroy(class_usbio);

	usb_set_intfdata (priv->interface, NULL);
	mutex_unlock (&priv->io_mutex);
	usb_put_dev (priv->usb_dev);
	ida_simple_remove (&usbspi_devid_ida, priv->id);
	kfree (priv);
}

static void
usbspi_draw_down (struct spi_tiny_usb *dev)
{
	;
}


static int
usbspi_suspend (struct usb_interface *interface, pm_message_t message)
{
	struct spi_tiny_usb *dev = usb_get_intfdata (interface);

	if (!dev)
		return 0;
	usbspi_draw_down (dev);
	return 0;
}

static int
usbspi_resume (struct usb_interface *interface)
{
	return 0;
}

static int
usbspi_pre_reset (struct usb_interface *interface)
{
	struct spi_tiny_usb *dev = usb_get_intfdata (interface);

	mutex_lock (&dev->io_mutex);
	usbspi_draw_down (dev);

	return 0;
}

static int
usbspi_post_reset (struct usb_interface *interface)
{
	struct spi_tiny_usb *dev = usb_get_intfdata (interface);

	dev->errors = -EPIPE;
	mutex_unlock (&dev->io_mutex);

	return 0;
}


static struct usb_driver spi_tiny_usb_driver = {
	.name = "spi-tiny-usb",
	.probe = spi_tiny_usb_probe,
	.disconnect = spi_tiny_usb_disconnect,
	.suspend = usbspi_suspend,
	.resume = usbspi_resume,
	.pre_reset = usbspi_pre_reset,
	.post_reset = usbspi_post_reset,
	.id_table = spi_tiny_usb_table,
};

module_usb_driver (spi_tiny_usb_driver);

/* ----- end of usb layer ------------------------------------------------ */

MODULE_AUTHOR ("Ben Maddocks <bm16ton@gmail.com>");
MODULE_DESCRIPTION ("spi-tiny-usb driver 16ton edition");
MODULE_LICENSE ("GPL");
