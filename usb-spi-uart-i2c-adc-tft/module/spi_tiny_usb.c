
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
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
#include <linux/workqueue.h> //for work_struct
#include <linux/gpio.h> //for led
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
#include "spi-tiny-usb.h"

#define USB_SPI_MINOR_BASE	192
#define MAX_TRANSFER		(PAGE_SIZE - 512)
#define WRITES_IN_FLIGHT	8


#define USB_CMD_WRITE       0
#define USB_CMD_READ        1
#define USB_CMD_GPIO_OUTPUT 2
#define USB_CMD_GPIO_INPUT  3
#define USB_CMD_GPIO_SET    4
#define USB_CMD_GPIO_GET    5
#define CMD_HZ              7
#define CMD_MODE            8
#define CMD_BPW             9
#define CMD_LSB             10

#define FLAGS_BEGIN 1
#define FLAGS_END   2

#define VID 0x0403
#define PID 0xc631

static DEFINE_IDA(usbspi_devid_ida);


static char *name;
module_param(name, charp, 0000);
MODULE_PARM_DESC(name,
		 "Devicename modalias");

static int noirq = 0;
module_param(noirq, int, 0000);
MODULE_PARM_DESC(noirq,
		 "disable gpio irq support");
//static char poop[] = "spi-petra";

struct spi_tiny_usb {
    struct usb_interface *interface;
	struct usb_device *usb_dev;
	struct mutex		io_mutex;
	unsigned long		disconnected:1;
	struct mutex		ops_mutex;
	int			id;
	int			index;
	u8      			bulk_in;
	u8      			bulk_out;
	size_t  			bulk_in_sz;
	void    			*bulk_in_buf;
	struct gpio_chip chip; //this is our GPIO chip
	bool gpio_init;
    bool    hwirq;
    int                      gpio_irq_map[5];
    struct gpiod_lookup_table *lookup[5];
    struct gpio_desc *ce_gpiod;
//    struct gpio_desc *cs_gpiod;
    struct gpio_desc *interrupt_gpio;

    struct usb_endpoint_descriptor *int_in_endpoint;
	struct urb		*int_in_urb;		/* gpio irq urb */
	unsigned char           *int_in_buf;	/* gpio irq rec buffer */
	__u8			int_in_endpointAddr;	/* gpio irq endpoint address */
    struct irq_chip   irq;                                // chip descriptor for IRQs
    int               num;
    uint8_t           irq_num;                            // todo either enable 3 more interrupts or keep other 3 from reporting irq capabilities
    int               irq_base;                           // base IRQ allocated
//     struct irq_desc*  irq_descs    [5]; // IRQ descriptors used (irq_num elements)
    const struct cpumask *aff_mask;   // not currently used but for irq_to_desc'ish like behavior
     int               irq_types    [5]; // IRQ types (irq_num elements)
     bool              irq_enabled  [5]; // IRQ enabled flag (irq_num elements)
     int               irq_gpio_map [5]; // IRQ to GPIO pin map (irq_num elements)
     int               irq_hw;
	const struct usb_device_id	*usb_dev_id;
	struct usbspi_intf_info		*info;
	struct spi_master *master;
	struct platform_device		*spi_pdev;
	struct gpiod_lookup_table	*lookup_cs;
	struct spi_controller *spimaster;
	struct spi_device *spidev;
	struct spi_board_info spiinfo;
	struct uio_info *uio;
	struct work_struct work;  // todo  remove both work's in favor of either currently implmented  usb-send/rec functions or struct based option
    struct work_struct work2;
	struct usb_anchor	submitted;	// todo use as replacement for kill urb etc.
	int			errors;
	spinlock_t		err_lock;
	struct kref		kref;

	wait_queue_head_t	bulk_in_wait;

	u8 txrx_cmd;  //todo despite usb2 simplex add a txrx function
	u8 rx_cmd;
	u8 tx_cmd;
	u16 last_mode;   //used to compare old/new spi modes if diff update controller
	u32 last_speed_hz; // same as above but for spi speed
	u16 last_bpw; // same as two above but for bits per word
	u16 oldlsb; // same same for msb/lsb


	int oldspihz;
	u16 gbase;
	u8 bufr[4];
};
#define to_usbspi_dev(d) container_of(d, struct spi_tiny_usb, kref)

#define USBSPI_READ_TIMEOUT	5000
#define USBSPI_WRITE_TIMEOUT	5000

struct usbspi_intf_info {
	int (*probe)(struct usb_interface *interface, const void *plat_data);
	int (*remove)(struct usb_interface *interface);
	const void *plat_data; /* optional, passed to probe() */
};


/*
struct spi_gpio {
	unsigned int hwnum;
	const char *label;
	enum gpiod_flags dflags;
};

static const struct spi_gpio spi_gpio_cs[4] = {
	{ 0,  "CS0", GPIOD_OUT_HIGH },
	{ 1,  "CS1", GPIOD_OUT_HIGH },
	{ 2,  "CS2", GPIOD_OUT_HIGH },
	{ 4,  "CS3", GPIOD_OUT_HIGH }
};
*/
static int i2c_gpio_to_irq(struct gpio_chip *chip, unsigned offset);
static struct usb_driver spi_tiny_usb_driver;
static void usbspi_draw_down(struct spi_tiny_usb *dev);


unsigned int GPIO_irqNumber;

static uint8_t gpio_val = 0;      // brequest
static uint8_t offs = 0;          // windex? I didnt know the actuall names of these when I started, been cp'ing the same code around since
static uint8_t usbval = 0;        // wvalue

// TODO PUT THE FOLLOWING 3 VARIABLES IN spi_tiny_usb
int irqt = 2;
int irqyup = 0;
int edget = GPIO_ACTIVE_HIGH;
int gbase = 0;

static void
_gpio_work_job(struct work_struct *work)
{
   struct spi_tiny_usb *sd = container_of(work, struct spi_tiny_usb, work);
mutex_lock(&sd->io_mutex);
   printk(KERN_ALERT "gpioval i/o: %d \n", gpio_val);
   printk(KERN_ALERT "usbval i/o: %d \n", usbval);
   printk(KERN_ALERT "offset i/o: %d \n",offs);
   usb_control_msg(sd->usb_dev,
                   usb_sndctrlpipe(sd->usb_dev, 0),
                   gpio_val, USB_TYPE_VENDOR | USB_DIR_OUT,
                   usbval, offs,
                   NULL, 0,
                   3000);
mutex_unlock(&sd->io_mutex);
}

static void
_gpio_work_job2(struct work_struct *work2)
{
   struct spi_tiny_usb *sd = container_of(work2, struct spi_tiny_usb, work2);
mutex_lock(&sd->io_mutex);
   printk(KERN_ALERT "Read port i/o: %d \n", offs);
   usb_control_msg(sd->usb_dev,
                   usb_rcvctrlpipe(sd->usb_dev, 0),
                   gpio_val, USB_TYPE_VENDOR | USB_DIR_IN,
                   usbval, offs,
                   (u8 *)sd->bufr, 4,
                   100);
mutex_unlock(&sd->io_mutex);
}

static void
int_cb(struct urb *urb)
{
   struct spi_tiny_usb *sd = urb->context;
   unsigned long flags;
   char *intrxbuf = kmalloc(4, GFP_KERNEL);
   if (!intrxbuf)
		printk(KERN_ALERT "Failed to create intrxbuf \n");

   printk(KERN_ALERT "urb interrupt is called \n");
   memcpy(intrxbuf, sd->int_in_buf, 4);
   printk(KERN_ALERT "received data 0: %d \n", sd->int_in_buf[0]);  // TODO MAKE 4/5 GPIO IRQ COMPAT AND SEPERATE/CALL CORRECTLY THE IRQ FROM HERE
   printk(KERN_ALERT "received data 1: %d \n", sd->int_in_buf[1]);
   printk(KERN_ALERT "received data 2: %d \n", sd->int_in_buf[2]);
   printk(KERN_ALERT "received data 3: %d \n", sd->int_in_buf[3]);

       if (irqyup == 1) {
       printk(KERN_ALERT "irq alive and fired\n");
       local_irq_save(flags);
       generic_handle_irq(GPIO_irqNumber);
       local_irq_restore(flags);
       }

   usb_submit_urb(sd->int_in_urb, GFP_KERNEL);
   kfree(intrxbuf);
}

static void
_gpioa_set(struct gpio_chip *chip,
           unsigned offset, int value)
{
   struct spi_tiny_usb *data = container_of(chip, struct spi_tiny_usb,
                                      chip);
   printk(KERN_INFO "GPIO SET INFO for pin: %d \n", offset);

   usbval = 0;

		offs = offset;
        gpio_val = value;
        schedule_work(&data->work);
}

static int
_gpioa_get(struct gpio_chip *chip,
           unsigned offset)
{
   struct spi_tiny_usb *data = container_of(chip, struct spi_tiny_usb,
                                     chip);

   int retval, retval1, retval2, retval3;
   char *rxbuf = kmalloc(4, GFP_KERNEL); //4 for 4 gpio for todo grab multi pin states, also seems the usb rx buffer needs to be same size or issues
   if (!rxbuf)
		return -ENOMEM;

    printk(KERN_INFO "GPIO GET INFO: %d \n", offset);

    usbval = 3;
	offs = offset;
    gpio_val = 1;

    usb_control_msg(data->usb_dev,
                   usb_rcvctrlpipe(data->usb_dev, 0),
                   gpio_val, USB_TYPE_VENDOR | USB_DIR_IN,
                   usbval, offs,
                   rxbuf, 4,
                   100);

    memcpy(data->bufr, rxbuf, 4);

    //THE FOLLOWING IS OVERKILL BUT WHEN YOU FINALLY FIGURE OUT HOW TO ACTUALLY RECEIVE SOME DATA.....
    retval = rxbuf[0];
    retval1 = rxbuf[1];
    retval2 = rxbuf[2];
    retval3 = rxbuf[3];
    printk("buf0 =  %d \n", retval);
    printk("buf1 =  %d \n", retval1);
    printk("buf2 =  %d \n", retval2);
    printk("buf3 =  %d \n", retval3);

    kfree(rxbuf);

    return retval1 - 3;
}

static int
_direction_output(struct gpio_chip *chip,
                  unsigned offset, int value)
{
	   struct spi_tiny_usb *data = container_of(chip, struct spi_tiny_usb,
                                      chip);
   printk("Setting pin to OUTPUT gppio-offset %d\n", offset);

        usbval = 2;
		offs = offset;

        schedule_work(&data->work);

   return 0;
}

static int
_direction_input(struct gpio_chip *chip,
                  unsigned offset)
{
   struct spi_tiny_usb *data = container_of(chip, struct spi_tiny_usb,
                                      chip);

   printk("Setting pin to INPUT gpio-offset %d\n", offset);

        usbval = 1;
		offs = offset;

        schedule_work(&data->work);

   return 0;
}

static int
i2c_gpio_to_irq(struct gpio_chip *chip,
                  unsigned offset)
{
   struct spi_tiny_usb *data = container_of(chip, struct spi_tiny_usb,
                                      chip);
   GPIO_irqNumber = irq_create_mapping(data->chip.irq.domain, offset);
   pr_info("GPIO_irqNumber = %d\n", GPIO_irqNumber);

   return GPIO_irqNumber;
}

static void usb_gpio_irq_enable(struct irq_data *irqd)
{
	struct spi_tiny_usb *dev = irq_data_get_irq_chip_data(irqd);
    irqyup = 1;
	/* Is that needed? */
//	if (dev->irq.irq_enable)
	if (dev->irq_enabled[4])
		return;

//	dev->irq.irq_enable = true;
    dev->irq_enabled[4] = true;
//	usb_submit_urb(dev->int_in_urb, GFP_ATOMIC);
}

void set_irq_disabled(struct irq_data *irqd)
{
    struct spi_tiny_usb *dev = irq_data_get_irq_chip_data(irqd);
    irqyup = 0;
    if (!dev->irq_enabled[4])
        return;

    dev->irq_enabled[4] = false;
}

static void usb_gpio_irq_disable(struct irq_data *irqd)
{
    struct gpio_chip *chip = irq_data_get_irq_chip_data(irqd);
    struct spi_tiny_usb *data = container_of(chip, struct spi_tiny_usb,
                                      chip);
	/* Is that needed? */
//	if (!chip->irq_enabled[4])
//		return;

   usbval = 9;
   offs = 1;
   gpio_val = 9;
   schedule_work(&data->work);
   set_irq_disabled(irqd);
}

static int usbirq_irq_set_type(struct irq_data *irqd, unsigned type)
{
    struct gpio_chip *chip = irq_data_get_irq_chip_data(irqd);
    struct spi_tiny_usb *data = container_of(chip, struct spi_tiny_usb,
                                      chip);
    int pin = irqd_to_hwirq(irqd);
    pr_info("irq pin = %d\n", pin);
    irqt = type;
    pr_info("setting irq type GPIO_irqNumber = %d\n", GPIO_irqNumber);
    	switch (type) {
    case IRQ_TYPE_NONE:
		   usbval = 9;
           offs = 1;
           gpio_val = 9;
           schedule_work(&data->work);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		   usbval = 9;
           offs = 2;
           gpio_val = 9;
           schedule_work(&data->work);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		   usbval = 9;
           offs = 3;
           gpio_val = 9;
           schedule_work(&data->work);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		   usbval = 9;
           offs = 4;
           gpio_val = 9;
           schedule_work(&data->work);
		break;
	case IRQ_TYPE_EDGE_RISING:
		   usbval = 9;
           offs = 5;
           gpio_val = 9;
           edget = GPIO_ACTIVE_LOW;
           schedule_work(&data->work);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		   usbval = 9;
           offs = 6;
           gpio_val = 9;
           schedule_work(&data->work);
           edget = GPIO_ACTIVE_HIGH;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

//TODO dynamically append to gpionames for multi-instance
// const char *gpio_names[] = { "CSpi1", "PC7", "PG7", "PJ11", "IRQpk1" };
const char *gpio_names[] = { "spi-cs", "ce", "csn", "PJ11", "IRQpk1" };

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,18,0)    //dunno actual kernel ver when it switched but somewhere around then
static const struct irq_chip usb_gpio_irqchip = {
	.name = "usbgpio-irq",
	.irq_enable =  usb_gpio_irq_enable,
	.irq_disable = usb_gpio_irq_disable,
	.irq_set_type = usbirq_irq_set_type,
	.flags = IRQCHIP_IMMUTABLE, GPIOCHIP_IRQ_RESOURCE_HELPERS,
};
#endif



static void spi_tiny_usb_free(struct spi_tiny_usb *priv)
{
	usb_put_dev(priv->usb_dev);
	kfree(priv);
}

// static int usb_read(struct spi_tiny_usb *dev, int cmd, int value, int index, void *data, int len);  //WILL NEED/USE AGAIN IM SURE
size_t maxmsgsize(struct spi_device *spi);
size_t usegpiodesc(struct spi_device *spi);
static int usb_write(struct spi_tiny_usb *dev, int cmd, int value, int index, void *data, int len);
static int usb_write2(struct usb_interface *interface, int cmd, int value, int index, void *data, int len);
static int usbspi_bulk_xfer(struct usb_interface *interface, struct bulk_desc *desc);
static int usbspi_write_data(struct usb_interface *interface,
			   const char *buf, size_t len);
static int usbspi_read_data(struct usb_interface *interface, void *buf, size_t len);
static void setcs(struct usb_interface *interface,  bool enable);
void csset(struct spi_device *spi, bool enable);

// I THINK I DID THE FOLLOWING 2 FUNCTIONS THIS WAY FOR A REASON...WHATEVER THAT WAS
size_t maxmsgsize(struct spi_device *spi) {
(void)spi;
return 230;
}

size_t usegpiodesc(struct spi_device *spi) {
(void)spi;
return true;
}

int usbspi_write_data(struct usb_interface *interface,
			   const char *buf, size_t len)
{
	struct bulk_desc desc;
	int ret;

	desc.act_len = 0;
	desc.dir_out = true;
	desc.data = (char *)buf;
	desc.len = len;
	desc.timeout = USBSPI_WRITE_TIMEOUT;

	ret = usbspi_bulk_xfer(interface, &desc);
	if (ret < 0)
		return ret;

	return desc.act_len;
}



static int usbspi_read_data(struct usb_interface *interface, void *buf, size_t len)
{
	struct spi_tiny_usb *priv = usb_get_intfdata(interface);
	struct bulk_desc desc;
	int ret;

	desc.act_len = 0;
	desc.dir_out = false;
	desc.data = priv->bulk_in_buf;

	desc.len = min_t(size_t, len, priv->bulk_in_sz);
	desc.timeout = USBSPI_READ_TIMEOUT;

	ret = usbspi_bulk_xfer(interface, &desc);
	if (ret)
		return ret;

	ret = desc.act_len;
	if (ret > len)
		ret = len;
	memcpy(buf, desc.data, ret);
	return ret;
}


static int usbspi_bulk_xfer(struct usb_interface *interface, struct bulk_desc *desc)
{
	struct spi_tiny_usb *priv = usb_get_intfdata(interface);
	struct usb_device *usb_dev = priv->usb_dev;
	unsigned int pipe;
	int ret;

	mutex_lock(&priv->io_mutex);
	if (!priv->interface) {
	    printk("failed to get interface \r\n");
		ret = -ENODEV;
		goto exit;
	}

	if (desc->dir_out)
		pipe = usb_sndbulkpipe(usb_dev, priv->bulk_out);
	else
		pipe = usb_rcvbulkpipe(usb_dev, priv->bulk_in);

	ret = usb_bulk_msg(usb_dev, pipe, desc->data, desc->len,
			   &desc->act_len, desc->timeout);
	if (ret)
		dev_dbg(&usb_dev->dev, "bulk msg failed: %d\n", ret);

exit:
	mutex_unlock(&priv->io_mutex);
	return ret;
}

static void usbspi_lock(struct usb_interface *intf)
{
	struct spi_tiny_usb *priv = usb_get_intfdata(intf);

	mutex_lock(&priv->ops_mutex);
}

static void usbspi_unlock(struct usb_interface *intf)
{
	struct spi_tiny_usb *priv = usb_get_intfdata(intf);

	mutex_unlock(&priv->ops_mutex);
}

static void setcs(struct usb_interface *interface,  bool enable) {
struct spi_tiny_usb *priv = usb_get_intfdata(interface);
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

static void usbspi_config(struct usb_interface *dev, int cmd, int value, int index)
{
struct spi_tiny_usb *priv = usb_get_intfdata(dev);
int ret;
mutex_lock(&priv->io_mutex);
	ret = usb_control_msg(priv->usb_dev, usb_sndctrlpipe(priv->usb_dev, 0),
			       cmd, USB_TYPE_VENDOR | USB_RECIP_INTERFACE,
			       value, index, NULL, 0, 2000);
mutex_unlock(&priv->io_mutex);

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
//	.ctrl_xfer = usbspi_bulk_xfer,
	.bulk_xfer = usbspi_bulk_xfer,
	.read_data = usbspi_read_data,
	.write_data = usbspi_write_data,
	.lock = usbspi_lock,
	.unlock = usbspi_unlock,
	.chcs = setcs,
	.setup_data = usbspi_config,
//	.set_bitmode = ftdi_set_bitmode,
//	.set_baudrate = ftdi_set_baudrate,
//	.init_pins = ftdi_mpsse_init_pins,
//	.cfg_bus_pins = ftdi_mpsse_cfg_bus_pins,
//	.set_clock = ftdi_set_clock,
//	.set_latency = ftdi_set_latency,
};


/* ----- begin of spi layer ---------------------------------------------- */




int spi_tiny_usb_xfer_one_two(struct spi_master *master, struct spi_message *m)
{
	struct spi_tiny_usb *priv = spi_master_get_devdata(master);
	struct usb_interface *interface;
	struct spi_transfer *t;
	static int total = 0;
	int spi_flags;
	int ret = 0;
	int temp;
    int wait;
    int offset2 = 0;
    interface = usb_get_intfdata(priv->interface);


    m->actual_length = 0;

//    printk("bens oldspihz = %d\n", priv->oldspihz);
//    printk("bens gbase = %d\n", gbase);
//    printk("bens priv->gbase = %d\n", priv->gbase);   // FUNNY BOTH priv->chip.base AND THE CP'D priv->gpiobase ARE SET AND RETURN FINE I THINK SWITCHING INTERFACES
    // PLOWS IT THO IM NOT SURE HOW OR WHY gpiobase ALSO GETS CLOBBERED, UNLESS THE USB INTERFACES ARE LIMITED ON WHAT THEY CAN SEE IN WHICH CASE FUCK ME.
	printk("actual length aka total sent bytes from all segments = %d\n", m->actual_length);
	printk("frame length aka total num of bytes in this msg = %d\n", m->frame_length);

//    csset(priv->spidev, true);

	spi_flags = FLAGS_BEGIN;
	list_for_each_entry(t, &m->transfers, transfer_list) {
		if (list_is_last(&t->transfer_list, &m->transfers))
			spi_flags |= FLAGS_END;

        if (priv->oldlsb != (priv->spidev->mode & SPI_LSB_FIRST)) {
        printk("mode lsb claims change \r\n");
            if (priv->oldlsb != 0x08) {
                usb_write(priv, 160, 0x08, 0x08, NULL, 0);
                } else {
                usb_write(priv, 160, 0x00, 0x00, NULL, 0);
            }
        }

        priv->oldlsb = (priv->spidev->mode & SPI_LSB_FIRST);

        if (priv->oldspihz != t->speed_hz) {
        printk("setting clock \r\n");
        if ((t->speed_hz / 10000) > 9999) {
        t->speed_hz = 99990000;
        }

        wait = usb_write(priv, 67, (t->speed_hz / 10000), (t->speed_hz / 10000), NULL, 0);
           if (wait) {
           ;
           }

        }
        priv->oldspihz = t->speed_hz;


	if (priv->last_mode != priv->spidev->mode) {
		u8 spi_mode = priv->spidev->mode & (SPI_CPOL | SPI_CPHA);
		printk("spi_mode = %d\r\n", spi_mode);

        wait = usb_write(priv, 68, spi_mode, spi_mode, NULL, 0);
           if (wait) {
           ;
           }

		}

    priv->last_mode = priv->spidev->mode;

   if (priv->last_bpw != priv->spidev->bits_per_word) {

      wait = usb_write(priv, 69, priv->spidev->bits_per_word, priv->spidev->bits_per_word, NULL, 0);
           if (wait) {
           ;
           }

   printk("changed bits_per_word to %d\r\n", priv->spidev->bits_per_word);
   }

   priv->last_bpw = priv->spidev->bits_per_word;

		printk("tx: %p rx: %p len: %d speed: %d flags: %d delay: %d\n", t->tx_buf,
			t->rx_buf, t->len, t->speed_hz, spi_flags, t->delay.value);

		if (t->cs_change)
			spi_flags |= FLAGS_END;


// IF TX and RX

        if (t->tx_buf && t->rx_buf) {
		if (t->len > 508) {
		void *txbuf = kmalloc(512, GFP_KERNEL);
		void *rxbuf = kmalloc(t->len, GFP_KERNEL);
		offset2 = 0;
		while (t->len - offset2 > 508) {
		memcpy(txbuf, t->tx_buf + offset2, 508);
		usbspi_write_data(priv->interface, txbuf, 508);
		usb_write(priv, 65, 508, 508, NULL, 0);
		usbspi_read_data(priv->interface, t->rx_buf, 508);
		memcpy(rxbuf + offset2, t->rx_buf, 508);
		offset2 += 508;
		m->actual_length += 508;
		}
		temp = t->len - offset2;
		memcpy(txbuf, t->tx_buf + offset2, temp);
		usbspi_write_data(priv->interface, txbuf, temp);
		usb_write(priv, 65, temp, temp, NULL, 0);
		usbspi_read_data(priv->interface, t->rx_buf, temp);
		memcpy(rxbuf + offset2, t->rx_buf, temp);
		memcpy(t->rx_buf, rxbuf, t->len);
		offset2 = 0;
		m->actual_length += temp;
		mutex_lock(&priv->io_mutex);
		kfree(txbuf);
		kfree(rxbuf);
		mutex_unlock(&priv->io_mutex);
		} else {
		printk(" inside tx, tx: %p rx: %p len: %d total: %d\n", t->tx_buf,
			t->rx_buf, t->len, total);
            wait = usbspi_write_data(priv->interface, t->tx_buf, t->len);
            m->actual_length += t->len;
            if (wait) {
            ;
            }
            usb_write(priv, 65, t->len, t->len, NULL, 0);
			usbspi_read_data(priv->interface, t->rx_buf, t->len);
            total = total + 1;
		}

		spi_finalize_current_message(master);
		spi_flags = 0;
    	m->status = ret;
	    return 0;
		}


// IF TX


		if (t->tx_buf) {
		if (t->len > 508) {
		void *txbuf = kmalloc(512, GFP_KERNEL);
		while (t->len - offset2 > 508) {
		memcpy(txbuf, t->tx_buf + offset2, 508);
		usbspi_write_data(priv->interface, txbuf, 508);
		offset2 += 508;
		m->actual_length += 508;
		}
		temp = t->len - offset2;
		memcpy(txbuf, t->tx_buf + offset2, temp);
		usbspi_write_data(priv->interface, txbuf, temp);
		offset2 = 0;
		m->actual_length += temp;
		mutex_lock(&priv->io_mutex);
		kfree(txbuf);
		mutex_unlock(&priv->io_mutex);
		} else {
		printk(" inside tx, tx: %p rx: %p len: %d total: %d\n", t->tx_buf,
			t->rx_buf, t->len, total);

            wait = usbspi_write_data(priv->interface, t->tx_buf, t->len);
            m->actual_length += t->len;
            if (wait) {
            ;
            }
            total = total + 1;


			}
		  }

// IF RX

		if (t->rx_buf) {
		offset2 = 0;
		if (t->len > 508) {
		void *rxbuf = kmalloc(t->len, GFP_KERNEL);
		while (t->len - offset2 > 508) {
		void *txbuf = kmalloc(t->len, GFP_KERNEL);
		memset(txbuf, 0x00, t->len);
		usbspi_write_data(priv->interface, txbuf, t->len);
		mutex_lock(&priv->io_mutex);
		kfree(txbuf);
		mutex_unlock(&priv->io_mutex);
		usb_write(priv, 65, 508, 508, NULL, 0);
		usbspi_read_data(priv->interface, t->rx_buf, 508);
		memcpy(rxbuf + offset2, t->rx_buf, 508);
		offset2 += 508;
		m->actual_length += 508;
		}
		void *txbuf = kmalloc(t->len, GFP_KERNEL);
		memset(txbuf, 0x00, t->len);
		usbspi_write_data(priv->interface, txbuf, t->len);
		mutex_lock(&priv->io_mutex);
		kfree(txbuf);
		mutex_unlock(&priv->io_mutex);
		temp = t->len - offset2;
		usb_write(priv, 65, temp, temp, NULL, 0);
		usbspi_read_data(priv->interface, t->rx_buf, temp);
		memcpy(rxbuf + offset2, t->rx_buf, temp);
		memcpy(t->rx_buf, rxbuf, t->len);
		offset2 = 0;
		m->actual_length += temp;
		mutex_lock(&priv->io_mutex);
		kfree(rxbuf);
		mutex_unlock(&priv->io_mutex);
		} else {
		    void *txbuf = kmalloc(t->len, GFP_KERNEL);
		    printk("RX bulk start len = %d \r\n", t->len);
		    printk("frame length in rec total num of bytes in this msg = %d\n", m->frame_length);
		    memset(txbuf, 0x00, t->len);
		    usbspi_write_data(priv->interface, txbuf, t->len);
		    mutex_lock(&priv->io_mutex);
		    kfree(txbuf);
		    mutex_unlock(&priv->io_mutex);
			usb_write(priv, 65, t->len, t->len, NULL, 0);

            usbspi_read_data(priv->interface, t->rx_buf, t->len);

            m->actual_length += t->len;
		}
		}
		printk("frame_length = %d actual_length = %d\n", m->frame_length, m->actual_length);
        spi_finalize_current_message(master);

		if (t->delay.value)
			udelay(t->delay.value);

		spi_flags = 0;

		if (t->cs_change)
			spi_flags |= FLAGS_BEGIN;

        }
        csset(priv->spidev, false);

	m->status = ret;
	return 0;
}


static int usbspi_intf_spi_remove(struct usb_interface *intf)
{
	struct spi_tiny_usb *priv = usb_get_intfdata(intf);
	struct device *dev = &intf->dev;

	dev_dbg(dev, "%s: spi pdev %p\n", __func__, priv->spi_pdev);
	gpiod_remove_lookup_table(priv->lookup_cs);
    priv->irq_enabled[4] = false;
	platform_device_unregister(priv->spi_pdev);
	return 0;
}

static const struct usb_spi_platform_data usbspi_bus_plat_data;

static int usbspi_intf_spi_probe(struct usb_interface *interface,
				 const void *plat_data);

static const struct usbspi_intf_info usbspi_bus_intf_info = {
    .probe  = usbspi_intf_spi_probe,
    .remove  = usbspi_intf_spi_remove,
    .plat_data  = &usbspi_bus_plat_data,
};

static struct usb_device_id spi_tiny_usb_table[] = {
	{USB_DEVICE(VID, PID),
	    .driver_info = (kernel_ulong_t)&usbspi_bus_intf_info },
	{}
};
MODULE_DEVICE_TABLE(usb, spi_tiny_usb_table);

static int usb_write(struct spi_tiny_usb *dev, int cmd, int value, int index, void *data, int len)
{
int ret;
mutex_lock(&dev->io_mutex);
	ret = usb_control_msg(dev->usb_dev, usb_sndctrlpipe(dev->usb_dev, 0),
			       cmd, USB_TYPE_VENDOR | USB_RECIP_INTERFACE,
			       value, index, data, len, 2000);
mutex_unlock(&dev->io_mutex);
			       return ret;
}





struct property_entry mcp2515_properties[] = {
	PROPERTY_ENTRY_U32("clock-frequency", 16000000),
//	PROPERTY_ENTRY_U32("xceiver", 1),
//	PROPERTY_ENTRY_U32("gpio-controller", 3),
	{}
};

struct property_entry w25q32_properties[] = {
//	PROPERTY_ENTRY_U32("spi-max-frequency", 10000000),
//	PROPERTY_ENTRY_BOOL("m25p,read"),
//	PROPERTY_ENTRY_U32("gpio-controller", 3),
	{}
};

static const struct software_node mcp2515_node = {
	.properties = mcp2515_properties,
};


static const struct software_node w25q32_node = {
	.properties = w25q32_properties,
};

static int usb_spi_setup(struct spi_device *spi)
{
	struct spi_tiny_usb *priv = spi_master_get_devdata(spi->master);

	dev_info(&priv->usb_dev->dev, "usb_spi_setup() for %s on CS %d",
	         spi->modalias, spi->chip_select);

	usb_write(priv, 74, 6, 6, NULL, 0);

	return 0;
}


static struct dev_io_desc_data usb_spi_bus_dev_io[] = {
//         { "CS", 0, GPIO_ACTIVE_LOW },
//       { "ce", 1, GPIO_ACTIVE_HIGH },
//       { "csn", 2, GPIO_ACTIVE_LOW },
};


static const struct usbspi_dev_data usb_spi_dev_data[] = {
	{
	.magic		= USBSPI_IO_DESC_MAGIC,
	.desc		= usb_spi_bus_dev_io,
	.desc_len	= ARRAY_SIZE(usb_spi_bus_dev_io),
	},
};


void csset(struct spi_device *spi, bool enable) {
struct spi_tiny_usb *priv = spi_master_get_devdata(spi->master);
if (enable == true) {
usb_write(priv, 71, 0, 0, NULL, 0);
printk("setting cs low\n");
}
if (enable == false) {
usb_write(priv, 72, 0, 0, NULL, 0);
printk("setting cs high\n");
  }
}

static struct spi_board_info usb_spi_bus_info[] = {
    {
//    .modalias	= "yx240qv29",
//	.modalias	= "ili9341",
//    .modalias	= "w25q32",
	.modalias	= "mcp2515",
//    .modalias	= "spi-petra",
//    .modalias	= "nrf24",
//	.modalias	= "ili9341",
    .mode		= SPI_MODE_0,
//    .mode		= SPI_MODE_0 | SPI_LSB_FIRST | SPI_CS_HIGH,
    .max_speed_hz	= 4000000,
//    .max_speed_hz	= 30000000,
    .bus_num	= 0,
    .chip_select	= 0,
    .platform_data	= usb_spi_dev_data,
    .swnode  =  &mcp2515_node,
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

#define SPI_INTF_DEVNAME	"spi_plat_usb"

static const struct usb_spi_platform_data usbspi_bus_plat_data = {
    .ops		= &usbspi_intf_ops,
    .spi_info	= usb_spi_bus_info,
    .spi_info_len	= ARRAY_SIZE(usb_spi_bus_info),
};

static struct platform_device *usbspi_dev_register(struct spi_tiny_usb *priv,
				const struct usb_spi_platform_data *pd)
{
	struct device *parent = &priv->interface->dev;
	struct platform_device *pdev;
	struct gpiod_lookup_table *lookup;
	size_t lookup_size, tbl_size;
	int i, ret;

    printk("spi_tiny_usb Start of platform probe\n");

	pdev = platform_device_alloc(SPI_INTF_DEVNAME, 0);
	if (!pdev)
		printk("spi_tiny_usb failed to add platform device\n");


	pdev->dev.parent = parent;
	pdev->dev.fwnode = NULL;

	priv->spi_pdev = pdev;


	tbl_size = pd->spi_info_len + 1;
	printk("spi_tiny_usb Spi info length = %ld\n",pd->spi_info_len + 1 );
	lookup_size = sizeof(*lookup) + tbl_size * sizeof(struct gpiod_lookup);
	lookup = devm_kzalloc(parent, lookup_size, GFP_KERNEL);
	if (!lookup) {
	    printk("failed lookuo_size\n");
		ret = -ENOMEM;
		goto err;
	}

    printk("spi_tiny_usb starting spi info length loop\n");
	for (i = 0; i < pd->spi_info_len; i++) {
		dev_dbg(parent, "INFO: %s cs %d\n",
			pd->spi_info[i].modalias, pd->spi_info[i].chip_select);
	}

    printk("spi_tiny_usb starting platform_device_add_data\n");
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

    printk("spi_tiny_usb starting lookup loop\n");
	for (i = 0; i < pd->spi_info_len; i++) {
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

	gpiod_add_lookup_table(priv->lookup_cs);

    printk("spi_tiny_usb starting platform_device_add\n");
	ret = platform_device_add(pdev);
	if (ret < 0)
		printk("error onn platform_device_add \n");

	dev_dbg(&pdev->dev, "%s done\n", __func__);

	printk("spi_tiny_usb end of platform probe \n");

	return pdev;

err_add:
	gpiod_remove_lookup_table(priv->lookup_cs);
err:
	platform_device_put(pdev);
	return ERR_PTR(ret);
}

static const struct property_entry nrf24_properties[] = {
	PROPERTY_ENTRY_U32("interrupts", 4),
	{}
};

static const struct software_node nrf24_node = {
	.properties = nrf24_properties,
};

static int usbspi_intf_spi_probe(struct usb_interface *interface,
				 const void *plat_data)
{
	struct spi_tiny_usb *priv = usb_get_intfdata(interface);
	struct device *dev = &interface->dev;
	struct platform_device *pdev;
    int inf;

    inf = priv->interface->cur_altsetting->desc.bInterfaceNumber;
    printk("starting usbspi_intf_spi_probe FOR USB-2-SPI\n");

//    if (inf != 2) {
//    printk("Ignoring interface reserved for SPI\n");
//    return -ENODEV;
 //   }

//    printk("spi interface probe begin\n");
	pdev = usbspi_dev_register(priv, plat_data);
	if (IS_ERR(pdev)) {
		dev_err(dev, "%s: Can't create USBSPI device %ld\n",
			__func__, PTR_ERR(pdev));
		return PTR_ERR(pdev);
	}

	priv->spi_pdev = pdev;
    printk("ending usbspi_intf_spi_probe FOR USB-2-SP\nI");
	return 0;
}

static int spi_tiny_usb_probe(struct usb_interface *interface,
			      const struct usb_device_id *id)
{
    struct device *dev = &interface->dev;
//    struct usb_interface *interface;
    struct spi_master *master = container_of(dev, struct spi_master, dev);
	struct spi_tiny_usb *priv = spi_master_get_devdata(master);
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
	int wait;
	dev_dbg(&interface->dev, "probing usb device\n");

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	priv->usb_dev = usb_get_dev(interface_to_usbdev(interface));
	inf = interface->cur_altsetting->desc.bInterfaceNumber;
	priv->interface = interface;
	iface_desc = interface->cur_altsetting;

	if (inf == 0) {
    return -ENODEV;
    }

    if (inf == 1) {
   priv->usb_dev = usb_get_dev(interface_to_usbdev(interface));
   endpoint = &iface_desc->endpoint[1].desc;
   retval = usb_find_int_in_endpoint(interface->cur_altsetting, &endpoint);
   if (retval) {
   printk("USBSPI Could not find int-in endpoint\n");
   return retval;
   }
   priv->int_in_endpoint = endpoint;
   priv->int_in_endpointAddr = endpoint->bEndpointAddress;
   priv->int_in_endpoint->wMaxPacketSize =  0x10;
   printk(KERN_INFO "MaxPacketSize: %d \n", endpoint->wMaxPacketSize);
   printk(KERN_INFO "actual MaxPacketSize: %d \n", usb_endpoint_maxp(endpoint));
   printk(KERN_INFO "priv->int_in_endpointAddr: %d \n", priv->int_in_endpointAddr);
   printk(KERN_INFO "priv->int_in_endpoint->bEndpointAddress: %d \n", priv->int_in_endpoint->bEndpointAddress);
   // allocate our urb for interrupt in
   if (noirq == 0) {
   priv->int_in_urb = usb_alloc_urb(0, GFP_KERNEL);
   //allocate the interrupt buffer to be used
   priv->int_in_buf = kmalloc(le16_to_cpu(priv->int_in_endpoint->wMaxPacketSize), GFP_KERNEL);
   //initialize our interrupt urb
   //notice the rcvintpippe -- it is for recieving data from privice at interrupt endpoint
   usb_fill_int_urb(priv->int_in_urb, priv->usb_dev,
                    usb_rcvintpipe(priv->usb_dev, priv->int_in_endpoint->bEndpointAddress),
                    priv->int_in_buf,
                    le16_to_cpu(priv->int_in_endpoint->wMaxPacketSize),
                    int_cb, // this callback is called when we are done sending/recieving urb
                    priv,
                    (priv->int_in_endpoint->bInterval));

   usb_set_intfdata(interface, priv);

   printk(KERN_INFO "usb gpio irq is connected \n");

   i = usb_submit_urb(priv->int_in_urb, GFP_KERNEL);
   if (i)
     {
        printk(KERN_ALERT "Failed to submit urb \n");
     }
    }   // noirq

   INIT_WORK(&priv->work, _gpio_work_job);
   INIT_WORK(&priv->work2, _gpio_work_job2);

	label = devm_kasprintf(dev, GFP_KERNEL, "ftdi-mpsse-gpio.%d", priv->id);
	if (!label)
		return -ENOMEM;

   /// gpio_chip struct info is inside KERNEL/include/linux/gpio/driver.h
   priv->chip.label = "usbgpio"; //name for diagnostics
   priv->chip.parent = &interface->dev;
   priv->chip.owner = THIS_MODULE; // helps prevent removal of modules exporting active GPIOs, so this is required for proper cleanup
   priv->chip.base = -1; // identifies the first GPIO number handled by this chip;
   // or, if negative during registration, requests dynamic ID allocation. setting it statically now deprecated.
   priv->chip.ngpio = 5; // the number of GPIOs handled by this controller; the last GPIO
   priv->chip.can_sleep = true; //
   /*
      flag must be set iff get()/set() methods sleep, as they
    * must while accessing GPIO expander chips over I2C or SPI. This
    * implies that if the chip supports IRQs, these IRQs need to be threaded
    * as the chip access may sleep when e.g. reading out the IRQ status
    * registers.
    */
   priv->chip.set = _gpioa_set;
   priv->chip.get = _gpioa_get;
   priv->chip.direction_input = _direction_input;
   priv->chip.direction_output = _direction_output;
   priv->chip.to_irq = i2c_gpio_to_irq;
   priv->chip.names = gpio_names;
   if (noirq == 0) {
   #if LINUX_VERSION_CODE <= KERNEL_VERSION(5,18,0)    //yeah sure well call it 5.18
   priv->irq.name = "usbgpio-irq";
   priv->irq.irq_set_type = usbirq_irq_set_type;
   priv->irq.irq_enable = usb_gpio_irq_enable;
   priv->irq.irq_disable = usb_gpio_irq_disable;

	girq = &priv->chip.irq;
	girq->chip = &priv->irq;
	#else
	girq = &priv->chip.irq;
    gpio_irq_chip_set_chip(girq, &usb_gpio_irqchip);
    #endif
	girq->parent_handler = NULL;
	girq->num_parents = 0;
	girq->parents = NULL;
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_simple_irq;

	rc = irq_alloc_desc(0);
	if (rc < 0) {
    ;
		return rc;
	}

    priv->irq_base = rc;

} //end of noirq

	names = devm_kcalloc(dev, priv->chip.ngpio, sizeof(char *),
			     GFP_KERNEL);
	if (!names)
		return -ENOMEM;


	priv->chip.names = gpio_names;


   if (gpiochip_add(&priv->chip) < 0)
     {
        printk(KERN_ALERT "Failed to add gpio chip \n");
        priv->gpio_init = false;
     }
   else
     {
        printk(KERN_INFO "Able to add gpiochip: %s \n", priv->chip.label);
        priv->gpio_init = true;
     }

/*	ret = devm_gpiochip_add_data(dev, &priv->chip, priv);
	if (ret < 0) {
		printk("Failed to add MPSSE GPIO chip: %d\n", ret);
		return ret;
	}

    priv->cs_gpiod = gpiochip_request_own_desc(&priv->chip,
                                                0,
                                                "CSpi1",
                                                GPIO_LOOKUP_FLAGS_DEFAULT,
                                                GPIOD_OUT_LOW);

     set_bit(GPIOF_ACTIVE_LOW, &priv->cs_gpiod->flags);
*/
irq_clear_status_flags(priv->irq_base + 4, IRQ_NOREQUEST | IRQ_NOPROBE);
irq_set_status_flags(priv->irq_base + 4, IRQ_TYPE_EDGE_FALLING);

if (noirq == 0) {
    priv->interrupt_gpio = gpiochip_request_own_desc(&priv->chip,
                                                4,
                                                "IRQpk1",
                                                GPIOD_IN,
                                                edget);

   i2c_gpio_to_irq(&priv->chip, 4);



   printk("gpio-base = %d\n", priv->chip.base);
   gbase = (cpu_to_le16(priv->chip.base));
//   priv->gbase = 300;
//   printk("priv->gbase = %d\n", priv->gbase);
   printk("gbase = %d\n", gbase);
/*
   gpiochip_free_own_desc(priv->interrupt_gpio);
   gpiochip_free_own_desc(priv->cs_gpiod);
    priv->interrupt_gpio = devm_gpiod_get(dev, "IRQpk1", GPIOD_IN);

    ret = devm_gpio_request_one(dev,  gbase + 4, GPIOD_IN, "IRQpk1");
		if (ret) {
			if (ret == -EPROBE_DEFER)
				pr_info("failed request one  = IRQpk1\n");
		}


        priv->ce_gpiod = gpiochip_request_own_desc(&priv->chip,
                                                1,
                                                "ce",
                                                GPIO_LOOKUP_FLAGS_DEFAULT,
                                                GPIO_ACTIVE_HIGH);

        gpiod_direction_output(priv->ce_gpiod, true);

        gpiochip_free_own_desc(priv->ce_gpiod);

*/

   }

    }   // end of interface 0 setup

  if (inf == 2) {


//	inf = interface->cur_altsetting->desc.bInterfaceNumber;
//	priv->interface = interface;

	retval = usb_find_common_endpoints(interface->cur_altsetting,
			&bulk_in, &bulk_out, NULL, NULL);
	if (retval) {
		dev_err(&interface->dev,
			"Could not find both bulk-in and bulk-out endpoints\n");
		goto error;
	}

	priv->bulk_in_sz = usb_endpoint_maxp(bulk_in);
	priv->bulk_in = bulk_in->bEndpointAddress;
	priv->bulk_out = bulk_out->bEndpointAddress;

	priv->usb_dev_id = id;
	priv->index = 1;
	priv->interface = interface;
    priv->info = (struct usbspi_intf_info *)id->driver_info;
    info = priv->info;

    mutex_init(&priv->io_mutex);
	mutex_init(&priv->ops_mutex);

	priv->bulk_in_buf = devm_kmalloc(dev, priv->bulk_in_sz, GFP_KERNEL);
	if (!priv->bulk_in_buf)
		return -ENOMEM;

    priv->usb_dev = usb_get_dev(interface_to_usbdev(interface));

	priv->id = ida_simple_get(&usbspi_devid_ida, 0, 0, GFP_KERNEL);
	if (priv->id < 0)
		return priv->id;

	printk("out endpoint size = %ld \n", priv->bulk_in_sz);
    printk("out endpoint address = %02x \n", priv->bulk_out);
    printk("in endpoint address = %02x \n", priv->bulk_in);
	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, priv);

	version = le16_to_cpu(priv->usb_dev->descriptor.bcdDevice);
	dev_info(&interface->dev,
		 "version %x.%02x found at bus %03d address %03d\n",
		 version >> 8, version & 0xff, priv->usb_dev->bus->busnum,
		 priv->usb_dev->devnum);

	dev_info(&interface->dev, "connected spi-tiny-usb device\n");

if (noirq == 0) {
     irq_set_irq_type(GPIO_irqNumber, irqt);
        }
 /*
	// SPI master
	priv->last_bpw = 8;
	priv->master = spi_alloc_master(&interface->dev, sizeof(*priv));
	if (!priv->master)
		goto error;
	priv->master->mode_bits = SPI_CPOL | SPI_CPHA;  //SPI_3WIRE | SPI_CS_HIGH | SPI_LOOP | SPI_LSB_FIRST
	priv->master->flags = SPI_MASTER_GPIO_SS; //0;
	priv->master->setup = usb_spi_setup;
	priv->master->set_cs = csset;
	priv->master->use_gpio_descriptors = usegpiodesc;
	priv->master->transfer_one_message = spi_tiny_usb_xfer_one;
	priv->master->dev.of_node = interface->dev.of_node;
	priv->master->num_chipselect = 1;
	priv->master->max_speed_hz = 6000000;
	priv->master->min_speed_hz = 1000000;
	spi_master_set_devdata(priv->master, priv);

	ret = spi_register_master(priv->master);
	if (ret)
		goto error2;

	usleep_range(1000, 1200);
// also can use bk4
//	strcpy(priv->spiinfo.modalias, "spi-petra");
    if (name) {
        strcpy(priv->spiinfo.modalias, name);
    } else {
        strcpy(priv->spiinfo.modalias, "mcp2515");
    }
	priv->spiinfo.max_speed_hz = 4000000;

	wait = usb_write(priv, 67, (priv->spiinfo.max_speed_hz / 10000), (priv->spiinfo.max_speed_hz / 10000), NULL, 0);
       if (wait) {
       ;
       }
*/
	if (info->probe) {
		ret = info->probe(interface, info->plat_data);
		if (ret < 0)
			printk("info->probe fail\n");
	}

//    usleep_range(2000, 2200);

//    priv->oldspihz = priv->spiinfo.max_speed_hz;
//	priv->spiinfo.chip_select = 0;
//	priv->spiinfo.swnode = &mcp2515_node;
//    priv->spiinfo.swnode = &nrf24_node;
//	priv->spiinfo.mode = SPI_MODE_0;
//	priv->spiinfo.platform_data	= usb_spi_dev_data;
//	if (noirq == 0) {
//    priv->spiinfo.irq = GPIO_irqNumber;
//    }
//	priv->spiinfo.controller_data = priv;
//	usleep_range(5000, 5500);


//	priv->spidev = spi_new_device(priv->master, &priv->spiinfo);
//	if (!priv->spidev)
//		goto error2;

//	dev_info(&interface->dev,
//		 "USBSPI device now attached to spidev%d.%d",
//		 interface->minor, priv->spiinfo.chip_select);

 } //elseif inf 2
    printk("spi-tiny-usb leaving regular probe\n");
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

void spi_tiny_usb_remove(struct spi_tiny_usb *priv);
void spi_tiny_usb_gpio_remove(struct spi_tiny_usb *priv);

void spi_tiny_usb_remove(struct spi_tiny_usb *priv)
{

	if (priv->master == NULL) {
		return;
    }
		spi_unregister_device(priv->spidev);
		priv->spidev = NULL;


	spi_unregister_master(priv->master);
}

void spi_tiny_usb_gpio_remove(struct spi_tiny_usb *priv)
{
	if (!priv->gpio_init)
		return;

	usb_kill_urb(priv->int_in_urb);
	usb_free_urb(priv->int_in_urb);
//    gpiochip_free_own_desc(priv->cs_gpiod);
	gpiochip_remove(&priv->chip);
//	if (noirq == 0) {
//	irq_free_desc(&priv->chip.irq);
//	}
}


static void spi_tiny_usb_disconnect(struct usb_interface *interface)
{
	struct spi_tiny_usb *priv = usb_get_intfdata(interface);

    spi_tiny_usb_remove(priv);
    spi_tiny_usb_gpio_remove(priv);
    usb_set_intfdata(priv->interface, NULL);
	usb_put_dev(priv->usb_dev);

	kfree(priv);
}

static void usbspi_draw_down(struct spi_tiny_usb *dev)
{
;
}


static int usbspi_suspend(struct usb_interface *interface, pm_message_t message)
{
	struct spi_tiny_usb *dev = usb_get_intfdata(interface);

	if (!dev)
		return 0;
	usbspi_draw_down(dev);
	return 0;
}

static int usbspi_resume(struct usb_interface *interface)
{
	return 0;
}

static int usbspi_pre_reset(struct usb_interface *interface)
{
	struct spi_tiny_usb *dev = usb_get_intfdata(interface);

	mutex_lock(&dev->io_mutex);
	usbspi_draw_down(dev);

	return 0;
}

static int usbspi_post_reset(struct usb_interface *interface)
{
	struct spi_tiny_usb *dev = usb_get_intfdata(interface);

	dev->errors = -EPIPE;
	mutex_unlock(&dev->io_mutex);

	return 0;
}


static struct usb_driver spi_tiny_usb_driver = {
	.name = "spi-tiny-usb",
	.probe = spi_tiny_usb_probe,
	.disconnect = spi_tiny_usb_disconnect,
	.suspend =	usbspi_suspend,
	.resume =	usbspi_resume,
	.pre_reset =	usbspi_pre_reset,
	.post_reset =	usbspi_post_reset,
	.id_table = spi_tiny_usb_table,
};

module_usb_driver(spi_tiny_usb_driver);

/* ----- end of usb layer ------------------------------------------------ */

MODULE_AUTHOR("Ben Maddockd <bm16ton@gmail.com>");
MODULE_DESCRIPTION("spi-tiny-usb driver 16ton edition");
MODULE_LICENSE("GPL");