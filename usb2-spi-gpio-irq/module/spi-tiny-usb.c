
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/usb.h>
#include <linux/gpio.h>
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
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/pwm.h>

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
#define PID 0xc633

static DEFINE_IDA(usbspi_devid_ida);


static char *name;
module_param(name, charp, 0000);
MODULE_PARM_DESC(name,
		 "Devicename modalias");

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
    bool    hwirq;
    int                      gpio_irq_map[4];
    struct usb_endpoint_descriptor *int_in_endpoint;
	struct urb		*int_in_urb;		/* the urb to read data with */
	unsigned char           *int_in_buf;	/* the buffer to receive data */
	__u8			int_in_endpointAddr;	/* the address of the bulk in endpoint */
    struct irq_chip   irq;                                // chip descriptor for IRQs
    int               num;
    uint8_t           irq_num;                            // number of pins with IRQs
    int               irq_base;                           // base IRQ allocated
//     struct irq_desc*  irq_descs    [5]; // IRQ descriptors used (irq_num elements)
    const struct cpumask *aff_mask;
     int               irq_types    [5]; // IRQ types (irq_num elements)
     bool              irq_enabled  [5]; // IRQ enabled flag (irq_num elements)
     int               irq_gpio_map [5]; // IRQ to GPIO pin map (irq_num elements)
     int               irq_hw;           
	const struct usb_device_id	*usb_dev_id;
	struct usbspi_intf_info		*info;
	struct spi_master *master;	
	struct spi_controller *spimaster;
	struct spi_device *spidev;	
	struct spi_board_info spiinfo;	
	struct uio_info *uio;	
	struct work_struct work;
    struct work_struct work2;
	struct gpio_chip gpio_chip;	
	struct semaphore	limit_sem;	
	struct usb_anchor	submitted;	
	int			errors;		
	spinlock_t		err_lock;	
	struct kref		kref;

	wait_queue_head_t	bulk_in_wait;	
	
	u8 txrx_cmd;
	u8 rx_cmd;
	u8 tx_cmd;
	u8 xfer_buf[SZ_64K];
	u16 last_mode;
	u32 last_speed_hz;
	u16 last_bpw;
	u16 oldlsb;
	
	u8 bufr[4];
};
#define to_usbspi_dev(d) container_of(d, struct spi_tiny_usb, kref)

#define USBSPI_READ_TIMEOUT	5000
#define USBSPI_WRITE_TIMEOUT	5000


static int i2c_gpio_to_irq(struct gpio_chip *chip, unsigned offset);
static struct usb_driver spi_tiny_usb_driver;
static void usbspi_draw_down(struct spi_tiny_usb *dev);


unsigned int GPIO_irqNumber;

static uint8_t gpio_val = 0;      // brequest
static uint8_t offs = 0;          // windex?
static uint8_t usbval = 0;        // wvalue
int pdown = 0;
int powdn = 0;
int irqt = 2;
int irqyup = 0;

static void
_gpio_work_job(struct work_struct *work)
{
   struct spi_tiny_usb *sd = container_of(work, struct spi_tiny_usb, work);

   printk(KERN_ALERT "gpioval i/o: %d \n", gpio_val);
   printk(KERN_ALERT "usbval i/o: %d \n", usbval);
   printk(KERN_ALERT "offset i/o: %d \n",offs);
   usb_control_msg(sd->usb_dev,
                   usb_sndctrlpipe(sd->usb_dev, 0),
                   gpio_val, USB_TYPE_VENDOR | USB_DIR_OUT,
                   usbval, offs,
                   NULL, 0,
                   3000);
}

static void
_gpio_work_job2(struct work_struct *work2)
{
   struct spi_tiny_usb *sd = container_of(work2, struct spi_tiny_usb, work2);

   printk(KERN_ALERT "Read port i/o: %d \n", offs);
   usb_control_msg(sd->usb_dev,
                   usb_rcvctrlpipe(sd->usb_dev, 0),
                   gpio_val, USB_TYPE_VENDOR | USB_DIR_IN,
                   usbval, offs,
                   (u8 *)sd->bufr, 4,
                   100);
}

static void
int_cb(struct urb *urb)
{
   struct spi_tiny_usb *sd = urb->context;
   unsigned long flags;
   char *intrxbuf = kmalloc(32, GFP_KERNEL);
   if (!intrxbuf)
		printk(KERN_ALERT "Failed to create intrxbuf \n");
		
 //  		spin_lock_irqsave(&sd->err_lock, flags);
//		urb->status = 0;
	//	spin_unlock_irqrestore(&sd->err_lock, flags);
   
   printk(KERN_ALERT "urb interrupt is called \n");
   printk(KERN_ALERT "received data: %s \n", sd->int_in_buf);
   memcpy(intrxbuf, sd->int_in_buf, 32);
//   i2c_gpio_to_irq(&sd->chip, 3);
//   GPIO_irqNumber = gpio_to_irq(2);
//   pr_info("GPIO_irqNumber = %d\n", GPIO_irqNumber);
//   generic_handle_domain_irq(sd->chip.irq.domain, 2);
//    handle_simple_irq (sd->irq_descs[3]);
    
       if (irqyup == 1) {
       printk(KERN_ALERT "irq alive and fired\n");
       local_irq_save(flags);
       generic_handle_irq(GPIO_irqNumber);
       local_irq_restore(flags);
       }
//   printk(KERN_ALERT "received data: %s \n", sd->int_in_buf);
   printk(KERN_ALERT "received data intrxbuf: %s \n", intrxbuf);
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

//    usleep_range(1000, 1200);
//    schedule_work(&data->work2);
//    usleep_range(1000, 1200);
    usb_control_msg(data->usb_dev,
                   usb_rcvctrlpipe(data->usb_dev, 0),
                   gpio_val, USB_TYPE_VENDOR | USB_DIR_IN,
                   usbval, offs,
                   rxbuf, 4,
                   100);
              
    memcpy(data->bufr, rxbuf, 4);          
                                      
    retval = rxbuf[0];
    retval1 = rxbuf[1];
    retval2 = rxbuf[2];
    retval3 = rxbuf[3];
    printk("buf0 =  %d \n", retval);
    printk("buf1 =  %d \n", retval1);
    printk("buf2 =  %d \n", retval2);
    printk("buf3 =  %d \n", retval3);

    kfree(rxbuf);
//    kfree(data->bufr);
 
    return retval1 - 3; 

}

static int
_direction_output(struct gpio_chip *chip,
                  unsigned offset, int value)
{
	   struct spi_tiny_usb *data = container_of(chip, struct spi_tiny_usb,
                                      chip);
   printk("Setting pin to OUTPUT \n");
   
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
                                      
   printk("Setting pin to INPUT \n");


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
//   dev->irq_enabled[4] = false;
//	dev->irq.irq_enable = false;
//	usb_kill_urb(dev->int_in_urb);
}

static int usbirq_irq_set_type(struct irq_data *irqd, unsigned type)
{
    struct gpio_chip *chip = irq_data_get_irq_chip_data(irqd);
    struct spi_tiny_usb *data = container_of(chip, struct spi_tiny_usb,
                                      chip);
    int pin = irqd_to_hwirq(irqd);
    pr_info("irq pin = %d\n", pin);
//    GPIO_irqNumber = gpio_to_irq(pin);
    irqt = type;
    pr_info("GPIO_irqNumber = %d\n", GPIO_irqNumber);
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
           schedule_work(&data->work);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		   usbval = 9;
           offs = 6;
           gpio_val = 9;
           schedule_work(&data->work);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}    


const char *gpio_names[] = { "LED", "usbGPIO2", "BTN", "usbGPIO4", "IRQpin" };

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,18,0)    
static const struct irq_chip usb_gpio_irqchip = {
	.name = "usbgpio-irq",
	.irq_enable =  usb_gpio_irq_enable,
	.irq_disable = usb_gpio_irq_disable,
	.irq_set_type = usbirq_irq_set_type,
	.flags = IRQCHIP_IMMUTABLE, GPIOCHIP_IRQ_RESOURCE_HELPERS,
};
#endif



static void usbspi_delete(struct kref *kref)
{
	struct spi_tiny_usb *dev = to_usbspi_dev(kref);

	usb_put_intf(dev->interface);
	usb_put_dev(dev->usb_dev);
	kfree(dev);
}

struct bulk_desc {
	unsigned int dir_out;
	void *data;
	int len;
	int act_len;
	int timeout;
};

static void spi_tiny_usb_free(struct spi_tiny_usb *priv)
{
	usb_put_dev(priv->usb_dev);
	kfree(priv);
}

// static int usb_read(struct spi_tiny_usb *dev, int cmd, int value, int index, void *data, int len);
static int usb_write(struct spi_tiny_usb *dev, int cmd, int value, int index, void *data, int len);
static int usbspi_bulk_xfer(struct usb_interface *interface, struct bulk_desc *desc);
static int usbspi_write_data(struct usb_interface *interface,
			   const char *buf, size_t len);
static int usbspi_read_data(struct usb_interface *interface, void *buf, size_t len);

static int usbspi_write_data(struct usb_interface *interface,
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

/* ----- begin of spi layer ---------------------------------------------- */



static int spi_tiny_usb_xfer_one(struct spi_master *master, struct spi_message *m)
{
	struct spi_tiny_usb *priv = spi_master_get_devdata(master);
	struct usb_interface *interface; 
	struct spi_transfer *t;
	int spi_flags;
	int ret = 0;
    static int oldspihz;
    int wait;
    interface = usb_get_intfdata(priv->interface);

	m->actual_length = 0;

	spi_flags = FLAGS_BEGIN;
	list_for_each_entry(t, &m->transfers, transfer_list) {
		if (list_is_last(&t->transfer_list, &m->transfers))
			spi_flags |= FLAGS_END;

        if (priv->oldlsb != (priv->spidev->mode & SPI_LSB_FIRST)) {
        printk("mode lsb claims change \r\n");
            if (priv->oldlsb != 0x08) {
                usb_write(priv, 10, 0x08, 0x08, NULL, 0);
                } else {
                usb_write(priv, 10, 0x00, 0x00, NULL, 0);
            }
        }

        priv->oldlsb = (priv->spidev->mode & SPI_LSB_FIRST);

        if (oldspihz != t->speed_hz) {
        printk("setting clock \r\n");
        wait = usb_write(priv, 7, (t->speed_hz / 10000), (t->speed_hz / 10000), NULL, 0);
           if (wait) {
           ;
           }
        }
        
        oldspihz = t->speed_hz;
        
//    printk("spi->mode = %d\r\n", priv->spidev->mode);
//    printk("spi->bits_per_word = %d\r\n", priv->spidev->bits_per_word);
//    printk("priv->mode_bits = %d\r\n", priv->master->mode_bits);    
	if (priv->last_mode != priv->spidev->mode) {
		u8 spi_mode = priv->spidev->mode & (SPI_CPOL | SPI_CPHA);
		printk("spi_mode = %d\r\n", spi_mode);
        wait = usb_write(priv, 8, spi_mode, spi_mode, NULL, 0);
           if (wait) {
           ;
           }
		}
		
    priv->last_mode = priv->spidev->mode;
    
   if (priv->last_bpw != priv->spidev->bits_per_word) {
      wait = usb_write(priv, 9, priv->spidev->bits_per_word, priv->spidev->bits_per_word, NULL, 0);
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

		if (t->tx_buf) {
			usb_write(priv, 4, t->len, t->len, NULL, 0);
		dev_dbg(&master->dev,
			"tx: %p rx: %p len: %d speed: %d flags: %d delay: %d\n", t->tx_buf,
			t->rx_buf, t->len, t->speed_hz, spi_flags, t->delay.value);
//			printk("TX break between control and bulk tlen = %d\r\n", t->len);
//            printk("TX bulk start \r\n");
            usbspi_write_data(priv->interface, t->tx_buf, t->len);
		}else {
			void *txbuf = kmalloc(t->len, GFP_KERNEL);
			memset(txbuf, 0x00, t->len);
			usbspi_write_data(priv->interface, txbuf, t->len);
			kfree(txbuf);
		}

		if (t->rx_buf) {
//		    printk("RX bulk start \r\n");
			usb_write(priv, 5, t->len, t->len, NULL, 0);
            usbspi_read_data(priv->interface, t->rx_buf, t->len);
		}

        spi_finalize_current_message(master);
		m->actual_length += t->len;

		if (t->delay.value)
			udelay(t->delay.value);

		spi_flags = 0;

		if (t->cs_change)
			spi_flags |= FLAGS_BEGIN;
	}

	m->status = ret;
//	spi_finalize_current_message(master);

	return 0;
}

static const struct usb_device_id spi_tiny_usb_table[] = {
	{USB_DEVICE(VID, PID)},
	{}
};

MODULE_DEVICE_TABLE(usb, spi_tiny_usb_table);

/*
static int
usb_read(struct spi_tiny_usb *dev, int cmd, int value, int index, void *data, int len)
{
	return usb_control_msg(dev->usb_dev, usb_rcvctrlpipe(dev->usb_dev, 0),
			       cmd,
			       USB_TYPE_VENDOR | USB_RECIP_INTERFACE |
			       USB_DIR_IN, value, index, data, len, 2000);
}
*/

static int usb_write(struct spi_tiny_usb *dev, int cmd, int value, int index, void *data, int len)
{
	return usb_control_msg(dev->usb_dev, usb_sndctrlpipe(dev->usb_dev, 0),
			       cmd, USB_TYPE_VENDOR | USB_RECIP_INTERFACE,
			       value, index, data, len, 2000);
}


static int spi_tiny_usb_probe(struct usb_interface *interface,
			      const struct usb_device_id *id)
{
    struct device *dev = &interface->dev; 
    struct spi_master *master = container_of(dev, struct spi_master, dev);
	struct spi_tiny_usb *priv = spi_master_get_devdata(master);
	struct usb_host_interface *iface_desc;
    struct usb_endpoint_descriptor *endpoint, *bulk_in, *bulk_out;
    struct usbspi_intf_info *info;
    struct gpio_irq_chip *girq;
	int ret = -ENOMEM;
	u16 version;
    int retval;
	int inf;
	int i;
	int rc;
	dev_dbg(&interface->dev, "probing usb device\n");

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	priv->usb_dev = usb_get_dev(interface_to_usbdev(interface));
	inf = interface->cur_altsetting->desc.bInterfaceNumber;
	priv->interface = interface;
	iface_desc = interface->cur_altsetting;
/*	
	for (i = 0; i < iface_desc->desc.bNumEndpoints; i++) {
		endpoint = &iface_desc->endpoint[i].desc;

		if (usb_endpoint_is_bulk_out(endpoint))
			priv->bulk_out = endpoint->bEndpointAddress;

		if (usb_endpoint_is_bulk_in(endpoint)) {
			priv->bulk_in = endpoint->bEndpointAddress;
			priv->bulk_in_sz = usb_endpoint_maxp(endpoint);
		}
	}
*/
	if (inf == 1) {
 //  priv->usb_dev = usb_get_dev(usb_dev);
   priv->usb_dev = usb_get_dev(interface_to_usbdev(interface));
//   priv->int_in_endpoint = endpoint;
    endpoint = &iface_desc->endpoint[2].desc;
   retval = usb_find_int_in_endpoint(interface->cur_altsetting, &endpoint);
   if (retval) {
//   pr_err(&interface, "Could not find int-in endpoint\n");
   return retval;
   }
   priv->int_in_endpoint = endpoint;
//   priv->int_in_endpoint = int_in_endpoint;
   priv->int_in_endpointAddr = endpoint->bEndpointAddress;
   priv->int_in_endpoint->wMaxPacketSize = 0x10;
   printk(KERN_INFO "MaxPacketSize: %d \n", priv->int_in_endpoint->wMaxPacketSize);
//   priv->int_in_endpoint->bEndpointAddress = 130;
   printk(KERN_INFO "priv->int_in_endpointAddr: %d \n", priv->int_in_endpointAddr);
   printk(KERN_INFO "priv->int_in_endpoint->bEndpointAddress: %d \n", priv->int_in_endpoint->bEndpointAddress);
   // allocate our urb for interrupt in 
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
     


   /// gpio_chip struct info is inside KERNEL/include/linux/gpio/driver.h
   priv->chip.label = "vusb-gpio"; //name for diagnostics
//   data->chip.priv = &data->usb_dev->priv; // optional privice providing the GPIOs
   priv->chip.parent = &interface->dev;
   priv->chip.owner = THIS_MODULE; // helps prevent removal of modules exporting active GPIOs, so this is required for proper cleanup
   priv->chip.base = -1; // identifies the first GPIO number handled by this chip; 
   // or, if negative during registration, requests dynamic ID allocation.
   // i was getting 435 on -1.. nice. Although, it is deprecated to provide static/fixed base value. 

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
   //TODO  implement it later in firmware
   priv->chip.direction_input = _direction_input;
   priv->chip.direction_output = _direction_output;
   priv->chip.to_irq = i2c_gpio_to_irq;
   priv->chip.names = gpio_names;
   #if LINUX_VERSION_CODE <= KERNEL_VERSION(5,18,0)    
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
//		pr_err(&interface->priv, "Cannot allocate an IRQ desc \n");
		return rc;
	}
	
//   girq->irq_num = rc;
	
   if (gpiochip_add(&priv->chip) < 0)
     {
        printk(KERN_ALERT "Failed to add gpio chip \n");
     }
   else
     {
        printk(KERN_INFO "Able to add gpiochip: %s \n", priv->chip.label);
     }

//   gpio_direction_input(5);
//   gpio_export_link(data->chip, 3, BTN);
   i2c_gpio_to_irq(&priv->chip, 4);
  
   INIT_WORK(&priv->work, _gpio_work_job);
   INIT_WORK(&priv->work2, _gpio_work_job2);
   
    } else {


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

	// SPI master
	priv->last_bpw = 8;
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
	priv->master->max_speed_hz = 100000000;
	priv->master->min_speed_hz = 300000;
	// priv->master->dev.platform_data = priv;
	spi_master_set_devdata(priv->master, priv);

	ret = spi_register_master(priv->master);
	if (ret)
		goto error2;
// also can use bk4
//	strcpy(priv->spiinfo.modalias, "spi-petra");
    if (name) {
        strcpy(priv->spiinfo.modalias, name);
    } else {
        strcpy(priv->spiinfo.modalias, "spi-petra");
    }
	priv->spiinfo.max_speed_hz = 100000000;
	priv->spiinfo.chip_select = 0;
	priv->spiinfo.mode = SPI_MODE_0;

	priv->spiinfo.controller_data = priv;
	priv->spidev = spi_new_device(priv->master, &priv->spiinfo);
	if (!priv->spidev)
		goto error2;
	dev_info(&interface->dev, "added new SPI device\n");

	dev_info(&interface->dev,
		 "USBSPI device now attached to spidev%d.%d",
		 interface->minor, priv->spiinfo.chip_select);
		 
  
 } //elseif inf 0

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
    int minor = interface->minor;
    int inf;
    irqyup = 0;
    
//	priv->dev = usb_get_intfdata(interface);
    inf = priv->interface->cur_altsetting->desc.bInterfaceNumber;
    printk(KERN_INFO "inf = %d \n", inf);
//	priv = usb_get_intfdata(interface);
		if (inf == 1) {
	
    printk(KERN_INFO "b4 set infdata  \n");
	usb_set_intfdata(interface, NULL);
	printk(KERN_INFO "b4 remove sysfs: \n");

	/* give back our minor */
//	printk(KERN_INFO "b4 gpiochip remove: \n");
	gpiochip_remove(&dev->chip);
    usb_kill_urb(dev->int_in_urb);

    }
	mutex_lock(&priv->io_mutex);
	priv->disconnected = 1;
	mutex_unlock(&priv->io_mutex);

    kref_put(&priv->kref, usbspi_delete);
    dev_info(&interface->dev, "USBSPI #%d now disconnected", minor);

	dev_dbg(&interface->dev, "spi_unregister_master\n");
	spi_unregister_master(priv->master);
	mutex_lock(&priv->io_mutex);
 	priv->interface = NULL;
	usb_set_intfdata(interface, NULL);
	mutex_unlock(&priv->io_mutex);

	usb_put_dev(priv->usb_dev);
	ida_simple_remove(&usbspi_devid_ida, priv->id);
	dev_dbg(&interface->dev, "disconnected\n");
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

MODULE_AUTHOR("Ben Maddockd <krystian.duzynski@gmail.com>");
MODULE_DESCRIPTION("spi-tiny-usb driver 16ton edition");
MODULE_LICENSE("GPL");
