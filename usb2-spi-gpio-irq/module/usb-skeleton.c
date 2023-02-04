// SPDX-License-Identifier: GPL-2.0
/*
 * USB Skeleton driver - 2.2
 *
 * Copyright (C) 2001-2004 Greg Kroah-Hartman (greg@kroah.com)
 *
 * This driver is based on the 2.6.3 version of drivers/usb/usb-skeleton.c
 * but has been rewritten to be easier to read and use.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/module.h>
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

/* Define these values to match your devices */
#define USB_SKEL_VENDOR_ID	0x1d50
#define USB_SKEL_PRODUCT_ID	0x6018

/* table of devices that work with this driver */
static const struct usb_device_id skel_table[] = {
	{ USB_DEVICE_AND_INTERFACE_INFO(0x1d50, 0x6018, 0xFF, 0x00, 0x00) },   /* FTDI */
	{ USB_DEVICE(0x0403, 0xc631) },
//	{ USB_DEVICE_AND_INTERFACE_INFO(0x0403, 0xc631, 0x00, 0x00, 0x00) },
	{ }					/* Terminating entry */
};
MODULE_DEVICE_TABLE(usb, skel_table);

static int i2c_gpio_to_irq(struct gpio_chip *chip, unsigned offset);
/* Get a minor range for your devices from the usb maintainer */
#define USB_SKEL_MINOR_BASE	192

/* our private defines. if this grows any larger, use your own .h file */
#define MAX_TRANSFER		(PAGE_SIZE - 512)
/*
 * MAX_TRANSFER is chosen so that the VM is not stressed by
 * allocations > PAGE_SIZE and the number of packets in a page
 * is an integer 512 is the largest possible packet on EHCI
 */
#define WRITES_IN_FLIGHT	8
/* arbitrarily chosen */

/* Structure to hold all of our device specific stuff */
struct usb_skel {
	struct usb_device	*usb_dev;			/* the usb device for this device */
	struct usb_interface	*interface;		/* the interface for this device */
	struct semaphore	limit_sem;		/* limiting the number of writes in progress */
	struct usb_anchor	submitted;		/* in case we need to retract our submissions */
	struct urb		*bulk_in_urb;		/* the urb to read data with */
	unsigned char           *bulk_in_buffer;	/* the buffer to receive data */
	size_t			bulk_in_size;		/* the size of the receive buffer */
	size_t			bulk_in_filled;		/* number of bytes in the buffer */
	size_t			bulk_in_copied;		/* already copied to user space */
	__u8			bulk_in_endpointAddr;	/* the address of the bulk in endpoint */
	__u8			bulk_out_endpointAddr;	/* the address of the bulk out endpoint */
	int			errors;			/* the last request tanked */
	bool			ongoing_read;		/* a read is going on */
	spinlock_t		err_lock;		/* lock for errors */
	struct kref		kref;
	struct mutex		io_mutex;		/* synchronize I/O with disconnect */
	unsigned long		disconnected:1;
	wait_queue_head_t	bulk_in_wait;		/* to wait for an ongoing read */
     struct work_struct work;
     struct work_struct work2;
     struct gpio_chip chip; //this is our GPIO chip
     bool    hwirq;
     int                      gpio_irq_map[4]; // GPIO to IRQ map (gpio_num elements)
     struct usb_endpoint_descriptor *int_in_endpoint;
	struct urb		*int_in_urb;		/* the urb to read data with */
	unsigned char           *int_in_buf;	/* the buffer to receive data */
	size_t			int_in_size;		/* the size of the receive buffer */
	size_t			int_in_filled;		/* number of bytes in the buffer */
	size_t			int_in_copied;		/* already copied to user space */
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
     int               irq_hw;                             // IRQ for GPIO with hardware IRQ (default -1)
    
     struct pwm_chip pwmchip;
     int               duty_cycle;
     int               period;
     int               polarity;
     int duty_ns, period_ns;
    
     u8 bufr[4];

};
#define to_skel_dev(d) container_of(d, struct usb_skel, kref)

static struct usb_driver skel_driver;
static void skel_draw_down(struct usb_skel *dev);

static void skel_delete(struct kref *kref)
{
    int inf;
	struct usb_skel *dev = to_skel_dev(kref);
	inf = dev->interface->cur_altsetting->desc.bInterfaceNumber;
    printk(KERN_INFO "skel delete inf = %d \n", inf);
	if (inf == 3) {
	    printk(KERN_INFO "b4 cancel work:\n");
	    cancel_work_sync(&dev->work);
        cancel_work_sync(&dev->work2);
    if (dev->int_in_urb != NULL) {
        usb_free_urb(dev->int_in_urb);
    }
    printk(KERN_INFO "after kill int urb:  \n");
    kfree(dev->int_in_buf);
	kfree(dev->bufr);
	printk(KERN_INFO "b4 inf3 usb puts:  \n");
    usb_put_intf(dev->interface);
    usb_put_dev(dev->usb_dev);
    }
    if (inf == 5) {
	usb_free_urb(dev->bulk_in_urb);
	printk(KERN_INFO "after kill bulk urb:  \n");
    printk(KERN_INFO "b4 inf3 usb puts:  \n");
	usb_put_intf(dev->interface);
	usb_put_dev(dev->usb_dev);
	kfree(dev->bulk_in_buffer);
    }
	kfree(dev);
}

static int skel_open(struct inode *inode, struct file *file)
{
	struct usb_skel *dev;
	struct usb_interface *interface;
	int subminor;
	int retval = 0;

	subminor = iminor(inode);

	interface = usb_find_interface(&skel_driver, subminor);
	if (!interface) {
		pr_err("%s - error, can't find device for minor %d\n",
			__func__, subminor);
		retval = -ENODEV;
		goto exit;
	}

	dev = usb_get_intfdata(interface);
	if (!dev) {
		retval = -ENODEV;
		goto exit;
	}

	retval = usb_autopm_get_interface(interface);
	if (retval)
		goto exit;

	/* increment our usage count for the device */
	kref_get(&dev->kref);

	/* save our object in the file's private structure */
	file->private_data = dev;

exit:
	return retval;
}

static int skel_release(struct inode *inode, struct file *file)
{
	struct usb_skel *dev;

	dev = file->private_data;
	if (dev == NULL)
		return -ENODEV;

	/* allow the device to be autosuspended */
	usb_autopm_put_interface(dev->interface);

	/* decrement the count on our device */
	kref_put(&dev->kref, skel_delete);
	return 0;
}

static int skel_flush(struct file *file, fl_owner_t id)
{
	struct usb_skel *dev;
	int res;

	dev = file->private_data;
	if (dev == NULL)
		return -ENODEV;

	/* wait for io to stop */
	mutex_lock(&dev->io_mutex);
	skel_draw_down(dev);

	/* read out errors, leave subsequent opens a clean slate */
	spin_lock_irq(&dev->err_lock);
	res = dev->errors ? (dev->errors == -EPIPE ? -EPIPE : -EIO) : 0;
	dev->errors = 0;
	spin_unlock_irq(&dev->err_lock);

	mutex_unlock(&dev->io_mutex);

	return res;
}

static void skel_read_bulk_callback(struct urb *urb)
{
	struct usb_skel *dev;
	unsigned long flags;

	dev = urb->context;

	spin_lock_irqsave(&dev->err_lock, flags);
	/* sync/async unlink faults aren't errors */
	if (urb->status) {
		if (!(urb->status == -ENOENT ||
		    urb->status == -ECONNRESET ||
		    urb->status == -ESHUTDOWN))
			dev_err(&dev->interface->dev,
				"%s - nonzero write bulk status received: %d\n",
				__func__, urb->status);

		dev->errors = urb->status;
	} else {
		dev->bulk_in_filled = urb->actual_length;
	}
	dev->ongoing_read = 0;
	spin_unlock_irqrestore(&dev->err_lock, flags);

	wake_up_interruptible(&dev->bulk_in_wait);
}

static int skel_do_read_io(struct usb_skel *dev, size_t count)
{
	int rv;

	/* prepare a read */
	usb_fill_bulk_urb(dev->bulk_in_urb,
			dev->usb_dev,
			usb_rcvbulkpipe(dev->usb_dev,
				dev->bulk_in_endpointAddr),
			dev->bulk_in_buffer,
			min(dev->bulk_in_size, count),
			skel_read_bulk_callback,
			dev);
	/* tell everybody to leave the URB alone */
	spin_lock_irq(&dev->err_lock);
	dev->ongoing_read = 1;
	spin_unlock_irq(&dev->err_lock);

	/* submit bulk in urb, which means no data to deliver */
	dev->bulk_in_filled = 0;
	dev->bulk_in_copied = 0;

	/* do it */
	rv = usb_submit_urb(dev->bulk_in_urb, GFP_KERNEL);
	if (rv < 0) {
		dev_err(&dev->interface->dev,
			"%s - failed submitting read urb, error %d\n",
			__func__, rv);
		rv = (rv == -ENOMEM) ? rv : -EIO;
		spin_lock_irq(&dev->err_lock);
		dev->ongoing_read = 0;
		spin_unlock_irq(&dev->err_lock);
	}

	return rv;
}

static ssize_t skel_read(struct file *file, char *buffer, size_t count,
			 loff_t *ppos)
{
	struct usb_skel *dev;
	int rv;
	bool ongoing_io;

	dev = file->private_data;

	if (!count)
		return 0;

	/* no concurrent readers */
	rv = mutex_lock_interruptible(&dev->io_mutex);
	if (rv < 0)
		return rv;

	if (dev->disconnected) {		/* disconnect() was called */
		rv = -ENODEV;
		goto exit;
	}

	/* if IO is under way, we must not touch things */
retry:
	spin_lock_irq(&dev->err_lock);
	ongoing_io = dev->ongoing_read;
	spin_unlock_irq(&dev->err_lock);

	if (ongoing_io) {
		/* nonblocking IO shall not wait */
		if (file->f_flags & O_NONBLOCK) {
			rv = -EAGAIN;
			goto exit;
		}
		/*
		 * IO may take forever
		 * hence wait in an interruptible state
		 */
		rv = wait_event_interruptible(dev->bulk_in_wait, (!dev->ongoing_read));
		if (rv < 0)
			goto exit;
	}

	/* errors must be reported */
	rv = dev->errors;
	if (rv < 0) {
		/* any error is reported once */
		dev->errors = 0;
		/* to preserve notifications about reset */
		rv = (rv == -EPIPE) ? rv : -EIO;
		/* report it */
		goto exit;
	}

	/*
	 * if the buffer is filled we may satisfy the read
	 * else we need to start IO
	 */

	if (dev->bulk_in_filled) {
		/* we had read data */
		size_t available = dev->bulk_in_filled - dev->bulk_in_copied;
		size_t chunk = min(available, count);

		if (!available) {
			/*
			 * all data has been used
			 * actual IO needs to be done
			 */
			rv = skel_do_read_io(dev, count);
			if (rv < 0)
				goto exit;
			else
				goto retry;
		}
		/*
		 * data is available
		 * chunk tells us how much shall be copied
		 */

		if (copy_to_user(buffer,
//        if (sprintf(buffer,
				 dev->bulk_in_buffer + dev->bulk_in_copied,
				 chunk))
			rv = -EFAULT;
		else
			rv = chunk;

		dev->bulk_in_copied += chunk;

		/*
		 * if we are asked for more than we have,
		 * we start IO but don't wait
		 */
		if (available < count)
			skel_do_read_io(dev, count - chunk);
	} else {
		/* no data in the buffer */
		rv = skel_do_read_io(dev, count);
		if (rv < 0)
			goto exit;
		else
			goto retry;
	}
exit:
	mutex_unlock(&dev->io_mutex);
	return rv;
}

static void skel_write_bulk_callback(struct urb *urb)
{
	struct usb_skel *dev;
	unsigned long flags;

	dev = urb->context;

	/* sync/async unlink faults aren't errors */
	if (urb->status) {
		if (!(urb->status == -ENOENT ||
		    urb->status == -ECONNRESET ||
		    urb->status == -ESHUTDOWN))
			dev_err(&dev->interface->dev,
				"%s - nonzero write bulk status received: %d\n",
				__func__, urb->status);

		spin_lock_irqsave(&dev->err_lock, flags);
		dev->errors = urb->status;
		spin_unlock_irqrestore(&dev->err_lock, flags);
	}

	/* free up our allocated buffer */
	usb_free_coherent(urb->dev, urb->transfer_buffer_length,
			  urb->transfer_buffer, urb->transfer_dma);
	up(&dev->limit_sem);
}

static ssize_t skel_write(struct file *file, const char *user_buffer,
			  size_t count, loff_t *ppos)
{
	struct usb_skel *dev;
	int retval = 0;
	struct urb *urb = NULL;
	char *buf = NULL;
	size_t writesize = min(count, (size_t)MAX_TRANSFER);

	dev = file->private_data;

	/* verify that we actually have some data to write */
	if (count == 0)
		goto exit;

	/*
	 * limit the number of URBs in flight to stop a user from using up all
	 * RAM
	 */
	if (!(file->f_flags & O_NONBLOCK)) {
		if (down_interruptible(&dev->limit_sem)) {
			retval = -ERESTARTSYS;
			goto exit;
		}
	} else {
		if (down_trylock(&dev->limit_sem)) {
			retval = -EAGAIN;
			goto exit;
		}
	}

	spin_lock_irq(&dev->err_lock);
	retval = dev->errors;
	if (retval < 0) {
		/* any error is reported once */
		dev->errors = 0;
		/* to preserve notifications about reset */
		retval = (retval == -EPIPE) ? retval : -EIO;
	}
	spin_unlock_irq(&dev->err_lock);
	if (retval < 0)
		goto error;

	/* create a urb, and a buffer for it, and copy the data to the urb */
	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb) {
		retval = -ENOMEM;
		goto error;
	}

	buf = usb_alloc_coherent(dev->usb_dev, writesize, GFP_KERNEL,
				 &urb->transfer_dma);
	if (!buf) {
		retval = -ENOMEM;
		goto error;
	}

	if (copy_from_user(buf, user_buffer, writesize)) {
		retval = -EFAULT;
		goto error;
	}

	/* this lock makes sure we don't submit URBs to gone devices */
	mutex_lock(&dev->io_mutex);
	if (dev->disconnected) {		/* disconnect() was called */
		mutex_unlock(&dev->io_mutex);
		retval = -ENODEV;
		goto error;
	}

	/* initialize the urb properly */
	usb_fill_bulk_urb(urb, dev->usb_dev,
			  usb_sndbulkpipe(dev->usb_dev, dev->bulk_out_endpointAddr),
			  buf, writesize, skel_write_bulk_callback, dev);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	usb_anchor_urb(urb, &dev->submitted);

	/* send the data out the bulk port */
	retval = usb_submit_urb(urb, GFP_KERNEL);
	mutex_unlock(&dev->io_mutex);
	if (retval) {
		dev_err(&dev->interface->dev,
			"%s - failed submitting write urb, error %d\n",
			__func__, retval);
		goto error_unanchor;
	}

	/*
	 * release our reference to this urb, the USB core will eventually free
	 * it entirely
	 */
	usb_free_urb(urb);


	return writesize;

error_unanchor:
	usb_unanchor_urb(urb);
error:
	if (urb) {
		usb_free_coherent(dev->usb_dev, writesize, buf, urb->transfer_dma);
		usb_free_urb(urb);
	}
	up(&dev->limit_sem);

exit:
	return retval;
}

static const struct file_operations skel_fops = {
	.owner =	THIS_MODULE,
	.read =		skel_read,
	.write =	skel_write,
	.open =		skel_open,
	.release =	skel_release,
	.flush =	skel_flush,
	.llseek =	noop_llseek,
};

/*
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with the driver core
 */

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
   struct usb_skel *sd = container_of(work, struct usb_skel, work);

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
   struct usb_skel *sd = container_of(work2, struct usb_skel, work2);

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
   struct usb_skel *sd = urb->context;
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
   struct usb_skel *data = container_of(chip, struct usb_skel,
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
   struct usb_skel *data = container_of(chip, struct usb_skel,
                                     chip);

   int retval, retval1, retval2, retval3;
   char *rxbuf = kmalloc(4, GFP_KERNEL);
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
	   struct usb_skel *data = container_of(chip, struct usb_skel,
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
   struct usb_skel *data = container_of(chip, struct usb_skel,
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
   struct usb_skel *data = container_of(chip, struct usb_skel,
                                      chip);
   GPIO_irqNumber = irq_create_mapping(data->chip.irq.domain, offset);
   pr_info("GPIO_irqNumber = %d\n", GPIO_irqNumber);

   return GPIO_irqNumber;
}

static void usb_gpio_irq_enable(struct irq_data *irqd)
{
	struct usb_skel *dev = irq_data_get_irq_chip_data(irqd);
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
    struct usb_skel *dev = irq_data_get_irq_chip_data(irqd);
    irqyup = 0;
    if (!dev->irq_enabled[4])
        return;
        
    dev->irq_enabled[4] = false;
}

static void usb_gpio_irq_disable(struct irq_data *irqd)
{
    struct gpio_chip *chip = irq_data_get_irq_chip_data(irqd);
    struct usb_skel *data = container_of(chip, struct usb_skel,
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
    struct usb_skel *data = container_of(chip, struct usb_skel,
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


static struct usb_class_driver skel_class = {
	.name =		"skel%d",
	.fops =		&skel_fops,
	.minor_base =	USB_SKEL_MINOR_BASE,
};

static ssize_t loop_show(struct device *dev, struct device_attribute *attr,
                          char *buf)
{
struct usb_skel *data = dev_get_drvdata(dev);
   usb_control_msg(data->usb_dev,
                   usb_sndctrlpipe(data->usb_dev, 0),
                   3, USB_TYPE_VENDOR | USB_DIR_IN,
                   3, 3,
                   NULL, 0,
                   1000);
                   
   return 0;
}

static DEVICE_ATTR_RO(loop);

static void remove_sysfs_attrs(struct usb_interface *interface)
{

			device_remove_file(&interface->dev, &dev_attr_loop);

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


static int skel_probe(struct usb_interface *interface,
		      const struct usb_device_id *id)
{
	struct usb_skel *dev;
	struct usb_device *usb_dev = interface_to_usbdev(interface);
    struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint, *int_in_endpoint, *bulk_in, *bulk_out;
	struct gpio_irq_chip *girq;
	int retval;
    int inf;
    int i;
    int rc;
	/* allocate memory for our device state and initialize it */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	kref_init(&dev->kref);
	sema_init(&dev->limit_sem, WRITES_IN_FLIGHT);
	mutex_init(&dev->io_mutex);
	spin_lock_init(&dev->err_lock);
	init_usb_anchor(&dev->submitted);
	init_waitqueue_head(&dev->bulk_in_wait);

	dev->usb_dev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = usb_get_intf(interface);
	iface_desc = interface->cur_altsetting;
    inf = dev->interface->cur_altsetting->desc.bInterfaceNumber;

    printk(KERN_INFO "inf = %d \n", inf);
    if (inf == 0) {
    printk(KERN_INFO "ignoring interface");
    return -ENODEV;
    }
	if (inf == 1) {
   //increase ref count, make sure u call usb_put_dev() in disconnect()
//   endpoint = &iface_desc->endpoint[0].desc;
   
   dev->usb_dev = usb_get_dev(usb_dev);
//   dev->int_in_endpoint = endpoint;
   retval = usb_find_int_in_endpoint(interface->cur_altsetting, &endpoint);
   if (retval) {
   dev_err(&interface->dev, "Could not find int-in endpoint\n");
   return retval;
   }
   int_in_endpoint = endpoint;
   dev->int_in_endpoint = int_in_endpoint;
   dev->int_in_endpointAddr = endpoint->bEndpointAddress;
   dev->int_in_endpoint->wMaxPacketSize = 0x10;
   printk(KERN_INFO "MaxPacketSize: %d \n", dev->int_in_endpoint->wMaxPacketSize);
//   dev->int_in_endpoint->bEndpointAddress = 130;
   printk(KERN_INFO "dev->int_in_endpointAddr: %d \n", dev->int_in_endpointAddr);
   printk(KERN_INFO "dev->int_in_endpoint->bEndpointAddress: %d \n", dev->int_in_endpoint->bEndpointAddress);
   // allocate our urb for interrupt in 
   dev->int_in_urb = usb_alloc_urb(0, GFP_KERNEL);
   //allocate the interrupt buffer to be used
   dev->int_in_buf = kmalloc(le16_to_cpu(dev->int_in_endpoint->wMaxPacketSize), GFP_KERNEL);

   //initialize our interrupt urb
   //notice the rcvintpippe -- it is for recieving data from device at interrupt endpoint
   usb_fill_int_urb(dev->int_in_urb, usb_dev,
                    usb_rcvintpipe(usb_dev, dev->int_in_endpoint->bEndpointAddress),
                    dev->int_in_buf,
                    le16_to_cpu(dev->int_in_endpoint->wMaxPacketSize),
                    int_cb, // this callback is called when we are done sending/recieving urb
                    dev,
                    (dev->int_in_endpoint->bInterval));

   usb_set_intfdata(interface, dev);

   printk(KERN_INFO "usb gpio irq is connected \n");

   i = usb_submit_urb(dev->int_in_urb, GFP_KERNEL);
   if (i)
     {
        printk(KERN_ALERT "Failed to submit urb \n");
     }
     


   /// gpio_chip struct info is inside KERNEL/include/linux/gpio/driver.h
   dev->chip.label = "vusb-gpio"; //name for diagnostics
//   data->chip.dev = &data->usb_dev->dev; // optional device providing the GPIOs
   dev->chip.parent = &interface->dev;
   dev->chip.owner = THIS_MODULE; // helps prevent removal of modules exporting active GPIOs, so this is required for proper cleanup
   dev->chip.base = -1; // identifies the first GPIO number handled by this chip; 
   // or, if negative during registration, requests dynamic ID allocation.
   // i was getting 435 on -1.. nice. Although, it is deprecated to provide static/fixed base value. 

   dev->chip.ngpio = 5; // the number of GPIOs handled by this controller; the last GPIO
   dev->chip.can_sleep = true; // 
   /*
      flag must be set iff get()/set() methods sleep, as they
    * must while accessing GPIO expander chips over I2C or SPI. This
    * implies that if the chip supports IRQs, these IRQs need to be threaded
    * as the chip access may sleep when e.g. reading out the IRQ status
    * registers.
    */
   dev->chip.set = _gpioa_set;
   dev->chip.get = _gpioa_get;
   //TODO  implement it later in firmware
   dev->chip.direction_input = _direction_input;
   dev->chip.direction_output = _direction_output;
   dev->chip.to_irq = i2c_gpio_to_irq;
   dev->chip.names = gpio_names;
   #if LINUX_VERSION_CODE <= KERNEL_VERSION(5,18,0)    
   dev->irq.name = "usbgpio-irq";
   dev->irq.irq_set_type = usbirq_irq_set_type;
   dev->irq.irq_enable = usb_gpio_irq_enable;
   dev->irq.irq_disable = usb_gpio_irq_disable;

	girq = &dev->chip.irq;
	girq->chip = &dev->irq;
	#else 
	girq = &dev->chip.irq;
    gpio_irq_chip_set_chip(girq, &usb_gpio_irqchip);
    #endif
	girq->parent_handler = NULL;
	girq->num_parents = 0;
	girq->parents = NULL;
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_simple_irq;
	
	rc = irq_alloc_desc(0);
	if (rc < 0) {
		dev_err(&interface->dev, "Cannot allocate an IRQ desc \n");
		return rc;
	}
	
//   girq->irq_num = rc;
	
   if (gpiochip_add(&dev->chip) < 0)
     {
        printk(KERN_ALERT "Failed to add gpio chip \n");
     }
   else
     {
        printk(KERN_INFO "Able to add gpiochip: %s \n", dev->chip.label);
     }

//   gpio_direction_input(5);
//   gpio_export_link(data->chip, 3, BTN);
   i2c_gpio_to_irq(&dev->chip, 4);
  
   INIT_WORK(&dev->work, _gpio_work_job);
   INIT_WORK(&dev->work2, _gpio_work_job2);
    }

	/* set up the endpoint information */
	/* use only the first bulk-in and bulk-out endpoints */
	if (inf == 2) {
	retval = usb_find_common_endpoints(interface->cur_altsetting,
			&bulk_in, &bulk_out, NULL, NULL);
	if (retval) {
		dev_err(&interface->dev,
			"Could not find both bulk-in and bulk-out endpoints\n");
		goto error;
	}

	dev->bulk_in_size = usb_endpoint_maxp(bulk_in);
	dev->bulk_in_endpointAddr = bulk_in->bEndpointAddress;
	dev_info(&interface->dev,
		 "USB Skeleton bulkin endpoint- %d",
		 dev->bulk_in_endpointAddr);
	dev->bulk_in_buffer = kmalloc(dev->bulk_in_size, GFP_KERNEL);
	if (!dev->bulk_in_buffer) {
		retval = -ENOMEM;
		goto error;
	}
	dev->bulk_in_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev->bulk_in_urb) {
		retval = -ENOMEM;
		goto error;
	}

	dev->bulk_out_endpointAddr = bulk_out->bEndpointAddress;
	dev_info(&interface->dev,
		 "USB Skeleton bulkout endpoint- %d",
		 dev->bulk_out_endpointAddr);
  retval = device_create_file(&interface->dev, &dev_attr_loop);
   if (retval) {
     goto error_create_file;
  }  
  
	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, dev);

	/* we can register the device now, as it is ready */
	retval = usb_register_dev(interface, &skel_class);
	if (retval) {
		/* something prevented us from registering this driver */
		dev_err(&interface->dev,
			"Not able to get a minor for this device.\n");
		usb_set_intfdata(interface, NULL);
		goto error;
	}

	/* let the user know what node this device is now attached to */
	dev_info(&interface->dev,
		 "USB Skeleton device now attached to USBSkel-%d",
		 interface->minor);
    }


	return 0;

error:
	/* this frees allocated memory */
	kref_put(&dev->kref, skel_delete);

	return retval;
	
error_create_file:
 //  kfree(buf);
   usb_put_dev(dev->usb_dev);
   usb_set_intfdata(interface, NULL);
   return -1; 
 
}

static void skel_disconnect(struct usb_interface *interface)
{
	struct usb_skel *dev;
	int minor = interface->minor;
    int inf;
    
    irqyup = 0;
	dev = usb_get_intfdata(interface);
    inf = dev->interface->cur_altsetting->desc.bInterfaceNumber;
    printk(KERN_INFO "inf = %d \n", inf);
	if (inf == 1) {
	
    printk(KERN_INFO "b4 set infdata  \n");
	usb_set_intfdata(interface, NULL);
	printk(KERN_INFO "b4 remove sysfs: \n");
    remove_sysfs_attrs(interface);
	/* give back our minor */
//	printk(KERN_INFO "b4 gpiochip remove: \n");
//	gpiochip_remove(&dev->chip);
//    usb_kill_urb(dev->int_in_urb);
	printk(KERN_INFO "b4usb inf3 deregister: \n");
	usb_deregister_dev(interface, &skel_class);
    }
    if (inf == 2) {
    printk(KERN_INFO "inf5 mutex disconnect: \n");
	/* prevent more I/O from starting */
	mutex_lock(&dev->io_mutex);
	dev->disconnected = 1;
	mutex_unlock(&dev->io_mutex);


//	usb_kill_urb(dev->bulk_in_urb);
	usb_kill_anchored_urbs(&dev->submitted);
    printk(KERN_INFO "b4usb inf5 deregister: \n");
	usb_deregister_dev(interface, &skel_class);
	/* decrement our usage count */
	}
	kref_put(&dev->kref, skel_delete);
    
	dev_info(&interface->dev, "USB Skeleton #%d now disconnected", minor);
}

static void skel_draw_down(struct usb_skel *dev)
{
	int time;

	time = usb_wait_anchor_empty_timeout(&dev->submitted, 1000);
	if (!time)
		usb_kill_anchored_urbs(&dev->submitted);
	usb_kill_urb(dev->bulk_in_urb);
}

static int skel_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct usb_skel *dev = usb_get_intfdata(intf);

	if (!dev)
		return 0;
	skel_draw_down(dev);
	return 0;
}

static int skel_resume(struct usb_interface *intf)
{
	return 0;
}

static int skel_pre_reset(struct usb_interface *intf)
{
	struct usb_skel *dev = usb_get_intfdata(intf);

	mutex_lock(&dev->io_mutex);
	skel_draw_down(dev);

	return 0;
}

static int skel_post_reset(struct usb_interface *intf)
{
	struct usb_skel *dev = usb_get_intfdata(intf);

	/* we are sure no URBs are active - no locking needed */
	dev->errors = -EPIPE;
	mutex_unlock(&dev->io_mutex);

	return 0;
}

static struct usb_driver skel_driver = {
	.name =		"skeleton",
	.probe =	skel_probe,
	.disconnect =	skel_disconnect,
	.suspend =	skel_suspend,
	.resume =	skel_resume,
	.pre_reset =	skel_pre_reset,
	.post_reset =	skel_post_reset,
	.id_table =	skel_table,
	.supports_autosuspend = 1,
};

module_usb_driver(skel_driver);

MODULE_LICENSE("GPL v2");
