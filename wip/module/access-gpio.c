#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
//#include <linux/gpio/consumer.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/property.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include "intf.h"
#include <linux/gpio/driver.h>
#include <linux/of_gpio.h>
#include <linux/gpio/machine.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Amitesh Singh");
MODULE_DESCRIPTION("Access GPIO created by platform-gpio module");
MODULE_VERSION("0.1");

#define SPI_INTF_DEVNAME	"spi-tiny-usb"



				
struct tinyusb_intf_priv {
    struct usb_interface	*intf;
	struct tinyusb_intf_info		*info;
	struct platform_device		*spi_pdev;
};


struct tinyusb_intf_info {
	const void *plat_data; /* optional, passed to probe() */
};


struct gpio_desc *desc;
static long gpio_no = 1;

static struct dev_io_desc_data tinyusb_spi_bus_dev_io[] = {
       { "ce", 1, GPIO_ACTIVE_HIGH },
       { "csn", 2, GPIO_ACTIVE_LOW },
};

static const struct tinyusb_spi_dev_data tinyusb_spi_dev_data[] = {
	{
	.magic		= USBSPI_IO_DESC_MAGIC,
	.desc		= tinyusb_spi_bus_dev_io,
	.desc_len	= ARRAY_SIZE(tinyusb_spi_bus_dev_io),
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

static const struct software_node nrf24_node = {
	.properties = nrf24_properties,
};

static const struct software_node mcp2515_node = {
	.properties = mcp2515_properties,
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

static const struct tinyusb_spi_platform_data tinyusb_spi_bus_plat_data = {
//    .ops		= &tinyusbh_intf_ops,
    .spi_info	= tinyusb_spi_bus_info,
    .spi_info_len	= ARRAY_SIZE(tinyusb_spi_bus_info),
};



static struct platform_device *tinyusb_dev_register(struct tinyusb_intf_priv *priv,
				const struct tinyusb_spi_platform_data *pd)
{
	struct platform_device *pdev;
	int ret;
	
	pdev = platform_device_alloc(SPI_INTF_DEVNAME, 0);
	if (!pdev)
		return NULL;	


	priv->spi_pdev = pdev;
	ret = platform_device_add_data(pdev, pd, sizeof(*pd));
	if (ret)
		goto err;

	pdev->id = priv->id;

	pdev = platform_device_alloc(SPI_INTF_DEVNAME, 0);
	if (!pdev)
		return NULL;

	pdev->dev.parent = THIS_MODULE;
	pdev->dev.fwnode = NULL;
	
	
	ret = platform_device_add(pdev);

		
    return pdev;	
err:
	platform_device_put(pdev);
	return ERR_PTR(ret);
}	

static int __init
_gpio_access_init(void)
{
    struct tinyusbh_intf_priv *priv;
    struct platform_device *pdev;
    struct tinyusb_intf_info *pldata;
   int status;
   int result;
	

	pdev = tinyusb_dev_register(pldata, plat_data);
	if (IS_ERR(pdev)) {
		return PTR_ERR(pdev);
	}

	priv->spi_pdev = pdev;

   return 0;
}


	/*
static const struct tinyusbh_intf_info tinyusb_spi_bus_intf_info = {
    .probe  = tinyusbh_intf_spi_probe,
    .remove  = tinyusbh_intf_spi_remove,
    .plat_data  = &tinyusb_spi_bus_plat_data,
};
*/

static struct platform_driver tinyusbh_intf_spi_driver = {
     .probe = _gpio_access_init,
     .remove = _gpio_access_exit,
     .plat_data  = &tinyusb_spi_bus_plat_data,
     .driver = {
          .name = tinyusbh_intf_spi, //platform_device will also use same name
          .owner = THIS_MODULE, //good practice to declare it
          .driver_info = (kernel_ulong_t)&tinyusb_spi_bus_intf_info
     },
};

	
static void __exit
_gpio_access_exit(void)
{
   printk(KERN_INFO "GPIO access exit");

}

module_init(_gpio_access_init);
module_exit(_gpio_access_exit);
