#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/version.h>

#include <linux/module.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/machine.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/idr.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/sizes.h>
#include <linux/usb.h>
#include "spi-tiny-usb.h"
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/stringify.h>
#include <linux/property.h>

#define FLAGS_BEGIN 1
#define FLAGS_END   2

struct spi_tiny_usb {
	struct platform_device *pdev;
	struct usb_interface *intf;
	struct spi_controller *master;
	struct spi_device *spidev;
	const struct usbspi_intf_ops *iops;
	struct gpiod_lookup_table *lookup[5];
	struct gpio_desc **cs_gpios;
//	struct gpio_desc **dc_gpios;
//	struct gpio_desc **reset_gpios;
//	struct gpio_desc **interrupts_gpios;
//	struct gpio_desc **irq_gpios;

	u8 txrx_cmd;  //todo despite usb2 simplex add a txrx function
	u8 rx_cmd;
	u8 tx_cmd;
	u8 xfer_txbuf[SZ_512K];
	u8 xfer_rxbuf[SZ_512K];
	u16 last_mode;   //used to compare old/new spi modes if diff update controller
	u32 last_speed_hz; // same as above but for spi speed
	u16 last_bpw; // same as two above but for bits per word
	u16 oldlsb; // same same for msb/lsb
	int oldspihz;
};

void chcs(struct spi_device *spi, bool enable) {
//(void)spi;
struct spi_tiny_usb *priv = spi_master_get_devdata(spi->master);
//const struct usbspi_intf_ops *iops;
const struct usbspi_intf_ops *ops = priv->iops;
if (enable == true) {

ops->chcs(priv->intf, true);

printk("setting cs low\n");
}
if (enable == false) {

ops->chcs(priv->intf, false);

printk("setting cs high\n");
  }
}

/*

static int spi_tiny_usb_xfer_one(struct spi_master *master, struct spi_message *m)
{
	struct spi_tiny_usb *priv = spi_master_get_devdata(master);
	struct usb_interface *interface;
	struct spi_transfer *t;
	const struct usbspi_intf_ops *iops  = priv->iops;

	static int total = 0;
	int spi_flags;
	int ret = 0;
	int temp;
    int wait;
    int offset2 = 0;
    interface = usb_get_intfdata(priv->intf);


    m->actual_length = 0;

//    printk("bens oldspihz = %d\n", priv->oldspihz);
//    printk("bens gbase = %d\n", gbase);
//    printk("bens priv->gbase = %d\n", priv->gbase);   // FUNNY BOTH priv->chip.base AND THE CP'D priv->gpiobase ARE SET AND RETURN FINE I THINK SWITCHING INTERFACES
    // PLOWS IT THO IM NOT SURE HOW OR WHY gpiobase ALSO GETS CLOBBERED, UNLESS THE USB INTERFACES ARE LIMITED ON WHAT THEY CAN SEE IN WHICH CASE FUCK ME.
	printk("actual length aka total sent bytes from all segments = %d\n", m->actual_length);
	printk("frame length aka total num of bytes in this msg = %d\n", m->frame_length);

//    iops->lock(priv->intf);
//    chcs(priv->spidev, true);
//    iops->unlock(priv->intf);
//gpiod_set_raw_value_cansleep(priv->cs_gpios[0], true);
	spi_flags = FLAGS_BEGIN;
	list_for_each_entry(t, &m->transfers, transfer_list) {
		if (list_is_last(&t->transfer_list, &m->transfers))
			spi_flags |= FLAGS_END;

        if (priv->oldlsb != (priv->spidev->mode & SPI_LSB_FIRST)) {
        printk("mode lsb claims change \r\n");
            if (priv->oldlsb != 0x08) {

               //iops->ctrl_xfer(priv->intf, 160, 0x08, 0x08, NULL, 0);
              priv->iops->setup_data(priv->intf, 160, 0x08, 0x08);
                } else {
               priv->iops->setup_data(priv->intf, 160, 0x08, 0x08);
              //  iops->ctrl_xfer(priv->intf, 160, 0x00, 0x00, NULL, 0);
                priv->iops->setup_data(priv->intf, 160, 0x00, 0x00);

            }
        }

        priv->oldlsb = (priv->spidev->mode & SPI_LSB_FIRST);

        if (priv->oldspihz != t->speed_hz) {
        printk("setting clock \r\n");
        if ((t->speed_hz / 10000) > 9999) {
        t->speed_hz = 99990000;
        }

        iops->setup_data(priv->intf, 67, (t->speed_hz / 10000),  (t->speed_hz / 10000));


        }
        priv->oldspihz = t->speed_hz;


	if (priv->last_mode != priv->spidev->mode) {
		u8 spi_mode = priv->spidev->mode & (SPI_CPOL | SPI_CPHA);
		printk("spi_mode = %d\r\n", spi_mode);

        //wait = iops->ctrl_xfer(priv->intf, 68, spi_mode, spi_mode, NULL, 0);
        priv->iops->setup_data(priv->intf, 68, spi_mode, spi_mode);
           if (wait) {
           ;
           }

		}

    priv->last_mode = priv->spidev->mode;

   if (priv->last_bpw != priv->spidev->bits_per_word) {

   //   wait = iops->ctrl_xfer(priv->intf, 69, priv->spidev->bits_per_word, priv->spidev->bits_per_word, NULL, 0);
       priv->iops->setup_data(priv->intf, 69, priv->spidev->bits_per_word, priv->spidev->bits_per_word);
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
        printk("iside tx rx\n");
		if (t->len > 508) {
//		void *txbuf = kmalloc(512, GFP_KERNEL);
		void *rxbuf = kmalloc(t->len, GFP_KERNEL);
		offset2 = 0;
		while (t->len - offset2 > 508) {
		memcpy(priv->xfer_txbuf, t->tx_buf + offset2, 508);
		iops->write_data(priv->intf, priv->xfer_txbuf, 508);
		iops->setup_data(priv->intf, 65, 508, 508);
		(void)iops->read_data(priv->intf, priv->xfer_rxbuf, 508);
		memcpy(rxbuf + offset2, priv->xfer_txbuf, 508);
		offset2 += 508;
		m->actual_length += 508;
		}
		temp = t->len - offset2;
		memcpy(priv->xfer_txbuf, t->tx_buf + offset2, temp);
		iops->write_data(priv->intf, priv->xfer_txbuf, temp);
		iops->setup_data(priv->intf, 65, temp, temp);
		(void)iops->read_data(priv->intf, priv->xfer_rxbuf, temp);
		memcpy(rxbuf + offset2, priv->xfer_rxbuf, temp);
		memcpy(t->rx_buf, rxbuf, t->len);
		offset2 = 0;
		m->actual_length += temp;
        iops->lock(priv->intf);
//		kfree(txbuf);
		kfree(rxbuf);
		iops->unlock(priv->intf);
		} else {
		printk(" inside tx, tx: %p rx: %p len: %d total: %d\n", t->tx_buf,
			t->rx_buf, t->len, total);
			memcpy(priv->xfer_txbuf, t->tx_buf, t->len);
			wait = iops->write_data(priv->intf, t->tx_buf, t->len);
            if (wait) {
            ;
            }
            m->actual_length += t->len;
            iops->setup_data(priv->intf, 65, t->len, t->len);
			(void)iops->read_data(priv->intf, priv->xfer_rxbuf, t->len);
            memcpy(t->rx_buf, priv->xfer_rxbuf, t->len);
            total = total + 1;
		}

		spi_finalize_current_message(master);
		spi_flags = 0;
    	m->status = ret;
	    return 0;
		}


// IF TX


		if (t->tx_buf) {
		printk("iside tx \n");
		if (t->len > 508) {
//		void *txbuf = kmalloc(512, GFP_KERNEL);
		while (t->len - offset2 > 508) {
		memcpy(priv->xfer_txbuf, t->tx_buf + offset2, 508);

		iops->write_data(priv->intf, priv->xfer_txbuf, 508);

		offset2 += 508;
		m->actual_length += 508;
		}
		temp = t->len - offset2;
		memcpy(priv->xfer_txbuf, t->tx_buf + offset2, temp);

		iops->write_data(priv->intf, priv->xfer_txbuf, temp);

		offset2 = 0;
		m->actual_length += temp;

 //       iops->lock(priv->intf);
//		kfree(txbuf);
 //       iops->unlock(priv->intf);

		} else {
		printk(" inside tx, tx: %p rx: %p len: %d total: %d\n", t->tx_buf,
			t->rx_buf, t->len, total);
            memcpy(priv->xfer_txbuf, t->tx_buf + offset2, 508);
            wait = iops->write_data(priv->intf, priv->xfer_txbuf, t->len);

            m->actual_length += t->len;
            if (wait) {
            ;
            }
            total = total + 1;


			}
		  }

// IF RX

		if (t->rx_buf) {
		printk("iside rx\n");
		offset2 = 0;
		if (t->len > 508) {
//		void *rxbuf = kmalloc(t->len, GFP_KERNEL);
		while (t->len - offset2 > 508) {
//		void *txbuf = kmalloc(t->len, GFP_KERNEL);

		memset(priv->xfer_txbuf, 0x00, t->len);

		iops->write_data(priv->intf, priv->xfer_txbuf, t->len);

 //       iops->lock(priv->intf);
//		kfree(txbuf);
//        iops->unlock(priv->intf);

		iops->setup_data(priv->intf, 65, 508, 508);
		(void)iops->read_data(priv->intf, priv->xfer_rxbuf, 508);
		memcpy(t->rx_buf + offset2, priv->xfer_rxbuf, 508);

		offset2 += 508;
		m->actual_length += 508;
		}
//		void *txbuf = kmalloc(t->len, GFP_KERNEL);
		memset(priv->xfer_txbuf, 0x00, t->len);

		iops->write_data(priv->intf, priv->xfer_txbuf, t->len);

 //       iops->lock(priv->intf);
//		kfree(txbuf);
//        iops->unlock(priv->intf);
		temp = t->len - offset2;

		iops->setup_data(priv->intf, 65, temp, temp);
		(void)iops->read_data(priv->intf, priv->xfer_rxbuf, temp);

		memcpy(t->rx_buf + offset2, priv->xfer_rxbuf, temp);
//		memcpy(t->rx_buf, rxbuf, t->len);
		offset2 = 0;
		m->actual_length += temp;

 //       iops->lock(priv->intf);
//		kfree(rxbuf);
 //       iops->unlock(priv->intf);

		} else {
//		    void *txbuf = kmalloc(t->len, GFP_KERNEL);
		    printk("RX bulk start len = %d \r\n", t->len);
		    printk("frame length in rec total num of bytes in this msg = %d\n", m->frame_length);
		    memset(priv->xfer_txbuf, 0x00, t->len);

		    iops->write_data(priv->intf, priv->xfer_txbuf, t->len);

//            iops->lock(priv->intf);
//		    kfree(txbuf);
 //           iops->unlock(priv->intf);

			iops->setup_data(priv->intf, 65, t->len, t->len);

            (void)iops->read_data(priv->intf, priv->xfer_rxbuf, t->len);
            memcpy(t->rx_buf, priv->xfer_rxbuf, t->len);
            m->actual_length += t->len;
		}
		}
		printk("frame_length = %d actual_length = %d\n", m->frame_length, m->actual_length);
        spi_finalize_current_message(master);

		if (t->delay.value)
			udelay(t->delay.value);

		spi_flags = 0;

//		if (t->cs_change)
//			spi_flags |= FLAGS_BEGIN;

        }
 //       csset(NULL, false);
//    iops->lock(priv->intf);
//    chcs(priv->spidev, false);
//    iops->unlock(priv->intf);
	m->status = ret;
	return 0;
}
*/

static int usbspi_init_io(struct spi_controller *master, unsigned int dev_idx)
{
	struct spi_tiny_usb *priv = spi_controller_get_devdata(master);
	struct platform_device *pdev = priv->pdev;
	const struct usb_spi_platform_data *pd;
	const struct usbspi_dev_data *data;
	struct gpiod_lookup_table *lookup;
	size_t lookup_size, size;
	char *label;
	unsigned int i;
	u16 cs;

	pd = pdev->dev.platform_data;

	data = pd->spi_info[dev_idx].platform_data;
	if (!data || data->magic != USBSPI_IO_DESC_MAGIC)
		return 0;

	size = data->desc_len + 1;

	lookup_size = sizeof(*lookup) + size * sizeof(struct gpiod_lookup);
	lookup = devm_kzalloc(&pdev->dev, lookup_size, GFP_KERNEL);
	if (!lookup)
		return -ENOMEM;

	cs = pd->spi_info[dev_idx].chip_select;


	lookup->dev_id = devm_kasprintf(&pdev->dev, GFP_KERNEL, "spi%d.%d",
					master->bus_num, cs);
	if (!lookup->dev_id) {
	printk("!lookup->dev_id failed \n");
		devm_kfree(&pdev->dev, lookup);
		return -ENOMEM;
	}
	dev_dbg(&pdev->dev, "LOOKUP ID '%s'\n", lookup->dev_id);

	label = devm_kasprintf(&pdev->dev, GFP_KERNEL, "usbspi-gpio.%d",
			       pdev->id);
	if (!label) {
	    printk("no mem for labwls\n");
		devm_kfree(&pdev->dev, (void *)lookup->dev_id);
		devm_kfree(&pdev->dev, lookup);
		return -ENOMEM;
	}

    printk("spi tiny data desc_length %ld\n", data->desc_len);

	for (i = 0; i < data->desc_len; i++) {
		printk("con_id: '%s' idx: %d, flags: 0x%x\n",
			data->desc[i].con_id, data->desc[i].idx,
			data->desc[i].flags);
		lookup->table[i].key = label;
		lookup->table[i].chip_hwnum = data->desc[i].idx;
		lookup->table[i].idx = 0;
		lookup->table[i].con_id = data->desc[i].con_id;
		lookup->table[i].flags = data->desc[i].flags;
	}

	priv->lookup[cs] = lookup;
	gpiod_add_lookup_table(lookup);

	return 0;
}

size_t usegpiodesc(struct spi_device *spi);



size_t usegpiodesc(struct spi_device *spi) {
(void)spi;
return true;
}


static int usbspi_probe(struct platform_device *pdev)
{
	const struct usb_spi_platform_data *pd;
	struct device *dev = &pdev->dev;
	struct spi_controller *master;
	struct spi_tiny_usb *priv;
	struct gpio_desc *desc;
//	const struct usbspi_intf_ops *iops   = priv->iops;
	u8 num_cs, max_cs = 0;   //removed dc, reset, interrupts, irq,
	unsigned int i;
	int ret;
	int wait;
//	int ret2;
    int inf;


	pd = dev->platform_data;
	if (!pd) {
		dev_err(dev, "Missing platform data.\n");
		return -EINVAL;
	}

/*	if (!pd->ops ||
	    !pd->ops->read_data || !pd->ops->write_data ||
	    !pd->ops->lock || !pd->ops->unlock)
	    	return -EINVAL;

	if (pd->spi_info_len > 5)
		return -EINVAL;
*/
	/* Find max. slave chipselect number */
	printk("ld spi-info length  set to %ld\n", pd->spi_info_len);
	printk("lu spi info length set to %lu\n", pd->spi_info_len);
	printk("num_cs b4 making equal to spi info length %u\n",num_cs);
	num_cs = pd->spi_info_len;
	printk("and num_cs after %u\n",num_cs);
//	if (num_cs >= 5) {
//	num_cs = 4;
//	}
	printk("max_cs b4 loop %u\n",max_cs);
	for (i = 0; i < num_cs; i++) {
		if (max_cs < pd->spi_info[i].chip_select)
			max_cs = pd->spi_info[i].chip_select;
	}
    printk("max_cs after loop %u\n",max_cs);

	if (max_cs > 4) {
		printk( "u Invalid max CS in platform data: %u\n", max_cs);
		printk( "d Invalid max CS in platform data: %d\n", max_cs);
//		return -EINVAL;
	}
	printk( "CS count %u, max CS %u\n", num_cs, max_cs);
	max_cs += 1; /* including CS0 */
    printk("starting spi_alloc_master \n");
	master = spi_alloc_master(&pdev->dev, sizeof(*priv));
	if (!master)
	    printk("failed spi_alloc_master\n");
//		return -ENOMEM;
//    device_set_node(&master->dev, dev_fwnode(dev));     // HRM dont remember this
    printk("starting platform_set_drvdata \n");
	platform_set_drvdata(pdev, master);
    printk("starting spi_controller_get_devdat \n");
	priv = spi_controller_get_devdata(master);
	priv->master = master;
	priv->pdev = pdev;
	priv->intf = to_usb_interface(dev->parent);
	priv->iops = pd->ops;


	master->bus_num = -1;
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LOOP |
			    SPI_CS_HIGH | SPI_LSB_FIRST;
	master->num_chipselect = max_cs;
	master->min_speed_hz = 450;
//	master->min_speed_hz = 1000000;
	master->max_speed_hz = 30000000;
//	master->max_speed_hz = 25000000;
	master->bits_per_word_mask = SPI_BPW_MASK(8);
//	master->set_cs = chcs;
//	master->use_gpio_descriptors = usegpiodesc;
	master->transfer_one_message = spi_tiny_usb_xfer_one_two;
	master->auto_runtime_pm = false;


	priv->cs_gpios = devm_kcalloc(&master->dev, max_cs, sizeof(desc),
				      GFP_KERNEL);
	if (!priv->cs_gpios) {
	    printk("failed to set mem for storing cs\n");
		spi_controller_put(master);
		return -ENOMEM;
	}

	master->max_speed_hz = 6000000;
    printk("starting spi set speed control tx \n");
//	wait = iops->ctrl_xfer(priv->intf, 67, (master->max_speed_hz / 10000), (master->max_speed_hz / 10000), NULL, 0);
//       if (wait) {
//       ;
//       }
//	create_sysfs_attrs(priv->intf);
/*
	priv->dc_gpios = devm_kcalloc(&master->dev, dc, sizeof(desc),
				      GFP_KERNEL);

	priv->reset_gpios = devm_kcalloc(&master->dev, reset, sizeof(desc),
				      GFP_KERNEL);

	priv->interrupts_gpios = devm_kcalloc(&master->dev, interrupts, sizeof(desc),
				      GFP_KERNEL);

	priv->irq_gpios = devm_kcalloc(&master->dev, irq, sizeof(desc),
				      GFP_KERNEL);
*/

    printk("starting cs loop\n");

	for (i = 0; i < num_cs; i++) {
		unsigned int idx = pd->spi_info[i].chip_select;

		dev_dbg(&pdev->dev, "CS num: %u\n", idx);
		desc = devm_gpiod_get_index(&priv->pdev->dev, "spi-cs",
					    i, GPIOD_OUT_LOW);
		if (IS_ERR(desc)) {
			ret = PTR_ERR(desc);
			dev_err(&pdev->dev, "CS %u gpiod err: %d\n", i, ret);
			continue;
		}
		priv->cs_gpios[idx] = desc;
	}

    printk("starting spi_register_controller \n");
	ret = spi_register_controller(master);
	if (ret < 0) {
		printk("Failed to register spi master\n");
		spi_controller_put(master);
		return ret;
	}



	priv->last_mode = 0xffff;


     printk("starting loop for spi_info,  usbspi_init_io,spi_new_device \n");
	for (i = 0; i < pd->spi_info_len; i++) {
		struct spi_device *sdev;
		u16 cs;

		dev_dbg(&pdev->dev, "slave: '%s', CS: %u\n",
			pd->spi_info[i].modalias, pd->spi_info[i].chip_select);

		ret = usbspi_init_io(master, i);
		if (ret < 0) {
			printk("Can't add slave IO: %d\n", ret);
			continue;
		}

		sdev = spi_new_device(master, &pd->spi_info[i]);
		if (!sdev) {
			cs = pd->spi_info[i].chip_select;
			dev_warn(&pdev->dev, "Can't add slave '%s', CS %u\n",
				 pd->spi_info[i].modalias, cs);
			if (priv->lookup[cs]) {
				gpiod_remove_lookup_table(priv->lookup[cs]);
				priv->lookup[cs] = NULL;
			}
		}
	}

	return 0;
err:
	platform_set_drvdata(pdev, NULL);
	spi_unregister_controller(master);
	return ret;
}

static int usbspi_slave_release(struct device *dev, void *data)
{
	struct spi_device *spi = to_spi_device(dev);
	struct spi_tiny_usb *priv = data;
	u16 cs = spi->chip_select;

	dev_dbg(dev, "%s: remove CS %u\n", __func__, cs);
	spi_unregister_device(to_spi_device(dev));

	if (priv->lookup[cs])
		gpiod_remove_lookup_table(priv->lookup[cs]);
	return 0;
}

static int usbspi_remove(struct platform_device *pdev)
{
	struct spi_controller *master;
	struct spi_tiny_usb *priv;

	master = platform_get_drvdata(pdev);
	priv = spi_controller_get_devdata(master);
//    remove_sysfs_attrs(priv->intf);
	device_for_each_child(&master->dev, priv, usbspi_slave_release);

	spi_unregister_controller(master);
	return 0;
}

static const struct of_device_id usbspi_of_match[] = {
	{ .compatible = "usbspi,spi_plat_usb", },
	{},
};
MODULE_DEVICE_TABLE(of, usbspi_of_match);

static const struct spi_device_id usbspi_ids[] = {
	{ .name = "spi_plat_usb", (unsigned long)usbspi_probe },
	{},
};
MODULE_DEVICE_TABLE(spi, usbspi_ids);

static struct platform_driver spi_plat_usb = {
	.driver		= {
				.name	= "spi_plat_usb",
				.of_match_table = of_match_ptr(usbspi_of_match),
	},
	.probe		= usbspi_probe,
	.remove		= usbspi_remove,
};
module_platform_driver(spi_plat_usb);

MODULE_ALIAS("platform:spi_plat_usb");
MODULE_AUTHOR("Ben Maddocks <bm16ton@gmail.com>");
MODULE_DESCRIPTION("USBSPI master driver");
MODULE_LICENSE("GPL v2");
