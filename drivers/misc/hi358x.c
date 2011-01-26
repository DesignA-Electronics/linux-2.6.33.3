/*
 * hi358x.c
 *
 * The hi3585/hi3588 ARINC 429 receiver from Holt
 * Copyright (C) 2009 Andre Renaud <andre@bluewatersys.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/spi/spi.h>
#include <linux/spi/hi358x.h>

/* HI358x op codes */
#define HI358X_MASTER_RESET	0x01
#define HI358X_LABEL_RESET_ALL	0x02
#define HI358X_LABEL_SET_ALL	0x03
#define HI358X_LABEL_RESET	0x04
#define HI358X_LABEL_SET	0x05
#define HI358X_LABEL_BULK	0x06
#define HI358X_ACLK_SET		0x07
#define HI358X_READ		0x08
#define HI358X_READ_BULK	0x09
#define HI358X_READ_STATUS	0x0a
#define HI358X_READ_CONTROL	0x0b
#define HI358X_ACLK_GET		0x0c
#define HI358X_LABEL_READ	0x0d
#define HI358X_WRITE_BULK	0x0e
#define HI358X_WRITE_CONTROL	0x10
#define HI358X_WRITE_RESET	0x11
#define HI358X_WRITE_TRIGGER	0x12

/*
 * Handle a fixed length write up to 32 bits
 * FIXME - Add bulk write support
 */
static ssize_t hi358x_op_write(struct device *dev, 
			       struct device_attribute *devattr, 
			       const char* buf, size_t count,
			       unsigned char op, int plen)
{
	struct spi_device *spi = to_spi_device(dev);
	u32 val = simple_strtoul(buf, NULL, 0);
	u8 buffer[5];
	int i, res;

	buffer[0] = op;
	for (i = plen; i > 0; i--) {
		buffer[i] = val & 0xff;
		val = val >> 8;
	}
	
#if 0
	printk ("%s SPI > ", __FUNCTION__);
	for (i = 0; i < plen+1; i++) {
		printk("%02x ", buffer[i]);
	}
	printk("\n");
#endif
	
	res = spi_write(spi, buffer, plen+1);
	if (res < 0)
		return res;
	return count;
}

// Handle fixed-length read of up to 32 bits
static ssize_t hi358x_op_read(struct device *dev, 
			      struct device_attribute *devattr, 
			      char *buf, unsigned char op, int plen)
{
	struct spi_device *spi = to_spi_device(dev);
	u8 buffer[4];
	ssize_t status;
	int i, count;
	
#if 0
	printk("%s SPI > %02x", __FUNCTION__, op);
#endif
	
	status = spi_write_then_read(spi, &op, 1, buffer, plen);
	if (status < 0)
		return status;
	
#if 0
	printk(" < ");
	for (i = 0; i < plen; i++) {
		printk("%02x", buffer[i]);
	}
	printk("\n");
#endif
	
	count = sprintf(buf,"0x");
	for (i = 0; i < plen; i++) {
		count += sprintf(buf + count, "%02x", buffer[i]);
	}
	count += sprintf(buf + count, "\n");
	return count;
}

static ssize_t hi358x_reset(struct device *dev, 
			    struct device_attribute *devattr, 
			    const char *buf, size_t count)
{
	return hi358x_op_write(dev, devattr, buf, count,
			       HI358X_MASTER_RESET, 0);
}
static DEVICE_ATTR(reset, S_IWUSR, NULL, hi358x_reset);


static ssize_t hi358x_label_none(struct device *dev, 
				 struct device_attribute *devattr, 
				 const char *buf, size_t count)
{
	return hi358x_op_write(dev, devattr, buf, count, 
			       HI358X_LABEL_RESET_ALL, 0);
}
static DEVICE_ATTR(label_none, S_IWUSR, NULL, hi358x_label_none);


static ssize_t hi358x_label_all(struct device *dev, 
				struct device_attribute *devattr, 
				const char *buf, size_t count)
{
	return hi358x_op_write(dev, devattr, buf, count, 
			       HI358X_LABEL_SET_ALL, 0);
}
static DEVICE_ATTR(label_all, S_IWUSR, NULL, hi358x_label_all);


static ssize_t hi358x_label_clear(struct device *dev, 
				  struct device_attribute *devattr, 
				  const char *buf, size_t count)
{
	return hi358x_op_write(dev, devattr, buf, count, 
			       HI358X_LABEL_RESET, 1);
}
static DEVICE_ATTR(label_clear, S_IWUSR, NULL, hi358x_label_clear);


static ssize_t hi358x_label_set(struct device *dev, 
				struct device_attribute *devattr, 
				const char *buf, size_t count)
{
	return hi358x_op_write(dev, devattr, buf, count, HI358X_LABEL_SET, 1);
}
static DEVICE_ATTR(label_set, S_IWUSR, NULL, hi358x_label_set);

static ssize_t hi358x_set_aclk_div(struct device *dev, 
				   struct device_attribute *devattr, 
				   const char *buf, size_t count)
{
	return hi358x_op_write(dev, devattr, buf, count, HI358X_ACLK_SET, 1);
}
static ssize_t hi358x_get_aclk_div(struct device *dev, 
				   struct device_attribute *devattr, char *buf)
{
	return hi358x_op_read(dev, devattr, buf, HI358X_ACLK_GET, 1);
}
static DEVICE_ATTR(aclk_div, S_IWUSR | S_IRUGO, hi358x_get_aclk_div,
		   hi358x_set_aclk_div);


static ssize_t hi358x_fifo_read(struct device *dev, 
				struct device_attribute *devattr, char *buf)
{
	return hi358x_op_read(dev, devattr, buf, HI358X_READ, 4);
}
static ssize_t hi358x_fifo_write(struct device *dev, 
				 struct device_attribute *devattr, 
				 const char *buf, size_t count)
{
	return hi358x_op_write(dev, devattr, buf, count, HI358X_WRITE_BULK, 4);
}
static DEVICE_ATTR(fifo_ro, S_IRUGO, hi358x_fifo_read, NULL);
static DEVICE_ATTR(fifo_wo, S_IWUSR, NULL, hi358x_fifo_write);
static DEVICE_ATTR(fifo_rw, S_IWUSR | S_IRUGO, hi358x_fifo_read, 
		   hi358x_fifo_write);

static ssize_t hi358x_get_status(struct device *dev, 
				 struct device_attribute *devattr, char *buf)
{
	return hi358x_op_read(dev, devattr, buf, HI358X_READ_STATUS, 1);
}
static DEVICE_ATTR(status, S_IRUGO, hi358x_get_status, NULL);

static ssize_t hi358x_set_control(struct device *dev, 
				  struct device_attribute *devattr, 
				  const char *buf, size_t count)
{
	return hi358x_op_write(dev, devattr, buf, count, 
			       HI358X_WRITE_CONTROL, 2);
}
static ssize_t hi358x_get_control(struct device *dev, 
				  struct device_attribute *devattr, char *buf)
{
	return hi358x_op_read(dev, devattr, buf, HI358X_READ_CONTROL, 2);
}
static DEVICE_ATTR(control, S_IWUSR | S_IRUGO, hi358x_get_control,
		   hi358x_set_control);

static ssize_t hi358x_label_read(struct device *dev, 
				 struct device_attribute *devattr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	u8 buffer[33];
	int i, count;	
	int status;
	struct spi_message message;
	struct spi_transfer x;

	for (i = 0; i < sizeof(buffer); i++)
		buffer[i] = i;
	
	spi_message_init(&message);
	memset(&x, 0, sizeof x);
	x.len = 33;
	spi_message_add_tail(&x, &message);

	buffer[0] = HI358X_LABEL_READ;
	x.tx_buf = buffer;
	x.rx_buf = buffer;

#if 0
	printk("%s SPI > %02x < ", __FUNCTION__, buffer[0]);
#endif
	
	/* do the i/o */
	status = spi_sync(spi, &message);
	if (status < 0)
		return status;
	
#if 0
	for (i = 1; i < sizeof(buffer); i++) {
		printk("%02x", buffer[i]);
	}
	printk("\n");
#endif
	
	count = sprintf(buf,"0x");
	for (i = 1; i < sizeof(buffer); i++) {
		count += sprintf(buf + count, "%02x ", buffer[i]);
	}
	count += sprintf(buf + count, "\n");
	return count;
}
static DEVICE_ATTR(labels, S_IRUGO, hi358x_label_read, NULL);

static ssize_t hi358x_reset_write(struct device *dev, 
				  struct device_attribute *devattr, 
				  const char *buf, size_t count)
{
	return hi358x_op_write(dev, devattr, buf, count,
			       HI358X_WRITE_RESET, 0);
}
static DEVICE_ATTR(reset_write, S_IWUSR, NULL, hi358x_reset_write);


static ssize_t hi358x_trigger_write(struct device *dev, 
				    struct device_attribute *devattr, 
				    const char *buf, size_t count)
{
	return hi358x_op_write(dev, devattr, buf, count,
			       HI358X_WRITE_TRIGGER , 0);
}
static DEVICE_ATTR(trigger_write, S_IWUSR, NULL, hi358x_trigger_write);

static ssize_t hi358x_model(struct device *dev, 
			    struct device_attribute *devattr, char *buf)
{
	struct hi358x_platform_data *pd = dev->platform_data;
	
	if (!pd)
		return -EINVAL;
	switch (pd->model) {
	case HI3585:
		return sprintf(buf, "HI3585\n"); 
	case HI3587:
		return sprintf(buf, "HI3587\n");
	case HI3588:
		return sprintf(buf, "HI3588\n"); 
	default:
		return sprintf(buf, "Unknown %u\n", pd->model);
	}
	return -1;
}
static DEVICE_ATTR(model, S_IRUGO, hi358x_model, NULL);

/* Attributes common to all models */
static struct attribute *hi358x_attrs[] = {
	&dev_attr_reset.attr,
	&dev_attr_label_none.attr,
	&dev_attr_label_all.attr,
	&dev_attr_label_clear.attr,
	&dev_attr_label_set.attr,
	&dev_attr_aclk_div.attr,
	&dev_attr_status.attr,
	&dev_attr_control.attr,
	&dev_attr_labels.attr,
	&dev_attr_model.attr,
	NULL,
};
static const struct attribute_group hi358x_sysfs_files = {
	.attrs = hi358x_attrs,
};

/* HI3588-only attributes */
static struct attribute *hi3588_attrs[] = {
	&dev_attr_fifo_ro.attr,
	NULL,
};
static const struct attribute_group hi3588_sysfs_files = {
	.attrs = hi3588_attrs,
};

/* HI3587-only attributes */
static struct attribute *hi3587_attrs[] = {
	&dev_attr_fifo_wo.attr,
	NULL,
};

static const struct attribute_group hi3587_sysfs_files = {
	.attrs = hi3587_attrs,
};

/* HI3585-only attributes */
static struct attribute *hi3585_attrs[] = {
	&dev_attr_fifo_rw.attr,
	&dev_attr_reset_write.attr,
	&dev_attr_trigger_write.attr,
	NULL,
};
static const struct attribute_group hi3585_sysfs_files = {
	.attrs = hi3585_attrs,
};

static irqreturn_t hi358x_rx_int(int irq, void *dev_id)
{
	struct device *dev = dev_id;
	dev_dbg(dev, "%s: Interrupt %u RX handler called\n",
		__FUNCTION__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t hi358x_tx_int(int irq, void *dev_id)
{
	struct device *dev = dev_id;
	dev_dbg(dev, "%s: Interrupt %u TX handler called\n", 
		__FUNCTION__, irq);
	return IRQ_HANDLED;
}

static int __devinit hi358x_probe(struct spi_device *spi)
{
	struct hi358x_platform_data *pd = spi->dev.platform_data;
	int err, status;
	u8 buffer[3];
	
	if (!pd) {
		dev_err(&spi->dev, "Missing platform data!\n");
		return -EINVAL;
	}

	/* Set up registers to sane base states */
 	buffer[0] = HI358X_ACLK_SET;
	buffer[1] = 1;
	status = spi_write(spi, buffer, 2);
	if (status < 0) {
		dev_err(&spi->dev, "Cannot set ACLK_SET: %d\n", status);
		return status;
	}
 
	buffer[0] = HI358X_LABEL_SET_ALL;
	status = spi_write(spi, buffer, 1);
	if (status < 0) {
		dev_err(&spi->dev, "Cannot set LABEL_SET_ALL: %d\n", status);
		return status;
	}
	
	buffer[0] = HI358X_WRITE_CONTROL;
	buffer[1] = 0x20;	// Control 0x2020 - instant tx, external bus
	buffer[2] = 0x20;
	status = spi_write(spi, buffer, 3);
	if (status < 0) {
		dev_err(&spi->dev, "Cannot set WRITE_CONTROL: %d\n", status);
		return status;
	}
	
	buffer[0] = HI358X_MASTER_RESET;
	status = spi_write(spi, buffer, 1);
	if (status < 0) {
		dev_err(&spi->dev, "Cannot set MASTER_RESET: %d\n", status);
		return err;
	}
		
	if (pd->tx_irq >= 0) {
		err = request_irq(pd->tx_irq, hi358x_tx_int, 
				  IRQ_TYPE_EDGE_BOTH, 
				  "hi358x_tx_irq", &spi->dev);
		if (err) {
			dev_err(&spi->dev, "Cannot get tx irq %d: %d\n",
				pd->tx_irq, err);
			return err;
		}
	}

	if (pd->rx_irq >= 0) {
		err = request_irq(pd->rx_irq, hi358x_rx_int, 
				  IRQ_TYPE_EDGE_BOTH, 
				  "hi358x_rx_irq", &spi->dev);
		if (err) {
			dev_err(&spi->dev, "Cannot get rx irq %d: %d\n",
				pd->rx_irq, err);
			return err;
		}
	}

	/* Register interrupt callbacks */
	//pd->flag_init(&spi->dev, hi358x_rx_int, hi358x_tx_int, pd);
	
	status = sysfs_create_group(&spi->dev.kobj, &hi358x_sysfs_files);
	if (status) {
		dev_dbg(&spi->dev, "device_create_file failure.\n");
		return status;
	}
	
	switch (pd->model) {
	case HI3585:
		status = sysfs_create_group(&spi->dev.kobj,  
					    &hi3585_sysfs_files);
		if (status) {
			dev_dbg(&spi->dev, "device_create_file failure.\n");
			return status;
		}
		break;

	case HI3587:
		status = sysfs_create_group(&spi->dev.kobj,  
					    &hi3587_sysfs_files);
		if (status) {
			dev_dbg(&spi->dev, "device_create_file failure.\n");
			return status;
		}
		break;

		
	case HI3588:
		status = sysfs_create_group(&spi->dev.kobj, 
					    &hi3588_sysfs_files);
		if (status) {
			dev_dbg(&spi->dev, "device_create_file failure.\n");
			return status;
		}
		break;
		
	default:
		dev_warn(&spi->dev, "unknown model %u", pd->model);
		break;
	}
	
	return 0;
}

static int __devexit hi358x_remove(struct spi_device *spi)
{
	struct hi358x_platform_data *pd = spi->dev.platform_data;
	
	sysfs_remove_group(&spi->dev.kobj, &hi358x_sysfs_files);
	switch (pd->model) {
	case HI3585:
		sysfs_remove_group(&spi->dev.kobj, &hi3585_sysfs_files);
		break;
		
	case HI3587:
		sysfs_remove_group(&spi->dev.kobj, &hi3587_sysfs_files);
		break;
		
	case HI3588:
		sysfs_remove_group(&spi->dev.kobj, &hi3588_sysfs_files);
		break;
	}
	
	//pd->flag_exit(&spi->dev, pd);
	if (pd->tx_irq >= 0)
		free_irq(pd->tx_irq, &spi->dev);
	if (pd->rx_irq >= 0)
		free_irq(pd->rx_irq, &spi->dev);
	return 0;
}

static struct spi_driver hi358x_driver = {
	.driver = {
		.name	= "hi358x",
		.owner	= THIS_MODULE,
	},
	.probe	= hi358x_probe,
	.remove	= __devexit_p(hi358x_remove),
};

static int __init hi358x_init(void)
{
	return spi_register_driver(&hi358x_driver);
}

static void __exit hi358x_cleanup(void)
{
	spi_unregister_driver(&hi358x_driver);
}

module_init(hi358x_init);
module_exit(hi358x_cleanup);

MODULE_AUTHOR("Andre Renaud");
MODULE_DESCRIPTION("HI358x ARINC 429 Receiver Driver");
MODULE_LICENSE("GPL");
