/*
 * drivers/misc/ext_shutdown.c
 *
 *  Copyright (C) 2011 Aiotec Ltd
 *  Author: Andrew Turner <aturner@bluewatersys.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
/*
 * This module listens for a number of GPIO's. If any of them are incorrect
 * it will disable another group of GPIO's, e.g. if any of the over current
 * GPIO's are active it will disable the power supply for an external module.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

#include <asm/io.h>

#include <mach/gpio.h>

struct plat_data {
	unsigned int irq_count;
	unsigned int *irq_gpios;
	unsigned int shdn_count;
	unsigned int *shdn_gpios;
};

struct ext_shutdown {
        struct device *dev;
	struct plat_data *pdata;
};

int shdn_flag = 0;
int shdn_gpio = 0;

static irqreturn_t ext_shutdown_irq(int irq, void *dev_id)
{
	struct ext_shutdown *priv = dev_id;
	int active = 0;
	int i;

	for (i = 0; i < priv->pdata->irq_count; i++) {
		if (gpio_get_value(priv->pdata->irq_gpios[i]) == 0) {
			active = 1;
			break;
		}
	}
	if (active) {
		shdn_gpio = priv->pdata->irq_gpios[i] - PIN_BASE;
		for (i = 0; i < priv->pdata->shdn_count; i++) {
			gpio_set_value(priv->pdata->shdn_gpios[i], 0);
		}
		shdn_flag = 1;

		printk("Over current\n");
	}

	return IRQ_HANDLED;
}

static ssize_t ext_shutdown_read_flag(struct device *dev,
                                      struct device_attribute *attr, char *buf)
{
	ssize_t ret;

	ret = sprintf(buf, "%d", shdn_flag);
	shdn_flag = 0;

	return ret;
}
static DEVICE_ATTR(flag, S_IRUSR | S_IRUGO, ext_shutdown_read_flag, NULL);

static ssize_t ext_shutdown_read_gpio(struct device *dev,
                                      struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d", shdn_gpio);
}
static DEVICE_ATTR(gpio, S_IRUSR | S_IRUGO, ext_shutdown_read_gpio, NULL);

static struct attribute *ext_shutdown_attrs[] = {
	&dev_attr_flag.attr,
	&dev_attr_gpio.attr,
	NULL,
};

static const struct attribute_group ext_shutdown_sysfs_files = {
        .attrs = ext_shutdown_attrs,
};

static int __init ext_shutdown_probe(struct platform_device *pdev)
{
	struct ext_shutdown *priv;
	int i, rc;

	priv = kmalloc(sizeof(struct ext_shutdown), GFP_KERNEL);
	if (!priv) {
		return -ENOMEM;
	}

	priv->dev = &pdev->dev;
        platform_set_drvdata(pdev, priv);

	priv->pdata = pdev->dev.platform_data;

        rc = sysfs_create_group(&pdev->dev.kobj, &ext_shutdown_sysfs_files);
	if (rc) {
		dev_err(priv->dev, "unable to create sysfs group");
		return rc;
	}

	for (i = 0; i < priv->pdata->irq_count; i++) {
	        rc = request_irq(priv->pdata->irq_gpios[i], ext_shutdown_irq,
		                 IRQF_DISABLED, pdev->name, priv);
	        if (rc) {
	                dev_err(priv->dev, "cannot claim IRQ");
	                return rc;
	        }

		rc = gpio_request(priv->pdata->irq_gpios[i], "Shutdown GPIO");
		if (rc) {
			dev_err(priv->dev, "gpio request failed");
			return rc;
		}
	}

	for (i = 0; i < priv->pdata->shdn_count; i++) {
		rc = gpio_request(priv->pdata->shdn_gpios[i], "Shutdown GPIO");
		if (rc) {
			dev_err(priv->dev, "gpio request failed");
			return rc;
		}
	}

	printk("Loaded External Shutdown\n");

	return 0;
}

static int ext_shutdown_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver ext_shutdown_driver = {
        .probe          = ext_shutdown_probe,
        .remove         = __exit_p(ext_shutdown_remove),
        .driver         = {
                .name   = "ext_shutdown",
                .owner  = THIS_MODULE,
        },
};

static int __init ext_shutdown_init(void)
{
        return platform_driver_register(&ext_shutdown_driver);
}

static void __exit ext_shutdown_exit(void)
{
        platform_driver_unregister(&ext_shutdown_driver);
}

module_init(ext_shutdown_init);
module_exit(ext_shutdown_exit);

MODULE_DESCRIPTION("External Shutdown");
MODULE_AUTHOR("Andrew Turner <aturner@bluewatersys.com>");
MODULE_LICENSE("GPL");

