/*
 * drivers/misc/at91_bootcount.c
 *
 * Copyright (C) 2010 DesignA Electronics Ltd
 * Author: Phillip Sanderson <psanderson@designa-electronics.com>
 *
 * U-Boot stores a count of the number of boots since a power cycle
 * in the fourth GPBR (general purpose backup register). This driver
 * simply exposes this bootcount value to sysfs so it can be accessed
 * and reset from user space at
 * 
 * /sys/devices/platform/at91_bootcount.0/bootcount
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <mach/at91sam9g45.h>
#include <mach/gpio.h> 

#define BOOTCOUNT_MAGIC 		0xB001C041
#define AT91_GPBR_BOOTCOUNT_REGISTER 	3
#define AT91_BOOTCOUNT_ADDRESS 		(AT91_GPBR + 4*AT91_GPBR_BOOTCOUNT_REGISTER)


/* placeholder device to register sysfs files under */
static struct platform_device at91_bootcount_device = {
        .name = "at91_bootcount",
};

/* function for reading the 'bootcount' attribute */
static ssize_t bootcount_show(struct kobject *kobj, struct kobj_attribute *attr, 
				char *buf)
{
	unsigned long bootcount = at91_sys_read(AT91_BOOTCOUNT_ADDRESS);
	
	if((bootcount & 0xffff0000) != (BOOTCOUNT_MAGIC & 0xffff0000)) {
		printk("%s: bad magic!\n", __FUNCTION__);
		bootcount = 0;
	} else {
		//printk("%s: magic ok\n", __FUNCTION__);
		bootcount &= 0x0000ffff;
	}	
	return sprintf(buf, "%u\n", bootcount);
}

/* function for writing the 'bootcount' attribute */
static ssize_t bootcount_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned long bootcount;
	sscanf(buf, "%u", &bootcount);
	at91_sys_write(AT91_BOOTCOUNT_ADDRESS, (bootcount & 0x0000ffff) | (BOOTCOUNT_MAGIC & 0xffff0000));
	return count;
}

/* create the sysfs 'bootcount' attribute */
static struct kobj_attribute bootcount_attribute =
	__ATTR(bootcount, 0666, bootcount_show, bootcount_store);

static struct attribute *bootcount_attrs[] = {
	&bootcount_attribute.attr,
	NULL,	
};

static struct attribute_group bootcount_attr_group = {
	.attrs = bootcount_attrs,
};

/* function for module init */
static int __init bootcount_init(void)
{
	int error;
	error = platform_device_register(&at91_bootcount_device);
	if(!error)
		error = sysfs_create_group(&at91_bootcount_device.dev.kobj, 
				&bootcount_attr_group);
	return error;
}

/* function for module exit */
static void __exit bootcount_exit(void)
{
	sysfs_remove_group(&at91_bootcount_device.dev.kobj, 
				&bootcount_attr_group);
	platform_device_unregister(&at91_bootcount_device);

}

module_init(bootcount_init);
module_exit(bootcount_exit);

MODULE_DESCRIPTION("AT91 Bootcount Driver");
MODULE_AUTHOR("Phillip Sanderson <psanderson@designa-electronics.com>");
MODULE_LICENSE("GPL");

