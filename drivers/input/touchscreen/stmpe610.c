/*
 * STMPE610 based touchscreen driver
 *
 * Copyright (c) 2010 Bluewater Systems Ltd.
 *  Carl Smith <carl@bluewatersys.com>
 *
 * Using code from:
 *  - ads7846.c
 *  Copyright (c) 2005 David Brownell
 *  Copyright (c) 2006 Nokia Corporation
 *  - corgi_ts.c
 *	Copyright (C) 2004-2005 Richard Purdie
 *  - omap_ts.[hc], ads7846.h, ts_osk.c
 *	Copyright (C) 2002 MontaVista Software
 *	Copyright (C) 2004 Texas Instruments
 *	Copyright (C) 2005 Dirk Behme
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>

#define TS_POLL_DELAY	(1 * 1000000)	/* ns delay before the first sample */
#define TS_POLL_PERIOD	(5 * 1000000)	/* ns delay between samples */

/* this driver doesn't aim at the peak continuous sample rate */
#define	SAMPLE_BITS	(8 /*cmd*/ + 16 /*sample*/ + 2 /* before, after */)

#define MAX_8BIT	((1 <<  8) - 1)
#define	MAX_12BIT	((1 << 12) - 1)

/* Registers */
#define CHIP_ID            0x00
#define ID_VER		        0x02
#define SYS_CTRL1          0x03
#define SYS_CTRL2          0x04
#define SPI_CFG            0x08
#define INT_CTRL           0x09
#define INT_EN             0x0A
#define INT_STA      		0x0B
#define GPIO_EN      		0x0C
#define GPIO_INT_STA 		0x0D
#define ADC_INT_EN   		0x0E
#define ADC_INT_STA        0x0F
#define GPIO_SET_PIN       0x10
#define GPIO_CLR_PIN 		0x11
#define GPIO_MP_STA  		0x12
#define GPIO_DIR           0x13
#define GPIO_ED            0x14
#define GPIO_RE            0x15
#define GPIO_FE            0x16
#define GPIO_AF            0x17
#define ADC_CTRL1          0x20
#define ADC_CTRL2    		0x21
#define ADC_CAPT     		0x22
#define ADC_DATA_CH0       0x30
#define ADC_DATA_CH1       0x32
#define ADC_DATA_CH4       0x38
#define ADC_DATA_CH5       0x3A
#define ADC_DATA_CH6       0x3C
#define ADC_DATA_CH7   	0x3E
#define TSC_CTRL       	0x40
#define TSC_CFG        	0x41
#define WDW_TR_X           0x42
#define WDW_TR_Y       	0x44
#define WDW_BL_X       	0x46
#define WDW_BL_Y       	0x48
#define FIFO_TH        	0x4A
#define FIFO_STA           0x4B
#define FIFO_SIZE      	0x4C
#define TSC_DATA_X         0x4D
#define TSC_DATA_Y         0x4F
#define TSC_DATA_Z         0x51
#define TSC_DATA_XYZ       0x52
#define TSC_FRACT_XYZ      0x56
#define TSC_DATA		    0x57
#define TSC_I_DRIVE        0x58
#define TSC_SHIELD         0x59
#define TSC_DATA4	0xd7

#define PENDOWN_TIMEOUT		msecs_to_jiffies(100)

struct stmpe610 {
	struct input_dev	*input;
	char			phys[32];
	struct spi_device	*spi;
	struct work_struct	work;
	struct timer_list	timer;

	unsigned 		last_irq_time;
	int			pendown;
};

static int stmpe610_write_register(struct stmpe610 *ts, u8 reg, u8 val)
{
	struct spi_device *spi = ts->spi;
	struct spi_message message;
	struct spi_transfer transfer[2];

	/* Init message */
	spi_message_init(&message);

	/* Address */
	memset(&transfer[0], 0, sizeof(struct spi_transfer));
	transfer[0].tx_buf = &reg;
	transfer[0].rx_buf = 0;
	transfer[0].len = 1;
	transfer[0].cs_change = 0;
	spi_message_add_tail(&transfer[0], &message);

	/* Data */
	memset(&transfer[1], 0, sizeof(struct spi_transfer));
	transfer[1].tx_buf = &val;
	transfer[1].rx_buf = 0;
	transfer[1].len = 1;
	transfer[1].cs_change = 0;
	spi_message_add_tail(&transfer[1], &message);

	return spi_sync(spi, &message);
}

static u8 stmpe610_read_register(struct stmpe610 *ts, u8 reg)
{
	struct spi_device *spi = ts->spi;
	struct spi_message message;
	struct spi_transfer transfer[2];
	u8 command;
	u8 data;
	u8 ret;
	/* Init message */
	spi_message_init(&message);

	/* Address */
	memset(&transfer[0], 0, sizeof(struct spi_transfer));
	command = 0x80 | reg;
	transfer[0].tx_buf = &command;
	transfer[0].rx_buf = 0;
	transfer[0].len = 1;
	transfer[0].cs_change = 0;
	spi_message_add_tail(&transfer[0], &message);

	/* Data */
	memset(&transfer[1], 0, sizeof(struct spi_transfer));
	transfer[1].tx_buf = 0;
	transfer[1].rx_buf = &data;
	transfer[1].len = 1;
	transfer[1].cs_change = 0;
	spi_message_add_tail(&transfer[1], &message);

	ret = spi_sync(spi, &message);
	if (!ret && message.status) {
		printk(KERN_INFO "%s: Error %u\n",
		       __FUNCTION__, message.status);
	} else {
		ret = data;
	}

	return ret;
}

static int stmpe610_read_xyz_data(struct stmpe610 *ts, int *x, int *y, int *z)
{
	u8 buffer[4];
	u32 word;
	int i;
	
	for (i = 0; i < 4; i++)
		buffer[3 - i] = stmpe610_read_register(ts, TSC_DATA_XYZ + i);

	word = *(u32 *)buffer;
	*x = (word >> 20) & 0xfff;
	*y = (word >>  8) & 0xfff;
	*z = (word & 0xff);

	pr_debug("stimpy: raw value: %.2x %.2x %.2x %.2x (%.8x): "
		 "x=%.4d, y=%.4d, z=%.2d, fifo = %d\n", 
		 buffer[0], buffer[1], buffer[2], buffer[3], word, *x, *y, *z,
		 stmpe610_read_register(ts, FIFO_SIZE));
	return 0;
}

static inline u16 stmpe610_read_register16(struct stmpe610 *ts, u8 reg)
{
	u8 msb, lsb;

	msb = stmpe610_read_register(ts, reg);
	lsb = stmpe610_read_register(ts, reg + 1);

	return (msb << 8) | lsb;
}

static int configure_tsc(struct stmpe610 *ts)
{
	u8 value;
	u16 chip_id;

	/* Reset the touchscreen */
	stmpe610_write_register(ts, SYS_CTRL1, 0x02);

	/* Confirm we have the correct firmware revision */
	chip_id = stmpe610_read_register16(ts, CHIP_ID);

	dev_info(&ts->spi->dev, "STMPE610: chip: %xv%x\n",
		chip_id, stmpe610_read_register(ts, ID_VER));
	if (chip_id != 0x811) {
		dev_err(&ts->spi->dev, "incorrect stmpe610 chip id: 0x%x\n", 
			chip_id);
		return -EINVAL;
	}
	/* Enable the touchscreen and ADC */
	value = stmpe610_read_register(ts, SYS_CTRL2);
	value &= ~(1<<0|1<<1|1<<2);
	stmpe610_write_register(ts, SYS_CTRL2, value);

	/* Enable TSC global interrupts */
	value = stmpe610_read_register(ts, INT_EN);
	value |= ((1 << 0) | (1 << 1) | (1 << 4));
	stmpe610_write_register(ts, INT_EN, value);

	/* Select Sample Time, bit number and ADC Reference */
	stmpe610_write_register(ts, ADC_CTRL1, 0x69);

	/* Wait for ~20 ms */
	mdelay(20);

	/* Select the ADC clock speed: 3.25 MHz */
	stmpe610_write_register(ts, ADC_CTRL2, 0x00);

	/* Select TSC pins in non default mode */
	value = stmpe610_read_register(ts, GPIO_AF);
	value &= ~(1<<1|1<<2|1<<3|1<<4);
	stmpe610_write_register(ts, GPIO_AF, value);

	/* Select 2 nF filter capacitor */
	stmpe610_write_register(ts, TSC_CFG, 0x9A);

	/* Select single point reading  */
	stmpe610_write_register(ts, FIFO_TH, 0x01);

	/* Write 0x01 to clear the FIFO memory content. */
	stmpe610_write_register(ts, FIFO_STA, 0x01);

	/* Write 0x00 to put the FIFO back into operation mode  */
	stmpe610_write_register(ts, FIFO_STA, 0x00);

	/*
	 * set the data format for Z value: 7 fractional part 
	 * and 1 whole part
	 */
	stmpe610_write_register(ts, TSC_FRACT_XYZ, 0x01);

	/* set the driving capability of the device for TSC pins: 50mA */
	stmpe610_write_register(ts, TSC_I_DRIVE, 0x01);

	/* 
	 * Use no tracking index, touchscreen controller operation mode 
	 * (XYZ) and enable the TSC
	 */
	stmpe610_write_register(ts, TSC_CTRL, 0x01);

	/* Clear all the status pending bits */
	stmpe610_write_register(ts, INT_STA, 0xFF);

	/* Enable edge based interrupts and enable all interrupts */
	stmpe610_write_register(ts, INT_CTRL, 0x01);

	return 0;
}

static ssize_t stmpe610_registers_show(struct device *dev, 
				       struct device_attribute *attr, 
				       char *buf)
{
	struct stmpe610	*ts = dev_get_drvdata(dev);
	u8 value;
	int i;

	for(i=0;i<0x60;i++) {
		value = stmpe610_read_register(ts, i);
		sprintf(buf+strlen(buf), "0x%02x: 0x%02x\n", i, value);
	}

	return strlen(buf);
}

static ssize_t stmpe610_registers_set(struct device *dev, 
				      struct device_attribute *attr, 
				      const char *buf, size_t count)
{
	struct stmpe610	*ts = dev_get_drvdata(dev);
	unsigned int value, ret;
	unsigned int reg;

	if (sscanf(buf, "0x%x 0x%x", &reg, &value) == 2) {
		stmpe610_write_register(ts, (u8)reg, (u8)value);	
		ret = stmpe610_read_register(ts, reg);
		printk(KERN_ERR "Wrote 0x%.2x to register 0x%.2x, "
		       "read back 0x%.2x\n", value, reg, ret);
	
	} else {
		printk(KERN_ERR "Invalid format\n");
	}

	return count;
}

static DEVICE_ATTR(registers, 0664, stmpe610_registers_show, 
		   stmpe610_registers_set);

static struct attribute *stmpe610_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static struct attribute_group stmpe610_attr_group = {
	.attrs = stmpe610_attributes,
};

static void 
stmpe610_work(struct work_struct *work)
{
	struct stmpe610 *ts = container_of(work, struct stmpe610, work);
	int x, y, z, pendown;
	
	pendown = !!(stmpe610_read_register(ts, TSC_CTRL) & (1 << 7));
	if (pendown) {
		ts->last_irq_time = jiffies;
	} else {
		if (ts->pendown && 
		    time_after(jiffies, ts->last_irq_time + PENDOWN_TIMEOUT)) {
			/* Generate a touch release event */
			pr_debug("stimpy: Generating touch release event\n");
			input_report_key(ts->input, BTN_TOUCH, 0);
			input_sync(ts->input);
			ts->pendown = 0;
		}
		return;
	}

	if (stmpe610_read_register(ts, FIFO_SIZE) == 0)
		return;

	/* Read the touch screen position */
	stmpe610_read_xyz_data(ts, &x, &y, &z);

	/* Report the input event */
	if (!ts->pendown) {
		pr_debug("stimpy: Generating touch event\n");
		input_report_key(ts->input, BTN_TOUCH, 1);
		ts->pendown = 1;
	}
	
	input_report_abs(ts->input, ABS_X, x);
	input_report_abs(ts->input, ABS_Y, y);
	input_report_abs(ts->input, ABS_PRESSURE, z);
	input_sync(ts->input);
	
	/* Clear the FIFO and interrupts */
	stmpe610_write_register(ts, FIFO_STA, 0x01);
	stmpe610_write_register(ts, FIFO_STA, 0x00);
	stmpe610_write_register(ts, INT_STA, 0xff);
}

static void stmpe610_timer(unsigned long data)
{
	struct stmpe610 *ts = (struct stmpe610 *)data;
	
	schedule_work(&ts->work);
	mod_timer(&ts->timer, jiffies + PENDOWN_TIMEOUT);
}

static irqreturn_t stmpe610_irq(int irq, void *handle)
{
	struct stmpe610 *ts = handle;       

	schedule_work(&ts->work);
	return IRQ_HANDLED;
}

static int __devinit stmpe610_probe(struct spi_device *spi)
{
	struct input_dev *input_dev;
	struct stmpe610 *ts;
	int err;

	/* Check we have a specified IRQ */
	if (!spi->irq) {
		dev_err(&spi->dev, "no IRQ?\n");
		return -ENODEV;
	}

	/* Don't exceed max specified sample rate */
	if (spi->max_speed_hz > 1000000) {
		dev_err(&spi->dev, "f(sample) %d KHz?\n",
				(spi->max_speed_hz/SAMPLE_BITS)/1000);
		return -EINVAL;
	}

	/* We'd set TX wordsize 8 bits and RX wordsize to 13 bits ... except
	 * that even if the hardware can do that, the SPI controller driver
	 * may not.  So we stick to very-portable 8 bit words, both RX and TX.
	 */
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_2;
	err = spi_setup(spi);
	if (err < 0) {
		dev_err(&spi->dev, "setup SPI failed\n");
		return err;
	}

	/* Allocate our local context and input device */
	ts = kzalloc(sizeof(struct stmpe610), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		dev_err(&spi->dev, "allocate input device failed\n");
		err = -ENOMEM;
		goto err_free_mem;
	}

	dev_set_drvdata(&spi->dev, ts);
	spi->dev.power.power_state = PMSG_ON;

	ts->spi = spi;
	ts->input = input_dev;

	snprintf(ts->phys, sizeof(ts->phys), "%s/input0", dev_name(&spi->dev));

	input_dev->name = "STMPE610 Touchscreen";
	input_dev->phys = ts->phys;
	input_dev->dev.parent = &spi->dev;
	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);
	input_set_abs_params(input_dev, ABS_X, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, MAX_8BIT, 0, 0);

	err = request_irq(spi->irq, stmpe610_irq, IRQF_TRIGGER_FALLING,
			  spi->dev.driver->name, ts);
	if (err) {
		dev_info(&spi->dev, "trying pin change workaround on irq %d\n",
			 spi->irq);
		err = request_irq(spi->irq, stmpe610_irq, 
				  IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				  spi->dev.driver->name, ts);
		if (err) {
			dev_err(&spi->dev, "irq %d busy?\n", spi->irq);
			goto err_free_mem;
		}
	}

	dev_info(&spi->dev, "touchscreen, irq %d\n", spi->irq);

	/* Create sysfs files */
	err = sysfs_create_group(&spi->dev.kobj, &stmpe610_attr_group);
	if (err) {
		dev_err(&spi->dev, "create sysfs files failed\n");
		goto err_free_irq;
	}

	/* Register input device */
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&spi->dev, "register inpt device failed\n");
		goto err_remove_attr_group;
	}

	INIT_WORK(&ts->work, stmpe610_work);
	init_timer(&ts->timer);
	ts->timer.function = stmpe610_timer;
	ts->timer.data = (unsigned long)ts;
	mod_timer(&ts->timer, jiffies + PENDOWN_TIMEOUT);

	ts->pendown = 0;
	ts->last_irq_time = 0;

	err = configure_tsc(ts);
	if (err) {
		dev_err(&spi->dev, "failed to configure device\n");
		goto err_configure;
	}

	return 0;

 err_configure:
	del_timer_sync(&ts->timer);
 err_remove_attr_group:
	sysfs_remove_group(&spi->dev.kobj, &stmpe610_attr_group);
 err_free_irq:
	free_irq(spi->irq, ts);
 err_free_mem:
	input_free_device(input_dev);
	kfree(ts);
	return err;
}

static int __devexit stmpe610_remove(struct spi_device *spi)
{
	struct stmpe610 *ts = dev_get_drvdata(&spi->dev);

	input_unregister_device(ts->input);
	sysfs_remove_group(&spi->dev.kobj, &stmpe610_attr_group);
	free_irq(ts->spi->irq, ts);	
	del_timer_sync(&ts->timer);
	kfree(ts);

	dev_dbg(&spi->dev, "Removed stmpe610 touchscreen controller\n");
	return 0;
}

static struct spi_driver stmpe610_driver = {
	.driver = {
		.name	= "stmpe610",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= stmpe610_probe,
	.remove		= __devexit_p(stmpe610_remove),
};

static int __init stmpe610_init(void)
{
	return spi_register_driver(&stmpe610_driver);
}

static void __exit stmpe610_exit(void)
{
	spi_unregister_driver(&stmpe610_driver);
}

module_init(stmpe610_init);
module_exit(stmpe610_exit);

MODULE_DESCRIPTION("STMPE610 TouchScreen Driver");
MODULE_LICENSE("GPL");
