/*
 * Simple SPI SSD2119 display driver
 *
 * Author: Carl Smith <carl@bluewatersys.com>
 *
 * Copyright (c) 2009, Bluewater Systems Ltd
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

#include <linux/spi/spi.h>

#include <video/ssd2119fb.h>

/* Amount to DMA to frame buffer per SPI transfer */
#define SSD2119FB_DMA_BLOCK_SIZE	1024

#define REG_OSC_START		0x00
#define REG_OUTPUT_CTRL		0x01
#define REG_LCD_DRIVE_AC_CTRL	0x02
#define REG_DISPLAY_CTRL	0x07
#define REG_PWR_CTRL_2		0x0c
#define REG_PWR_CTRL_3		0x0d
#define REG_PWR_CTRL_4		0x0e
#define REG_SLEEP_MODE		0x10
#define REG_ENTRY_MODE		0x11
#define REG_RAM_DATA		0x22
#define REG_V_RAM_POS		0x44
#define REG_H_RAM_START		0x45
#define REG_H_RAM_END		0x46
#define REG_X_RAM_ADDR		0x4e
#define REG_Y_RAM_ADDR		0x4f

/* FIXME - Remove these */
#define LCD_WIDTH	320
#define LCD_HEIGHT	240
#define BPP		24

/* Frame buffer info */
struct ssd2119_fb_info {
	struct fb_info			*info;
	struct spi_device		*spi;
	struct timer_list		auto_update_timer; 
        struct work_struct		task;	
	struct ssd2119_platform_data	*pd;
};

static struct fb_fix_screeninfo ssd2119fb_fix = {
	.id =		"ssd2119fb",
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	FB_VISUAL_TRUECOLOR,
	.xpanstep =	0,
	.ypanstep =	0,
	.ywrapstep =	0,
	.accel =	FB_ACCEL_NONE,
};

static struct fb_var_screeninfo ssd2119fb_var = {
	.activate	= FB_ACTIVATE_NOW,
	.xres		= 320,
	.yres		= 240,
	.xres_virtual	= 320,
	.yres_virtual	= 240,
	.bits_per_pixel	= 24,
	.nonstd		= 1,
};

static int ssd2119fb_write(struct ssd2119_fb_info *sinfo, u8 *buf, int len)
{
        struct spi_transfer t = {
                .tx_buf = buf,
                .len = len,
        };
        struct spi_message m;

        spi_message_init(&m);
        spi_message_add_tail(&t, &m);
        return spi_sync(sinfo->spi, &m);
}

static int ssd2119fb_dma_write(struct ssd2119_fb_info *sinfo,
			       u8 *buf, dma_addr_t dma_addr, int len,
			       int *actual_len)
{
	int ret;
        struct spi_transfer t = {
		.tx_buf = buf,
                .tx_dma = dma_addr,
                .len 	= len,
        };
        struct spi_message m;

        spi_message_init(&m);
	m.is_dma_mapped = 1;
        spi_message_add_tail(&t, &m);
        ret = spi_sync(sinfo->spi, &m);
	*actual_len = m.actual_length;
	return ret;
}

static int ssd2119_command(struct ssd2119_fb_info *sinfo, uint8_t reg)
{
        u8 tx[2];
        int status;

        tx[0] = 0;
        tx[1] = reg;

	gpio_set_value(sinfo->pd->gpio_dc, 0);
	status = ssd2119fb_write(sinfo, tx, 2);
        if (status < 0) {
                dev_warn(&sinfo->spi->dev, "spi xfer failed with status %d\n",
                                status);
                return status;
        }
        return 0;
}

static int ssd2119_data(struct ssd2119_fb_info *sinfo, uint16_t data)
{
        u8 tx[2];
        int status;

        tx[0] = data >> 8;
        tx[1] = data;

	gpio_set_value(sinfo->pd->gpio_dc, 1);
        status = ssd2119fb_write(sinfo, tx, 2);
        if (status < 0) {
                dev_warn(&sinfo->spi->dev, "spi xfer failed with status %d\n",
                                status);
                return status;
        }
        return 0;
}

static int ssd2119_full_command(struct ssd2119_fb_info *sinfo, uint8_t reg,
                uint16_t data)
{
        int s;
        s = ssd2119_command(sinfo, reg);
        if (s < 0)
                return s;
        return ssd2119_data(sinfo, data);
}

static int ssd2119fb_check_var(struct fb_var_screeninfo *var,
			       struct fb_info *info)
{
	/* Force same alignment for each line */
	var->xres = (var->xres + 3) & ~3UL;
	var->xres_virtual = (var->xres_virtual + 3) & ~3UL;

	var->red.msb_right = var->green.msb_right = var->blue.msb_right = 0;
	var->transp.offset = var->transp.length = 0;

	switch (var->bits_per_pixel) {
	case 24:
		var->red.offset 	= 16;
		var->red.length 	= 8;
		var->green.offset 	= 8;
		var->green.length 	= 8;
		var->blue.offset 	= 0;
		var->blue.length 	= 8;
		var->transp.offset 	= 0;
		var->transp.length 	= 0;
		break;
		
	default:
		return -EINVAL;
	}

	var->xoffset = var->yoffset = 0;
	var->red.msb_right = var->green.msb_right = var->blue.msb_right =
		var->transp.msb_right = 0;

	return 0;
}

static int ssd2119_fb_blank(int blank, struct fb_info *info)
{
        switch (blank) {
        case FB_BLANK_POWERDOWN:
        case FB_BLANK_VSYNC_SUSPEND:
        case FB_BLANK_HSYNC_SUSPEND:
        case FB_BLANK_NORMAL:
                /* FIXME: Should be going into sleep mode here */
                break;
        case FB_BLANK_UNBLANK:
                /* FIXME: Should be exiting sleep mode here */
                break;
        }
        return 0;

}

static int ssd2119fb_update_region(struct ssd2119_fb_info *sinfo,
				   int x, int y, int w, int h)
{
        struct fb_info *info = sinfo->info;
	int count, block, written, total, offset, bpp;
	
	dev_dbg(&sinfo->spi->dev, "Updating region (%d, %d)-(%d, %d)\n",
		x, y, x + w, x + h);

	/*
	 * Set the rectangle window
	 *
	 * TODO: Because SPI bandwidth is limited, it would be useful to have
	 * an ioctl which can update a sub-rectangle of the screen. This
	 * update function supports this, but no userspace interface exists
	 * yet.
	 *
	 */
	ssd2119_full_command(sinfo, REG_H_RAM_START, x);
	ssd2119_full_command(sinfo, REG_H_RAM_END, x + w - 1);
	ssd2119_full_command(sinfo, REG_V_RAM_POS, ((y + h - 1) << 8) | y);

	/* Write to GRAM */
	ssd2119_full_command(sinfo, REG_X_RAM_ADDR, 0);
        ssd2119_full_command(sinfo, REG_Y_RAM_ADDR, 0);
        ssd2119_command(sinfo, REG_RAM_DATA);
	gpio_set_value(sinfo->pd->gpio_dc, 1);

	bpp = info->var.bits_per_pixel / 8;
	offset = (y * (info->var.xres * bpp)) + (x * bpp);
	total = w * h * bpp;
	count = 0;
	while (count < w * h * bpp) {
		if (w == info->var.xres)
			block = min(SSD2119FB_DMA_BLOCK_SIZE, total - count);
		else
			block = w;

		ssd2119fb_dma_write(sinfo, ((u8 *)info->screen_base) + offset,
				    info->fix.smem_start + offset,
				    block, &written);

		count  += written;
		offset += written + ((info->var.xres - w) * bpp);
	}

	return 0;
}

static void ssd2119_fb_update_work(struct work_struct *work)
{
        struct ssd2119_fb_info *sinfo = container_of(work,
					struct ssd2119_fb_info, task);
        struct fb_info *info = sinfo->info;

        ssd2119fb_update_region(sinfo, 0, 0, info->var.xres, info->var.yres);
}

static void ssd2119fb_mod_timer(struct ssd2119_fb_info *sinfo)
{
	unsigned timeout;

	if (!sinfo->pd->refresh_speed) { 
		if (timer_pending(&sinfo->auto_update_timer))
			del_timer_sync(&sinfo->auto_update_timer);
		return;
	}

	if (timer_pending(&sinfo->auto_update_timer))
		return;

	timeout = msecs_to_jiffies(1000 / sinfo->pd->refresh_speed);
	mod_timer(&sinfo->auto_update_timer, jiffies + timeout);
}

static void ssd2119fb_timer_func(unsigned long arg)
{
        struct ssd2119_fb_info *sinfo = (struct ssd2119_fb_info *)arg;

        schedule_work(&sinfo->task);
	ssd2119fb_mod_timer(sinfo);
}

static void ssd2119fb_reset(struct ssd2119_fb_info *sinfo)
{
	gpio_set_value(sinfo->pd->gpio_dc, 0);

	/* Reset */
	gpio_set_value(sinfo->pd->gpio_reset, 0);
	udelay(10);
	gpio_set_value(sinfo->pd->gpio_reset, 1);

	/* Enable sleep mode */
        ssd2119_full_command(sinfo, REG_SLEEP_MODE, 0x0001);

	/* Enable oscillator */
        ssd2119_full_command(sinfo, REG_OSC_START, 0x0001);

        ssd2119_full_command(sinfo, REG_OUTPUT_CTRL, 0x38ef);
        //ssd2119_full_command(sinfo, REG_OUTPUT_CTRL, 0x78ef);
        ssd2119_full_command(sinfo, REG_LCD_DRIVE_AC_CTRL, 0x0600);

	/* Disable sleep mode */
        ssd2119_full_command(sinfo, REG_SLEEP_MODE, 0x0000);
        mdelay(30);
        ssd2119_full_command(sinfo, REG_DISPLAY_CTRL, 0x0033);
        // ssd2119_full_command(sinfo, REG_PWR_CTRL_2, 0x0005);
        // ssd2119_full_command(sinfo, REG_PWR_CTRL_3, 0x0007);
        // ssd2119_full_command(sinfo, REG_PWR_CTRL_4, 0x3100);
        ssd2119_full_command(sinfo, REG_V_RAM_POS, (LCD_HEIGHT - 1) << 8);
        ssd2119_full_command(sinfo, REG_H_RAM_START, 0);
        ssd2119_full_command(sinfo, REG_H_RAM_END, LCD_WIDTH - 1);
#if BPP == 24
	ssd2119_full_command(sinfo, REG_ENTRY_MODE, 0x4230);
#else
	ssd2119_full_command(sinfo, REG_ENTRY_MODE, 0x6230);
#endif
        // ssd2119_full_command(sinfo, REG_ENTRY_MODE, 0x6830);
        // ssd2119_full_command(sinfo, REG_ENTRY_MODE, 0x6230);
}

static ssize_t ssd2119_fb_refresh_hz_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
        struct fb_info *info = dev_get_drvdata(dev);
        struct ssd2119_fb_info *sinfo = info->par;

	sinfo->pd->refresh_speed = simple_strtoul(buf, NULL, 0);
	ssd2119fb_mod_timer(sinfo);
	
	return count;
}

static ssize_t ssd2119_fb_refresh_hz_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{	
        struct fb_info *info = dev_get_drvdata(dev);
        struct ssd2119_fb_info *sinfo = info->par;

	return sprintf(buf, "%d\n", sinfo->pd->refresh_speed);
}

static ssize_t ssd2119_fb_reset_sysfs(struct device *dev, 
                                       struct device_attribute *attr,
                                       const char *buf, size_t count)
{
        struct fb_info *info = dev_get_drvdata(dev);
        struct ssd2119_fb_info *sinfo = info->par;

	ssd2119fb_reset(sinfo);
	return count;
}

static ssize_t ssd2119_fb_update_sysfs(struct device *dev, 
                                       struct device_attribute *attr,
                                       const char *buf, size_t count)
{
        struct fb_info *info = dev_get_drvdata(dev);
        struct ssd2119_fb_info *sinfo = info->par;

        ssd2119fb_update_region(sinfo, 0, 0, info->var.xres, info->var.yres);
        return count;
}

static DEVICE_ATTR(reset, S_IWUSR, NULL, ssd2119_fb_reset_sysfs);
static DEVICE_ATTR(update, S_IWUSR, NULL, ssd2119_fb_update_sysfs);
static DEVICE_ATTR(refresh_hz, S_IWUSR | S_IRUGO, ssd2119_fb_refresh_hz_show,
		   ssd2119_fb_refresh_hz_store);

static struct attribute *ssd2119_fb_attrs[] = {
	&dev_attr_refresh_hz.attr,
	&dev_attr_reset.attr,
        &dev_attr_update.attr,	
        NULL,
};

static const struct attribute_group sysfs_files = {
        .attrs = ssd2119_fb_attrs,
};

static struct fb_ops ssd2119_fb_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= ssd2119fb_check_var,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
        .fb_blank	= ssd2119_fb_blank,
};

static int ssd2119_fb_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct ssd2119_fb_info *sinfo;
	struct fb_info *info;
	dma_addr_t dma_addr;
	int ret = -ENOMEM;

	info = framebuffer_alloc(sizeof(struct ssd2119_fb_info), dev);
	if (!info)
		goto out;

	ret = sysfs_create_group(&dev->kobj, &sysfs_files);
        if (ret)
                goto free_info;

	info->var = ssd2119fb_var;
	info->fix = ssd2119fb_fix;
	sinfo = info->par;
	sinfo->info = info;
	sinfo->spi = spi;
        sinfo->pd = dev->platform_data;
	
	strcpy(info->fix.id, spi->modalias);
	info->fbops = &ssd2119_fb_ops;

	/* 
	 * FIXME - Should probably pass dev to dma_alloc_coherent, but don't
	 * know how to set dma_coherent_mask for spi devices using 
	 * spi_board_info struct
	 */
	info->fix.line_length = LCD_WIDTH * BPP / 8;
	info->fix.smem_len = LCD_HEIGHT * info->fix.line_length;
	info->screen_base = dma_alloc_coherent(NULL, 
					       PAGE_ALIGN(info->fix.smem_len),
					       &dma_addr, GFP_KERNEL);
	if (!info->screen_base) {
		ret = -ENOMEM;
		goto remove_sysfs;
	}
	info->fix.smem_start = (unsigned long)dma_addr;

	ret = ssd2119fb_check_var(&info->var, info);

	dev_set_drvdata(dev, info);

	/*
	 * Request GPIOs
	 * FIXME - error handling
	 */
	gpio_request(sinfo->pd->gpio_dc, "ssd2119_dc");
	gpio_direction_output(sinfo->pd->gpio_dc, 0);
	gpio_request(sinfo->pd->gpio_reset, "ssd2119_reset");
	gpio_direction_output(sinfo->pd->gpio_reset, 1);

	ret = register_framebuffer(info);
	if (ret < 0)
		goto release_fb;

	INIT_WORK(&sinfo->task, ssd2119_fb_update_work);

	ssd2119fb_reset(sinfo);

        /* Initialise the auto-refresh timer and kick it off if necessary */
	setup_timer(&sinfo->auto_update_timer, ssd2119fb_timer_func,
		    (unsigned long)sinfo);
	ssd2119fb_mod_timer(sinfo);

	if (sinfo->pd->refresh_speed)
		dev_info(dev, "Initialised - Auto refreshing at %d hz\n",
			 sinfo->pd->refresh_speed);
	else
		dev_info(dev, "Initialised - Auto refresh disabled\n"); 
	return 0;

release_fb:
	dma_free_coherent(NULL, PAGE_ALIGN(info->fix.smem_len),
			  info->screen_base, dma_addr);
remove_sysfs:
	sysfs_remove_group(&dev->kobj, &sysfs_files);
free_info:	
	framebuffer_release(info);
out:
	return ret;
}

static int ssd2119_fb_remove(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct fb_info *info = dev_get_drvdata(dev);
	struct ssd2119_fb_info *sinfo = info->par;

	/* FIMXE - Is the timer running? */
	del_timer_sync(&sinfo->auto_update_timer);
	sysfs_remove_group(&dev->kobj, &sysfs_files);
	dma_free_coherent(NULL, PAGE_ALIGN(info->fix.smem_len), 
			  info->screen_base, (dma_addr_t)info->fix.smem_start);
	gpio_free(sinfo->pd->gpio_dc);
	gpio_free(sinfo->pd->gpio_reset);
	framebuffer_release(info);
	return 0;
}

static struct spi_driver ssd2119fb_driver = {
    	.driver = {
        	.name	= "ssd2119fb",
                .bus	= &spi_bus_type,
                .owner	= THIS_MODULE,
        },
        .probe	= ssd2119_fb_probe,
        .remove	= ssd2119_fb_remove,
};

static int __init ssd2119fb_init(void)
{
	return spi_register_driver(&ssd2119fb_driver);
}

static void __exit ssd2119fb_exit(void)
{
	spi_unregister_driver(&ssd2119fb_driver);
}

module_init(ssd2119fb_init);
module_exit(ssd2119fb_exit);

MODULE_DESCRIPTION("SSD2119 LCD driver");
MODULE_AUTHOR("Carl Smith <carl@bluewatersys.com>");
MODULE_LICENSE("GPL");

