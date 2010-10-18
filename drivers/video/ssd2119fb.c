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
//#undef DEBUG
//#define DEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

#include <linux/spi/spi.h>

#include <video/ssd2119fb.h>

#define SSD2119_OSC_START_REG         0x00
#define SSD2119_OUTPUT_CTRL_REG       0x01
#define SSD2119_LCD_DRIVE_AC_CTRL_REG 0x02
#define SSD2119_DISPLAY_CTRL_REG      0x07
#define SSD2119_PWR_CTRL_2_REG        0x0C
#define SSD2119_PWR_CTRL_3_REG        0x0D
#define SSD2119_PWR_CTRL_4_REG        0x0E
#define SSD2119_SLEEP_MODE_REG        0x10
#define SSD2119_ENTRY_MODE_REG        0x11
#define SSD2119_RAM_DATA_REG          0x22
#define SSD2119_V_RAM_POS_REG         0x44
#define SSD2119_H_RAM_START_REG       0x45
#define SSD2119_H_RAM_END_REG         0x46
#define SSD2119_X_RAM_ADDR_REG        0x4E
#define SSD2119_Y_RAM_ADDR_REG        0x4F

#define LCD_WIDTH	320
#define LCD_HEIGHT	240

/* Frame buffer info */
struct ssd2119_fb_info {
	struct fb_info 		*info;
	struct spi_device	*spi;
	struct timer_list	auto_update_timer;
        dma_addr_t              map_dma;
        u_char 			*map_cpu;
        uint32_t                map_size;
        struct ssd2119_platform_data *pd;
        struct work_struct      task;

	struct mutex		lock;
};

// 16 bpp produces incorrect colours
#define BPP 16
//#define BPP 24

#define MAX_PALETTES      16

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
	.bits_per_pixel	= BPP,
	.nonstd		= 1,
};

static int ssd2119_write(struct ssd2119_fb_info *sinfo, u8 *buf, int len)
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

static int ssd2119_command(struct ssd2119_fb_info *sinfo, uint8_t reg)
{
        u8 tx[2];
        int status;

        tx[0] = 0;
        tx[1] = reg;

	gpio_set_value(sinfo->pd->gpio_dc, 0);
	status = ssd2119_write(sinfo, tx, 2);
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
        status = ssd2119_write(sinfo, tx, 2);
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

static int ssd2119_fb_check_var(struct fb_var_screeninfo *var,
			     struct fb_info *info)
{
	struct device *dev = info->device;

	dev_info(dev, "%s:\n", __func__);
	dev_info(dev, "  resolution: %ux%u\n", var->xres, var->yres);
	dev_info(dev, "  pixclk:     %u Hz\n", var->pixclock);
	dev_info(dev, "  bpp:        %u\n", var->bits_per_pixel);

	/* Force same alignment for each line */
	var->xres = (var->xres + 3) & ~3UL;
	var->xres_virtual = (var->xres_virtual + 3) & ~3UL;

	var->red.msb_right = var->green.msb_right = var->blue.msb_right = 0;
	var->transp.offset = var->transp.length = 0;

	switch (var->bits_per_pixel) {
	case 2:
	case 4:
	case 8:
		var->red.offset = var->green.offset = var->blue.offset = 0;
		var->red.length = var->green.length = var->blue.length
			= var->bits_per_pixel;
		break;
	case 16:
		var->red.offset 	= 0;
		var->red.length		= 6;
		var->green.offset	= 6;
		var->green.length	= 12;
		var->blue.offset	= 12;
		var->blue.length	= 6;
		var->transp.offset	= 0;
		var->transp.length	= 0;
		break;
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
		dev_err(dev, "color depth %d not supported\n",
					var->bits_per_pixel);
		return -EINVAL;
	}

	var->xoffset = var->yoffset = 0;
	var->red.msb_right = var->green.msb_right = var->blue.msb_right =
		var->transp.msb_right = 0;

	return 0;
}

static int ssd2119_fb_setcolreg(unsigned int regno, unsigned int red,
                unsigned int green, unsigned int blue,
                unsigned int transp, struct fb_info *info)
{
	unsigned int val;
        int ret = 0;

	if (regno < MAX_PALETTES) {
		u32 *pal = info->pseudo_palette;

		val = ((red & 0xff00) << 8) | (green & 0xff00) | ((blue & 0xff00) >> 8);
		pal[regno] = val;
		ret = 0;
	}
        return ret;
}


/**
 *      ssd2119_fb_set_par - Alters the hardware state.
 *      @info: frame buffer structure that represents a single frame buffer
 *
 *	Using the fb_var_screeninfo in fb_info we set the resolution
 *	of the this particular framebuffer. This function alters the
 *	par AND the fb_fix_screeninfo stored in fb_info. It doesn't
 *	not alter var in fb_info since we are using that data. This
 *	means we depend on the data in var inside fb_info to be
 *	supported by the hardware. If you can't
 *	change the resolution you don't need this function.
 *
 */
static int ssd2119_fb_set_par(struct fb_info *info)
{
	int ret = 0;

	dev_info(info->device, "+%s\n", __func__);
	dev_info(info->device, "    resolution: %ux%u (%ux%u virtual) %u bpp\n",
		 info->var.xres, info->var.yres,
		 info->var.xres_virtual, info->var.yres_virtual,
		 info->var.bits_per_pixel);

	dev_info(info->device, "-%s\n", __func__);

	return ret;
}

static int ssd2119_fb_pan_display(struct fb_var_screeninfo *var,
			       struct fb_info *info)
{
	int ret = 0;

	dev_info(info->device, "+%s\n", __func__);
	dev_info(info->device, "    xoffset=%u yoffset=%u\n",
			var->xoffset, var->yoffset);

	dev_info(info->device, "-%s\n", __func__);
	return ret;
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

static int ssd2119_fb_update(struct ssd2119_fb_info *sinfo)
{
        struct fb_info *info = sinfo->info;
        int i;
        ssd2119_full_command(sinfo, SSD2119_X_RAM_ADDR_REG, 0);
        ssd2119_full_command(sinfo, SSD2119_Y_RAM_ADDR_REG, 0);
        ssd2119_command(sinfo, SSD2119_RAM_DATA_REG);

	gpio_set_value(sinfo->pd->gpio_dc, 1);

        /**
         * FIXME: For some reason we can't do a DMA of the entire
         * frame at once. Not sure why this is, but we seem to be
         * able to work around it by limiting our DMA transfers to 
         * less than 2048
         */
#define STEP (LCD_WIDTH * 2 * 6)
        for (i = 0; i < LCD_WIDTH * LCD_HEIGHT * BPP / 8; i+= STEP)
                ssd2119_write(sinfo, &((u8*)info->screen_base)[i], STEP);
        return 0;
}

static void ssd2119_fb_update_work(struct work_struct *work)
{
        struct ssd2119_fb_info *sinfo = container_of(work,
					struct ssd2119_fb_info, task);
        ssd2119_fb_update(sinfo);
}

static void ssd2119_fb_mod_timer(struct ssd2119_fb_info *sinfo)
{
	unsigned timeout;

	//mutex_lock(&sinfo->lock);

	if (!sinfo->pd->refresh_speed) { 
		if (timer_pending(&sinfo->auto_update_timer))
			del_timer_sync(&sinfo->auto_update_timer);
		return;
	}

	if (timer_pending(&sinfo->auto_update_timer))
		return;

	timeout = msecs_to_jiffies(1000 / sinfo->pd->refresh_speed);
	mod_timer(&sinfo->auto_update_timer, jiffies + timeout);
	//mutex_unlock(&sinfo->lock);
}

static void ssd2119fb_timer_func(unsigned long arg)
{
        struct ssd2119_fb_info *sinfo = (struct ssd2119_fb_info *)arg;

        schedule_work(&sinfo->task);
	ssd2119_fb_mod_timer(sinfo);
}

static void ssd2119fb_reset(struct ssd2119_fb_info *sinfo)
{
	gpio_set_value(sinfo->pd->gpio_dc, 0);

	/* Reset */
	gpio_set_value(sinfo->pd->gpio_reset, 0);
	udelay(10);
	gpio_set_value(sinfo->pd->gpio_reset, 1);

        // Initialise screen
        ssd2119_full_command(sinfo, SSD2119_SLEEP_MODE_REG, 0x0001);
        ssd2119_full_command(sinfo, SSD2119_OSC_START_REG, 0x0001);
        //ssd2119_full_command(sinfo, SSD2119_OUTPUT_CTRL_REG, 0x30ef);
        ssd2119_full_command(sinfo, SSD2119_OUTPUT_CTRL_REG, 0x38ef);
        //ssd2119_full_command(sinfo, SSD2119_OUTPUT_CTRL_REG, 0x78ef);
        ssd2119_full_command(sinfo, SSD2119_LCD_DRIVE_AC_CTRL_REG, 0x0600);
        ssd2119_full_command(sinfo, SSD2119_SLEEP_MODE_REG, 0x0000);
        mdelay(30);
        ssd2119_full_command(sinfo, SSD2119_DISPLAY_CTRL_REG, 0x0033);
        // ssd2119_full_command(sinfo, SSD2119_PWR_CTRL_2_REG, 0x0005);
        // ssd2119_full_command(sinfo, SSD2119_PWR_CTRL_3_REG, 0x0007);
        // ssd2119_full_command(sinfo, SSD2119_PWR_CTRL_4_REG, 0x3100);
        ssd2119_full_command(sinfo, SSD2119_V_RAM_POS_REG, (LCD_HEIGHT - 1) << 8);
        ssd2119_full_command(sinfo, SSD2119_H_RAM_START_REG, 0);
        ssd2119_full_command(sinfo, SSD2119_H_RAM_END_REG, LCD_WIDTH - 1);
#if BPP == 24
	ssd2119_full_command(sinfo, SSD2119_ENTRY_MODE_REG, 0x4230);
#else
	ssd2119_full_command(sinfo, SSD2119_ENTRY_MODE_REG, 0x6230);
#endif
        // ssd2119_full_command(sinfo, SSD2119_ENTRY_MODE_REG, 0x6830);
        // ssd2119_full_command(sinfo, SSD2119_ENTRY_MODE_REG, 0x6230);
}

static ssize_t ssd2119_fb_refresh_hz_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
        struct fb_info *info = dev_get_drvdata(dev);
        struct ssd2119_fb_info *sinfo = info->par;

	sinfo->pd->refresh_speed = simple_strtoul(buf, NULL, 0);
	ssd2119_fb_mod_timer(sinfo);
	
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
#ifdef SSD2119FB_DEBUG
        struct timespec start_time;
        struct timespec finish_time;
        unsigned long diff;

        start_time = current_kernel_time();
#endif
        ssd2119_fb_update(sinfo);

#ifdef SSD2119FB_DEBUG
        finish_time = current_kernel_time();
        diff = ((1000000000*finish_time.tv_sec)+finish_time.tv_nsec) - 
                ((1000000000*start_time.tv_sec)+start_time.tv_nsec);
        printk("Fill took %luns\n", diff);
#endif

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
	.owner			= THIS_MODULE,
	.fb_check_var	= ssd2119_fb_check_var,
	.fb_set_par		= ssd2119_fb_set_par,
	.fb_setcolreg	= ssd2119_fb_setcolreg,
	.fb_pan_display	= ssd2119_fb_pan_display,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
        .fb_blank       = ssd2119_fb_blank,
};

static int ssd2119_fb_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct ssd2119_fb_info *sinfo;
	struct fb_info *info;
	int ret = -ENOMEM;

	// Allocate Frame buffer
	info = framebuffer_alloc(sizeof(struct ssd2119_fb_info), dev);
	if (!info) {
			dev_err(dev, "Cannot allocate memory\n");
			goto out;
	}

	info->pseudo_palette = kmalloc(sizeof (u32) * MAX_PALETTES, GFP_KERNEL);
	if (!info->pseudo_palette) {
		dev_err(dev, "Cannot allocate memory\n");
                goto free_info;
	}

	// Create SYSFS group
	ret = sysfs_create_group(&dev->kobj, &sysfs_files);
        if (ret) {
                dev_err(dev, "Unable to create sysfs files\n");
                goto free_pseudo_palette;
        }

	info->var = ssd2119fb_var;
	info->fix = ssd2119fb_fix;
	sinfo = info->par;
	sinfo->info = info;
	sinfo->spi = spi;
        sinfo->pd = dev->platform_data;
	
        if (!sinfo->pd->refresh_speed) {
                dev_warn(dev, "No refresh speed selected - default to 1 FPS\n");
                sinfo->pd->refresh_speed = HZ;
        }

	strcpy(info->fix.id, spi->modalias);
	info->fbops = &ssd2119_fb_ops;

        sinfo->map_size = LCD_WIDTH * LCD_HEIGHT * BPP / 8;
	// Allocate frame buffer
	sinfo->map_cpu = (void*)__get_free_pages(GFP_KERNEL, 6);
        sinfo->map_dma = dma_map_single(dev, sinfo->map_cpu, sinfo->map_size, DMA_TO_DEVICE);

        info->screen_base = sinfo->map_cpu;

	if (!info->screen_base) {
			dev_err(dev, "Out of memory\n");
			goto remove_sysfs;
	}
	info->fix.smem_start = sinfo->map_dma;
	info->fix.smem_len = sinfo->map_size;
	info->fix.line_length = LCD_WIDTH * BPP / 8;
	dev_info(info->device,
	       "%uKiB frame buffer at %08x (mapped at %p)\n",
               sinfo->map_size / 1024,
               (int)sinfo->map_dma,
	       sinfo->map_cpu);

	ret = ssd2119_fb_check_var(&info->var, info);

	dev_set_drvdata(dev, info);

	/*
	 * Request GPIOs
	 * FIXME - error handling
	 */
	gpio_request(sinfo->pd->gpio_dc, "ssd2119_dc");
	gpio_direction_output(sinfo->pd->gpio_dc, 0);
	gpio_request(sinfo->pd->gpio_reset, "ssd2119_reset");
	gpio_direction_output(sinfo->pd->gpio_reset, 1);
		
#if 0
	// Configure GPIO's
        if (sinfo->pd->init)
                sinfo->pd->init();
        sinfo->pd->set_dc(0);
        sinfo->pd->reset();

        // Initialise screen
        ssd2119_full_command(sinfo, SSD2119_SLEEP_MODE_REG, 0x0001);
        ssd2119_full_command(sinfo, SSD2119_OSC_START_REG, 0x0001);
        //ssd2119_full_command(sinfo, SSD2119_OUTPUT_CTRL_REG, 0x30ef);
        ssd2119_full_command(sinfo, SSD2119_OUTPUT_CTRL_REG, 0x38ef);
        //ssd2119_full_command(sinfo, SSD2119_OUTPUT_CTRL_REG, 0x78ef);
        ssd2119_full_command(sinfo, SSD2119_LCD_DRIVE_AC_CTRL_REG, 0x0600);
        ssd2119_full_command(sinfo, SSD2119_SLEEP_MODE_REG, 0x0000);
        mdelay(30);
        ssd2119_full_command(sinfo, SSD2119_DISPLAY_CTRL_REG, 0x0033);
        // ssd2119_full_command(sinfo, SSD2119_PWR_CTRL_2_REG, 0x0005);
        // ssd2119_full_command(sinfo, SSD2119_PWR_CTRL_3_REG, 0x0007);
        // ssd2119_full_command(sinfo, SSD2119_PWR_CTRL_4_REG, 0x3100);
        ssd2119_full_command(sinfo, SSD2119_V_RAM_POS_REG, (LCD_HEIGHT - 1) << 8);
        ssd2119_full_command(sinfo, SSD2119_H_RAM_START_REG, 0);
        ssd2119_full_command(sinfo, SSD2119_H_RAM_END_REG, LCD_WIDTH - 1);
#if BPP == 24
	ssd2119_full_command(sinfo, SSD2119_ENTRY_MODE_REG, 0x4230);
#else
	ssd2119_full_command(sinfo, SSD2119_ENTRY_MODE_REG, 0x6230);
#endif
        // ssd2119_full_command(sinfo, SSD2119_ENTRY_MODE_REG, 0x6830);
        // ssd2119_full_command(sinfo, SSD2119_ENTRY_MODE_REG, 0x6230);
#else
#endif

	// Register the frame buffer
	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(dev, "Failed to register framebuffer device: %d\n", ret);
		goto release_fb;
	}

	INIT_WORK(&sinfo->task, ssd2119_fb_update_work);
	mutex_init(&sinfo->lock);

	ssd2119fb_reset(sinfo);

        /* Initialise the auto-refresh timer and kick it off if necessary */
	setup_timer(&sinfo->auto_update_timer, ssd2119fb_timer_func,
		    (unsigned long)sinfo);
	ssd2119_fb_mod_timer(sinfo);

	if (sinfo->pd->refresh_speed)
		dev_info(dev, "Initialised - Auto refreshing at %d hz\n",
			 sinfo->pd->refresh_speed);
	else
		dev_info(dev, "Initialised - Auto refresh disabled\n"); 
	return 0;

release_fb:
	free_pages((unsigned long)sinfo->map_cpu, 6);
remove_sysfs:
	sysfs_remove_group(&dev->kobj, &sysfs_files);
free_pseudo_palette:
	kfree(info->pseudo_palette);
free_info:
	framebuffer_release(info);
out:
	dev_dbg(dev, "%s FAILED\n", __func__);
	return ret;
}

static int ssd2119_fb_remove(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct fb_info *info = dev_get_drvdata(dev);
	struct ssd2119_fb_info *sinfo = info->par;

	// Remove update timer
	del_timer_sync(&sinfo->auto_update_timer);

	// Remove SYS files
	sysfs_remove_group(&dev->kobj, &sysfs_files);

	// Free frame buffer
	if (info->screen_base)
                free_pages((unsigned long)sinfo->map_cpu, 6);

	/* Free gpios */
	gpio_free(sinfo->pd->gpio_dc);
	gpio_free(sinfo->pd->gpio_reset);

	kfree(info->pseudo_palette);

	dev_set_drvdata(dev, NULL);
	framebuffer_release(info);
	return 0;
}

static struct spi_driver ssd2119fb_driver = {
    	.driver = {
        	.name   = "ssd2119fb",
                .bus = &spi_bus_type,
                .owner  = THIS_MODULE,
        },
        .probe  = ssd2119_fb_probe,
        .remove = ssd2119_fb_remove,
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

