/*
 * drivers/misc/at91_adc.c
 *
 *  Copyright (C) 2009 Bluewater Systems Ltd
 *  Author: Andre Renaud <andre@bluewatersys.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
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

#include <asm/io.h>

#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/at91_adc.h>

struct at91_adc {
        void __iomem *base;
        struct clk *clk;
        struct device *dev;
        int irq;
        int adc_vals[8];
        struct completion converted;

        struct at91_adc_data *data;
};

#define at91_adc_read(adc,reg)      __raw_readl((adc)->base + (reg))
#define at91_adc_write(adc,reg,val) __raw_writel((val), (adc)->base + (reg))

static void __init at91_adc_hwinit(struct at91_adc *adc)
{
        int i;
	
	/* Reset the ADC system */
        at91_adc_write(adc, AT91_ADC_CR, AT91_ADC_SWRST);
	
        /* Set up the prescale, startup & sample times */
        at91_adc_write(adc, AT91_ADC_MR, (adc->data->prescale << 8 | 
					  adc->data->startup << 16 | 
					  adc->data->sample << 24));
		       
        /* Enable all the interrupts */
        for (i = 0; i < ARRAY_SIZE(adc->data->gpios); i++)
                if (adc->data->gpios[i] >= 0) {
                        at91_adc_write(adc, AT91_ADC_IER, AT91_ADC_EOC(i));

			/*
			 * The first four ADCs are also touchscreen pins and
			 * use peripheral A mode. The second 4 ADC's (9G45
			 * only) are configured as gpio inputs with no pullup
			 */
			if (i < 4)
				at91_set_A_periph(adc->data->gpios[i], 0);
			else
				at91_set_gpio_input(adc->data->gpios[i], 0);
                }
}

static irqreturn_t at91_adc_irq(int irq, void *dev_id)
{
        struct at91_adc *adc = dev_id;
        int i;
        for (i = 0; i < ARRAY_SIZE(adc->data->gpios); i++) {
                if (at91_adc_read (adc, AT91_ADC_SR) & AT91_ADC_EOC(i)) {
                        adc->adc_vals[i] = at91_adc_read(adc, AT91_ADC_CDR(i) &
							 AT91_ADC_CDR_MASK);
                }
        }
	
        complete(&adc->converted);
        return IRQ_HANDLED;
}

int at91_adc_get_value(struct at91_adc *adc, int channel)
{
        init_completion(&adc->converted);
        
	/* Enable the channel */
        at91_adc_write(adc, AT91_ADC_CHER, 1 << channel);
	
	/* Start the conversion */
        at91_adc_write(adc, AT91_ADC_CR, AT91_ADC_START);

        wait_for_completion(&adc->converted);

	/* Disable the channel */
        at91_adc_write(adc, AT91_ADC_CHDR, 1 << channel);
        return adc->adc_vals[channel];
}
EXPORT_SYMBOL(at91_adc_get_value);

#define ADC_READ_ATTR(x)						\
        static ssize_t at91_adc_read_value_##x(struct device *dev,	\
					       struct device_attribute *attr, \
					       char *buf)		\
        {                                                               \
                struct at91_adc *adc = dev_get_drvdata(dev);            \
                if (adc->data->gpios[x] < 0)                            \
                        return sprintf (buf, "-1\n");                   \
                return sprintf(buf, "0x%x\n", at91_adc_get_value (adc, x)); \
        }                                                               \
        static DEVICE_ATTR(adc##x, S_IRUSR | S_IRUGO,			\
			   at91_adc_read_value_##x, NULL);

ADC_READ_ATTR(0);
ADC_READ_ATTR(1);
ADC_READ_ATTR(2);
ADC_READ_ATTR(3);
#ifdef CONFIG_ARCH_AT91SAM9G45
ADC_READ_ATTR(4);
ADC_READ_ATTR(5);
ADC_READ_ATTR(6);
ADC_READ_ATTR(7);
#endif

static struct attribute *at91_adc_attrs[] = {
        &dev_attr_adc0.attr,
        &dev_attr_adc1.attr,
        &dev_attr_adc2.attr,
        &dev_attr_adc3.attr,

#ifdef CONFIG_ARCH_AT91SAM9G45
        &dev_attr_adc4.attr,
        &dev_attr_adc5.attr,
        &dev_attr_adc6.attr,
        &dev_attr_adc7.attr,
#endif

        NULL,
};

static const struct attribute_group at91_adc_sysfs_files = {
        .attrs = at91_adc_attrs,
};

static int __init at91_adc_probe(struct platform_device *pdev)
{
        struct resource *res, *irq_res;
        int rc;
        struct at91_adc *adc;

        res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (!res)
                return -ENXIO;
        irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
        if (!irq_res)
                return -ENXIO;

        if (!request_mem_region(res->start, res->end - res->start + 1, 
                                "at91_adc"))
                return -EBUSY;

        adc = kmalloc(sizeof (struct at91_adc), GFP_KERNEL);
        if (!adc) {
                rc = -ENOMEM;
                goto fail0;
        }
        adc->irq = irq_res->start;
        adc->dev = &pdev->dev;
        adc->data = (struct at91_adc_data *)pdev->dev.platform_data;

        adc->base = ioremap(res->start, res->end - res->start + 1);
        if (!adc->base) {
                rc = -ENOMEM;
                goto fail1;
        }

#ifdef CONFIG_ARCH_AT91SAM9G45
	adc->clk = clk_get(NULL, "tsc_clk");
#else
        adc->clk = clk_get(NULL, "adc_clk");
#endif
        if (IS_ERR(adc->clk)) {
                dev_err(adc->dev, "no clock defined\n");
                rc = -ENODEV;
                goto fail2;
        }
        clk_enable(adc->clk);            /* enable peripheral clock */

        rc = request_irq(adc->irq, at91_adc_irq, IRQF_DISABLED, 
                        pdev->name, adc);
        if (rc != 0) {
                dev_err(adc->dev, "cannot claim IRQ\n");
                goto fail3;
        }

        rc = sysfs_create_group(&pdev->dev.kobj, &at91_adc_sysfs_files);
        if (rc)
                goto fail4;

        platform_set_drvdata(pdev, adc);
       
        at91_adc_hwinit (adc);
        dev_info(adc->dev, "AT91 ADC driver.\n");
        return 0;

fail4:
        free_irq(irq_res->start, NULL);
fail3:
        clk_disable(adc->clk);
        clk_put(adc->clk);
fail2:
        iounmap(adc->base);
fail1:
        kfree(adc);
fail0:
        release_mem_region(res->start, res->end - res->start + 1);

        return rc;
}

static int __exit at91_adc_remove(struct platform_device *pdev)
{
        struct at91_adc *adc = platform_get_drvdata(pdev);
        struct resource *res;

        free_irq(adc->irq, NULL);

        clk_disable(adc->clk);           /* disable peripheral clock */
        clk_put(adc->clk);

        res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        iounmap(adc->base);
        release_mem_region(res->start, res->end - res->start + 1);

        platform_set_drvdata(pdev, NULL);

        return 0;
}

#ifdef CONFIG_PM
/* NOTE: could save a few mA by keeping clock off outside of at91_xfer... */

static int at91_adc_suspend(struct platform_device *pdev, pm_message_t mesg)
{
        struct at91_adc *adc = platform_get_drvdata(pdev);

        clk_disable(adc->clk);
        return 0;
}

static int at91_adc_resume(struct platform_device *pdev)
{
        struct at91_adc *adc = platform_get_drvdata(pdev);
	
        return clk_enable(adc->clk);
}

#else
#define at91_adc_suspend        NULL
#define at91_adc_resume         NULL
#endif

static struct platform_driver at91_adc_driver = {
        .probe          = at91_adc_probe,
        .remove         = __exit_p(at91_adc_remove),
        .suspend        = at91_adc_suspend,
	.resume		= at91_adc_resume,
        .driver         = {
                .name   = "at91_adc",
                .owner  = THIS_MODULE,
        },
};

static int __init at91_adc_init(void)
{
        return platform_driver_register(&at91_adc_driver);
}

static void __exit at91_adc_exit(void)
{
        platform_driver_unregister(&at91_adc_driver);
}

module_init(at91_adc_init);
module_exit(at91_adc_exit);

MODULE_DESCRIPTION("AT91 ADC Driver");
MODULE_AUTHOR("Andre Renaud <andre@bluewatersys.com>");
MODULE_LICENSE("GPL");

