/*
 *  Board-specific setup code for the Bluewater Systems Snapper 9g45 module
 *
 *  Copyright (C) 2010 Bluewater Systems Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/hi358x.h>
#include <linux/platform_device.h>

#include <mach/board.h>
#include <asm/hardware/snapper_baseboard.h>
#include <video/ssd2119fb.h>

#define SSD2119_GPIO_DC	
#define SSD2119_GPIO_RESET
#define SSD2119_GPIO_BUS_ENABLE

static int baseboard_rev = -1;

static struct at91_adc_data adc_data = {
	.gpios[0] = AT91_PIN_PD20,
	.gpios[1] = AT91_PIN_PD21,
	.gpios[2] = AT91_PIN_PD22,
	.gpios[3] = AT91_PIN_PD23,
	.gpios[4] = AT91_PIN_PD24,
	.gpios[5] = AT91_PIN_PD25,
	.gpios[6] = AT91_PIN_PD26,
	.gpios[7] = AT91_PIN_PD27,

	.prescale = 0xff,
	.startup  = 12,
	.sample   = 0xf,
};

static struct ssd2119_platform_data ssd2119_data = {
	/* gpio_reset and gpio_dc get set in salmon_test_init */
	.refresh_speed	= 0,
};

static struct hi358x_platform_data salmon_test_429_csa_data = {
	.model	= HI3585,
	.tx_irq	= gpio_to_irq(AT91_PIN_PE2),
	.rx_irq = gpio_to_irq(AT91_PIN_PE3),
};

static struct hi358x_platform_data salmon_test_429_csb_data = {
	.model	= HI3587,
	.tx_irq	= gpio_to_irq(AT91_PIN_PE4),
	.rx_irq = -1,
};

static struct hi358x_platform_data salmon_test_429_csc_data = {
	.model	= HI3587,
	.tx_irq	= gpio_to_irq(AT91_PIN_PE5),
	.rx_irq = -1,
};
	
static struct spi_board_info salmon_test_spi_devices[] = {
	{
		.modalias	= "ssd2119fb",
		.chip_select	= 0,
		.bus_num	= 0,
		.max_speed_hz	= 90 * 1000 * 1000,
		.platform_data	= &ssd2119_data,
	},
	{
		/*
		 * The stmpe610's IRQ gets set in salmon_test_init.
		 * NOTE: It must be at position 1 in the array
		 */
		.modalias	= "stmpe610",
		.chip_select	= 3,
		.bus_num	= 1,
		.max_speed_hz	= 800000,
	},
	{
		/* 429_CSA */
		.modalias	= "hi358x",
		.max_speed_hz	= 2000000,
		.bus_num	= 0,
		.chip_select	= 1,
		.platform_data	= &salmon_test_429_csa_data,
	},
	{
		/* 429_CSB */
		.modalias	= "hi358x",
		.max_speed_hz	= 2000000,
		.bus_num	= 0,
		.chip_select	= 2,
		.platform_data	= &salmon_test_429_csb_data,
	},
	{
		/* 429_CSC */
		.modalias	= "hi358x",
		.max_speed_hz	= 2000000,
		.bus_num	= 0,
		.chip_select	= 3,
		.platform_data	= &salmon_test_429_csc_data,
	},
};

unsigned int irq_gpios[] = { AT91_PIN_PE1, AT91_PIN_PE10, AT91_PIN_PE8 };
unsigned int shdn_gpios[] = { AT91_PIN_PE0, AT91_PIN_PE7, AT91_PIN_PE9 };

static struct {
	unsigned int irq_count;
	unsigned int *irq_gpios;
	unsigned int shdn_count;
	unsigned int *shdn_gpios;
} gpios = {
	.irq_count = ARRAY_SIZE(irq_gpios),
	.irq_gpios = irq_gpios,
	.shdn_count = ARRAY_SIZE(shdn_gpios),
	.shdn_gpios = shdn_gpios,
};

static struct platform_device ext_shutdown = {
	.name	= "ext_shutdown",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpios,
	},
};

static int __init salmon_test_init(void)
{
	printk("Salmon Test System init rev %d\n", baseboard_rev);
	/* 
	 * Put lcd pins in gpio mode and disable their pullups (they get put
	 * in periph mode by sn9g45 ssc init code.
	 *
	 * Set the touch screen GPIO.
	 *
	 * Set the SSD pins.
	 */
	switch(baseboard_rev) {
	default:
	case 0:
		at91_set_GPIO_periph(AT91_PIN_PD1, 0);
		at91_set_GPIO_periph(AT91_PIN_PB11, 0);

		at91_set_GPIO_periph(AT91_PIN_PD0, 0);

		ssd2119_data.gpio_reset	= AT91_PIN_PD1;
		ssd2119_data.gpio_dc	= AT91_PIN_PB11;

		salmon_test_spi_devices[1].irq = gpio_to_irq(AT91_PIN_PD0);
		break;

	case 1:
		at91_set_GPIO_periph(AT91_PIN_PD5, 0);
		at91_set_GPIO_periph(AT91_PIN_PD10, 0);

		at91_set_GPIO_periph(AT91_PIN_PD15, 0);

		ssd2119_data.gpio_reset	= AT91_PIN_PD5;
		ssd2119_data.gpio_dc	= AT91_PIN_PD10;

		salmon_test_spi_devices[1].irq = gpio_to_irq(AT91_PIN_PD15);

		/* Disable the pullups to resolve parasitic power issues */
		at91_set_GPIO_periph(AT91_PIN_PE16, 0);
		at91_set_GPIO_periph(AT91_PIN_PE17, 0);
		at91_set_GPIO_periph(AT91_PIN_PE18, 0);
		at91_set_GPIO_periph(AT91_PIN_PE19, 0);
		at91_set_GPIO_periph(AT91_PIN_PE20, 0);
		at91_set_GPIO_periph(AT91_PIN_PE21, 0);
		at91_set_GPIO_periph(AT91_PIN_PE22, 0);
		at91_set_GPIO_periph(AT91_PIN_PE23, 0);
		at91_set_GPIO_periph(AT91_PIN_PE24, 0);
		at91_set_GPIO_periph(AT91_PIN_PE25, 0);
		at91_set_GPIO_periph(AT91_PIN_PE26, 0);
		at91_set_GPIO_periph(AT91_PIN_PE27, 0);

		platform_device_register(&ext_shutdown);
		break;
	}

	at91_add_device_spi(salmon_test_spi_devices,
			    ARRAY_SIZE(salmon_test_spi_devices));

	at91_add_device_adc(&adc_data);
	return 0;
}

static int __init salmon_test_init_rev(char *str)
{
	baseboard_rev = simple_strtol(str, NULL, 10);
	return 0;
}

__setup("baseboard_rev=", salmon_test_init_rev);

SNAPPER_BASEBOARD(salmon_test_system, salmon_test_init);
