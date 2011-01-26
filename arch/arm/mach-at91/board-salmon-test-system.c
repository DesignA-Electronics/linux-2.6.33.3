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

#include <mach/board.h>
#include <asm/hardware/snapper_baseboard.h>
#include <video/ssd2119fb.h>

#define SSD2119_GPIO_DC	
#define SSD2119_GPIO_RESET
#define SSD2119_GPIO_BUS_ENABLE

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
	.gpio_reset	= AT91_PIN_PD1,
	.gpio_dc	= AT91_PIN_PB11,
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
		.modalias	= "stmpe610",
		.chip_select	= 3,
		.bus_num	= 1,
		.irq		= gpio_to_irq(AT91_PIN_PD0),
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

static int __init salmon_test_init(void)
{
	/* 
	 * Put lcd pins in gpio mode and disable their pullups (they get put
	 * in periph mode by sn9g45 ssc init code
	 */
	at91_set_GPIO_periph(AT91_PIN_PD1, 0);
	at91_set_GPIO_periph(AT91_PIN_PB11, 0);

	/* Touchscreen interrupt gpio */
	at91_set_GPIO_periph(AT91_PIN_PD0, 0);

	at91_add_device_spi(salmon_test_spi_devices,
			    ARRAY_SIZE(salmon_test_spi_devices));

	at91_add_device_adc(&adc_data);
	return 0;
}

SNAPPER_BASEBOARD(salmon_test_system, salmon_test_init);
