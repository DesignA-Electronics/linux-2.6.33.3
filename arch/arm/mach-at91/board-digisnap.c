/*
 * linux/arch/arm/mach-at91/board-digisnap.c
 *
 *  Copyright (C) 2011 Bluewater Systems Ltd
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/spi/spi.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/platform_device.h>
#include <linux/input.h>

#include <mach/board.h>

static struct spi_board_info digisnap_spi_board_info[] __initdata = {
	{
		.modalias	= "spidev",
		.max_speed_hz	= 1000000,
		.bus_num	= 1,
		.chip_select	= 0,
	},
};

static struct gpio_keys_button digisnap_buttons[] = {
	{
		.gpio		= AT91_PIN_PB8,
		.code		= KEY_UP,
		.desc		= "Sup",
		.active_low	= 0,
		.wakeup		= 0,
	},
	{
		.gpio		= AT91_PIN_PB9,
		.code		= KEY_DOWN,
		.desc		= "Sdown",
		.active_low	= 0,
		.wakeup		= 0,
	},
	{
		.gpio		= AT91_PIN_PB6,
		.code		= KEY_LEFT,
		.desc		= "Sleft",
		.active_low	= 0,
		.wakeup		= 0,
	},
	{
		.gpio		= AT91_PIN_PB7,
		.code		= KEY_RIGHT,
		.desc		= "Sright",
		.active_low	= 1,
		.wakeup		= 0,
	},
	{
		.gpio		= AT91_PIN_PB5,
		.code		= KEY_ENTER,
		.desc		= "Scenter",
		.active_low	= 1,
		.wakeup		= 1,
	},
	{
		/* Not really a button, but we want it as a wakeup source */
		.gpio		= AT91_PIN_PB18,
		.code		= KEY_1,
		.desc		= "TrigIn",
		.active_low	= 1,
		.wakeup		= 1,
	},
};

static struct gpio_keys_platform_data digisnap_button_data = {
	.buttons	= digisnap_buttons,
	.nbuttons	= ARRAY_SIZE(digisnap_buttons),
};

static struct platform_device digisnap_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data = &digisnap_button_data,
	},
};

static struct at91_adc_data digisnap_adc_data = {
	.gpios[1]	= AT91_PIN_PC1, /* Battery voltage */
	.gpios[2]	= AT91_PIN_PC2, /* Camera voltage */
	.gpios[3]	= AT91_PIN_PC3, /* Camera current */
	.prescale	= 0xff,
	.startup	= 12,
	.sample		= 0xf,
};

static int __init digisnap_init(void)
{
	int i;

	at91_add_device_spi(digisnap_spi_board_info,
			    ARRAY_SIZE(digisnap_spi_board_info));
	at91_add_device_adc(&digisnap_adc_data);

	for (i = 0; i < ARRAY_SIZE(digisnap_buttons); i++)
		at91_set_gpio_input(digisnap_buttons[i].gpio, 1);
	platform_device_register(&digisnap_button_device);
	return 0;
}
arch_initcall(digisnap_init);

