/*
 *  Board-specific setup code for the AT91SAM9M10G45 Evaluation Kit family
 *
 *  Covers: * AT91SAM9G45-EKES  board
 *          * AT91SAM9M10G45-EK board
 *
 *  Copyright (C) 2009 Atmel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/fb.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/leds.h>
#include <linux/clk.h>

#include <mach/hardware.h>
#include <video/atmel_lcdc.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/at91sam9_smc.h>
#include <mach/at91_shdwc.h>

#include "sam9_smc.h"
#include "generic.h"

#define PWR_HOLD_GPIO	AT91_PIN_PD10

static void hammerhead_poweroff(void)
{
	at91_sys_write(AT91_SHDW_CR, AT91_SHDW_KEY | AT91_SHDW_SHDW);
	at91_set_gpio_output(PWR_HOLD_GPIO, 0);
}

static void __init hammerhead_map_io(void)
{
	/* Initialize processor: 12.000 MHz crystal */
	at91sam9g45_initialize(12000000);

	/* Override power off */
	pm_power_off = hammerhead_poweroff;

	/* DGBU on ttyS0. (Rx & Tx only) */
	at91_register_uart(0, 0, 0);

	/* USART0 not connected on the -EK board */
	/* USART1 on ttyS2. (Rx, Tx, RTS, CTS) */
	at91_register_uart(AT91SAM9G45_ID_US1, 2, ATMEL_UART_CTS | ATMEL_UART_RTS);

	/* set serial console to ttyS0 (ie, DBGU) */
	at91_set_serial_console(0);
}

static void __init hammerhead_init_irq(void)
{
	at91sam9g45_init_interrupts(NULL);
}


/*
 * USB HS Host port (common to OHCI & EHCI)
 */
static struct at91_usbh_data __initdata hammerhead_usbh_hs_data = {
	.ports		= 2,
	.vbus_pin	= {AT91_PIN_PD1, AT91_PIN_PD3},
};


/*
 * USB HS Device port
 */
static struct usba_platform_data __initdata hammerhead_usba_udc_data = {
	.vbus_pin	= AT91_PIN_PB19,
};


/*
 * MACB Ethernet device
 */
static struct at91_eth_data __initdata hammerhead_macb_data = {
	/*.phy_irq_pin	= AT91_PIN_PD5,*/
	.is_rmii	= 1,
	.phy_mask	= ~(1 << 0x01), /* LAN8720 Phy Address = 0x01 */
};

/*
 * NAND flash
 */
static struct mtd_partition __initdata hammerhead_nand_partition[] = {
	{
		.name	= "Bootstrap",
		.offset	= 0,
		.size	= SZ_256K,
	},
	{
		.name	= "Bootloader",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= SZ_256K,
	},
	{
		.name	= "Environment",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= SZ_256K,
	},
	{
		.name	= "Reserved",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= SZ_256K,
	},
	{
		.name	= "Fpga1",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= SZ_8M,
	},
	{
		.name	= "Fpga2",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= SZ_8M,
	},
	{
		.name	= "Safe",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= SZ_16M,
	},
	{
		.name	= "Working",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= SZ_16M,
	},
	{
		.name	= "Filesystem",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct mtd_partition * __init nand_partitions(int size, int *num_partitions)
{
	*num_partitions = ARRAY_SIZE(hammerhead_nand_partition);
	return hammerhead_nand_partition;
}

/* det_pin is not connected */
static struct atmel_nand_data __initdata hammerhead_nand_data = {
	.ale		= 21,
	.cle		= 22,
	.rdy_pin	= AT91_PIN_PC8,
	.enable_pin	= AT91_PIN_PC14,
	.partition_info	= nand_partitions,
#if defined(CONFIG_MTD_NAND_AT91_BUSWIDTH_16)
	.bus_width_16	= 1,
#else
	.bus_width_16	= 0,
#endif
};

static struct sam9_smc_config __initdata hammerhead_nand_smc_config = {
	.ncs_read_setup		= 0,
	.nrd_setup		= 2,
	.ncs_write_setup	= 0,
	.nwe_setup		= 2,

	.ncs_read_pulse		= 4,
	.nrd_pulse		= 4,
	.ncs_write_pulse	= 4,
	.nwe_pulse		= 4,

	.read_cycle		= 7,
	.write_cycle		= 7,

	.mode			= AT91_SMC_READMODE | AT91_SMC_WRITEMODE | AT91_SMC_EXNWMODE_DISABLE,
	.tdf_cycles		= 3,
};

static void __init hammerhead_add_device_nand(void)
{
	/* setup bus-width (8 or 16) */
	if (hammerhead_nand_data.bus_width_16)
		hammerhead_nand_smc_config.mode |= AT91_SMC_DBW_16;
	else
		hammerhead_nand_smc_config.mode |= AT91_SMC_DBW_8;

	/* configure chip-select 3 (NAND) */
	sam9_smc_configure(3, &hammerhead_nand_smc_config);

	at91_add_device_nand(&hammerhead_nand_data);
}


/*
 * LCD Controller
 */
#if defined(CONFIG_FB_ATMEL) || defined(CONFIG_FB_ATMEL_MODULE)
static struct fb_videomode at91_tft_vga_modes[] = {
	{
		.name           = "LG",
		.refresh	= 60,
		.xres		= 480,		.yres		= 272,
		.pixclock	= KHZ2PICOS(9000),

		.left_margin	= 1,		.right_margin	= 1,
		.upper_margin	= 40,		.lower_margin	= 1,
		.hsync_len	= 45,		.vsync_len	= 1,

		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
};

static struct fb_monspecs at91fb_default_monspecs = {
	.manufacturer	= "LG",
	.monitor        = "LB043WQ1",

	.modedb		= at91_tft_vga_modes,
	.modedb_len	= ARRAY_SIZE(at91_tft_vga_modes),
	.hfmin		= 15000,
	.hfmax		= 17640,
	.vfmin		= 57,
	.vfmax		= 67,
};

#define AT91SAM9G45_DEFAULT_LCDCON2 	(ATMEL_LCDC_MEMOR_LITTLE \
					| ATMEL_LCDC_DISTYPE_TFT \
					| ATMEL_LCDC_CLKMOD_ALWAYSACTIVE)

/* Driver datas */
static struct atmel_lcdfb_info __initdata hammerhead_lcdc_data = {
	.lcdcon_is_backlight		= true,
	.default_bpp			= 32,
	.default_dmacon			= ATMEL_LCDC_DMAEN,
	.default_lcdcon2		= AT91SAM9G45_DEFAULT_LCDCON2,
	.default_monspecs		= &at91fb_default_monspecs,
	.guard_time			= 9,
	.lcd_wiring_mode		= ATMEL_LCDC_WIRING_RGB,
};

#else
static struct atmel_lcdfb_info __initdata hammerhead_lcdc_data;
#endif


/*
 * Touchscreen
 */
static struct at91_tsadcc_data hammerhead_tsadcc_data = {
	.adc_clock		= 300000,
	.pendet_debounce	= 0x0d,
	.ts_sample_hold_time	= 0x0a,
};


/*
 * AC97
 * reset_pin is not connected: NRST
 */
static struct ac97c_platform_data hammerhead_ac97_data = {
};

/*
 * A/D
 */
static struct at91_adc_data hammerhead_adc_data = {
        .gpios[0] = AT91_PIN_PD26,
        .gpios[1] = AT91_PIN_PD27,
        .gpios[2] = -1,
        .gpios[3] = -1,
		.gpios[4] = -1,
		.gpios[5] = -1,
		.gpios[6] = -1,
		.gpios[7] = -1,
        .prescale = 4,
        .startup = 12,
        .sample = 12,
};

static struct i2c_board_info __initdata hammerhead_i2c0_devices[] = {
	{
        },
};

static struct i2c_board_info __initdata hammerhead_i2c1_devices[] = {
	{
        },
};

static struct spi_board_info hammerhead_spi_devices[] = {
	{
		.modalias	= "tlv320aic26",
		.chip_select	= 0,
		.bus_num	= 0,
		.max_speed_hz	= 20 * 1000 * 1000,
	},
};

/*
 * GPIO Buttons
 */

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
static struct gpio_keys_button hammerhead_buttons[] = {
	{
		.type	= EV_PWR,
		.code	= KEY_POWER,
		.gpio	= AT91_PIN_PC9,
		.desc	= "Power Button",
		.wakeup = 1,
		.active_low = 1,
	},
};

static struct gpio_keys_platform_data hammerhead_button_data = {
	.buttons	= hammerhead_buttons,
	.nbuttons	= ARRAY_SIZE(hammerhead_buttons),
};

static struct platform_device hammerhead_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &hammerhead_button_data,
	}
};

static void __init hammerhead_add_device_buttons(void)
{
	at91_set_GPIO_periph(AT91_PIN_PC9, 1);	/* user push button, pull up enabled */
	at91_set_deglitch(AT91_PIN_PC9, 1);

	platform_device_register(&hammerhead_button_device);
}
#else
static void __init hammerhead_add_device_buttons(void) {}
#endif

static void __init hammerhead_board_init(void)
{
	/* Serial */
	at91_add_device_serial();
	/* USB HS Host */
	at91_add_device_usbh_ohci(&hammerhead_usbh_hs_data);
	at91_add_device_usbh_ehci(&hammerhead_usbh_hs_data);
	/* USB HS Device */
	at91_add_device_usba(&hammerhead_usba_udc_data);
	/* Ethernet */
	at91_add_device_eth(&hammerhead_macb_data);
	/* NAND */
	hammerhead_add_device_nand();
	/* I2C */
	at91_add_device_i2c(0, hammerhead_i2c0_devices, ARRAY_SIZE(hammerhead_i2c0_devices));
	at91_add_device_i2c(1, hammerhead_i2c1_devices, ARRAY_SIZE(hammerhead_i2c1_devices));
	/* SPI */
	at91_add_device_spi(hammerhead_spi_devices, ARRAY_SIZE(hammerhead_spi_devices));
	/* Audio */
        /* FIXME: Don't have record pins here */
	at91_add_device_ssc(AT91SAM9G45_ID_SSC0, ATMEL_SSC_TX);
	/* adc */
	at91_add_device_adc(&hammerhead_adc_data);
#if 0
	/* LCD Controller */
	at91_add_device_lcdc(&hammerhead_lcdc_data);
	/* Touch Screen */
	at91_add_device_tsadcc(&hammerhead_tsadcc_data);
#endif
	/* GPIO keys */
	hammerhead_add_device_buttons();
}

MACHINE_START(HAMMERHEAD, "Hammerhead")
	/* Maintainer: Atmel */
	.phys_io	= AT91_BASE_SYS,
	.io_pg_offst	= (AT91_VA_BASE_SYS >> 18) & 0xfffc,
	.boot_params	= AT91_SDRAM_BASE + 0x100,
	.timer		= &at91sam926x_timer,
	.map_io		= hammerhead_map_io,
	.init_irq	= hammerhead_init_irq,
	.init_machine	= hammerhead_board_init,
MACHINE_END
