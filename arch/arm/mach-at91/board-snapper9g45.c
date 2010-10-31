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

#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/clk.h>
#include <linux/atmel-mci.h>

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

static void __init sn9g45_map_io(void)
{
	/* Initialize processor: 12.000 MHz crystal */
	at91sam9g45_initialize(12000000);

	/* DGBU on ttyS0. (Rx & Tx only) */
	at91_register_uart(0, 0, 0);

	/* USART0 not connected on the -EK board */
	/* USART1 on ttyS2. (Rx, Tx, RTS, CTS) */
	at91_register_uart(AT91SAM9G45_ID_US1, 1, 0);
	at91_register_uart(AT91SAM9G45_ID_US2, 2, 
			   ATMEL_UART_CTS | ATMEL_UART_RTS);
	at91_register_uart(AT91SAM9G45_ID_US3, 3, ATMEL_UART_RTS);

	/* set serial console to ttyS0 (ie, DBGU) */
	at91_set_serial_console(0);
}

static void __init sn9g45_init_irq(void)
{
	at91sam9g45_init_interrupts(NULL);
}

/*
 * USB HS Host port (common to OHCI & EHCI)
 */
static struct at91_usbh_data __initdata sn9g45_usbh_hs_data = {
	.ports		= 1,
};

/*
 * USB HS Device port
 */
static struct usba_platform_data __initdata sn9g45_usba_udc_data = {
	.vbus_pin	= AT91_PIN_PC0,
};

/*
 * MACB Ethernet device
 */
static struct at91_eth_data __initdata sn9g45_macb_data = {
	.is_rmii	= 1,
        .phy_mask       = ~(1 << 27), // Phy 0x1b
};

/*
 * NAND flash
 */
static struct mtd_partition __initdata sn9g45_nand_partition[] = {
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
		.name	= "Kernel",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= SZ_4M,
	},
	{
		.name	= "Filesystem",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct mtd_partition * __init nand_partitions(int size, int *num_partitions)
{
	*num_partitions = ARRAY_SIZE(sn9g45_nand_partition);
	return sn9g45_nand_partition;
}

/* det_pin is not connected */
static struct atmel_nand_data __initdata sn9g45_nand_data = {
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

static struct sam9_smc_config __initdata sn9g45_nand_smc_config = {
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

static void __init sn9g45_add_device_nand(void)
{
	/* setup bus-width (8 or 16) */
	if (sn9g45_nand_data.bus_width_16)
		sn9g45_nand_smc_config.mode |= AT91_SMC_DBW_16;
	else
		sn9g45_nand_smc_config.mode |= AT91_SMC_DBW_8;

	/* configure chip-select 3 (NAND) */
	sam9_smc_configure(3, &sn9g45_nand_smc_config);

	at91_add_device_nand(&sn9g45_nand_data);
}

static struct mci_platform_data __initdata sn9g45_mmc_data = {
	.slot[0] = {
		.bus_width	= 4,
		.detect_pin	= -1,
		.wp_pin		= -1,
	},
};

/*
 * LCD Controller
 */
#if defined(CONFIG_FB_ATMEL) || defined(CONFIG_FB_ATMEL_MODULE)
static struct fb_videomode at91_tft_vga_modes[] = {
	{
		.name           = "SDTV",
		.refresh	= 60,
		.xres		= 720,		.yres		= 576,
		.pixclock	= KHZ2PICOS(33333),

		.left_margin	= 45,		.right_margin	= 16,
		.upper_margin	= 44,		.lower_margin	= 11,
		.hsync_len	= 96,		.vsync_len	= 2,

		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
	{
		.name           = "VGA",
		.refresh	= 60,
		.xres		= 640,		.yres		= 480,
		.pixclock	= KHZ2PICOS(27000),

		.left_margin	= 65,		.right_margin	= 16,
		.upper_margin	= 51,		.lower_margin	= 11,
		.hsync_len	= 96,		.vsync_len	= 2,

		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
	{
		.name           = "SVGA",
		.refresh	= 60,
		.xres		= 800,		.yres		= 600,
		.pixclock	= KHZ2PICOS(45000),

		.left_margin	= 174,		.right_margin	= 32,
		.upper_margin	= 75,		.lower_margin	= 3,
		.hsync_len	= 80,		.vsync_len	= 4,

		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
	{
		.name           = "XGA",
		.refresh	= 60,
		.xres		= 1024,		.yres		= 768,
		.pixclock	= KHZ2PICOS(67600),

		.left_margin	= 174,		.right_margin	= 24,
		.upper_margin	= 41,		.lower_margin	= 3,
		.hsync_len	= 136,		.vsync_len	= 6,

		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
};

static struct fb_monspecs at91fb_default_monspecs = {
	.manufacturer	= NULL,
	.monitor        = NULL,

	.modedb		= at91_tft_vga_modes,
	.modedb_len	= ARRAY_SIZE(at91_tft_vga_modes),
	.hfmin		= 15000,
	.hfmax		= 17640,
	.vfmin		= 60,
	.vfmax		= 75,
};

#define AT91SAM9G45_DEFAULT_LCDCON2 	(ATMEL_LCDC_MEMOR_LITTLE \
					| ATMEL_LCDC_DISTYPE_TFT \
					| ATMEL_LCDC_CLKMOD_ALWAYSACTIVE)

/* Driver datas */
static struct atmel_lcdfb_info __initdata sn9g45_lcdc_data = {
	.lcdcon_is_backlight		= true,
	.default_bpp			= 16,
	.default_dmacon			= ATMEL_LCDC_DMAEN,
	.default_lcdcon2		= AT91SAM9G45_DEFAULT_LCDCON2,
	.default_monspecs		= &at91fb_default_monspecs,
	.guard_time			= 9,
	.lcd_wiring_mode		= ATMEL_LCDC_WIRING_RGB,
};

#else
static struct atmel_lcdfb_info __initdata sn9g45_lcdc_data;
#endif

/*
 * Touchscreen
 */
static struct at91_tsadcc_data sn9g45_tsadcc_data = {
	.adc_clock		= 300000,
	.pendet_debounce	= 0x08,
	.ts_sample_hold_time	= 0x0a,
};

static struct i2c_board_info __initdata sn9g45_i2c_devices[] = {
	{
		I2C_BOARD_INFO("tlv320aic23", 0x1a),
        },
};

static void __init sn9g45_board_init(void)
{
	/* Serial */
	at91_add_device_serial();
	/* USB HS Host */
	at91_add_device_usbh_ohci(&sn9g45_usbh_hs_data);
	at91_add_device_usbh_ehci(&sn9g45_usbh_hs_data);
	/* USB HS Device */
	at91_add_device_usba(&sn9g45_usba_udc_data);
	/* Ethernet */
	at91_add_device_eth(&sn9g45_macb_data);
	/* NAND */
	sn9g45_add_device_nand();
	/* I2C */
	at91_add_device_i2c(0, sn9g45_i2c_devices, ARRAY_SIZE(sn9g45_i2c_devices));
	/* Audio */
        /* FIXME: Don't have record pins here */
	at91_add_device_ssc(AT91SAM9G45_ID_SSC0, ATMEL_SSC_TX);
	/* LCD Controller */
	at91_add_device_lcdc(&sn9g45_lcdc_data);
	/* Touch Screen */
	at91_add_device_tsadcc(&sn9g45_tsadcc_data);
	at91_add_device_mci(0, &sn9g45_mmc_data);
}

MACHINE_START(SNAPPER9G45, "Snapper9G45")
	/* Maintainer: Atmel */
	.phys_io	= AT91_BASE_SYS,
	.io_pg_offst	= (AT91_VA_BASE_SYS >> 18) & 0xfffc,
	.boot_params	= AT91_SDRAM_BASE + 0x100,
	.timer		= &at91sam926x_timer,
	.map_io		= sn9g45_map_io,
	.init_irq	= sn9g45_init_irq,
	.init_machine	= sn9g45_board_init,
MACHINE_END
