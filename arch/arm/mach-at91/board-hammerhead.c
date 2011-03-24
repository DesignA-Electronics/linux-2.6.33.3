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
#include <linux/jiffies.h>

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
#include <mach/at91_adc.h>
#include <mach/hammerhead.h>

#include "sam9_smc.h"
#include "generic.h"

#define PWR_HOLD_GPIO	AT91_PIN_PD10

static int board_variant_id;

const char *hammerhead_variant_id_string(void)
{
	switch (board_variant_id) {
	case HAMMERHEAD_BOARD_REV0: return "Hammerhead rev 0";		
	case HAMMERHEAD_BOARD_REV1: return "Hammerhead rev 1";		
	}

	return "Unknown";
}
EXPORT_SYMBOL(hammerhead_variant_id_string);

int hammerhead_variant_id(void)
{
	return board_variant_id;
}
EXPORT_SYMBOL(hammerhead_variant_id);

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
	.vbus_pin 	= {AT91_PIN_PD20},
	.oc_irq		= {gpio_to_irq(AT91_PIN_PD19)},
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
	.phy_mask	= 0x00, /* Auto-detect LAN8720 Phy Address */
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
        .gpios[0] = -1,
        .gpios[1] = -1,
        .gpios[2] = -1,
        .gpios[3] = -1,
		.gpios[4] = -1,
		.gpios[5] = -1,
		.gpios[6] = AT91_PIN_PD26,
		.gpios[7] = AT91_PIN_PD27,
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
		.mode		= SPI_MODE_1,
		.max_speed_hz	= 10 * 1000 * 1000,
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

struct adc_thresh {
	unsigned min;
	unsigned max;
};

static void __init hammerhead_read_variant_id(void)
{
	struct clk *adc_clk;
	unsigned long timeout;
	unsigned adc_val, adc_vals[10];
	void __iomem *regs;
	int i;
	struct resource res = {
		.flags = IORESOURCE_MEM,
		.start = AT91SAM9G45_BASE_TSC,
		.end   = AT91SAM9G45_BASE_TSC + 0x100,
	}; 			 
	/* FIXME - These have been determined experimentally */
	struct adc_thresh board_id_table[] = {
		[HAMMERHEAD_BOARD_UNKNOWN] = {0xfff, 0xfff},
		[HAMMERHEAD_BOARD_REV0]    = {0x290, 0x2a5},
		[HAMMERHEAD_BOARD_REV1]    = {0x2d0, 0x2e5},
	};

	/* 
	 * ADC Channel 7 (PD27) contains a board variant id. Read it here and
	 * store the result
	 */       
        if (!request_mem_region(res.start, resource_size(&res), "at91_adc")) {
		pr_err("%s - Failed to request memory region\n", __func__);
                goto fail;
	}

	regs = ioremap(res.start, resource_size(&res));
	if (!regs) {
		pr_err("%s - Failed to remap memory\n", __func__);
		goto fail_release_io;
	}

	adc_clk = clk_get(NULL, "tsc_clk");
	if (IS_ERR(adc_clk)) {
		pr_err("%s - Cannot get ADC clock: %ld\n", 
		       __func__, PTR_ERR(adc_clk));
		goto fail_unmap_io;
	}

	clk_enable(adc_clk);
	at91_set_gpio_input(AT91_PIN_PD27, 0);

	/* Setup the ADC */
	__raw_writel(AT91_ADC_SWRST, regs + AT91_ADC_CR);
	__raw_writel((hammerhead_adc_data.prescale <<  8) |
		     (hammerhead_adc_data.startup  << 16) |
		     (hammerhead_adc_data.sample   << 24),
		     regs + AT91_ADC_MR);

	/*
	 * Read the ADC several times since it takes it some time to
	 * stabilise properly.
	 */	
	for (i = 0; i < ARRAY_SIZE(adc_vals); i++) {
		__raw_readl(regs + AT91_ADC_LCDR);
		__raw_writel(1 << 7, regs + AT91_ADC_CHER);
		__raw_writel(AT91_ADC_START, regs + AT91_ADC_CR);
		
		timeout = jiffies + msecs_to_jiffies(1000);
		while (1) {
			unsigned status = __raw_readl(regs + AT91_ADC_SR);

			if ((status & AT91_ADC_EOC(7)) &&
			    (status & AT91_ADC_DRDY)   &&
			    !(status & AT91_ADC_OVRE(7)))
				break;

			if (time_after(jiffies, timeout)) {
				pr_err("%s - Timeout waiting for ADC\n",
				       __func__);
				goto fail_put_clk;
			}
		}
		
		adc_vals[i]  = __raw_readl(regs + AT91_ADC_LCDR);
		adc_vals[i] &= AT91_ADC_CDR_MASK;
		__raw_writel(1 << 7, regs + AT91_ADC_CHDR);
	}

	/* Get the average ADC value */
	for (adc_val = 0, i = 0; i < ARRAY_SIZE(adc_vals); i++)
		adc_val += adc_vals[i];
	adc_val /= ARRAY_SIZE(adc_vals);

	/* Convert the ADC value to a board ID */
	board_variant_id = -1;
	for (i = 0; i < ARRAY_SIZE(board_id_table); i++) {
		if (adc_val >= board_id_table[i].min &&
		    adc_val <= board_id_table[i].max) {
			board_variant_id = i;
			break;
		}
	}

	pr_info("Hammerhead: Board variant %s (adc = %.3x)\n",
		hammerhead_variant_id_string(), adc_val);
	
	clk_disable(adc_clk);
	clk_put(adc_clk);
	iounmap(regs);
	release_mem_region(res.start, resource_size(&res));
	return;

fail_put_clk:
	clk_disable(adc_clk);
	clk_put(adc_clk);	
fail_unmap_io:
	iounmap(regs);
fail_release_io:
	release_mem_region(res.start, resource_size(&res));
fail:
	pr_err("Hammerhead: Failed to read board variant id\n");
}

static void __init hammerhead_board_init(void)
{
	hammerhead_read_variant_id();

	/* Serial */
	at91_add_device_serial();
	/* USB HS Host */
	at91_set_gpio_output(AT91_PIN_PD20, 1); /* USB_0_EN */
	at91_add_device_usbh_ohci(&hammerhead_usbh_hs_data);
	at91_add_device_usbh_ehci(&hammerhead_usbh_hs_data);
	/* USB HS Device */
	at91_add_device_usba(&hammerhead_usba_udc_data);
	/* Ethernet */
	at91_add_device_eth(&hammerhead_macb_data);
	/* NAND */
	at91_add_device_nand(&hammerhead_nand_data);
	/* I2C */
	at91_add_device_i2c(0, hammerhead_i2c0_devices,
			    ARRAY_SIZE(hammerhead_i2c0_devices));
	at91_add_device_i2c(1, hammerhead_i2c1_devices,
			    ARRAY_SIZE(hammerhead_i2c1_devices));
	/* SPI */
	at91_add_device_spi(hammerhead_spi_devices,
			    ARRAY_SIZE(hammerhead_spi_devices));

	/* Audio - RX is on TX Clock*/
	at91_add_device_ssc(AT91SAM9G45_ID_SSC0,
			    (ATMEL_SSC_TF | ATMEL_SSC_TX | ATMEL_SSC_RK |
			     ATMEL_SSC_TD | ATMEL_SSC_RD));

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
