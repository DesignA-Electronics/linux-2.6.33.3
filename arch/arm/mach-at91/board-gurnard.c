/*
 *  Board-specific setup code for the Bluewater Systems Gurnard board
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

#define GURNARD_REVISION 1

#define PWR_HOLD_GPIO   AT91_PIN_PB16

#define FPGA_IRQ        AT91SAM9G45_ID_IRQ0
#define FPGA_IRQ_LEVEL  IRQ_TYPE_LEVEL_LOW

/* Taken from 7.3 of document gurnard 06 */
#define FPGA_ROM_PHYS          AT91_CHIPSELECT_0
#define FPGA_ROM_VIRT          (0xf0000000)
#define FPGA_ROM_SIZE          0x1000

#define FPGA_IRQ_PHYS           (FPGA_ROM_PHYS + 0x01000000)
#define FPGA_IRQ_VIRT           (0xf0100000)
#define FPGA_IRQ_SIZE           (0x1000)

#define FPGA_NAND_CTRL_PHYS          (FPGA_ROM_PHYS + 0x02000000)
#define FPGA_NAND_DATA_PHYS          (FPGA_ROM_PHYS + 0x02800000)

#define FPGA_ROM_REG(x) (*(volatile uint32_t *)(FPGA_ROM_VIRT + (x)))
#define FPGA_ID         FPGA_ROM_REG(0x00)
#define FPGA_BUILD_DATE FPGA_ROM_REG(0x04)
#define FPGA_EMULATION  FPGA_ROM_REG(0x08)
#define FPGA_BUILD_TEXT FPGA_ROM_REG(0x100)

#define FPGA_IRQ_REG(x) (*(volatile uint32_t *)(FPGA_IRQ_VIRT + (x)))
#define FPGA_IER        FPGA_IRQ_REG(0x00)
#define FPGA_IFR        FPGA_IRQ_REG(0x04)

static struct map_desc fpga_io_desc[] __initdata = {
        {
                .virtual        = FPGA_ROM_VIRT,
                .pfn            = __phys_to_pfn(FPGA_ROM_PHYS),
                .length         = FPGA_ROM_SIZE,
                .type           = MT_DEVICE,
        },
        {
                .virtual        = FPGA_IRQ_VIRT,
                .pfn            = __phys_to_pfn(FPGA_IRQ_PHYS),
                .length         = FPGA_IRQ_SIZE,
                .type           = MT_DEVICE,
        },
};

/* Power hold */
static void gurnard_poweroff(void)
{
        at91_sys_write(AT91_SHDW_CR, AT91_SHDW_KEY | AT91_SHDW_SHDW);
        at91_set_gpio_output(PWR_HOLD_GPIO, 0);
}

static void __init gurnard_map_io(void)
{
	/* Initialize processor: 12.000 MHz crystal */
	at91sam9g45_initialize(12000000);

	/* DGBU on ttyS0. (Rx & Tx only) */
	at91_register_uart(0, 0, 0);

        /* UART0 and UART1 on gurnard */
	at91_register_uart(AT91SAM9G45_ID_US0, 1,
			   ATMEL_UART_CTS | ATMEL_UART_RTS);
	at91_register_uart(AT91SAM9G45_ID_US1, 2,
			   ATMEL_UART_CTS | ATMEL_UART_RTS);

        pm_power_off = gurnard_poweroff;

	/* set serial console to ttyS0 (ie, DBGU) */
	at91_set_serial_console(0);

        /* Map the FPGA base area */
        iotable_init(fpga_io_desc, ARRAY_SIZE(fpga_io_desc));
}

/**
 * FPGA interrupt counter
 */
static uint32_t fpga_irq_count = 0;

static void gurnard_irq_handler(unsigned int irq, struct irq_desc *desc)
{
        uint32_t pending = FPGA_IFR;

        /**
         * If the FPGA is not programmed, then skip it.
         * These spurious interrupts can happen during the programming
         * of the FPGA
         **/
        if ((FPGA_ID & 0xffff0000) != 0x000d0000 &&
            (FPGA_ID & 0xffff0000) != 0x00010000) {
                pr_err("Gurnard FPGA IRQ, but FPGA not programmed\n");
                return;
        }

        fpga_irq_count++;

        do {
                if (likely(pending)) {
                        irq = BOARD_IRQ(0) + __ffs(pending);
                        generic_handle_irq(irq);
                        FPGA_IFR = 1 << (irq - BOARD_IRQ(0));
                }
                pending = FPGA_IFR;
        } while (pending);
}

static void gurnard_mask_irq(unsigned int irq)
{
        int fpga_irq = (irq - BOARD_IRQ(0));
        FPGA_IER &= ~(1 << fpga_irq);
}

static void gurnard_unmask_irq(unsigned int irq)
{
        int fpga_irq = (irq - BOARD_IRQ(0));
        FPGA_IER |= 1 << fpga_irq;
}

static struct irq_chip gurnard_irq_chip = {
        .name           = "FPGA",
        .ack            = gurnard_mask_irq,
        .mask           = gurnard_mask_irq,
        .unmask         = gurnard_unmask_irq,
};

static void __init gurnard_init_irq(void)
{
        int irq;
	at91sam9g45_init_interrupts(NULL);

        /* Until the FPGA is programmed, the IRQ pin will float.
         * Turn on the in-CPU pull-up to prevent this being a problem
         */
        at91_set_gpio_input(FPGA_IRQ, 1);

        /* The Gurnard FPGA has two irqs - one for NAND and one for
           emulation
         */
        for (irq = BOARD_IRQ(0); irq <= BOARD_IRQ(1); irq++) {
                set_irq_chip(irq, &gurnard_irq_chip);
                set_irq_handler(irq, handle_level_irq);
                set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
        }

        set_irq_type(FPGA_IRQ, FPGA_IRQ_LEVEL);
        set_irq_chained_handler(FPGA_IRQ, gurnard_irq_handler);
}

/*
 * MACB Ethernet device
 */
static struct at91_eth_data __initdata gurnard_macb_data = {
	.is_rmii	= 1,
        .phy_mask       = ~(1 << 0), // Phy 0x0
};

#if 0
/**
 * Board detection ADC
 */
static struct at91_adc_data gurnard_adc_data = {
        .gpios[0] = -1,
        .gpios[1] = AT91_PIN_PD21,
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
#endif

/* LEDS */
static struct gpio_led gurnard_leds[] = {
        {	/* power LED */
                .name			= "heartbeat",
                .gpio			= AT91_PIN_PB27,
                .active_low		= 1,
                .default_trigger	= "heartbeat",
        },
};

/*
 * NAND flash
 */
static struct mtd_partition __initdata gurnard_nand_partition[] = {
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
		.name	= "Kernel0",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= SZ_32M,
	},
        {
                .name   = "Kernel1",
                .offset = MTDPART_OFS_NXTBLK,
                .size   = SZ_32M,
        },
        {
		.name	= "Working",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct mtd_partition * __init nand_partitions(int size,
                int *num_partitions)
{
	*num_partitions = ARRAY_SIZE(gurnard_nand_partition);
	return gurnard_nand_partition;
}

/* det_pin is not connected */
static struct atmel_nand_data __initdata gurnard_nand_data = {
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

static struct sam9_smc_config __initdata gurnard_nand_smc_config = {
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

	.mode			= AT91_SMC_READMODE | AT91_SMC_WRITEMODE |
                                  AT91_SMC_EXNWMODE_DISABLE,
	.tdf_cycles		= 3,
};

static void __init gurnard_add_device_nand(void)
{
	/* setup bus-width (8 or 16) */
	if (gurnard_nand_data.bus_width_16)
		gurnard_nand_smc_config.mode |= AT91_SMC_DBW_16;
	else
		gurnard_nand_smc_config.mode |= AT91_SMC_DBW_8;

	/* configure chip-select 3 (NAND) */
	sam9_smc_configure(3, &gurnard_nand_smc_config);

	at91_add_device_nand(&gurnard_nand_data);
}

static struct spi_board_info gurnard_spi_board_info[] __initdata = {
	{
                /* FPGA SPI device for user-space programming */
		.modalias = "spidev",
		.max_speed_hz = 3000000,
		.bus_num = 0,
		.chip_select = 0,
	},
#if 0
        {
                .modalias = "lq043t3",
                .max_speed_hz = 10000000, /* 20 MHz max clock */
                .bus_num = 0,
                .chip_select = 1,
                .controller_data = AT91_PIN_PA9, /* CS pin */
        },
#endif
};

/* FPGA NAND access */
static struct resource fpga_nand_resources[] = {
	[0] = { /* Control registers */
		.start	= FPGA_NAND_CTRL_PHYS,
		.end	= (FPGA_NAND_CTRL_PHYS + 0x1000) - 1,
		.flags	= IORESOURCE_MEM,
	},
        [1] = { /* Data registers */
                .start  = FPGA_NAND_CTRL_PHYS + 0x00800000,
                /* Make it a large (ie: 32k) window, so we can do stm/rdm
                 * commands for better speed */
                .end    = FPGA_NAND_CTRL_PHYS + 0x00808000 - 1,
                .flags  = IORESOURCE_MEM,
        },
        [2] = { /* This is not actually used */
                .start  = BOARD_IRQ(0),
                .end    = BOARD_IRQ(0),
                .flags  = IORESOURCE_IRQ,
        },
};

static struct platform_device gurnard_nand_device = {
	.name		= "gurnard_nand",
	.resource	= fpga_nand_resources,
	.num_resources	= ARRAY_SIZE(fpga_nand_resources),
};

/*
 * LCD Controller
 */
static struct fb_videomode at91_tft_vga_modes[] = {
	{
		.name           = "Sharp LQ043T3DX0A",
		.refresh	= 50,
		.xres		= 480,
                .yres		= 272,
		.pixclock	= KHZ2PICOS(9000),

		.left_margin	= 0,		.right_margin	= 0,
		.upper_margin	= 4,		.lower_margin	= 2,
		.hsync_len	= 0,		.vsync_len	= 0,

		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
};

static struct fb_monspecs at91fb_default_monspecs = {
	.manufacturer	= "SH",
	.monitor        = "LQ043T3DX0A",

	.modedb		= at91_tft_vga_modes,
	.modedb_len	= ARRAY_SIZE(at91_tft_vga_modes),
	.hfmin		= 15000,
	.hfmax		= 17640,
	.vfmin		= 53,
	.vfmax		= 61,
};

#define AT91SAM9G45_DEFAULT_LCDCON2 	(ATMEL_LCDC_MEMOR_LITTLE \
					| ATMEL_LCDC_DISTYPE_TFT \
					| ATMEL_LCDC_CLKMOD_ALWAYSACTIVE)

/* Driver datas */
static struct atmel_lcdfb_info __initdata gurnard_lcdc_data = {
	.lcdcon_is_backlight		= true,
	.default_bpp			= 16,
	.default_dmacon			= ATMEL_LCDC_DMAEN,
	.default_lcdcon2		= AT91SAM9G45_DEFAULT_LCDCON2,
	.default_monspecs		= &at91fb_default_monspecs,
	.guard_time			= 9,
#if GURNARD_REVISION > 0
	.lcd_wiring_mode		= ATMEL_LCDC_WIRING_RGB,
#else
	.lcd_wiring_mode		= ATMEL_LCDC_WIRING_BGR565,
#endif
        .invert                         = 1,
};


static struct sam9_smc_config __initdata gurnard_fpga_smc_config = {
	.ncs_read_setup		= 2,
	.nrd_setup		= 0,
	.ncs_write_setup	= 2,
	.nwe_setup		= 1,

	.ncs_read_pulse		= 4,
	.nrd_pulse		= 6,
	.ncs_write_pulse	= 4,
	.nwe_pulse		= 5,

	.read_cycle		= 6,
	.write_cycle		= 6,

	.mode			= AT91_SMC_BAT_WRITE | AT91_SMC_DBW_32 |
                                  AT91_SMC_EXNWMODE_DISABLE | AT91_SMC_TDFMODE,
	.tdf_cycles		= 2,
};

/*
 * Touchscreen
 */
static struct at91_tsadcc_data gurnard_tsadcc_data = {
	.adc_clock		= 300000,
	.pendet_debounce	= 0x0d,
	.ts_sample_hold_time	= 0x0a,
};

/*
 * USB HS Host port (common to OHCI & EHCI)
 */
static struct at91_usbh_data __initdata gurnard_usbh_hs_data = {
	.ports		= 2,
	.vbus_pin	= {AT91_PIN_PB22, AT91_PIN_PB23},
};

/* MMC/SD */
static struct mci_platform_data __initdata gurnard_mmc_data = {
	.slot[0] = {
		.bus_width	= 4,
		.detect_pin	= AT91_PIN_PA6,
		.wp_pin		= AT91_PIN_PA7,
	},
};

/* Placeholder device for us to register sysfs files under */
static struct platform_device gurnard_fpga_device = {
	.name		= "gurnard_fpga",
};

/* FPGA Register display functions */
#define FPGA_REG_SHOW(r) \
static ssize_t r##_show(struct device *dev,                             \
                        struct device_attribute *attr, char *buf)       \
{                                                                       \
        return sprintf(buf, "0x%x\n", FPGA_##r);                        \
}                                                                       \
static DEVICE_ATTR(r, S_IRUGO, r##_show, NULL);

FPGA_REG_SHOW(IER);
FPGA_REG_SHOW(IFR);
FPGA_REG_SHOW(ID);
FPGA_REG_SHOW(EMULATION);
FPGA_REG_SHOW(BUILD_DATE);

static ssize_t fpga_irq_count_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%d\n", fpga_irq_count);
}
static DEVICE_ATTR(irq_count, S_IRUGO, fpga_irq_count_show, NULL);

static ssize_t fpga_build_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%s\n", (char *)&FPGA_BUILD_TEXT);
}
static DEVICE_ATTR(BUILD, S_IRUGO, fpga_build_show, NULL);

static ssize_t fpga_build_date_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        int year = (FPGA_BUILD_DATE & 0xffff0000) >> 16;
        int month = (FPGA_BUILD_DATE & 0xff00) >> 8;
        int day = (FPGA_BUILD_DATE & 0xff);
        return sprintf(buf, "%4.4d-%2.2d-%2.2d\n", year, month, day);
}
static DEVICE_ATTR(BUILD_DATE_STR, S_IRUGO, fpga_build_date_show, NULL);

static const struct attribute *fpga_attrs[] = {
        &dev_attr_IER.attr,
        &dev_attr_IFR.attr,
        &dev_attr_ID.attr,
        &dev_attr_EMULATION.attr,
        &dev_attr_BUILD.attr,
        &dev_attr_BUILD_DATE.attr,
        &dev_attr_BUILD_DATE_STR.attr,
        &dev_attr_irq_count.attr,
        NULL,
};

static const struct attribute_group fpga_attr_group = {
        .attrs = (struct attribute **)fpga_attrs,
};

static void __init gurnard_fpga_init(void)
{
        /* FPGA IRQ Needs to be in periph B mode */
        at91_set_B_periph(AT91_PIN_PD18, 1);
        /* FPGA platform dev, for us to attach SYSFS files to */
	platform_device_register(&gurnard_fpga_device);
        /* FPGA NAND */
	platform_device_register(&gurnard_nand_device);

        /* FPGA memory timings */
	sam9_smc_configure(0, &gurnard_fpga_smc_config);

        sysfs_create_group(&gurnard_fpga_device.dev.kobj,
                        &fpga_attr_group);
}

static void __init gurnard_board_init(void)
{
	/* Serial */
	at91_add_device_serial();
	/* Ethernet */
	at91_add_device_eth(&gurnard_macb_data);
	/* NAND */
	gurnard_add_device_nand();
        /* FPGA */
        gurnard_fpga_init();
	/* LEDs */
	at91_gpio_leds(gurnard_leds, ARRAY_SIZE(gurnard_leds));
        /* FPGA & LCD attached to the SPI bus */
	at91_add_device_spi(gurnard_spi_board_info,
		ARRAY_SIZE(gurnard_spi_board_info));

	/* LCD */
	at91_add_device_lcdc(&gurnard_lcdc_data);
	/* Backlight */
	at91_set_gpio_output(AT91_PIN_PE1, 1);
	at91_set_gpio_value(AT91_PIN_PE1, 1);
        /* Board detect ADC - now done via tsadcc driver */
        /* at91_add_device_adc(&gurnard_adc_data); */
	/* Touch Screen */
	at91_add_device_tsadcc(&gurnard_tsadcc_data);
	/* USB HS Host */
	at91_add_device_usbh_ohci(&gurnard_usbh_hs_data);
	at91_add_device_usbh_ehci(&gurnard_usbh_hs_data);
	/* SD */
        /* Enable to SD power supply */
	at91_set_gpio_output(AT91_PIN_PA8, 1);
	at91_set_gpio_value(AT91_PIN_PA8, 0);
	at91_add_device_mci(0, &gurnard_mmc_data);
}

MACHINE_START(GURNARD, "Gurnard")
	/* Maintainer: Bluewater Systems */
	.phys_io	= AT91_BASE_SYS,
	.io_pg_offst	= (AT91_VA_BASE_SYS >> 18) & 0xfffc,
	.boot_params	= AT91_SDRAM_BASE + 0x100,
	.timer		= &at91sam926x_timer,
	.map_io		= gurnard_map_io,
	.init_irq	= gurnard_init_irq,
	.init_machine	= gurnard_board_init,
MACHINE_END

