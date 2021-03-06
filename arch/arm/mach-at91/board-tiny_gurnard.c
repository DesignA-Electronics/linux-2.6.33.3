/*
 *  Board-specific setup code for the Bluewater Systems Tiny Gurnard board
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
#include <linux/clk.h>

#include <mach/hardware.h>

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

#define TINY_GURNARD_REV	1
//#define TINY_GURNARD_REV	0

/**
 * In rev0 this was on PC11
 */
#if TINY_GURNARD_REV == 0
#define FPGA_IRQ        AT91_PIN_PC11
#define FPGA_IRQ_LEVEL IRQ_TYPE_EDGE_BOTH
#endif
/**
 * In rev1 we are on PD18, which in periph B mode is the external IRQ
 */
#if TINY_GURNARD_REV == 1
#define FPGA_IRQ        AT91SAM9G45_ID_IRQ0
#define FPGA_IRQ_LEVEL IRQ_TYPE_LEVEL_LOW
#endif

/* Taken from 7.3 of document tiny_gurnard 06 */
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

static void __init tiny_gurnard_map_io(void)
{
	/* Initialize processor: 12.000 MHz crystal */
	at91sam9g45_initialize(12000000);

	/* DGBU on ttyS0. (Rx & Tx only) */
	at91_register_uart(0, 0, 0);

        /* No other UARTS on tiny gurnard */

	/* set serial console to ttyS0 (ie, DBGU) */
	at91_set_serial_console(0);

        /* Map the FPGA base area */
        iotable_init(fpga_io_desc, ARRAY_SIZE(fpga_io_desc));
}

/**
 * FPGA interrupt counter
 */
static uint32_t fpga_irq_count = 0;

static void tiny_gurnard_irq_handler(unsigned int irq, struct irq_desc *desc)
{
        uint32_t pending = FPGA_IFR;

        /* If the FPGA is not programmed, then skip it */
        if ((FPGA_ID & 0xffff0000) != 0x000d0000) {
                pr_err("Tiny Gurnard FPGA IRQ, but FPGA not programmed\n");
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

static void tiny_gurnard_mask_irq(unsigned int irq)
{
        int fpga_irq = (irq - BOARD_IRQ(0));
        FPGA_IER &= ~(1 << fpga_irq);
}

static void tiny_gurnard_unmask_irq(unsigned int irq)
{
        int fpga_irq = (irq - BOARD_IRQ(0));
        FPGA_IER |= 1 << fpga_irq;
}

static struct irq_chip tiny_gurnard_irq_chip = {
        .name           = "FPGA",
        .ack            = tiny_gurnard_mask_irq,
        .mask           = tiny_gurnard_mask_irq,
        .unmask         = tiny_gurnard_unmask_irq,
};

static void __init tiny_gurnard_init_irq(void)
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
                set_irq_chip(irq, &tiny_gurnard_irq_chip);
                set_irq_handler(irq, handle_level_irq);
                set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
        }

        set_irq_type(FPGA_IRQ, FPGA_IRQ_LEVEL);
        set_irq_chained_handler(FPGA_IRQ, tiny_gurnard_irq_handler);
}

/*
 * MACB Ethernet device
 */
static struct at91_eth_data __initdata tiny_gurnard_macb_data = {
	.is_rmii	= 1,
        .phy_mask       = ~(1 << 31), // Phy 0x1f
};

/**
 * Board detection ADC
 */
static struct at91_adc_data tiny_gurnard_adc_data = {
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

/* LEDS */
static struct gpio_led tiny_gurnard_leds[] = {
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
static struct mtd_partition __initdata tiny_gurnard_nand_partition[] = {
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
	*num_partitions = ARRAY_SIZE(tiny_gurnard_nand_partition);
	return tiny_gurnard_nand_partition;
}

/* det_pin is not connected */
static struct atmel_nand_data __initdata tiny_gurnard_nand_data = {
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

static struct sam9_smc_config __initdata tiny_gurnard_nand_smc_config = {
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

static void __init tiny_gurnard_add_device_nand(void)
{
	/* setup bus-width (8 or 16) */
	if (tiny_gurnard_nand_data.bus_width_16)
		tiny_gurnard_nand_smc_config.mode |= AT91_SMC_DBW_16;
	else
		tiny_gurnard_nand_smc_config.mode |= AT91_SMC_DBW_8;

	/* configure chip-select 3 (NAND) */
	sam9_smc_configure(3, &tiny_gurnard_nand_smc_config);

	at91_add_device_nand(&tiny_gurnard_nand_data);
}

/* FPGA SPI device for user-space programming */
static struct spi_board_info tiny_gurnard_spi_board_info[] __initdata = {
	{
		.modalias = "spidev",
		.max_speed_hz = 3000000,
		.bus_num = 0,
		.chip_select = 0,
	},
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
                /* Make it a large (ie: 4k) window, so we can do stm/rdm
                 * commands for better speed */
                .end    = FPGA_NAND_CTRL_PHYS + 0x00801000 - 1,
                .flags  = IORESOURCE_MEM,
        },
        [2] = { /* This is not actually used */
                .start  = BOARD_IRQ(0),
                .end    = BOARD_IRQ(0),
                .flags  = IORESOURCE_IRQ,
        },
};

static struct platform_device tiny_gurnard_nand_device = {
	.name		= "gurnard_nand",
	.resource	= fpga_nand_resources,
	.num_resources	= ARRAY_SIZE(fpga_nand_resources),
};

static struct sam9_smc_config __initdata tiny_gurnard_fpga_smc_config = {
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

/* Placeholder device for us to register sysfs files under */
static struct platform_device tiny_gurnard_fpga_device = {
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

static ssize_t board_revision(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%d\n", TINY_GURNARD_REV);
}

static DEVICE_ATTR(board_revision, S_IRUGO, board_revision, NULL);

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
        &dev_attr_board_revision.attr,
        NULL,
};

static const struct attribute_group fpga_attr_group = {
        .attrs = (struct attribute **)fpga_attrs,
};

static void __init tiny_gurnard_fpga_init(void)
{
        /* FPGA IRQ Needs to be in periph B mode */
        at91_set_B_periph(AT91_PIN_PD18, 1);
        /* FPGA attached to the SPI bus */
	at91_add_device_spi(tiny_gurnard_spi_board_info,
		ARRAY_SIZE(tiny_gurnard_spi_board_info));

        /* FPGA platform dev, for us to attach SYSFS files to */
	platform_device_register(&tiny_gurnard_fpga_device);
        /* FPGA NAND */
	platform_device_register(&tiny_gurnard_nand_device);

        /* FPGA memory timings */
	sam9_smc_configure(0, &tiny_gurnard_fpga_smc_config);

        /* Board detect ADC */
        at91_add_device_adc(&tiny_gurnard_adc_data);

        sysfs_create_group(&tiny_gurnard_fpga_device.dev.kobj,
                        &fpga_attr_group);
}

static void __init tiny_gurnard_board_init(void)
{
	/* Serial */
	at91_add_device_serial();
	/* Ethernet */
	at91_add_device_eth(&tiny_gurnard_macb_data);
	/* NAND */
	tiny_gurnard_add_device_nand();
        /* FPGA */
        tiny_gurnard_fpga_init();
	/* LEDs */
	at91_gpio_leds(tiny_gurnard_leds, ARRAY_SIZE(tiny_gurnard_leds));
}

MACHINE_START(TINY_GURNARD, "Tiny Gurnard")
	/* Maintainer: Atmel */
	.phys_io	= AT91_BASE_SYS,
	.io_pg_offst	= (AT91_VA_BASE_SYS >> 18) & 0xfffc,
	.boot_params	= AT91_SDRAM_BASE + 0x100,
	.timer		= &at91sam926x_timer,
	.map_io		= tiny_gurnard_map_io,
	.init_irq	= tiny_gurnard_init_irq,
	.init_machine	= tiny_gurnard_board_init,
MACHINE_END
