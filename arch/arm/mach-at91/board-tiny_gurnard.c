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
#include <linux/debugfs.h>

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

#define FPGA_IRQ        AT91_PIN_PC11

/* Take from 7.3 of document tiny_gurnard 06 */
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
#define FPGA_BUILD_TEXT FPGA_ROM_REG(0x400)

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

static void tiny_gurnard_irq_handler(unsigned int irq, struct irq_desc *desc)
{
        uint32_t pending = FPGA_IER;

        /* If the FPGA is not programmed, then skip it */
        if ((FPGA_ID & 0xffff0000) != 0x000d0000) {
                pr_err("Tiny Gurnard FPGA IRQ, but FPGA not programmed\n");
                return;
        }

        do {
                if (likely(pending)) {
                        irq = BOARD_IRQ(0) + __ffs(pending);
                        generic_handle_irq(irq);
                }
                pending = FPGA_IER;
        } while (pending);
}

static void tiny_gurnard_mask_irq(unsigned int irq)
{
        int fpga_irq = (irq - BOARD_IRQ(0));
        FPGA_IER |= 1 << fpga_irq;
}

static void tiny_gurnard_unmask_irq(unsigned int irq)
{
        int fpga_irq = (irq - BOARD_IRQ(0));
        FPGA_IER &= ~(1 << fpga_irq);
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

        set_irq_type(FPGA_IRQ, IRQ_TYPE_EDGE_BOTH);
        set_irq_chained_handler(FPGA_IRQ, tiny_gurnard_irq_handler);
}

/*
 * MACB Ethernet device
 */
static struct at91_eth_data __initdata tiny_gurnard_macb_data = {
	.is_rmii	= 1,
        .phy_mask       = ~(1 << 31), // Phy 0x1f
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
		.name	= "Bootloader_Environment",
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

static struct mtd_partition * __init nand_partitions(int size, int *num_partitions)
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

	.mode			= AT91_SMC_READMODE | AT91_SMC_WRITEMODE | AT91_SMC_EXNWMODE_DISABLE,
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
	[0] = {
		.start	= FPGA_NAND_CTRL_PHYS,
		.end	= (FPGA_NAND_CTRL_PHYS + 0x1000) - 1,
		.flags	= IORESOURCE_MEM,
	},
        [1] = {
                .start  = FPGA_NAND_CTRL_PHYS + 0x00800000,
                /* Make it a large (ie: 4k) window, so we can do stm/rdm
                 * commands for better speed */
                .end    = FPGA_NAND_CTRL_PHYS + 0x00801000 - 1,
                .flags  = IORESOURCE_MEM,
        },
        [2] = {
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

static void __init tiny_gurnard_board_init(void)
{
#ifdef CONFIG_DEBUG_FS
        struct dentry *dir;
#endif
	/* Serial */
	at91_add_device_serial();
	/* Ethernet */
	at91_add_device_eth(&tiny_gurnard_macb_data);
	/* NAND */
	tiny_gurnard_add_device_nand();
        /* FPGA attached to the SPI bus */
	at91_add_device_spi(tiny_gurnard_spi_board_info,
		ARRAY_SIZE(tiny_gurnard_spi_board_info));
        /* FPGA NAND */
	platform_device_register(&tiny_gurnard_nand_device);

#ifdef CONFIG_DEBUG_FS
        dir = debugfs_create_dir("fpga", NULL);
        if (!dir)
                return;
        debugfs_create_x32("IER", S_IRUGO | S_IWUSR, dir, (uint32_t *)&FPGA_IER);
        debugfs_create_x32("IFR", S_IRUGO | S_IWUSR, dir, (uint32_t *)&FPGA_IFR);
        debugfs_create_x32("ID", S_IRUGO, dir, (uint32_t *)&FPGA_ID);
        debugfs_create_x32("BUILD_DATE", S_IRUGO, dir, (uint32_t *)&FPGA_BUILD_DATE);
        debugfs_create_x32("EMULATION", S_IRUGO, dir, (uint32_t *)&FPGA_EMULATION);
#endif
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
