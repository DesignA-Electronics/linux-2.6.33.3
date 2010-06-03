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

static void __init tiny_gurnard_map_io(void)
{
	/* Initialize processor: 12.000 MHz crystal */
	at91sam9g45_initialize(12000000);

	/* DGBU on ttyS0. (Rx & Tx only) */
	at91_register_uart(0, 0, 0);

        /* No other UARTS on tiny gurnard */

	/* set serial console to ttyS0 (ie, DBGU) */
	at91_set_serial_console(0);
}

static void __init tiny_gurnard_init_irq(void)
{
	at91sam9g45_init_interrupts(NULL);
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

static void __init tiny_gurnard_board_init(void)
{
	/* Serial */
	at91_add_device_serial();
	/* Ethernet */
	at91_add_device_eth(&tiny_gurnard_macb_data);
	/* NAND */
	tiny_gurnard_add_device_nand();
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
