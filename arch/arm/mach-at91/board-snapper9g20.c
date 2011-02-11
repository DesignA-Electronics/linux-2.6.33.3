/*
 * linux/arch/arm/mach-at91/board-snapper9g20.c
 *
 *  Copyright (C) 2010 Bluewater Systems Ltd
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

#include <linux/init.h>
#include <linux/list.h>
#include <linux/platform_device.h>

#include <mach/at91sam9_smc.h>
#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/gpio.h>

#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/setup.h>
#include <asm/irq.h>

#include "sam9_smc.h"
#include "generic.h"

static void __init sn9260_map_io(void)
{
	/* Initialize processor: 18.432 MHz crystal */
	at91sam9260_initialize(18432000);

	/* Debug on ttyS0 */
	at91_register_uart(0, 0, 0);
	at91_set_serial_console(0);

	/* UARTs ttyS1 and ttyS2 */
	at91_register_uart(AT91SAM9260_ID_US0, 1,
			   ATMEL_UART_CTS | ATMEL_UART_RTS);
	at91_register_uart(AT91SAM9260_ID_US1, 2,
			   ATMEL_UART_CTS | ATMEL_UART_RTS);
	at91_register_uart(AT91SAM9260_ID_US2, 3, 0);
}

static void __init sn9260_init_irq(void)
{
	at91sam9260_init_interrupts(NULL);
}

static struct at91_usbh_data __initdata sn9260_usbh_data = {
	.ports		= 2,
};

static struct at91_udc_data __initdata sn9260_udc_data = {
	.vbus_pin	= AT91_PIN_PC5,		/* FIXME - On IO Expander */
	.pullup_pin	= 0,		/* pull-up driven by UDC */
};

static struct __initdata at91_eth_data sn9260_macb_data = {
	.is_rmii	= 1,
};

/*
 * The partition sizes have changed to support the NAND part fitted to the
 * Snapper9G20. The revision 1 Snapper9260 has 128K erase blocks where the
 * revision 2 Sanpper9G20 has 256K erase blocks.
 *
 * The old values have been commented out for comparision.
 */
static struct mtd_partition __initdata sn9260_nand_partition[] = {
	{
		.name	= "Preboot",
		.offset	= 0,
		/*.size	= SZ_128K,*/
		.size	= SZ_256K,
	},
	{
		.name	= "Bootloader",
		.offset	= MTDPART_OFS_APPEND,
		.size	= SZ_256K,
	},
	{
		.name	= "Environment",
		.offset	= MTDPART_OFS_APPEND,
		/*.size	= SZ_128K,*/
		.size	= SZ_256K,
	},
	{
		.name	= "Kernel",
		.offset	= MTDPART_OFS_APPEND,
		.size	= SZ_4M,
	},
	{
		.name	= "Filesystem",
		.offset	= MTDPART_OFS_APPEND,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct mtd_partition * __init nand_partitions(int size,
						     int *num_partitions)
{
	*num_partitions = ARRAY_SIZE(sn9260_nand_partition);
	return sn9260_nand_partition;
}

static struct atmel_nand_data __initdata sn9260_nand_data = {
	.ale		= 21,
	.cle		= 22,
	.rdy_pin	= AT91_PIN_PC13,
	.partition_info	= nand_partitions,
	.bus_width_16	= 0,
};

static struct sam9_smc_config __initdata sn9260_nand_smc_config = {
	.ncs_read_setup		= 0,
	.nrd_setup		= 2,
	.ncs_write_setup	= 0,
	.nwe_setup		= 2,

	.ncs_read_pulse		= 5,
	.nrd_pulse		= 4,
	.ncs_write_pulse	= 5,
	.nwe_pulse		= 4,

	.read_cycle		= 7,
	.write_cycle		= 7,

	.mode			= (AT91_SMC_READMODE | AT91_SMC_WRITEMODE |
				   AT91_SMC_EXNWMODE_DISABLE | AT91_SMC_DBW_8),
	.tdf_cycles		= 3,
};

static struct i2c_board_info __initdata sn9260_i2c_devices[] = {
	{
		/* IO Expander */
		I2C_BOARD_INFO("max7311", 0x28),
	},
	{
		/* Audio codec */
		I2C_BOARD_INFO("tlv320aic23", 0x1a),
	},
	{
		/* ISL1208 RTC */
		I2C_BOARD_INFO("isl1208", 0x6f),
		.irq = gpio_to_irq(AT91_PIN_PA31),
	},
};

static void __init sn9260_add_device_nand(void)
{
	at91_set_A_periph(AT91_PIN_PC14, 0);
	sam9_smc_configure(3, &sn9260_nand_smc_config);
	at91_add_device_nand(&sn9260_nand_data);
}

#if 0
static struct at91_adc_data sn9260_adc_data = {
        .gpios[0] = -1,
        .gpios[1] = -1,
        .gpios[2] = AT91_PIN_PC2,
        .gpios[3] = AT91_PIN_PC3,
        .prescale = 4,
        .startup = 12,
        .sample = 12,
};
#endif

#if defined(CONFIG_TOUCHSCREEN_SN9260)
static struct platform_device sn9260_ts_device = {
	.name		= "sn9260_ts",
	.id		= 0,
	.num_resources	= 0,
};
#endif

static void __init sn9260_board_init(void)
{
        at91_add_device_i2c(sn9260_i2c_devices, ARRAY_SIZE(sn9260_i2c_devices));
	//at91_add_device_adc(&sn9260_adc_data);
	at91_add_device_serial();
	at91_add_device_usbh(&sn9260_usbh_data);
	at91_add_device_udc(&sn9260_udc_data);
	at91_add_device_eth(&sn9260_macb_data);
	at91_add_device_ssc(AT91SAM9260_ID_SSC, (ATMEL_SSC_TF | ATMEL_SSC_TK |
						 ATMEL_SSC_TD | ATMEL_SSC_RD));
	sn9260_add_device_nand();

#if defined(CONFIG_TOUCHSCREEN_SN9260)
	platform_device_register(&sn9260_ts_device);
#endif
}

MACHINE_START(SNAPPER_9260, "Bluewater Systems Snapper 9260/9G20")
	.phys_io	= AT91_BASE_SYS,
	.io_pg_offst	= (AT91_VA_BASE_SYS >> 18) & 0xfffc,
	.boot_params	= AT91_SDRAM_BASE + 0x100,
	.timer		= &at91sam926x_timer,
	.map_io		= sn9260_map_io,
	.init_irq	= sn9260_init_irq,
	.init_machine	= sn9260_board_init,
MACHINE_END

