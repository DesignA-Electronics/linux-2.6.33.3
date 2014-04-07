#ifndef GURNARD_DEBUG_GPIOS_H
#define GURNARD_DEBUG_GPIOS_H

#define DBG_PIN_MO_IRQ		15 /* Set when we're inside an MO IRQ */
#define DBG_PIN_MO_FPGA_XFER	14 /* Set when we're bulk writing to FPGA */
#define DBG_PIN_MO_ERR		13 /* Pulsed whenever we have an MO fault */
#define DBG_PIN_FPGA_IRQ	12 /* Set when we're inside any FPGA IRQ */
#define DBG_PIN_NAND_IRQ	11 /* Set when we're inside a NAND stack IRQ */
#define DBG_PIN_NAND_BUSY	10 /* Set when we're waiting for the FPGA to drain */
#define DBG_PIN_NAND_RNB	 9 /* Set when we're waiting for RnB */
#define DBG_PIN_FPGA_IRQ_EN	 8
#define DBG_PIN_MO_WAITING       7 /* Set when we're waiting for a userspace
                                    * response
				    */
#define DBG_PIN_MO_STORAGE_READ	 6 /* Set when we're reading from the MO
				    * backing store (ie: from NAND flash)
				    */
#define DBG_PIN_MO_STORAGE_WRITE 5 /* Set when we're writing to the MO
				    * backing store (ie: from NAND flash)
				    */
#define DBG_PIN_MO_QUEUE_BLOCK 4 /* Set when user space is blocked on the
				    command queue */
#define DBG_PIN_MO_QUEUE_EMPTY 3 /* Set when user space is blocked on the
				    command queue */
#define DBG_PIN_MO_USER_SPACE  2 /* Set when user space is handling something
                                    from the SCSI queue */
#define DBG_PIN_TEST           0 /* For temporary testing */

#if /*defined(KERNEL) && */ defined(CONFIG_ARCH_AT91)
#include <mach/gpio.h>
#include <linux/delay.h>

/* CPU Debug GPIOs for assistance with logic analyser debugging */
/* Reordered from schematics to make mictor pinout linear */
static const int debug_gpios[16] = {
    AT91_PIN_PD11, AT91_PIN_PD12, AT91_PIN_PD13, AT91_PIN_PD14,
    AT91_PIN_PD15, AT91_PIN_PB28, AT91_PIN_PB29, AT91_PIN_PD27,
    AT91_PIN_PD26, AT91_PIN_PD19, AT91_PIN_PD25, AT91_PIN_PD8,
    AT91_PIN_PD28, AT91_PIN_PD29, AT91_PIN_PD30, AT91_PIN_PD31};

static inline void gpio_debug_set(int index, int value)
{
    at91_set_gpio_value(debug_gpios[index], value ? 1 : 0);
}

static inline void gpio_debug_pulse(int index)
{
    gpio_debug_set(index, 1);
    udelay(1);
    gpio_debug_set(index, 0);
}

static inline void gpio_debug_init(void)
{
    int i;
    for (i = 0; i < ARRAY_SIZE(debug_gpios); i++) {
        at91_set_GPIO_periph(debug_gpios[i], 0);
        at91_set_gpio_output(debug_gpios[i], 0);
    }
}

static inline void gpio_debug_walk(void)
{
    int i;
    for (i = 0; i < ARRAY_SIZE(debug_gpios); i++)
        gpio_debug_pulse(i);
}
#else
static inline void gpio_debug_set(int index, int value) {}
static inline void gpio_debug_pulse(int index) {}
static inline void gpio_debug_init(void) {}
static inline void gpio_debug_walk(void) {}
#endif

#endif /* GPIO_DEBUG_H */

