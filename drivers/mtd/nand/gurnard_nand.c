/*
 *  Copyright (C) 2010 Andre Renaud
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

/***
 * Note:
 * Although this is under the NAND subheading, it does not use any of the
 * existing nand subsystem, but reimplements it all on top of MTD.
 * This is because we are re-implementing all of the fundamental
 * commands to cope with the 4-way multiplexed NAND bus that we have
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/delay.h>
#include <linux/mutex.h>

#include <asm/io.h>

#include <mach/gurnard_debug_gpios.h>

/* Over how many bytes does the FPGA calculate its ECC values? */
#define ECC_CHUNK_SIZE 256
/* How many bytes of ECC do we get for each chunk? */
#define ECC_CALC_SIZE 3

#define MAX_STACKS      2
#define MAX_OOBSIZE 512
#define MAX_WRITESIZE 8192
#define RAID_ADDR 3 /* Address to access in the FPGA to get both devices */

/* Display debugging each time we do a low-level NAND access */
// #define RAW_ACCESS_DEBUG

/* Check that the Linux ECC calculations match the FPGA ones */
// #define VERIFY_FPGA_ECC_CALC

/* Actually use the FPGA ECC results */
#define USE_FPGA_ECC

/* If set, let the FPGA compare the buffers after a readback. Saves
 * us transfering the data into the CPU memory */
#define USE_FPGA_READ_VERIFY

/* If set, intelligently cache the BBT information from the OOB. This
 * results in fewer reads of the OOB, which should speed up YAFFS */
#define USE_BBT_CACHE

/* If set, the FPGA supports timing changes, and so we can change
 * the ONFI timing mode
 */
#define USE_TIMING_MODE

/* If set, then force the NAND devices into mode 0, rather than
 * auto-probing the chips.
 * Only makes a difference if 'USE_TIMING_MODE' is defined
 */
//#define FORCE_SLOW_NAND


#ifdef USE_BBT_CACHE
/* Bit settings in the bbt cache */
enum {
	BLOCK_UNKNOWN = 0,
	BLOCK_GOOD    = 1,
	BLOCK_BAD     = 2,
	BLOCK_MASK    = 3,
};
#endif

/* The first 4-bytes (32-bit word) of the ecc is the bad-block marker
 * for each of the 4 8-bit devices. We then have 192 bytes of ECC,
 * followed by 316 bytes of available oob space
 */
static struct nand_ecclayout gurnard_ecc_layout_4k = {
        /* There are 3 bytes of ecc for each 256 bytes of data,
         * and 4 * 4k = 16k of data. 16k / 256 * 3 = 192
         **/
        .eccbytes = 192,
        /* Note: These must be contiguous, as gurnard_ecc_check
         * uses a simple memcmp to get the info out
         */
        .eccpos = { 4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15,
                   16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27,
                   28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
                   40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51,
                   52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63,
                   64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75,
                   76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87,
                   88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99,
                   100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111,
                   112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123,
                   124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135,
                   136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147,
                   148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159,
                   160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171,
                   172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183,
                   184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195,
        },
        .oobfree = {{.offset = 196, .length = 316}},
        .oobavail = 316,
};

/* The first 4-bytes (32-bit word) of the ecc is the bad-block marker
 * for each of the 4 8-bit devices. We then have 96 bytes of ECC,
 * followed by 156 bytes of available oob space
 */
static struct nand_ecclayout gurnard_ecc_layout_2k = {
        /* There are 3 bytes of ecc for each 256 bytes of data,
         * and 4 * 2k = 8k of data. 8k / 256 * 3 = 96
         **/
        .eccbytes = 96,
        /* Note: These must be contiguous, as gurnard_ecc_check
         * uses a simple memcmp to get the info out
         */
        .eccpos = { 4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15,
                   16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27,
                   28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
                   40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51,
                   52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63,
                   64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75,
                   76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87,
                   88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99,
        },
        .oobfree = {{.offset = 100, .length = 156}},
        .oobavail = 156,
};

enum {
        REG_CFG_SET     = 0x000,
        REG_CFG_CLR     = 0x004,
        REG_BYPASS      = 0x008,
	REG_IFR		= 0x00c,
#ifdef USE_TIMING_MODE
	REG_RD_TIMER	= 0x010,
	REG_WR_TIMER	= 0x014,
#endif
        REG_VERIFY_FAIL = 0x01c,
        REG_BUFF_LEN    = 0x200,
        REG_ECC         = 0x400,

        BIT_ALE         = 1 << 0,
        BIT_CLE         = 1 << 1,
        NCE_SHIFT       = 2,
        MASK_NCE        = 3 << NCE_SHIFT,
	BIT_BUSY_IER	= 1 << 4,
        BIT_BUFF_CLEAR  = 1 << 8,
        BIT_VERIFY      = 1 << 9,
	BIT_RNB_IER	= 1 << 15,
	BIT_BUFF_EMPTY	= 1 << 16,
        BIT_RNB         = 1 << 17,
        BIT_BUSY        = 1 << 18,
        BIT_VERIFY_OK   = 1 << 19,

	IFR_RNB		= 1 << 0,
	IFR_BUSY	= 1 << 1,
};

#define STACK_ADDR(i) (1 << (i))

#ifdef USE_TIMING_MODE
struct mode_timing_info {
	uint32_t wr_timer;
	uint32_t rd_timer;
};

/* See Section 7.6.1 of the Gurnard specification
 * These are essentially bitmasks for a 7.5ns clock, telling the FPGA
 * how to clock in/out data from the NAND devices
 */
struct mode_timing_info mode_timing[] = {
	[0] = {0x200001fe, 0x20000fe0},
	[1] = {0x0040001e, 0x0040000f},
	[2] = {0x00100007, 0x00400007},
	[3] = {0x00080006, 0x00200003},
	[4] = {0x00080006, 0x00200003},
	[5] = {0x00040002, 0x00200003},
};
#endif

#define BUS_EXPAND(a) ((a) << 24 | (a) << 16 | (a) << 8 | (a))

#define reg_writel(v,r) writel(v, host->reg_base + REG_##r)
#define reg_readl(r) readl(host->reg_base + REG_##r)

struct gurnard_nand_host;

struct gurnard_nand_stack {
        struct gurnard_nand_host *host;
        int device_addr;
        char name[20];
        struct mtd_info mtd;
        uint8_t width; /* How many chips wide is it (1-4) */
	int page_shift; /* What is the shift to get the page size? */
#ifdef USE_BBT_CACHE
	uint8_t *bbt; /* Array of two-bit values indicating whether a block
			 is bad, good, or unknown */
#endif
};

struct gurnard_nand_host {
        struct platform_device *pdev;
        void __iomem    *reg_base;
        void __iomem    *data_base;
        struct gurnard_nand_stack stack[MAX_STACKS];
        uint8_t         *oob_tmp;       /* Temporary area for oob data */
        uint8_t         *data_tmp;      /* Temporary area for buffer data */

        struct gurnard_nand_stack raid_stack;       /* Combined RAID stack */

	struct completion rnb_completion;
	struct completion busy_completion;

	int		irq;
	int		has_irq;
};

/**
 * Low level NAND access functions
 */
static inline void gurnard_select_chips(struct gurnard_nand_host *host,
		unsigned int chip_mask)
{
	/* Chips should never be busy when we're toggling the nCE lines */
	if (!(reg_readl(CFG_SET) & BIT_RNB))
		dev_warn(&host->pdev->dev,
				"Selecting chips %d, but devices not ready\n",
				chip_mask);

	/* If we're selecting a chip, then there should be nothing in the
	 * write fifos
	 */
	if (chip_mask && reg_readl(BUFF_LEN))
		dev_warn(&host->pdev->dev,
				"Selecting chips %d, but fpga fifo is at %d\n",
				chip_mask, reg_readl(BUFF_LEN));
        /* FIXME: Should be taking the page address as an argument, and
         * selecting the appropriate level in the stack via the STACK ADDR
         * bits in the FPGA CFG_SET/CFG_CLR registers
         */
       reg_writel(MASK_NCE | BIT_BUFF_CLEAR, CFG_SET);
       if (chip_mask)
               reg_writel((chip_mask & 0x3) << NCE_SHIFT, CFG_CLR);
}

/**
 * This chould be done with a completion from the IRQ handler.
 * Given that the NAND interface is faster than the FPGA->CPU interface,
 * we are actually better off to use a busy wait at this stage,
 * as it's unlikely to be in here long
 */
static inline void gurnard_busy_wait(struct gurnard_nand_host *host)
{
	unsigned long timeo = jiffies + HZ;
	gpio_debug_set(DBG_PIN_NAND_BUSY, 1);
	do {
		if (!(reg_readl(CFG_SET) & BIT_BUSY)) {
			gpio_debug_set(DBG_PIN_NAND_BUSY, 0);
			return;
		}
	} while (time_before(jiffies, timeo));
	dev_err(&host->pdev->dev,
		"Timeout waiting for busy signal\n");
	dump_stack();
	gpio_debug_set(DBG_PIN_NAND_BUSY, 0);
}

/**
 * FIXME: This should be done with a completion from the IRQ handler
 */
static inline void gurnard_rnb_wait(struct gurnard_nand_host *host)
{
#if 0
	int timeout;
	gpio_debug_set(DBG_PIN_NAND_RNB, 1);
	timeout = wait_for_completion_timeout(&host->rnb_completion,
			msecs_to_jiffies(10000));
	if (!timeout) {
		dev_err(&host->pdev->dev,
				"Timeout waiting for RnB signal\n");
		dump_stack();
	}
#else
	unsigned long timeo = jiffies + HZ;
	/* Could take as long as 200ns before RnB changes (tWB) */
	gpio_debug_set(DBG_PIN_NAND_RNB, 1);
	reg_readl(CFG_SET);
	reg_readl(CFG_SET);
	reg_readl(CFG_SET);
	do {
		if (reg_readl(CFG_SET) & BIT_RNB) {
			gpio_debug_set(DBG_PIN_NAND_RNB, 0);
			return;
		}
	} while (time_before(jiffies, timeo));
	dev_err(&host->pdev->dev,
		"Timeout waiting for RnB signal\n");
	dump_stack();
#endif
	gpio_debug_set(DBG_PIN_NAND_RNB, 0);
}

static inline void gurnard_write_command(struct gurnard_nand_host *host,
					 uint8_t command)
{
        reg_writel(BIT_CLE, CFG_SET);
        reg_writel(BUS_EXPAND(command), BYPASS);
        gurnard_busy_wait(host);
        reg_writel(BIT_CLE, CFG_CLR);
}

static inline void gurnard_write_address(struct gurnard_nand_host *host,
		uint8_t *address, int address_len)
{
        reg_writel(BIT_ALE, CFG_SET);
        while (address_len--) {
                uint32_t a = *address++;
                reg_writel(BUS_EXPAND(a), BYPASS);
                gurnard_busy_wait(host);
        }
        reg_writel(BIT_ALE, CFG_CLR);
}

static inline void gurnard_raw_read_buf(struct gurnard_nand_host *host,
					uint32_t *buf, uint32_t buflen,
					uint32_t offset)
{
#if 1
        uint32_t read = 0;
	unsigned long timeo = jiffies + HZ;
        /* Note: BUFF_LEN doesn't reduce as we read from it, it is the
         * count of bytes read since it was last reset
         */
        do {
                uint32_t total_read;
		uint32_t avail;

		total_read = reg_readl(BUFF_LEN);
		if (total_read < offset)
			BUG();
		total_read -= offset;

		total_read = min(total_read, buflen);
		avail = (total_read - read) >> 2;
                if (avail) {
                        readsl(host->data_base, buf, avail);
                        read += avail << 2;
                        buf += avail;
		}

		if (!time_before(jiffies, timeo)) {
			dev_err(&host->pdev->dev,
				"Timeout waiting for read response to %d bytes (offset=%d)\n",
				buflen, offset);
			return;
		}
        } while (read != buflen);
#else
#warning "Using non-optimal read mechanism"
        gurnard_busy_wait(host);
        readsl(host->data_base, buf, buflen >> 2);
#endif
}

static inline void gurnard_read_buf(struct gurnard_nand_host *host,
				    uint32_t *buf, uint32_t buflen)
{
	if (buflen & 0x3)
		BUG();
#if 0
        /* Make sure that we're not accidentally reading from both
         * devices at once. This will cause bus contention
         */
        if ((reg_readl(CFG_SET) & MASK_NCE) == 0) {
                dev_err(&host->pdev->dev,
                        "Attempt to read from NAND chips with both selected\n");
                dump_stack();
        }
#endif
        reg_writel(buflen - 1, BUFF_LEN);
	gurnard_raw_read_buf(host, buf, buflen, 0);
	if (reg_readl(BUFF_LEN) != buflen) {
		dev_err(&host->pdev->dev,
				"Read buffer, but lengths don't line up: %d vs %d\n",
				reg_readl(BUFF_LEN), buflen);
		//BUG();
	}
}

static inline void gurnard_write_buf(struct gurnard_nand_host *host,
				     const uint32_t *buf, uint32_t buflen)
{
        writesl(host->data_base, buf, buflen >> 2);
        gurnard_busy_wait(host);
}

static inline int n_way_match(uint32_t v, int n)
{
        uint8_t val = v & 0xff;
        int i;

        for (i = 1; i < n; i++)
                if (((v >> (i * 8)) & 0xff) != val)
                        return 0;
        return 1;
#if 0
        return ((((v & 0xff000000) >> 24) == (v & 0xff)) &&
                (((v & 0x00ff0000) >> 16) == (v & 0xff)) &&
                (((v & 0x0000ff00) >>  8) == (v & 0xff)));
#endif
}

/* Converts the 64-bit byte offset within the nand device to the
 * 5-byte page address
 */
static inline void offset_to_page(struct gurnard_nand_stack *stack,
		uint64_t from, uint8_t *addr)
{
	from >>= stack->page_shift;
        addr[0] = 0; /* We never want to address individual bytes */
        addr[1] = 0;
        addr[2] = from;
        addr[3] = from >> 8;
        addr[4] = from >> 16;
}

/**
 * Jump a 5-byte address to the OOB region
 */
static inline void addr_to_oob(struct gurnard_nand_stack *stack,
		uint8_t *addr)
{
        addr[1] = 1 << (stack->page_shift - 8 - 2);
}

/* Converts the 64-bit byte offset within the nand device to the
 * 3-byte block address
 */
static inline void offset_to_block(struct gurnard_nand_stack *stack,
		uint64_t from, uint8_t *addr)
{
	from >>= stack->page_shift;
        addr[0] = from;
        addr[1] = from >> 8;
        addr[2] = from >> 16;
}

#ifdef USE_BBT_CACHE
static inline uint32_t offset_to_block_id(struct gurnard_nand_stack *stack,
		uint64_t block)
{
	/*
	 * 6 because 2^6 = 64 pages/block
	 */
	return block >> (stack->page_shift + 6);
}
#endif

/**
 * High level NAND command functions
 */
static void gurnard_reset(struct gurnard_nand_host *host, uint32_t device_addr)
{
	/* Make sure all of the fifos from the FPGA are drained */
        gurnard_select_chips(host, device_addr);
        gurnard_write_command(host, NAND_CMD_RESET);
        gurnard_rnb_wait(host);
        gurnard_select_chips(host, 0);
}

static int gurnard_read_onfi(struct gurnard_nand_host *host, uint32_t device_addr,
                char *buf)
{
        uint8_t addr = 0;
        int i;
        uint32_t *id;
	struct gurnard_nand_stack *stack;

        if (device_addr != 1 && device_addr != 2) {
                dev_err(&host->pdev->dev,
                        "Cannot read ONFI from anything other than device 1 or 2\n");
                return -ENODEV;
        }
	stack = &host->stack[device_addr - 1];
	id = vmalloc(256 * 4);
	if (!id) {
		dev_err(&host->pdev->dev,
			"Cannot allocate %d bytes for onfi data\n",
			256 * 4);
		return -ENOMEM;
	}
        gurnard_select_chips(host, device_addr);
        gurnard_write_command(host, NAND_CMD_READPARAM);
        gurnard_write_address(host, &addr, 1);
	/* The datasheet says that RnB should go low here,
	 * but it doesn't seem to be doing that, so do a fixed
	 * delay as well to be safe */
        gurnard_rnb_wait(host);
        udelay(25); /* tR */
        gurnard_read_buf(host, id, 256 * 4);
        gurnard_select_chips(host, 0);

	if ((id[0] & 0xff) != 'O' ||
	    (id[1] & 0xff) != 'N' ||
	    (id[2] & 0xff) != 'F' ||
	    (id[3] & 0xff) != 'I') {
		dev_err(&host->pdev->dev, "Invalid ONFI prefix: %x %x %x %x\n",
				id[0], id[1], id[2], id[3]);
		vfree(id);
		return -ENODEV;
	}

        for (i = 0; i < 256; i++) {
                if (!n_way_match(id[i], stack->width)) {
                        dev_err(&host->pdev->dev,
				"ONFI mismatch between chips at %d: 0x%8.8x\n",
                                i, id[i]);
			vfree(id);
			return -ENODEV;
		}
                *buf++ = id[i] & 0xff;
        }
	vfree(id);

        return 256;
}

/**
 * Determine if these NAND chips support the get/set features
 * commands. This is available from the ONFI data
 */
static int gurnard_supports_features(struct gurnard_nand_host *host)
{
	char buf[256];
	int ret;

	ret = gurnard_read_onfi(host, 1, buf);

	if (ret < 0)
		return ret;

	if (!(buf[8] & 0x4))
		return 0;

	ret = gurnard_read_onfi(host, 2, buf);

	if (ret < 0)
		return ret;

	return (buf[8] & 0x4) != 0;
}

static int gurnard_get_features(struct gurnard_nand_host *host,
		uint32_t device_addr, uint8_t feature_addr,
		uint8_t *featuresp)
{
	uint32_t features[4];
	int i;
	struct gurnard_nand_stack *stack;

        if (device_addr != 1 && device_addr != 2) {
                dev_err(&host->pdev->dev,
                        "Cannot read ID from anything other than device 1 or 2\n");
                return -ENODEV;
        }

	stack = &host->stack[device_addr - 1];

        gurnard_select_chips(host, device_addr);
        gurnard_write_command(host, NAND_CMD_GET_FEATURES);
        gurnard_write_address(host, &feature_addr, 1);
        gurnard_rnb_wait(host);
	gurnard_read_buf(host, features, 4 * ARRAY_SIZE(features));
        gurnard_select_chips(host, 0);

        if (!n_way_match(features[0], stack->width) ||
	    !n_way_match(features[1], stack->width) ||
            !n_way_match(features[2], stack->width) ||
	    !n_way_match(features[3], stack->width)) {
                dev_err(&host->pdev->dev,
                                "Device %d: Features do not match between chips: "
                                "0x%x 0x%x 0x%x 0x%x\n",
				device_addr,
                                features[0], features[1],
				features[2], features[3]);
                return -EINVAL;
        }

	for (i = 0; i < ARRAY_SIZE(features); i++)
		featuresp[i] = features[i] & 0xff;

	return ARRAY_SIZE(features);
}

static int gurnard_set_features(struct gurnard_nand_host *host,
		uint32_t device_addr, uint8_t feature_addr,
		uint32_t *features)
{
        gurnard_select_chips(host, device_addr);
        gurnard_write_command(host, NAND_CMD_SET_FEATURES);
        gurnard_write_address(host, &feature_addr, 1);
        /* FIXME: What is the correct delay system here? tADL */
        udelay(1);
	gurnard_write_buf(host, features, 16);
        gurnard_rnb_wait(host);
        gurnard_select_chips(host, 0);

	return 0;
}

static int gurnard_read_id(struct gurnard_nand_host *host, uint32_t device_addr,
                uint8_t *mfr, uint8_t *device, uint8_t *cellinfo, uint32_t *writesize,
                uint32_t *oobsize, uint32_t *erasesize, uint8_t *busw,
                uint8_t *stack_width)
{
        uint8_t addr = 0;
        uint32_t id[5];
        uint8_t v;
        uint32_t width;

        if (device_addr != 1 && device_addr != 2) {
                dev_err(&host->pdev->dev,
                        "Cannot read ID from anything other than device 1 or 2\n");
                return -ENODEV;
        }
        gurnard_select_chips(host, device_addr);
        gurnard_write_command(host, NAND_CMD_READID);
        gurnard_write_address(host, &addr, 1);
        gurnard_read_buf(host, id, 20);
        gurnard_select_chips(host, 0);

        /* If they are all 0, then there is no device */
        if (!id[0] && !id[1] && !id[2] && !id[3] && !id[4]) {
                dev_err(&host->pdev->dev, "No device available at %d\n",
                                device_addr);
                return -ENODEV;
        }

        /* Determine 32, 24, 16 or 8-bit bus */
        v = id[0] & 0xff;
        if ((id[0] >> 24) == v)
                width = 4;
        else if ((id[0] >> 16) == v)
                width = 3;
        else if ((id[0] >> 8) == v)
                width = 2;
        else
                width = 1;

        printk("Stack %d Width: %d ID: %8.8x %8.8x %8.8x %8.8x %8.8x\n",
                        device_addr, width,
                        id[0], id[1], id[2], id[3], id[4]);
        if (!n_way_match(id[0], width) || !n_way_match(id[1], width) ||
            !n_way_match(id[2], width) || !n_way_match(id[3], width) ||
            !n_way_match(id[4], width)) {
                dev_err(&host->pdev->dev,
                                "Device %d IDs do not match between chips: "
                                "0x%x 0x%x 0x%x 0x%x 0x%x\n",
                                device_addr,
                                id[0], id[1], id[2], id[3], id[4]);
                return -EINVAL;
        }
        *mfr = id[0];
        *device = id[1];
        *cellinfo = id[2];
        *writesize = 1024 << (id[3] & 0x3);
        *oobsize = (8 << ((id[3] >> 2) & 0x1)) * (*writesize >> 9);
        *erasesize = (64 * 1024) << ((id[3] >> 4) & 0x3);
        *busw = (id[3] & (1 << 6)) ? 16 : 8;
        *stack_width = width;
        return 0;
}

static inline int gurnard_read_stack_status(struct gurnard_nand_host *host,
		int device_addr)
{
	uint32_t status;
	uint32_t offset;

        gurnard_select_chips(host, device_addr);
        gurnard_write_command(host, NAND_CMD_STATUS);
	udelay(1); // FIXME: tWHR
	/**
	 * Since we do this after a write, and we then want
	 * to do a read-verify, we can't clear the fifo.
	 * The BUFF_LEN value is actually the absolute
	 * fifo position to read to, not the number of bytes.
	 */
	offset = reg_readl(BUFF_LEN);
        reg_writel(offset + 4 - 1, BUFF_LEN);
	gurnard_raw_read_buf(host, &status, 4, offset);
        gurnard_select_chips(host, 0);

        return status;
}

static inline int gurnard_read_status(struct gurnard_nand_host *host,
		int device_addr)
{
	/* Raid status needs to be read twice and intelligently combined */
        if (device_addr == RAID_ADDR) {
		uint32_t status0, status1;

		status0 = gurnard_read_stack_status(host, STACK_ADDR(0));
		status1 = gurnard_read_stack_status(host, STACK_ADDR(1));

#if 0
		if (status0 != status1)
			dev_warn(&host->pdev->dev, "Status diff: 0x%x 0x%x\n",
					status0, status1);
#endif

		/* Combined the nWP, RDY & ARDY signals as &
		 * But the FAIL & FAILC signals as |
		 * Skip all the 'reserved' bits
		 */
		return ((status0 & 0xe0e0e0e0) & (status1 & 0xe0e0e0e0)) |
		       ((status0 & 0x03030303) | (status1 & 0x03030303));
	} else
		return gurnard_read_stack_status(host, device_addr);
}

/**
 * Compare the ECC we read back from the FPGA with the one that we've
 * just read out of the oob. If they don't match, warn and try to
 * correct things. If it is uncorrectable, this function will retun < 0
 */
static int gurnard_ecc_check(struct mtd_info *mtd, loff_t addr,
                uint8_t *buf, uint8_t *oob, uint8_t *fpga_ecc)
{
        struct gurnard_nand_stack *stack = mtd->priv;
        struct gurnard_nand_host *host = stack->host;
	/* FIXME: It is assumed that the ECC data is all contiguous
	 * in the OOB */
        uint8_t *nand_ecc = &oob[mtd->ecclayout->eccpos[0]];
        int i;
#if !defined(USE_FPGA_ECC) || defined(VERIY_FPGA_ECC_CALC)
        uint8_t calc_ecc[ECC_CALC_SIZE];
#endif

#ifndef USE_FPGA_ECC
#warning "Overwriting the FPGA ECC with the one from the CPU"
        for (i = 0; i < mtd->writesize / ECC_CHUNK_SIZE; i++) {
                __nand_calculate_ecc(&buf[i * ECC_CHUNK_SIZE], ECC_CHUNK_SIZE,
                                calc_ecc);
                memcpy(&fpga_ecc[i * ECC_CALC_SIZE], calc_ecc, ECC_CALC_SIZE);
        }
#endif
#ifdef VERIFY_FPGA_ECC_CALC
#warning "Software ECC calculations being done"
        //nand_ecc[0] ^= 0x1; // put a single bit error into the ecc we get back from the device
        for (i = 0; i < mtd->writesize / ECC_CHUNK_SIZE; i++) {
                __nand_calculate_ecc(&buf[i * ECC_CHUNK_SIZE], ECC_CHUNK_SIZE,
                                calc_ecc);
                if (memcmp(&fpga_ecc[i * ECC_CALC_SIZE], calc_ecc,
                                        ECC_CALC_SIZE) != 0) {
                        dev_err(&host->pdev->dev,
                                "FPGA/Linux differ in ECC calc. 0x%llx/%d: "
                                "fpga:0x%2.2x%2.2x%2.2x != "
                                "calc:0x%2.2x%2.2x%2.2x\n",
                                        addr, i * ECC_CALC_SIZE,
                                        fpga_ecc[i * ECC_CALC_SIZE],
                                        fpga_ecc[i * ECC_CALC_SIZE + 1],
                                        fpga_ecc[i * ECC_CALC_SIZE + 2],
                                        calc_ecc[0], calc_ecc[1], calc_ecc[2]);
                        return -EINVAL;
                }
        }
#endif

        /* Insert a double bit fault */
        //fpga_ecc[0] ^= 0x3;

        if (memcmp(fpga_ecc, nand_ecc, mtd->ecclayout->eccbytes) == 0)
                return 0;

	/* If the block is all 0xff, then we're ok with an all
	 * 0xff ecc, as that is just an erased page
	 */
	for (i = 0; i < mtd->writesize; i++)
		if (buf[i] != 0xff)
			break;

	/* If the page is all 0xff, check the ecc */
	if (i == mtd->writesize) {
		for (i = 0; i < mtd->ecclayout->eccbytes; i++)
			if (nand_ecc[i] != 0xff)
				break;
		/* All 0xff in both data & ECC? Then we're just
		 * looking at an erased page, which is fine */
		if (i == mtd->ecclayout->eccbytes)
			return 0;
	}

        dev_warn(&host->pdev->dev, "ECC Error at 0x%llx\n", addr);
        /* Iterate over each 256 byte block checking if it is valid -
         * one of them at least is guaranteed to not be */
        for (i = 0; i < mtd->writesize / ECC_CHUNK_SIZE; i++) {
                int ret;
                //printk("Checking block %d, %d, %d\n", i, i * ECC_CALC_SIZE,
                                //i * ECC_CHUNK_SIZE);
                if (memcmp(&fpga_ecc[i * ECC_CALC_SIZE],
                           &nand_ecc[i * ECC_CALC_SIZE], ECC_CALC_SIZE) == 0)
                        continue;

                ret = __nand_correct_data(&buf[i * ECC_CHUNK_SIZE],
                                &nand_ecc[i * ECC_CALC_SIZE],
                                &fpga_ecc[i * ECC_CALC_SIZE],
                                ECC_CHUNK_SIZE);
                if (ret < 0) {
                        dev_err(&host->pdev->dev,
                                "Uncorrectable ECC error at 0x%llx, chunk offset %d\n",
                                addr, i * ECC_CHUNK_SIZE);
                        mtd->ecc_stats.failed++;
                        return -EBADMSG;
                } else if (ret > 0) {
                        dev_warn(&host->pdev->dev,
                                 "Corrected %d ECC failures at 0x%llx, chunk offset %d\n",
                                 ret, addr, i * ECC_CHUNK_SIZE);
                        mtd->ecc_stats.corrected += ret;
                } else
                        dev_warn(&host->pdev->dev,
                                 "Empty ECC correction at 0x%llx, chunk offset %d\n",
                                 addr, i * ECC_CHUNK_SIZE);


        }
        return 0;
}

/**
 * Trigger the FPGA to read the page, but don't actually transfer
 * the data out.
 */
static void gurnard_nand_read_full_page_dummy(struct mtd_info *mtd,
		int device_addr, loff_t from, int data, int oob)
{
        struct gurnard_nand_stack *stack = mtd->priv;
        struct gurnard_nand_host *host = stack->host;
        uint8_t addr[5];
	uint32_t len;

	gurnard_select_chips(host, device_addr);

        offset_to_page(stack, from, addr);
        if (!data)
                addr_to_oob(stack, addr);
	len = data ? mtd->writesize : 0 +
	      oob ? mtd->oobsize : 0;

        gurnard_write_command(host, NAND_CMD_READ0);
        gurnard_write_address(host, addr, 5);
        gurnard_write_command(host, NAND_CMD_READSTART);
        gurnard_rnb_wait(host);

	reg_writel(len - 1, BUFF_LEN);
        gurnard_busy_wait(host);
	gurnard_select_chips(host, 0);
}


static int gurnard_nand_read_page(struct mtd_info *mtd, loff_t from,
                uint8_t *buf, uint8_t *oob, int max_len)
{
        struct gurnard_nand_stack *stack = mtd->priv;
        struct gurnard_nand_host *host = stack->host;
        uint8_t addr[5];
        uint32_t fpga_ecc[mtd->ecclayout->eccbytes / 4];
        int ret = 0;
        int oob_len;

        if (from >= mtd->size)
                return -EINVAL;
        if (from & (mtd->writesize - 1)) {
                dev_err(&host->pdev->dev, "Invalid read page offset: 0x%llx\n",
                                from);
                return -EINVAL;
        }

	/* We always read at least the oob */
        if (max_len < 0) {
		max_len = mtd->oobsize;
		if (buf)
			max_len += mtd->writesize;
	}

	/* If we've asked for a read of > writesize, then that means
	 * we want to do an oob & data read into a single buffer
	 */
	if (max_len > mtd->writesize && !oob)
		oob = &buf[mtd->writesize];

        /* If this is the raid device, then we actually just read
         * from the first device
         * FIXME: Should be falling back to the second device if the
         * first one has a failure
         */
        if (stack->device_addr == RAID_ADDR)
                gurnard_select_chips(host, STACK_ADDR(0));
        else
                gurnard_select_chips(host, stack->device_addr);

        offset_to_page(stack, from, addr);

        /* OOB only read, so skip past the page to the oob */
        if (!buf)
                addr_to_oob(stack, addr);

#ifdef RAW_ACCESS_DEBUG
        printk("read: len=%d dev=0x%x 0x%llx 0x%2.2x%2.2x%2.2x%2.2x%2.2x%s%s\n",
                        max_len, stack->device_addr, from,
                        addr[4], addr[3], addr[2], addr[1], addr[0],
                        buf ? " data" : "", oob ? " oob" : "");
#endif

        /* We always do oob reads, just so that we can verify the ecc */
        if (!oob)
                oob = host->oob_tmp;

        gurnard_write_command(host, NAND_CMD_READ0);
        gurnard_write_address(host, addr, 5);
        gurnard_write_command(host, NAND_CMD_READSTART);
        gurnard_rnb_wait(host);
	/* Most common case - doing a full page read with oob */
	if (max_len == mtd->writesize + mtd->oobsize) {
		reg_writel(max_len - 1, BUFF_LEN);
		gurnard_raw_read_buf(host, (uint32_t *)buf, mtd->writesize, 0);
		gurnard_raw_read_buf(host, (uint32_t *)oob, mtd->oobsize,
				mtd->writesize);
		if (reg_readl(BUFF_LEN) != max_len) {
			dev_err(&host->pdev->dev,
				"Read double buffer, but lengths don't line up: %d,%d vs %d\n",
				reg_readl(BUFF_LEN), reg_readl(BUFF_LEN), max_len);
			//BUG();
		}

		/* Drag out the ECC results */
		readsl(host->reg_base + REG_ECC, fpga_ecc,
				mtd->ecclayout->eccbytes >> 2);

		oob_len = mtd->oobsize;
	} else {
		/* Doing some kind of partial read, either just OOB,
		 * or a partial page
                 */
                if (buf) {
                        int buf_len = min((int)mtd->writesize, max_len);
                        max_len -= buf_len;
                        gurnard_read_buf(host, (uint32_t *)buf, buf_len);

                        /* We've just finished reading the entire page,
                         * so now read the ECC from the fpga so we
                         * can compare later
                         */
                        readsl(host->reg_base + REG_ECC, fpga_ecc,
                                        mtd->ecclayout->eccbytes >> 2);
                }

                oob_len = min((int)mtd->oobsize, max_len);
                if (oob_len > 0)
                        gurnard_read_buf(host, (uint32_t *)oob, oob_len);
        }

        /* If we've read the full page of data, then we can do ECC checking */
        if (buf && oob_len == mtd->oobsize)
                ret = gurnard_ecc_check(mtd, from, buf, oob,
                                (uint8_t *)fpga_ecc);

        gurnard_select_chips(host, 0);

        return ret;
}

static int gurnard_nand_write_page(struct mtd_info *mtd, loff_t to,
                const uint8_t *buf, uint8_t *oob, int auto_ecc)
{
        struct gurnard_nand_stack *stack = mtd->priv;
        struct gurnard_nand_host *host = stack->host;
        uint8_t addr[5];
        int ret = 0;
        uint32_t status;

        if (to >= mtd->size)
                return -EINVAL;
        if (to & (mtd->writesize - 1)) {
                dev_err(&host->pdev->dev, "Invalid write page offset: 0x%llx\n",
                                to);
                return -EINVAL;
        }

        gurnard_select_chips(host, stack->device_addr);

        offset_to_page(stack, to, addr);

        /* OOB only write, so skip past the page to the oob */
        if (!buf)
                addr_to_oob(stack, addr);

#ifdef RAW_ACCESS_DEBUG
        printk("write: dev=0x%x 0x%llx 0x%2.2x%2.2x%2.2x%2.2x%2.2x%s%s\n",
                        stack->device_addr, to,
                        addr[4], addr[3], addr[2], addr[1], addr[0],
                        buf ? " data" : "", oob ? " oob" : "");
#endif

        gurnard_write_command(host, NAND_CMD_SEQIN);
        gurnard_write_address(host, addr, 5);
	udelay(1); /* FIXME: tADL */
        if (buf) {
		writesl(host->data_base, (uint32_t *)buf, mtd->writesize >> 2);
                if (auto_ecc && oob) {
#ifdef USE_FPGA_ECC
                        uint32_t *ecc;
                        ecc = (uint32_t *)&oob[mtd->ecclayout->eccpos[0]];
			readsl(host->reg_base + REG_ECC, ecc,
				mtd->ecclayout->eccbytes / 4);
#else
                        int i;
                        for (i = 0; i < mtd->writesize / ECC_CHUNK_SIZE; i++)
                                __nand_calculate_ecc(&buf[i * ECC_CHUNK_SIZE],
                                                ECC_CHUNK_SIZE,
                                                &oob[mtd->ecclayout->eccpos[0] +
                                                     i * ECC_CALC_SIZE]);
#endif
                }
        }
        if (oob)
		writesl(host->data_base, (uint32_t *)oob, mtd->oobsize >> 2);

	/* Wait for the FIFO to full drain */
        gurnard_busy_wait(host);

        gurnard_write_command(host, NAND_CMD_PAGEPROG);

        gurnard_rnb_wait(host);
        gurnard_select_chips(host, 0);

        status = gurnard_read_status(host, stack->device_addr);

        /* If bit 0 on any of the chips is set indicates a write failure */
        if (status & 0x03030303) {
                dev_err(&host->pdev->dev, "Write failure at 0x%llx: 0x%x\n",
                                to, status);
                ret = -EIO;
        }

	if (ret)
		return ret;

#ifdef CONFIG_MTD_NAND_VERIFY_WRITE
	/* We've just written a page, so read it back &
	 * confirm the data is valid */
#ifdef USE_FPGA_READ_VERIFY
	reg_writel(BIT_VERIFY, CFG_SET);
#if 0	// is the verify bit working?
	if (!(reg_readl(CFG_SET) & BIT_VERIFY))
		dev_err(&host->pdev->dev,
			"Failed to set verify bit: 0x%X\n", reg_readl(CFG_SET));
#endif
	if (stack->device_addr == RAID_ADDR) {
		/* Raid reads need two separate verifications */
		gurnard_nand_read_full_page_dummy(mtd, STACK_ADDR(0),
				to, buf ? 1 : 0, oob ? 1 : 0);
		if (!(reg_readl(CFG_SET) & BIT_VERIFY_OK)) {
			dev_err(&host->pdev->dev,
				"Write verify failure to Raid(0) at 0x%llx (Offset: %d)\n",
				to, reg_readl(VERIFY_FAIL));
			ret = -EIO;
			goto out;
		}
		gurnard_nand_read_full_page_dummy(mtd, STACK_ADDR(1),
				to, buf ? 1 : 0, oob ? 1 : 0);
		if (!(reg_readl(CFG_SET) & BIT_VERIFY_OK)) {
			dev_err(&host->pdev->dev,
				"Write verify failure to Raid(1) at 0x%llx (Offset: %d)\n",
				to, reg_readl(VERIFY_FAIL));
			ret = -EIO;
			goto out;
		}
	} else {
		gurnard_nand_read_full_page_dummy(mtd, stack->device_addr, to,
				buf ? 1 : 0, oob ? 1 : 0);
		if (!(reg_readl(CFG_SET) & BIT_VERIFY_OK)) {
			dev_err(&host->pdev->dev,
				"Write verify failure to %s at 0x%llx (buf: %p, oob: %p Offset: %d)\n",
				stack->name, to, buf, oob, reg_readl(VERIFY_FAIL));
			ret = -EIO;
			goto out;
		}
	}
out:
	reg_writel(BIT_VERIFY, CFG_CLR);
#else
#warning "Using in-CPU data comparison"
	ret = gurnard_nand_read_page(mtd, to,
			buf ? host->data_tmp : NULL,
			oob ? host->oob_tmp : NULL, -1);
	if (ret)
		return ret;
	if (buf && memcmp(host->data_tmp, buf, mtd->writesize) != 0) {
		dev_err(&host->pdev->dev,
				"Write verify failure at 0x%llx\n",
				to);
		ret = -EIO;
	}

	if (oob && memcmp(host->oob_tmp, oob, mtd->oobsize) != 0) {
		dev_err(&host->pdev->dev,
				"Write verify oob failure at 0x%llx\n",
				to);
		ret = -EIO;
	}
#endif
        return ret;
#endif

}

static int gurnard_nand_erase_block(struct gurnard_nand_stack *stack,
                loff_t block_addr)
{
        char addr[3];
        uint32_t status;
        struct gurnard_nand_host *host = stack->host;

#ifdef RAW_ACCESS_DEBUG
        printk("erase block: dev=%d block_addr=0x%llx\n",
			stack->device_addr, block_addr);
#endif
        gurnard_select_chips(host, stack->device_addr);
        offset_to_block(stack, block_addr, addr);
        gurnard_write_command(host, NAND_CMD_ERASE1);
        gurnard_write_address(host, addr, ARRAY_SIZE(addr));
        gurnard_write_command(host, NAND_CMD_ERASE2);
        gurnard_rnb_wait(host);
        gurnard_select_chips(host, 0);

        status = gurnard_read_status(host, stack->device_addr);

        /* If bit 0 on any of the chips is set indicates an erase failure.
	 * Note: The FAILC bit (bit 1) is not valid for erases, only
	 * the FAIL bit (bit 0)
	 */
        if (status & 0x01010101) {
                dev_err(&host->pdev->dev, "Erase failure at 0x%llx: 0x%x\n",
                                block_addr, status);
                return -EIO;
        }
        return 0;
}

#ifdef USE_BBT_CACHE
/* Set the cached BBT value */
static void gurnard_block_bbt_set(struct gurnard_nand_stack *stack,
		loff_t offset, int marker)
{
	/* The raid stack doesn't have a cache, it
	 * just refers to the underlying stacks */
	if (stack->device_addr == RAID_ADDR) {
		struct gurnard_nand_host *host = stack->host;
		gurnard_block_bbt_set(&host->stack[0], offset, marker);
		gurnard_block_bbt_set(&host->stack[1], offset, marker);
	} else {
		int block = offset_to_block_id(stack, offset);
		/* Clear the old marker */
		stack->bbt[block / 4] &= ~(BLOCK_MASK << ((block & 3) * 2));
		/* Set the new one */
		stack->bbt[block / 4] |= marker << ((block & 3) * 2);
	}
}

static inline uint8_t gurnard_block_bbt_get(struct gurnard_nand_stack *stack,
		loff_t offset)
{
	int block = offset_to_block_id(stack, offset);
	if (stack->device_addr == RAID_ADDR)
		BUG();
	return (stack->bbt[block / 4] >> ((block & 3) * 2)) & BLOCK_MASK;
}
#endif


/**
 * MTD Interface functions
 */
static int gurnard_nand_erase(struct mtd_info *mtd, struct erase_info *instr)
{
        struct gurnard_nand_stack *stack = mtd->priv;
        struct gurnard_nand_host *host = stack->host;
        int ret = 0;
        uint64_t erased = 0, cur_addr = instr->addr;

        if (cur_addr & (mtd->erasesize - 1)) {
                dev_err(&host->pdev->dev, "Erase from invalid block address: 0x%llx\n",
                                cur_addr);
                return -EINVAL;
        }

	if (instr->len & (mtd->erasesize - 1))
		dev_warn(&host->pdev->dev,
			 "Attempt to erase a non-eraseblock aligned area: 0x%llx\n",
			 instr->len);

#ifdef RAW_ACCESS_DEBUG
        printk("erase start: dev=0x%x start=0x%llx len=0x%llx\n",
                        stack->device_addr, instr->addr, instr->len);
#endif

        while (erased < instr->len && cur_addr < mtd->size && ret == 0) {
#ifdef USE_BBT_CACHE
		/* Clear out the bbt */
		gurnard_block_bbt_set(stack, cur_addr,
				BLOCK_UNKNOWN);
#endif
                ret = gurnard_nand_erase_block(stack, cur_addr);

                erased += mtd->erasesize;
                cur_addr += mtd->erasesize;
        }

        if (ret == 0) {
                instr->state = MTD_ERASE_DONE;
                mtd_erase_callback(instr);
        } else
                instr->state = MTD_ERASE_FAILED;

        return ret;
}


static int gurnard_nand_read(struct mtd_info *mtd, loff_t from, size_t len,
                size_t *retlen, uint8_t *buf)
{
        struct gurnard_nand_stack *stack = mtd->priv;
        struct gurnard_nand_host *host = stack->host;
        uint64_t cur_addr = from;
        int ret = 0;
        struct mtd_ecc_stats stats;

        //printk("gurnard_nand_read: from=0x%llx len=0x%x\n", from, len);

        if (from + len > mtd->size)
                return -EINVAL;
        if (!len)
                return 0;
        if (from & (mtd->writesize - 1)) {
                dev_err(&host->pdev->dev, "Invalid read offset: 0x%llx\n",
                                from);
                return -EINVAL;
        }
        if (len & (mtd->writesize - 1)) {
                dev_err(&host->pdev->dev, "Invalid read length: 0x%x\n",
                                len);
                return -EINVAL;
        }

        stats = mtd->ecc_stats;

        while (cur_addr < from + len) {
                ret = gurnard_nand_read_page(mtd, cur_addr, buf, NULL, -1);
                /* FIXME: If we're the raid device, and read_page fails,
                 * we should read from the other half.
                 * We should also re-write the page that just failed with
                 * the good data
                 */
                if (ret)
                        break;
                buf += mtd->writesize;
                cur_addr += mtd->writesize;
        }
        *retlen = len;

        if (ret)
                return ret;

        return mtd->ecc_stats.corrected - stats.corrected ? -EUCLEAN : 0;
}

static int gurnard_nand_write_oob(struct mtd_info *mtd, loff_t to,
                struct mtd_oob_ops *ops)
{
        struct gurnard_nand_stack *stack = mtd->priv;
        struct gurnard_nand_host *host = stack->host;
        uint8_t *buf, *oob;
        uint64_t len = 0, cur_addr;
        int auto_ecc = (ops->mode != MTD_OOB_RAW);
        int ret = 0;

        if (to & (mtd->writesize - 1)) {
                dev_err(&host->pdev->dev,
                        "OOB write to non page-aligned address 0x%llx\n",
                        to);
                return -EINVAL;
        }

        if (ops->mode == MTD_OOB_RAW) {
                dev_err(&host->pdev->dev,
                        "No write support for OOB raw mode\n");
                return -EINVAL;
        }

        if (!ops->datbuf && !ops->oobbuf)
                return -EINVAL;

        if (ops->datbuf) {
                if (ops->len == 0)
                        return 0;
                len = ops->len;
        }

        if (ops->oobbuf) {
                if ((ops->mode == MTD_OOB_AUTO &&
                    ops->ooblen > mtd->ecclayout->oobavail) ||
                    (ops->mode == MTD_OOB_PLACE &&
                     ops->ooblen > mtd->oobsize)) {
                        dev_err(&host->pdev->dev,
                                "OOB write with too much OOB data: 0x%x mode=%d\n",
                                ops->ooblen, ops->mode);
                        return -EINVAL;
                }
                /* Always just do a single block write if we're doing oob */
                if (len && len != mtd->writesize) {
                        dev_err(&host->pdev->dev,
                                "OOB write with more than one page of data: %lld\n",
                                len);
                        return -EINVAL;
                }
                len = mtd->writesize;
                if (ops->mode == MTD_OOB_AUTO) {
                        oob = host->oob_tmp;
                        memset(oob, 0xff, mtd->oobsize);
                        memcpy(&oob[mtd->ecclayout->oobfree[0].offset],
                                        ops->oobbuf, ops->ooblen);
                } else
                        oob = ops->oobbuf;
        } else
                oob = NULL;

        if (to + len > mtd->size)
                return -EINVAL;

        buf = ops->datbuf;

        cur_addr = to;

        while (len && !ret) {
                ret = gurnard_nand_write_page(mtd, cur_addr, buf, oob, auto_ecc);
                if (buf)
                        buf += mtd->writesize;
                cur_addr += mtd->writesize;
		len -= mtd->writesize;
        }

        if (buf)
                ops->retlen = buf - ops->datbuf;
        if (oob)
                ops->oobretlen = ops->mode == MTD_OOB_AUTO ?
                        ops->ooblen : mtd->oobsize;

        return ret;
}

static int gurnard_nand_read_oob(struct mtd_info *mtd, loff_t from,
                struct mtd_oob_ops *ops)
{
        struct gurnard_nand_stack *stack = mtd->priv;
        struct gurnard_nand_host *host = stack->host;
        uint64_t cur_addr = from;
        uint8_t *buf, *oob;
        uint64_t len = 0;
        int ret = 0;
        struct mtd_ecc_stats stats;

        if (!ops->datbuf && !ops->oobbuf)
                return -EINVAL;

        if (ops->datbuf) {
                if (!ops->len)
                        return 0;
                len = ops->len;
        }
        if (ops->mode == MTD_OOB_RAW) {
                dev_err(&host->pdev->dev,
                        "No read support for OOB raw mode\n");
                return -EINVAL;
        }

        if (ops->oobbuf) {
                if ((ops->mode == MTD_OOB_AUTO &&
                    ops->ooblen > mtd->ecclayout->oobavail) ||
                    (ops->mode == MTD_OOB_PLACE &&
                     ops->ooblen > mtd->oobsize)) {
                        dev_err(&host->pdev->dev,
                                "OOB read with too much OOB data: %d\n",
                                ops->ooblen);
                        return -EINVAL;
                }
                if (len && len != mtd->writesize) {
                        dev_err(&host->pdev->dev,
                                "OOB read with more than one page of data: %lld\n",
                                len);
                        return -EINVAL;
                }
                len = mtd->writesize;
                if (ops->mode == MTD_OOB_AUTO)
                        oob = host->oob_tmp;
                else
                        oob = ops->oobbuf;
        } else
                oob = NULL;

        if (!len)
                return -EINVAL;

        //printk("gurnard_nand_read_oob: from=0x%llx len=0x%llx (dat:%d oob:%d)\n",
                        //from, len, ops->datbuf ? 1 : 0, ops->oobbuf ? 1 : 0);

        if (from + len > mtd->size)
                return -EINVAL;
        if (!len)
                return 0;
        if (from & (mtd->writesize - 1)) {
                dev_err(&host->pdev->dev, "Invalid full read offset: 0x%llx\n",
                                from);
                return -EINVAL;
        }
        buf = ops->datbuf;

        stats = mtd->ecc_stats;

        while (cur_addr < from + len) {
                ret = gurnard_nand_read_page(mtd, cur_addr, buf, oob, -1);
                if (ret)
                        break;
                if (buf)
                        buf += mtd->writesize;
                if (ops->oobbuf && ops->mode == MTD_OOB_AUTO)
                        memcpy(ops->oobbuf,
                               &oob[mtd->ecclayout->oobfree[0].offset],
                               ops->ooblen);
                cur_addr += mtd->writesize;
        }

        if (buf)
                ops->retlen = buf - ops->datbuf;
        if (oob)
                ops->oobretlen = mtd->oobsize;

        if (ret)
                return ret;
        return mtd->ecc_stats.corrected - stats.corrected ? -EUCLEAN : 0;

}

static int gurnard_block_is_bad(struct mtd_info *mtd, loff_t offs)
{
        struct gurnard_nand_stack *stack = mtd->priv;
        uint32_t oob;
        int ret;
	int bad;
#ifdef USE_BBT_CACHE
	uint8_t marker;
#endif

        if (offs > mtd->size)
                return -EINVAL;

	/* For the raid device, check it off on both underlying pages */
	if (stack->device_addr == RAID_ADDR) {
		struct gurnard_nand_host *host = stack->host;
		ret = gurnard_block_is_bad(&host->stack[0].mtd, offs);
		if (ret)
			return ret;
		return gurnard_block_is_bad(&host->stack[1].mtd, offs);
	}

	/* We're only interested in the page at the beginning of the block */
	offs &= ~(mtd->erasesize - 1);

#ifdef USE_BBT_CACHE
	marker = gurnard_block_bbt_get(stack, offs);
	if (marker == BLOCK_UNKNOWN) {
#endif
		ret = gurnard_nand_read_page(mtd, offs, NULL,
				(uint8_t *)&oob, 4);
		if (ret)
			return ret;
		bad = oob != 0xffffffff;

#ifdef USE_BBT_CACHE
		/* Make sure the cache is up to date */
		marker = (bad ? BLOCK_BAD : BLOCK_GOOD);
		gurnard_block_bbt_set(stack, offs, marker);
	} else
		bad = (marker == BLOCK_BAD);
#endif
	return bad;
}

static int gurnard_block_mark_bad(struct mtd_info *mtd, loff_t offs)
{
        struct gurnard_nand_stack *stack = mtd->priv;
        struct gurnard_nand_host *host = stack->host;

	/* We're only interested in the page at the beginning of the block */
	offs &= ~(mtd->erasesize - 1);

        memset(host->oob_tmp, 0xff, mtd->oobsize);
	/* Set the bad-block marker to 0 */
        host->oob_tmp[0] = 0;
#ifdef USE_BBT_CACHE
	/* Make it unknown, since we're just about to read it back
	 * to confirm */
	gurnard_block_bbt_set(stack, offs, BLOCK_UNKNOWN);
#endif

	/* Ignore any error messages, as its quite likely that the verify,
	 * or ecc could cause a failure
	 */
        gurnard_nand_write_page(mtd, offs, NULL, host->oob_tmp, 0);

	if (gurnard_block_is_bad(mtd, offs) < 0) {
		dev_err(&host->pdev->dev,
			"Setting block 0x%llx to bad, but it didn't take\n",
			offs);
		return -EIO;
	}

	return 0;
}

static int gurnard_nand_write(struct mtd_info *mtd, loff_t to, size_t len,
                size_t *retlen, const uint8_t *buf)
{
        struct gurnard_nand_stack *stack = mtd->priv;
        struct gurnard_nand_host *host = stack->host;
        uint64_t cur_addr = to;
        int ret = 0;

        //printk("gurnard_nand_write: to=0x%llx len=0x%x\n", to, len);

        if (to + len > mtd->size)
                return -EINVAL;
        if (!len)
                return 0;
        if (to & (mtd->writesize - 1)) {
                dev_err(&host->pdev->dev, "Invalid write offset: 0x%llx\n",
                                to);
                return -EINVAL;
        }
        if (len & (mtd->writesize - 1)) {
                dev_err(&host->pdev->dev, "Invalid write length: 0x%x\n",
                                len);
                return -EINVAL;
        }
        memset(host->oob_tmp, 0xff, mtd->oobsize);
        while (len) {
                ret = gurnard_nand_write_page(mtd, cur_addr, buf, host->oob_tmp, 1);
                if (ret < 0)
                        break;
                buf += mtd->writesize;
                cur_addr += mtd->writesize;
		len -= mtd->writesize;
        }
        *retlen = cur_addr - to;
        //printk("nand_write done: %d 0x%llx\n", ret, cur_addr - to);

        return ret;
}

static irqreturn_t gurnard_nand_irq(int irq, void *dev_id)
{
        struct gurnard_nand_host *host = dev_id;
	uint32_t ifr;

	gpio_debug_set(DBG_PIN_NAND_IRQ, 1);

	ifr = reg_readl(IFR);
	/* Acknowledge the IRQ */
	reg_writel(ifr, IFR);

        printk("%s(): ifr=0x%x/0x%x cfg=0x%x\n", __FUNCTION__, ifr,
			reg_readl(IFR), reg_readl(CFG_SET));


	if (ifr & IFR_RNB)
		complete(&host->rnb_completion);

	if (ifr & IFR_BUSY)
		complete(&host->busy_completion);

	gpio_debug_set(DBG_PIN_NAND_IRQ, 0);

        return IRQ_HANDLED;
}

#ifdef USE_TIMING_MODE
static int gurnard_get_timing_mode(struct gurnard_nand_host *host,
		uint32_t *modep)
{
	uint8_t features[4];
	int ret;
	uint32_t mode;

	ret = gurnard_get_features(host, 1, NAND_TIMING_FEATURE, features);
	if (ret < 0)
		return ret;

	mode = features[0];

	/* Confirm that Stack 2 is the same as Stack 1 */
	ret = gurnard_get_features(host, 2, NAND_TIMING_FEATURE, features);
	if (ret < 0)
		return ret;

	if (features[0] != mode) {
		dev_err(&host->pdev->dev,
			"Mode timing differs between stacks: 0x%x vs 0x%x\n",
			mode, features[0]);
		return -EINVAL;
	}

	*modep = mode;

	return 0;
}

static int gurnard_set_timing_mode(struct gurnard_nand_host *host, uint8_t mode)
{
	uint32_t features[4];
	uint32_t new_mode;
	int ret;

	if (mode < 0 || mode > ARRAY_SIZE(mode_timing)) {
		dev_warn(&host->pdev->dev,
			 "Available timing mode %d is not supported, "
			 "defaulting to mode 0\n",
			 mode);
		mode = 0;
	}

	/* Update the NAND devices with the best speed */
	features[0] = BUS_EXPAND(mode);
	features[1] = 0;
	features[2] = 0;
	features[3] = 0;

	/* Set it on both */
	ret = gurnard_set_features(host, RAID_ADDR,
			NAND_TIMING_FEATURE, features);
	if (ret < 0)
		return ret;

	ret = gurnard_get_timing_mode(host, &new_mode);
	if (ret < 0)
		return ret;
	if (new_mode != mode) {
		dev_err(&host->pdev->dev,
			"Set mode to 0x%x, but it didn't apply: 0x%x\n",
			mode, new_mode);
		return -EINVAL;
	}

	/* Update the FPGA speed */
	reg_writel(mode_timing[mode].wr_timer, WR_TIMER);
	reg_writel(mode_timing[mode].rd_timer, RD_TIMER);

	return 0;
}


/**
 * Use the ONFI timing information to determine the optimal speed
 * that we can run these NAND flash devices at. Reconfigure both
 * the NAND flash devices, and the FPGA timings to these speeds
 */
static int gurnard_autoconfigure_timings(struct gurnard_nand_host *host)
{
	uint32_t best_mode;

#ifdef FORCE_SLOW_NAND
	best_mode = 0;
#else
	int ret;
	char buf[256];
	uint16_t timing_mode;

	/* FIXME: Should be reading timings from both stack 1 & 2.
	 * We don't really support disparat NAND stacks, so not doing this
	 * is probably ok. */

	ret = gurnard_read_onfi(host, 1, buf);
	if (ret < 0)
		return ret;

	timing_mode = buf[130] << 8 | buf[129];
	if ((timing_mode & 0x3f) == 0) {
		dev_warn(&host->pdev->dev,
			 "ONFI data doesn't seem to contain valid timing mode: 0x%x\n",
			 timing_mode);
		return -EINVAL;
	}
	/* Get the index of the highest bit that was set */
	best_mode = ffs(~timing_mode);

	/* ffs is 1 based, we want it 0 based, and the invert
	 * will have moved the bit on by one. */
	if (best_mode > 1)
		best_mode-=2;
#endif

	return gurnard_set_timing_mode(host, best_mode);
}
#endif

static ssize_t force_probe(struct device *dev, struct device_attribute *attr,
                           const char *buf, size_t count)
{
        struct platform_device *pdev = to_platform_device(dev);
        struct gurnard_nand_host *host = platform_get_drvdata(pdev);
        int i, n;
        int res;
#ifdef USE_TIMING_MODE
	int do_timing;
#endif

        /* Make sure the temporary buffers are all in place */
        if (!host->oob_tmp) {
                host->oob_tmp = kmalloc(MAX_OOBSIZE, GFP_KERNEL);
                if (!host->oob_tmp) {
                        dev_err(dev, "Unable to allocate %d bytes of oob area\n",
                                        MAX_OOBSIZE);
                        return -ENOMEM;
                }
        }

        if (!host->data_tmp) {
                host->data_tmp = kmalloc(MAX_WRITESIZE, GFP_KERNEL);
                if (!host->data_tmp) {
                        dev_err(dev, "Unable to allocate %d bytes of data area\n",
                                        MAX_WRITESIZE);
                        return -ENOMEM;
                }
        }

#if 0
	if (!host->has_irq) {
		int ret;

		/* Disable all the IRQs */
		reg_writel(BIT_RNB_IER | BIT_BUSY_IER, CFG_CLR);

		/* Acknowledge any outstanding IRQs */
		reg_writel(IFR_RNB | IFR_BUSY, IFR);

		ret = request_irq(host->irq, gurnard_nand_irq, 0,
				pdev->name, host);
		if (ret) {
			dev_err(&pdev->dev, "Cannot claim IRQ\n");
			return ret;
		}
		host->has_irq = 1;

		/* Enable the IRQs that we're interested in */
		reg_writel(BIT_RNB_IER | BIT_BUSY_IER, CFG_SET);
	}
#endif

	/* Reset the devices */
	gurnard_reset(host, RAID_ADDR);
#ifdef USE_TIMING_MODE
	do_timing = gurnard_supports_features(host);
	if (do_timing < 0)
		return do_timing;

	if (do_timing > 0) {
		/* Force the device into mode 0, just so
		 * we make sure the onfi read happens ok
		 */
		res = gurnard_set_timing_mode(host, 0);
		if (res < 0)
			return res;
	} else
		printk("NAND: Features not supported, so timing not adjusted\n");
#endif

        for (i = 0; i < MAX_STACKS; i++) {
                struct gurnard_nand_stack *stack = &host->stack[i];
                struct mtd_info *mtd = &stack->mtd;
                uint8_t mfr, device, cellinfo, busw, stack_width;
                uint32_t writesize, oobsize, erasesize;
                uint64_t size = 0;
                int addr = STACK_ADDR(i);

                if (mtd->priv) {
                        dev_warn(dev, "NAND stack %d already probed\n", i);
                        continue;
                }

                memset(stack, 0, sizeof(*stack));

                stack->host = host;
                stack->device_addr = addr;

                if (gurnard_read_id(host, addr, &mfr, &device, &cellinfo,
                                        &writesize, &oobsize, &erasesize,
                                        &busw, &stack_width) < 0)
                        continue;
                if (oobsize > MAX_OOBSIZE) {
                        dev_err(dev, "Stack %d: Too much OOB data: %d > %d\n",
                                        i, oobsize, MAX_OOBSIZE);
                        continue;
                }
                if (writesize > MAX_WRITESIZE) {
                        dev_err(dev, "Stack %d: Too much page data: %d > %d\n",
                                        i, writesize, MAX_WRITESIZE);
                        continue;
                }
                if (busw != 8) {
                        dev_err(dev, "Stack %d: Only 8-bit "
                                        "devices supported\n", i);
                        continue;
                }
		if (stack_width != 4) {
			dev_err(dev, "Stack %d: "
				     "Only support 32-bit NAND bus\n",
				     i);
			continue;
		}
                for (n = 0; nand_flash_ids[n].name != NULL; n++) {
                        if (device == nand_flash_ids[n].id)
                                size = nand_flash_ids[n].chipsize << 20;
                }
                if (size == 0) {
                        dev_err(dev, "Stack %d: Can't find nand "
                                        "record for device 0x%x\n",
                                        i, device);
                        continue;
                }

                stack->width = stack_width;

                mtd->priv = stack;
                snprintf(stack->name, sizeof(stack->name), "Stack%d", i);
                mtd->name = stack->name;
                mtd->type = MTD_NANDFLASH;
                mtd->flags = MTD_CAP_NANDFLASH;

                mtd->erase = gurnard_nand_erase;
                mtd->read = gurnard_nand_read;
                mtd->write = gurnard_nand_write;
                mtd->read_oob = gurnard_nand_read_oob;
                mtd->write_oob = gurnard_nand_write_oob;
                mtd->block_isbad = gurnard_block_is_bad;
                mtd->block_markbad = gurnard_block_mark_bad;

                if (writesize == 4096)
                        mtd->ecclayout = &gurnard_ecc_layout_4k;
		else if (writesize == 2048)
                        mtd->ecclayout = &gurnard_ecc_layout_2k;
                else {
                        dev_err(dev, "Stack %d: Invalid device page size: %d\n",
                                        i, writesize);
                        mtd->priv = NULL;
                        continue;
                }

                /* Since the banks are multi-plexed up 4-ways,
                 * everything is 4 times bigger
                 */
                mtd->writesize = writesize * 4;
                mtd->erasesize = erasesize * 4;
                mtd->size = size * 4;
                mtd->oobsize = oobsize * 4;
		stack->page_shift = ffs(mtd->writesize) - 1;

		printk("Page shift=0x%x writesize=0x%x\n", stack->page_shift,
				mtd->writesize);

#if 0
		if (mtd->erasesize != 0x00100000) {
			dev_err(dev, "Stack %d: Need to have 1MB/block sizes (0x%x)\n",
					i, mtd->erasesize);
			mtd->priv = NULL;
			continue;
		}
#endif

#ifdef USE_BBT_CACHE
		/* We need two bits for each block, so can fit 4
		 * into each byte. 0 values indicate unknwon */
		stack->bbt = vmalloc(offset_to_block_id(stack, mtd->size) / 4);
		memset(stack->bbt, 0, offset_to_block_id(stack, mtd->size) / 4);
#endif

                /* We don't have partitions on this device, just
                 * a single large area
                 */
                res = add_mtd_device(mtd);
                if (res) {
                        dev_err(dev, "Stack %d: Can't add MTD device\n", i);
                        mtd->priv = NULL;
                }
        }

        for (i = 0; i < MAX_STACKS; i++) {
                struct gurnard_nand_stack *stack = &host->stack[i];
                struct mtd_info *mtd = &stack->mtd;

                if (!mtd->priv) {
                        dev_err(dev, "Stacks not fully populated - "
                                        "not generating RAID device\n");
                        break;
                }
                if (i > 0) {
                        struct mtd_info *last_mtd = &host->stack[i - 1].mtd;
                        if (mtd->writesize != last_mtd->writesize ||
                            mtd->erasesize != last_mtd->erasesize ||
                            mtd->size != last_mtd->size ||
                            mtd->oobsize != last_mtd->oobsize) {
                                dev_err(dev, "Stacks do not have matching sizes"
                                                " - RAID not possible\n");
                                break;
                        }
                }
        }
	/* We found all the stacks, so create the raid device */
        if (i == MAX_STACKS) {
                struct gurnard_nand_stack *stack = &host->raid_stack;
                struct mtd_info *mtd = &stack->mtd;

                if (mtd->priv)
                        dev_err(dev, "RAID Stack already probed\n");
                else {
			int e;
                        memset(stack, 0, sizeof(*stack));

                        stack->host = host;
                        stack->device_addr = RAID_ADDR;

#ifdef USE_TIMING_MODE
			if (do_timing) {
				e = gurnard_autoconfigure_timings(host);
				if (e < 0) {
					dev_err(dev, "Unable to configure NAND timings\n");
					return e;
				}
			}
#endif
                        /* Just take the settings from stack 0,
                         * since they're all identical */
                        stack->width = host->stack[0].width;
			stack->page_shift = host->stack[0].page_shift;
                        mtd->priv = stack;
                        snprintf(stack->name, sizeof(stack->name), "Raid");
                        mtd->name = stack->name;
                        mtd->type = MTD_NANDFLASH;
                        mtd->flags = MTD_CAP_NANDFLASH;

                        mtd->erase = gurnard_nand_erase;
                        mtd->read = gurnard_nand_read;
                        mtd->write = gurnard_nand_write;
                        mtd->read_oob = gurnard_nand_read_oob;
                        mtd->write_oob = gurnard_nand_write_oob;
                        mtd->block_isbad = gurnard_block_is_bad;
                        mtd->block_markbad = gurnard_block_mark_bad;

                        mtd->ecclayout = host->stack[0].mtd.ecclayout;
                        mtd->writesize = host->stack[0].mtd.writesize;
                        mtd->erasesize = host->stack[0].mtd.erasesize;
                        mtd->size = host->stack[0].mtd.size;
                        mtd->oobsize = host->stack[0].mtd.oobsize;

#ifdef USE_BBT_CACHE
			/* The raid device uses the BBT cache from the
			 * separate stacks */
			stack->bbt = NULL;
#endif

                        /* We don't have partitions on this device, just
                         * a single large area
                         */
                        res = add_mtd_device(mtd);
                        if (res) {
                                dev_err(dev, "Raid: Can't add MTD device\n");
                                mtd->priv = NULL;
                        }
                }

        }

        return count;
}

static ssize_t read_id_0(struct device *dev, struct device_attribute *attr,
                char *buf)
{
        struct platform_device *pdev = to_platform_device(dev);
        struct gurnard_nand_host *host = platform_get_drvdata(pdev);
        uint8_t mfr, device, cellinfo, busw, stack_width;
        uint32_t writesize, oobsize, erasesize;

        if (gurnard_read_id(host, 1, &mfr, &device, &cellinfo, &writesize,
                                &oobsize, &erasesize, &busw, &stack_width) < 0)
                return 0;

        return sprintf(buf, "stack=0\nmfr=0x%x\ndevice=0x%x\n"
                        "writesize=0x%x\noobsize=0x%x\nerasesize=0x%x\n"
                        "busw=%d\nstack_width=%d\n",
                        mfr, device, writesize, oobsize, erasesize, busw,
                        stack_width);
}

static ssize_t read_id_1(struct device *dev, struct device_attribute *attr,
                char *buf)
{
        struct platform_device *pdev = to_platform_device(dev);
        struct gurnard_nand_host *host = platform_get_drvdata(pdev);
        uint8_t mfr, device, cellinfo, busw, stack_width;
        uint32_t writesize, oobsize, erasesize;

        if (gurnard_read_id(host, 2, &mfr, &device, &cellinfo, &writesize,
                                &oobsize, &erasesize, &busw, &stack_width) < 0)
                return 0;

        return sprintf(buf, "stack=1\nmfr=0x%x\ndevice=0x%x\n"
                        "writesize=0x%x\noobsize=0x%x\nerasesize=0x%x\n"
                        "busw=%d\nstack_width=%d\n",
                        mfr, device, writesize, oobsize, erasesize, busw,
                        stack_width);
}

static ssize_t read_onfi_0(struct device *dev, struct device_attribute *attr,
                char *buf)
{
        struct platform_device *pdev = to_platform_device(dev);
        struct gurnard_nand_host *host = platform_get_drvdata(pdev);

        return gurnard_read_onfi(host, 1, buf);
}

static ssize_t read_onfi_1(struct device *dev, struct device_attribute *attr,
                char *buf)
{
        struct platform_device *pdev = to_platform_device(dev);
        struct gurnard_nand_host *host = platform_get_drvdata(pdev);

        return gurnard_read_onfi(host, 2, buf);
}

static ssize_t reset(struct device *dev, struct device_attribute *attr,
                           const char *buf, size_t count)
{
        struct platform_device *pdev = to_platform_device(dev);
        struct gurnard_nand_host *host = platform_get_drvdata(pdev);
        gurnard_reset(host, RAID_ADDR);
        return count;
}

#ifdef USE_TIMING_MODE
static ssize_t write_timing_mode(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
        struct platform_device *pdev = to_platform_device(dev);
        struct gurnard_nand_host *host = platform_get_drvdata(pdev);
	int mode = simple_strtol(buf, NULL, 0);
	int ret;

	ret = gurnard_set_timing_mode(host, mode);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t read_timing_mode(struct device *dev, struct device_attribute *attr,
                char *buf)
{
        struct platform_device *pdev = to_platform_device(dev);
        struct gurnard_nand_host *host = platform_get_drvdata(pdev);
	uint32_t mode;
	int ret;

	ret = gurnard_get_timing_mode(host, &mode);
	if (ret < 0)
		return ret;
	return sprintf(buf, "%d\n", mode);
}

static DEVICE_ATTR(timing_mode, S_IWUSR | S_IRUGO, read_timing_mode,
						   write_timing_mode);
#endif /* USE_TIMING_MODE */

static DEVICE_ATTR(probe, S_IWUSR, NULL, force_probe);
static DEVICE_ATTR(id0, S_IRUGO, read_id_0, NULL);
static DEVICE_ATTR(id1, S_IRUGO, read_id_1, NULL);
static DEVICE_ATTR(onfi0, S_IRUGO, read_onfi_0, NULL);
static DEVICE_ATTR(onfi1, S_IRUGO, read_onfi_1, NULL);
static DEVICE_ATTR(reset, S_IWUSR, NULL, reset);

static struct attribute *gurnard_attributes[] = {
        &dev_attr_probe.attr,
        &dev_attr_reset.attr,
        &dev_attr_id0.attr,
        &dev_attr_id1.attr,
        &dev_attr_onfi0.attr,
        &dev_attr_onfi1.attr,
#ifdef USE_TIMING_MODE
	&dev_attr_timing_mode.attr,
#endif
        NULL,
};

static const struct attribute_group gurnard_group = {
        .attrs = gurnard_attributes,
};

static int __init gurnard_nand_probe(struct platform_device *pdev)
{
        struct gurnard_nand_host *host;
        struct resource *regs;
        struct resource *data;
        struct resource *irq;
        int ret;

        regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (!regs) {
                dev_err(&pdev->dev, "can't get register memory resource\n");
                return -ENXIO;
        }

        data = platform_get_resource(pdev, IORESOURCE_MEM, 1);
        if (!data) {
                dev_err(&pdev->dev, "can't get data memory resource\n");
                return -ENXIO;
        }

        irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
        if (!irq) {
                dev_err(&pdev->dev, "can't get irq resource\n");
                return -ENXIO;
        }

        host = kzalloc(sizeof(*host), GFP_KERNEL);
        host->pdev = pdev;

        host->reg_base = ioremap(regs->start, resource_size(regs));
        if (!host->reg_base) {
                dev_err(&pdev->dev, "Unable to map registers\n");
                ret = -EIO;
                goto map_failed;
        }

        host->data_base = ioremap(data->start, resource_size(data));
        if (!host->data_base) {
                dev_err(&pdev->dev, "Unable to map data region\n");
                ret = -EIO;
                goto map_data_failed;
        }

	init_completion(&host->rnb_completion);
	init_completion(&host->busy_completion);

	host->irq = irq->start;
        platform_set_drvdata(pdev, host);

        sysfs_create_group(&pdev->dev.kobj, &gurnard_group);

        return 0;

map_data_failed:
        iounmap(host->reg_base);
map_failed:
        kfree(host);
        return ret;
}

static int __exit gurnard_nand_remove(struct platform_device *pdev)
{
        struct gurnard_nand_host *host = platform_get_drvdata(pdev);
	int i;

	dev_warn(&host->pdev->dev, "Deleting MTD device\n");

	if (host->has_irq)
		free_irq(host->irq, host);

	for (i = 0; i < MAX_STACKS; i++) {
		if (host->stack[i].mtd.priv) {
			del_mtd_device(&host->stack[i].mtd);
			host->stack[i].mtd.priv = NULL;
		}
#ifdef USE_BBT_CACHE
		if (host->stack[i].bbt) {
			vfree(host->stack[i].bbt);
			host->stack[i].bbt = NULL;
		}
#endif
	}
	if (host->raid_stack.mtd.priv) {
		del_mtd_device(&host->raid_stack.mtd);
		host->raid_stack.mtd.priv = NULL;
	}

        if (host->oob_tmp)
                kfree(host->oob_tmp);
        if (host->data_tmp)
                kfree(host->data_tmp);

        sysfs_remove_group(&pdev->dev.kobj, &gurnard_group);
        platform_set_drvdata(pdev, NULL);
        iounmap(host->reg_base);
        iounmap(host->data_base);
        kfree(host);

        return 0;
}

static struct platform_driver gurnard_nand_driver = {
        .remove         = __exit_p(gurnard_nand_remove),
        .driver         = {
                .name   = "gurnard_nand",
                .owner  = THIS_MODULE,
        },
};

static int __init gurnard_nand_init(void)
{
        return platform_driver_probe(&gurnard_nand_driver, gurnard_nand_probe);
}

static void __exit gurnard_nand_exit(void)
{
        platform_driver_unregister(&gurnard_nand_driver);
}

module_init(gurnard_nand_init);
module_exit(gurnard_nand_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andre Renaud <andre@bluewatersys.com>");
MODULE_DESCRIPTION("NAND Driver for FPGA-attached RAID NAND");
