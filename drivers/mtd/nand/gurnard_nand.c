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

/* Over how many bytes does the FPGA calculate its ECC values? */
#define ECC_CHUNK_SIZE 256
/* How many bytes of ECC do we get for each chunk? */
#define ECC_CALC_SIZE 3

#define MAX_STACKS      2
#define MAX_OOBSIZE 512
#define MAX_WRITESIZE 8192
#define RAID_ADDR 3 /* Address toe access in the FPGA to get both devices */

/* Display debugging each time we do a low-level NAND access */
// #define RAW_ACCESS_DEBUG

/* Check that the Linux ECC calculations match the FPGA ones */
// #define VERIFY_FPGA_ECC_CALC

/* Actually use the FPGA ECC results */
#define USE_FPGA_ECC

/* If set, then force the NAND devices into mode 0, rather than
 * auto-probing the chips */
// #define FORCE_SLOW_NAND

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

enum {
        REG_CFG_SET     = 0x000,
        REG_CFG_CLR     = 0x004,
        REG_BYPASS      = 0x008,
	REG_RD_TIMER	= 0x010,
	REG_WR_TIMER	= 0x014,
        REG_VERIFY_FAIL = 0x01c,
        REG_BUFF_LEN    = 0x200,
        REG_ECC         = 0x400,

        BIT_ALE         = 1 << 0,
        BIT_CLE         = 1 << 1,
        NCE_SHIFT       = 2,
        MASK_NCE        = 3 << NCE_SHIFT,
        BIT_BUFF_CLEAR  = 1 << 8,
        BIT_VERIFY      = 1 << 9,
        BIT_RNB         = 1 << 17,
        BIT_BUSY        = 1 << 18,
        BIT_VERIFY_OK   = 1 << 19,
};

struct mode_timing_info {
	uint32_t wr_timer;
	uint32_t rd_timer;
};

/* See Section 7.6.1 of the Gurnard specification
 * These are essneitally bitmasks for a 7.5ns clock, telling the FPGA
 * how to clock in/out data from the NAND devices
 */
struct mode_timing_info mode_timing[] = {
	[0] = {0x200003fc, 0x20000fe0},
	[1] = {0x0040001e, 0x0040000f},
	[2] = {0x00100007, 0x00400007},
	[3] = {0x00080006, 0x00200003},
	[4] = {0x00080006, 0x00200003},
	[5] = {0x00040002, 0x00200003},
};

#if 1
#define reg_writel(v,r) writel(v, host->reg_base + REG_##r)
#define reg_readl(r) readl(host->reg_base + REG_##r)
#else
#define reg_writel(v,r) { writel(v, host->reg_base + REG_##r); printk("W 0x%x=0x%x\n", REG_##r, v); }
#define reg_readl(r) ({uint32_t __v = readl(host->reg_base + REG_##r); printk("R 0x%x=0x%x\n", REG_##r, __v); __v;})
#endif

struct gurnard_nand_host;

struct gurnard_nand_stack {
        struct gurnard_nand_host *host;
        int device_addr;
        char name[20];
        struct mtd_info mtd;
        uint8_t width; /* How many chips wide is it (1-4) */
};

struct gurnard_nand_host {
        struct platform_device *pdev;
        void __iomem    *reg_base;
        void __iomem    *data_base;
        struct gurnard_nand_stack stack[MAX_STACKS];
        uint8_t         *oob_tmp;       /* Temporary area for oob data */
        uint8_t         *data_tmp;      /* Temporary area for buffer data */

        struct gurnard_nand_stack raid_stack;       /* Combined RAID stack */
};

/**
 * Low level NAND access functions
 */
static inline void gurnard_select_chips(struct gurnard_nand_host *host,
		unsigned int chip_mask)
{
        /* FIXME: Should we be aquiring a lock if chip_mask is non-zero, and
         * clearing the lock if it is zero, or does the MTD layer cover us
         * for this?
         */
        /* FIXME: Should be taking the page address as an argument, and
         * selecting the appropriate level in the stack via the STACK ADDR
         * bits in the FPGA CFG_SET/CFG_CLR registers
         */
       reg_writel(MASK_NCE, CFG_SET);
       if (chip_mask)
               reg_writel((chip_mask & 0x3) << NCE_SHIFT, CFG_CLR);
       if (((reg_readl(CFG_SET) & MASK_NCE) >> NCE_SHIFT) !=
               (~(chip_mask) & 0x3))
                dev_err(&host->pdev->dev,
                        "Unable to apply NCE settings (mask=%d reg=0x%x)\n",
                                chip_mask, reg_readl(CFG_SET));
}

static inline void gurnard_busy_wait(struct gurnard_nand_host *host)
{
        while (reg_readl(CFG_SET) & BIT_BUSY)
                ;
}

static inline void gurnard_rnb_wait(struct gurnard_nand_host *host)
{
        // make sure we wait at least 100ns
        reg_readl(CFG_SET);
        reg_readl(CFG_SET);
        reg_readl(CFG_SET);
        while (!(reg_readl(CFG_SET) & BIT_RNB))
                ;
}

static inline void gurnard_write_command(struct gurnard_nand_host *host,
					 uint32_t command)
{
        reg_writel(BIT_CLE | BIT_BUFF_CLEAR, CFG_SET);
        reg_writel(command << 24 | command << 16 | command << 8 | command, BYPASS);
        gurnard_busy_wait(host);
        reg_writel(BIT_CLE, CFG_CLR);
}

static inline void gurnard_write_address(struct gurnard_nand_host *host,
		uint8_t *address, int address_len)
{
        uint32_t a;
        reg_writel(BIT_ALE | BIT_BUFF_CLEAR, CFG_SET);
        while (address_len--) {
                a = *address++;
                a = a << 24 | a << 16 | a << 8 | a;
                reg_writel(a, BYPASS);
                gurnard_busy_wait(host);
        }
        reg_writel(BIT_ALE, CFG_CLR);
}

static inline void gurnard_read_buf(struct gurnard_nand_host *host, uint32_t *buf, uint32_t buflen)
{
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
/* FIXME: Issue reading this way from the FPGA.
 * Smarter reading, where we read as they become available rather
 * than wait until until all the data is in the FPGA before we
 * take it out
 */
#if 0
        uint32_t read = 0;
        reg_writel(buflen - 1, BUFF_LEN);
        /* Note: BUFF_LEN doesn't reduce as we read from it, it is the
         * count of bytes read since it was last reset
         */
        do {
                uint32_t total = reg_readl(BUFF_LEN);
                int32_t avail = total - read;
                if (avail) {
                        //printk("%d bytes available %d read %d/%d total\n", avail, read, total, buflen);
                        readsl(host->data_base, buf, avail >> 2);
                        read += avail;
                        buf += avail / 4;
                }
        } while (read != buflen);
#endif
#if 1
        reg_writel(buflen - 1, BUFF_LEN);
        gurnard_busy_wait(host);
        readsl(host->data_base, buf, buflen >> 2);
#endif
}

static inline void gurnard_write_buf(struct gurnard_nand_host *host, const uint32_t *buf, uint32_t buflen)
{
        //int i;
        //for (i = 0; i < buflen >> 2; i++)
                //writel(buf[i], host->data_base);
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
static inline void offset_to_page(uint64_t from, uint8_t *addr)
{
        /* FIXME: Should be Converting the pagesize into this shift */
        from >>= 12; // we skip the first 4096 bytes, since that is the page size
        from >>= 2; // then we divide by 4, since we're multiplexed up that far
        addr[0] = 0; /* We never want to address individual bytes */
        addr[1] = 0;
        addr[2] = from;
        addr[3] = from >> 8;
        addr[4] = from >> 16;
}

/**
 * Jump a 5-byte address to the OOB region
 */
static inline void addr_to_oob(uint8_t *addr)
{
        /* FIXME: should this be
         * addr[1] = (pagesize / 4) >> 8;
         */
        addr[1] = 0x10;
}

/* Converts the 64-bit byte offset within the nand device to the
 * 3-byte block address
 */
static inline void offset_to_block(uint64_t from, uint8_t *addr)
{
        /* FIXME: Should be Converting the pagesize into this shift */
        from >>= 12; // we skip the first 4096 bytes, since that is the page size
        from >>= 2; // then we divide by 4, since we're multiplexed up that far
        addr[0] = from;
        addr[1] = from >> 8;
        addr[2] = from >> 16;
}

/**
 * High level NAND command functions
 */
static void gurnard_reset(struct gurnard_nand_host *host, uint32_t device_mask)
{
        gurnard_select_chips(host, device_mask);
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

        if (device_addr != 1 && device_addr != 2) {
                dev_err(&host->pdev->dev,
                        "Cannot read ID from anything other than device 1 or 2\n");
                return -ENODEV;
        }
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
        /* FIXME: What is the correct delay system here? */
        udelay(100);
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
                if (!n_way_match(id[i], 4))
                        dev_err(&host->pdev->dev, "Parameters mismatch at %d: 0x%8.8x",
                                        i, id[i]);
                *buf++ = id[i] & 0xff;
        }
	vfree(id);

        return 256;
}

static int gurnard_get_features(struct gurnard_nand_host *host,
		uint32_t device_addr, uint8_t feature_addr,
		uint8_t *featuresp)
{
	uint32_t features[4];
	int i;

        if (device_addr != 1 && device_addr != 2) {
                dev_err(&host->pdev->dev,
                        "Cannot read ID from anything other than device 1 or 2\n");
                return -ENODEV;
        }
        gurnard_select_chips(host, device_addr);
        gurnard_write_command(host, NAND_CMD_GET_FEATURES);
        gurnard_write_address(host, &feature_addr, 1);
        /* FIXME: What is the correct delay system here? tWB */
        udelay(1);
        gurnard_rnb_wait(host);
	gurnard_read_buf(host, features, 4 * ARRAY_SIZE(features));
        gurnard_select_chips(host, 0);

        if (!n_way_match(features[0], 4) || !n_way_match(features[1], 4) ||
            !n_way_match(features[2], 4) || !n_way_match(features[3], 4)) {
                dev_err(&host->pdev->dev,
                                "Features do not match between chips: "
                                "0x%x 0x%x 0x%x 0x%x\n",
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

        printk("Width: %d ID: %8.8x %8.8x %8.8x %8.8x %8.8x\n",
                        width,
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

static int gurnard_read_status(struct gurnard_nand_host *host)
{
        uint32_t status;
        gurnard_write_command(host, NAND_CMD_STATUS);
        gurnard_read_buf(host, &status, 4);
        return status;
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


static int gurnard_nand_read_page(struct mtd_info *mtd, loff_t from,
                uint8_t *buf, uint8_t *oob, int max_len)
{
        struct gurnard_nand_stack *stack = mtd->priv;
        struct gurnard_nand_host *host = stack->host;
        uint8_t addr[5];
        uint32_t fpga_ecc[mtd->ecclayout->eccbytes / 4];
        int ret = 0;
        int i, len;

        if (from >= mtd->size)
                return -EINVAL;
        if (from & (mtd->writesize - 1)) {
                dev_err(&host->pdev->dev, "Invalid read page offset: 0x%llx\n",
                                from);
                return -EINVAL;
        }

        if (max_len < 0)
                max_len = mtd->oobsize + mtd->writesize;

        /* If this is the raid device, then we actually just read
         * from the first device
         * FIXME: Should be falling back to the second device if the
         * first one has a failure
         */
        if (stack->device_addr == RAID_ADDR)
                gurnard_select_chips(host, 1 << 0);
        else
                gurnard_select_chips(host, stack->device_addr);

        offset_to_page(from, addr);

        /* OOB only read, so skip past the page to the oob */
        if (!buf)
                addr_to_oob(addr);

#ifdef RAW_ACCESS_DEBUG
        printk("read: dev=0x%x 0x%llx 0x%2.2x%2.2x%2.2x%2.2x%2.2x%s%s\n",
                        stack->device_addr, from,
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
        reg_writel(BIT_BUFF_CLEAR, CFG_SET);
        if (buf) {
                gurnard_read_buf(host, (uint32_t *)buf,
                                min((int)mtd->writesize, max_len));
                max_len -= mtd->writesize;
                if (max_len < 0)
                        max_len = 0;

                /* We've just finished reading the entire page,
                 * so now read the ECC from the fpga so we
                 * can compare later
                 */
                for (i = 0; i < mtd->ecclayout->eccbytes / 4; i++)
                        fpga_ecc[i] = reg_readl(ECC);
        }

        /* Reset the buffers for the OOB read, otherwise when we
         * go to read them back we get incorrect counts from BUFF_LEN
         */
        reg_writel(BIT_BUFF_CLEAR, CFG_SET);

        len = min((int)mtd->oobsize, max_len);
        gurnard_read_buf(host, (uint32_t *)oob, len);

        /* If we've read the full page of data, then we can do ECC checking */
        if (buf && len == mtd->oobsize)
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

#if 0
        /* Is this a problem? */
        if (auto_ecc && (!oob || !buf))
                dev_warn(&host->pdev->dev,
                        "Page write at 0x%llx with automatic ecc, "
                        "but no oob/data provided\n", to);
#endif

        gurnard_select_chips(host, stack->device_addr);

        offset_to_page(to, addr);

        /* OOB only write, so skip past the page to the oob */
        if (!buf)
                addr_to_oob(addr);

#ifdef RAW_ACCESS_DEBUG
        printk("write: dev=0x%x 0x%llx 0x%2.2x%2.2x%2.2x%2.2x%2.2x%s%s\n",
                        stack->device_addr, to,
                        addr[4], addr[3], addr[2], addr[1], addr[0],
                        buf ? " data" : "", oob ? " oob" : "");
#endif


        gurnard_write_command(host, NAND_CMD_SEQIN);
        gurnard_write_address(host, addr, 5);
        gurnard_rnb_wait(host);
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

        status = gurnard_read_status(host);

        /* If bit 0 on any of the chips is set indicates an erase failure */
        if (status & 0x03030303) {
                dev_err(&host->pdev->dev, "Write failure at 0x%llx: 0x%x\n",
                                to, status);
                ret = -EIO;
        }
        gurnard_select_chips(host, 0);

#ifdef CONFIG_MTD_NAND_VERIFY_WRITE
        if (!ret) {
                /* We've just written a page, so read it back &
                 * confirm the data is valid */

                /* FIXME: Should be using the in-fpga read-verify mechanism */
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
        }

#endif

        return ret;
}

static int gurnard_nand_erase_block(struct gurnard_nand_host *host,
                loff_t block_addr)
{
        char addr[3];
        uint32_t status;

#ifdef RAW_ACCESS_DEBUG
        printk("erase block: 0x%llx\n", block_addr);
#endif

        offset_to_block(block_addr, addr);
        gurnard_write_command(host, NAND_CMD_ERASE1);
        gurnard_write_address(host, addr, ARRAY_SIZE(addr));
        gurnard_write_command(host, NAND_CMD_ERASE2);
        gurnard_rnb_wait(host);
        status = gurnard_read_status(host);

        /* If bit 0 on any of the chips is set indicates an erase failure */
        if (status & 0x01010101) {
                dev_err(&host->pdev->dev, "Erase failure at 0x%llx: 0x%x\n",
                                block_addr, status);
                return -EIO;
        }
        return 0;
}

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

#ifdef RAW_ACCESS_DEBUG
        printk("erase start: dev=0x%x start=0x%llx len=0x%llx\n",
                        stack->device_addr, instr->addr, instr->len);
#endif

        gurnard_select_chips(host, stack->device_addr);
        while (erased < instr->len && cur_addr < mtd->size && ret == 0) {
                ret = gurnard_nand_erase_block(host, cur_addr);

                erased += mtd->erasesize;
                cur_addr += mtd->erasesize;
        }
        gurnard_select_chips(host, 0);

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

        while (cur_addr < to + len && !ret) {
                ret = gurnard_nand_write_page(mtd, cur_addr, buf, oob, auto_ecc);
                if (buf)
                        buf += mtd->writesize;
                cur_addr += mtd->writesize;
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
        uint32_t oob;
        int ret;

        if (offs > mtd->size)
                return -EINVAL;
        /* FIXME:
         * Should be caching the bad block information, and restoring from
         * the cache. Can we use the bbt stuff built into mtd?
         */

        /* We're only interested in the page at the beginning of the block */
        offs = offs & ~(mtd->erasesize - 1);

        ret = gurnard_nand_read_page(mtd, offs, NULL, (uint8_t *)&oob, 4);
        if (ret)
                return ret;
#if 0 //def RAW_ACCESS_DEBUG
        if (oob != 0xffffffff)
                printk("block_is_bad: 0x%llx=0x%x\n", offs, oob);
#endif
        return (oob != 0xffffffff);
}

static int gurnard_block_mark_bad(struct mtd_info *mtd, loff_t offs)
{
        struct gurnard_nand_stack *stack = mtd->priv;
        struct gurnard_nand_host *host = stack->host;

        memset(host->oob_tmp, 0xff, mtd->oobsize);
        host->oob_tmp[0] = 0;

        return gurnard_nand_write_page(mtd, offs, NULL, host->oob_tmp, 0);
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
        while (cur_addr < to + len) {
                ret = gurnard_nand_write_page(mtd, cur_addr, buf, host->oob_tmp, 1);
                if (ret < 0)
                        break;
                buf += mtd->writesize;
                cur_addr += mtd->writesize;
        }
        *retlen = cur_addr - to;
        //printk("nand_write done: %d 0x%llx\n", ret, cur_addr - to);

        return ret;
}

static int gurnard_get_timing_mode(struct gurnard_nand_host *host)
{
	uint8_t features[4];
	int ret;
	int mode;

	ret = gurnard_get_features(host, 1, NAND_TIMING_FEATURE, features);
	if (ret < 0)
		return ret;

	mode = (int)features[0];

	ret = gurnard_get_features(host, 2, NAND_TIMING_FEATURE, features);
	if (ret < 0)
		return ret;

	if (features[0] != mode) {
		dev_err(&host->pdev->dev,
			"Mode timing differs between stacks: 0x%x vs 0x%x\n",
			mode, features[0]);
		return -EINVAL;
	}

	return mode;
}

static int gurnard_set_timing_mode(struct gurnard_nand_host *host, int mode)
{
	uint32_t features[4];
	int new_mode;
	int ret;

	if (mode < 0 || mode > ARRAY_SIZE(mode_timing)) {
		dev_warn(&host->pdev->dev,
			 "Available timing mode %d is not supported, "
			 "defaulting to mode 0\n",
			 mode);
		mode = 0;
	}

	/* Update the NAND devices with the best speed */
	features[0] = mode       | mode << 8 |
		      mode << 16 | mode << 24;
	features[1] = 0;
	features[2] = 0;
	features[3] = 0;

	/* Set it on both */
	ret = gurnard_set_features(host, 3, NAND_TIMING_FEATURE, features);
	if (ret < 0)
		return ret;

	new_mode = gurnard_get_timing_mode(host);
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
	int ret;
	int best_mode;

#ifdef FORCE_SLOW_NAND
	best_mode = 0;
#else
	char buf[256];
	uint16_t timing_mode;

	/* FIXME: Should be reading timings from both stack 1 & 2 */

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

	ret = gurnard_set_timing_mode(host, best_mode);
	if (ret < 0)
		return ret;

	return 0;
}

static ssize_t force_probe(struct device *dev, struct device_attribute *attr,
                           const char *buf, size_t count)
{
        struct platform_device *pdev = to_platform_device(dev);
        struct gurnard_nand_host *host = platform_get_drvdata(pdev);
        int i, n;
        int res;

        /* Make sure the temporary buffers are all in place */
        if (!host->oob_tmp) {
                host->oob_tmp = kmalloc(MAX_OOBSIZE, GFP_KERNEL);
                if (!host->oob_tmp) {
                        dev_err(dev, "Unable to allocate %d bytes of oob area\n",
                                        MAX_OOBSIZE);
                        return -1;
                }
        }

        if (!host->data_tmp) {
                host->data_tmp = kmalloc(MAX_WRITESIZE, GFP_KERNEL);
                if (!host->data_tmp) {
                        dev_err(dev, "Unable to allocate %d bytes of data area\n",
                                        MAX_WRITESIZE);
                        return -1;
                }
        }


        for (i = 0; i < MAX_STACKS; i++) {
                struct gurnard_nand_stack *stack = &host->stack[i];
                struct mtd_info *mtd = &stack->mtd;
                uint8_t mfr, device, cellinfo, busw, stack_width;
                uint32_t writesize, oobsize, erasesize;
                uint64_t size = 0;
                int addr = 1 << i;

                if (mtd->priv) {
                        dev_warn(dev, "NAND stack %d already probed\n", i);
                        continue;
                }

                memset(stack, 0, sizeof(*stack));

                stack->host = host;
                stack->device_addr = addr;

                gurnard_reset(host, stack->device_addr);

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
                // cap it for testing to 128MB
                //mtd->size = 128 * 1024 * 1024;
                mtd->oobsize = oobsize * 4;

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

                        gurnard_reset(host, stack->device_addr);

			/* Force the device into mode 0, just so
			 * we make sure the onfi read happens ok
			 */
			gurnard_set_timing_mode(host, 0);
#if 0
			e = gurnard_autoconfigure_timings(host);
			if (e < 0) {
				dev_err(dev, "Unable to configure NAND timings\n");
				return e;
			}
#endif
                        /* Just take the settings from stack 0,
                         * since they're all identical */
                        stack->width = host->stack[0].width;
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
	int mode;

	mode = gurnard_get_timing_mode(host);
	if (mode < 0)
		return mode;
	return sprintf(buf, "%d\n", mode);
}

static DEVICE_ATTR(probe, S_IWUSR, NULL, force_probe);
static DEVICE_ATTR(id0, S_IRUGO, read_id_0, NULL);
static DEVICE_ATTR(id1, S_IRUGO, read_id_1, NULL);
static DEVICE_ATTR(onfi0, S_IRUGO, read_onfi_0, NULL);
static DEVICE_ATTR(onfi1, S_IRUGO, read_onfi_1, NULL);
static DEVICE_ATTR(reset, S_IWUSR, NULL, reset);
static DEVICE_ATTR(timing_mode, S_IWUSR | S_IRUGO, read_timing_mode,
						   write_timing_mode);

static struct attribute *gurnard_attributes[] = {
        &dev_attr_probe.attr,
        &dev_attr_reset.attr,
        &dev_attr_id0.attr,
        &dev_attr_id1.attr,
        &dev_attr_onfi0.attr,
        &dev_attr_onfi1.attr,
	&dev_attr_timing_mode.attr,
        NULL,
};

static const struct attribute_group gurnard_group = {
        .attrs = gurnard_attributes,
};

#if 0
static irqreturn_t gurnard_nand_irq(int irq, void *dev_id)
{
        struct gurnard_nand_host *host = dev_id;

        printk("GURNARD NAND IRQ\n");

        return IRQ_HANDLED;
}
#endif

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

#if 0
        ret = request_irq(irq->start, gurnard_nand_irq, 0, pdev->name, host);
        if (ret) {
                dev_err(&pdev->dev, "Cannot claim IRQ\n");
                goto request_irq_failed;
        }
#endif
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

        for (i = 0; i < MAX_STACKS; i++)
                if (host->stack[i].mtd.priv)
                                del_mtd_device(&host->stack[i].mtd);
        if (host->raid_stack.mtd.priv)
                del_mtd_device(&host->raid_stack.mtd);

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
