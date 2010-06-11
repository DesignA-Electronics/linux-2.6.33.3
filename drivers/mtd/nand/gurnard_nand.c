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
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <asm/io.h>

#define MAX_STACKS      2

/* FIXME: This should be provided via the platform data */
static struct mtd_partition gurnard_partitions[] = {
        {
                .name   = "Data",
                .offset = 0,
                .size   = MTDPART_SIZ_FULL,
        },
};

/* The first 4-bytes (32-bit word) of the ecc is the bad-block marker
 * for each of the 4 8-bit devices. We then have 192 bytes of ECC,
 * followed by 316 bytes of available oob space
 */
static struct nand_ecclayout gurnard_ecc_layout_4k = {
        /* There are 3 bytes of ecc for each 256 bytes of data,
         * and 4 * 4k = 16k of data. 16k / 256 * 3 = 192
         **/
        .eccbytes = 192,
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
};

struct gurnard_nand_host {
        struct platform_device *pdev;
        void __iomem    *reg_base;
        void __iomem    *data_base;
        struct gurnard_nand_stack stack[MAX_STACKS];
};

/**
 * Low level NAND access functions
 */
static inline void gurnard_select_chips(struct gurnard_nand_host *host, unsigned int chip_mask)
{
        /* FIXME: Should be aquiring a lock if chip_mask is non-zero, and
         * clearing the lock if it is zero
         */
       reg_writel(MASK_NCE, CFG_SET);
       if (chip_mask)
               reg_writel((chip_mask & 0x3) << NCE_SHIFT, CFG_CLR);
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

static inline void gurnard_write_command(struct gurnard_nand_host *host, uint32_t command)
{
        reg_writel(BIT_CLE | BIT_BUFF_CLEAR, CFG_SET);
        reg_writel(command << 24 | command << 16 | command << 8 | command, BYPASS);
        gurnard_busy_wait(host);
        reg_writel(BIT_CLE, CFG_CLR);
}

static inline void gurnard_write_address(struct gurnard_nand_host *host, uint8_t *address, int address_len)
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
        reg_writel(buflen - 1, BUFF_LEN);
        /* FIXME: Don't have to wait till it is completely done,
         * we can read BUFF_LEN and read that many bytes out, and
         * keep doing that until we've read the lot
         */
#if 0
        do {
                uint32_t avail = reg_readl(BUFF_LEN);
                if (avail) {
                        /* For some reason BUFF_LEN doesn't reduce? */
                        printk("%d bytes available %d remaining\n", avail, buflen);
                        avail = min(avail, buflen);
                        readsl(host->data_base, buf, avail >> 2);
                        buflen -= avail;
                        buf += avail;
                }
        } while (buflen);
#endif

#if 1
        gurnard_busy_wait(host);
        readsl(host->data_base, buf, buflen >> 2);
#endif

#if 0
        gurnard_busy_wait(host);
        buflen>>=2;
        while (buflen--)
                *buf++ = readl(host->data_base);
#endif
}

static inline void gurnard_write_buf(struct gurnard_nand_host *host, const uint32_t *buf, uint32_t buflen)
{
        writesl(host->data_base, buf, buflen >> 2);
        gurnard_busy_wait(host);

}

static inline int four_way_match(uint32_t v)
{
        return ((((v & 0xff000000) >> 24) == (v & 0xff)) &&
                (((v & 0x00ff0000) >> 16) == (v & 0xff)) &&
                (((v & 0x0000ff00) >>  8) == (v & 0xff)));
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
        gurnard_select_chips(host, 0);
        gurnard_rnb_wait(host);
}

static int gurnard_read_id(struct gurnard_nand_host *host, uint32_t device_addr,
                uint8_t *mfr, uint8_t *device, uint8_t *cellinfo, uint32_t *writesize,
                uint32_t *oobsize, uint32_t *erasesize, uint8_t *busw)
{
        uint8_t addr = 0;
        uint32_t id[5];
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
        //printk("ID: %8.8x %8.8x %8.8x %8.8x %8.8x\n",
                        //id[0], id[1], id[2], id[3], id[4]);
        if (!four_way_match(id[0]) || !four_way_match(id[1]) ||
            !four_way_match(id[2]) || !four_way_match(id[3]) ||
            !four_way_match(id[4])) {
                dev_err(&host->pdev->dev, "IDs do not match between chips: "
                                "0x%x 0x%x 0x%x 0x%x 0x%x\n",
                                id[0], id[1], id[2], id[3], id[4]);
                return -EINVAL;
        }
        if (!id[0] && !id[1] && !id[2] && !id[3] && !id[4]) {
                dev_err(&host->pdev->dev, "No device available at %d\n",
                                device_addr);
                return -ENODEV;
        }
        *mfr = id[0];
        *device = id[1];
        *cellinfo = id[2];
        *writesize = 1024 << (id[3] & 0x3);
        *oobsize = (8 << ((id[3] >> 2) & 0x1)) * (*writesize >> 9);
        *erasesize = (64 * 1024) << ((id[3] >> 4) & 0x3);
        *busw = (id[3] & (1 << 6)) ? 16 : 8;
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
 * MTD Interface functions
 */
static int gurnard_nand_erase(struct mtd_info *mtd, struct erase_info *instr)
{
        struct gurnard_nand_stack *stack = mtd->priv;
        struct gurnard_nand_host *host = stack->host;
        int ret = 0;
        uint8_t addr[3];
        uint32_t status;
        uint64_t erased = 0, cur_addr = instr->addr;

        if (cur_addr & (mtd->erasesize - 1)) {
                dev_err(&host->pdev->dev, "Erase from invalid block address: 0x%llx\n",
                                cur_addr);
                return -EINVAL;
        }

        gurnard_select_chips(host, stack->device_addr);
        while (erased < instr->len && cur_addr < mtd->size && ret == 0) {
                //printk("gurnard_nand_erase: Addr=0x%llx Length=0x%llx\n",
                                //cur_addr, instr->len);
                offset_to_block(cur_addr, addr);
                gurnard_write_command(host, NAND_CMD_ERASE1);
                gurnard_write_address(host, addr, 3);
                gurnard_write_command(host, NAND_CMD_ERASE2);
                gurnard_rnb_wait(host);
                status = gurnard_read_status(host);

                /* If bit 0 on any of the chips is set indicates an erase failure */
                if (status & 0x01010101) {
                        dev_err(&host->pdev->dev, "Erase failure at 0x%llx: 0x%x\n",
                                        cur_addr, status);
                        ret = -EIO;
                }

                erased += mtd->erasesize;
                cur_addr += mtd->erasesize;
        }
        gurnard_select_chips(host, 0);
        if (ret == 0)
                instr->state = MTD_ERASE_DONE;
        else
                instr->state = MTD_ERASE_FAILED;
        if (!ret)
                mtd_erase_callback(instr);
        return ret;
}

static int gurnard_nand_read_page(struct mtd_info *mtd, loff_t from,
                uint8_t *buf, uint8_t *oob, int max_len)
{
        struct gurnard_nand_stack *stack = mtd->priv;
        struct gurnard_nand_host *host = stack->host;
        uint8_t addr[5];

        if (from >= mtd->size)
                return -EINVAL;
        if (from & (mtd->writesize - 1)) {
                dev_err(&host->pdev->dev, "Invalid read page offset: 0x%llx\n",
                                from);
                return -EINVAL;
        }

        if (max_len < 0)
                max_len = mtd->oobsize + mtd->writesize;

        gurnard_select_chips(host, stack->device_addr);
        offset_to_page(from, addr);
        /* OOB only read, so skip past the page to the oob */
        if (!buf) {
                addr[0] = mtd->writesize / 4;
                addr[1] = (mtd->writesize / 4) >> 8;
        }
        //printk("gurnard_nand_read_page: from=0x%llx%s%s addr=0x%2.2x%2.2x%2.2x%2.2x%2.2x\n",
                        //from, buf ? " data" : "", oob ? " oob" : "",
                        //addr[4], addr[3], addr[2], addr[1], addr[0]);


        gurnard_write_command(host, NAND_CMD_READ0);
        gurnard_write_address(host, addr, 5);
        gurnard_write_command(host, NAND_CMD_READSTART);
        gurnard_rnb_wait(host);
        if (buf) {
                gurnard_read_buf(host, (uint32_t *)buf,
                                min((int)mtd->writesize, max_len));
                max_len -= mtd->writesize;
                if (max_len < 0)
                        max_len = 0;
        }
        if (oob) {
                gurnard_read_buf(host, (uint32_t *)oob,
                                min((int)mtd->oobsize, max_len));
        }
        gurnard_select_chips(host, 0);

        return 0;
}

static int gurnard_nand_write_page(struct mtd_info *mtd, loff_t to,
                const uint8_t *buf, uint8_t *oob, int auto_ecc)
{
        struct gurnard_nand_stack *stack = mtd->priv;
        struct gurnard_nand_host *host = stack->host;
        uint8_t addr[5];
        int ret = 0;
        uint32_t status;

        //printk("gurnard_nand_write_page: to=0x%llx%s%s\n", to,
                        //buf ? " data" : "", oob ? " oob" : "");

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
        /* OOB only read, so skip past the page to the oob */
        if (!buf) {
                addr[0] = (mtd->writesize / 4);
                addr[1] = (mtd->writesize / 4) >> 8;
        }

        gurnard_write_command(host, NAND_CMD_SEQIN);
        gurnard_write_address(host, addr, 5);
        gurnard_rnb_wait(host);
        if (buf) {
                gurnard_write_buf(host, (uint32_t *)buf, mtd->writesize);
                if (auto_ecc && oob) {
                        uint32_t *ecc;
                        int i;
                        ecc = (uint32_t *)&oob[mtd->ecclayout->eccpos[0]];
                        for (i = 0; i < mtd->ecclayout->eccbytes / 4; i++)
                                *ecc++ = reg_readl(ECC);
                }
        }
        if (oob)
                gurnard_write_buf(host, (uint32_t *)oob, mtd->oobsize);

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

        return ret;
}

static int gurnard_nand_read(struct mtd_info *mtd, loff_t from, size_t len,
                size_t *retlen, uint8_t *buf)
{
        struct gurnard_nand_stack *stack = mtd->priv;
        struct gurnard_nand_host *host = stack->host;
        uint64_t cur_addr = from;

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

        while (cur_addr < from + len) {
                gurnard_nand_read_page(mtd, cur_addr, buf, NULL, -1);
                buf += mtd->writesize;
                cur_addr += mtd->writesize;
        }
        *retlen = len;

        return 0;
}

static int gurnard_nand_write_oob(struct mtd_info *mtd, loff_t to,
                struct mtd_oob_ops *ops)
{
        struct gurnard_nand_stack *stack = mtd->priv;
        struct gurnard_nand_host *host = stack->host;
        uint8_t *buf, *oob;
        uint64_t len, cur_addr;
        int auto_ecc = (ops->mode != MTD_OOB_RAW);
        int ret = 0;

        if (to & (mtd->writesize - 1)) {
                dev_err(&host->pdev->dev,
                        "OOB Write to non page-aligned address 0x%llx\n",
                        to);
                return -EINVAL;
        }

        if (ops->datbuf)
                len = ops->len;
        else if (ops->oobbuf) {
                if (ops->ooblen % mtd->oobsize) {
                        dev_err(&host->pdev->dev,
                                "OOB Write with invalid OOB length: %d\n",
                                ops->ooblen);
                        return -EINVAL;
                }
                len = (ops->ooblen / mtd->oobsize) * mtd->writesize;
        } else
                return -EINVAL;

        if (to + len > mtd->size)
                return -EINVAL;
        if (!len)
                return 0;
        oob = ops->oobbuf;
        buf = ops->datbuf;

        cur_addr = to;

        while (cur_addr < to + len && !ret) {
                ret = gurnard_nand_write_page(mtd, cur_addr, buf, oob, auto_ecc);
                if (buf)
                        buf += mtd->writesize;
                if (oob)
                        oob += mtd->oobsize;
                cur_addr += mtd->writesize;
        }

        ops->retlen = buf - ops->datbuf;
        ops->oobretlen = oob - ops->oobbuf;

        return ret;
}

static int gurnard_nand_read_oob(struct mtd_info *mtd, loff_t from,
                struct mtd_oob_ops *ops)
{
        struct gurnard_nand_stack *stack = mtd->priv;
        struct gurnard_nand_host *host = stack->host;
        uint64_t cur_addr = from;
        uint8_t *buf, *oob;
        uint64_t len;

        if (ops->datbuf)
                len = ops->len;
        else if (ops->oobbuf)
                len = (ops->ooblen / mtd->oobsize) * mtd->writesize;
        else
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
        oob = ops->oobbuf;
        buf = ops->datbuf;

        while (cur_addr < from + len) {
                gurnard_nand_read_page(mtd, cur_addr, buf, oob, -1);
                if (buf)
                        buf += mtd->writesize;
                if (oob)
                        oob += mtd->oobsize;
                cur_addr += mtd->writesize;
        }

        ops->retlen = buf - ops->datbuf;
        ops->oobretlen = oob - ops->oobbuf;

        return 0;

}

static int gurnard_block_is_bad(struct mtd_info *mtd, loff_t offs)
{
        uint32_t oob[mtd->oobsize / 4];
        int ret;

        if (offs > mtd->size)
                return -EINVAL;

        /* We're only interested in the page at the beginning of the block */
        offs = offs & ~(mtd->erasesize - 1);

        ret = gurnard_nand_read_page(mtd, offs, NULL, (uint8_t *)oob, 4);
        if (ret)
                return ret;
        //if (oob[0] != 0xffffffff)
                //printk("block_is_bad: 0x%llx=0x%x\n", offs, oob[0]);
        return (oob[0] != 0xffffffff);
}

static int gurnard_block_mark_bad(struct mtd_info *mtd, loff_t offs)
{
        uint32_t oob[mtd->oobsize / 4];

        memset(oob, 0xff, sizeof(oob));
        oob[0] = 0;

        return gurnard_nand_write_page(mtd, offs, NULL, (uint8_t *)oob, 0);
}

static int gurnard_nand_write(struct mtd_info *mtd, loff_t to, size_t len,
                size_t *retlen, const uint8_t *buf)
{
        struct gurnard_nand_stack *stack = mtd->priv;
        struct gurnard_nand_host *host = stack->host;
        uint64_t cur_addr = to;
        uint8_t oob[mtd->oobsize];
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
        memset(oob, 0xff, mtd->oobsize);
        while (cur_addr < to + len) {
                ret = gurnard_nand_write_page(mtd, cur_addr, buf, oob, 1);
                if (ret < 0)
                        break;
                buf += mtd->writesize;
                cur_addr += mtd->writesize;
        }
        *retlen = cur_addr - to;

        return ret;
}

static ssize_t force_probe(struct device *dev, struct device_attribute *attr,
                           const char *buf, size_t count)
{
        struct platform_device *pdev = to_platform_device(dev);
        struct gurnard_nand_host *host = platform_get_drvdata(pdev);
        int i;
        int res;
        for (i = 0; i < MAX_STACKS; i++) {
                struct gurnard_nand_stack *stack = &host->stack[i];
                struct mtd_info *mtd = &stack->mtd;
                uint8_t mfr, device, cellinfo, busw;
                uint32_t writesize, oobsize, erasesize;
                uint64_t size = 0;
                int addr = 1 << i;

                memset(stack, 0, sizeof(*stack));

                stack->host = host;
                stack->device_addr = addr;

                if (gurnard_read_id(host, addr, &mfr, &device, &cellinfo,
                                        &writesize, &oobsize, &erasesize,
                                        &busw) < 0)
                        continue;
                if (busw != 8) {
                        dev_err(dev, "Only 8-bit devices supported\n");
                        continue;
                }
                for (i = 0; nand_flash_ids[i].name != NULL; i++) {
                        if (device == nand_flash_ids[i].id)
                                size = nand_flash_ids[i].chipsize << 20;
                }
                if (size == 0) {
                        dev_err(dev, "Can't find nand record for device 0x%x\n",
                                        device);
                        continue;
                }

                mtd->priv = stack;
                snprintf(stack->name, sizeof(stack->name), "Stack %d", i);
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
                        dev_err(dev, "Invalid device page size: %d\n",
                                        writesize);
                        continue;
                }

                /* Since the banks are multi-plexed up 4-ways,
                 * everything is 4 times bigger
                 */
                mtd->writesize = writesize * 4;
                mtd->erasesize = erasesize * 4;
                mtd->size = size * 4;
                // cap it for testing to 64MB
                //mtd->size = 64 * 1024 * 1024;
                mtd->oobsize = oobsize * 4;

                res = add_mtd_partitions(mtd, gurnard_partitions,
                                ARRAY_SIZE(gurnard_partitions));
                if (res)
                        dev_err(dev, "Can't add MTD partitions\n");
        }

        return count;
}

static ssize_t read_id_0(struct device *dev, struct device_attribute *attr,
                char *buf)
{
        struct platform_device *pdev = to_platform_device(dev);
        struct gurnard_nand_host *host = platform_get_drvdata(pdev);
        uint8_t mfr, device, cellinfo, busw;
        uint32_t writesize, oobsize, erasesize;

        if (gurnard_read_id(host, 1, &mfr, &device, &cellinfo, &writesize,
                                &oobsize, &erasesize, &busw) < 0)
                return 0;

        return sprintf(buf, "mfr = 0x%x\ndevice=0x%x\nwritesize=0x%x\noobsize=0x%x\nerasesize=0x%x\nbusw=%d\n",
                        mfr, device, writesize, oobsize, erasesize, busw);
}

static ssize_t read_id_1(struct device *dev, struct device_attribute *attr,
                char *buf)
{
        struct platform_device *pdev = to_platform_device(dev);
        struct gurnard_nand_host *host = platform_get_drvdata(pdev);
        uint8_t mfr, device, cellinfo, busw;
        uint32_t writesize, oobsize, erasesize;

        if (gurnard_read_id(host, 2, &mfr, &device, &cellinfo, &writesize,
                                &oobsize, &erasesize, &busw) < 0)
                return 0;

        return sprintf(buf, "mfr = 0x%x\ndevice=0x%x\nwritesize=0x%x\noobsize=0x%x\nerasesize=0x%x\nbusw=%d\n",
                        mfr, device, writesize, oobsize, erasesize, busw);
}

static ssize_t reset(struct device *dev, struct device_attribute *attr,
                           const char *buf, size_t count)
{
        struct platform_device *pdev = to_platform_device(dev);
        struct gurnard_nand_host *host = platform_get_drvdata(pdev);
        gurnard_reset(host, 3);
        return count;
}

static DEVICE_ATTR(probe, S_IWUSR, NULL, force_probe);
static DEVICE_ATTR(id0, S_IRUGO, read_id_0, NULL);
static DEVICE_ATTR(id1, S_IRUGO, read_id_1, NULL);
static DEVICE_ATTR(reset, S_IWUSR, NULL, reset);

static struct attribute *gurnard_attributes[] = {
        &dev_attr_probe.attr,
        &dev_attr_reset.attr,
        &dev_attr_id0.attr,
        &dev_attr_id1.attr,
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
