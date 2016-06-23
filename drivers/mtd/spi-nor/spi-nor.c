/*
 * Based on m25p80.c, by Mike Lavender (mike@steroidmicros.com), with
 * influence from lart.c (Abraham Van Der Merwe) and mtd_dataflash.c
 *
 * Copyright (C) 2005, Intec Automation Inc.
 * Copyright (C) 2014, Freescale Semiconductor, Inc.
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/math64.h>
#include <linux/sizes.h>
#include <linux/slab.h>

#include <linux/mtd/mtd.h>
#include <linux/of_platform.h>
#include <linux/spi/flash.h>
#include <linux/mtd/spi-nor.h>

/* Define max times to check status register before we give up. */

/*
 * For everything but full-chip erase; probably could be much smaller, but kept
 * around for safety for now
 */
#define DEFAULT_READY_WAIT_JIFFIES		(40UL * HZ)

/*
 * For full-chip erase, calibrated to a 2MB flash (M25P16); should be scaled up
 * for larger flash
 */
#define CHIP_ERASE_2MB_READY_WAIT_JIFFIES	(40UL * HZ)

#define SPI_NOR_MAX_ID_LEN	6

struct flash_info {
	char		*name;

	/*
	 * This array stores the ID bytes.
	 * The first three bytes are the JEDIC ID.
	 * JEDEC ID zero means "no ID" (mostly older chips).
	 */
	u8		id[SPI_NOR_MAX_ID_LEN];
	u8		id_len;

	/* The size listed here is what works with SPINOR_OP_SE, which isn't
	 * necessarily called a "sector" by the vendor.
	 */
	unsigned	sector_size;
	u16		n_sectors;

	u16		page_size;
	u16		addr_width;

	u16		flags;
#define	SECT_4K			0x01	/* SPINOR_OP_BE_4K works uniformly */
#define	SPI_NOR_NO_ERASE	0x02	/* No erase command needed */
#define	SST_WRITE		0x04	/* use SST byte programming */
#define	SPI_NOR_NO_FR		0x08	/* Can't do fastread */
#define	SECT_4K_PMC		0x10	/* SPINOR_OP_BE_4K_PMC works uniformly */
#define	SPI_NOR_DUAL_READ	0x20    /* Flash supports Dual Read */
#define	SPI_NOR_QUAD_READ	0x40    /* Flash supports Quad Read */
#define	USE_FSR			0x80	/* use flag status register */
#define SPI_NOR_4B_OPCODES	BIT(10)	/*
					 * Use dedicated 4byte address op codes
					 * to support memory size above 128Mib.
					 */
#define SPI_NOR_SKIP_SFDP	BIT(11) /* Skip read of SFDP tables */
};

#define JEDEC_MFR(info)	((info)->id[0])

static const struct flash_info *spi_nor_match_id(const char *name);

/*
 * Read the status register, returning its value in the location
 * Return the status register value.
 * Returns negative if error occurred.
 */
static int read_sr(struct spi_nor *nor)
{
	int ret;
	u8 val;

	ret = nor->read_reg(nor, SPINOR_OP_RDSR, &val, 1);
	if (ret < 0) {
		pr_err("error %d reading SR\n", (int) ret);
		return ret;
	}

	return val;
}

/*
 * Read the flag status register, returning its value in the location
 * Return the status register value.
 * Returns negative if error occurred.
 */
static int read_fsr(struct spi_nor *nor)
{
	int ret;
	u8 val;

	ret = nor->read_reg(nor, SPINOR_OP_RDFSR, &val, 1);
	if (ret < 0) {
		pr_err("error %d reading FSR\n", ret);
		return ret;
	}

	return val;
}

/*
 * Read configuration register, returning its value in the
 * location. Return the configuration register value.
 * Returns negative if error occured.
 */
static int read_cr(struct spi_nor *nor)
{
	int ret;
	u8 val;

	ret = nor->read_reg(nor, SPINOR_OP_RDCR, &val, 1);
	if (ret < 0) {
		dev_err(nor->dev, "error %d reading CR\n", ret);
		return ret;
	}

	return val;
}

/*
 * Write status register 1 byte
 * Returns negative if error occurred.
 */
static inline int write_sr(struct spi_nor *nor, u8 val)
{
	nor->cmd_buf[0] = val;
	return nor->write_reg(nor, SPINOR_OP_WRSR, nor->cmd_buf, 1);
}

/*
 * Set write enable latch with Write Enable command.
 * Returns negative if error occurred.
 */
static inline int write_enable(struct spi_nor *nor)
{
	return nor->write_reg(nor, SPINOR_OP_WREN, NULL, 0);
}

/*
 * Send write disble instruction to the chip.
 */
static inline int write_disable(struct spi_nor *nor)
{
	return nor->write_reg(nor, SPINOR_OP_WRDI, NULL, 0);
}

static inline struct spi_nor *mtd_to_spi_nor(struct mtd_info *mtd)
{
	return mtd->priv;
}


struct spi_nor_address_entry {
	u8	src_opcode;
	u8	dst_opcode;
};

static u8 spi_nor_convert_opcode(u8 opcode,
				 const struct spi_nor_address_entry *entries,
				 size_t num_entries)
{
	int min, max;

	min = 0;
	max = num_entries - 1;
	while (min <= max) {
		int mid = (min + max) >> 1;
		const struct spi_nor_address_entry *entry = &entries[mid];

		if (opcode == entry->src_opcode)
			return entry->dst_opcode;

		if (opcode < entry->src_opcode)
			max = mid - 1;
		else
			min = mid + 1;
	}

	/* No conversion found */
	return opcode;
}

static u8 spi_nor_3to4_opcode(u8 opcode)
{
	/* MUST be sorted by 3byte opcode */
#define ENTRY_3TO4(_opcode)	{ _opcode, _opcode##_4B }
	static const struct spi_nor_address_entry spi_nor_3to4_table[] = {
		ENTRY_3TO4(SPINOR_OP_PP),		/* 0x02 */
		ENTRY_3TO4(SPINOR_OP_READ),		/* 0x03 */
		ENTRY_3TO4(SPINOR_OP_READ_FAST),	/* 0x0b */
		ENTRY_3TO4(SPINOR_OP_BE_4K),		/* 0x20 */
		ENTRY_3TO4(SPINOR_OP_PP_1_1_4),		/* 0x32 */
		ENTRY_3TO4(SPINOR_OP_PP_1_4_4),		/* 0x38 */
		ENTRY_3TO4(SPINOR_OP_READ_1_1_2),	/* 0x3b */
		ENTRY_3TO4(SPINOR_OP_BE_32K),		/* 0x52 */
		ENTRY_3TO4(SPINOR_OP_READ_1_1_4),	/* 0x6b */
		ENTRY_3TO4(SPINOR_OP_READ_1_2_2),	/* 0xbb */
		ENTRY_3TO4(SPINOR_OP_SE),		/* 0xd8 */
		ENTRY_3TO4(SPINOR_OP_READ_1_4_4),	/* 0xeb */
	};
#undef ENTRY_3TO4

	return spi_nor_convert_opcode(opcode, spi_nor_3to4_table,
				      ARRAY_SIZE(spi_nor_3to4_table));
}

static void spi_nor_set_4byte_opcodes(struct spi_nor *nor,
				      const struct flash_info *info)
{
	/* Do some manufacturer fixups first */
	switch (JEDEC_MFR(info)) {
	case SNOR_MFR_SPANSION:
		/* No small sector erase for 4-byte command set */
		nor->erase_opcode = SPINOR_OP_SE;
		nor->mtd.erasesize = info->sector_size;
		break;

	default:
		break;
	}

	nor->read_opcode	= spi_nor_3to4_opcode(nor->read_opcode);
	nor->program_opcode	= spi_nor_3to4_opcode(nor->program_opcode);
	nor->erase_opcode	= spi_nor_3to4_opcode(nor->erase_opcode);
}

/* Enable/disable 4-byte addressing mode. */
static inline int set_4byte(struct spi_nor *nor, const struct flash_info *info,
			    int enable)
{
	int status;
	bool need_wren = false;
	u8 cmd;

	switch (JEDEC_MFR(info)) {
	case SNOR_MFR_MICRON:
		/* Some Micron need WREN command; all will accept it */
		need_wren = true;
	case SNOR_MFR_MACRONIX:
	case SNOR_MFR_WINBOND:
		if (need_wren)
			write_enable(nor);

		cmd = enable ? SPINOR_OP_EN4B : SPINOR_OP_EX4B;
		status = nor->write_reg(nor, cmd, NULL, 0);
		if (need_wren)
			write_disable(nor);

		return status;
	default:
		/* Spansion style */
		nor->cmd_buf[0] = enable << 7;
		return nor->write_reg(nor, SPINOR_OP_BRWR, nor->cmd_buf, 1);
	}
}
static inline int spi_nor_sr_ready(struct spi_nor *nor)
{
	int sr = read_sr(nor);
	if (sr < 0)
		return sr;
	else
		return !(sr & SR_WIP);
}

static inline int spi_nor_fsr_ready(struct spi_nor *nor)
{
	int fsr = read_fsr(nor);
	if (fsr < 0)
		return fsr;
	else
		return fsr & FSR_READY;
}

static int spi_nor_ready(struct spi_nor *nor)
{
	int sr, fsr;
	sr = spi_nor_sr_ready(nor);
	if (sr < 0)
		return sr;
	fsr = nor->flags & SNOR_F_USE_FSR ? spi_nor_fsr_ready(nor) : 1;
	if (fsr < 0)
		return fsr;
	return sr && fsr;
}

/*
 * Service routine to read status register until ready, or timeout occurs.
 * Returns non-zero if error.
 */
static int spi_nor_wait_till_ready_with_timeout(struct spi_nor *nor,
						unsigned long timeout_jiffies)
{
	unsigned long deadline;
	int timeout = 0, ret;

	deadline = jiffies + timeout_jiffies;

	while (!timeout) {
		if (time_after_eq(jiffies, deadline))
			timeout = 1;

		ret = spi_nor_ready(nor);
		if (ret < 0)
			return ret;
		if (ret)
			return 0;

		cond_resched();
	}

	dev_err(nor->dev, "flash operation timed out\n");

	return -ETIMEDOUT;
}

static int spi_nor_wait_till_ready(struct spi_nor *nor)
{
	return spi_nor_wait_till_ready_with_timeout(nor,
						    DEFAULT_READY_WAIT_JIFFIES);
}

/*
 * Erase the whole flash memory
 *
 * Returns 0 if successful, non-zero otherwise.
 */
static int erase_chip(struct spi_nor *nor)
{
	dev_dbg(nor->dev, " %lldKiB\n", (long long)(nor->mtd.size >> 10));

	return nor->write_reg(nor, SPINOR_OP_CHIP_ERASE, NULL, 0);
}

static int spi_nor_lock_and_prep(struct spi_nor *nor, enum spi_nor_ops ops)
{
	int ret = 0;

	mutex_lock(&nor->lock);

	if (nor->prepare) {
		ret = nor->prepare(nor, ops);
		if (ret) {
			dev_err(nor->dev, "failed in the preparation.\n");
			mutex_unlock(&nor->lock);
			return ret;
		}
	}
	return ret;
}

static void spi_nor_unlock_and_unprep(struct spi_nor *nor, enum spi_nor_ops ops)
{
	if (nor->unprepare)
		nor->unprepare(nor, ops);
	mutex_unlock(&nor->lock);
}

/*
 * Erase an address range on the nor chip.  The address range may extend
 * one or more erase sectors.  Return an error is there is a problem erasing.
 */
static int spi_nor_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct spi_nor *nor = mtd_to_spi_nor(mtd);
	u32 addr, len;
	uint32_t rem;
	int ret;

	dev_dbg(nor->dev, "at 0x%llx, len %lld\n", (long long)instr->addr,
			(long long)instr->len);

	div_u64_rem(instr->len, mtd->erasesize, &rem);
	if (rem)
		return -EINVAL;

	addr = instr->addr;
	len = instr->len;

	ret = spi_nor_lock_and_prep(nor, SPI_NOR_OPS_ERASE);
	if (ret)
		return ret;

	/* whole-chip erase? */
	if (len == mtd->size) {
		unsigned long timeout;

		write_enable(nor);

		if (erase_chip(nor)) {
			ret = -EIO;
			goto erase_err;
		}

		/*
		 * Scale the timeout linearly with the size of the flash, with
		 * a minimum calibrated to an old 2MB flash. We could try to
		 * pull these from CFI/SFDP, but these values should be good
		 * enough for now.
		 */
		timeout = max(CHIP_ERASE_2MB_READY_WAIT_JIFFIES,
			      CHIP_ERASE_2MB_READY_WAIT_JIFFIES *
			      (unsigned long)(mtd->size / SZ_2M));
		ret = spi_nor_wait_till_ready_with_timeout(nor, timeout);
		if (ret)
			goto erase_err;

	/* REVISIT in some cases we could speed up erasing large regions
	 * by using SPINOR_OP_SE instead of SPINOR_OP_BE_4K.  We may have set up
	 * to use "small sector erase", but that's not always optimal.
	 */

	/* "sector"-at-a-time erase */
	} else {
		while (len) {
			write_enable(nor);

			if (nor->erase(nor, addr)) {
				ret = -EIO;
				goto erase_err;
			}

			addr += mtd->erasesize;
			len -= mtd->erasesize;

			ret = spi_nor_wait_till_ready(nor);
			if (ret)
				goto erase_err;
		}
	}

	write_disable(nor);

	spi_nor_unlock_and_unprep(nor, SPI_NOR_OPS_ERASE);

	instr->state = MTD_ERASE_DONE;
	mtd_erase_callback(instr);

	return ret;

erase_err:
	spi_nor_unlock_and_unprep(nor, SPI_NOR_OPS_ERASE);
	instr->state = MTD_ERASE_FAILED;
	return ret;
}

static void stm_get_locked_range(struct spi_nor *nor, u8 sr, loff_t *ofs,
				 uint64_t *len)
{
	struct mtd_info *mtd = &nor->mtd;
	u8 mask = SR_BP2 | SR_BP1 | SR_BP0;
	int shift = ffs(mask) - 1;
	int pow;

	if (!(sr & mask)) {
		/* No protection */
		*ofs = 0;
		*len = 0;
	} else {
		pow = ((sr & mask) ^ mask) >> shift;
		*len = mtd->size >> pow;
		*ofs = mtd->size - *len;
	}
}

/*
 * Return 1 if the entire region is locked, 0 otherwise
 */
static int stm_is_locked_sr(struct spi_nor *nor, loff_t ofs, uint64_t len,
			    u8 sr)
{
	loff_t lock_offs;
	uint64_t lock_len;

	stm_get_locked_range(nor, sr, &lock_offs, &lock_len);

	return (ofs + len <= lock_offs + lock_len) && (ofs >= lock_offs);
}

/*
 * Lock a region of the flash. Compatible with ST Micro and similar flash.
 * Supports only the block protection bits BP{0,1,2} in the status register
 * (SR). Does not support these features found in newer SR bitfields:
 *   - TB: top/bottom protect - only handle TB=0 (top protect)
 *   - SEC: sector/block protect - only handle SEC=0 (block protect)
 *   - CMP: complement protect - only support CMP=0 (range is not complemented)
 *
 * Sample table portion for 8MB flash (Winbond w25q64fw):
 *
 *   SEC  |  TB   |  BP2  |  BP1  |  BP0  |  Prot Length  | Protected Portion
 *  --------------------------------------------------------------------------
 *    X   |   X   |   0   |   0   |   0   |  NONE         | NONE
 *    0   |   0   |   0   |   0   |   1   |  128 KB       | Upper 1/64
 *    0   |   0   |   0   |   1   |   0   |  256 KB       | Upper 1/32
 *    0   |   0   |   0   |   1   |   1   |  512 KB       | Upper 1/16
 *    0   |   0   |   1   |   0   |   0   |  1 MB         | Upper 1/8
 *    0   |   0   |   1   |   0   |   1   |  2 MB         | Upper 1/4
 *    0   |   0   |   1   |   1   |   0   |  4 MB         | Upper 1/2
 *    X   |   X   |   1   |   1   |   1   |  8 MB         | ALL
 *
 * Returns negative on errors, 0 on success.
 */
static int stm_lock(struct spi_nor *nor, loff_t ofs, uint64_t len)
{
	struct mtd_info *mtd = &nor->mtd;
	u8 status_old, status_new;
	u8 mask = SR_BP2 | SR_BP1 | SR_BP0;
	u8 shift = ffs(mask) - 1, pow, val;

	status_old = read_sr(nor);

	/* SPI NOR always locks to the end */
	if (ofs + len != mtd->size) {
		/* Does combined region extend to end? */
		if (!stm_is_locked_sr(nor, ofs + len, mtd->size - ofs - len,
				      status_old))
			return -EINVAL;
		len = mtd->size - ofs;
	}

	/*
	 * Need smallest pow such that:
	 *
	 *   1 / (2^pow) <= (len / size)
	 *
	 * so (assuming power-of-2 size) we do:
	 *
	 *   pow = ceil(log2(size / len)) = log2(size) - floor(log2(len))
	 */
	pow = ilog2(mtd->size) - ilog2(len);
	val = mask - (pow << shift);
	if (val & ~mask)
		return -EINVAL;
	/* Don't "lock" with no region! */
	if (!(val & mask))
		return -EINVAL;

	status_new = (status_old & ~mask) | val;

	/* Only modify protection if it will not unlock other areas */
	if ((status_new & mask) <= (status_old & mask))
		return -EINVAL;

	write_enable(nor);
	return write_sr(nor, status_new);
}

/*
 * Unlock a region of the flash. See stm_lock() for more info
 *
 * Returns negative on errors, 0 on success.
 */
static int stm_unlock(struct spi_nor *nor, loff_t ofs, uint64_t len)
{
	struct mtd_info *mtd = &nor->mtd;
	uint8_t status_old, status_new;
	u8 mask = SR_BP2 | SR_BP1 | SR_BP0;
	u8 shift = ffs(mask) - 1, pow, val;

	status_old = read_sr(nor);

	/* Cannot unlock; would unlock larger region than requested */
	if (stm_is_locked_sr(nor, ofs - mtd->erasesize, mtd->erasesize,
			     status_old))
		return -EINVAL;

	/*
	 * Need largest pow such that:
	 *
	 *   1 / (2^pow) >= (len / size)
	 *
	 * so (assuming power-of-2 size) we do:
	 *
	 *   pow = floor(log2(size / len)) = log2(size) - ceil(log2(len))
	 */
	pow = ilog2(mtd->size) - order_base_2(mtd->size - (ofs + len));
	if (ofs + len == mtd->size) {
		val = 0; /* fully unlocked */
	} else {
		val = mask - (pow << shift);
		/* Some power-of-two sizes are not supported */
		if (val & ~mask)
			return -EINVAL;
	}

	status_new = (status_old & ~mask) | val;

	/* Only modify protection if it will not lock other areas */
	if ((status_new & mask) >= (status_old & mask))
		return -EINVAL;

	write_enable(nor);
	return write_sr(nor, status_new);
}

/*
 * Check if a region of the flash is (completely) locked. See stm_lock() for
 * more info.
 *
 * Returns 1 if entire region is locked, 0 if any portion is unlocked, and
 * negative on errors.
 */
static int stm_is_locked(struct spi_nor *nor, loff_t ofs, uint64_t len)
{
	int status;

	status = read_sr(nor);
	if (status < 0)
		return status;

	return stm_is_locked_sr(nor, ofs, len, status);
}

static int spi_nor_lock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	struct spi_nor *nor = mtd_to_spi_nor(mtd);
	int ret;

	ret = spi_nor_lock_and_prep(nor, SPI_NOR_OPS_LOCK);
	if (ret)
		return ret;

	ret = nor->flash_lock(nor, ofs, len);

	spi_nor_unlock_and_unprep(nor, SPI_NOR_OPS_UNLOCK);
	return ret;
}

static int spi_nor_unlock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	struct spi_nor *nor = mtd_to_spi_nor(mtd);
	int ret;

	ret = spi_nor_lock_and_prep(nor, SPI_NOR_OPS_UNLOCK);
	if (ret)
		return ret;

	ret = nor->flash_unlock(nor, ofs, len);

	spi_nor_unlock_and_unprep(nor, SPI_NOR_OPS_LOCK);
	return ret;
}

static int spi_nor_is_locked(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	struct spi_nor *nor = mtd_to_spi_nor(mtd);
	int ret;

	ret = spi_nor_lock_and_prep(nor, SPI_NOR_OPS_UNLOCK);
	if (ret)
		return ret;

	ret = nor->flash_is_locked(nor, ofs, len);

	spi_nor_unlock_and_unprep(nor, SPI_NOR_OPS_LOCK);
	return ret;
}

/* Used when the "_ext_id" is two bytes at most */
#define INFO(_jedec_id, _ext_id, _sector_size, _n_sectors, _flags)	\
		.id = {							\
			((_jedec_id) >> 16) & 0xff,			\
			((_jedec_id) >> 8) & 0xff,			\
			(_jedec_id) & 0xff,				\
			((_ext_id) >> 8) & 0xff,			\
			(_ext_id) & 0xff,				\
			},						\
		.id_len = (!(_jedec_id) ? 0 : (3 + ((_ext_id) ? 2 : 0))),	\
		.sector_size = (_sector_size),				\
		.n_sectors = (_n_sectors),				\
		.page_size = 256,					\
		.flags = (_flags),

#define INFO6(_jedec_id, _ext_id, _sector_size, _n_sectors, _flags)	\
		.id = {							\
			((_jedec_id) >> 16) & 0xff,			\
			((_jedec_id) >> 8) & 0xff,			\
			(_jedec_id) & 0xff,				\
			((_ext_id) >> 16) & 0xff,			\
			((_ext_id) >> 8) & 0xff,			\
			(_ext_id) & 0xff,				\
			},						\
		.id_len = 6,						\
		.sector_size = (_sector_size),				\
		.n_sectors = (_n_sectors),				\
		.page_size = 256,					\
		.flags = (_flags),

#define CAT25_INFO(_sector_size, _n_sectors, _page_size, _addr_width, _flags)	\
		.sector_size = (_sector_size),				\
		.n_sectors = (_n_sectors),				\
		.page_size = (_page_size),				\
		.addr_width = (_addr_width),				\
		.flags = (_flags),

/* NOTE: double check command sets and memory organization when you add
 * more nor chips.  This current list focusses on newer chips, which
 * have been converging on command sets which including JEDEC ID.
 *
 * All newly added entries should describe *hardware* and should use SECT_4K
 * (or SECT_4K_PMC) if hardware supports erasing 4 KiB sectors. For usage
 * scenarios excluding small sectors there is config option that can be
 * disabled: CONFIG_MTD_SPI_NOR_USE_4K_SECTORS.
 * For historical (and compatibility) reasons (before we got above config) some
 * old entries may be missing 4K flag.
 */
static const struct flash_info spi_nor_ids[] = {
	/* Atmel -- some are (confusingly) marketed as "DataFlash" */
	{ "at25fs010",  INFO(0x1f6601, 0, 32 * 1024,   4, SECT_4K) },
	{ "at25fs040",  INFO(0x1f6604, 0, 64 * 1024,   8, SECT_4K) },

	{ "at25df041a", INFO(0x1f4401, 0, 64 * 1024,   8, SECT_4K) },
	{ "at25df321a", INFO(0x1f4701, 0, 64 * 1024,  64, SECT_4K) },
	{ "at25df641",  INFO(0x1f4800, 0, 64 * 1024, 128, SECT_4K) },

	{ "at26f004",   INFO(0x1f0400, 0, 64 * 1024,  8, SECT_4K) },
	{ "at26df081a", INFO(0x1f4501, 0, 64 * 1024, 16, SECT_4K) },
	{ "at26df161a", INFO(0x1f4601, 0, 64 * 1024, 32, SECT_4K) },
	{ "at26df321",  INFO(0x1f4700, 0, 64 * 1024, 64, SECT_4K) },

	{ "at45db081d", INFO(0x1f2500, 0, 64 * 1024, 16, SECT_4K) },

	/* EON -- en25xxx */
	{ "en25f32",    INFO(0x1c3116, 0, 64 * 1024,   64, SECT_4K) },
	{ "en25p32",    INFO(0x1c2016, 0, 64 * 1024,   64, 0) },
	{ "en25q32b",   INFO(0x1c3016, 0, 64 * 1024,   64, 0) },
	{ "en25p64",    INFO(0x1c2017, 0, 64 * 1024,  128, 0) },
	{ "en25q64",    INFO(0x1c3017, 0, 64 * 1024,  128, SECT_4K) },
	{ "en25qh128",  INFO(0x1c7018, 0, 64 * 1024,  256, 0) },
	{ "en25qh256",  INFO(0x1c7019, 0, 64 * 1024,  512, 0) },
	{ "en25s64",	INFO(0x1c3817, 0, 64 * 1024,  128, SECT_4K) },

	/* ESMT */
	{ "f25l32pa", INFO(0x8c2016, 0, 64 * 1024, 64, SECT_4K) },

	/* Everspin */
	{ "mr25h256", CAT25_INFO( 32 * 1024, 1, 256, 2, SPI_NOR_NO_ERASE | SPI_NOR_NO_FR) },
	{ "mr25h10",  CAT25_INFO(128 * 1024, 1, 256, 3, SPI_NOR_NO_ERASE | SPI_NOR_NO_FR) },

	/* Fujitsu */
	{ "mb85rs1mt", INFO(0x047f27, 0, 128 * 1024, 1, SPI_NOR_NO_ERASE) },

	/* GigaDevice */
	{ "gd25q32", INFO(0xc84016, 0, 64 * 1024,  64, SECT_4K) },
	{ "gd25q64", INFO(0xc84017, 0, 64 * 1024, 128, SECT_4K) },
	{ "gd25q128", INFO(0xc84018, 0, 64 * 1024, 256, SECT_4K) },

	/* Intel/Numonyx -- xxxs33b */
	{ "160s33b",  INFO(0x898911, 0, 64 * 1024,  32, 0) },
	{ "320s33b",  INFO(0x898912, 0, 64 * 1024,  64, 0) },
	{ "640s33b",  INFO(0x898913, 0, 64 * 1024, 128, 0) },

	/* ISSI */
	{ "is25cd512", INFO(0x7f9d20, 0, 32 * 1024,   2, SECT_4K) },

	/* Macronix */
	{ "mx25l512e",   INFO(0xc22010, 0, 64 * 1024,   1, SECT_4K) },
	{ "mx25l2005a",  INFO(0xc22012, 0, 64 * 1024,   4, SECT_4K) },
	{ "mx25l4005a",  INFO(0xc22013, 0, 64 * 1024,   8, SECT_4K) },
	{ "mx25l8005",   INFO(0xc22014, 0, 64 * 1024,  16, 0) },
	{ "mx25l1606e",  INFO(0xc22015, 0, 64 * 1024,  32, SECT_4K) },
	{ "mx25l3205d",  INFO(0xc22016, 0, 64 * 1024,  64, 0) },
	{ "mx25l3255e",  INFO(0xc29e16, 0, 64 * 1024,  64, SECT_4K) },
	{ "mx25l6405d",  INFO(0xc22017, 0, 64 * 1024, 128, 0) },
	{ "mx25u6435f",  INFO(0xc22537, 0, 64 * 1024, 128, SECT_4K) },
	{ "mx25l12805d", INFO(0xc22018, 0, 64 * 1024, 256, 0) },
	{ "mx25l12855e", INFO(0xc22618, 0, 64 * 1024, 256, 0) },
	{ "mx25l25635e", INFO(0xc22019, 0, 64 * 1024, 512, 0) },
	{ "mx25l25655e", INFO(0xc22619, 0, 64 * 1024, 512, 0) },
	{ "mx66l51235l", INFO(0xc2201a, 0, 64 * 1024, 1024, SPI_NOR_QUAD_READ) },
	{ "mx66l1g55g",  INFO(0xc2261b, 0, 64 * 1024, 2048, SPI_NOR_QUAD_READ) },

	/* Micron */
	{ "n25q032",	 INFO(0x20ba16, 0, 64 * 1024,   64, SPI_NOR_QUAD_READ) },
	{ "n25q032a",	 INFO(0x20bb16, 0, 64 * 1024,   64, SPI_NOR_QUAD_READ) },
	{ "n25q064",     INFO(0x20ba17, 0, 64 * 1024,  128, SECT_4K | SPI_NOR_QUAD_READ) },
	{ "n25q064a",    INFO(0x20bb17, 0, 64 * 1024,  128, SECT_4K | SPI_NOR_QUAD_READ) },
	{ "n25q128a11",  INFO(0x20bb18, 0, 64 * 1024,  256, SPI_NOR_QUAD_READ) },
	{ "n25q128a13",  INFO(0x20ba18, 0, 64 * 1024,  256, SPI_NOR_QUAD_READ) },
	{ "n25q256a",    INFO(0x20ba19, 0, 64 * 1024,  512, SECT_4K | SPI_NOR_QUAD_READ) },
	{ "n25q512a",    INFO(0x20bb20, 0, 64 * 1024, 1024, SECT_4K | USE_FSR | SPI_NOR_QUAD_READ) },
	{ "n25q512ax3",  INFO(0x20ba20, 0, 64 * 1024, 1024, SECT_4K | USE_FSR | SPI_NOR_QUAD_READ) },
	{ "n25q00",      INFO(0x20ba21, 0, 64 * 1024, 2048, SECT_4K | USE_FSR | SPI_NOR_QUAD_READ) },

	/* PMC */
	{ "pm25lv512",   INFO(0,        0, 32 * 1024,    2, SECT_4K_PMC) },
	{ "pm25lv010",   INFO(0,        0, 32 * 1024,    4, SECT_4K_PMC) },
	{ "pm25lq032",   INFO(0x7f9d46, 0, 64 * 1024,   64, SECT_4K) },

	/* Spansion -- single (large) sector size only, at least
	 * for the chips listed here (without boot sectors).
	 */
	{ "s25sl032p",  INFO(0x010215, 0x4d00,  64 * 1024,  64, SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ "s25sl064p",  INFO(0x010216, 0x4d00,  64 * 1024, 128, SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ "s25fl256s0", INFO(0x010219, 0x4d00, 256 * 1024, 128, 0) },
	{ "s25fl256s1", INFO(0x010219, 0x4d01,  64 * 1024, 512, SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ "s25fl512s",  INFO(0x010220, 0x4d00, 256 * 1024, 256, SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ "s70fl01gs",  INFO(0x010221, 0x4d00, 256 * 1024, 256, 0) },
	{ "s25sl12800", INFO(0x012018, 0x0300, 256 * 1024,  64, 0) },
	{ "s25sl12801", INFO(0x012018, 0x0301,  64 * 1024, 256, 0) },
	{ "s25fl128s",	INFO6(0x012018, 0x4d0180, 64 * 1024, 256, SECT_4K | SPI_NOR_QUAD_READ) },
	{ "s25fl129p0", INFO(0x012018, 0x4d00, 256 * 1024,  64, SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ "s25fl129p1", INFO(0x012018, 0x4d01,  64 * 1024, 256, SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ "s25sl004a",  INFO(0x010212,      0,  64 * 1024,   8, 0) },
	{ "s25sl008a",  INFO(0x010213,      0,  64 * 1024,  16, 0) },
	{ "s25sl016a",  INFO(0x010214,      0,  64 * 1024,  32, 0) },
	{ "s25sl032a",  INFO(0x010215,      0,  64 * 1024,  64, 0) },
	{ "s25sl064a",  INFO(0x010216,      0,  64 * 1024, 128, 0) },
	{ "s25fl004k",  INFO(0xef4013,      0,  64 * 1024,   8, SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ "s25fl008k",  INFO(0xef4014,      0,  64 * 1024,  16, SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ "s25fl016k",  INFO(0xef4015,      0,  64 * 1024,  32, SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ "s25fl064k",  INFO(0xef4017,      0,  64 * 1024, 128, SECT_4K) },
	{ "s25fl132k",  INFO(0x014016,      0,  64 * 1024,  64, SECT_4K) },
	{ "s25fl164k",  INFO(0x014017,      0,  64 * 1024, 128, SECT_4K) },
	{ "s25fl204k",  INFO(0x014013,      0,  64 * 1024,   8, SECT_4K | SPI_NOR_DUAL_READ) },

	/* SST -- large erase sizes are "overlays", "sectors" are 4K */
	{ "sst25vf040b", INFO(0xbf258d, 0, 64 * 1024,  8, SECT_4K | SST_WRITE) },
	{ "sst25vf080b", INFO(0xbf258e, 0, 64 * 1024, 16, SECT_4K | SST_WRITE) },
	{ "sst25vf016b", INFO(0xbf2541, 0, 64 * 1024, 32, SECT_4K | SST_WRITE) },
	{ "sst25vf032b", INFO(0xbf254a, 0, 64 * 1024, 64, SECT_4K | SST_WRITE) },
	{ "sst25vf064c", INFO(0xbf254b, 0, 64 * 1024, 128, SECT_4K) },
	{ "sst25wf512",  INFO(0xbf2501, 0, 64 * 1024,  1, SECT_4K | SST_WRITE) },
	{ "sst25wf010",  INFO(0xbf2502, 0, 64 * 1024,  2, SECT_4K | SST_WRITE) },
	{ "sst25wf020",  INFO(0xbf2503, 0, 64 * 1024,  4, SECT_4K | SST_WRITE) },
	{ "sst25wf020a", INFO(0x621612, 0, 64 * 1024,  4, SECT_4K) },
	{ "sst25wf040b", INFO(0x621613, 0, 64 * 1024,  8, SECT_4K) },
	{ "sst25wf040",  INFO(0xbf2504, 0, 64 * 1024,  8, SECT_4K | SST_WRITE) },
	{ "sst25wf080",  INFO(0xbf2505, 0, 64 * 1024, 16, SECT_4K | SST_WRITE) },

	/* ST Microelectronics -- newer production may have feature updates */
	{ "m25p05",  INFO(0x202010,  0,  32 * 1024,   2, 0) },
	{ "m25p10",  INFO(0x202011,  0,  32 * 1024,   4, 0) },
	{ "m25p20",  INFO(0x202012,  0,  64 * 1024,   4, 0) },
	{ "m25p40",  INFO(0x202013,  0,  64 * 1024,   8, 0) },
	{ "m25p80",  INFO(0x202014,  0,  64 * 1024,  16, 0) },
	{ "m25p16",  INFO(0x202015,  0,  64 * 1024,  32, 0) },
	{ "m25p32",  INFO(0x202016,  0,  64 * 1024,  64, 0) },
	{ "m25p64",  INFO(0x202017,  0,  64 * 1024, 128, 0) },
	{ "m25p128", INFO(0x202018,  0, 256 * 1024,  64, 0) },

	{ "m25p05-nonjedec",  INFO(0, 0,  32 * 1024,   2, 0) },
	{ "m25p10-nonjedec",  INFO(0, 0,  32 * 1024,   4, 0) },
	{ "m25p20-nonjedec",  INFO(0, 0,  64 * 1024,   4, 0) },
	{ "m25p40-nonjedec",  INFO(0, 0,  64 * 1024,   8, 0) },
	{ "m25p80-nonjedec",  INFO(0, 0,  64 * 1024,  16, 0) },
	{ "m25p16-nonjedec",  INFO(0, 0,  64 * 1024,  32, 0) },
	{ "m25p32-nonjedec",  INFO(0, 0,  64 * 1024,  64, 0) },
	{ "m25p64-nonjedec",  INFO(0, 0,  64 * 1024, 128, 0) },
	{ "m25p128-nonjedec", INFO(0, 0, 256 * 1024,  64, 0) },

	{ "m45pe10", INFO(0x204011,  0, 64 * 1024,    2, 0) },
	{ "m45pe80", INFO(0x204014,  0, 64 * 1024,   16, 0) },
	{ "m45pe16", INFO(0x204015,  0, 64 * 1024,   32, 0) },

	{ "m25pe20", INFO(0x208012,  0, 64 * 1024,  4,       0) },
	{ "m25pe80", INFO(0x208014,  0, 64 * 1024, 16,       0) },
	{ "m25pe16", INFO(0x208015,  0, 64 * 1024, 32, SECT_4K) },

	{ "m25px16",    INFO(0x207115,  0, 64 * 1024, 32, SECT_4K) },
	{ "m25px32",    INFO(0x207116,  0, 64 * 1024, 64, SECT_4K) },
	{ "m25px32-s0", INFO(0x207316,  0, 64 * 1024, 64, SECT_4K) },
	{ "m25px32-s1", INFO(0x206316,  0, 64 * 1024, 64, SECT_4K) },
	{ "m25px64",    INFO(0x207117,  0, 64 * 1024, 128, 0) },
	{ "m25px80",    INFO(0x207114,  0, 64 * 1024, 16, 0) },

	/* Winbond -- w25x "blocks" are 64K, "sectors" are 4KiB */
	{ "w25x05", INFO(0xef3010, 0, 64 * 1024,  1,  SECT_4K) },
	{ "w25x10", INFO(0xef3011, 0, 64 * 1024,  2,  SECT_4K) },
	{ "w25x20", INFO(0xef3012, 0, 64 * 1024,  4,  SECT_4K) },
	{ "w25x40", INFO(0xef3013, 0, 64 * 1024,  8,  SECT_4K) },
	{ "w25x80", INFO(0xef3014, 0, 64 * 1024,  16, SECT_4K) },
	{ "w25x16", INFO(0xef3015, 0, 64 * 1024,  32, SECT_4K) },
	{ "w25x32", INFO(0xef3016, 0, 64 * 1024,  64, SECT_4K) },
	{ "w25q32", INFO(0xef4016, 0, 64 * 1024,  64, SECT_4K) },
	{ "w25q32dw", INFO(0xef6016, 0, 64 * 1024,  64, SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ "w25x64", INFO(0xef3017, 0, 64 * 1024, 128, SECT_4K) },
	{ "w25q64", INFO(0xef4017, 0, 64 * 1024, 128, SECT_4K) },
	{ "w25q64dw", INFO(0xef6017, 0, 64 * 1024, 128, SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ "w25q128fw", INFO(0xef6018, 0, 64 * 1024, 256, SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ "w25q80", INFO(0xef5014, 0, 64 * 1024,  16, SECT_4K) },
	{ "w25q80bl", INFO(0xef4014, 0, 64 * 1024,  16, SECT_4K) },
	{ "w25q128", INFO(0xef4018, 0, 64 * 1024, 256, SECT_4K) },
	{ "w25q256", INFO(0xef4019, 0, 64 * 1024, 512, SECT_4K) },

	/* Catalyst / On Semiconductor -- non-JEDEC */
	{ "cat25c11", CAT25_INFO(  16, 8, 16, 1, SPI_NOR_NO_ERASE | SPI_NOR_NO_FR) },
	{ "cat25c03", CAT25_INFO(  32, 8, 16, 2, SPI_NOR_NO_ERASE | SPI_NOR_NO_FR) },
	{ "cat25c09", CAT25_INFO( 128, 8, 32, 2, SPI_NOR_NO_ERASE | SPI_NOR_NO_FR) },
	{ "cat25c17", CAT25_INFO( 256, 8, 32, 2, SPI_NOR_NO_ERASE | SPI_NOR_NO_FR) },
	{ "cat25128", CAT25_INFO(2048, 8, 64, 2, SPI_NOR_NO_ERASE | SPI_NOR_NO_FR) },
	{ },
};

static const struct flash_info *spi_nor_read_id(struct spi_nor *nor)
{
	int			tmp;
	u8			id[SPI_NOR_MAX_ID_LEN];
	const struct flash_info	*info;

	tmp = nor->read_reg(nor, SPINOR_OP_RDID, id, SPI_NOR_MAX_ID_LEN);
	if (tmp < 0) {
		dev_dbg(nor->dev, " error %d reading JEDEC ID\n", tmp);
		return ERR_PTR(tmp);
	}

	for (tmp = 0; tmp < ARRAY_SIZE(spi_nor_ids) - 1; tmp++) {
		info = &spi_nor_ids[tmp];
		if (info->id_len) {
			if (!memcmp(info->id, id, info->id_len))
				return &spi_nor_ids[tmp];
		}
	}
	dev_err(nor->dev, "unrecognized JEDEC id bytes: %02x, %2x, %2x\n",
		id[0], id[1], id[2]);
	return ERR_PTR(-ENODEV);
}

static int spi_nor_read(struct mtd_info *mtd, loff_t from, size_t len,
			size_t *retlen, u_char *buf)
{
	struct spi_nor *nor = mtd_to_spi_nor(mtd);
	int ret;

	dev_dbg(nor->dev, "from 0x%08x, len %zd\n", (u32)from, len);

	ret = spi_nor_lock_and_prep(nor, SPI_NOR_OPS_READ);
	if (ret)
		return ret;

	ret = nor->read(nor, from, len, retlen, buf);

	spi_nor_unlock_and_unprep(nor, SPI_NOR_OPS_READ);
	return ret;
}

static int sst_write(struct mtd_info *mtd, loff_t to, size_t len,
		size_t *retlen, const u_char *buf)
{
	struct spi_nor *nor = mtd_to_spi_nor(mtd);
	size_t actual;
	int ret;

	dev_dbg(nor->dev, "to 0x%08x, len %zd\n", (u32)to, len);

	ret = spi_nor_lock_and_prep(nor, SPI_NOR_OPS_WRITE);
	if (ret)
		return ret;

	write_enable(nor);

	nor->sst_write_second = false;

	actual = to % 2;
	/* Start write from odd address. */
	if (actual) {
		nor->program_opcode = SPINOR_OP_BP;

		/* write one byte. */
		nor->write(nor, to, 1, retlen, buf);
		ret = spi_nor_wait_till_ready(nor);
		if (ret)
			goto time_out;
	}
	to += actual;

	/* Write out most of the data here. */
	for (; actual < len - 1; actual += 2) {
		nor->program_opcode = SPINOR_OP_AAI_WP;

		/* write two bytes. */
		nor->write(nor, to, 2, retlen, buf + actual);
		ret = spi_nor_wait_till_ready(nor);
		if (ret)
			goto time_out;
		to += 2;
		nor->sst_write_second = true;
	}
	nor->sst_write_second = false;

	write_disable(nor);
	ret = spi_nor_wait_till_ready(nor);
	if (ret)
		goto time_out;

	/* Write out trailing byte if it exists. */
	if (actual != len) {
		write_enable(nor);

		nor->program_opcode = SPINOR_OP_BP;
		nor->write(nor, to, 1, retlen, buf + actual);

		ret = spi_nor_wait_till_ready(nor);
		if (ret)
			goto time_out;
		write_disable(nor);
	}
time_out:
	spi_nor_unlock_and_unprep(nor, SPI_NOR_OPS_WRITE);
	return ret;
}

/*
 * Write an address range to the nor chip.  Data must be written in
 * FLASH_PAGESIZE chunks.  The address range may be any size provided
 * it is within the physical boundaries.
 */
static int spi_nor_write(struct mtd_info *mtd, loff_t to, size_t len,
	size_t *retlen, const u_char *buf)
{
	struct spi_nor *nor = mtd_to_spi_nor(mtd);
	u32 page_offset, page_size, i;
	int ret;

	dev_dbg(nor->dev, "to 0x%08x, len %zd\n", (u32)to, len);

	ret = spi_nor_lock_and_prep(nor, SPI_NOR_OPS_WRITE);
	if (ret)
		return ret;

	write_enable(nor);

	page_offset = to & (nor->page_size - 1);

	/* do all the bytes fit onto one page? */
	if (page_offset + len <= nor->page_size) {
		nor->write(nor, to, len, retlen, buf);
	} else {
		/* the size of data remaining on the first page */
		page_size = nor->page_size - page_offset;
		nor->write(nor, to, page_size, retlen, buf);

		/* write everything in nor->page_size chunks */
		for (i = page_size; i < len; i += page_size) {
			page_size = len - i;
			if (page_size > nor->page_size)
				page_size = nor->page_size;

			ret = spi_nor_wait_till_ready(nor);
			if (ret)
				goto write_err;

			write_enable(nor);

			nor->write(nor, to + i, page_size, retlen, buf + i);
		}
	}

	ret = spi_nor_wait_till_ready(nor);
write_err:
	spi_nor_unlock_and_unprep(nor, SPI_NOR_OPS_WRITE);
	return ret;
}

static int macronix_quad_enable(struct spi_nor *nor)
{
	int ret, val;

	val = read_sr(nor);
	if (val < 0)
		return val;
	if (val & SR_QUAD_EN_MX)
		return 0;

	/* Update the Quad Enable bit. */
	dev_info(nor->dev, "setting Macronix Quad Enable (non-volatile) bit\n");
	write_enable(nor);

	write_sr(nor, val | SR_QUAD_EN_MX);

	if (spi_nor_wait_till_ready(nor))
		return 1;

	ret = read_sr(nor);
	if (!(ret > 0 && (ret & SR_QUAD_EN_MX))) {
		dev_err(nor->dev, "Macronix Quad bit not set\n");
		return -EINVAL;
	}

	return 0;
}

/*
 * Write status Register and configuration register with 2 bytes
 * The first byte will be written to the status register, while the
 * second byte will be written to the configuration register.
 * Return negative if error occured.
 */
static int write_sr_cr(struct spi_nor *nor, u16 val)
{
	nor->cmd_buf[0] = val & 0xff;
	nor->cmd_buf[1] = (val >> 8);

	return nor->write_reg(nor, SPINOR_OP_WRSR, nor->cmd_buf, 2);
}

static int spansion_quad_enable(struct spi_nor *nor)
{
	int ret;
	int quad_en = CR_QUAD_EN_SPAN << 8;

	write_enable(nor);

	ret = write_sr_cr(nor, quad_en);
	if (ret < 0) {
		dev_err(nor->dev,
			"error while writing configuration register\n");
		return -EINVAL;
	}

	/* read back and check it */
	ret = read_cr(nor);
	if (!(ret > 0 && (ret & CR_QUAD_EN_SPAN))) {
		dev_err(nor->dev, "Spansion Quad bit not set\n");
		return -EINVAL;
	}

	return 0;
}

static int spansion_new_quad_enable(struct spi_nor *nor)
{
	u8 sr_cr[2];
	int ret;

	/* Check current Quad Enable bit value. */
	ret = read_cr(nor);
	if (ret < 0) {
		dev_err(nor->dev,
			"error while reading configuration register\n");
		return -EINVAL;
	}
	sr_cr[1] = ret;
	if (sr_cr[1] & CR_QUAD_EN_SPAN)
		return 0;

	dev_info(nor->dev, "setting Spansion Quad Enable (non-volatile) bit\n");

	/* Keep the current value of the Status Register. */
	ret = read_sr(nor);
	if (ret < 0) {
		dev_err(nor->dev,
			"error while reading status register\n");
		return -EINVAL;
	}
	sr_cr[0] = ret;
	sr_cr[1] |= CR_QUAD_EN_SPAN;

	write_enable(nor);

	ret = nor->write_reg(nor, SPINOR_OP_WRSR, sr_cr, 2);
	if (ret < 0) {
		dev_err(nor->dev,
			"error while writing configuration register\n");
		return -EINVAL;
	}

	/* read back and check it */
	ret = read_cr(nor);
	if (!(ret > 0 && (ret & CR_QUAD_EN_SPAN))) {
		dev_err(nor->dev, "Spansion Quad bit not set\n");
		return -EINVAL;
	}

	return 0;
}

static int spi_nor_check(struct spi_nor *nor)
{
	if (!nor->dev || !nor->read || !nor->write ||
		!nor->read_reg || !nor->write_reg || !nor->erase) {
		pr_err("spi-nor: please fill all the necessary fields!\n");
		return -EINVAL;
	}

	return 0;
}

static inline void spi_nor_set_read_settings(struct spi_nor_read *read,
					     u8 num_mode_clocks,
					     u8 num_wait_states,
					     u8 opcode)
{
	read->num_mode_clocks = num_mode_clocks;
	read->num_wait_states = num_wait_states;
	read->opcode = opcode;
}

static inline void spi_nor_set_erase_settings(struct spi_nor_erase_type *erase,
					      u8 size, u8 opcode)
{
	erase->size = size;
	erase->opcode = opcode;
}

static int spi_nor_read_sfdp(struct spi_nor *nor, u32 addr, size_t len, void *buf)
{
	u8 addr_width, read_opcode, read_dummy;
	size_t retlen;
	int ret;

	read_opcode = nor->read_opcode;
	addr_width = nor->addr_width;
	read_dummy = nor->read_dummy;

	nor->read_opcode = SPINOR_OP_RDSFDP;
	nor->addr_width = 3;
	nor->read_dummy = 8;

	ret = nor->read(nor, addr, len, &retlen, (u8 *)buf);

	nor->read_opcode = read_opcode;
	nor->addr_width = addr_width;
	nor->read_dummy = read_dummy;

	return (ret < 0) ? ret : 0;
}

struct sfdp_parameter_header {
	u8		id_lsb;
	u8		minor;
	u8		major;
	u8		length; /* in double words */
	u8		parameter_table_pointer[3]; /* byte address */
	u8		id_msb;
};

#define SFDP_PARAM_HEADER_ID(p)	((u16)(((p)->id_msb << 8) | (p)->id_lsb))
#define SFDP_PARAM_HEADER_PTP(p) \
	((u32)(((p)->parameter_table_pointer[2] << 16) | \
	       ((p)->parameter_table_pointer[1] <<  8) | \
	       ((p)->parameter_table_pointer[0] <<  0)))


#define SFDP_BFPT_ID		0xff00u	/* Basic Flash Parameter Table */
#define SFDP_4BAIT_ID		0xff84u	/* 4-byte Address Instruction Table */

#define SFDP_SIGNATURE		0x50444653u
#define SFDP_JESD216_MAJOR	1
#define SFDP_JESD216_MINOR	0
#define SFDP_JESD216A_MINOR	5
#define SFDP_JESD216B_MINOR	6

struct sfdp_header {
	u32		signature; /* Ox50444653 <=> "SFDP" */
	u8		minor;
	u8		major;
	u8		nph; /* 0-base number of parameter headers */
	u8		unused;

	/* Basic Flash Parameter Table. */
	struct sfdp_parameter_header	bfpt_header;
};

/* Basic Flash Parameter Table */

/* 1st DWORD. */
#define BFPT_WORD0_FAST_READ_1_1_2	BIT(16)
#define BFPT_WORD0_ADDRESS_BYTES_MASK	GENMASK(18, 17)
#define BFPT_WORD0_ADDRESS_BYTES_3_ONLY	(0u << 17)
#define BFPT_WORD0_ADDRESS_BYTES_3_OR_4	(1u << 17)
#define BFPT_WORD0_ADDRESS_BYTES_4_ONLY	(2u << 17)
#define BFPT_WORD0_DTR			BIT(19)
#define BFPT_WORD0_FAST_READ_1_2_2	BIT(20)
#define BFPT_WORD0_FAST_READ_1_4_4	BIT(21)
#define BFPT_WORD0_FAST_READ_1_1_4	BIT(22)

/* 15th DWORD. */

/*
 * (from JESD216B)
 * Quad Enable Requirements (QER):
 * - 000b: Device does not have a QE bit. Device detects 1-1-4 and 1-4-4
 *         reads based on instruction. DQ3/HOLD# functions are hold during
 *         instruction pahse.
 * - 001b: QE is bit 1 of status register 2. It is set via Write Status with
 *         two data bytes where bit 1 of the second byte is one.
 *         [...]
 *         Writing only one byte to the status register has the side-effect of
 *         clearing status register 2, including the QE bit. The 100b code is
 *         used if writing one byte to the status register does not modify
 *         status register 2.
 * - 010b: QE is bit 6 of status register 1. It is set via Write Status with
 *         one data byte where bit 6 is one.
 *         [...]
 * - 011b: QE is bit 7 of status register 2. It is set via Write status
 *         register 2 instruction 3Eh with one data byte where bit 7 is one.
 *         [...]
 *         The status register 2 is read using instruction 3Fh.
 * - 100b: QE is bit 1 of status register 2. It is set via Write Status with
 *         two data bytes where bit 1 of the second byte is one.
 *         [...]
 *         In contrast to the 001b code, writing one byte to the status
 *         register does not modify status register 2.
 * - 101b: QE is bit 1 of status register 2. Status register 1 is read using
 *         Read Status instruction 05h. Status register2 is read using
 *         instruction 35h. QE is set via Writ Status instruction 01h with
 *         two data bytes where bit 1 of the second byte is one.
 *         [...]
 */
#define BFPT_WORD14_QER_MASK		GENMASK(22, 20)
#define BFPT_WORD14_QER_NONE		(0 << 20) /* Micron */
#define BFPT_WORD14_QER_SR2_BIT1_BUGGY	(1 << 20)
#define BFPT_WORD14_QER_SR1_BIT6	(2 << 20) /* Macronix */
#define BFPT_WORD14_QER_SR2_BIT7	(3 << 20)
#define BFPT_WORD14_QER_SR2_BIT1_NO_RD	(4 << 20)
#define BFPT_WORD14_QER_SR2_BIT1	(5 << 20) /* Spansion */

/* JESD216B defines a Basic Flash Parameter Table of 16 words. */
#define SFDP_BFPT_MAX_WORDS	16

struct sfdp_bfpt {
	u32	word[SFDP_BFPT_MAX_WORDS];
};

/* Fast Read settings. */
#define BFPT_FAST_READ_WAIT_STATES(half)	((((u16)(half)) >> 0) & 0x1f)
#define BFPT_FAST_READ_MODE_CLOCKS(half)	((((u16)(half)) >> 5) & 0x07)
#define BFPT_FAST_READ_OP_CODE(half)		((((u16)(half)) >> 8) & 0xff)

struct sfdp_read {
	/* The protocol index of SPI x-y-z. */
	enum spi_nor_protocol_index	pindex;

	/*
	 * The bit <wbit> in DWORD<windex> tells us whether the
	 * Fast Read x-y-z command is supported.
	 */
	int				windex;
	int				wbit;

	/*
	 * The half-word at offset <hshift> in DWORD<hindex> encodes the
	 * op code, the number of mode clocks and the number of wait states
	 * to be used by Fast Read x-y-z commands.
	 */
	int				hindex;
	int				hshift;
};

static const struct sfdp_read sfdp_reads[] = {
	/* Supported: DWORD0 bit 16,  Settings: DWORD3 bit[15:0] */
	{SNOR_PINDEX_1_1_2, 0, 16, 3,  0},

	/* Supported: DWORD0 bit 20,  Settings: DWORD3 bit[31:16] */
	{SNOR_PINDEX_1_2_2, 0, 20, 3, 16},

	/* Supported: DWORD4 bit  0,  Settings: DWORD5 bit[31:16] */
	{SNOR_PINDEX_2_2_2, 4,  0, 5, 16},

	/* Supported: DWORD0 bit 22,  Settings: DWORD2 bit[31:16] */
	{SNOR_PINDEX_1_1_4, 0, 22, 2, 16},

	/* Supported: DWORD0 bit 21,  Settings: DWORD2 bit[15:0] */
	{SNOR_PINDEX_1_4_4, 0, 21, 2, 0},

	/* Supported: DWORD4 bit  4,  Settings: DWORD6 bit[31:16] */
	{SNOR_PINDEX_4_4_4, 4, 4, 6, 16},
};


/* Sector Erase settings. */
#define BFPT_ERASE_SIZE(half)		((((u16)(half)) >> 0) & 0xff)
#define BFPT_ERASE_OP_CODE(half)	((((u16)(half)) >> 8) & 0xff)

struct sfdp_erase {
	/*
	 * The half-word at offset <hshift> in DWORD<hindex> encodes the
	 * op code and erase sector size to be used by Sector Erase commands.
	 */
	int		hindex;
	int		hshift;
};

static const struct sfdp_erase sfdp_erases[SNOR_MAX_ERASE_TYPES] = {
	/* Erase Type 1 in DWORD7 bits[15:0] */
	{7, 0},

	/* Erase Type 2 in DWORD7 bits[31:16] */
	{7, 16},

	/* Erase Type 3 in DWORD8 bits[15:0] */
	{8, 0},

	/* Erase Type 4: in DWORD8 bits[31:16] */
	{8, 16},
};


static int spi_nor_parse_bfpt(struct spi_nor *nor,
			      const struct flash_info *info,
			      const struct sfdp_parameter_header *bfpt_header,
			      struct spi_nor_basic_flash_parameter *params)
{
	struct sfdp_bfpt bfpt;
	size_t len;
	int i, err;
	u32 addr;
	u16 half;

	/* JESD216 Basic Flash Parameter Table length is at least 9 words. */
	if (bfpt_header->length < 9)
		return -EINVAL;

	/* Read the Basic Flash Parameter Table. */
	len = min_t(size_t, sizeof(bfpt),
		    bfpt_header->length * sizeof(uint32_t));
	addr = SFDP_PARAM_HEADER_PTP(bfpt_header);
	memset(&bfpt, 0, sizeof(bfpt));
	err = spi_nor_read_sfdp(nor,  addr, len, &bfpt);
	if (err)
		return err;

	for (i = 0; i < SFDP_BFPT_MAX_WORDS; ++i)
		bfpt.word[i] = le32_to_cpu(bfpt.word[i]);

	memset(params, 0, sizeof(*params));

	/* Fast Read settings. */
	params->rd_modes = (SNOR_MODE_SLOW | SNOR_MODE_1_1_1);
	spi_nor_set_read_settings(&params->reads[SNOR_PINDEX_SLOW],
				  0, 0, SPINOR_OP_READ);
	spi_nor_set_read_settings(&params->reads[SNOR_PINDEX_1_1_1],
				  0, 8, SPINOR_OP_READ_FAST);

	for (i = 0; i < ARRAY_SIZE(sfdp_reads); ++i) {
		const struct sfdp_read *rd_info = &sfdp_reads[i];
		struct spi_nor_read *read = &params->reads[rd_info->pindex];

		if (!(bfpt.word[rd_info->windex] & BIT(rd_info->wbit)))
			continue;

		params->rd_modes |= BIT(rd_info->pindex);
		half = bfpt.word[rd_info->hindex] >> rd_info->hshift;
		read->num_mode_clocks = BFPT_FAST_READ_MODE_CLOCKS(half);
		read->num_wait_states = BFPT_FAST_READ_WAIT_STATES(half);
		read->opcode = BFPT_FAST_READ_OP_CODE(half);
	}

	/* Page Program settings. */
	params->wr_modes = SNOR_MODE_1_1_1;
	params->page_programs[SNOR_PINDEX_1_1_1] = SPINOR_OP_PP;

	/* Sector Erase settings. */
	for (i = 0; i < SNOR_MAX_ERASE_TYPES; ++i) {
		const struct sfdp_erase *er_info = &sfdp_erases[i];
		struct spi_nor_erase_type *erase = &params->erase_types[i];

		half = bfpt.word[er_info->hindex] >> er_info->hshift;
		erase->size = BFPT_ERASE_SIZE(half);
		erase->opcode = BFPT_ERASE_OP_CODE(half);
	}

	/* Stop here if not JESD216 rev B or later. */
	if (bfpt_header->length < 15)
		return 0;

	/* Enable Quad I/O. */
	switch (bfpt.word[14] & BFPT_WORD14_QER_MASK) {
	default:
	case BFPT_WORD14_QER_NONE:
		break;

	case BFPT_WORD14_QER_SR2_BIT1_BUGGY:
	case BFPT_WORD14_QER_SR2_BIT1_NO_RD:
		params->enable_quad_io = spansion_quad_enable;
		break;

	case BFPT_WORD14_QER_SR1_BIT6:
		params->enable_quad_io = macronix_quad_enable;
		break;

	case BFPT_WORD14_QER_SR2_BIT7:
		// TODO:
		break;

	case BFPT_WORD14_QER_SR2_BIT1:
		params->enable_quad_io = spansion_new_quad_enable;
		break;
	}

	return 0;
}

struct sfdp_4bait {
	enum spi_nor_protocol_index	pindex;
	int				wbit;
};

static int spi_nor_parse_4bait(struct spi_nor *nor,
			      const struct flash_info *info,
			      const struct sfdp_parameter_header *param_header,
			      struct spi_nor_basic_flash_parameter *params)
{
	static const struct sfdp_4bait reads[] = {
		{SNOR_PINDEX_SLOW,  0},	/* 0x13 */
		{SNOR_PINDEX_1_1_1, 1}, /* 0x0c */
		{SNOR_PINDEX_1_1_2, 2}, /* 0x3c */
		{SNOR_PINDEX_1_2_2, 3},	/* 0xbc */
		{SNOR_PINDEX_1_1_4, 4},	/* 0x6c */
		{SNOR_PINDEX_1_4_4, 5},	/* 0xec */
	};
	static const struct sfdp_4bait programs[] = {
		{SNOR_PINDEX_1_1_1, 6}, /* 0x12 */
		{SNOR_PINDEX_1_1_4, 7}, /* 0x34 */
		{SNOR_PINDEX_1_4_4, 8}, /* 0x3e */
	};
	static const struct sfdp_4bait erases[SNOR_MAX_ERASE_TYPES] = {
		{SNOR_PINDEX_1_1_1,  9},
		{SNOR_PINDEX_1_1_1, 10},
		{SNOR_PINDEX_1_1_1, 11},
		{SNOR_PINDEX_1_1_1, 12},
	};
	u32 word[2], addr, rd_modes, wr_modes, erase_modes;
	int i, err;

	if (param_header->major != SFDP_JESD216_MAJOR ||
	    param_header->length < 2)
		return -EINVAL;

	/* Read the 4-byte Address Instruction Table. */
	addr = SFDP_PARAM_HEADER_PTP(param_header);
	err = spi_nor_read_sfdp(nor, addr, sizeof(word), word);
	if (err)
		return err;

	for (i = 0; i < 2; ++i)
		word[i] = le32_to_cpu(word[i]);

	/*
	 * Compute the subset of (Fast) Read commands for which the 4-byte
	 * version is supported.
	 */
	rd_modes = 0;
	for (i = 0; i < ARRAY_SIZE(reads); ++i) {
		const struct sfdp_4bait *read = &reads[i];

		if ((params->rd_modes & BIT(read->pindex)) &&
		    (word[0] & BIT(read->wbit)))
			rd_modes |= BIT(read->pindex);
	}

	/*
	 * Compute the subset of Page Program commands for which the 4-byte
	 * version is supported.
	 */
	wr_modes = 0;
	for (i = 0; i < ARRAY_SIZE(programs); ++i) {
		const struct sfdp_4bait *program = &programs[i];

		if ((params->wr_modes & BIT(program->pindex)) &&
		    (word[0] & BIT(program->wbit)))
			wr_modes |= BIT(program->pindex);
	}

	/*
	 * Compute the subet of Sector Erase commands for which the 4-byte
	 * version is supported.
	 */
	erase_modes = 0;
	for (i = 0; i < SNOR_MAX_ERASE_TYPES; ++i) {
		const struct sfdp_4bait *erase = &erases[i];

		if ((params->erase_types[i].size > 0) &&
		    (word[0] & BIT(erase->wbit)))
			erase_modes |= BIT(i);
	}

	/*
	 * We need at least one 4-byte op code per read, program and erase
	 * operation; the .read(), .write() and .erase() hooks share the
	 * nor->addr_width value.
	 */
	if (!rd_modes || !wr_modes || !erase_modes)
		return 0;

	/*
	 * Discard all operations from the 4-byte instruction set which are
	 * not supported by this memory.
	 */
	params->rd_modes = rd_modes;
	params->wr_modes = wr_modes;
	for (i = 0; i < SNOR_MAX_ERASE_TYPES; ++i)
		if (!(erase_modes & BIT(i)))
			params->erase_types[i].size = 0;

	/* Use the 4-byte address instruction set. */
	params->reads[SNOR_PINDEX_SLOW].opcode  = SPINOR_OP_READ_4B;
	params->reads[SNOR_PINDEX_1_1_1].opcode = SPINOR_OP_READ_FAST_4B;
	params->reads[SNOR_PINDEX_1_1_2].opcode = SPINOR_OP_READ_1_1_2_4B;
	params->reads[SNOR_PINDEX_1_2_2].opcode = SPINOR_OP_READ_1_2_2_4B;
	params->reads[SNOR_PINDEX_1_1_4].opcode = SPINOR_OP_READ_1_1_4_4B;
	params->reads[SNOR_PINDEX_1_4_4].opcode = SPINOR_OP_READ_1_4_4_4B;
	params->page_programs[SNOR_PINDEX_1_1_1] = SPINOR_OP_PP_4B;
	params->page_programs[SNOR_PINDEX_1_1_4] = SPINOR_OP_PP_1_1_4_4B;
	params->page_programs[SNOR_PINDEX_1_4_4] = SPINOR_OP_PP_1_4_4_4B;
	for (i = 0; i < SNOR_MAX_ERASE_TYPES; ++i)
		params->erase_types[i].opcode = (word[1] >> (i * 8)) & 0xff;

	nor->addr_width = 4;
	return 0;
}

static int spi_nor_parse_sfdp(struct spi_nor *nor,
			      const struct flash_info *info,
			      struct spi_nor_basic_flash_parameter *params)
{
	const struct sfdp_parameter_header *param_header, *bfpt_header;
	struct sfdp_parameter_header *param_headers = NULL;
	struct sfdp_header header;
	size_t psize;
	int i, err;

	/* Get the SFDP header. */
	err = spi_nor_read_sfdp(nor, 0, sizeof(header), &header);
	if (err)
		return err;

	/* Check the SFDP header version. */
	if (le32_to_cpu(header.signature) != SFDP_SIGNATURE ||
	    header.major != SFDP_JESD216_MAJOR ||
	    header.minor < SFDP_JESD216_MINOR)
		return -EINVAL;

	/*
	 * Verify that the first and only mandatory parameter header is a
	 * Basic Flash Parameter Table header as specified in JESD216.
	 */
	bfpt_header = &header.bfpt_header;
	if (SFDP_PARAM_HEADER_ID(bfpt_header) != SFDP_BFPT_ID ||
	    bfpt_header->major != SFDP_JESD216_MAJOR)
		return -EINVAL;

	/* Allocate memory for parameter headers. */
	if (header.nph) {
		psize = header.nph * sizeof(*param_headers);

		param_headers = kmalloc(psize, GFP_KERNEL);
		if (!param_headers) {
			dev_err(nor->dev,
				"failed to allocate memory for SFDP parameter headers\n");
			return -ENOMEM;
		}

		err = spi_nor_read_sfdp(nor, sizeof(header),
					psize, param_headers);
		if (err) {
			dev_err(nor->dev,
				"failed to read SFDP parameter headers\n");
			goto exit;
		}
	}

	/*
	 * Check other parameter headers to get the latest revision of
	 * the basic flash parameter table.
	 */
	for (i = 0; i < header.nph; ++i) {
		param_header = &param_headers[i];

		if (SFDP_PARAM_HEADER_ID(param_header) == SFDP_BFPT_ID &&
		    param_header->major == SFDP_JESD216_MAJOR &&
		    (param_header->minor > bfpt_header->minor ||
		     (param_header->minor == bfpt_header->minor &&
		      param_header->length > bfpt_header->length)))
			bfpt_header = param_header;
	}
	err = spi_nor_parse_bfpt(nor, info, bfpt_header, params);
	if (err)
		goto exit;

	/* Parse other parameter headers. */
	for (i = 0; i < header.nph; ++i) {
		param_header = &param_headers[i];

		switch (SFDP_PARAM_HEADER_ID(param_header)) {
		case SFDP_4BAIT_ID:
			err = spi_nor_parse_4bait(nor, info, param_header,
						  params);
			break;

		default:
			break;
		}

		if (err)
			goto exit;
	}

exit:
	kfree(param_headers);
	return (err) ? err : bfpt_header->minor;
}

static int spi_nor_init_params(struct spi_nor *nor,
			       const struct flash_info *info,
			       struct spi_nor_basic_flash_parameter *params)
{
	int jesd216_minor = -EINVAL;

	/* First trying to parse SFDP tables. */
	if (!(info->flags & SPI_NOR_SKIP_SFDP)) {
		jesd216_minor = spi_nor_parse_sfdp(nor, info, params);

		if (jesd216_minor >= SFDP_JESD216B_MINOR)
			return 0;

		if (jesd216_minor >= SFDP_JESD216_MINOR)
			goto set_enable_quad_io;
	}

	/* If SFDP tables are not available, use legacy settings. */
	memset(params, 0, sizeof(*params));

	/* (Fast) Read settings. */
	params->rd_modes = SNOR_MODE_1_1_1;
	spi_nor_set_read_settings(&params->reads[SNOR_PINDEX_1_1_1],
				  0, 8, SPINOR_OP_READ_FAST);
	if (!(info->flags & SPI_NOR_NO_FR)) {
		params->rd_modes |= SNOR_MODE_SLOW;
		spi_nor_set_read_settings(&params->reads[SNOR_PINDEX_SLOW],
					  0, 0, SPINOR_OP_READ);
	}
	if (info->flags & SPI_NOR_DUAL_READ) {
		params->rd_modes |= SNOR_MODE_1_1_2;
		spi_nor_set_read_settings(&params->reads[SNOR_PINDEX_1_1_2],
					  0, 8, SPINOR_OP_READ_1_1_2);
	}
	if (info->flags & SPI_NOR_QUAD_READ) {
		params->rd_modes|= SNOR_MODE_1_1_4;
		spi_nor_set_read_settings(&params->reads[SNOR_PINDEX_1_1_4],
					  0, 8, SPINOR_OP_READ_1_1_4);
	}

	/* Page Program settings. */
	params->wr_modes = SNOR_MODE_1_1_1;
	params->page_programs[SNOR_PINDEX_1_1_1] = SPINOR_OP_PP;

	/* Sector Erase settings. */
	spi_nor_set_erase_settings(&params->erase_types[0],
				   SNOR_ERASE_64K, SPINOR_OP_SE);
	if (info->flags & SECT_4K)
		spi_nor_set_erase_settings(&params->erase_types[1],
					   SNOR_ERASE_4K, SPINOR_OP_BE_4K);
	else if (info->flags & SECT_4K_PMC)
		spi_nor_set_erase_settings(&params->erase_types[1],
					   SNOR_ERASE_4K, SPINOR_OP_BE_4K_PMC);

set_enable_quad_io:
	/* Select the procedure to set the Quad Enable bit. */
	if (params->rd_modes & (SNOR_MODE_1_1_4 |
				SNOR_MODE_1_4_4 |
				SNOR_MODE_4_4_4)) {
		switch (JEDEC_MFR(info)) {
		case SNOR_MFR_MACRONIX:
			params->enable_quad_io = macronix_quad_enable;
			break;

		case SNOR_MFR_MICRON:
			break;

		default:
			params->enable_quad_io = spansion_quad_enable;
			break;
		}
	}

	return 0;
}

static int spi_nor_pindex2proto(int pindex, enum spi_nor_protocol *proto)
{
	enum spi_nor_protocol_width pwidth;
	enum spi_nor_protocol_class pclass;
	uint8_t width;

	if (pindex < 0)
		return -EINVAL;

	pwidth = (enum spi_nor_protocol_width)(pindex / SNOR_PCLASS_MAX);
	pclass = (enum spi_nor_protocol_class)(pindex % SNOR_PCLASS_MAX);

	width = (1 << pwidth) & 0xf;
	if (!width)
		return -EINVAL;

	switch (pclass) {
	case SNOR_PCLASS_1_1_N:
		*proto = SNOR_PROTO(1, 1, width);
		break;

	case SNOR_PCLASS_1_N_N:
		*proto = SNOR_PROTO(1, width, width);
		break;

	case SNOR_PCLASS_N_N_N:
		*proto = SNOR_PROTO(width, width, width);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int spi_nor_setup(struct spi_nor *nor, const struct flash_info *info,
			 const struct spi_nor_basic_flash_parameter *params,
			 const struct spi_nor_modes *modes)
{
	bool enable_quad_io;
	u32 rd_modes, wr_modes, mask;
	const struct spi_nor_erase_type *erase_type = NULL;
	const struct spi_nor_read *read;
	int rd_pindex, wr_pindex, i, err = 0;
	u8 erase_size = SNOR_ERASE_64K;

	/* 2-2-2 or 4-4-4 modes are not supported yet. */
	mask = (SNOR_MODE_2_2_2 | SNOR_MODE_4_4_4);
	rd_modes = modes->rd_modes & ~mask;
	wr_modes = modes->wr_modes & ~mask;

	/* Setup read operation. */
	rd_pindex = fls(params->rd_modes & rd_modes) - 1;
	if (spi_nor_pindex2proto(rd_pindex, &nor->read_proto)) {
		dev_err(nor->dev, "invalid (fast) read\n");
		return -EINVAL;
	}
	read = &params->reads[rd_pindex];
	nor->read_opcode = read->opcode;
	nor->read_dummy = read->num_mode_clocks + read->num_wait_states;

	/* Set page program op code and protocol. */
	wr_pindex = fls(params->wr_modes & wr_modes) - 1;
	if (spi_nor_pindex2proto(wr_pindex, &nor->write_proto)) {
		dev_err(nor->dev, "invalid page program\n");
		return -EINVAL;
	}
	nor->program_opcode = params->page_programs[wr_pindex];

	/* Set sector erase op code and size. */
	erase_type = &params->erase_types[0];
#ifdef CONFIG_MTD_SPI_NOR_USE_4K_SECTORS
	erase_size = SNOR_ERASE_4K;
#endif
	for (i = 0; i < SNOR_MAX_ERASE_TYPES; ++i) {
		if (params->erase_types[i].size == erase_size) {
			erase_type = &params->erase_types[i];
			break;
		}
	}
	nor->erase_opcode = erase_type->opcode;
	nor->mtd.erasesize = (1 << erase_type->size);


	enable_quad_io = (SNOR_PROTO_DATA_FROM_PROTO(nor->read_proto) == 4 ||
			  SNOR_PROTO_DATA_FROM_PROTO(nor->write_proto) == 4);

	/* Enable Quad I/O if needed. */
	if (enable_quad_io && params->enable_quad_io) {
		err = params->enable_quad_io(nor);
		if (err) {
			dev_err(nor->dev,
				"failed to enable the Quad I/O mode\n");
			return err;
		}
	}

	dev_dbg(nor->dev,
		"(Fast) Read:  opcode=%02Xh, protocol=%03x, mode=%u, wait=%u\n",
		nor->read_opcode, nor->read_proto,
		read->num_mode_clocks, read->num_wait_states);
	dev_dbg(nor->dev,
		"Page Program: opcode=%02Xh, protocol=%03x\n",
		nor->program_opcode, nor->write_proto);
	dev_dbg(nor->dev,
		"Sector Erase: opcode=%02Xh, protocol=%03x, sector size=%u\n",
		nor->erase_opcode, nor->reg_proto, (u32)nor->mtd.erasesize);

	return 0;
}

int spi_nor_scan(struct spi_nor *nor, const char *name,
		 const struct spi_nor_modes *modes)
{
	struct spi_nor_basic_flash_parameter params;
	struct spi_nor_modes fixed_modes = *modes;
	const struct flash_info *info = NULL;
	struct device *dev = nor->dev;
	struct mtd_info *mtd = &nor->mtd;
	struct device_node *np = nor->flash_node;
	int ret;
	int i;

	ret = spi_nor_check(nor);
	if (ret)
		return ret;

	/* Reset SPI protocol for all commands */
	nor->reg_proto = SNOR_PROTO_1_1_1;
	nor->read_proto = SNOR_PROTO_1_1_1;
	nor->write_proto = SNOR_PROTO_1_1_1;

	if (name)
		info = spi_nor_match_id(name);
	/* Try to auto-detect if chip name wasn't specified or not found */
	if (!info)
		info = spi_nor_read_id(nor);
	if (IS_ERR_OR_NULL(info))
		return -ENOENT;

	/*
	 * If caller has specified name of flash model that can normally be
	 * detected using JEDEC, let's verify it.
	 */
	if (name && info->id_len) {
		const struct flash_info *jinfo;

		jinfo = spi_nor_read_id(nor);
		if (IS_ERR(jinfo)) {
			return PTR_ERR(jinfo);
		} else if (jinfo != info) {
			/*
			 * JEDEC knows better, so overwrite platform ID. We
			 * can't trust partitions any longer, but we'll let
			 * mtd apply them anyway, since some partitions may be
			 * marked read-only, and we don't want to lose that
			 * information, even if it's not 100% accurate.
			 */
			dev_warn(dev, "found %s, expected %s\n",
				 jinfo->name, info->name);
			info = jinfo;
		}
	}

	/* Parse the Serial Flash Discoverable Parameters table */
	ret = spi_nor_init_params(nor, info, &params);
	if (ret)
		return ret;

	mutex_init(&nor->lock);

	/*
	 * Atmel, SST, Intel/Numonyx, and others serial NOR tend to power up
	 * with the software protection bits set
	 */

	if (JEDEC_MFR(info) == SNOR_MFR_ATMEL ||
	    JEDEC_MFR(info) == SNOR_MFR_INTEL ||
	    JEDEC_MFR(info) == SNOR_MFR_SST) {
		write_enable(nor);
		write_sr(nor, 0);
	}

	if (!mtd->name)
		mtd->name = dev_name(dev);
	mtd->priv = nor;
	mtd->type = MTD_NORFLASH;
	mtd->writesize = 1;
	mtd->flags = MTD_CAP_NORFLASH;
	mtd->size = info->sector_size * info->n_sectors;
	mtd->_erase = spi_nor_erase;
	mtd->_read = spi_nor_read;

	/* NOR protection support for STmicro/Micron chips and similar */
	if (JEDEC_MFR(info) == SNOR_MFR_MICRON) {
		nor->flash_lock = stm_lock;
		nor->flash_unlock = stm_unlock;
		nor->flash_is_locked = stm_is_locked;
	}

	if (nor->flash_lock && nor->flash_unlock && nor->flash_is_locked) {
		mtd->_lock = spi_nor_lock;
		mtd->_unlock = spi_nor_unlock;
		mtd->_is_locked = spi_nor_is_locked;
	}

	/* sst nor chips use AAI word program */
	if (info->flags & SST_WRITE)
		mtd->_write = sst_write;
	else
		mtd->_write = spi_nor_write;

	if (info->flags & USE_FSR)
		nor->flags |= SNOR_F_USE_FSR;

#ifdef CONFIG_MTD_SPI_NOR_USE_4K_SECTORS
	/* prefer "small sector" erase if possible */
	if (info->flags & SECT_4K) {
		nor->erase_opcode = SPINOR_OP_BE_4K;
		mtd->erasesize = 4096;
	} else if (info->flags & SECT_4K_PMC) {
		nor->erase_opcode = SPINOR_OP_BE_4K_PMC;
		mtd->erasesize = 4096;
	} else
#endif
	{
		nor->erase_opcode = SPINOR_OP_SE;
		mtd->erasesize = info->sector_size;
	}

	if (info->flags & SPI_NOR_NO_ERASE)
		mtd->flags |= MTD_NO_ERASE;

	mtd->dev.parent = dev;
	nor->page_size = info->page_size;
	mtd->writebufsize = nor->page_size;

	if (np) {
		/* If we were instantiated by DT, use it */
		if (of_property_read_bool(np, "m25p,fast-read"))
			fixed_modes.rd_modes |= SNOR_MODE_1_1_1;
		else
			fixed_modes.rd_modes &= ~SNOR_MODE_1_1_1;
	} else {
		/* If we weren't instantiated by DT, default to fast-read */
		fixed_modes.rd_modes |= SNOR_MODE_1_1_1;
	}

	/* Some devices cannot do fast-read, no matter what DT tells us */
	if (info->flags & SPI_NOR_NO_FR)
		fixed_modes.rd_modes &= ~SNOR_MODE_1_1_1;

	nor->program_opcode = SPINOR_OP_PP;

	/*
	 * Configure the SPI memory:
	 * - select op codes for (Fast) Read, Page Program and Sector Erase.
	 * - set the number of dummy cycles (mode cycles + wait states).
	 * - set the SPI protocols for register and memory accesses.
	 * - set the Quad Enable bit if needed (required by SPI x-y-4 protos).
	 */
	ret = spi_nor_setup(nor, info, &params, &fixed_modes);
	if (ret)
		return ret;

	if (nor->addr_width)
		; /* already configured by spi_nor_setup(). */
	else if (info->addr_width)
		nor->addr_width = info->addr_width;
	else if (mtd->size > 0x1000000) {
		/* enable 4-byte addressing if the device exceeds 16MiB */
		nor->addr_width = 4;
		if (JEDEC_MFR(info) == SNOR_MFR_SPANSION ||
		    info->flags & SPI_NOR_4B_OPCODES)
			spi_nor_set_4byte_opcodes(nor, info);
		else
			set_4byte(nor, info, 1);
	} else {
		nor->addr_width = 3;
	}

	dev_info(dev, "%s (%lld Kbytes)\n", info->name,
			(long long)mtd->size >> 10);

	dev_dbg(dev,
		"mtd .name = %s, .size = 0x%llx (%lldMiB), "
		".erasesize = 0x%.8x (%uKiB) .numeraseregions = %d\n",
		mtd->name, (long long)mtd->size, (long long)(mtd->size >> 20),
		mtd->erasesize, mtd->erasesize / 1024, mtd->numeraseregions);

	if (mtd->numeraseregions)
		for (i = 0; i < mtd->numeraseregions; i++)
			dev_dbg(dev,
				"mtd.eraseregions[%d] = { .offset = 0x%llx, "
				".erasesize = 0x%.8x (%uKiB), "
				".numblocks = %d }\n",
				i, (long long)mtd->eraseregions[i].offset,
				mtd->eraseregions[i].erasesize,
				mtd->eraseregions[i].erasesize / 1024,
				mtd->eraseregions[i].numblocks);
	return 0;
}
EXPORT_SYMBOL_GPL(spi_nor_scan);

static const struct flash_info *spi_nor_match_id(const char *name)
{
	const struct flash_info *id = spi_nor_ids;

	while (id->name) {
		if (!strcmp(name, id->name))
			return id;
		id++;
	}
	return NULL;
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Huang Shijie <shijie8@gmail.com>");
MODULE_AUTHOR("Mike Lavender");
MODULE_DESCRIPTION("framework for SPI NOR");
