/*
 * Copyright (C) 2017 Spreadtrum Communications Inc.
 *
 * host software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * host program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>

#include "nfc_base.h"

const char *part_probes[] = { "cmdlinepart", NULL };

static struct mtd_info sprd_mtd;

static int sprd_nfc_init_device(struct sprd_nfc_base *this)
{
	spin_lock_init(&this->lock);
	this->state = FL_READY;
	init_waitqueue_head(&this->wq);
	return 0;
}

static int sprd_nfc_get_device(struct sprd_nfc_base *this, int new_state)
{
	spinlock_t *lock = &this->lock;
	wait_queue_head_t *wq = &this->wq;
	DECLARE_WAITQUEUE(wait, current);
retry:
	spin_lock(lock);
	if (this->state == FL_READY) {
		this->state = new_state;
		spin_unlock(lock);
		if (new_state != FL_PM_SUSPENDED)
			this->ctrl_en(this, 1);
		return 0;
	}
	set_current_state(TASK_UNINTERRUPTIBLE);
	add_wait_queue(wq, &wait);
	spin_unlock(lock);
	schedule();
	remove_wait_queue(wq, &wait);
	goto retry;
}

static void sprd_nfc_put_device(struct sprd_nfc_base *this,
							char *callName, int ret)
{
	/*  Release the this and the chip  */
	if (this->state != FL_PM_SUSPENDED)
		this->ctrl_en(this, 0);
	spin_lock(&this->lock);
	this->state = FL_READY;
	wake_up(&this->wq);
	spin_unlock(&this->lock);
	if (ret)
		pr_err("nfc base: %s ret code %d", callName, ret);
}

static void init_param(struct sprd_nfc_base *this)
{
	this->page_mask = (1 << this->pageshift) - 1;
	this->blk_mask = (1 << this->blkshift) - 1;
	this->page_per_buf =
		(1 << (this->bufshift - this->pageshift));
	this->page_per_mtd =
		(1 << (this->chip_shift - this->pageshift)) * this->chip_num;
}
/* because the *bbt[ ] type is uint32_t,so x >> 5 */
#define set_bbt(x) {this->bbt[(x)>>5] |= (1<<((x)-(((x)>>5)<<5))); }
#define is_bbt(x) (!!(this->bbt[(x)>>5] & (1<<((x)-(((x)>>5)<<5)))))

static int init_bbt(struct sprd_nfc_base *this)
{
	uint32_t blk, cnt, bbt_size;
	int ret;

	cnt = (1 << (this->chip_shift - this->blkshift)) * this->chip_num;
	bbt_size = (cnt >> 5) << 2;
	this->bbt = kmalloc(bbt_size, GFP_KERNEL);
	memset(this->bbt, 0, bbt_size);
	pr_info("nand:cnt %d,bbt_size %d,chipshift %d,blkshift %d,chipnum %d\n",
			cnt, bbt_size, this->chip_shift,
			this->blkshift, this->chip_num);
	for (blk = 0; blk < cnt; blk++) {
		ret = this->nand_is_bad_blk(this,
				blk << (this->blkshift - this->pageshift));
		if (ret < 0) {
			pr_info("nand:init bbt err..\n");
			kfree(this->bbt);
			this->bbt = 0;
			return ret;
		} else if (ret) {
			set_bbt(blk);
			pr_info("nand:blk %d, blkpage %d is bad block.\n",
				blk, blk << (this->blkshift - this->pageshift));
		}
	}

	return ret;
}

static int nand_read_oob(struct mtd_info *mtd,
		struct sprd_nfc_base *this, loff_t from,
		struct mtd_oob_ops *ops, struct mtd_ecc_stats *ecc_stats)
{
	int ret = 0, tmpret = 0;
	uint32_t page_s, page_e, page_num, page_per_buf;
	uint32_t remain_page_inbuf, ret_num, i;
	struct mtd_ecc_stats stats;
	uint32_t read_len;
	uint32_t obb_read_len;
	uint32_t max_obb_read_len;
	uint8_t *mbuf, *obb_buf, *sbuf_v;
	uint32_t toread_m, toread_s, col;

	stats = mtd->ecc_stats;
	ops->retlen = 0;
	ops->oobretlen = 0;

	read_len = ops->len;
	obb_read_len = ops->ooblen;
	if (ops->mode == MTD_OPS_AUTO_OOB)
		max_obb_read_len = this->layout.oobavail;
	else
		max_obb_read_len = this->obb_size;

	mbuf = ops->datbuf;
	obb_buf = ops->oobbuf;
	col = from & this->page_mask;

	page_per_buf = this->page_per_buf;
	page_s = from >> this->pageshift;

	if (mbuf) {
		if (!read_len)
			return 0;

		if ((obb_buf) && (max_obb_read_len <= ops->ooboffs))
			return -EINVAL;

		page_e = ((from + read_len - 1) >> this->pageshift);
		page_num = page_e - page_s + 1;
	} else if (obb_buf) {
		if (((max_obb_read_len) && (max_obb_read_len <= ops->ooboffs))
		    || ((!max_obb_read_len) && (ops->ooboffs > 0))
		    ) {
			return -EINVAL;
		}
		if (!obb_read_len)
			return 0;

		page_num = obb_read_len / (max_obb_read_len - ops->ooboffs);
		if (obb_read_len % (max_obb_read_len - ops->ooboffs))
			page_num++;

	} else {
		return -EINVAL;
	}

/* check param */
	if ((page_s + page_num) > this->page_per_mtd)
		return -EINVAL;

	remain_page_inbuf = (page_per_buf - (page_s & (page_per_buf - 1)));
	if (remain_page_inbuf >= page_num) {
		tmpret = this->read_page_in_blk(this, page_s, page_num,
			&ret_num, (uint32_t) (!!ops->datbuf),
			(uint32_t) (!!ops->oobbuf), ops->mode, ecc_stats);
		if (tmpret == -EUCLEAN)
			ret = -EUCLEAN;
		else if (tmpret < 0)
			return tmpret;
		if (mbuf) {
			memcpy(mbuf, this->mbuf_v + col, read_len);
			ops->retlen = read_len;
		}
		if (obb_buf) {
			sbuf_v = this->sbuf_v;
			for (i = 0; i < page_num; i++) {
				toread_s = min(max_obb_read_len - ops->ooboffs,
						obb_read_len);
				memcpy(obb_buf,
					sbuf_v + ops->ooboffs, toread_s);
				obb_buf += toread_s;
				obb_read_len -= toread_s;
				sbuf_v += max_obb_read_len;
			}
			ops->oobretlen = ops->ooblen - obb_read_len;
		}
		return ret;
		}
	tmpret = this->read_page_in_blk(this, page_s, remain_page_inbuf,
				&ret_num, (uint32_t) (!!ops->datbuf),
				(uint32_t) (!!ops->oobbuf), ops->mode,
				ecc_stats);
	if (tmpret == -EUCLEAN)
		ret = -EUCLEAN;
	else if (tmpret < 0)
		return tmpret;
	if (mbuf) {
		toread_m =
			((remain_page_inbuf << this->pageshift) - col);
		memcpy(mbuf, this->mbuf_v + col, toread_m);
		mbuf += toread_m;
		read_len -= toread_m;
		col = 0;
		ops->retlen += toread_m;
	}
	if (obb_buf) {
		sbuf_v = this->sbuf_v;
		for (i = 0; i < remain_page_inbuf; i++) {
			toread_s =
				min(max_obb_read_len - ops->ooboffs,
						obb_read_len);
			memcpy(obb_buf, sbuf_v + ops->ooboffs, toread_s);
			obb_buf += toread_s;
			sbuf_v += max_obb_read_len;
			obb_read_len -= toread_s;
		}
		ops->oobretlen = ops->ooblen - obb_read_len;
	}
	page_s += remain_page_inbuf;
	page_num -= remain_page_inbuf;

	while (page_num >= page_per_buf) {
		tmpret = this->read_page_in_blk(this, page_s, page_per_buf,
			&ret_num, (uint32_t)(!!ops->datbuf),
			(uint32_t)(!!ops->oobbuf), ops->mode, ecc_stats);
		if (tmpret == -EUCLEAN)
			ret = -EUCLEAN;
		else if (tmpret < 0)
			return tmpret;
		if (mbuf) {
			toread_m = min(read_len,
				(page_per_buf << this->pageshift));
			memcpy(mbuf, this->mbuf_v, toread_m);
			mbuf += toread_m;
			read_len -= toread_m;
			ops->retlen += toread_m;
		}
		if (obb_buf) {
			sbuf_v = this->sbuf_v;
			for (i = 0; i < page_per_buf; i++) {
				toread_s = min(max_obb_read_len -
					ops->ooboffs, obb_read_len);
				memcpy(obb_buf, sbuf_v + ops->ooboffs,
					toread_s);
				obb_buf += toread_s;
				sbuf_v += max_obb_read_len;
				obb_read_len -= toread_s;
			}
			ops->oobretlen = ops->ooblen - obb_read_len;
		}
		page_s += page_per_buf;
		page_num -= page_per_buf;
	}
	if (page_num) {
		tmpret = this->read_page_in_blk(this, page_s,
				page_num, &ret_num,
				(uint32_t) (!!ops->datbuf),
				(uint32_t) (!!ops->oobbuf),
				ops->mode, ecc_stats);
		if (tmpret == -EUCLEAN)
			ret = -EUCLEAN;
		else if (tmpret < 0)
			return tmpret;
		if (mbuf) {
			toread_m = read_len;
			memcpy(mbuf, this->mbuf_v, toread_m);
			mbuf += toread_m;
			read_len -= toread_m;
			ops->retlen += toread_m;
		}
		if (obb_buf) {
			sbuf_v = this->sbuf_v;
			for (i = 0; i < page_num; i++) {
				toread_s =
					min(max_obb_read_len - ops->ooboffs,
					obb_read_len);
				memcpy(obb_buf,
					sbuf_v + ops->ooboffs, toread_s);
				obb_buf += toread_s;
				sbuf_v += max_obb_read_len;
				obb_read_len -= toread_s;
			}
			ops->oobretlen = ops->ooblen - obb_read_len;
		}
	}
	if (mtd->ecc_stats.failed - stats.failed)
		return -EBADMSG;

	return ret;
}

static int nand_write_oob(struct sprd_nfc_base *this, loff_t to,
		struct mtd_oob_ops *ops)
{
	int ret = 0;
	uint32_t page_s, page_e, page_num;
	uint32_t page_per_buf, remain_page_inbuf, ret_num, i;

	uint32_t read_len;
	uint32_t obb_read_len;
	uint32_t max_obb_read_len;
	uint8_t *mbuf, *obb_buf, *sbuf_v;
	uint32_t toread_m, toread_s, col;

	ops->retlen = 0;
	ops->oobretlen = 0;
	read_len = ops->len;
	obb_read_len = ops->ooblen;
	if (ops->mode == MTD_OPS_AUTO_OOB)
		max_obb_read_len = this->layout.oobavail;
	else
		max_obb_read_len = this->obb_size;

	mbuf = ops->datbuf;
	obb_buf = ops->oobbuf;
	sbuf_v = this->sbuf_v;
	col = to & this->page_mask;
	page_s = to >> this->pageshift;

	/* check whether write spl image */
	if (to < NFC_SPL_MAX_SIZE)
		page_per_buf = 1;
	else
		page_per_buf = this->page_per_buf;

	if (mbuf) {
		if (!read_len)
			return 0;

		if ((obb_buf) && (max_obb_read_len <= ops->ooboffs))
			return -EINVAL;

		page_e = ((to + read_len - 1) >> this->pageshift);
		page_num = page_e - page_s + 1;
	} else if (obb_buf) {
		if (((max_obb_read_len) && (max_obb_read_len <= ops->ooboffs))
		    || ((!max_obb_read_len) && (ops->ooboffs > 0))
		    || (max_obb_read_len < (ops->ooboffs + obb_read_len))
		    )
			return -EINVAL;

		if (!obb_read_len)
			return 0;

		page_num = obb_read_len / (max_obb_read_len - ops->ooboffs);
		if (obb_read_len % (max_obb_read_len - ops->ooboffs))
			page_num++;
		else
			return -EINVAL;
	}
	if ((page_s + page_num) > this->page_per_mtd)
		return -EINVAL;

	if ((!mbuf) && (ops->mode != MTD_OPS_RAW)) {
		memset(this->mbuf_v, 0xFF,
			(min(page_per_buf, page_num)) << this->pageshift);
	}
	if ((!obb_buf) && (ops->mode != MTD_OPS_RAW)) {
		memset(this->sbuf_v, 0xFF,
				(min(page_per_buf, page_num)) * this->obb_size);
	}

	remain_page_inbuf = (page_per_buf - (page_s & (page_per_buf - 1)));
	if (remain_page_inbuf >= page_num) {
		if (mbuf) {
			memset(this->mbuf_v, 0xFF, col);
			memcpy(this->mbuf_v + col, mbuf, read_len);
			if ((col + read_len) & this->page_mask) {
				memset(this->mbuf_v + col + read_len,
					0xFF, ((1 << this->pageshift) -
					((col + read_len) & this->page_mask)));
			}
		}
		if (obb_buf) {
			sbuf_v = this->sbuf_v;
			for (i = 0; i < page_num; i++) {
				toread_s = min(max_obb_read_len - ops->ooboffs,
							obb_read_len);
				memset(sbuf_v,
					0xFF, ops->ooboffs);
				memcpy(sbuf_v + ops->ooboffs,
					obb_buf, toread_s);
				memset(sbuf_v + ops->ooboffs + toread_s,
					0xFF,
					max_obb_read_len -
						ops->ooboffs - toread_s);
				obb_buf += toread_s;
				obb_read_len -= toread_s;
				sbuf_v += max_obb_read_len;
			}
		}
		ret = this->write_page_in_blk(this, page_s, page_num, &ret_num,
				(uint32_t) (!!ops->datbuf),
				(uint32_t) (!!ops->oobbuf), ops->mode);
		if (ret == 0) {
			if (ops->datbuf)
				ops->retlen = read_len;

			if (ops->oobbuf)
				ops->oobretlen = ops->ooblen - obb_read_len;
		}
		return ret;
	}
	if (mbuf) {
		toread_m = ((remain_page_inbuf << this->pageshift) - col);
		memset(this->mbuf_v, 0xFF, col);
		memcpy(this->mbuf_v + col, mbuf, toread_m);
		mbuf += toread_m;
		read_len -= toread_m;
		col = 0;
	}
	if (obb_buf) {
		sbuf_v = this->sbuf_v;
		for (i = 0; i < remain_page_inbuf; i++) {
			toread_s = min(max_obb_read_len - ops->ooboffs,
				obb_read_len);
			memset(sbuf_v, 0xFF, ops->ooboffs);
			memcpy(sbuf_v + ops->ooboffs, obb_buf, toread_s);
			memset(sbuf_v + ops->ooboffs + toread_s, 0xFF,
				max_obb_read_len - ops->ooboffs - toread_s);
			obb_buf += toread_s;
			sbuf_v += max_obb_read_len;
			obb_read_len -= toread_s;
		}
	}
	ret = this->write_page_in_blk(this, page_s, remain_page_inbuf, &ret_num,
			(uint32_t) (!!ops->datbuf), (uint32_t) (!!ops->oobbuf),
			ops->mode);
	if (ret < 0)
		return ret;

	if (ret == 0) {
		if (ops->datbuf)
			ops->retlen += toread_m;

		if (ops->oobbuf)
			ops->oobretlen = ops->ooblen - obb_read_len;
	}

		page_s += remain_page_inbuf;
		page_num -= remain_page_inbuf;

	while (page_num >= page_per_buf) {
		if (mbuf) {
			toread_m =
				min(read_len,
					(page_per_buf << this->pageshift));
			memcpy(this->mbuf_v, mbuf, toread_m);
			mbuf += toread_m;
			read_len -= toread_m;
		}
		if (obb_buf) {
			sbuf_v = this->sbuf_v;
			for (i = 0; i < page_per_buf; i++) {
				toread_s = min(max_obb_read_len - ops->ooboffs,
					obb_read_len);
				memset(sbuf_v, 0xFF, ops->ooboffs);
				memcpy(sbuf_v + ops->ooboffs,
					obb_buf, toread_s);
				memset(sbuf_v + ops->ooboffs + toread_s,
					0xFF, max_obb_read_len -
						ops->ooboffs - toread_s);
				obb_buf += toread_s;
				sbuf_v += max_obb_read_len;
				obb_read_len -= toread_s;
			}
		}
		ret = this->write_page_in_blk(this, page_s, page_per_buf,
				&ret_num, (uint32_t) (!!ops->datbuf),
				(uint32_t) (!!ops->oobbuf), ops->mode);
		if (ret < 0)
			return ret;

		if (ret == 0) {
			if (ops->datbuf)
				ops->retlen += toread_m;

			if (ops->oobbuf)
				ops->oobretlen = ops->ooblen - obb_read_len;

		}
		page_s += page_per_buf;
		page_num -= page_per_buf;
	}
	if (page_num) {
		if (mbuf) {
			toread_m = read_len;
			memcpy(this->mbuf_v, mbuf, toread_m);
			if (toread_m & this->page_mask) {
				memset(this->mbuf_v + toread_m, 0xFF,
					((1 << this->pageshift) -
						(toread_m & this->page_mask)));
			}
			mbuf += toread_m;
			read_len -= toread_m;
		}
		if (obb_buf) {
			sbuf_v = this->sbuf_v;
			for (i = 0; i < page_num; i++) {
				toread_s = min(max_obb_read_len - ops->ooboffs,
					obb_read_len);
				memset(sbuf_v, 0xFF, ops->ooboffs);
				memcpy(sbuf_v + ops->ooboffs,
					obb_buf, toread_s);
				memset(sbuf_v + ops->ooboffs + toread_s,
					0xFF,
					max_obb_read_len -
						ops->ooboffs - toread_s);
				obb_buf += toread_s;
				sbuf_v += max_obb_read_len;
				obb_read_len -= toread_s;
			}
		}
		ret = this->write_page_in_blk(this, page_s, page_num,
				&ret_num, (uint32_t) (!!ops->datbuf),
				(uint32_t) (!!ops->oobbuf), ops->mode);
		if (ret < 0)
			return ret;

		if (ret == 0) {
			if (ops->datbuf)
				ops->retlen += toread_m;

			if (ops->oobbuf)
				ops->oobretlen = ops->ooblen - obb_read_len;
		}
	}

	return ret;
}

static int callback_block_isbad(struct mtd_info *mtd, loff_t ofs)
{
	struct sprd_nfc_base *this = (struct sprd_nfc_base *) mtd->priv;
	int ret;

	sprd_nfc_get_device(this, FL_READING);
	ret = is_bbt(ofs >> this->blkshift);
	sprd_nfc_put_device(this, __FILE__, ret);

	return ret;
}

static int callback_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	struct sprd_nfc_base *this = (struct sprd_nfc_base *) mtd->priv;
	int ret;
	uint32_t page;

	if (is_bbt(ofs >> this->blkshift))
		return 0;

	page = ((ofs >> this->blkshift) << (this->blkshift - this->pageshift));

	sprd_nfc_get_device(this, FL_ERASING);
	ret = this->nand_erase_blk(this, page);
	sprd_nfc_put_device(this, __FILE__, ret);

	sprd_nfc_get_device(this, FL_WRITING);
	ret = this->nand_mark_bad_blk(this, page);
	sprd_nfc_put_device(this, __FILE__, ret);

	set_bbt(ofs >> this->blkshift);

	return ret;
}

static int callback_read(struct mtd_info *mtd, loff_t from,
		size_t len, size_t *retlen, u_char *buf)
{
	struct sprd_nfc_base *this = (struct sprd_nfc_base *) mtd->priv;
	int ret;
	struct mtd_oob_ops ops;

	memset(&ops, 0, sizeof(struct mtd_oob_ops));
	ops.mode = MTD_OPS_PLACE_OOB;
	ops.len = len;
	ops.datbuf = (uint8_t *) buf;
	ops.oobbuf = NULL;
	sprd_nfc_get_device(this, FL_READING);
	ret = nand_read_oob(mtd, this, from, &ops, &mtd->ecc_stats);
	*retlen = ops.retlen;
	sprd_nfc_put_device(this, __FILE__, ret);

	return ret;
}

static int callback_write(struct mtd_info *mtd, loff_t to,
		size_t len, size_t *retlen, const u_char *buf)
{
	struct sprd_nfc_base *this = (struct sprd_nfc_base *) mtd->priv;
	int ret;
	struct mtd_oob_ops ops;

	memset(&ops, 0, sizeof(struct mtd_oob_ops));
	ops.mode = MTD_OPS_PLACE_OOB;
	ops.len = len;
	ops.datbuf = (uint8_t *) buf;
	ops.oobbuf = NULL;
	sprd_nfc_get_device(this, FL_WRITING);
	ret = nand_write_oob(this, to, &ops);
	*retlen = ops.retlen;
	sprd_nfc_put_device(this, __FILE__, ret);

	return ret;
}

static int callback_read_oob(struct mtd_info *mtd, loff_t from,
		struct mtd_oob_ops *ops)
{
	struct sprd_nfc_base *this = (struct sprd_nfc_base *) mtd->priv;
	int ret;

	sprd_nfc_get_device(this, FL_READING);
	ret = nand_read_oob(mtd, this, from, ops, &mtd->ecc_stats);
	sprd_nfc_put_device(this, __FILE__, ret);

	return ret;
}

static int callback_write_oob(struct mtd_info *mtd, loff_t to,
		struct mtd_oob_ops *ops)
{
	struct sprd_nfc_base *this = (struct sprd_nfc_base *) mtd->priv;
	int ret;

	sprd_nfc_get_device(this, FL_WRITING);
	ret = nand_write_oob(this, to, ops);
	sprd_nfc_put_device(this, __FILE__, ret);

	return ret;
}

static int callback_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct sprd_nfc_base *this = (struct sprd_nfc_base *) mtd->priv;
	int ret;
	uint32_t blk_s, blk_e, i;

	if ((instr->addr & this->blk_mask) || (instr->len & this->blk_mask)) {
		/* check whether addr and len is block align */
		return -EINVAL;
	}

	sprd_nfc_get_device(this, FL_ERASING);
	do {
		blk_s = instr->addr >> this->blkshift;
		blk_e = (instr->addr + instr->len - 1) >> this->blkshift;

		instr->state = MTD_ERASING;
		for (i = blk_s; i <= blk_e; i++) {
			if (is_bbt(i) && (!this->allow_erase_badblock)) {
				instr->state = MTD_ERASE_FAILED;
				break;
			}
			if (this->nand_erase_blk(this, i << (this->blkshift -
							this->pageshift))) {
				instr->fail_addr = (i << this->blkshift);
				instr->state = MTD_ERASE_FAILED;
				break;
			}
		}
		if (instr->state == MTD_ERASING)
			instr->state = MTD_ERASE_DONE;

	} while (0);
	ret = (instr->state == MTD_ERASE_DONE ? 0 : -EIO);
	sprd_nfc_put_device(this, __FILE__, ret);

	if (!ret)
		mtd_erase_callback(instr);

	return ret;
}

static void callback_sync(struct mtd_info *mtd)
{
	struct sprd_nfc_base *this = (struct sprd_nfc_base *) mtd->priv;

	sprd_nfc_get_device(this, FL_SYNCING);
	sprd_nfc_put_device(this, __FILE__, 0);
}

static int callback_suspend(struct mtd_info *mtd)
{
	struct sprd_nfc_base *this = (struct sprd_nfc_base *) mtd->priv;

	sprd_nfc_get_device(this, FL_PM_SUSPENDED);
	this->ctrl_suspend(this);
	return 0;
}

static void callback_resume(struct mtd_info *mtd)
{
	struct sprd_nfc_base *this = (struct sprd_nfc_base *) mtd->priv;

	if (this->state == FL_PM_SUSPENDED) {
		this->ctrl_resume(this);
		sprd_nfc_put_device(this, __FILE__, 0);
	} else
		pr_err("nfc_base: resume multi\n");
}

static void init_mtd(struct mtd_info *mtd, struct sprd_nfc_base *this)
{
	memset((char *)mtd, 0, sizeof(struct mtd_info));
	mtd->priv = this;
	mtd->type = MTD_NANDFLASH;
	mtd->flags = MTD_CAP_NANDFLASH;
	mtd->size = (1 << this->chip_shift) * this->chip_num;
	mtd->erasesize = (1 << this->blkshift);
	mtd->writesize = (1 << this->pageshift);
	mtd->writebufsize = (1 << this->bufshift);
	mtd->oobsize = this->obb_size;
	mtd->oobavail = this->layout.oobavail;

	mtd->bitflip_threshold = this->risk_threshold;
	mtd->name = "sprd-nand";

	mtd->ecclayout = &this->layout;
	mtd->ecc_strength = this->ecc_mode;

	mtd->_erase = callback_erase;
	mtd->_point = NULL;
	mtd->_unpoint = NULL;
	mtd->_get_unmapped_area = NULL;
	mtd->_read = callback_read;
	mtd->_write = callback_write;
	mtd->_panic_write = NULL;
	mtd->_read_oob = callback_read_oob;
	mtd->_write_oob = callback_write_oob;
	mtd->_get_fact_prot_info = NULL;
	mtd->_read_fact_prot_reg = NULL;
	mtd->_get_user_prot_info = NULL;
	mtd->_read_user_prot_reg = NULL;
	mtd->_write_user_prot_reg = NULL;
	mtd->_lock_user_prot_reg = NULL;
	mtd->_writev = NULL;
	mtd->_sync = callback_sync;
	mtd->_lock = NULL;
	mtd->_unlock = NULL;
	mtd->_is_locked = NULL;
	mtd->_block_isbad = callback_block_isbad;
	mtd->_block_markbad = callback_block_markbad;
	mtd->_suspend = callback_suspend;
	mtd->_resume = callback_resume;
	mtd->_get_device = NULL;
	mtd->_put_device = NULL;

	mtd->subpage_sft = 0;
	mtd->owner = THIS_MODULE;
}

int sprd_nfc_base_register(struct sprd_nfc_base *this)
{
	int ret;

	sprd_nfc_init_device(this);
	init_param(this);
	init_bbt(this);
	init_mtd(&sprd_mtd, this);
	ret = mtd_device_parse_register(&sprd_mtd, part_probes, 0, 0, 0);

	return 0;
}

