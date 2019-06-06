
#include <linux/mtd/flashchip.h>

#define NFC_MAX_ID_LEN 8
#define NFC_SPL_MAX_SIZE (64 << 10)

struct sprd_nfc_base {
/* resource0:These information is initialize by nfc base self; */
	spinlock_t lock;
	flstate_t state;
	wait_queue_head_t wq;

	uint32_t page_mask;
	uint32_t blk_mask;
	uint32_t page_per_buf;
	uint32_t page_per_mtd;
	uint32_t *bbt;

	struct dentry *debugfs_root;
	uint32_t allow_erase_badblock;

/* resource1:This information must be give by user */
	uint8_t id[NFC_MAX_ID_LEN];
	uint32_t ecc_mode;
	uint32_t risk_threshold;
	struct nand_ecclayout layout;
	uint32_t obb_size;
	uint32_t pageshift;
	uint32_t blkshift;
	uint32_t chip_shift;
	uint32_t chip_num;
	uint32_t bufshift;

	uint8_t *mbuf_v;
	uint8_t *sbuf_v;

	int (*ctrl_en)(struct sprd_nfc_base *this, uint32_t en);
	void (*ctrl_suspend)(struct sprd_nfc_base *this);
	int (*ctrl_resume)(struct sprd_nfc_base *this);

	int (*read_page_in_blk)(struct sprd_nfc_base *this, uint32_t page,
		uint32_t num, uint32_t *ret_num, uint32_t if_has_mbuf,
		uint32_t if_has_sbuf, uint32_t mode,
		struct mtd_ecc_stats *ecc_sts);

	int (*write_page_in_blk)(struct sprd_nfc_base *this, uint32_t page,
		uint32_t num, uint32_t *ret_num, uint32_t if_has_mbuf,
		uint32_t if_has_sbuf, uint32_t mode);

	int (*nand_erase_blk)(struct sprd_nfc_base *this, uint32_t page);
	int (*nand_is_bad_blk)(struct sprd_nfc_base *this, uint32_t page);
	int (*nand_mark_bad_blk)(struct sprd_nfc_base *this, uint32_t page);

	int (*debugfs_drv_show)(struct sprd_nfc_base *this, struct seq_file *s,
		void *data);

	void *priv;
};
int sprd_nfc_base_register(struct sprd_nfc_base *this);
