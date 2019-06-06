#define SPRD_MAP_IOCTRL_MAGIC        'o'

/**
 * DOC: MAP_USER_VIR
 *
 * Takes a pmem_info struct
 */
#define MAP_USER_VIR  _IOWR(SPRD_MAP_IOCTRL_MAGIC, 0, struct pmem_info)

struct pmem_info {
	unsigned long phy_addr;
	unsigned int phys_offset;
	size_t size;
};

