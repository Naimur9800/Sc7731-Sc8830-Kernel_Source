/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _SPRD_CPROC_H
#define _SPRD_CPROC_H

#define CPROC_TAG	"sprd_cproc "

#define MAX_NAME_LEN		0x14
#define MAX_NODE_NAME_LEN	0x20
#define MAX_CPROC_ENTRY_NUM	0x10

#define DECOUPLE_MAGIC_INFO	0x5043454D
#define HEADER_MAGIC_INFO	0x44485043
#define MAX_REGION_NUM		0x8
#define MAX_RSVMEM_NUM		0x4
#define MAX_RES_NUM		0x4
#define MAX_CODE_NUM		0x32

struct mem_resource {
	char name[MAX_NAME_LEN];

	u64 base __aligned(8);
	u32 size;
};

struct region_info {
	struct mem_resource res;
	u32 flag;
};

struct boot_code {
	u32 count;
	u32 code[MAX_CODE_NUM];
};

struct rsvmem_info_conf {
	struct region_info rsvmem[MAX_RSVMEM_NUM];
};

struct boot_info_conf {
	struct boot_code bcode;
	struct region_info regions[MAX_REGION_NUM];
	struct mem_resource res[MAX_RES_NUM];
	u32 ex_info;
};

struct comm_info_conf {
	struct mem_resource res[MAX_RES_NUM];
	u32 ex_info;
};

struct cp_mem_coupling_info {
	u32 magic;
	u32 version;
	u32 length;
	u32 ex_info;
	struct rsvmem_info_conf rsvmem_info;
	struct boot_info_conf boot_info;
	struct comm_info_conf comm_info;
};

/* the new format */
struct img_desc {
	char desc[MAX_NAME_LEN];
	u32 offs;
	u32 size;
};

struct iheader_desc {
	u32 magic;
	struct img_desc desc[1];
};

struct region_desc {
	char name[MAX_NAME_LEN];

	u64 base __aligned(8);
	u32 size;
	u32 attr;
};

struct mem_coupling_info {
	u32 magic;
	u32 version;
	struct region_desc region[1];
};

union composite_header {
	struct iheader_desc iheader;
	struct cp_mem_coupling_info mem_decoup;
};

struct load_node {
	char name[MAX_NODE_NAME_LEN];
	u32 base __aligned(4);
	u32 size;
};

struct cproc_dump_info {
	char parent_name[MAX_NAME_LEN];
	char name[MAX_NAME_LEN];
	u32 start_addr;
	u32 size;
};

enum {
	LOAD_FIELD = 1,
	PARAM_FIELD,
	DUMP_FIELD,
};

struct cproc_segment {
	char name[MAX_NAME_LEN];
	void *v_base;
	u32 base;
	u32 size;
	u32 attr;
};

struct cproc_controller {
	char devname[MAX_NAME_LEN];

	phys_addr_t map[2];

	void *v_base;
	phys_addr_t base;
	u32 maxsz;

	void *v_iram;
	phys_addr_t iram;
	u32 iramsz;

	int wdtirq;
	int wdtcnt;
	wait_queue_head_t wdtwait;

	int (*start)(void *arg);
	int (*stop)(void *arg);

	u32 segnr;
	struct cproc_segment segs[];
};

struct cproc_proc_fs;

struct cproc_proc_entry {
	char *name;
	umode_t				mode;
	struct proc_dir_entry *entry;
	struct cproc_device *cproc;
	unsigned index;
};

struct cproc_proc_fs {
	struct proc_dir_entry *procdir;
	struct cproc_proc_entry entrys[MAX_CPROC_ENTRY_NUM];
};

struct cproc_device {
	int status;

	struct cproc_controller *controller;
	struct platform_device *pdev;
	struct miscdevice miscdev;
	struct cproc_proc_fs procfs;
};

int load_cp_boot_code(void *info, void *loader);
int register_cproc_controller(struct cproc_controller *controller);

#endif

