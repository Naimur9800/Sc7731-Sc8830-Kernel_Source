#ifndef KMEMPAGE_RECORDER_H
#define KMEMPAGE_RECORDER_H

#include <linux/types.h>

#define OBJECT_TABLE_SIZE      1543
#define BACKTRACE_SIZE         10
#define BACKTRACE_LEVEL        10

enum page_record_types {
	NODE_PAGE_RECORD,
	NODE_PAGE_MAX
};

enum hash_page_node_types {
	HASH_PAGE_NODE_KERNEL_BACKTRACE,
	HASH_PAGE_NODE_KERNEL_SYMBOL,
	HASH_PAGE_NODE_KERNEL_PAGE_ALLOC_BACKTRACE,
	HASH_PAGE_NODE_MAX
};

/* for backtrace and symbol */
struct page_object_entry {
	size_t slot;
	struct page_object_entry *prev;
	struct page_object_entry *next;
	size_t nr_entries;
	size_t reference;
	unsigned int size;
	void *object[0];
};

struct page_object_table {
	size_t count;
	struct page_object_entry *slots[OBJECT_TABLE_SIZE];
};

/* for page */
struct page_hash_entry {
	void *page;
	unsigned int size;
	struct page_hash_entry *prev;
	struct page_hash_entry *next;
	struct page_object_entry *alloc_map_entry;
	struct page_object_entry *bt_entry;
	unsigned int flag;
};

struct page_hash_table {
	size_t count;
	size_t table_size;
	struct page_hash_entry *hash_table[OBJECT_TABLE_SIZE];
};

struct page_mapping {
	char *name;
	unsigned long address;
	unsigned int size;
};

struct page_record_params {
	unsigned int *page;
	unsigned int address_type;
	unsigned int address;
	unsigned int length;
	unsigned long backtrace[BACKTRACE_SIZE];
	unsigned int backtrace_num;
	unsigned long kernel_symbol[BACKTRACE_SIZE];
	struct page_mapping mapping_record[BACKTRACE_SIZE];
	unsigned int size;
};

/*	rank object	*/
struct page_object_rank_entry {
	struct page_object_entry *entry;
	struct page_object_rank_entry *prev;
	struct page_object_rank_entry *next;
};

void page_debug_show_backtrace(void);

int record_page_record(void *page, unsigned int order);

int remove_page_record(void *page, unsigned int order);

void disable_page_alloc_tracer(void);

#endif
