#include <linux/mm.h>
#include <linux/spinlock.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/kallsyms.h>
#include <linux/module.h>
#include <linux/stacktrace.h>
#include <linux/hash.h>
#include <linux/seq_file.h>
#include <linux/kmempagerecorder.h>

#define DEBUG_DEFAULT_FLAGS    1
#define RECORDER_LIMIT         (1024 * 1024 * 2)
#define RECORD_MAX             50412

#define PAGE_ALLOC_BACKTRACE   HASH_PAGE_NODE_KERNEL_PAGE_ALLOC_BACKTRACE
#define PAGE_OBJECT_ENTRY_LEN  (sizeof(struct page_object_entry) + \
				  20 * sizeof(unsigned long))

struct page_hash_table page_hash_table;
struct page_object_table page_symbol_table;
struct page_object_table page_bt_table;
static struct kmem_cache *page_cachep;
static unsigned int page_cache_created;

static unsigned int object_rank_max = 10;
static unsigned long queried_address;
static unsigned int debug_log;
static struct dentry *debug_root;
static unsigned int page_records_total;
static unsigned int page_records_max;
static unsigned int page_records_count;
static unsigned int bt_records_total;
static unsigned int bt_records_max;

/* init hash table mutex	*/
unsigned int page_record_lock_init;
unsigned int bt_record_lock_init;
unsigned int symbol_record_lock_init;
spinlock_t page_record_lock;
spinlock_t bt_record_lock;
spinlock_t symbol_record_lock;
int page_recorder_debug = DEBUG_DEFAULT_FLAGS;
unsigned int page_recorder_memory_usage;
unsigned int page_recorder_limit = RECORDER_LIMIT;
static char page_recorder_debug_function;

static int page_recorder_debug_show(struct seq_file *s, void *unused);
static inline struct page_hash_entry *find_page_entry(void *page, int slot);

void disable_page_alloc_tracer(void)
{
	page_recorder_debug = 0;
}

static int page_recorder_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, page_recorder_debug_show, inode->i_private);
}

static const struct file_operations debug_page_recorder_fops = {
	.open = page_recorder_debug_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int query_page_bt_open(struct seq_file *s, void *data)
{
	return 0;
}

static int query_page_single_open(struct inode *inode, struct file *file)
{
	return single_open(file, query_page_bt_open, inode->i_private);
}

static const struct file_operations query_page_ios_fops = {
	.open = query_page_single_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static unsigned int get_kernel_backtrace(unsigned long *backtrace,
					 unsigned int debug)
{
	unsigned long stack_entries[BACKTRACE_LEVEL];
	unsigned int i = 0;
	char tmp[KSYM_SYMBOL_LEN];
	struct stack_trace trace = {
		.nr_entries = 0,
		.entries = &stack_entries[0],
		.max_entries = BACKTRACE_LEVEL,
#ifdef CONFIG_64BIT
		.skip = 4
#else
		.skip = 1
#endif
	};

	save_stack_trace(&trace);
	if (trace.nr_entries > 0) {
		if (debug) {
			for (i = 0; i < trace.nr_entries; i++) {
				sprint_symbol(tmp, trace.entries[i]);
				pr_debug("[%d] 0x%lx %s\n", i,
					 trace.entries[i], tmp);
			}
		} else {
			memcpy(backtrace, (unsigned long *)trace.entries,
				sizeof(unsigned long) * trace.nr_entries);
		}
	} else {
		pr_err("Fail to get backtrace backtrace num: [%d]\n",
			trace.nr_entries);
	}

	return trace.nr_entries;
}

static uint64_t get_hash(void *object, size_t nr_entries)
{
	unsigned long *backtrace = NULL;
	unsigned long hash = 0;
	size_t i;

	backtrace = (unsigned long *)object;
	if (backtrace == NULL)
		return 0;

	for (i = 0; i < nr_entries; i++)
		hash = (hash * 33) + (*(backtrace + i) >> 2);

	return hash;
}

struct page_object_entry *find_entry(struct page_object_table *table,
				     unsigned int slot, void *object,
				     unsigned int nr_entries,
				     unsigned int size)
{
	struct page_object_entry *entry = table->slots[slot];

	while (entry != NULL) {
		if (entry->nr_entries == nr_entries &&
		    !memcmp(object, entry->object,
			    nr_entries * sizeof(unsigned long))) {
			return entry;
		}
		entry = entry->next;
	}
	return NULL;
}

static void *alloc_page_record(void)
{
	if (!page_cache_created) {
		/* page_cachep = kmem_cache_create("page_record",
		   sizeof(struct page_hash_entry),0,SLAB_NO_DEBUG,NULL); */
		page_cachep = kmem_cache_create("page_record",
						sizeof(struct page_hash_entry),
						0, 0, NULL);
		page_cache_created = true;
	}

	/* if system ram < 2G, page_records_total should less than 524288 */
	if ((page_cachep != NULL)
	    && (page_records_total < page_recorder_limit)) {
		return  (void *)kmem_cache_alloc(page_cachep, GFP_KERNEL);
	}

	return NULL;
}

static void *allocate_record(unsigned int type)
{
	switch (type) {
	case NODE_PAGE_RECORD:
	   return alloc_page_record();
	}

	return NULL;
}

static struct page_object_entry *create_kernel_backtrace(void)
{
	struct page_object_entry *entry = NULL;

	/* total bt reocrd size should less than 5MB */
	if (bt_records_total >= RECORD_MAX) {
		pr_err("total records outnumber max value.\n");
		return NULL;
	}

	entry = kmalloc(sizeof(struct page_object_entry) +
			(20 * sizeof(unsigned long)), GFP_KERNEL);
	if (entry == NULL) {
		pr_err("%s can't get memory from kmalloc.\n", __func__);
		return NULL;
	}

	/* kmalloc can't get right memory space when booting */
	if ((unsigned long)entry < STACK_TOP_MAX) {
		pr_err("entry (%p) drop address\n", entry);
		return NULL;
	}

	return entry;
}

static struct page_object_entry *alloc_kernel_backtrace(
		struct page_record_params *param)
{
	unsigned long hash;
	unsigned int slot;
	unsigned long flags;
	struct page_object_entry *entry = NULL;

	hash = get_hash(param->backtrace, param->backtrace_num);
	slot = hash % OBJECT_TABLE_SIZE;

	spin_lock_irqsave(&bt_record_lock, flags);
	entry = find_entry(&page_bt_table, slot, (void *)param->backtrace,
			   param->backtrace_num, param->size);
	if (entry != NULL) {
		entry->reference++;
		entry->size += (1 << param->size);
		pr_debug("%s entry->size %d\n", __func__, entry->size);
		spin_unlock_irqrestore(&bt_record_lock, flags);
	} else {
		spin_unlock_irqrestore(&bt_record_lock, flags);

		entry = create_kernel_backtrace();
		if (entry == NULL) {
			pr_err("Fail to create backtrace record.\n");
			return NULL;
		}

		entry->reference = 1;
		entry->prev = NULL;
		entry->slot = slot;
		entry->nr_entries = param->backtrace_num;
		entry->size = 1 << param->size;
		pr_debug("%s new entry->size %d\n", __func__, entry->size);
		memcpy(entry->object, param->backtrace,
		       entry->nr_entries * sizeof(unsigned long));

		spin_lock_irqsave(&bt_record_lock, flags);
		entry->next = page_bt_table.slots[slot];
		page_bt_table.slots[slot] = entry;
		if (entry->next != NULL)
			entry->next->prev = entry;
		page_bt_table.count++;

		bt_records_total++;
		if (bt_records_total > bt_records_max)
			bt_records_max = bt_records_total;

		spin_unlock_irqrestore(&bt_record_lock, flags);
	}
	return entry;
}

static struct page_object_entry *alloc_kernel_symbol(
		struct page_record_params *param)
{
	struct page_object_entry *entry = NULL;
	unsigned long hash;
	unsigned int slot;
	unsigned long flags;

	hash = get_hash(param->kernel_symbol, param->backtrace_num);
	slot = hash % OBJECT_TABLE_SIZE;

	spin_lock_irqsave(&symbol_record_lock, flags);
	entry = find_entry(&page_symbol_table, slot,
			   (void *)param->kernel_symbol,
			   param->backtrace_num, param->size);

	if (entry != NULL) {
		entry->reference++;
		spin_unlock_irqrestore(&symbol_record_lock, flags);
	} else {
		spin_unlock_irqrestore(&symbol_record_lock, flags);
		entry = kmalloc(sizeof(struct page_object_entry) +
				(param->backtrace_num * sizeof(unsigned long)),
				GFP_KERNEL);
		if (entry == NULL) {
			pr_err("%s can't get memory from kmalloc\n", __func__);
			return NULL;
		}
		entry->reference = 1;
		entry->prev = NULL;
		entry->slot = slot;
		entry->nr_entries = param->backtrace_num;
		memcpy(entry->object, param->kernel_symbol,
		       entry->nr_entries * sizeof(unsigned long));

		spin_lock_irqsave(&symbol_record_lock, flags);
		entry->next = page_symbol_table.slots[slot];
		page_symbol_table.slots[slot] = entry;
		if (entry->next != NULL)
			entry->next->prev = entry;
		page_symbol_table.count++;
		spin_unlock_irqrestore(&symbol_record_lock, flags);
	}
	return entry;
}

/* get record from hash table or create new node from slab allocator */
static void *get_record(unsigned int type, struct page_record_params *record)
{
	if (record == NULL) {
		pr_err("%s record is null", __func__);
		return NULL;
	}

	switch (type) {
	case PAGE_ALLOC_BACKTRACE:
		return (void *)alloc_kernel_backtrace(record);
	case HASH_PAGE_NODE_KERNEL_SYMBOL:
		return (void *)alloc_kernel_symbol(record);
	}
	return NULL;
}

static inline struct page_hash_entry *find_page_entry(void *page, int slot)
{
	struct page_hash_entry *entry = page_hash_table.hash_table[slot];

	while (entry != NULL) {
		if (entry->page == page)
			return entry;
		entry = entry->next;
	}
	return NULL;
}

struct page_hash_entry *record_page_info(struct page_object_entry *bt_entry,
				struct page_object_entry *map_entry, void *page,
				unsigned int order, unsigned int flag)
{
	/* calculate the hash value */
	unsigned long hash = hash_ptr((const void *)page, 16);
	unsigned int slot = hash % OBJECT_TABLE_SIZE;
	unsigned long flags;

	struct page_hash_entry *entry =
	    (struct page_hash_entry *) allocate_record(NODE_PAGE_RECORD);
	if (!entry) {
		pr_err("%s Fail to create page record entry.\n", __func__);
		return NULL;
	}
	/* initialize page entry */
	entry->page = page;
	entry->size = 1 << order;
	entry->alloc_map_entry = map_entry;
	entry->bt_entry = bt_entry;
	entry->flag = (2 | flag);
	entry->prev = NULL;
	spin_lock_irqsave(&page_record_lock, flags);

	/* insert the entry to the head of slot list */
	if (page_hash_table.hash_table[slot] == NULL) {
		entry->next = NULL;
	} else {
		(page_hash_table.hash_table[slot])->prev = entry;
		entry->next = page_hash_table.hash_table[slot];
	}
	page_hash_table.hash_table[slot] = entry;
	page_hash_table.count++;
	page_records_total++;
	if (page_records_total > page_records_max)
		page_records_max = page_records_total;
	page_records_count++;
	if (page_records_count >= 1000) {
		page_recorder_memory_usage =
		    page_records_total * sizeof(struct page_hash_entry) +
		    bt_records_total * (sizeof(struct page_object_entry) +
				       (20 * sizeof(unsigned long)));
#ifdef CONFIG_64BIT
		pr_debug
		    ("page record size is %lu max page record size is %lu\n",
#else
		pr_debug
		    ("page record size is %d max page record size is %d\n",
#endif
		     page_records_total * sizeof(struct page_hash_entry),
		     page_records_max * sizeof(struct page_hash_entry));
#ifdef CONFIG_64BIT
		pr_debug
		    ("bt record size is %lu max bt record size is %lu\n",
#else
		pr_debug
		    ("bt record size is %d max bt record size is %d\n",
#endif
		     bt_records_total * (sizeof(struct page_object_entry) +
					(20 * sizeof(unsigned long))),
		     bt_records_max * (sizeof(struct page_object_entry) +
				      (20 * sizeof(unsigned long))));
		page_records_count = 0;
	}

	spin_unlock_irqrestore(&page_record_lock, flags);
	return entry;
}

int remove_page_info(void *page, unsigned int order)
{
	unsigned long hash = hash_ptr((const void *)page, 16);
	unsigned int slot = hash % OBJECT_TABLE_SIZE;
	struct page_object_entry *bt_entry = NULL;
	struct page_hash_entry *entry = NULL;
	unsigned long flags;

	/* search page record in hash table */
	if (page_record_lock_init == 0) {
		page_record_lock_init = 1;
		spin_lock_init(&page_record_lock);
	}
	if (bt_record_lock_init == 0) {
		bt_record_lock_init = 1;
		spin_lock_init(&bt_record_lock);
	}

	spin_lock_irqsave(&page_record_lock, flags);
	pr_debug("remove_page_info page *%p order%d\n", page, order);
	entry = find_page_entry(page, slot);
	if (entry == NULL) {
		spin_unlock_irqrestore(&page_record_lock, flags);
		pr_err("[remove_page_info]can't find page info %p\n", page);
		if (debug_log)
			get_kernel_backtrace(NULL, 1);
		return -1;
	}
	/* remove page record from hash table */
	/* head */
	if (entry->prev == NULL) {
		page_hash_table.hash_table[slot] = entry->next;
		/* not only one entry in the slot */
		if (page_hash_table.hash_table[slot] != NULL)
			page_hash_table.hash_table[slot]->prev = NULL;
	} else if (entry->next == NULL) {
		entry->prev->next = NULL;
	} else {
		entry->next->prev = entry->prev;
		entry->prev->next = entry->next;
	}

	page_hash_table.count--;
	page_records_total--;
	spin_unlock_irqrestore(&page_record_lock, flags);

	/* clean page entry */
	entry->next = NULL;
	entry->prev = NULL;
	bt_entry = entry->bt_entry;
	kmem_cache_free(page_cachep, entry);

	/* create alloc bt entry for historical allocation */
	if (bt_entry == NULL)
		return -1;

	spin_lock_irqsave(&bt_record_lock, flags);
	if (bt_entry->reference > 1) {
		(bt_entry->reference)--;
		bt_entry->size = bt_entry->size - (1 << order);
		spin_unlock_irqrestore(&bt_record_lock, flags);
		pr_debug("[remove_page_info] bt_entry->size %d\n",
			 bt_entry->size);
	} else if (bt_entry->reference == 1) {
		unsigned long hash_bt;
		unsigned int slot_bt;

		hash_bt = get_hash(bt_entry->object, bt_entry->nr_entries);
		slot_bt = hash_bt % OBJECT_TABLE_SIZE;

		if (bt_entry->prev == NULL) {
			page_bt_table.slots[slot_bt] = bt_entry->next;
			/* not only one entry in the slot */
			if (page_bt_table.slots[slot_bt] != NULL)
				page_bt_table.slots[slot_bt]->prev = NULL;
		} else if (bt_entry->next == NULL) {
			bt_entry->prev->next = NULL;
		} else {
			bt_entry->next->prev = bt_entry->prev;
			bt_entry->prev->next = bt_entry->next;
		}
		spin_unlock_irqrestore(&bt_record_lock, flags);
		bt_records_total--;
		kfree(bt_entry);
	} else {
		spin_unlock_irqrestore(&bt_record_lock, flags);
		pr_err("ERROR !!!!free page info\n");
	}

	return 0;
}

int record_page_record(void *page, unsigned int order)
{
	void *entry, *map_entry = NULL;
	struct page_record_params record;

	if (!page)
		return 0;

	pr_debug("record_page_record enter page*%p, order:%d\n", page, order);

	if (!page_recorder_debug)
		return 0;

	if (page_record_lock_init == 0) {
		page_record_lock_init = 1;
		spin_lock_init(&page_record_lock);
	}
	if (bt_record_lock_init == 0) {
		bt_record_lock_init = 1;
		spin_lock_init(&bt_record_lock);
	}

	record.page = page;
	record.size = order;
	record.backtrace_num = get_kernel_backtrace((unsigned long *)
						    record.backtrace,
						    (unsigned int)0);

	entry = get_record(PAGE_ALLOC_BACKTRACE, &record);
	if (entry == NULL) {
		pr_err("can't get enough memory to create backtrace object\n");
		return 0;
	}

	record_page_info((struct page_object_entry *) entry,
			 (struct page_object_entry *) map_entry,
			 record.page, record.size, 0);

	pr_debug("record_page_record exit page*%p, order:%d\n", page, order);
	return 1;
}
EXPORT_SYMBOL(record_page_record);

int remove_page_record(void *page, unsigned int order)
{
	struct page_record_params record_param;

	record_param.page = page;
	record_param.size = order;

	if (!page)
		return 0;

	if (!page_recorder_debug)
		return 0;

	remove_page_info(record_param.page, record_param.size);
	return 1;
}
EXPORT_SYMBOL(remove_page_record);


/*
 * setup rank list
 * @pgobj                object in the rank list
 * @max_object_rank      the length of the rank list
 * @pobject_rank_count   current object count in the rank list
 * @ **rank_head         the list which by setup
 */
static int setup_rank_list(struct page_object_entry *pgobj,
			   unsigned int max_object_rank,
			   unsigned int *pobject_rank_count,
			   struct page_object_rank_entry **prank_head,
			   struct page_object_rank_entry **prank_tail)
{
	unsigned int rank_index = 0;
	struct page_object_entry *tmp = pgobj;

	struct page_object_rank_entry *rank_tmp = *prank_head;
	struct page_object_rank_entry *rank_tmp_prev = *prank_head;

	for (rank_index = 0; rank_index < max_object_rank; rank_index++) {
		struct page_object_rank_entry *new_rank_entry = NULL;
		struct page_object_entry *entry = NULL;

		if ((rank_tmp != NULL)
		    && (rank_tmp->entry->size <= tmp->size)) {
			/* insert current record into list */
			struct page_object_entry *entry = NULL;

			new_rank_entry = (struct page_object_rank_entry *)
					  kmalloc(sizeof(struct
						   page_object_rank_entry),
						  GFP_ATOMIC);
			if (new_rank_entry == NULL) {
				pr_err
				    ("%s fail to alloc rank entry\n", __func__);
				return -1;
			}
			entry = kmalloc(PAGE_OBJECT_ENTRY_LEN, GFP_ATOMIC);
			if (entry == NULL) {
				pr_err
				    ("%s fail to alloc page entry\n", __func__);
				return -1;
			}

			memcpy(entry, tmp, PAGE_OBJECT_ENTRY_LEN);
			new_rank_entry->entry = entry;
			new_rank_entry->prev = rank_tmp->prev;
			if (rank_tmp->prev != NULL)
				rank_tmp->prev->next = new_rank_entry;

			rank_tmp->prev = new_rank_entry;
			new_rank_entry->next = rank_tmp;
			if (new_rank_entry->prev == NULL)
				*prank_head = new_rank_entry;

			if (*pobject_rank_count < max_object_rank) {
				(*pobject_rank_count)++;
			} else {
				/* free last rank_entry */
				if (*prank_tail != NULL) {
					struct page_object_rank_entry
							*new_tail = NULL;

					new_tail =  (*prank_tail)->prev;
					(*prank_tail)->prev->next = NULL;
					kfree((*prank_tail)->entry);
					kfree(*prank_tail);
					*prank_tail = new_tail;
				} else {
					pr_err("ERROR!!! rank_tail is NULL\n");
				}
			}
			break;
		} else if ((rank_tmp == NULL)
		    && (*pobject_rank_count < max_object_rank)) {
			/* if rank entry is less than object_entry_max,
			   create new rank entry and insert it in rank list */
			new_rank_entry = (struct page_object_rank_entry *)
					  kmalloc(sizeof(struct
							page_object_rank_entry),
						  GFP_ATOMIC);
			if (new_rank_entry == NULL) {
				pr_err
				    ("%s fail to alloc rank entry\n", __func__);
				return -1;
			}

			entry = kmalloc(PAGE_OBJECT_ENTRY_LEN, GFP_ATOMIC);
			if (entry == NULL) {
				pr_err
				    ("%s fail to alloc page entry\n", __func__);
				return -1;
			}

			memcpy(entry, tmp, PAGE_OBJECT_ENTRY_LEN);
			new_rank_entry->entry = entry;
			new_rank_entry->next = NULL;
			new_rank_entry->prev = rank_tmp_prev;

			if (rank_tmp_prev != NULL)
				rank_tmp_prev->next = new_rank_entry;
			if (new_rank_entry->prev == NULL)
				*prank_head = new_rank_entry;

			*prank_tail = new_rank_entry;
			(*pobject_rank_count)++;
			break;
		}
		rank_tmp_prev = rank_tmp;
		rank_tmp = rank_tmp->next;
	}

	return 0;
}

/* print top object_rank_max record */
static int show_rank_list(struct seq_file *s,
			  struct page_object_rank_entry *rank_head)
{
	unsigned long *backtrace;
	unsigned int rank_index = 0;
	char symbol[KSYM_SYMBOL_LEN];
	unsigned int i = 0;
	struct page_object_rank_entry *rank_tmp = rank_head;
	struct page_object_rank_entry *tmp_record = NULL;

	while (rank_tmp != NULL) {
		backtrace = (unsigned long *)rank_tmp->entry->object;
		seq_printf(s, "[%d]%s %d %s\n", rank_index, "Backtrace pages ",
				rank_tmp->entry->size * 4096, "bytes");
		for (i = 0; i < rank_tmp->entry->nr_entries; i++) {
			sprint_symbol(symbol, *(backtrace + i));
			seq_printf(s, "  KERNEL[%d] 0x%lx :: symbol %s\n", i,
					backtrace[i], symbol);
		}

		rank_index++;
		tmp_record = rank_tmp;
		rank_tmp = rank_tmp->next;
	}
	return 0;
}

static int free_rank_list(struct page_object_rank_entry *rank_head)
{
	struct page_object_rank_entry *rank_tmp = rank_head;
	struct page_object_rank_entry *tmp_record = NULL;

	while (rank_tmp != NULL) {
		tmp_record = rank_tmp;
		rank_tmp = rank_tmp->next;
		kfree(tmp_record->entry);
		kfree(tmp_record);
	}
	return 0;
}

static int page_recorder_debug_show(struct seq_file *s, void *unused)
{
	unsigned int index = 0;
	unsigned long flags;
	struct page_object_rank_entry *rank_head = NULL;
	struct page_object_rank_entry *rank_tail = NULL;
	unsigned int object_rank_count = 0;
	struct page_object_entry *tmp = NULL;

	seq_printf(s, "page_recorder_debug: [%d]\n", page_recorder_debug);
	seq_printf(s, "page_recorder_limit: [%d]\n", page_recorder_limit);
	seq_printf(s, "TOP %d page allocation\n", object_rank_max);

	for (index = 0; index < OBJECT_TABLE_SIZE; index++) {
		spin_lock_irqsave(&bt_record_lock, flags);
		tmp = page_bt_table.slots[index];
		while (tmp != NULL) {
			setup_rank_list(tmp,
					object_rank_max,
					&object_rank_count,
					&rank_head,
					&rank_tail);
			tmp = tmp->next;
		}
		spin_unlock_irqrestore(&bt_record_lock, flags);
	}

	show_rank_list(s, rank_head);
	free_rank_list(rank_head);

	return 0;
}

static int __init setup_page_recorder_debug(char *str)
{
	page_recorder_debug = DEBUG_DEFAULT_FLAGS;
	if (*str++ != '=' || !*str)
		/*
		 * No options specified. Switch on full debugging.
		 */
		goto out;

	if (*str == ',')
		/*
		 * No options but restriction on page recorder. This means full
		 * debugging for page recorder matching a pattern.
		 */
		goto check_page_recorder;

	page_recorder_debug = 0;
	if (*str == '-')
		/*
		 * Switch off all debugging measures.
		 */
		goto out;

check_page_recorder:
	if (*str == ',')
		page_recorder_debug_function = *(str + 1);
out:
	return 1;
}

__setup("page_recorder_debug", setup_page_recorder_debug);

#ifdef CONFIG_E_SHOW_MEM
static int page_recorder_debug_show_printk(enum e_show_mem_type type)
{
	unsigned int index = 0;
	unsigned long flags;
	struct page_object_rank_entry *rank_head = NULL;
	struct page_object_rank_entry *rank_tail = NULL;
	unsigned int object_rank_count = 0;
	struct page_object_entry *tmp = NULL;
	unsigned int temp_object_rank_max;
	unsigned long long total_used = 0;
	unsigned long *backtrace;
	unsigned int i, rank_index = 0;
	char symbol[KSYM_SYMBOL_LEN];
	struct page_object_rank_entry *rank_tmp = NULL;
	struct page_object_rank_entry *tmp_record = NULL;

	if (E_SHOW_MEM_BASIC == type)
		temp_object_rank_max = 3;
	else if (E_SHOW_MEM_CLASSIC == type)
		temp_object_rank_max = 6;
	else
		temp_object_rank_max = 10;

	printk("Detail:\n");
	printk("        page_recorder_debug: [%d]\n", page_recorder_debug);
	printk("        page_recorder_limit: [%d]\n", page_recorder_limit);
	printk("TOP %d page allocation:\n", temp_object_rank_max);

	for (index = 0; index < OBJECT_TABLE_SIZE; index++) {
		spin_lock_irqsave(&bt_record_lock, flags);
		tmp = page_bt_table.slots[index];
		while (tmp != NULL) {
			setup_rank_list(tmp,
					temp_object_rank_max,
					&object_rank_count,
					&rank_head,
					&rank_tail);
			tmp = tmp->next;
		}
		spin_unlock_irqrestore(&bt_record_lock, flags);
	}

	rank_tmp = rank_head;
	while (rank_tmp != NULL) {
		backtrace = (unsigned long *)rank_tmp->entry->object;
		printk("[%d]%s %d %s\n", rank_index,
				"Backtrace pages ",
				 rank_tmp->entry->size * 4096, "bytes");
		total_used += rank_tmp->entry->size * 4096;
		for (i = 0; i < rank_tmp->entry->nr_entries; i++) {
			sprint_symbol(symbol, *(backtrace + i));
			printk("  KERNEL[%d] 0x%lx :: symbol %s\n",
				i, backtrace[i], symbol);
		}
		rank_index++;
		tmp_record = rank_tmp;
		rank_tmp = rank_tmp->next;
		kfree(tmp_record->entry);
		kfree(tmp_record);
	}

	printk("Total used:%llu kB\n", total_used / 1024);
	return 0;
}

static int page_recorder_e_show_mem_handler(struct notifier_block *nb,
			unsigned long val, void *data)
{
	enum e_show_mem_type type = val;
	printk("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");
	printk("Enhanced Mem-info :PAGE RECORDER\n");
	return page_recorder_debug_show_printk(type);
}

static struct notifier_block page_recorder_e_show_mem_notifier = {
	.notifier_call = page_recorder_e_show_mem_handler,
};
#endif

static int __init page_recorder_init(void)
{
	/* Create page allocate */
	debug_root = debugfs_create_dir("page_recorder", NULL);
	debugfs_create_file("usage_rank", 0444, debug_root, NULL,
			    &debug_page_recorder_fops);
	debugfs_create_u32("rank_number", 0644, debug_root, &object_rank_max);
	debugfs_create_file("query_page", 0644, debug_root, NULL,
			    &query_page_ios_fops);
	debugfs_create_u64("page_virtual_address", 0644, debug_root,
			   (u64 *)&queried_address);
	debugfs_create_u32("debug_log", 0644, debug_root, &debug_log);
	debugfs_create_u32("page_recorder_debug", 0644, debug_root,
			   &page_recorder_debug);
	debugfs_create_u32("page_recorder_memory_usage", 0644, debug_root,
			   &page_recorder_memory_usage);
	debugfs_create_u32("page_recorder_limit", 0644, debug_root,
			   &page_recorder_limit);
#ifdef CONFIG_E_SHOW_MEM
	register_e_show_mem_notifier(&page_recorder_e_show_mem_notifier);
#endif
	return 0;
}

late_initcall(page_recorder_init);
