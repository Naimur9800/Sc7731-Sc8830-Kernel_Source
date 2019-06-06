/* ----------------------------------------------------------------------------
 *  Copyright (C) 2015 Intel Corporation

 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  You should have received a copy of the GNU General Public License Version 2
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.

 ---------------------------------------------------------------------------*/

/*
 * NOTES:
 * 1) This source file is included in guests including Linux and purposely
 * kept in line with Linux kernel coding guidelines for ease of portability and
 * maintainability. See linux/Documentation/CodingStyle for coding guidelines.
 * Please check all further changes are compliant with Linux coding standard
 * and clear all commands reported by Linux checkpatch.pl
 * Use command: linux/scripts/checkpatch.pl -f <filename>
 * Clear ALL warnings and errors reported.
 *
 */

#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/smp.h>
#include <linux/memblock.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <asm/mv/mv_hypercalls.h>
#include <asm/mv/mv_service_list.h>
#include <asm/mv/mv_gal.h>
#include <asm/mv/irq_vectors.h>
#if defined(CONFIG_SYSTEM_PROFILING)
#include <asm/mv/sysprofile.h>
#endif
#include <asm/pat.h>

/* define VM_MULTIPLE_VCPUS if guest VM has more than 1 VCPU */
#ifdef CONFIG_SMP
#define VM_MULTIPLE_VCPUS 1
#endif

struct mv_shared_data *mv_shared_data[CONFIG_MAX_VCPUS_PER_VM];
static const char *vm_command_line;
struct irq_domain *hirq_domain;
static void *pmem_vbase;

struct mv_shared_data *mv_gal_get_shared_data(void)
{
#ifdef VM_MULTIPLE_VCPUS
	unsigned cpu = get_cpu();

	put_cpu();
	return mv_shared_data[cpu];
#else
	return mv_shared_data[0];
#endif
}

struct hirq_handler_wrapper {
	int irq;
	irq_handler_t handler;
	void *cookie;
};

irqreturn_t generic_hirq_handler(int irq, void *cookie)
{
	struct hirq_handler_wrapper *hirq_wrapper =
		(struct hirq_handler_wrapper *)cookie;
	return hirq_wrapper->handler(irq, hirq_wrapper->cookie);
}

void mv_gal_panic(char *panic_msg)
{
	if (panic_msg)
		panic("PANIC: %s\n", panic_msg);
	else
		panic("PANIC\n");
}

void *mv_gal_ptov(mv_paddr_t paddr)
{
	void *ptr;
	struct mv_shared_data *p_shared_data;

	if (!pmem_vbase)
		return NULL;

	p_shared_data = mv_gal_get_shared_data();

	if (paddr >= p_shared_data->ivmc_shmem_paddr &&
		paddr <= p_shared_data->ivmc_shmem_paddr +
		p_shared_data->ivmc_shmem_size) {
		ptr = pmem_vbase + (paddr - p_shared_data->ivmc_shmem_paddr);
		return ptr;
	} else {
		return (void *)((uintptr_t)paddr); /* How would this work? */
	}
}

static unsigned int myid;

void mv_gal_services_init(void)
{
	int i;

	for (i = VMM_FIRST_SERVICE; i <= VMM_LAST_SERVICE; i++)
		vmm_service_init_handlers[i]();
}

void mv_gal_init(struct mv_shared_data *data)
{
	myid = data->os_id;
	vm_command_line = data->vm_cmdline; /* vm_command_line is never used! */

	mv_gal_services_init();
}

int mv_map_vcpu_shmem(void)
{
	phys_addr_t ptr;

	if (mv_gal_get_shared_data())
		return 0;

	ptr = (uintptr_t)mv_vcpu_get_data();

	mv_shared_data[smp_processor_id()] = phys_to_virt(ptr);
	pr_info("ptr=%pa, mv_shared_data=%p\n",
		&ptr, mv_gal_get_shared_data());

	return 0;
}

int mv_init_secondary(void)
{
	pr_debug("In mv_init_secondary\n");
	mv_virq_request(MV_LOCAL_TIMER_VECTOR, 1);
	mv_virq_unmask(MV_LOCAL_TIMER_VECTOR);
	mv_virq_request(MV_RESCHEDULE_VECTOR, 1);
	mv_virq_unmask(MV_RESCHEDULE_VECTOR);
	mv_virq_request(MV_CALL_FUNCTION_VECTOR, 1);
	mv_virq_unmask(MV_CALL_FUNCTION_VECTOR);
	mv_virq_request(MV_CALL_FUNCTION_SINGLE_VECTOR, 1);
	mv_virq_unmask(MV_CALL_FUNCTION_SINGLE_VECTOR);
	mv_virq_request(MV_REBOOT_VECTOR, 1);
	mv_virq_unmask(MV_REBOOT_VECTOR);
#ifdef CONFIG_IRQ_WORK
	mv_virq_request(MV_IRQ_WORK_VECTOR, 1);
	mv_virq_unmask(MV_IRQ_WORK_VECTOR);
#endif
	mv_map_vcpu_shmem();

	mv_hirq_ready();

	return 0;
}

int __init mv_init(void)
{
	struct mv_shared_data *shmem;

	pr_debug("In mv_init\n");

	if (mv_map_vcpu_shmem())
		panic("Unable to map vcpu shared mem\n");

	shmem = mv_gal_get_shared_data();

	pmem_vbase = ioremap_cache(shmem->ivmc_shmem_paddr, shmem->ivmc_shmem_size);
	pr_info("ivmc_shmem_paddr=0x%llx ivmc_shmem_size=0x%llx pmem_vbase=0x%p\n",
		shmem->ivmc_shmem_paddr,
		shmem->ivmc_shmem_size,
		pmem_vbase);
	if (!pmem_vbase)
		panic("Unable to map PMEM\n");

	pr_debug("Calling mv_al_init. os_id=%d\n", shmem->os_id);
	mv_gal_init(shmem);

	mv_hirq_ready();

	return 0;
}

inline unsigned int mv_gal_os_id(void)
{
	return myid;
}

#define HIRQ_HANDLER_NAME_SIZE	16
void *mv_gal_register_hirq_callback(uint32_t hirq, irq_handler_t cb,
		void *cookie)
{
	int virq = irq_create_mapping(hirq_domain, hirq - VMM_HIRQ_START);
	char *handler_name = NULL;
	struct hirq_handler_wrapper *wrapper = NULL;

	wrapper = kmalloc(sizeof(struct hirq_handler_wrapper), GFP_KERNEL);
	if (!wrapper) {
		pr_err("failed to allocate wrapper\n");
		return NULL;
	}
	wrapper->irq = virq;
	wrapper->handler = cb;
	wrapper->cookie = cookie;
	handler_name = kmalloc(HIRQ_HANDLER_NAME_SIZE, GFP_KERNEL);
	if (!handler_name) {
		pr_err("failed to allocate handler_name\n");
		goto free_wrapper;
	}
	snprintf(handler_name, HIRQ_HANDLER_NAME_SIZE, "hirq-%d", hirq);
	if (request_irq(virq, generic_hirq_handler,
				IRQF_SHARED | IRQF_NO_SUSPEND, handler_name,
				(void *)wrapper)) {
		pr_err("failed to request irq\n");
		goto free_handler;
	}
	return wrapper;
free_handler:
	kfree(handler_name);
free_wrapper:
	kfree(wrapper);
	return NULL;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_gal_register_hirq_callback);
#endif

void mv_gal_hirq_detach(void *id)
{
	struct hirq_handler_wrapper *hirq_wrapper =
		(struct hirq_handler_wrapper *)id;
	free_irq(hirq_wrapper->irq, id);
	kfree(id);
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_gal_hirq_detach);
#endif


static int32_t mv_gal_probe(struct platform_device *pdev)
{
	struct device_node *np;

	pr_debug("mv_gal_probe\n");
	np = pdev->dev.of_node;
	hirq_domain = irq_find_host(of_irq_find_parent(np));

	mv_ipc_init();
	return 0;
}

static const struct of_device_id mv_gal_of_match[] = {
	{
		.compatible = "intel,mobilevisor",
	},
	{},
};

MODULE_DEVICE_TABLE(of, mv_gal_of_match);

static struct platform_driver mv_gal_driver = {
	.probe = mv_gal_probe,
	.driver = {
		.name = "mv_gal",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(mv_gal_of_match),
	}
};

static int32_t __init mv_gal_driver_init(void)
{
	return platform_driver_register(&mv_gal_driver);
}

static void __exit mv_gal_driver_exit(void)
{
	 platform_driver_unregister(&mv_gal_driver);
}

core_initcall(mv_gal_driver_init);
module_exit(mv_gal_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("mobilevisor guest adaption driver");
