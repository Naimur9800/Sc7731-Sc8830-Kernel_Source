/*
 * Architecture specific OF callbacks.
 */

#define pr_fmt(fmt) "x86-of: " fmt

#include <linux/bootmem.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/of_pci.h>
#include <linux/initrd.h>

#include <asm/irqdomain.h>
#include <asm/hpet.h>
#include <asm/apic.h>
#include <asm/pci_x86.h>
#include <asm/setup.h>
#include <asm/i8259.h>

#include <asm/mv/cpu.h>

__initdata u64 initial_dtb;
char __initdata cmd_line[COMMAND_LINE_SIZE];

int __initdata of_ioapic;

void __init early_init_dt_scan_chosen_arch(unsigned long node)
{
	BUG();
}

void __init early_init_dt_add_memory_arch(u64 base, u64 size)
{
	BUG();
}

void * __init early_init_dt_alloc_memory_arch(u64 size, u64 align)
{
	return __alloc_bootmem(size, align, __pa(MAX_DMA_ADDRESS));
}

void __init add_dtb(u64 data)
{
	initial_dtb = data + offsetof(struct setup_data, data);
}

#ifdef CONFIG_PCI
struct device_node *pcibios_get_phb_of_node(struct pci_bus *bus)
{
	struct device_node *np;

	for_each_node_by_type(np, "pci") {
		const void *prop;
		unsigned int bus_min;

		prop = of_get_property(np, "bus-range", NULL);
		if (!prop)
			continue;
		bus_min = be32_to_cpup(prop);
		if (bus->number == bus_min)
			return np;
	}
	return NULL;
}

static int x86_of_pci_irq_enable(struct pci_dev *dev)
{
	u32 virq;
	int ret;
	u8 pin;

	ret = pci_read_config_byte(dev, PCI_INTERRUPT_PIN, &pin);
	if (ret)
		return ret;
	if (!pin)
		return 0;

	virq = of_irq_parse_and_map_pci(dev, 0, 0);
	if (virq == 0)
		return -EINVAL;
	dev->irq = virq;
	return 0;
}

static void x86_of_pci_irq_disable(struct pci_dev *dev)
{
}

void x86_of_pci_init(void)
{
	pcibios_enable_irq = x86_of_pci_irq_enable;
	pcibios_disable_irq = x86_of_pci_irq_disable;
}
#endif

static void __init dtb_setup_hpet(void)
{
#ifdef CONFIG_HPET_TIMER
	struct device_node *dn;
	struct resource r;
	int ret;

	dn = of_find_compatible_node(NULL, NULL, "intel,x86-hpet");
	if (!dn)
		return;
	ret = of_address_to_resource(dn, 0, &r);
	if (ret) {
		WARN_ON(1);
		return;
	}
	hpet_address = r.start;
#endif
}

static void __init dtb_lapic_setup(void)
{
#ifdef CONFIG_X86_LOCAL_APIC
	struct device_node *dn;
	struct resource r;
	int ret;

	dn = of_find_compatible_node(NULL, NULL, "intel,x86-lapic");
	if (!dn)
		return;

	ret = of_address_to_resource(dn, 0, &r);
	if (WARN_ON(ret))
		return;

#ifdef CONFIG_X86_32
	/* Did the boot loader setup the local APIC ? */
	if (!cpu_has_apic) {
		if (apic_force_enable(r.start))
			return;
	}
#endif

	register_lapic_address(r.start);
#endif
}

static void __init dtb_cpus_setup(void)
{
#ifdef CONFIG_X86_LOCAL_APIC
	unsigned apic_id;
	struct device_node *cpu, *cpus;

	cpus = of_find_node_by_path("/cpus");
	if (!cpus) { /* UP configuration (one CPU) */
		generic_processor_info(boot_cpu_physical_apicid,
			GET_APIC_VERSION(apic_read(APIC_LVR)));
		pr_info("no /cpus found from dtb\n");
		return;
	}

	for_each_child_of_node(cpus, cpu) {
		if (of_node_cmp(cpu->type, "cpu"))
			continue;

		if (of_property_read_u32(cpu, "reg", &apic_id)) {
			pr_err("/cpus %s requires reg property\n",
						cpu->full_name);
			return;
		}

#ifdef CONFIG_MOBILEVISOR
		if (mv_is_irq_bypass()) {
			/* Pass the mapped apic id for irq bypass mode */
			apic_id = mv_cpu_get_apicid(apic_id);
			pr_debug("irq_bypass apicid = %x\n", apic_id);
		}
#endif

		generic_processor_info(apic_id,
			GET_APIC_VERSION(apic_read(APIC_LVR)));
	}

	if (num_processors > 1)
		smp_found_config = 1;
#endif
}

#ifdef CONFIG_X86_IO_APIC
static unsigned int ioapic_id;

struct of_ioapic_type {
	u32 out_type;
	u32 trigger;
	u32 polarity;
};

static struct of_ioapic_type of_ioapic_type[] =
{
	{
		.out_type	= IRQ_TYPE_EDGE_RISING,
		.trigger	= IOAPIC_EDGE,
		.polarity	= 0,
	},
	{
		.out_type	= IRQ_TYPE_LEVEL_LOW,
		.trigger	= IOAPIC_LEVEL,
		.polarity	= 1,
	},
	{
		.out_type	= IRQ_TYPE_LEVEL_HIGH,
		.trigger	= IOAPIC_LEVEL,
		.polarity	= 0,
	},
	{
		.out_type	= IRQ_TYPE_EDGE_FALLING,
		.trigger	= IOAPIC_EDGE,
		.polarity	= 1,
	},
};

static int dt_irqdomain_alloc(struct irq_domain *domain, unsigned int virq,
			      unsigned int nr_irqs, void *arg)
{
	struct of_phandle_args *irq_data = (void *)arg;
	struct of_ioapic_type *it;
	struct irq_alloc_info tmp;

	if (WARN_ON(irq_data->args_count < 2))
		return -EINVAL;
	if (irq_data->args[1] >= ARRAY_SIZE(of_ioapic_type))
		return -EINVAL;

	it = &of_ioapic_type[irq_data->args[1]];
	ioapic_set_alloc_attr(&tmp, NUMA_NO_NODE, it->trigger, it->polarity);
	tmp.ioapic_id = mpc_ioapic_id(mp_irqdomain_ioapic_idx(domain));
	tmp.ioapic_pin = irq_data->args[0];

	return mp_irqdomain_alloc(domain, virq, nr_irqs, &tmp);
}

static const struct irq_domain_ops ioapic_irq_domain_ops = {
	.alloc		= dt_irqdomain_alloc,
	.free		= mp_irqdomain_free,
	.activate	= mp_irqdomain_activate,
	.deactivate	= mp_irqdomain_deactivate,
};

static void __init dtb_add_ioapic(struct device_node *dn)
{
	struct resource r;
	int ret;
	struct ioapic_domain_cfg cfg = {
		.type = IOAPIC_DOMAIN_DYNAMIC,
		.ops = &ioapic_irq_domain_ops,
		.dev = dn,
	};

	ret = of_address_to_resource(dn, 0, &r);
	if (ret) {
		printk(KERN_ERR "Can't obtain address from node %s.\n",
				dn->full_name);
		return;
	}
	mp_register_ioapic(++ioapic_id, r.start, gsi_top, &cfg);
}

static void __init dtb_ioapic_setup(void)
{
	struct device_node *dn;

	for_each_compatible_node(dn, NULL, "intel,x86-ioapic")
		dtb_add_ioapic(dn);

	if (nr_ioapics) {
		of_ioapic = 1;
		pic_mode = 0; /* pic mode must be 0 when ioapic is present */
		return;
	}
	printk(KERN_ERR "Error: No information about IO-APIC in OF.\n");
}
#else
static void __init dtb_ioapic_setup(void) {}
#endif

static void __init dtb_apic_setup(void)
{
	dtb_lapic_setup();
	dtb_ioapic_setup();
}

void __init x86_setup_dtb(void)
{
	/* do some early initialization based on DT */
	early_init_dt_scan(phys_to_virt(initial_dtb));
}

void __init x86_dtb_init(void)
{
	unflatten_and_copy_device_tree();

	if (!of_have_populated_dt())
		return;

	dtb_setup_hpet();
	dtb_apic_setup();
	dtb_cpus_setup();
}
