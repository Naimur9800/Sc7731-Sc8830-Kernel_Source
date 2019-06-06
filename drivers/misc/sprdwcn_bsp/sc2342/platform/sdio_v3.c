#include <linux/sdiohal_api.h>

#include "wcn_bus.h"

static int sdiohal_preinit(void)
{
	sdiohal_init();

	return 0;
}

static unsigned int sdiohal_direct_read(unsigned int system_addr, void *buf,
				      unsigned int len)
{
	return sdiohal_dt_read(system_addr, buf, len);
}

static unsigned int sdiohal_direct_write(unsigned int system_addr,
				       void *buf, unsigned int len)
{
	return sdiohal_dt_write(system_addr, buf, len);
}

static void sdiohal_reg_rescan_cb(void *func)
{
	sdiohal_register_rescan_cb(func);
}

static struct sprdwcn_bus_ops wcn_sdiohal_bus_ops = {
	.preinit = sdiohal_preinit,
	.direct_read = sdiohal_direct_read,
	.direct_write = sdiohal_direct_write,
	.register_rescan_cb = sdiohal_reg_rescan_cb,
	.rescan = sdiohal_sdio_rescan,
	.get_carddump_status = sdiohal_get_carddump_status,
	.set_carddump_status = sdiohal_set_carddump_status,
};

void module_bus_init(void)
{
	module_ops_register(&wcn_sdiohal_bus_ops);
}
EXPORT_SYMBOL(module_bus_init);

void module_bus_deinit(void)
{
	module_ops_unregister();
}
EXPORT_SYMBOL(module_bus_deinit);
