#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <video/disp_pw_domain.h>

#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(__fmt) "disp_pw_domain:<%d> [%s] "__fmt,  __LINE__, __func__

void disp_pw_on(u8 client)
{
	pr_info("disp_pw_off Enter \n");
	return;
}
EXPORT_SYMBOL(disp_pw_on);

int disp_pw_off(u8 client)
{
	pr_info("disp_pw_on Enter \n");
	return 0;
}
EXPORT_SYMBOL(disp_pw_off);

static int __init disp_pw_domain_init(void)
{
	pr_info("disp_pw_domain_init \n");
	return 0;
}
fs_initcall(disp_pw_domain_init);
