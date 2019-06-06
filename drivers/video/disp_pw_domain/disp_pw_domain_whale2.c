#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/sprd/whale2/whale2_glb.h>
#include <linux/regmap.h>
#include <video/disp_pw_domain.h>

#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(__fmt) "disp_pw_domain:<%d> [%s] "__fmt,  __LINE__, __func__

struct pw_domain_info *pw_domain_info;

void disp_pw_on(u8 client)
{
	u32 read_count = 0;
	u32 power_state;
	u32 temp;

	pr_info("disp_pw_on Enter \n");
	mutex_lock(&pw_domain_info->client_lock);
	if(pw_domain_info->pw_count == 0)
	{
		pr_info("do disp_pw_on set regs\n");
		regmap_update_bits(pw_domain_info->pmu_apb,
				REG_PMU_APB_PD_DISP_SYS_CFG,
				BIT_PMU_APB_PD_DISP_SYS_FORCE_SHUTDOWN,
				(unsigned int)(~BIT_PMU_APB_PD_DISP_SYS_FORCE_SHUTDOWN));

		do{
			mdelay(10);
			read_count++;
			/*when BIT_PMU_APB_PD_DISP_SYS_FORCE_SHUTDOWN = 0,
			BIT_PMU_APB_PD_DISP_SYS_STATE = 0*/
			regmap_read(pw_domain_info->pmu_apb,
					REG_PMU_APB_PWR_STATUS4_DBG,&power_state);
			power_state = power_state &
					BIT_PMU_APB_PD_DISP_SYS_STATE(0x1F); 
			regmap_read(pw_domain_info->pmu_apb,
					REG_PMU_APB_PD_DISP_SYS_CFG, &temp);
			pr_info("REG_PMU_APB_PD_DISP_SYS_CFG = 0x%x\n", temp);
			regmap_read(pw_domain_info->pmu_apb,
					REG_PMU_APB_PWR_STATUS4_DBG, &temp);
			pr_info("REG_PMU_APB_PWR_STATUS4_DBG = 0x%x\n", temp);
			regmap_read(pw_domain_info->pmu_apb,
					REG_PMU_APB_NOC_SLEEP_STOP_STATUS, &temp);
			pr_info("REG_PMU_APB_NOC_SLEEP_STOP_STATUS = 0x%x\n", temp);
		}while(power_state && (read_count < 4));

		if(read_count > 3)
			pr_err("set ~BIT_PMU_APB_PD_DISP_SYS_FORCE_SHUTDOWN error! \n");

		pr_info("disp_pw_on is set \n");
	}
	else
	{
		pr_info("disp_domain is already power on \n");
	}
	switch(client){
	case DISP_PW_DOMAIN_DISPC:
		if(pw_domain_info->pw_dispc_info.pw_state == DISP_PW_DOMAIN_OFF)
		{
			pw_domain_info->pw_dispc_info.pw_state = DISP_PW_DOMAIN_ON;
			pw_domain_info->pw_count++;
			pr_info("DISP_PW_DOMAIN_DISPC \n");
		}
		break;
	case DISP_PW_DOMAIN_GSP:
		if(pw_domain_info->pw_gsp_info.pw_state == DISP_PW_DOMAIN_OFF)
		{
			pw_domain_info->pw_gsp_info.pw_state = DISP_PW_DOMAIN_ON;
			pw_domain_info->pw_count++;
			pr_info("DISP_PW_DOMAIN_GSP \n");
		}
		break;
	case DISP_PW_DOMAIN_VPP:
		if(pw_domain_info->pw_vpp_info.pw_state == DISP_PW_DOMAIN_OFF)
		{
			pw_domain_info->pw_vpp_info.pw_state = DISP_PW_DOMAIN_ON;
			pw_domain_info->pw_count++;
			pr_info("DISP_PW_DOMAIN_VPP \n");
		}
		break;
	case DISP_PW_DOMAIN_DSI:
		if(pw_domain_info->pw_dsi_info.pw_state == DISP_PW_DOMAIN_OFF)
		{
			pw_domain_info->pw_dsi_info.pw_state = DISP_PW_DOMAIN_ON;
			pw_domain_info->pw_count++;
			pr_info("DISP_PW_DOMAIN_DSI \n");
		}
		break;
	default:
		break;
	}
	mutex_unlock(&pw_domain_info->client_lock);

}
EXPORT_SYMBOL(disp_pw_on);

int disp_pw_off(u8 client)
{
	u32 temp;
	pr_info("disp_pw_off Enter \n");
	mutex_lock(&pw_domain_info->client_lock);
	if(pw_domain_info->pw_count == 0)
	{
		pr_info("disp_domain is already power off \n");
		mutex_unlock(&pw_domain_info->client_lock);
		return 0;
	}
	else if(pw_domain_info->pw_count == 1)
	{

		regmap_read(pw_domain_info->aon_apb,
				REG_AON_APB_APB_EB1, &temp);
		regmap_update_bits(pw_domain_info->aon_apb,
				REG_AON_APB_APB_EB1,
				BIT_AON_APB_DISP_EB,
				BIT_AON_APB_DISP_EB);
		/* we need eable DISPC & GSP NOC_FORCE_CKG_EN bit first
		   before set power down, or the display module can not
		   power down */
		regmap_update_bits(pw_domain_info->disp_ahb,
				REG_DISP_AHB_GEN_CKG_CFG,
				BIT_DISP_AHB_DISPC_NOC_FORCE_CKG_EN |
				BIT_DISP_AHB_GSP_NOC_FORCE_CKG_EN,
				BIT_DISP_AHB_DISPC_NOC_FORCE_CKG_EN |
				BIT_DISP_AHB_GSP_NOC_FORCE_CKG_EN
				);
		regmap_update_bits(pw_domain_info->aon_apb,
				REG_AON_APB_APB_EB1,
				BIT_AON_APB_DISP_EB,
				temp);
		mdelay(10);

		regmap_update_bits(pw_domain_info->pmu_apb,
				REG_PMU_APB_PD_DISP_SYS_CFG,
				BIT_PMU_APB_PD_DISP_SYS_FORCE_SHUTDOWN,
				BIT_PMU_APB_PD_DISP_SYS_FORCE_SHUTDOWN);

		regmap_read(pw_domain_info->aon_apb,
				REG_AON_APB_APB_EB1, &temp);
		pr_info("REG_AON_APB_APB_EB1 = 0x%x\n", temp);
		regmap_read(pw_domain_info->pmu_apb,
				REG_PMU_APB_PD_DISP_SYS_CFG, &temp);
		pr_info("REG_PMU_APB_PD_DISP_SYS_CFG = 0x%x\n", temp);
		regmap_read(pw_domain_info->pmu_apb,
				REG_PMU_APB_NOC_SLEEP_STOP_STATUS, &temp);
		pr_info("REG_PMU_APB_NOC_SLEEP_STOP_STATUS = 0x%x\n", temp);
		pr_info("disp_pw_off is set \n");
	}
	switch(client){
	case DISP_PW_DOMAIN_DISPC:
		if(pw_domain_info->pw_dispc_info.pw_state == DISP_PW_DOMAIN_ON)
		{
			pw_domain_info->pw_dispc_info.pw_state = DISP_PW_DOMAIN_OFF;
			pw_domain_info->pw_count--;
			pr_info("DISP_PW_DOMAIN_DISPC \n");
		}
		break;
	case DISP_PW_DOMAIN_GSP:
		if(pw_domain_info->pw_gsp_info.pw_state == DISP_PW_DOMAIN_ON)
		{
			pw_domain_info->pw_gsp_info.pw_state = DISP_PW_DOMAIN_OFF;
			pw_domain_info->pw_count--;
			pr_info("DISP_PW_DOMAIN_GSP \n");
		}
		break;
	case DISP_PW_DOMAIN_VPP:
		if(pw_domain_info->pw_vpp_info.pw_state == DISP_PW_DOMAIN_ON)
		{
			pw_domain_info->pw_vpp_info.pw_state = DISP_PW_DOMAIN_OFF;
			pw_domain_info->pw_count--;
			pr_info("DISP_PW_DOMAIN_VPP \n");
		}
		break;
	case DISP_PW_DOMAIN_DSI:
		if(pw_domain_info->pw_dsi_info.pw_state == DISP_PW_DOMAIN_ON)
		{
			pw_domain_info->pw_dsi_info.pw_state = DISP_PW_DOMAIN_OFF;
			pw_domain_info->pw_count--;
			pr_info("DISP_PW_DOMAIN_DSI \n");
		}
		break;
	default:
		break;
	}
	mutex_unlock(&pw_domain_info->client_lock);
	return 0;
}
EXPORT_SYMBOL(disp_pw_off);

static int __init disp_pw_domain_init(void)
{
	/*dispc need ctl in uboot ,so start kernel no need set pw_domain*/
	pr_info("disp_pw_domain_init \n");
	pw_domain_info = kmalloc(sizeof(struct pw_domain_info),GFP_KERNEL);
	memset(pw_domain_info,0,sizeof(struct pw_domain_info));

	pw_domain_info->aon_apb = syscon_regmap_lookup_by_compatible("sprd,sys-aon-apb");
	if (IS_ERR(pw_domain_info->aon_apb))
		return PTR_ERR(pw_domain_info->aon_apb);
	pw_domain_info->disp_ahb = syscon_regmap_lookup_by_compatible("sprd,sys-disp-ahb");
	if (IS_ERR(pw_domain_info->disp_ahb))
		return PTR_ERR(pw_domain_info->disp_ahb);
	pw_domain_info->pmu_apb = syscon_regmap_lookup_by_compatible("sprd,sys-pmu-apb");
	if (IS_ERR(pw_domain_info->pmu_apb))
		return PTR_ERR(pw_domain_info->pmu_apb);

	mutex_init(&pw_domain_info->client_lock);
	return 0;
}
fs_initcall(disp_pw_domain_init);
