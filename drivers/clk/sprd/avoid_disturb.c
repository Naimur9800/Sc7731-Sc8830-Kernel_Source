#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/sipc.h>
#include <linux/slab.h>

#include "avoid_disturb.h"

static DEFINE_RAW_SPINLOCK(avoid_disturb_spin_lock);
static RAW_NOTIFIER_HEAD(ads_notifier_chain);

struct pll_chg_info *pc_info;
struct completion ads_done;

static struct request_infos *req_infos;
static struct avoid_disturb ads_data __initdata = {};

static int sprd_ads_of_get_off(struct platform_device *pdev,
		int index, u64 *addr, u64 *size)
{
	const __be32 *prop;
	unsigned int psize;
	int onesize, i, na, ns;
	struct device_node *dev = pdev->dev.of_node;

	na = of_n_addr_cells(dev);
	ns = of_n_size_cells(dev);
	if (na < 0 || na > 4 || ns < 0) {
		dev_err(&pdev->dev, "failed get na and ns!\n");
		return -EINVAL;
	}

	prop = of_get_property(dev, "sprd,ads-off-info", &psize);
	if (prop == NULL) {
		dev_err(&pdev->dev, "failed get ads off info!\n");
		return -EINVAL;
	}

	psize /= 4;

	onesize = na + ns;
	for (i = 0; psize >= onesize; psize -= onesize, prop += onesize, i++)
		if (i == index) {
			if (size)
				*size = of_read_number(prop + na, ns);

			if (addr)
				*addr = of_read_number(prop, na);
		}

	return 0;
}

static struct request_infos *
__init sprd_ads_get_req_infos_func(struct platform_device *pdev)
{
	u32 val;
	unsigned long naddr, nsize;
	int ret;
	void __iomem *base_addr;
	void __iomem *addr;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	base_addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base_addr)) {
		dev_err(&pdev->dev, "%s failed to remap iram address\n",
			__func__);
		return base_addr;
	}

	req_infos->base_addr = base_addr;

	req_infos = devm_kzalloc(&pdev->dev, sizeof(req_infos), GFP_KERNEL);
	if (!req_infos)
		return ERR_PTR(-ENOMEM);

	/* read count info, fix me later */
	val = readl_relaxed(base_addr);
	req_infos->disturbing_cnt = val & DBS_MSK;
	req_infos->disturbed_cnt = (val & DBN_MSK) >> DBS_BIT_NUM;

	ret = sprd_ads_of_get_off(pdev, 0, (u64 *)&naddr, (u64 *)&nsize);
	if (ret < 0) {
		dev_err(&pdev->dev, "get clk num off base failed!\n");
		return ERR_PTR(ret);
	}
	addr = base_addr + naddr;

	/* get disturb clock src cnt */
	req_infos->clk_src_cnt = devm_kzalloc(&pdev->dev,
			req_infos->disturbing_cnt, GFP_KERNEL);
	if (!req_infos->clk_src_cnt)
		return ERR_PTR(-ENOMEM);

	memcpy(req_infos->clk_src_cnt, addr, req_infos->disturbing_cnt);

	/* get cur_info_base */
	ret = sprd_ads_of_get_off(pdev, 1, (u64 *)&naddr, (u64 *)&nsize);
	if (ret < 0) {
		dev_err(&pdev->dev, "get cur clk off base failed!\n");
		return ERR_PTR(ret);
	}
	req_infos->cur_info_base = base_addr + naddr;

	/* get req_info_base */
	ret = sprd_ads_of_get_off(pdev, 1, (u64 *)&naddr, (u64 *)&nsize);
	if (ret < 0) {
		dev_err(&pdev->dev, "get cur req info base failed!\n");
		return ERR_PTR(ret);
	}
	req_infos->req_info_base = base_addr + naddr;

	return req_infos;
}

int sprd_ads_write_aon_cur_info(enum DISTURBING_ID_E id, enum PLL_ID_E pll_id,
			unsigned int state, u8 clk_src_id)
{
	struct disturbing_curstatus *cur_status;

	cur_status = kzalloc(sizeof(struct disturbing_curstatus), GFP_KERNEL);
	if (!cur_status)
		return -ENOMEM;

	cur_status->time_stamp = 0;
	cur_status->state = state;
	cur_status->clk_mode = pc_info[pll_id].prefer;
	cur_status->delt_freq_index = pc_info[pll_id].delt;
	cur_status->clk_freq_index = clk_src_id;
	cur_status->pll_id = pll_id;

	memcpy(req_infos->cur_info_base + id,
		cur_status, sizeof(struct disturbing_curstatus));

	kfree(cur_status);

	return 0;
}
EXPORT_SYMBOL_GPL(sprd_ads_write_aon_cur_info);

int sprd_ads_register_notifier(struct notifier_block *nb)
{
	unsigned long flags;
	int ret;

	raw_spin_lock_irqsave(&avoid_disturb_spin_lock, flags);
	ret = raw_notifier_chain_register(&ads_notifier_chain, nb);
	raw_spin_unlock_irqrestore(&avoid_disturb_spin_lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(sprd_ads_register_notifier);

int sprd_ads_unregister_notifier(struct notifier_block *nb)
{
	unsigned long flags;
	int ret;

	raw_spin_lock_irqsave(&avoid_disturb_spin_lock, flags);
	ret = raw_notifier_chain_unregister(&ads_notifier_chain, nb);
	raw_spin_unlock_irqrestore(&avoid_disturb_spin_lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(sprd_ads_unregister_notifier);

static int sprd_ads_fill_smsg(struct smsg *mrecv, int index)
{
	u16 p1;
	u32 p2;
	unsigned long flags;

	p1 = mrecv->flag;
	p2 = mrecv->value;

	raw_spin_lock_irqsave(&avoid_disturb_spin_lock, flags);
	ads_data.ads_smsg_array[index].disturbed_id = p1;
	ads_data.ads_smsg_array[index].disturbing_id = p2;
	raw_spin_unlock_irqrestore(&avoid_disturb_spin_lock, flags);

	return 0;
}

static int sprd_ads_read_aon_req_info(struct request_info *r_info,
			u32 disturbing_id, u32 disturbed_id)
{
	int i;
	int request_offset = 0;
	void __iomem *addr;

	for (i = 0; i < disturbing_id; i++)
		request_offset += req_infos->disturbed_cnt * (1 +
				req_infos->clk_src_cnt[i]);

	/* calc request base addr, only 32bit valid */
	addr = req_infos->req_info_base + request_offset +
		(disturbed_id - 1) *
		(1 + req_infos->clk_src_cnt[disturbing_id]);
	memcpy(r_info, addr, sizeof(struct avoidance_src_info) +
		sizeof(struct delt_bitmap) *
		req_infos->clk_src_cnt[disturbing_id]);

	return 0;
}

static void sprd_ads_get_best_solution(struct request_info *rinfo,
				       int src_cnt)
{
	u8 pll_id, dbs;

	pll_id = rinfo->delt[src_cnt].pll_id;

	if (rinfo->src_info.prio < pc_info[pll_id].prio)
		return;

	dbs = rinfo->delt[src_cnt].delt_bitmap_spread;
	if (dbs > (pc_info[pll_id].delt & DBS_MSK))
		pc_info[pll_id].delt &= ~DBS_MSK;
	pc_info[pll_id].delt |= dbs;

	dbs = rinfo->delt[src_cnt].delt_bitmap_normal;
	if (dbs > ((pc_info[pll_id].delt >> DBS_BIT_NUM) & DBS_MSK))
		pc_info[pll_id].delt &= ~DBN_MSK;
	pc_info[pll_id].delt |= dbs << DBS_BIT_NUM;

	pc_info[pll_id].prefer = rinfo->src_info.prefer;

	pc_info[pll_id].prio = rinfo->src_info.prio;
}

static int sprd_ads_mthread_func(void *data)
{
	u16 val1;
	u32 val2;
	int i, j, k, ret;
	unsigned long flags, rate;
	struct request_info *rinfo;
	struct clk *clk;

	while (!kthread_should_stop()) {
		wait_for_completion(&ads_done);

		/*
		 * After wakeup, start arbitration process
		 */
		raw_spin_lock_irqsave(&avoid_disturb_spin_lock, flags);
		for (i = 0; i < ads_data.sthread_num; i++) {
			val1 = ads_data.ads_smsg_array[i].disturbed_id;
			val2 = ads_data.ads_smsg_array[i].disturbing_id;

			for (j = 0; j < DISTURBING_ID_MAX; j++) {
				if (((val2 >> j) & 1) == 0)
					continue;

				rinfo =
				kzalloc(sizeof(struct avoidance_src_info) +
					sizeof(struct delt_bitmap) *
					req_infos->clk_src_cnt[j], GFP_ATOMIC);
				if (!rinfo)
					continue;

				sprd_ads_read_aon_req_info(rinfo, j, val1);

				for (k = 0; k < req_infos->clk_src_cnt[j]; k++)
					sprd_ads_get_best_solution(rinfo, k);

				kfree(rinfo);
			}
		}

		raw_spin_unlock_irqrestore(&avoid_disturb_spin_lock, flags);
		/* now we have the best solution */

		for (i = 0; i < ads_data.c_info.ads_pll_num; i++) {
			clk = ads_data.c_info.clk_array[i];
			rate = ads_data.c_info.pll_old_rate[i];

			if (pc_info[i].delt == 0 ||  !clk)
				continue;

			ret = raw_notifier_call_chain(&ads_notifier_chain,
					0, &pc_info[i]);
			if (ret != NOTIFY_DONE || ret != NOTIFY_OK) {
				pr_warn("the range is too big, ignore it!\n");
				continue;
			}

			clk_set_rate(clk, rate);
		}
	}

	/* the thread must cannot be here */
	return 0;
}

static int sprd_ads_sthread_func(void *data)
{
	u8 *dst;
	int ret;
	struct smsg mrecv;

	dst = data;

	ret = smsg_ch_open(*dst, SMSG_CH_COMM, -1);
	if (ret)
		return ret;

	while (!kthread_should_stop()) {
		smsg_set(&mrecv, SMSG_CH_COMM, 0, 0, 0);

		ret = smsg_recv(*dst, &mrecv, -1);
		if (ret == -EIO) {
			msleep(500);
			continue;
		}

		pr_info("%s recv %d message success!\n",
			__func__, *dst);

		if (mrecv.type != SMSG_TYPE_EVENT) {
			pr_warn("%s recv the wrong message!\n",
				__func__);
			continue;
		}

		sprd_ads_fill_smsg(&mrecv, 1);
		complete(&ads_done);
	}

	/* the thread must cannot be here */
	return 0;
}

static int sprd_ads_pll_chg_slightly(enum PLL_ID_E id, unsigned long val,
				     bool add)
{
	int i, j;
	unsigned long rate, rate_delt;

	rate = ads_data.c_info.pll_old_rate[id];
	rate_delt = rate / 1000;

	if (add)
		rate = rate + rate_delt;
	else
		rate = rate - rate_delt;

	for (i = 0; i < val; i++)
		for (j = 0; j < 5; j++)
			clk_set_rate(ads_data.c_info.clk_array[j], rate);

	return 0;
}

static int sprd_ads_pll_chg_cb(struct notifier_block *nb,
			unsigned long event, void *data)
{
	int i;
	unsigned long pval, nval;
	struct clk_notifier_data *cnd_data;

	for (i = 0; i < ads_data.c_info.ads_pll_num; i++) {
		cnd_data = (struct clk_notifier_data *)data;
		if (cnd_data->clk == ads_data.c_info.clk_array[i])
			break;
	}

	switch (event) {
	case PRE_RATE_CHANGE:
		break;

	case POST_RATE_CHANGE:
		/* normal */
		if (pc_info[i].prefer == NORMAL_MODE) {
			pval = pc_info[i].delt & POS_NORM_MSK;
			nval = pc_info[i].delt & NEG_NORM_MSK;
			if ((pval && nval) && (pval != POS_NORM_MSK) &&
			    (nval != NEG_NORM_MSK)) {
				pr_err("%s normal conflicted!\n", __func__);
				break;
			}

			pval = pval >> DBS_BIT_NUM;
			nval = nval >> (DBS_BIT_NUM + UNI_DELT_NUM);
			if ((pval == POS_NORM_MSK) &&
			    (nval == POS_NORM_MSK)) {
				memset(&pc_info[i], 0, sizeof(pc_info[i]));
				break;
			}

			if (pval != POS_NORM_MSK)
				sprd_ads_pll_chg_slightly(i, pval, true);

			if (nval != POS_SPREAD_MSK)
				sprd_ads_pll_chg_slightly(i, nval, false);
		} else {
			pval = pc_info[i].delt & POS_SPREAD_MSK;
			if (pval == DBS_MSK)
				memset(&pc_info[i],
					0, sizeof(struct pll_chg_info));
			else
				writel_relaxed(pval,
					ads_data.c_info.ccs_base_addr + i);
		}

		break;

	default:
		break;
	}

	return NOTIFY_DONE;
}

static struct notifier_block clk_chg_cb = {
	.notifier_call = sprd_ads_pll_chg_cb,
};

static int sprd_ads_get_info(struct platform_device *pdev)
{
	u32 chn, id;
	int i, cnt, ret;
	struct resource *res;
	struct device_node *np = pdev->dev.of_node;

	cnt = of_property_count_u32_elems(np, "sprd,pll_index");
	if (cnt < 0) {
		dev_err(&pdev->dev, "get pll count failed!\n");
		return cnt;
	}
	ads_data.c_info.ads_pll_num = cnt;

	pc_info = devm_kzalloc(&pdev->dev,
		cnt * sizeof(struct pll_chg_info), GFP_KERNEL);
	if (!pc_info)
		return -ENOMEM;

	ads_data.c_info.pll_old_rate = devm_kzalloc(&pdev->dev,
		cnt * sizeof(unsigned long), GFP_KERNEL);
	if (!ads_data.c_info.pll_old_rate)
		return -ENOMEM;

	ads_data.c_info.pll_id = devm_kzalloc(&pdev->dev,
		cnt * sizeof(u8), GFP_KERNEL);
	if (!ads_data.c_info.pll_id)
		return -ENOMEM;

	ads_data.c_info.clk_array = devm_kzalloc(&pdev->dev,
		cnt * sizeof(struct clk *), GFP_KERNEL);
	if (!ads_data.c_info.clk_array)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ads_data.c_info.ccs_base_addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ads_data.c_info.ccs_base_addr))
		return PTR_ERR(ads_data.c_info.ccs_base_addr);

	for (cnt = 0; cnt < ads_data.c_info.ads_pll_num; cnt++) {
		ret = of_property_read_u32_index(np, "sprd,pll_index",
			cnt, &id);
		if (ret < 0) {
			dev_err(&pdev->dev, "get pll index failed!\n");
			goto unregister_clk_notifier;
		}

		ads_data.c_info.pll_id[cnt] = (u8) id;
		ads_data.c_info.clk_array[cnt] = of_clk_get(np, cnt);
		if (IS_ERR(ads_data.c_info.clk_array[cnt]))
			return PTR_ERR(ads_data.c_info.clk_array[cnt]);

		ads_data.c_info.pll_old_rate[cnt] =
			clk_get_rate(ads_data.c_info.clk_array[cnt]);

		clk_notifier_register(ads_data.c_info.clk_array[cnt],
			&clk_chg_cb);
	}

	ads_data.sthread_num =
		of_property_count_u32_elems(np, "sprd,sad_dst_id");
	if (ads_data.sthread_num < 0) {
		dev_err(&pdev->dev, "get sipc count failed!\n");
		ret = ads_data.sthread_num;
		goto unregister_clk_notifier;
	}

	ads_data.ads_sthread = (struct task_struct **)devm_kzalloc(&pdev->dev,
			ads_data.sthread_num * sizeof(struct task_struct *),
			GFP_KERNEL);
	if (!ads_data.ads_sthread) {
		ret = -ENOMEM;
		goto unregister_clk_notifier;
	}

	/*
	 * Because of sipc recv interface will block if no data is usable, one
	 * destination should has one thread.
	 * The sthread's number is depend on how many sipc destinations are
	 * used by ads.
	 */
	for (i = 0; i < ads_data.sthread_num; i++) {
		of_property_read_u32_index(np, "sprd,sad_dst_id",
			i, &chn);
		ads_data.ads_sthread[i] = kthread_run(sprd_ads_sthread_func,
				&i, "ads_sthread%d", chn);
		if (IS_ERR(ads_data.ads_sthread[i])) {
			dev_err(&pdev->dev, "%s create ads sthread  %d failed!\n",
				__func__, i);
			ret = PTR_ERR(ads_data.ads_sthread[i]);
			goto kthread_stop_sthread;
		}
	}

	ads_data.ads_smsg_array = devm_kzalloc(&pdev->dev,
		ads_data.sthread_num * sizeof(struct ads_sipc_message),
		GFP_KERNEL);
	if (!ads_data.ads_smsg_array) {
		ret = -ENOMEM;
		goto kthread_stop_sthread;
	}

	return 0;

kthread_stop_sthread:
	for (; i >= 0; i--)
		kthread_stop(ads_data.ads_sthread[i]);

unregister_clk_notifier:
	for (; cnt >= 0; cnt--)
		clk_notifier_unregister(ads_data.c_info.clk_array[cnt],
				&clk_chg_cb);

	return ret;
}

static const struct of_device_id ads_of_match[] = {
	{
		.compatible = "sprd,pike2-ads",
		.data = &ads_data,
	},
	{},
};

static int sprd_ads_probe(struct platform_device *pdev)
{
	int ret;
	struct device_node *np;
	const struct of_device_id *of_id;

	np = pdev->dev.of_node;
	if (!np) {
		dev_err(&pdev->dev, "device node not found\n");
		return -ENODEV;
	}

	of_id = of_match_node(ads_of_match, pdev->dev.of_node);
	if (!of_id) {
		dev_err(&pdev->dev, "get device of id failed!\n");
		return -ENODEV;
	}

	req_infos = sprd_ads_get_req_infos_func(pdev);
	if (IS_ERR(req_infos))
		return PTR_ERR(req_infos);

	ret = sprd_ads_get_info(pdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "get ads info failed!\n");
		return ret;
	}

	ads_data.ads_mthread = kthread_run(sprd_ads_mthread_func,
			NULL, "ads_mthread");
	if (IS_ERR(ads_data.ads_mthread)) {
		dev_err(&pdev->dev, "%s create mthread failed!\n",
			__func__);
		return PTR_ERR(ads_data.ads_mthread);
	}

	return 0;
}

static struct platform_driver sprd_ads_driver = {
	.probe = sprd_ads_probe,
	.driver = {
		.name = "sprd-ads",
		.of_match_table = ads_of_match,
	},
};

static int __init sprd_ads_init(void)
{
	return platform_driver_register(&sprd_ads_driver);
}

static void __exit sprd_ads_exit(void)
{
	platform_driver_unregister(&sprd_ads_driver);
}

device_initcall_sync(sprd_ads_init);
module_exit(sprd_ads_exit);

MODULE_AUTHOR("Xiaolong Zhang <xiaolong.zhang@spreadtrum.com>");
MODULE_DESCRIPTION("spreadtrum avoid disturb solution");
MODULE_LICENSE("GPL");
