#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/thermal.h>
#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/sprd-glb.h>

#define ITHM_PWRGD_STS		BIT(1)
#define ITHM_EFUSE_DONE_STS	BIT(2)
#define ITHM_TSEN_STS		BIT(5)
#define ITHM_TSRST_STS		BIT(6)

#define ITHM_ENABLE		BIT(0)

#define ITHM_DET_RDY_STS	BIT(0)
#define ITHM_ALRT_CLR		BIT(0)
#define ITHM_OVRHT_CLR		BIT(1)
#define ITHM_SFT_RST		BIT(2)

#define ITHM_ALRT_EN		BIT(0)
#define ITHM_OVRHT_EN		BIT(1)
#define ITHM_ALRT_INT_TYP	BIT(2)
#define ITHM_OVRHT_INT_TYP	BIT(3)

#define ITHM_INNER_STS_OVHT	BIT(4)
#define ITHM_TRIP_TRSHLD(val)	(val << 10)

#define ITHM_TSDIGPWRCTRL_OVRD BIT(8)
#define IThM_TSFILTERRATE      BIT(0)

/* value 0 means -50 degree celcius */
#define ITHM_ALRT_TRSHLD(val)	((val + 50) << 2)
#define ITHM_ALRT_HYST(val)	(val)

#define ALRT_HYST		0x2	/*Hyseteresis 2*/
#define ITHM_POWGD_RST_CYC	0x4000
#define SOC_SENSOR "bia-dts"

struct ithm_regs {
	u32 ithm_int_clr;
	u32 ithm_int_en;
	u32 ithm_en;
	u32 ithm_temp;
	u32 ithm_calb0;
	u32 ithm_calb1;
	u32 ithm_sns_stage;
	u32 ithm_sd_config;
	u32 ithm_logic_cfg0;
	u32 ithm_logic_cfg1;
	u32 ithm_det_per_h;
	u32 ithm_det_per_l;
	u32 ithm_pwrgd2rst_cyc;
	u32 ithm_tsen_cyc;
	u32 ithm_inner_sts;
	u32 ithm_efuse_dat;
};

struct bia_sensor {
	u32 sensor_id;
	struct regmap *aon_apb;
	struct platform_device *pdev;
	int alarm_irq;
	int crit_irq;
	u32 rf_period;
	u32 burst_mode;
	u32 warning_temp;
	u32 crit_temp;
	u32 ready_flag;
	u32 temp_ready;
	u32 disirq2ioapic;
	struct thermal_zone_device *tzd;
	struct ithm_regs __iomem *dts_base;
	struct clk *clk;
};

static struct thermal_zone_of_device_ops sensor_ops;

/* Formula to convert the adc value to temperature
 * T = (X - 400) /8
 * X - TEMPERATURE READING OF SDADC
 * This formula is taken from IP733TS spec
 * @returns in millicelcius
 */
static int intel_thermal_get_temp(void *data, int *temp)
{
	u32 value;
	int tpv;
	struct bia_sensor *dts = (struct bia_sensor *)data;

	value = ioread32(&dts->dts_base->ithm_inner_sts);
	/*when initializain is not ready or status is not ready
	 *return *temp = 0
	 */
	if ((!dts->ready_flag) || (!(ITHM_DET_RDY_STS & value))) {
		*temp = 0;
		dev_dbg(&dts->pdev->dev, "temp = %d\n", *temp);
		return 0;
	}
	tpv = ioread32(&dts->dts_base->ithm_temp);
	tpv = (tpv - 400) * 1000;
	tpv >>= 3;
	*temp = tpv;
	dev_dbg(&dts->pdev->dev, "temp = %d\n", *temp);
	return 0;
}

static inline void intel_thermal_set_ops(void)
{
	sensor_ops.get_temp = intel_thermal_get_temp;
}

/* This function is basically a place holder.
 * If we need to override caliberation configurations
 * use this function
 */
static void intel_thermal_calib_config(struct bia_sensor *dts)
{
	u32 value;
	struct ithm_regs __iomem *base = dts->dts_base;

	value = ioread32(&base->ithm_calb0);
	dev_dbg(&dts->pdev->dev, "calib0 value %x\n", value);
	value = ioread32(&base->ithm_calb0);
	dev_dbg(&dts->pdev->dev, "calib1 value %x\n", value);
}


/* This function is basically a place holder.
 * If we need to change any sigma delta adc configurations
 * use this function
 */

static void intel_thermal_sdadc_config(struct bia_sensor *dts)
{
	u32 value;
	struct ithm_regs __iomem *base = dts->dts_base;

	value = ioread32(&base->ithm_sd_config);
	dev_dbg(&dts->pdev->dev, "sd config value %x\n", value);

}

/* This function is basically a place holder.
 * If we need to change any sense stage configurations
 * use this function
 */
static void intel_thermal_sens_config(struct bia_sensor *dts)
{
	u32 value;
	struct ithm_regs __iomem *base = dts->dts_base;

	value = ioread32(&base->ithm_sns_stage);
	dev_dbg(&dts->pdev->dev, "sens stage value %x\n", value);
}

/* configure alert/crit thresholds*/
static void intel_thermal_threshold_config(struct bia_sensor *dts)
{

	u32 value;
	struct ithm_regs __iomem *base = dts->dts_base;
	/*check crit temp
	 *range should be from 115 to 125,which is safe for chip
	 *according to spec , the min critical is 115
	 */
	if (dts->crit_temp < 115000)
		dts->crit_temp = 115000;
	if (dts->crit_temp > 125000)
		dts->crit_temp = 125000;
	value = ITHM_TRIP_TRSHLD(((dts->crit_temp/1000) - 115)) |
		ITHM_ALRT_TRSHLD((dts->warning_temp/1000)) |
		ITHM_ALRT_HYST(ALRT_HYST);
	iowrite32(value, &base->ithm_logic_cfg0);

}

/* configure logic config */
static void intel_thermal_logic_config(struct bia_sensor *dts)
{
	u32 value;
	struct ithm_regs __iomem *base = dts->dts_base;

	value = ioread32(&base->ithm_logic_cfg1);
	value |= (ITHM_TSDIGPWRCTRL_OVRD | IThM_TSFILTERRATE);
	iowrite32(value, &base->ithm_logic_cfg1);
}

static void intel_thermal_period(struct bia_sensor *dts)
{
	u32 value;
	struct ithm_regs __iomem *base = dts->dts_base;
	/*From Alan:
	 *When the controller is in burst mode, because itsen of ip733ts will
	 *push up and down periodically, the alert signal will not hold when
	 *itsen push down if you set the int type of 0x1.  At this case,
	 *controllers detect period need to set to zero to let the itsen always
	 *on
	*/
	value = 0x0;
	iowrite32(value, &base->ithm_det_per_h);
	iowrite32(value, &base->ithm_det_per_l);

}

static void intel_dump_thm_regs(struct bia_sensor *dts)
{
	struct ithm_regs __iomem *base = dts->dts_base;
	u32 value;
	int tpv;

	value = ioread32(&base->ithm_inner_sts);
	dev_emerg(&dts->pdev->dev, "ithm_inner_sts = 0x%x\n", value);
	value = ioread32(&base->ithm_int_clr);
	dev_emerg(&dts->pdev->dev, "ithm_int_clr = 0x%x\n", value);
	value = ioread32(&base->ithm_calb0);
	dev_emerg(&dts->pdev->dev, "ithm_calb0 = 0x%x\n", value);
	value = ioread32(&base->ithm_calb1);
	dev_emerg(&dts->pdev->dev, "ithm_calb1 = 0x%x\n", value);
	value = ioread32(&base->ithm_sns_stage);
	dev_emerg(&dts->pdev->dev, "ithm_sns_stage = 0x%x\n", value);
	value = ioread32(&base->ithm_sd_config);
	dev_emerg(&dts->pdev->dev, "ithm_sd_config = 0x%x\n", value);
	value = ioread32(&base->ithm_logic_cfg0);
	dev_emerg(&dts->pdev->dev, "ithm_logic_cfg0 = 0x%x\n", value);
	value = ioread32(&base->ithm_logic_cfg1);
	dev_emerg(&dts->pdev->dev, "ithm_logic_cfg1 = 0x%x\n", value);
	value = ioread32(&base->ithm_det_per_h);
	dev_emerg(&dts->pdev->dev, "ithm_det_per_h = 0x%x\n", value);
	value = ioread32(&base->ithm_det_per_l);
	dev_emerg(&dts->pdev->dev, "ithm_det_per_l = 0x%x\n", value);
	value = ioread32(&base->ithm_pwrgd2rst_cyc);
	dev_emerg(&dts->pdev->dev, "ithm_pwrgd2rst_cyc = 0x%x\n", value);
	value = ioread32(&base->ithm_tsen_cyc);
	dev_emerg(&dts->pdev->dev, "ithm_tsen_cyc = 0x%x\n", value);
	value = ioread32(&base->ithm_efuse_dat);
	dev_emerg(&dts->pdev->dev, "ithm_efuse_dat = 0x%x\n", value);
	value = ioread32(&base->ithm_int_en);
	dev_emerg(&dts->pdev->dev, "ithm_int_en = 0x%x\n", value);
	value = ioread32(&base->ithm_en);
	dev_emerg(&dts->pdev->dev, "ithm_en = 0x%x\n", value);
	tpv = ioread32(&base->ithm_temp);
	tpv = (tpv - 400) * 1000;
	tpv >>= 3;
	dev_emerg(&dts->pdev->dev, "bia temp = %d\n", tpv);
	value = ioread32(&base->ithm_inner_sts);
	dev_emerg(&dts->pdev->dev, "ithm_inner_sts = 0x%x\n", value);
}
/* Power up of sequencce of the thermal sensor
 * pwr_gd->enable->reset
 * minimum delay between the pwr_gd and reset
 * should be 4us.
 */
static int intel_thermal_init(struct bia_sensor *dts)
{
	u32 value, retry = 10;
	struct ithm_regs __iomem *base = dts->dts_base;
	u32 efused = 0;
	u32 mask, looptry = 10;

	if (dts->disirq2ioapic == 1) {
		/* disable alarm sw int to ioapic for cpu thermal.
		 * it is meanless to trigger
		 * interrupt to cpu since thermal_zone_device_update will be
		 * called by IPA
		 */
		mask = BIT_AON_APB_SP_THM_OVERHEAT0_EN;
		regmap_update_bits(dts->aon_apb, REG_AON_APB_THERMAL_ALERT_EN,
					mask, mask);
	}

	/* config powergd to rst cyc */
	value = ITHM_POWGD_RST_CYC;
	iowrite32(value, &base->ithm_pwrgd2rst_cyc);

	/*when ITHM_EFUSE_DONE_STS, check if efuse data is not
	 *empty. If empty , write harding code cali data.Or else
	 *make use of cali data efused.
	 */
	value = ioread32(&base->ithm_inner_sts);
	if (value & ITHM_EFUSE_DONE_STS) {
		value = ioread32(&base->ithm_efuse_dat);
		if (value)
			efused = 1;
	} else {
		dev_err(&dts->pdev->dev, "efused data is not ready\n");
		return -EIO;
	}

	if (!efused) {
		/*set default calibration data and enable software config*/
		iowrite32(0x3F15, &base->ithm_calb0);
		iowrite32(0xAC08, &base->ithm_calb1);
	}

	/*enable thermal after set calibration data*/
	do {
		value = ioread32(&base->ithm_en);
		value |= ITHM_ENABLE;
		iowrite32(value, &base->ithm_en);

		/*wait till the first temp is ready*/
		retry = 10;
		do {
			/*according to spec, max is 120us before ready*/
			udelay(120);
			value = ioread32(&dts->dts_base->ithm_inner_sts);
		} while ((!(ITHM_DET_RDY_STS & value)) && --retry);

		if (value & ITHM_DET_RDY_STS) {
			dts->temp_ready = 1;
			break;
		} else {
			value = ioread32(&base->ithm_en);
			value &= ~ITHM_ENABLE;
			iowrite32(value, &base->ithm_en);
			/* delay before enable ithm */
			udelay(100);
		}
	} while (--looptry);

	if (dts->temp_ready) {
		/* clear critial interrupr */
		value = ITHM_OVRHT_CLR;
		iowrite32(value, &base->ithm_int_clr);

		/* enable alert and crit interrupt */
		value = ioread32(&base->ithm_int_en);
		/* for cpu sensor, we enable aler int
		 * for other sensor like gpu , we disable alert int
		 */
		value |= ((dts->disirq2ioapic == 1) ? ITHM_ALRT_EN : 0)
			| ITHM_OVRHT_EN;

		/* Two types of configuration are available for
		 *interrupts. Hardware will de-assert the interrupt
		 * when temperature goes below the threshold. Other
		 *type, driver manually needs to clear this bit.
		 *Right now configuring for manual clearing. For
		 *critical interrupt, need to reset the IP in order to
		 * de-assert interrupt according IP733TS spec. Need to
		 * confirm  Alan.
		 */

		/* If we set software type, currently we do not have
		 *scheme to de-assert alarm interrupt as we want when
		 *temperature goes below the threshold to make punit
		 *scale back cpu frequency to 2184M but HW type can
		 */
		 /*set hardware type */
		value |= ITHM_ALRT_INT_TYP;
		/*set software clear for overheat int */
		value &= ~ITHM_OVRHT_INT_TYP;
		iowrite32(value, &base->ithm_int_en);
	} else {
		intel_dump_thm_regs(dts);
		panic("failed to get temperature ready bit\n");
	}

	return 0;
}

static irqreturn_t intel_thermal_crit_isr(int irq, void *data)
{
	return IRQ_WAKE_THREAD;
}


static irqreturn_t intel_thermal_crit_isr_thread(int irq, void *data)
{
	u32 value;
	struct bia_sensor *dts = (struct bia_sensor *)data;
	struct ithm_regs __iomem *base = dts->dts_base;

	/*check if the crit is really triggerred by thermal controller
	 *If not, ignore it to avoid unexpected kernel panic
	 */
	value = ioread32(&base->ithm_inner_sts);
	if ((!dts->ready_flag) && (!(value & ITHM_INNER_STS_OVHT))) {
		dev_dbg(&dts->pdev->dev, "invalid crit interrupt\n");
		return IRQ_HANDLED;
	}

	/* clear the interrupt */
	value = ioread32(&base->ithm_int_clr);
	value &= ~ITHM_OVRHT_CLR;
	iowrite32(value, &base->ithm_int_clr);

	return IRQ_HANDLED;
}

static irqreturn_t intel_thermal_alarm_isr(int irq, void *data)
{
	unsigned int value;
	unsigned int mask;
	struct bia_sensor *dts = (struct bia_sensor *)data;
	struct ithm_regs __iomem *base = dts->dts_base;

	/* disable alarm sw int */
	mask = BIT_AON_APB_SP_THM_OVERHEAT0_EN;
	regmap_update_bits(dts->aon_apb, REG_AON_APB_THERMAL_ALERT_EN,
			    mask, mask);

	/* clear the interrupt */
	value = ioread32(&base->ithm_int_clr);
	value &= ~ITHM_ALRT_CLR;
	iowrite32(value, &base->ithm_int_clr);
	return IRQ_WAKE_THREAD;
}


static irqreturn_t intel_thermal_alarm_isr_thread(int irq, void *data)
{

	unsigned int mask;
	struct bia_sensor *sensor = (struct bia_sensor *)data;


	thermal_zone_device_update(sensor->tzd);
	/* enable alarm sw int	*/
	mask = BIT_AON_APB_SP_THM_OVERHEAT0_EN;
	regmap_update_bits(sensor->aon_apb, REG_AON_APB_THERMAL_ALERT_EN,
			    mask, ~mask);
	return IRQ_HANDLED;
}

static int bia_thermal_probe(struct platform_device *pdev)
{
	struct bia_sensor *sensor;
	struct device_node *sensor_np;
	struct clk *clk;
	struct resource *res;
	int size, ret;

	sensor = devm_kzalloc(&pdev->dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->ready_flag = 0;
	sensor->temp_ready = 0;
	sensor_np = of_node_get(pdev->dev.of_node);
	if (!sensor_np) {
		dev_err(&pdev->dev, "device node not found\n");
		ret = -ENODEV;
		goto freemem;
	}

	clk = of_clk_get_by_name(sensor_np, "enable");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "Failed to get the clock\n");
		of_node_put(sensor_np);
		ret = -EINVAL;
		goto freemem;
	}

	sensor->aon_apb = syscon_regmap_lookup_by_phandle(sensor_np,
						  "sprd,syscon-enable");
	if (IS_ERR(sensor->aon_apb)) {
		dev_err(&pdev->dev, "aon_common_apb_controller failed!\n");
		of_node_put(sensor_np);
		ret = -ENODEV;
		goto freemem;
	}

	ret = of_alias_get_id(sensor_np, "thm-sensor");
	if (ret == -ENODEV) {
		dev_err(&pdev->dev, "no id property get\n");
		of_node_put(sensor_np);
		goto freemem;
	}

	sensor->sensor_id = ret;

	ret = of_property_read_u32(sensor_np, "intel,warning-temp",
					&sensor->warning_temp);
	if (ret) {
		dev_err(&pdev->dev, "no warning-temp property get\n");
		of_node_put(sensor_np);
		goto freemem;
	}

	ret = of_property_read_u32(sensor_np, "intel,otp-temp", &sensor->crit_temp);
	if (ret) {
		dev_err(&pdev->dev, "no otp-temp property get\n");
		of_node_put(sensor_np);
		goto freemem;
	}

	if (of_get_property(sensor_np,
			"sprd,disable-int2ioapic", NULL)) {
		sensor->disirq2ioapic = 1;
		dev_info(&pdev->dev, "disable irq to ioapic\n");
	}

	clk_prepare_enable(clk);
	sensor->clk = clk;
	of_node_put(sensor_np);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get register address\n");
		ret = -ENODEV;
		goto free;
	}
	size = resource_size(res);
	sensor->dts_base = devm_ioremap_nocache(
					&pdev->dev, res->start, size);
	if (IS_ERR(sensor->dts_base)) {
		dev_err(&pdev->dev, "io-remap failed\n");
		ret = PTR_ERR(sensor->dts_base);
		goto free;
	}

	sensor->alarm_irq = platform_get_irq(pdev, 0);
	if (sensor->alarm_irq < 0) {
		dev_err(&pdev->dev, "Failed to get irq number\n");
		ret = sensor->alarm_irq;
		goto free;
	}
	ret = devm_request_threaded_irq(&pdev->dev, sensor->alarm_irq,
				intel_thermal_alarm_isr,
				intel_thermal_alarm_isr_thread,
				IRQF_ONESHOT, "dts-alarm", sensor);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register irq %d\n",
						sensor->alarm_irq);
		goto free;
	}

	sensor->crit_irq = platform_get_irq(pdev, 1);
	if (sensor->crit_irq < 0) {
		dev_err(&pdev->dev, "Failed to get irq number\n");
		ret = sensor->crit_irq;
		goto free;
	}
	ret = devm_request_threaded_irq(&pdev->dev, sensor->crit_irq,
				intel_thermal_crit_isr,
				intel_thermal_crit_isr_thread,
				IRQF_ONESHOT, "dts-crit", sensor);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register irq %d\n",
						sensor->crit_irq);
		goto free;
	}
	sensor->pdev = pdev;
	platform_set_drvdata(pdev, sensor);

	/* single sensor module */
	dev_info(&pdev->dev, "Probed %s sensor. Id=%hu\n",
				SOC_SENSOR, sensor->sensor_id);
	intel_thermal_set_ops();

	intel_thermal_sdadc_config(sensor);
	intel_thermal_sens_config(sensor);
	intel_thermal_calib_config(sensor);
	intel_thermal_threshold_config(sensor);
	intel_thermal_period(sensor);
	intel_thermal_logic_config(sensor);
	if (intel_thermal_init(sensor)) {
		dev_err(&pdev->dev, "Failed to init\n");
		ret = -EIO;
		goto free;
	}
	sensor->tzd = thermal_zone_of_sensor_register(&pdev->dev,
							sensor->sensor_id,
							sensor,
							&sensor_ops);
	if (IS_ERR(sensor->tzd)) {
		dev_err(&pdev->dev, "Error registering sensor: %p\n",
							sensor->tzd);
		ret = PTR_ERR(sensor->tzd);
		goto free;
	}
	sensor->ready_flag = 1;
	thermal_zone_device_update(sensor->tzd);

	return 0;
free:
	clk_disable_unprepare(clk);
freemem:
	devm_kfree(&pdev->dev, sensor);
	platform_set_drvdata(pdev, NULL);
	return ret;
}

static int bia_thm_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	struct bia_sensor *sensor = platform_get_drvdata(pdev);
	struct ithm_regs __iomem *base = sensor->dts_base;
	u32 value;

	sensor->ready_flag = 0;
	sensor->temp_ready = 0;
	/*disable interrupt */
	value = ioread32(&base->ithm_int_en);
	value &= (~(ITHM_ALRT_EN | ITHM_OVRHT_EN));
	iowrite32(value, &base->ithm_int_en);

	/*disable thermal*/
	value = ioread32(&base->ithm_en);
	value &= ~ITHM_ENABLE;
	iowrite32(value, &base->ithm_en);

	clk_disable_unprepare(sensor->clk);

	return 0;
}

static int bia_thm_resume(struct platform_device *pdev)
{
	struct bia_sensor *sensor = platform_get_drvdata(pdev);

	clk_prepare_enable(sensor->clk);
	intel_thermal_threshold_config(sensor);
	intel_thermal_period(sensor);
	intel_thermal_logic_config(sensor);
	intel_thermal_init(sensor);
	sensor->ready_flag = 1;
	return 0;
}


static int bia_thermal_remove(struct platform_device *pdev)
{
	struct bia_sensor *sensor = platform_get_drvdata(pdev);

	sensor->ready_flag = 0;
	thermal_zone_device_unregister(sensor->tzd);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id bia_thermal_of_match[] = {
	{ .compatible = "intel,bia-thm" },
	{},
};
MODULE_DEVICE_TABLE(of, bia_thermal_of_match);

static struct platform_driver bia_thermal_platdrv = {
	.driver = {
		.name		= "bia-thm",
		.of_match_table = bia_thermal_of_match,
	},
	.suspend = bia_thm_suspend,
	.resume = bia_thm_resume,
	.probe	= bia_thermal_probe,
	.remove	= bia_thermal_remove,
};
module_platform_driver(bia_thermal_platdrv);

MODULE_LICENSE("GPL");
