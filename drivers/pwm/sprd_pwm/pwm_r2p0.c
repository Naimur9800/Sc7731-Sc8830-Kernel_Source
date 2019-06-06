#include "../pwm-sprd.h"

enum pwm_r2p0_reg_t {
	PWM_R2P0_PRESCALE	= 0x0000,
	PWM_R2P0_CNT		= 0x0004,
	PWM_R2P0_DIV		= 0x0008,
	PWM_R2P0_LOW		= 0x000c,
	PWM_R2P0_HIGH		= 0x0010,
};

static void sprd_pwm_early_config(struct pwm_chip *chip,
	struct pwm_device *pwm, bool enable)
{
	struct sprd_pwm_chip *pc = container_of(chip,
		struct sprd_pwm_chip, chip);
	int mask, val;

	mask = 0x10 << (pwm->hwpwm);
	if (enable) {
		val = mask;
		regmap_update_bits(pc->syscon, REG_AON_APB_APB_EB0, mask, val);
		sprd_pwm_writel(pc, pwm->hwpwm, PWM_R2P0_PRESCALE, 0);
	} else {
		val = ~mask;
		sprd_pwm_writel(pc, pwm->hwpwm, PWM_R2P0_PRESCALE, 0);
		regmap_update_bits(pc->syscon, REG_AON_APB_APB_EB0, mask, val);
	}
}
static void sprd_pwm_config(struct pwm_chip *chip,
	struct pwm_device *pwm, int level, int prescale)
{
	struct sprd_pwm_chip *pc = container_of(chip,
		struct sprd_pwm_chip, chip);

	sprd_pwm_writel(pc, pwm->hwpwm, PWM_R2P0_CNT,
			(level << 8) | PWM_MOD_MAX);
	sprd_pwm_writel(pc, pwm->hwpwm, PWM_R2P0_LOW, PWM_REG_MSK);
	sprd_pwm_writel(pc, pwm->hwpwm, PWM_R2P0_HIGH, PWM_REG_MSK);
	sprd_pwm_writel(pc, pwm->hwpwm, PWM_R2P0_PRESCALE, (1 << 8) | prescale);
}

struct pwm_core_ops pwm_r2p0_ops = {
	.pwm_early_config = sprd_pwm_early_config,
	.pwm_config = sprd_pwm_config,
};

