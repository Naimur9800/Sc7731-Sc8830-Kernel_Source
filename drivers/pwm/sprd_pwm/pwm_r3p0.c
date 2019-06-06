#include "../pwm-sprd.h"

enum pwm_r3p0_reg_t {
	PWM_R3P0_PRESCALE	= 0x0000,
	PWM_R3P0_MOD		= 0x0004,
	PWM_R3P0_DUTY		= 0x0008,
	PWM_R3P0_DIV		= 0x000c,
	PWM_R3P0_PAT_LOW	= 0x0010,
	PWM_R3P0_PAT_HIGH	= 0x0014,
	PWM_R3P0_ENABLE		= 0x0018,
	PWM_R3P0VERSION		= 0x001c,
};

static void sprd_pwm_early_config(struct pwm_chip *chip,
	struct pwm_device *pwm, bool enable)
{
	struct sprd_pwm_chip *pc = container_of(chip,
		struct sprd_pwm_chip, chip);
	unsigned int mask = 0xf0;

	if (enable)
		regmap_update_bits(pc->syscon,
				REG_AON_APB_APB_EB0, mask, mask);
	else
		regmap_update_bits(pc->syscon,
				REG_AON_APB_APB_EB0, mask, ~mask);
}

static void sprd_pwm_config(struct pwm_chip *chip,
	struct pwm_device *pwm, int level, int prescale)
{
	struct sprd_pwm_chip *pc = container_of(chip,
		struct sprd_pwm_chip, chip);

	if (level == 0) {
		sprd_pwm_writel(pc, pwm->hwpwm, PWM_R3P0_ENABLE, 0);
		return;
	}

	sprd_pwm_writel(pc, pwm->hwpwm, PWM_R3P0_MOD, PWM_MOD_MAX);
	sprd_pwm_writel(pc, pwm->hwpwm, PWM_R3P0_DUTY, level);
	sprd_pwm_writel(pc, pwm->hwpwm, PWM_R3P0_PAT_LOW, PWM_REG_MSK);
	sprd_pwm_writel(pc, pwm->hwpwm, PWM_R3P0_PAT_HIGH, PWM_REG_MSK);
	sprd_pwm_writel(pc, pwm->hwpwm, PWM_R3P0_PRESCALE, prescale);
	sprd_pwm_writel(pc, pwm->hwpwm, PWM_R3P0_ENABLE, 1);
}

struct pwm_core_ops pwm_r3p0_ops = {
	.pwm_early_config = sprd_pwm_early_config,
	.pwm_config = sprd_pwm_config,
};

