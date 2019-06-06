#ifndef __SPRD_CPUFREQHW_H__
#define __SPRD_CPUFREQHW_H__

#if defined(CONFIG_ARM_SPRD_CPUFREQHW_SHARKL3)
int sprd_cpufreqhw_opp_add(unsigned int cluster,
		unsigned long hz_freq, unsigned long u_volt, int idx);
bool sprd_cpufreqhw_enable(int cluster, bool en);
bool sprd_cpufreqhw_probed(int cluster);
int sprd_cpufreqhw_set_target(unsigned int cluster,
					unsigned int idx);
unsigned int sprd_cpufreqhw_get(int cluster);
#else
static int sprd_cpufreqhw_opp_add(unsigned int cluster,
		unsigned long hz_freq, unsigned long u_volt, int idx)
{
	return -ENODEV;
}
static bool sprd_cpufreqhw_enable(int cluster, bool en)
{
	return false;
}
static bool sprd_cpufreqhw_probed(int cluster)
{
	return false;
}
static int sprd_cpufreqhw_set_target(unsigned int cluster,
					unsigned int idx)
{
	return -ENODEV;
}
static unsigned int sprd_cpufreqhw_get(int cluster)
{
	return 0;
}
#endif

#endif


