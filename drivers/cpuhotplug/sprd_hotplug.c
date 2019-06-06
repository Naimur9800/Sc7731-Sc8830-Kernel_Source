#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kernel_stat.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/percpu-defs.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/tick.h>
#include <linux/types.h>
#include <linux/cpu.h>
#include <linux/thermal.h>
#include <linux/err.h>
#include <linux/earlysuspend.h>
#include <linux/suspend.h>
#include <asm/cacheflush.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/pm_qos.h>

static struct kobject hotplug_kobj;
static struct task_struct *ksprd_hotplug;
static struct sd_dbs_tuners *g_sd_tuners = NULL;
static unsigned long boot_done;
static struct mutex cpu_num_lock;

#ifdef CONFIG_SS_TOUCH_BOOST_CPU_HOTPLUG
struct semaphore tb_sem;
static struct task_struct *ksprd_tb;
bool g_is_suspend=false;
static unsigned long tp_time;
#endif

static struct delayed_work plugin_work;
static struct delayed_work unplug_work;
static struct work_struct plugin_request_work;
static struct work_struct unplug_request_work;

u64 g_prev_cpu_wall[4] = {0};
u64 g_prev_cpu_idle[4] = {0};

/* On-demand governor macros */
#define DEF_SAMPLING_DOWN_FACTOR		(1)
#define MAX_SAMPLING_DOWN_FACTOR		(100000)

#define GOVERNOR_BOOT_TIME	(50*HZ)

#define DEF_CPU_UP_MID_THRESHOLD		(80)
#define DEF_CPU_UP_HIGH_THRESHOLD		(90)
#define DEF_CPU_DOWN_MID_THRESHOLD		(30)
#define DEF_CPU_DOWN_HIGH_THRESHOLD		(40)

#define MAX_CPU_NUM  (4)
#define MAX_PERCPU_TOTAL_LOAD_WINDOW_SIZE  (8)
#define MAX_PLUG_AVG_LOAD_SIZE (2)
#define DEF_CPU_NUM_MIN	(1)
/* default boost pulse time, us */
#define DEF_BOOSTPULSE_DURATION		(500)

#define mod(n, div) ((n) % (div))

enum core_min {
	HOTPLUG_TOUCH_BOOST = 0,	/* for one driver */
	HOTPLUG_POWER_HINT,	/* for one user at HAL */
	HOTPLUG_TEST_MIN,	/* for debug */
	HOTPLUG_CORE_MIN_ALL,
};

enum core_max {
	HOTPLUG_THERMAL = 0,	/* for thermal */
	HOTPLUG_TEST_MAX,	/* for debug */
	HOTPLUG_CORE_MAX_ALL,
};

struct sd_dbs_tuners {
	unsigned int ignore_nice;
	unsigned int sampling_down_factor;
	unsigned int io_is_busy;

	unsigned int cpu_hotplug_disable;
	unsigned int is_suspend;
	unsigned int cpu_num_max_limit;	/* max num of cpu for thermal */
	unsigned int cpu_num_min_limit;	/* min num of cpu for touch boost */
	unsigned int cpu_up_mid_threshold;
	unsigned int cpu_up_high_threshold;
	unsigned int cpu_down_mid_threshold;
	unsigned int cpu_down_high_threshold;
	unsigned int up_window_size;
	unsigned int down_window_size;
	int boostpulse_duration;
	struct pm_qos_request max_cpu_request[HOTPLUG_CORE_MAX_ALL];
	struct pm_qos_request min_cpu_request[HOTPLUG_CORE_MIN_ALL];
};

struct cpufreq_conf {
	struct clk 					*clk;
	struct clk 					*mpllclk;
	struct clk 					*tdpllclk;
	struct regulator 				*regulator;
	struct cpufreq_frequency_table			*freq_tbl;
	unsigned int					*vddarm_mv;
};

extern struct cpufreq_conf *sprd_cpufreq_conf;

static inline u64 get_cpu_idle_time_jiffy(unsigned int cpu, u64 *wall)
{
	u64 idle_time;
	u64 cur_wall_time;
	u64 busy_time;

	cur_wall_time = jiffies64_to_cputime64(get_jiffies_64());

	busy_time = kcpustat_cpu(cpu).cpustat[CPUTIME_USER];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_SYSTEM];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_STEAL];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_NICE];

	idle_time = cur_wall_time - busy_time;
	if (wall)
		*wall = cputime_to_usecs(cur_wall_time);

	return cputime_to_usecs(idle_time);
}

static inline u64
get_cpu_idle_time_sprd(unsigned int cpu, u64 *wall, int io_busy)
{
	u64 idle_time = get_cpu_idle_time_us(cpu, io_busy ? wall : NULL);

	if (idle_time == -1ULL)
		return get_cpu_idle_time_jiffy(cpu, wall);
	else if (!io_busy)
		idle_time += get_cpu_iowait_time_us(cpu, wall);

	return idle_time;
}

static void __cpuinit sprd_plugin_one_cpu_ss(struct work_struct *work)
{
	int cpuid;

#ifdef CONFIG_HOTPLUG_CPU
	if (num_online_cpus() < g_sd_tuners->cpu_num_max_limit) {
		cpuid = cpumask_next_zero(0, cpu_online_mask);
		if (!g_sd_tuners->cpu_hotplug_disable) {
			pr_info("!!  we gonna plugin cpu%d  !!\n", cpuid);
			cpu_up(cpuid);
		}
	}
#endif
	return;
}

static void sprd_unplug_one_cpu_ss(struct work_struct *work)
{
	unsigned int cpuid = 0;

#ifdef CONFIG_HOTPLUG_CPU
	if (num_online_cpus() > g_sd_tuners->cpu_num_min_limit) {
		if (!g_sd_tuners->cpu_hotplug_disable) {
			cpuid = cpumask_next(0, cpu_online_mask);
			pr_info("!!  we gonna unplug cpu%d  !!\n", cpuid);
			cpu_down(cpuid);
		}
	}
#endif
	return;
}

static void sprd_unplug_cpus(struct work_struct *work)
{
	int cpu;
	int be_offline_num;

#ifdef CONFIG_HOTPLUG_CPU
	int max_num;
	max_num = g_sd_tuners->cpu_num_max_limit;
	if (num_online_cpus() > max_num) {
		be_offline_num = num_online_cpus() - max_num;
		for_each_online_cpu(cpu) {
			if (0 == cpu)
				continue;
			pr_info("!!  all gonna unplug cpu%d  !!\n", cpu);
			cpu_down(cpu);
			if (--be_offline_num <= 0)
				break;
		}
	}
#endif
	return;
}

static void sprd_plugin_cpus(struct work_struct *work)
{
	int cpu, max_num;
	int be_online_num = 0;

#ifdef CONFIG_HOTPLUG_CPU
	mutex_lock(&cpu_num_lock);
	if (!g_sd_tuners->cpu_hotplug_disable)
		max_num = g_sd_tuners->cpu_num_min_limit;
	else
		max_num = g_sd_tuners->cpu_num_max_limit;
	mutex_unlock(&cpu_num_lock);
	if (num_online_cpus() < max_num) {
		be_online_num = max_num - num_online_cpus();
		for_each_possible_cpu(cpu) {
			if (!cpu_online(cpu)) {
				pr_info("!! all gonna plugin cpu%d  !!\n",
						cpu);
				cpu_up(cpu);
				if (--be_online_num <= 0)
					break;
			}
		}
	}
#endif
	return;
}

#define MAX_ARRAY_SIZE  (10)
#define UP_LOAD_WINDOW_SIZE  (1)
#define DOWN_LOAD_WINDOW_SIZE  (2)
unsigned int load_array[CONFIG_NR_CPUS][MAX_ARRAY_SIZE] = { {0} };
unsigned int window_index[CONFIG_NR_CPUS] = {0};

static unsigned int sd_avg_load(int cpu, struct sd_dbs_tuners *sd_tuners,
			unsigned int load, bool up)
{
	unsigned int window_size;
	unsigned int count;
	unsigned int scale;
	unsigned int sum_scale = 0;
	unsigned int sum_load = 0;
	unsigned int window_tail = 0, window_head = 0;

	if (up) {
		window_size = sd_tuners->up_window_size;
	} else {
		window_size = sd_tuners->down_window_size;
		goto skip_load;
	}

	load_array[cpu][window_index[cpu]] = load;
	window_index[cpu]++;
	window_index[cpu] = mod(window_index[cpu], MAX_ARRAY_SIZE);

skip_load:
	if (!window_index[cpu])
		window_tail = MAX_ARRAY_SIZE - 1;
	else
		window_tail = window_index[cpu] - 1;

	window_head = mod(MAX_ARRAY_SIZE + window_tail - window_size + 1,
			MAX_ARRAY_SIZE);
	for (scale = 1, count = 0; count < window_size;
			scale += scale, count++) {
		pr_debug("%s load_array[%d][%d]: %d, scale: %d\n",
				up ? "up" : "down", cpu, window_head,
				load_array[cpu][window_head], scale);
		sum_load += (load_array[cpu][window_head] * scale);
		if (load_array[cpu][window_head] != 0)
			sum_scale += scale;
		window_head++;
		window_head = mod(window_head, MAX_ARRAY_SIZE);
	}

	return sum_scale ? sum_load / sum_scale : 0;
}

void sd_check_cpu_sprd(unsigned int load)
{
	unsigned int itself_avg_load = 0;
	struct sd_dbs_tuners *sd_tuners = g_sd_tuners;

	if (time_before(jiffies, boot_done))
		return;

	if (sd_tuners->cpu_hotplug_disable)
		return;

	pr_debug("efficient load %d, ---- online CPUs %d ----\n",
					load, num_online_cpus());

	/* cpu plugin check */
	itself_avg_load = sd_avg_load(0, sd_tuners, load, true);
	pr_debug("up itself_avg_load %d\n", itself_avg_load);
	if (num_online_cpus() < sd_tuners->cpu_num_max_limit) {
		int cpu_up_threshold;

		if (num_online_cpus() == 1)
			cpu_up_threshold = sd_tuners->cpu_up_mid_threshold;
		else
			cpu_up_threshold = sd_tuners->cpu_up_high_threshold;

		if (itself_avg_load > cpu_up_threshold) {
			schedule_delayed_work_on(0, &plugin_work, 0);
			return;
		}
	}

	/* cpu unplug check */
	if (num_online_cpus() > sd_tuners->cpu_num_min_limit) {
		int cpu_down_threshold;

		itself_avg_load = sd_avg_load(0, sd_tuners, load, false);
		pr_debug("down itself_avg_load %d\n", itself_avg_load);

		if (num_online_cpus() > sd_tuners->cpu_num_min_limit + 1)
			cpu_down_threshold = sd_tuners->cpu_down_high_threshold;
		else
			cpu_down_threshold = sd_tuners->cpu_down_mid_threshold;

		if (itself_avg_load < cpu_down_threshold)
			schedule_delayed_work_on(0, &unplug_work, 0);
	}
}

void dbs_check_cpu_sprd(void)
{
	unsigned int max_load = 0;
	unsigned int j;

	/* Get Absolute Load (in terms of freq for ondemand gov) */
	for_each_cpu(j, cpu_online_mask) {
		u64 cur_wall_time, cur_idle_time;
		unsigned int idle_time, wall_time;
		unsigned int load;
		int io_busy = 0;
		u64 prev_cpu_wall;
		u64 prev_cpu_idle;

		prev_cpu_wall = g_prev_cpu_wall[j];
		prev_cpu_idle = g_prev_cpu_idle[j];

		/*
		 * For the purpose of ondemand, waiting for disk IO is
		 * an indication that you're performance critical, and
		 * not that the system is actually idle. So do not add
		 * the iowait time to the cpu idle time.
		 */
		io_busy = g_sd_tuners->io_is_busy;
		cur_idle_time = get_cpu_idle_time_sprd(j, &cur_wall_time,
					io_busy);

		wall_time = (unsigned int)
			(cur_wall_time - prev_cpu_wall);

		idle_time = (unsigned int)
			(cur_idle_time - prev_cpu_idle);

		g_prev_cpu_wall[j] = cur_wall_time;
		g_prev_cpu_idle[j] = cur_idle_time;

		if (unlikely(!wall_time || wall_time < idle_time))
			continue;

		load = 100 * (wall_time - idle_time) / wall_time;
		if (load > max_load)
			max_load = load;
	}
	sd_check_cpu_sprd(max_load);
}

int _store_cpu_num_min_limit(unsigned int input)
{
	struct sd_dbs_tuners *sd_tuners = g_sd_tuners;

	pr_info("%s: input = %d\n", __func__, input);

	if (sd_tuners) {
		sd_check_cpu_sprd(50);
	} else {
		pr_info("[store_cpu_num_min_limit] current governor is not sprdemand\n");
		return -EINVAL;
	}

	return 0;
}

static int should_io_be_busy(void)
{
	return 1;
}

static int update_cpu_num_limit(void *param)
{
	int online_cpu_min, online_cpu_max;
	struct sd_dbs_tuners *sd_tuners = g_sd_tuners;

	if (!sd_tuners)
		return 0;

	mutex_lock(&cpu_num_lock);
	online_cpu_min = min(pm_qos_request(PM_QOS_CPU_CORE_MIN), nr_cpu_ids);
	online_cpu_max = min(pm_qos_request(PM_QOS_CPU_CORE_MAX), nr_cpu_ids);
	if (online_cpu_min > online_cpu_max)
		online_cpu_min = online_cpu_max;
	sd_tuners->cpu_num_max_limit = online_cpu_max;
	sd_tuners->cpu_num_min_limit = online_cpu_min;
	mutex_unlock(&cpu_num_lock);

	/* make sure num of cores within min and max
	 * for PM_QOS_CPU_CORE_MIN and PM_QOS_CPU_CORE_MAX class
	 */
	schedule_work_on(0, &plugin_request_work);
#if 0
	/* enable if thermal need it */
	schedule_work_on(0, &unplug_request_work);
#endif
	return 0;
}

/*
 * If PM_QOS_CPU_CORE_MIN and PM_QOS_CPU_CORE_MAX request is updated,
 * cpu_hotplug_qos_handler is called.
 */
static int cpu_hotplug_qos_handler(struct notifier_block *b,
		unsigned long val, void *v)
{
	return update_cpu_num_limit(v);
}

static struct notifier_block cpu_hotplug_qos_notifier = {
	.notifier_call = cpu_hotplug_qos_handler,
};

#ifdef CONFIG_BOOT_MIN_CPUHOTPLUG
static struct pm_qos_request boot_min_cpu_hotplug_request;
#define BOOT_MIN_CPUHOTPLUG_TIME	50
#endif
static void cpu_hotplug_pm_qos_init(struct sd_dbs_tuners *tuners)
{
	/* Register PM QoS notifier handler */
	pm_qos_add_notifier(PM_QOS_CPU_CORE_MIN, &cpu_hotplug_qos_notifier);
	pm_qos_add_notifier(PM_QOS_CPU_CORE_MAX, &cpu_hotplug_qos_notifier);
#ifdef CONFIG_BOOT_MIN_CPUHOTPLUG
	/* Guarantee all CPUs running during booting time */
	pm_qos_add_request(&boot_min_cpu_hotplug_request,
			PM_QOS_CPU_CORE_MIN, nr_cpu_ids);
	pm_qos_update_request_timeout(&boot_min_cpu_hotplug_request,
			nr_cpu_ids, BOOT_MIN_CPUHOTPLUG_TIME * USEC_PER_SEC);
#endif
	/* Add PM QoS for drivers */
	pm_qos_add_request(&tuners->max_cpu_request[HOTPLUG_THERMAL],
			PM_QOS_CPU_CORE_MAX, PM_QOS_CPU_CORE_MAX_DEFAULT_VALUE);
	pm_qos_add_request(&tuners->min_cpu_request[HOTPLUG_TOUCH_BOOST],
			PM_QOS_CPU_CORE_MIN, PM_QOS_CPU_CORE_MIN_DEFAULT_VALUE);

	/* Add PM QoS for debug test */
	pm_qos_add_request(&tuners->max_cpu_request[HOTPLUG_TEST_MAX],
			PM_QOS_CPU_CORE_MAX, PM_QOS_CPU_CORE_MAX_DEFAULT_VALUE);
	pm_qos_add_request(&tuners->min_cpu_request[HOTPLUG_TEST_MIN],
			PM_QOS_CPU_CORE_MIN, PM_QOS_CPU_CORE_MIN_DEFAULT_VALUE);

	/* Add PM QoS for one user at HAL */
	pm_qos_add_request(&tuners->min_cpu_request[HOTPLUG_POWER_HINT],
			PM_QOS_CPU_CORE_MIN, PM_QOS_CPU_CORE_MIN_DEFAULT_VALUE);
}

static int sd_tuners_init(struct sd_dbs_tuners *tuners)
{
	if (!tuners) {
		pr_err("%s: kzalloc failed\n", __func__);
		return -ENOMEM;
	}

	tuners->sampling_down_factor = DEF_SAMPLING_DOWN_FACTOR;
	tuners->ignore_nice = 0;
	tuners->io_is_busy = should_io_be_busy();

	tuners->cpu_hotplug_disable = true;
	tuners->is_suspend = false;
	tuners->cpu_up_mid_threshold = DEF_CPU_UP_MID_THRESHOLD;
	tuners->cpu_up_high_threshold = DEF_CPU_UP_HIGH_THRESHOLD;
	tuners->cpu_down_mid_threshold = DEF_CPU_DOWN_MID_THRESHOLD;
	tuners->cpu_down_high_threshold = DEF_CPU_DOWN_HIGH_THRESHOLD;
	tuners->up_window_size = UP_LOAD_WINDOW_SIZE;
	tuners->down_window_size = DOWN_LOAD_WINDOW_SIZE;
	tuners->boostpulse_duration = DEF_BOOSTPULSE_DURATION;
	tuners->cpu_num_min_limit = DEF_CPU_NUM_MIN;
	tuners->cpu_num_max_limit = nr_cpu_ids;
	if (tuners->cpu_num_max_limit > DEF_CPU_NUM_MIN)
		tuners->cpu_hotplug_disable = false;

	mutex_init(&cpu_num_lock);
	INIT_DELAYED_WORK(&plugin_work, sprd_plugin_one_cpu_ss);
	INIT_DELAYED_WORK(&unplug_work, sprd_unplug_one_cpu_ss);
	INIT_WORK(&plugin_request_work, sprd_plugin_cpus);
	INIT_WORK(&unplug_request_work, sprd_unplug_cpus);
	return 0;
}

static int sprd_hotplug(void *data)
{
	while (1) {
		if (time_before(jiffies, boot_done)) {
			msleep(1000);
			continue;
		}
		dbs_check_cpu_sprd();
		msleep(40);
	}
	return 0;
}

static ssize_t
cpufreq_table_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	memcpy(buf, sprd_cpufreq_conf->freq_tbl,
			sizeof(*sprd_cpufreq_conf->freq_tbl));
	return sizeof(*sprd_cpufreq_conf->freq_tbl);
}

static ssize_t
store_io_is_busy(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	struct sd_dbs_tuners *sd_tuners = g_sd_tuners;
	unsigned int input;
	int ret;
	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	sd_tuners->io_is_busy = !!input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		g_prev_cpu_idle[j] = get_cpu_idle_time_sprd(j,
				&g_prev_cpu_wall[j], should_io_be_busy());
	}
	return count;
}

static ssize_t
show_io_is_busy(struct device *dev, struct device_attribute *attr, char *buf)
{
	snprintf(buf, 10, "%d\n", g_sd_tuners->io_is_busy);
	return strlen(buf) + 1;
}

static ssize_t
store_sampling_down_factor(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct sd_dbs_tuners *sd_tuners = g_sd_tuners;
	unsigned int input;

	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_SAMPLING_DOWN_FACTOR || input < 1)
		return -EINVAL;
	sd_tuners->sampling_down_factor = input;

	return count;
}

static ssize_t
show_sampling_down_factor(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	snprintf(buf, 10, "%d\n", g_sd_tuners->sampling_down_factor);
	return strlen(buf) + 1;
}

static ssize_t
store_ignore_nice(struct device *dev, struct device_attribute *attr,
		  const char *buf, size_t count)
{
	struct sd_dbs_tuners *sd_tuners = g_sd_tuners;
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == sd_tuners->ignore_nice) { /* nothing to do */
		return count;
	}
	sd_tuners->ignore_nice = input;

	return count;
}

static ssize_t
show_ignore_nice(struct device *dev, struct device_attribute *attr, char *buf)
{
	snprintf(buf, 10, "%d\n", g_sd_tuners->ignore_nice);
	return strlen(buf) + 1;
}

static ssize_t store_cpu_num_max_limit(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sd_dbs_tuners *sd_tuners = g_sd_tuners;
	unsigned int input = 0;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input < DEF_CPU_NUM_MIN || input > nr_cpu_ids)
		return -EINVAL;

	pm_qos_update_request(&sd_tuners->max_cpu_request[HOTPLUG_TEST_MAX],
			input);
	schedule_work_on(0, &unplug_request_work);
	return count;
}

static ssize_t show_cpu_num_max_limit(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	snprintf(buf, 10, "%d\n", g_sd_tuners->cpu_num_max_limit);
	return strlen(buf) + 1;
}

static ssize_t store_cpu_num_min_limit(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sd_dbs_tuners *sd_tuners = g_sd_tuners;
	unsigned int input = 0;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input < DEF_CPU_NUM_MIN || input > nr_cpu_ids)
		return -EINVAL;

	pm_qos_update_request(&sd_tuners->min_cpu_request[HOTPLUG_TEST_MIN],
			input);
	return count;
}

static ssize_t
show_cpu_num_min_limit(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	snprintf(buf, 10, "%d\n", g_sd_tuners->cpu_num_min_limit);
	return strlen(buf) + 1;
}

static ssize_t __ref store_cpu_hotplug_disable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)

{
	struct sd_dbs_tuners *sd_tuners = g_sd_tuners;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1) {
		return -EINVAL;
	}

	if (sd_tuners->cpu_hotplug_disable == input) {
		return count;
	}

	sd_tuners->cpu_hotplug_disable = input;

	smp_wmb();
	/* plug-in all offline cpu mandatory if we didn't
	 * enbale CPU_DYNAMIC_HOTPLUG
	*/
#ifdef CONFIG_HOTPLUG_CPU
	do {
		int max_num = sd_tuners->cpu_num_max_limit;
		if (sd_tuners->cpu_hotplug_disable &&
				num_online_cpus() < max_num) {
			schedule_work_on(0, &plugin_request_work);
			do {
				msleep(5);
				pr_debug("wait for all cpu online!\n");
			} while (num_online_cpus() < max_num);
		}
	} while (0);
#endif
	return count;
}

static ssize_t show_cpu_hotplug_disable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	snprintf(buf,10,"%d\n",g_sd_tuners->cpu_hotplug_disable);
	return strlen(buf) + 1;
}

static ssize_t store_cpu_up_mid_threshold(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sd_dbs_tuners *sd_tuners = g_sd_tuners;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	sd_tuners->cpu_up_mid_threshold = input;
	return count;
}

static ssize_t show_cpu_up_mid_threshold(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 10, "%d\n", g_sd_tuners->cpu_up_mid_threshold);
}

static ssize_t store_cpu_up_high_threshold(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sd_dbs_tuners *sd_tuners = g_sd_tuners;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	sd_tuners->cpu_up_high_threshold = input;
	return count;
}

static ssize_t show_cpu_up_high_threshold(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 10, "%d\n", g_sd_tuners->cpu_up_high_threshold);
}

static ssize_t store_cpu_down_mid_threshold(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sd_dbs_tuners *sd_tuners = g_sd_tuners;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	sd_tuners->cpu_down_mid_threshold = input;
	return count;
}

static ssize_t show_cpu_down_mid_threshold(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 10, "%d\n", g_sd_tuners->cpu_down_mid_threshold);
}

static ssize_t store_cpu_down_high_threshold(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sd_dbs_tuners *sd_tuners = g_sd_tuners;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	sd_tuners->cpu_down_high_threshold = input;
	return count;
}

static ssize_t show_cpu_down_high_threshold(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 10, "%d\n", g_sd_tuners->cpu_down_high_threshold);
}

static ssize_t store_up_window_size(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sd_dbs_tuners *sd_tuners = g_sd_tuners;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	if (input > MAX_ARRAY_SIZE || input < 1)
		return -EINVAL;

	sd_tuners->up_window_size = input;
	return count;
}

static ssize_t show_up_window_size(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 10, "%d\n", g_sd_tuners->up_window_size);
}

static ssize_t store_down_window_size(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sd_dbs_tuners *sd_tuners = g_sd_tuners;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	if (input > MAX_ARRAY_SIZE || input < 1)
		return -EINVAL;

	sd_tuners->down_window_size = input;
	return count;
}

static ssize_t show_down_window_size(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 10, "%d\n", g_sd_tuners->down_window_size);
}

static ssize_t store_boostpulse_duration(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sd_dbs_tuners *sd_tuners = g_sd_tuners;
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	sd_tuners->boostpulse_duration = val;
	return count;
}

static ssize_t show_boostpulse_duration(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 10, "%d\n", g_sd_tuners->boostpulse_duration);
}

static ssize_t store_hotplug_boostpulse(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sd_dbs_tuners *sd_tuners = g_sd_tuners;
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val < DEF_CPU_NUM_MIN || val > nr_cpu_ids)
		return -EINVAL;

	pm_qos_update_request_timeout(
			&sd_tuners->min_cpu_request[HOTPLUG_POWER_HINT],
			val, sd_tuners->boostpulse_duration * USEC_PER_MSEC);
	return count;
}

static DEVICE_ATTR(cpufreq_table, 0440, cpufreq_table_show, NULL);
static DEVICE_ATTR(io_is_busy, 0660, show_io_is_busy, store_io_is_busy);
static DEVICE_ATTR(sampling_down_factor, 0660,
		show_sampling_down_factor, store_sampling_down_factor);
static DEVICE_ATTR(ignore_nice, 0660, show_ignore_nice, store_ignore_nice);
static DEVICE_ATTR(cpu_num_max_limit, 0660,
		show_cpu_num_max_limit, store_cpu_num_max_limit);
static DEVICE_ATTR(cpu_num_min_limit, 0660,
		show_cpu_num_min_limit, store_cpu_num_min_limit);
static DEVICE_ATTR(cpu_hotplug_disable, 0660,
		show_cpu_hotplug_disable, store_cpu_hotplug_disable);
static DEVICE_ATTR(cpu_up_mid_threshold, 0660,
		show_cpu_up_mid_threshold, store_cpu_up_mid_threshold);
static DEVICE_ATTR(cpu_up_high_threshold, 0660,
		show_cpu_up_high_threshold, store_cpu_up_high_threshold);
static DEVICE_ATTR(cpu_down_mid_threshold, 0660,
		show_cpu_down_mid_threshold, store_cpu_down_mid_threshold);
static DEVICE_ATTR(cpu_down_high_threshold, 0660,
		show_cpu_down_high_threshold, store_cpu_down_high_threshold);
static DEVICE_ATTR(up_window_size, 0660,
		show_up_window_size, store_up_window_size);
static DEVICE_ATTR(down_window_size, 0660,
		show_down_window_size, store_down_window_size);
static DEVICE_ATTR(boostpulse_duration, 0660,
		show_boostpulse_duration, store_boostpulse_duration);
static DEVICE_ATTR(boostpulse, 0220,
		NULL, store_hotplug_boostpulse);

static struct attribute *g[] = {
	&dev_attr_cpufreq_table.attr,
	&dev_attr_io_is_busy.attr,
	&dev_attr_sampling_down_factor.attr,
	&dev_attr_ignore_nice.attr,
	&dev_attr_cpu_num_max_limit.attr,
	&dev_attr_cpu_num_min_limit.attr,
	&dev_attr_cpu_hotplug_disable.attr,
	&dev_attr_cpu_up_mid_threshold.attr,
	&dev_attr_cpu_up_high_threshold.attr,
	&dev_attr_cpu_down_mid_threshold.attr,
	&dev_attr_cpu_down_high_threshold.attr,
	&dev_attr_up_window_size.attr,
	&dev_attr_down_window_size.attr,
	&dev_attr_boostpulse_duration.attr,
	&dev_attr_boostpulse.attr,
	NULL,
};

static struct kobj_type hotplug_dir_ktype = {
	.sysfs_ops	= &kobj_sysfs_ops,
	.default_attrs	= g,
};

#ifdef CONFIG_SS_TOUCH_BOOST_CPU_HOTPLUG
static void dbs_input_event(struct input_handle *handle, unsigned int type,
		unsigned int code, int value)
{

	if (time_before(jiffies, boot_done))
		return;

	if (time_after(jiffies, tp_time))
		tp_time = jiffies + HZ / 2;
	else
		return;

	up(&tb_sem);
}

static bool dbs_match(struct input_handler *handler, struct input_dev *dev)
{
	/* touchpads and touchscreens */
	if (test_bit(EV_KEY, dev->evbit) &&
		test_bit(EV_ABS, dev->evbit) &&
		/* test_bit(BTN_TOUCH, dev->keybit) && */
		test_bit(ABS_MT_TOUCH_MAJOR, dev->absbit) &&
		test_bit(ABS_MT_POSITION_X, dev->absbit) &&
		test_bit(ABS_MT_POSITION_Y, dev->absbit) &&
		test_bit(ABS_MT_WIDTH_MAJOR, dev->absbit)
	) {
		return true;
	}

	return false;
}

static int dbs_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cpufreq";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	pr_info("[DVFS] dbs_input_connect register success\n");
	return 0;
err1:
	pr_info("[DVFS] dbs_input_connect register fail err1\n");
	input_unregister_handle(handle);
err2:
	pr_info("[DVFS] dbs_input_connect register fail err2\n");
	kfree(handle);
	return error;
}

static void dbs_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id dbs_ids[] = {
	{ .driver_info = 1 },
	{ },
};

struct input_handler dbs_input_handler = {
	.event		= dbs_input_event,
	.match		= dbs_match,
	.connect	= dbs_input_connect,
	.disconnect	= dbs_input_disconnect,
	.name		= "cpufreq_ond",
	.id_table	= dbs_ids,
};

static int sprd_tb_thread(void *data)
{
	while (1) {
		down(&tb_sem);
		if (num_online_cpus() < 3 && g_is_suspend == false)
			schedule_delayed_work_on(0, &plugin_work, 0);

	}
	return 0;
}
#endif

int cpu_core_thermal_limit(int cluster, int max_core)
{
	struct sd_dbs_tuners *sd_tuners = g_sd_tuners;

	if (!g_sd_tuners || max_core > nr_cpu_ids
			|| max_core < DEF_CPU_NUM_MIN)
		return -EINVAL;

	pm_qos_update_request(&sd_tuners->max_cpu_request[HOTPLUG_THERMAL],
			max_core);
	/* schedule_work_on(0, &unplug_request_work); */

	return 0;
}

#ifdef CONFIG_SPRD_TOUCH_BOOST
int cpu_core_touch_boost(int cluster, int min_core)
{
	struct sd_dbs_tuners *sd_tuners = g_sd_tuners;

	if (!g_sd_tuners || min_core > nr_cpu_ids
			|| min_core < DEF_CPU_NUM_MIN)
		return -EINVAL;

	pm_qos_update_request(&sd_tuners->min_cpu_request[HOTPLUG_TOUCH_BOOST],
			min_core);
	return 0;
}
#endif

static int __init sprd_hotplug_init(void)
{
	int ret;
	
	boot_done = jiffies + 50 * HZ;
	
	g_sd_tuners = kzalloc(sizeof(struct sd_dbs_tuners), GFP_KERNEL);

	sd_tuners_init(g_sd_tuners);

#ifdef CONFIG_SS_TOUCH_BOOST_CPU_HOTPLUG
	tp_time = jiffies;

	if (input_register_handler(&dbs_input_handler))
		pr_err("[DVFS] input_register_handler failed\n");

	sema_init(&tb_sem, 0);

	ksprd_tb = kthread_create(sprd_tb_thread, NULL, "sprd_tb_thread");

	wake_up_process(ksprd_tb);
#endif
	ksprd_hotplug = kthread_create(sprd_hotplug, NULL, "sprd_hotplug");

	wake_up_process(ksprd_hotplug);

	ret = kobject_init_and_add(&hotplug_kobj, &hotplug_dir_ktype,
				   &(cpu_subsys.dev_root->kobj), "cpuhotplug");
	if (ret)
		pr_err("%s: Failed to add kobject for hotplug\n", __func__);
	/* Initialize pm_qos request and handler */
	cpu_hotplug_pm_qos_init(g_sd_tuners);

	return ret;
}

MODULE_AUTHOR("sprd");
MODULE_LICENSE("GPL");
module_init(sprd_hotplug_init);
