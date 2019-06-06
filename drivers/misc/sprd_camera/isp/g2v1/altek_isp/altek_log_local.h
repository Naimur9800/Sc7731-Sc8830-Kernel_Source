/*
 * File: altek_log_local.h						*
 * Description: definition of log api					*
 *									*
 * (C)Copyright altek Corporation 2016					*
 *									*
 * History:								*
 *   2016/05/27; Caedmon Lai; Initial version				*
 */
#ifndef _ALTEK_ISP_LOG_LOCAL_H_
#define _ALTEK_ISP_LOG_LOCAL_H_

#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sched.h>

/*
 * log_severity	  :  Smaller number means MORE important
 * log_threshold  :  Bigger number allows MORE logs
 *
 * If ( log_severity <= log_threshold ) --> Log will be printed out
 *
 * Threshold 1(ALERT)
	Something bad happened and action must be taken immediately
 * Threshold 3(ERROR)
	An error condition, often used by drivers to indicate
	difficulties with the hardware
 * Threshold 4(WARNNING)
	A warning, meaning nothing serious by itself
	but might indicate problems
 * Threshold 6(INFO)
	Informational message e.g. startup information at
	driver initialization
 * Threshold 7(DEBUG)
	Debug messages
 */

#define isp_alert(fmt, ...) \
	do { \
		if (drvlog_threshold >= 1) \
			pr_alert(ALTEK_ISP_LOG_TAG ": " fmt "\n", \
				 ##__VA_ARGS__); \
	} while (0)
#define isp_err(fmt, ...) \
	do { \
		if (drvlog_threshold >= 3) \
			pr_alert(ALTEK_ISP_LOG_TAG ": " fmt "\n", \
				 ##__VA_ARGS__); \
	} while (0)
#define isp_warn(fmt, ...) \
	do { \
		if (drvlog_threshold >= 4) \
			pr_alert(ALTEK_ISP_LOG_TAG ": " fmt "\n", \
				 ##__VA_ARGS__); \
	} while (0)
#define isp_info(fmt, ...) \
	do { \
		if (drvlog_threshold >= 6) \
			pr_alert(ALTEK_ISP_LOG_TAG ": " fmt "\n", \
				 ##__VA_ARGS__); \
	} while (0)
#define isp_debug(fmt, ...) \
	do { \
		if (drvlog_threshold >= 7) \
			pr_alert(ALTEK_ISP_LOG_TAG ": " fmt "\n", \
				 ##__VA_ARGS__); \
	} while (0)

#define isp_info_err(fmt, ...) \
	do { isp_info("It: isp driver err: " fmt, ##__VA_ARGS__);; } while (0)
#define isp_debug_err(fmt, ...) \
	do { isp_debug("It: isp driver err: " fmt, ##__VA_ARGS__);; } while (0)
#define isp_pr_debug(fmt, ...) \
	do { \
		pr_debug(ALTEK_ISP_LOG_TAG ": %s - " fmt "\n", \
			__func__, ##__VA_ARGS__);; \
	} while (0)

/* Combination Item Tag - public */
#define isp_alert_pu_combo_item_tag(fmt, ...) \
	do { \
		isp_alert("Pu: %s - start.", __func__); \
		isp_alert("It: " fmt, ##__VA_ARGS__); \
		isp_alert("Pu: %s - end.", __func__); \
	} while (0)
#define isp_err_pu_combo_item_tag(fmt, ...) \
	do { \
		isp_err("Pu: %s - start.", __func__); \
		isp_err("It: " fmt, ##__VA_ARGS__); \
		isp_err("Pu: %s - end.", __func__); \
	} while (0)
#define isp_warn_pu_combo_item_tag(fmt, ...) \
	do { \
		isp_warn("Pu: %s - start.", __func__); \
		isp_warn("It: " fmt, ##__VA_ARGS__); \
		isp_warn("Pu: %s - end.", __func__); \
	} while (0)
/* Combination Item Tag - local */
#define isp_alert_lo_combo_item_tag(fmt, ...) \
	do { \
		isp_alert("Lo: %s - start.", __func__); \
		isp_alert("It: " fmt, ##__VA_ARGS__); \
		isp_alert("Lo: %s - end.", __func__); \
	} while (0)
#define isp_err_lo_combo_item_tag(fmt, ...) \
	do { \
		isp_err("Lo: %s - start.", __func__); \
		isp_err("It: " fmt, ##__VA_ARGS__); \
		isp_err("Lo: %s - end.", __func__); \
	} while (0)
#define isp_warn_lo_combo_item_tag(fmt, ...) \
	do { \
		isp_warn("Lo: %s - start.", __func__); \
		isp_warn("It: " fmt, ##__VA_ARGS__); \
		isp_warn("Lo: %s - end.", __func__); \
	} while (0)
#define isp_info_lo_combo_item_tag(fmt, ...) \
	do { \
		isp_info("Lo: %s - start.", __func__); \
		isp_info("It: " fmt, ##__VA_ARGS__); \
		isp_info("Lo: %s - end.", __func__); \
	} while (0)


/* Combination Description Tag - public */
#define isp_alert_pu_combo_desc_tag(fmt, ...) \
	do { \
		isp_alert("Pu: %s - start.", __func__); \
		isp_alert("De: " fmt, ##__VA_ARGS__); \
		isp_alert("Pu: %s - end.", __func__); \
	} while (0)
#define isp_err_pu_combo_desc_tag(fmt, ...) \
	do { \
		isp_err("Pu: %s - start.", __func__); \
		isp_err("De: " fmt, ##__VA_ARGS__); \
		isp_err("Pu: %s - end.", __func__); \
	} while (0)
#define isp_warn_pu_combo_desc_tag(fmt, ...) \
	do { \
		isp_warn("Pu: %s - start.", __func__); \
		isp_warn("De: " fmt, ##__VA_ARGS__); \
		isp_warn("Pu: %s - end.", __func__); \
	} while (0)
/* Combination Description Tag - local */
#define isp_alert_lo_combo_desc_tag(fmt, ...) \
	do { \
		isp_alert("Pu: %s - start.", __func__); \
		isp_alert("De: " fmt, ##__VA_ARGS__); \
		isp_alert("Pu: %s - end.", __func__); \
	} while (0)
#define isp_err_lo_combo_desc_tag(fmt, ...) \
	do { \
		isp_err("Lo: %s - start.", __func__); \
		isp_err("De: " fmt, ##__VA_ARGS__); \
		isp_err("Lo: %s - end.", __func__); \
	} while (0)
#define isp_warn_lo_combo_desc_tag(fmt, ...) \
	do { \
		isp_warn("Lo: %s - start.", __func__); \
		isp_warn("De: " fmt, ##__VA_ARGS__); \
		isp_warn("Lo: %s - end.", __func__); \
	} while (0)
#define isp_info_lo_combo_desc_tag(fmt, ...) \
	do { \
		isp_info("Lo: %s - start.", __func__); \
		isp_info("De: " fmt, ##__VA_ARGS__); \
		isp_info("Lo: %s - end.", __func__); \
	} while (0)


/* Start Tag - public */
#define isp_alert_pu_start_tag() \
	do { isp_alert("Pu: %s - start.", __func__);; } while (0)
#define isp_info_pu_start_tag() \
	do { isp_info("Pu: %s - start.", __func__);; } while (0)
#define isp_debug_pu_start_tag() \
	do { isp_debug("Pu: %s - start.", __func__);; } while (0)
/* Start Tag - local */
#define isp_alert_lo_start_tag() \
	do { isp_alert("Lo: %s - start.", __func__);; } while (0)
#define isp_info_lo_start_tag() \
	do { isp_info("Lo: %s - start.", __func__);; } while (0)
#define isp_debug_lo_start_tag() \
	do { isp_debug("Lo: %s - start.", __func__);; } while (0)


/* End Tag - public */
#define isp_alert_pu_end_tag() \
	do { isp_info("Pu: %s - end.", __func__);; } while (0)
#define isp_info_pu_end_tag() \
	do { isp_info("Pu: %s - end.", __func__);; } while (0)
#define isp_debug_pu_end_tag() \
	do { isp_debug("Pu: %s - end.", __func__);; } while (0)
/* End Tag - local */
#define isp_alert_lo_end_tag() \
	do { isp_info("Lo: %s - end.", __func__);; } while (0)
#define isp_info_lo_end_tag() \
	do { isp_info("Lo: %s - end.", __func__);; } while (0)
#define isp_debug_lo_end_tag() \
	do { isp_debug("Lo: %s - end.", __func__);; } while (0)


/* Item Tag */
#define isp_alert_item_tag(fmt, ...) \
	do { isp_alert("It: " fmt, ##__VA_ARGS__);; } while (0)
#define isp_err_item_tag(fmt, ...) \
	do { isp_err("It: " fmt, ##__VA_ARGS__);; } while (0)
#define isp_info_item_tag(fmt, ...) \
	do { isp_info("It: " fmt, ##__VA_ARGS__);; } while (0)
#define isp_debug_item_tag(fmt, ...) \
	do { isp_debug("It: " fmt, ##__VA_ARGS__);; } while (0)

/* Description Tag */
#define isp_info_desc_tag(fmt, ...) \
	do { isp_info("De: " fmt, ##__VA_ARGS__);; } while (0)
#define isp_debug_desc_tag(fmt, ...) \
	do { isp_debug("De: " fmt, ##__VA_ARGS__);; } while (0)

extern u32 drvlog_threshold;
extern u32 fwlog_threshold;
extern u8 fwlog_force_switch;

#endif /* _ALTEK_ISP_LOG_LOCAL_H_ */
