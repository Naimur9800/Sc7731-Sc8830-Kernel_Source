/* ----------------------------------------------------------------------------
   Copyright (C) 2014 Intel Mobile Communications GmbH

 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  You should have received a copy of the GNU General Public License Version 2
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.

  ---------------------------------------------------------------------------*/
/*
 * NOTES:
 * 1) This source file is included in guests including Linux and purposely
 * kept in line with Linux kernel coding guidelines for ease of portability and
 * maintainability. See linux/Documentation/CodingStyle for coding guidelines.
 * Please check all further changes are compliant with Linux coding standard
 * and clear all commands reported by Linux checkpatch.pl
 * Use command: linux/scripts/checkpatch.pl -f <filename>
 * Clear ALL warnings and errors reported.
 *
 * 2) Use only C99 fixed width types for definition as this header needs to be
 * both 32bit/64bit portable.
 * Avoid the use of pointers/enum in structure as that make the structure
 * variable size based on 32/64bit toolchain used.
*/
#ifndef _MV_SVC_PM_H
#define _MV_SVC_PM_H

#define PM_API_VER		1
#define PM_API_MIN_VER		1

struct pm_control_shared_data {
	volatile uint32_t emif_curr_freq;
	volatile uint32_t emif_clk_src;
	volatile uint32_t prh_user_id;
	volatile uint32_t prh_per_id;
	volatile uint32_t prh_param[20];
	volatile uint32_t reserved0[4];
	volatile uint32_t vm_pow_state_param[20];
	volatile uint32_t vm_cpu_freq_param[20];
	volatile uint32_t prh_request_control_flag;
	volatile uint32_t prh_request_return_value;
	volatile uint32_t target_power_state;
	volatile uint32_t actual_power_state;
	volatile uint32_t modem_state;
	volatile uint32_t exit_latency;
	volatile uint32_t vm_blocker_id;
	volatile uint32_t vm_blocking_reason;
	volatile uint32_t calibration_state;
	volatile uint32_t gsm_sleep_timer_frames_in;
	volatile uint32_t gsm_sleep_timer_frames_out;
	volatile uint32_t gsm_sleep_timer_stopped;
	volatile uint32_t cpu_drv_param;
	volatile uint32_t cpu_scaling_states[7];
	volatile uint32_t cpu_clk;
	volatile uint32_t vcpu_c0[12];
	volatile uint32_t num_dev_sleep_blockers;
};

struct pm_control_shared_data *mv_svc_pm_get_shared_data(uint32_t cpu);

enum VMM_POWER_STATE_E {
	PM_S0,
	PM_S1,
	PM_S0i3 = 4,
	PM_S3 = 4,
	PM_S5 = 5,
};

enum vmm_pm_opcode {
        PM_OPCODE_START = 0,
        ENTER_IDLE = 1,
        ENTER_SLEEP = 2,
        SYSTEM_READY = 3,
        EMIC_INIT = 4,
        PM_PRH_PRE_INIT_SET_MODE = 5,
        PM_PRH_INIT_SET_MODE = 6,
        PM_PRH_LATE_INIT_SET_MODE = 7,
        PM_PRH_SET_PER_MODE = 8,
        PM_WAKEUP_CONTROL = 9,
        PM_CALIB_CONTROL = 10,
        PM_PRH_SET_PER_MODE_SYNC = 11,
        PM_PRH_SET_PER_MODE_ASYNC = 12,
        PM_CPU_DIS_PRF_INT = 13,
        PM_CPU_EN_PRF_INT = 14,
        PM_CPU_SCALING_START = 15,
        PM_CPU_LOAD2 = 16,
        PM_OMP_SET_POLICY = 17,
        PM_GET_VCPU_C0 = 18,
        PM_REQ_FREQ_CHNG = 19,
        PM_GSM_SLEEP_TIMER_START = 20,
        PM_GSM_SLEEP_TIMER_STOP = 21,
        PM_GSM_SLEEP_TIMER_GET_SLEPT_FRAMES = 22,
        PM_GSM_SLEEP_TIMER_IS_STOPPED = 23,
        PM_CPU_GET_NUM_FREQ_STEPS = 24,
        PM_CPU_GET_FREQ_STEPS = 25,
	PM_CPU_SET_FREQ_STEPS = 26,
	PM_DEBUG_IOCTL = 27,
	PM_IDLE_GET_NUM_CPU_IDLE_STATES = 28,
	PM_IDLE_GET_CPU_IDLE_TABLE = 29,
	PM_IDLE_GET_DEFAULT_SLEEP_STATE = 30,
	PM_OPCODE_END = 31,
};

/*  CPU idle state tables */
#define MAX_NUM_CPU_IDLE_STATES 10

#define CPUIDLE_ATTR_FLAGS_SYSTEM_SLEEP (1<<0)
#define CPUIDLE_ATTR_FLAGS_L1CACHE_FLUSHED (1<<1)
#define CPUIDLE_ATTR_FLAGS_L2CACHE_FLUSHED (1<<2)
#define CPUIDLE_ATTR_FLAGS_TLB_FLUSHED (1<<3)
#define CPUIDLE_ATTR_FLAGS_HPET_STOPPED (1<<4)
#define CPUIDLE_ATTR_FLAGS_LAPIC_TIMER_STOPPED (1<<5)
#define CPUIDLE_ATTR_FLAGS_STU_STOPPED (1<<6)
#define CPUIDLE_ATTR_FLAGS_TSC_STOPPED (1<<7)
struct mv_svc_cpu_idle_state {
        char name[8];
        uint32_t target_power_state;
        uint32_t latency;       /*us */
        uint32_t break_even_residency;  /* us */
        uint32_t power_consumption;     /* uW */
        uint32_t attr_flags;
};

struct mv_svc_cpu_idle_table {
        uint32_t num_idle_states;
        uint32_t default_idle_state;
        uint32_t default_sleep_state;
        uint32_t valid_cpus;
        struct mv_svc_cpu_idle_state states[MAX_NUM_CPU_IDLE_STATES];
};

/** @brief PM control
 *
 */
uint32_t mv_svc_pm_control(uint32_t pm_opcode, uint32_t arg1,
			uint32_t arg2, uint32_t arg3);
void mv_svc_modem_2g_sleep_time(
	struct pm_control_shared_data *pm_control_shared_data_p,
	uint32_t duration);
void mv_svc_modem_3g_sleep_time(
	struct pm_control_shared_data *pm_control_shared_data_p,
	uint32_t duration);
void mv_svc_modem_next_timeout(
	struct pm_control_shared_data *pm_control_shared_data_p,
	uint32_t duration);
void mv_svc_vm_enter_idle(
	struct pm_control_shared_data *pm_control_shared_data_p,
	uint32_t target_power_state);
void mv_svc_vm_enter_suspend(
	struct pm_control_shared_data *pm_control_shared_data_p,
	unsigned int state);
int32_t mv_svc_pm_idle_get_num_cpu_idle_states(unsigned int cpuid);
int32_t mv_svc_pm_idle_get_cpu_idle_table(struct mv_svc_cpu_idle_table
                                          *mv_svc_cpu_idle_table,
                                          unsigned int size,
                                          unsigned int cpuid);
int32_t mv_svc_pm_get_default_sleep_state(void);
int32_t mv_svc_pm_set_freq_table(unsigned int modid,
					u16 *freq_table,
					unsigned int size,
					unsigned int boost_index);
#endif /* _MV_SVC_PM_H */
