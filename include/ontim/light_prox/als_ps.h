
// device configuration
struct als_ps_platform_data {
        u32     calibrate_target;
        u16     als_time;
        u16     scale_factor;
        u16     gain_trim;
        u8      filter_history;
        u8      filter_count;
        u8      gain;
	u16	prox_threshold_hi;
	u16     prox_threshold_lo;
	u16	als_threshold_hi;
	u16     als_threshold_lo;
	u8	prox_int_time;
	u8	prox_adc_time;
	u8	prox_wait_time;
	u8	prox_intr_filter;
	u8	prox_config;
	u8	prox_pulse_cnt;
	u8	prox_gain;
	u8  int_gpio;/*<jewel add for test>*/
	int (*power_on)(int on);
};

