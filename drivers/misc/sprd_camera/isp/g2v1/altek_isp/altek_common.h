/*
* File: altek_local.h                                                       *
* Description: delcalration of local shared api and data                    *
*                                                                           *
* (C)Copyright altek Corporation 2016                                       *
*                                                                           *
* History:                                                                  *
*   2016/03/012; Caedmon Lai; Initial version                               *
*/
#ifndef _ALTEK_COMMON_
#define _ALTEK_COMMON_

/* Page size definition*/
#define PAGE_SIZE_4K	 0x1000
#define PHY_BASE_ADDR	0x0
/* Default timeout for waiting IRQ*/
/* msec*/
#define IRQ_WAIT_TIME 600

#define NUMBER_ZERO			0

typedef void (*common_isp_drv_init_t)(void *pri_data);
typedef void (*common_isp_drv_deinit_t)(void *pri_data);

struct common_ispdrv_init_mod {
	const char *name;
	common_isp_drv_init_t init_cb;
	common_isp_drv_deinit_t deinit_cb;
	void *pri_data;
};

/* get the command semaphore*/
struct semaphore *get_command_semaphore(void);

u32 command_start(const char *func_name);

u32 command_start_poll(const char *func_name);

u32 wait_for_irq(const char *func_name, u32 event);

void upload_data_file_mmu(unsigned int Addr, char *data, unsigned int size);

int dump_kva(char *filepath, void *_addr, u64 size);

int dump_memory(char *filepath, u64 addr, u64 size);

int dump_isp_ahb(char *filepath, u32 size);

int dump_isp_internal_reg(char *filepath, u32 start, u32 size, u8 hwid);

/* dump the data when the format is raw10*/
void dump_raw(void);

/* dump output Img*/
void dump_output_img(void);

u32 dump_img_by_index(u8 a_uc_sensor, u8 a_uc_output_image,
			     u8 a_uc_index);

void get_sum_xor_value(u8 *addr, u32 size, u32 *a_pud_sum,
			      u32 *a_pud_xor);

u32 load_golden_img(void *v4l2_device);

void common_ispdrv_init(void);
void common_ispdrv_deinit(void);
#endif
