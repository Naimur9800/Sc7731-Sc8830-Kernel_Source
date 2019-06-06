#ifndef __SPRD_K_CPP_H__
#define  __SPRD_K_CPP_H__

void isp_cpp_init(void *dev);
void isp_path_done_cpp_cfg(struct isp_pipe_dev *isp_dev,
	enum isp_scl_id path_id);
int isp_cpp_set_next_frame_for_path(struct isp_module *module,
	    enum isp_scl_id path_id);
void isp_cpp_deinit(void *dev);
void isp_cpp_get_cpp_dev(struct device **dev);
#endif
