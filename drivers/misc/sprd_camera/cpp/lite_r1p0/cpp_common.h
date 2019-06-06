#ifndef _CPP_COMMON_H_
#define _CPP_COMMON_H_

/* Macro Definition */

#define SCALE_LOWEST_ADDR              0x800
#define SCALE_ADDR_INVALID(addr) \
	((unsigned long)(addr) < SCALE_LOWEST_ADDR)
#define SCALE_YUV_ADDR_INVALID(y, u, v) \
	(SCALE_ADDR_INVALID(y) && \
	SCALE_ADDR_INVALID(u) && \
	SCALE_ADDR_INVALID(v))

#define SCALE_FRAME_WIDTH_MAX          8192
#define SCALE_FRAME_HEIGHT_MAX         8192
#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
#define SCALE_SC_COEFF_MAX             8
#else
#define SCALE_SC_COEFF_MAX             4
#endif
#define SCALE_DECI_FAC_MAX             4
#define SCALE_LINE_BUF_LENGTH          4096

#define SC_H_COEF_SIZE                 0x80
#define SC_H_CHROM_COEF_SIZE           0x40
#define SC_V_COEF_SIZE                 0x210

#define SC_COEFF_H_NUM                 (SC_H_COEF_SIZE / 4)
#define SC_COEFF_H_CHROMA_NUM          (SC_H_CHROM_COEF_SIZE / 4)
#define SC_COEFF_V_NUM                 (SC_V_COEF_SIZE / 4)

#define SC_COEFF_COEF_SIZE             (1 << 10)
#define SC_COEFF_TMP_SIZE \
	(SC_COEFF_BUF_SIZE - (SC_COEFF_COEF_SIZE*3))

#define SCALE_PIXEL_ALIGNED            4

#define SC_COEFF_BUF_SIZE              (16 << 10)

#endif
