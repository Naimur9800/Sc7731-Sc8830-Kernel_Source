#ifndef __SC2342_INTEGRATE_GLB_H__
#define __SC2342_INTEGRATE_GLB_H__

#ifdef PIKE2
#include "sc2342_glb_pike2.h"
#endif

#ifdef SHARKLE
#include "sc2342_glb_sharkle.h"
#endif

#ifdef SHARKL3
#include "sc2342_glb_sharkl3.h"
#endif
#include "wcn_dump_integrate.h"
#include "wcn_integrate.h"
#include "wcn_integrate_boot.h"
#include "wcn_integrate_dev.h"

#define MDBG_RX_RING_SIZE	(128 * 1024)
#endif
