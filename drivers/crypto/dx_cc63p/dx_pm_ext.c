/**************************************************************
* Copyright 2014 (c) Discretix Technologies Ltd.              *
* This software is protected by copyright, international      *
* treaties and various patents. Any copy or reproduction of   *
* the software as permitted below, must include this          *
* Copyright Notice as well as any other notices provided      *
* under the relevant license.                                 *
*                                                             *
* This software shall be governed by, and may be used and     *
* redistributed under the terms and conditions of the GNU     *
* General Public License version 2, as published by the       *
* Free Software Foundation.                                   *
* This software is distributed in the hope that it will be    *
* useful, but WITHOUT ANY liability and WARRANTY; without     *
* even the implied warranty of MERCHANTABILITY or FITNESS     *
* FOR A PARTICULAR PURPOSE. See the GNU General Public        *
* License for more details.                                   *
* You should have received a copy of the GNU General          *
* Public License along with this software; if not, please     *
* write to the Free Software Foundation, Inc.,                *
* 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.   *
**************************************************************/


#include "dx_config.h"
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <crypto/ctr.h>
#include <linux/pm_runtime.h>
#include "dx_driver.h"
#include "dx_sram_mgr.h"
#include <linux/clk.h>


/*
This function should suspend the HW (if possiable), It should be implemented by 
the driver user. 
The reference code clears the internal SRAM to imitate lose of state. 
*/
void dx_pm_ext_hw_suspend(struct device *dev)
{
	struct dx_drvdata *drvdata =
		(struct dx_drvdata *)dev_get_drvdata(dev);
	unsigned int val;
	void __iomem *cc_base = drvdata->cc_base;
	unsigned int  sram_addr = 0;
#if DX_CC_HAS_ROM 
	sram_addr = READ_REGISTER(cc_base + DX_CC_REG_OFFSET(HOST_RGF, HOST_SEP_SRAM_THRESHOLD));
#endif

	WRITE_REGISTER(cc_base + DX_CC_REG_OFFSET(HOST_RGF, SRAM_ADDR), sram_addr);

	for (;sram_addr < DX_CC_SRAM_SIZE ; sram_addr+=4) {
		WRITE_REGISTER(cc_base + DX_CC_REG_OFFSET(HOST_RGF, SRAM_DATA), 0x0);

		do {
			val = READ_REGISTER(cc_base + DX_CC_REG_OFFSET(HOST_RGF, SRAM_DATA_READY));
		} while (!(val &0x1));
	}
    WRITE_REGISTER(cc_base + DX_CC_REG_OFFSET(HOST_RGF, HOST_CC_SW_RST), 1);

	clk_disable_unprepare(drvdata->clk_eb);

}

/*
This function should resume the HW (if possiable).It should be implemented by 
the driver user. 
*/
void dx_pm_ext_hw_resume(struct device *dev)
{
	struct dx_drvdata *drvdata =
		(struct dx_drvdata *)dev_get_drvdata(dev);

    if (NULL == drvdata) {
        DX_LOG_ERR("dx_pm_ext_hw_resume: dev_get_drvdata error!\n");
        return;
    }
	
	clk_prepare_enable(drvdata->clk_eb);
	
}

