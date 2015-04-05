/* ------------------------------------------------------------------------- */
/* file ao32-drv.c                                                                 */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2011 Peter Milne, D-TACQ Solutions Ltd
 *                      <Peter dot Milne at D hyphen TACQ dot com>
 *  Created on: Feb 3, 2012
 *      Author: pgm

    http://www.d-tacq.com

    This program is free software; you can redistribute it and/or modify
    it under the terms of Version 2 of the GNU General Public License
    as published by the Free Software Foundation;

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.                */
/* ------------------------------------------------------------------------- */

/** @file ao32-drv.c DESCR 
 *
 */
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/pci.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>

#include <asm/uaccess.h>  /* VERIFY_READ|WRITE */


#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#include <linux/module.h>
#endif

#include "lk-shim.h"

#include "rtm-t.h"
#include "rtm-t-hostdrv.h"
#include "rtm-t_ioctl.h"
#include "rtm-t-dmac.h"
#define acq200_debug ao32_debug
#include "acq200_debug.h"
#include "acq200.h"

#include "ao32-drv.h"
#undef MAXDIOBIT
#include "ao32cpci-regs.h"


int ao32_debug = 0;
module_param(ao32_debug, int, 0644);

void ao32_write_reg(void* reg_base, unsigned offset, unsigned value)
{
	dbg(2, "%p : %02x := %08x", reg_base, offset, value);
	writel(value, reg_base + offset);
}

unsigned ao32_read_reg(void* reg_base, unsigned offset)
{
	u32 reg = readl(reg_base + offset);
	dbg(2, "%p @ %02x => %08x", reg_base, offset, reg);
	return reg;
}
void ao32_onStart_LLC(struct RTM_T_DEV *tdev)
{
	void *reg_base = tdev->mappings[PBI_BAR].va;
	u32 fifcon = AO32_FIFCON_LLC_ENABLE;
	u32 syscon = 0;

	dbg(1, "01");
	ao32_write_reg(reg_base, AO32_FIFCON, fifcon|AO32_FIFCON_LLC_RESET);
	syscon |= AO32_SYSCON_DO_OE;
	syscon |= M_LLI 	<< AO32_SYSCON_DO_MODE_SHL;
	syscon |= S_PXI_3	<< AO32_SYSCON_DO_TRG_SEL_SHL;
	syscon |= S_PXI_1 	<< AO32_SYSCON_DO_EC_SEL_SHL;
	syscon |= AO32_SYSCON_DO_EXTCLK;
	syscon |= AO32_SYSCON_DAC_CLR_N;
	syscon |= M_LLI		<< AO32_SYSCON_AO_MODE_SHL;
	syscon |= S_PXI_3	<< AO32_SYSCON_AO_TRG_SEL_SHL;
	syscon |= S_PXI_1	<< AO32_SYSCON_AO_EC_SEL_SHL;
	syscon |= AO32_SYSCON_AO_EXTCLK;
	syscon |= AO32_SYSCON_AO_ENABLE;
	ao32_write_reg(reg_base, AO32_SYSCON, syscon);
	ao32_write_reg(reg_base, AO32_FIFCON, fifcon);
	dbg(1, "99");
}
void ao32_onStop_LLC(struct RTM_T_DEV *tdev)
{
	void *reg_base = tdev->mappings[PBI_BAR].va;
	u32 fifcon = 0;
	u32 syscon = 0;

	dbg(1, "01");
	syscon |= AO32_SYSCON_DO_OE;
	syscon |= AO32_SYSCON_AO_ENABLE;
	ao32_write_reg(reg_base, AO32_SYSCON, syscon);
	ao32_write_reg(reg_base, AO32_FIFCON, fifcon);
	dbg(1, "99");
}
