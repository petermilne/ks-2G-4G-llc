/* ------------------------------------------------------------------------- */
/* file rtm_t_core_drv.c                                                              */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2011 Peter Milne, D-TACQ Solutions Ltd
 *                      <Peter dot Milne at D hyphen TACQ dot com>
 *  Created on: May 28, 2011
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

/** @file rtm_t_core_drv.c DESCR
 *
 */

#include <linux/kernel.h>

#include <linux/pci.h>
#include <linux/mm.h>

#include <linux/moduleparam.h>

#include <linux/mutex.h>

#include <asm/uaccess.h>  /* VERIFY_READ|WRITE */


#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#include <linux/module.h>
#endif


#include "rtm-t.h"
#include "rtm-t-hostdrv.h"
#define acq200_debug rtm_t_debug_regs
#include "acq200_debug.h"
#include "acq200.h"



int rtm_t_debug_regs = 0;
module_param(rtm_t_debug_regs, int, 0644);
MODULE_PARM_DESC(rtm_t_debug_regs, "set to 1 to debug regs access");


void rtd_write_reg(struct RTM_T_DEV *tdev, int regoff, u32 value)
{
	dbg(1, "%p = %08x", tdev->mappings[REGS_BAR].va + regoff, value);
	writel(value, tdev->mappings[REGS_BAR].va + regoff);
	if (rtm_t_debug_regs > 2){
		u32 rb = readl(tdev->mappings[REGS_BAR].va + regoff);
		dbg(2, "%p : %08x", tdev->mappings[REGS_BAR].va+regoff,rb);
	}
}

u32 rtd_read_reg(struct RTM_T_DEV *tdev, int regoff)
{
	u32 rv = readl(tdev->mappings[REGS_BAR].va + regoff);
	dbg(1, "%p = %08x", tdev->mappings[REGS_BAR].va + regoff, rv);
	return rv;
}

