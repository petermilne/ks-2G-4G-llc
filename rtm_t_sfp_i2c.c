/* ------------------------------------------------------------------------- */
/* rtm_t_sfp_i2c.c RTM-T SFP monitoring device definitions 		     	     */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2010 Peter Milne, D-TACQ Solutions Ltd
 *                      <Peter dot Milne at D hyphen TACQ dot com>

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

/* HOSTDRV VERSION! */

/** @file rtm_t_sfp_i2c.c  RTM-T SFP monitoring driver hooks */

#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#include <linux/module.h>
#endif

#include <linux/moduleparam.h>


#include "rtm-t.h"

#define REVID	"acq100_rtm_t B1000"

#define acq200_debug	rtm_t_debug
#include "acq200_debug.h"

#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/i2c-gpio.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "rtm-t.h"
#include "rtm-t-hostdrv.h"

extern int rtm_t_debug;

static struct i2c_gpio_platform_data rtm_t_i2c_gpio_data_templates[] = {
	{
		.sda_pin	= RTMT_H_I2C_SDA1_R,
		.scl_pin	= RTMT_H_I2C_SCL1_R,
		.udelay 	= 1,		/* AVO min time=1usec */
		.sda_is_open_drain = 1,
		.scl_is_open_drain = 1
	},
	{
		.sda_pin	= RTMT_H_I2C_SDA2_R,
		.scl_pin	= RTMT_H_I2C_SCL2_R,
		.udelay 	= 1,		/* AVO min time=1usec */
		.sda_is_open_drain = 1,
		.scl_is_open_drain = 1
	}
};

#define CHIX(ch)	((ch)==2)	/* 2=>1 *=>0 */

static int acq100_rtm_t_sfp_i2c_create(struct RTM_T_DEV *td, int ch)
{
	int rc;
	struct platform_device* pd =
			kzalloc(2*sizeof(struct platform_device), GFP_KERNEL);
	pd->name = "i2c-hba_sfp";
	pd->id = td->idx*2+CHIX(ch);
	pd->dev.platform_data = &rtm_t_i2c_gpio_data_templates[CHIX(ch)];
	/** HACK ALERT */
	pd[1].resource = (struct resource*)td;
	rc = platform_device_register(pd);

	if (rc != 0){
		err("platform device register failed");
		kfree(pd);
	}else{
		td->hba_sfp_i2c[CHIX(ch)] = pd;
	}
	return rc;
}

static void acq100_rtm_t_sfp_i2c_remove(struct RTM_T_DEV *td, int ch)
{
	struct platform_device* pd = td->hba_sfp_i2c[CHIX(ch)];
	if (pd){
		td->hba_sfp_i2c[CHIX(ch)] = 0;
		platform_device_unregister(pd);
		kfree(pd);
	}
}

int create_sfp_i2c(struct RTM_T_DEV *td)
{
	acq100_rtm_t_sfp_i2c_create(td, 1);
	acq100_rtm_t_sfp_i2c_create(td, 2);
	return 0;
}

int remove_sfp_i2c(struct RTM_T_DEV *td)
{
	acq100_rtm_t_sfp_i2c_remove(td, 1);
	acq100_rtm_t_sfp_i2c_remove(td, 2);
	return 0;
}


