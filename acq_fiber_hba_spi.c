/* ------------------------------------------------------------------------- */
/* file acq_fiber_hba_spi.c                                                                 */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2011 Peter Milne, D-TACQ Solutions Ltd
 *                      <Peter dot Milne at D hyphen TACQ dot com>
 *  Created on: May 30, 2011
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

/** @file acq_fiber_hba_spi.c DESCR 
 *
 * with acknowledgement to linux/drivers/spi/spi_s3c24xx.c
 *
 * Copyright (c) 2006 Ben Dooks
 * Copyright (c) 2006 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 */
/* ------------------------------------------------------------------------- */


#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>

/*
#include <asm/mach/flash.h>
*/
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#define acq200_debug acqfhba_spi_debug
#include "acq200_debug.h"

#include "rtm-t.h"
#include "rtm-t-hostdrv.h"

int acqfhba_spi_debug;
module_param(acqfhba_spi_debug, int, 0644);

int spi_write_msleep = 1;
module_param(spi_write_msleep, int, 0664);


struct acqfhba_spi {
	/* bitbang has to be first ?why? */
	struct spi_bitbang	bitbang;

	/* data buffers */
	const unsigned char	*tx;
	unsigned char		*rx;
	int			len;
	int			count;
	struct spi_master	*master;
	struct device		*dev;
	struct RTM_T_DEV 	*tdev;
	u32 ctrl_mirror;
	int sent_reset;
};


static inline void acqfhba_spi_write_ctl(struct acqfhba_spi *hw, u32 value)
{
	hw->ctrl_mirror = value;

	dbg(4, "\t%04x := 0x%08x)", RTMT_H_SPI_CTL, value);
	rtd_write_reg(hw->tdev, RTMT_H_SPI_CTL, hw->ctrl_mirror);
}

static inline u32 acqfhba_spi_read_ctl(struct acqfhba_spi *hw)
{
	u32 cc = rtd_read_reg(hw->tdev, RTMT_H_SPI_CTL);

	dbg(4, "\t%04x => 0x%08x", RTMT_H_SPI_CTL, cc);
	return cc;
}

static void acqfhba_spi_wait_busy(struct acqfhba_spi *hw)
{
	int pollcat = 0;

	dbg(3, "start..");
	while((acqfhba_spi_read_ctl(hw)&RTMT_Q_SPI_BUSY) != 0){
		yield();
		//msleep(1);
		++pollcat;
	}
	dbg(2, "done in %d", pollcat);
}

static void acqfhba_spi_start(struct acqfhba_spi *hw, int on)
{
	if (on){
		hw->ctrl_mirror |= RTMT_Q_SPI_CTL_START;
	}else{
		hw->ctrl_mirror &= ~RTMT_Q_SPI_CTL_START;
	}
	dbg(3, "%08x %s", hw->ctrl_mirror, on? "ON": "OFF");
	acqfhba_spi_write_ctl(hw, hw->ctrl_mirror);
}
static inline void acqfhba_spi_write(struct acqfhba_spi *hw, unsigned char cc)
{
	dbg(2, "\t%04x := %02x", RTMT_H_SPI_DAT, cc);

	rtd_write_reg(hw->tdev, RTMT_H_SPI_DAT, cc);
	acqfhba_spi_start(hw, 1);
	acqfhba_spi_wait_busy(hw);
	acqfhba_spi_start(hw, 0);
}

static inline unsigned char acqfhba_spi_read(struct acqfhba_spi *hw)
{
	unsigned char cc;

	acqfhba_spi_start(hw, 1);
	acqfhba_spi_wait_busy(hw);
	acqfhba_spi_start(hw, 0);
	cc = rtd_read_reg(hw->tdev, RTMT_H_SPI_DAT);
	dbg(2, "\t\tread(%04x) => %02x", RTMT_H_SPI_DAT, cc);

	return cc;
}

static inline struct acqfhba_spi *to_hw(struct spi_device *sdev)
{
	return spi_master_get_devdata(sdev->master);
}

static inline unsigned int hw_txbyte(struct acqfhba_spi *hw, int count)
{
	return hw->tx ? hw->tx[count] : 0;
}

static void rtm_t_spi_chipsel(struct spi_device *spi, int value)
{
	unsigned int cspol = spi->mode & SPI_CS_HIGH ? 1 : 0;
	struct acqfhba_spi *hw = to_hw(spi);

	dbg(1, "value %d", value);
	if (acqfhba_spi_debug >= 2){
		acqfhba_spi_read_ctl(hw);
	}

	switch(value){
	case BITBANG_CS_INACTIVE:
		acqfhba_spi_write_ctl(hw, cspol? 0: RTMT_Q_SPI_CS);
		break;
	case BITBANG_CS_ACTIVE:
		acqfhba_spi_write_ctl(hw, cspol? RTMT_Q_SPI_CS: 0);
		break;
	}

	if (acqfhba_spi_debug >= 2){
		acqfhba_spi_read_ctl(hw);
	}
}
static int rtm_t_spi_setupxfer(struct spi_device *spi,
				 struct spi_transfer *t)
{
	struct acqfhba_spi *hw = to_hw(spi);

	spin_lock(&hw->bitbang.lock);
	if (!hw->bitbang.busy) {
		hw->bitbang.chipselect(spi, BITBANG_CS_INACTIVE);
	}
	spin_unlock(&hw->bitbang.lock);
	return 0;
}

static int rtm_t_spi_setup(struct spi_device *spi)
{
	int rc;


	if (spi->bits_per_word != 8){
		spi->bits_per_word = 8;
	}
	if ((spi->mode & SPI_LSB_FIRST) != 0)
		return -EINVAL;

	rc = rtm_t_spi_setupxfer(spi, NULL);
	return rc;
}


static int rtm_t_spi_txrx(struct spi_device *spi, struct spi_transfer *t)
{
	struct acqfhba_spi *hw = to_hw(spi);

	dbg(1, "txrx: tx %p, rx %p, len %d",
		t->tx_buf, t->rx_buf, t->len);

	hw->tx = t->tx_buf;
	hw->rx = t->rx_buf;

	if (hw->tx != NULL && hw->rx != NULL){
		err("tx and rx requested. This is NOT good!");
	}

	hw->len = t->len;
	hw->count = 0;

	if (hw->tx){
	/* send the first byte */
		acqfhba_spi_write(hw, hw_txbyte(hw, 0));

		dbg(2, "02: done first write now while %d < %d",
			hw->count, hw->len);
		++hw->count;
	}

	for (; hw->count < hw->len; ++hw->count){
		//msleep(spi_write_msleep); 
		yield();

		if (hw->rx){
			hw->rx[hw->count] = acqfhba_spi_read(hw);
		}

		if (hw->tx){
			acqfhba_spi_write(hw, hw_txbyte(hw, hw->count));
		}
	}

	dbg(1, "return %d\n",hw->count);
	return hw->count;
}


/* arm style
static struct flash_platform_data rtm_t_flash_data = {
	.map_name		= "jedec_probe",
	.width			= 2,
	.type		        = "m25p64"
};
*/
static struct flash_platform_data rtm_t_flash_data = {
	.name			= "rtm-t-flash",
	.type			= "W25Q64"
};
static struct spi_board_info rtm_t_spi_devices[] = {
	{	/* DataFlash chip */
//		.modalias	= "mtd_dataflash",
//		.modalias	= "jedec_probe",
		.modalias	= "m25p80",
		.chip_select	= 0,
		.max_speed_hz	= 15 * 1000 * 1000,
		.mode = SPI_CS_HIGH,
		.platform_data = &rtm_t_flash_data
	},
};

int acqfhba_spi_master_init(struct RTM_T_DEV *tdev)
{
	struct device *dev = &tdev->pci_dev->dev;
	struct spi_master *master;
	int rc = 0;
	struct acqfhba_spi *hw;

	info("call spi_alloc_master, dev %p", dev);

	master = spi_alloc_master(dev, sizeof(struct acqfhba_spi));
	if (master == NULL){
		dev_err(dev, "No memory for spi_master\n");
		rc = -ENOMEM;
		goto err_nomem;
	}
	hw = spi_master_get_devdata(master);
	memset(hw, 0, sizeof(struct acqfhba_spi));

	hw->master = spi_master_get(master);
	hw->master->bus_num = 0;
//	hw->pdata = pdev->dev.platform_data;
	hw->dev = dev;
	hw->tdev = tdev;

	hw->bitbang.master         = hw->master;
	hw->bitbang.setup_transfer = rtm_t_spi_setupxfer;
	hw->bitbang.chipselect     = rtm_t_spi_chipsel;
	hw->bitbang.txrx_bufs      = rtm_t_spi_txrx;
	hw->bitbang.master->setup  = rtm_t_spi_setup;

	/* register our spi controller */

	rc = spi_bitbang_start(&hw->bitbang);
	if (rc) {
		dev_err(dev, "Failed to register SPI master\n");
		goto err_register;
	}

	dbg(1, "registering %s\n", rtm_t_spi_devices[0].modalias);
	spi_new_device(master, &rtm_t_spi_devices[0]);

	return 0;

err_register:

err_nomem:
	return rc;
}

