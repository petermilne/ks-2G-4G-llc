/* ------------------------------------------------------------------------- */
/* rtm-t-sysfs.c RTM-T PCIe Host Side driver, sysfs (knobs)	             */
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


/** @file rtm-t-sysfs.c D-TACQ PCIe RTM_T driver, sysfs (knobs) */



#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
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

#include <linux/proc_fs.h>

#include <linux/seq_file.h>

#include <asm/uaccess.h>  /* VERIFY_READ|WRITE */

#include "lk-shim.h"

#include "rtm-t.h"
#include "rtm-t-hostdrv.h"
#include "rtm-t-dmac.h"
#include "rtm-t-dio.h"
#define acq200_debug rtm_t_debug
#include "acq200_debug.h"
#include "acq200.h"


int pulse_top_usec = 10;
module_param(pulse_top_usec, int, 0644);
MODULE_PARM_DESC(pulse_top_usec, "dio pulse {PN} duration");

extern int nbuffers;
extern struct list_head devices;

struct proc_dir_entry *rtm_t_proc_root;

static ssize_t show_job(
	struct device *dev, 
	struct device_attribute *attr,
	char *buf)
{	
	struct RTM_T_DEV *tdev = rtm_t_lookupDev(dev);
	int bstates[NSTATES] = { 0, };
	int ii;
	int data_rate = tdev->job.rx_rate*tdev->req_len;

	if (data_rate > 0x100000){
		data_rate /= 0x100000;
	}
	for (ii = 0; ii != nbuffers; ++ii){
		bstates[tdev->hb[ii].bstate]++;
	}


	return sprintf(buf, 
		"dev=%s idx=%d demand=%d queued=%d "
		"rx=%d rx_rate=%d int_rate=%d "
		"MBPS=%d "
		"BS_EMPTY=%-2d BS_FILLING=%-2d BS_FULL=%-2d BS_FULL_APP=%-2d " 
		"STATUS=%s ERRORS=%d\n",
		       tdev->name, tdev->idx,
		       tdev->job.buffers_demand,
		       tdev->job.buffers_queued,
		       tdev->job.buffers_received,
		       tdev->job.rx_rate,  tdev->job.int_rate,
		       data_rate,
		       bstates[0], bstates[1], bstates[2], bstates[3],
		       tdev->job.please_stop==PS_PLEASE_STOP? "PLEASE_STOP":
		       tdev->job.please_stop==PS_STOP_DONE? "STOP_DONE": "",
		       tdev->job.errors
		);
}

static DEVICE_ATTR(job, S_IRUGO, show_job, 0);


static ssize_t show_list_status(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct RTM_T_DEV *tdev = rtm_t_lookupDev(dev);
	int* fill_stash = kmalloc(nbuffers*sizeof(int), GFP_KERNEL);
	int len = 0;
	struct HostBuffer* hb;
	struct HostBuffer* tmp;
	int fill_count = 0;
	int filling_good = 0;
	int filling_bad = 0;
	int not_in_filling_q = 0;
	int ib;

	if (mutex_lock_interruptible(&tdev->list_mutex)){
		len = -1;
		goto cleanup;
	}

	list_for_each_entry_safe(hb, tmp, &tdev->bp_filling.list, list){
		fill_stash[fill_count++] = hb->ibuf;
	}
	mutex_unlock(&tdev->list_mutex);

	for (ib = 0; ib != nbuffers; ++ib){
		int ifound;
		int buf_in_stash = 0;
		for (ifound = 0; ifound < fill_count; ++ifound){
			if (fill_stash[ifound] == ib){
				buf_in_stash = 1;
				break;
			}
		}
		if (tdev->hb[ib].bstate == BS_FILLING){
			if (buf_in_stash){
				filling_good++;
			}else{
				err("buffer %02d marked BS_FILLING but not in filling q", ib);
				not_in_filling_q++;
			}
		}else if (buf_in_stash){
			filling_bad++;
			err("buffer %02d marked %d but found in filling Q", ib, tdev->hb[ib].bstate);
		}
	}

	len = sprintf(buf, "bp_filling len:%d good:%d absent:%d wrong q:%d\n",
			fill_count, filling_good, not_in_filling_q, filling_bad);

	cleanup:
	kfree(fill_stash);
	return len;
}

static DEVICE_ATTR(list_status, S_IRUGO, show_list_status, 0);

static ssize_t store_reset_buffers(
	struct device * dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	unsigned mode;

	if (sscanf(buf, "%u", &mode) > 0){
		if (mode == 1){
			rtm_t_reset_buffers(rtm_t_lookupDev(dev));	
		}
		return count;
	}else{
		return -EPERM;
	}
}

static DEVICE_ATTR(reset_buffers, S_IWUGO, 0, store_reset_buffers);

static inline void DIO_SET_OUTPUT1(struct RTM_T_DEV* tdev, int ib) 
{
	tdev->regs.dio |= (1 << (ib + RTMT_D_DIO_DRVON_SHL));
	tdev->regs.dio |= (1 << (ib + RTMT_D_DIO_SETOUT_SHL));
	tdev->regs.dio |= (1 << (ib + RTMT_D_DIO_OUTDAT_SHL));
}
static inline void DIO_SET_OUTPUT0(struct RTM_T_DEV* tdev, int ib) 
{
	tdev->regs.dio |= (1 << (ib + RTMT_D_DIO_DRVON_SHL));
	tdev->regs.dio |= (1 << (ib + RTMT_D_DIO_SETOUT_SHL));
	tdev->regs.dio &= ~(1 << (ib + RTMT_D_DIO_OUTDAT_SHL));
}
static inline void DIO_SET_INPUT_IOP(struct RTM_T_DEV* tdev, int ib)
{
	tdev->regs.dio |= (1 << (ib + RTMT_D_DIO_DRVON_SHL));
	tdev->regs.dio &= ~(1 << (ib+RTMT_D_DIO_SETOUT_SHL));
}
static inline void DIO_SET_INPUT_HD15(struct RTM_T_DEV* tdev, int ib)
{
	tdev->regs.dio &= ~(1 << (ib + RTMT_D_DIO_DRVON_SHL));
	tdev->regs.dio &= ~(1 << (ib+RTMT_D_DIO_SETOUT_SHL));
}
static inline int DIO_IS_OUTPUT(struct RTM_T_DEV* tdev, int ib)
{
	return (tdev->regs.dio & (1 << (ib+RTMT_D_DIO_SETOUT_SHL))) != 0;
}
static inline int DIO_IS_OUTPUT1(struct RTM_T_DEV* tdev, int ib)
{
	return DIO_IS_OUTPUT(tdev, ib) &&
		(tdev->regs.dio & (1 << (ib + RTMT_D_DIO_OUTDAT_SHL))) != 0;
}
static inline int DIO_IS_INPUTH(struct RTM_T_DEV* tdev, int ib)
{
	return (tdev->regs.dio_read & (1 << ib)) != 0;
}

static void set_outputs(struct RTM_T_DEV* tdev)
{
	rtd_write_reg(tdev, RTMT_H_DIO, tdev->regs.dio);
}

static void read_inputs(struct RTM_T_DEV* tdev)
{
	tdev->regs.dio_read = rtd_read_reg(tdev, RTMT_H_DIO);
}

static void do_store_dio_bit(struct RTM_T_DEV *tdev, int ibit, char code)
{
	switch(code){
	case DIO_MASK_OUTPUT1:
		DIO_SET_OUTPUT1(tdev, ibit);
		break;
	case DIO_MASK_OUTPUT0:
		DIO_SET_OUTPUT0(tdev, ibit);
		break;
	case DIO_MASK_OUTPUT_PP:
		DIO_SET_OUTPUT0(tdev, ibit); set_outputs(tdev);
		DIO_SET_OUTPUT1(tdev, ibit); set_outputs(tdev);
		udelay(pulse_top_usec);
		DIO_SET_OUTPUT0(tdev, ibit);
		break;
	case DIO_MASK_OUTPUT_NP:
		DIO_SET_OUTPUT1(tdev, ibit); set_outputs(tdev);
		DIO_SET_OUTPUT0(tdev, ibit); set_outputs(tdev);
		udelay(pulse_top_usec);
		DIO_SET_OUTPUT1(tdev, ibit);
		break;
	case DIO_MASK_INPUT_IOP:
		DIO_SET_INPUT_IOP(tdev, ibit);
		break;
	case DIO_MASK_INPUT:
	case DIO_MASK_INPUT_HD15:
		DIO_SET_INPUT_HD15(tdev, ibit);
		break;
	default:
		; /* do nothing 'x' by convention */
	}
}

static char do_show_dio_bit(struct RTM_T_DEV* tdev, int ibit)
{
	if (DIO_IS_OUTPUT(tdev, ibit)){
		return DIO_IS_OUTPUT1(tdev, ibit)? 
				DIO_MASK_OUTPUT1: DIO_MASK_OUTPUT0;
	}else{
		return DIO_IS_INPUTH(tdev, ibit)?
				DIO_MASK_INPUT1: DIO_MASK_INPUT0;
	} 
}

static ssize_t store_dio(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int ibit;
	struct RTM_T_DEV* tdev = rtm_t_lookupDev(dev);

	for (ibit = 0; ibit != MAXDIOBIT; ++ibit){
		do_store_dio_bit(tdev, ibit, buf[ibit]);
	}
	set_outputs(tdev);
        return strlen(buf);
}

static ssize_t show_dio(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	int ibit;
	struct RTM_T_DEV* tdev = rtm_t_lookupDev(dev);
	read_inputs(tdev);

	for (ibit = 0; ibit != MAXDIOBIT; ++ibit){
		buf[ibit] = do_show_dio_bit(tdev, ibit);
	}
	buf[ibit++] = '\n';
	buf[ibit] = '\0';
	return strlen(buf);
}

static DEVICE_ATTR(dio, S_IRUGO|S_IWUGO, show_dio, store_dio);

static ssize_t store_dio_bit(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int ibit;
	char value;
	struct RTM_T_DEV* tdev = rtm_t_lookupDev(dev);

	if (sscanf(buf, "%d %c", &ibit, &value) == 2 ||
            sscanf(buf, "%d=%c", &ibit, &value) == 2    ){
		if (ibit >= 0 && ibit < MAXDIOBIT){
			do_store_dio_bit(tdev, ibit, value);
			set_outputs(tdev);
			strncpy(tdev->last_dio_bit_store, buf,
				min(count, sizeof(tdev->last_dio_bit_store)));
		}
	}

        return strlen(buf);
}

static ssize_t show_dio_bit(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	struct RTM_T_DEV* tdev = rtm_t_lookupDev(dev);
	strcpy(buf, tdev->last_dio_bit_store);
	return strlen(tdev->last_dio_bit_store);
}


static DEVICE_ATTR(dio_bit, S_IRUGO|S_IWUGO, show_dio_bit, store_dio_bit);


static ssize_t store_dio_bitx(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count,
	int bitx)
{
	struct RTM_T_DEV* tdev = rtm_t_lookupDev(dev);
	do_store_dio_bit(tdev, bitx, buf[0]);
	set_outputs(tdev);
	return strlen(buf);
}

static ssize_t show_dio_bitx(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf,
	int bitx)
{
	struct RTM_T_DEV* tdev = rtm_t_lookupDev(dev);
	int ibit = 0;

	read_inputs(tdev);
	buf[ibit] = do_show_dio_bit(tdev, bitx);
	buf[++ibit] = '\n';
	buf[++ibit] = '\0';
	return strlen(buf);
}

#define DIO_BIT_KNOB(BX)						\
	static ssize_t show_dio_bit_##BX(				\
		struct device * dev,					\
		struct device_attribute *attr,				\
		char * buf)						\
	{								\
		return show_dio_bitx(dev, attr, buf, BX);		\
	}								\
	static ssize_t store_dio_bit_##BX(				\
		struct device * dev,					\
		struct device_attribute *attr,				\
		const char * buf,					\
		size_t count)						\
	{								\
		return store_dio_bitx(dev, attr, buf, count, BX);	\
	}								\
									\
	static DEVICE_ATTR(dio_bit_##BX, S_IRUGO|S_IWUGO,		\
			   show_dio_bit_##BX, store_dio_bit_##BX)	\



DIO_BIT_KNOB(0);
DIO_BIT_KNOB(1);
DIO_BIT_KNOB(2);
DIO_BIT_KNOB(3);
DIO_BIT_KNOB(4);
DIO_BIT_KNOB(5);
DIO_BIT_KNOB(6);



static ssize_t show_dio_raw(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	struct RTM_T_DEV* tdev = rtm_t_lookupDev(dev);
	read_inputs(tdev);
	memcpy(buf, &tdev->regs.dio_read, sizeof(u32));
	return sizeof(u32);
}

static ssize_t store_dio_raw(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, 
	size_t count)
{
	struct RTM_T_DEV* tdev = rtm_t_lookupDev(dev);
	memcpy(&tdev->regs.dio, buf, sizeof(u32));
	set_outputs(tdev);
	return strlen(buf);
}

static DEVICE_ATTR(dio_raw, S_IRUGO|S_IWUGO, show_dio_raw, store_dio_raw);


static ssize_t show_idx(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	struct RTM_T_DEV* tdev = rtm_t_lookupDev(dev);
	if (tdev){
		return sprintf(buf, "%d\n", tdev->idx);
	}else{
		return -ENODEV;
	}
}

static DEVICE_ATTR(idx, S_IRUGO, show_idx, 0);

static ssize_t show_cable_connected(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	struct RTM_T_DEV* tdev = rtm_t_lookupDev(dev);
	if (tdev){
		return sprintf(buf, "%d\n", cable_is_connected(tdev));
	}else{
		return -ENODEV;
	}
}

static DEVICE_ATTR(cable_connected, S_IRUGO, show_cable_connected, 0);
       
#define MBOX_KNOB(MB, reg, perms)					\
static ssize_t store_mbox##MB(						\
	struct device * dev,						\
	struct device_attribute *attr,					\
	const char * buf,						\
	size_t count)							\
{									\
	struct RTM_T_DEV* tdev = rtm_t_lookupDev(dev);			\
	u32 value = 0;							\
									\
	if (sscanf(buf, "0x%x", &value) || sscanf(buf, "%d", &value)){	\
		rtd_write_reg(tdev, reg, value);			\
	}								\
									\
        return strlen(buf);						\
}									\
									\
static ssize_t show_mbox##MB(						\
	struct device * dev,						\
	struct device_attribute *attr,					\
	char * buf)							\
{									\
	struct RTM_T_DEV* tdev = rtm_t_lookupDev(dev);			\
	u32 value = rtd_read_reg(tdev, reg);				\
	sprintf(buf, "0x%08x %d\n", value, value);			\
									\
	return strlen(buf);						\
}									\
									\
static DEVICE_ATTR(mbox##MB, (perms), show_mbox##MB, store_mbox##MB)

MBOX_KNOB(Q1, RTMT_Q_MBOX1, S_IRUGO);
MBOX_KNOB(Q2, RTMT_Q_MBOX2, S_IRUGO);
MBOX_KNOB(H1, RTMT_H_MBOX1, S_IRUGO|S_IWUGO);
MBOX_KNOB(H2, RTMT_H_MBOX2, S_IRUGO|S_IWUGO);



static ssize_t store_lowlat(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int ll_length;
	struct RTM_T_DEV* tdev = rtm_t_lookupDev(dev);

	if (sscanf(buf, "%d", &ll_length) == 1){
		u32 fcr = tdev->regs.H_FCR;
		if (ll_length != 0){
			int rem = ll_length % tdev->lldma_page_size;
			int pages = ll_length/tdev->lldma_page_size + (rem!=0? 1: 0);

			tdev->init_descriptors = init_descriptors_ll;
			tdev->lowlat_length = pages*tdev->lldma_page_size;
			fcr |=  RTMT_D_FCR_LOWLAT;
		}else{
			tdev->init_descriptors = init_descriptors_ht;
			tdev->lowlat_length = 0;
			fcr &= ~RTMT_D_FCR_LOWLAT;
		}
		if (fcr != tdev->regs.H_FCR){
			tdev->regs.H_FCR = fcr;
			rtd_write_reg(tdev, RTMT_H_FCR, tdev->regs.H_FCR);
		}

	}
        return strlen(buf);
}

static ssize_t show_lowlat(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct RTM_T_DEV* tdev = rtm_t_lookupDev(dev);

	sprintf(buf, "%d\n", tdev->lowlat_length);
	return strlen(buf);
}

static DEVICE_ATTR(lowlat, S_IRUGO|S_IWUGO, show_lowlat, store_lowlat);



static ssize_t store_buffer_len(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int ll_length;
	struct RTM_T_DEV* tdev = rtm_t_lookupDev(dev);

	if (sscanf(buf, "%d", &ll_length) == 1 && ll_length > 0){
		tdev->buffer_len = min(ll_length, buffer_len);
		return strlen(buf);

	}else{
		return -1;
	}
}

static ssize_t show_buffer_len(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct RTM_T_DEV* tdev = rtm_t_lookupDev(dev);

	sprintf(buf, "%d\n", tdev->buffer_len);
	return strlen(buf);
}

static DEVICE_ATTR(buffer_len, S_IRUGO|S_IWUGO, show_buffer_len, store_buffer_len);

static ssize_t store_iop_push(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int enable;
	struct RTM_T_DEV* tdev = rtm_t_lookupDev(dev);

	if (sscanf(buf, "%d", &enable)){
		tdev->iop_push = enable != 0;
		return strlen(buf);
	}else{
		return -1;
	}
}

static ssize_t show_iop_push(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct RTM_T_DEV* tdev = rtm_t_lookupDev(dev);

	sprintf(buf, "%d\n", tdev->iop_push);
	return strlen(buf);
}

static DEVICE_ATTR(iop_push, S_IRUGO|S_IWUGO, show_iop_push, store_iop_push);



static ssize_t store_single_recycle(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int enable;
	struct RTM_T_DEV* tdev = rtm_t_lookupDev(dev);

	if (sscanf(buf, "%d", &enable) == 1){
		u32 fcr = tdev->regs.H_FCR;
		if (enable){
			fcr |=  RTMT_D_FCR_DMA_SINGLE_RECYCLE;
		}else{
			fcr &= ~RTMT_D_FCR_DMA_SINGLE_RECYCLE;
		}
		if (fcr != tdev->regs.H_FCR){
			tdev->regs.H_FCR = fcr;
			rtd_write_reg(tdev, RTMT_H_FCR, tdev->regs.H_FCR);
		}
	}
        return strlen(buf);
}

static ssize_t show_single_recycle(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct RTM_T_DEV* tdev = rtm_t_lookupDev(dev);

	sprintf(buf, "%d\n",
		(tdev->regs.H_FCR&RTMT_D_FCR_DMA_SINGLE_RECYCLE)? 1: 0);
	return strlen(buf);
}

static DEVICE_ATTR(single_recycle, S_IRUGO|S_IWUGO,
		show_single_recycle, store_single_recycle);

static ssize_t show_acqtype(struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct RTM_T_DEV* tdev = rtm_t_lookupDev(dev);

	sprintf(buf, "%s\n", tdev->is_ao32? "AO32": "ACQ196");
	return strlen(buf);
}

static DEVICE_ATTR(acqtype, S_IRUGO, show_acqtype, 0);


static ssize_t show_reg(struct device * dev,
		struct device_attribute *attr,
		char * buf, unsigned reg)
{
	struct RTM_T_DEV* tdev = rtm_t_lookupDev(dev);
	u32 value = rtd_read_reg(tdev, reg);
	return sprintf(buf, "0x%08x\n", value);
}

#define REG_QUERY(knob, REG)							\
static ssize_t show_##REG(						\
	struct device * dev,						\
	struct device_attribute *attr,					\
	char * buf)							\
{									\
	return show_reg(dev, attr, buf, REG);				\
}									\
									\
static DEVICE_ATTR(reg_##knob, S_IRUGO, show_##REG, 0)

REG_QUERY(REVID, 		RTMT_C_REVID);
REG_QUERY(PCI_CSR, 		RTMT_C_PCI_CSR);
REG_QUERY(PCIE_DEV_CSR, 	RTMT_C_PCIE_DEV_CSR);
REG_QUERY(PCIE_LNK_CSR,		RTMT_C_PCIE_LNK_CSR);
REG_QUERY(PCIE_ERR_CNT,		RTMT_C_PCIE_ERR_CNT);
REG_QUERY(PCIE_LAST_ERR, 	RTMT_C_PCIE_LAST_ERR);
REG_QUERY(PCIE_COMPLETION,	RTMT_C_PCIE_COMPLETION);

REG_QUERY(IB_DMA_STAT, 		RTMT_H_IB_DMA_STAT);
REG_QUERY(IB_DMA_FIFSTA, 	RTMT_H_IB_DMA_FIFSTA);
REG_QUERY(O1_DMA_STAT, 		RTMT_H_O1_DMA_STAT);
REG_QUERY(O1_DMA_FIFSTA, 	RTMT_H_O1_DMA_FIFSTA);
REG_QUERY(O2_DMA_STAT, 		RTMT_H_O2_DMA_STAT);
REG_QUERY(O2_DMA_FIFSTA, 	RTMT_H_O2_DMA_FIFSTA);

void create_query_knobs(struct device *dev)
{
	DEVICE_CREATE_FILE(dev, &dev_attr_reg_REVID);
	DEVICE_CREATE_FILE(dev, &dev_attr_reg_PCI_CSR);
	DEVICE_CREATE_FILE(dev, &dev_attr_reg_PCIE_DEV_CSR);
	DEVICE_CREATE_FILE(dev, &dev_attr_reg_PCIE_LNK_CSR);
	DEVICE_CREATE_FILE(dev, &dev_attr_reg_PCIE_ERR_CNT);
	DEVICE_CREATE_FILE(dev, &dev_attr_reg_PCIE_LAST_ERR);
	DEVICE_CREATE_FILE(dev, &dev_attr_reg_PCIE_COMPLETION);

	DEVICE_CREATE_FILE(dev, &dev_attr_reg_IB_DMA_STAT);
	DEVICE_CREATE_FILE(dev, &dev_attr_reg_IB_DMA_FIFSTA);
	DEVICE_CREATE_FILE(dev, &dev_attr_reg_O1_DMA_STAT);
	DEVICE_CREATE_FILE(dev, &dev_attr_reg_O1_DMA_FIFSTA);
	DEVICE_CREATE_FILE(dev, &dev_attr_reg_O2_DMA_STAT);
	DEVICE_CREATE_FILE(dev, &dev_attr_reg_O2_DMA_FIFSTA);
}


static ssize_t show_fifo(struct device * dev,
	struct device_attribute *attr,
	char * buf, unsigned reg)
{
	struct RTM_T_DEV* tdev = rtm_t_lookupDev(dev);
	u32 stat = rtd_read_reg(tdev, reg);
	u32 fifsta = rtd_read_reg(tdev, reg+4);

	return sprintf(buf,
			"fifo: 0x%08x %s last:0x%08x 0x%08x %s len:%d id:%x\n",
			fifsta,
			fifsta&RTMT_H_XX_DMA_FIFSTA_FULL?  "FULL ":
			fifsta&RTMT_H_XX_DMA_FIFSTA_EMPTY? "EMPTY":
					                   "     ",
			stat,
			stat&RTDMAC_DESC_ADDR_MASK,
			stat&RTDMAC_DESC_EOT? "EOT": "   ",
			(stat&RTDMAC_DESC_LEN_MASK)>>RTDMAC_DESC_LEN_SHL,
			stat&RTDMAC_DESC_ID_MASK);
}

#define FIFO_QUERY(the_fifo, REG)					\
static ssize_t show_##the_fifo(						\
	struct device * dev,						\
	struct device_attribute *attr,					\
	char * buf)							\
{									\
	return show_fifo(dev, attr, buf, REG);				\
}									\
									\
static DEVICE_ATTR(the_fifo##_FIFO, S_IRUGO, show_##the_fifo, 0)

FIFO_QUERY(INPUT_DMA, RTMT_H_IB_DMA_STAT);
FIFO_QUERY(OUTPUT_DMA, RTMT_H_O1_DMA_STAT);


void create_sfp_knobs(struct device *dev);

void rtm_t_createSysfs(struct RTM_T_DEV *tdev)
{
	struct device *dev = &tdev->pci_dev->dev;
	DEVICE_CREATE_FILE(dev, &dev_attr_buffer_len);
	DEVICE_CREATE_FILE(dev, &dev_attr_iop_push);
	DEVICE_CREATE_FILE(dev, &dev_attr_idx);
	DEVICE_CREATE_FILE(dev, &dev_attr_cable_connected);
	DEVICE_CREATE_FILE(dev, &dev_attr_job);
	DEVICE_CREATE_FILE(dev, &dev_attr_list_status);
	DEVICE_CREATE_FILE(dev, &dev_attr_reset_buffers);
	DEVICE_CREATE_FILE(dev, &dev_attr_dio);
	DEVICE_CREATE_FILE(dev, &dev_attr_dio_bit);
	DEVICE_CREATE_FILE(dev, &dev_attr_dio_raw);

	DEVICE_CREATE_FILE(dev, &dev_attr_dio_bit_0);
	DEVICE_CREATE_FILE(dev, &dev_attr_dio_bit_1);
	DEVICE_CREATE_FILE(dev, &dev_attr_dio_bit_2);
	DEVICE_CREATE_FILE(dev, &dev_attr_dio_bit_3);
	DEVICE_CREATE_FILE(dev, &dev_attr_dio_bit_4);
	DEVICE_CREATE_FILE(dev, &dev_attr_dio_bit_5);
	DEVICE_CREATE_FILE(dev, &dev_attr_dio_bit_6);

	DEVICE_CREATE_FILE(dev, &dev_attr_mboxQ1);
	DEVICE_CREATE_FILE(dev, &dev_attr_mboxQ2);
	DEVICE_CREATE_FILE(dev, &dev_attr_mboxH1);
	DEVICE_CREATE_FILE(dev, &dev_attr_mboxH2);

	DEVICE_CREATE_FILE(dev, &dev_attr_lowlat);
	DEVICE_CREATE_FILE(dev, &dev_attr_single_recycle);
	DEVICE_CREATE_FILE(dev, &dev_attr_acqtype);

	DEVICE_CREATE_FILE(dev, &dev_attr_INPUT_DMA_FIFO);
	DEVICE_CREATE_FILE(dev, &dev_attr_OUTPUT_DMA_FIFO);
	create_query_knobs(dev);
	create_sfp_knobs(dev);
}


static ssize_t show_dev(
	struct CLASS_DEVICE * dev,
#ifndef ORIGINAL_CLASS_DEVICE_INTERFACE
	struct device_attribute *attr, 
#endif 
	char * buf)
{
	struct RTM_T_DEV* tdev = rtm_t_lookupDeviceFromClass(dev);
	if (tdev){
		return sprintf(buf, "%d:0\n", tdev->major);
	}else{
		return -ENODEV;
	}
}
static CLASS_DEVICE_ATTR(dev, S_IRUGO, show_dev, 0);



#ifdef ORIGINAL_CLASS_DEVICE_INTERFACE
void rtm_t_class_device_create_file(
        struct class_device * dev, struct class_device_attribute * attr,
        const char *file, int line)
{
        if (class_device_create_file(dev, attr)){
                err("%s:%d device_create_file", file, line);
        }
}

#define CLASS_DEVICE_CREATE_FILE(dev, attr) \
        rtm_t_class_device_create_file(dev, attr, __FILE__, __LINE__)

void rtm_t_create_sysfs_class(struct RTM_T_DEV *tdev)
{
        dbg(1, "01");
        CLASS_DEVICE_CREATE_FILE(tdev->class_dev, &class_device_attr_dev);
        dbg(1, "9");
}

void rtm_t_remove_sysfs_class(struct RTM_T_DEV *tdev)
{
        class_device_remove_file(tdev->class_dev, &class_device_attr_dev);
}


#else
void rtm_t_create_sysfs_class(struct RTM_T_DEV *tdev)
{
	int rc;
	dbg(1, "01");
	DEVICE_CREATE_FILE(tdev->class_dev, &dev_attr_dev);

	rc = sysfs_create_link(
		&tdev->class_dev->kobj, &tdev->pci_dev->dev.kobj, "device");
	if (rc) {
		err("failed to create symlink %s\n", "device");
	}	
	dbg(1, "9");
}

void rtm_t_remove_sysfs_class(struct RTM_T_DEV *tdev)
{
	device_remove_file(tdev->class_dev, &dev_attr_dev);
}

#endif

static void *hb_seq_start(struct seq_file *sfile, loff_t *pos)
/* *pos == iblock. returns next TBLOCK* */
{
	if (*pos >= nbuffers){
		return NULL;
	}else{
		return pos;
	}
}

static void *hb_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	(*pos)++;
	if (*pos >= nbuffers){
		return NULL;
	}else{
		return pos;
	}
}



static void hb_seq_stop(struct seq_file *s, void *v) {}


static inline const char *BSTATE2S(enum BSTATE bstate)
{
	switch(bstate){
	case BS_EMPTY:		return "BS_EMPTY";
	case BS_FILLING:	return "BS_FILLING";
	case BS_FULL:		return "BS_FULL";
	case BS_FULL_APP:	return "BS_FULL_APP";
	default:		return "BS_ERROR";
	}
}


static int hb_seq_show(struct seq_file *sfile, void *v)
{
	struct file *file = (struct file *)sfile->private;
	struct RTM_T_DEV *tdev = PDE_DATA(file_inode(file));
	int ib = *(int*)v;
	struct HostBuffer *hb = tdev->hb+ib;
	int len = 0;

	if (ib == 0){
		len = seq_printf(sfile, "ix va pa len req_len descr state\n");
	}
	len += seq_printf(sfile,
			 "[%02d] %p %08x %06x %06x %08x %s\n",
			 ib, hb->va, hb->pa, hb->len, hb->req_len,
			 hb->descr, BSTATE2S(hb->bstate));
	return len;
}

static int hbd_seq_show(struct seq_file *sfile, void *v)
{
	struct file *file = (struct file *)sfile->private;
	struct RTM_T_DEV *tdev = PDE_DATA(file_inode(file));
	int ib = *(int*)v;
	struct HostBuffer *hb = tdev->hb+ib;
	int len = 0;

	len += seq_printf(sfile, "%08x\n", hb->descr);
	return len;
}




static int ab_seq_show(struct seq_file *sfile, void *v)
/* shows buffers that are FULL but NOT in FULL LIST */
{
	struct file *file = (struct file *)sfile->private;
	struct RTM_T_DEV *tdev = PDE_DATA(file_inode(file));
	int ib = *(int*)v;
	struct HostBuffer *hb = tdev->hb+ib;
	int len = 0;

	if (ib == 0){
		len = seq_printf(sfile, "ix va pa len req_len descr state\n");
	}
	if (hb->bstate == BS_FULL_APP){
		len += seq_printf(sfile,
				 "[%02d] %p %08x %06x %06x %08x %s\n",
				 ib, hb->va, hb->pa, hb->len, hb->req_len,
				 hb->descr, BSTATE2S(hb->bstate));
	}
	return len;
}


static int __proc_open(
	struct inode *inode, struct file *file,
	struct seq_operations *_seq_ops)
{
	int rc = seq_open(file, _seq_ops);

	if (rc == 0){
		struct seq_file* seq_file =
			(struct seq_file*)file->private_data;
		seq_file->private = file;
	}

	return rc;
}

static int hb_proc_open(struct inode *inode, struct file *file)
{
	static struct seq_operations _seq_ops = {
		.start = hb_seq_start,
		.next  = hb_seq_next,
		.stop  = hb_seq_stop,
		.show  = hb_seq_show
	};
	return __proc_open(inode, file, &_seq_ops);
}
static int hbd_proc_open(struct inode *inode, struct file *file)
{
	static struct seq_operations _seq_ops = {
		.start = hb_seq_start,
		.next  = hb_seq_next,
		.stop  = hb_seq_stop,
		.show  = hbd_seq_show
	};
	return __proc_open(inode, file, &_seq_ops);
}

static int addHostBufferProcFiles(struct RTM_T_DEV *tdev)
{
	static struct file_operations hb_proc_fops = {
		.owner = THIS_MODULE,
		.open = hb_proc_open,
		.read = seq_read,
		.llseek = seq_lseek,
		.release = seq_release
	};
	static struct file_operations hbd_proc_fops = {
		.owner = THIS_MODULE,
		.open = hbd_proc_open,
		.read = seq_read,
		.llseek = seq_lseek,
		.release = seq_release
	};

	struct proc_dir_entry *hb_entry =
			proc_create_data("HostBuffers", S_IRUGO, tdev->proc_dir_root, &hb_proc_fops, tdev);

	if (hb_entry){
		hb_entry = proc_create_data("HostDescriptors", S_IRUGO, tdev->proc_dir_root, &hbd_proc_fops, tdev);
		if (hb_entry){
			return 0;
		}
	}

	err("Failed to create entry");
	return -1;
}

static int ab_proc_open(struct inode *inode, struct file *file)
{
	static struct seq_operations _seq_ops = {
		.start = hb_seq_start,
		.next  = hb_seq_next,
		.stop  = hb_seq_stop,
		.show  = ab_seq_show
	};
	return __proc_open(inode, file, &_seq_ops);
}

static int addAppBufferProcFiles(struct RTM_T_DEV *tdev)
{
	static struct file_operations ab_proc_fops = {
		.owner = THIS_MODULE,
		.open = ab_proc_open,
		.read = seq_read,
		.llseek = seq_lseek,
		.release = seq_release
	};
	struct proc_dir_entry *ab_entry =
		proc_create_data("AppBuffers", S_IRUGO, tdev->proc_dir_root, &ab_proc_fops, tdev);
	if (ab_entry){
		return 0;
	}

	err("Failed to create entry");
	return -1;
}


int initProcFs(struct RTM_T_DEV *tdev)
{
	int rc;

	if (!rtm_t_proc_root){
		rtm_t_proc_root = proc_mkdir("driver/rtm-t", NULL);
		assert(rtm_t_proc_root);
	}

	tdev->proc_dir_root = proc_mkdir(tdev->name, rtm_t_proc_root);

	if ((rc = addHostBufferProcFiles(tdev)) == 0 &&
	    (rc = addAppBufferProcFiles(tdev))  == 0)
		return 0;

	return rc;
}

