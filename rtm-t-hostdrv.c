/* ------------------------------------------------------------------------- */
/* rtm-t-hostdrv.c RTM-T PCIe Host Side driver			             */
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


/** @file rtm-t-hostdrv.c D-TACQ PCIe RTM_T driver */

/** <!-- README: -->
 
@mainpage "D-TACQ RTM_T Driver"
@author Peter Milne <peter.milne@d-tacq.com>
@section Overview
RTM-T host driver. Works on Linux 2.6.18 to 2.6.35, i686 and x86_64
Compatible with
- RTM-T : PCIe cable extension via transparent bridge
- ACQ-FIBER-HBA : D-TACQ dual-port fiber extension.

@section Installation

- make
- sudo make install

- The driver is automatically loaded by modprobe(8) and
- device nodes are created by udev(7).
- RTM-T includes a 16550 uart pair to allow communication with the ACQ196,
the udev hotplug brings up a CSLIP link, host and ACQ then gain a backplane
network capability. 
- Slot name convention: slot 100 up
- The acqcmd idiom is supported through the SLIP network. A toplevel acqcmd script routes requests automatically to:
 - slot 1-10 : physical backplane slots using kernel device driver
 - slot 9-99 : "dt100-hub" emulated slots using kernel device driver
 - slot 100+ : direct connect using IP
- A modified version of acqcmd communicates with the ACQ196 card in the normal
way, while the usual ssh, http and ftp facilities are available. Note the the
CSLIP link runs at 1Mbit/sec - it's suitable for control and firmware upgrade,
while the main path for acquisition data is the PCIe channel itself. The PCIe link has sufficient bandwidth to support concurrent ip channels for control and monitoring as well as data at 100MB/s. The SLIP addressing convention is:
 - Slot 100: Host ip: 10.0.196.200, Target ip: 10.0.196.100
 - Slot 101: Host ip: 10.0.196.201, Target ip: 10.0.196.101 
etc.
 - Clearly other addresses could be chosen if these are not compatible with the site network. It's possible to set up remote access to the embedded cards either by standard IP routing, or perhaps better by SSH port-forwarding. Full public access is not recommended, since available network bandwidth is limited.
- ACQ-FIBER-HBA:
 - operation on fiber is identical to copper.
 - Supports flash firmware update on acq-fiber-hba. Requires special build, see
README for details. RTM-T firmware is upgraded from the IOP.
 - Supports SFP condition monitoring - see test-scripts/avago-monitor

@section Defects
- CSLIP link bring-up on hotplug is unreliable on FC14/x86_64
 - workaround: run rtm-t-connect 0 on first log-in. This is set as a SYSV init script. This may be disabled where not required.

@section Worktodo
- fiber low latency [single sample] operation is not supported.
- second fiber port is not supported.

@section Example Applications
- rtm-t-stream-disk.cpp : high throughput streaming to disk.
- rtm-t-llcontrol.cpp : low latency control example.
*/

//#define UART_TAKES_IRQ

#define __RTM_T_HOSTDRV_C__

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
#define acq200_debug rtm_t_debug
#include "acq200_debug.h"
#include "acq200.h"

#include "ao32-drv.h"

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,30)
#define L0S_HACK
#warning L0S_HACK enabled
#include <linux/pci-aspm.h>
#endif





static int MAP2BAR(struct RTM_T_DEV *tdev, int imap)
{
	switch(imap){
	case UART_BAR:
	case FIFO_BAR:
	case REGS_BAR:
		return imap;
	case PBI_BAR:
		if (tdev->is_ao32){
			return imap;
		}else{
			return NO_BAR;
		}
	default:
		return NO_BAR;
	}
}

static int minor2bar(int iminor)
{
	switch(iminor){
	default:
		err("bad call minor2bar %d", iminor); /* fallthru */
	case MINOR_REGREAD:
		return REGS_BAR;
	case MINOR_UART:
		return UART_BAR;
	case MINOR_PBI:
		return PBI_BAR;
	case MINOR_FIFO:
		return FIFO_BAR;
	}

}
#define VALID_BAR(bar)	((bar) != NO_BAR)

char rtm_t_driver_name[] = "rtm_t";
char rtm_t__driver_string[] = "D-TACQ RTM-T Driver";
char rtm_t__driver_version[] = "B1214";
char rtm_t__copyright[] = "Copyright (c) 2010/2014 D-TACQ Solutions Ltd";

char *version = rtm_t__driver_version;
module_param(version, charp, 0444);

int rtm_t_debug = 0;
module_param(rtm_t_debug, int, 0644);

int rtm_t_debug_isr = 0;
module_param(rtm_t_debug_isr, int, 0644);
MODULE_PARM_DESC(rtm_t_debug_isr, "set to 3 to debug ISR");

#define D_ISR	(4-rtm_t_debug_isr)

int rtm_t_debug_descr = 0;
module_param(rtm_t_debug_descr, int, 0644);
MODULE_PARM_DESC(rtm_t_debug_descr, "set to 3 to debug descr");

#define D_DESCR (4-rtm_t_debug_descr)

int nbuffers = NBUFFERS;
module_param(nbuffers, int, 0444);
MODULE_PARM_DESC(nbuffers, "number of host-side buffers");

int buffer_len = BUFFER_LEN;
module_param(buffer_len, int, 0644);
MODULE_PARM_DESC(buffer_len, "length of each buffer in bytes");


int transfer_buffers = 0x7fffffff;
module_param(transfer_buffers, int, 0664);
MODULE_PARM_DESC(transfer_buffers, "number of buffers to transfer");

int stalls = 0;
module_param(stalls, int, 0644);
MODULE_PARM_DESC(stalls, "number of times ISR ran with no buffers to queue");

int RX_TO = 10*HZ;
module_param(RX_TO, int, 0644);
MODULE_PARM_DESC(RX_TO, "RX timeout (jiffies) [0.1Hz]");

int WORK_TO = HZ/10;
module_param(WORK_TO, int, 0644);
MODULE_PARM_DESC(WORK_TO, 
	"WORK timeout (jiffies) [10Hz] - decrease for hi fifo stat poll rate");

int SMOO = 7;
module_param(SMOO, int, 0644);
MODULE_PARM_DESC(SMOO, "rate smoothing factor 0..9 none..smooth");

int test_mode = 0;
module_param(test_mode, int, 0644);
MODULE_PARM_DESC(test_mode, "0: ADC, 1..15 : TEST, 1=/1 (fastest)");

int iop_push = 0;
module_param(iop_push, int, 0644);
MODULE_PARM_DESC(iop_push, "1: DATA pushed by IOP 0:RMT-T pulls from S3 FIFO");

int mbox_poll_ticks = 10;
module_param(mbox_poll_ticks, int, 0644);
MODULE_PARM_DESC(mbox_poll_ticks, "LLC mbox poll rate in jiffies 0:continuous");

struct class* rtm_t_device_class;

int llc_disable_interrupts = 0;
module_param(llc_disable_interrupts, int, 0644);

int llc_imask;
module_param(llc_imask, int, 0644);

int stop_on_skipped_buffer = 0;
module_param(stop_on_skipped_buffer, int, 0644);

int ignore_bit_test;
module_param(ignore_bit_test, int, 0444);

static int queue_next_free_buffer(struct RTM_T_DEV* tdev);

#define HB_ENTRY(plist)	list_entry(plist, struct HostBuffer, list)

LIST_HEAD(devices);

#define TDEV_ENTRY(plist) list_entry(plist, struct RTM_T_DEV, list)



struct RTM_T_DEV_PATH {
	int minor;
	struct RTM_T_DEV *dev;
	struct list_head my_buffers;
	void* private;
};

#define PSZ	  (sizeof (struct RTM_T_DEV_PATH))
#define PD(file)  ((struct RTM_T_DEV_PATH *)(file)->private_data)
#define DEV(file) (PD(file)->dev)

#if 0
#ifndef TASK_INTERRUPTIBLE
#define TASK_INTERRUPTIBLE 0	/* hack for kernel 2.6.33+ */
#endif
#endif

#define DEBUG	1

#if DEBUG > 0
static void debug_bars(struct pci_dev *dev)
{
	int bar;

	for (bar = 0; bar <= 5; ++bar){
		dbg(1, "bar:%d start:%08x end:%08x flags %x",
		     bar,
		     (unsigned)pci_resource_start(dev, bar),
		     (unsigned)pci_resource_end(dev, bar),
		     (unsigned)pci_resource_flags(dev, bar));
	}
}
#endif /* DEBUG > 0 */

#define COPY_FROM_USER(src, dest, len) \
	if (copy_from_user(src, dest, len)) { return -EFAULT; }


static void _write_descr(struct RTM_T_DEV *tdev, unsigned offset, u32 descr)
{
	dbg(D_DESCR, "offset:%04x %p = %08x", offset,
			tdev->mappings[FIFO_BAR].va+offset, descr);
	writel(descr, tdev->mappings[FIFO_BAR].va+offset);
}

static void rtd_write_descr(struct RTM_T_DEV *tdev, u32 descr)
{
	_write_descr(tdev, FIFO_OFFSET_INBOUND, descr);
}

static void rtd_load_descriptor(struct RTM_T_DEV *tdev, int idesc)
{
	dbg(D_DESCR-1, "ibuf %d", idesc);
/* change descr status .. */
	rtd_write_descr(tdev, tdev->hb[idesc].descr);
}

static void rtd_write_outbound1_descriptor(struct RTM_T_DEV *tdev, u32 descr)
{
	dbg(1, "descr:0x%08x", descr);
	_write_descr(tdev, FIFO_OFFSET_OUTBOUND1, descr);
}
#if 0
static void rtd_write_outbound2_descriptor(struct RTM_T_DEV *tdev, u32 descr)
{
	dbg(1, "descr:0x%08x", descr);
	_write_descr(tdev, FIFO_OFFSET_OUTBOUND2, descr);
}
#endif
static void rtd_start_dma(struct RTM_T_DEV *tdev)
{
	tdev->regs.H_FCR |= RTMT_D_FCR_WANTIT;

	rtd_write_reg(tdev, RTMT_H_FCR,
			tdev->regs.H_FCR |= RTMT_D_FCR_FIFO_RESET);
	udelay(10);	/** for ACQ-FIBER-HBA */
	rtd_write_reg(tdev, RTMT_H_FCR,
			tdev->regs.H_FCR &= ~RTMT_D_FCR_FIFO_RESET);

	if (!tdev->iop_push){
		rtd_write_reg(tdev, RTMT_H_FCR,
			tdev->regs.H_FCR |= RTMT_D_FCR_PCIE_DATAON);
	}
	rtd_write_reg(tdev, RTMT_H_DMAC_CTRL,
			tdev->regs.ctrl |= RTMT_H_DMAC_CTRL_IB_EN);
}

static void rtd_start_aodma(struct RTM_T_DEV *tdev)
{
	tdev->regs.H_FCR |= RTMT_D_FCR_WANTIT;
#if 0
	rtd_write_reg(tdev, RTMT_H_FCR,
			tdev->regs.H_FCR |= RTMT_D_FCR_FIFO_RESET);
	udelay(10);	/** for ACQ-FIBER-HBA */
	rtd_write_reg(tdev, RTMT_H_FCR,
			tdev->regs.H_FCR &= ~RTMT_D_FCR_FIFO_RESET);
#endif
	if (!tdev->iop_push){
		rtd_write_reg(tdev, RTMT_H_FCR,
			tdev->regs.H_FCR |= RTMT_D_FCR_PCIE_DATAON);
	}
	rtd_write_reg(tdev, RTMT_H_DMAC_CTRL,
			tdev->regs.ctrl |= RTMT_H_DMAC_CTRL_OB1_EN);
}

static int rtd_dma_started(struct RTM_T_DEV *tdev)
{
	return (tdev->regs.ctrl&RTMT_H_DMAC_CTRL_IB_EN) != 0;
}

static void rtd_stop_dma(struct RTM_T_DEV *tdev)
{
	rtd_write_reg(tdev, RTMT_H_FCR,
		      tdev->regs.H_FCR &= ~RTMT_D_FCR_PCIE_DATAON);
			       
	rtd_write_reg(tdev, RTMT_H_DMAC_CTRL,
		tdev->regs.ctrl &= ~RTMT_H_DMAC_CTRL_IB_EN);
}


static void rtd_dma_reset(struct RTM_T_DEV *tdev)
{
	dbg(1, "RESET");
	rtd_write_reg(tdev, RTDMAC_RESET, RTDMAC_RESET_RBIT);
	rtd_stop_dma(tdev);
	rtd_write_reg(tdev, RTMT_H_DMAC_CTRL, 0);
	rtd_write_reg(tdev, RTDMAC_RESET, 0);		
}


static int getOrder(int len)
{
	int order;
	len /= PAGE_SIZE;

	for (order = 0; 1 << order < len; ++order){
		;
	}
	return order;
}

static int getRTDMAC_Order(int len)
{
	int order;
	len /= RTDMAC_PAGE;

	for (order = 0; 1 << order < len; ++order){
		;
	}
	return order;
}

void init_descriptors_ht(struct RTM_T_DEV *tdev)
{
	int ii;

	for (ii = 0; ii < nbuffers; ++ii){
		u32 descr = tdev->hb[ii].descr;

		descr &= ~RTDMAC_DESC_LEN_MASK;
		descr |= getRTDMAC_Order(tdev->buffer_len)<< RTDMAC_DESC_LEN_SHL;
		descr |= RTDMAC_DESC_EOT;
		tdev->hb[ii].descr = descr;
	}
}


u32 make_single_descriptor_ll(struct RTM_T_DEV *tdev, u32 pa)
{
	int pages = tdev->lowlat_length / tdev->lldma_page_size;
	u32 descr = pa&RTDMAC_DESC_ADDR_MASK;

	descr |= (pages-1) << RTDMAC_DESC_LEN_SHL;
	descr |= RTDMAC_DESC_ID_MASK & 0x1;

	// !RTDMAC_DESC_WRITE
	// !RTDMAC_DESC_EOT

	dbg(1, "descr=%08x pa=%08x pages=%d", descr, pa, pages);
	return descr;
}
void init_descriptors_ll(struct RTM_T_DEV *tdev)
{
	int ii;
	int pages = tdev->lowlat_length / tdev->lldma_page_size;

	assert(pages >= 1);

	for (ii = 0; ii < nbuffers; ++ii){
		u32 descr = tdev->hb[ii].descr;

		descr &= ~RTDMAC_DESC_LEN_MASK;
		descr |=  (pages-1) << RTDMAC_DESC_LEN_SHL;
		tdev->hb[ii].descr = descr;
	}
}


int rtm_t_reset_buffers(struct RTM_T_DEV* tdev)
/* handle with care! */
{
	struct HostBuffer *hb = tdev->hb;
	int ii;

	if (mutex_lock_interruptible(&tdev->list_mutex)){
		return -1;
	}
        INIT_LIST_HEAD(&tdev->bp_empties.list);
	INIT_LIST_HEAD(&tdev->bp_filling.list);
	INIT_LIST_HEAD(&tdev->bp_full.list);

	

	for (ii = 0; ii < nbuffers; ++ii, ++tdev->nbuffers, ++hb){
		hb->bstate = BS_EMPTY;
		list_add_tail(&hb->list, &tdev->bp_empties.list); 
	}

	tdev->init_descriptors(tdev);
	mutex_unlock(&tdev->list_mutex);

	memset(tdev->data_fifo_histo, 0, DATA_FIFO_SZ);
	memset(tdev->desc_fifo_histo, 0, DESC_FIFO_SZ);

	return 0;
}

static void init_histo_buffers(struct RTM_T_DEV* tdev)
{
	int ii;

	tdev->data_fifo_histo = kzalloc(DATA_FIFO_SZ, GFP_KERNEL);
	tdev->desc_fifo_histo =	kzalloc(DESC_FIFO_SZ, GFP_KERNEL);

	/* give it a test pattern .. */

	for (ii = 0; ii != RTDMAC_DATA_FIFO_CNT; ++ii){
		tdev->data_fifo_histo[ii] = 0x70000000 + ii;
	}
	for (ii = 0; ii != RTDMAC_DESC_FIFO_CNT; ++ii){
		tdev->desc_fifo_histo[ii] = 0x50000000 + ii;
	}
}
static void init_buffers(struct RTM_T_DEV* tdev)
{
	int ii;
	int order = getOrder(BUFFER_LEN);
	struct HostBuffer *hb = tdev->hb;

        INIT_LIST_HEAD(&tdev->bp_empties.list);
	INIT_LIST_HEAD(&tdev->bp_filling.list);
	INIT_LIST_HEAD(&tdev->bp_full.list);
	spin_lock_init(&tdev->job_lock);

	mutex_init(&tdev->list_mutex);
	mutex_lock(&tdev->list_mutex);
	
	dbg(1, "allocating %d buffers size:%d dev.dma_mask:%08llx",
			nbuffers, BUFFER_LEN, *tdev->pci_dev->dev.dma_mask);

	for (ii = 0; ii < nbuffers; ++ii, ++tdev->nbuffers, ++hb){
		void *buf = (void*)__get_free_pages(GFP_KERNEL|GFP_DMA32, order);

		if (!buf){
			err("failed to allocate buffer %d", ii);
			break;
		}

		dbg(3, "buffer %2d allocated at %p, map it", ii, buf);

		hb->ibuf = ii;
		hb->pa = dma_map_single(&tdev->pci_dev->dev, buf, 
				BUFFER_LEN, PCI_DMA_FROMDEVICE);
		hb->va = buf;
		hb->len = BUFFER_LEN;
		
		dbg(3, "buffer %2d allocated, map done", ii);

		if ((hb->pa & (RTDMAC_PAGE-1)) != 0){
			err("HB NOT PAGE ALIGNED");
			BUG();
		}
		
		hb->descr = hb->pa | 0 | RTDMAC_DESC_EOT | (ii&RTDMAC_DESC_ID_MASK);
		hb->bstate = BS_EMPTY;

		dbg(3, "[%d] %p %08x %d %08x",
		    ii, hb->va, hb->pa, hb->len, hb->descr);
		list_add_tail(&hb->list, &tdev->bp_empties.list); 
	}
	tdev->init_descriptors = init_descriptors_ht;
	tdev->init_descriptors(tdev);
	init_waitqueue_head(&tdev->work.w_waitq);
	init_waitqueue_head(&tdev->return_waitq);

	mutex_unlock(&tdev->list_mutex);

	init_histo_buffers(tdev);
}




static int registerDevice(struct RTM_T_DEV *tdev)
{
	dbg(2, "name %s", tdev->name);
	list_add_tail(&tdev->list, &devices);	
	return initProcFs(tdev);
}

static void deleteDevice(struct RTM_T_DEV *tdev)
{
	list_del(&tdev->list);
	kfree(tdev->data_fifo_histo);
	kfree(tdev->desc_fifo_histo);
	kfree(tdev->hb);
	kfree(tdev);
}
static struct RTM_T_DEV* lookupDevice(int major)
{
	struct RTM_T_DEV *pos;

	list_for_each_entry(pos, &devices, list){
		if (pos->major == major){
			return pos;
		}
	}
	BUG();
	return 0;
}

struct RTM_T_DEV *rtm_t_lookupDeviceFromClass(struct CLASS_DEVICE *dev)
{
	struct RTM_T_DEV *pos;

	list_for_each_entry(pos, &devices, list){
		if (pos->class_dev == dev){
			return pos;
		}
	}
	BUG();
	return 0;
}

static struct RTM_T_DEV* lookupDevicePci(struct pci_dev *pci_dev)
{
	struct RTM_T_DEV *pos;

	list_for_each_entry(pos, &devices, list){
		if (pos->pci_dev == pci_dev){
			return pos;
		}
	}
	BUG();
	return 0;
}

struct RTM_T_DEV* rtm_t_lookupDev(struct device *dev)
{
	struct RTM_T_DEV *pos;

	list_for_each_entry(pos, &devices, list){
		if (&pos->pci_dev->dev == dev){
			return pos;
		}
	}
	BUG();
	return 0;
}





static void return_empty(struct RTM_T_DEV *tdev, struct HostBuffer *hb)
/** caller MUST lock the list */
{
	dbg(D_DESCR, "ibuf %d", hb->ibuf);
	hb->bstate = BS_EMPTY;
	list_move_tail(&hb->list, &tdev->bp_empties.list);
}

void load_llc_single_dma(struct RTM_T_DEV *tdev, u32 target_pa)
{
	u32 descr = make_single_descriptor_ll(tdev,
		target_pa == RTM_T_USE_HOSTBUF? tdev->hb[0].pa: target_pa);

	if (tdev->is_ao32){
		rtd_write_outbound1_descriptor(tdev, descr);
	}else{
		rtd_write_descr(tdev, descr);
	}
	tdev->regs.H_FCR |= RTMT_D_FCR_DMA_SINGLE_RECYCLE;
	rtd_write_reg(tdev, RTMT_H_FCR, tdev->regs.H_FCR);
}



int poll_acq(struct RTM_T_DEV *tdev, u32 cmd)
/* return 0 if OK */
{
	u32 q_mbox1 = rtd_read_reg(tdev, RTMT_Q_MBOX1);
	u32 new_qmb = q_mbox1;
	int pollcat = 0;

	wait_queue_head_t queue;
	init_waitqueue_head(&queue);

	while((new_qmb&H_MBOX1_LLC_CMD) != (cmd&H_MBOX1_LLC_CMD)){
		new_qmb = rtd_read_reg(tdev, RTMT_Q_MBOX1);
		++pollcat;
		if (new_qmb != q_mbox1){
			info("%5d input change %08x => %08x",
					pollcat, q_mbox1, new_qmb);
			q_mbox1 = new_qmb;
		}

		if (wait_event_interruptible_timeout(
				queue, 0, mbox_poll_ticks)){
			err("NO RESPONSE from ACQ pollcat %d", pollcat);
			return -1;
		}
		dbg(3, "[%5d] want:%08x got@%08x",
				pollcat,
				(new_qmb&H_MBOX1_LLC_CMD),
				(cmd&H_MBOX1_LLC_CMD));
	}

	return 0;
}


void ints_off(void)
{
	if (llc_imask){
		acq200_intsDisable(llc_imask);
	}
	if (llc_disable_interrupts){
		local_irq_disable();
	}
}

void ints_on(void)
{
	if (llc_disable_interrupts){
		local_irq_enable();
	}
	if (llc_imask){
		acq200_intsEnable(llc_imask);
	}
}

void rtm_t_stop_aollc(struct RTM_T_DEV *tdev)
{
	ao32_onStop_LLC(tdev);
	ints_on();
}
void rtm_t_stop_llc(struct RTM_T_DEV *tdev)
{
	ints_on();

	if ((tdev->is_llc&LLC_AI) != 0){
		rtd_write_reg(tdev, RTMT_H_MBOX1, H_MBOX1_LLC_STOP);
		if (poll_acq(tdev, H_MBOX1_LLC_STOP)){
			tdev->is_llc = 0;
		}
		rtd_stop_dma(tdev);
		tdev->is_llc &= ~LLC_AI;
	}
	if ((tdev->is_llc&LLC_AO) != 0){
		tdev->is_llc &= ~LLC_AO;
	}
}

void rtm_t_start_llc(struct RTM_T_DEV *tdev, struct LLC_DEF *llc_def)
{
	u32 cmd;

	tdev->onStop = rtm_t_stop_llc;
	rtd_dma_reset(tdev);
	rtd_start_dma(tdev);
	load_llc_single_dma(tdev, llc_def->target_pa);

	cmd = MAKE_LLC_RUN(llc_def->clk_div, llc_def->clk_pos, llc_def->trg_pos);

	rtd_write_reg(tdev, RTMT_H_MBOX1, cmd);

	if (poll_acq(tdev, cmd) == 0){
		tdev->is_llc |= LLC_AI;
	}
	ints_off();
}

void rtm_t_start_aollc(struct RTM_T_DEV *tdev, struct AO_LLC_DEF* ao_llc_def)
{
	tdev->onStop = rtm_t_stop_aollc;
	ao32_onStart_LLC(tdev);
	rtd_start_aodma(tdev);
	load_llc_single_dma(tdev, ao_llc_def->src_pa);

	tdev->is_llc |= LLC_AO;
	ints_off();
}


static struct file_operations rtm_t_fops_dma;
static struct file_operations rtm_t_fops_dma_poll;
static struct file_operations rtm_t_fops_regs;


int rtm_t_dmaread_open(struct inode *inode, struct file *file)
{
	struct RTM_T_DEV *tdev = PD(file)->dev;


	int ii;

	dbg(1, "45: DMA open");
	dbg(2, "01 %s %d %p<-%p->%p", 
	    tdev->name, PD(file)->minor, 
	    PD(file)->my_buffers.prev, 
	    &PD(file)->my_buffers,
	    PD(file)->my_buffers.next);

	if (rtm_t_reset_buffers(tdev)){
		return -ERESTARTSYS;
	}
	/** @@todo protect with lock ? */
	if (tdev->pid == 0){
		tdev->pid = current->pid;
	}

	if (tdev->pid != current->pid){
		return -EBUSY;
	}

	tdev->req_len = tdev->lowlat_length?
				tdev->lowlat_length: min(tdev->buffer_len, BUFFER_LEN);
	for (ii = 0; ii != nbuffers; ++ii){
		tdev->hb[ii].req_len = tdev->req_len;
	}

	if ((file->f_flags & O_NONBLOCK) != 0){
		file->f_op = &rtm_t_fops_dma_poll;
	}else{
		file->f_op = &rtm_t_fops_dma;
	}
			
	dbg(1, "99");
	return 0;
}



static struct file_operations rtm_t_fops_histo;

int rtm_t_histo_open(struct inode *inode, struct file *file, unsigned *histo)
{
	file->f_op = &rtm_t_fops_histo;
	PD(file)->private = histo;
	return 0;
}

int rtm_t_regs_open(struct inode *inode, struct file *file)
{
	file->f_op = &rtm_t_fops_regs;
	return 0;
}


int rtm_t_open(struct inode *inode, struct file *file)
{
	struct RTM_T_DEV *tdev = lookupDevice(MAJOR(inode->i_rdev));

	dbg(2, "01");
	if (tdev == 0){
		return -ENODEV;
	}else{
		file->private_data = kmalloc(PSZ, GFP_KERNEL);
		PD(file)->dev = tdev;
		PD(file)->minor = MINOR(inode->i_rdev);
		INIT_LIST_HEAD(&PD(file)->my_buffers);

		dbg(2, "33: minor %d", PD(file)->minor);

		rtd_write_reg(
			tdev, RTMT_H_FCR,
			tdev->regs.H_FCR |= RTMT_D_FCR_WANTIT);

		switch((PD(file)->minor)){
		case MINOR_REGREAD:
		case MINOR_UART:
		case MINOR_PBI:
		case MINOR_FIFO:
			return rtm_t_regs_open(inode, file);
		case MINOR_DMAREAD:
			return rtm_t_dmaread_open(inode, file);
		case MINOR_DATA_FIFO:
			return rtm_t_histo_open(
				inode, file, tdev->data_fifo_histo);
		case MINOR_DESC_FIFO:
			return rtm_t_histo_open(
				inode, file, tdev->desc_fifo_histo);
		default:
			dbg(2,"99 tdev %p name %s", tdev, tdev->name);
			return 0;
		}
	}
}


ssize_t bar_read(
	struct file *file, char *buf, size_t count, loff_t *f_pos, int BAR)
{
	struct RTM_T_DEV *tdev = PD(file)->dev; 
	int ii;
	int rc;
	void *va = tdev->mappings[BAR].va;
	int len = tdev->mappings[BAR].len;

	dbg(2, "01 tdev %p va %p name %s", tdev, va, tdev->name);

	if (count > len){
		count = len;
	}
	if (*f_pos >= len){
		return 0;
	}else{
		va += *f_pos;
	}
	
	for (ii = 0; ii < count/sizeof(u32); ++ii){
		u32 reg = readl(va + ii*sizeof(u32));
		rc = copy_to_user(buf+ii*sizeof(u32), &reg, sizeof(u32));
		if (rc){
			return -1;
		}
	}
	*f_pos += count;
	return count;
}


ssize_t rtm_t_read(struct file *file, char __user *buf, size_t count, loff_t *f_pos)
{
	return bar_read(file, buf, count, f_pos, minor2bar(PD(file)->minor));
}


ssize_t bar_write(
	struct file *file, const char *buf, size_t count, loff_t *f_pos,
	int BAR, int LEN, int OFFSET)
{
	struct RTM_T_DEV *tdev = PD(file)->dev; 
	u32 data;
	void *va = tdev->mappings[BAR].va + OFFSET;
	int rc = copy_from_user(&data, buf, min(count, sizeof(u32)));

	int ii;
	if (rc){
		return -1;
	}
	if (*f_pos > LEN){
		return -1;
	}else if (count + *f_pos > LEN){
		count = LEN - *f_pos;
	}

	for (ii = 0; ii < count; ii += sizeof(u32)){
		u32 readval = readl(va+ii);
		dbg(2, "writing %p = 0x%08x was 0x%08x", 
					va+ii, data+ii, readval);
	
		writel(data+ii, va+ii);
	}

	*f_pos += count;
	return count;
}

ssize_t rtm_t_write(
	struct file *file, const char *buf, size_t count, loff_t *f_pos)
{
	return bar_write(file, buf, count, f_pos, REGS_BAR, REGS_LEN, 0);
}

ssize_t uart_read(struct file *file, char *buf, size_t count, loff_t *f_pos)
{
	return bar_read(file, buf, count, f_pos, UART_BAR);
		       
}

static void write_test(u32 *scr, u32 value)
{
	u32 readback;

	writel(value, scr);

	if (rtm_t_debug){
		readback = readl(scr);
		dbg(1, "%p wrote %02x read %02x %s", 
		    scr, value, readback, readback==value? "OK": "FAIL");
	}
}
ssize_t uart_write(
	struct file *file, const char *buf, size_t count, loff_t *f_pos)
{
	struct RTM_T_DEV *tdev = PD(file)->dev; 
	u32 *scr = ((u32*)tdev->mappings[UART_BAR].va) + 7;
	int ii;

	for (ii = 0; ii < 8; ++ii){
		write_test(scr, 1 << ii);
	}
	write_test(scr, 0x55);
	write_test(scr, 0xaa);
	write_test(scr, 0x37);

	return count;
//	return bar_write(file, buf, count, f_pos, UART_BAR, UART_LEN, 0);
}

static int rtm_t_start_stream(struct RTM_T_DEV *tdev, unsigned buffers_demand)
{
	dbg(1, "01");
	rtd_dma_reset(tdev);
	memset(&tdev->job, 0, sizeof(struct JOB));

	/* configure MODE. actual reg write comes later .. */
	if (test_mode == 0){
		tdev->regs.H_FCR &= ~RTMT_D_FCR_SIM_CLR;
	}else{
		u32 tm = ((test_mode-1)&RTMT_D_FCR_SIM_THROTTLE_MSK) <<
			RTMT_D_FCR_SIM_THROTTLE_SHL;
		tdev->regs.H_FCR = RTMT_D_FCR_SIM_MODE | tm;
	}
	tdev->job.buffers_demand = buffers_demand;
	if (unlikely(list_empty_careful(&tdev->bp_empties.list))){
		err("no free buffers");
		return -ERESTARTSYS;
	}

	spin_lock(&tdev->job_lock);
	tdev->job.please_stop = PS_OFF;
	spin_unlock(&tdev->job_lock);
	set_bit(WORK_REQUEST, &tdev->work.w_to_do);
	wake_up_interruptible(&tdev->work.w_waitq);
	dbg(1, "99");
	return 0;
}

int rtm_t_release(struct inode *inode, struct file *file)
{
	dbg(2, "01");
	kfree(file->private_data);
	return 0;
}

long  rtm_t_dma_ioctl(struct file *filp,
                        unsigned int cmd, unsigned long arg)
{
	struct RTM_T_DEV *tdev = PD(filp)->dev;
	void* varg = (void*)arg;


	switch(cmd){
	case RTM_T_START_STREAM:
		return rtm_t_start_stream(tdev, transfer_buffers);
	case RTM_T_START_STREAM_MAX: {
		u32 my_transfer_buffers;
		COPY_FROM_USER(&my_transfer_buffers, varg, sizeof(u32));
		return rtm_t_start_stream(tdev, my_transfer_buffers);
	}
	case RTM_T_START_LLC: {
		struct LLC_DEF llc_def;
		COPY_FROM_USER(&llc_def, varg, sizeof(struct LLC_DEF));
		rtm_t_start_llc(tdev, &llc_def);
		return 0;
	} case RTM_T_START_AOLLC: {
		struct AO_LLC_DEF ao_llc_def;
		COPY_FROM_USER(&ao_llc_def, varg, sizeof(struct AO_LLC_DEF));
		rtm_t_start_aollc(tdev, &ao_llc_def);
		return 0;
	} default:
		return -ENOTTY;
	}

}


int rtm_t_dma_release(struct inode *inode, struct file *file)
{
	struct RTM_T_DEV *tdev = PD(file)->dev;
	struct HostBuffer *hb;
	struct HostBuffer *tmp;

	dbg(1, "01 %s %d %p<-%p->%p", 
		tdev->name, PD(file)->minor, 
		PD(file)->my_buffers.prev, 
		&PD(file)->my_buffers,
		PD(file)->my_buffers.next);

	if (mutex_lock_interruptible(&tdev->list_mutex)){
		return -ERESTARTSYS;
	}
	list_for_each_entry_safe(hb, tmp, &PD(file)->my_buffers, list){
		dbg(3, "returning %d", hb->ibuf);
		return_empty(tdev, hb);
	}	

	mutex_unlock(&tdev->list_mutex);

	dbg(2, "90");
	tdev->job.please_stop = PS_PLEASE_STOP;
	tdev->job.buffers_demand = 0;

	if (tdev->onStop){
		tdev->onStop(tdev);
		tdev->onStop = 0;
	}
	tdev->pid = 0;
	return rtm_t_release(inode, file);
}



int rtm_t_mmap_host(struct file* file, struct vm_area_struct* vma)
/**
 * mmap the host buffer.
 */
{
	struct RTM_T_DEV *tdev = PD(file)->dev; 
	int ibuf = PD(vma->vm_file)->minor&NBUFFERS_MASK;
	struct HostBuffer *hb = &tdev->hb[ibuf];
	unsigned long vsize = vma->vm_end - vma->vm_start;
	unsigned long psize = hb->len;
	unsigned pfn = hb->pa >> PAGE_SHIFT;

	dbg(2, "%c vsize %lu psize %lu %s",
		'D', vsize, psize, vsize>psize? "EINVAL": "OK");

	if (vsize > psize){
		return -EINVAL;                   /* request too big */
	}
	if (remap_pfn_range(
		vma, vma->vm_start, pfn, vsize, vma->vm_page_prot)){
		return -EAGAIN;
	}else{
		return 0;
	}
}

int rtm_t_mmap_bar(struct file* file, struct vm_area_struct* vma)
{
	struct RTM_T_DEV *tdev = PD(file)->dev;
	int bar = minor2bar(PD(file)->minor);
	unsigned long vsize = vma->vm_end - vma->vm_start;
	unsigned long psize = tdev->mappings[bar].len;
	unsigned pfn = tdev->mappings[bar].pa >> PAGE_SHIFT;

	dbg(2, "%c vsize %lu psize %lu %s",
		'D', vsize, psize, vsize>psize? "EINVAL": "OK");

	if (vsize > psize){
		return -EINVAL;                   /* request too big */
	}
	if (io_remap_pfn_range(
		vma, vma->vm_start, pfn, vsize, vma->vm_page_prot)){
		return -EAGAIN;
	}else{
		return 0;
	}
}

ssize_t rtm_t_histo_read(
	struct file *file, char *buf, size_t count, loff_t *f_pos)
{
	unsigned *the_histo = PD(file)->private;	
	int maxentries = PD(file)->minor == MINOR_DATA_FIFO ?
		RTDMAC_DATA_FIFO_CNT: RTDMAC_DESC_FIFO_CNT;
	unsigned cursor = *f_pos;	/* f_pos counts in entries */
	int rc;

	if (cursor >= maxentries){
		return 0;
	}else{
		int headroom = (maxentries - cursor) * sizeof(unsigned);
		if (count > headroom){
			count = headroom;
		}
	}
	rc = copy_to_user(buf, the_histo+cursor, count);
	if (rc){
		return -1;
	}

	*f_pos += count/sizeof(unsigned);
	return count;
}

static void mark_empty(struct device *dev, struct HostBuffer *hb){
	u32 mark_len = 2 * sizeof(u32);
	u32 offset = hb->req_len - mark_len;
	u32 *pmark = (u32*)(hb->va + offset);

	pmark[0] = EMPTY1;
	pmark[1] = EMPTY2;

	/* direction may be wrong - we're trying to flush */
	dma_sync_single_for_device(dev, hb->pa, hb->req_len, PCI_DMA_TODEVICE);
}


static int is_marked_empty(struct device *dev, struct HostBuffer *hb){
	u32 mark_len = 2 * sizeof(u32);
	u32 offset = hb->req_len - mark_len;
	u32 *pmark = (u32*)(hb->va + offset);
	int is_empty;

	dma_sync_single_for_cpu(dev, hb->pa, hb->req_len, PCI_DMA_FROMDEVICE);

	is_empty = pmark[0] == EMPTY1 && pmark[1] == EMPTY2;

	return is_empty;
}
static int queue_next_free_buffer(struct RTM_T_DEV* tdev)
{
	int rc = 0;

	if (mutex_lock_interruptible(&tdev->list_mutex)){
		return -ERESTARTSYS;
	}
	if (!list_empty_careful(&tdev->bp_empties.list)){
		struct HostBuffer *hb = HB_ENTRY(tdev->bp_empties.list.next);
			
		mark_empty(&tdev->pci_dev->dev, hb);

		rtd_load_descriptor(tdev, hb->ibuf);
		hb->bstate = BS_FILLING;
		list_move_tail(&hb->list, &tdev->bp_filling.list);
		rc = 1;
	}
	mutex_unlock(&tdev->list_mutex);
	return rc;
}

static void queue_free_buffers(struct RTM_T_DEV* tdev)
{	
	unsigned queued = tdev->job.buffers_queued;
	int in_queue =  queued -
		(tdev->job.buffers_received + tdev->job.buffers_discarded);

	while (queued < tdev->job.buffers_demand){
		if (queue_next_free_buffer(tdev)){
	                ++queued;
		        ++in_queue;
		}else{
			if (in_queue == 0){
				++stalls;
			}
			break;
		}
        }
        tdev->job.buffers_queued = queued;
}

struct HostBuffer* hb_from_descr(struct RTM_T_DEV* tdev, u32 inflight_descr)
{
	int ii;

	for (ii = 0; ii < nbuffers; ++ii){
		if (tdev->hb[ii].descr == inflight_descr){
			return &tdev->hb[ii];
		}
	}
	return 0;
}
static void report_inflight(struct RTM_T_DEV* tdev, int ibuf, int is_error, char *msg)
{
	u32 inflight_descr = rtd_read_reg(tdev, RTMT_H_IB_DMA_STAT);
	struct HostBuffer*  inflight = hb_from_descr(tdev, inflight_descr);

	if (tdev->job.buffers_demand == 0){
		return;
	}
	if (is_error){
		err("%30s: buffer %02d  last descr:%08x [%02d] fifo:%08x",
			msg,
			ibuf,
			inflight_descr,
			inflight? inflight->ibuf: -1,
			rtd_read_reg(tdev, RTMT_H_IB_DMA_FIFSTA));
	}else{
		dbg(1, "%30s: buffer %02d last descr:%08x [%02d] fifo:%08x",
				msg,
				ibuf,
				inflight_descr,
				inflight? inflight->ibuf: -1,
				rtd_read_reg(tdev, RTMT_H_IB_DMA_FIFSTA));
	}
}
static void report_stuck_buffer(struct RTM_T_DEV* tdev, int ibuf)
{
	report_inflight(tdev, ibuf, 0, "buffer was skipped");
}

static int queue_full_buffers(struct RTM_T_DEV* tdev)
{
	struct HostBuffer* hb;
	struct HostBuffer* tmp;
	struct HostBuffer* first = 0;
	int nrx = 0;
	int ifilling = 0;

	if (mutex_lock_interruptible(&tdev->list_mutex)){
		return -ERESTARTSYS;
	}

	list_for_each_entry_safe(hb, tmp, &tdev->bp_filling.list, list){
		if (++ifilling == 1){
			first = hb;
		}
		if (is_marked_empty(&tdev->pci_dev->dev, hb)){
			if (ifilling > 1){
				break; 	/* top 2 buffers empty: no action */
			}else{
				continue;  /* check skipped data. */
			}
		}else{
			if (ifilling > 1 && first && hb != first){
				tdev->job.buffers_discarded++;
				report_stuck_buffer(tdev, first->ibuf);
				return_empty(tdev, first);
				first = 0;
				if (stop_on_skipped_buffer){
					tdev->job.please_stop = PS_PLEASE_STOP;
				}
			}
			if (rtm_t_debug){
				report_inflight(tdev, hb->ibuf, 0, "->FULL");
			}

			hb->bstate = BS_FULL;
			list_move_tail(&hb->list, &tdev->bp_full.list);
			tdev->job.buffers_received++;
			++nrx;
		}
	}	

	mutex_unlock(&tdev->list_mutex);
	return nrx;
}


ssize_t rtm_t_dma_read(
	struct file *file, char __user *buf, size_t count, loff_t *f_pos)
/* returns when buffer[s] available
 * data is buffer index as array of unsigned
 * return len is sizeof(array)
 */
{
	struct RTM_T_DEV *tdev = PD(file)->dev; 
	int rc;

	dbg(2, "01 ps %u count %ld demand %d received %d waiting %d", 
	    (unsigned)*f_pos,	(long)count,
		tdev->job.buffers_demand, tdev->job.buffers_received, 
		!list_empty(&tdev->bp_full.list)	);

	if (tdev->job.buffers_received >= tdev->job.buffers_demand &&
		list_empty(&tdev->bp_full.list)	){
		dbg(3, "job done");
		return 0;
	}

	if (*f_pos == 0){
		rc = wait_event_interruptible(
			tdev->return_waitq, 
			!list_empty(&tdev->bp_full.list));
	}else{
		rc = wait_event_interruptible_timeout(
			tdev->return_waitq, 
			!list_empty(&tdev->bp_full.list),
			RX_TO);
	}

	dbg(3, "done waiting, rc %d", rc);

	if (rc < 0){
		dbg(3, "RESTART");
		return -ERESTARTSYS;
	}else if (mutex_lock_interruptible(&tdev->list_mutex)){
		return -ERESTARTSYS;
	}else{
		struct HostBuffer *hb;
		struct HostBuffer *tmp;
		int nbytes = 0;
	      
		list_for_each_entry_safe(hb, tmp, &tdev->bp_full.list, list){
			if (nbytes+sizeof(int) > count){
				dbg(3, "quit nbytes %d count %lu", 
				    nbytes, (long)count);
				break;
			}

			if (copy_to_user(buf+nbytes, &hb->ibuf, sizeof(int))){
				rc = -EFAULT;
				goto read99;
			}
			dbg(2, "add my_buffers %d", hb->ibuf);

			list_move_tail(&hb->list, &PD(file)->my_buffers);
			hb->bstate = BS_FULL_APP;
			nbytes += sizeof(int);
		}

		if (rc == 0 && nbytes == 0){
			dbg(3, "TIMEOUT");
			rc = -ETIMEDOUT;
		}else{
			*f_pos += nbytes;
			dbg(3, "return %d", nbytes);
			rc = nbytes;
		}
	}
read99:
	mutex_unlock(&tdev->list_mutex);
	return rc;
}

ssize_t rtm_t_dma_read_poll(
	struct file *file, char __user *buf, size_t count, loff_t *f_pos)
/* returns when buffer[s] available
 * data is buffer index as array of unsigned
 * return len is sizeof(array)
 */
{
	struct RTM_T_DEV *tdev = PD(file)->dev; 
	int rc = 0;
	struct HostBuffer *hb;
	struct HostBuffer *tmp;
	int nbytes = 0;

	dbg(2, "01 ps %u count %ld demand %d received %d waiting %d", 
	    (unsigned)*f_pos,	(long)count,
	    tdev->job.buffers_demand, tdev->job.buffers_received, 
	    !list_empty(&tdev->bp_full.list)	);

	if (tdev->job.buffers_received >= tdev->job.buffers_demand &&
	    list_empty(&tdev->bp_full.list)	){
		dbg(3, "job done");
		return 0;
	}

	if (!rtd_dma_started(tdev)){
		rtd_start_dma(tdev);	
	}
	if (queue_full_buffers(tdev)){	      
		list_for_each_entry_safe(hb, tmp, &tdev->bp_full.list, list){
			if (nbytes+sizeof(int) > count){
				dbg(3, "quit nbytes %d count %lu", 
				    nbytes, (long)count);
				break;
			}

			if (copy_to_user(buf+nbytes, &hb->ibuf, sizeof(int))){
				rc = -EFAULT;
				goto read99;
			}
			dbg(2, "add my_buffers %d", hb->ibuf);

			list_move_tail(&hb->list, &PD(file)->my_buffers);
			hb->bstate = BS_FULL_APP;
			nbytes += sizeof(int);
		}

		if (rc == 0 && nbytes == 0){
			dbg(3, "TIMEOUT");
			rc = -ETIMEDOUT;
		}else{
			*f_pos += nbytes;
			dbg(3, "return %d", nbytes);
			rc = nbytes;
		}
	}
read99:
	return rc;
}


ssize_t rtm_t_dma_write(
	struct file *file, const char *buf, size_t count, loff_t *f_pos)
/* write completed data.
 * data is array of full buffer id's
 * id's are removed from full and placed onto empty.
 */
{
	struct RTM_T_DEV *tdev = PD(file)->dev; 
	int nbytes = 0;
	int rc = 0;

	dbg(2, "pos %u count %lu", (unsigned)*f_pos, (long)count);

	if (mutex_lock_interruptible(&tdev->list_mutex)){
		return -ERESTARTSYS;
	}
	while (nbytes+sizeof(int) <= count){
		int id;

		if (copy_from_user(&id, buf+nbytes, sizeof(int))){
			return -EFAULT;
		}else{
			dbg(2, "[%u] recycle buffer %d", 
			    (unsigned)(nbytes/sizeof(int)), id);

			if (id < 0){
				err("ID < 0");
				rc = -100;
				goto write99;
			}else if (id >= nbuffers){
				err("ID > NBUFFERS");
				rc = -101;
				goto write99;
			}else if (tdev->hb[id].bstate != BS_FULL_APP){
				err("STATE != BS_FULL_APP %d", 
						tdev->hb[id].bstate);
				rc = -102;
				goto write99;
			}else{
				struct HostBuffer *hb;
				
				rc = -1;

				list_for_each_entry(
					hb, &PD(file)->my_buffers, list){

					dbg(3, "listing %d", hb->ibuf);
					assert(hb != 0);
					assert(hb->ibuf >= 0 && 
						hb->ibuf < nbuffers);
					if (hb->ibuf == id){
						return_empty(tdev, hb);
						nbytes += sizeof(int);
						rc = 0;
						break;
					}
				}	
				if (rc == -1){
					err("ATTEMPT TO RET BUFFER NOT MINE");
					goto write99;
				}
			}
		}
	}
		
	*f_pos += nbytes;
	rc = nbytes;

write99:
	mutex_unlock(&tdev->list_mutex);
	dbg(2, "99 return %d", rc);
	return rc;
}

static void smooth(unsigned *rate, unsigned *old, unsigned *new)
{
#define RATE	*rate
#define OLD	*old
#define NEW	*new
	
	if (likely(NEW > OLD)){
		RATE = (SMOO*RATE + (10-SMOO)*(NEW-OLD))/10;
	}else{
		RATE = 0;
	}
	OLD = NEW;
#undef NEW
#undef OLD
#undef RATE
}
static int rtm_t_mon(void *arg)
{
	struct RTM_T_DEV* tdev = (struct RTM_T_DEV*)arg;
	wait_queue_head_t waitq;

	init_waitqueue_head(&waitq);	

	while(!kthread_should_stop()){
		struct JOB *job = &tdev->job;
		wait_event_interruptible_timeout(waitq, 0, HZ);

		smooth(&job->rx_rate, 
			&job->rx_buffers_previous, &job->buffers_received);
	
		smooth(&job->int_rate, &job->int_previous, &job->ints);
	}

	return 0;
}

unsigned check_fifo_xxxx(unsigned* histo, u32 regval)
{
	u32 ptr = (regval >> RTMT_H_XX_DMA_FIFSTA_COUNT_SHL) & RTMT_H_XX_DMA_FIFSTA_COUNT_MSK;
	histo[ptr]++;
	return regval & ~(ptr << RTMT_H_XX_DMA_FIFSTA_COUNT_SHL);
}

static void check_fifo_status(struct RTM_T_DEV* tdev)
{
	u32 desc_sta = rtd_read_reg(tdev, RTMT_H_IB_DMA_FIFSTA);
	u32 desc_flags = check_fifo_xxxx(tdev->desc_fifo_histo, desc_sta);
	u32 data_sta = rtd_read_reg(tdev, RTMT_C_DATA_FIFSTA);
	u32 data_flags = check_fifo_xxxx(tdev->data_fifo_histo, data_sta);

	if ((data_flags & RTMT_H_XX_DMA_FIFSTA_FULL)   &&
					tdev->job.errors < 10){
		/** @@todo .. do something! */
		err("GAME OVER: %d FIFSTA_DATA_OVERFLOW: 0x%08x", 
		    tdev->idx, data_sta);		
		if (++tdev->job.errors == 10){
			err("too many errors, turning reporting off ..");
		}
	}
	if ((desc_flags & RTMT_H_XX_DMA_FIFSTA_FULL) != 0 &&
					tdev->job.errors < 10){
		err("GAME OVER: %d FIFSTA_DESC_OVERFLOW: 0x%08x", 
		    tdev->idx, desc_sta);	
		if (++tdev->job.errors == 10){
			err("too many errors, turning reporting off ..");
		}
	}	
}

static int rtm_t_isr_work(void *arg)
{
	struct RTM_T_DEV* tdev = (struct RTM_T_DEV*)arg;
	int loop_count = 0;
/* this is going to be the top RT process */
	struct sched_param param = { .sched_priority = 10 };
	int please_check_fifo = 0;

	sched_setscheduler(current, SCHED_FIFO, &param);
	

	for ( ; 1; ++loop_count){
		int timeout = wait_event_interruptible_timeout(
			tdev->work.w_waitq,
			test_and_clear_bit(WORK_REQUEST, &tdev->work.w_to_do) ||
			kthread_should_stop(),
			WORK_TO) == 0;

		if (!timeout || loop_count%10 == 0){
			dbg(3, "TIMEOUT? %d queue_free_buffers() ? %d",
			    timeout, 
			    !tdev->job.please_stop && 
			    tdev->job.buffers_queued < tdev->job.buffers_demand);
		}
	        if (!tdev->job.please_stop && 
			tdev->job.buffers_queued < tdev->job.buffers_demand){
			if (!rtd_dma_started(tdev)){
				rtd_start_dma(tdev);	
			}
			queue_free_buffers(tdev);
		}


		if (tdev->job.buffers_demand > 0 ){
			if (queue_full_buffers(tdev) > 0){
				wake_up_interruptible(&tdev->return_waitq);
			}
		}					

		spin_lock(&tdev->job_lock);
		switch(tdev->job.please_stop){
		case PS_STOP_DONE:
			break;
		case PS_PLEASE_STOP:
			rtd_stop_dma(tdev);
			tdev->job.please_stop = PS_STOP_DONE;
			break;
		default:
			if (rtd_dma_started(tdev)){
				please_check_fifo = 1;
			}
		}
		spin_unlock(&tdev->job_lock);

		if (please_check_fifo){
			check_fifo_status(tdev);
			please_check_fifo = 0;
		}
	}

	return 0;
}

static irqreturn_t rtm_t_rx_isr(int irq, void *data)
{
       struct RTM_T_DEV* tdev = (struct RTM_T_DEV*)data;

       dbg(D_ISR, "01 irq %d", irq);

       if (tdev->job.buffers_demand == 0 && 
			tdev->job.please_stop != PS_PLEASE_STOP){
	       return !IRQ_HANDLED;
       }
       
       tdev->job.ints++;
       set_bit(WORK_REQUEST, &tdev->work.w_to_do);
       wake_up_interruptible(&tdev->work.w_waitq);       

       dbg(D_ISR, "99");
       return IRQ_HANDLED;
}


static struct file_operations rtm_t_fops = {
	.open = rtm_t_open,
	.release = rtm_t_release,
	.read = rtm_t_read,
	.write = rtm_t_write,
	.mmap = rtm_t_mmap_host
};

static struct file_operations rtm_t_fops_regs = {
	.open = rtm_t_open,
	.release = rtm_t_release,
	.read = rtm_t_read,
	.write = rtm_t_write,
	.mmap = rtm_t_mmap_bar
};


static struct file_operations rtm_t_fops_dma = {
	.open = rtm_t_open,
	.release = rtm_t_dma_release,
	.read = rtm_t_dma_read,
	.write = rtm_t_dma_write,
	.mmap = rtm_t_mmap_host,
	.unlocked_ioctl = rtm_t_dma_ioctl
};

static struct file_operations rtm_t_fops_dma_poll = {
	.open = rtm_t_open,
	.release = rtm_t_dma_release,
	.read = rtm_t_dma_read_poll,
	.write = rtm_t_dma_write,
	.mmap = rtm_t_mmap_host,
	.unlocked_ioctl = rtm_t_dma_ioctl
};

static struct file_operations rtm_t_fops_histo = {
	.read = rtm_t_histo_read,
	.release = rtm_t_release
};


static int hook_interrupts(struct RTM_T_DEV* tdev)
{
	struct pci_dev *dev = tdev->pci_dev;
	int rc = pci_enable_msi(dev);

	dbg(2, "%d IRQ %d", __LINE__, dev->irq);

	if (rc < 0){
		err("pci_enable_msi FAILED");
		return rc;
	}

	rc = request_irq(dev->irq+MSI_DMA, RTM_T_RX_ISR, 
			 MSI_DMA==MSI_UART?IRQF_SHARED: 0, tdev->name, tdev);
	if (rc){
		err("request_irq %d failed", dev->irq+MSI_DMA);
	}

	return rc;
}


static int _rw_test(struct RTM_T_DEV *tdev, u32 tv)
{
	u32 rv;

	rtd_write_reg(tdev, RTMT_H_TEST, tv);
	rv = rtd_read_reg(tdev, RTMT_H_TEST);
	if (rv != tv){
		err("w:%08x r:%08x", tv, rv);		
	}
	return rv != tv;
}
static int bit_test(struct RTM_T_DEV *tdev)
{
	int rc = 0;
	rc |= _rw_test(tdev, 0xdeadbeef);
	rc |= _rw_test(tdev, 0xaa55aa55);
	rc |= _rw_test(tdev, 0x55aa55aa);	
	return rc;
}

static void rtm_t_id(struct RTM_T_DEV *tdev)
{
	info("REVID :%08x PCIECSR:%08x", 
	     rtd_read_reg(tdev, RTMT_C_REVID),
	     rtd_read_reg(tdev, RTMT_C_PCI_CSR));

}

static void startWork(struct RTM_T_DEV *tdev)
{
	tdev->work.w_task = kthread_run(rtm_t_isr_work, tdev, tdev->name);
	tdev->work.mon_task = kthread_run(rtm_t_mon, tdev, tdev->mon_name);
}

static void stopWork(struct RTM_T_DEV *tdev)
{
	if (tdev->work.w_task){
		kthread_stop(tdev->work.w_task);
	}
	if (tdev->work.mon_task){
		kthread_stop(tdev->work.mon_task);
	}
}

static void 
early_init_personality(struct RTM_T_DEV *tdev, struct pci_dev *dev)
{
	switch(dev->subsystem_device){
	case PCI_SUBDID_RTMT_AO:
		info("RTMT_AO device detected");
		tdev->dma_page_size = AODMAC_PAGE;
		tdev->lldma_page_size = AODMAC_LL_PAGE;
		tdev->is_ao32 = 1;
		break;
	default:
		tdev->dma_page_size = RTDMAC_PAGE;
		tdev->lldma_page_size = RTDMAC_LL_PAGE;
		break;
	}
}

static void 
init_personality(struct RTM_T_DEV *tdev, struct pci_dev *dev)
{
	switch(dev->subsystem_device){
	case  PCI_SUBDID_FHBA:
		create_sfp_i2c(tdev);
	#ifdef 	SPI_SUPPORT
		acqfhba_spi_master_init(tdev);
	#endif
		tdev->is_fhba = 1;
	default:
		;
	}

	tdev->buffer_len = buffer_len;
	tdev->iop_push = iop_push;
}
static int
rtm_t_probe(struct pci_dev *dev, const struct pci_device_id *ent)
{
	static int idx;
	int rc;
	int imap;
	static u64 dma_mask = DMA_BIT_MASK(32);

	struct RTM_T_DEV *tdev = 
		kzalloc(sizeof(struct RTM_T_DEV), GFP_KERNEL);
	tdev->hb = kzalloc(nbuffers*sizeof(struct HostBuffer), GFP_KERNEL);

	if (ent->device == PCI_DEVICE_ID_XILINX_PCIE){
		info("OLD device id %04x : RTM-T firmware update RECOMMENDED",
				ent->device);
	}
	dbg(2, "01");
	tdev->pci_dev = dev;
	tdev->idx = idx++;
	dev->dev.dma_mask = &dma_mask;
	sprintf(tdev->name, "rtm-t.%d", tdev->idx);
	sprintf(tdev->mon_name, "rtm-t-mon.%d", tdev->idx);


	rc = register_chrdev(0, tdev->name, &rtm_t_fops);
	if (rc < 0){
		err("can't get major");
		kfree(tdev);
		return rc;
	}else{
		tdev->major = rc;
	}



#if DEBUG > 0
	debug_bars(dev);
#endif	
#if 0
	rc = pci_request_regions(dev, tdev->name);
	if (rc) {	
		err("region request failed");
		return rc;
	}
#endif
	early_init_personality(tdev, dev);

	for (imap = 0; imap < MAP_COUNT; ++imap){
		struct PciMapping* mp = tdev->mappings+imap;
		int bar = MAP2BAR(tdev, imap);

		if (VALID_BAR(bar)){		
			sprintf(mp->name, "rtm_t.%d.%d", tdev->idx, bar);
					
			mp->pa = pci_resource_start(dev,bar)&
						PCI_BASE_ADDRESS_MEM_MASK;
			mp->len = pci_resource_len(dev, bar);
	
			if (bar != UART_BAR){
				mp->region = request_mem_region(
					mp->pa, mp->len, mp->name);	
			}
			if (bar == UART_BAR || mp->region != 0){
				mp->va = ioremap_nocache(mp->pa, mp->len);
			}else{
				err( "request_region(0x%08x, %d, %s) failed\n",
					mp->pa, mp->len, mp->name);
				return rc = -ENODEV;
			}

			dbg(2, "BAR %d va:%p", bar, mp->va);
		}
	}

	dbg(2, "%d IRQ %d", __LINE__, dev->irq);
	rc = pci_enable_device(dev);
	dbg(2, "%d IRQ %d", __LINE__, dev->irq);
	if (rc != 0){
		err("pci_enable_device failed");
		return rc;
	}

#ifdef L0S_HACK
	info("disable L0S %04x", PCIE_LINK_STATE_L0S|PCIE_LINK_STATE_L1|
			PCIE_LINK_STATE_CLKPM);
	pci_disable_link_state(dev,
			PCIE_LINK_STATE_L0S|PCIE_LINK_STATE_L1|
			PCIE_LINK_STATE_CLKPM);
#endif
	init_personality(tdev, dev);
	init_buffers(tdev);
	rtm_t_id(tdev);
	if (bit_test(tdev)){
		if (!ignore_bit_test){
			err("BIT failed: FATAL");
			return -1;
		}else{
			err("BIT failed but continuing");
		}
	}

	if ((rc = hook_interrupts(tdev)) < 0){
		err("hook_interrupt failed, unwind needed");
	}

	registerDevice(tdev);
	rtm_t_createSysfs(tdev);


	tdev->class_dev = CLASS_DEVICE_CREATE(
		rtm_t_device_class,			/* cls */
		NULL,					/* cls_parent */
		tdev->idx,				/* "devt" */
		&tdev->pci_dev->dev,			/* device */
		"rtm-t.%d", tdev->idx);			/* fmt, idx */

	rtm_t_create_sysfs_class(tdev);
	startWork(tdev);
	rc = rtm_t_uart_init(dev, tdev->mappings[UART_BAR].va);

	return rc;
}

static void rtm_t_remove (struct pci_dev *dev)
{
	struct RTM_T_DEV *tdev = lookupDevicePci(dev);
	int imap;

	stopWork(tdev);

	if (tdev->is_fhba){
		remove_sfp_i2c(tdev);
	}
	rtm_t_uart_remove(dev);
	rtm_t_remove_sysfs_class(tdev);
	unregister_chrdev(tdev->major, tdev->name);

	pci_release_regions(dev);

	for (imap = 0; imap < MAP_COUNT; ++imap){
		struct PciMapping* mp = tdev->mappings+imap;
		if (mp->va){
			iounmap(mp->va);
		}
		if (mp->region){
			release_region(mp->pa, mp->len);
		}
	}
	pci_disable_device(dev);
	deleteDevice(tdev);
}
/*
 *
 * { Vendor ID, Device ID, SubVendor ID, SubDevice ID,
 *   Class, Class Mask, String Index }
 */
static struct pci_device_id rtm_t_pci_tbl[] = {
	{ PCI_VENDOR_ID_XILINX, PCI_DEVICE_ID_DTACQ_PCIE, 
			PCI_SUBVID_DTACQ, PCI_SUBDID_RTMT, 0 },
	{ PCI_VENDOR_ID_XILINX, PCI_DEVICE_ID_DTACQ_PCIE, 
			PCI_SUBVID_DTACQ, PCI_SUBDID_FHBA, 0 },
	{ PCI_VENDOR_ID_XILINX, PCI_DEVICE_ID_DTACQ_PCIE, 
			PCI_SUBVID_DTACQ, PCI_SUBDID_CPSC, 0 },
	{ PCI_VENDOR_ID_XILINX, PCI_DEVICE_ID_DTACQ_PCIE,
			PCI_SUBVID_DTACQ, PCI_SUBDID_RTMT_AO, 0 },
	{ PCI_VENDOR_ID_XILINX, PCI_DEVICE_ID_XILINX_PCIE,
			PCI_SUBVID_DTACQ, PCI_SUBDID_RTMT, 0 },
	{ PCI_VENDOR_ID_XILINX, PCI_DEVICE_ID_XILINX_PCIE,
			PCI_SUBVID_DTACQ, PCI_SUBDID_FHBA, 0 },
	{ PCI_VENDOR_ID_XILINX, PCI_DEVICE_ID_XILINX_PCIE,
			PCI_SUBVID_DTACQ, PCI_SUBDID_CPSC, 0 },
	{ PCI_VENDOR_ID_XILINX, PCI_DEVICE_ID_XILINX_PCIE,
			PCI_SUBVID_DTACQ, PCI_SUBDID_RTMT_AO, 0 },
        { }
};
static struct pci_driver rtm_t_driver = {
        .name     = rtm_t_driver_name,
        .id_table = rtm_t_pci_tbl,
        .probe    = rtm_t_probe,
//        .remove   = __devexit_p(rtm_t_remove),
	.remove = rtm_t_remove
};



int __init rtm_t_init_module(void)
{
	int rc;

	info("%s %s %s\n%s",
	     rtm_t_driver_name, rtm_t__driver_string,
	     rtm_t__driver_version,
	     rtm_t__copyright);

	rtm_t_device_class = class_create(THIS_MODULE, "rtm-t");
	rc = pci_register_driver(&rtm_t_driver);
	return rc;	
}

void rtm_t_exit_module(void)
{
	pci_unregister_driver(&rtm_t_driver);
}

module_init(rtm_t_init_module);
module_exit(rtm_t_exit_module);

MODULE_DEVICE_TABLE(pci, rtm_t_pci_tbl);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Host Driver for RTM-T");
