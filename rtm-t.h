/* ------------------------------------------------------------------------- */
/* rtm-t.h RTM-T PCIe register defs					     */
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


/** @file rtm-t-hostdrv.c D-TACQ RTM-T PCIe regdefs */

#ifndef __RTM_T_H__
#define __RTM_T_H__


#define PCI_VENDOR_ID_XILINX      0x10ee
#define PCI_DEVICE_ID_XILINX_PCIE 0x0007
// D-TACQ changes the device ID to work around unwanted zomojo lspci listing */
#define PCI_DEVICE_ID_DTACQ_PCIE  0xadc1

#define PCI_SUBVID_DTACQ	0xd1ac
#define PCI_SUBDID_RTMT		0x4000
#define PCI_SUBDID_FHBA		0x4100
#define PCI_SUBDID_CPSC		0x4110
#define PCI_SUBDID_RTMT_AO	0x4080


/* REGISTERS = offset from BAR0 */
/* NEW: from PRM REV 3 */

/* _H_ HOST: */

#define RTMT_H_PCIE_CSR		0x00
#define RTMT_H_DMAC_CTRL	0x04
#define RTMT_H_DEBUG		0x08
#define RTMT_H_INT		0x0c
/* NOT USED 			0x10 */
/* NOT USED			0x14 */
/* NOT USED			0x18 */
#define RTMT_H_DEBUG_COUNT	0x1c
#define RTMT_H_TEST		0x20
#define RTMT_H_FCR		0x24
#define RTMT_H_DIO		0x28
#define RTMT_H_SPI_CTL		0x2c	/* not on RTM-T (yet) */
#define RTMT_H_SPI_DAT		0x30	/* not on RTM-T (yet) */
#define RTMT_H_I2C		0x34	/* ACQ-FIBER-HBA ONLY! */
#define RTMT_H_MBOX1		0x38
#define RTMT_H_MBOX2		0x3c



/* _C_ : COMMON */
#define RTMT_C_REVID		0x40	/* FPGA Revision Reg */
#define RTMT_C_DATA_FIFSTA	0x44
#define RTMT_C_PCI_CSR		0x48
#define RTMT_C_PCIE_DEV_CSR	0x4c
#define RTMT_C_PCIE_LNK_CSR	0x50
#define RTMT_C_PCIE_ERR_CNT	0x60
#define RTMT_C_PCIE_LAST_ERR	0x64
#define RTMT_C_PCIE_COMPLETION	0x68



/* _Q_ ACQ (IOP) Control Regs */
#define RTMT_Q_CSR		0x80
#define RTMT_Q_SPI		0x84
#define RTMT_Q_TEST		0xa0
#define RTMT_Q_FCR		0xa4
#define RTMT_Q_DIO		0xa8

#define RTMT_Q_SPI_CTL		0xac
#define RTMT_Q_SPI_DAT		0xb0
#define RTMT_Q_I2C		0xb4
#define RTMT_Q_MBOX1		0xb8
#define RTMT_Q_MBOX2		0xbc
#define	RTMT_Q_END		RTMT_Q_MBOX2

#define RTMT_H_AURORA0_CTRL	0xc0
#define RTMT_H_AURORA0_STAT	0xc4

/* HOST STATUS */

#define RTMT_H_IB_DMA_STAT	0x100
#define RTMT_H_IB_DMA_FIFSTA	0x104
#define RTMT_H_O1_DMA_STAT	0x108
#define RTMT_H_O1_DMA_FIFSTA	0x10c
#define RTMT_H_O2_DMA_STAT	0x110
#define RTMT_H_O2_DMA_FIFSTA	0x114

#define REGS_LEN	        0x118


/* BITS */

#define RTMT_H_PCIE_CSR_FPGA_FAMILY	0xff000000
#define RTMT_H_PCIE_CSR_INT_TYPE	0x00070000
#define RTMT_H_PCIE_CSR_VERSION		0x0000ff00
#define RTMT_H_PCIE_CSR_DMA_RESET	0x00000001	/* RW: 1: reset */

#define RTDMAC_RESET		RTMT_H_PCIE_CSR
#define RTDMAC_RESET_RBIT	RTMT_H_PCIE_CSR_DMA_RESET


/* IB: INBOUND, OB: OUTBOUND */
#define RTMT_H_DMAC_CTRL_IB_EN		(1<<0)
#define RTMT_H_DMAC_CTRL_OB1_EN		(1<<16)
#define RTMT_H_DMAC_CTRL_OB2_EN		(1<<24)

#define RTMT_D_FCR_HASIT	(1<<31)		/* RO 1: you have control */
/* unused 29-24 */
#define RTMT_D_FCR_PCIE_APP_RDY (1<<23)		/* RO 1: cable plugged in */
/* unused 23-16 */
#define RTMT_D_FCR_WANTIT	(1<<15)		/* 1: request control */
#define RTMT_D_FCR_SIM_THROTTLE_SHL 8
#define RTMT_D_FCR_SIM_THROTTLE_MSK 0xf		/* 0x0 : /1, 0x1 : /2 .. etc */
#define RTMT_D_FCR_SIM_MODE	(1<<7)		/* 1: Simulate on source=ctr */
#define RTMT_D_FCR_PCIE_DATAON	(1<<6)		/* 1: request data frm ACQ, 0: IOP-PUSH */
#define RTMT_D_FCR_LOWLAT	(1<<5)
/* unused 4-1 */
#define RTMT_D_FCR_DMA_SINGLE_RECYCLE	(1<<1)	/* 1: reuse #1 DMA descr forever */
#define RTMT_D_FCR_FIFO_RESET	(1<<0)		/* 1: reset COMMS FIFO Which One? */

#define RTMT_D_FCR_SIM_CLR	\
		(RTMT_D_FCR_SIM_MODE|RTMT_D_FCR_SIM_THROTTLE_MSK<<RTMT_D_FCR_SIM_THROTTLE_SHL)

#define RTMT_D_DIO_DRVON_SHL	24	/* 1: HD15 is OUTPUT */
#define RTMT_D_DIO_SETOUT_SHL	16	/* 1: FPGA is OUTPUT */
#define RTMT_D_DIO_OUTDAT_SHL	 8	/* output value set (when output) */
#define RTMT_D_DIO_INPDAT_SHL	 0

#define RTMT_DIO_MASK		0x00ff	/* 8 bits DIO */
#define MAXDIOBIT		8



/* 31-26 not used */
#define RTMT_Q_CSR_PCIe_RSTn		(1<<25)	/* RO 0: RESET active */
#define RTMT_Q_CSR_PCIe_PRSNT		(1<<24) /* RO 1: CABLE plugged in */
/* 23-18 not used */
#define RTMT_Q_CSR_PG_MGT		(1<<17)	/* RO 1: MGT POWER GOOD */
#define RTMT_Q_CSR_PG_DDR3		(1<<16)	/* RO 1: DDR POWER GOOD */
/* 15-10 not used */
#define RTMT_Q_CSR_PCIe_CABLE_WAKE	(1<<9)	/* QW: wake cable request */
#define RTMT_Q_CSR_PCIe_DRVR_PRSNT	(1<<8)	/* QW IWHAT? */
/* 7-1 not used */
#define RTMT_Q_CSR_BURST		(1<<0)	/* QW PBI Turbo mode */

/* RTMT_H_XX_DMASTAT :  RTDMAC_DESC_ */

#define RTMT_H_XX_DMA_FIFSTA_COUNT_MSK	0xfff
#define RTMT_H_XX_DMA_FIFSTA_COUNT_SHL	4
#define RTMT_H_XX_DMA_FIFSTA_FULL	(1<<1)
#define RTMT_H_XX_DMA_FIFSTA_EMPTY	(1<<0)


#define RTMT_Q_SPI_BUSY			(1<<31)
#define RTMT_Q_SPI_CTL_START		(1<<7)
#define RTMT_Q_SPI_CS			(1<<0)
#define RTMT_Q_SPI_HOLD			(1<<1)
#define RTMT_Q_SPI_WP			(1<<2)


#define RTMT_D_AURORA_CTRL_RESET	(1<<31)
#define RTMT_D_AURORA_CTRL_CLR		(1<<7)
#define RTMT_D_AURORA_CTRL_PWR_DWN	(1<<4)
#define RTMT_D_AURORA_CTRL_LOOPBACK	(0x7)

#define RTMT_D_AURORA_STAT_SFP_PRESENTn	(1<<31)
#define RTMT_D_AURORA_STAT_SFP_LOS	(1<<30)
#define RTMT_D_AURORA_STAT_SFP_TX_FAULT (1<<29)
#define RTMT_D_AURORA_STAT_HARD_ERR	(1<<6)
#define RTMT_D_AURORA_STAT_SOFT_ERR	(1<<5)
#define RTMT_D_AURORA_STAT_FRAME_ERR	(1<<4)
#define RTMT_D_AURORA_STAT_CHANNEL_UP	(1<<1)
#define RTMT_D_AURORA_STAT_LANE_UP	(1<<0)

#define RTMT_Q_I2C_SCL_R		0
#define RTMT_Q_I2C_SDA_R		1
#define RTMT_Q_I2C_SCL_W		8
#define RTMT_Q_I2C_SDA_W		9

#define RTMT_I2C_W(R)			((R)+8)


#define RTMT_H_I2C_SCL1_R		0
#define RTMT_H_I2C_SDA1_R		1
#define	RTMT_H_I2C_SCL1_W		8
#define RTMT_H_I2C_SDA1_W		9
#define RTMT_H_I2C_SCL2_R		16
#define RTMT_H_I2C_SDA2_R		17
#define	RTMT_H_I2C_SCL2_W		24
#define RTMT_H_I2C_SDA2_W		25


#define UART_BAR	4
#define FIFO_BAR	2
#define REGS_BAR	0
#define NO_BAR		-1
#define PBI_BAR		1		/* PBI_BAR .. AO32 regs */
#define MAP_COUNT	5


#define FIFO_OFFSET_INBOUND	0x000
#define FIFO_OFFSET_OUTBOUND1	0x800
#define FIFO_OFFSET_OUTBOUND2	0xc00

/* we wanted an elegant multi-vector solution, but we can only have one int */
#define MSI_DMA		0
#ifdef MSI_BLOCK_WORKS
#define MSI_UART	1
#else
#define MSI_UART	0
#endif

#define EMPTY1	0xee11ee11
#define EMPTY2  0x22ee22ee

#define MAGIC1	0xdeadbeef
#define MAGIC2	0xcafebabe

#define RTDMAC_DATA_FIFO_CNT	0x1000
#define RTDMAC_DESC_FIFO_CNT	0x1000

#define DATA_FIFO_SZ	(RTDMAC_DATA_FIFO_CNT*sizeof(unsigned))
#define DESC_FIFO_SZ	(RTDMAC_DESC_FIFO_CNT*sizeof(unsigned))

#define UART_LEN	512

/* it's actually double, but then we'd have to update stty */
#define RTM_T_UART_XTAL	(14181800)


#define FPGA_ID	0		/* S6 unique ID TBA */


/* LLC SIGNALING */


#define H_MBOX1_LLC_CMD		0xffff0000
#define H_MBOX1_LLC_RUN		0x11c10000
#define H_MBOX1_LLC_STOP	0x11c00000

#define Q_MBOX1_LLC_ACK		0xFFFF0000
#define Q_MBOX1_LLC_SHOT	0x0000ffff

#define __MAKE_LLC_CMD(cmd, clkdiv, clk_pos, trg_pos) \
	((cmd) | ((clkdiv)&0x00ff) |\
	 ((clk_pos)? 0x00008000:0) | ((trg_pos)? 0x00004000:0))

#define MAKE_LLC_RUN(clkdiv, clk_pos, trg_pos) \
		__MAKE_LLC_CMD(H_MBOX1_LLC_RUN, clkdiv, clk_pos, trg_pos)

#define DECODE_LLC_CMD(mbox, clkdiv, clk_pos, trg_pos) \
	do { \
		clkdiv = (mbox)&0x00ff; \
		clk_pos = ((mbox)&0x00008000) != 0; \
		trg_pos = ((mbox)&0x00004000) != 0; \
	} while(0)

#define IS_LLC_RUN(mbox)	(((mbox)&H_MBOX1_LLC_CMD)==H_MBOX1_LLC_RUN)
#define IS_LLC_STOP(mbox)	(((mbox)&H_MBOX1_LLC_CMD)==H_MBOX1_LLC_STOP)


/*  Q_MBOX2 is LLC_TLATCH */
#endif /* __RTM_T_H__ */




