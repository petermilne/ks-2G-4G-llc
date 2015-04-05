/* ------------------------------------------------------------------------- */
/* ao32cpci.h AO32 CPCI register model			                     */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2009 Peter Milne, D-TACQ Solutions Ltd
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


#ifndef __AO32CPCI_H__
#define __AO32CPCI_H__


/* REGS */

#define	AO32_DEBUG	0x0000	/* deadbeef */
#define AO32_FIFCON	0x0004  /* FIFO Control */
#define AO32_FIFSTA	0x0008	/* FIFO Status */
#define AO32_SYSCON	0x000c  /* SYSTEM Control */

#define AO32_AO_CLKCON	0x001c	/* AO Clock Control */
#define AO32_DO_CLKCON	0x0020	/* DO Clock Control */
#define AO32_DIOCON	0x0024  /* DIO (PXI) control */

#define AO32_DO0132	0x0030
#define AO32_DO3364	0x0034

#define AO32_REVID	0x0028
#define AO32_UNIPOLAR	0x002c
#define AO32_CTRL	0x0030

#define AO32_ICR	0x0080	/* Interrupt Control Reg */

#define AO32_AODIRECT	0x100	/* pgm recommendation */


/* offsets in MAP_FIFO: */
#define FIFO_AO		0x0800
#define FIFO_DO		0x0400
#define FIFO_LL		0x0000



#define DIWORDS	2
#define DIBYTES (DIWORDS*sizeof(u32))
#define MAXDIOBIT 32

#define LLC_LEN	(32*sizeof(short) + 8*sizeof(char))

#define AO32_FIFCON_LLC_RESET	0x00400000
#define AO32_FIFCON_LLC_ENABLE	0x00200000

#define AO32_FIFCON_DO_IE	0x00008000
#define AO32_FIFCON_DO_RESET	0x00004000
#define AO32_FIFCON_DO_ENABLE	0x00002000
#define AO32_FIFCON_DO_LTIDE	0x00001f00

#define AO32_FIFCON_AO_IE	0x00000080
#define AO32_FIFCON_AO_RESET	0x00000040
#define AO32_FIFCON_AO_ENABLE	0x00000020
#define AO32_FIFCON_AO_LTIDE	0x0000001f

/* d31 not used */
//				  20200a20
#define AO32_FIFSTA_LLC_FF	0x40000000
#define AO32_FIFSTA_LLC_NE	0x20000000
#define AO32_FIFSTA_LLC_PTR	0x1f000000

#define AO32_FIFSTA_DO_TR	0x00800000
/* d22 not used */
#define AO32_FIFSTA_DO_HT	0x00200000
#define AO32_FIFSTA_DO_OVER	0x00100000
#define AO32_FIFSTA_DO_UNDER	0x00080000
#define AO32_FIFSTA_DO_FF	0x00040000
#define AO32_FIFSTA_DO_NE	0x00020000
#define AO32_FIFSTA_DO_PTR	0x0001f000

#define AO32_FIFSTA_AO_TR	0x00000800
/* d10 not used */
#define AO32_FIFSTA_AO_HT	0x00000200
#define AO32_FIFSTA_AO_OVER	0x00000100
#define AO32_FIFSTA_AO_UNDER	0x00000080
#define AO32_FIFSTA_AO_FF	0x00000040
#define AO32_FIFSTA_AO_NE	0x00000020
#define AO32_FIFSTA_AO_PTR	0x0000001f

/* generic flags XO << SHL */
#define AO32_FIFSTA_XO_TR	0x00000800
/* d10 not used */
#define AO32_FIFSTA_XO_HT	0x00000200
#define AO32_FIFSTA_XO_OVER	0x00000100
#define AO32_FIFSTA_XO_UNDER	0x00000080
#define AO32_FIFSTA_XO_FF	0x00000040
#define AO32_FIFSTA_XO_NE	0x00000020
#define AO32_FIFSTA_XO_PTR	0x0000000f

#define AO32_FIFSTA_AO_SHL	0
#define AO32_FIFSTA_DO_SHL	12
#define AO32_FIFSTA_LL_SHL	24


#define FIFOBITS	16

#define AO32_SYSCON_DO_OE	0x80000000
#define AO32_SYSCON_DO_MODE	0x70000000
#define AO32_SYSCON_DO_TRG_POS	0x08000000
#define AO32_SYSCON_DO_TRG_SEL	0x07000000

#define AO32_SYSCON_DO_EC_POS	0x00800000
#define AO32_SYSCON_DO_EC_SEL	0x00700000

#define AO32_SYSCON_DO_EXTCLK	0x00080000
#define AO32_SYSCON_DO_CLKVAL	0x00030000
#define AO32_SYSCON_DO_TRIGGRD  0x00020000
#define AO32_SYSCON_DO_ENABLE	0x00010000


#define AO32_SYSCON_DAC_CLR_N	0x00008000
#define AO32_SYSCON_AO_MODE	0x00007000
#define AO32_SYSCON_AO_TRG_POS	0x00000800
#define AO32_SYSCON_AO_TRG_SEL	0x00000700

#define AO32_SYSCON_AO_EC_POS	0x00000080
#define AO32_SYSCON_AO_EC_SEL	0x00000070
#define AO32_SYSCON_AO_EXTCLK	0x00000008
#define AO32_SYSCON_AO_CLKVAL	0x00000004
#define AO32_SYSCON_AO_TRIGGRD  0x00000002
#define AO32_SYSCON_AO_ENABLE	0x00000001



#define AO32_SYSCON_DO_SHL	16




#define AO32_SYSCON_AO_SHL	0

#define AO32_SYSCON_DO_MODE_SHL 28
#define AO32_SYSCON_DO_TRG_SEL_SHL	24
#define AO32_SYSCON_DO_EC_SEL_SHL	20
#define AO32_SYSCON_AO_MODE_SHL		12
#define AO32_SYSCON_AO_TRG_SEL_SHL	 8
#define AO32_SYSCON_AO_EC_SEL_SHL	 4

#define AO32_SYSCON_XO_ENABLE \
	(AO32_SYSCON_AO_ENABLE|AO32_SYSCON_DO_ENABLE| \
	AO32_SYSCON_DO_OE|AO32_SYSCON_DAC_CLR_N)

#define AO32_SYSCON_XO_DISABLE \
	(AO32_SYSCON_AO_ENABLE|AO32_SYSCON_DO_ENABLE|AO32_SYSCON_DAC_CLR_N)

/* leaves us with nowhere to put SOFT_TRIG
 * ALSO, is there a separate DO_SOFT_TRIG, AO_SOFT_TRIG?
 * Peter Recommends: move the RO TRIGGRD bits into FIFSTA spares
 * OR - there are already TR bits in the FIFSTA - maybe the _SAME_ function
 * duplicated?

eg

These two the same?
#define AO32_FIFSTA_DO_TR	0x00800000
#define AO32_SYSCON_DO_TRIGGRD  0x00020000

And these two:
#define AO32_FIFSTA_AO_TR	0x00000800
#define AO32_SYSCON_AO_TRIGGRD  0x00000002

In that case simply drop the SYSCON bits (use for SOFT_TRIG)
and we're done

#define AO32_SYSCON_DO_SOFT_TRG 0x00020000
#define AO32_SYSCON_AO_SOFT_TRG 0x00000002

... and how would SOFT_TRIG work - momentarily hi?

 */
#define AO32_SYSCON_SOFT_TRIG	0x00008000
enum Sel {
	S_PXI_0,
	S_PXI_1,
	S_LEMO_CLK_DIRECT,
	S_LEMO_CLK_OPTO,
	S_PXI_3,
	S_PXI_4,
	S_LEMO_TRG_DIRECT,
	S_LEMO_TRG_OPTO
};

enum Mode {
	M_RIM,	/* Registered Immediate Mode */
	M_RTU,	/* Registered Triggered Update */
	M_AWGI,	/* AWG Immediate */
	M_AWGT,	/* AWG Triggered */
	M_LLI,	/* Low Latency Immediate */
	M_LLC	/* Low Latency Clocked */
};


#define AO32_AO_CLKCON_SOFTCLK_MODE	0x00000200
#define AO32_AO_CLKCON_CSEXT		0x00000080
#define AO32_AO_CLKCON_CS_SEL		0x00000070
#define AO32_AO_CLKCON_CLKMAS		0x00000008
#define AO32_AO_CLKCON_OCSMASK		0x00000007

#define AO32_AO_CLKCON_CS_SEL_SHL		4

#define AO32_DO_CLKCON_SOFTCLK_MODE	0x00000200
#define AO32_DO_CLKCON_CSEXT		0x00000080
#define AO32_DO_CLKCON_CS_SEL		0x00000070
#define AO32_DO_CLKCON_CLKMAS		0x00000008
#define AO32_DO_CLKCON_OCSMASK		0x00000007

#define AO32_DO_CLKCON_CS_SEL_SHL		4

#define AO32_CLKCON_CLKDAT_MASK		0xffff0000
#define AO32_CLKCON_CLKDAT_SHL		16

#define AO32_CLKCON_DIV_MAX		0x0ffff
#define AO32_DIOCON_INPDAT_SHL		0
#define AO32_DIOCON_OUTDAT_SHL		8
#define AO32_DIOCON_SETOUT_SHL		16

#define AO32_DIOCON_INPDAT		0x000000ff

#define AO32_DIOCON_INPBIT(pat) (1 << S_##pat)
#define AO32_DIOCON_OUTBIT(pat) (0x100 << S_##pat)
#define AO32_DIOCON_SETOUT_BIT(pat) (0x10000 << S_##pat)

#define AO32_ICR_INTEN	0x80

#define AO32_SET_AO32DIRECT(regs, ch, yy)


#define AO_FIFO_LEN	8192
#define DO_FIFO_LEN	8192
#define NHISTO		0x10

#define AO_FIFO_STEPSZ	(AO_FIFO_LEN/NHISTO)
#define DO_FIFO_STEPSZ	(DO_FIFO_LEN/NHISTO)

#define AO_WINDOW_SZ	2048
#define DO_WINDOW_SZ	1024
#define IDEAL_DMA_LEN	1024

struct FunctionBuf {
	void *p_start;
	void *p_limit;
	void *p_cursor;
	int wsize;
	int len;
};




#endif /* __AO32CPCI_H__ */
