/* rtm-t-dmac.h RTM-T Driver common defs				     */
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

/** @file rtm-t-dmac.h . */

#ifndef RTM_T_DMAC_H_
#define RTM_T_DMAC_H_

#define RTDMAC_PAGE		0x400		/* 1K pages */
#define RTDMAC_LL_PAGE		64		/* page length in LL */

#define RTDMAC_DESC_ADDR_MASK	0xfffffc00	/* base address */

#define RTDMAC_DESC_WRITE	0x00000200	/* Write BIT */
#define RTDMAC_DESC_EOT		0x00000100	/* End Of Transfer interrupt */

#define RTDMAC_DESC_LEN_MASK	0x000000f0	/* length (pages) */
#define RTDMAC_DESC_LEN_SHL	4

#define RTDMAC_DESC_ID_MASK	0x0000000f	/* ID 0..15 */

#define RTDMAC_DESC_ID		0x0000000f	/* ID 0..15 */

#define AODMAC_PAGE		0x400
#define AODMAC_LL_PAGE		8		/* bytes per unit */

#define RTDMAC_DESCR(pa, pages, id)	\
	(((pa)&RTDMAC_DESC_ADDR_MASK)|	\
	((((pages)-1) << RTDMAC_DESC_LEN_SHL)&RTDMAC_DESC_LEN_MASK)| \
	(id))


#endif /* RTM_T_DMAC_H_ */
