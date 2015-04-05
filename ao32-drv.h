/* ------------------------------------------------------------------------- */
/* file ao32-drv.h                                                                 */
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

/** @file ao32-drv.h DESCR
 *
 */

#ifndef AO32_DRV_H_
#define AO32_DRV_H_

#include "rtm-t-hostdrv.h"

void ao32_onStart_LLC(struct RTM_T_DEV *tdev);
void ao32_onStop_LLC(struct RTM_T_DEV *tdev);
#endif /* AO32_DRV_H_ */
