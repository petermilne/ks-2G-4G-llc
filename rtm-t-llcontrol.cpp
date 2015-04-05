/* ------------------------------------------------------------------------- */
/* file rtm-t-llcontrol.cpp						     */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2011 Peter Milne, D-TACQ Solutions Ltd
 *                      <Peter dot Milne at D hyphen TACQ dot com>

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

/** @file rtm-t-llcontrol.cpp demonstrates llcontrol with ACQ196/RTM-T.
 * llcontrol with rtm-t is a simplified version of ACQ196/SYNC2V.
 - data direction is INPUT ONLY.
 - on the ACQ196 UUT, run /usr/local/CARE/rtm_t_llc_debug
 - please follow the example setup script
  - ./test-scripts/setup.rtm-t-llcontrol.d3
 - Operation:
  -  LLC is controlled by a single ioctl() RTM_T_START_LLC.
  -  Subsequently all control is in application space.
  -  By default, the user app defines the driver Host Buffer 0 as host target buffer,
   however, it's possible to specify any physical address bys etting the target_pa
   field in struct LLC_DEF
  -  The ioctl call arms the ACQ196.
  -  Optionally, capture is triggered by setting a DIO line via the PCIe link,
   or a physical front panel DIO line can be used.
  -  The user app polls the TLATCH field in the host buffer.
  - The program includes attempts to improve real time performance in
  modern host kernels.
*/
using namespace std;

#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>

#include <sched.h>
#include "RTM_T_Device.h"
#include "local.h"
#include "popt.h"

#include <sched.h>

#include "rtm-t_ioctl.h"

/* default: never completes */
int MAXITER	       = 0xffffffff;
int MAXITER_MASK       = 0x7fffffff;



int NELEMS = RTM_T_Device::MAXBUF;
int USLEEP = 0;
int VERBOSE = 1;

int SSIZE = sizeof(short) * 96;

int yield = 1;
int trigger_bit = -1;
int sched_fifo_priority = 0;
int acq200_debug = 0;

int maxlen = RTM_T_Device::MAXLEN;
#define BUFFER_LEN "/sys/module/rtm_t/parameters/buffer_len"

const char* OUTROOT = "/mnt";
#define LLCV2_STATUS_TLATCH 6     /** TLATCH {11:0} */

#define TLATCH_OFFSET ((3 * 16) + LLCV2_STATUS_TLATCH)	/* longwords */
static RTM_T_Device *dev;

int samples = 100;

struct LLC_DEF llc_def = {
	100, 0, 0, 0,
	RTM_T_USE_HOSTBUF
};

/*
 * WARNING: SERVICE_ macros snipped from llc.ko
 * duplicate assignment statement adds a conditional bitop, but loses
 * an expensive str/ldr. Way to go!
 */

#define ACQ196_TCR_MASK 0x00000fff

#define SERVICE_ROLLOVER(tim, reg, mask, temp)				\
	temp = (reg) & (mask);						\
	if (((tim) & (mask)) > (temp)){					\
		(tim) = (((tim) & ~(mask)) | (temp)) + ((mask)+1);	\
	}else{								\
		(tim) = (((tim) & ~(mask)) | (temp));			\
	}

u32 llv2_extend32(u32 old32, u32 new12)
/** return 32 bit count as function of old32, new12. */
{
	u32 y = old32;
	u32 t;

	SERVICE_ROLLOVER(y, new12, ACQ196_TCR_MASK, t);
	return y;
}

void goRealTime(void)
{
	struct sched_param p;
	p.sched_priority = sched_fifo_priority;


	int rc = sched_setscheduler(0, SCHED_FIFO, &p);

	if (rc){
		perror("failed to set RT priority");
	}
}
void auto_trigger(int sense)
/* sense -> 1 triggers */
{
	char dio_knob[80];

	sprintf(dio_knob, "%s/dio_bit_%d", dev->getControlRoot(), trigger_bit);
	FILE *fp = fopen(dio_knob, "w");
	if (!fp){
		perror("failed to open knob");
		exit(-1);
	}
	fprintf(fp, "%d\n", sense);
	fclose(fp);
}
int llcontrol() {
	volatile u32 *hb = (volatile u32 *)dev->getHostBufferMapping(0);
	u32 tlatches[3];
#define tlatch 	tlatches[1]	// u32 extended tlatch
#define tlatch1 tlatches[0]	// 12 bit raw tlatch
#define tlatch2 tlatches[2]     // raw tlatch detects change of sample

	if (trigger_bit >= 0){
		auto_trigger(0);
	}
	u32* tlbuffer = (u32*)calloc(samples*2, sizeof(u32));

	mlockall(MCL_CURRENT);

	if (sched_fifo_priority){
		goRealTime();
	}

	tlatch2 = hb[TLATCH_OFFSET] = 0xdeadbeef;

	int rc = ioctl(dev->getDeviceHandle(), RTM_T_START_LLC, &llc_def);

	dbg(1, "ioctl complete ..");

	if (rc != 0){
		err("ioctl RTM_T_START_LLC failed %d", rc);
		_exit(-rc);
	}

	for (int sample = 0; sample < samples; ++sample){
		while ((tlatch1 = hb[TLATCH_OFFSET]) == tlatch2){
/*
			if (yield){
				sched_yield();
			}
*/
			if (sample == 0){
				if (trigger_bit >= 0){
					auto_trigger(1);
					trigger_bit = -1;
				}
			}
		}
		if (sample == 0){
			tlatch = 0;
		}
		tlatch = llv2_extend32(tlatch, tlatch1);
		//tlatch = tlatch1;
		memcpy(tlbuffer+sample*2, tlatches, sizeof(u32)*2);
		//fwrite(tlatches, sizeof(u32), 2, fp);
		dbg(2, "%d %u", sample, tlatch);
		tlatch2 = tlatch1;
short *sptr;
sptr = (short *)&hb[0];
//fprintf(stderr,">> %d: %d %d\n",tlatch,sptr[0],sptr[1]);
	}

	munlockall();

	FILE* fp = fopen("/tmp/tlatch", "w");
	if (!fp){
		perror("failed to open /tmp/tlatch");
	}
	fwrite(tlbuffer, sizeof(u32), 2*samples, fp);
	fclose(fp);
	free(tlbuffer);
	dbg(1, "99");
}
static void init_defaults(int argc, char* argv[])
{
	int devnum = 0;

	int nbuffers = 1;

	if (getenv("RTM_DEVNUM")){
		devnum = atol(getenv("RTM_DEVNUM"));
	}
	dev = new RTM_T_Device(devnum, nbuffers);

	if (getenv("RTM_MAXITER")){
		MAXITER = atol(getenv("RTM_MAXITER"));
		printf("MAXITER set %d\n", MAXITER);
	}
	if (getenv("RTM_DEBUG")){
		acq200_debug = atol(getenv("RTM_DEBUG"));
		printf("DEBUG set %d\n", acq200_debug);
	}
	if (getenv("RTM_USLEEP")){
		USLEEP = atol(getenv("RTM_USLEEP"));
		printf("USLEEP set %d\n", USLEEP);
	}
	if (getenv("RTM_VERBOSE")){
		VERBOSE = atol(getenv("RTM_VERBOSE"));
		printf("VERBOSE set %d\n", VERBOSE);
	}
	if (getenv("SSIZE")){
		SSIZE = atoi(getenv("SSIZE"));
		printf("SSIZE set %d\n", SSIZE);
	}

	if (getenv("CLKDIV")){
		llc_def.clk_div = atoi(getenv("CLKDIV"));
	}
	if (getenv("OUTROOT")){
		OUTROOT=getenv("OUTROOT");
	}
	if (getenv("SAMPLES")){
		samples = atoi(getenv("SAMPLES"));
	}
	if (getenv("YIELD")){
		yield = atoi(getenv("YIELD"));
	}
	if (getenv("TRIGGER_BIT")){
		trigger_bit = atoi(getenv("TRIGGER_BIT"));
	}
	if (getenv("SCHED_FIFO")){
		sched_fifo_priority = atoi(getenv("SCHED_FIFO"));
	}

	FILE *fp = fopen(BUFFER_LEN, "r");
	if (!fp){
		perror(BUFFER_LEN);
		exit(errno);
	}
	if (fscanf(fp, "%d", &maxlen) == 1){
		printf("maxlen set %d", maxlen);
	}else{
		err("maxlen not set");
	}
	fclose(fp);
}
int main(int argc, char* argv[])
{
	init_defaults(argc, argv);
	return llcontrol();
}
