/* ------------------------------------------------------------------------- */
/* file rtm-t-feedback_1AI_2AO.cpp						 */
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

/** @file rtm-t-feedback_1AI_2AO.cpp demonstrates llcontrol with ACQ196/RTM-T.
 * llcontrol with rtm-t is a simplified version of ACQ196/SYNC2V.
 - USAGE: rmt-t-feedback AISLOT AOSLOT1 AOSLOT2
 - Optionally outputs 2 channel AWG on AO01, AO02.
 - Feeds back AI01, AI02 to AO03-AO32
 - Best usage is to connect a signal to AI01, and monitor output on AO03
 - on the ACQ196 UUT, run /usr/local/CARE/rtm_t_llc_debug
 - please follow the example setup script
  - ./test-scripts/setup.rtm-t-feedback.d3
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

#include <assert.h>
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

int feed_forward = 0; 	/* generate own waveform */

int maxlen = RTM_T_Device::MAXLEN;
#define BUFFER_LEN "/sys/module/rtm_t/parameters/buffer_len"

const char* OUTROOT = "/mnt";
#define LLCV2_STATUS_TLATCH 6     /** TLATCH {11:0} */

#define TLATCH_OFFSET ((3 * 16) + LLCV2_STATUS_TLATCH)	/* longwords */


static RTM_T_Device *ai_dev;
static RTM_T_Device *ao_dev1;
static RTM_T_Device *ao_dev2;

u32 *awg;
int nawg;

int samples = 100;


/**
 * When using an external device eg CUDA, set buffer pa
 * (1), (2) to absolute device pa's.
 * NB: MUST BE 32-bit addressable.
 */
struct LLC_DEF llc_def = {
	100, 0, 0, 0,
	RTM_T_USE_HOSTBUF	/* src_pa (1) */
};

struct AO_LLC_DEF aollc_def1 = {
	32 * sizeof(short) + 2 * sizeof(u32),
	RTM_T_USE_HOSTBUF	/* target pa (2) */
};

struct AO_LLC_DEF aollc_def2 = {
	32 * sizeof(short) + 2 * sizeof(u32),
	RTM_T_USE_HOSTBUF	/* target_pa (2) */
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

void cpyfill(volatile u32* dst, u32 src, int nwords)
{
	while(nwords--){
		*dst++ = src;
	}
}

void auto_trigger(int sense)
/* sense -> 1 triggers */
{
	char dio_knob[80];

	sprintf(dio_knob, "%s/dio_bit_%d", ai_dev->getControlRoot(), trigger_bit);
	FILE *fp = fopen(dio_knob, "w");
	if (!fp){
		perror(dio_knob);
		exit(-1);
	}
	fprintf(fp, "%d\n", sense);
	fclose(fp);
}
#define PAT_STEP (32000/32)

void test_pattern(short* hb, short x1, int nchan)
{
	int ic;
	for (ic = 0; ic < nchan; ++ic){
		hb[ic] = x1 + ic*PAT_STEP;
	}
}

int llcontrol() {
	volatile u32 *hb = (volatile u32 *)ai_dev->getHostBufferMapping();
	volatile u32 *ao_hb1 = (volatile u32*)ao_dev1->getHostBufferMapping();
	volatile u32 *ao_hb2 = (volatile u32*)ao_dev2->getHostBufferMapping();
	u32 tlatches[3];
#define tlatch 	tlatches[1]	// u32 extended tlatch
#define tlatch1 tlatches[0]	// 12 bit raw tlatch
#define tlatch2 tlatches[2]     // raw tlatch detects change of sample

	if (trigger_bit >= 0){
		auto_trigger(0);
	}
	u32* tlbuffer = (u32*)calloc(samples*2, sizeof(u32));
	bool do_test_pattern = false;

	mlockall(MCL_CURRENT);

	if (sched_fifo_priority){
		goRealTime();
	}
	if (getenv("TEST_PATTERN")){
		do_test_pattern = atoi(getenv("TEST_PATTERN"));
	}

	ao_hb1[0] = ao_hb2[0] = awg[0];
	tlatch2 = hb[TLATCH_OFFSET] = 0xdeadbeef;

	int rc1 = ioctl(ao_dev1->getDeviceHandle(), RTM_T_START_AOLLC, &aollc_def1);
	dbg(1, "ioctl complete ..");
	if (rc1 != 0){
		err("ioctl RTM_T_START_LLC failed %d", rc1);
		_exit(-rc1);
	}
	int rc2 = ioctl(ao_dev2->getDeviceHandle(), RTM_T_START_AOLLC, &aollc_def2);
	dbg(1, "ioctl complete ..");
	if (rc2 != 0){
		err("ioctl RTM_T_START_LLC failed %d", rc2);
		_exit(-rc2);
	}

	int rc = ioctl(ai_dev->getDeviceHandle(), RTM_T_START_LLC, &llc_def);
	dbg(1, "ioctl complete ..");
	if (rc != 0){
		err("ioctl RTM_T_START_LLC failed %d", rc);
		_exit(-rc);
	}

	for (int sample = 0; sample < samples; ++sample){
		while ((tlatch1 = hb[TLATCH_OFFSET]) == tlatch2){
			if (yield){
				sched_yield();
			}
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

		if (do_test_pattern){
			test_pattern((short*)ao_hb1, *(short*)hb, 32);
			test_pattern((short*)ao_hb2, *(short*)hb, 32);
		}else{
			/* this is the feedback: */
			cpyfill(ao_hb1, hb[0], 16);
			cpyfill(ao_hb2, hb[0], 16);
		}

		if (feed_forward){
		/* feedforward waveform */
			ao_hb1[0] = awg[(sample+1)%nawg];
			ao_hb2[0] = awg[(sample+1)%nawg];
		}
		//fwrite(tlatches, sizeof(u32), 2, fp);
		dbg(2, "%d %u", sample, tlatch);
		tlatch2 = tlatch1;
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

static void init_awg_default(void)
{
	const int NP = 16;
	const int STEP = 1000;

	awg = new u32 [NP];
	nawg = NP;
	short xx = 0;

	for (int ii = 0; ii < NP; ++ii){
		awg[ii] = xx << 16 | xx;
		xx += STEP;
	}
}
static void init_awg_file(const char* awg_file)
{
	fprintf(stderr, "awg_file: feature not implemented\n");
	exit(1);
}
static void init_awg(const char* awg_file)
{
	if (awg_file == 0){
		init_awg_default();
	}else{
		init_awg_file(awg_file);
	}
}

/** @@todo ... zcopy didn't actually work .. not recommended */

void setup_zcopy()
/* configure ao to use ai buffer.
 * Actually not easy, since we don't have any PA's to go on
 [peter@krypton ~]$ cat /proc/driver/rtm-t/rtm-t.0/HostBuffers
ix va pa len req_len descr state
[00] f7e00000 37e00000 100000 000100 37e00130 BS_EMPTY
[01] f7f00000 37f00000 100000 000100 37f00131 BS_EMPTY
[02] f7800000 37800000 100000 000100 37800132 BS_EMPTY

 */
{
	char path[128];
	sprintf(path, "/proc/driver/rtm-t/rtm-t.%d/HostBuffers",
			ai_dev->getDevnum());
	FILE *fp = fopen(path, "r");
	char buf[128];
	bool pa_set = false;
	if (fp == 0){
		perror(path);
		exit(1);
	}
	while(fgets(buf, 128, fp)){
		int ix;
		unsigned va, pa;
		if (sscanf(buf, "[%2d] %x %x", &ix, &va, &pa) == 3){
			if (ix == 0){
				llc_def.target_pa = aollc_def1.src_pa = pa;
				printf("ao src_pa set to AI 0x%08x", pa);
				pa_set = true;
				break;
			}
		}
	}
	fclose(fp);

	if (!pa_set){
		fprintf(stderr, "setup_zcopy(), sorry failed to set pa\n");
	}
}
static void init_defaults(int argc, char* argv[])
{
	int nbuffers = 1;

	if (argc < 4){
		fprintf(stderr, "ERROR: usage rtm-t-feedback AI AO1 AO2");
	}
	int ai_devnum = atoi(argv[1]);
	int ao_devnum1 = atoi(argv[2]);
	int ao_devnum2 = atoi(argv[3]);

	assert(IN_RANGE(ai_devnum, 100, 110));
	assert(IN_RANGE(ao_devnum1, 100, 110));
	assert(IN_RANGE(ao_devnum2, 100, 110));
	assert(ai_devnum != ao_devnum1);
	assert(ai_devnum != ao_devnum2);

	ai_dev = new RTM_T_Device(ai_devnum, nbuffers);
	ao_dev1 = new RTM_T_Device(ao_devnum1, nbuffers);
	ao_dev2 = new RTM_T_Device(ao_devnum2, nbuffers);

	init_awg(getenv("AO_DATA"));

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
	if (getenv("FEED_FORWARD")){
		feed_forward = atoi(getenv("FEED_FORWARD"));
	}
	/* EXPERIMENT ALERT: the following two options do not seem to work */
	if (getenv("AI_CLK_RISING")){
		llc_def.clk_pos = 1;
	}
	if (getenv("ZERO_COPY")){
		setup_zcopy();
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
