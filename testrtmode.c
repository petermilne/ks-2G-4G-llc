/*
******************************************************************************
testrtmode.c

Diagnostic to check to see if any interrupts are occuring on a cpu
while it is in real-time mode.  This program can also be used to
test the effects of various interrupts on real-time performance.
It depends on the real-time module driver codes developed for
the pcs.

This program needs to be run under the root account and will attempt
to disable all interrupts listed in the /proc/interrupts file.
******************************************************************************
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include  <stdarg.h>//<varargs.h>
#include <values.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include<stdio.h>
#include<linux/unistd.h>
#include<unistd.h>
#include<sched.h>

//#define NUMSECONDS (10*60)
//#define NUMSECONDS (5*60)
//#define NUMSECONDS (30*60)
int NUMSECONDS;

#define THRESHOLD 1 /* microseconds */
#define MAX_SAMPLES 100000

// prototypes
int readticks(unsigned int *);
int set_affinity(int);

int
main(int argc, char **argv)
{
    int ii;
    unsigned int t_current,t_last,t_diff;
    unsigned int elapsed_hwclocks;
    unsigned int elapsed_milliseconds;
    unsigned int elapsed_seconds;
    int CLOCKMILLISECOND=0;
    int CLOCKMICROSECOND=0;
    int total_delays;
    int num_x_second_delays[6];
    unsigned int save_elapsed_seconds[MAX_SAMPLES];
    unsigned int save_elapsed_milliseconds[MAX_SAMPLES];
    unsigned int save_t_diffs[MAX_SAMPLES];
    int max_delay;

//    printf("%d\n",getpid());
    struct sched_param p;
    sched_getparam(getpid(),&p);
    sched_getparam(0,&p);
//    printf("sched priority: %d\n",p.sched_priority);

//    printf("setting to 99\n");
    p.sched_priority = 99;
    sched_setscheduler(0,SCHED_RR,&p);

    sched_getparam(0,&p);
  //  printf("sched priority: %d\n",p.sched_priority);

    int affinity;
    int cpu_mask,cpu_af_num;

    if((argc-1) >=1 )
        sscanf(argv[1],"%d",&cpu_af_num);
    else
    {
        printf("Enter CPU number to use (0 to N-1) -> ");
        scanf("%d",&cpu_af_num);
    }
    affinity=set_affinity(cpu_af_num);


    if((argc-1) >=2 )
        sscanf(argv[2],"%d",&NUMSECONDS);
    else
    {
        printf("Enter Number of Seconds to the Run Test -> ");
        scanf("%d",&NUMSECONDS);
    }

    {/* get CPU clock info */
        FILE *fp;
        char line[512];
        float clockmhz;

        fp = fopen("/proc/cpuinfo","r");
        if(fp == NULL)
        {
            printf("Error trying to determine cpu clock speed\n");
            exit(-1);
        }
        fgets(line,512,fp);
        while(!feof(fp))
        {
            if(strstr(line,"cpu MHz")!= 0)
            {
                sscanf(line,"%*s %*s %*s %f",&clockmhz);
//clockmhz=2930; //xxx
                CLOCKMICROSECOND = (int)clockmhz;
                CLOCKMILLISECOND = CLOCKMICROSECOND*1000;
                break;
            }
            fgets(line,512,fp);
        }
        fclose(fp);
    }

#if 0 
    printf("Starting test\n");
    printf("%s: CLOCKMS = %d, CLOCKUS = %d\n",
        argv[0],CLOCKMILLISECOND,CLOCKMICROSECOND);
    printf("CPU %d: Going into realtime mode for %d seconds\n",
                cpu_af_num,NUMSECONDS);
#endif

    total_delays = 0;
    max_delay = 0;
    for(ii=0;ii<6;ii++) num_x_second_delays[ii] = 0;

    {
        elapsed_hwclocks = 0;
        elapsed_seconds = 0;
        elapsed_milliseconds=0;
        readticks(&t_last);
        while(elapsed_seconds < NUMSECONDS)
        {
            readticks(&t_current);
            t_diff = t_current-t_last;
            elapsed_hwclocks += t_diff;
            t_last = t_current;

            if(elapsed_hwclocks > CLOCKMILLISECOND)
            {
                elapsed_hwclocks = 0;
                elapsed_milliseconds++;
                if(elapsed_milliseconds > 1000)
                {
                    elapsed_milliseconds = 0;
                    elapsed_seconds++;
                }
            }

            if(t_diff> THRESHOLD*CLOCKMICROSECOND)
            {
                if( (t_diff/CLOCKMICROSECOND) >= 1)
                {
                    if(total_delays<MAX_SAMPLES)
                    {
                        save_elapsed_seconds[total_delays] = elapsed_seconds;
                        save_elapsed_milliseconds[total_delays]=
                                elapsed_milliseconds;
                        save_t_diffs[total_delays] =
                                t_diff/CLOCKMICROSECOND;
                    }

                    total_delays++;

                    if ( (t_diff/CLOCKMICROSECOND) == 1)
                        num_x_second_delays[1]++;
                    else if ( (t_diff/CLOCKMICROSECOND) == 2)
                        num_x_second_delays[2]++;
                    else if ( (t_diff/CLOCKMICROSECOND) == 3)
                        num_x_second_delays[3]++;
                    else if ( (t_diff/CLOCKMICROSECOND) == 4)
                        num_x_second_delays[4]++;
                    else if ( (t_diff/CLOCKMICROSECOND) == 5)
                        num_x_second_delays[5]++;
                    else
                        num_x_second_delays[0]++;

                    if ( (t_diff/CLOCKMICROSECOND) > max_delay)
                        max_delay = (t_diff/CLOCKMICROSECOND);
                }
            }
        }
    }

#if 0
    for(ii=0;ii<5;ii++)
        printf("CPU %d : Duration %d : Num %dus delays = %d\n",
                cpu_af_num,
                NUMSECONDS,
                ii+1,
                num_x_second_delays[ii+1]
                );

    printf("CPU %d : Duration %d : Num oth delays = %d\n",
            cpu_af_num,
            NUMSECONDS,
            num_x_second_delays[0]
            );

    printf("CPU %d : Duration %d : Total delays = %d, Max delay = %d\n",
            cpu_af_num,
            NUMSECONDS,
            total_delays,max_delay);
#endif

    printf("CPU %d : Dur %d : Delays: ",cpu_af_num,NUMSECONDS);
    for(ii=0;ii<5;ii++) printf("%d ",num_x_second_delays[ii+1]);
    printf("%d ",num_x_second_delays[0]);
    printf("Tot: %d Max: %d\n",total_delays,max_delay);

    printf("%d %d ",cpu_af_num,NUMSECONDS);
    for(ii=0;ii<5;ii++) printf("%d ",num_x_second_delays[ii+1]);
    printf(" %d ",num_x_second_delays[0]);
    printf(" %d %d\n",total_delays,max_delay);

#if 0 
    for(ii=0;ii<total_delays;ii++)
    {
        if( (int)save_t_diffs[ii] > 3)
            printf("%d) %d microsecond delay detected at %d seconds %d milliseconds\n",
                ii,
                (int)save_t_diffs[ii],
                (int)save_elapsed_seconds[ii],
                (int)save_elapsed_milliseconds[ii]);
    }
#endif

    return(0);
}

/*
******************************************************************************
SECTION: real time access routines
******************************************************************************
*/

/*
******************************************************************************
from - http://www.agner.org/assem/pentopt.htm#29
Copyright ) 1996, 1999 by Agner Fog. Last modified 1999-01-18.

routine to read from a pentium internal 64 bit clock counter

The Pentium family of processors have an internal 64 bit
clock counter which can be read into EDX:EAX using the
instruction RDTSC (read time stamp counter). This is very
useful for testing exactly how many clock cycles a piece of
code takes.

readclock returns a cpu clock cycle count, for the 450MHz PIII
there should be approximately 450,000,000 cpu cycles or ticks
per second.
******************************************************************************
*/
#include<stdio.h>

#if defined(COMPMAIN)
main()
{
    unsigned int myt1,myt2;

    readclock(&myt1,&myt2);
    printf("%u %u\n",myt1,myt2);
}
#else

int
readticks(unsigned int *ticks)
{
    unsigned int myt1,myt2;

    readclock(&myt1,&myt2);
    *ticks = myt2;
}
#endif


unsigned int t1;
unsigned int t2;
readclock(unsigned int *return_t1,unsigned int *return_t2)
{
__asm__ __volatile__(
  "rdtsc\n" /* move a value in */
  "movl %edx,t1\n" /* upper half of 64 bit time */
  "movl %eax,t2\n" /* lower half of 64 bit time */
);
*return_t1 = t1;
*return_t2 = t2;
}

/*
******************************************************************************
SUBROUTINE: set_affinity

Set this process to run on the input processor number.
The processor numbers start with 0 going to N-1 processors.
******************************************************************************
*/
int set_affinity(int processor)
{
extern int sched_getaffinity();
extern int sched_setaffinity();

unsigned long new_mask;

   unsigned int len = sizeof(new_mask);
   unsigned long cur_mask;
   pid_t p = 0;
   int ret;

   new_mask = 1<<(processor);

  //printf("set_affinity: %ld\n",new_mask);

   ret = sched_getaffinity(p, len, &cur_mask);
  // printf("sched_getaffinity = %d, cur_mask = %08lx\n",ret,cur_mask);
   if(ret != 0) abort();

   ret = sched_setaffinity(p, len, &new_mask);
  // printf("sched_setaffinity = %d, new_mask = %08lx\n",ret,new_mask);
   if(ret != 0) abort();

   ret = sched_getaffinity(p, len, &cur_mask);
  // printf("sched_getaffinity = %d, cur_mask = %08lx\n",ret,cur_mask);
   if(ret != 0) abort();
   if(cur_mask != new_mask)
   {
      printf("affinity did not get set! exiting\n");
      exit(-1);
   }
   fflush(stdout);

   return 0;
}
