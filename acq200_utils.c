/* acq200_utils - useful stuff */

#define CONFIG_GENERIC_HARDIRQS 1

#include <linux/module.h>

#include <linux/interrupt.h>

#include <linux/kernel.h> /* printk() */
#include <linux/fs.h>     /* everything... */
#include <linux/errno.h>  /* error codes */
#include <linux/types.h>  /* size_t */
#include <linux/proc_fs.h>
#include <linux/fcntl.h>        /* O_ACCMODE */
#include <linux/pci.h>
#include <linux/sched.h>


#include <asm/io.h>       /* ioremap()         */
#include <asm/irq.h>
#include <asm/uaccess.h>  /* VERIFY_READ|WRITE */
#include <asm/segment.h>  /* memcpy and such */


struct MASK_ITERATOR {
    unsigned mask;
    unsigned icursor;
    enum state { MI_IDLE, MI_STARTED, MI_FINISHED } state;
};

int hasNext( struct MASK_ITERATOR* it ) {
    unsigned next_cursor = 0;

    switch( it->state ){
    case MI_IDLE:
	next_cursor = 0x80000000;
	it->icursor = 31;
	break;
    case MI_STARTED:
	next_cursor = (1 << it->icursor) >> 1;
	break;
    case MI_FINISHED:
	return 0;
    }
    for ( ; next_cursor != 0; next_cursor >>= 1 ){
	if ( it->mask & next_cursor ){
	    return 1;
	}
    }

    return 0;
}
int getNext( struct MASK_ITERATOR* it ) {
    unsigned next_cursor = 0;

    switch( it->state ){
    case MI_IDLE:
	next_cursor = 0x80000000;
	it->state = MI_STARTED;
	it->icursor = 31;
	break;
    case MI_STARTED:
	next_cursor = 1 << --it->icursor;
	break;
    case MI_FINISHED:
	return -1;
    }
    for ( ; next_cursor != 0; next_cursor >>= 1, --it->icursor ){
	if ( it->mask & next_cursor ){
	    return it->icursor;
	}
    }

    it->state = MI_FINISHED;
    return -1;
}



int acq200_intsDisable( unsigned irqs )
/**< disable interrupts defined by clear bits in irqs */
{
    struct MASK_ITERATOR it = { irqs };

    while ( hasNext( &it ) ){
	disable_irq_nosync( getNext( &it ) );
    }

    return 0;
}
int acq200_intsEnable( unsigned irqs )
/**< enable interrupts defined by set bits in irqs */
{
    struct MASK_ITERATOR it = { irqs };

    while ( hasNext( &it ) ){
	enable_irq( getNext( &it ) );
    }

    return 0;
}
