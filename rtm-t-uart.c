/* ------------------------------------------------------------------------- */
/* rtm-t-uart.c RTM-T comms UART					     */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2010 Peter Milne, D-TACQ Solutions Ltd
 *                      <Peter dot Milne at D hyphen TACQ dot com>
                                                                               
    This program is fre software; you can redistribute it and/or modify
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


/** @file rtm-t-uart.c RTM-T comms uart */

/*
 * This code borrowed from 8250_pci.c ..
 * maybe rtm-t should end up in 8250_pci.c but then we'd have two
 * device drivers matching the same pci ident would that work?
 */

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
#define acq200_debug rtm_t_debug
#include "acq200_debug.h"
#include "acq200.h"

#include <linux/serial_core.h>
#include <linux/serial_8250.h>
#include <linux/8250_pci.h>	/* struct pciserial_board{} */

/* RTM-T contains a UART, but it's not a UART, so adding to 8250_pci.c
 * is not an option.
 * Instead, we have to emulate:

static int __devinit
pciserial_init_one(struct pci_dev *dev, const struct pci_device_id *ent)

and

static void __devexit pciserial_remove_one(struct pci_dev *dev)
{

*/


struct serial_private {
	struct pci_dev		*dev;
	unsigned int		nr;
	void __iomem		*remapped_bar;
//	struct pci_serial_quirk	*quirk;
	int			line[0];
};

#ifdef MONITOR_TRAFFIC
static inline int map_8250_in_reg(struct uart_port *p, int offset)
{
	return offset;
}

static inline int map_8250_out_reg(struct uart_port *p, int offset)
{
	return offset;
}



static void mem32_serial_out(struct uart_port *p, int offset, int value)
{
	static int iter;
	int reg_off = offset;
	offset = map_8250_out_reg(p, offset) << p->regshift;

	writel(value, p->membase + offset);

	if (iter++ < 100){
		dbg(2, "[%02x] {%02x} write  %02x", 
			reg_off, offset, value&0x00ff);

		if (iter == 100){
			dbg(2, "stopping messages");
		}
	}
}

static unsigned int mem32_serial_in(struct uart_port *p, int offset)
{
	static int iter;
	int reg_off = offset;
	unsigned int rv;

	offset = map_8250_in_reg(p, offset) << p->regshift;
	rv = readl(p->membase + offset);

	if (iter++ < 100){
		dbg(2, "[%02x] {%02x} read  %02x", reg_off, offset, rv&0x00ff);
		if (iter == 100){
			dbg(2, "stopping messages");
		}
	}

	return rv;
}
#endif

static int
setup_port(struct serial_private *priv, struct uart_port *port,
	   int bar, int offset, int regshift)
{
	struct pci_dev *dev = priv->dev;
	unsigned long base, len;

	base = pci_resource_start(dev, bar);

	if (pci_resource_flags(dev, bar) & IORESOURCE_MEM) {
		len =  pci_resource_len(dev, bar);

		if (!priv->remapped_bar)
			priv->remapped_bar = ioremap_nocache(base, len);
		if (!priv->remapped_bar)
			return -ENOMEM;
		port->iotype = UPIO_MEM32;
		port->iobase = 0;
		port->mapbase = base + offset;
		port->membase = priv->remapped_bar + offset;
		port->regshift = regshift;
#ifdef MONITOR_TRAFFIC
		port->serial_in = mem32_serial_in;
		port->serial_out = mem32_serial_out;
#endif
		info("bar:%d mapbase:0x%08lx membase:%p",
		     bar, (unsigned long)port->mapbase, port->membase);
	} else {
		port->iotype = UPIO_PORT;
		port->iobase = base + offset;
		port->mapbase = 0;
		port->membase = NULL;
		port->regshift = regshift;
	}
	return 0;
}

struct serial_private* /* __devinit */ rtm_t_init_ports(
	struct pci_dev *dev, struct pciserial_board *board, void *mapping)
{
	struct uart_8250_port uart;
	struct serial_private *priv;
	int ii;

	priv = kzalloc(sizeof(struct serial_private) +
		       sizeof(unsigned int) * board->num_ports,
		       GFP_KERNEL);
	if (!priv) {
		priv = ERR_PTR(-ENOMEM);
		//goto err_deinit;
		return priv;
	}
	priv->remapped_bar = mapping;
	priv->dev = dev;

	memset(&uart, 0, sizeof(uart));
	setup_port(priv, &uart.port, UART_BAR, 0, board->reg_shift);

	uart.port.flags = UPF_SKIP_TEST | UPF_BOOT_AUTOCONF | UPF_SHARE_IRQ;
	uart.port.uartclk = board->base_baud * 16;
	uart.port.irq = dev->irq+MSI_UART;
	uart.port.dev = &dev->dev;

	dbg(1, "register port with irq %d\n", uart.port.irq);
	for (ii = 0; ii < board->num_ports; ++ii){
		priv->line[ii] = serial8250_register_8250_port(&uart);
		if (priv->line[ii] < 0) {
			err("Couldn't register serial port %s: %d\n", 
				pci_name(dev), priv->line[ii]);
			break;
		}else{
			dbg(1, "registered serial port %d", priv->line[ii]);
		}
	}

	return priv;
}

void /* __devexit */ rtm_t_remove_ports(struct serial_private *priv)
{

}
struct pciserial_board BOARD = {
	.flags = 0,
	.num_ports = 1,
	.base_baud = RTM_T_UART_XTAL/16,
	.uart_offset = 0,
	.reg_shift = 2,
	.first_offset = 0
};

int /* __devinit */ rtm_t_uart_init(struct pci_dev *dev, void *mapping)
{
	int rc;
	struct serial_private *priv;
	dbg(1, "01 dev:%p &BOARD:%p mapping:%p", dev, &BOARD, mapping);

	priv = rtm_t_init_ports(dev, &BOARD, mapping);
	if (priv == 0){
		return 0;
	}
	if (!IS_ERR(priv)) {
		pci_set_drvdata(dev, priv);
		return 0;
	}

	rc = PTR_ERR(priv);

	return rc;
}


void /* __devexit */ rtm_t_uart_remove(struct pci_dev *dev)
{
	struct serial_private *priv = pci_get_drvdata(dev);

	dbg(1, "01");	
	pci_set_drvdata(dev, NULL);

	rtm_t_remove_ports(priv);

}
