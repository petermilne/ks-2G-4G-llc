/* liunux/arch/arm/mach-iop3xx/acq200syms.c */


#include <linux/ioport.h>

#include "acq200.h"
#include "acq200_debug.h"

void acq200_device_create_file(
        struct device * dev, struct device_attribute * attr,
        const char *file, int line)
{
        if (device_create_file(dev, attr)){
                err("%s:%d device_create_file", file, line);
        }
}

