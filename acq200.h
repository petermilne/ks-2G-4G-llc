
#include <linux/device.h>
#include <linux/module.h>
#include <linux/user.h>

extern void acq200_device_create_file(
	struct device * dev, struct device_attribute * attr,
	const char *file, int line);

#define DEVICE_CREATE_FILE(dev, attr) \
	acq200_device_create_file(dev, attr, __FILE__, __LINE__)

int acq200_intsDisable( unsigned irqs );
/**< disable interrupts defined by clear bits in irqs */

int acq200_intsEnable( unsigned irqs );
/**< enable interrupts defined by set bits in irqs */
