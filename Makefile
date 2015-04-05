obj-m := rtm-t.o acq_fiber_hba_i2c_bus.o

SRC := $(shell pwd)


ifeq ($(ACQ_FHBA_SPI),1)
SPI_MODS=acq_fiber_hba_spi.o
SPI_SUPPORT=spi_support
EXTRA_CFLAGS += -DSPI_SUPPORT
else
SPI_SUPPORT=
endif


# default build is the local kernel.
# build other kernels like this example:
# make KRNL=2.6.20-1.2948.fc6-i686 ARCH=i386
# make KRNL=2.6.18-194.26.1.el5 ARCH=i386



KRNL ?= $(shell uname -r)
# FEDORA:
# KHEADERS := /lib/modules/$(KRNL)/build
# SCI LIN:
KHEADERS := /usr/src/kernels/$(KRNL)/

rtm-t-objs := rtm-t-hostdrv.o rtm-t-sysfs.o rtm-t-uart.o \
		rtm_t_core_drv.o $(SPI_MODS) \
		acq100_rtm_t_sfp.o rtm_t_sfp_i2c.o \
		acq200.o acq200_utils.o ao32-drv.o \

acq_fiber_hba_i2c_bus-objs := rtm_t_i2c_bus.o rtm_t_core_drv.o

all: apps modules kernel_src_patch

modules: kernel_src_patch $(SPI_SUPPORT)
	make -C $(KHEADERS) M=$(SRC)  modules

spi_support:
	make -C $(KHEADERS) M=$(SRC)/spi obj-m="spi.o spi_bitbang.o" modules
	make -C $(KHEADERS) M=$(SRC)/mtd/devices obj-m=m25p80.o modules
        
mmap: mmap.o
	$(CC) -o mmap mmap.o -lpopt

rtm-t-stream: rtm-t-stream.o
	$(CC) -g -o rtm-t-stream rtm-t-stream.o -lpopt

rtm-t-stream-disk: rtm-t-stream-disk.o RTM_T_Device.o
	$(CXX) -g -o rtm-t-stream-disk rtm-t-stream-disk.o RTM_T_Device.o -lpopt
	
rtm-t-llcontrol: rtm-t-llcontrol.o RTM_T_Device.o
	$(CXX) -g -o rtm-t-llcontrol rtm-t-llcontrol.o RTM_T_Device.o -lpopt

rtm-t-feedback: rtm-t-feedback.o RTM_T_Device.o
	$(CXX) -g -o rtm-t-feedback rtm-t-feedback.o RTM_T_Device.o -lpopt

rtm-t-llcontrol-2GI-4GO-zcopy: rtm-t-llcontrol-2GI-4GO-zcopy.o RTM_T_Device.o
	$(CXX) -g -o rtm-t-llcontrol-2GI-4GO-zcopy rtm-t-llcontrol-2GI-4GO-zcopy.o RTM_T_Device.o -lpopt

rtm-t-llcontrol-2GI-4GO: rtm-t-llcontrol-2GI-4GO.o RTM_T_Device.o
	$(CXX) -g -o rtm-t-llcontrol-2GI-4GO rtm-t-llcontrol-2GI-4GO.o RTM_T_Device.o -lpopt

rtm-t-llcontrol-2GI-2GI-4GO: rtm-t-llcontrol-2GI-2GI-4GO.o RTM_T_Device.o
	$(CXX) -g -o rtm-t-llcontrol-2GI-4GO rtm-t-llcontrol-2GI-2GI-4GO.o RTM_T_Device.o -lpopt

rtm-t-feedback_1AI_2AO: rtm-t-feedback_1AI_2AO.o RTM_T_Device.o
	$(CXX) -g -o rtm-t-feedback_1AI_2AO rtm-t-feedback_1AI_2AO.o \
		RTM_T_Device.o -lpopt

	
APPS=mmap rtm-t-stream-disk inet_ntoa rtm-t-llcontrol \
	rtm-t-feedback rtm-t-feedback_1AI_2AO \
	rtm-t-llcontrol-2GI-4GO rtm-t-llcontrol-2GI-4GO-zcopy \
	rtm-t-llcontrol-2GI-2GI-4GO

apps: $(APPS)

KINCL := $(KHEADERS)/include/linux
GPIO_PATCH := i2c-gpio.h
MFILE := $(KINCL)/$(GPIO_PATCH)

kernel_src_patch:
	@if [ ! -f $(MFILE) ]; then \
		echo FILE $(MFILE) does not exist; \
		echo cp $(GPIO_PATCH) $(KINCL)/$(GPIO_PATCH); \
		if [ ! -w $(KINCL) ]; then \
			echo no permissions for patch, please rerun "make kernel_src_patch" as root;\
			exit 1; \
		else \
			cp $(GPIO_PATCH) $(KINCL)/$(GPIO_PATCH);\
		fi ;\
	fi	


clean:
	-rm -f $(APPS)
	make -C /lib/modules/$(KRNL)/build M=$(SRC) clean
	rm -f mmap rtm-t-stream rtm-t-stream-disk *.o

user_install:
	echo Copy all scripts to local ~/bin directory
	cp test-scripts/* bin/* ~/bin

install:
	cp 20-rtm-t.rules /etc/udev/rules.d
	mkdir -p /etc/hotplug.d/acqX00
	cp rtm-t.hotplug /etc/hotplug.d/acqX00
	cp -a bin/* /usr/local/bin/
	cp $(APPS) /usr/local/bin/
	-ln -s /usr/local/bin/inet_ntoa  /usr/local/bin/inet_aton 
	mkdir -p /lib/modules/$(KRNL)/d-tacq
	cp rtm-t.ko  /lib/modules/$(KRNL)/d-tacq
	cp rtm-t-hosts /etc
	-if which /sbin/depmod 2> /dev/null; then \
		echo depmod ... this takes a while ..; \
		/sbin/depmod -v $(KRNL) 2>&1 >/dev/null; \
	fi
	if [ -d /etc/init.d ]; then \
	   cp rtm-t /etc/init.d; \
	else \
           cp rtm-t /etc/rc.d/init.d; \
	fi   	
	-if which update-rc.d 2> /dev/null; then \
	    update-rc.d rtm-t defaults; \
	else \
	    /sbin/chkconfig --level 35 rtm-t on; \
	fi
	echo install done

DC ?= $(shell date +%y%m%d%H%M)

package:
	make clean
	doxygen
	(cd ../..; \
	tar cvzf /tmp/rtm-t-hostdrive-$(DC).tgz \
		--exclude-from=RTM-T/HOSTDRV/nosync \
		RTM-T/HOSTDRV \
		RTM-T/FUNCTIONAL_TESTS \
		RTM-T/FIRMWARE \
	)
	git tag P$(DC)
	mv /tmp/rtm-t-hostdrive-$(DC).tgz release
	echo ./release/rtm-t-hostdrive-$(DC).tgz created

git_export:
	rm -Rf GITEXPORT/*
	mkdir -p GITEXPORT/RTM-T-HOSTDRV.git
	git clone --bare . GITEXPORT/RTM-T-HOSTDRV.git/
	touch GITEXPORT/RTM-T-HOSTDRV.git/git-daemon-export-ok	
	(cd GITEXPORT/RTM-T-HOSTDRV.git;\
		git --bare update-server-info;\
		touch hooks/post-update)

flash_erase:
	cd mtd-utils;make
	
