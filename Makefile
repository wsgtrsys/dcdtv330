obj-y := dvb-usb/

KVER ?= $(shell uname -r)  
KDIR := /lib/modules/$(KVER)/build

#KDIR=/root/raspberrypi/linux/
#KDIR=/root/devel/HiSTBAndroidV600R002C00SPC030/device/hisilicon/bigfish/sdk/source/kernel/linux-3.18.y

all:
	@echo "make=$(MAKE)"
	@echo "KDIR=$(KDIR)"
	@echo "PWD=$(shell pwd)"
	$(MAKE)  -C $(KDIR) M=$(shell pwd) modules
install:
	$(MAKE) -C $(KDIR) M=$(shell pwd) modules_install
clean:
	$(MAKE) -C $(KDIR) M=$(shell pwd) clean

