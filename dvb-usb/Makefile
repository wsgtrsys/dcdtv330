#dvb-usb-objs += dvb-usb-firmware.o dvb-usb-init.o dvb-usb-urb.o dvb-usb-i2c.o
#dvb-usb-objs += dvb-usb-dvb.o usb-urb.o
#dvb3221-objs += dw2102.o

obj-m += dcdtv330.o
obj-m += admtv102.o

ccflags-y += -I$(srctree)/drivers/media/usb/dvb-usb
ccflags-y += -I$(srctree)/drivers/media/dvb-core
ccflags-y += -I$(srctree)/drivers/media/dvb-frontends/
