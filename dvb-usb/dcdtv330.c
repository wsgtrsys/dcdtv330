/* Digital China DCDTV330 DMB-TH USB Digital TV stick
 *
 * Copyright (C) 2011 Sword K23 (sword.k23@gmail.com)
 *
 *	This program is free software; you can redistribute it and/or modify it
 *	under the terms of the GNU General Public License as published by the Free
 *	Software Foundation, version 2.
 *
 * see Documentation/dvb/README.dvb-usb for more information
 */
/*
 * DM2016 key:81426CFB5303D111905F0000C0CC16BA
 *
 */
/* The architecture of the DMB-TH USB stick is:
Front:
        ------------------------------------------
       |                                          |
       |     AD9216                               |
       |                LGS-8G54                  |
Antenna-           30.400             CY7C68013A  -USB 
       |                                          |
       |    ADMTV102    M52S32162A                |
       |                                          |
	    ------------------------------------------
Back:
        ------------------------------------------
       |                                          |
       |                                          |
       |                                24.000    |
Antenna-                       DM2016             -USB 
       |                                          |
       |                                          |
       |                                          |
	    ------------------------------------------
 */

#define DVB_USB_LOG_PREFIX "dcdtv330"

#include "dvb-usb.h"
#include "admtv102.h"
#include "lgs8gxx.h"

#define USB_PID_DCDTV330                                0x1206

#define DCDTV330_READ_MSG 0
#define DCDTV330_WRITE_MSG 1


#define READ_FX2_REG_REQ  0xba

#define DCDTV330_WRITE_FX2 0xbb
#define DCDTV330_TUNER_REQ 0xb1


static int dvb_usb_dcdtv330_debug;
module_param_named(debug, dvb_usb_dcdtv330_debug, int, 0644);
MODULE_PARM_DESC(debug,
		 "set debugging level (1=info,xfer=2,pll=4,ts=8,err=16,rc=32,fw=64 (or-able))."
		 DVB_USB_DEBUG_STATUS);

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);


static int dcdtv330_rw(struct usb_device *dev, u8 request, u16 value,
			    u8 * data, u16 len, int flags)
{
	int ret;
	u8 tmp;
	u8 *buf;
	unsigned int pipe = (flags == DCDTV330_READ_MSG) ?
		usb_rcvctrlpipe(dev,0) : usb_sndctrlpipe(dev, 0);
	u8 request_type = (flags == DCDTV330_READ_MSG) ? USB_DIR_IN : USB_DIR_OUT;

	buf = kmalloc(len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	
	if (flags == DCDTV330_WRITE_MSG)
		memcpy(buf, data, len);
	ret = usb_control_msg(dev, pipe, request,
			request_type | USB_TYPE_VENDOR, value, 0x0,
			buf, len, 2000);

	if (request == DCDTV330_TUNER_REQ) {
		tmp = buf[0];
		if (usb_control_msg(dev, usb_rcvctrlpipe(dev, 0),
			    DCDTV330_TUNER_REQ, USB_DIR_IN | USB_TYPE_VENDOR,
			    0x01, 0x0, buf, 1, 2000) < 1 || buf[0] != 0x08) {
			ret = 0;
			goto out;
		}
		buf[0] = tmp;
	}
	if (flags == DCDTV330_READ_MSG)
		memcpy(data, buf, len);
out:
	// debug start
	if ((dvb_usb_dcdtv330_debug & 0x10) /*&& flags == DCDTV330_WRITE_MSG*/) {
		int j = 0;
			printk("usb mesage flag:%d bRequest:%02x wValue:%04x data:",
				flags, request,value);
			//printk("usb data: ");
			for (j = 0; j < len; ++j)
			    printk("%02x ", data[j]);
			printk("\n");
			
	}	
	// debug end
	kfree(buf);
	return ret;
}

/* I2C */
static int dcdtv330_usb_i2c_msgxfer(struct dvb_usb_device *dev, u16 addr,
				  u8 * buf, u16 len)
{
	int ret = 0;
	u8 request=0xb1;
	u16 value;

	if (!dev) {
		info("no usb_device");
		return -EINVAL;
	}
	if (mutex_lock_interruptible(&dev->usb_mutex) < 0)
		return -EAGAIN;

	value=addr;

	ret = dcdtv330_rw(dev->udev, request,
		value, buf, len,
		addr&0x01?DCDTV330_READ_MSG:DCDTV330_WRITE_MSG);
// debug start
/*
	if (dvb_usb_dcdtv330_debug & 0x10) {
		int j = 0;
			info("i2c mesage flag:%d bRequest:%02x wValue:%04x",
				 addr&0x01,request,value);
			printk("data: ");
			for (j = 0; j < len; ++j)
			    printk("%02x ", buf[j]);
			printk("\n");
			
	}
*/	
// debug end	

	mutex_unlock(&dev->usb_mutex);
	return ret;
}

static int dcdtv330_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msg[],
			   int num)
{
	struct dvb_usb_device *d = i2c_get_adapdata(adap);
	int i = 0, tmp = 0;

	if (!d)
		return -ENODEV;
	if (mutex_lock_interruptible(&d->i2c_mutex) < 0)
		return -EAGAIN;

	for (i = 0; i < num; i++) {
	/* write/read request */		
//debug begin
/*
        if (dvb_usb_dcdtv330_debug & 0x10) {
                int j = 0;
                        info("i2c mesage addr:%x flags:%x len:%d ",
                                 msg[i].addr,msg[i].flags,msg[i].len);
                        //printk("data: ");
                        for (j = 0; j < msg[i].len; ++j)
                            printk("%x ", msg[i].buf[j]);
                        printk("\n");

        }
*/
//debug end  
		if ((tmp = dcdtv330_usb_i2c_msgxfer(d,
					(msg[i].addr<<1)|(msg[i].flags&I2C_M_RD?0x01:0),
					msg[i].buf,
					msg[i].len
					)) != msg[i].len) {
			break;
		}
		
		/*
		if (dvb_usb_dcdtv330_debug & 0x10) {
			int j = 0,;
			printk("control read: data ");
			for (j = 0; j < msg[i].len; ++j)
			    printk("%02x ", msg[i].buf[j]);
			printk("\n");
			info("sending i2c mesage %d %d", tmp, msg[i].len);
		}
		*/
	}
	mutex_unlock(&d->i2c_mutex);
	return num;
}

static u32 dcdtv330_i2c_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C;
}

static struct i2c_algorithm dcdtv330_i2c_algo = {
	.master_xfer = dcdtv330_i2c_xfer,
	.functionality = dcdtv330_i2c_func,
};


static struct lgs8gxx_config dcdtv330_lgs8g54_cfg = {
	.prod = LGS8GXX_PROD_LGS8G52,
	.demod_address = 0x19,
	.serial_ts = 0,
	.ts_clk_pol = 1,
	.ts_clk_gated = 1,
	.if_clk_freq = 30400, /* 30.4 MHz */
	.if_freq = 0, /* Baseband */
	.if_neg_center = 1,
	.ext_adc = 1,
	.adc_signed = 1,
	.if_neg_edge = 1,
	.tuner_address = 0x61,
};


static int dcdtv330_frontend_attach(struct dvb_usb_adapter *adap)
{
	if ((adap->fe_adap[0].fe = 
			dvb_attach(lgs8gxx_attach,
			&dcdtv330_lgs8g54_cfg,
			&adap->dev->i2c_adap)) 
			!= NULL) {
		
		return 0;
	}
	info("not attached lgs8g54");
	return -ENODEV;
}

static struct admtv102_config dcdtv330_admtv102_cfg = {
	.i2c_address = 0x61,
	.ref_clk_type = ADMTV102_REFCLK30400
};

static int dcdtv330_tuner_attach(struct dvb_usb_adapter *adap)
{
	if (dvb_attach(admtv102_attach, adap->fe_adap[0].fe,
		&adap->dev->i2c_adap, &dcdtv330_admtv102_cfg) != NULL) {
		return 0;
	}
	return -ENODEV;
}



static struct usb_device_id dcdtv330_table[] = {
	{USB_DEVICE(USB_VID_CYPRESS, USB_PID_DCDTV330)},
	{USB_DEVICE(USB_VID_OPERA1, USB_PID_DCDTV330)},
	{}
};

MODULE_DEVICE_TABLE(usb, dcdtv330_table);


static struct dvb_usb_device_properties dcdtv330_properties = {
	.caps = DVB_USB_IS_AN_I2C_ADAPTER,
	.usb_ctrl = CYPRESS_FX2,
	.firmware = "dvb-usb-dcdtv330-01.fw",

	.i2c_algo = &dcdtv330_i2c_algo,

	.generic_bulk_ctrl_endpoint = 0x01,

	.num_adapters = 1,

	.adapter = {
		{
		.num_frontends = 1,
		.fe = {{	
			.frontend_attach  = dcdtv330_frontend_attach,
			.tuner_attach     = dcdtv330_tuner_attach,

	/* parameter for the MPEG2-data transfer */
			.stream = {
				.type = USB_BULK,
				.count = 7,
				.endpoint = 0x02,
				.u = {
					.bulk = {
						.buffersize = 4096,
					}
				}
			},
		}},	
		},
	},
	.num_device_descs = 1,
	.devices = {
		{"Digital China DCDTV330 DMB-TH",
			{&dcdtv330_table[0], NULL},
			{&dcdtv330_table[1], NULL},
		},
	}
};


static int dcdtv330_probe(struct usb_interface *intf,
			const struct usb_device_id *id)
{
	if (0 != dvb_usb_device_init(intf, &dcdtv330_properties,
				     THIS_MODULE, NULL, adapter_nr))
		return -EINVAL;
	return 0;
}

static struct usb_driver dcdtv330_driver = {
	.name = "dcdtv330",
	.probe = dcdtv330_probe,
	.disconnect = dvb_usb_device_exit,
	.id_table = dcdtv330_table,
};

static int __init dcdtv330_module_init(void)
{
	int result = 0;
	if ((result = usb_register(&dcdtv330_driver))) {
		err("usb_register failed. Error number %d", result);
	}
	return result;
}

static void __exit dcdtv330_module_exit(void)
{
	usb_deregister(&dcdtv330_driver);
}

module_init(dcdtv330_module_init);
module_exit(dcdtv330_module_exit);

MODULE_AUTHOR("Sword K23 (sword.k23@gmail.com)");
MODULE_DESCRIPTION("Driver for DCDTV330 DMB-TH device");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL");
