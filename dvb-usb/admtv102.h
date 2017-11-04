/*
 *  Driver for Microtune MT2060 "Single chip dual conversion broadband tuner"
 *
 *  Copyright (c) 2006 Olivier DANET <odanet@caramail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.=
 */

#ifndef ADMTV102_H
#define ADMTV102_H

#define ADMTV102_REFCLK13000  0
#define ADMTV102_REFCLK16384  1
#define ADMTV102_REFCLK19200  2
#define ADMTV102_REFCLK20480  3
#define ADMTV102_REFCLK24576  4
#define ADMTV102_REFCLK26000  5
#define ADMTV102_REFCLK30400  6
#define ADMTV102_REFCLK36000  7
#define ADMTV102_REFCLK38400  8
#define ADMTV102_REFCLK20000  9


struct dvb_frontend;
struct i2c_adapter;

struct admtv102_config {
	u8 i2c_address;
	u8 ref_clk_type;
};

//#if defined(CONFIG_MEDIA_TUNER_ADMTV102) || (defined(CONFIG_MEDIA_TUNER_ADMTV102_MODULE) && defined(MODULE))
struct dvb_frontend * admtv102_attach(struct dvb_frontend *fe, struct i2c_adapter *i2c, struct admtv102_config *cfg);
//#else
/*
static inline struct dvb_frontend * admtv102_attach(struct dvb_frontend *fe, struct i2c_adapter *i2c, struct admtv102_config *cfg)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
*/
//#endif // CONFIG_MEDIA_TUNER_MT2060

#endif
