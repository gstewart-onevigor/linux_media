/*
    Tmax TAS2101 - DVBS/S2 Satellite demodulator driver

    Copyright (C) 2014 Luis Alves <ljalvs@gmail.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/i2c-mux.h>

#include <media/dvb_frontend.h>

#include "tas2101.h"
#include "tas2101_priv.h"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0)
#if IS_ENABLED(CONFIG_I2C_MUX)
#define TAS2101_USE_I2C_MUX
#endif
#endif

/* return i2c adapter */
/* bus = 0   master   */
/* bus = 1   demod    */
/* bus = 2   tuner    */
struct i2c_adapter *tas2101_get_i2c_adapter(struct dvb_frontend *fe, int bus)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	switch (bus) {
	case 0:
	default:
		return priv->i2c;
	case 1:
		return priv->i2c_demod;
	case 2:
		return priv->i2c_tuner;
	}
}
EXPORT_SYMBOL_GPL(tas2101_get_i2c_adapter);

/* write multiple (continuous) registers */
/* the first value is the starting address */
static int tas2101_wrm(struct tas2101_priv *priv, u8 *buf, int len)
{
	int ret;
	struct i2c_msg msg = {
		.addr = priv->cfg->i2c_address,
		.flags = 0, .buf = buf, .len = len };

	dev_dbg(&priv->i2c->dev, "%s() i2c wrm @0x%02x (len=%d)\n",
		__func__, buf[0], len);

	ret = i2c_transfer(priv->i2c_demod, &msg, 1);
	if (ret < 0) {
		dev_warn(&priv->i2c->dev,
			"%s: i2c wrm err(%i) @0x%02x (len=%d)\n",
			KBUILD_MODNAME, ret, buf[0], len);
		return ret;
	}
	return 0;
}

/* write one register */
static int tas2101_wr(struct tas2101_priv *priv, u8 addr, u8 data)
{
	u8 buf[] = { addr, data };
	return tas2101_wrm(priv, buf, 2);
}

/* read multiple (continuous) registers starting at addr */
static int tas2101_rdm(struct tas2101_priv *priv, u8 addr, u8 *buf, int len)
{
	int ret;
	struct i2c_msg msg[] = {
		{ .addr = priv->cfg->i2c_address, .flags = 0,
			.buf = &addr, .len = 1 },
		{ .addr = priv->cfg->i2c_address, .flags = I2C_M_RD,
			.buf = buf, .len = len }
	};

	dev_dbg(&priv->i2c->dev, "%s() i2c rdm @0x%02x (len=%d)\n",
		__func__, addr, len);

	ret = i2c_transfer(priv->i2c_demod, msg, 2);
	if (ret < 0) {
		dev_warn(&priv->i2c->dev,
			"%s: i2c rdm err(%i) @0x%02x (len=%d)\n",
			KBUILD_MODNAME, ret, addr, len);
		return ret;
	}
	return 0;
}

/* read one register */
static int tas2101_rd(struct tas2101_priv *priv, u8 addr, u8 *data)
{
	return tas2101_rdm(priv, addr, data, 1);
}

static int tas2101_regmask(struct tas2101_priv *priv,
	u8 reg, u8 setmask, u8 clrmask)
{
	int ret;
	u8 b = 0;
	if (clrmask != 0xff) {
		ret = tas2101_rd(priv, reg, &b);
		if (ret)
			return ret;
		b &= ~clrmask;
	}
	return tas2101_wr(priv, reg, b | setmask);
}

static int tas2101_wrtable(struct tas2101_priv *priv,
	struct tas2101_regtable *regtable, int len)
{
	int ret, i;

	for (i = 0; i < len; i++) {
		ret = tas2101_regmask(priv, regtable[i].addr,
			regtable[i].setmask, regtable[i].clrmask);
		if (ret)
			return ret;
		if (regtable[i].sleep)
			msleep(regtable[i].sleep);
	}
	return 0;
}

static int tas2101_read_ber(struct dvb_frontend *fe, u32 *ber)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret;
	u8 buf[4];

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);

	switch (c->delivery_system) {
		case SYS_DVBS:
			ret = tas2101_rdm(priv, S1_BER_0, buf, 4);
			if (ret)
				return ret;

			*ber = ((((u32) buf[3] & 3) << 24) | (((u32) buf[2]) << 16)
				| (((u32) buf[1]) << 8) | ((u32) buf[0]));
			break;

		case SYS_DVBS2:
			ret = tas2101_rdm(priv, S2_BER_0, buf, 2);
			if (ret)
				return ret;

			*ber = ((((u32) buf[1]) << 8) | ((u32) buf[0]));
			break;

		default:
			*ber = 0;
			break;
	}

	dev_dbg(&priv->i2c->dev, "%s() ber = %d\n", __func__, *ber);
	return 0;
}

struct tas2101_dbmtable_pair dbm_raw_tab_1030000[] = 
{
	{ 65, 0x06B7 },
	{ 64, 0x0694 },
	{ 63, 0x0658 },
	{ 62, 0x06D6 },
	{ 61, 0x05F4 },
	{ 60, 0x05B5 },

	{ 59, 0x0663 },
	{ 58, 0x05D4 },
	{ 57, 0x0594 },
	{ 56, 0x0559 },
	{ 55, 0x0529 },
	{ 54, 0x04FC },
	{ 53, 0x0570 },
	{ 52, 0x0534 },
	{ 51, 0x0501 },
	{ 50, 0x04DA },

	{ 49, 0x04B8 },
	{ 48, 0x0496 },
	{ 47, 0x047A },
	{ 46, 0x045F },
	{ 45, 0x044C },
	{ 44, 0x0434 },
	{ 43, 0x0426 },
	{ 42, 0x0416 },
	{ 41, 0x0408 },
	{ 40, 0x03F9 },

	{ 39, 0x03E9 },
	{ 38, 0x03DB },
	{ 37, 0x03CC },
	{ 36, 0x03BC },
	{ 35, 0x03AB },
	{ 34, 0x039B },
	{ 33, 0x038D },
	{ 32, 0x0379 },
	{ 31, 0x036A },
	{ 30, 0x035A },

	{ 29, 0x034A },
	{ 28, 0x033A },
	{ 27, 0x032A },
	{ 26, 0x02F8 },
	{ 25, 0x02ED },
	{ 24, 0x02DC },
	{ 23, 0x02CF },
	{ 22, 0x02C1 },
	{ 21, 0x02B6 },
	{ 20, 0x02A9 },

	{ 19, 0x029A },
	{ 18, 0x0288 },
	{ 17, 0x027E },
	{ 16, 0x026D },
	{ 15, 0x025F }
};

struct tas2101_dbmtable_pair dbm_raw_tab_1090000[] = 
{
	{ 65, 0x05D1 },
	{ 64, 0x05A9 },
	{ 63, 0x057B },
	{ 62, 0x0544 },
	{ 61, 0x0508 },
	{ 60, 0x0581 },

	{ 59, 0x0573 },
	{ 58, 0x053D },
	{ 57, 0x04FC },
	{ 56, 0x04BD },
	{ 55, 0x0489 },
	{ 54, 0x045B },
	{ 53, 0x0434 },
	{ 52, 0x048F },
	{ 51, 0x0460 },
	{ 50, 0x0435 },

	{ 49, 0x0410 },
	{ 48, 0x03F2 },
	{ 47, 0x03D8 },
	{ 46, 0x03BE },
	{ 45, 0x03AB },
	{ 44, 0x0398 },
	{ 43, 0x0387 },
	{ 42, 0x0375 },
	{ 41, 0x0369 },
	{ 40, 0x035A },

	{ 39, 0x034D },
	{ 38, 0x033F },
	{ 37, 0x0330 },
	{ 36, 0x0321 },
	{ 35, 0x0313 },
	{ 34, 0x0303 },
	{ 33, 0x02F3 },
	{ 32, 0x02E7 },
	{ 31, 0x02D6 },
	{ 30, 0x02C9 },

	{ 29, 0x02BD },
	{ 28, 0x02AD },
	{ 27, 0x02A0 },
	{ 26, 0x0292 },
	{ 25, 0x0287 },
	{ 24, 0x025F },
	{ 23, 0x0255 },
	{ 22, 0x024B },
	{ 21, 0x0240 },
	{ 20, 0x0237 },

	{ 19, 0x022C },
	{ 18, 0x0221 },
	{ 17, 0x0218 },
	{ 16, 0x020D },
	{ 15, 0x0201 }
};

struct tas2101_dbmtable_pair dbm_raw_tab_1070000[] = 
{
	{ 65, 0x06CB },
	{ 64, 0x0699 },
	{ 63, 0x0664 },
	{ 62, 0x062F },
	{ 61, 0x05F3 },
	{ 60, 0x06A5 },

	{ 59, 0x0607 },
	{ 58, 0x05D7 },
	{ 57, 0x05A0 },
	{ 56, 0x0574 },
	{ 55, 0x0548 },
	{ 54, 0x051C },
	{ 53, 0x04F3 },
	{ 52, 0x055C },
	{ 51, 0x0532 },
	{ 50, 0x050A },

	{ 49, 0x04E1 },
	{ 48, 0x04C3 },
	{ 47, 0x04A7 },
	{ 46, 0x048E },
	{ 45, 0x0475 },
	{ 44, 0x045F },
	{ 43, 0x044E },
	{ 42, 0x0441 },
	{ 41, 0x042E },
	{ 40, 0x0420 },

	{ 39, 0x0413 },
	{ 38, 0x0404 },
	{ 37, 0x03F4 },
	{ 36, 0x03E5 },
	{ 35, 0x03D4 },
	{ 34, 0x03C5 },
	{ 33, 0x03B4 },
	{ 32, 0x03A5 },
	{ 31, 0x0395 },
	{ 30, 0x0386 },

	{ 29, 0x0376 },
	{ 28, 0x036D },
	{ 27, 0x0338 },
	{ 26, 0x032D },
	{ 25, 0x0320 },
	{ 24, 0x0314 },
	{ 23, 0x030A },
	{ 22, 0x02FE },
	{ 21, 0x02F1 },
	{ 20, 0x02E6 },

	{ 19, 0x02DB },
	{ 18, 0x02D0 },
	{ 17, 0x02C4 },
	{ 16, 0x02BA },
	{ 15, 0x02AF }
};

static int tas2101_read_signal_strength(struct dvb_frontend *fe,
	u16 *signal_strength)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	struct tas2101_dbmtable_pair *dbm_raw_tab;
	int dbm_raw_tab_length;
	int ret, i;
	long val;
	u16 agc_dBuV, agc_out;
	u8 buf[2];
	s64 svalue;
	u16 delta;

	svalue = 0;

	ret = tas2101_rdm(priv, SIGSTR_0, buf, 2);
	if (ret)
		return ret;

	agc_out = (((buf[1] & 0xf0)>>4) << 8) | buf[0];
#if 0

	for (i = 0; i < ARRAY_SIZE(tas2101_dbmtable) - 1; i++)
		if (tas2101_dbmtable[i].raw < dbm_raw)
			break;

	if( i == 0 )
		*signal_strength = tas2101_dbmtable[i].dbm;
	else
	{
		/* linear interpolation between two calibrated values */
		val = (dbm_raw - tas2101_dbmtable[i].raw) * tas2101_dbmtable[i-1].dbm;
		val += (tas2101_dbmtable[i-1].raw - dbm_raw) * tas2101_dbmtable[i].dbm;
		val /= (tas2101_dbmtable[i-1].raw - tas2101_dbmtable[i].raw);

		*signal_strength = (u16)val;
	}
#endif

	if(agc_out>2787){agc_dBuV=0;}

    if((agc_out>2094)&&(agc_out<=2787)){agc_dBuV=170+(2787-agc_out)*10/46;}

	if((agc_out>1773)&&(agc_out<=2094)){agc_dBuV=320+(2094-agc_out)*10/27;}

	if((agc_out>1005)&&(agc_out<=1773)){agc_dBuV=440+(1773-agc_out)*10/26;}

    if((agc_out>851)&&(agc_out<=1005)){agc_dBuV=730+(1005-agc_out)*10/17;}

    if((agc_out>676)&&(agc_out<=851)){agc_dBuV=820+(851-agc_out)*10/10;}

    if(agc_out<=676){agc_dBuV=999;}
	
	if(fe->ops.tuner_ops.get_rf_strength)
		fe->ops.tuner_ops.get_rf_strength(fe, &agc_dBuV);
			

	c->strength.len = 1;
	//c->strength.stat[0].scale = FE_SCALE_DECIBEL;
	//c->strength.stat[0].uvalue = agc_dBuV;

//	*signal_strength  = agc_dBuV;

	*signal_strength = c->strength.stat[0].scale == FE_SCALE_DECIBEL ? ((106000 + (s32)c->strength.stat[0].svalue) / 1000) * 656 : 0;
	c->strength.stat[0].svalue= c->strength.stat[0].svalue/10;

	dbm_raw_tab = NULL;

	/* override satellite signal strength values with lookup table (when available for the given frequency) */
	if (fe->dtv_property_cache.frequency == 1030000) {
		dbm_raw_tab = dbm_raw_tab_1030000;
		dbm_raw_tab_length = ARRAY_SIZE(dbm_raw_tab_1030000);
	}
	else if (fe->dtv_property_cache.frequency == 1090000) {
		dbm_raw_tab = dbm_raw_tab_1090000;
		dbm_raw_tab_length = ARRAY_SIZE(dbm_raw_tab_1090000);
	}
	else if (fe->dtv_property_cache.frequency == 1070000) {
		dbm_raw_tab = dbm_raw_tab_1070000;
		dbm_raw_tab_length = ARRAY_SIZE(dbm_raw_tab_1070000);
	}
	/* TODO: additional frequencies */

	if (dbm_raw_tab) {
		delta = 0xffff;
		svalue = 0;

		for (i = 0; i < dbm_raw_tab_length - 1; i++) {
			if (abs(dbm_raw_tab[i].raw - agc_out) < delta) {
				delta = abs(dbm_raw_tab[i].raw - agc_out);
				svalue = dbm_raw_tab[i].dbm * -1000;
			}
		}

		c->strength.stat[0].scale = FE_SCALE_DECIBEL;
		c->strength.stat[0].svalue = svalue;
	}
	 
	dev_dbg(&priv->i2c->dev, "%s() strength = 0x%04x\n",
		__func__, *signal_strength);
	return 0;
}

static int tas2101_read_snr(struct dvb_frontend *fe, u16 *snr)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret, i;
	long val;
	u16 snr_raw;
	u8 buf[2];

	ret = tas2101_rdm(priv, SNR_0, buf, 2);
	if (ret)
		return ret;

	snr_raw = (((u16)buf[1] & 0x0f) << 8) | buf[0];

	for (i = 0; i < ARRAY_SIZE(tas2101_snrtable) - 1; i++)
		if (tas2101_snrtable[i].raw < snr_raw)
			break;

	if( i == 0 )
		val = tas2101_snrtable[i].snr;
	else
	{
		/* linear interpolation between two calibrated values */
		val = (snr_raw - tas2101_snrtable[i].raw) * tas2101_snrtable[i-1].snr;
		val += (tas2101_snrtable[i-1].raw - snr_raw) * tas2101_snrtable[i].snr;
		val /= (tas2101_snrtable[i-1].raw - tas2101_snrtable[i].raw);
	}

	c->cnr.len = 1;
	c->cnr.stat[0].scale = FE_SCALE_DECIBEL;
	c->cnr.stat[0].uvalue = 100 * (s64) val;

	*snr = (u16) val * 328; /* 20dB = 100% */

	dev_dbg(&priv->i2c->dev, "%s() snr = 0x%04x\n",
		__func__, *snr);

	return 0;
}

/* unimplemented */
static int tas2101_read_ucblocks(struct dvb_frontend *fe, u32 *ucblocks)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	*ucblocks = 0;
	return 0;
}

static int tas2101_read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	int ret;
	u8 reg;
	u16 snr;

	*status = 0;

	ret = tas2101_rd(priv, DEMOD_STATUS, &reg);
	if (ret)
		return ret;

	reg &= DEMOD_STATUS_MASK;
	if (reg == DEMOD_LOCKED) {
		*status = FE_HAS_SIGNAL | FE_HAS_CARRIER |
			FE_HAS_VITERBI | FE_HAS_SYNC | FE_HAS_LOCK;

		ret = tas2101_rd(priv, REG_04, &reg);
		if (ret)
			return ret;
		if (reg & 0x08)
			ret = tas2101_wr(priv, REG_04, reg & ~0x08);
		
		tas2101_read_snr(fe, &snr);
	}

	dev_dbg(&priv->i2c->dev, "%s() status = 0x%02x\n", __func__, *status);
	return ret;
}

static void tas2101_spi_read(struct dvb_frontend *fe, struct ecp3_info *ecp3inf)
{

	struct tas2101_priv *priv = fe->demodulator_priv;
	struct i2c_adapter *adapter = priv->i2c;
	if (priv->cfg->read_properties)
		priv->cfg->read_properties(adapter,ecp3inf->reg, &(ecp3inf->data));
	return;
}
static void tas2101_spi_write(struct dvb_frontend *fe,struct ecp3_info *ecp3inf)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	struct i2c_adapter *adapter = priv->i2c;
	if (priv->cfg->write_properties)
		priv->cfg->write_properties(adapter,ecp3inf->reg, ecp3inf->data);
	return ;
}

static void tas2101_eeprom_read(struct dvb_frontend *fe, struct eeprom_info *eepinf)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	struct i2c_adapter *adapter = priv->i2c;

	if (priv->cfg->read_eeprom)
		priv->cfg->read_eeprom(adapter,eepinf->reg, &(eepinf->data));
	
	return ;
}

static void tas2101_eeprom_write(struct dvb_frontend *fe,struct eeprom_info *eepinf)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	struct i2c_adapter *adapter = priv->i2c;

	if (priv->cfg->write_eeprom)
		priv->cfg->write_eeprom(adapter,eepinf->reg, eepinf->data);
	
	return ;
}

static int tas2101_set_voltage(struct dvb_frontend *fe,
	enum fe_sec_voltage voltage)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	int ret = 0;
	
	dev_dbg(&priv->i2c->dev, "%s() %s\n", __func__,
		voltage == SEC_VOLTAGE_13 ? "SEC_VOLTAGE_13" :
		voltage == SEC_VOLTAGE_18 ? "SEC_VOLTAGE_18" :
		"SEC_VOLTAGE_OFF");

	switch (voltage) {
		case SEC_VOLTAGE_13:
			if (priv->cfg->lnb_power)
				priv->cfg->lnb_power(fe, LNB_ON);
			ret = tas2101_regmask(priv, LNB_CTRL,
				0, VSEL13_18);
			break;
		case SEC_VOLTAGE_18:
			if (priv->cfg->lnb_power)
				priv->cfg->lnb_power(fe, LNB_ON);
			ret = tas2101_regmask(priv, LNB_CTRL,
				VSEL13_18, 0);
			break;
		default: /* OFF */
			if (priv->cfg->lnb_power)
				priv->cfg->lnb_power(fe, LNB_OFF);
			break;
	}
	return ret;
}

static int tas2101_set_tone(struct dvb_frontend *fe,
	enum fe_sec_tone_mode tone)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	int ret = -EINVAL;

	dev_dbg(&priv->i2c->dev, "%s() %s\n", __func__,
		tone == SEC_TONE_ON ? "SEC_TONE_ON" : "SEC_TONE_OFF");

	switch (tone) {
	case SEC_TONE_ON:
		ret = tas2101_regmask(priv, LNB_CTRL,
			TONE_ON, DISEQC_CMD_MASK);
		break;
	case SEC_TONE_OFF:
		ret = tas2101_regmask(priv, LNB_CTRL,
			TONE_OFF, DISEQC_CMD_MASK);
		break;
	default:
		dev_warn(&priv->i2c->dev, "%s() invalid tone (%d)\n",
			__func__, tone);
		break;
	}
	return ret;
}

static int tas2101_send_diseqc_msg(struct dvb_frontend *fe,
	struct dvb_diseqc_master_cmd *d)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	int ret, i;
	u8 bck, buf[9];

	/* dump DiSEqC message */
	dev_dbg(&priv->i2c->dev, "%s() ( ", __func__);
	for (i = 0; i < d->msg_len; i++)
		dev_dbg(&priv->i2c->dev, "0x%02x ", d->msg[i]);
	dev_dbg(&priv->i2c->dev, ")\n");

	/* backup LNB tone state */
	ret = tas2101_rd(priv, LNB_CTRL, &bck);
	if (ret)
		return ret;

	ret = tas2101_regmask(priv, REG_34, 0, 0x40);
	if (ret)
		goto exit;

	/* setup DISEqC message to demod */
	buf[0] = DISEQC_BUFFER;
	memcpy(&buf[1], d->msg, 8);
	ret = tas2101_wrm(priv, buf, d->msg_len + 1);
	if (ret)
		goto exit;

	/* send DISEqC send command */
	buf[0] = (bck & ~(DISEQC_CMD_LEN_MASK | DISEQC_CMD_MASK)) |
		DISEQC_SEND_MSG | ((d->msg_len - 1) << 3);
	ret = tas2101_wr(priv, LNB_CTRL, buf[0]);
	if (ret)
		goto exit;

	/* wait at least diseqc typical tx time */
	msleep(54);

	/* Wait for busy flag to clear */
	for (i = 0; i < 10; i++) {
		ret = tas2101_rd(priv, LNB_STATUS, &buf[0]);
		if (ret)
			break;
		if (buf[0] & DISEQC_BUSY)
			goto exit;
		msleep(20);
	}

	/* try to restore the tone setting but return a timeout error */
	ret = tas2101_wr(priv, LNB_CTRL, bck);
	dev_warn(&priv->i2c->dev, "%s() timeout sending burst\n", __func__);
	return -ETIMEDOUT;
exit:
	/* restore tone setting */
	return tas2101_wr(priv, LNB_CTRL, bck);
}

static int tas2101_diseqc_send_burst(struct dvb_frontend *fe,
	enum fe_sec_mini_cmd burst)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	int ret, i;
	u8 bck, r;

	if ((burst != SEC_MINI_A) && (burst != SEC_MINI_B)) {
		dev_err(&priv->i2c->dev, "%s() invalid burst(%d)\n",
			__func__, burst);
		return -EINVAL;
	}

	dev_dbg(&priv->i2c->dev, "%s() %s\n", __func__,
		burst == SEC_MINI_A ? "SEC_MINI_A" : "SEC_MINI_B");

	/* backup LNB tone state */
	ret = tas2101_rd(priv, LNB_CTRL, &bck);
	if (ret)
		return ret;

	ret = tas2101_regmask(priv, REG_34, 0, 0x40);
	if (ret)
		goto exit;

	/* set tone burst cmd */
	r = (bck & ~DISEQC_CMD_MASK) |
		(burst == SEC_MINI_A) ? DISEQC_BURST_A : DISEQC_BURST_B;

	ret = tas2101_wr(priv, LNB_CTRL, r);
	if (ret)
		goto exit;

	/* spec = around 12.5 ms for the burst */
	for (i = 0; i < 10; i++) {
		ret = tas2101_rd(priv, LNB_STATUS, &r);
		if (ret)
			break;
		if (r & DISEQC_BUSY)
			goto exit;
		msleep(20);
	}

	/* try to restore the tone setting but return a timeout error */
	ret = tas2101_wr(priv, LNB_CTRL, bck);
	dev_warn(&priv->i2c->dev, "%s() timeout sending burst\n", __func__);
	return -ETIMEDOUT;
exit:
	/* restore tone setting */
	return tas2101_wr(priv, LNB_CTRL, bck);
}

static void tas2101_release(struct dvb_frontend *fe)
{
	struct tas2101_priv *priv = fe->demodulator_priv;

	dev_dbg(&priv->i2c->dev, "%s\n", __func__);
#ifdef TAS2101_USE_I2C_MUX
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0)
	i2c_mux_del_adapters(priv->muxc);
#else
	i2c_del_mux_adapter(priv->i2c_demod);
	i2c_del_mux_adapter(priv->i2c_tuner);
#endif
#endif
	kfree(priv);
}

#ifdef TAS2101_USE_I2C_MUX
/* channel 0: demod */
/* channel 1: tuner */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0)
static int tas2101_i2c_select(struct i2c_mux_core *muxc, u32 chan_id)
{
	struct tas2101_priv *priv = i2c_mux_priv(muxc);
	struct i2c_adapter *adap = priv->i2c;
#else
static int tas2101_i2c_select(struct i2c_adapter *adap,
	void *mux_priv, u32 chan_id)
{
	struct tas2101_priv *priv = mux_priv;
#endif
	int ret;
	u8 buf[2];
	struct i2c_msg msg_wr[] = {
		{ .addr = priv->cfg->i2c_address, .flags = 0,
			.buf = buf, .len = 2 }
	};
	struct i2c_msg msg_rd[] = {
		{ .addr = priv->cfg->i2c_address, .flags = 0,
			.buf = &buf[0], .len = 1 },
		{ .addr = priv->cfg->i2c_address, .flags = I2C_M_RD,
			.buf = &buf[1], .len = 1 }
	};

	dev_dbg(&priv->i2c->dev, "%s() ch=%d\n", __func__, chan_id);

	if (priv->i2c_ch == chan_id)
		return 0;

	buf[0] = REG_06;
	ret = __i2c_transfer(adap, msg_rd, 2);
	if (ret != 2)
		goto err;

	if (chan_id == 0)
		buf[1] &= ~I2C_GATE;
	else
		buf[1] |= I2C_GATE;

	ret = __i2c_transfer(adap, msg_wr, 1);
	if (ret != 1)
		goto err;

	priv->i2c_ch = chan_id;

	return 0;
err:
	dev_dbg(&priv->i2c->dev, "%s() failed=%d\n", __func__, ret);
	return -EREMOTEIO;
}
#endif

static struct dvb_frontend_ops tas2101_ops;

struct dvb_frontend *tas2101_attach(const struct tas2101_config *cfg,
	struct i2c_adapter *i2c)
{
	struct tas2101_priv *priv = NULL;
	int ret;
	u8 id[2];

	dev_dbg(&i2c->dev, "%s: Attaching frontend\n", KBUILD_MODNAME);

	/* allocate memory for the priv data */
	priv = kzalloc(sizeof(struct tas2101_priv), GFP_KERNEL);
	if (priv == NULL)
		goto err;

	priv->cfg = cfg;
	priv->i2c = i2c;
	priv->i2c_ch = 0;

#ifdef TAS2101_USE_I2C_MUX
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0)
	/* create mux i2c adapter for tuner */
	priv->muxc = i2c_mux_alloc(i2c, &i2c->dev,
				  2, 0, I2C_MUX_LOCKED,
				  tas2101_i2c_select, NULL);
	if (!priv->muxc) {
		ret = -ENOMEM;
		goto err1;
	}
	priv->muxc->priv = priv;
	ret = i2c_mux_add_adapter(priv->muxc, 0, 0, 0);
	if (ret)
		goto err1;
	ret = i2c_mux_add_adapter(priv->muxc, 0, 1, 0);
	if (ret)
		goto err1;
	priv->i2c_demod = priv->muxc->adapter[0];
	priv->i2c_tuner = priv->muxc->adapter[1];
#else
	/* create muxed i2c adapter for the demod */
	priv->i2c_demod = i2c_add_mux_adapter(i2c, &i2c->dev, priv, 0, 0, 0,
		tas2101_i2c_select, NULL);
	if (priv->i2c_demod == NULL)
		goto err1;

	/* create muxed i2c adapter for the tuner */
	priv->i2c_tuner = i2c_add_mux_adapter(i2c, &i2c->dev, priv, 0, 1, 0,
		tas2101_i2c_select, NULL);
	if (priv->i2c_tuner == NULL)
		goto err2;
#endif
#else
	priv->i2c_demod = i2c;
	priv->i2c_tuner = i2c;
#endif

	/* create dvb_frontend */
	memcpy(&priv->fe.ops, &tas2101_ops,
		sizeof(struct dvb_frontend_ops));
	priv->fe.demodulator_priv = priv;

	/* reset demod */
	if (cfg->reset_demod)
		cfg->reset_demod(&priv->fe);

	msleep(100);

	/* check if demod is alive */
	ret = tas2101_rdm(priv, ID_0, id, 2);
	if ((id[0] != 0x44) || (id[1] != 0x4c))
		ret |= -EIO;
	if (ret)
		goto err3;

	return &priv->fe;

err3:
#ifdef TAS2101_USE_I2C_MUX
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0)
	i2c_mux_del_adapters(priv->muxc);
#else
	i2c_del_mux_adapter(priv->i2c_tuner);
err2:
	i2c_del_mux_adapter(priv->i2c_demod);
#endif
#endif
err1:
	kfree(priv);
err:
	dev_err(&i2c->dev, "%s: Error attaching frontend\n", KBUILD_MODNAME);
	return NULL;
}
EXPORT_SYMBOL_GPL(tas2101_attach);

static int tas2101_initfe(struct dvb_frontend *fe)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	struct tas2101_regtable *t;
	u8 buf[7], size;
	int ret;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);

	if (priv->cfg->id == ID_TAS2101) {
		t = tas2101_initfe0;
		size = ARRAY_SIZE(tas2101_initfe0);
	} else {
		t = tas2100_initfe0;
		size = ARRAY_SIZE(tas2100_initfe0);
	}
	ret = tas2101_wrtable(priv, t, size);
	if (ret)
		return ret;

	buf[0] = 0xe6;
	memcpy(&buf[1], priv->cfg->init, 6);
	ret = tas2101_wrm(priv, buf, 7);
	if (ret)
		return ret;

	ret = tas2101_regmask(priv, 0xe0, priv->cfg->init[6], 0xff);
	if (ret)
		return ret;

	if (priv->cfg->id == ID_TAS2101) {
		t = tas2101_initfe1;
		size = ARRAY_SIZE(tas2101_initfe1);
	} else {
		t = tas2100_initfe1;
		size = ARRAY_SIZE(tas2100_initfe1);
	}
	ret = tas2101_wrtable(priv, t, size);
	if (ret)
		return ret;

	if (priv->cfg->init2) {
		t = tas2101_initfe2;
		size = ARRAY_SIZE(tas2101_initfe2);
		ret = tas2101_wrtable(priv, t, size);
		if (ret)
			return ret;
	}

	return 0;
}

static int tas2101_sleep(struct dvb_frontend *fe)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	return 0;
}

static int tas2101_set_frontend(struct dvb_frontend *fe)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	enum fe_status tunerstat;
	int ret, i;
	u32 s;
	u8 buf[3];

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);

	/* do some basic parameter validation */
	switch (c->delivery_system) {
	case SYS_DVBS:
		dev_dbg(&priv->i2c->dev, "%s() DVB-S\n", __func__);
		/* Only QPSK is supported for DVB-S */
		if (c->modulation != QPSK) {
			dev_dbg(&priv->i2c->dev,
				"%s() unsupported modulation (%d)\n",
				__func__, c->modulation);
			return -EINVAL;
		}
		break;
	case SYS_DVBS2:
		dev_dbg(&priv->i2c->dev, "%s() DVB-S2\n", __func__);
		break;
	default:
		dev_warn(&priv->i2c->dev,
			"%s() unsupported delivery system (%d)\n",
			__func__, c->delivery_system);
		return -EINVAL;
	}

	ret = tas2101_wrtable(priv, tas2101_setfe, ARRAY_SIZE(tas2101_setfe));
	if (ret)
		return ret;

	/* set symbol rate */
	s = c->symbol_rate / 1000;
	buf[0] = SET_SRATE0;
	buf[1] = (u8) s;
	buf[2] = (u8) (s >> 8);
	ret = tas2101_wrm(priv, buf, 3);
	if (ret)
		return ret;

	/* clear freq offset */
	buf[0] = FREQ_OS0;
	buf[1] = 0;
	buf[2] = 0;
	ret = tas2101_wrm(priv, buf, 3);
	if (ret)
		return ret;

	if (fe->ops.tuner_ops.set_params) {
#ifndef TAS2101_USE_I2C_MUX
		if (fe->ops.i2c_gate_ctrl)
			fe->ops.i2c_gate_ctrl(fe, 1);
#endif
		fe->ops.tuner_ops.set_params(fe);
#ifndef TAS2101_USE_I2C_MUX
		if (fe->ops.i2c_gate_ctrl)
			fe->ops.i2c_gate_ctrl(fe, 0);
#endif
	}

	ret = tas2101_regmask(priv, REG_30, 0x01, 0);
	if (ret)
		return ret;

	for (i = 0; i<15; i++) {
		ret = tas2101_read_status(fe, &tunerstat);
		if (tunerstat & FE_HAS_LOCK)
			return 0;
		msleep(20);
	}
	return -EINVAL;
}

static int tas2101_get_frontend(struct dvb_frontend *fe,
				struct dtv_frontend_properties *c)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	int ret;
	u8 reg, buf[2];

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);

	ret = tas2101_rd(priv, MODFEC_0, &reg);
	if (ret)
		return ret;

	if ((reg >> 6) == 0) {
		/* DVB-S */
		reg &= 0x07;
	} else {
		/* DVB-S2 */
		ret = tas2101_rd(priv, MODFEC_1, &reg);
		if (ret)
			return ret;
		reg += 5;
	}

	if (reg > 33) {
		dev_dbg(&priv->i2c->dev, "%s() Unable to get current delivery"
			" system and mode.\n", __func__);
		reg = 0;
	}

	c->fec_inner = tas2101_modfec_modes[reg].fec;
	c->modulation = tas2101_modfec_modes[reg].modulation;
	c->delivery_system = tas2101_modfec_modes[reg].delivery_system;
	c->inversion = INVERSION_AUTO;

	/* symbol rate */
	ret = tas2101_rdm(priv, GET_SRATE0, buf, 2);
	if (ret)
		return ret;
	c->symbol_rate = ((buf[1] << 8) | buf[0]) * 1000;

	return 0;
}

static int tas2101_tune(struct dvb_frontend *fe, bool re_tune,
	unsigned int mode_flags, unsigned int *delay, enum fe_status *status)
{
	struct tas2101_priv *priv = fe->demodulator_priv;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);

	*delay = HZ / 5;
	if (re_tune) {
		int ret = tas2101_set_frontend(fe);
		if (ret)
			return ret;
	}
	return tas2101_read_status(fe, status);
}

static enum dvbfe_algo tas2101_get_algo(struct dvb_frontend *fe)
{
	return DVBFE_ALGO_HW;
}

#ifndef TAS2101_USE_I2C_MUX
static int tas2101_i2c_gate_ctrl(struct dvb_frontend* fe, int enable)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	int ret;

	if (enable)
		ret = tas2101_regmask(priv, REG_06, I2C_GATE, 0);
	else
		ret = tas2101_regmask(priv, REG_06, 0, I2C_GATE);

	return ret;
}
#endif

static struct dvb_frontend_ops tas2101_ops = {
	.delsys = { SYS_DVBS, SYS_DVBS2 },
	.info = {
		.name = "Tmax TAS2101",
		.frequency_min_hz = 950 * MHz,
		.frequency_max_hz = 2150 * MHz,
		.symbol_rate_min = 1000000,
		.symbol_rate_max = 45000000,
		.caps = FE_CAN_INVERSION_AUTO |
			FE_CAN_FEC_1_2 | FE_CAN_FEC_2_3 | FE_CAN_FEC_3_4 |
			FE_CAN_FEC_4_5 | FE_CAN_FEC_5_6 | FE_CAN_FEC_6_7 |
			FE_CAN_FEC_7_8 | FE_CAN_FEC_AUTO |
			FE_CAN_2G_MODULATION |
			FE_CAN_QPSK | FE_CAN_RECOVER
	},
	.release = tas2101_release,

	.init = tas2101_initfe,
	.sleep = tas2101_sleep,
#ifndef TAS2101_USE_I2C_MUX
	.i2c_gate_ctrl = tas2101_i2c_gate_ctrl,
#endif
	.read_status = tas2101_read_status,
	.read_ber = tas2101_read_ber,
	.read_signal_strength = tas2101_read_signal_strength,
	.read_snr = tas2101_read_snr,
	.read_ucblocks = tas2101_read_ucblocks,

	.set_tone = tas2101_set_tone,
	.set_voltage = tas2101_set_voltage,
	.diseqc_send_master_cmd = tas2101_send_diseqc_msg,
	.diseqc_send_burst = tas2101_diseqc_send_burst,
	.get_frontend_algo = tas2101_get_algo,
	.tune = tas2101_tune,

	.set_frontend = tas2101_set_frontend,
	.get_frontend = tas2101_get_frontend,

	.spi_read			= tas2101_spi_read,
	.spi_write			= tas2101_spi_write,
	.eeprom_read            = tas2101_eeprom_read,
	.eeprom_write           = tas2101_eeprom_write


};

MODULE_DESCRIPTION("DVB Frontend module for Tmax TAS2101");
MODULE_AUTHOR("Luis Alves (ljalvs@gmail.com)");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

