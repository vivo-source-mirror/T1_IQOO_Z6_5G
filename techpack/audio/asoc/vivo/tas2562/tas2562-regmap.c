/*
 * ALSA SoC Texas Instruments TAS2562 High Performance 4W Smart Amplifier
 *
 * Copyright (C) 2016 Texas Instruments, Inc.
 *
 * Author: saiprasad
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#ifdef CONFIG_TAS2562_REGMAP

#define DEBUG 5
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <sound/soc.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/syscalls.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <dsp/smart_amp.h>
#include <linux/fcntl.h>

#include "tas2562.h"
#include "tas2562-codec.h"
#include "tas2562-misc.h"
#include "tas2562-calib.h"
#include "smartpa-debug-common.h"

#if defined(VIVO_PROJECT_MODEL)
/*GDPR code*/
#else
/* not GDPR code start */
#ifdef CONFIG_RSC_VAUDIT_DEBUG
#include <linux/vivo_rsc/rsc_vaudit.h>
#include <linux/vivo_rsc/rsc_audio.h>
#endif
/* not GDPR code end */
#endif

static char pICN[] = {0x00, 0x00, 0x95, 0x2c};
static char pICN2[] = {0x00, 0x00, 0x04, 0xb8};
static char delay_reg[] = {0x00, 0x00, 0x03, 0xc0};
static char delay_reg2[] = {0x00, 0x01, 0x77, 0x00};

#ifdef VIVO_PORT_SMARTPA
static struct tas2562_priv *tas2562_priv;
int smartpa_init_dbg(char *buffer, char *buffer1,int size);
int smartpa_read_freq_dbg(char *buffer, int size);
void smartpa_read_prars_dbg(int temp[5], unsigned char addr);
void smartpa_get_client(struct i2c_client **client, unsigned char addr);
int smartpa_check_calib_dbg(void);
static bool smartpa_check_re(void);
static int smartpa_calib_get(uint32_t* calib_value);
static bool rdc_check_valid(uint32_t rdc, uint8_t iter);

typedef enum {
	READ_RE = 1,
	READ_F0,
	READ_Q,
	READ_Tv,
	CALIB_INIT,
	CALIB_DEINIT,
	SET_PROFILE,
	SET_RE,
	OPTS_MAX
} param_id_t;

#define STR_SZ_TAS 256

typedef enum {
	MODE_UNDEFINED,
	MODE_READ,
	MODE_WRITE
}mode_rw;

static void smartpa_set_re(uint32_t *calibRe);
#endif

/* add temp Re value  */
uint32_t re_calib[2] = {0};
/* end */

void smartpa_init_re_value(uint32_t *re_value)
{
	memcpy(re_calib, re_value, sizeof(re_calib));
	memcpy(tas2562_priv->calibRe,re_calib,sizeof(re_calib));
	pr_info("[SmartPA]%s: re_value: %x,%x\n", __func__, *re_value, *(re_value+1));
	pr_info("[SmartPA]%s: re_calib: %x,%x\n", __func__, re_calib[0],re_calib[1]);
}

static int tas2562_change_book_page(struct tas2562_priv *pTAS2562, enum channel chn,
	int book, int page)
{
	int nResult = 0;


	if((chn&channel_left) || (pTAS2562->mnChannels == 1))
	{
		pTAS2562->client->addr = pTAS2562->mnLAddr;
		if (pTAS2562->mnLCurrentBook != book) {
			nResult = regmap_write(pTAS2562->regmap, TAS2562_BOOKCTL_PAGE, 0);
			if (nResult < 0) {
				dev_err(pTAS2562->dev, "addr:0x%x -> %s, ERROR, Line=%d, E=%d\n",
					pTAS2562->client->addr,__func__, __LINE__, nResult);
				goto end;
			}
			pTAS2562->mnLCurrentPage = 0;
			nResult = regmap_write(pTAS2562->regmap, TAS2562_BOOKCTL_REG, book);
			if (nResult < 0) {
				dev_err(pTAS2562->dev, "addr:0x%x -> %s, ERROR, Line=%d, E=%d\n",
					pTAS2562->client->addr,__func__, __LINE__, nResult);
				goto end;
			}
			pTAS2562->mnLCurrentBook = book;
		}

		if (pTAS2562->mnLCurrentPage != page) {
			nResult = regmap_write(pTAS2562->regmap, TAS2562_BOOKCTL_PAGE, page);
			if (nResult < 0) {
				dev_err(pTAS2562->dev, "addr:0x%x -> %s, ERROR, Line=%d, E=%d\n",
					pTAS2562->client->addr,__func__, __LINE__, nResult);
				goto end;
			}
			pTAS2562->mnLCurrentPage = page;
		}
	}

	if((chn&channel_right) && (pTAS2562->mnChannels == 2))
	{
		pTAS2562->client->addr = pTAS2562->mnRAddr;
		if (pTAS2562->mnRCurrentBook != book) {
			nResult = regmap_write(pTAS2562->regmap, TAS2562_BOOKCTL_PAGE, 0);
			if (nResult < 0) {
				dev_err(pTAS2562->dev, "addr:0x%x -> %s, ERROR, Line=%d, E=%d\n",
					pTAS2562->client->addr,__func__, __LINE__, nResult);
				goto end;
			}
			pTAS2562->mnRCurrentPage = 0;
			nResult = regmap_write(pTAS2562->regmap, TAS2562_BOOKCTL_REG, book);
			if (nResult < 0) {
				dev_err(pTAS2562->dev, "addr:0x%x -> %s, ERROR, Line=%d, E=%d\n",
					pTAS2562->client->addr,__func__, __LINE__, nResult);
				goto end;
			}
			pTAS2562->mnRCurrentBook = book;
		}

		if (pTAS2562->mnRCurrentPage != page) {
			nResult = regmap_write(pTAS2562->regmap, TAS2562_BOOKCTL_PAGE, page);
			if (nResult < 0) {
				dev_err(pTAS2562->dev, "addr:0x%x -> %s, ERROR, Line=%d, E=%d\n",
					pTAS2562->client->addr,__func__, __LINE__, nResult);
				goto end;
			}
			pTAS2562->mnRCurrentPage = page;
		}
	}

end:
	return nResult;
}

static int tas2562_dev_read(struct tas2562_priv *pTAS2562, enum channel chn,
	unsigned int reg, unsigned int *pValue)
{
	int nResult = 0;

	mutex_lock(&pTAS2562->dev_lock);

	nResult = tas2562_change_book_page(pTAS2562, chn,
		TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg));
	if (nResult < 0)
		goto end;

	if((chn == channel_left) || (pTAS2562->mnChannels == 1))
		pTAS2562->client->addr = pTAS2562->mnLAddr;
	else if(chn == channel_right)
		pTAS2562->client->addr = pTAS2562->mnRAddr;
	else
	{
		dev_err(pTAS2562->dev, "%s, wrong channel number\n", __func__);
	}

	nResult = regmap_read(pTAS2562->regmap, TAS2562_PAGE_REG(reg), pValue);
	if (nResult < 0)
		dev_err(pTAS2562->dev, "addr:0x%x -> %s, ERROR, Line=%d, E=%d\n",
			pTAS2562->client->addr,__func__, __LINE__, nResult);
	else
		dev_dbg(pTAS2562->dev, "addr:0x%x -> %s: chn:%x:BOOK:PAGE:REG %u:%u:%u,0x%x\n", pTAS2562->client->addr,__func__,
			pTAS2562->client->addr, TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg),
			TAS2562_PAGE_REG(reg), *pValue);

end:
	mutex_unlock(&pTAS2562->dev_lock);
	return nResult;
}

static int tas2562_dev_write(struct tas2562_priv *pTAS2562, enum channel chn,
	unsigned int reg, unsigned int value)
{
	int nResult = 0;
	static int i2c_error_l = 1;
	static int i2c_error_r = 1;

	mutex_lock(&pTAS2562->dev_lock);

	nResult = tas2562_change_book_page(pTAS2562, chn,
		TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg));
	if (nResult < 0)
		goto end;

	if((chn&channel_left) || (pTAS2562->mnChannels == 1))
	{
		pTAS2562->client->addr = pTAS2562->mnLAddr;

		nResult = regmap_write(pTAS2562->regmap, TAS2562_PAGE_REG(reg),
			value);
		if (nResult < 0) {
			dev_err(pTAS2562->dev, "addr:0x%x -> %s, ERROR, Line=%d, E=%d\n",
				pTAS2562->client->addr,__func__, __LINE__, nResult);
			if (i2c_error_l) {
#if defined(VIVO_PROJECT_MODEL)
		/*GDPR code*/
#else
		/* not GDPR code start */
#ifdef CONFIG_RSC_VAUDIT_DEBUG
				rsc_vaudit(VAUDIT_AUDIO, AUDIO_IIC_BUS, 0,
					"TAS2562 ERROR: chn%x:BOOK:PAGE:REG %u:%u:%u, VAL: 0x%02x, ret: %d.",
					pTAS2562->client->addr, TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg),
					TAS2562_PAGE_REG(reg), value, nResult);
#endif
		/* not GDPR code end */
#endif
				i2c_error_l = 0;
			}
		} else {
			dev_dbg(pTAS2562->dev, "addr:0x%x -> %s: chn%x:BOOK:PAGE:REG %u:%u:%u, VAL: 0x%02x\n",
				pTAS2562->client->addr,__func__, pTAS2562->client->addr,
				TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg),
				TAS2562_PAGE_REG(reg), value);
		}
	}

	if((chn&channel_right) && (pTAS2562->mnChannels == 2))
	{
		pTAS2562->client->addr = pTAS2562->mnRAddr;

		nResult = regmap_write(pTAS2562->regmap, TAS2562_PAGE_REG(reg),
			value);
		if (nResult < 0) {
			dev_err(pTAS2562->dev, "addr:0x%x -> %s, ERROR, Line=%d, E=%d\n",
				pTAS2562->client->addr,__func__, __LINE__, nResult);
			if (i2c_error_r) {
#if defined(VIVO_PROJECT_MODEL)
		/*GDPR code*/
#else
		/* not GDPR code start */
#ifdef CONFIG_RSC_VAUDIT_DEBUG
				rsc_vaudit(VAUDIT_AUDIO, AUDIO_IIC_BUS, 0,
					"TAS2562 ERROR: chn%x:BOOK:PAGE:REG %u:%u:%u, VAL: 0x%02x, ret: %d.",
					pTAS2562->client->addr, TAS2562_BOOK_ID(reg),
					TAS2562_PAGE_ID(reg), TAS2562_PAGE_REG(reg), value, nResult);
#endif
		/* not GDPR code end */
#endif
				i2c_error_r = 0;
			}
		} else {
			dev_dbg(pTAS2562->dev, "addr:0x%x -> %s: chn%x:BOOK:PAGE:REG %u:%u:%u, VAL: 0x%02x\n",
				pTAS2562->client->addr,__func__, pTAS2562->client->addr, TAS2562_BOOK_ID(reg),
				TAS2562_PAGE_ID(reg), TAS2562_PAGE_REG(reg), value);
		}
	}

end:
	mutex_unlock(&pTAS2562->dev_lock);
	return nResult;
}

static int tas2562_dev_bulk_write(struct tas2562_priv *pTAS2562, enum channel chn,
	unsigned int reg, unsigned char *pData, unsigned int nLength)
{
	int nResult = 0;

	mutex_lock(&pTAS2562->dev_lock);

	nResult = tas2562_change_book_page(pTAS2562, chn,
		TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg));
	if (nResult < 0)
		goto end;

	if((chn&channel_left) || (pTAS2562->mnChannels == 1))
	{
		pTAS2562->client->addr = pTAS2562->mnLAddr;
		nResult = regmap_bulk_write(pTAS2562->regmap,
			TAS2562_PAGE_REG(reg), pData, nLength);
		if (nResult < 0)
			dev_err(pTAS2562->dev, "addr:0x%x -> %s, ERROR, Line=%d, E=%d\n",
				pTAS2562->client->addr,__func__, __LINE__, nResult);
		else
			dev_dbg(pTAS2562->dev, "addr:0x%x -> %s: chn%x:BOOK:PAGE:REG %u:%u:%u, len: 0x%02x\n",
				pTAS2562->client->addr,__func__, pTAS2562->client->addr,
				TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg),
				TAS2562_PAGE_REG(reg), nLength);
	}

	if((chn&channel_right) && (pTAS2562->mnChannels == 2))
	{
		pTAS2562->client->addr = pTAS2562->mnRAddr;
				nResult = regmap_bulk_write(pTAS2562->regmap,
			TAS2562_PAGE_REG(reg), pData, nLength);
		if (nResult < 0)
			dev_err(pTAS2562->dev, "addr:0x%x -> %s, ERROR, Line=%d, E=%d\n",
				pTAS2562->client->addr,__func__, __LINE__, nResult);
		else
			dev_dbg(pTAS2562->dev, "addr:0x%x -> %s: %x:BOOK:PAGE:REG %u:%u:%u, len: 0x%02x\n",
				pTAS2562->client->addr,__func__, pTAS2562->client->addr,
				TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg),
				TAS2562_PAGE_REG(reg), nLength);
	}

end:
	mutex_unlock(&pTAS2562->dev_lock);
	return nResult;
}

static int tas2562_dev_bulk_read(struct tas2562_priv *pTAS2562, enum channel chn,
	unsigned int reg, unsigned char *pData, unsigned int nLength)
{
	int nResult = 0;

	mutex_lock(&pTAS2562->dev_lock);

	if((chn == channel_left) || (pTAS2562->mnChannels == 1))
		pTAS2562->client->addr = pTAS2562->mnLAddr;
	else if(chn == channel_right)
		pTAS2562->client->addr = pTAS2562->mnRAddr;
	else
	{
		dev_err(pTAS2562->dev, "addr:0x%x -> %s, wrong channel number\n", pTAS2562->client->addr,__func__);
	}

	nResult = tas2562_change_book_page(pTAS2562, chn,
		TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg));
	if (nResult < 0)
		goto end;

	nResult = regmap_bulk_read(pTAS2562->regmap,
	TAS2562_PAGE_REG(reg), pData, nLength);
	if (nResult < 0)
		dev_err(pTAS2562->dev, "addr:0x%x -> %s, ERROR, Line=%d, E=%d\n",
			pTAS2562->client->addr,__func__, __LINE__, nResult);
	else
		dev_dbg(pTAS2562->dev, "addr:0x%x -> %s: chn%x:BOOK:PAGE:REG %u:%u:%u, len: 0x%02x\n",
			pTAS2562->client->addr,__func__, pTAS2562->client->addr,
			TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg),
			TAS2562_PAGE_REG(reg), nLength);
end:
	mutex_unlock(&pTAS2562->dev_lock);
	return nResult;
}

static int tas2562_dev_update_bits(struct tas2562_priv *pTAS2562, enum channel chn,
	unsigned int reg, unsigned int mask, unsigned int value)
{
	int nResult = 0;

	mutex_lock(&pTAS2562->dev_lock);
	nResult = tas2562_change_book_page(pTAS2562, chn,
		TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg));
	if (nResult < 0)
		goto end;

	if((chn&channel_left) || (pTAS2562->mnChannels == 1))
	{
		pTAS2562->client->addr = pTAS2562->mnLAddr;
		nResult = regmap_update_bits(pTAS2562->regmap,
			TAS2562_PAGE_REG(reg), mask, value);
		if (nResult < 0)
			dev_err(pTAS2562->dev, "addr:0x%x -> %s, ERROR, Line=%d, E=%d\n",
				pTAS2562->client->addr,__func__, __LINE__, nResult);
		else
			dev_dbg(pTAS2562->dev, "addr:0x%x -> %s: chn%x:BOOK:PAGE:REG %u:%u:%u, mask: 0x%x, val: 0x%x\n",
				pTAS2562->client->addr,__func__, pTAS2562->client->addr,
				TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg),
				TAS2562_PAGE_REG(reg), mask, value);
	}

	if((chn&channel_right) && (pTAS2562->mnChannels == 2))
	{
		pTAS2562->client->addr = pTAS2562->mnRAddr;
		nResult = regmap_update_bits(pTAS2562->regmap,
			TAS2562_PAGE_REG(reg), mask, value);
		if (nResult < 0)
			dev_err(pTAS2562->dev, "addr:0x%x -> %s, ERROR, Line=%d, E=%d\n",
				pTAS2562->client->addr,__func__, __LINE__, nResult);
		else
			dev_dbg(pTAS2562->dev, "addr:0x%x -> %s: chn%x:BOOK:PAGE:REG %u:%u:%u, mask: 0x%x, val: 0x%x\n",
				pTAS2562->client->addr,__func__,pTAS2562->client->addr,
				TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg),
				TAS2562_PAGE_REG(reg), mask, value);
	}

end:
	mutex_unlock(&pTAS2562->dev_lock);
	return nResult;
}

static bool tas2562_volatile(struct device *dev, unsigned int reg)
{
	return true;
}

static bool tas2562_writeable(struct device *dev, unsigned int reg)
{
	return true;
}
static const struct regmap_config tas2562_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = tas2562_writeable,
	.volatile_reg = tas2562_volatile,
	.cache_type = REGCACHE_NONE,
	.max_register = 1 * 128,
};


static void tas2562_hw_reset(struct tas2562_priv *pTAS2562)
{
	if (gpio_is_valid(pTAS2562->mnResetGPIO)) {
		gpio_direction_output(pTAS2562->mnResetGPIO, 0);
		if (pTAS2562->mnChannels == 2)
			gpio_direction_output(pTAS2562->mnResetGPIO2, 0);
		msleep(5);
		gpio_direction_output(pTAS2562->mnResetGPIO, 1);
		if (pTAS2562->mnChannels == 2)
			gpio_direction_output(pTAS2562->mnResetGPIO2, 1);
		msleep(2);
	}
	dev_err(pTAS2562->dev, "gpio up !!\n");

	pTAS2562->mnLCurrentBook = -1;
	pTAS2562->mnLCurrentPage = -1;
	pTAS2562->mnRCurrentBook = -1;
	pTAS2562->mnRCurrentPage = -1;
}

void tas2562_enableIRQ(struct tas2562_priv *pTAS2562, bool enable)
{
	if (enable) {
		if (gpio_is_valid(pTAS2562->mnIRQGPIO))
			enable_irq(pTAS2562->mnIRQ);
		if (pTAS2562->mnChannels == 2
			&& gpio_is_valid(pTAS2562->mnIRQGPIO2))
			enable_irq(pTAS2562->mnIRQ2);

		pTAS2562->mbIRQEnable = true;
	} else {
		if (gpio_is_valid(pTAS2562->mnIRQGPIO))
			disable_irq_nosync(pTAS2562->mnIRQ);
		if (pTAS2562->mnChannels == 2
			&& gpio_is_valid(pTAS2562->mnIRQGPIO2))
			disable_irq_nosync(pTAS2562->mnIRQ2);
		pTAS2562->mbIRQEnable = false;
	}
}

static void irq_work_routine(struct work_struct *work)
{
	struct tas2562_priv *pTAS2562 =
		container_of(work, struct tas2562_priv, irq_work.work);
	unsigned int nDevInt1Status = 0, nDevInt2Status = 0, nDevInt3Status = 0, nDevInt4Status = 0;
	int nCounter = 2;
	int nResult = 0;
	int irqreg;
	enum channel chn;

	dev_info(pTAS2562->dev, "%s\n", __func__);
#ifdef CONFIG_TAS2562_CODEC
	mutex_lock(&pTAS2562->codec_lock);
#endif
	tas2562_enableIRQ(pTAS2562, false);
	if (pTAS2562->mbRuntimeSuspend) {
		dev_info(pTAS2562->dev, "%s, Runtime Suspended\n", __func__);
		goto end;
	}

	if (pTAS2562->mnPowerState == TAS2562_POWER_SHUTDOWN) {
		dev_info(pTAS2562->dev, "%s, device not powered\n", __func__);
		goto end;
	}

	nResult = pTAS2562->write(pTAS2562, channel_both, TAS2562_InterruptMaskReg0,
				TAS2562_InterruptMaskReg0_Disable);
	nResult = pTAS2562->write(pTAS2562, channel_both, TAS2562_InterruptMaskReg1,
				TAS2562_InterruptMaskReg1_Disable);

	if (nResult < 0)
		goto reload;

	if((pTAS2562->spk_l_control == 1) && (pTAS2562->spk_r_control == 1) && (pTAS2562->mnChannels == 2))
		chn = channel_both;
	else if(pTAS2562->spk_l_control == 1)
		chn = channel_left;
	else if((pTAS2562->spk_r_control == 1) && (pTAS2562->mnChannels == 2))
		chn = channel_right;
	else
		chn = channel_left; 

	if(chn & channel_left)
		nResult = pTAS2562->read(pTAS2562, channel_left, TAS2562_LatchedInterruptReg0, &nDevInt1Status);
	if (nResult >= 0)
		nResult = pTAS2562->read(pTAS2562, channel_left, TAS2562_LatchedInterruptReg1, &nDevInt2Status);
	else
		goto reload;

	if(chn & channel_right)
		nResult = pTAS2562->read(pTAS2562, channel_right, TAS2562_LatchedInterruptReg0, &nDevInt3Status);
	if (nResult >= 0)
		nResult = pTAS2562->read(pTAS2562, channel_right, TAS2562_LatchedInterruptReg1, &nDevInt4Status);
	else
		goto reload;

	dev_dbg(pTAS2562->dev, "IRQ status : 0x%x, 0x%x, 0x%x, 0x%x\n",
			nDevInt3Status, nDevInt4Status, nDevInt3Status, nDevInt4Status);

	if (((nDevInt1Status & 0x7) != 0) || ((nDevInt2Status & 0x0f) != 0) ||
		((nDevInt3Status & 0x7) != 0) || ((nDevInt4Status & 0x0f) != 0)) {
		/* in case of INT_CLK, INT_OC, INT_OT, INT_OVLT, INT_UVLT, INT_BO */

		if ((nDevInt1Status & TAS2562_LatchedInterruptReg0_TDMClockErrorSticky_Interrupt) ||
		 	(nDevInt3Status & TAS2562_LatchedInterruptReg0_TDMClockErrorSticky_Interrupt)) {
			pTAS2562->mnErrCode |= ERROR_CLOCK;
			dev_err(pTAS2562->dev, "TDM clock error!\n");
		} else
			pTAS2562->mnErrCode &= ~ERROR_OVER_CURRENT;

		if ((nDevInt1Status & TAS2562_LatchedInterruptReg0_OCEFlagSticky_Interrupt) ||
		 	(nDevInt3Status & TAS2562_LatchedInterruptReg0_OCEFlagSticky_Interrupt)) {
			pTAS2562->mnErrCode |= ERROR_OVER_CURRENT;
			dev_err(pTAS2562->dev, "SPK over current!\n");
		} else
			pTAS2562->mnErrCode &= ~ERROR_OVER_CURRENT;

		if ((nDevInt1Status & TAS2562_LatchedInterruptReg0_OTEFlagSticky_Interrupt) ||
			(nDevInt3Status & TAS2562_LatchedInterruptReg0_OTEFlagSticky_Interrupt)) {
			pTAS2562->mnErrCode |= ERROR_DIE_OVERTEMP;
			dev_err(pTAS2562->dev, "die over temperature!\n");
		} else
			pTAS2562->mnErrCode &= ~ERROR_DIE_OVERTEMP;

		if ((nDevInt2Status & TAS2562_LatchedInterruptReg1_VBATOVLOSticky_Interrupt) ||
			(nDevInt4Status & TAS2562_LatchedInterruptReg1_VBATOVLOSticky_Interrupt)) {
			pTAS2562->mnErrCode |= ERROR_OVER_VOLTAGE;
			dev_err(pTAS2562->dev, "SPK over voltage!\n");
		} else
			pTAS2562->mnErrCode &= ~ERROR_UNDER_VOLTAGE;

		if ((nDevInt2Status & TAS2562_LatchedInterruptReg1_VBATUVLOSticky_Interrupt) ||
			(nDevInt4Status & TAS2562_LatchedInterruptReg1_VBATUVLOSticky_Interrupt)) {
			pTAS2562->mnErrCode |= ERROR_UNDER_VOLTAGE;
			dev_err(pTAS2562->dev, "SPK under voltage!\n");
		} else
			pTAS2562->mnErrCode &= ~ERROR_UNDER_VOLTAGE;

		if ((nDevInt2Status & TAS2562_LatchedInterruptReg1_BrownOutFlagSticky_Interrupt) ||
			(nDevInt4Status & TAS2562_LatchedInterruptReg1_BrownOutFlagSticky_Interrupt)) {
			pTAS2562->mnErrCode |= ERROR_BROWNOUT;
			dev_err(pTAS2562->dev, "brownout!\n");
		} else
			pTAS2562->mnErrCode &= ~ERROR_BROWNOUT;

		goto reload;
	} else {
		nCounter = 2;

		while (nCounter > 0) {
			if(chn & channel_left)
				nResult = pTAS2562->read(pTAS2562, channel_left, TAS2562_PowerControl, &nDevInt1Status);
			if (nResult < 0)
				goto reload;
			if(chn & channel_right)
			nResult = pTAS2562->read(pTAS2562, channel_right, TAS2562_PowerControl, &nDevInt3Status);
			if (nResult < 0)
				goto reload;

			if ((nDevInt1Status & TAS2562_PowerControl_OperationalMode10_Mask)
				!= TAS2562_PowerControl_OperationalMode10_Shutdown) {
				/* If only left should be power on */
				if(chn == channel_left)
					break;
				/* If both should be power on */
				if ((nDevInt3Status & TAS2562_PowerControl_OperationalMode10_Mask)
					!= TAS2562_PowerControl_OperationalMode10_Shutdown)			
				break;
			}
			/*If only right should be power on */
			else if (chn == channel_right) {
				if ((nDevInt3Status & TAS2562_PowerControl_OperationalMode10_Mask)
					!= TAS2562_PowerControl_OperationalMode10_Shutdown)				
				break;
			}

			pTAS2562->read(pTAS2562, channel_left, TAS2562_LatchedInterruptReg0, &irqreg);
			dev_info(pTAS2562->dev, "IRQ reg is: %s %d, %d\n", __func__, irqreg, __LINE__);
			pTAS2562->read(pTAS2562, channel_right, TAS2562_LatchedInterruptReg0, &irqreg);
			dev_info(pTAS2562->dev, "IRQ reg is: %s %d, %d\n", __func__, irqreg, __LINE__);

			nResult = pTAS2562->update_bits(pTAS2562, chn, TAS2562_PowerControl,
				TAS2562_PowerControl_OperationalMode10_Mask,
				TAS2562_PowerControl_OperationalMode10_Active);
			if (nResult < 0)
				goto reload;

			dev_info(pTAS2562->dev, "set ICN to -80dB\n");
			// Make sure Power Up is completed.
			msleep(2);
			nResult = pTAS2562->write(pTAS2562, channel_both,
					TAS2562_REG(0x0, 0x01, 0x21), 0x08);
			if ((pTAS2562->mnChannels == 2) && pTAS2562->spk_r_control) {
				nResult = pTAS2562->bulk_write(pTAS2562, chn, TAS2562_DELAY_REG, delay_reg2, 4);
				/* Solve the receiver low noise 0x10 -> 0x30 */
				nResult = pTAS2562->write(pTAS2562, channel_both,
						TAS2562_REG(0x0, 0x0, 0x3e), 0x30);
				nResult = pTAS2562->bulk_write(pTAS2562, chn, TAS2562_ICN_REG, pICN2, 4);
				nResult = pTAS2562->write(pTAS2562, channel_both,
						TAS2562_REG(0x0, 0xfd, 0x0d), 0x0d);
				nResult = pTAS2562->write(pTAS2562, channel_both,
						TAS2562_REG(0x0, 0xfd, 0x12), 0x00);
				nResult = pTAS2562->write(pTAS2562, channel_both,
						TAS2562_REG(0x0, 0xfd, 0x46), 0x1f);
				nResult = pTAS2562->write(pTAS2562, channel_both,
						TAS2562_REG(0x0, 0xfd, 0x47), 0x00);
				nResult = pTAS2562->write(pTAS2562, channel_both,
						TAS2562_REG(0x0, 0xfd, 0x64), 0x10);
			} else if (pTAS2562->mnChannels == 2) {
				nResult = pTAS2562->bulk_write(pTAS2562, chn, TAS2562_DELAY_REG, delay_reg, 4);
				/* Solve the receiver low noise 0x10 */
				nResult = pTAS2562->write(pTAS2562, channel_left,
						TAS2562_REG(0x0, 0x0, 0x3e), 0x02);
				nResult = pTAS2562->bulk_write(pTAS2562, chn, TAS2562_ICN_REG, pICN, 4);
				nResult = pTAS2562->write(pTAS2562, channel_left,
						TAS2562_REG(0x0, 0xfd, 0x0d), 0x0d);
				nResult = pTAS2562->write(pTAS2562, channel_left,
						TAS2562_REG(0x0, 0xfd, 0x12), 0xc0);
				nResult = pTAS2562->write(pTAS2562, channel_left,
						TAS2562_REG(0x0, 0xfd, 0x46), 0x1f);
				nResult = pTAS2562->write(pTAS2562, channel_left,
						TAS2562_REG(0x0, 0xfd, 0x47), 0x04);
				nResult = pTAS2562->write(pTAS2562, channel_left,
						TAS2562_REG(0x0, 0xfd, 0x64), 0x14);
			} else if (pTAS2562->mnChannels == 1) {
				nResult = pTAS2562->bulk_write(pTAS2562, chn, TAS2562_DELAY_REG, delay_reg2, 4);
				/* Solve the receiver low noise 0x10 */
				nResult = pTAS2562->write(pTAS2562, channel_both,
						TAS2562_REG(0x0, 0x0, 0x3e), 0x10);
				nResult = pTAS2562->bulk_write(pTAS2562, chn, TAS2562_ICN_REG, pICN2, 4);
			}

			pTAS2562->read(pTAS2562, channel_left, TAS2562_LatchedInterruptReg0, &irqreg);
			dev_info(pTAS2562->dev, "IRQ reg is: %s, %d, %d\n", __func__, irqreg, __LINE__);
			pTAS2562->read(pTAS2562, channel_right, TAS2562_LatchedInterruptReg0, &irqreg);
			dev_info(pTAS2562->dev, "IRQ reg is: %s %d, %d\n", __func__, irqreg, __LINE__);

			nCounter--;
			if (nCounter > 0) {
				/* in case check pow status just after power on TAS2562 */
				dev_dbg(pTAS2562->dev, "PowSts B: 0x%x, check again after 10ms\n",
					nDevInt1Status);
				msleep(10);
			}
		}

		if ((((nDevInt1Status & TAS2562_PowerControl_OperationalMode10_Mask)
			== TAS2562_PowerControl_OperationalMode10_Shutdown) && (chn & channel_left))
			|| (((nDevInt3Status & TAS2562_PowerControl_OperationalMode10_Mask)
			== TAS2562_PowerControl_OperationalMode10_Shutdown) && (chn & channel_right)))
		{
			dev_err(pTAS2562->dev, "%s, Critical ERROR REG[0x%x] = 0x%x\n",
				__func__,
				TAS2562_PowerControl,
				nDevInt1Status);
			pTAS2562->mnErrCode |= ERROR_CLASSD_PWR;
			goto reload;
		}
		pTAS2562->mnErrCode &= ~ERROR_CLASSD_PWR;
	}

	nResult = pTAS2562->write(pTAS2562, chn, TAS2562_InterruptMaskReg0, 0xf8);
	if (nResult < 0)
		goto reload;

	nResult = pTAS2562->write(pTAS2562, chn, TAS2562_InterruptMaskReg1, 0xb1);
	if (nResult < 0)
		goto reload;

	goto end;

reload:
	/* hardware reset and reload */
	tas2562_LoadConfig(pTAS2562);


end:
	tas2562_enableIRQ(pTAS2562, true);
#ifdef CONFIG_TAS2562_CODEC
	mutex_unlock(&pTAS2562->codec_lock);
#endif
}

#ifdef VIVO_PORT_SMARTPA
#define CALIBRATE_FILE   "/mnt/vendor/persist/audio/smartamp.bin"
#define FREQ_FILE   "/data/engineermode/speakerleak"
#define MAX_CONTROL_NAME        48

static ssize_t smartpa_calibrate_show(struct kobject *kobj, struct kobj_attribute *attr, char *buffer)
{
	int ret = 0;
	struct afe_smartamp_set_params_t prot_config;
	uint32_t data = 0;
	int nSize = sizeof(uint32_t);
	/*u32 length = TAS_PAYLOAD_SIZE;*/
	uint32_t paramid = 0;
	uint32_t calib_re[MAX_CHANNELS];
	const int size = 512;
	int ret_count = 0;
	uint8_t iter = 0, channels = 1;
	memset(&prot_config, 0, sizeof(prot_config));

	pr_info("%s\n", __func__);

	if (!tas2562_priv) {
		pr_err("[SmartPA-%d]: SmartPA_priv is NULL\n", __LINE__);
		return -1;
	}
	channels = tas2562_priv->mnChannels;
	
	//calib init
	for(iter = 0; iter < channels; iter++)
	{
		data = 1;//Value is ignored
		memcpy(prot_config.payload, &data, nSize);
		ret = afe_smartamp_set_calib_data(((AFE_SA_CALIB_INIT)|((iter+1)<<24)|(1<<16))
			, &prot_config, nSize, AFE_SMARTAMP_MODULE);
		if (ret < 0)
			goto end;
	}
	pr_info("[SmartPA-%d]dbgfs_calibrate_read: calib init\n", __LINE__);

	msleep(2*1000);


	//get Re
	for(iter = 0; iter < channels; iter++)
	{
		paramid = ((AFE_SA_GET_RE)|((iter+1)<<24)|(1<<16));
		ret = afe_smartamp_algo_ctrl((u8*)&data, paramid,
				TAS_GET_PARAM, /*length */ 4, AFE_SMARTAMP_MODULE);
		if (ret < 0)
			goto deinit;

		calib_re[iter] = data;

		if ((calib_re[iter] < tas2562_priv->imped_min[iter]) || (calib_re[iter] > tas2562_priv->imped_max[iter]))
			calib_re[iter] = CALIB_FAILED;

		tas2562_priv->calibRe[iter] = calib_re[iter];
		pr_info("[SmartPa-%d]init_dbg: update Re value\n", __LINE__);
		pr_info("[SmartPA-%d]debugfs calib_re[%d] is %d\n", __LINE__, iter, calib_re[iter]);
	}

	ret = 0;
	if(channels == 2)
		ret = scnprintf(buffer+ret_count, size-ret_count, "Channel[0] = %d; Channel[1] = %d;", calib_re[0], calib_re[1]);
	else
		ret = scnprintf(buffer+ret_count, size-ret_count, "Channel[0] = %d;", calib_re[0]);	
	buffer[ret_count] = 0;
	pr_info("[SmartPA-%d]debugfs ret_count %d, ret %d\n", __LINE__, (int)ret_count, (int)ret); 

deinit:
	for(iter = 0; iter < channels; iter++)
	{
		data = 0;//Value is ignored
		memcpy(prot_config.payload, &data, nSize);
		ret = afe_smartamp_set_calib_data(((AFE_SA_CALIB_DEINIT)|((iter+1)<<24)|(1<<16)), &prot_config
			, nSize, AFE_SMARTAMP_MODULE);
	}
	pr_info("[SmartPA-%d]dbgfs_calibrate_read: decalib init\n", __LINE__);

end:
	pr_info("[SmartPA-%d]dbgfs_calibrate_read: end\n", __LINE__);
	
	if(ret < 0)
	{
		return ret;
	}else{
		return ret_count;
	}
}
ssize_t kernel_write1(struct file * file, const void * buf, size_t size, loff_t *count)
{
	pr_info("[SmartPA-%d]: enter %s,%x,%x,%x\n", __LINE__, __func__);
	return 0;
}

ssize_t kernel_read1(struct file * file, const void * buf, size_t size, loff_t *count)
{
	pr_info("[SmartPA-%d]: enter %s\n", __LINE__, __func__);
	return 0;
}


static ssize_t smartpa_f0qt_show(struct kobject *kobj, struct kobj_attribute *attr, char *buffer)
{
	u32 length = TAS_PAYLOAD_SIZE;
	uint32_t calibRe[MAX_CHANNELS];
	uint32_t F0[MAX_CHANNELS], Q[MAX_CHANNELS];
	int ret = 0, ret_count = 0;
	uint32_t data;
	uint32_t paramid = 0;
	int nSize = sizeof(uint32_t);
	struct afe_smartamp_set_params_t prot_config;
	//struct file *fp = NULL;
	//loff_t pos;
	const int size = 512;
	uint8_t iter = 0, channels = 1;
	pr_info("[SmartPA-%d]: enter %s\n", __LINE__, __func__);
	memset(&prot_config, 0, sizeof(prot_config));

	if (!tas2562_priv) {
		pr_err("[SmartPA-%d]: SmartPA_priv is NULL\n", __LINE__);
		return -1;
	}
	channels = tas2562_priv->mnChannels;
	
	//Load Calib
	if(smartpa_check_re()) {
		calibRe[0] = tas2562_priv->calibRe[0];
		if(channels == 2)
			calibRe[1] = tas2562_priv->calibRe[1];
		smartpa_set_re(calibRe);
	}
	
	for(iter = 0; iter < channels; iter++)
	{
		data = 1;//Value is ignored
		memcpy(prot_config.payload, &data, nSize);
		paramid = ((AFE_SA_F0_TEST_INIT) | (length << 16) | ((iter+1) << 24));
		ret = afe_smartamp_set_calib_data(paramid
			, &prot_config, nSize, AFE_SMARTAMP_MODULE);
		if (ret < 0)
			goto end;
	}
	//wait 5s
	msleep(5000);
	//read F0
	for(iter = 0; iter < channels; iter++)
	{
		data = 0;//Resets data to 0
		paramid = (AFE_SA_GET_F0 | (length << 16) | ((iter+1) << 24));
		ret = afe_smartamp_algo_ctrl((u8*)&data, paramid,
				TAS_GET_PARAM, /*length **/ 4, AFE_SMARTAMP_MODULE);
		if (ret < 0)
			goto end;
		F0[iter] = data;
		pr_err("[SmartPA-%d]: F0[%d] is %d\n", __LINE__, iter, F0[iter]);
	}

	ret = 0;
	if(channels == 2)
	{
		ret = scnprintf(buffer+ret_count, size-ret_count, "Channel[0] = %d; Channel[1] = %d;\n", F0[0], F0[1]);
	}
	else
		ret = scnprintf(buffer+ret_count, size-ret_count, "Channel[0] = %d;\n", F0[0]);
	buffer[ret_count] = 0;

	//read Q
	for(iter =0; iter < channels; iter++)
	{
		data = 0;//Reset data to 0
		paramid = (AFE_SA_GET_Q | (length << 16) | ((iter+1) << 24));
		ret = afe_smartamp_algo_ctrl((u8*)&data, paramid,
				TAS_GET_PARAM, /*length **/ 4, AFE_SMARTAMP_MODULE);
		if (ret < 0)
			goto deinit;
		Q[iter] = data;
		pr_err("[SmartPA-%d]: Q[%d] is %d\n", __LINE__, iter, Q[iter]);
	}
#if 0
	//write to file
	fp = filp_open(CALIBRATE_FILE, O_RDWR | O_CREAT, 0644);
	if (fp > 0) {
		pos = 0;
		nSize = kernel_write1(fp, (char *)&F0[0], sizeof(uint32_t), &pos);
		nSize = kernel_write1(fp, (char *)&Q[0], sizeof(uint32_t), &pos);
		pr_info("[SmartPA-%d] write to file channel[0], F0 = %d, Q = %d\n", __LINE__, F0[0], Q[0]);
		if(channels == 2)
		{
			nSize = kernel_write1(fp, (char *)&F0[1], sizeof(uint32_t), &pos);
			nSize = kernel_write1(fp, (char *)&Q[1], sizeof(uint32_t), &pos);
			pr_info("[SmartPA-%d] write to file channel[1], F0 = %d, Q = %d\n", __LINE__, F0[1], Q[1]);
		}
		filp_close(fp, NULL);
	}
#endif
deinit :
	for(iter = 0; iter < channels; iter++)
	{
		data = 0;//Value is ignored
		memcpy(prot_config.payload, &data, nSize);
		paramid = ((AFE_SA_F0_TEST_DEINIT) | (length << 16) | ((iter+1) << 24));
		ret = afe_smartamp_set_calib_data(paramid
			, &prot_config, nSize, AFE_SMARTAMP_MODULE);
		if (ret < 0)
			goto end;	
	}	
end:
	if(ret < 0)
	{
		return ret;
	}else{
		return ret_count;
	}
}

static ssize_t smartpa_i2c_show(struct kobject *kobj, struct kobj_attribute *attr, char *buffer)
{
	struct tas2562_priv *tas2562 = tas2562_priv;
	const int size = 512;
	int n = 0;

	if (!tas2562)
		return -ENOMEM;

	pr_info("[SmartPA-%d]%s enter.\n", __LINE__, __func__);

	if (tas2562->mnChannels == 2) {
		n += scnprintf(buffer+n, size-n, "SmartPA-stereo %d\n",
				tas2562->smartpa_i2c_check);
	} else {
		n += scnprintf(buffer+n, size-n, "SmartPA-mono %s\n",
				tas2562->smartpa_i2c_check ? "OK" : "ERROR");
	}
	buffer[n] = 0;

	return n;
}

#define DBGFS_REG_COUNT 15
static int dbgfs_reg[DBGFS_REG_COUNT] = {
	TAS2562_REG(0x00, 0x01, 0x21),
	TAS2562_REG(0x00, 0x02, 0x64),
	TAS2562_REG(0x00, 0x02, 0x65),
	TAS2562_REG(0x00, 0x02, 0x66),
	TAS2562_REG(0x00, 0x02, 0x67),
	TAS2562_REG(0x00, 0x02, 0x6c),
	TAS2562_REG(0x00, 0x02, 0x6d),
	TAS2562_REG(0x00, 0x02, 0x6e),
	TAS2562_REG(0x00, 0x02, 0x6f),
	TAS2562_REG(0x00, 0xfd, 0x0d),
	TAS2562_REG(0x00, 0xfd, 0x12),
	TAS2562_REG(0x00, 0xfd, 0x3d),
	TAS2562_REG(0x00, 0xfd, 0x46),
	TAS2562_REG(0x00, 0xfd, 0x47),
	TAS2562_REG(0x00, 0xfd, 0x64),
};

static ssize_t smartpa_reg_l_show(struct kobject *kobj, struct kobj_attribute *attr, char *buffer)
{
	struct tas2562_priv *tas2562 = tas2562_priv;
	u8 i;
	const int size = PAGE_SIZE;
	int data, n = 0;

	pr_info("[SmartPA-%d]%s: ======caught smartpa reg start ======\n", __LINE__, __func__);
	tas2562->client->addr = tas2562->mnLAddr;
	n += scnprintf(buffer+n, size-n, "i2c-addr: 0x%02x\n", tas2562->client->addr);
	for (i = 0; i < 128; i++) {
		tas2562->read(tas2562, channel_left, i, &data);
		n += scnprintf(buffer+n, size-n, "0x%02x:0x%02x\n", i, data);
	}
	for (i = 0; i < DBGFS_REG_COUNT; i++) {
		tas2562->read(tas2562, channel_left, dbgfs_reg[i], &data);
		n += scnprintf(buffer+n, size-n, "B0x%02xP0x%02xR0x%02x:0x%02x\n",
			TAS2562_BOOK_ID(dbgfs_reg[i]), TAS2562_PAGE_ID(dbgfs_reg[i]), TAS2562_PAGE_REG(dbgfs_reg[i]), data);
	}
	buffer[n] = 0;
	pr_info("[SmartPA-%d]%s: ======caught smartpa reg end ======\n", __LINE__, __func__);
	return n;
}

static ssize_t smartpa_reg_l_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *ubuf, size_t count)
{
	struct tas2562_priv *tas2562 = tas2562_priv;
	int ret = 0;
	unsigned int kbuf[4];
	char *temp;

	pr_info("[SmartPA-%d]%s: cnt %d\n", __LINE__, __func__, (int)count);
	tas2562->client->addr = tas2562->mnLAddr;
	if (count > 4) {
		temp = kmalloc(count, GFP_KERNEL);
		if (!temp) {
			return -ENOMEM;
		}
		ret = copy_from_user(temp, ubuf, count);
		ret = sscanf(temp, "%x %x %x %x", &kbuf[0], &kbuf[1], &kbuf[2], &kbuf[3]);
		if (!ret) {
			kfree(temp);
			return -EFAULT;
		}
		pr_info("[SmartPA-%d]:%s: kbuf[0]=0x%02x, kbuf[1]=0x%02x, kbuf[2]=0x%02x kbuf[3]=0x%02x cnt=%d\n",
			__LINE__, __func__, kbuf[0], kbuf[1], kbuf[2], kbuf[3], (int)count);
		/* regmap_write(tas2562->regmap, kbuf[0], kbuf[1]); */
		tas2562->write(tas2562, channel_left, 
			TAS2562_REG(kbuf[0], kbuf[1], kbuf[2]), kbuf[3]);
		kfree(temp);
	}else{
		pr_err("[SmartPA-%d]:%s count error.\n",__LINE__, __func__);
	}
	return count;
}

static ssize_t smartpa_reg_r_show(struct kobject *kobj, struct kobj_attribute *attr, char *buffer)
{
	struct tas2562_priv *tas2562 = tas2562_priv;
	u8 i;
	const int size = PAGE_SIZE;
	int data, n = 0;

	pr_info("[SmartPA-%d]%s: ======caught smartpa reg start ======\n", __LINE__, __func__);
	if (tas2562->mnChannels != 2)
		return 0;
	tas2562->client->addr = tas2562->mnRAddr;
	n += scnprintf(buffer+n, size-n, "i2c-addr: 0x%02x\n", tas2562->client->addr);
	for (i = 0; i < 128; i++) {
		tas2562->read(tas2562, channel_right, i, &data);
		n += scnprintf(buffer+n, size-n, "0x%02x:0x%02x\n", i, data);
	}
	for (i = 0; i < DBGFS_REG_COUNT; i++) {
		tas2562->read(tas2562, channel_right, dbgfs_reg[i], &data);
		n += scnprintf(buffer+n, size-n, "B0x%02xP0x%02xR0x%02x:0x%02x\n",
			TAS2562_BOOK_ID(dbgfs_reg[i]), TAS2562_PAGE_ID(dbgfs_reg[i]), TAS2562_PAGE_REG(dbgfs_reg[i]), data);
	}
	buffer[n] = 0;
	pr_info("[SmartPA-%d]%s: ======caught smartpa reg end ======\n", __LINE__, __func__);
	return n;
}

static ssize_t smartpa_reg_r_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *ubuf, size_t count)
{
	struct tas2562_priv *tas2562 = tas2562_priv;
	int ret = 0;
	unsigned int kbuf[4];
	char *temp;

	pr_info("[SmartPA-%d]%s: cnt %d\n", __LINE__, __func__, (int)count);
	if (tas2562->mnChannels != 2)
		return 0;
	tas2562->client->addr = tas2562->mnRAddr;
	if (count > 4) {
		temp = kmalloc(count, GFP_KERNEL);
		if (!temp) {
			return -ENOMEM;
		}
		ret = copy_from_user(temp, ubuf, count);
		ret = sscanf(temp, "%x %x %x %x", &kbuf[0], &kbuf[1], &kbuf[2], &kbuf[3]);
		if (!ret) {
			kfree(temp);
			return -EFAULT;
		}
		pr_info("[SmartPA-%d]:%s: kbuf[0]=0x%02x, kbuf[1]=0x%02x, kbuf[2]=0x%02x kbuf[3]=0x%02x cnt=%d\n",
			__LINE__, __func__, kbuf[0], kbuf[1], kbuf[2], kbuf[3], (int)count);

		tas2562->write(tas2562, channel_right, 
			TAS2562_REG(kbuf[0], kbuf[1], kbuf[2]), kbuf[3]);
		kfree(temp);
	}else{
		pr_err("[SmartPA-%d]:%s count error.\n",__LINE__, __func__);
	}
	return count;
}

static struct kobj_attribute dev_attr_calibrate =
	__ATTR(calibrate, 0664, smartpa_calibrate_show, NULL);
static struct kobj_attribute dev_attr_f0qt =
	__ATTR(f0qt, 0664, smartpa_f0qt_show, NULL);
static struct kobj_attribute dev_attr_i2c =
	__ATTR(i2c, 0664, smartpa_i2c_show, NULL);
static struct kobj_attribute dev_attr_reg_l =
	__ATTR(reg_l, 0664, smartpa_reg_l_show, smartpa_reg_l_store);
static struct kobj_attribute dev_attr_reg_r =
	__ATTR(reg_r, 0664, smartpa_reg_r_show, smartpa_reg_r_store);

static struct attribute *sys_node_attributes[] = {
	&dev_attr_calibrate.attr,
	&dev_attr_f0qt.attr,
	&dev_attr_i2c.attr,
	&dev_attr_reg_l.attr,
	&dev_attr_reg_r.attr,
	NULL
};

static struct attribute_group node_attribute_group = {
	.name = NULL,		/* put in device directory */
	.attrs = sys_node_attributes
};

static int class_attr_create(struct kobject *kobj, struct i2c_client *i2c)
{
	int ret = -1;
	char name[64];

	scnprintf(name, MAX_CONTROL_NAME, "audio-%s", i2c->name);

	kobj = kobject_create_and_add(name, kernel_kobj);
	if (!kobj) {
		pr_err("%s: kobject_create_and_add %s faild\n", __func__, name);
		return 0;
	}

	ret = sysfs_create_group(kobj, &node_attribute_group);
	if (ret) {
		kobject_del(kobj);
		kobj = NULL;
		pr_err("%s: sysfs_create_group %s faild\n", __func__, name);
	}

	pr_info("%s: sysfs create name successful\n", __func__, name);
	return ret;
}

static int class_attr_remove(struct kobject *kobj)
{
	if (kobj) {
		sysfs_remove_group(kobj, &node_attribute_group);
		kobject_del(kobj);
		kobj = NULL;
	}
	return 0;
}

static int smartpa_calib_save(uint32_t *calib_value, char * buffer1)
{
	int ret = 0;
	uint8_t channels	= 1;
	if (!tas2562_priv || !calib_value) {
	  pr_err("[SmartPA-%d]: SmartPA_priv or calib_value is NULL\n",
		  __LINE__);
		  ret = -1;
		  return ret;
	}
	channels = tas2562_priv->mnChannels;
	memcpy(buffer1,"new_tas:",strlen("new_tas:"));
	memcpy(buffer1+strlen("new_tas:"),calib_value,sizeof(uint32_t)*channels);
	smartpa_init_re_value(calib_value);
	pr_info("%s: end",__func__);
	return ret;

}

static void smartpa_set_re(uint32_t *calibRe)
{
	int nSize = sizeof(uint32_t);
	struct afe_smartamp_set_params_t prot_config;
	int ret;
	uint8_t iter = 0;
	struct vivo_calibration_rx vivo_calib_rx;
	memset(&prot_config, 0, sizeof(prot_config));

	if (!tas2562_priv || !calibRe){
		pr_err("[SmartPA-%d]: tas2562_priv or calibRe is NULL\n", __LINE__);
		return;
	}
	//if((calibRe != NO_CALIB) && (calibRe != CALIB_FAILED)){
	for(iter = 0; iter < tas2562_priv->mnChannels; iter++)
	{
		if (calibRe[iter] != 0) {
			if (tas2562_priv->vivo_smartpa_protection) {
				pr_info("[SmartPA-%d]: %s: vivo_smartpa_protection\n", __LINE__, __func__);
				nSize = sizeof(vivo_calib_rx);
				memset(&vivo_calib_rx, 0, nSize);
				vivo_calib_rx.re_q15 = calibRe[iter];

				vivo_calib_rx.pa_id = tas2562_priv->pa_id[iter];
				vivo_calib_rx.v_max_q15 = tas2562_priv->v_max_q15[iter];
				vivo_calib_rx.i_max_q15 = tas2562_priv->i_max_q15[iter];
				vivo_calib_rx.v_out_max_q15 = tas2562_priv->v_out_max_q15[iter];

				if (calibRe[iter] != CALIB_FAILED)
					vivo_calib_rx.calibration_success = 1;
				memcpy(prot_config.payload, &vivo_calib_rx, nSize);
				ret = afe_smartamp_set_calib_data(VIVO_SP_CALIBRATION+(iter<<8),
					&prot_config, nSize, CAPI_V2_VIVO_SP_RX);
			} else {
				memcpy(prot_config.payload, &calibRe[iter], nSize);
				pr_info("[SmartPA-%d]: smartamp : Payload : %d",__LINE__,prot_config.payload[0]);
				ret = afe_smartamp_set_calib_data(
					((iter+1) << 24) | (nSize << 16) | AFE_SA_SET_RE, &prot_config, nSize, AFE_SMARTAMP_MODULE);
			}
			pr_err("[SmartPA-%d]: set Re[%d]: %d", __LINE__, iter, calibRe[iter]);
		}
		else {
			pr_err("[SmartPA-%d]: Cannot set Re for calib status wrong", __LINE__);
			if (tas2562_priv->vivo_smartpa_protection) {
				pr_info("[SmartPA-%d]: %s: vivo_smartpa_protection\n", __LINE__, __func__);
				nSize = sizeof(vivo_calib_rx);
				memset(&vivo_calib_rx, 0, nSize);
				vivo_calib_rx.calibration_success = 1;
				vivo_calib_rx.pa_id = tas2562_priv->pa_id[iter];
				vivo_calib_rx.v_max_q15 = tas2562_priv->v_max_q15[iter];
				vivo_calib_rx.i_max_q15 = tas2562_priv->i_max_q15[iter];
				vivo_calib_rx.v_out_max_q15 = tas2562_priv->v_out_max_q15[iter];
				memcpy(prot_config.payload, &vivo_calib_rx, nSize);
				ret = afe_smartamp_set_calib_data(VIVO_SP_CALIBRATION+(iter<<8),
					&prot_config, nSize, CAPI_V2_VIVO_SP_RX);
			}
		}
	}
}

int smartpa_init_dbg(char *buffer, char *buffer1, int size)
{
	uint32_t calib_re[MAX_CHANNELS] = {0};
	uint32_t paramid = 0;
	int ret = 0, n = 0;
	uint32_t data = 0;
	bool done[MAX_CHANNELS] = {false};
	int nSize = sizeof(uint32_t);
	struct afe_smartamp_set_params_t prot_config;
	uint8_t iter = 0, channels = 1;
	struct vivo_calibration_rx vivo_calib_rx;
	memset(&prot_config, 0, sizeof(prot_config));

	pr_info("[SmartPA-%d]: enter %s\n", __LINE__, __func__);

	if (!tas2562_priv) {
		pr_err("[SmartPA-%d]: tas2562_priv is NULL\n", __LINE__);
		return -1;
	}
	channels = tas2562_priv->mnChannels;
	if(channels == 1)
		done[1] = true;
	
	if (tas2562_priv->mbPowerUp) {
		//calib init
		for(iter = 0; iter < channels; iter++)
		{
			if (tas2562_priv->vivo_smartpa_protection) {
				pr_info("[SmartPA-%d]: %s: vivo_smartpa_protection\n", __LINE__, __func__);
				data = 3; /* 3 means clibration mode */
				memcpy(prot_config.payload, &data, nSize);
				ret = afe_smartamp_set_calib_data(VIVO_SP_WORK_MODE_TX+(iter<<8),
					&prot_config, nSize, CAPI_V2_VIVO_SP_TX);
				ret |= afe_smartamp_set_calib_data(VIVO_SP_WORK_MODE_RX+(iter<<8),
					&prot_config, nSize, CAPI_V2_VIVO_SP_RX);
			} else {
				data = 1;//Value is ignored
				memcpy(prot_config.payload, &data, nSize);
				ret = afe_smartamp_set_calib_data(((AFE_SA_CALIB_INIT)|((iter+1)<<24)|(1<<16))
						, &prot_config, nSize,AFE_SMARTAMP_MODULE);
			}
			if (ret < 0) {
				done[iter] = false;
				pr_info("[SmartPA-%d]init_dbg:set calib_data error.\n", __LINE__);
				ret = -ENOMEM;
			}
		}
		pr_info("[SmartPA-%d]init_dbg: calib init\n", __LINE__);

		msleep(3 * 1000);

		//get Re
		for(iter = 0; iter < channels; iter++)
		{
			if (tas2562_priv->vivo_smartpa_protection) {
				memset(&vivo_calib_rx, 0, sizeof(vivo_calib_rx));
				ret = afe_smartamp_algo_ctrl((u8 *)&vivo_calib_rx, VIVO_SP_CALIBRATION+(iter<<8),
					TAS_GET_PARAM, sizeof(vivo_calib_rx), CAPI_V2_VIVO_SP_RX);
			} else {
				data = 0;//Reset data to 0
				paramid = ((AFE_SA_GET_RE)|((iter+1)<<24)|(1<<16));
				ret = afe_smartamp_algo_ctrl((u8*)&data, paramid,
					TAS_GET_PARAM, /*length */ 4, AFE_SMARTAMP_MODULE);
			}
			if (ret < 0) {
				done[iter] = false;
				pr_info("[SmartPA-%d]init_dbg: decalib init\n", __LINE__);
			} else {
				if (tas2562_priv->vivo_smartpa_protection) {
					calib_re[iter] = vivo_calib_rx.re_q15;
				} else
					calib_re[iter] = data;

				if ((calib_re[iter] < tas2562_priv->imped_min[iter]) || (calib_re[iter] > tas2562_priv->imped_max[iter]))
					done[iter] = false;
				else
					done[iter] = true;
				pr_info("[SmartPA-%d]init_dbg: calib_re is %d, valid range (%d %d)\n",
						__LINE__, calib_re[iter], tas2562_priv->imped_min[iter], tas2562_priv->imped_max[iter]);
			}
		}
	} else {
		done[0] = false;
		if(channels == 2)
			done[1] = false;
		ret = -EINVAL;
		pr_info("[SmartPA-%d]dbg init: failed to calibrate %d\n", __LINE__, ret);
	}

	n += scnprintf(buffer + n, size - n, "current status:[SmartPA] %s\n", (channels == 1) ? "Mono" : "Stereo"); 
	for(iter = 0; iter < channels; iter++)
	{
		if (tas2562_priv->vivo_smartpa_protection) {
			n += scnprintf(buffer + n, size - n,
					"Channel[%d]: impedance %02d.%02d ohm, valid range(%02d.%02d ~ %02d.%02d ohm). \n", iter,
					VIVO_TRANSF_IMPED_TO_USER_I(calib_re[iter]), VIVO_TRANSF_IMPED_TO_USER_M(calib_re[iter]),
					VIVO_TRANSF_IMPED_TO_USER_I(tas2562_priv->imped_min[iter]), VIVO_TRANSF_IMPED_TO_USER_M(tas2562_priv->imped_min[iter]),
					VIVO_TRANSF_IMPED_TO_USER_I(tas2562_priv->imped_max[iter]), VIVO_TRANSF_IMPED_TO_USER_M(tas2562_priv->imped_max[iter]));
		} else {
			n += scnprintf(buffer + n, size - n,
					"Channel[%d]: impedance %02d.%02d ohm, valid range(%02d.%02d ~ %02d.%02d ohm). \n", iter,
					TRANSF_IMPED_TO_USER_I(calib_re[iter]), TRANSF_IMPED_TO_USER_M(calib_re[iter]),
					TRANSF_IMPED_TO_USER_I(tas2562_priv->imped_min[iter]), TRANSF_IMPED_TO_USER_M(tas2562_priv->imped_min[iter]),
					TRANSF_IMPED_TO_USER_I(tas2562_priv->imped_max[iter]), TRANSF_IMPED_TO_USER_M(tas2562_priv->imped_max[iter]));
		}
		if (!done[iter]) {
			pr_info("[SmartPA-%d]init_dbg: calibrate failed:calibRe[%d] %d\n", __LINE__, iter, calib_re[iter]);
			calib_re[iter] = CALIB_FAILED;
		}
	}
	n += scnprintf(buffer + n, size - n, "\n Calibrate result: %s\n", (done[0] && done[1]) ? "OKAY(impedance ok)." : "ERROR!");
	buffer[n] = 0;

	pr_info("[SmartPA-%d]init_dbg: write to file\n", __LINE__);

	tas2562_priv->calibRe[0] = calib_re[0];
	tas2562_priv->calibRe[1] = calib_re[1];
	pr_info("[SmartPA-%d]init_dbg: update Re value\n", __LINE__);
	smartpa_calib_save(calib_re, buffer1);

//deinit:
	for(iter = 0; iter < channels; iter++)
	{
		if (!tas2562_priv->vivo_smartpa_protection) {
			data = 0;//Value is ignored
			memcpy(prot_config.payload, &data, nSize);
			ret = afe_smartamp_set_calib_data(((AFE_SA_CALIB_DEINIT)|((iter+1)<<24)|(1<<16)), &prot_config
				, nSize, AFE_SMARTAMP_MODULE);
			pr_info("[SmartPA-%d]init_dbg: decalib init\n", __LINE__);
		}
	}
//end:
	pr_info("[SmartPA-%d]init_dbg: end\n", __LINE__);

	return (done[0] && done[1]);
}

static int smartpa_freq_save(char *buffer, int count)
{
#if 0
	struct file *pfile = NULL;
	int ret = 0;
	loff_t pos = 0;

	pfile = filp_open(FREQ_FILE, O_RDWR | O_CREAT, 0666);
	if (!IS_ERR(pfile)) {
		pr_info("[SmartPA-%d]freq: save count=%d \n", __LINE__, count);
		kernel_write1(pfile, buffer, count, &pos);
		filp_close(pfile, NULL);
	} else {
		pr_info("[SmartPA-%d]freq: %s open failed! \n", __LINE__, FREQ_FILE);
		ret = -1;
	}

	return ret;
#endif
	return 0;
}

int smartpa_read_freq_dbg(char *buffer, int size)
{
	u32 length = TAS_PAYLOAD_SIZE;
	uint32_t calibRe[MAX_CHANNELS] = {0};
	uint32_t F0[MAX_CHANNELS][3] = {{0}, {0}}, Q[MAX_CHANNELS][3] = {{0}, {0}};
	int ret = 0, n = 0, i = 0, j[MAX_CHANNELS] = {0};
	uint32_t data = 0;
	uint32_t paramid = 0;
	int nSize = sizeof(uint32_t);
	struct afe_smartamp_set_params_t prot_config;
	uint8_t iter = 0, channels = 1;
	struct vivo_calibration_rx vivo_calib_rx;
	memset(&prot_config, 0, sizeof(prot_config));
	pr_info("[SmartPA-%d]: enter %s\n", __LINE__, __func__);

	if (!tas2562_priv) {
		pr_err("[SmartPA-%d]: tas2562_priv is NULL\n", __LINE__);
		return -1;
	}
	channels = tas2562_priv->mnChannels;
	
	//Load Calib
	if(smartpa_check_re()) {
		
		for(iter = 0; iter < channels; iter++)
		{
			calibRe[iter] = tas2562_priv->calibRe[iter];
		}
		smartpa_set_re(calibRe);
	}
	
	for(iter = 0; iter < channels; iter++)
	{
		if (tas2562_priv->vivo_smartpa_protection) {
			pr_info("[SmartPA-%d]: %s: vivo_smartpa_protection\n", __LINE__, __func__);
			data = 3; /* 3 means clibration mode */
			memcpy(prot_config.payload, &data, nSize);
			ret = afe_smartamp_set_calib_data(VIVO_SP_WORK_MODE_TX+(iter<<8),
				&prot_config, nSize, CAPI_V2_VIVO_SP_TX);
			ret |= afe_smartamp_set_calib_data(VIVO_SP_WORK_MODE_RX+(iter<<8),
				&prot_config, nSize, CAPI_V2_VIVO_SP_RX);
		} else {
			data = 1;//Value is ignored
			memcpy(prot_config.payload, &data, nSize);
			paramid = ((AFE_SA_F0_TEST_INIT) | (length << 16) | ((iter+1) << 24));
			ret = afe_smartamp_set_calib_data(paramid
				, &prot_config, nSize, AFE_SMARTAMP_MODULE);
		}
	}
	//wait 5s
	msleep(5000);

	for (i = 0; i < 3; i++) {
		msleep(500);
		for (iter = 0; iter < channels; iter++) {
			if (tas2562_priv->vivo_smartpa_protection) {
				memset(&vivo_calib_rx, 0, sizeof(vivo_calib_rx));
				ret = afe_smartamp_algo_ctrl((u8 *)&vivo_calib_rx, VIVO_SP_CALIBRATION+(iter<<8),
					TAS_GET_PARAM, sizeof(vivo_calib_rx), CAPI_V2_VIVO_SP_RX);
				F0[iter][i] = vivo_calib_rx.f0_q15;
				Q[iter][i] = vivo_calib_rx.qts_q15;
				if ((VIVO_TRANSF_IMPED_TO_USER_I(F0[iter][i]) < tas2562_priv->fres_min[iter]) ||
					(VIVO_TRANSF_IMPED_TO_USER_I(F0[iter][i]) > tas2562_priv->fres_max[iter]) ||
					(VIVO_TRANSF_IMPED_TO_USER_I(Q[iter][i] * 100) < tas2562_priv->Qt[iter]))
					j[iter] = 0;
				else
					j[iter]++;
			} else {
				//read F0
				data = 0;//Reset data to 0
				paramid = (AFE_SA_GET_F0 | (length << 16) | ((iter+1) << 24));
				ret = afe_smartamp_algo_ctrl((u8*)&data, paramid,
						TAS_GET_PARAM, /*length **/ 4, AFE_SMARTAMP_MODULE);
				F0[iter][i] = data;
				//read Q
				data = 0;//Reset data to 0
				paramid = (AFE_SA_GET_Q | (length << 16) | ((iter+1) << 24));
				ret = afe_smartamp_algo_ctrl((u8*)&data, paramid,
						TAS_GET_PARAM, /*length **/ 4, AFE_SMARTAMP_MODULE);
				Q[iter][i] = data;
				if (((F0[iter][i] >> 19) < tas2562_priv->fres_min[iter]) ||
					((F0[iter][i] >> 19) > tas2562_priv->fres_max[iter]) ||
					(((Q[iter][i] * 100) >> 19) < tas2562_priv->Qt[iter]))
					j[iter] = 0;
				else
					j[iter]++;
			}
			pr_info("[SmartPA-%d]read freq dbg channel[%d]: f0 = %d Qt = %d i = %d j = %d\n",
				__LINE__, iter, F0[iter][i], Q[iter][i], i, j[iter]);
		}
	}

	for (iter = 0; iter < channels; iter++) {
		n += scnprintf(buffer+n, size-n, "Channel[%d]\n", iter);
		if (tas2562_priv->vivo_smartpa_protection) {
			n += scnprintf(buffer+n, size-n, "impedance = %02d.%02d\n",
				VIVO_TRANSF_IMPED_TO_USER_I(calibRe[iter]), VIVO_TRANSF_IMPED_TO_USER_M(calibRe[iter]));
		} else {
			n += scnprintf(buffer+n, size-n, "impedance = %02d.%02d\n",
				TRANSF_IMPED_TO_USER_I(calibRe[iter]), TRANSF_IMPED_TO_USER_M(calibRe[iter]));
		}
		for (i = 0; i < 3; i++) {
			if (tas2562_priv->vivo_smartpa_protection) {
				n += scnprintf(buffer+n, size-n, "f0 = %d Qt = %01d.%02d\n",
					VIVO_TRANSF_IMPED_TO_USER_I(F0[iter][i]),
					VIVO_TRANSF_IMPED_TO_USER_I(Q[iter][i]),
					VIVO_TRANSF_IMPED_TO_USER_M(Q[iter][i]));
			} else {
				n += scnprintf(buffer+n, size-n, "f0 = %d Qt = %01d.%02d\n",
					(F0[iter][i] >> 19),
					TRANSF_IMPED_TO_USER_I(Q[iter][i]),
					TRANSF_IMPED_TO_USER_M(Q[iter][i]));
			}
		}
		n += scnprintf(buffer+n, size-n, "f0 (%d ~ %d)\nQt_Min: %01d.%02d \n",
			tas2562_priv->fres_min[iter], tas2562_priv->fres_max[iter],
			tas2562_priv->Qt[iter] / 100, tas2562_priv->Qt[iter] % 100);
		if (j[iter] == 3)
			n += scnprintf(buffer+n, size-n, "PASS\n");
		else
			n += scnprintf(buffer+n, size-n, "FAIL\n");
	}

	ret = smartpa_freq_save(buffer, n);
	buffer[n] = 0;

//deinit :
	for(iter = 0; iter < channels; iter++)
	{
		if (!tas2562_priv->vivo_smartpa_protection) {
			data = 0;//Value is ignored
			memcpy(prot_config.payload, &data, nSize);
			paramid = ((AFE_SA_F0_TEST_DEINIT) | (length << 16) | ((iter+1) << 24));
			ret = afe_smartamp_set_calib_data(paramid
				, &prot_config, nSize, AFE_SMARTAMP_MODULE);
		}
	}
//end:
	return 0;
}

void smartpa_read_prars_dbg(int temp[5], unsigned char addr)
{
	pr_info("[SmartPA-%d]: %s enter.\n",__LINE__, __func__);

	return ;
}

void smartpa_get_client(struct i2c_client **client, unsigned char addr)

{
	pr_info("[SmartPA-%d]: %s enter.\n",__LINE__, __func__);

	return ;
}

int smartpa_check_calib_dbg(void)
{
	uint32_t impedance[MAX_CHANNELS] = {0};
	uint8_t iter = 0, channels = 0;
	int ret = 0;

	if (!tas2562_priv)
		return 0;

	pr_info("[SmartPA-%d]: %s enter.\n", __LINE__, __func__);

	smartpa_calib_get(impedance);
	channels = tas2562_priv->mnChannels;
	for (iter = 0; iter < channels; iter++) {
		ret |= rdc_check_valid(impedance[iter], iter) << iter;
	}
	return ret; /* spk0 bit0, spk1 bit1 */
}

static bool rdc_check_valid(uint32_t rdc, uint8_t iter)
{
	if (!tas2562_priv) {
		pr_err("[SmartPA-%d]: tas2562_priv is NULL\n", __LINE__);
		return false;
	}

	if (rdc > tas2562_priv->imped_min[iter] && rdc < tas2562_priv->imped_max[iter]) {
		return true;
	}

	pr_info("[SmartPA-%d]rdc check: rdc: %d invalid, [%d, %d] \n",
		__LINE__, rdc, tas2562_priv->imped_min[iter],
		tas2562_priv->imped_max[iter]);

	return false;
}

static int smartpa_calib_get(uint32_t* calib_value)
{
//	struct file *pfile = NULL;
	int found = 0;
//	loff_t pos = 0;
	int channels = 1;
	
	if (!tas2562_priv || !calib_value) {
		pr_err("[SmartPA-%d]: tas2562_priv or calib_value is NULL\n", __LINE__);
		return false;
	}
	channels =  tas2562_priv->mnChannels;
	
	*calib_value = 0;
#if 0
	pfile = filp_open(CALIBRATE_FILE, O_RDONLY, 0);
	if (!IS_ERR_OR_NULL(pfile)) {
		found = 1;
		kernel_read1(pfile, (char *)calib_value, sizeof(uint32_t)*channels, &pos);
		pr_info("[SmartPA-%d]calibrate:get calib_value[0] %d  \n", __LINE__, calib_value[0]);
		if(channels == 2)
			pr_info("[SmartPA-%d]calibrate:get calib_value[1] %d  \n", __LINE__, calib_value[1]);
		filp_close(pfile, NULL);
	} else {
		pr_info("[SmartPA-%d]calibrate: No found\n", __LINE__);
		found = 0;
	}
#else
//smartpa_init_re_value(calib_value);
	memcpy(calib_value, re_calib, sizeof(re_calib));
	if (calib_value != NULL) {
		pr_info("[SmartPA-%d]calibrate:get calib_value[0] %x\n",
			__LINE__, calib_value[0]);
		found = 1;
		if(channels == 2)
			pr_info("[SmartPA-%d]calibrate:get calib_value[1] %x\n",
			__LINE__, calib_value[1]);
	} else {
		pr_info("[SmartPA-%d]calib_value is NULL\n",__LINE__);
	}
#endif
	return found;
}

static bool smartpa_check_re(void)
{
	int rc = 0;
	uint32_t impedance[MAX_CHANNELS] = {0};
	uint8_t iter = 0, channels = 0;
	bool re_ok = true;
	
	pr_info("[SmartPA-%d] smartpa_check_re enter.\n",__LINE__);
	if (!tas2562_priv) {
		pr_err("[SmartPA-%d]: tas2562_priv is NULL\n", __LINE__);
		return false;
	}

	channels = tas2562_priv->mnChannels;
	for (iter = 0; iter < channels; iter ++) {
		if (rdc_check_valid(tas2562_priv->calibRe[iter], iter)|| (tas2562_priv->calibRe[iter] == 0xCACACACA)) {
			pr_info("[SmartPA-%d] smartpa_check_re[%x]:%d ok.\n", __LINE__, iter, tas2562_priv->calibRe[iter]);
			rc += 1;
		}
	}
	if (rc == channels)
		re_ok = true;
	else {
		smartpa_calib_get(impedance);
		rc = 0;
		for (iter = 0; iter < channels; iter++) {
			tas2562_priv->calibRe[iter] = impedance[iter];
			if (rdc_check_valid(tas2562_priv->calibRe[iter], iter)) {
				pr_info("[SmartPA-%d] smartpa_check_re[%d]:%d success.\n", __LINE__, iter, tas2562_priv->calibRe[iter]);
				rc += 1;
			} else {
				if (tas2562_priv->calibRe[iter] == 0xCACACACA)
					rc += 1;
				pr_info("[SmartPA-%d] smartpa_check_re[%d]:%d failed.\n", __LINE__, iter, tas2562_priv->calibRe[iter]);
#if defined(VIVO_PROJECT_MODEL)
				/*GDPR code*/
#else
				/* not GDPR code start */
#ifdef CONFIG_RSC_VAUDIT_DEBUG
				rsc_vaudit(VAUDIT_AUDIO, AUDIO_SMART_PA, 0,
					"TAS2562 calibrate error: %u, R: [%d, %d].",
					tas2562_priv->calibRe[iter], tas2562_priv->imped_min[iter],
					tas2562_priv->imped_max[iter]);
#endif
				/* not GDPR code end */
#endif
			}
		}
		if (rc == channels)
			re_ok = true;
		else
			re_ok = false;
	}

	return re_ok;
}

static int smartpa_parse_dt(struct i2c_client *i2c)
{
	int temp, ret = 0;

	if (!tas2562_priv){
		pr_err("[SmartPA-%d]: tas2562_priv is NULL\n", __LINE__);
		return -1;
	}

	if (of_find_property(i2c->dev.of_node, "vivo-smartpa-protection", NULL))
		tas2562_priv->vivo_smartpa_protection = true;

	ret = of_property_read_u32(i2c->dev.of_node, "vivo,impedance-min", &temp);
	tas2562_priv->imped_min[0] = (!ret)? (int)temp : RDC_MIN_L;
	
	ret = of_property_read_u32(i2c->dev.of_node, "vivo,impedance-max", &temp);
	tas2562_priv->imped_max[0] = (!ret)? (int)temp : RDC_MAX_L;
	
	ret = of_property_read_u32(i2c->dev.of_node, "vivo,frequency-min", &temp);
	tas2562_priv->fres_min[0] = (!ret)? (int)temp : 500;

	ret = of_property_read_u32(i2c->dev.of_node, "vivo,frequency-max", &temp);
	tas2562_priv->fres_max[0] = (!ret)? (int)temp : 1100;
	
	ret = of_property_read_u32(i2c->dev.of_node, "vivo,Qt-min", &temp);
	tas2562_priv->Qt[0] = (!ret)? (int)temp : 100;

	ret = of_property_read_u32(i2c->dev.of_node, "vivo,pa_id", &temp);
	tas2562_priv->pa_id[0] = (!ret)? (uint32_t)temp : 62;

	ret = of_property_read_u32(i2c->dev.of_node, "vivo,v_max_q15", &temp);
	tas2562_priv->v_max_q15[0] = (!ret)? (uint32_t)temp : 458752;

	ret = of_property_read_u32(i2c->dev.of_node, "vivo,i_max_q15", &temp);
	tas2562_priv->i_max_q15[0] = (!ret)? (uint32_t)temp : 65536;

	ret = of_property_read_u32(i2c->dev.of_node, "vivo,v_out_max_q15", &temp);
	tas2562_priv->v_out_max_q15[0] = (!ret)? (uint32_t)temp : 275907;

	if(tas2562_priv->mnChannels == 2)
	{
		ret = of_property_read_u32(i2c->dev.of_node, "vivo,impedance2-min", &temp);
		tas2562_priv->imped_min[1] = (!ret)? (int)temp : RDC_MIN_R;
	
		ret = of_property_read_u32(i2c->dev.of_node, "vivo,impedance2-max", &temp);
		tas2562_priv->imped_max[1] = (!ret)? (int)temp : RDC_MAX_R;
	
		ret = of_property_read_u32(i2c->dev.of_node, "vivo,frequency2-min", &temp);
		tas2562_priv->fres_min[1] = (!ret)? (int)temp : 500;

		ret = of_property_read_u32(i2c->dev.of_node, "vivo,frequency2-max", &temp);
		tas2562_priv->fres_max[1] = (!ret)? (int)temp : 1100;
	
		ret = of_property_read_u32(i2c->dev.of_node, "vivo,Qt2-min", &temp);
		tas2562_priv->Qt[1] = (!ret)? (int)temp : 100;
	}
	
	return 0;
}
#endif

static enum hrtimer_restart timer_func(struct hrtimer *timer)
{
	struct tas2562_priv *pTAS2562 = container_of(timer,
		struct tas2562_priv, mtimer);

	if (pTAS2562->mbPowerUp) {
		if (!delayed_work_pending(&pTAS2562->irq_work))
			schedule_delayed_work(&pTAS2562->irq_work,
				msecs_to_jiffies(20));
	}

	return HRTIMER_NORESTART;
}

static irqreturn_t tas2562_irq_handler(int irq, void *dev_id)
{
	struct tas2562_priv *pTAS2562 = (struct tas2562_priv *)dev_id;

	//tas2562_enableIRQ(pTAS2562, false);
	/* get IRQ status after 100 ms */
	schedule_delayed_work(&pTAS2562->irq_work, msecs_to_jiffies(100));
	return IRQ_HANDLED;
}

static int tas2562_runtime_suspend(struct tas2562_priv *pTAS2562)
{
	dev_dbg(pTAS2562->dev, "%s\n", __func__);

	pTAS2562->mbRuntimeSuspend = true;

	if (hrtimer_active(&pTAS2562->mtimer)) {
		dev_dbg(pTAS2562->dev, "cancel die temp timer\n");
		hrtimer_cancel(&pTAS2562->mtimer);
	}

	if (delayed_work_pending(&pTAS2562->irq_work)) {
		dev_dbg(pTAS2562->dev, "cancel IRQ work\n");
		cancel_delayed_work_sync(&pTAS2562->irq_work);
	}

	return 0;
}

static int tas2562_runtime_resume(struct tas2562_priv *pTAS2562)
{
	dev_dbg(pTAS2562->dev, "%s\n", __func__);

	if (pTAS2562->mbPowerUp) {
/*		if (!hrtimer_active(&pTAS2562->mtimer)) {
		dev_dbg(pTAS2562->dev, "%s, start check timer\n", __func__);
			hrtimer_start(&pTAS2562->mtimer,
				ns_to_ktime((u64)CHECK_PERIOD * NSEC_PER_MSEC),
				HRTIMER_MODE_REL);
		}
*/
	}

	pTAS2562->mbRuntimeSuspend = false;

	return 0;
}

static void tas2562_reg_table(const struct device_node *np,
			struct tas2562_priv *pTAS2562)
{
	const char *reg_table = "vivo,reg_table";
	int reg_size = 0, *array = NULL;

	pTAS2562->reg_size = 0;
	pTAS2562->reg_table = NULL;

	if (!of_find_property(np, reg_table, &reg_size)) {
		dev_err(pTAS2562->dev, "%s: Missing %s in dt node.\n",
			__func__, reg_table);
		return;
	}

	reg_size /= sizeof(int);
	if (reg_size % 5) {
		dev_err(pTAS2562->dev, "%s: Abnormal number of parameters.\n",
			__func__);
		return;
	}

	array = kzalloc(sizeof(int) * reg_size, GFP_KERNEL);
	if (!array) {
		dev_err(pTAS2562->dev, "%s: Out of memory.\n",
			__func__);
		return;
	}

	if (of_property_read_u32_array(np, reg_table, array, reg_size)) {
		kfree(array);
		array = NULL;
		dev_err(pTAS2562->dev, "%s: Read property %s node failed.\n",
				__func__, reg_table);
		return;
	}

	pTAS2562->reg_size = reg_size / 5;
	pTAS2562->reg_table = array;
	dev_info(pTAS2562->dev, "%s: special params, reg_size: %d.\n",
		__func__, pTAS2562->reg_size);

	return;
}

static int tas2562_parse_dt(struct device *dev, struct tas2562_priv *pTAS2562)
{
	struct device_node *np = dev->of_node;
	int rc = 0, ret = 0;

	rc = of_property_read_u32(np, "ti,asi-format", &pTAS2562->mnASIFormat);
	if (rc) {
		dev_err(pTAS2562->dev, "Looking up %s property in node %s failed %d\n",
			"ti,asi-format", np->full_name, rc);
	} else {
		dev_dbg(pTAS2562->dev, "ti,asi-format=%d",
			pTAS2562->mnASIFormat);
	}

	rc = of_property_read_u32(np, "ti,channels", &pTAS2562->mnChannels);
	if (rc) {
		dev_err(pTAS2562->dev, "Looking up %s property in node %s failed %d\n",
			"ti,channels", np->full_name, rc);
	} else {
		dev_dbg(pTAS2562->dev, "ti,channels=%d",
			pTAS2562->mnChannels);
	}

	rc = of_property_read_u32(np, "ti,left-channel", &pTAS2562->mnLAddr);
	if (rc) {
		dev_err(pTAS2562->dev, "Looking up %s property in node %s failed %d\n",
			"ti,left-channel", np->full_name, rc);
	} else {
		dev_dbg(pTAS2562->dev, "ti,left-channel=0x%x",
			pTAS2562->mnLAddr);
	}

	rc = of_property_read_u32(np, "ti,right-channel", &pTAS2562->mnRAddr);
	if (rc) {
		dev_err(pTAS2562->dev, "Looking up %s property in node %s failed %d\n",
			"ti,right-channel", np->full_name, rc);
	} else {
		dev_dbg(pTAS2562->dev, "ti,right-channel=0x%x",
			pTAS2562->mnRAddr);
	}

	pTAS2562->mnResetGPIO = of_get_named_gpio(np, "ti,reset-gpio", 0);
	if (!gpio_is_valid(pTAS2562->mnResetGPIO)) {
		dev_err(pTAS2562->dev, "Looking up %s property in node %s failed %d\n",
			"ti,reset-gpio", np->full_name, pTAS2562->mnResetGPIO);
	} else {
		dev_dbg(pTAS2562->dev, "ti,reset-gpio=%d",
			pTAS2562->mnResetGPIO);
	}

	pTAS2562->mnIRQGPIO = of_get_named_gpio(np, "ti,irq-gpio", 0);
	if (!gpio_is_valid(pTAS2562->mnIRQGPIO)) {
		dev_err(pTAS2562->dev, "Looking up %s property in node %s failed %d\n",
			"ti,irq-gpio", np->full_name, pTAS2562->mnIRQGPIO);
	} else {
		dev_dbg(pTAS2562->dev, "ti,irq-gpio=%d", pTAS2562->mnIRQGPIO);
	}

	if (pTAS2562->mnChannels == 2) {
		pTAS2562->mnResetGPIO2 = of_get_named_gpio(np, "ti,reset-gpio2", 0);
		if (!gpio_is_valid(pTAS2562->mnResetGPIO2)) {
			dev_err(pTAS2562->dev, "Looking up %s property in node %s failed %d\n",
				"ti,reset-gpio2", np->full_name, pTAS2562->mnResetGPIO2);
		} else {
			dev_dbg(pTAS2562->dev, "ti,reset-gpio2=%d",
				pTAS2562->mnResetGPIO2);
		}

		pTAS2562->mnIRQGPIO2 = of_get_named_gpio(np, "ti,irq-gpio2", 0);
		if (!gpio_is_valid(pTAS2562->mnIRQGPIO2)) {
			dev_err(pTAS2562->dev, "Looking up %s property in node %s failed %d\n",
				"ti,irq-gpio2", np->full_name, pTAS2562->mnIRQGPIO2);
		} else {
			dev_dbg(pTAS2562->dev, "ti,irq-gpio2=%d", pTAS2562->mnIRQGPIO2);
		}
	}

	tas2562_reg_table(np, pTAS2562);

	return ret;
}

static int tas2562_i2c_probe(struct i2c_client *pClient,
			const struct i2c_device_id *id)
{
	struct tas2562_priv *pTAS2562;
	int nResult;

	dev_info(&pClient->dev, "%s enter... Driver ID: %s.\n",
		__func__, TAS2562_DRIVER_ID);

	pTAS2562 = devm_kzalloc(&pClient->dev,
		sizeof(struct tas2562_priv), GFP_KERNEL);
	if (pTAS2562 == NULL) {
		dev_err(&pClient->dev, "failed to get i2c device\n");
		nResult = -ENOMEM;
		return nResult;
	}

	pTAS2562->client = pClient;
	pTAS2562->dev = &pClient->dev;
	i2c_set_clientdata(pClient, pTAS2562);
	dev_set_drvdata(&pClient->dev, pTAS2562);
	dev_set_name(&pClient->dev, "%s", "tas2562-codec");

	pTAS2562->regmap = devm_regmap_init_i2c(pClient, &tas2562_i2c_regmap);
	if (IS_ERR(pTAS2562->regmap)) {
		nResult = PTR_ERR(pTAS2562->regmap);
		dev_err(&pClient->dev, "Failed to allocate register map: %d\n",
					nResult);
		goto err;
	}

	if (pClient->dev.of_node)
		tas2562_parse_dt(&pClient->dev, pTAS2562);

	if (gpio_is_valid(pTAS2562->mnResetGPIO)) {
		nResult = gpio_request(pTAS2562->mnResetGPIO, "TAS2562_RESET");
		if (nResult) {
			dev_err(pTAS2562->dev, "%s: Failed to request gpio %d\n",
				__func__, pTAS2562->mnResetGPIO);
			nResult = -EINVAL;
			goto err;
		}
		tas2562_hw_reset(pTAS2562);
	}

	if (pTAS2562->mnChannels == 2 && gpio_is_valid(pTAS2562->mnResetGPIO2)) {
		nResult = gpio_request(pTAS2562->mnResetGPIO2, "TAS2562_RESET2");
		if (nResult) {
			dev_err(pTAS2562->dev, "%s: Failed to request gpio %d\n",
				__func__, pTAS2562->mnResetGPIO2);
			nResult = -EINVAL;
			goto err;
		}
		tas2562_hw_reset(pTAS2562);
	}

	pTAS2562->read = tas2562_dev_read;
	pTAS2562->write = tas2562_dev_write;
	pTAS2562->bulk_read = tas2562_dev_bulk_read;
	pTAS2562->bulk_write = tas2562_dev_bulk_write;
	pTAS2562->update_bits = tas2562_dev_update_bits;
	pTAS2562->hw_reset = tas2562_hw_reset;
	pTAS2562->enableIRQ = tas2562_enableIRQ;
	pTAS2562->runtime_suspend = tas2562_runtime_suspend;
	pTAS2562->runtime_resume = tas2562_runtime_resume;
	pTAS2562->mnPowerState = TAS2562_POWER_SHUTDOWN;
	pTAS2562->spk_l_control = 1;
#ifdef VIVO_PORT_SMARTPA
	memset(pTAS2562->calibRe, 0, sizeof(uint32_t)*pTAS2562->mnChannels);
	pTAS2562->set_re = smartpa_set_re;
	pTAS2562->check_re = smartpa_check_re;
#endif

	mutex_init(&pTAS2562->dev_lock);

	dev_info(&pClient->dev, "Before SW reset\n");
	/* Reset the chip */
	nResult = tas2562_dev_write(pTAS2562, channel_left, TAS2562_SoftwareReset, 0x01);
	if (nResult < 0)
		dev_err(&pClient->dev, "channel_left I2c fail, %d", nResult);
	else
		pTAS2562->smartpa_i2c_check |= 1;
	if (pTAS2562->mnChannels == 2) {
		nResult = tas2562_dev_write(pTAS2562, channel_right, TAS2562_SoftwareReset, 0x01);
		if (nResult < 0)
			dev_err(&pClient->dev, "channel_right I2c fail, %d", nResult);
		else
			pTAS2562->smartpa_i2c_check |= 2;
	}
	if (((pTAS2562->mnChannels == 1) && (pTAS2562->smartpa_i2c_check != 1)) ||
		((pTAS2562->mnChannels == 2) && (pTAS2562->smartpa_i2c_check != 3))) {
		dev_err(&pClient->dev, "I2c fail, smartpa_i2c_check %d\n",
			pTAS2562->smartpa_i2c_check);
		goto err;
	}
	dev_info(&pClient->dev, "After SW reset\n");

	if (gpio_is_valid(pTAS2562->mnIRQGPIO)) {
		nResult = gpio_request(pTAS2562->mnIRQGPIO, "TAS2562-IRQ");
		if (nResult < 0) {
			dev_err(pTAS2562->dev, "%s: GPIO %d request error\n",
				__func__, pTAS2562->mnIRQGPIO);
			goto err;
		}
		gpio_direction_input(pTAS2562->mnIRQGPIO);
		tas2562_dev_write(pTAS2562, channel_both, TAS2562_MiscConfigurationReg0, 0xce);

		pTAS2562->mnIRQ = gpio_to_irq(pTAS2562->mnIRQGPIO);
		dev_info(pTAS2562->dev, "irq = %d\n", pTAS2562->mnIRQ);
		INIT_DELAYED_WORK(&pTAS2562->irq_work, irq_work_routine);
		nResult = request_threaded_irq(pTAS2562->mnIRQ, tas2562_irq_handler,
				NULL, IRQF_TRIGGER_FALLING|IRQF_ONESHOT,
				pClient->name, pTAS2562);
		if (nResult < 0) {
			dev_err(pTAS2562->dev,
				"request_irq failed, %d\n", nResult);
			goto err;
		}
		tas2562_enableIRQ(pTAS2562, true);
	}
	if (gpio_is_valid(pTAS2562->mnIRQGPIO2) && (pTAS2562->mnChannels == 2)) {
		nResult = gpio_request(pTAS2562->mnIRQGPIO2, "TAS2562-IRQ2");
		if (nResult < 0) {
			dev_err(pTAS2562->dev, "%s: GPIO %d request error\n",
				__func__, pTAS2562->mnIRQGPIO2);
			goto err;
		}
		gpio_direction_input(pTAS2562->mnIRQGPIO2);
		tas2562_dev_write(pTAS2562, channel_both, TAS2562_MiscConfigurationReg0, 0xce);

		pTAS2562->mnIRQ2 = gpio_to_irq(pTAS2562->mnIRQGPIO2);
		dev_info(pTAS2562->dev, "irq = %d\n", pTAS2562->mnIRQ2);
		INIT_DELAYED_WORK(&pTAS2562->irq_work, irq_work_routine);
		nResult = request_threaded_irq(pTAS2562->mnIRQ2, tas2562_irq_handler,
				NULL, IRQF_TRIGGER_FALLING|IRQF_ONESHOT,
				pClient->name, pTAS2562);
		if (nResult < 0) {
			dev_err(pTAS2562->dev,
				"request_irq failed, %d\n", nResult);
			goto err;
		}
		tas2562_enableIRQ(pTAS2562, true);
	}

#ifdef CONFIG_TAS2562_CODEC
	mutex_init(&pTAS2562->codec_lock);
	nResult = tas2562_register_codec(pTAS2562);
	if (nResult < 0) {
		dev_err(pTAS2562->dev,
			"register codec failed, %d\n", nResult);
		goto err;
	}
#endif

#ifdef CONFIG_TAS2562_MISC
	mutex_init(&pTAS2562->file_lock);
	nResult = tas2562_register_misc(pTAS2562);
	if (nResult < 0) {
		dev_err(pTAS2562->dev,
			"register codec failed, %d\n", nResult);
		goto err;
	}
#endif

	hrtimer_init(&pTAS2562->mtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pTAS2562->mtimer.function = timer_func;

#ifdef VIVO_PORT_SMARTPA
#ifdef CONFIG_TAS2562_CALIB
	tas_calib_init();
#endif
	tas2562_priv = pTAS2562;
	nResult = smartpa_debug_probe(pClient);
	if (nResult != 0) {
		pr_err("[SmartPA-%d]Failed to probe debug interface: %d\n",__LINE__,nResult);
	}
	nResult = smartpa_parse_dt(pClient);
	class_attr_create(pTAS2562->kobj, pClient);
#endif

err:

	return nResult;
}

static int tas2562_i2c_remove(struct i2c_client *pClient)
{
	struct tas2562_priv *pTAS2562 = i2c_get_clientdata(pClient);

	dev_info(pTAS2562->dev, "%s\n", __func__);

#ifdef VIVO_PORT_SMARTPA
	class_attr_remove(pTAS2562->kobj);
#ifdef CONFIG_TAS2562_CALIB
	tas_calib_exit();
#endif
#endif

#ifdef CONFIG_TAS2562_CODEC
	tas2562_deregister_codec(pTAS2562);
	mutex_destroy(&pTAS2562->codec_lock);
#endif

#ifdef CONFIG_TAS2562_MISC
	tas2562_deregister_misc(pTAS2562);
	mutex_destroy(&pTAS2562->file_lock);
#endif

	if (gpio_is_valid(pTAS2562->mnResetGPIO))
		gpio_free(pTAS2562->mnResetGPIO);
	if (gpio_is_valid(pTAS2562->mnIRQGPIO))
		gpio_free(pTAS2562->mnIRQGPIO);
	if (pTAS2562->mnChannels == 2 && gpio_is_valid(pTAS2562->mnResetGPIO2))
		gpio_free(pTAS2562->mnResetGPIO2);
	if (pTAS2562->mnChannels == 2 && gpio_is_valid(pTAS2562->mnIRQGPIO2))
		gpio_free(pTAS2562->mnIRQGPIO2);

	return 0;
}


static const struct i2c_device_id tas2562_i2c_id[] = {
	{ "tas2562", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, tas2562_i2c_id);

#if defined(CONFIG_OF)
static const struct of_device_id tas2562_of_match[] = {
	{ .compatible = "ti,tas2562" },
	{},
};
MODULE_DEVICE_TABLE(of, tas2562_of_match);
#endif


static struct i2c_driver tas2562_i2c_driver = {
	.driver = {
		.name   = "tas2562",
		.owner  = THIS_MODULE,
#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(tas2562_of_match),
#endif
	},
	.probe      = tas2562_i2c_probe,
	.remove     = tas2562_i2c_remove,
	.id_table   = tas2562_i2c_id,
};

module_i2c_driver(tas2562_i2c_driver);

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("TAS2562 I2C Smart Amplifier driver");
MODULE_LICENSE("GPL v2");
#endif
