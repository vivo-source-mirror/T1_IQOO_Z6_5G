/*
** =============================================================================
** Copyright (c) 2016  Texas Instruments Inc.
**
** This program is free software; you can redistribute it and/or modify it under
** the terms of the GNU General Public License as published by the Free Software
** Foundation; version 2.
**
** This program is distributed in the hope that it will be useful, but WITHOUT
** ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
** FOR A PARTICULAR PURPOSE.See the GNU General Public License for more details.
**
** File:
**     tas2562-codec.c
**
** Description:
**     ALSA SoC driver for Texas Instruments TAS2562 High Performance 4W Smart
**     Amplifier
**
** =============================================================================
*/

#ifdef CONFIG_TAS2562_CODEC
#define DEBUG 5
#include <linux/module.h>
#include <linux/moduleparam.h>
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
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <dsp/smart_amp.h>
#include <dsp/q6afe-v2.h>
#include "tas2562.h"

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

#define TAS2562_MDELAY 0xFFFFFFFE
#define TAS2562_MSLEEP 0xFFFFFFFD
#define TAS2562_IVSENSER_ENABLE  1
#define TAS2562_IVSENSER_DISABLE 0

static char const *iv_enable_text[] = {"Off", "On"};
static int tas2562iv_enable;
static int mbMute;
static int ampoutput_l_val = 15; /* 15.5db: 8+0.5*val */
static int ampoutput_r_val = 15;
static int boostvoltage_l_val = 8; /* 10v: 6+0.5*val */
static int boostvoltage_r_val = 8;

static const struct soc_enum tas2562_enum[] = {
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(iv_enable_text), iv_enable_text),
};
static int tas2562_set_fmt(struct tas2562_priv *pTAS2562, unsigned int fmt);

static int tas2562_i2c_load_data(struct tas2562_priv *pTAS2562,
			enum channel chn, unsigned int *pData);
static int tas2562_mute_ctrl_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue);
static int tas2562_mute_ctrl_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue);

static unsigned int p_tas2562_classH_D_data[] = {
		/* reg address			size	values */
	TAS2562_ClassHHeadroom, 0x4, 0x09, 0x99, 0x99, 0x9a,
	TAS2562_ClassHHysteresis, 0x4, 0x0, 0x0, 0x0, 0x0,
	TAS2562_ClassHMtct, 0x4, 0xb, 0x0, 0x0, 0x0,
	TAS2562_VBatFilter, 0x1, 0x38,
	TAS2562_ClassHReleaseTimer, 0x1, 0x3c,
	TAS2562_BoostSlope, 0x1, 0x78,
	TAS2562_TestPageConfiguration, 0x1, 0xd,
	TAS2562_ClassDConfiguration3, 0x1, 0x8e,
	TAS2562_ClassDConfiguration2, 0x1, 0x49,
	TAS2562_ClassDConfiguration4, 0x1, 0x21,
	TAS2562_ClassDConfiguration1, 0x1, 0x80,
	TAS2562_EfficiencyConfiguration, 0x1, 0xc1,
	TAS2562_BoostPeakCurrentLimit, 0x1, 0x37,
	TAS2562_ICN_REG, 0x4, 0x00, 0x03, 0x46, 0xdc,
	TAS2562_REG(0x0, 0xfd, 0xd), 0x1, 0xd,
	TAS2562_REG(0x0, 0xfd, 0x47), 0x1, 0x0,
	TAS2562_REG(0x0, 0x1, 0x21), 0x1, 0x8,
	0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int tas2562_codec_read(struct snd_soc_component *component,
		unsigned int reg)
{
	struct tas2562_priv *pTAS2562 =
		snd_soc_component_get_drvdata(component);

	dev_err(pTAS2562->dev, "Should not get here\n");

	return 0;
}

static int tas2562_iv_enable(struct tas2562_priv *pTAS2562, int enable)
{
	int nResult;

	if (enable) {
		pr_debug("%s: tas2562iv_enable \n", __func__);
		nResult = pTAS2562->update_bits(pTAS2562,
			channel_both, TAS2562_PowerControl,
		    TAS2562_PowerControl_ISNSPower_Mask |
		    TAS2562_PowerControl_VSNSPower_Mask,
		    TAS2562_PowerControl_VSNSPower_Active |
		    TAS2562_PowerControl_ISNSPower_Active);
	} else {
		pr_debug("%s: tas2562iv_disable \n", __func__);
		nResult = pTAS2562->update_bits(pTAS2562,
			channel_both, TAS2562_PowerControl,
			TAS2562_PowerControl_ISNSPower_Mask |
			TAS2562_PowerControl_VSNSPower_Mask,
			TAS2562_PowerControl_VSNSPower_PoweredDown |
			TAS2562_PowerControl_ISNSPower_PoweredDown);
	}
	tas2562iv_enable = enable;

	return nResult;
}

static int tas2562iv_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct tas2562_priv *pTAS2562 =
		snd_soc_component_get_drvdata(component);
	int iv_enable = 0, nResult = 0;

    if (component == NULL) {
		pr_err("%s: component is NULL \n",  __func__);
		return 0;
    }

    iv_enable = ucontrol->value.integer.value[0];

	nResult = tas2562_iv_enable(pTAS2562, iv_enable);

	pr_debug("%s: tas2562iv_enable = %d\n",
		__func__, tas2562iv_enable);

	return nResult;
}

static int tas2562iv_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct tas2562_priv *pTAS2562 =
		snd_soc_component_get_drvdata(component);
    int ret, value;

    ret = pTAS2562->read(pTAS2562, channel_left, TAS2562_PowerControl, &value);
    if (ret < 0)
		dev_err(pTAS2562->dev, "can't get ivsensor state %s, L=%d\n",
			__func__, __LINE__);
    else if (((value & TAS2562_PowerControl_ISNSPower_Mask)
             == TAS2562_PowerControl_ISNSPower_Active)
             && ((value & TAS2562_PowerControl_VSNSPower_Mask)
             == TAS2562_PowerControl_VSNSPower_Active)) {
        ucontrol->value.integer.value[0] = TAS2562_IVSENSER_ENABLE;
    } else {
        ucontrol->value.integer.value[0] = TAS2562_IVSENSER_DISABLE;
    }

    tas2562iv_enable = ucontrol->value.integer.value[0];	
	dev_err(pTAS2562->dev, "value: 0x%x, tas2562iv_enable %d\n",
		value, tas2562iv_enable);

    return 0;
}

static const struct snd_kcontrol_new tas2562_controls[] = {
SOC_ENUM_EXT("TAS2562 IVSENSE ENABLE", tas2562_enum[0],
		    tas2562iv_get, tas2562iv_put),
};

static int tas2562_codec_write(struct snd_soc_component *component,
	unsigned int reg, unsigned int value)
{
	struct tas2562_priv *pTAS2562 =
		snd_soc_component_get_drvdata(component);

	dev_err(pTAS2562->dev, "Should not get here\n");

	return 0;
}

static int tas2562_i2c_load_data(struct tas2562_priv *pTAS2562,
	enum channel chn, unsigned int *pData)
{
	unsigned int nRegister;
	unsigned int *nData;
	unsigned char Buf[128];
	unsigned int nLength = 0;
	unsigned int i = 0;
	unsigned int nSize = 0;
	int nResult = 0;

	do {
		nRegister = pData[nLength];
		nSize = pData[nLength + 1];
		nData = &pData[nLength + 2];
		if (nRegister == TAS2562_MSLEEP) {
			msleep(nData[0]);
			dev_dbg(pTAS2562->dev, "%s, msleep = %d\n",
				__func__, nData[0]);
		} else if (nRegister == TAS2562_MDELAY) {
			mdelay(nData[0]);
			dev_dbg(pTAS2562->dev, "%s, mdelay = %d\n",
				__func__, nData[0]);
		} else {
			if (nRegister != 0xFFFFFFFF) {
				if (nSize > 128) {
					dev_err(pTAS2562->dev,
						"%s, Line=%d, invalid size, maximum is 128 bytes!\n",
						__func__, __LINE__);
					break;
				}
				if (nSize > 1) {
					for (i = 0; i < nSize; i++)
						Buf[i] = (unsigned char)nData[i];
					nResult = pTAS2562->bulk_write(pTAS2562,
						chn, nRegister, Buf, nSize);
					if (nResult < 0)
						break;
				} else if (nSize == 1) {
					nResult = pTAS2562->write(pTAS2562,
						chn, nRegister, nData[0]);
					if (nResult < 0)
						break;
				} else {
					dev_err(pTAS2562->dev,
						"%s, Line=%d,invalid size, minimum is 1 bytes!\n",
						__func__, __LINE__);
				}
			}
		}
		nLength = nLength + 2 + pData[nLength + 1];
	} while (nRegister != 0xFFFFFFFF);

	return nResult;
}

static int tas2562_codec_suspend(struct snd_soc_component *component)
{
	struct tas2562_priv *pTAS2562 =
		snd_soc_component_get_drvdata(component);
	int ret = 0;

	mutex_lock(&pTAS2562->codec_lock);

	dev_dbg(pTAS2562->dev, "%s\n", __func__);
	pTAS2562->runtime_suspend(pTAS2562);

	mutex_unlock(&pTAS2562->codec_lock);

	return ret;
}

static int tas2562_codec_resume(struct snd_soc_component *component)
{
	struct tas2562_priv *pTAS2562 =
		snd_soc_component_get_drvdata(component);
	int ret = 0;

	mutex_lock(&pTAS2562->codec_lock);

	dev_dbg(pTAS2562->dev, "%s\n", __func__);
	pTAS2562->runtime_resume(pTAS2562);

	mutex_unlock(&pTAS2562->codec_lock);

	return ret;
}

static int tas2562_set_power_state(struct tas2562_priv *pTAS2562,
	enum channel chn, int state)
{
	int nResult = 0;
	int irqreg;
	/*unsigned int nValue;*/

	if ((pTAS2562->mbMute) && (state == TAS2562_POWER_ACTIVE))
		state = TAS2562_POWER_MUTE;

	dev_err(pTAS2562->dev, "set power state: %d\n", state);

	switch (state) {
	case TAS2562_POWER_ACTIVE:
        //if set format was not called by asoc, then set it default
		if(pTAS2562->mnASIFormat == 0)
			pTAS2562->mnASIFormat = SND_SOC_DAIFMT_CBS_CFS
				| SND_SOC_DAIFMT_IB_NF
				| SND_SOC_DAIFMT_I2S;

		nResult = tas2562_set_fmt(pTAS2562, pTAS2562->mnASIFormat);
		if (nResult < 0)
			return nResult;

#ifdef VIVO_PORT_SMARTPA
		if(pTAS2562->check_re()) {
			pTAS2562->set_re(pTAS2562->calibRe);
			pr_info("[SmartPA-%d] SetRe[0] called %d(0x%x)", __LINE__
								, pTAS2562->calibRe[0], pTAS2562->calibRe[0]);
			if(pTAS2562->mnChannels == 2)
				pr_info("[SmartPA-%d] SetRe[1] called %d(0x%x)", __LINE__
								, pTAS2562->calibRe[1], pTAS2562->calibRe[1]);
		} else {
			pr_err("[SmartPA-%d] SetRe is not called", __LINE__);
			if (pTAS2562->vivo_smartpa_protection) {
				pr_info("[SmartPA-%d] SetRe is not called;vivo_smartpa_protection", __LINE__);
				pTAS2562->calibRe[0] = 0;
				pTAS2562->calibRe[1] = 0;
				pTAS2562->set_re(pTAS2562->calibRe);
			}
		}
#endif
//Clear latched IRQ before power on

		pTAS2562->update_bits(pTAS2562, chn, TAS2562_InterruptConfiguration,
					TAS2562_InterruptConfiguration_LTCHINTClear_Mask,
					TAS2562_InterruptConfiguration_LTCHINTClear);

		pTAS2562->read(pTAS2562, channel_left, TAS2562_LatchedInterruptReg0, &irqreg);
			dev_info(pTAS2562->dev, "IRQ reg is: %s %d, %d\n", __func__, irqreg, __LINE__);
			pTAS2562->read(pTAS2562, channel_right, TAS2562_LatchedInterruptReg0, &irqreg);
			dev_info(pTAS2562->dev, "IRQ reg is: %s %d, %d\n", __func__, irqreg, __LINE__);

		pTAS2562->mbPowerUp = true;
		pTAS2562->mnPowerState = TAS2562_POWER_ACTIVE;
		schedule_delayed_work(&pTAS2562->irq_work, msecs_to_jiffies(10));
		break;
	case TAS2562_POWER_MUTE:
		nResult = pTAS2562->update_bits(pTAS2562, chn, TAS2562_PowerControl,
			TAS2562_PowerControl_OperationalMode10_Mask |
			TAS2562_PowerControl_ISNSPower_Mask |
			TAS2562_PowerControl_VSNSPower_Mask,
			TAS2562_PowerControl_OperationalMode10_Mute |
			TAS2562_PowerControl_VSNSPower_Active |
			TAS2562_PowerControl_ISNSPower_Active);
			pTAS2562->mbPowerUp = true;
			pTAS2562->mnPowerState = TAS2562_POWER_MUTE;
		break;
	case TAS2562_POWER_SHUTDOWN:
		//pTAS2562->enableIRQ(pTAS2562, false);
		if (hrtimer_active(&pTAS2562->mtimer)) {
			dev_info(pTAS2562->dev, "cancel timer\n");
			hrtimer_cancel(&pTAS2562->mtimer);
		}

		// Disable Comparator Hystersis before power Down
		nResult = pTAS2562->write(pTAS2562, chn,
			TAS2562_REG(0x0, 0x01, 0x21), 0x0);
		msleep(2);

		nResult = pTAS2562->update_bits(pTAS2562, chn, TAS2562_PowerControl,
			TAS2562_PowerControl_OperationalMode10_Mask,
			TAS2562_PowerControl_OperationalMode10_Shutdown);
			pTAS2562->mbPowerUp = false;
			pTAS2562->mnPowerState = TAS2562_POWER_SHUTDOWN;
		msleep(20);

		nResult = pTAS2562->write(pTAS2562, channel_both,
				TAS2562_REG(0x0, 0x0, 0x3e), 0x30);
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

		break;
	default:
		dev_err(pTAS2562->dev, "wrong power state setting %d\n", state);
	}

	return nResult;
}


static int tas2562_dac_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
		snd_soc_dapm_to_component(w->dapm);
	struct tas2562_priv *pTAS2562 =
		snd_soc_component_get_drvdata(component);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		dev_info(pTAS2562->dev, "SND_SOC_DAPM_POST_PMU\n");
		break;
	case SND_SOC_DAPM_PRE_PMD:
		dev_info(pTAS2562->dev, "SND_SOC_DAPM_PRE_PMD\n");
		break;

	}

	return 0;

}

static const struct snd_soc_dapm_widget tas2562_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("ASI1", "ASI1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC_E("DAC", NULL, SND_SOC_NOPM, 0, 0, tas2562_dac_event,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUTPUT("OUT"),
	SND_SOC_DAPM_SIGGEN("VMON"),
	SND_SOC_DAPM_SIGGEN("IMON")
};

static const struct snd_soc_dapm_route tas2562_audio_map[] = {
	{"DAC", NULL, "ASI1"},
	{"OUT", NULL, "DAC"},
	/*
	{"Voltage Sense", NULL, "VMON"},
	{"Current Sense", NULL, "IMON"},
	*/
};

static int tas2562_get_left_speaker_switch(struct snd_kcontrol *pKcontrol,
					struct snd_ctl_elem_value *pUcontrol)
{
    struct snd_soc_component *component =
		snd_soc_kcontrol_component(pKcontrol);
	struct tas2562_priv *pTAS2562 =
		snd_soc_component_get_drvdata(component);

	pUcontrol->value.integer.value[0] = pTAS2562->spk_l_control;
	dev_info(pTAS2562->dev, "%s, spk_l_control = %ld\n",
		__func__, pUcontrol->value.integer.value[0]);

	return 0;
}

static int tas2562_set_left_speaker_switch(struct snd_kcontrol *pKcontrol,
					struct snd_ctl_elem_value *pUcontrol)
{
    struct snd_soc_component *component =
		snd_soc_kcontrol_component(pKcontrol);
	struct tas2562_priv *pTAS2562 =
		snd_soc_component_get_drvdata(component);

	pTAS2562->spk_l_control = pUcontrol->value.integer.value[0];
	dev_info(pTAS2562->dev, "%s, spk_l_control = %d\n",
		__func__, pTAS2562->spk_l_control);

	return 0;
}

static int tas2562_get_right_speaker_switch(struct snd_kcontrol *pKcontrol,
					struct snd_ctl_elem_value *pUcontrol)
{
    struct snd_soc_component *component =
		snd_soc_kcontrol_component(pKcontrol);
	struct tas2562_priv *pTAS2562 =
		snd_soc_component_get_drvdata(component);

	pUcontrol->value.integer.value[0] = pTAS2562->spk_r_control;
	dev_info(pTAS2562->dev, "%s, spk_r_control = %ld\n",
		__func__, pUcontrol->value.integer.value[0]);

	return 0;
}

static int tas2562_set_right_speaker_switch(struct snd_kcontrol *pKcontrol,
					struct snd_ctl_elem_value *pUcontrol)
{
    struct snd_soc_component *component =
		snd_soc_kcontrol_component(pKcontrol);
	struct tas2562_priv *pTAS2562 =
		snd_soc_component_get_drvdata(component);

	pTAS2562->spk_r_control = pUcontrol->value.integer.value[0];
	dev_info(pTAS2562->dev, "%s, spk_r_control = %d\n",
		__func__, pTAS2562->spk_r_control);

	return 0;
}

static int tas2562_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_component *component = dai->component;
	struct tas2562_priv *pTAS2562 =
		snd_soc_component_get_drvdata(component);
	enum channel chn;

	if (!pTAS2562)
		return 0;

	dev_dbg(pTAS2562->dev, "%s: %d martpa_switch_enable %d\n",
		__func__, mute, pTAS2562->mbSmart_pa_switch_enable);

	if (!pTAS2562->mbSmart_pa_switch_enable) {
		pr_err("%s: smartpa_switch_enable not enable\n", __func__);
		return 0;
	}

	mutex_lock(&pTAS2562->codec_lock);

	if((pTAS2562->spk_l_control == 1)
		&& (pTAS2562->spk_r_control == 1)
		&& (pTAS2562->mnChannels == 2))
		chn = channel_both;
	else if(pTAS2562->spk_l_control == 1)
		chn = channel_left;
	else if((pTAS2562->spk_r_control == 1)
		&& (pTAS2562->mnChannels == 2))
		chn = channel_right;
	else
	{
		chn = channel_left;
	}
	

	if (mute) {
		tas2562_set_power_state(pTAS2562,
			channel_both, TAS2562_POWER_SHUTDOWN);
	} else {
		tas2562_set_power_state(pTAS2562, chn, TAS2562_POWER_ACTIVE);
	}

	mutex_unlock(&pTAS2562->codec_lock);

	return 0;
}

static int tas2562_IV_slot_config(struct tas2562_priv *pTAS2562)
{
	int ret = 0;

	dev_info(pTAS2562->dev, "%s, %d\n",
		__func__, pTAS2562->mnSlot_width);

	if (pTAS2562->mnChannels == 2) {
		pTAS2562->update_bits(pTAS2562, channel_left,
			TAS2562_TDMConfigurationReg5, 0xff, 0x42);

		pTAS2562->update_bits(pTAS2562, channel_left,
			TAS2562_TDMConfigurationReg6, 0xff, 0x40);

		pTAS2562->update_bits(pTAS2562, channel_right,
			TAS2562_TDMConfigurationReg5, 0xff, 0x46);

		pTAS2562->update_bits(pTAS2562, channel_right,
			TAS2562_TDMConfigurationReg6, 0xff, 0x44);
	} else if((pTAS2562->mnChannels == 1)
		&& (pTAS2562->mnSlot_width == 32)) {
		pTAS2562->update_bits(pTAS2562, channel_left,
			TAS2562_TDMConfigurationReg5, 0xff, 0x44);

		pTAS2562->update_bits(pTAS2562, channel_left,
			TAS2562_TDMConfigurationReg6, 0xff, 0x40);
	} else if((pTAS2562->mnChannels == 1)
		&& (pTAS2562->mnSlot_width == 16)) {
		pTAS2562->update_bits(pTAS2562, channel_left,
			TAS2562_TDMConfigurationReg5, 0xff, 0x42);

		pTAS2562->update_bits(pTAS2562, channel_left,
			TAS2562_TDMConfigurationReg6, 0xff, 0x40);
	} else {
		dev_err(pTAS2562->dev, "%s, wrong params, %d\n",
			__func__, pTAS2562->mnSlot_width);
	}

	return ret;
}

static int tas2562_set_slot(struct tas2562_priv *pTAS2562, int slot_width)
{
	int ret = 0;

	dev_info(pTAS2562->dev, "%s %d\n", __func__, slot_width);

	switch (slot_width) {
	case 16:
		ret = pTAS2562->update_bits(pTAS2562, channel_both,
			TAS2562_TDMConfigurationReg2,
			TAS2562_TDMConfigurationReg2_RXSLEN10_Mask,
			TAS2562_TDMConfigurationReg2_RXSLEN10_16Bits);
		break;
	case 24:
		ret = pTAS2562->update_bits(pTAS2562, channel_both,
			TAS2562_TDMConfigurationReg2,
			TAS2562_TDMConfigurationReg2_RXSLEN10_Mask,
			TAS2562_TDMConfigurationReg2_RXSLEN10_24Bits);
		break;
	case 32:
		ret = pTAS2562->update_bits(pTAS2562, channel_both,
			TAS2562_TDMConfigurationReg2,
			TAS2562_TDMConfigurationReg2_RXSLEN10_Mask,
			TAS2562_TDMConfigurationReg2_RXSLEN10_32Bits);
		break;
	case 0:
		/* Do not change slot width */
		break;
	default:
		dev_err(pTAS2562->dev, "slot width not supported");
		ret = -EINVAL;
	}

	if (ret >= 0)
		pTAS2562->mnSlot_width = slot_width;

	return ret;
}

static int tas2562_set_bitwidth(struct tas2562_priv *pTAS2562, int bitwidth)
{
	int slot_width_tmp = 16;

	dev_info(pTAS2562->dev, "%s %d\n", __func__, bitwidth);

	switch (bitwidth) {
	case SNDRV_PCM_FORMAT_S16_LE:
		pTAS2562->update_bits(pTAS2562, channel_both,
		TAS2562_TDMConfigurationReg2,
		TAS2562_TDMConfigurationReg2_RXWLEN32_Mask,
		TAS2562_TDMConfigurationReg2_RXWLEN32_16Bits);
		pTAS2562->mnCh_size = 16;
		slot_width_tmp = 16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		pTAS2562->update_bits(pTAS2562, channel_both,
		TAS2562_TDMConfigurationReg2,
		TAS2562_TDMConfigurationReg2_RXWLEN32_Mask,
		TAS2562_TDMConfigurationReg2_RXWLEN32_24Bits);
		pTAS2562->mnCh_size = 24;
		slot_width_tmp = 32;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		pTAS2562->update_bits(pTAS2562, channel_both,
		TAS2562_TDMConfigurationReg2,
		TAS2562_TDMConfigurationReg2_RXWLEN32_Mask,
		TAS2562_TDMConfigurationReg2_RXWLEN32_32Bits);
		pTAS2562->mnCh_size = 32;
		slot_width_tmp = 32;
		break;
	default:
		dev_info(pTAS2562->dev, "Not supported params format\n");
	}

	dev_info(pTAS2562->dev, "slot_width_tmp:%d\n", slot_width_tmp);
	tas2562_set_slot(pTAS2562, slot_width_tmp);

	pTAS2562->update_bits(pTAS2562, channel_left,
			TAS2562_TDMConfigurationReg2,
			TAS2562_TDMConfigurationReg2_RXSCFG54_Mask,
			TAS2562_TDMConfigurationReg2_RXSCFG54_Mono_Left);
	pTAS2562->update_bits(pTAS2562, channel_right,
			TAS2562_TDMConfigurationReg2,
			TAS2562_TDMConfigurationReg2_RXSCFG54_Mask,
			TAS2562_TDMConfigurationReg2_RXSCFG54_Mono_Right);

	tas2562_IV_slot_config(pTAS2562);
	
	dev_info(pTAS2562->dev, "mnCh_size: %d\n", pTAS2562->mnCh_size);
	pTAS2562->mnPCMFormat = bitwidth;

	return 0;
}

static int tas2562_set_samplerate(struct tas2562_priv *pTAS2562, int samplerate)
{
	switch (samplerate) {
	case 48000:
		pTAS2562->update_bits(pTAS2562, channel_both,
			TAS2562_TDMConfigurationReg0,
			TAS2562_TDMConfigurationReg0_SAMPRATERAMP_Mask,
			TAS2562_TDMConfigurationReg0_SAMPRATERAMP_48KHz);
		pTAS2562->update_bits(pTAS2562, channel_both,
			TAS2562_TDMConfigurationReg0,
			TAS2562_TDMConfigurationReg0_SAMPRATE31_Mask,
			TAS2562_TDMConfigurationReg0_SAMPRATE31_44_1_48kHz);
		break;
	case 44100:
		pTAS2562->update_bits(pTAS2562, channel_both,
			TAS2562_TDMConfigurationReg0,
			TAS2562_TDMConfigurationReg0_SAMPRATERAMP_Mask,
			TAS2562_TDMConfigurationReg0_SAMPRATERAMP_44_1KHz);
		pTAS2562->update_bits(pTAS2562, channel_both,
			TAS2562_TDMConfigurationReg0,
			TAS2562_TDMConfigurationReg0_SAMPRATE31_Mask,
			TAS2562_TDMConfigurationReg0_SAMPRATE31_44_1_48kHz);
		break;
	case 96000:
		pTAS2562->update_bits(pTAS2562, channel_both,
			TAS2562_TDMConfigurationReg0,
			TAS2562_TDMConfigurationReg0_SAMPRATERAMP_Mask,
			TAS2562_TDMConfigurationReg0_SAMPRATERAMP_48KHz);
		pTAS2562->update_bits(pTAS2562, channel_both,
			TAS2562_TDMConfigurationReg0,
			TAS2562_TDMConfigurationReg0_SAMPRATE31_Mask,
			TAS2562_TDMConfigurationReg0_SAMPRATE31_88_2_96kHz);
		break;
	case 88200:
		pTAS2562->update_bits(pTAS2562, channel_both,
			TAS2562_TDMConfigurationReg0,
			TAS2562_TDMConfigurationReg0_SAMPRATERAMP_Mask,
			TAS2562_TDMConfigurationReg0_SAMPRATERAMP_44_1KHz);
		pTAS2562->update_bits(pTAS2562, channel_both,
			TAS2562_TDMConfigurationReg0,
			TAS2562_TDMConfigurationReg0_SAMPRATE31_Mask,
			TAS2562_TDMConfigurationReg0_SAMPRATE31_88_2_96kHz);
		break;
	case 19200:
		pTAS2562->update_bits(pTAS2562, channel_both,
			TAS2562_TDMConfigurationReg0,
			TAS2562_TDMConfigurationReg0_SAMPRATERAMP_Mask,
			TAS2562_TDMConfigurationReg0_SAMPRATERAMP_48KHz);
		pTAS2562->update_bits(pTAS2562, channel_both,
			TAS2562_TDMConfigurationReg0,
			TAS2562_TDMConfigurationReg0_SAMPRATE31_Mask,
			TAS2562_TDMConfigurationReg0_SAMPRATE31_176_4_192kHz);
		break;
	case 17640:
		pTAS2562->update_bits(pTAS2562, channel_both,
			TAS2562_TDMConfigurationReg0,
			TAS2562_TDMConfigurationReg0_SAMPRATERAMP_Mask,
			TAS2562_TDMConfigurationReg0_SAMPRATERAMP_44_1KHz);
		pTAS2562->update_bits(pTAS2562, channel_both,
			TAS2562_TDMConfigurationReg0,
			TAS2562_TDMConfigurationReg0_SAMPRATE31_Mask,
			TAS2562_TDMConfigurationReg0_SAMPRATE31_176_4_192kHz);
		break;
	default:
		dev_info(pTAS2562->dev, "%s, unsupported sample rate, %d\n",
			__func__, samplerate);
	}

	pTAS2562->mnSamplingRate = samplerate;

	return 0;
}
static int tas2562_mute_ctrl_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
    struct snd_soc_component *component =
		snd_soc_kcontrol_component(pKcontrol);
	struct tas2562_priv *pTAS2562 =
		snd_soc_component_get_drvdata(component);

	pValue->value.integer.value[0] = pTAS2562->mbMute;
	dev_dbg(pTAS2562->dev, "tas2562_mute_ctrl_get = %d\n",
		pTAS2562->mbMute);

	return 0;
}

static int tas2562_mute_ctrl_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
    struct snd_soc_component *component =
		snd_soc_kcontrol_component(pKcontrol);
	struct tas2562_priv *pTAS2562 =
		snd_soc_component_get_drvdata(component);

	mbMute = pValue->value.integer.value[0];
	dev_dbg(pTAS2562->dev, "tas2562_mute_ctrl_put = %d\n", mbMute);
	pTAS2562->mbMute = !!mbMute;

	return 0;
}

static int tas2562_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct tas2562_priv *pTAS2562 =
		snd_soc_component_get_drvdata(component);
	//int blr_clk_ratio;
	int nResult = 0;
	unsigned int value=0;

	dev_dbg(pTAS2562->dev, "%s, format: %d\n", __func__,
		params_format(params));

	mutex_lock(&pTAS2562->codec_lock);

	if (pTAS2562->mnPCMFormat != params_format(params))
		nResult = tas2562_set_bitwidth(pTAS2562, params_format(params));
	if (nResult < 0) {
		dev_info(pTAS2562->dev, "set bitwidth failed, %d\n", nResult);
		goto ret;
	}

//	blr_clk_ratio = params_channels(params) * pTAS2562->mnCh_size;
//	dev_info(pTAS2562->dev, "blr_clk_ratio: %d\n", blr_clk_ratio);
//	if((pTAS2562->mnPCMFormat != params_format(params)) && (blr_clk_ratio != 0))
//		nResult = tas2562_IV_slot_config(pTAS2562, blr_clk_ratio);
	/*audio_v add for 1586.tx no refrence signal*/
		if (pTAS2562->mnChannels == 2) {
			pTAS2562->update_bits(pTAS2562, channel_left,
				TAS2562_TDMConfigurationReg6, 0xff, 0x40);
			pTAS2562->read(pTAS2562, channel_left, TAS2562_TDMConfigurationReg6, &value);

			pTAS2562->update_bits(pTAS2562, channel_right,
				TAS2562_TDMConfigurationReg6, 0xff, 0x44);
			pTAS2562->read(pTAS2562, channel_right, TAS2562_TDMConfigurationReg6, &value);
		} else if(pTAS2562->mnChannels == 1){
			pTAS2562->update_bits(pTAS2562, channel_left,
				TAS2562_TDMConfigurationReg6, 0xff, 0x40);
			pTAS2562->read(pTAS2562, channel_left, TAS2562_TDMConfigurationReg6, &value);
		} else {
			dev_err(pTAS2562->dev, "%s, wrong Channels, %d\n",
				__func__, pTAS2562->mnChannels);
		}
	/*audio_v add*/

	dev_info(pTAS2562->dev, "%s, sample rate: %d\n", __func__,
		params_rate(params));

	if (pTAS2562->mnSamplingRate != params_rate(params))
		nResult = tas2562_set_samplerate(pTAS2562, params_rate(params));

ret:
	mutex_unlock(&pTAS2562->codec_lock);
	return nResult;
}

static int tas2562_set_fmt(struct tas2562_priv *pTAS2562, unsigned int fmt)
{
	u8 tdm_rx_start_slot = 0, asi_cfg_1 = 0;
	int ret = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		asi_cfg_1 = 0x00;
		break;
	default:
		dev_err(pTAS2562->dev, "ASI format master is not found\n");
		ret = -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		dev_info(pTAS2562->dev, "INV format: NBNF\n");
		asi_cfg_1 |= TAS2562_TDMConfigurationReg1_RXEDGE_Rising;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		dev_info(pTAS2562->dev, "INV format: IBNF\n");
		asi_cfg_1 |= TAS2562_TDMConfigurationReg1_RXEDGE_Falling;
		break;
	default:
		dev_err(pTAS2562->dev, "ASI format Inverse is not found\n");
		ret = -EINVAL;
	}

	pTAS2562->update_bits(pTAS2562, channel_both, TAS2562_TDMConfigurationReg1,
		TAS2562_TDMConfigurationReg1_RXEDGE_Mask,
		asi_cfg_1);

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case (SND_SOC_DAIFMT_I2S):
		tdm_rx_start_slot = 1;
		break;
	case (SND_SOC_DAIFMT_DSP_A):
	case (SND_SOC_DAIFMT_DSP_B):
		tdm_rx_start_slot = 1;
		break;
	case (SND_SOC_DAIFMT_LEFT_J):
		tdm_rx_start_slot = 0;
		break;
	default:
		dev_err(pTAS2562->dev, "DAI Format is not found, default start 1, fmt=0x%x\n", fmt);
		//ret = -EINVAL;
		tdm_rx_start_slot = 1;
		break;
	}

	pTAS2562->update_bits(pTAS2562, channel_left, TAS2562_TDMConfigurationReg1,
		TAS2562_TDMConfigurationReg1_RXOFFSET51_Mask,
		(tdm_rx_start_slot << TAS2562_TDMConfigurationReg1_RXOFFSET51_Shift));

	if (pTAS2562->mnChannels == 2) {
		pTAS2562->update_bits(pTAS2562, channel_right, TAS2562_TDMConfigurationReg1,
			TAS2562_TDMConfigurationReg1_RXOFFSET51_Mask,
			((tdm_rx_start_slot + pTAS2562->mnSlot_width) << 
				TAS2562_TDMConfigurationReg1_RXOFFSET51_Shift));
	}

	pTAS2562->mnASIFormat = fmt;

	return 0;
}

static int tas2562_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
	struct tas2562_priv *pTAS2562 =
		snd_soc_component_get_drvdata(component);
	int ret = 0;

	dev_dbg(pTAS2562->dev, "%s, format=0x%x\n", __func__, fmt);

	ret = tas2562_set_fmt(pTAS2562, fmt);

	return ret;
}

static int tas2562_set_dai_tdm_slot(struct snd_soc_dai *dai,
		unsigned int tx_mask, unsigned int rx_mask,
		int slots, int slot_width)
{
	int ret = 0;
	struct snd_soc_component *component = dai->component;
	struct tas2562_priv *pTAS2562 =
		snd_soc_component_get_drvdata(component);

	dev_dbg(pTAS2562->dev, "%s, tx_mask:%d, rx_mask:%d, slots:%d, slot_width:%d",
			__func__, tx_mask, rx_mask, slots, slot_width);

	ret = tas2562_set_slot(pTAS2562, slot_width);

	return ret;
}

static struct snd_soc_dai_ops tas2562_dai_ops = {
	.digital_mute = tas2562_mute,
	.hw_params  = tas2562_hw_params,
	.set_fmt    = tas2562_set_dai_fmt,
	.set_tdm_slot = tas2562_set_dai_tdm_slot,
};

#define TAS2562_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
		SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

#define TAS2562_RATES (SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 \
						SNDRV_PCM_RATE_88200 |\
						SNDRV_PCM_RATE_96000 |\
						SNDRV_PCM_RATE_176400 |\
						SNDRV_PCM_RATE_192000\
						)

static struct snd_soc_dai_driver tas2562_dai_driver[] = {
	{
		.name = "tas2562 ASI1",
		.id = 0,
		.playback = {
			.stream_name    = "ASI1 Playback",
			.channels_min   = 2,
			.channels_max   = 2,
			.rates      = SNDRV_PCM_RATE_8000_192000,
			.formats    = TAS2562_FORMATS,
		},
		.capture = {
			.stream_name    = "ASI1 Capture",
			.channels_min   = 0,
			.channels_max   = 2,
			.rates          = SNDRV_PCM_RATE_8000_192000,
			.formats    = TAS2562_FORMATS,
		},
		.ops = &tas2562_dai_ops,
		.symmetric_rates = 1,
	},
};

static int tas2562_load_init(struct tas2562_priv *pTAS2562)
{
	int ret;

	/* set voltage to 10V */
	ret = pTAS2562->update_bits(pTAS2562, channel_left, TAS2562_BoostConfiguration2,
					TAS2562_BoostConfiguration2_BoostMaxVoltage_Mask,
					boostvoltage_l_val);
	ret = pTAS2562->update_bits(pTAS2562, channel_right, TAS2562_BoostConfiguration2,
					TAS2562_BoostConfiguration2_BoostMaxVoltage_Mask,
					boostvoltage_r_val);

	ret = pTAS2562->update_bits(pTAS2562, channel_left, TAS2562_PlaybackConfigurationReg0,
					TAS2562_PlaybackConfigurationReg0_AmplifierLevel51_Mask,
					ampoutput_l_val << 1);
	ret = pTAS2562->update_bits(pTAS2562, channel_right, TAS2562_PlaybackConfigurationReg0,
					TAS2562_PlaybackConfigurationReg0_AmplifierLevel51_Mask,
					ampoutput_r_val << 1);

	ret = pTAS2562->write(pTAS2562, channel_both, TAS2562_MiscConfigurationReg0, 0xca);
	if(ret < 0)
		return ret;

	if(pTAS2562->mnChannels == 2) {
		/* Solve the receiver low noise 0x10 -> 0x30 */
		ret = pTAS2562->write(pTAS2562, channel_left,
				TAS2562_REG(0x0, 0x0, 0x3e), 0x10);
		ret = pTAS2562->write(pTAS2562, channel_left,
				TAS2562_TDMConfigurationReg4, 0xf3);
		ret = pTAS2562->write(pTAS2562, channel_right,
				TAS2562_TDMConfigurationReg4, 0x13);
		if (ret < 0)
			return ret;
	} else {
		ret = pTAS2562->write(pTAS2562, channel_both,
				TAS2562_TDMConfigurationReg4, 0x03);
		if (ret < 0)
			return ret;
	}

	ret = pTAS2562->write(pTAS2562, channel_both, TAS2562_ClockConfiguration, 0x0c);
	if(ret < 0)
		return ret;
	ret = tas2562_i2c_load_data(pTAS2562, channel_both, p_tas2562_classH_D_data);

	return ret;
}

static int tas2562_codec_probe(struct snd_soc_component *component)
{

	struct tas2562_priv *pTAS2562 =
		snd_soc_component_get_drvdata(component);
	int ret, i;

	pTAS2562->component = component;
	snd_soc_component_init_regmap(component, pTAS2562->regmap);

	ret = snd_soc_add_component_controls(component, tas2562_controls,
					 ARRAY_SIZE(tas2562_controls));
	if (ret < 0) {
		pr_err("%s: add_codec_controls failed, err %d\n",
			__func__, ret);
		return ret;
	}

	tas2562_load_init(pTAS2562);
	tas2562_iv_enable(pTAS2562, 1);

	if (pTAS2562->reg_size) {
		for (i = 0; i < pTAS2562->reg_size; i++) {
			ret = pTAS2562->update_bits(pTAS2562, channel_both,
				TAS2562_REG(pTAS2562->reg_table[i * 5],
				pTAS2562->reg_table[i * 5 + 1],
				pTAS2562->reg_table[i * 5 + 2]),
				pTAS2562->reg_table[i * 5 + 3],
				pTAS2562->reg_table[i * 5 + 4]);
			dev_info(pTAS2562->dev, "%s: special params,"
				"book: 0x%x, page: 0x%x, reg: 0x%x, mask: 0x%x, val: 0x%x, ret: %d.\n",
				__func__, pTAS2562->reg_table[i * 5], pTAS2562->reg_table[i * 5 + 1],
				pTAS2562->reg_table[i * 5 + 2], pTAS2562->reg_table[i * 5 + 3],
				pTAS2562->reg_table[i * 5 + 4], ret);
		}
	}

	dev_info(pTAS2562->dev, "%s leave,\n", __func__);

	return 0;
}

static void tas2562_codec_remove(struct snd_soc_component *component)
{
	return;
}

static DECLARE_TLV_DB_SCALE(tas2562_digital_tlv, 800, 50, 0);
static DECLARE_TLV_DB_SCALE(tas2562_voltage_tlv, 600, 50, 0);

static const char *speaker_switch_text[] = {"Off", "On"};
static const struct soc_enum spk_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(speaker_switch_text), speaker_switch_text),
};

#ifdef VIVO_PORT_SMARTPA
static int tas_ctl_noimpl (struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
			(struct soc_mixer_control *)kcontrol->private_value;

	int shift = mc->shift;

	printk(KERN_INFO "%s: module=0x%x(%d)",
		__func__, shift, shift);

	return 0;
}

static int tas_spkr_prot_put_vi_lch_port(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct tas2562_priv *pTAS2562 =
		snd_soc_component_get_drvdata(component);
	uint32_t enable = ucontrol->value.integer.value[0];
	int ret = 0;

	if (!pTAS2562)
		return 0;

	if (enable) {
		pr_info("[smartamp] Setting the feedback module info for TAS");
		ret = afe_spk_prot_feed_back_cfg(pTAS2562->mi2s_tx_port_id,
				pTAS2562->mi2s_rx_port_id, 1, 0, 1);
	}

	return ret;
}

static const char *smart_pa_switch_text[] = {
		"0", "1"};

static const struct soc_enum smart_pa_switch_enum =
	SOC_ENUM_SINGLE_EXT(2, smart_pa_switch_text);

static int smartpa_switch_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct tas2562_priv *pTAS2562 =
		snd_soc_component_get_drvdata(component);

	if (!pTAS2562)
		return 0;

	ucontrol->value.integer.value[0] = pTAS2562->mbSmart_pa_switch_enable;

	return 0;
}

static int smartpa_switch_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct tas2562_priv *pTAS2562 =
		snd_soc_component_get_drvdata(component);

	if (!pTAS2562)
		return 0;

	pr_info("%s: smart pa enable %ld\n", __func__,
		ucontrol->value.integer.value[0]);

	mutex_lock(&pTAS2562->codec_lock);
	pTAS2562->mbSmart_pa_switch_enable = ucontrol->value.integer.value[0];
	mutex_unlock(&pTAS2562->codec_lock);

	return 0;
}
#endif

static int tas2562_amp_output_l_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct tas2562_priv *pTAS2562 =
		snd_soc_component_get_drvdata(component);
	if (!pTAS2562)
		return 0;
	pTAS2562->client->addr = pTAS2562->mnLAddr;
	snd_soc_get_volsw(kcontrol, ucontrol);
	pr_info("%s: addr %x value %ld\n", __func__,
		pTAS2562->client->addr, ucontrol->value.integer.value[0]);
	return 0;
}

static int tas2562_amp_output_l_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct tas2562_priv *pTAS2562 =
		snd_soc_component_get_drvdata(component);
	if (!pTAS2562)
		return 0;
	pTAS2562->client->addr = pTAS2562->mnLAddr;
	ampoutput_l_val = ucontrol->value.integer.value[0];
	snd_soc_put_volsw(kcontrol, ucontrol);
	pr_info("%s: addr %x value %ld\n", __func__,
		pTAS2562->client->addr, ucontrol->value.integer.value[0]);
	return 0;
}

static int tas2562_amp_output_r_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct tas2562_priv *pTAS2562 =
		snd_soc_component_get_drvdata(component);
	if (!pTAS2562)
		return 0;
	pTAS2562->client->addr = pTAS2562->mnRAddr;
	snd_soc_get_volsw(kcontrol, ucontrol);
	pr_info("%s: addr %x value %ld\n", __func__,
		pTAS2562->client->addr, ucontrol->value.integer.value[0]);
	return 0;
}

static int tas2562_amp_output_r_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct tas2562_priv *pTAS2562 =
		snd_soc_component_get_drvdata(component);
	if (!pTAS2562)
		return 0;
	pTAS2562->client->addr = pTAS2562->mnRAddr;
	ampoutput_r_val = ucontrol->value.integer.value[0];
	snd_soc_put_volsw(kcontrol, ucontrol);
	pr_info("%s: addr %x value %ld\n", __func__,
		pTAS2562->client->addr, ucontrol->value.integer.value[0]);
	return 0;
}

static int tas2562_boost_voltage_l_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct tas2562_priv *pTAS2562 =
		snd_soc_component_get_drvdata(component);
	if (!pTAS2562)
		return 0;
	pTAS2562->client->addr = pTAS2562->mnLAddr;
	snd_soc_get_volsw(kcontrol, ucontrol);
	pr_info("%s: addr %x value %ld\n", __func__,
		pTAS2562->client->addr, ucontrol->value.integer.value[0]);
	return 0;
}

static int tas2562_boost_voltage_l_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct tas2562_priv *pTAS2562 =
		snd_soc_component_get_drvdata(component);
	if (!pTAS2562)
		return 0;
	pTAS2562->client->addr = pTAS2562->mnLAddr;
	boostvoltage_l_val = ucontrol->value.integer.value[0];
	snd_soc_put_volsw(kcontrol, ucontrol);
	pr_info("%s: addr %x value %ld\n", __func__,
		pTAS2562->client->addr, ucontrol->value.integer.value[0]);
	return 0;
}

static int tas2562_boost_voltage_r_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct tas2562_priv *pTAS2562 =
		snd_soc_component_get_drvdata(component);
	if (!pTAS2562)
		return 0;
	pTAS2562->client->addr = pTAS2562->mnRAddr;
	snd_soc_get_volsw(kcontrol, ucontrol);
	pr_info("%s: addr %x value %ld\n", __func__,
		pTAS2562->client->addr, ucontrol->value.integer.value[0]);
	return 0;
}

static int tas2562_boost_voltage_r_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct tas2562_priv *pTAS2562 =
		snd_soc_component_get_drvdata(component);
	if (!pTAS2562)
		return 0;
	pTAS2562->client->addr = pTAS2562->mnRAddr;
	boostvoltage_r_val = ucontrol->value.integer.value[0];
	snd_soc_put_volsw(kcontrol, ucontrol);
	pr_info("%s: addr %x value %ld\n", __func__,
		pTAS2562->client->addr, ucontrol->value.integer.value[0]);
	return 0;
}

static const struct snd_kcontrol_new tas2562_snd_controls[] = {
	SOC_SINGLE_EXT_TLV("Amp Output L Level", TAS2562_PlaybackConfigurationReg0,
		1, 28, 0, tas2562_amp_output_l_get, tas2562_amp_output_l_put,
		tas2562_digital_tlv),
	SOC_SINGLE_EXT_TLV("Amp Output R Level", TAS2562_PlaybackConfigurationReg0,
		1, 28, 0, tas2562_amp_output_r_get, tas2562_amp_output_r_put,
		tas2562_digital_tlv),
	SOC_SINGLE_EXT_TLV("Boost Voltage L Level", TAS2562_BoostConfiguration2,
		0, 13, 0, tas2562_boost_voltage_l_get, tas2562_boost_voltage_l_put,
		tas2562_voltage_tlv),
	SOC_SINGLE_EXT_TLV("Boost Voltage R Level", TAS2562_BoostConfiguration2,
		0, 13, 0, tas2562_boost_voltage_r_get, tas2562_boost_voltage_r_put,
		tas2562_voltage_tlv),
	SOC_SINGLE_EXT("SmartPA Mute", SND_SOC_NOPM, 0, 0x0001, 0,
		tas2562_mute_ctrl_get, tas2562_mute_ctrl_put),
	SOC_ENUM_EXT("TAS2562 Left Speaker Switch", spk_enum[0],
		tas2562_get_left_speaker_switch, tas2562_set_left_speaker_switch),
	SOC_ENUM_EXT("TAS2562 Right Speaker Switch", spk_enum[0],
		tas2562_get_right_speaker_switch, tas2562_set_right_speaker_switch),
#ifdef VIVO_PORT_SMARTPA
	SOC_SINGLE_EXT("CAPI_V2_TAS_FEEDBACK_INFO", 0, 0, 1, 0,
			tas_ctl_noimpl, tas_spkr_prot_put_vi_lch_port),
	SOC_ENUM_EXT("SmartPA Switch", smart_pa_switch_enum,
			smartpa_switch_get, smartpa_switch_put),
#endif
};

static struct snd_soc_component_driver soc_codec_driver_tas2562 = {
	.name			= TAS2562_CODEC,
	.probe			= tas2562_codec_probe,
	.remove			= tas2562_codec_remove,
	.read			= tas2562_codec_read,
	.write			= tas2562_codec_write,
	.suspend		= tas2562_codec_suspend,
	.resume			= tas2562_codec_resume,
	.controls		= tas2562_snd_controls,
	.num_controls		= ARRAY_SIZE(tas2562_snd_controls),
	.dapm_widgets		= tas2562_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(tas2562_dapm_widgets),
	.dapm_routes		= tas2562_audio_map,
	.num_dapm_routes	= ARRAY_SIZE(tas2562_audio_map),
};

int tas2562_register_codec(struct tas2562_priv *pTAS2562)
{
	int nResult = 0;

	dev_info(pTAS2562->dev, "%s, enter\n", __func__);

	nResult = snd_soc_register_component(pTAS2562->dev,
		&soc_codec_driver_tas2562, tas2562_dai_driver,
		ARRAY_SIZE(tas2562_dai_driver));

	if (nResult) {
#if defined(VIVO_PROJECT_MODEL)
/*GDPR code*/
#else
/* not GDPR code start */
#ifdef CONFIG_RSC_VAUDIT_DEBUG
		rsc_vaudit(VAUDIT_AUDIO, AUDIO_SOUND_CRAD, 0,
			"Failed to register TAS2562 codec: %d.", nResult);
#endif
/* not GDPR code end */
#endif
	}

	return nResult;
}

int tas2562_deregister_codec(struct tas2562_priv *pTAS2562)
{
	snd_soc_unregister_component(pTAS2562->dev);

	return 0;
}

void tas2562_LoadConfig(struct tas2562_priv *pTAS2562)
{
	int ret = 0;

	if (hrtimer_active(&pTAS2562->mtimer)) {
		dev_info(pTAS2562->dev, "cancel timer\n");
		hrtimer_cancel(&pTAS2562->mtimer);
	} else {
		dev_info(pTAS2562->dev, "timer not active\n");
	}

	pTAS2562->hw_reset(pTAS2562);
	msleep(2);
	pTAS2562->write(pTAS2562, channel_both, TAS2562_SoftwareReset,
			TAS2562_SoftwareReset_SoftwareReset_Reset);
	msleep(3);

	tas2562_IV_slot_config(pTAS2562);

	tas2562_load_init(pTAS2562);
	tas2562_iv_enable(pTAS2562, tas2562iv_enable);

	ret = tas2562_set_slot(pTAS2562, pTAS2562->mnSlot_width);
	if (ret < 0)
		goto end;

	ret = tas2562_set_fmt(pTAS2562, pTAS2562->mnASIFormat);
	if (ret < 0)
		goto end;

	ret = tas2562_set_bitwidth(pTAS2562, pTAS2562->mnPCMFormat);
	if (ret < 0)
		goto end;

	ret = tas2562_set_samplerate(pTAS2562, pTAS2562->mnSamplingRate);
	if (ret < 0)
		goto end;

	ret = tas2562_set_power_state(pTAS2562, channel_both, pTAS2562->mnPowerState);
	if (ret < 0)
		goto end;

end:
/* power up failed, restart later */
	if (ret < 0)
		schedule_delayed_work(&pTAS2562->irq_work,
				msecs_to_jiffies(1000));
}

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("TAS2562 ALSA SOC Smart Amplifier driver");
MODULE_LICENSE("GPL v2");
#endif /* CONFIG_TAS2562_CODEC */
