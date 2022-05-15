/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2018-2020, The Linux Foundation. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/soc/qcom/max20328-i2c.h>

#ifdef pr_debug
#undef pr_debug
#endif
#define pr_debug pr_err

#ifdef dev_dbg
#undef dev_dbg
#endif
#define dev_dbg dev_err

#ifdef pr_info
#undef pr_info
#endif
#define pr_info pr_err

#ifdef dev_info
#undef dev_info
#endif
#define dev_info dev_err

#define MAX20328_I2C_NAME	"max20328-driver"

#define WAKEBYTE_TIMEOUT_MSEC	(3000)

#define USB_HSPHY_3P3_VOL_MIN			3050000 /* uV */
#define USB_HSPHY_3P3_VOL_MAX			3300000 /* uV */
#define USB_HSPHY_3P3_HPM_LOAD			16000	/* uA */
#define USB_HSPHY_3P3_VOL_FSHOST		3150000 /* uV */

#define USB_HSPHY_1P8_VOL_MIN			1704000 /* uV */
#define USB_HSPHY_1P8_VOL_MAX			1800000 /* uV */
#define USB_HSPHY_1P8_HPM_LOAD			19000	/* uA */

#define SYS_NODE

enum max20328_regs_def {
	MAX20328_DEVICE_ID       = 0x00,
	MAX20328_ADC_VAL         = 0x01,
	MAX20328_STATUS1         = 0x02,
	MAX20328_STATUS2         = 0x03,
	MAX20328_INTERRUPT       = 0x04,
	MAX20328_MASK            = 0x05,
	MAX20328_CONTROL1        = 0x06,
	MAX20328_CONTROL2        = 0x07,
	MAX20328_CONTROL3        = 0x08,
	MAX20328_ADC_CONTROL1    = 0x09,
	MAX20328_ADC_CONTROL2    = 0x0A,
	MAX20328_HIHS_VAL        = 0x0B,
	MAX20328_OMTP_VAL        = 0x0C,
	MAX20328_SW_DEFLT1       = 0x0D,
	MAX20328_SW_DEFLT2       = 0x0E,
	MAX20328_REG_MAX         = MAX20328_SW_DEFLT2,
};

enum fsa4480_regs_def {
	FSA4480_REG_DEVID    		= 0x00,
	FSA4480_REG_OVP_INT_MASK	= 0x01,
	FSA4480_REG_OVP_INT_FLAG	= 0x02,
	FSA4480_REG_OVP_STA 		= 0x03,
	FSA4480_REG_SW_EN  			= 0x04,
	FSA4480_REG_SW_SEL 			= 0x05,
	FSA4480_REG_SW_STA0 		= 0x06,
	FSA4480_REG_SW_STA1 		= 0x07,
	FSA4480_REG_AUDIO_L 		= 0x08,
	FSA4480_REG_AUDIO_R 		= 0x09,
	FSA4480_REG_MIC_SW			= 0x0a,
	FSA4480_REG_SENSE_SW		= 0x0b,
	FSA4480_REG_AUDIO_G 		= 0x0c,
	FSA4480_REG_RL_DELAY		= 0x0d,
	FSA4480_REG_ML_DELAY		= 0x0e,
	FSA4480_REG_SL_DELAY		= 0x0f,
	FSA4480_REG_GL_DELAY		= 0x10,
	FSA4480_REG_AUDIO_STA		= 0x11,
	FSA4480_REG_FUNC_EN 		= 0x12,
	FSA4480_REG_RES_DET_SET 	= 0x13,
	FSA4480_REG_RES_DET_VAL 	= 0x14,
	FSA4480_REG_RES_DET_THD 	= 0x15,
	FSA4480_REG_RES_DET_IVAL	= 0x16,
	FSA4480_REG_AUDIO_JACK_STA	= 0x17,
	FSA4480_REG_DET_INT 		= 0x18,
	FSA4480_REG_DET_INT_MASK	= 0x19,
	FSA4480_REG_AUDIO_RGE1		= 0x1a,
	FSA4480_REG_AUDIO_RGE2		= 0x1b,
	FSA4480_REG_MIC_THD0		= 0x1c,
	FSA4480_REG_MIC_THD1		= 0x1d,
	FSA4480_REG_I2C_RESET		= 0x1e,
	FSA4480_REG_CS_SET			= 0x1f,
	FSA4480_REG_CS_E0           = 0xe0,
	FSA4480_REG_CS_E1 			= 0xe1,
	FSA4480_REG_CS_E2			= 0xe2,
	FSA4480_REG_CS_E3 			= 0xe3,
	FSA4480_REG_CS_E4			= 0xe4,
	FSA4480_REG_CS_E5			= 0xe5,
	FSA4480_REG_CS_E6 			= 0xe6,
	FSA4480_REG_CS_E7 			= 0xe7,
	FSA4480_REG_CS_E8 			= 0xe8,
	FSA4480_REG_CS_E9 			= 0xe9,
	FSA4480_REG_CS_EA			= 0xea,
	FSA4480_REG_CS_EB			= 0xeb,
	FSA4480_REG_CS_EC 			= 0xec,
	FSA4480_REG_CS_ED			= 0xed,
	FSA4480_REG_CS_EE			= 0xee,
	FSA4480_REG_CS_EF			= 0xef,
	FSA4480_REG_CS_F0			= 0xf0,
	FSA4480_REG_CS_F1			= 0xf1,
	FSA4480_REG_CS_F2			= 0xf2,
	FSA4480_REG_MAX  			= FSA4480_REG_CS_F2,
};

#define USB_HSPHY_3P3_VOL_MIN			3050000 /* uV */
#define USB_HSPHY_3P3_VOL_MAX			3300000 /* uV */
#define USB_HSPHY_3P3_HPM_LOAD			16000	/* uA */

static struct max20328_priv *usbc_switch_mmax_priv;

int usb_typec_analog_audio_switch_ic = -1;
EXPORT_SYMBOL(usb_typec_analog_audio_switch_ic);

static u8 max20328_regs[] = {
	MAX20328_DEVICE_ID,
	MAX20328_ADC_VAL,
	MAX20328_STATUS1,
	MAX20328_STATUS2,
	MAX20328_INTERRUPT,
	MAX20328_MASK,
	MAX20328_CONTROL1,
	MAX20328_CONTROL2,
	MAX20328_CONTROL3,
	MAX20328_ADC_CONTROL1,
	MAX20328_ADC_CONTROL2,
	MAX20328_HIHS_VAL,
	MAX20328_OMTP_VAL,
	MAX20328_SW_DEFLT1,
	MAX20328_SW_DEFLT2,
	MAX20328_REG_MAX,
};

static u8 fsa4480_regs[] = {
	FSA4480_REG_DEVID,
	FSA4480_REG_OVP_INT_MASK,
	FSA4480_REG_OVP_INT_FLAG,
	FSA4480_REG_OVP_STA,
	FSA4480_REG_SW_EN,
	FSA4480_REG_SW_SEL,
	FSA4480_REG_SW_STA0,
	FSA4480_REG_SW_STA1,
	FSA4480_REG_AUDIO_L,
	FSA4480_REG_AUDIO_R,
	FSA4480_REG_MIC_SW,
	FSA4480_REG_SENSE_SW,
	FSA4480_REG_AUDIO_G,
	FSA4480_REG_RL_DELAY,
	FSA4480_REG_ML_DELAY,
	FSA4480_REG_SL_DELAY,
	FSA4480_REG_GL_DELAY,
	FSA4480_REG_AUDIO_STA,
	FSA4480_REG_FUNC_EN,
	FSA4480_REG_RES_DET_SET,
	FSA4480_REG_RES_DET_VAL,
	FSA4480_REG_RES_DET_THD,
	FSA4480_REG_RES_DET_IVAL,
	FSA4480_REG_AUDIO_JACK_STA,
	FSA4480_REG_DET_INT,
	FSA4480_REG_DET_INT_MASK,
	FSA4480_REG_AUDIO_RGE1,
	FSA4480_REG_AUDIO_RGE2,
	FSA4480_REG_MIC_THD0,
	FSA4480_REG_MIC_THD1,
	FSA4480_REG_I2C_RESET,
	FSA4480_REG_CS_SET,
	FSA4480_REG_CS_E0,
	FSA4480_REG_CS_E1,
	FSA4480_REG_CS_E2,
	FSA4480_REG_CS_E3,
	FSA4480_REG_CS_E4,
	FSA4480_REG_CS_E5,
	FSA4480_REG_CS_E6,
	FSA4480_REG_CS_E7,
	FSA4480_REG_CS_E8,
	FSA4480_REG_CS_E9,
	FSA4480_REG_CS_EA,
	FSA4480_REG_CS_EB,
	FSA4480_REG_CS_EC,
	FSA4480_REG_CS_ED,
	FSA4480_REG_CS_EE,
	FSA4480_REG_CS_EF,
	FSA4480_REG_CS_F0,
	FSA4480_REG_CS_F1,
	FSA4480_REG_CS_F2,
	FSA4480_REG_MAX,
};

struct max20328_reg_val {
	u16 reg;
	u8 val;
};

static struct regmap_config max20328_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX20328_REG_MAX,
};

static const struct max20328_reg_val mmax_reg_i2c_defaults[] = {
	{MAX20328_SW_DEFLT1, 0x40}, /* 0x0D*/
	{MAX20328_SW_DEFLT2, 0x00}, /* 0x0E*/
	{MAX20328_ADC_CONTROL2, 0xF0}, /* 0x0A*/
	{MAX20328_CONTROL2, 0x00}, /* 0x07*/
	{MAX20328_CONTROL3, 0x00}, /* 0x08*/
	{MAX20328_ADC_CONTROL1, 0x30}, /* 0x09*/
	{MAX20328_CONTROL1, 0x13}, /* 0x06*/
};

static const struct max20328_reg_val fsa_reg_i2c_defaults[] = {
	{FSA4480_REG_SW_SEL, 0x18}, /* 0x05*/
	{FSA4480_REG_SW_EN, 0x98}, /* 0x04*/
};

static int max20328_usbc_analog_setup_switches(struct max20328_priv *mmax_priv);


static int max20328_usbc_wait_event_wake(struct max20328_priv *mmax_priv)
{
	int ret;

	pr_debug("%s: enter.\n", __func__);

	__pm_wakeup_event(mmax_priv->usbc_wake_lock, WAKEBYTE_TIMEOUT_MSEC);

	ret = wait_event_timeout(mmax_priv->irq_waitq,
				(mmax_priv->dev->power.is_suspended == false),
				msecs_to_jiffies(1000));
	if (!ret)
		pr_err("%s: check suspend timed out.\n", __func__);

	pr_debug("%s: leave.\n", __func__);
	return ret;
}

static int max20328_drp_wait_event_wake(struct max20328_priv *mmax_priv)
{
	int ret;

	pr_debug("%s: enter.\n", __func__);

	__pm_wakeup_event(mmax_priv->usbc_wake_lock, WAKEBYTE_TIMEOUT_MSEC);

	ret = wait_event_timeout(mmax_priv->irq_waitq,
				(usbc_switch_mmax_priv->drp_suspend_state == false),
				msecs_to_jiffies(2000));
	if (!ret)
		pr_err("%s: check suspend timed out.\n", __func__);

	pr_debug("%s: leave.\n", __func__);
	return ret;
}

/*audio_v add */
static int max20328_sbu_sel_enable_power(struct max20328_priv *mmax_priv, bool on)
{
	int ret = 0;
	if (!mmax_priv || !mmax_priv->vdda307)
		return -EINVAL;

	dev_dbg(mmax_priv->dev, "%s turn %s\n", __func__, on ? "on" : "off");

	if (mmax_priv->sbu_vcc_enabled == on) {
		dev_dbg(mmax_priv->dev, "sbu_sel regulators no change,so nothing to do!\n");
		return 0;
	}

	if (!on)
		goto disable_vdda307;

	ret = regulator_set_load(mmax_priv->vdda307, USB_HSPHY_3P3_HPM_LOAD);
	if (ret < 0) {
		dev_err(mmax_priv->dev, "Unable to set HPM of vdda307:%d\n", ret);
		goto err_vdd;
	}

	ret = regulator_set_voltage(mmax_priv->vdda307, USB_HSPHY_3P3_VOL_MIN,
						USB_HSPHY_3P3_VOL_MAX);
	if (ret) {
		dev_err(mmax_priv->dev,
				"Unable to set voltage for vdda307:%d\n", ret);
		goto put_vdda307_lpm;
	}

	ret = regulator_enable(mmax_priv->vdda307);
	if (ret) {
		dev_err(mmax_priv->dev, "Unable to enable vdda307:%d\n", ret);
		goto unset_vdd307;
	}

	mmax_priv->sbu_vcc_enabled = true;

	dev_dbg(mmax_priv->dev, "%s(): sbu_sel's regulators are turned ON.\n", __func__);
	return ret;

disable_vdda307:
	ret = regulator_disable(mmax_priv->vdda307);
	if (ret)
		dev_err(mmax_priv->dev, "Unable to disable vdda307:%d\n", ret);

unset_vdd307:
	ret = regulator_set_voltage(mmax_priv->vdda307, 0, USB_HSPHY_3P3_VOL_MAX);
	if (ret)
		dev_err(mmax_priv->dev,
			"Unable to set (0) voltage for vdda307:%d\n", ret);

put_vdda307_lpm:
	ret = regulator_set_load(mmax_priv->vdda307, 0);
	if (ret < 0)
		dev_err(mmax_priv->dev, "Unable to set (0) HPM of vdda307\n");
err_vdd:
	mmax_priv->sbu_vcc_enabled = false;
	dev_dbg(mmax_priv->dev, "sbu_sel's regulators are turned OFF.\n");
	return ret;
}
/*audio_v add end*/

/*audio_v: archer add */
static int msm_hsphy_config_vdd(struct max20328_priv *mmax_priv, int high)
{
	int min, ret;

	min = high ? 1 : 0; /* low or none? */
	ret = regulator_set_voltage(mmax_priv->vdd, mmax_priv->vdd_levels[min],
				    mmax_priv->vdd_levels[2]);
	if (ret) {
		dev_err(mmax_priv->dev, "unable to set voltage for hsusb vdd\n");
		return ret;
	}

	dev_dbg(mmax_priv->dev, "%s: min_vol:%d max_vol:%d\n", __func__,
		mmax_priv->vdd_levels[min], mmax_priv->vdd_levels[2]);

	return ret;
}
/*audio_v: archer add end*/

static int max20328_usbc_enable_power(struct max20328_priv *mmax_priv, bool on)
{
	int ret = 0;

	if (!mmax_priv || !mmax_priv->vdda33)
		return -EINVAL;

	dev_dbg(mmax_priv->dev, "%s turn %s regulators. power_enabled:%d\n",
			__func__, on ? "on" : "off", mmax_priv->power_enabled);

	if (mmax_priv->power_enabled == on) {
		dev_dbg(mmax_priv->dev, "%s USBC' regulators not change, so nothing to do.\n", __func__);
		return 0;
	}

	if (!on)
		goto disable_vdda33;

	/*audio_v: archer add for PD2049,sm8350 L5B,L1C,L2B have Timing requirements,can't just control L2B*/
	if (mmax_priv->supply_nonindepend == true) {
		ret = msm_hsphy_config_vdd(mmax_priv, true);
		if (ret) {
			dev_err(mmax_priv->dev, "Unable to config VDD:%d\n", ret);
			goto err_vdd;
		}
		ret = regulator_enable(mmax_priv->vdd);
		if (ret) {
			dev_err(mmax_priv->dev, "Unable to enable VDD\n");
			goto unconfig_vdd;
		}

		ret = regulator_set_load(mmax_priv->vdda18, USB_HSPHY_1P8_HPM_LOAD);
		if (ret < 0) {
			dev_err(mmax_priv->dev, "Unable to set HPM of vdda18:%d\n", ret);
			goto disable_vdd;
		}

		ret = regulator_set_voltage(mmax_priv->vdda18, USB_HSPHY_1P8_VOL_MIN,
							USB_HSPHY_1P8_VOL_MAX);
		if (ret) {
			dev_err(mmax_priv->dev,
					"Unable to set voltage for vdda18:%d\n", ret);
			goto put_vdda18_lpm;
		}

		ret = regulator_enable(mmax_priv->vdda18);
		if (ret) {
			dev_err(mmax_priv->dev, "Unable to enable vdda18:%d\n", ret);
			goto unset_vdda18;
		}
	}
	/*audio_v: archer add end*/

	ret = regulator_set_load(mmax_priv->vdda33, USB_HSPHY_3P3_HPM_LOAD);
	if (ret < 0) {
		dev_err(mmax_priv->dev, "Unable to set HPM of vdda33:%d\n", ret);
		goto disable_vdda18;
	}

	ret = regulator_set_voltage(mmax_priv->vdda33, USB_HSPHY_3P3_VOL_MIN,
						USB_HSPHY_3P3_VOL_MAX);
	if (ret) {
		dev_err(mmax_priv->dev,
				"Unable to set voltage for vdda33:%d\n", ret);
		goto put_vdda33_lpm;
	}

	ret = regulator_enable(mmax_priv->vdda33);
	if (ret) {
		dev_err(mmax_priv->dev, "Unable to enable vdda33:%d\n", ret);
		goto unset_vdd33;
	}

	mmax_priv->power_enabled = true;

	dev_dbg(mmax_priv->dev, "%s(): USBC's regulators are turned ON.\n", __func__);
	return ret;

disable_vdda33:
	ret = regulator_disable(mmax_priv->vdda33);
	if (ret)
		dev_err(mmax_priv->dev, "Unable to disable vdda33:%d\n", ret);

unset_vdd33:
	ret = regulator_set_voltage(mmax_priv->vdda33, 0, USB_HSPHY_3P3_VOL_MAX);
	if (ret)
		dev_err(mmax_priv->dev,
			"Unable to set (0) voltage for vdda33:%d\n", ret);

put_vdda33_lpm:
	ret = regulator_set_load(mmax_priv->vdda33, 0);
	if (ret < 0)
		dev_err(mmax_priv->dev, "Unable to set (0) HPM of vdda33\n");

/*audio_v : archer add*/
disable_vdda18:
	if (mmax_priv->supply_nonindepend == true) {
		ret = regulator_disable(mmax_priv->vdda18);
		if (ret)
			dev_err(mmax_priv->dev, "Unable to disable vdda18:%d\n", ret);
	}
	
unset_vdda18:
	if (mmax_priv->supply_nonindepend == true) {
		ret = regulator_set_voltage(mmax_priv->vdda18, 0, USB_HSPHY_1P8_VOL_MAX);
		if (ret)
			dev_err(mmax_priv->dev, "Unable to set (0) voltage for vdda18:%d\n", ret);
	}
put_vdda18_lpm:
	if (mmax_priv->supply_nonindepend == true) {
		ret = regulator_set_load(mmax_priv->vdda18, 0);
		if (ret < 0)
			dev_err(mmax_priv->dev, "Unable to set LPM of vdda18\n");
	}
		
disable_vdd:
	if (mmax_priv->supply_nonindepend == true) {
		ret = regulator_disable(mmax_priv->vdd);
		if (ret)
			dev_err(mmax_priv->dev, "Unable to disable vdd:%d\n", ret);
	}
unconfig_vdd:
	if (mmax_priv->supply_nonindepend == true) {
		ret = msm_hsphy_config_vdd(mmax_priv, false);
		if (ret)
			dev_err(mmax_priv->dev, "Unable unconfig VDD:%d\n", ret);
	}
/*audio_v : archer add end*/

err_vdd:
	mmax_priv->power_enabled = false;
	dev_dbg(mmax_priv->dev, "USBC's regulators are turned OFF.\n");
	return ret;
}

static void max20328_usbc_switch_enable(struct max20328_priv *mmax_priv, bool enable)
{
	unsigned int val = 0, reg_06 = 0;
	int ret;

	if (!mmax_priv->regmap) {
		dev_err(mmax_priv->dev, "%s: regmap invalid\n", __func__);
		return;
	}

	val = enable ? 0x13 : 0x03;
	ret = regmap_write(mmax_priv->regmap, 0x06, val);
	if (ret)
		dev_err(mmax_priv->dev, "%s: failed %d\n", __func__, ret);
	usleep_range(5 * 1000, 5 * 1000);
	regmap_read(mmax_priv->regmap, 0x06, &reg_06);
	dev_dbg(mmax_priv->dev, "%s: enable (%d) reg_0x06 (0x%x)\n",
			__func__, enable, reg_06);
}

static void max20328_usbc_set_switch_mode(struct max20328_priv *mmax_priv, int mode)
{
	int i;
	unsigned int val = 0;

	if (!mmax_priv->regmap) {
		dev_err(mmax_priv->dev, "%s: regmap invalid\n", __func__);
		return;
	}

	mutex_lock(&mmax_priv->usbc_switch_lock);
	dev_dbg(mmax_priv->dev, "%s: mode (%d)\n", __func__, mode);

	switch (mmax_priv->usbc_ic_mode) {
	case MAX20328:
		switch (mode) {
		case TYPEC_ACCESSORY_NONE: /* USB mode */
			/*audio_v modify for mos test*/
			if (mmax_priv->is_mostest) {
				regmap_write(mmax_priv->regmap, 0x0E, 0x40); /* DEF register2 set 00 */
				regmap_write(mmax_priv->regmap, 0x0D, 0x47); /* DEF register1 set TOP side closed in data connection, bottom side is open */
			} else {
				regmap_write(mmax_priv->regmap, 0x0E, 0x00); /* DEF register2 set 00 */
				regmap_write(mmax_priv->regmap, 0x0D, 0x40); /* DEF register1 set TOP side closed in data connection, bottom side is open */
			}
			/*audio_v modify end*/
			regmap_write(mmax_priv->regmap, 0x07, 0x00); /* CONTROL2 register, switch state NOT Force mode nor follow MODE[0:2] */
			regmap_write(mmax_priv->regmap, 0x08, 0x00); /* CONTROL3 register, force value is not use, anyway default it. */
			regmap_write(mmax_priv->regmap, 0x09, 0x30); /* ADC CONTROL1, ADC is always off on USB MODE */
			regmap_write(mmax_priv->regmap, 0x06, 0x13); /* CONTROL1 register, switch enable, default programmable with registers 0x0D and 0x0E */
			break;
		case TYPEC_ACCESSORY_AUDIO:
			regmap_write(mmax_priv->regmap, 0x0D, 0x03); /* DEF register */
			regmap_write(mmax_priv->regmap, 0x0E, 0x10); /* DEF register2 */
			regmap_write(mmax_priv->regmap, 0x07, 0x02); /* CONTROL2 register */
			regmap_write(mmax_priv->regmap, 0x08, 0x00); /* CONTROL3 register */
			regmap_write(mmax_priv->regmap, 0x09, 0x00); /* ADC CONTROL1, ADC is always off */
			regmap_write(mmax_priv->regmap, 0x06, 0x13); /* CONTROL1 register, switch enable, single Audio accessory */
			break;
		default:
			break;
		}
		break;

	case FSA4480:
		switch (mode) {
		case TYPEC_ACCESSORY_NONE: /* USB mode */
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_SEL, 0x18);
			/*audio_v modify for mos test*/
			if (mmax_priv->is_mostest) {
				regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x9f);
			} else {
				regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x98);
			}
			/*audio_v modify for mos test end*/
			break;
		case TYPEC_ACCESSORY_AUDIO:
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_SEL, 0x00);
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x87);
			break;
		default:
			break;
		}

		for (i = FSA4480_REG_DEVID; i <= FSA4480_REG_CS_SET; i++) {
			regmap_read(mmax_priv->regmap, i, &val);
			dev_dbg(mmax_priv->dev, "%s: reg[0x%x]: 0x%x\n",
					__func__, i, val);
		}

		break;

	default:
		break;
	}

	mutex_unlock(&mmax_priv->usbc_switch_lock);
}

#if 0
static void max20328_usbc_update_settings(struct max20328_priv *mmax_priv,
		u32 switch_control, u32 switch_enable)
{
	if (!mmax_priv->regmap) {
		dev_err(mmax_priv->dev, "%s: regmap invalid\n", __func__);
		return;
	}

	regmap_write(mmax_priv->regmap, MAX20328_SWITCH_SETTINGS, 0x80);
	regmap_write(mmax_priv->regmap, MAX20328_SWITCH_CONTROL, switch_control);
	/* MAX20328 chip hardware requirement */
	usleep_range(50, 55);
	regmap_write(mmax_priv->regmap, MAX20328_SWITCH_SETTINGS, switch_enable);
}
#endif

int max20328_extern_usbc_event_changed(int acc)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	struct device *dev;

	if (!mmax_priv)
		return -EINVAL;

	dev = mmax_priv->dev;
	if (!dev)
		return -EINVAL;

	dev_dbg(dev, "%s: USB change event received, supply mode %d, usbc mode %ld, expected %d\n",
			__func__, acc, mmax_priv->usbc_mode.counter,
			TYPEC_ACCESSORY_AUDIO);

	switch (acc) {
	case TYPEC_ACCESSORY_AUDIO:
	case TYPEC_ACCESSORY_NONE:
		if (atomic_read(&(mmax_priv->usbc_mode)) == acc)
			break; /* filter notifications received before */
		atomic_set(&(mmax_priv->usbc_mode), acc);

		dev_dbg(dev, "%s: queueing usbc_analog_work\n",
			__func__);
		pm_stay_awake(mmax_priv->dev);
		schedule_work(&mmax_priv->usbc_analog_work);
		break;
	default:
		break;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(max20328_extern_usbc_event_changed);

static int max20328_usbc_event_changed(struct notifier_block *nb,
				      unsigned long evt, void *ptr)
{
	struct max20328_priv *mmax_priv =
			container_of(nb, struct max20328_priv, ucsi_nb);
	struct device *dev;
	enum typec_accessory acc = ((struct ucsi_glink_constat_info *)ptr)->acc;

	if (!mmax_priv)
		return -EINVAL;

	dev = mmax_priv->dev;
	if (!dev)
		return -EINVAL;

	dev_dbg(dev, "%s: USB change event received, supply mode %d, usbc mode %ld, expected %d\n",
			__func__, acc, mmax_priv->usbc_mode.counter,
			TYPEC_ACCESSORY_AUDIO);

	switch (acc) {
	case TYPEC_ACCESSORY_AUDIO:
	case TYPEC_ACCESSORY_NONE:
		if (atomic_read(&(mmax_priv->usbc_mode)) == acc)
			break; /* filter notifications received before */
		atomic_set(&(mmax_priv->usbc_mode), acc);

		dev_dbg(dev, "%s: queueing usbc_analog_work\n",
			__func__);
		pm_stay_awake(mmax_priv->dev);
		schedule_work(&mmax_priv->usbc_analog_work);
		break;
	default:
		break;
	}

	return 0;
}

/*
 * fsa4480_reg_notifier - register notifier block with mmax driver
 *
 * @nb - notifier block of max20328
 * @node - phandle node to max20328 device
 *
 * Returns 0 on success, or error code
 */
int max20328_reg_notifier(struct notifier_block *nb,
			 struct device_node *node)
{
	int rc = 0;
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct max20328_priv *mmax_priv;

	if (!client)
		return -EINVAL;

	mmax_priv = (struct max20328_priv *)i2c_get_clientdata(client);
	if (!mmax_priv)
		return -EINVAL;

	rc = blocking_notifier_chain_register
				(&mmax_priv->max20328_notifier, nb);
	if (rc)
		return rc;

	/*
	 * as part of the init sequence check if there is a connected
	 * USB C analog adapter
	 */
	dev_dbg(mmax_priv->dev, "%s: verify if USB adapter is already inserted\n",
		__func__);
	rc = max20328_usbc_analog_setup_switches(mmax_priv);

	return rc;
}
EXPORT_SYMBOL(max20328_reg_notifier);

/*
 * fsa4480_unreg_notifier - unregister notifier block with mmax driver
 *
 * @nb - notifier block of max20328
 * @node - phandle node to max20328 device
 *
 * Returns 0 on pass, or error code
 */
int max20328_unreg_notifier(struct notifier_block *nb,
			     struct device_node *node)
{
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct max20328_priv *mmax_priv;

	if (!client)
		return -EINVAL;

	mmax_priv = (struct max20328_priv *)i2c_get_clientdata(client);
	if (!mmax_priv)
		return -EINVAL;

	atomic_set(&(mmax_priv->usbc_mode), 0);
	max20328_usbc_set_switch_mode(mmax_priv, TYPEC_ACCESSORY_NONE);
	return blocking_notifier_chain_unregister
					(&mmax_priv->max20328_notifier, nb);
}
EXPORT_SYMBOL(max20328_unreg_notifier);

#if 0
static int max20328_validate_display_port_settings(struct max20328_priv *mmax_priv)
{
	u32 switch_status = 0;

	regmap_read(mmax_priv->regmap, MAX20328_SWITCH_STATUS1, &switch_status);

	if ((switch_status != 0x23) && (switch_status != 0x1C)) {
		pr_err("AUX SBU1/2 switch status is invalid = %u\n",
				switch_status);
		return -EIO;
	}

	return 0;
}
#endif

static int __max20328_switch_mode_event(struct max20328_priv *mmax_priv,
	enum fsa_function event)
{
	int i;
	bool is_audio_adapter;
	unsigned int val = 0;
	unsigned int val_0x0D = 0, val_0x0E = 0;

	if (!mmax_priv) {
		pr_err("%s: max20328 mmax_priv is null\n", __func__);
		return -EINVAL;
	}
	if (!mmax_priv->regmap) {
		pr_err("%s: max20328 regmap is null\n", __func__);
		return -EINVAL;
	}

	max20328_usbc_wait_event_wake(mmax_priv);

	mutex_lock(&mmax_priv->usbc_switch_lock);

	if (atomic_read(&(mmax_priv->usbc_mode)) == TYPEC_ACCESSORY_AUDIO)
		is_audio_adapter = true;
	else
		is_audio_adapter = false;

	dev_dbg(mmax_priv->dev, "%s: max20328: event: %d, mode: %d, is_audio_adapter: %d.\n",
			__func__, event, atomic_read(&(mmax_priv->usbc_mode)), is_audio_adapter);

	switch (mmax_priv->usbc_ic_mode) {
	case MAX20328:
		switch (event) {
		case FSA_MIC_GND_SWAP:
			if (is_audio_adapter) {
				regmap_read(mmax_priv->regmap, 0x0D, &val);
				if ((val & 0x0f) == 0x07) {
					val = 0x03 | (val & 0xf0);
					regmap_write(mmax_priv->regmap, 0x0D, val);
					regmap_write(mmax_priv->regmap, 0x0E, 0x10);
				} else {
					val = 0x07 | (val & 0xf0);
					regmap_write(mmax_priv->regmap, 0x0D, val);
					regmap_write(mmax_priv->regmap, 0x0E, 0x40);
				}
			}
			break;
		case FSA_USBC_AUIDO_HP_ON:
			if (is_audio_adapter) {
				regmap_read(mmax_priv->regmap, 0x0D, &val);
				if ((val & 0x0f) == 0x03)
					regmap_write(mmax_priv->regmap, 0x0E, 0x10);
				else
					regmap_write(mmax_priv->regmap, 0x0E, 0x40);
				usleep_range(2000, 2020);
				val = 0xa0 | (val & 0x0f);
				regmap_write(mmax_priv->regmap, 0x0D, val);
			}
			break;
		case FSA_USBC_AUIDO_HP_OFF:
			if (is_audio_adapter) {
				regmap_read(mmax_priv->regmap, 0x0D, &val);
				val = val & 0x0f;
				regmap_write(mmax_priv->regmap, 0x0D, val);
				usleep_range(2000, 2020);
				regmap_write(mmax_priv->regmap, 0x0E, 0x00);
			}
			break;
		case FSA_USBC_ORIENTATION_CC2:
			regmap_write(mmax_priv->regmap, 0x06, 0x34);
			break;
		case FSA_USBC_DISPLAYPORT_DISCONNECTED:
			regmap_write(mmax_priv->regmap, 0x06, 0x14);
			break;
		case FSA_USBC_FAST_CHARGE_SELECT:
			if (!is_audio_adapter)
				regmap_write(mmax_priv->regmap, 0x0D, 0x10);
			break;
		case FSA_USBC_FAST_CHARGE_EXIT:
			if (!is_audio_adapter) {
				/*audio_v modify for mos test*/
				if (mmax_priv->is_mostest) {
					regmap_write(mmax_priv->regmap, 0x0D, 0x47);
				} else {
					regmap_write(mmax_priv->regmap, 0x0D, 0x40);
				}
				/*audio_v modify for mos test*/
			}
			break;
		case FSA_USBC_SWITCH_ENABLE:
			if (!is_audio_adapter)
				max20328_usbc_switch_enable(mmax_priv, true);
			break;
		case FSA_USBC_SWITCH_DISABLE:
			if (!is_audio_adapter)
				max20328_usbc_switch_enable(mmax_priv, false);
			break;
		default:
			break;
		}
		regmap_read(mmax_priv->regmap, 0x0D, &val_0x0D);
		regmap_read(mmax_priv->regmap, 0x0E, &val_0x0E);
		dev_dbg(mmax_priv->dev, "%s: max20328: val_0x0D = 0x%x, val_0x0E = 0x%x, is_audio_adapter %d\n",
				__func__, val_0x0D, val_0x0E, is_audio_adapter);
		break;

	case FSA4480:
		switch (event) {
		case FSA_MIC_GND_SWAP:
			if (is_audio_adapter) {
				if (mmax_priv->supply_nonindepend) {
					max20328_usbc_enable_power(mmax_priv, true);
				}
				if (mmax_priv->vdda307) {
					max20328_sbu_sel_enable_power(mmax_priv, true);
				}
				if (gpio_is_valid(mmax_priv->sbu_sel)) {
					dev_dbg(mmax_priv->dev, "%s: set sbu-sel gpio high\n",
							__func__);
					gpio_direction_output(mmax_priv->sbu_sel, 1);
				}
				regmap_read(mmax_priv->regmap, FSA4480_REG_SW_SEL, &val);
				if ((val & 0x07) == 0x07) {
					regmap_write(mmax_priv->regmap, FSA4480_REG_SW_SEL, 0x00);
					/* regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x87); */
				} else {
					regmap_write(mmax_priv->regmap, FSA4480_REG_SW_SEL, 0x07);
					/* regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x87); */
				}
			}
			break;
		case FSA_USBC_AUIDO_HP_ON:
			if (is_audio_adapter) {
				regmap_read(mmax_priv->regmap, FSA4480_REG_SW_EN, &val);
				val = 0x04 | (val & 0x9f);
				regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, val);
				usleep_range(2000, 2020);
				val = 0x18 | (val & 0x9f);
				regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, val);
			}
			break;
		case FSA_USBC_AUIDO_HP_OFF:
			if (is_audio_adapter) {
				regmap_read(mmax_priv->regmap, FSA4480_REG_SW_EN, &val);
				val = val & 0xe7;
				regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, val);
				usleep_range(2000, 2020);
				val = val & 0xe3;
				regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, val);
			}
			break;
		case FSA_USBC_ORIENTATION_CC1:
			if (mmax_priv->supply_nonindepend) {
				max20328_usbc_enable_power(mmax_priv, true);
			}
			if (mmax_priv->vdda307) {
				max20328_sbu_sel_enable_power(mmax_priv, true);
			}
			if (gpio_is_valid(mmax_priv->sbu_sel)) {
				dev_dbg(mmax_priv->dev, "%s: set sbu-sel gpio high\n",
						__func__);
				gpio_direction_output(mmax_priv->sbu_sel, 1);
			}
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x80);
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_SEL, 0x18);
			/* FSA4480 chip hardware requirement */
			usleep_range(50, 55);
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0xF8);
			break;
		case FSA_USBC_ORIENTATION_CC2:
			if (mmax_priv->supply_nonindepend) {
				max20328_usbc_enable_power(mmax_priv, true);
			}
			if (mmax_priv->vdda307) {
				max20328_sbu_sel_enable_power(mmax_priv, true);
			}
			if (gpio_is_valid(mmax_priv->sbu_sel)) {
				dev_dbg(mmax_priv->dev, "%s: set sbu-sel gpio high\n",
						__func__);
				gpio_direction_output(mmax_priv->sbu_sel, 1);
			}
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x80);
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_SEL, 0x78);
			/* FSA4480 chip hardware requirement */
			usleep_range(50, 55);
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0xF8);
			break;
		case FSA_USBC_DISPLAYPORT_DISCONNECTED:
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_SEL, 0x18);
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x98);
			break;
		case FSA_USBC_FAST_CHARGE_SELECT:
			if (!is_audio_adapter) {
				regmap_write(mmax_priv->regmap, FSA4480_REG_SW_SEL, 0x18);
				regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x98);
			}
			break;
		case FSA_USBC_FAST_CHARGE_EXIT:
			if (!is_audio_adapter) {
				regmap_write(mmax_priv->regmap, FSA4480_REG_SW_SEL, 0x18);
				/*audio_v modify for mos test*/
				if (mmax_priv->is_mostest) {
					regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x9f);
				} else {
					regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x98);
				}
				/*audio_v modify for mos test*/
			}
			break;
		/*
		case FSA_USBC_SWITCH_ENABLE:
			if (!is_audio_adapter) {
				regmap_write(mmax_priv->regmap, FSA4480_REG_SW_SEL, 0x18);
				regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x98);
			}
			break;
		case FSA_USBC_SWITCH_DISABLE:
			if (!is_audio_adapter) {
				regmap_write(mmax_priv->regmap, FSA4480_REG_SW_SEL, 0x18);
				regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x98);
			}
			break;
		*/
		case FSA_USBC_SWITCH_SBU_DIRECT_CONNECT:
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_SEL, 0x18);
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0xF8);
			break;
		case FSA_USBC_SWITCH_SBU_FLIP_CONNECT:
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_SEL, 0x78);
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0xF8);
			break;
		case FSA_USBC_SWITCH_SBU_HIZ:
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_SEL, 0x18);
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x98);
			break;
		default:
			break;
		}
		regmap_read(mmax_priv->regmap, FSA4480_REG_SW_SEL, &val_0x0D);
		regmap_read(mmax_priv->regmap, FSA4480_REG_SW_EN, &val_0x0E);
		dev_dbg(mmax_priv->dev, "%s: fsa4480: val_sel = 0x%x,val_en = 0x%x is_audio_adapter %d\n",
				__func__, val_0x0D, val_0x0E, is_audio_adapter);

		for (i = FSA4480_REG_DEVID; i <= FSA4480_REG_CS_SET; i++) {
			regmap_read(mmax_priv->regmap, i, &val);
			dev_dbg(mmax_priv->dev, "%s: reg[0x%x]: 0x%x\n",
					__func__, i, val);
		}
		break;

	default:
		break;
	}

	mutex_unlock(&mmax_priv->usbc_switch_lock);

	return 0;
}


/*
 * max20328_switch_event - configure MMAX switch position based on event
 *
 * @node - phandle node to max20328 device
 * @event - fsa_function enum
 *
 * Returns int on whether the switch happened or not
 */
int max20328_switch_event(struct device_node *node,
			 enum fsa_function event)
{
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct max20328_priv *mmax_priv;

	if (!client)
		return -EINVAL;

	mmax_priv = (struct max20328_priv *)i2c_get_clientdata(client);
	if (!mmax_priv)
		return -EINVAL;
	if (!mmax_priv->regmap)
		return -EINVAL;

	__max20328_switch_mode_event(mmax_priv, event);

	return 0;
}
EXPORT_SYMBOL(max20328_switch_event);

int max20328_switch_mode_event(enum fsa_function event)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;

	if (!mmax_priv) {
		pr_err("%s: max20328 mmax_priv is null\n", __func__);
		return -EINVAL;
	}

	if (!mmax_priv->regmap) {
		pr_err("%s: max20328 regmap is null\n", __func__);
		return -EINVAL;
	}

	__max20328_switch_mode_event(mmax_priv, event);

	return 0;
}
EXPORT_SYMBOL(max20328_switch_mode_event);


/*  Resistance detect Function :
  *    @range : resistance detection range setting (1: 10K to 2560K; 0: 1K to 256K).
  *    @pin_sel : resistance PIN selection(CC_IN/DP/DN/SBU1/SBU2)
  *    @interval : resistance detection interval
  */
int fsa4480_RES_detect(int range, int pin_sel, int interval)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	int res_kohm = 0;
	int val = 0;

	if (!mmax_priv) {
		pr_err("%s: mmax_priv is null\n", __func__);
		return INT_MAX;
	}

	pr_err("%s: setting range=%d, pin_sel=%d, interval=%d.", __func__, range, pin_sel, interval);

	regmap_write(mmax_priv->regmap, FSA4480_REG_RES_DET_SET, pin_sel);
	regmap_write(mmax_priv->regmap, FSA4480_REG_RES_DET_IVAL, interval);

	regmap_read(mmax_priv->regmap, FSA4480_REG_FUNC_EN, &val);
	val = val & ~(0x22);
	val = val | (((range << 5) | 0x02) & 0x22);
	regmap_write(mmax_priv->regmap, FSA4480_REG_FUNC_EN, val);

	msleep(50);

	regmap_read(mmax_priv->regmap, FSA4480_REG_RES_DET_VAL, &res_kohm);

	return res_kohm * ((range == 1) ? 10 : 1);
}

/* return resistance in KOhm(1k~2550K). */
int get_usb_connecter_pin_resistance_value(int pin_sel)
{
	int res_kohm = 0;

	res_kohm = fsa4480_RES_detect(0, pin_sel, 0);	/*SBU1 = 3*/

	pr_err("%s: %s=%d kOhm", __func__, RES_detection_pin[pin_sel], res_kohm);
	return res_kohm;
}
EXPORT_SYMBOL(get_usb_connecter_pin_resistance_value);


static int max20328_usbc_analog_setup_switches(struct max20328_priv *mmax_priv)
{
	int rc = 0;
	int mode;
	struct device *dev;
	unsigned int reg_06 = 0;
	int ret = 0;
	unsigned int val_sel = 0, val_en = 0;

	if (!mmax_priv) {
		pr_err("%s: mmax container invalid\n", __func__);
		goto err;
	}

	dev = mmax_priv->dev;
	if (!dev) {
		pr_err("%s: mmax dev invalid\n", __func__);
		goto err;
	}

	if (!mmax_priv->regmap) {
		dev_err(mmax_priv->dev, "%s: regmap invalid\n", __func__);
		goto err;
	}

	max20328_usbc_wait_event_wake(mmax_priv);

	mutex_lock(&mmax_priv->usbc_switch_lock);
	switch (mmax_priv->usbc_ic_mode) {
	case MAX20328:
		ret = regmap_write(mmax_priv->regmap, 0x06, 0x13);
		if (ret)
			dev_err(mmax_priv->dev, "%s: failed %d\n", __func__, ret);
		usleep_range(5 * 1000, 5 * 1000);
		regmap_read(mmax_priv->regmap, 0x06, &reg_06);
		dev_dbg(mmax_priv->dev, "%s: reg_0x06 (0x%x)\n",
				__func__, reg_06);
		break;
	case FSA4480:
		regmap_read(mmax_priv->regmap, FSA4480_REG_SW_SEL, &val_sel);
		regmap_read(mmax_priv->regmap, FSA4480_REG_SW_EN, &val_en);
		dev_dbg(mmax_priv->dev, "%s: fsa4480: val_sel = 0x%x,val_en = 0x%x\n",
				__func__, val_sel, val_en);
		break;
	default:
		break;
	}
	mutex_unlock(&mmax_priv->usbc_switch_lock);

	/* get latest mode again within locked context */
	mode = atomic_read(&(mmax_priv->usbc_mode));

	dev_dbg(dev, "%s: setting GPIOs active = %d\n",
		__func__, mode != TYPEC_ACCESSORY_NONE);

	switch (mode) {
	/* add all modes FSA should notify for in here */
	case TYPEC_ACCESSORY_AUDIO:
		if (mmax_priv->supply_nonindepend) {
			max20328_usbc_enable_power(mmax_priv, true);
		}
		if (mmax_priv->vdda307) {
			max20328_sbu_sel_enable_power(mmax_priv, true);
		}
		if (gpio_is_valid(mmax_priv->sbu_sel)) {
			dev_dbg(mmax_priv->dev, "%s: set sbu-sel gpio high\n",
					__func__);
			gpio_direction_output(mmax_priv->sbu_sel, 1);
		}
		/* activate switches */
		max20328_usbc_set_switch_mode(mmax_priv, mode);

		/* notify call chain on event */
		blocking_notifier_call_chain(&mmax_priv->max20328_notifier,
					     mode, NULL);
		break;
	case TYPEC_ACCESSORY_NONE:
		/* notify call chain on event */
		blocking_notifier_call_chain(&mmax_priv->max20328_notifier,
				TYPEC_ACCESSORY_NONE, NULL);

		/* deactivate switches */
		max20328_usbc_set_switch_mode(mmax_priv, TYPEC_ACCESSORY_NONE);
		if (mmax_priv->vdda307) {
			max20328_sbu_sel_enable_power(mmax_priv, false);
		}
		if (mmax_priv->supply_nonindepend) {
			max20328_usbc_enable_power(mmax_priv, false);
		}
		if (gpio_is_valid(mmax_priv->sbu_sel)) {
			dev_dbg(mmax_priv->dev, "%s: set sbu-sel gpio low\n",
					__func__);
			gpio_direction_output(mmax_priv->sbu_sel, 0);
		}
		break;
	default:
		/* ignore other usb connection modes */
		break;
	}

err:

	return rc;
}

static void max20328_usbc_analog_work_fn(struct work_struct *work)
{
	struct max20328_priv *mmax_priv =
		container_of(work, struct max20328_priv, usbc_analog_work);

	if (!mmax_priv) {
		pr_err("%s: max20328 container invalid\n", __func__);
		return;
	}
	max20328_usbc_analog_setup_switches(mmax_priv);
	pm_relax(mmax_priv->dev);
}

static void max20328_update_reg_defaults(struct max20328_priv *mmax_priv)
{
	u8 i;

	if (!mmax_priv)
		return;

	switch (mmax_priv->usbc_ic_mode) {
	case MAX20328:
		for (i = 0; i < ARRAY_SIZE(mmax_reg_i2c_defaults); i++)
			regmap_write(mmax_priv->regmap, mmax_reg_i2c_defaults[i].reg,
					   mmax_reg_i2c_defaults[i].val);
		/*audio_v add for mos test*/
		if (mmax_priv->is_mostest) {
			regmap_write(mmax_priv->regmap, 0x0d, 0x47);
			regmap_write(mmax_priv->regmap, 0x0e, 0x40);
		}
		/*audio_v add for mos test end*/
		break;
	case FSA4480:
		/* i2c reset */
		regmap_write(mmax_priv->regmap, FSA4480_REG_I2C_RESET, 0x01);
		for (i = 0; i < ARRAY_SIZE(fsa_reg_i2c_defaults); i++)
			regmap_write(mmax_priv->regmap, fsa_reg_i2c_defaults[i].reg,
					   fsa_reg_i2c_defaults[i].val);
		/*audio_v add for mos test*/
		if (mmax_priv->is_mostest) {
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x9f);
		}
		/*audio_v add for mos test end*/
		break;
	default:
		break;
	}
}

static void max20328_usbc_removed_work_fn(struct work_struct *work)
{
#if 0
	union power_supply_propval pval = {0, };
#endif
	struct max20328_priv *mmax_priv =
		container_of(work, struct max20328_priv, usbc_removed_work);

	if (!mmax_priv) {
		pr_err("%s: mmax container invalid\n", __func__);
		return;
	}

	pr_info("%s: enter\n", __func__);
	max20328_drp_wait_event_wake(mmax_priv);
	msleep(300);

	mutex_lock(&mmax_priv->usbc_switch_lock);

#ifdef VUCSI_SET_CCOM
	if (!mmax_priv->current_plug) {
		if (mmax_priv->vucsi_vote && !mmax_priv->vucsi_vote(VUCSI_VOTER_AUDIO, VUCSI_CC_SINK)) {
			pr_info("%s: vote VUCSI_VOTER_AUDIO VUCSI_CC_SINK successful\n", __func__);
		}
	}
#endif

	mutex_unlock(&mmax_priv->usbc_switch_lock);
}

static irqreturn_t max20328_usbc_irq_handler(int irq, void *data)
{
	struct max20328_priv *mmax_priv = data;
	union power_supply_propval pval = {0, };
	int try_times = 3, read_times, i;
	bool new_type;

	pr_info("%s: enter\n", __func__);

	if (!mmax_priv)
		return -ENOMEM;

	disable_irq_nosync(mmax_priv->dev_gpio_irq);
	mutex_lock(&mmax_priv->usbc_switch_lock);

	if (mmax_priv->current_plug)
		read_times = 3000;
	else
		read_times = 400;

	mmax_priv->gpio_detection_type = !gpio_get_value_cansleep(mmax_priv->dev_gpio);

	for (; try_times > 0; try_times--) {
		for (i = read_times; i > 0; i--) {
			new_type = !gpio_get_value_cansleep(mmax_priv->dev_gpio);
			if (mmax_priv->gpio_detection_type != new_type) {
				mmax_priv->gpio_detection_type = new_type;
				break;
			}
		if (i == (read_times / 4))
			usleep_range(5 * 1000, 5 * 1000);
		else if (i == (read_times / 3))
			usleep_range(10 * 1000, 10 * 1000);
		else if (i == (read_times / 2))
			usleep_range(15 * 1000, 15 * 1000);
		}
		pr_info("%s: try_times %d, i %d\n",
			__func__, try_times, i);
		if (i <= 0)
			break;

		usleep_range(30 * 1000, 30 * 1000);
	}

	if (try_times <= 0) {
		pr_info("%s: detect usb failed, keep current state %d\n",
				__func__, mmax_priv->current_plug);
		goto leave;
	} else {
		pr_info("%s: detect usb success, detection_type %d\n",
			__func__, mmax_priv->gpio_detection_type);
		mmax_priv->current_plug = mmax_priv->gpio_detection_type;

		if (mmax_priv->is_unuseirq) {
			pr_info("%s: is_unuseirq, to ignore\n", __func__);
			goto leave;
		}

		if (mmax_priv->gpio_detection_type) {
			memset(&pval, 0, sizeof(pval));
			max20328_drp_wait_event_wake(mmax_priv);
#ifdef VUCSI_SET_CCOM
			if (mmax_priv->vucsi_vote && !mmax_priv->vucsi_vote(VUCSI_VOTER_AUDIO, VUCSI_CC_DRP)) {
				pr_info("%s: vote VUCSI_VOTER_AUDIO VUCSI_CC_DRP successful\n", __func__);
			}
#endif
		} else {
			pr_info("%s: queueing usbc_removed_work\n", __func__);
			schedule_work(&mmax_priv->usbc_removed_work);
		}
	}

leave:

	if (mmax_priv->current_plug)
		irq_set_irq_type(mmax_priv->dev_gpio_irq, (IRQF_ONESHOT | IRQF_TRIGGER_HIGH));
	else
		irq_set_irq_type(mmax_priv->dev_gpio_irq, (IRQF_ONESHOT | IRQF_TRIGGER_LOW));

	mutex_unlock(&mmax_priv->usbc_switch_lock);
	enable_irq(mmax_priv->dev_gpio_irq);

	pr_info("%s: leave\n", __func__);
	return IRQ_HANDLED;
}

static void max20328_irq_handler_work_fn(struct work_struct *work)
{
	struct max20328_priv *mmax_priv =
		container_of(work, struct max20328_priv, max20328_irq_handler_work);
	unsigned int data = 0, i;

	if (!mmax_priv) {
		pr_err("%s: mmax container invalid\n", __func__);
		return;
	}

	if (!mmax_priv->regmap) {
		pr_err("%s: mmax regmap is null\n", __func__);
		return;
	}

	max20328_usbc_wait_event_wake(mmax_priv);

	mutex_lock(&mmax_priv->usbc_switch_lock);
	switch (mmax_priv->usbc_ic_mode) {
	case MAX20328:
		for (i = 0; i < sizeof(max20328_regs); i++) {
			regmap_read(mmax_priv->regmap, max20328_regs[i], &data);
			pr_info("%s: reg[0x%02x]: 0x%02x\n",
				__func__, max20328_regs[i], data);
		}
		break;
	case FSA4480:
		for (i = 0; i < sizeof(fsa4480_regs); i++) {
			regmap_read(mmax_priv->regmap, fsa4480_regs[i], &data);
			pr_info("%s: reg[0x%02x]: 0x%02x\n",
				__func__, fsa4480_regs[i], data);
		}
		break;
	default:
		break;
	}
	mutex_unlock(&mmax_priv->usbc_switch_lock);

	msleep(500);
	enable_irq(mmax_priv->mmax_int_irq);
}

static irqreturn_t max20328_irq_handler(int irq, void *data)
{
	struct max20328_priv *mmax_priv = data;

	pr_info("%s: enter\n", __func__);

	if (!mmax_priv)
		return -ENOMEM;

	disable_irq_nosync(mmax_priv->mmax_int_irq);
	schedule_work(&mmax_priv->max20328_irq_handler_work);

	pr_info("%s: leave\n", __func__);
	return IRQ_HANDLED;
}

static int max20328_usbc_irq_init(struct max20328_priv *mmax_priv)
{
	int ret;

	mmax_priv->dev_gpio = of_get_named_gpio(mmax_priv->dev->of_node,
									"vivo,usbc-irq-gpio", 0);
	if (!gpio_is_valid(mmax_priv->dev_gpio))
		mmax_priv->dev_gpio = -EINVAL;

	if (gpio_is_valid(mmax_priv->dev_gpio)) {
		ret = gpio_request(mmax_priv->dev_gpio, "usbc irq");
		if (ret < 0) {
			dev_err(mmax_priv->dev, "%s: gpio %d request error %d\n",
			       __func__, mmax_priv->dev_gpio, ret);
		}
		ret = gpio_direction_input(mmax_priv->dev_gpio);
		if (ret < 0)
			dev_err(mmax_priv->dev, "%s: set gpio %d input error %d\n", __func__,
					mmax_priv->dev_gpio, ret);
		mmax_priv->dev_gpio_irq = gpio_to_irq(mmax_priv->dev_gpio);
		dev_dbg(mmax_priv->dev, "%s: gpio_irq = %d\n", __func__,
				mmax_priv->dev_gpio_irq);
	} else {
		dev_dbg(mmax_priv->dev, "%s: gpio irq may not to be supported, return value %d\n",
				__func__, mmax_priv->dev_gpio);
		return 0;
	}

	if (mmax_priv->dev_gpio_irq) {
		ret = request_threaded_irq(mmax_priv->dev_gpio_irq,	NULL,
									max20328_usbc_irq_handler,
									(IRQF_ONESHOT | IRQF_TRIGGER_LOW),
									"usbc irq", mmax_priv);
		if (ret != 0) {
			dev_err(mmax_priv->dev, "%s: Failed to request IRQ: %d\n", __func__, ret);
			free_irq(mmax_priv->dev_gpio_irq, mmax_priv);
			gpio_free(mmax_priv->dev_gpio);
			return ret;
		}
	}

	ret = enable_irq_wake(mmax_priv->dev_gpio_irq);
	if (ret)
		dev_err(mmax_priv->dev, "%s: Failed to enable wake up irq %d\n",
			  __func__, mmax_priv->dev_gpio_irq);

	return 0;
}

static int max20328_irq_init(struct max20328_priv *mmax_priv)
{
	int ret;

	mmax_priv->mmax_int = of_get_named_gpio(mmax_priv->dev->of_node,
									"max20328-irq-gpio", 0);
	if (!gpio_is_valid(mmax_priv->mmax_int))
		mmax_priv->mmax_int = -EINVAL;

	if (gpio_is_valid(mmax_priv->mmax_int)) {
		ret = gpio_request(mmax_priv->mmax_int, "max20328 irq");
		if (ret < 0) {
			dev_err(mmax_priv->dev, "%s: gpio %d request error %d\n",
			       __func__, mmax_priv->mmax_int, ret);
		}
		ret = gpio_direction_input(mmax_priv->mmax_int);
		if (ret < 0)
			dev_err(mmax_priv->dev, "%s: set gpio %d input error %d\n", __func__,
					mmax_priv->mmax_int, ret);
		mmax_priv->mmax_int_irq = gpio_to_irq(mmax_priv->mmax_int);
		dev_dbg(mmax_priv->dev, "%s: gpio_irq = %d\n", __func__,
				mmax_priv->mmax_int_irq);
	} else {
		dev_dbg(mmax_priv->dev, "%s: gpio irq may not to be supported, return value %d\n",
				__func__, mmax_priv->mmax_int);
		return 0;
	}

	if (mmax_priv->mmax_int_irq) {
		ret = request_threaded_irq(mmax_priv->mmax_int_irq,	NULL,
									max20328_irq_handler,
									(IRQF_ONESHOT | IRQF_TRIGGER_LOW),
									"max20328 irq", mmax_priv);
		if (ret != 0) {
			dev_err(mmax_priv->dev, "%s: Failed to request IRQ: %d\n", __func__, ret);
			free_irq(mmax_priv->mmax_int_irq, mmax_priv);
			gpio_free(mmax_priv->mmax_int);
			return ret;
		}
	}

	ret = enable_irq_wake(mmax_priv->mmax_int_irq);
	if (ret)
		dev_err(mmax_priv->dev, "%s: Failed to enable wake up irq %d\n",
			  __func__, mmax_priv->mmax_int_irq);

	return 0;
}

int get_usbc_mg_status(void)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	unsigned int val_0x0D = 0, val_0x0E = 0, val = 0;
	int ret = -1;

	if (!mmax_priv) {
		pr_err("%s: mmax container invalid\n", __func__);
		return ret;
	}

	if (!mmax_priv->regmap) {
		pr_err("%s: max20328 regmap is null\n", __func__);
		return ret;
	}

	if (atomic_read(&(mmax_priv->usbc_mode)) != TYPEC_ACCESSORY_AUDIO) {
		pr_err("%s: peripheral is not audio adapter, %d, return!\n",
			__func__, atomic_read(&(mmax_priv->usbc_mode)));
		return ret;
	}

	mutex_lock(&mmax_priv->usbc_switch_lock);
	switch (mmax_priv->usbc_ic_mode) {
	case MAX20328:
		regmap_read(mmax_priv->regmap, MAX20328_SW_DEFLT1, &val_0x0D);
		regmap_read(mmax_priv->regmap, MAX20328_SW_DEFLT2, &val_0x0E);
		pr_info("%s: max20328: val_0x0D: 0x%x, val_0x0E: 0x%x\n",
				__func__, val_0x0D, val_0x0E);
		if (val_0x0E & 0x40) {
			ret = 1;
		} else if (val_0x0E & 0x10) {
			ret = 2;
		}
		break;
	case FSA4480:
		regmap_read(mmax_priv->regmap, FSA4480_REG_SW_SEL, &val);
		pr_info("%s: fsa4480: val_sel: 0x%x\n", __func__, val);
		if ((val & 0x07) == 0x07) {
			ret = 1;
		} else if ((val & 0x07) == 0x00) {
			ret = 2;
		}
		break;
	default:
		break;
	}
	mutex_unlock(&mmax_priv->usbc_switch_lock);

	return ret;
}
EXPORT_SYMBOL(get_usbc_mg_status);

int get_usbc_peripheral_status(void)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	int ret = -1;

	if (!mmax_priv)
		return ret;

	mutex_lock(&mmax_priv->usbc_switch_lock);
	ret = mmax_priv->current_plug;
	pr_info("%s: gpio_current_plug: %d.\n",
		__func__, ret);
	mutex_unlock(&mmax_priv->usbc_switch_lock);

	return ret;
}
EXPORT_SYMBOL(get_usbc_peripheral_status);

/*
 * max20328_notify_mos
 * audio_v : archer add for mos test
*/
int max20328_notify_mos(int state)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	if (!mmax_priv)
		return -EINVAL;
	if (!mmax_priv->regmap)
		return -EINVAL;

	mmax_priv->is_mostest = state;

	max20328_update_reg_defaults(mmax_priv);

	return 0;
}
EXPORT_SYMBOL(max20328_notify_mos);


#ifdef SYS_NODE
static ssize_t max20328_reg_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *ubuf, size_t count)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	unsigned int kbuf[2];
	int ret = 0;
	int reg_max = (mmax_priv->usbc_ic_mode == MAX20328) ? sizeof(max20328_regs) : sizeof(fsa4480_regs);

	ret = sscanf(ubuf, "%x %x", &kbuf[0], &kbuf[1]);
	if (!ret) {
		pr_err("%s: sscanf fail.\n", __func__);
	}

	pr_info("%s: kbuf[0]: %x, kbuf[1]: %x cnt: %d\n",
		__func__, kbuf[0], kbuf[1], (int)count);

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: Invalid data.\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: Invalid regmap.\n ", __func__);
		return -EFAULT;
	}

	if (1 || (kbuf[0] <= reg_max)) {
		mutex_lock(&mmax_priv->usbc_switch_lock);
		regmap_write(mmax_priv->regmap, kbuf[0], kbuf[1]);
		mutex_unlock(&mmax_priv->usbc_switch_lock);
	} else {
		pr_err("%s: reg addr 0x%x out of range.\n", __func__, kbuf[0]);
	}

	return count;
}

static ssize_t max20328_reg_show(struct kobject *kobj, struct kobj_attribute *attr, char *ubuf)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	const int size = 1024;
	int n = 0, i;
	unsigned int data;
	u8 *regs = max20328_regs;
	u8 regs_size = sizeof(max20328_regs);

	switch (mmax_priv->usbc_ic_mode) {
	case MAX20328:
		regs = max20328_regs;
		regs_size = sizeof(max20328_regs);
		break;
	case FSA4480:
		regs = fsa4480_regs;
		regs_size = sizeof(fsa4480_regs);
		break;
	default:
		break;
	}

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: Invalid data.\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: Invalid regmap.\n ", __func__);
		return -EFAULT;
	}

	mutex_lock(&mmax_priv->usbc_switch_lock);
	for (i = 0; i < regs_size; i++) {
		regmap_read(mmax_priv->regmap, regs[i], &data);
		n += scnprintf(ubuf+n, size-n, "0x%02x: 0x%02x\n", regs[i], data);
		pr_info("%s: reg[0x%02x]: 0x%02x.\n", __func__, regs[i], data);
	}
	mutex_unlock(&mmax_priv->usbc_switch_lock);

	ubuf[n] = 0;

	return n;
}

static ssize_t max20328_i2c_show(struct kobject *kobj, struct kobj_attribute *attr, char *ubuf)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	struct i2c_client *i2c = NULL;
	const int size = 512;
	int n = 0, ret = 0, reg_01 = 0;

	pr_info("%s: i2c read enter.\n", __func__);

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: Invalid data.\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(mmax_priv->dev)) {
		pr_err("%s: Invalid client.\n ", __func__);
		return -EFAULT;
	}

	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: Invalid regmap.\n ", __func__);
		return -EFAULT;
	}

	i2c = to_i2c_client(mmax_priv->dev);

	switch (mmax_priv->usbc_ic_mode) {
	case MAX20328:
		mutex_lock(&mmax_priv->usbc_switch_lock);
		ret = regmap_read(mmax_priv->regmap, MAX20328_DEVICE_ID, &reg_01);
		mutex_unlock(&mmax_priv->usbc_switch_lock);
		n += scnprintf(ubuf+n, size-n, "MAX20328-0x%x %s\n",
			i2c->addr, ((ret < 0) || !(reg_01 & 0x80)) ? "ERROR" : "OK");
		break;
	case FSA4480:
		mutex_lock(&mmax_priv->usbc_switch_lock);
		ret = regmap_read(mmax_priv->regmap, FSA4480_REG_DEVID, &reg_01);
		mutex_unlock(&mmax_priv->usbc_switch_lock);
		n += scnprintf(ubuf+n, size-n, "FSA4480-0x%x %s\n",
			i2c->addr, ((ret < 0) || !(reg_01 & 0x08)) ? "ERROR" : "OK");
		break;
	default:
		break;
	}

	ubuf[n] = 0;
	return n;
}

static ssize_t max20328_unuseirq_show(struct kobject *kobj, struct kobj_attribute *attr, char *ubuf)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	const int size = 512;
	int n = 0;

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: Invalid data.\n", __func__);
		return -EINVAL;
	}

	n += scnprintf(ubuf+n, size-n, "is_unuseirq: %d\n", mmax_priv->is_unuseirq);

	ubuf[n] = 0;
	return n;
}

static ssize_t max20328_unuseirq_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *ubuf, size_t cnt)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	unsigned int kbuf[2];
	int ret = 0;

	ret = sscanf(ubuf, "%x", &kbuf[0]);
	if (!ret) {
		pr_err("%s: sscanf fail.\n", __func__);
		return 0;
	}

	pr_info("%s: kbuf[0]: %x cnt: %d\n",
		__func__, kbuf[0], (int)cnt);

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: Invalid data.\n", __func__);
		return 0;
	}

	mutex_lock(&mmax_priv->usbc_switch_lock);
	if (kbuf[0] == 1) {
		mmax_priv->is_unuseirq = 1;
#ifdef VUCSI_SET_CCOM
		if (mmax_priv->vucsi_vote && !mmax_priv->vucsi_vote(VUCSI_VOTER_AUDIO, VUCSI_CC_SINK)) {
			pr_info("%s: vote VUCSI_VOTER_AUDIO VUCSI_CC_SINK successful\n", __func__);
		}
#endif
	} else {
		mmax_priv->is_unuseirq = 0;
	}
	mutex_unlock(&mmax_priv->usbc_switch_lock);

	return cnt;
}

static ssize_t max20328_mos_show(struct kobject *kobj, struct kobj_attribute *attr, char *ubuf)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	const int size = 512;
	int n = 0;

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: Invalid data.\n", __func__);
		return -EINVAL;
	}

	n += scnprintf(ubuf+n, size-n, "getprop vendor.audio.vivo.mos.test : %s\n", mmax_priv->is_mostest ? "true" : "false");

	ubuf[n] = 0;
	return n;
}
 
static ssize_t max20328_mos_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *ubuf, size_t cnt)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	unsigned int kbuf[2];
	int ret = 0;

	ret = sscanf(ubuf, "%x", &kbuf[0]);
	if (!ret) {
		pr_err("%s: sscanf fail.\n", __func__);
		return 0;
	}

	pr_info("%s: kbuf[0]: %x cnt: %d\n",
		__func__, kbuf[0], (int)cnt);

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: Invalid data.\n", __func__);
		return 0;
	}

	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: Invalid regmap.\n ", __func__);
		return 0;
	}

	mutex_lock(&mmax_priv->usbc_switch_lock);
	  if (kbuf[0] == true) {
		  mmax_priv->is_mostest = true;
	  } else {
		  mmax_priv->is_mostest = false;
	  }
	  max20328_update_reg_defaults(mmax_priv);
	mutex_unlock(&mmax_priv->usbc_switch_lock);

	return cnt;
}

static ssize_t max20328_powerstate_show(struct kobject *kobj, struct kobj_attribute *attr, char *ubuf)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	const int size = 512;
	int n = 0;

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: Invalid data.\n", __func__);
		return -EINVAL;
	}

	n += scnprintf(ubuf+n, size-n, "power %s\n", mmax_priv->power_enabled ? "on" : "off");

	ubuf[n] = 0;
	return n;
}

static struct kobj_attribute dev_attr_reg =
	__ATTR(reg, 0664, max20328_reg_show, max20328_reg_store);
static struct kobj_attribute dev_attr_i2c =
	__ATTR(i2c, 0664, max20328_i2c_show, NULL);
static struct kobj_attribute dev_attr_unuseirq =
	__ATTR(unuseirq, 0664, max20328_unuseirq_show, max20328_unuseirq_store);
static struct kobj_attribute dev_attr_mos =
	__ATTR(mos, 0664, max20328_mos_show, max20328_mos_store);
static struct kobj_attribute dev_attr_powerstate =
	__ATTR(powerstate, 0664, max20328_powerstate_show, NULL);

static struct attribute *sys_node_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_i2c.attr,
	&dev_attr_unuseirq.attr,
	&dev_attr_mos.attr,
	&dev_attr_powerstate.attr,
	NULL
};

static struct attribute_group node_attribute_group = {
	.name = NULL,		/* put in device directory */
	.attrs = sys_node_attributes
};

static int class_attr_create(struct kobject *kobj)
{
	int ret = -1;
	char name[64];

	scnprintf(name, 48, "audio-max20328");

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

#endif

#ifdef VUCSI_SET_CCOM
/*vivo ouenhui add for dp function begin*/
extern int set_max20328_priv(struct max20328_priv *mmax_priv);
/*vivo ouenhui add for dp function end*/

static int pm_resume_notify(struct notifier_block *notifier,
			     unsigned long pm_event, void *unused)
{
	usbc_switch_mmax_priv->drp_suspend_state = pm_event;
	return NOTIFY_OK;
}

static struct notifier_block pm_resume_nb = {
	.notifier_call = pm_resume_notify,
};
#endif

static int max20328_probe(struct i2c_client *i2c,
			 const struct i2c_device_id *id)
{
	struct max20328_priv *mmax_priv;
//	union power_supply_propval pval = {0, };
//	union power_supply_propval usb_in = {0,};
	int rc = 0;

	mmax_priv = devm_kzalloc(&i2c->dev, sizeof(*mmax_priv),
				GFP_KERNEL);
	if (!mmax_priv)
		return -ENOMEM;

	mmax_priv->dev = &i2c->dev;
	mutex_init(&mmax_priv->usbc_switch_lock);
	/*
	mmax_priv->usb_psy = power_supply_get_by_name("usb");
	if (!mmax_priv->usb_psy) {
		rc = -EPROBE_DEFER;
		dev_dbg(mmax_priv->dev,
			"%s: could not get USB psy info: %d\n",
			__func__, rc);
		goto err_data;
	}
	*/
	dev_dbg(mmax_priv->dev, "%s enter\n", __func__);

#ifdef VUCSI_SET_CCOM
	vucsi_pm_register_notify(&pm_resume_nb);
	mmax_priv->vucsi_vote = vucsi_set_vote;
#endif

	usbc_switch_mmax_priv = mmax_priv;
	mmax_priv->is_unuseirq = 0;

	if (of_find_property(mmax_priv->dev->of_node, "max,usbc-ic-mode", NULL)) {
		rc = of_property_read_u32(mmax_priv->dev->of_node,
					"max,usbc-ic-mode", &mmax_priv->usbc_ic_mode);
		if (rc < 0) {
			dev_err(mmax_priv->dev, "%s: read property failed %d\n", __func__, rc);
		} else {
			dev_dbg(mmax_priv->dev, "%s: usbc_ic_mode %d\n", __func__, mmax_priv->usbc_ic_mode);
		}
	}

	/*audio_v: archer add for PD2049,sm8350 L5B,L1C,L2B have Timing requirements,can't just control L2B*/
	if (of_find_property(mmax_priv->dev->of_node, "vivo-supply-non-independ", NULL)) {
		mmax_priv->supply_nonindepend = true;
		dev_dbg(mmax_priv->dev, "%s: have define vivo-supply-non-independ\n", __func__);
	}

	if (mmax_priv->supply_nonindepend == true) {
		if (of_find_property(mmax_priv->dev->of_node, "vdd-supply", NULL)) {
			mmax_priv->vdd = devm_regulator_get(&i2c->dev, "vdd");
			if (IS_ERR(mmax_priv->vdd)) {
				dev_err(&i2c->dev, "unable to get vdd supply\n");
				rc = PTR_ERR(mmax_priv->vdd);			
			}		
		}
		if (of_find_property(mmax_priv->dev->of_node, "vdda18-supply", NULL)) {
			mmax_priv->vdda18 = devm_regulator_get(&i2c->dev, "vdda18");
			if (IS_ERR(mmax_priv->vdda18)) {
				dev_err(&i2c->dev, "unable to get vdda18 supply\n");
				rc = PTR_ERR(mmax_priv->vdda18);
			}		
		}
		if (of_property_read_u32_array(mmax_priv->dev->of_node, "qcom,vdd-voltage-level",
						(u32 *)mmax_priv->vdd_levels, ARRAY_SIZE(mmax_priv->vdd_levels))) {
			dev_err(&i2c->dev, "error reading qcom,vdd-voltage-level property\n");
		}
	}
	/*audio_v: archer add end*/

	if (of_find_property(mmax_priv->dev->of_node, "vdda33-supply", NULL)) {
		mmax_priv->vdda33 = devm_regulator_get(&i2c->dev, "vdda33");
		if (IS_ERR(mmax_priv->vdda33)) {
			dev_err(&i2c->dev, "unable to get vdda33 supply\n");
			rc = PTR_ERR(mmax_priv->vdda33);
			goto err_supply;
		}
		if (mmax_priv->supply_nonindepend != true)
			max20328_usbc_enable_power(mmax_priv, true);
	}
	/*audio_v add*/
	if (of_find_property(mmax_priv->dev->of_node, "sbusel,vdda307-supply", NULL)) {
		dev_dbg(mmax_priv->dev, "%s: had find sbusel, vdda307-supply \n", __func__);
		mmax_priv->vdda307 = devm_regulator_get(&i2c->dev, "sbusel,vdda307");
		if (IS_ERR(mmax_priv->vdda307)) {
			dev_err(&i2c->dev, "unable to get vdda307 supply\n");
			rc = PTR_ERR(mmax_priv->vdda307);			
		}
		//archer del max20328_sbu_sel_enable_power(mmax_priv, true);
	}
	/*audio_v add end*/
	if (of_find_property(mmax_priv->dev->of_node, "max,en-gpio", NULL)) {
		mmax_priv->mmax_en = of_get_named_gpio(mmax_priv->dev->of_node,
										"max,en-gpio", 0);
		if (!gpio_is_valid(mmax_priv->mmax_en))
			mmax_priv->mmax_en = -EINVAL;

		if (gpio_is_valid(mmax_priv->mmax_en)) {
			rc = gpio_request(mmax_priv->mmax_en, "max20328 en");
			if (rc < 0) {
				dev_err(mmax_priv->dev, "%s: gpio %d request error %d\n",
					   __func__, mmax_priv->mmax_en, rc);
			} else {
				dev_dbg(mmax_priv->dev, "%s: request en gpio %d\n",
						__func__, mmax_priv->mmax_en);
				gpio_direction_output(mmax_priv->mmax_en, 0);
			}
		}
	}
	if (of_find_property(mmax_priv->dev->of_node, "max,sbu-sel-gpio", NULL)) {
		mmax_priv->sbu_sel = of_get_named_gpio(mmax_priv->dev->of_node,
										"max,sbu-sel-gpio", 0);
		if (!gpio_is_valid(mmax_priv->sbu_sel))
			mmax_priv->sbu_sel = -EINVAL;

		if (gpio_is_valid(mmax_priv->sbu_sel)) {
			rc = gpio_request(mmax_priv->sbu_sel, "max20328 sbu sel");
			if (rc < 0) {
				dev_err(mmax_priv->dev, "%s: gpio %d request error %d\n",
					   __func__, mmax_priv->sbu_sel, rc);
			} else {
				dev_dbg(mmax_priv->dev, "%s: request sbu-sel gpio %d,set low\n",
						__func__, mmax_priv->sbu_sel);
				gpio_direction_output(mmax_priv->sbu_sel, 0);
			}
		}
	}

	usb_typec_analog_audio_switch_ic = mmax_priv->usbc_ic_mode;
	switch (mmax_priv->usbc_ic_mode) {
	case MAX20328:
		max20328_regmap_config.max_register = MAX20328_REG_MAX;
		break;
	case FSA4480:
		max20328_regmap_config.max_register = FSA4480_REG_MAX;
		break;
	default:
		break;
	}
	mmax_priv->regmap = devm_regmap_init_i2c(i2c, &max20328_regmap_config);
	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		dev_err(mmax_priv->dev, "%s: Failed to initialize regmap: %d\n",
			__func__, rc);
		if (!mmax_priv->regmap) {
			rc = -EINVAL;
			goto err_supply;
		}
		rc = PTR_ERR(mmax_priv->regmap);
		goto err_supply;
	}
	max20328_update_reg_defaults(mmax_priv);
	mmax_priv->ucsi_nb.notifier_call = max20328_usbc_event_changed;
	mmax_priv->ucsi_nb.priority = 0;
	rc = power_supply_reg_notifier(&mmax_priv->ucsi_nb);
	if (rc) {
		dev_err(mmax_priv->dev, "%s: power supply reg failed: %d\n",
			__func__, rc);
		goto err_supply;
	}

	mmax_priv->usbc_wake_lock = wakeup_source_register(mmax_priv->dev,
					dev_name(mmax_priv->dev));
	if (!mmax_priv->usbc_wake_lock) {
		dev_err(mmax_priv->dev,
			"Failed to register wakeup_source\n");	
	}

	init_waitqueue_head(&mmax_priv->irq_waitq);

	rc = max20328_irq_init(mmax_priv);
	if (rc < 0)
		goto mmax_irq_err;

	i2c_set_clientdata(i2c, mmax_priv);

	INIT_WORK(&mmax_priv->usbc_analog_work,
		  max20328_usbc_analog_work_fn);
	INIT_WORK(&mmax_priv->usbc_removed_work,
			  max20328_usbc_removed_work_fn);
	INIT_WORK(&mmax_priv->max20328_irq_handler_work,
			  max20328_irq_handler_work_fn);

	mmax_priv->max20328_notifier.rwsem =
		(struct rw_semaphore)__RWSEM_INITIALIZER
		((mmax_priv->max20328_notifier).rwsem);
	mmax_priv->max20328_notifier.head = NULL;

	rc = max20328_usbc_irq_init(mmax_priv);
	if (rc < 0)
		goto irq_err;

#ifdef VUCSI_SET_CCOM
	/* audio_v: archer add.
	*  if hardware no designed the usb_det gpio comparator, software force to pull power supply.
	*/
	pr_info("%s: mmax_priv->dev_gpio=%d, mmax_priv->vucsi_vote=%x\n", 
			__func__, mmax_priv->dev_gpio, mmax_priv->vucsi_vote);

	if (!gpio_is_valid(mmax_priv->dev_gpio)) {
		if (mmax_priv->vucsi_vote && !mmax_priv->vucsi_vote(VUCSI_VOTER_AUDIO, VUCSI_CC_DRP)) {
			pr_info("%s: vote VUCSI_VOTER_AUDIO VUCSI_CC_DRP successful\n", __func__);
		}
	}
#endif
	//  audio_v: archer add end

#ifdef SYS_NODE
	class_attr_create(mmax_priv->kobj);
#endif
	/*vivo ouenhui add for dp function begin*/
	mmax_priv->switch_mode_event = max20328_switch_mode_event;
#ifdef VUCSI_SET_CCOM
	set_max20328_priv(usbc_switch_mmax_priv);
#endif
	/*vivo ouenhui add for dp function begin*/

	/*audio_v: archer add*/
	if (mmax_priv->supply_nonindepend == true)
		max20328_usbc_enable_power(mmax_priv, false);
	/*audio_v: archer add end*/

	dev_dbg(mmax_priv->dev, "%s leave\n", __func__);
	return 0;

irq_err:
	if (gpio_is_valid(mmax_priv->mmax_int)) {
		disable_irq_nosync(mmax_priv->mmax_int_irq);
		free_irq(mmax_priv->mmax_int_irq, mmax_priv);
		gpio_free(mmax_priv->mmax_int);
	}
mmax_irq_err:
	wakeup_source_unregister(mmax_priv->usbc_wake_lock);
err_supply:
	if (gpio_is_valid(mmax_priv->mmax_en))
		gpio_free(mmax_priv->mmax_en);
	if (gpio_is_valid(mmax_priv->sbu_sel))
		gpio_free(mmax_priv->sbu_sel);
//err_data:
	usbc_switch_mmax_priv = NULL;
	mutex_destroy(&mmax_priv->usbc_switch_lock);
	devm_kfree(&i2c->dev, mmax_priv);
	return rc;
}

static int max20328_remove(struct i2c_client *i2c)
{
	struct max20328_priv *mmax_priv =
			(struct max20328_priv *)i2c_get_clientdata(i2c);

	if (!mmax_priv)
		return -EINVAL;

	dev_dbg(mmax_priv->dev, "%s\n", __func__);

#ifdef VUCSI_SET_CCOM
	vucsi_pm_unregister_notify(&pm_resume_nb);
#endif

#ifdef SYS_NODE
	class_attr_remove(mmax_priv->kobj);
#endif

	max20328_usbc_set_switch_mode(mmax_priv, TYPEC_ACCESSORY_NONE);
	if (gpio_is_valid(mmax_priv->dev_gpio)) {
		disable_irq_nosync(mmax_priv->dev_gpio_irq);
		free_irq(mmax_priv->dev_gpio_irq, mmax_priv);
		gpio_free(mmax_priv->dev_gpio);
	}
	if (gpio_is_valid(mmax_priv->mmax_int)) {
		disable_irq_nosync(mmax_priv->mmax_int_irq);
		free_irq(mmax_priv->mmax_int_irq, mmax_priv);
		gpio_free(mmax_priv->mmax_int);
	}
	cancel_work_sync(&mmax_priv->max20328_irq_handler_work);
	cancel_work_sync(&mmax_priv->usbc_removed_work);
	cancel_work_sync(&mmax_priv->usbc_analog_work);
	pm_relax(mmax_priv->dev);
	/* deregister from PMI */
	power_supply_unreg_notifier(&mmax_priv->ucsi_nb);
	max20328_usbc_enable_power(mmax_priv, false);	
	if (mmax_priv->vdda307) {
		max20328_sbu_sel_enable_power(mmax_priv, false);
	}
	dev_set_drvdata(&i2c->dev, NULL);
	mutex_destroy(&mmax_priv->usbc_switch_lock);
	usbc_switch_mmax_priv = NULL;
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max20328_suspend(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct max20328_priv *mmax_priv =
			(struct max20328_priv *)i2c_get_clientdata(i2c);

	if (!mmax_priv)
		return -EINVAL;

	dev_dbg(mmax_priv->dev, "%s\n", __func__);

	return 0;
}

static int max20328_resume(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct max20328_priv *mmax_priv =
			(struct max20328_priv *)i2c_get_clientdata(i2c);

	if (!mmax_priv)
		return -EINVAL;

	dev_dbg(mmax_priv->dev, "%s\n", __func__);
	wake_up(&mmax_priv->irq_waitq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(max20328_pm_ops, max20328_suspend, max20328_resume);
#define MAX20328_PM_OPS (&max20328_pm_ops)
#else
#define MAX20328_PM_OPS NULL
#endif /* CONFIG_PM_SLEEP */

static const struct of_device_id max20328_i2c_dt_match[] = {
	{
		.compatible = "qcom,max20328-i2c",
	},
	{}
};

static struct i2c_driver max20328_i2c_driver = {
	.driver = {
		.name = MAX20328_I2C_NAME,
		.of_match_table = max20328_i2c_dt_match,
		.pm = MAX20328_PM_OPS,
	},
	.probe = max20328_probe,
	.remove = max20328_remove,
};

static int __init max20328_init(void)
{
	int rc;

	rc = i2c_add_driver(&max20328_i2c_driver);
	if (rc)
		pr_err("max20328: Failed to register I2C driver: %d\n", rc);

	return rc;
}
module_init(max20328_init);

static void __exit max20328_exit(void)
{
	i2c_del_driver(&max20328_i2c_driver);
}
module_exit(max20328_exit);

MODULE_DESCRIPTION("MAX20328 I2C driver");
MODULE_LICENSE("GPL v2");
