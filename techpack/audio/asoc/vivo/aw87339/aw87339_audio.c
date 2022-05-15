/*
 * aw87339_audio.c   aw87339 pa module
 *
 * Version: v1.3.2
 *
 * Copyright (c) 2019 AWINIC Technology CO., LTD
 *
 *  Author: Joseph <zhangzetao@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/gameport.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/debugfs.h>
#include "../vivo-codec-common.h"
#include "aw87339_audio.h"

/***************************************************
 * aw87339 marco
 ***************************************************/
#define AW87339_I2C_NAME        "aw87339_pa"
#define AW87339_DRIVER_VERSION  "v1.3.2"

#define SYS_NODE

#ifndef BBK_IQOO_AUDIO_DEBUG
#define BBK_IQOO_AUDIO_DEBUG
#endif

#ifdef BBK_IQOO_AUDIO_DEBUG
static struct dentry *aw87339_debugfs_root;
static struct dentry *aw87339_debugfs_reg;
static struct dentry *aw87339_debugfs_i2c;
#endif

static bool chipid_ok = false;

/****************************************************
 * aw87339 variable
 ***************************************************/
struct aw87339 *aw87339;
struct aw87339_container *aw87339_kspk_cnt;
struct aw87339_container *aw87339_drcv_cnt;
struct aw87339_container *aw87339_abrcv_cnt;
struct aw87339_container *aw87339_rcvspk_cnt;

static const char *aw87339_kspk_name = "aw87339_kspk.bin";
static const char *aw87339_drcv_name = "aw87339_drcv.bin";
static const char *aw87339_abrcv_name = "aw87339_abrcv.bin";
static const char *aw87339_rcvspk_name = "aw87339_rcvspk.bin";

unsigned int kspk_load_cont;
unsigned int drcv_load_cont;
unsigned int abrcv_load_cont;
unsigned int rcvspk_load_cont;

int aw87339_hw_reset(struct aw87339 *aw87339);

/**********************************************************
 * i2c write and read
*********************************************************/
static int aw87339_i2c_write(struct aw87339 *aw87339,
			     unsigned char reg_addr, unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

    if (aw87339 == NULL) {
        pr_err("%s: aw87339=%p\n", __func__, aw87339);
        return ret;
    }

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_write_byte_data(aw87339->i2c_client,
						reg_addr, reg_data);
		if (ret < 0) {
			pr_err("%s: i2c_write cnt=%d error=%d\n",
			       __func__, cnt, ret);
		} else {
			break;
		}
		cnt++;
		mdelay(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

static int aw87339_i2c_read(struct aw87339 *aw87339,
			    unsigned char reg_addr, unsigned char *reg_data)
{
	int ret = 1;
	unsigned char cnt = 0;

    if (aw87339 == NULL) {
        pr_err("%s: aw87339=%p\n", __func__, aw87339);
        return ret;
    }

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(aw87339->i2c_client, reg_addr);
		if (ret < 0) {
			pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt,
			       ret);
		} else {
			*reg_data = ret;
			break;
		}
		cnt++;
		mdelay(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

/******************************************************
 * aw87339 hardware control
*******************************************************/
unsigned int aw87339_hw_on(struct aw87339 *aw87339)
{
	int ret = 1;

	pr_info("%s enter\n", __func__);

    if (aw87339 == NULL) {
        pr_err("%s: aw87339=%p\n", __func__, aw87339);
        return ret;
    }

	if (aw87339 && gpio_is_valid(aw87339->reset_gpio)) {
		gpio_set_value_cansleep(aw87339->reset_gpio, 0);
		mdelay(2);
		gpio_set_value_cansleep(aw87339->reset_gpio, 1);
		mdelay(2);
		aw87339->hwen_flag = 1;
	} else {
		dev_err(&aw87339->i2c_client->dev, "%s: failed\n", __func__);
	}

	return 0;
}

unsigned int aw87339_hw_off(struct aw87339 *aw87339)
{
	int ret = 1;

	pr_info("%s enter\n", __func__);

    if (aw87339 == NULL) {
        pr_err("%s: aw87339=%p\n", __func__, aw87339);
        return ret;
    }

	if (aw87339 && gpio_is_valid(aw87339->reset_gpio)) {
		gpio_set_value_cansleep(aw87339->reset_gpio, 0);
		mdelay(2);
		aw87339->hwen_flag = 0;
	} else {
		dev_err(&aw87339->i2c_client->dev, "%s: failed\n", __func__);
	}
	return 0;
}

/**********************************************************
 * aw87339 control interface
**********************************************************/
unsigned char aw87339_audio_kspk(void)
{
	unsigned int i;
	//unsigned int length;

	pr_info("%s enter\n", __func__);

	if (aw87339 == NULL)
		return 2;

	if (!aw87339->hwen_flag)
		aw87339_hw_on(aw87339);
/*
	length = sizeof(aw87339_kspk_cfg_default) / sizeof(char);

	if (aw87339->kspk_cfg_update_flag == 0) {
		for (i = 0; i < length; i = i + 2) {
			aw87339_i2c_write(aw87339,
					  aw87339_kspk_cfg_default[i],
					  aw87339_kspk_cfg_default[i + 1]);
		}
	}
*/

	/* update firmware data */
	if (aw87339->kspk_cfg_update_flag == 1) {
		for (i = 0; i < aw87339_kspk_cnt->len; i = i + 2) {
			aw87339_i2c_write(aw87339,
					  aw87339_kspk_cnt->data[i],
					  aw87339_kspk_cnt->data[i + 1]);
		}
	}

	return 0;
}

unsigned char aw87339_audio_drcv(void)
{
	unsigned int i;
	//unsigned int length;

	pr_info("%s enter\n", __func__);

	if (aw87339 == NULL)
		return 2;

	if (!aw87339->hwen_flag)
		aw87339_hw_on(aw87339);
/*
	length = sizeof(aw87339_drcv_cfg_default) / sizeof(char);
	if (aw87339->drcv_cfg_update_flag == 0) {
		for (i = 0; i < length; i = i + 2) {
			aw87339_i2c_write(aw87339,
					  aw87339_drcv_cfg_default[i],
					  aw87339_drcv_cfg_default[i + 1]);
		}
	}
*/
	if (aw87339->drcv_cfg_update_flag == 1) {	/*send firmware data */
		for (i = 0; i < aw87339_drcv_cnt->len; i = i + 2) {
			aw87339_i2c_write(aw87339,
					  aw87339_drcv_cnt->data[i],
					  aw87339_drcv_cnt->data[i + 1]);
		}
	}
	return 0;
}

unsigned char aw87339_audio_abrcv(void)
{
	unsigned int i;
	//unsigned int length;

	pr_info("%s enter\n", __func__);

	if (aw87339 == NULL)
		return 2;

	if (!aw87339->hwen_flag)
		aw87339_hw_on(aw87339);
/*
	length = sizeof(aw87339_abrcv_cfg_default) / sizeof(char);
	if (aw87339->abrcv_cfg_update_flag == 0) {
		for (i = 0; i < length; i = i + 2) {
			aw87339_i2c_write(aw87339,
					  aw87339_abrcv_cfg_default[i],
					  aw87339_abrcv_cfg_default[i + 1]);
		}
	}
*/
	if (aw87339->abrcv_cfg_update_flag == 1) {	/*send firmware data */
		for (i = 0; i < aw87339_abrcv_cnt->len; i = i + 2) {
			aw87339_i2c_write(aw87339,
					  aw87339_abrcv_cnt->data[i],
					  aw87339_abrcv_cnt->data[i + 1]);
		}
	}
	return 0;
}

unsigned char aw87339_audio_rcvspk(void)
{
	unsigned int i;
	//unsigned int length;

	pr_info("%s enter\n", __func__);

	if (aw87339 == NULL)
		return 2;

	if (!aw87339->hwen_flag)
		aw87339_hw_on(aw87339);
/*
	length = sizeof(aw87339_rcvspk_cfg_default) / sizeof(char);
	if (aw87339->rcvspk_cfg_update_flag == 0) {
		for (i = 0; i < length; i = i + 2) {
			aw87339_i2c_write(aw87339,
					  aw87339_rcvspk_cfg_default[i],
					  aw87339_rcvspk_cfg_default[i + 1]);
		}
	}
*/
	if (aw87339->rcvspk_cfg_update_flag == 1) {	/*send firmware data */
		for (i = 0; i < aw87339_rcvspk_cnt->len; i = i + 2) {
			aw87339_i2c_write(aw87339,
					  aw87339_rcvspk_cnt->data[i],
					  aw87339_rcvspk_cnt->data[i + 1]);
		}
	}

	return 0;
}

unsigned char aw87339_audio_off(void)
{
	if (aw87339 == NULL)
		return 2;

	if (aw87339->hwen_flag)
		aw87339_i2c_write(aw87339, 0x01, 0x00);	/*CHIP Disable */

	aw87339_hw_off(aw87339);

	return 0;
}

/**********************************************************
 * aw87339 firmware cfg update
**********************************************************/
static void aw87339_rcvspk_cfg_loaded(struct aw87339 *aw87339)
{
	int i = 0;
	int ram_timer_val = 2000;
	const struct firmware *cont;
	int err = 0;

	pr_info("%s enter\n", __func__);

    if (aw87339 == NULL) {
        pr_err("%s: aw87339=%p\n", __func__, aw87339);
        return;
    }
	err = request_firmware(&cont, aw87339_rcvspk_name,
		&aw87339->i2c_client->dev);

	rcvspk_load_cont++;
	if (unlikely(cont == NULL || (cont->size == 0) || (err != 0))) {
		pr_err("%s: failed to read %s\n", __func__,
			aw87339_rcvspk_name);
		release_firmware(cont);
		if (rcvspk_load_cont <= 2) {
			schedule_delayed_work(&aw87339->ram_work,
					msecs_to_jiffies(ram_timer_val));
			pr_info("%s:restart hrtimer to load firmware\n",
			__func__);
		}
		return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__, aw87339_rcvspk_name,
				cont ? cont->size : 0);

	for (i = 0; i < cont->size; i = i+2) {
		pr_info("%s: addr:0x%04x, data:0x%02x\n",
		__func__, *(cont->data+i), *(cont->data+i+1));
	}
	/* aw87339 ram update */
	aw87339_rcvspk_cnt = kzalloc(cont->size+sizeof(int), GFP_KERNEL);
	if (!aw87339_rcvspk_cnt) {
		release_firmware(cont);
		pr_err("%s: Error allocating memory\n", __func__);
		return;
	}
	aw87339_rcvspk_cnt->len = cont->size;
	memcpy(aw87339_rcvspk_cnt->data, cont->data, cont->size);
	release_firmware(cont);
	aw87339->rcvspk_cfg_update_flag = 1;

	pr_info("%s: all fw update complete\n", __func__);
}

static void aw87339_rcvspk_update(struct aw87339 *aw87339)
{
	pr_info("%s enter\n", __func__);
	aw87339_rcvspk_cfg_loaded(aw87339);
}

static void aw87339_abrcv_cfg_loaded(struct aw87339 *aw87339)
{
	int i = 0;
	int ram_timer_val = 2000;
	const struct firmware *cont;
	int err = 0;

	pr_info("%s enter\n", __func__);

    if (aw87339 == NULL) {
        pr_err("%s: aw87339=%p\n", __func__, aw87339);
        return;
    }
	err = request_firmware(&cont, aw87339_abrcv_name,
		&aw87339->i2c_client->dev);

	abrcv_load_cont++;
	if (unlikely(cont == NULL || (cont->size == 0) || (err != 0))) {
		pr_err("%s: failed to read %s\n", __func__,
			aw87339_abrcv_name);
		release_firmware(cont);
		if (abrcv_load_cont <= 2) {
			schedule_delayed_work(&aw87339->ram_work,
					msecs_to_jiffies(ram_timer_val));
			pr_info("%s:restart hrtimer to load firmware\n",
			__func__);
		}
		return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__, aw87339_abrcv_name,
					cont ? cont->size : 0);

	for (i = 0; i < cont->size; i = i+2) {
		pr_info("%s: addr:0x%04x, data:0x%02x\n",
		__func__, *(cont->data+i), *(cont->data+i+1));
	}


	/* aw87339 ram update */
	aw87339_abrcv_cnt = kzalloc(cont->size+sizeof(int), GFP_KERNEL);
	if (!aw87339_abrcv_cnt) {
		release_firmware(cont);
		pr_err("%s: Error allocating memory\n", __func__);
		return;
	}
	aw87339_abrcv_cnt->len = cont->size;
	memcpy(aw87339_abrcv_cnt->data, cont->data, cont->size);
	release_firmware(cont);
	aw87339->abrcv_cfg_update_flag = 1;

	pr_info("%s: fw update complete\n", __func__);
}

static void aw87339_abrcv_update(struct aw87339 *aw87339)
{
	pr_info("%s enter\n", __func__);
	aw87339_abrcv_cfg_loaded(aw87339);
}

static void aw87339_drcv_cfg_loaded(struct aw87339 *aw87339)
{
	int i = 0;
	int ram_timer_val = 2000;
	const struct firmware *cont;
	int err = 0;

	pr_info("%s enter\n", __func__);

    if (aw87339 == NULL) {
        pr_err("%s: aw87339=%p\n", __func__, aw87339);
        return;
    }
	err = request_firmware(&cont, aw87339_drcv_name,
		&aw87339->i2c_client->dev);

	drcv_load_cont++;
	if (unlikely(cont == NULL || (cont->size == 0) || (err != 0))) {
		pr_err("%s: failed to read %s\n", __func__, aw87339_drcv_name);
		release_firmware(cont);
		if (drcv_load_cont <= 2) {
			schedule_delayed_work(&aw87339->ram_work,
					msecs_to_jiffies(ram_timer_val));
			pr_info("%s:restart hrtimer to load firmware\n",
			__func__);
		}
		return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__, aw87339_drcv_name,
					cont ? cont->size : 0);

	for (i = 0; i < cont->size; i = i+2) {
		pr_info("%s: addr:0x%04x, data:0x%02x\n",
		__func__, *(cont->data+i), *(cont->data+i+1));
	}

	if (aw87339_drcv_cnt != NULL)
		aw87339_drcv_cnt = NULL;

	/* aw87339 ram update */
	aw87339_drcv_cnt = kzalloc(cont->size+sizeof(int), GFP_KERNEL);
	if (!aw87339_drcv_cnt) {
		release_firmware(cont);
		pr_err("%s: Error allocating memory\n", __func__);
		return;
	}
	aw87339_drcv_cnt->len = cont->size;
	memcpy(aw87339_drcv_cnt->data, cont->data, cont->size);
	release_firmware(cont);
	aw87339->drcv_cfg_update_flag = 1;

	pr_info("%s: fw update complete\n", __func__);
}

static void aw87339_drcv_update(struct aw87339 *aw87339)
{
	pr_info("%s enter\n", __func__);
	aw87339_drcv_cfg_loaded(aw87339);
}

static void aw87339_kspk_cfg_loaded(struct aw87339 *aw87339)
{
	int i = 0;
	int ram_timer_val = 2000;
	const struct firmware *cont;
	int err = 0;

	pr_info("%s enter\n", __func__);

    if (aw87339 == NULL) {
        pr_err("%s: aw87339=%p\n", __func__, aw87339);
        return;
    }
    err = request_firmware(&cont, aw87339_kspk_name,
		&aw87339->i2c_client->dev);

	kspk_load_cont++;
	if (unlikely(cont == NULL || (cont->size == 0) || (err != 0))) {
		pr_err("%s: failed to read %s\n", __func__, aw87339_kspk_name);
		release_firmware(cont);
		if (kspk_load_cont <= 2) {
			schedule_delayed_work(&aw87339->ram_work,
					msecs_to_jiffies(ram_timer_val));
			pr_info("%s:restart hrtimer to load firmware\n",
			__func__);
		}
		return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__, aw87339_kspk_name,
				  cont ? cont->size : 0);

	for (i = 0; i < cont->size; i = i+2) {
		pr_info("%s: addr:0x%04x, data:0x%02x\n",
		__func__, *(cont->data+i), *(cont->data+i+1));
	}

	if (aw87339_kspk_cnt != NULL)
		aw87339_kspk_cnt = NULL;

	/* aw87339 ram update */
	aw87339_kspk_cnt = kzalloc(cont->size+sizeof(int), GFP_KERNEL);
	if (!aw87339_kspk_cnt) {
		release_firmware(cont);
		pr_err("%s: Error allocating memory\n", __func__);
		return;
	}
	aw87339_kspk_cnt->len = cont->size;
	memcpy(aw87339_kspk_cnt->data, cont->data, cont->size);
	release_firmware(cont);
	aw87339->kspk_cfg_update_flag = 1;

	pr_info("%s: fw update complete\n", __func__);
}

static void aw87339_kspk_update(struct aw87339 *aw87339)
{
	pr_info("%s enter\n", __func__);

	aw87339_kspk_cfg_loaded(aw87339);
}

static void aw87339_cfg_work_routine(struct work_struct *work)
{
	pr_info("%s enter\n", __func__);
	if (aw87339->kspk_cfg_update_flag == 0)
		aw87339_kspk_update(aw87339);
	if (aw87339->drcv_cfg_update_flag == 0)
		aw87339_drcv_update(aw87339);
	if (aw87339->abrcv_cfg_update_flag == 0)
		aw87339_abrcv_update(aw87339);
	if (aw87339->rcvspk_cfg_update_flag == 0)
		aw87339_rcvspk_update(aw87339);
}

static int aw87339_cfg_init(struct aw87339 *aw87339)
{
	int ret = -1;
	int cfg_timer_val = 5000;

    if (aw87339 == NULL) {
        pr_err("%s: aw87339=%p\n", __func__, aw87339);
        return ret;
    }
	INIT_DELAYED_WORK(&aw87339->ram_work, aw87339_cfg_work_routine);
	schedule_delayed_work(&aw87339->ram_work,
				msecs_to_jiffies(cfg_timer_val));
	return ret;
}

/**********************************************************
 * aw87339 attribute
***********************************************************/
static ssize_t aw87339_get_reg(struct device *cd,
			       struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	unsigned int i = 0;
	unsigned char reg_val = 0;

    if (aw87339 == NULL) {
        pr_err("%s: aw87339=%p\n", __func__, aw87339);
        return len;
    }

	for (i = 0; i < aw87339_REG_MAX; i++) {
		if (aw87339->aw873xx_reg_access[i] & REG_RD_ACCESS) {
			aw87339_i2c_read(aw87339, i, &reg_val);
			len += snprintf(buf + len, PAGE_SIZE - len,
					"reg:0x%02x=0x%02x\n", i, reg_val);
		}
	}

	return len;
}

static ssize_t aw87339_set_reg(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t len)
{
	unsigned int databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2)
		aw87339_i2c_write(aw87339, databuf[0], databuf[1]);

	return len;
}

static ssize_t aw87339_get_hwen(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

    if (aw87339 == NULL) {
        pr_err("%s: aw87339=%p\n", __func__, aw87339);
        return len;
    }

	len += snprintf(buf+len, PAGE_SIZE-len, "hwen: %d\n",
			aw87339->hwen_flag);

	return len;
}

static ssize_t aw87339_set_hwen(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t len)
{
	ssize_t ret;
	unsigned int state;

    if (aw87339 == NULL) {
        pr_err("%s: aw87339=%p\n", __func__, aw87339);
        return len;
    }

	ret = kstrtouint(buf, 10, &state);
	if (ret)
		goto out_strtoint;
	if (state == 0) {		/*OFF*/
		aw87339_hw_off(aw87339);
	} else {			/*ON*/
		aw87339_hw_on(aw87339);
	}

	if (ret < 0)
		goto out;

	return len;

out:
	dev_err(&aw87339->i2c_client->dev, "%s: i2c access fail to register\n",
		__func__);
out_strtoint:
	dev_err(&aw87339->i2c_client->dev, "%s: fail to change str to int\n",
		__func__);
	return ret;
}

static ssize_t aw87339_get_update(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	return len;
}

static ssize_t aw87339_set_update(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	ssize_t ret;
	unsigned int state;
	int cfg_timer_val = 10;

    if (aw87339 == NULL) {
        pr_err("%s: aw87339=%p\n", __func__, aw87339);
        return len;
    }

	ret = kstrtouint(buf, 10, &state);
	if (ret)
		goto out_strtoint;
	if (state == 0) {
	} else {
		aw87339->kspk_cfg_update_flag = 0;
		aw87339->drcv_cfg_update_flag = 0;
		aw87339->abrcv_cfg_update_flag = 0;
		aw87339->rcvspk_cfg_update_flag = 0;
		schedule_delayed_work(&aw87339->ram_work,
					msecs_to_jiffies(cfg_timer_val));
	}

	if (ret < 0)
		goto out;

	return len;
out:
	dev_err(&aw87339->i2c_client->dev, "%s: i2c access fail to register\n",
		__func__);
out_strtoint:
	dev_err(&aw87339->i2c_client->dev, "%s: fail to change str to int\n",
		__func__);
	return ret;
}

static ssize_t aw87339_get_mode(struct device *cd,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len, "0: off mode\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "1: kspk mode\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "2: drcv mode\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "3: abrcv mode\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "4: rcvspk mode\n");

	return len;
}

static ssize_t aw87339_set_mode(struct device *cd,
		struct device_attribute *attr, const char *buf, size_t len)
{
	ssize_t ret;
	unsigned int state;

    if (aw87339 == NULL) {
        pr_err("%s: aw87339=%p\n", __func__, aw87339);
        return len;
    }

	ret = kstrtouint(buf, 10, &state);
	if (ret)
		goto out_strtoint;
	if (state == 0)
		aw87339_audio_off();
	else if (state == 1)
		aw87339_audio_kspk();
	else if (state == 2)
		aw87339_audio_drcv();
	else if (state == 3)
		aw87339_audio_abrcv();
	else if (state == 4)
		aw87339_audio_rcvspk();
	else
		aw87339_audio_off();

	if (ret < 0)
		goto out;

	return len;
out:
	dev_err(&aw87339->i2c_client->dev, "%s: i2c access fail to register\n",
		__func__);
out_strtoint:
	dev_err(&aw87339->i2c_client->dev, "%s: fail to change str to int\n",
		__func__);
	return ret;
}

static ssize_t aw87339_get_iic(struct device *cd,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	unsigned int i = 0;
	unsigned char reg_val = 0;
	int ret = 0;
	bool iic_ok = false;

	pr_info("%s: enter, iic_ok :%d\n", __func__, iic_ok);

	if (aw87339 == NULL) {
		pr_err("%s: aw87339=%p\n", __func__, aw87339);
		return len;
	}

	for (i = 0; i < aw87339_REG_MAX; i++) {
		if (aw87339->aw873xx_reg_access[i] & REG_RD_ACCESS) {
			ret = aw87339_i2c_read(aw87339, i, &reg_val);
			if (ret < 0) {
				iic_ok = false;
				pr_info("iic read, iic_ok :%d, reg[0x%x]=0x%x\n", iic_ok , i, reg_val);
				break;
			}
			else
				iic_ok = true;
		}
	}

	pr_info("%s: leave, iic_ok :%d\n", __func__, iic_ok);

	len += snprintf(buf+len, PAGE_SIZE-len, "%d\n", iic_ok);

	return len;
}

static ssize_t aw87339_set_iic(struct device *cd,
		struct device_attribute *attr, const char *buf, size_t len)
{

	return 0;
}

static DEVICE_ATTR(reg, 0660, aw87339_get_reg, aw87339_set_reg);
static DEVICE_ATTR(hwen, 0660, aw87339_get_hwen, aw87339_set_hwen);
static DEVICE_ATTR(update, 0660, aw87339_get_update, aw87339_set_update);
static DEVICE_ATTR(mode, 0660, aw87339_get_mode, aw87339_set_mode);
static DEVICE_ATTR(iic, 0660, aw87339_get_iic, aw87339_set_iic);

static struct attribute *aw87339_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_hwen.attr,
	&dev_attr_update.attr,
	&dev_attr_mode.attr,
	&dev_attr_iic.attr,
	NULL
};

static struct attribute_group aw87339_attribute_group = {
	.attrs = aw87339_attributes
};

#ifdef SYS_NODE
static ssize_t i2c_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	const int size = 20;
	int n = 0;

	pr_info("[SmartPA-%d]%s enter.\n", __LINE__, __func__);

	n += scnprintf(buf, size, "SmartPA-0x%x %s\n", 
		aw87339->i2c_client->addr, chipid_ok ? "OK" : "ERROR");

	return n;
}

static struct kobj_attribute dev_attr_i2c =
	__ATTR(i2c, 0664, i2c_show, NULL);

static struct attribute *sys_node_attributes[] = {
	&dev_attr_i2c.attr,
	NULL
};
static struct attribute_group aw87339_node_attribute_group = {
	.name = NULL,		/* put in device directory */
	.attrs = sys_node_attributes
};

static int class_attr_create(struct aw87339 *aw87339)
{
	int ret = -1;

	if (aw87339 == NULL) {
		return ret;
	}

	aw87339->k_obj = kobject_create_and_add("audio-aw87339", kernel_kobj);
	if (!aw87339->k_obj) {
		pr_info("kobject_create_and_add audio-aw87339 file faild\n");
		return 0;
	}

	ret = sysfs_create_group(aw87339->k_obj, &aw87339_node_attribute_group);
	if (ret) {
		pr_info("sysfs_create_group audio-aw87339 file faild\n");
	}

	return ret;
}

static int class_attr_remove(struct aw87339 *aw87339)
{
	if (aw87339 != NULL && aw87339->k_obj) {
		sysfs_remove_group(aw87339->k_obj, &aw87339_node_attribute_group);
		kobject_del(aw87339->k_obj);
		aw87339->k_obj = NULL;
	}
	return 0;
}
#endif

#ifdef BBK_IQOO_AUDIO_DEBUG
static int aw87339_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t aw87339_debug_write(struct file *filp, const char __user *ubuf,
									size_t cnt, loff_t *ppos)
{
	int ret = 0;
	unsigned int kbuf[2];
	char *temp;

	temp = kzalloc(cnt, GFP_KERNEL);
	if (!temp) {
		return -ENOMEM;
	}

	ret = copy_from_user(temp, ubuf, cnt);
	ret = sscanf(temp, "%x %x", &kbuf[0], &kbuf[1]);
	if (!ret) {
		kfree(temp);
		return -EFAULT;
	}

	pr_info("%s: kbuf[0]: %x, kbuf[1]: %x cnt: %d\n",
		__func__, kbuf[0], kbuf[1], (int)cnt);

	aw87339_i2c_write(aw87339, kbuf[0], kbuf[1]);

	kfree(temp);
	return cnt;
}

static ssize_t aw87339_debug_read(struct file *file, char __user *buf,
                                  size_t count, loff_t *pos)
{
	unsigned int i = 0;
	unsigned char reg_val = 0;
	const int size = 512;
	char buffer[size];
	int n = 0;

	for (i = 0; i < aw87339_REG_MAX; i++) {
		if (aw87339->aw873xx_reg_access[i] & REG_RD_ACCESS) {
			aw87339_i2c_read(aw87339, i, &reg_val);
			n += snprintf(buffer + n, size - n,
					"reg:0x%02x=0x%02x\n", i, reg_val);
			pr_info("%s:catch aw87339 reg[0x%x]:0x%x\n",
				__func__, i, reg_val);
		}
	}

	buffer[n] = 0;

	return simple_read_from_buffer(buf, count, pos, buffer, n);
}

static struct file_operations aw87339_debugfs_fops = {
	.open = aw87339_debug_open,
	.read = aw87339_debug_read,
	.write = aw87339_debug_write,
};

static ssize_t aw87339_debug_i2c_read(struct file *file, char __user *buf,
                                      size_t count, loff_t *pos)
{
	const int size = 512;
	char buffer[size];
	int n = 0;

	pr_info("%s enter.\n", __func__);

	n += scnprintf(buffer + n, size - n, "SmartPA-0x%x %s\n",
		aw87339->i2c_client->addr, chipid_ok ? "OK" : "ERROR");

	buffer[n] = 0;

	return simple_read_from_buffer(buf, count, pos, buffer, n);
}

static struct file_operations aw87339_i2c_debugfs_fops = {
	.open = aw87339_debug_open,
	.read = aw87339_debug_i2c_read,
};

static void aw87339_debugfs_init(void)
{
	aw87339_debugfs_root = debugfs_create_dir("audio-aw87339", NULL);
	if (IS_ERR_OR_NULL(aw87339_debugfs_root)) {
		pr_err("%s debugfs create dir error\n", __func__);
		return;
	}

	aw87339_debugfs_reg = debugfs_create_file("reg", 0644,
		aw87339_debugfs_root, NULL, &aw87339_debugfs_fops);
	if (IS_ERR_OR_NULL(aw87339_debugfs_reg)) {
		pr_err("aw87339 debugfs create fail.\n");
	}

	aw87339_debugfs_i2c = debugfs_create_file("i2c", 0444,
		aw87339_debugfs_root, NULL, &aw87339_i2c_debugfs_fops);
	if (IS_ERR_OR_NULL(aw87339_debugfs_i2c)) {
		pr_err("aw87339 i2c create fail.\n");
	}

	return;
}

static void aw87339_debugfs_deinit(void)
{
	debugfs_remove(aw87339_debugfs_i2c);
	debugfs_remove(aw87339_debugfs_reg);
	debugfs_remove(aw87339_debugfs_root);
	return;
}
#endif

static bool aw87339_get_pa_state(void)
{

	if(!aw87339)
		return false;
	else
		return !!aw87339->hwen_flag;
}

static int aw87339_pa_enble(int state)
{
	int ret = 0;

	pr_info("%s: enter, state: %d\n",
		__func__, state);

	switch(state) {
	case EXT_PA_SWICH_RCV:
		ret = aw87339_audio_drcv();
		break;
	case EXT_PA_SWICH_SPK:
		ret = aw87339_audio_kspk();
		break;
	case EXT_PA_SWICH_SPK_FM:
		ret = aw87339_audio_abrcv();
		break;
	case EXT_PA_SWICH_NONE:
	default:
		ret = aw87339_audio_off();
		break;
	}

	pr_info("%s: leave.\n", __func__);

	return ret;
}

/******************************************************/
static int aw87339_parse_dt(struct device *dev, struct device_node *np)
{
	static const char *kspk_config = NULL;
	static const char *drcv_config = NULL;
	static const char *abrcv_config = NULL;
	static const char *rcvspk_config = NULL;
	int ret = 0;

    if (aw87339 == NULL) {
        pr_err("%s: aw87339=%p\n", __func__, aw87339);
        return -1;
    }

	aw87339->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (aw87339->reset_gpio < 0) {
		dev_err(dev, "%s: no reset gpio provided .\n", __func__);
		return -1;
	}
	dev_info(dev, "%s: reset gpio provided ok\n", __func__);

	ret = of_property_read_string(np, "vivo,aw873xx-kspk-config", &kspk_config);
	if (kspk_config)
		aw87339_kspk_name = kspk_config;

	ret = of_property_read_string(np, "vivo,aw873xx-drcv-config", &drcv_config);
	if (drcv_config)
		aw87339_drcv_name = drcv_config;

	ret = of_property_read_string(np, "vivo,aw873xx-abrcv-config", &abrcv_config);
	if (abrcv_config)
		aw87339_abrcv_name = abrcv_config;

	ret = of_property_read_string(np, "vivo,aw873xx-rcvspk-config", &rcvspk_config);
	if (rcvspk_config)
		aw87339_rcvspk_name = rcvspk_config;

	return 0;
}

int aw87339_hw_reset(struct aw87339 *aw87339)
{
	int ret = -1;
	pr_info("%s enter\n", __func__);

    if (aw87339 == NULL) {
        pr_err("%s: aw87339=%p\n", __func__, aw87339);
		return ret;
    }

	if (aw87339 && gpio_is_valid(aw87339->reset_gpio)) {
		gpio_set_value_cansleep(aw87339->reset_gpio, 0);
		mdelay(2);
		gpio_set_value_cansleep(aw87339->reset_gpio, 1);
		mdelay(2);
		aw87339->hwen_flag = 1;
	} else {
		aw87339->hwen_flag = 0;
		dev_err(&aw87339->i2c_client->dev, "%s:  failed\n", __func__);
	}
	return 0;
}

/*****************************************************
 * check chip id
 *****************************************************/
int aw87339_read_chipid(struct aw87339 *aw87339)
{
	unsigned int cnt = 0;
	int ret = -1;
	unsigned char reg_val = 0;

	while (cnt < AW_READ_CHIPID_RETRIES) {
		ret = aw87339_i2c_read(aw87339, aw87339_REG_CHIPID, &reg_val);
		if (reg_val == AW87339_CHIPID || reg_val == AW87359_CHIPID) {
			chipid_ok = true;
			aw87339->chipid = reg_val;
			aw87339->aw873xx_reg_access = aw87339_reg_access;
			pr_info("%s This Chip is  AW87339 chipid=0x%x\n",
				__func__, reg_val);

			if (AW87359_CHIPID == aw87339->chipid) {
				aw87339->aw873xx_reg_access = aw87359_reg_access;
			}
			return 0;
		}

		cnt++;

		mdelay(AW_READ_CHIPID_RETRY_DELAY);
	}
	pr_info("%s: aw87339 chipid=0x%x error\n", __func__, reg_val);
	return -EINVAL;
}

/*********************************************************
 * aw87339 i2c driver
 *********************************************************/
static int
aw87339_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct vivo_codec_function *vivo_codec_function = NULL;
	struct device_node *np = client->dev.of_node;
	int ret = -1;

	pr_info("%s Enter\n", __func__);

	vivo_codec_function = get_vivo_codec_function();
	if (!vivo_codec_function) {
		pr_err("%s:vivo_codec_function malloc failed\n", __func__);
		return -EPROBE_DEFER;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,
			"%s: check_functionality failed\n", __func__);
		ret = -ENODEV;
		goto exit_check_functionality_failed;
	}

	aw87339 = devm_kzalloc(&client->dev,
			       sizeof(struct aw87339), GFP_KERNEL);
	if (aw87339 == NULL) {
		ret = -ENOMEM;
		goto exit_devm_kzalloc_failed;
	}

	aw87339->i2c_client = client;
	i2c_set_clientdata(client, aw87339);

	/* aw87339 rst */
	if (np) {
		ret = aw87339_parse_dt(&client->dev, np);
		if (ret) {
			dev_err(&client->dev,
				"%s: failed to parse device tree node\n",
				__func__);
			goto exit_gpio_get_failed;
		}
	} else {
		aw87339->reset_gpio = -1;
	}

	if (gpio_is_valid(aw87339->reset_gpio)) {
		ret =
		    devm_gpio_request_one(&client->dev,
					  aw87339->reset_gpio,
					  GPIOF_OUT_INIT_LOW, "aw87339_rst");
		if (ret) {
			dev_err(&client->dev,
				"%s: rst request failed\n", __func__);
			goto exit_gpio_request_failed;
		}
	}

	/* hardware reset */
	aw87339_hw_reset(aw87339);

	/* aw87339 chip id */
	ret = aw87339_read_chipid(aw87339);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s:aw87339_read_chipid failed ret=%d\n",
			__func__, ret);
		goto exit_i2c_check_id_failed;
	}
	dev_err(&client->dev,"%s: kspk_name read %s\n", __func__, aw87339_kspk_name);
	ret = sysfs_create_group(&client->dev.kobj, &aw87339_attribute_group);
	if (ret < 0) {
		dev_info(&client->dev,
			 "%s error creating sysfs attr files\n", __func__);
	}

	/* aw87339 cfg update */
	kspk_load_cont = 0;
	drcv_load_cont = 0;
	abrcv_load_cont = 0;
	rcvspk_load_cont = 0;
	aw87339->kspk_cfg_update_flag = 0;
	aw87339->drcv_cfg_update_flag = 0;
	aw87339->abrcv_cfg_update_flag = 0;
	aw87339->rcvspk_cfg_update_flag = 0;
	aw87339_cfg_init(aw87339);

#ifdef SYS_NODE
		class_attr_create(aw87339);
#endif

#ifdef BBK_IQOO_AUDIO_DEBUG
	aw87339_debugfs_init();
#endif
	vivo_codec_function->ext_pa_enable = aw87339_pa_enble;
	vivo_codec_function->ext_get_pa_state = aw87339_get_pa_state;
	/* aw87339 hardware off */
	aw87339_hw_off(aw87339);

	return 0;

 exit_i2c_check_id_failed:
	devm_gpio_free(&client->dev, aw87339->reset_gpio);
 exit_gpio_request_failed:
 exit_gpio_get_failed:
	devm_kfree(&client->dev, aw87339);
	aw87339 = NULL;
 exit_devm_kzalloc_failed:
 exit_check_functionality_failed:
	return ret;
}

static int aw87339_i2c_remove(struct i2c_client *client)
{
	struct aw87339 *aw87339 = i2c_get_clientdata(client);

#ifdef SYS_NODE
		class_attr_remove(aw87339);
#endif

#ifdef BBK_IQOO_AUDIO_DEBUG
	aw87339_debugfs_deinit();
#endif

	if (gpio_is_valid(aw87339->reset_gpio))
		devm_gpio_free(&client->dev, aw87339->reset_gpio);

	return 0;
}

static const struct i2c_device_id aw87339_i2c_id[] = {
	{AW87339_I2C_NAME, 0},
	{}
};

static const struct of_device_id extpa_of_match[] = {
	{.compatible = "awinic,aw87339_pa"},
	{},
};

static struct i2c_driver aw87339_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = AW87339_I2C_NAME,
		   .of_match_table = extpa_of_match,
		   },
	.probe = aw87339_i2c_probe,
	.remove = aw87339_i2c_remove,
	.id_table = aw87339_i2c_id,
};

static int __init aw87339_pa_init(void)
{
	int ret;

	pr_info("%s enter\n", __func__);
	pr_info("%s: driver version: %s\n", __func__, AW87339_DRIVER_VERSION);

	ret = i2c_add_driver(&aw87339_i2c_driver);
	if (ret) {
		pr_info("****[%s] Unable to register driver (%d)\n",
			__func__, ret);
		return ret;
	}
	return 0;
}

static void __exit aw87339_pa_exit(void)
{
	pr_info("%s enter\n", __func__);
	i2c_del_driver(&aw87339_i2c_driver);
}

module_init(aw87339_pa_init);
module_exit(aw87339_pa_exit);

MODULE_AUTHOR("<zhangzeta@awinic.com.cn>");
MODULE_DESCRIPTION("AWINIC aw87339 PA driver");
MODULE_LICENSE("GPL v2");
