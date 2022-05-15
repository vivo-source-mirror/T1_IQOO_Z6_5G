// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2017-2020, The Linux Foundation. All rights reserved. */

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/errno.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>
/*added by vsen sensor team, for vsen haptic printf pid start*/
#include <linux/sched.h>
#include <linux/smp.h>
#include <linux/export.h>
#include <asm/ptrace.h>
#include <linux/printk.h>
/*added by vsen sensor team, for vsen haptic printf pid end*/
/*added by vivo sensor team, for vivo haptic start*/
#include <linux/gpio.h>
#include <linux/of_gpio.h>
/*added by vivo sensor team, for vivo haptic end*/


/*
 * Define vibration periods: default(5sec), min(50ms), max(15sec)
 */
#define QPNP_VIB_MIN_PLAY_MS		50
#define QPNP_VIB_PLAY_MS		5000
#define QPNP_VIB_MAX_PLAY_MS		15000

struct vib_ldo_chip {
	struct led_classdev	cdev;
	struct mutex		lock;
	struct hrtimer		stop_timer;
	struct work_struct	vib_work;

	int			state;
	u64			vib_play_ms;
	bool			vib_enabled;
	int			enable_gpio;
};

static inline int qpnp_vib_ldo_enable(struct vib_ldo_chip *chip, bool enable)
{
	int ret;
	if (chip->vib_enabled == enable)
		return 0;

	if (chip->enable_gpio) {
		if (gpio_is_valid(chip->enable_gpio)) {
			ret = gpio_direction_output(chip->enable_gpio, enable);
			if (ret) {
				pr_err(" Unable to gpio_direction_output, val:%d\n", enable);
				return ret;
			}
		}
	}
	pr_err("qpnp_vib_gpio_enable state = %d, time = %llums\n", enable, chip->vib_play_ms);
	chip->vib_enabled = enable;

	return 0;
}

static int qpnp_vibrator_play_on(struct vib_ldo_chip *chip)
{
	int ret;

	ret = qpnp_vib_ldo_enable(chip, true);
	if (ret < 0) {
		pr_err("vibration enable failed, ret=%d\n", ret);
		return ret;
	}

	return ret;
}

static void qpnp_vib_work(struct work_struct *work)
{
	struct vib_ldo_chip *chip = container_of(work, struct vib_ldo_chip,
						vib_work);
	int ret = 0;

	if (chip->state) {
		if (!chip->vib_enabled)
			ret = qpnp_vibrator_play_on(chip);

		if (ret == 0)
			hrtimer_start(&chip->stop_timer,
				      ms_to_ktime(chip->vib_play_ms),
				      HRTIMER_MODE_REL);
	} else {
		qpnp_vib_ldo_enable(chip, false);
	}
}

static enum hrtimer_restart vib_stop_timer(struct hrtimer *timer)
{
	struct vib_ldo_chip *chip = container_of(timer, struct vib_ldo_chip,
					     stop_timer);

	chip->state = 0;
	schedule_work(&chip->vib_work);
	return HRTIMER_NORESTART;
}

static ssize_t qpnp_vib_show_state(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct vib_ldo_chip *chip = container_of(cdev, struct vib_ldo_chip,
						cdev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->vib_enabled);
}

static ssize_t qpnp_vib_store_state(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	/* At present, nothing to do with setting state */
	return count;
}

static ssize_t qpnp_vib_show_duration(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct vib_ldo_chip *chip = container_of(cdev, struct vib_ldo_chip,
						cdev);
	ktime_t time_rem;
	s64 time_ms = 0;

	if (hrtimer_active(&chip->stop_timer)) {
		time_rem = hrtimer_get_remaining(&chip->stop_timer);
		time_ms = ktime_to_ms(time_rem);
	}

	return scnprintf(buf, PAGE_SIZE, "%lld\n", time_ms);
}

static ssize_t qpnp_vib_store_duration(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct vib_ldo_chip *chip = container_of(cdev, struct vib_ldo_chip,
						cdev);
	u32 val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0)
		return ret;

	/* setting 0 on duration is NOP for now */
	if (val <= 0)
		return count;

	if (val < QPNP_VIB_MIN_PLAY_MS)
		val = QPNP_VIB_MIN_PLAY_MS;

	if (val > QPNP_VIB_MAX_PLAY_MS)
		val = QPNP_VIB_MAX_PLAY_MS;

	mutex_lock(&chip->lock);
	chip->vib_play_ms = val;
	mutex_unlock(&chip->lock);

	return count;
}

static ssize_t qpnp_vib_show_activate(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	/* For now nothing to show */
	return scnprintf(buf, PAGE_SIZE, "%d\n", 0);
}

static ssize_t qpnp_vib_store_activate(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct vib_ldo_chip *chip = container_of(cdev, struct vib_ldo_chip,
						cdev);
	u32 val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val != 0 && val != 1)
		return count;

	mutex_lock(&chip->lock);
	hrtimer_cancel(&chip->stop_timer);
	chip->state = val;
	/*added by vsen sensor team, for vsen haptic printf pid start*/
	pr_err("haptics state = %d, time = %llums, pid is %d\n", chip->state, chip->vib_play_ms, current->pid);
	/*added by vsen sensor team, for vsen haptic printf pid end*/
	mutex_unlock(&chip->lock);
	schedule_work(&chip->vib_work);

	return count;
}


static struct device_attribute qpnp_vib_attrs[] = {
	__ATTR(state, 0664, qpnp_vib_show_state, qpnp_vib_store_state),
	__ATTR(duration, 0664, qpnp_vib_show_duration, qpnp_vib_store_duration),
	__ATTR(activate, 0664, qpnp_vib_show_activate, qpnp_vib_store_activate),
};

static int qpnp_vib_parse_dt(struct device *dev, struct vib_ldo_chip *chip)
{
	chip->enable_gpio = of_get_named_gpio(dev->of_node, "qcom,vibrator_enable", 0);
	pr_err("vivo-haptic enable gpio: %d \n", chip->enable_gpio);

	return 0;
}

/* Dummy functions for brightness */
static enum led_brightness qpnp_vib_brightness_get(struct led_classdev *cdev)
{
	return 0;
}

static void qpnp_vib_brightness_set(struct led_classdev *cdev,
			enum led_brightness level)
{
}

static int qpnp_vibrator_ldo_suspend(struct device *dev)
{
	struct vib_ldo_chip *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->lock);

	hrtimer_cancel(&chip->stop_timer);
	cancel_work_sync(&chip->vib_work);
	qpnp_vib_ldo_enable(chip, false);
	mutex_unlock(&chip->lock);

	return 0;
}
static SIMPLE_DEV_PM_OPS(qpnp_vibrator_ldo_pm_ops, qpnp_vibrator_ldo_suspend,
			NULL);

static int qpnp_vibrator_ldo_probe(struct platform_device *pdev)
{
	struct vib_ldo_chip *chip;
	int i, ret;

	pr_err("%s begin\n", __func__);

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	ret = qpnp_vib_parse_dt(&pdev->dev, chip);
	if (ret < 0) {
		pr_err("couldn't parse device tree, ret=%d\n", ret);
		return ret;
	}

	chip->vib_play_ms = QPNP_VIB_PLAY_MS;
	mutex_init(&chip->lock);
	INIT_WORK(&chip->vib_work, qpnp_vib_work);

	hrtimer_init(&chip->stop_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	chip->stop_timer.function = vib_stop_timer;
	dev_set_drvdata(&pdev->dev, chip);

	chip->cdev.name = "vibrator";
	chip->cdev.brightness_get = qpnp_vib_brightness_get;
	chip->cdev.brightness_set = qpnp_vib_brightness_set;
	chip->cdev.max_brightness = 100;
	ret = devm_led_classdev_register(&pdev->dev, &chip->cdev);
	if (ret < 0) {
		pr_err("Error in registering led class device, ret=%d\n", ret);
		goto fail;
	}

	for (i = 0; i < ARRAY_SIZE(qpnp_vib_attrs); i++) {
		ret = sysfs_create_file(&chip->cdev.dev->kobj,
				&qpnp_vib_attrs[i].attr);
		if (ret < 0) {
			dev_err(&pdev->dev, "Error in creating sysfs file, ret=%d\n",
				ret);
			goto sysfs_fail;
		}
	}

	pr_err("%s success\n", __func__);
	return 0;

sysfs_fail:
	for (--i; i >= 0; i--)
		sysfs_remove_file(&chip->cdev.dev->kobj,
				&qpnp_vib_attrs[i].attr);
fail:
	mutex_destroy(&chip->lock);
	dev_set_drvdata(&pdev->dev, NULL);
	return ret;
}

static int qpnp_vibrator_ldo_remove(struct platform_device *pdev)
{
	struct vib_ldo_chip *chip = dev_get_drvdata(&pdev->dev);

	hrtimer_cancel(&chip->stop_timer);
	cancel_work_sync(&chip->vib_work);
	mutex_destroy(&chip->lock);
	dev_set_drvdata(&pdev->dev, NULL);

	return 0;
}

static const struct of_device_id vibrator_ldo_match_table[] = {
	{ .compatible = "qcom,qpnp-vibrator-gpio" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, vibrator_ldo_match_table);

static struct platform_driver qpnp_vibrator_ldo_driver = {
	.driver	= {
		.name		= "qcom,qpnp-vibrator-gpio",
		.of_match_table	= vibrator_ldo_match_table,
		.pm		= &qpnp_vibrator_ldo_pm_ops,
	},
	.probe	= qpnp_vibrator_ldo_probe,
	.remove	= qpnp_vibrator_ldo_remove,
};
module_platform_driver(qpnp_vibrator_ldo_driver);

MODULE_DESCRIPTION("QPNP Vibrator-GPIO driver");
MODULE_LICENSE("GPL v2");
