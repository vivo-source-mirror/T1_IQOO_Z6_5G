/*
 * platform indepent driver interface
 *
 * Coypritht (c) 2017 Goodix
 */
#define pr_fmt(fmt)		"[FP_KERN] " KBUILD_MODNAME ": " fmt
 
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>

#include "gf9518_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

int gf9518_parse_dts(struct gf_dev *gf_dev)
{
	int rc = 0;

	gf_dev->vdd_use_gpio = of_property_read_bool(gf_dev->spi->dev.of_node, "goodix,vdd_use_gpio");
	gf_dev->vdd_use_pmic = of_property_read_bool(gf_dev->spi->dev.of_node, "goodix,vdd_use_pmic");
	gf_dev->vddio_use_gpio = of_property_read_bool(gf_dev->spi->dev.of_node, "goodix,vddio_use_gpio");
	gf_dev->vddio_use_pmic = of_property_read_bool(gf_dev->spi->dev.of_node, "goodix,vddio_use_pmic");
    gf_dev->disable_vddio_pmic = of_property_read_bool(gf_dev->spi->dev.of_node, "goodix,disable_vddio");

	pr_info("%s, vdd_use_gpio %d", __func__, gf_dev->vdd_use_gpio);
	pr_info("%s, vdd_use_pmic %d", __func__, gf_dev->vdd_use_pmic);
	pr_info("%s, vddio_use_gpio %d", __func__, gf_dev->vddio_use_gpio);
	pr_info("%s, vddio_use_pmic %d", __func__, gf_dev->vddio_use_pmic);
    pr_info("%s, disable_vddio_pmic %d", __func__, gf_dev->disable_vddio_pmic);

	// VDD
	if (gf_dev->vdd_use_gpio) {
		gf_dev->vdd_en_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node, "goodix,gpio_vdd_en", 0);
		if (!gpio_is_valid(gf_dev->vdd_en_gpio)) {
			pr_err("%s, VDD GPIO is invalid", __func__);
			return -EIO;
		} else {
			pr_info("%s, VDD GPIO is %d", __func__, gf_dev->vdd_en_gpio);
		}
		rc = gpio_request(gf_dev->vdd_en_gpio, "goodix_vdd_en");
		if (rc) {
			pr_err("%s, Failed to request VDD GPIO, rc = %d", __func__, rc);
			return -EIO;
		} else {
			pr_info("%s, Success to request VDD GPIO", __func__);
		}
	}

	if (gf_dev->vdd_use_pmic) {
		gf_dev->vdd = regulator_get(&gf_dev->spi->dev, "vdd");
		if (IS_ERR(gf_dev->vdd)) {
			pr_err("%s, Unable to get vdd", __func__);
			return -EINVAL;
		} else {
			pr_info("%s, Success to get vdd", __func__);
		}
	}

	// VDDIO
	if (gf_dev->vddio_use_gpio) {
		gf_dev->vddio_en_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node, "goodix,gpio_vddio_en", 0);
		if (!gpio_is_valid(gf_dev->vddio_en_gpio)) {
			pr_err("%s, VDDIO GPIO is invalid", __func__);
			return -EIO;
		} else {
			pr_info("%s, VDDIO GPIO is %d", __func__, gf_dev->vddio_en_gpio);
		}
		rc = gpio_request(gf_dev->vddio_en_gpio, "goodix_vddio_en");
		if (rc) {
			pr_err("%s, Failed to request VDDIO GPIO, rc = %d", __func__, rc);
			return -EIO;
		} else {
			pr_info("%s, Success to request VDDIO GPIO", __func__);
		}
	}

	if (gf_dev->vddio_use_pmic) {
		gf_dev->vddio = regulator_get(&gf_dev->spi->dev, "vddio");
		if (IS_ERR(gf_dev->vddio)) {
		pr_info("%s, Unable to get vddio", __func__);
			return -EINVAL;
		} else {
			pr_info("%s, Success to get vddio", __func__);
		}
	}
	// Disable vddio for PD2056
	if (gf_dev->disable_vddio_pmic) {
		gf_dev->disable_vddio = regulator_get(&gf_dev->spi->dev, "disable_vddio");
		if (IS_ERR(gf_dev->disable_vddio)) {
			pr_info("%s, Unable to get disable_vddio", __func__);
			return -EINVAL;
		} else {
			pr_info("%s, Success to get disable_vddio", __func__);
		}
		rc = regulator_set_load(gf_dev->disable_vddio, 0);
		if (rc < 0) {
			pr_err("%s, vdd regulator_set_load(uA_load = %d) failed, rc = %d", __func__, 0, rc);
		}
		regulator_disable(gf_dev->disable_vddio);
		pr_info("%s, disable  vddio %d", __func__, rc);
	}

	/*get reset resource */
	gf_dev->reset_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node, "goodix,gpio_reset", 0);
	if (!gpio_is_valid(gf_dev->reset_gpio)) {
		pr_err("%s, RESET GPIO is invalid", __func__);
		return -EIO;
	}

	rc = gpio_request(gf_dev->reset_gpio, "goodix_reset");
	if (rc) {
		pr_err("%s, Failed to request RESET GPIO, rc = %d", __func__, rc);
		return -EIO;
	}

	gpio_direction_output(gf_dev->reset_gpio, 0);

	/*get irq resourece */
	gf_dev->irq_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node, "goodix,gpio_irq", 0);
	pr_info("%s, irq_gpio:%d", __func__, gf_dev->irq_gpio);
	if (!gpio_is_valid(gf_dev->irq_gpio)) {
		pr_err("%s, IRQ GPIO is invalid", __func__);
		return -EIO;
	}

	rc = gpio_request(gf_dev->irq_gpio, "goodix_irq");
	if (rc) {
		pr_info("%s, Failed to request IRQ GPIO, rc = %d", __func__, rc);
		return -EIO;
	}
	gpio_direction_input(gf_dev->irq_gpio);
	pr_info("%s, exit", __func__);
	return 0;
}

void gf9518_cleanup(struct gf_dev *gf_dev)
{
	pr_info("%s", __func__);
	if (gpio_is_valid(gf_dev->irq_gpio)) {
		gpio_free(gf_dev->irq_gpio);
		pr_info("%s, remove irq_gpio success", __func__);
	}
	if (gpio_is_valid(gf_dev->reset_gpio)) {
		gpio_free(gf_dev->reset_gpio);
		pr_info("%s, remove reset_gpio success", __func__);
	}
}

int gf9518_power_on(struct gf_dev *gf_dev)
{
	int rc = 0;

    // VDD ON
    if (gf_dev->vdd_use_gpio) {
		rc = gpio_direction_output(gf_dev->vdd_en_gpio, 1);
		if (rc) {
		pr_err("%s, vdd power on fail", __func__);
		return -EIO;
		}
	}

	if (gf_dev->vdd_use_pmic) {
		if (regulator_is_enabled(gf_dev->vdd)) {
			pr_info("%s, vdd is on, don't set repeatedly!", __func__);
			return rc;
		}

		rc = regulator_set_load(gf_dev->vdd, 600000);
		if (rc < 0) {
			pr_err("%s, vdd regulator_set_load(uA_load = %d) failed, rc = %d", __func__, 1000, rc);
		}

		if (regulator_count_voltages(gf_dev->vdd) > 0) {
			rc = regulator_set_voltage(gf_dev->vdd, 3300000, 3300000);
			if (rc) {
				pr_err("%s, Unable to set voltage on vdd", __func__);
				return rc;
			}
		}
		rc = regulator_enable(gf_dev->vdd);
	}

	// VDDIO ON
	if (gf_dev->vddio_use_gpio) {
		rc = gpio_direction_output(gf_dev->vddio_en_gpio, 1);
		if (rc) {
			pr_err("%s, vddio power on fail", __func__);
			return -EIO;
		}
	}

	if (gf_dev->vddio_use_pmic) {
		if (regulator_is_enabled(gf_dev->vddio)) {
			pr_info("%s, vddio is on, don't set repeatedly!", __func__);
			return rc;
		}

		rc = regulator_set_load(gf_dev->vddio, 600000);
		if (rc < 0) {
			pr_err("%s, vddio regulator_set_load(uA_load = %d) failed, rc = %d", __func__, 1000, rc);
		}

		if (regulator_count_voltages(gf_dev->vddio) > 0) {
			rc = regulator_set_voltage(gf_dev->vddio, 1800000, 1800000);
			if (rc) {
				pr_err("%s, Unable to set voltage on vddio", __func__);
				return rc;
			}
		}
		rc = regulator_enable(gf_dev->vddio);
	}

	pr_info("%s, exit", __func__);

	return rc;
}

int gf9518_power_off(struct gf_dev *gf_dev)
{
	int rc = 0;

	rc = gpio_direction_output(gf_dev->reset_gpio, 0);
	if (rc) {
		pr_info("%s, set reset gpio output fail", __func__);
		return -EIO;
	}

	// VDD OFF
	if (gf_dev->vdd_use_gpio) {
		rc = gpio_direction_output(gf_dev->vdd_en_gpio, 0);
		if (rc) {
			pr_info("%s, vdd power off fail", __func__);
			return -EIO;
		}
	}

	if (gf_dev->vdd_use_pmic) {
		rc = regulator_set_load(gf_dev->vdd, 0);
		if (rc < 0) {
			pr_err("%s, vdd regulator_set_load(uA_load = %d) failed, rc = %d", __func__, 0, rc);
		}
		if (regulator_is_enabled(gf_dev->vdd)) {
			regulator_disable(gf_dev->vdd);
		}
		pr_info("%s, disable  vdd %d", __func__, rc);
	}

	// VDDIO OFF
	if (gf_dev->vddio_use_gpio) {
		rc = gpio_direction_output(gf_dev->vddio_en_gpio, 0);
		if (rc) {
			pr_err("%s, vddio power off fail", __func__);
			return -EIO;
		}
	}

	if (gf_dev->vddio_use_pmic) {
	rc = regulator_set_load(gf_dev->vddio, 0);
		if (rc < 0) {
			pr_err("%s, vddio regulator_set_load(uA_load=%d) failed, rc = %d", __func__, 0, rc);
		}
		if (regulator_is_enabled(gf_dev->vddio)) {
			regulator_disable(gf_dev->vddio);
		}
		pr_info("%s, disable  vddio %d", __func__, rc);
	}

	pr_info("%s, exit", __func__);

	return rc;
}

int gf9518_hw_get_power_state(struct gf_dev *gf_dev)
{

	int retval = 0;

	retval = gpio_get_value(gf_dev->vdd_en_gpio);
	pr_info("%s, retval = %d", __func__, retval);

	return retval;
}

int gf9518_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
	if (gf_dev == NULL) {
		pr_err("%s, Input buff is NULL", __func__);
		return -EFAULT;
	}
	gpio_direction_output(gf_dev->reset_gpio, 0);
	mdelay(10);
	gpio_set_value(gf_dev->reset_gpio, 1);
	mdelay(delay_ms);
	return 0;
}

int gf9518_irq_num(struct gf_dev *gf_dev)
{
	if (gf_dev == NULL) {
		pr_err("%s, Input buff is NULL", __func__);
		return -EFAULT;
	} else {
		return gpio_to_irq(gf_dev->irq_gpio);
	}
}

