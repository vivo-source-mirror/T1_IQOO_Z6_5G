#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/delay.h>

/*****************************************************************************
 * Define
 *****************************************************************************/
#define DDIC_NAME "ddic"
#define QRD_LCD_VPOS_ADDRESS 0x00
#define QRD_LCD_VNEG_ADDRESS 0x01
#define QRD_LCD_BIAS_ADDRESS 0x03

static struct i2c_client *lcm_ddic_client;


static int lcm_i2c_read_reg(struct i2c_client *client, 
	unsigned reg, uint8_t *value)
{
	uint8_t buf[1];
	int ret = 0;

	if (!client) {
		printk("%s: client is NULL\n", __func__);
		return -EINVAL;
	}

	if (!value)
		return -EINVAL;
	buf[0] = reg;
	ret = i2c_master_send(client, buf, 1);
	if (ret > 0) {
		msleep_interruptible(1);
		ret = i2c_master_recv(client, buf, 1);
		if (ret > 0)
			*value = buf[0];
	}
	return ret;
}

static int lcm_i2c_write_reg(struct i2c_client *client, 
	unsigned reg, uint8_t value, const char *caller)
{
	uint8_t buf[2] = {reg,  value};
	int ret = 0;
	
	if (!client) {
		printk("%s: client is NULL\n", __func__);
		return -EINVAL;
	}
	ret = i2c_master_send(client, buf, 2);
	if (ret < 0)
		printk("%s: i2c_master_send error %d\n", 
			caller, ret);
	return ret;
}

static int bias_ic_id_read (struct i2c_client *client)
{
	uint8_t buf[1];
	int ret = 0;
	ret = lcm_i2c_read_reg(client, QRD_LCD_VPOS_ADDRESS, buf);
	pr_info("read id ret =%d, bias ic pos =0x%x", ret, buf[0]);
	return ret;
}

extern int lcm_error_code_report(int error_code);
void lcm_bias_set_avdd_n_avee(int level)
{
	int avdd = 0x0f;
	int avee = 0x0f;
	int result,result2,result3;

	switch (level) {
	case 50:
		avdd = 0x0a;
		avee = 0x0a;
		break;
	case 58:
		avdd = 0x12;
		avee = 0x12;
		break;
	case 60:
		avdd = 0x14;
		avee = 0x14;
		break;
	case 55:
	default:
		avdd = 0x0f;
		avee = 0x0f;
		break;
	}
	result = lcm_i2c_write_reg(lcm_ddic_client, QRD_LCD_VPOS_ADDRESS, avdd, __func__);
	result2 = lcm_i2c_write_reg(lcm_ddic_client, QRD_LCD_VNEG_ADDRESS, avee, __func__);
	result3 = lcm_i2c_write_reg(lcm_ddic_client, QRD_LCD_BIAS_ADDRESS, 0x4F, __func__);
	pr_info("%s: result_temp=%#x\n", __func__, result);
	return;
}
EXPORT_SYMBOL(lcm_bias_set_avdd_n_avee);

static int lcm_ddic_probe (struct i2c_client *client, 
	const struct i2c_device_id *id)
{
	int ret = 0;

	pr_info("%s: enter \n", __func__);
	
	pr_info("%s: enter, I2C address = 0x%x, flags = 0x%x\n", 
		__func__, client->addr, client->flags);

	lcm_ddic_client = client;

	/* We should be able to read and write byte data */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk ("%s: I2C_FUNC_I2C not supported\n", 
			__func__);
		return -ENOTSUPP;
	}

	ret = bias_ic_id_read(client);
	pr_info("result_temp=%#x\n", ret);
	return 0;
}

static int lcm_ddic_remove(struct i2c_client *client)
{
	pr_info("exit  lcm_ddic_i2c_remove\n");
	lcm_ddic_client = NULL;
	i2c_unregister_device(client);
	return 0;
}

static const struct i2c_device_id ddic_id[] = {
	{ DDIC_NAME,  0 }, 
	{ }
};

static struct of_device_id ddic_match_table[] = {
	{ .compatible = "ddic_i2c", }, 
	{ },
};

static struct i2c_driver lcm_ddic_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = DDIC_NAME,
		.of_match_table = ddic_match_table,
	},
	.id_table = ddic_id,
	.probe = lcm_ddic_probe,
	.remove = lcm_ddic_remove,
};

module_i2c_driver(lcm_ddic_driver);
MODULE_DESCRIPTION("LCM_DDIC DISPLAY BIAS DRIVER");
MODULE_LICENSE("GPL v2");
