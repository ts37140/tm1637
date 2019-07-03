// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2019, Tero Salminen
 */

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/device.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#include <asm-generic/errno.h>

#include <linux/gpio/consumer.h>
#include <asm/delay.h>
#include <linux/delay.h>

#define DRIVER_NAME "tm1637"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("TERO SALMINEN");

#define TM1637_BITBANG_DELAY()		usleep_range(100, 300)

/* tm1637 can drive 6 pcs 7-segment leds */
#define TM1637_GRID_NUM			6U

#define TM1637_DISP_CTRL_OFF		0x81	/* B1000_0000*/
#define TM1637_DISP_CTRL_ON_MIN		0x88	/* B1000_1000*/
#define TM1637_DISP_CTRL_ON_MED		0x8C	/* B1000_1100*/
#define TM1637_DISP_CTRL_ON_MAX		0x8F	/* B1000_1111*/

#define TM1637_WRITE_DATA_AUTO_ADDR	0x40	/* B0100_0000 */
#define TM1637_ADDR_DISP_C0H		0xC0	/* B1100_0000*/

#define TM1637_7_SEGMENT_EMPTY		0x0
#define TM1637_7_SEGMENT_ALL_ON		0xFF
#define TM1637_7_SEGMENT_MINUS		0x40

/* Characters for seven segment digits */
static const uint8_t SEGMENT_CHAR[] = {
	0x3f,	/* 0 */
	0x6,	/* 1 */
	0x5b,	/* 2 */
	0x4f,	/* 3 */
	0x66,	/* 4 */
	0x6d,	/* 5 */
	0x7d,	/* 6 */
	0x7,	/* 7 */
	0x7f,	/* 8 */
	0x6F,	/* 9 */
};

#define TM1637_SELFTEST_ON_TIME		1000UL	/* 1 sec */

enum tm1637_status {
	TM1637_STATUS_LOADED		= 0,
	TM1637_STATUS_SELFTEST_OK,
	TM1637_STATUS_SELFTEST_FAIL,
	TM1637_STATUS_OK,
	TM1637_STATUS_ERROR,
};

struct tm1637_gpios {
	struct gpio_desc *pwr;
	struct gpio_desc *dio;
	struct gpio_desc *clk;
};

struct drv_priv {
	enum tm1637_status chip_status;
	struct tm1637_gpios gpio;
	uint8_t segment[TM1637_GRID_NUM];
};

#define TM1637_ASCII_OFFSET		48 /* 0 */
#define TM1637_ASCII_LAST_DIGIT		57 /* 9 */
#define TM1637_ASCII_MINUS		0x2D /* - */
#define TM1637_ASCII_DOT		0x2E /* . */

/*
 * Convert string to seven segment characters. Supported characters are decimal
 * numbers and '-'.
 * 
 * '.'-character marks segment off.
 * 
 * Conversion stops after 6 characters (TM1637_GRID_NUM) and rest characters
 * are discarded.
 * 
 * Conversion stops for invalid character without raising error.
 */
static void tm1637_str_to_digits(struct device *dev, const char* value,
						uint8_t *digits)
{
	int i = 0;
	uint32_t len = min(strlen(value), TM1637_GRID_NUM);

	memset(digits, TM1637_7_SEGMENT_EMPTY, TM1637_GRID_NUM);

	for (i = 0; i < len; i++) {
		if (value[i] == TM1637_ASCII_MINUS) {
			digits[i] = TM1637_7_SEGMENT_MINUS;
			continue;
		}

		if (value[i] == TM1637_ASCII_DOT) {
			digits[i] = TM1637_7_SEGMENT_EMPTY;
			continue;
		}

		if (value[i] < TM1637_ASCII_OFFSET ||
		    value[i] > TM1637_ASCII_LAST_DIGIT)
			break;

		digits[i] = SEGMENT_CHAR[value[i] - TM1637_ASCII_OFFSET];
	}

	return;
}

/*
 * In data input mode chip reads data from DIO pin on raising edge of
 * CLK signal.
 *
 * DIO value should be set when CLK signal is low and should not change
 * when CLK signal is high.
 *
 * Chip will acknowledge end of successful byte writing sequence. DIO line
 * is pulled down on falling edge of CLK signal after 8th bit and released
 * on the falling edge of next CLK signal.
 *
 * Pin states on function entry:
 *   -CLK low
 *
 * Pin states on function exit:
 *   -CLK low
 *   -DIO low
 */
static int tm1637_write_8_bits(struct device *dev, uint8_t cmd)
{
	struct drv_priv *priv = dev_get_drvdata(dev);
	int i = 0;
	uint8_t pin = 0;

	for (i = 0; i < 8; i++) {
		TM1637_BITBANG_DELAY();

		if ((cmd >> i) & 0x1)
			gpiod_set_value(priv->gpio.dio, 1);
		else
			gpiod_set_value(priv->gpio.dio, 0);
		TM1637_BITBANG_DELAY();

		gpiod_set_value(priv->gpio.clk, 1);

		TM1637_BITBANG_DELAY();
		TM1637_BITBANG_DELAY();

		gpiod_set_value(priv->gpio.clk, 0);
	}

	/* Check chip ack */
	gpiod_direction_input(priv->gpio.dio);

	TM1637_BITBANG_DELAY();

	pin = gpiod_get_value(priv->gpio.dio);
	if (pin == 1) {
		dev_err(dev, "ACK error: line high\n");
		return -EIO;
	}

	gpiod_set_value(priv->gpio.clk, 1);

	TM1637_BITBANG_DELAY();

	gpiod_set_value(priv->gpio.clk, 0);

	TM1637_BITBANG_DELAY();

	pin = gpiod_get_value(priv->gpio.dio);
	if (pin == 0) {
		dev_err(dev, "ACK error: line low\n");
		return -EIO;
	}

	gpiod_direction_output(priv->gpio.dio, 0);

	return 0;
}

/*
 * Data input mode is ended with DIO raising edge when CLK is high.
 *
 * Pin states on function entry:
 *   -CLK low
 *   -DIO low
 *
 * Pin states on function exit:
 *   -CLK high
 *   -DIO high
 */

static void tm1637_data_input_exit(struct device *dev)
{
	struct drv_priv *priv = dev_get_drvdata(dev);

	TM1637_BITBANG_DELAY();
	gpiod_set_value(priv->gpio.clk, 1);
	TM1637_BITBANG_DELAY();
	gpiod_set_value(priv->gpio.dio, 1);
	TM1637_BITBANG_DELAY();
}

/*
 * Chip is set to data input mode with DIO falling edge when CLK is high.
 *
 * Pin states on function entry:
 *   -CLK high
 *   -DIO high
 *
 * Pin states on function exit:
 *   -CLK low
 *   -DIO low
 */
static void tm1637_data_input_enter(struct device *dev)
{
	struct drv_priv *priv = dev_get_drvdata(dev);

	gpiod_set_value(priv->gpio.dio, 0);
	TM1637_BITBANG_DELAY();
	gpiod_set_value(priv->gpio.clk, 0);
}

/*
 * Write display control command.
 *
 */
static int tm1637_set_disp_ctrl(struct device *dev, uint8_t cmd)
{
	int res = 0;

	tm1637_data_input_enter(dev);

	res = tm1637_write_8_bits(dev, cmd);

	if (res)
		return res;

	tm1637_data_input_exit(dev);

	return res;
}

/*
 * Write seven segment led configuration to chip.
 */
static int tm1637_write_seven_seg_config(struct device *dev)
{
	int res = 0;
	struct drv_priv *priv = dev_get_drvdata(dev);

	tm1637_data_input_enter(dev);

	res = tm1637_write_8_bits(dev, TM1637_WRITE_DATA_AUTO_ADDR);
	if (res)
		return res;

	tm1637_data_input_exit(dev);

	tm1637_data_input_enter(dev);

	res = tm1637_write_8_bits(dev, TM1637_ADDR_DISP_C0H);

	res = tm1637_write_8_bits(dev, priv->segment[0]);
	res = tm1637_write_8_bits(dev, priv->segment[1]);
	res = tm1637_write_8_bits(dev, priv->segment[2]);
	res = tm1637_write_8_bits(dev, priv->segment[3]);
	res = tm1637_write_8_bits(dev, priv->segment[4]);
	res = tm1637_write_8_bits(dev, priv->segment[5]);

	tm1637_data_input_exit(dev);

	return res;
}

/*
 * Chip power is toggled off/on and data input mode exit sequence
 * is executed. Chip is ready to start data input mode after reset.
 *
 * Pin states on function exit:
 *  -CLK high
 *  -DIO high
 */
static void tm1637_reset(struct device *dev)
{
	struct drv_priv *priv = dev_get_drvdata(dev);

	dev_info(dev, "reset\n");

	gpiod_set_value(priv->gpio.pwr, 0);
	gpiod_set_value(priv->gpio.clk, 0);
	gpiod_set_value(priv->gpio.dio, 0);

	TM1637_BITBANG_DELAY();

	gpiod_set_value(priv->gpio.pwr, 1);

	TM1637_BITBANG_DELAY();

	tm1637_data_input_exit(dev);
}

/*
 * Selftest verifies communication is working with the chip
 * by setting all leds active and switching display on and off.
 */
static void tm1637_selftest(struct device *dev)
{
	int res = 0;
	struct drv_priv *priv = dev_get_drvdata(dev);

	priv->chip_status = TM1637_STATUS_SELFTEST_FAIL;

	tm1637_reset(dev);

	memset(priv->segment, TM1637_7_SEGMENT_ALL_ON, TM1637_GRID_NUM);
	
	res = tm1637_write_seven_seg_config(dev);
	if (res) {
		dev_err(dev, "Selftest: segment write failed %d", res);
		return;
	}

	res = tm1637_set_disp_ctrl(dev, TM1637_DISP_CTRL_ON_MED);
	if (res) {
		dev_err(dev, "Selftest: display ON failed %d", res);
		return;
	}

	msleep_interruptible(TM1637_SELFTEST_ON_TIME);

	res = tm1637_set_disp_ctrl(dev, TM1637_DISP_CTRL_OFF);
	if (res) {
		dev_err(dev, "Selftest: display OFF failed %d", res);
		return;
	}

	priv->chip_status = TM1637_STATUS_SELFTEST_OK;

	dev_info(dev, "Selftest: OK\n");
}

/*
 * Set digits shown in the display.
 */
static ssize_t set_digits_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	int res =  0;
	struct drv_priv *priv = dev_get_drvdata(dev);

	priv->chip_status = TM1637_STATUS_ERROR;

	tm1637_str_to_digits(dev, buf, priv->segment);

	res = tm1637_write_seven_seg_config(dev);
	if (res)
		return res;

	res = tm1637_set_disp_ctrl(dev, TM1637_DISP_CTRL_ON_MED);
	if (res)
		return res;

	priv->chip_status = TM1637_STATUS_OK;

	return count;
}

static ssize_t get_status_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	int res = 0;
	struct drv_priv *priv = dev_get_drvdata(dev);

	switch (priv->chip_status) {
	case TM1637_STATUS_LOADED:
		res = sprintf(buf, "Status: module loaded\n");
		break;

	case TM1637_STATUS_SELFTEST_OK:
		res = sprintf(buf, "Status: selftest OK\n");
		break;

	case TM1637_STATUS_SELFTEST_FAIL:
		res = sprintf(buf, "Status: selftest FAILED\n");
		break;

	case TM1637_STATUS_OK:
		res = sprintf(buf, "Status: OK\n");
		break;
	
	case TM1637_STATUS_ERROR:
		res = sprintf(buf, "Status: ERROR\n");
		break;

	default:
		dev_err(dev, "Unknown status\n");
		res = -EIO;
	}

	return res;
}

static DEVICE_ATTR_WO(set_digits);
static DEVICE_ATTR_RO(get_status);

static struct attribute *tm1637_attributes_attrs[] = {
	&dev_attr_set_digits.attr,
	&dev_attr_get_status.attr,
	NULL,
};

ATTRIBUTE_GROUPS(tm1637_attributes);

static int tm1637_setup_gpios(struct platform_device *pdev)
{
	int res = 0;
	struct drv_priv *priv = platform_get_drvdata(pdev);

	priv->gpio.pwr = devm_gpiod_get(&pdev->dev, "pwr", GPIOD_OUT_LOW);
	if (IS_ERR(priv->gpio.pwr)) {
		res = PTR_ERR(priv->gpio.pwr);
		dev_err(&pdev->dev, "gpio pwr error %d\n", res);
		return res;
	}

	priv->gpio.dio = devm_gpiod_get(&pdev->dev, "data", GPIOD_OUT_LOW);
	if (IS_ERR(priv->gpio.dio)) {
		res = PTR_ERR(priv->gpio.dio);
		dev_err(&pdev->dev, "gpio dio error %d\n", res);
		return res;
	}

	priv->gpio.clk = devm_gpiod_get(&pdev->dev, "clk", GPIOD_OUT_LOW);
	if (IS_ERR(priv->gpio.clk)) {
		res = PTR_ERR(priv->gpio.clk);
		dev_err(&pdev->dev, "gpio clk error %d\n", res);
		return res;
	}

	return res;
}

static int tm1637_probe(struct platform_device *pdev)
{
	int res = 0;
	struct drv_priv *priv = NULL;
	char *kobject_name = NULL;

	dev_info(&pdev->dev, "probe\n");

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);

	res = tm1637_setup_gpios(pdev);
	if (res)
		return res;

	kobject_name = kobject_get_path(&pdev->dev.kobj, GFP_KERNEL);

	res = devm_device_add_groups(&pdev->dev, tm1637_attributes_groups);
	if (res) {
		dev_err(&pdev->dev, "dev add groups %d\n", res);
		return res;
	}

	tm1637_selftest(&pdev->dev);

	return res;
}

static int tm1637_remove(struct platform_device *pdev)
{
	struct drv_priv *priv = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "remove\n");

	gpiod_set_value(priv->gpio.pwr, 0);
	gpiod_set_value(priv->gpio.clk, 0);
	gpiod_set_value(priv->gpio.dio, 0);

	return 0;
}

static const struct of_device_id tm1637_dt_ids[] = {
	{ .compatible = "titan,tm1637", },
	{ }
};
MODULE_DEVICE_TABLE(of, tm1637_dt_ids);

static struct platform_driver tm1637_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table	= tm1637_dt_ids,
	},
	.probe = tm1637_probe,
	.remove = tm1637_remove,
};

module_platform_driver(tm1637_driver);
