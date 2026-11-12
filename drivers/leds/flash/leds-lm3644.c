// SPDX-License-Identifier: GPL-2.0
// Flash and torch driver for the Texas Instruments LM3644 dual LED flash driver

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/led-class-flash.h>
#include <linux/leds.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/regmap.h>

#define LM3644_ENABLE_REG		0x01
#define LM3644_LED1_FLASH_REG		0x03
#define LM3644_LED2_FLASH_REG		0x04
#define LM3644_LED1_TORCH_REG		0x05
#define LM3644_LED2_TORCH_REG		0x06
#define LM3644_BOOST_REG		0x07
#define LM3644_TIMING_REG		0x08
#define LM3644_FLAGS1_REG		0x0a
#define LM3644_FLAGS2_REG		0x0b
#define LM3644_DEVICE_ID_REG		0x0c

#define LM3644_LED1_EN			BIT(0)
#define LM3644_LED2_EN			BIT(1)
#define LM3644_MODE_MASK		GENMASK(3, 2)
#define LM3644_MODE_STANDBY		0
#define LM3644_MODE_TORCH		2
#define LM3644_MODE_FLASH		3
#define LM3644_TX_PIN_EN		BIT(7)

#define LM3644_LED2_OVERRIDE		BIT(7)
#define LM3644_BRIGHTNESS_MASK		GENMASK(6, 0)

#define LM3644_SW_RESET			BIT(7)

#define LM3644_FLASH_TIMEOUT_MASK	GENMASK(3, 0)

#define LM3644_DEVICE_ID_MASK		GENMASK(5, 3)

#define LM3644_FLAGS1_TX		BIT(7)
#define LM3644_FLAGS1_VOUT_SHORT	BIT(6)
#define LM3644_FLAGS1_VLED1_SHORT	BIT(5)
#define LM3644_FLAGS1_VLED2_SHORT	BIT(4)
#define LM3644_FLAGS1_CURRENT_LIMIT	BIT(3)
#define LM3644_FLAGS1_TSD		BIT(2)
#define LM3644_FLAGS1_UVLO		BIT(1)
#define LM3644_FLAGS1_TIMEOUT		BIT(0)

#define LM3644_FLAGS2_IVFM		BIT(2)
#define LM3644_FLAGS2_OVP		BIT(1)

#define LM3644_TORCH_MIN_UA		977
#define LM3644_TORCH_STEP_UA		1400
#define LM3644_TORCH_MAX_UA		179000

#define LM3644_FLASH_MIN_UA		10900
#define LM3644_FLASH_STEP_UA		11725
#define LM3644_FLASH_MAX_UA		1500000

#define LM3644_TIMEOUT_MIN_US		10000
#define LM3644_TIMEOUT_LOWER_STEP_US	10000
#define LM3644_TIMEOUT_UPPER_STEP_US	50000
#define LM3644_TIMEOUT_XOVER_US		100000
#define LM3644_TIMEOUT_MAX_US		400000

struct lm3644 {
	struct led_classdev_flash fled_cdev;
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *enable_gpio;
	/* Serializes register access between torch and flash paths */
	struct mutex lock;

	u8 led_en;
	u32 torch_current_max;
	u32 flash_current_max;
	u32 flash_timeout_max;
};

static const struct regmap_config lm3644_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = LM3644_DEVICE_ID_REG,
};

static struct lm3644 *fled_cdev_to_lm3644(struct led_classdev_flash *fled_cdev)
{
	return container_of(fled_cdev, struct lm3644, fled_cdev);
}

static int lm3644_read_faults(struct lm3644 *chip, u32 *fault)
{
	unsigned int flags1, flags2;
	int ret;

	ret = regmap_read(chip->regmap, LM3644_FLAGS1_REG, &flags1);
	if (ret)
		return ret;

	ret = regmap_read(chip->regmap, LM3644_FLAGS2_REG, &flags2);
	if (ret)
		return ret;

	*fault = 0;

	if (flags1 & LM3644_FLAGS1_TIMEOUT)
		*fault |= LED_FAULT_TIMEOUT;

	if (flags1 & LM3644_FLAGS1_UVLO)
		*fault |= LED_FAULT_UNDER_VOLTAGE;

	if (flags1 & LM3644_FLAGS1_TSD)
		*fault |= LED_FAULT_OVER_TEMPERATURE;

	if (flags1 & LM3644_FLAGS1_CURRENT_LIMIT)
		*fault |= LED_FAULT_OVER_CURRENT;

	if (flags1 & (LM3644_FLAGS1_VOUT_SHORT | LM3644_FLAGS1_VLED1_SHORT |
		      LM3644_FLAGS1_VLED2_SHORT))
		*fault |= LED_FAULT_SHORT_CIRCUIT;

	if (flags2 & LM3644_FLAGS2_OVP)
		*fault |= LED_FAULT_OVER_VOLTAGE;

	if (flags2 & LM3644_FLAGS2_IVFM)
		*fault |= LED_FAULT_INPUT_VOLTAGE;

	return 0;
}

static int lm3644_set_mode(struct lm3644 *chip, unsigned int mode)
{
	unsigned int val = FIELD_PREP(LM3644_MODE_MASK, mode);

	if (mode != LM3644_MODE_STANDBY)
		val |= chip->led_en;

	return regmap_update_bits(chip->regmap, LM3644_ENABLE_REG,
				  LM3644_MODE_MASK | LM3644_LED1_EN | LM3644_LED2_EN,
				  val);
}

static int lm3644_write_brightness(struct lm3644 *chip, unsigned int led1_reg,
				   unsigned int led2_reg, unsigned int code)
{
	int ret;

	if (chip->led_en & LM3644_LED1_EN) {
		unsigned int val = code;

		if (chip->led_en & LM3644_LED2_EN)
			val |= LM3644_LED2_OVERRIDE;

		return regmap_write(chip->regmap, led1_reg, val);
	}

	ret = regmap_write(chip->regmap, led1_reg, 0);
	if (ret)
		return ret;

	return regmap_write(chip->regmap, led2_reg, code);
}

static int lm3644_brightness_set(struct led_classdev *led_cdev,
				 enum led_brightness brightness)
{
	struct led_classdev_flash *fled_cdev = lcdev_to_flcdev(led_cdev);
	struct lm3644 *chip = fled_cdev_to_lm3644(fled_cdev);
	u32 fault;
	int ret;

	mutex_lock(&chip->lock);

	ret = lm3644_read_faults(chip, &fault);
	if (ret)
		goto out;

	if (brightness == LED_OFF) {
		ret = lm3644_set_mode(chip, LM3644_MODE_STANDBY);
		goto out;
	}

	ret = lm3644_write_brightness(chip, LM3644_LED1_TORCH_REG,
				      LM3644_LED2_TORCH_REG, brightness - 1);
	if (ret)
		goto out;

	ret = lm3644_set_mode(chip, LM3644_MODE_TORCH);
out:
	mutex_unlock(&chip->lock);
	return ret;
}

static int lm3644_flash_brightness_set(struct led_classdev_flash *fled_cdev,
				       u32 brightness)
{
	struct lm3644 *chip = fled_cdev_to_lm3644(fled_cdev);
	unsigned int code;
	int ret;

	code = (brightness - LM3644_FLASH_MIN_UA) / LM3644_FLASH_STEP_UA;

	mutex_lock(&chip->lock);
	ret = lm3644_write_brightness(chip, LM3644_LED1_FLASH_REG,
				      LM3644_LED2_FLASH_REG, code);
	mutex_unlock(&chip->lock);

	return ret;
}

static int lm3644_timeout_set(struct led_classdev_flash *fled_cdev,
			      u32 timeout)
{
	struct lm3644 *chip = fled_cdev_to_lm3644(fled_cdev);
	unsigned int code;
	int ret;

	if (timeout <= LM3644_TIMEOUT_XOVER_US)
		code = timeout / LM3644_TIMEOUT_LOWER_STEP_US - 1;
	else
		code = (timeout - LM3644_TIMEOUT_XOVER_US) /
		       LM3644_TIMEOUT_UPPER_STEP_US + 9;

	mutex_lock(&chip->lock);
	ret = regmap_update_bits(chip->regmap, LM3644_TIMING_REG,
				 LM3644_FLASH_TIMEOUT_MASK, code);
	mutex_unlock(&chip->lock);

	return ret;
}

static int lm3644_strobe_set(struct led_classdev_flash *fled_cdev, bool state)
{
	struct lm3644 *chip = fled_cdev_to_lm3644(fled_cdev);
	u32 fault;
	int ret;

	mutex_lock(&chip->lock);

	ret = lm3644_read_faults(chip, &fault);
	if (ret)
		goto out;

	ret = lm3644_set_mode(chip, state ? LM3644_MODE_FLASH : LM3644_MODE_STANDBY);
out:
	mutex_unlock(&chip->lock);
	return ret;
}

static int lm3644_strobe_get(struct led_classdev_flash *fled_cdev, bool *state)
{
	struct lm3644 *chip = fled_cdev_to_lm3644(fled_cdev);
	unsigned int val;
	int ret;

	mutex_lock(&chip->lock);
	ret = regmap_read(chip->regmap, LM3644_ENABLE_REG, &val);
	mutex_unlock(&chip->lock);
	if (ret)
		return ret;

	*state = FIELD_GET(LM3644_MODE_MASK, val) == LM3644_MODE_FLASH;

	return 0;
}

static int lm3644_fault_get(struct led_classdev_flash *fled_cdev, u32 *fault)
{
	struct lm3644 *chip = fled_cdev_to_lm3644(fled_cdev);
	int ret;

	mutex_lock(&chip->lock);
	ret = lm3644_read_faults(chip, fault);
	mutex_unlock(&chip->lock);

	return ret;
}

static const struct led_flash_ops lm3644_flash_ops = {
	.flash_brightness_set = lm3644_flash_brightness_set,
	.strobe_set = lm3644_strobe_set,
	.strobe_get = lm3644_strobe_get,
	.timeout_set = lm3644_timeout_set,
	.fault_get = lm3644_fault_get,
};

static int lm3644_parse_led(struct lm3644 *chip, struct fwnode_handle *child)
{
	u32 sources[2];
	int count, i;
	int ret;

	count = fwnode_property_count_u32(child, "led-sources");
	if (count <= 0) {
		chip->led_en = LM3644_LED1_EN | LM3644_LED2_EN;
	} else if (count > ARRAY_SIZE(sources)) {
		return dev_err_probe(chip->dev, -EINVAL, "too many led-sources\n");
	} else {
		ret = fwnode_property_read_u32_array(child, "led-sources",
						     sources, count);
		if (ret)
			return dev_err_probe(chip->dev, ret,
					     "failed to read led-sources\n");

		for (i = 0; i < count; i++) {
			if (sources[i] < 1 || sources[i] > 2)
				return dev_err_probe(chip->dev, -EINVAL,
						     "invalid led-sources entry %u\n",
						     sources[i]);

			chip->led_en |= BIT(sources[i] - 1);
		}
	}

	chip->torch_current_max = LM3644_TORCH_MAX_UA;
	fwnode_property_read_u32(child, "led-max-microamp",
				 &chip->torch_current_max);
	chip->torch_current_max = clamp(chip->torch_current_max,
					LM3644_TORCH_MIN_UA, LM3644_TORCH_MAX_UA);

	chip->flash_current_max = LM3644_FLASH_MAX_UA;
	fwnode_property_read_u32(child, "flash-max-microamp",
				 &chip->flash_current_max);
	chip->flash_current_max = clamp(chip->flash_current_max,
					LM3644_FLASH_MIN_UA, LM3644_FLASH_MAX_UA);

	chip->flash_timeout_max = LM3644_TIMEOUT_MAX_US;
	fwnode_property_read_u32(child, "flash-max-timeout-us",
				 &chip->flash_timeout_max);
	chip->flash_timeout_max = clamp(chip->flash_timeout_max,
					LM3644_TIMEOUT_MIN_US,
					LM3644_TIMEOUT_MAX_US);

	return 0;
}

static int lm3644_register_led(struct lm3644 *chip, struct fwnode_handle *child)
{
	struct led_classdev_flash *fled_cdev = &chip->fled_cdev;
	struct led_classdev *led_cdev = &fled_cdev->led_cdev;
	struct led_init_data init_data = {};
	struct led_flash_setting *setting;

	fled_cdev->ops = &lm3644_flash_ops;

	setting = &fled_cdev->brightness;
	setting->min = LM3644_FLASH_MIN_UA;
	setting->max = chip->flash_current_max;
	setting->step = LM3644_FLASH_STEP_UA;
	setting->val = chip->flash_current_max;

	setting = &fled_cdev->timeout;
	setting->min = LM3644_TIMEOUT_MIN_US;
	setting->max = chip->flash_timeout_max;
	setting->step = LM3644_TIMEOUT_LOWER_STEP_US;
	setting->val = chip->flash_timeout_max;

	led_cdev->max_brightness = (chip->torch_current_max - LM3644_TORCH_MIN_UA) /
				   LM3644_TORCH_STEP_UA + 1;
	led_cdev->brightness_set_blocking = lm3644_brightness_set;
	led_cdev->flags |= LED_DEV_CAP_FLASH;

	init_data.fwnode = child;
	init_data.devicename = "lm3644";
	init_data.default_label = "flash";

	return devm_led_classdev_flash_register_ext(chip->dev, fled_cdev,
						    &init_data);
}

static int lm3644_init(struct lm3644 *chip)
{
	unsigned int id;
	int ret;

	ret = regmap_write(chip->regmap, LM3644_BOOST_REG, LM3644_SW_RESET);
	if (ret)
		return ret;

	usleep_range(1000, 2000);

	ret = regmap_read(chip->regmap, LM3644_DEVICE_ID_REG, &id);
	if (ret)
		return ret;

	if (FIELD_GET(LM3644_DEVICE_ID_MASK, id) != 0)
		return dev_err_probe(chip->dev, -ENODEV,
				     "unexpected device id 0x%02x\n", id);

	return regmap_update_bits(chip->regmap, LM3644_ENABLE_REG,
				  LM3644_TX_PIN_EN, 0);
}

static void lm3644_standby(void *data)
{
	struct lm3644 *chip = data;

	lm3644_set_mode(chip, LM3644_MODE_STANDBY);
	gpiod_set_value_cansleep(chip->enable_gpio, 0);
}

static int lm3644_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *child;
	struct lm3644 *chip;
	int ret;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = dev;

	ret = devm_mutex_init(dev, &chip->lock);
	if (ret)
		return ret;

	chip->regmap = devm_regmap_init_i2c(client, &lm3644_regmap_config);
	if (IS_ERR(chip->regmap))
		return dev_err_probe(dev, PTR_ERR(chip->regmap),
				     "failed to init regmap\n");

	chip->enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(chip->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(chip->enable_gpio),
				     "failed to get enable gpio\n");

	usleep_range(1000, 2000);

	child = device_get_named_child_node(dev, "led");
	if (!child)
		return dev_err_probe(dev, -ENODEV, "no LED child node\n");

	ret = lm3644_parse_led(chip, child);
	if (ret)
		goto out_put;

	ret = lm3644_init(chip);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init device\n");
		goto out_put;
	}

	ret = devm_add_action_or_reset(dev, lm3644_standby, chip);
	if (ret)
		goto out_put;

	ret = lm3644_register_led(chip, child);
	if (ret)
		dev_err_probe(dev, ret, "failed to register LED\n");

out_put:
	fwnode_handle_put(child);
	return ret;
}

static const struct i2c_device_id lm3644_id[] = {
	{ "lm3644" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lm3644_id);

static const struct of_device_id lm3644_of_match[] = {
	{ .compatible = "ti,lm3644" },
	{ }
};
MODULE_DEVICE_TABLE(of, lm3644_of_match);

static struct i2c_driver lm3644_driver = {
	.driver = {
		.name = "leds-lm3644",
		.of_match_table = lm3644_of_match,
	},
	.probe = lm3644_probe,
	.id_table = lm3644_id,
};
module_i2c_driver(lm3644_driver);

MODULE_DESCRIPTION("Texas Instruments LM3644 flash LED driver");
MODULE_AUTHOR("sidharthify <wednisegit@gmail.com>");
MODULE_LICENSE("GPL");
