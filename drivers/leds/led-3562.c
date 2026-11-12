// SPDX-License-Identifier: GPL-2.0
/*
 * led-3562: power LED driver for RK3562 handhelds
 *
 * Supports any number of GPIO LEDs, each with its own label, exposed
 * through the LED class (e.g. /sys/class/leds/red). A battery capacity
 * threshold can drive the LEDs automatically; every LED chooses whether
 * it lights below or above that threshold.
 *
 * Device tree:
 *   power-leds {
 *       compatible = "led-3562";
 *       battery-threshold = <20>;      // 0 = user control only
 *
 *       led-red {
 *           label = "red";
 *           gpios = <&gpio3 RK_PC2 GPIO_ACTIVE_HIGH>;
 *           on-below-threshold;      // lights when capacity <  threshold
 *       };
 *       led-blue {
 *           label = "blue";
 *           gpios = <&gpio3 RK_PC1 GPIO_ACTIVE_HIGH>;
 *           // no on-below-threshold: lights when capacity >= threshold
 *       };
 *   };
 *
 * Sysfs:
 *   /sys/devices/platform/power-leds/status             (RO)
 *   /sys/devices/platform/power-leds/battery_threshold  (RW, 0..100)
 *   /sys/devices/platform/power-leds/gpio               (RW, "<label> <0|1>")
 *   /sys/class/leds/<label>/brightness                  (RW, 0|1)
 *   /sys/class/leds/<label>/battery_threshold           (RW, shared value)
 *
 * Threshold mode (battery_threshold > 0) takes over the LEDs until the
 * threshold is set back to 0.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/slab.h>

#define LED3562_POLL_INTERVAL	2000	/* ms */
#define LED3562_MAX_LEDS	8

struct led3562;

struct led3562_led {
	struct led_classdev cdev;
	struct gpio_desc *gpiod;
	struct led3562 *priv;
	bool valid;
	bool on_below;		/* true: on when capacity < threshold */
	int battery_threshold;	/* 0 = user control */
};

struct led3562 {
	struct device *dev;
	struct led3562_led *leds;
	int num_leds;

	struct power_supply *psy;
	struct delayed_work poll_work;
	bool charge_monitoring;
	int capacity;

	int battery_threshold;	/* DT default applied to LEDs without override */
};

static bool led3562_auto_mode(struct led3562 *priv, struct led3562_led *led)
{
	return priv->charge_monitoring && led->battery_threshold > 0;
}

static void led3562_raw_set(struct led3562_led *led, int level)
{
	if (led->valid && led->gpiod)
		gpiod_set_value_cansleep(led->gpiod, level ? 1 : 0);
}

static int led3562_raw_get(struct led3562_led *led)
{
	if (!led->valid || !led->gpiod)
		return 0;

	return gpiod_get_value_cansleep(led->gpiod) ? 1 : 0;
}

static void led3562_update(struct led3562 *priv)
{
	int i;

	for (i = 0; i < priv->num_leds; i++) {
		struct led3562_led *led = &priv->leds[i];
		bool below;

		if (!led->valid || !led3562_auto_mode(priv, led))
			continue;

		/* capacity unknown: keep warning LEDs on, normal LEDs off */
		if (priv->capacity < 0) {
			led3562_raw_set(led, led->on_below);
			continue;
		}

		below = priv->capacity < led->battery_threshold;
		led3562_raw_set(led, led->on_below ? below : !below);
	}
}

static void led3562_brightness_set(struct led_classdev *cdev,
				   enum led_brightness brightness)
{
	struct led3562_led *led = container_of(cdev, struct led3562_led, cdev);
	struct led3562 *priv = led->priv;

	if (led3562_auto_mode(priv, led)) {
		dev_dbg(priv->dev, "%s: threshold mode active, user control ignored\n",
			cdev->name);
		return;
	}

	led3562_raw_set(led, brightness != LED_OFF);
}

static enum led_brightness led3562_brightness_get(struct led_classdev *cdev)
{
	struct led3562_led *led = container_of(cdev, struct led3562_led, cdev);

	return led3562_raw_get(led) ? LED_FULL : LED_OFF;
}

static int led3562_set_threshold(struct led3562 *priv, unsigned long val)
{
	int i;

	if (val > 100)
		return -EINVAL;

	priv->battery_threshold = val;
	for (i = 0; i < priv->num_leds; i++)
		priv->leds[i].battery_threshold = val;

	led3562_update(priv);

	return 0;
}

static ssize_t battery_threshold_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct led3562 *priv = dev_get_drvdata(dev);
	int i, count = 0;

	for (i = 0; i < priv->num_leds; i++) {
		if (priv->leds[i].valid)
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "%s: %d\n", priv->leds[i].cdev.name,
					   priv->leds[i].battery_threshold);
	}

	if (!count)
		count = scnprintf(buf, PAGE_SIZE, "0\n");

	return count;
}

static ssize_t battery_threshold_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct led3562 *priv = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;

	ret = led3562_set_threshold(priv, val);
	if (ret)
		return ret;

	return count;
}
static DEVICE_ATTR_RW(battery_threshold);

static ssize_t led_threshold_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct led3562_led *led = container_of(cdev, struct led3562_led, cdev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", led->battery_threshold);
}

static ssize_t led_threshold_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct led3562_led *led = container_of(cdev, struct led3562_led, cdev);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;

	if (val > 100)
		return -EINVAL;

	led->battery_threshold = val;
	led3562_update(led->priv);

	return count;
}

static struct device_attribute dev_attr_led_battery_threshold = {
	.attr = { .name = "battery_threshold", .mode = 0644 },
	.show = led_threshold_show,
	.store = led_threshold_store,
};

static struct attribute *led3562_led_attrs[] = {
	&dev_attr_led_battery_threshold.attr,
	NULL,
};
ATTRIBUTE_GROUPS(led3562_led);

static ssize_t status_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct led3562 *priv = dev_get_drvdata(dev);
	int i, count = 0;

	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "capacity: %d%%\n", priv->capacity);

	for (i = 0; i < priv->num_leds; i++) {
		if (priv->leds[i].valid)
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "%s: %d (threshold %d, %s)\n",
					   priv->leds[i].cdev.name,
					   led3562_raw_get(&priv->leds[i]),
					   priv->leds[i].battery_threshold,
					   priv->leds[i].on_below ? "below" : "above");
	}

	return count;
}
static DEVICE_ATTR_RO(status);

static ssize_t gpio_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct led3562 *priv = dev_get_drvdata(dev);
	int i, count = 0;

	for (i = 0; i < priv->num_leds; i++) {
		if (priv->leds[i].valid)
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "%s %d\n", priv->leds[i].cdev.name,
					   led3562_raw_get(&priv->leds[i]));
	}

	if (!count)
		count = scnprintf(buf, PAGE_SIZE, "no leds\n");

	return count;
}

static ssize_t gpio_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct led3562 *priv = dev_get_drvdata(dev);
	char name[32];
	int value, i;

	if (sscanf(buf, "%31s %d", name, &value) != 2)
		return -EINVAL;

	for (i = 0; i < priv->num_leds; i++) {
		if (priv->leds[i].valid &&
		    !strcmp(name, priv->leds[i].cdev.name)) {
			led3562_brightness_set(&priv->leds[i].cdev,
					       value ? LED_FULL : LED_OFF);
			return count;
		}
	}

	return -ENODEV;
}
static DEVICE_ATTR_RW(gpio);

static struct attribute *led3562_platform_attrs[] = {
	&dev_attr_status.attr,
	&dev_attr_gpio.attr,
	&dev_attr_battery_threshold.attr,
	NULL,
};

static const struct attribute_group led3562_platform_group = {
	.attrs = led3562_platform_attrs,
};

static void led3562_poll_work(struct work_struct *work)
{
	struct led3562 *priv =
		container_of(to_delayed_work(work), struct led3562, poll_work);
	union power_supply_propval val;
	bool changed = false;
	int ret;

	if (priv->psy) {
		ret = power_supply_get_property(priv->psy,
						POWER_SUPPLY_PROP_CAPACITY, &val);
		if (!ret && priv->capacity != val.intval) {
			priv->capacity = val.intval;
			changed = true;
		}
	}

	if (changed || priv->battery_threshold > 0)
		led3562_update(priv);

	schedule_delayed_work(&priv->poll_work,
			      msecs_to_jiffies(LED3562_POLL_INTERVAL));
}

static struct power_supply *led3562_find_psy(void)
{
	static const char * const names[] = {
		"battery", "charger", "usb", "dc", "mains",
	};
	struct power_supply *psy;
	int i;

	for (i = 0; i < ARRAY_SIZE(names); i++) {
		psy = power_supply_get_by_name(names[i]);
		if (psy)
			return psy;
	}

	return NULL;
}

static int led3562_led_init(struct led3562 *priv, struct led3562_led *led,
			    struct device_node *np, int index)
{
	struct device *dev = priv->dev;
	const char *label;
	int ret;

	led->priv = priv;
	led->valid = false;
	led->on_below = of_property_read_bool(np, "on-below-threshold");
	led->battery_threshold = priv->battery_threshold;

	led->gpiod = devm_gpiod_get_from_of_node(dev, np, "gpios", 0,
						 GPIOD_OUT_LOW,
						 devm_kasprintf(dev, GFP_KERNEL,
								"led%d", index));
	if (IS_ERR(led->gpiod)) {
		ret = PTR_ERR(led->gpiod);
		led->gpiod = NULL;
		if (ret == -ENOENT)
			return -ENOENT;
		dev_warn(dev, "led%d: failed to get GPIO: %d\n", index, ret);
		return ret;
	}

	{
		u32 threshold;

		if (!of_property_read_u32(np, "battery-threshold", &threshold))
			led->battery_threshold = min_t(u32, threshold, 100);
	}

	if (of_property_read_string(np, "label", &label))
		label = devm_kasprintf(dev, GFP_KERNEL, "led%d", index);
	if (!label)
		return -ENOMEM;

	led->cdev.name = label;
	led->cdev.brightness_set = led3562_brightness_set;
	led->cdev.brightness_get = led3562_brightness_get;
	led->cdev.max_brightness = 1;
	led->cdev.brightness = LED_OFF;
	led->cdev.groups = led3562_led_groups;

	ret = devm_led_classdev_register(dev, &led->cdev);
	if (ret) {
		dev_err(dev, "led%d (%s): register failed: %d\n", index, label, ret);
		return ret;
	}

	led->valid = true;
	dev_info(dev, "%s registered (%s threshold)\n", label,
		 led->on_below ? "below" : "above");

	return 0;
}

static int led3562_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *child;
	struct led3562 *priv;
	int count = 0, index = 0, ret;
	u32 threshold;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->capacity = -1;
	priv->leds = devm_kcalloc(dev, LED3562_MAX_LEDS, sizeof(*priv->leds),
				  GFP_KERNEL);
	if (!priv->leds)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);

	if (!of_property_read_u32(np, "battery-threshold", &threshold))
		priv->battery_threshold = min_t(u32, threshold, 100);

	for_each_available_child_of_node(np, child) {
		if (index >= LED3562_MAX_LEDS) {
			dev_warn(dev, "more than %d LEDs, ignored\n",
				 LED3562_MAX_LEDS);
			break;
		}

		ret = led3562_led_init(priv, &priv->leds[index], child, index);
		if (!ret)
			count++;
		index++;
	}

	priv->num_leds = index;

	if (!count) {
		dev_warn(dev, "no LEDs configured\n");
		return -ENODEV;
	}

	ret = sysfs_create_group(&dev->kobj, &led3562_platform_group);
	if (ret) {
		dev_err(dev, "failed to create sysfs group: %d\n", ret);
		return ret;
	}

	priv->psy = led3562_find_psy();
	if (priv->psy) {
		priv->charge_monitoring = true;
		INIT_DELAYED_WORK(&priv->poll_work, led3562_poll_work);
		schedule_delayed_work(&priv->poll_work, msecs_to_jiffies(500));
		dev_info(dev, "charge monitoring enabled\n");
	} else {
		dev_info(dev, "no power supply found, user control only\n");
	}

	dev_info(dev, "led-3562 loaded (%d LED%s)\n", count, count > 1 ? "s" : "");
	return 0;
}

static int led3562_remove(struct platform_device *pdev)
{
	struct led3562 *priv = platform_get_drvdata(pdev);
	int i;

	if (priv->charge_monitoring)
		cancel_delayed_work_sync(&priv->poll_work);

	sysfs_remove_group(&pdev->dev.kobj, &led3562_platform_group);

	for (i = 0; i < priv->num_leds; i++)
		led3562_raw_set(&priv->leds[i], 0);

	if (priv->psy)
		power_supply_put(priv->psy);

	return 0;
}

static int __maybe_unused led3562_suspend(struct device *dev)
{
	struct led3562 *priv = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < priv->num_leds; i++)
		led3562_raw_set(&priv->leds[i], 0);

	return 0;
}

static int __maybe_unused led3562_resume(struct device *dev)
{
	struct led3562 *priv = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < priv->num_leds; i++) {
		struct led3562_led *led = &priv->leds[i];

		if (led3562_auto_mode(priv, led))
			continue;

		led3562_raw_set(led, led->cdev.brightness);
	}

	led3562_update(priv);

	return 0;
}

static SIMPLE_DEV_PM_OPS(led3562_pm_ops, led3562_suspend, led3562_resume);

static const struct of_device_id led3562_of_match[] = {
	{ .compatible = "led-3562" },
	{ }
};
MODULE_DEVICE_TABLE(of, led3562_of_match);

static struct platform_driver led3562_driver = {
	.probe	= led3562_probe,
	.remove	= led3562_remove,
	.driver	= {
		.name = "led-3562",
		.of_match_table = led3562_of_match,
		.pm = &led3562_pm_ops,
	},
};
module_platform_driver(led3562_driver);

MODULE_AUTHOR("lcdyk0517");
MODULE_DESCRIPTION("Power LED driver for RK3562 handhelds");
MODULE_LICENSE("GPL v2");
