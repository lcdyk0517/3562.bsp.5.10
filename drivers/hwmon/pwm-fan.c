// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * pwm-fan.c - Hwmon driver for fans connected to PWM lines.
 *
 * Copyright (c) 2014 Samsung Electronics Co., Ltd.
 *
 * Author: Kamil Debski <k.debski@samsung.com>
 */

#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/thermal.h>
#include <linux/timer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <soc/rockchip/rockchip_system_monitor.h>

#define MAX_PWM 255
/* 智能档迟滞: 温度低于档位阈值 5C 才降档 (45/55/65 升, 40/50/60 降) */
#define PWM_FAN_HYS	5000
/* 智能档两次换档最小间隔 */
#define PWM_FAN_SWITCH_MS	5000
/* 开机 5 分钟内不启动第一档 (仅当前停转时) */
#define PWM_FAN_BOOT_GRACE	300

struct thermal_trips {
	int temp;
	int state;
};

struct pwm_fan_ctx {
	struct mutex lock;
	struct pwm_device *pwm;
	struct regulator *reg_en;

	int irq;
	atomic_t pulses;
	unsigned int rpm;
	u8 pulses_per_revolution;
	ktime_t sample_start;
	struct timer_list rpm_timer;

	unsigned int pwm_value;
	unsigned int pwm_fan_state;
	unsigned int pwm_fan_max_state;
	unsigned int *pwm_fan_cooling_levels;
	struct thermal_cooling_device *cdev;
	struct notifier_block thermal_nb;
	struct thermal_trips *thermal_trips;
	bool thermal_notifier_is_ok;

	bool smart;
	struct thermal_zone_device *tz;
	struct delayed_work smart_work;
	ktime_t last_change;
	/* protects smart / pwm_value / pwm_fan_state across notifier,
	 * delayed work and sysfs paths */
	struct mutex policy_lock;
};

/* This handler assumes self resetting edge triggered interrupt. */
static irqreturn_t pulse_handler(int irq, void *dev_id)
{
	struct pwm_fan_ctx *ctx = dev_id;

	atomic_inc(&ctx->pulses);

	return IRQ_HANDLED;
}

static void sample_timer(struct timer_list *t)
{
	struct pwm_fan_ctx *ctx = from_timer(ctx, t, rpm_timer);
	unsigned int delta = ktime_ms_delta(ktime_get(), ctx->sample_start);
	int pulses;

	if (delta) {
		pulses = atomic_read(&ctx->pulses);
		atomic_sub(pulses, &ctx->pulses);
		ctx->rpm = (unsigned int)(pulses * 1000 * 60) /
			(ctx->pulses_per_revolution * delta);

		ctx->sample_start = ktime_get();
	}

	mod_timer(&ctx->rpm_timer, jiffies + HZ);
}

static int  __set_pwm(struct pwm_fan_ctx *ctx, unsigned long pwm)
{
	unsigned long period;
	int ret = 0;
	struct pwm_state state = { };

	mutex_lock(&ctx->lock);
	if (ctx->pwm_value == pwm)
		goto exit_set_pwm_err;

	pwm_init_state(ctx->pwm, &state);
	period = ctx->pwm->args.period;
	state.duty_cycle = DIV_ROUND_UP(pwm * (period - 1), MAX_PWM);
	state.enabled = pwm ? true : false;

	ret = pwm_apply_state(ctx->pwm, &state);
	if (!ret)
		ctx->pwm_value = pwm;
exit_set_pwm_err:
	mutex_unlock(&ctx->lock);
	return ret;
}

static void pwm_fan_update_state(struct pwm_fan_ctx *ctx, unsigned long pwm)
{
	int i;

	for (i = 0; i < ctx->pwm_fan_max_state; ++i)
		if (pwm < ctx->pwm_fan_cooling_levels[i + 1])
			break;

	ctx->pwm_fan_state = i;
}

static ssize_t pwm_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);
	unsigned long pwm;
	int ret;

	if (kstrtoul(buf, 10, &pwm) || pwm > MAX_PWM)
		return -EINVAL;

	/* 手动写 pwm1 即退出智能档, 避免内核策略覆盖用户设置 */
	mutex_lock(&ctx->policy_lock);
	ctx->smart = false;
	mutex_unlock(&ctx->policy_lock);
	cancel_delayed_work_sync(&ctx->smart_work);

	mutex_lock(&ctx->policy_lock);
	ret = __set_pwm(ctx, pwm);
	if (!ret)
		pwm_fan_update_state(ctx, pwm);
	mutex_unlock(&ctx->policy_lock);

	return ret ? ret : count;
}

static ssize_t pwm_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", ctx->pwm_value);
}

static ssize_t rpm_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", ctx->rpm);
}

static ssize_t smart_show(struct device *dev, struct device_attribute *attr,
			  char *buf);
static ssize_t smart_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count);

static SENSOR_DEVICE_ATTR_RW(pwm1, pwm, 0);
static SENSOR_DEVICE_ATTR_RO(fan1_input, rpm, 0);
static SENSOR_DEVICE_ATTR_RW(smart, smart, 0);

static struct attribute *pwm_fan_attrs[] = {
	&sensor_dev_attr_pwm1.dev_attr.attr,
	&sensor_dev_attr_fan1_input.dev_attr.attr,
	&sensor_dev_attr_smart.dev_attr.attr,
	NULL,
};

static umode_t pwm_fan_attrs_visible(struct kobject *kobj, struct attribute *a,
				     int n)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);

	/* Hide fan_input in case no interrupt is available  */
	if (n == 1 && ctx->irq <= 0)
		return 0;

	return a->mode;
}

static const struct attribute_group pwm_fan_group = {
	.attrs = pwm_fan_attrs,
	.is_visible = pwm_fan_attrs_visible,
};

static const struct attribute_group *pwm_fan_groups[] = {
	&pwm_fan_group,
	NULL,
};

/* thermal cooling device callbacks */
static int pwm_fan_get_max_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	struct pwm_fan_ctx *ctx = cdev->devdata;

	if (!ctx)
		return -EINVAL;

	*state = ctx->pwm_fan_max_state;

	return 0;
}

static int pwm_fan_get_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	struct pwm_fan_ctx *ctx = cdev->devdata;

	if (!ctx)
		return -EINVAL;

	*state = ctx->pwm_fan_state;

	return 0;
}

static int
pwm_fan_set_cur_state(struct thermal_cooling_device *cdev, unsigned long state)
{
	struct pwm_fan_ctx *ctx = cdev->devdata;
	int ret;

	if (!ctx || (state > ctx->pwm_fan_max_state))
		return -EINVAL;

	if (state == ctx->pwm_fan_state)
		return 0;

	ret = __set_pwm(ctx, ctx->pwm_fan_cooling_levels[state]);
	if (ret) {
		dev_err(&cdev->device, "Cannot set pwm!\n");
		return ret;
	}

	ctx->pwm_fan_state = state;

	return ret;
}

static const struct thermal_cooling_device_ops pwm_fan_cooling_ops = {
	.get_max_state = pwm_fan_get_max_state,
	.get_cur_state = pwm_fan_get_cur_state,
	.set_cur_state = pwm_fan_set_cur_state,
};

static int pwm_fan_of_get_cooling_data(struct device *dev,
				       struct pwm_fan_ctx *ctx)
{
	struct device_node *np = dev->of_node;
	int num, i, ret;

	if (!of_find_property(np, "cooling-levels", NULL))
		return 0;

	ret = of_property_count_u32_elems(np, "cooling-levels");
	if (ret <= 0) {
		dev_err(dev, "Wrong data!\n");
		return ret ? : -EINVAL;
	}

	num = ret;
	ctx->pwm_fan_cooling_levels = devm_kcalloc(dev, num, sizeof(u32),
						   GFP_KERNEL);
	if (!ctx->pwm_fan_cooling_levels)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, "cooling-levels",
					 ctx->pwm_fan_cooling_levels, num);
	if (ret) {
		dev_err(dev, "Property 'cooling-levels' cannot be read!\n");
		return ret;
	}

	for (i = 0; i < num; i++) {
		if (ctx->pwm_fan_cooling_levels[i] > MAX_PWM) {
			dev_err(dev, "PWM fan state[%d]:%d > %d\n", i,
				ctx->pwm_fan_cooling_levels[i], MAX_PWM);
			return -EINVAL;
		}
	}

	ctx->pwm_fan_max_state = num - 1;

	return 0;
}

static void pwm_fan_regulator_disable(void *data)
{
	regulator_disable(data);
}

static void pwm_fan_pwm_disable(void *__ctx)
{
	struct pwm_fan_ctx *ctx = __ctx;
	pwm_disable(ctx->pwm);
	del_timer_sync(&ctx->rpm_timer);
}

static int pwm_fan_get_thermal_trips(struct device *dev, char *porp_name,
				     struct thermal_trips **trips)
{
	struct device_node *np = dev->of_node;
	struct thermal_trips *thermal_trips;
	const struct property *prop;
	int count, i;

	prop = of_find_property(np, porp_name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;
	count = of_property_count_u32_elems(np, porp_name);
	if (count < 0)
		return -EINVAL;
	if (count % 2)
		return -EINVAL;
	thermal_trips = devm_kzalloc(dev,
				     sizeof(*thermal_trips) * (count / 2 + 1),
				     GFP_KERNEL);
	if (!thermal_trips)
		return -ENOMEM;

	for (i = 0; i < count / 2; i++) {
		of_property_read_u32_index(np, porp_name, 2 * i,
					   &thermal_trips[i].temp);
		of_property_read_u32_index(np, porp_name, 2 * i + 1,
					   &thermal_trips[i].state);
	}
	thermal_trips[i].temp = 0;
	thermal_trips[i].state = INT_MAX;

	*trips = thermal_trips;

	return 0;
}

static int pwm_fan_temp_to_state(struct pwm_fan_ctx *ctx, int temp)
{
	struct thermal_trips *trips = ctx->thermal_trips;
	int i, state = 0, cur = -1, cur_trip = -1;

	for (i = 0; trips[i].state != INT_MAX; i++) {
		if (temp >= trips[i].temp)
			state = trips[i].state;
		if (ctx->pwm_fan_cooling_levels[trips[i].state] == ctx->pwm_value) {
			cur = trips[i].state;
			cur_trip = trips[i].temp;
		}
	}

	/* 降档迟滞: 没低于当前档位阈值 5C 就保持, 避免阈值附近抖动 */
	if (cur > state && cur_trip >= 0 && temp >= cur_trip - PWM_FAN_HYS)
		state = cur;

	return state;
}

/* caller holds ctx->policy_lock */
static int pwm_fan_smart_apply(struct pwm_fan_ctx *ctx, int temp)
{
	unsigned int state, level;
	int ret;

	if (!ctx->smart)
		return 0;
	if (!ctx->thermal_trips)
		return -EOPNOTSUPP;
	if (temp < 0)
		return -EINVAL;

	state = pwm_fan_temp_to_state(ctx, temp);
	if (state > ctx->pwm_fan_max_state)
		state = ctx->pwm_fan_max_state;

	/* 开机 5 分钟内不启动第一档, 当前已停转时才钳制; 二/三档照常 */
	if (state == 2 && ctx->pwm_value == 0 &&
	    ktime_get_boottime_seconds() < PWM_FAN_BOOT_GRACE)
		state = 0;

	level = ctx->pwm_fan_cooling_levels[state];
	if (level == ctx->pwm_value) {
		ctx->pwm_fan_state = state;
		return 0;
	}

	/* 换档最小间隔 5s (notifier 和 2s 复检共用) */
	if (ctx->last_change &&
	    ktime_ms_delta(ktime_get(), ctx->last_change) < PWM_FAN_SWITCH_MS)
		return 0;

	ret = __set_pwm(ctx, level);
	if (!ret) {
		ctx->pwm_fan_state = state;
		ctx->last_change = ktime_get();
	}

	return ret;
}

static int pwm_fan_read_temp(struct pwm_fan_ctx *ctx)
{
	int temp;

	if (!ctx->tz) {
		ctx->tz = thermal_zone_get_zone_by_name("soc-thermal");
		if (IS_ERR_OR_NULL(ctx->tz)) {
			ctx->tz = NULL;
			return -ENODEV;
		}
	}
	if (thermal_zone_get_temp(ctx->tz, &temp))
		return -EINVAL;

	return temp;
}

static void pwm_fan_smart_work_fn(struct work_struct *work)
{
	struct pwm_fan_ctx *ctx = container_of(work, struct pwm_fan_ctx,
					       smart_work.work);
	bool reschedule;
	int temp;

	mutex_lock(&ctx->policy_lock);
	reschedule = ctx->smart;
	if (reschedule) {
		temp = pwm_fan_read_temp(ctx);
		if (temp >= 0)
			pwm_fan_smart_apply(ctx, temp);
		reschedule = ctx->smart;
	}
	mutex_unlock(&ctx->policy_lock);

	if (reschedule)
		schedule_delayed_work(&ctx->smart_work, 2 * HZ);
}

static void pwm_fan_smart_cancel(void *data)
{
	struct pwm_fan_ctx *ctx = data;

	mutex_lock(&ctx->policy_lock);
	ctx->smart = false;
	mutex_unlock(&ctx->policy_lock);
	cancel_delayed_work_sync(&ctx->smart_work);
}

static int pwm_fan_thermal_notifier_call(struct notifier_block *nb,
					 unsigned long event, void *data)
{
	struct pwm_fan_ctx *ctx = container_of(nb, struct pwm_fan_ctx, thermal_nb);
	struct system_monitor_event_data *event_data = data;
	int ret;

	if (event != SYSTEM_MONITOR_CHANGE_TEMP)
		return NOTIFY_OK;

	mutex_lock(&ctx->policy_lock);
	ret = pwm_fan_smart_apply(ctx, event_data->temp);
	mutex_unlock(&ctx->policy_lock);

	return ret ? NOTIFY_BAD : NOTIFY_OK;
}

static ssize_t smart_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);
	bool on;
	int temp, ret;

	ret = kstrtobool(buf, &on);
	if (ret)
		return ret;

	if (!on) {
		mutex_lock(&ctx->policy_lock);
		ctx->smart = false;
		mutex_unlock(&ctx->policy_lock);
		cancel_delayed_work_sync(&ctx->smart_work);
		return count;
	}

	if (!ctx->thermal_trips)
		return -EOPNOTSUPP;

	temp = pwm_fan_read_temp(ctx);
	if (temp < 0)
		return temp;

	mutex_lock(&ctx->policy_lock);
	ctx->smart = true;
	ctx->last_change = 0;
	ret = pwm_fan_smart_apply(ctx, temp);
	mutex_unlock(&ctx->policy_lock);

	if (ret) {
		mutex_lock(&ctx->policy_lock);
		ctx->smart = false;
		mutex_unlock(&ctx->policy_lock);
		return ret;
	}

	schedule_delayed_work(&ctx->smart_work, 2 * HZ);
	return count;
}

static ssize_t smart_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);
	bool on;

	mutex_lock(&ctx->policy_lock);
	on = ctx->smart;
	mutex_unlock(&ctx->policy_lock);

	return sprintf(buf, "%d\n", on ? 1 : 0);
}

static int pwm_fan_register_thermal_notifier(struct device *dev,
					     struct pwm_fan_ctx *ctx)
{
	if (pwm_fan_get_thermal_trips(dev, "rockchip,temp-trips",
				      &ctx->thermal_trips))
		return -EINVAL;

	ctx->thermal_nb.notifier_call = pwm_fan_thermal_notifier_call;

	return rockchip_system_monitor_register_notifier(&ctx->thermal_nb);
}

static int pwm_fan_probe(struct platform_device *pdev)
{
	struct thermal_cooling_device *cdev;
	struct device *dev = &pdev->dev;
	struct pwm_fan_ctx *ctx;
	struct device *hwmon;
	int ret;
	struct pwm_state state = { };
	u32 ppr = 2;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mutex_init(&ctx->lock);
	mutex_init(&ctx->policy_lock);

	ctx->pwm = devm_of_pwm_get(dev, dev->of_node, NULL);
	if (IS_ERR(ctx->pwm))
		return dev_err_probe(dev, PTR_ERR(ctx->pwm), "Could not get PWM\n");

	platform_set_drvdata(pdev, ctx);

	ctx->irq = platform_get_irq_optional(pdev, 0);
	if (ctx->irq == -EPROBE_DEFER)
		return ctx->irq;

	ctx->reg_en = devm_regulator_get_optional(dev, "fan");
	if (IS_ERR(ctx->reg_en)) {
		if (PTR_ERR(ctx->reg_en) != -ENODEV)
			return PTR_ERR(ctx->reg_en);

		ctx->reg_en = NULL;
	}

	/*
	 * dArkOS: 不要按上游那样在 probe 时先拉满占空比再等 hwmon/thermal
	 * 改写, 否则 U-Boot 交接到内核时会看到风扇突然全速一下. 这里直接
	 * 以 0 占空比启动 (输出脚被 PWM 驱动拉低, 不会浮空), 转速由 hwmon
	 * (fanctl) / thermal cooling 之后设置.
	 */
	ctx->pwm_value = 0;

	pwm_init_state(ctx->pwm, &state);
	/*
	 * __set_pwm assumes that MAX_PWM * (period - 1) fits into an unsigned
	 * long. Check this here to prevent the fan running at a too low
	 * frequency.
	 */
	if (state.period > ULONG_MAX / MAX_PWM + 1) {
		dev_err(dev, "Configured period too big\n");
		return -EINVAL;
	}

	/* Enable PWM output at zero duty */
	state.duty_cycle = 0;
	state.enabled = true;

	ret = pwm_apply_state(ctx->pwm, &state);
	if (ret) {
		dev_err(dev, "Failed to configure PWM: %d\n", ret);
		return ret;
	}

	/* PWM 已经是 0 占空比, 再给风扇供电: 交接时不会全速一下 */
	if (ctx->reg_en) {
		ret = regulator_enable(ctx->reg_en);
		if (ret) {
			dev_err(dev, "Failed to enable fan supply: %d\n", ret);
			return ret;
		}
		ret = devm_add_action_or_reset(dev, pwm_fan_regulator_disable,
					       ctx->reg_en);
		if (ret)
			return ret;
	}
	timer_setup(&ctx->rpm_timer, sample_timer, 0);
	INIT_DELAYED_WORK(&ctx->smart_work, pwm_fan_smart_work_fn);
	ret = devm_add_action_or_reset(dev, pwm_fan_smart_cancel, ctx);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, pwm_fan_pwm_disable, ctx);
	if (ret)
		return ret;

	of_property_read_u32(dev->of_node, "pulses-per-revolution", &ppr);
	ctx->pulses_per_revolution = ppr;
	if (!ctx->pulses_per_revolution) {
		dev_err(dev, "pulses-per-revolution can't be zero.\n");
		return -EINVAL;
	}

	if (ctx->irq > 0) {
		ret = devm_request_irq(dev, ctx->irq, pulse_handler, 0,
				       pdev->name, ctx);
		if (ret) {
			dev_err(dev, "Failed to request interrupt: %d\n", ret);
			return ret;
		}
		ctx->sample_start = ktime_get();
		mod_timer(&ctx->rpm_timer, jiffies + HZ);
	}

	hwmon = devm_hwmon_device_register_with_groups(dev, "pwmfan",
						       ctx, pwm_fan_groups);
	if (IS_ERR(hwmon)) {
		dev_err(dev, "Failed to register hwmon device\n");
		return PTR_ERR(hwmon);
	}

	ret = pwm_fan_of_get_cooling_data(dev, ctx);
	if (ret)
		return ret;

	ctx->pwm_fan_state = ctx->pwm_fan_max_state;
	if (IS_REACHABLE(CONFIG_ROCKCHIP_SYSTEM_MONITOR) &&
	    of_find_property(dev->of_node, "rockchip,temp-trips", NULL)) {
		ret = pwm_fan_register_thermal_notifier(dev, ctx);
		if (ret)
			dev_err(dev, "Failed to register thermal notifier: %d\n", ret);
		else
			ctx->thermal_notifier_is_ok = true;
		return 0;
	}
	if (IS_ENABLED(CONFIG_THERMAL)) {
		cdev = devm_thermal_of_cooling_device_register(dev,
			dev->of_node, "pwm-fan", ctx, &pwm_fan_cooling_ops);
		if (IS_ERR(cdev)) {
			ret = PTR_ERR(cdev);
			dev_err(dev,
				"Failed to register pwm-fan as cooling device: %d\n",
				ret);
			return ret;
		}
		ctx->cdev = cdev;
		thermal_cdev_update(cdev);
	}

	return 0;
}

static int pwm_fan_disable(struct device *dev)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);
	struct pwm_args args;
	int ret;

	pwm_get_args(ctx->pwm, &args);

	if (ctx->pwm_value || ctx->thermal_notifier_is_ok) {
		ret = pwm_config(ctx->pwm, 0, args.period);
		if (ret < 0)
			return ret;

		pwm_disable(ctx->pwm);
	}

	if (ctx->reg_en) {
		ret = regulator_disable(ctx->reg_en);
		if (ret) {
			dev_err(dev, "Failed to disable fan supply: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static void pwm_fan_shutdown(struct platform_device *pdev)
{
	pwm_fan_disable(&pdev->dev);
}

#ifdef CONFIG_PM_SLEEP
static int pwm_fan_suspend(struct device *dev)
{
	return pwm_fan_disable(dev);
}

static int pwm_fan_resume(struct device *dev)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);
	struct pwm_args pargs;
	unsigned long duty;
	int ret;

	if (ctx->reg_en) {
		ret = regulator_enable(ctx->reg_en);
		if (ret) {
			dev_err(dev, "Failed to enable fan supply: %d\n", ret);
			return ret;
		}
	}

	if (ctx->pwm_value == 0 && !ctx->thermal_notifier_is_ok)
		return 0;

	pwm_get_args(ctx->pwm, &pargs);
	duty = DIV_ROUND_UP_ULL(ctx->pwm_value * (pargs.period - 1), MAX_PWM);
	ret = pwm_config(ctx->pwm, duty, pargs.period);
	if (ret)
		return ret;
	return pwm_enable(ctx->pwm);
}
#endif

static SIMPLE_DEV_PM_OPS(pwm_fan_pm, pwm_fan_suspend, pwm_fan_resume);

static const struct of_device_id of_pwm_fan_match[] = {
	{ .compatible = "pwm-fan", },
	{},
};
MODULE_DEVICE_TABLE(of, of_pwm_fan_match);

static struct platform_driver pwm_fan_driver = {
	.probe		= pwm_fan_probe,
	.shutdown	= pwm_fan_shutdown,
	.driver	= {
		.name		= "pwm-fan",
		.pm		= &pwm_fan_pm,
		.of_match_table	= of_pwm_fan_match,
	},
};

module_platform_driver(pwm_fan_driver);

MODULE_AUTHOR("Kamil Debski <k.debski@samsung.com>");
MODULE_ALIAS("platform:pwm-fan");
MODULE_DESCRIPTION("PWM FAN driver");
MODULE_LICENSE("GPL");
