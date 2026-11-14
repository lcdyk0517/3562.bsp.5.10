// SPDX-License-Identifier: GPL-2.0-only
/*
 * RK3562 掌机手柄驱动 (SARADC 摇杆 & GPIO 按键)
 *
 * 面向 RK3562 掌机 (RG52 Mini / RG43H Pro / AIS / NGP45HT 等) 的通用手柄驱动,
 * 与原厂固件中的 "play_joystick" 设备树节点兼容.
 *
 * 功能一览:
 *   - 摇杆:   4 轴 IIO 通道 (LX/LY/RX/RY), 静息自动校准 + 每轴死区
 *   - 扳机:   模拟轴 ABS_Z/ABS_RZ 与数字键 BTN_TL2/BTN_TR2 双路上报 (可选,
 *             全 GPIO 扳机的机型如 NGP45HT 可省略)
 *   - ADC 键: 电阻梯分压按键 (可选, 子节点定义)
 *   - GPIO 键: IRQ + 消抖, 每键一个子节点 (odroidgo3 风格)
 *   - 轴变换: 左/右摇杆 XY 交换, 单轴反转
 *             (invert-absx/-absy/-absrx/-absry, odroidgo3 属性契约;
 *              左摇杆整体反转 = 同时打开 invert-absx + invert-absy)
 *   - 振动:   双马达 GPIO 力反馈 (rumble-gpios / rumble-r-gpios, 可选)
 *   - sysfs:  axis_to_dpad / swap_start_home / invert_absx..invert_absry
 *
 * 设备树绑定 (odroidgo3-joypad 风格):
 *
 *   play_joystick {
 *           compatible = "play_joystick";
 *           pinctrl-0 = <&gpio_key_pin &moto_gpio>;
 *
 *           // 摇杆轴 + 模拟扳机 + ADC 键通道; ADC 键通道名与子节点名一致
 *           io-channels = <&saradc0 2>, ..., <&saradc1 5>;
 *           io-channel-names = "joy_x", "joy_y", "joy_rx", "joy_ry",
 *                              "l2-abs", "r2-abs",
 *                              "left-key", ..., "y-key";
 *
 *           poll-interval = <16>;
 *           axis-min-value-mv = <250>;
 *           axis-max-value-mv = <1550>;
 *           axis-dead-zone-l = <790>;
 *           axis-dead-zone-h = <1010>;
 *           l2-r2-max-value-mv = <800>;
 *           invert-absx;                // 可选 (单轴反转, 四轴独立)
 *           invert-absy;                // 可选
 *           invert-absrx;               // 可选
 *           invert-absry;               // 可选
 *
 *           rumble-gpios = <&gpio3 RK_PA7 GPIO_ACTIVE_HIGH>;
 *           rumble-r-gpios = <&gpio3 RK_PB0 GPIO_ACTIVE_HIGH>;
 *
 *           // GPIO 按键: 每键一个子节点, 自带 gpios + 键码
 *           sw1 { label = "BTN_L1"; gpios = <&gpio1 RK_PC7 1>; linux,code = <BTN_TL>; };
 *           ...
 *
 *           // ADC 按键: 无 gpios 的子节点, 通道名 = 节点名
 *           b-key { label = "BTN_B"; linux,code = <BTN_SOUTH>;
 *                   press-threshold-microvolt = <80000>; };
 *   };
 *
 * 注: 原厂 zed_joystick 驱动会读取 l_x_swap / l_y_swap / r_x_swap / r_y_swap
 * 属性但从未使用; 本驱动保持一致, 直接忽略这些属性.
 */

/*----------------------------------------------------------------------------*/
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/iio/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/slab.h>

/*----------------------------------------------------------------------------*/
#define DRIVER_NAME	"rk3562-joystick"

/* 摇杆轴数 (LX, LY, RX, RY) */
#define NUM_STICK_CHANS	4
/* 模拟扳机轴数 (L2, R2) */
#define NUM_TRIG_CHANS	2
/* ADC 电阻梯按键数 (十字键 左/右/下 + B + X + Y) */
#define NUM_ADC_BTNS	6

/* 上报给用户空间的轴范围 */
#define AXIS_MIN	-32767
#define AXIS_MAX	32767
#define TRIG_MIN	0
#define TRIG_MAX	32767

/* 摇杆转十字键模式的方向触发阈值: ~55% of 32767 = 18000 */
#define DPAD_THRESHOLD	18000

/* 默认消抖间隔 (ms) -- 硅胶微动开关 1-3ms 内稳定 */
#define DEFAULT_DEBOUNCE_MS	5

/* 自动校准参数 */
#define CALIBRATION_SAMPLES	16	/* 探测时采样的平均次数 */
#define CALIBRATION_DELAY_US	1000	/* 采样间隔 1ms */

/* 扳机: 完全按下时的一致电压下限 (微伏) */
#define TRIG_MIN_UV		11000	/* 实测 11mV */
/* 扳机: 静息附近的输出死区 -- 抑制低于该输出值的噪声.
 * 0-32767 量程下 L2 噪声峰约 536, R2 约 77, 取 800 留裕量. */
#define TRIG_DZ_OUTPUT		800
/* 扳机: 校准后的事件 fuzz/flat (远小于校准前的数值) */
#define TRIG_FLAT_CALIBRATED	512
#define TRIG_FUZZ_CALIBRATED	64
/* 扳机数字键阈值 (0..32767 映射量程上):
 * 除模拟 ABS_Z/ABS_RZ 轴外, 重按锁定数字 BTN_TL2/BTN_TR2, 让期望数字扳机的
 * 软件 (许多 PortMaster 移植, 裸 evdev 消费者) 与模拟轴 (SDL/RetroArch 绑定)
 * 同时可用. 迟滞 (按下高 / 释放低) 消除阈值附近的抖动. 内核手柄规范允许扳机
 * 同时报模拟与数字; hid-playstation 对 ABS_Z/RZ 也是如此. */
#define TRIG_BTN_PRESS_THRESH	21000	/* ~64% 处锁定 */
#define TRIG_BTN_RELEASE_THRESH	14000	/* ~43% 处释放 */

/* DTS 键码中需要重映射到 BTN_* 区间 (joydev 可见) 的 KEY_* 码 */
#define KEY_HOME_DTS	102
#define KEY_FN_DTS	464

/*----------------------------------------------------------------------------*/
/* 数据结构 */

struct rk3562_joystick;

/* GPIO 按键 (IRQ 驱动, 带消抖) */
struct rk3562_gpio_btn {
	struct gpio_desc *gpiod;	/* 按键 GPIO 描述符 */
	unsigned int code;		/* 重映射后的 linux 键码 */
	int irq;			/* 按键中断号 */
	struct delayed_work work;	/* 消抖工作队列 */
	struct rk3562_joystick *joy;	/* 所属驱动上下文 */
};

/* 主驱动上下文 */
struct rk3562_joystick {
	struct device *dev;
	struct input_dev *input;

	/* ---- IIO 通道 ---- */
	/* 摇杆轴通道: 必须存在 */
	struct iio_channel *stick_chans[NUM_STICK_CHANS];
	/* 扳机通道: 可选 (NULL = 数字 GPIO 扳机机型) */
	struct iio_channel *trig_chans[NUM_TRIG_CHANS];
	/* ADC 按键通道: 可选 (NULL = 面键全 GPIO 机型) */
	struct iio_channel *adc_btn_chans[NUM_ADC_BTNS];

	/* ---- GPIO 按键 ---- */
	struct rk3562_gpio_btn *gpio_btns;	/* 按键数组 (动态大小) */
	int num_gpio_btns;
	unsigned int debounce_ms;

	/* ---- ADC 按键 ---- */
	unsigned int adc_btn_codes[NUM_ADC_BTNS];	/* 键码 (0 = 未用) */
	int adc_btn_thresh[NUM_ADC_BTNS];		/* 按下上限 (微伏, 不含) */
	int adc_btn_min[NUM_ADC_BTNS];			/* 按下下限 (微伏, 含) */

	/* ---- 摇杆校准 (DT 毫伏 -> 微伏) ---- */
	int axis_min_uv;
	int axis_max_uv;
	int axis_dz_lo_uv[NUM_STICK_CHANS];	/* 每轴死区下界 */
	int axis_dz_hi_uv[NUM_STICK_CHANS];	/* 每轴死区上界 */
	int axis_center_uv[NUM_STICK_CHANS];	/* 每轴静息中心 */
	int axis_min_uv_a[NUM_STICK_CHANS];	/* 每轴校准下界 */
	int axis_max_uv_a[NUM_STICK_CHANS];	/* 每轴校准上界 */
	int axis_last_uv[NUM_STICK_CHANS];	/* 最近一次原始读数 */
	int dt_dz_uv[NUM_STICK_CHANS];		/* DT 死区半宽 (微伏) */
	int adc_deadzone;			/* 运行时死区(mV), 0=DT 默认 */

	/* ---- 扳机校准 ---- */
	int trig_min_uv;
	int trig_max_uv[NUM_TRIG_CHANS];	/* 每扳机静息电压 */

	/* 数字 BTN_TL2/BTN_TR2 锁存状态 (模拟值上的迟滞,
	 * 使 L2/R2 同时以轴 + 按键两种形式上报) */
	bool trig_btn_pressed[NUM_TRIG_CHANS];
	/* 线性扳机上报开关: 关 = 只报数字 BTN_TL2/BTN_TR2,
	 * 不上报 ABS_Z/ABS_RZ 线性轴 (sysfs 可运行时切换) */
	bool linear_triggers;

	/* ---- 轴变换 ---- */
	/* 摇杆 X/Y 交换 */
	bool l_xy_swap;
	bool r_xy_swap;
	/* 单轴反转 (odroidgo3 属性契约: invert-absx/-absy/-absrx/-absry;
	 * 用于左摇杆模块装反、X/Y 电位器接法镜像的机型).
	 * 在 XY 交换解析之后, 按最终 ABS 码生效; sysfs 可运行时切换. */
	bool inv_x, inv_y, inv_rx, inv_ry;

	/* ---- 振动马达 (可选) ---- */
	struct gpio_desc *rumble_gpio;		/* 左/主马达 */
	struct gpio_desc *rumble_r_gpio;	/* 右/副马达 */
	bool rumble_on;				/* 挂起前的振动状态 */

	/* ---- 摇杆转十字键模式 (sysfs 切换) ---- */
	bool axis_to_dpad;
	bool last_dpad_up;
	bool last_dpad_down;
	bool last_dpad_left;
	bool last_dpad_right;

	/* ---- AB / XY 键码交换 (sysfs 切换, 上报时映射) ---- */
	bool swap_ab;
	bool swap_xy;

	/* ---- Start/Select <-> HOME/BACK 交换 (sysfs 切换) ---- */
	bool swap_start_home;
	bool swap_available;		/* 四个键齐全时才可交换 */
	int swap_indices[4];		/* gpio_btns[] 下标: START, SELECT, MODE, HAPPY1 */
};

/*----------------------------------------------------------------------------*/
/* 通道名与轴码表 */

/* 通道按名字获取 (与 DTS io-channel-names 一一对应) */
static const char * const stick_chan_names[NUM_STICK_CHANS] = {
	"joy_x", "joy_y", "joy_rx", "joy_ry",
};

static const char * const trig_chan_names[NUM_TRIG_CHANS] = {
	"l2-abs", "r2-abs",
};

/* ADC 按键通道不经此表: 通道名 = ADC 按键子节点名 (见 rk3562_parse_adc_buttons) */

/* 摇杆基础轴码 (XY 交换之前) */
static const unsigned int stick_abs_codes[NUM_STICK_CHANS] = {
	ABS_X, ABS_Y, ABS_RX, ABS_RY,
};

/* 扳机轴码: ABS_Z (左/L2) 与 ABS_RZ (右/R2).
 *
 * 这是 Linux 规范的模拟扳机码, 我们关心的两类消费方都能看到真实模拟轴:
 * SDL2 自动映射 ABS_Z->lefttrigger / ABS_RZ->righttrigger; RetroArch 的
 * udev 手柄驱动会应用扳机修正 (neg_trigger 检测 + (val+0x7fff)/2 重映射),
 * 为静息在 0 的扳机给出干净的 0..max 范围.
 *
 * 此前使用 ABS_HAT2Y/ABS_HAT2X (手柄规范的 ZL/ZR 码) 以维持 joydev
 * 轴序 0-3=摇杆, 4/5=扳机. 但 udev 把 ABS_HAT 区间 (0x10-0x17) 当作
 * 数字帽子键, RetroArch 只能绑成 h2down/h2right -- 模拟值永远到不了
 * Flycast 之类的核心. ABS_Z (0x02) 位于摇杆之间, joydev/SDL 现在的
 * 轴序为 0=LX,1=LY,2=L2,3=RX,4=RY,5=R2; 所有按索引配置的库
 * (gamecontrollerdb / es_input / mupen / dolphin / retroarch 自动配置)
 * 都已按此布局更新. */
static const unsigned int trig_abs_codes[NUM_TRIG_CHANS] = {
	ABS_Z, ABS_RZ,
};

/* 扳机数字键码, 与上面的模拟轴并行上报 (带迟滞的阈值判定, 见轮询).
 * BTN_TL2/BTN_TR2 在探测时已作为能力注册以稳定 joydev 索引;
 * 轮询中它们被真正驱动. */
static const unsigned int trig_btn_codes[NUM_TRIG_CHANS] = {
	BTN_TL2, BTN_TR2,
};

/*----------------------------------------------------------------------------*/
/* 轴码解析与映射辅助函数 */

/**
 * rk3562_stick_code() - 返回摇杆通道 @i 的 ABS 码 (考虑 XY 交换).
 * @joy: 驱动上下文
 * @i:   摇杆通道下标 (0=LX 1=LY 2=RX 3=RY)
 *
 * 返回: 该通道最终上报的 ABS 码
 */
static unsigned int rk3562_stick_code(struct rk3562_joystick *joy, int i)
{
	switch (i) {
	case 0: /* LX */
		return joy->l_xy_swap ? ABS_Y : ABS_X;
	case 1: /* LY */
		return joy->l_xy_swap ? ABS_X : ABS_Y;
	case 2: /* RX */
		return joy->r_xy_swap ? ABS_RY : ABS_RX;
	case 3: /* RY */
		return joy->r_xy_swap ? ABS_RX : ABS_RY;
	default:
		return stick_abs_codes[i];
	}
}

/**
 * rk3562_axis_inverted() - 查询单轴反转标志 (odroidgo3 属性契约:
 *                          invert-absx / -absy / -absrx / -absry).
 * @joy:  驱动上下文
 * @code: 最终 ABS 码 (即 XY 交换解析之后的轴)
 *
 * 按用户空间可见的轴判定, 与交换设置正确叠加.
 *
 * 返回: 该轴是否需要取反
 */
static bool rk3562_axis_inverted(struct rk3562_joystick *joy,
				 unsigned int code)
{
	switch (code) {
	case ABS_X:  return joy->inv_x;
	case ABS_Y:  return joy->inv_y;
	case ABS_RX: return joy->inv_rx;
	case ABS_RY: return joy->inv_ry;
	default:     return false;
	}
}

/**
 * rk3562_map_stick() - 将原始微伏 ADC 读数映射到 [-32767, 32767] 轴范围.
 * @joy:  驱动上下文
 * @uv:   原始读数 (微伏)
 * @axis: 摇杆通道下标
 *
 * 先钳位到校准范围; 死区内映射为 0; 死区外线性映射:
 *   低于死区: [min, dz_lo] -> [+32767, 0]
 *     (右摇杆: 低电压 = 右/下 即正方向. 左摇杆电位器可能极性相反,
 *      由 DT 的 invert-absx/invert-absy 在最终 ABS 码上取反纠正.)
 *   高于死区: [dz_hi, max] -> [0, -32767]
 *
 * 返回: 映射后的轴值
 */
static int rk3562_map_stick(struct rk3562_joystick *joy, int uv, int axis)
{
	int dz_lo = joy->axis_dz_lo_uv[axis];
	int dz_hi = joy->axis_dz_hi_uv[axis];
	int min_uv = joy->axis_min_uv_a[axis];
	int max_uv = joy->axis_max_uv_a[axis];
	int val;

	joy->axis_last_uv[axis] = uv;

	/* 钳位到校准范围 */
	if (uv < min_uv)
		uv = min_uv;
	if (uv > max_uv)
		uv = max_uv;

	/* 死区 -> 0 */
	if (uv >= dz_lo && uv <= dz_hi)
		return 0;

	if (uv < dz_lo) {
		if (dz_lo == min_uv)
			return 32767;
		val = (int)((long long)(dz_lo - uv) * 32767 /
			    (dz_lo - min_uv));
	} else {
		if (dz_hi == max_uv)
			return -32767;
		val = (int)((long long)(dz_hi - uv) * 32767 /
			    (max_uv - dz_hi));
	}

	if (val < AXIS_MIN)
		val = AXIS_MIN;
	if (val > AXIS_MAX)
		val = AXIS_MAX;

	return val;
}

/**
 * rk3562_map_trigger() - 将原始微伏读数映射到 [0, 32767] 扳机范围.
 * @joy:  驱动上下文
 * @uv:   原始读数 (微伏)
 * @trig: 扳机通道下标
 *
 * 以每扳机静息电压作为量程上限. 扳机是上拉: 高电压 = 松开, 低 = 按下.
 * 静息附近的输出噪声被 TRIG_DZ_OUTPUT 死区抑制.
 *
 * 返回: 映射后的扳机值 (0 = 松开, 32767 = 按到底)
 */
static int rk3562_map_trigger(struct rk3562_joystick *joy, int uv, int trig)
{
	int trig_max = joy->trig_max_uv[trig];
	int val;

	if (uv < joy->trig_min_uv)
		uv = joy->trig_min_uv;
	if (uv > trig_max)
		uv = trig_max;

	/* 上拉: (max - uv) 越大 = 按得越深 */
	val = (int)((long long)(trig_max - uv) * 32767 /
		    (trig_max - joy->trig_min_uv));

	/* 抑制静息位置附近的噪声 */
	if (val < TRIG_DZ_OUTPUT)
		val = 0;

	if (val < TRIG_MIN)
		val = TRIG_MIN;
	if (val > TRIG_MAX)
		val = TRIG_MAX;

	return val;
}

/**
 * rk3562_remap_code() - 把落在 BTN_* 区间之外 (joydev 不可见) 的 DTS 键码
 *                       重映射为 joydev 可见的 BTN_* 码.
 * @dts_code: DTS 中的 linux,code
 *
 * KEY_HOME -> BTN_MODE (SDL 的 Guide 键).
 * KEY_FN   -> BTN_TRIGGER_HAPPY1: BTN_TRIGGER_HAPPY1..40 (evdev
 *             0x2c0..0x2e7) 中一个通用的"额外按键"槽位, 内核输入子系统为
 *             放不进标准手柄布局的按键保留. 避免把 BTN_TL2 (规范的
 *             左扳机 2 码) 挪用给无关的功能键.
 *
 * 返回: 重映射后的键码
 */
static unsigned int rk3562_remap_code(unsigned int dts_code)
{
	switch (dts_code) {
	case KEY_HOME_DTS:
		return BTN_MODE;            /* 0x13c -- HOME -> Guide */
	case KEY_FN_DTS:
		return BTN_TRIGGER_HAPPY1;  /* 0x2c0 -- FN/BACK -> 额外键 */
	default:
		return dts_code;
	}
}

/**
 * rk3562_read_avg() - 对 IIO 通道采样 @samples 次并返回平均毫伏值.
 * @chan:    IIO 通道
 * @samples: 采样次数
 *
 * 第一次读数作为预热丢弃. 返回负数为错误.
 *
 * 返回: 平均值 (毫伏) 或负错误码
 */
static int rk3562_read_avg(struct iio_channel *chan, int samples)
{
	int i, ret, val;
	long sum = 0;
	int count = 0;

	/* 预热读数 (丢弃) */
	ret = iio_read_channel_processed(chan, &val);
	if (ret < 0)
		return ret;
	udelay(CALIBRATION_DELAY_US);

	for (i = 0; i < samples; i++) {
		ret = iio_read_channel_processed(chan, &val);
		if (ret < 0)
			return ret;
		sum += val;
		count++;
		if (i < samples - 1)
			udelay(CALIBRATION_DELAY_US);
	}

	return (int)(sum / count);  /* 毫伏 */
}

/* 前向声明: 供 GPIO/ADC 按键上报使用 */
static unsigned int rk3562_swap_code(struct rk3562_joystick *joy,
				     unsigned int code);

/*----------------------------------------------------------------------------*/
/* IRQ 驱动的 GPIO 按键 */

/**
 * rk3562_gpio_work_func() - 消抖到期后读取 GPIO 并上报按键状态.
 * @work: 按键的 delayed_work
 *
 * 中断只负责调度本工作; 真正的读取与上报发生在消抖间隔之后.
 */
static void rk3562_gpio_work_func(struct work_struct *work)
{
	struct rk3562_gpio_btn *btn =
		container_of(work, struct rk3562_gpio_btn, work.work);
	struct rk3562_joystick *joy = btn->joy;
	int val;

	val = gpiod_get_value_cansleep(btn->gpiod);
	if (val < 0) {
		dev_err_ratelimited(joy->dev,
				    "GPIO read failed for btn %u: %d\n",
				    btn->code, val);
		return;
	}

	input_report_key(joy->input, rk3562_swap_code(joy, btn->code), val);
	input_sync(joy->input);
}

/**
 * rk3562_gpio_isr() - 按键中断服务函数.
 * @irq:  中断号
 * @data: 对应的 rk3562_gpio_btn
 *
 * 双边沿触发; 每次电平变化重新调度消抖工作 (mod_delayed_work 会取消
 * 尚未执行的上一次调度, 天然实现了消抖窗口的滑动).
 *
 * 返回: 始终 IRQ_HANDLED
 */
static irqreturn_t rk3562_gpio_isr(int irq, void *data)
{
	struct rk3562_gpio_btn *btn = data;
	struct rk3562_joystick *joy = btn->joy;

	mod_delayed_work(system_wq, &btn->work,
			 msecs_to_jiffies(joy->debounce_ms));

	return IRQ_HANDLED;
}

/**
 * rk3562_cancel_gpio_work() - devm 释放动作: 取消所有按键的消抖工作.
 * @data: 驱动上下文
 */
static void rk3562_cancel_gpio_work(void *data)
{
	struct rk3562_joystick *joy = data;
	int i;

	for (i = 0; i < joy->num_gpio_btns; i++)
		cancel_delayed_work_sync(&joy->gpio_btns[i].work);
}

/*----------------------------------------------------------------------------*/
/* 力反馈 (振动马达) */

/**
 * rk3562_play_effect() - 力反馈效果回调.
 * @dev:    输入设备
 * @data:   驱动上下文 (input_set_drvdata 传入)
 * @effect: 力反馈效果
 *
 * 双马达为开关式 (无强度分级): strong/weak 任一非零即开启.
 * 同时记录 rumble_on 供挂起/恢复使用.
 *
 * 返回: 始终 0
 */
static int rk3562_play_effect(struct input_dev *dev, void *data,
			      struct ff_effect *effect)
{
	struct rk3562_joystick *joy = input_get_drvdata(dev);
	bool on;

	on = effect->u.rumble.strong_magnitude ||
	     effect->u.rumble.weak_magnitude;

	joy->rumble_on = on;

	if (joy->rumble_gpio)
		gpiod_set_value_cansleep(joy->rumble_gpio, on);
	if (joy->rumble_r_gpio)
		gpiod_set_value_cansleep(joy->rumble_r_gpio, on);

	return 0;
}

/*----------------------------------------------------------------------------*/
/* 依据中心与当前量程重算某轴死区 */
static void rk3562_recalc_deadzone(struct rk3562_joystick *joy, int axis)
{
	int dz = joy->adc_deadzone > 0 ?
		 joy->adc_deadzone * 1000 : joy->dt_dz_uv[axis];

	joy->axis_dz_lo_uv[axis] = joy->axis_center_uv[axis] - dz;
	joy->axis_dz_hi_uv[axis] = joy->axis_center_uv[axis] + dz;
}

/* AB / XY 键码交换: 上报时映射, 避免改动按键表 */
static unsigned int rk3562_swap_code(struct rk3562_joystick *joy,
				     unsigned int code)
{
	if (joy->swap_ab) {
		if (code == BTN_SOUTH)
			return BTN_EAST;
		if (code == BTN_EAST)
			return BTN_SOUTH;
	}

	if (joy->swap_xy) {
		if (code == BTN_NORTH)
			return BTN_WEST;
		if (code == BTN_WEST)
			return BTN_NORTH;
	}

	return code;
}

/* sysfs 属性 */

/**
 * 属性:
 *
 * /sys/devices/platform/rk3562-joystick/axis_to_dpad [读写]
 * 1 = 左摇杆模拟轴转为十字键上报 (右摇杆不受影响); 写 0 时释放全部
 * 十字键事件, 避免按键卡住.
 */
static ssize_t axis_to_dpad_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct rk3562_joystick *joy = platform_get_drvdata(to_platform_device(dev));

	return sysfs_emit(buf, "%d\n", joy->axis_to_dpad);
}

static ssize_t axis_to_dpad_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct rk3562_joystick *joy = platform_get_drvdata(to_platform_device(dev));
	bool val;
	int ret;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	if (joy->axis_to_dpad && !val) {
		/* 关闭模式时释放按住的十字键 */
		if (joy->last_dpad_up) {
			input_report_key(joy->input, BTN_DPAD_UP, 0);
			joy->last_dpad_up = false;
		}
		if (joy->last_dpad_down) {
			input_report_key(joy->input, BTN_DPAD_DOWN, 0);
			joy->last_dpad_down = false;
		}
		if (joy->last_dpad_left) {
			input_report_key(joy->input, BTN_DPAD_LEFT, 0);
			joy->last_dpad_left = false;
		}
		if (joy->last_dpad_right) {
			input_report_key(joy->input, BTN_DPAD_RIGHT, 0);
			joy->last_dpad_right = false;
		}
		input_sync(joy->input);
	}

	joy->axis_to_dpad = val;

	return count;
}

static DEVICE_ATTR_RW(axis_to_dpad);

/**
 * 属性:
 *
 * /sys/devices/platform/rk3562-joystick/swap_start_home [读写]
 * 1 = 两两交换: START<->BACK(TRIGGER_HAPPY1), SELECT<->HOME/MODE.
 * 四个键 (START/SELECT/MODE/HAPPY1) 不齐的机型返回 -ENODEV.
 */
static ssize_t swap_start_home_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct rk3562_joystick *joy = platform_get_drvdata(to_platform_device(dev));

	return sysfs_emit(buf, "%d\n", joy->swap_start_home);
}

static ssize_t swap_start_home_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct rk3562_joystick *joy = platform_get_drvdata(to_platform_device(dev));
	bool val;
	int ret;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	if (!joy->swap_available)
		return -ENODEV;

	if (val != joy->swap_start_home) {
		/* 两两交换: START<->BACK, SELECT<->HOME/FN */
		joy->gpio_btns[joy->swap_indices[0]].code =
			val ? BTN_TRIGGER_HAPPY1 : BTN_START;     /* START -> BACK */
		joy->gpio_btns[joy->swap_indices[1]].code =
			val ? BTN_MODE : BTN_SELECT;   /* SELECT -> HOME/FN */
		joy->gpio_btns[joy->swap_indices[2]].code =
			val ? BTN_SELECT : BTN_MODE;   /* HOME/FN -> SELECT */
		joy->gpio_btns[joy->swap_indices[3]].code =
			val ? BTN_START : BTN_TRIGGER_HAPPY1;     /* BACK -> START */
		joy->swap_start_home = val;
	}

	return count;
}

static DEVICE_ATTR_RW(swap_start_home);

/**
 * 属性:
 *
 * /sys/devices/platform/rk3562-joystick/invert_absx   [读写]
 * /sys/devices/platform/rk3562-joystick/invert_absy   [读写]
 * /sys/devices/platform/rk3562-joystick/invert_absrx  [读写]
 * /sys/devices/platform/rk3562-joystick/invert_absry  [读写]
 *
 * 与 DT 的 invert-abs* 一一对应, 1 = 该轴取反. 左摇杆模块装反的机型
 * (如 RG52 Mini / RG43H) 由 DT 同时打开 invert-absx + invert-absy;
 * RG43V 这类同 DT 但极性相反的机型可在运行时由 udev 规则归零.
 */
#define RK3562_INV_ATTR(_name, _field)					\
static ssize_t _name##_show(struct device *dev,				\
			    struct device_attribute *attr, char *buf)	\
{									\
	struct rk3562_joystick *joy =					\
		platform_get_drvdata(to_platform_device(dev));		\
									\
	return sysfs_emit(buf, "%d\n", joy->_field);			\
}									\
static ssize_t _name##_store(struct device *dev,			\
			     struct device_attribute *attr,		\
			     const char *buf, size_t count)		\
{									\
	struct rk3562_joystick *joy =					\
		platform_get_drvdata(to_platform_device(dev));		\
	bool val;							\
	int ret;							\
									\
	ret = kstrtobool(buf, &val);					\
	if (ret)							\
		return ret;						\
	joy->_field = val;						\
	return count;							\
}									\
static DEVICE_ATTR_RW(_name)

RK3562_INV_ATTR(invert_absx, inv_x);
RK3562_INV_ATTR(invert_absy, inv_y);
RK3562_INV_ATTR(invert_absrx, inv_rx);
RK3562_INV_ATTR(invert_absry, inv_ry);

/* 运行时死区 (mV, 10..1800; 0 = DT 默认) */
static ssize_t adc_deadzone_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct rk3562_joystick *joy =
		platform_get_drvdata(to_platform_device(dev));

	return sysfs_emit(buf, "%d\n", joy->adc_deadzone);
}

static ssize_t adc_deadzone_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct rk3562_joystick *joy =
		platform_get_drvdata(to_platform_device(dev));
	int val, ret, i;

	ret = kstrtoint(buf, 10, &val);
	if (ret)
		return ret;
	if (val != 0 && (val < 10 || val > 1800))
		return -EINVAL;

	joy->adc_deadzone = val;
	for (i = 0; i < NUM_STICK_CHANS; i++)
		rk3562_recalc_deadzone(joy, i);

	return count;
}
static DEVICE_ATTR_RW(adc_deadzone);

/* 线性扳机上报: 关 = L2/R2 只做数字按键, 不再上报 ABS_Z/ABS_RZ */
static ssize_t linear_triggers_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct rk3562_joystick *joy =
		platform_get_drvdata(to_platform_device(dev));

	return sysfs_emit(buf, "%d\n", joy->linear_triggers);
}

static ssize_t linear_triggers_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct rk3562_joystick *joy =
		platform_get_drvdata(to_platform_device(dev));
	bool val;
	int ret, i;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	joy->linear_triggers = val;

	/* 关闭时把线性轴归零, 避免停留在按下状态 */
	if (!val) {
		for (i = 0; i < NUM_TRIG_CHANS; i++)
			if (joy->trig_chans[i])
				input_report_abs(joy->input, trig_abs_codes[i],
						 TRIG_MIN);
		input_sync(joy->input);
	}

	return count;
}
static DEVICE_ATTR_RW(linear_triggers);

#define RK3562_SWAP_ATTR(_name)						static ssize_t _name##_show(struct device *dev,							    struct device_attribute *attr, char *buf)	{										struct rk3562_joystick *joy =							platform_get_drvdata(to_platform_device(dev));												return sysfs_emit(buf, "%d\n", joy->_name);			}									static ssize_t _name##_store(struct device *dev,						     struct device_attribute *attr,					     const char *buf, size_t count)		{										struct rk3562_joystick *joy =							platform_get_drvdata(to_platform_device(dev));			bool val;								int ret;																	ret = kstrtobool(buf, &val);						if (ret)									return ret;							joy->_name = val;							return count;							}									static DEVICE_ATTR_RW(_name)

RK3562_SWAP_ATTR(swap_ab);
RK3562_SWAP_ATTR(swap_xy);

/* 摇杆校准节点:
 *   cat joypad_tuning  -> "x: raw=<uv> cal=<uv> min=<uv> max=<uv>" (每轴一行)
 *   echo "x <center_uv> <min_uv> <max_uv>" > joypad_tuning
 */
static const char * const tuning_axis_names[NUM_STICK_CHANS] = {
	"x", "y", "rx", "ry",
};

static ssize_t joypad_tuning_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct rk3562_joystick *joy =
		platform_get_drvdata(to_platform_device(dev));
	int i, count = 0;

	for (i = 0; i < NUM_STICK_CHANS; i++)
		count += sysfs_emit_at(buf, count,
				       "%s: raw=%d cal=%d min=%d max=%d\n",
				       tuning_axis_names[i],
				       joy->axis_last_uv[i],
				       joy->axis_center_uv[i],
				       joy->axis_min_uv_a[i],
				       joy->axis_max_uv_a[i]);

	return count;
}

static ssize_t joypad_tuning_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct rk3562_joystick *joy =
		platform_get_drvdata(to_platform_device(dev));
	char axis[8];
	int center, min_uv, max_uv, i;

	if (sscanf(buf, "%7s %d %d %d", axis, &center, &min_uv, &max_uv) != 4)
		return -EINVAL;

	if (min_uv >= center || center >= max_uv || max_uv - min_uv < 200000)
		return -EINVAL;

	for (i = 0; i < NUM_STICK_CHANS; i++) {
		if (strcmp(axis, tuning_axis_names[i]))
			continue;

		joy->axis_center_uv[i] = center;
		joy->axis_min_uv_a[i] = min_uv;
		joy->axis_max_uv_a[i] = max_uv;
		rk3562_recalc_deadzone(joy, i);
		return count;
	}

	return -ENODEV;
}
static DEVICE_ATTR_RW(joypad_tuning);

static struct attribute *rk3562_attrs[] = {
	&dev_attr_axis_to_dpad.attr,
	&dev_attr_swap_start_home.attr,
	&dev_attr_adc_deadzone.attr,
	&dev_attr_linear_triggers.attr,
	&dev_attr_swap_ab.attr,
	&dev_attr_swap_xy.attr,
	&dev_attr_joypad_tuning.attr,
	&dev_attr_invert_absx.attr,
	&dev_attr_invert_absy.attr,
	&dev_attr_invert_absrx.attr,
	&dev_attr_invert_absry.attr,
	NULL,
};
ATTRIBUTE_GROUPS(rk3562);

/*----------------------------------------------------------------------------*/
/* 轮询: 模拟轴 / 扳机 / ADC 按键 */

/**
 * rk3562_poll_sticks() - 读取全部摇杆轴通道并映射.
 * @joy:        驱动上下文
 * @stick_vals: 输出数组 (NUM_STICK_CHANS 个映射值)
 *
 * 读数失败时对应轴置 0 并只警告一次.
 */
static void rk3562_poll_sticks(struct rk3562_joystick *joy,
			       int stick_vals[NUM_STICK_CHANS])
{
	int i, ret, raw;

	for (i = 0; i < NUM_STICK_CHANS; i++) {
		ret = iio_read_channel_processed(joy->stick_chans[i], &raw);
		if (ret < 0) {
			dev_warn_once(joy->dev,
				      "Stick %d (%s): IIO read failed: %d\n",
				      i, stick_chan_names[i], ret);
			stick_vals[i] = 0;
			continue;
		}

		/* iio_read_channel_processed 返回毫伏; 转为微伏 */
		raw *= 1000;

		stick_vals[i] = rk3562_map_stick(joy, raw, i);
	}
}

/**
 * rk3562_poll_triggers() - 读取扳机并上报: 模拟轴 + 数字键双路.
 * @joy:   驱动上下文
 * @input: 输入设备
 *
 * 模拟值映射到 ABS_Z/ABS_RZ; 超过 TRIG_BTN_PRESS_THRESH 时锁定数字
 * BTN_TL2/BTN_TR2, 回落到 TRIG_BTN_RELEASE_THRESH 以下释放 (迟滞).
 * 数字 (GPIO) 扳机机型跳过 (trig_chans[i] == NULL).
 */
static void rk3562_poll_triggers(struct rk3562_joystick *joy,
				 struct input_dev *input)
{
	int i, ret, raw;

	for (i = 0; i < NUM_TRIG_CHANS; i++) {
		int mapped;

		if (!joy->trig_chans[i])
			continue;

		ret = iio_read_channel_processed(joy->trig_chans[i], &raw);
		if (ret < 0)
			continue;

		raw *= 1000;
		mapped = rk3562_map_trigger(joy, raw, i);
		if (joy->linear_triggers)
			input_report_abs(input, trig_abs_codes[i], mapped);

		/* 数字键: 高电平锁定, 低电平释放 (迟滞) */
		if (joy->trig_btn_pressed[i]) {
			if (mapped < TRIG_BTN_RELEASE_THRESH)
				joy->trig_btn_pressed[i] = false;
		} else if (mapped >= TRIG_BTN_PRESS_THRESH) {
			joy->trig_btn_pressed[i] = true;
		}
		input_report_key(input, trig_btn_codes[i],
				 joy->trig_btn_pressed[i]);
	}
}

/**
 * rk3562_poll_adc_buttons() - 读取 ADC 电阻梯按键并上报.
 * @joy:   驱动上下文
 * @input: 输入设备
 *
 * 读数 (微伏) 低于该键按下阈值即为按下. 全 GPIO 机型跳过.
 */
static void rk3562_poll_adc_buttons(struct rk3562_joystick *joy,
				    struct input_dev *input)
{
	int i, ret, raw;

	for (i = 0; i < NUM_ADC_BTNS; i++) {
		if (!joy->adc_btn_chans[i] || !joy->adc_btn_codes[i])
			continue;

		ret = iio_read_channel_processed(joy->adc_btn_chans[i], &raw);
		if (ret < 0)
			continue;

		raw *= 1000; /* 毫伏 -> 微伏 */
		/* 区间判定: press-min-microvolt <= raw < press-threshold-microvolt.
		 * 同一电阻梯上只挂部分按键时 (FN 和音量键共用通道), 用下限把
		 * 别人的电压段排除掉, 避免 FN 被音量键电压误触发. */
		input_report_key(input,
				 rk3562_swap_code(joy, joy->adc_btn_codes[i]),
				 (raw >= joy->adc_btn_min[i] &&
				  raw < joy->adc_btn_thresh[i]) ? 1 : 0);
	}
}

/**
 * rk3562_apply_axis_transforms() - 对一个映射值应用轴变换 (取反链).
 * @joy:  驱动上下文
 * @code: 最终 ABS 码 (XY 交换之后)
 * @val:  映射值
 *
 * 变换顺序: 按最终 ABS 码套用对应单轴反转标志.
 *
 * 返回: 变换后的值
 */
static int rk3562_apply_axis_transforms(struct rk3562_joystick *joy,
					unsigned int code, int val)
{
	if (rk3562_axis_inverted(joy, code))
		val = -val;

	return val;
}

/**
 * rk3562_report_axes_normal() - 常规模式: 上报全部 4 个摇杆轴.
 * @joy:        驱动上下文
 * @input:      输入设备
 * @stick_vals: rk3562_poll_sticks() 的映射值
 *
 * 每轴经过 XY 交换解析 + 取反链后上报.
 */
static void rk3562_report_axes_normal(struct rk3562_joystick *joy,
				      struct input_dev *input,
				      int stick_vals[NUM_STICK_CHANS])
{
	int i;

	for (i = 0; i < NUM_STICK_CHANS; i++) {
		unsigned int code = rk3562_stick_code(joy, i);
		int val = rk3562_apply_axis_transforms(joy, code,
						       stick_vals[i]);

		input_report_abs(input, code, val);
	}
}

/**
 * rk3562_report_axes_dpad() - 十字键模式: 左摇杆逻辑值转十字键.
 * @joy:        驱动上下文
 * @input:      输入设备
 * @stick_vals: rk3562_poll_sticks() 的映射值
 *
 * 取左摇杆 (通道 0/1) 的逻辑 X/Y 与 DPAD_THRESHOLD 比较, 仅在方向
 * 变化时上报十字键; 左摇杆 ABS 轴归零上报 (避免应用读到残留值),
 * 右摇杆 (通道 2/3) 不受影响, 照常上报.
 */
static void rk3562_report_axes_dpad(struct rk3562_joystick *joy,
				    struct input_dev *input,
				    int stick_vals[NUM_STICK_CHANS])
{
	int log_x = 0, log_y = 0;
	bool up, down, left, right;
	int i;

	for (i = 0; i < 2; i++) {
		unsigned int code = rk3562_stick_code(joy, i);
		int val = rk3562_apply_axis_transforms(joy, code,
						       stick_vals[i]);

		if (code == ABS_X)
			log_x = val;
		else
			log_y = val;
	}

	up    = log_y < -DPAD_THRESHOLD;
	down  = log_y >  DPAD_THRESHOLD;
	left  = log_x < -DPAD_THRESHOLD;
	right = log_x >  DPAD_THRESHOLD;

	if (up != joy->last_dpad_up) {
		input_report_key(input, BTN_DPAD_UP, up);
		joy->last_dpad_up = up;
	}
	if (down != joy->last_dpad_down) {
		input_report_key(input, BTN_DPAD_DOWN, down);
		joy->last_dpad_down = down;
	}
	if (left != joy->last_dpad_left) {
		input_report_key(input, BTN_DPAD_LEFT, left);
		joy->last_dpad_left = left;
	}
	if (right != joy->last_dpad_right) {
		input_report_key(input, BTN_DPAD_RIGHT, right);
		joy->last_dpad_right = right;
	}

	/* 左摇杆 ABS 归零, 避免应用看到残留值 */
	input_report_abs(input, rk3562_stick_code(joy, 0), 0);
	input_report_abs(input, rk3562_stick_code(joy, 1), 0);

	/* 右摇杆照常上报 (同样经过取反链, 与常规模式方向一致) */
	for (i = 2; i < NUM_STICK_CHANS; i++) {
		unsigned int code = rk3562_stick_code(joy, i);
		int val = rk3562_apply_axis_transforms(joy, code,
						       stick_vals[i]);

		input_report_abs(input, code, val);
	}
}

/**
 * rk3562_poll() - 主轮询函数, 由输入轮询子系统按固定间隔调用.
 * @input: 输入设备
 *
 * 步骤:
 *   1. 读取摇杆轴通道
 *   2. 读取扳机 (模拟轴 + 数字键)
 *   3. 读取 ADC 按键
 *   4. (GPIO 按键由 IRQ 驱动, 不参与轮询)
 *   5. 按模式上报摇杆轴或十字键
 *   6. 单次 input_sync() 刷新所有待处理事件
 */
static void rk3562_poll(struct input_dev *input)
{
	struct rk3562_joystick *joy = input_get_drvdata(input);
	int stick_vals[NUM_STICK_CHANS];

	/* 1. 摇杆轴 */
	rk3562_poll_sticks(joy, stick_vals);

	/* 2. 扳机 */
	rk3562_poll_triggers(joy, input);

	/* 3. ADC 按键 */
	rk3562_poll_adc_buttons(joy, input);

	/* 5. 摇杆轴 / 十字键 */
	if (joy->axis_to_dpad)
		rk3562_report_axes_dpad(joy, input, stick_vals);
	else
		rk3562_report_axes_normal(joy, input, stick_vals);

	/* 6. 单次同步 */
	input_sync(input);
}

/*----------------------------------------------------------------------------*/
/* 自动校准 */

/**
 * rk3562_calibrate_sticks() - 静息校准各摇杆轴的死区.
 * @joy: 驱动上下文
 *
 * 以静息中心电压 +/- DT 死区半宽作为该轴死区. 中心电压必须
 * 落在量程内缩 200mV 的安全区间内, 否则保留 DT 死区并告警.
 */
static void rk3562_calibrate_sticks(struct rk3562_joystick *joy)
{
	struct device *dev = joy->dev;
	static const char * const axis_labels[] = { "LX", "LY", "RX", "RY" };
	int i, center_mv;

	for (i = 0; i < NUM_STICK_CHANS; i++) {
		center_mv = rk3562_read_avg(joy->stick_chans[i],
					    CALIBRATION_SAMPLES);
		if (center_mv < 0) {
			dev_warn(dev, "Stick %d (%s/%s): calibration read failed (%d), using DT defaults\n",
				 i, axis_labels[i], stick_chan_names[i],
				 center_mv);
			continue;
		}

		/* 校验: 中心应明显落在物理量程内 */
		if (center_mv * 1000 < joy->axis_min_uv + 200000 ||
		    center_mv * 1000 > joy->axis_max_uv - 200000) {
			dev_warn(dev, "Stick %d (%s/%s): center %d mV outside safe range [%d, %d] mV, using DT defaults\n",
				 i, axis_labels[i], stick_chan_names[i],
				 center_mv,
				 (joy->axis_min_uv + 200000) / 1000,
				 (joy->axis_max_uv - 200000) / 1000);
			continue;
		}

		joy->axis_center_uv[i] = center_mv * 1000;
		rk3562_recalc_deadzone(joy, i);

		dev_info(dev, "Stick %d (%s/%s): center %d mV, dead zone [%d, %d] mV\n",
			 i, axis_labels[i], stick_chan_names[i], center_mv,
			 joy->axis_dz_lo_uv[i] / 1000,
			 joy->axis_dz_hi_uv[i] / 1000);
	}
}

/**
 * rk3562_calibrate_triggers() - 静息校准各扳机的量程上限.
 * @joy: 驱动上下文
 *
 * 以静息电压 (松开时被上拉到的电压) 作为该扳机的量程上限. 静息电压
 * 低于 400mV 视为卡键/按下状态, 保留 DT 默认并告警. 不存在的扳机通道
 * (数字扳机机型) 跳过.
 */
static void rk3562_calibrate_triggers(struct rk3562_joystick *joy)
{
	struct device *dev = joy->dev;
	int i, rest_mv;

	for (i = 0; i < NUM_TRIG_CHANS; i++) {
		if (!joy->trig_chans[i])
			continue;

		rest_mv = rk3562_read_avg(joy->trig_chans[i],
					  CALIBRATION_SAMPLES);
		if (rest_mv < 0) {
			dev_warn(dev, "Trigger %d: calibration read failed (%d), using DT default\n",
				 i, rest_mv);
			continue;
		}

		/* 校验: 静息电压应足够高 (扳机未卡住) */
		if (rest_mv < 400) {
			dev_warn(dev, "Trigger %d: rest %d mV too low (stuck/pressed at boot?), using DT default\n",
				 i, rest_mv);
			continue;
		}

		joy->trig_max_uv[i] = rest_mv * 1000;

		dev_info(dev, "Trigger %d: rest %d mV\n", i, rest_mv);
	}
}

/**
 * rk3562_calibrate() - 静息自动校准 (摇杆死区 + 扳机量程).
 * @joy: 驱动上下文
 *
 * 必须在 IIO 通道获取之后, input 设备注册之前调用.
 */
static void rk3562_calibrate(struct rk3562_joystick *joy)
{
	rk3562_calibrate_sticks(joy);
	rk3562_calibrate_triggers(joy);
}

/*----------------------------------------------------------------------------*/
/* 设备树解析与资源获取 */

/**
 * rk3562_parse_basic() - 解析基础属性: 轮询间隔 / 消抖 / 轴变换开关.
 * @joy:           驱动上下文
 * @dev:           设备指针
 * @poll_interval: 输出参数, 轮询间隔 (ms), DT 缺省 8
 *
 * 注意: DT 的 debounce-interval (原厂 100ms) 被有意忽略 -- 原厂自家驱动
 * 也是硬编码 10ms 而不用它; 硅胶微动开关的正确消抖是 5ms.
 */
static void rk3562_parse_basic(struct rk3562_joystick *joy,
			       struct device *dev, u32 *poll_interval)
{
	*poll_interval = 8;
	of_property_read_u32(dev->of_node, "poll-interval", poll_interval);

	joy->debounce_ms = DEFAULT_DEBOUNCE_MS;

	/* 轴 X/Y 交换 */
	joy->l_xy_swap = of_property_read_bool(dev->of_node, "l_xy_swap");
	joy->r_xy_swap = of_property_read_bool(dev->of_node, "r_xy_swap");

	/* 单轴反转 (odroidgo3 属性契约; 左摇杆双轴反转 = 同时打开 x/y) */
	joy->inv_x = of_property_read_bool(dev->of_node, "invert-absx");
	joy->inv_y = of_property_read_bool(dev->of_node, "invert-absy");
	joy->inv_rx = of_property_read_bool(dev->of_node, "invert-absrx");
	joy->inv_ry = of_property_read_bool(dev->of_node, "invert-absry");
}

/**
 * rk3562_parse_stick_range() - 解析摇杆量程与死区 (毫伏 -> 微伏).
 * @joy: 驱动上下文
 * @dev: 设备指针
 *
 * DT 值先作为所有轴的默认死区; 自动校准会覆盖.
 */
static void rk3562_parse_stick_range(struct rk3562_joystick *joy,
				     struct device *dev)
{
	u32 axis_min_mv = 250, axis_max_mv = 1550;
	u32 dz_lo_mv = 790, dz_hi_mv = 1010;
	int i;

	of_property_read_u32(dev->of_node, "axis-min-value-mv", &axis_min_mv);
	of_property_read_u32(dev->of_node, "axis-max-value-mv", &axis_max_mv);
	of_property_read_u32(dev->of_node, "axis-dead-zone-l", &dz_lo_mv);
	of_property_read_u32(dev->of_node, "axis-dead-zone-h", &dz_hi_mv);

	joy->axis_min_uv = axis_min_mv * 1000;
	joy->axis_max_uv = axis_max_mv * 1000;
	for (i = 0; i < NUM_STICK_CHANS; i++) {
		joy->axis_min_uv_a[i] = joy->axis_min_uv;
		joy->axis_max_uv_a[i] = joy->axis_max_uv;
		joy->axis_center_uv[i] = (joy->axis_min_uv + joy->axis_max_uv) / 2;
	}

	/* DT 死区半宽作为所有轴的默认值 (自动校准只改中心) */
	for (i = 0; i < NUM_STICK_CHANS; i++) {
		joy->dt_dz_uv[i] = (dz_hi_mv - dz_lo_mv) * 1000 / 2;
		joy->axis_dz_lo_uv[i] = dz_lo_mv * 1000;
		joy->axis_dz_hi_uv[i] = dz_hi_mv * 1000;
	}
}

/**
 * rk3562_parse_trigger_range() - 解析扳机量程默认值.
 * @joy: 驱动上下文
 * @dev: 设备指针
 *
 * 量程下限使用实测常量 TRIG_MIN_UV (11mV 完全按下地板值) 而非 DT 的
 * l2-r2-min-value-mv; DT 上限作为两扳机默认静息电压 (校准覆盖).
 */
static void rk3562_parse_trigger_range(struct rk3562_joystick *joy,
				       struct device *dev)
{
	u32 trig_max_mv = 800;
	int i;

	of_property_read_u32(dev->of_node, "l2-r2-max-value-mv", &trig_max_mv);

	joy->trig_min_uv = TRIG_MIN_UV;
	for (i = 0; i < NUM_TRIG_CHANS; i++)
		joy->trig_max_uv[i] = trig_max_mv * 1000;
}

/**
 * rk3562_get_channel_optional() - 获取可选 IIO 通道; 不存在时置 NULL.
 * @dev:  设备指针
 * @name: io-channel-names 中的通道名
 *
 * -ENODEV 表示 DTS 未定义该名字的通道 (合法的可选缺失);
 * 其他错误 (含 -EPROBE_DEFER) 原样返回.
 *
 * 返回: 通道指针 (缺失时 NULL) 或错误码
 */
static struct iio_channel *rk3562_get_channel_optional(struct device *dev,
						       const char *name)
{
	struct iio_channel *chan;
	int ret;

	chan = devm_iio_channel_get(dev, name);
	if (IS_ERR(chan)) {
		ret = PTR_ERR(chan);
		if (ret == -ENODEV)
			return NULL;
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to get IIO channel %s: %d\n",
				name, ret);
		return chan;
	}

	return chan;
}

/**
 * rk3562_get_channels() - 获取摇杆与扳机 IIO 通道.
 * @joy: 驱动上下文
 * @dev: 设备指针
 *
 * 摇杆 4 通道 (joy_x/joy_y/joy_rx/joy_ry) 必须存在; 扳机 2 通道
 * (l2-abs/r2-abs) 可选 -- 缺失表示数字 GPIO 扳机机型.
 * ADC 按键通道按子节点名单独获取 (见 rk3562_parse_adc_buttons).
 *
 * 返回: 成功返回 0, 失败返回负错误码
 */
static int rk3562_get_channels(struct rk3562_joystick *joy,
			       struct device *dev)
{
	int i;

	/* 摇杆: 必须存在 */
	for (i = 0; i < NUM_STICK_CHANS; i++) {
		joy->stick_chans[i] = devm_iio_channel_get(dev,
							   stick_chan_names[i]);
		if (IS_ERR(joy->stick_chans[i])) {
			int ret = PTR_ERR(joy->stick_chans[i]);

			if (ret != -EPROBE_DEFER)
				dev_err(dev, "Failed to get IIO channel %s: %d\n",
					stick_chan_names[i], ret);
			return ret;
		}
	}

	/* 扳机: 可选 */
	for (i = 0; i < NUM_TRIG_CHANS; i++) {
		joy->trig_chans[i] = rk3562_get_channel_optional(dev,
								 trig_chan_names[i]);
		if (IS_ERR(joy->trig_chans[i]))
			return PTR_ERR(joy->trig_chans[i]);
	}

	return 0;
}

/**
 * rk3562_parse_adc_buttons() - 解析 ADC 按键子节点并获取各自通道.
 * @joy: 驱动上下文
 * @dev: 设备指针
 *
 * odroidgo3 风格: 无 gpios 而带 linux,code 的子节点是 ADC 按键,
 * 其 IIO 通道名 = 子节点名 (在顶层 io-channel-names 中按名查找):
 *
 *   b-key {
 *           label = "BTN_B";
 *           linux,code = <BTN_SOUTH>;
 *           press-threshold-microvolt = <80000>;   // 缺省 80mV
 *   };
 *
 * 0 个 ADC 子节点合法 (面键全 GPIO 的机型, 如 NGP45HT).
 *
 * 返回: 成功返回 0, 失败返回负错误码
 */
static int rk3562_parse_adc_buttons(struct rk3562_joystick *joy,
				    struct device *dev)
{
	struct device_node *node = dev->of_node;
	struct device_node *child;
	int i = 0;
	int ret;

	for_each_child_of_node(node, child) {
		u32 code, thresh, min_uv;

		/* GPIO 按键子节点 (带 gpios) 在别处处理 */
		if (of_find_property(child, "gpios", NULL))
			continue;

		/* 无键码的节点忽略 */
		if (of_property_read_u32(child, "linux,code", &code))
			continue;

		if (i >= NUM_ADC_BTNS) {
			of_node_put(child);
			break;
		}

		/* 通道名 = 子节点名 */
		joy->adc_btn_chans[i] = devm_iio_channel_get(dev, child->name);
		if (IS_ERR(joy->adc_btn_chans[i])) {
			ret = PTR_ERR(joy->adc_btn_chans[i]);
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "ADC button %s: no io-channel named after this node: %d\n",
					child->name, ret);
			of_node_put(child);
			return ret;
		}

		if (of_property_read_u32(child, "press-threshold-microvolt",
					 &thresh))
			thresh = 80000; /* 缺省 80 mV */

		if (of_property_read_u32(child, "press-min-microvolt", &min_uv))
			min_uv = 0;

		joy->adc_btn_codes[i] = code;
		joy->adc_btn_thresh[i] = thresh;
		joy->adc_btn_min[i] = min_uv;
		i++;
	}

	if (i)
		dev_info(dev, "%d ADC buttons\n", i);

	return 0;
}

/**
 * rk3562_setup_gpio_buttons() - 解析 GPIO 按键子节点并注册双边沿 IRQ.
 * @joy: 驱动上下文
 * @dev: 设备指针
 *
 * odroidgo3 风格: 每个带 gpios 属性的子节点定义一个按键 (数量动态):
 *
 *   sw1 {
 *           label = "BTN_L1";
 *           gpios = <&gpio1 RK_PC7 GPIO_ACTIVE_LOW>;
 *           linux,code = <BTN_TL>;
 *   };
 *
 * 键码经 rk3562_remap_code() 重映射 (KEY_HOME/KEY_FN -> joydev 可见码);
 * 直接写 BTN_* 码时重映射为空操作.
 *
 * 返回: 成功返回 0, 失败返回负错误码
 */
static int rk3562_setup_gpio_buttons(struct rk3562_joystick *joy,
				     struct device *dev)
{
	struct device_node *node = dev->of_node;
	struct device_node *child;
	int count = 0;
	int i = 0;
	int ret;

	/* 第一遍: 统计带 gpios 的子节点 */
	for_each_child_of_node(node, child) {
		if (of_find_property(child, "gpios", NULL))
			count++;
	}

	if (count == 0) {
		dev_err(dev, "No GPIO button child nodes found\n");
		return -EINVAL;
	}

	joy->num_gpio_btns = count;
	joy->gpio_btns = devm_kcalloc(dev, count, sizeof(*joy->gpio_btns),
				      GFP_KERNEL);
	if (!joy->gpio_btns)
		return -ENOMEM;

	/* 第二遍: 逐键获取 GPIO + IRQ */
	for_each_child_of_node(node, child) {
		struct rk3562_gpio_btn *btn;
		const char *label = NULL;
		u32 code;

		if (!of_find_property(child, "gpios", NULL))
			continue;

		btn = &joy->gpio_btns[i++];
		btn->joy = joy;
		INIT_DELAYED_WORK(&btn->work, rk3562_gpio_work_func);

		ret = of_property_read_u32(child, "linux,code", &code);
		if (ret) {
			dev_err(dev, "Button %s missing linux,code\n",
				child->name);
			of_node_put(child);
			return -EINVAL;
		}
		btn->code = rk3562_remap_code(code);

		of_property_read_string(child, "label", &label);
		if (!label)
			label = child->name;

		btn->gpiod = devm_fwnode_gpiod_get_index(dev,
							 of_fwnode_handle(child),
							 NULL, 0, GPIOD_IN,
							 label);
		if (IS_ERR(btn->gpiod)) {
			ret = PTR_ERR(btn->gpiod);
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "Failed to get GPIO for %s: %d\n",
					child->name, ret);
			of_node_put(child);
			return ret;
		}

		btn->irq = gpiod_to_irq(btn->gpiod);
		if (btn->irq < 0) {
			dev_err(dev, "Failed to get IRQ for %s: %d\n",
				child->name, btn->irq);
			of_node_put(child);
			return btn->irq;
		}

		ret = devm_request_any_context_irq(dev, btn->irq,
						   rk3562_gpio_isr,
						   IRQF_TRIGGER_RISING |
						   IRQF_TRIGGER_FALLING,
						   DRIVER_NAME, btn);
		if (ret < 0) {
			dev_err(dev, "Failed to request IRQ %d for %s: %d\n",
				btn->irq, label, ret);
			of_node_put(child);
			return ret;
		}
	}

	return devm_add_action_or_reset(dev, rk3562_cancel_gpio_work, joy);
}

/**
 * rk3562_find_swap_buttons() - 定位 swap_start_home 功能需要的四个键.
 * @joy: 驱动上下文
 *
 * 在 GPIO 按键数组中查找 START / SELECT / MODE / TRIGGER_HAPPY1 四个码,
 * 记录下标; 四键齐全时 swap_available 置位.
 */
static void rk3562_find_swap_buttons(struct rk3562_joystick *joy)
{
	static const unsigned int swap_codes[4] = {
		BTN_START, BTN_SELECT, BTN_MODE, BTN_TRIGGER_HAPPY1,
	};
	struct device *dev = joy->dev;
	int found = 0;
	int i, j;

	for (i = 0; i < 4; i++)
		joy->swap_indices[i] = -1;

	for (i = 0; i < joy->num_gpio_btns; i++) {
		for (j = 0; j < 4; j++) {
			if (joy->gpio_btns[i].code == swap_codes[j]) {
				joy->swap_indices[j] = i;
				found++;
				break;
			}
		}
	}

	joy->swap_available = (found == 4);
	if (joy->swap_available)
		dev_info(dev, "Button swap available (START=%d SELECT=%d MODE=%d HAPPY1=%d)\n",
			 joy->swap_indices[0], joy->swap_indices[1],
			 joy->swap_indices[2], joy->swap_indices[3]);
	else
		dev_info(dev, "Button swap not available (found %d/4 buttons)\n",
			 found);
}

/**
 * rk3562_setup_rumble() - 获取振动马达 GPIO (可选).
 * @joy: 驱动上下文
 * @dev: 设备指针
 *
 * rumble-gpios (主) 与 rumble-r-gpios (副) 均可选; 输出初始为低 (马达关).
 *
 * 返回: 成功返回 0, 失败返回负错误码
 */
static int rk3562_setup_rumble(struct rk3562_joystick *joy,
			       struct device *dev)
{
	joy->rumble_gpio = devm_gpiod_get_optional(dev, "rumble", GPIOD_OUT_LOW);
	if (IS_ERR(joy->rumble_gpio))
		return dev_err_probe(dev, PTR_ERR(joy->rumble_gpio),
				     "Failed to get rumble GPIO\n");

	joy->rumble_r_gpio = devm_gpiod_get_optional(dev, "rumble-r",
						    GPIOD_OUT_LOW);
	if (IS_ERR(joy->rumble_r_gpio))
		return dev_err_probe(dev, PTR_ERR(joy->rumble_r_gpio),
				     "Failed to get rumble-r GPIO\n");

	return 0;
}

/*----------------------------------------------------------------------------*/
/* 输入设备注册 */

/**
 * rk3562_input_setup() - 创建并注册输入设备.
 * @joy:           驱动上下文
 * @poll_interval: 轮询间隔 (ms)
 *
 * 能力注册顺序: 摇杆轴 -> 扳机轴 -> GPIO/ADC 按键 -> joydev 索引稳定键 ->
 * 十字键 -> 力反馈 -> 轮询.
 *
 * BTN_TL2/BTN_TR2/BTN_MODE 无条件注册, 保证所有 RK3562 机型上 joydev
 * 分配相同的顺序按键索引, 无论 HOME/FN 物理键是否存在 (RG52 Mini 有,
 * RG43H 没有). BTN_TR2 (evdev 0x139) 无条件置位以填补 BTN_TL2 (0x138)
 * 与 BTN_SELECT (0x13a) 之间的空洞 -- 缺了它 joydev 会把 SELECT 收缩为
 * 按钮 7 而非 8, 其后所有按键 (十字键等) 顺次前移一位, 打破按标准
 * SDL_GameController 布局写的模拟器配置. input_set_capability 幂等.
 * BTN_TL2/BTN_TR2 另由 rk3562_poll() 作为数字扳机驱动 (与 ABS_Z/ABS_RZ
 * 模拟轴并行).
 *
 * 返回: 成功返回 0, 失败返回负错误码
 */
static int rk3562_input_setup(struct rk3562_joystick *joy, u32 poll_interval)
{
	struct device *dev = joy->dev;
	struct input_dev *input;
	int ret, i;

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	joy->input = input;
	input->name = DRIVER_NAME;
	input->id.bustype = BUS_HOST;
	input->id.vendor = 0;
	input->id.product = 0;
	input->id.version = 0x0100;

	input_set_drvdata(input, joy);

	/* 摇杆轴 */
	for (i = 0; i < NUM_STICK_CHANS; i++)
		input_set_abs_params(input, stick_abs_codes[i],
				     AXIS_MIN, AXIS_MAX, 16, 128);

	/* 扳机轴 (校准后无需大的 fuzz/flat).
	 * 模拟扳机通道缺失 (数字 GPIO 扳机) 时跳过. */
	for (i = 0; i < NUM_TRIG_CHANS; i++) {
		if (!joy->trig_chans[i])
			continue;

		input_set_abs_params(input, trig_abs_codes[i],
				     TRIG_MIN, TRIG_MAX,
				     TRIG_FUZZ_CALIBRATED,
				     TRIG_FLAT_CALIBRATED);
	}

	/* GPIO 按键 */
	for (i = 0; i < joy->num_gpio_btns; i++)
		input_set_capability(input, EV_KEY, joy->gpio_btns[i].code);

	/* ADC 按键 (仅注册 DT 中实际定义的) */
	for (i = 0; i < NUM_ADC_BTNS; i++) {
		if (joy->adc_btn_codes[i])
			input_set_capability(input, EV_KEY,
					     joy->adc_btn_codes[i]);
	}

	/* joydev 索引稳定键 (见函数头注释) */
	input_set_capability(input, EV_KEY, BTN_TL2);
	input_set_capability(input, EV_KEY, BTN_TR2);
	input_set_capability(input, EV_KEY, BTN_MODE);

	/* 十字键能力 (摇杆转十字键模式用) */
	input_set_capability(input, EV_KEY, BTN_DPAD_UP);
	input_set_capability(input, EV_KEY, BTN_DPAD_DOWN);
	input_set_capability(input, EV_KEY, BTN_DPAD_LEFT);
	input_set_capability(input, EV_KEY, BTN_DPAD_RIGHT);

	/* 力反馈 (有马达时) */
	if (joy->rumble_gpio || joy->rumble_r_gpio) {
		input_set_capability(input, EV_FF, FF_RUMBLE);
		ret = input_ff_create_memless(input, NULL,
					      rk3562_play_effect);
		if (ret) {
			dev_err(dev, "Failed to create FF device: %d\n", ret);
			return ret;
		}
	}

	/* 轮询设置 (模拟轴与 ADC 按键) */
	ret = input_setup_polling(input, rk3562_poll);
	if (ret) {
		dev_err(dev, "Failed to setup polling: %d\n", ret);
		return ret;
	}

	input_set_poll_interval(input, poll_interval);
	input_set_min_poll_interval(input, 8);
	input_set_max_poll_interval(input, 100);

	ret = input_register_device(input);
	if (ret) {
		dev_err(dev, "Failed to register input device: %d\n", ret);
		return ret;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
/* 电源管理 */

/**
 * rk3562_suspend() - 挂起时关闭马达.
 */
static int rk3562_suspend(struct device *dev)
{
	struct rk3562_joystick *joy = dev_get_drvdata(dev);

	if (joy->rumble_gpio)
		gpiod_set_value_cansleep(joy->rumble_gpio, 0);
	if (joy->rumble_r_gpio)
		gpiod_set_value_cansleep(joy->rumble_r_gpio, 0);

	return 0;
}

/**
 * rk3562_resume() - 恢复时还原挂起前的振动状态.
 */
static int rk3562_resume(struct device *dev)
{
	struct rk3562_joystick *joy = dev_get_drvdata(dev);

	if (joy->rumble_on) {
		if (joy->rumble_gpio)
			gpiod_set_value_cansleep(joy->rumble_gpio, 1);
		if (joy->rumble_r_gpio)
			gpiod_set_value_cansleep(joy->rumble_r_gpio, 1);
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(rk3562_pm_ops, rk3562_suspend, rk3562_resume);

/*----------------------------------------------------------------------------*/
/* 探测 */

/**
 * rk3562_probe() - 平台设备探测.
 * @pdev: 平台设备
 *
 * 流程: 解析 DT 基础属性 -> 摇杆/扳机量程 -> IIO 通道 -> 静息自动校准 ->
 * ADC 按键子节点 -> GPIO 按键 IRQ -> 可换绑键定位 -> 振动马达 ->
 * 输入设备注册.
 *
 * 返回: 成功返回 0, 失败返回负错误码
 */
static int rk3562_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rk3562_joystick *joy;
	u32 poll_interval;
	int ret;

	joy = devm_kzalloc(dev, sizeof(*joy), GFP_KERNEL);
	if (!joy)
		return -ENOMEM;

	joy->dev = dev;
	platform_set_drvdata(pdev, joy);
	joy->linear_triggers = true;

	/* 1. DT 基础属性 + 量程 */
	rk3562_parse_basic(joy, dev, &poll_interval);
	rk3562_parse_stick_range(joy, dev);
	rk3562_parse_trigger_range(joy, dev);

	/* 2. IIO 通道 */
	ret = rk3562_get_channels(joy, dev);
	if (ret)
		return ret;

	/* 2.5 尽早接管马达脚: pinctrl 应用后脚呈高阻, 这里立刻驱为低,
	 *      后续任何解析失败也不再让马达保持上电状态 (上电常震的教训).
	 *      马达是可选功能: 获取失败仅禁用力反馈, 不中止 probe. */
	ret = rk3562_setup_rumble(joy, dev);
	if (ret) {
		joy->rumble_gpio = NULL;
		joy->rumble_r_gpio = NULL;
		dev_warn(dev, "rumble unavailable: %d\n", ret);
		ret = 0;
	}

	/* 3. 静息自动校准 (摇杆死区 + 扳机量程) */
	rk3562_calibrate(joy);

	/* 4. ADC 按键子节点 */
	ret = rk3562_parse_adc_buttons(joy, dev);
	if (ret)
		return ret;

	/* 5. GPIO 按键 (IRQ 驱动) */
	ret = rk3562_setup_gpio_buttons(joy, dev);
	if (ret)
		return ret;

	/* 6. 定位 Start/Select <-> HOME/BACK 交换需要的键 */
	rk3562_find_swap_buttons(joy);

	/* 7. 振动马达 — 已在第 2.5 步尽早初始化, 无需重复 */

	/* 8. 输入设备注册 */
	ret = rk3562_input_setup(joy, poll_interval);
	if (ret)
		return ret;

	dev_info(dev, "RK3562 joystick registered (%d GPIOs, poll %u ms, debounce %u ms, rumble %s, analog-triggers %s)\n",
		 joy->num_gpio_btns, poll_interval, joy->debounce_ms,
		 (joy->rumble_gpio || joy->rumble_r_gpio) ? "yes" : "no",
		 joy->trig_chans[0] ? "yes" : "no");

	if (joy->inv_x || joy->inv_y || joy->inv_rx || joy->inv_ry)
		dev_info(dev, "Per-axis inversion: x %s y %s rx %s ry %s\n",
			 joy->inv_x ? "on" : "-",
			 joy->inv_y ? "on" : "-",
			 joy->inv_rx ? "on" : "-",
			 joy->inv_ry ? "on" : "-");

	return 0;
}

/*----------------------------------------------------------------------------*/
/* 驱动注册 */

static const struct of_device_id rk3562_of_match[] = {
	{ .compatible = "play_joystick" },
	{ },
};
MODULE_DEVICE_TABLE(of, rk3562_of_match);

static struct platform_driver rk3562_driver = {
	.probe = rk3562_probe,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = rk3562_of_match,
		.pm = &rk3562_pm_ops,
		.dev_groups = rk3562_groups,
	},
};
module_platform_driver(rk3562_driver);

MODULE_DESCRIPTION("RK3562 handheld gamepad driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("dArkOS contributors");
