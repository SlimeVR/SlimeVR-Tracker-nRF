#include "globals.h"

#include <math.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>

#include "led.h"

LOG_MODULE_REGISTER(led, LOG_LEVEL_INF);

static void led_thread(void);
K_THREAD_DEFINE(led_thread_id, 512, led_thread, NULL, NULL, NULL, LED_THREAD_PRIORITY, 0, 0);

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, led_en_gpios)
#define LED_EN_EXISTS true
static const struct gpio_dt_spec led_en = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, led_en_gpios);
#endif

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, led_gpios)
#define LED_EXISTS true
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, led_gpios);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led0))
#ifndef LED_EXISTS
#define LED_EXISTS true
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#else
#define LED0_EXISTS true
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#endif
#endif
#ifndef LED_EXISTS
#warning "LED GPIO does not exist"
//static const struct gpio_dt_spec led = {0};
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led1))
#define LED1_EXISTS true
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led2))
#define LED2_EXISTS true
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led3))
#define LED3_EXISTS true
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios);
#endif

#if DT_NODE_EXISTS(DT_ALIAS(pwm_led0))
#define PWM_LED_EXISTS true
static const struct pwm_dt_spec pwm_led = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));
#else
#warning "PWM LED node does not exist"
#endif
#if DT_NODE_EXISTS(DT_ALIAS(pwm_led1))
#define PWM_LED1_EXISTS true
static const struct pwm_dt_spec pwm_led1 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led1));
#endif
#if DT_NODE_EXISTS(DT_ALIAS(pwm_led2))
#define PWM_LED2_EXISTS true
static const struct pwm_dt_spec pwm_led2 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led2));
#endif

static enum sys_led_pattern current_led_pattern;
static int current_priority;

#if LED_EXISTS
static enum sys_led_pattern led_patterns[SYS_LED_PATTERN_DEPTH] = {[0 ... (SYS_LED_PATTERN_DEPTH - 1)] = SYS_LED_PATTERN_OFF};
static int led_pattern_state;
static int64_t led_pattern_start_time;

static int led_pin_init(void)
{
	LOG_DBG("led_pin_init");
	gpio_pin_configure_dt(&led, GPIO_OUTPUT);
	gpio_pin_set_dt(&led, 0);
#if LED0_EXISTS
	gpio_pin_configure_dt(&led0, GPIO_OUTPUT);
	gpio_pin_set_dt(&led0, 0);
#endif
#if LED1_EXISTS
	gpio_pin_configure_dt(&led1, GPIO_OUTPUT);
	gpio_pin_set_dt(&led1, 0);
#endif
#if LED2_EXISTS
	gpio_pin_configure_dt(&led2, GPIO_OUTPUT);
	gpio_pin_set_dt(&led2, 0);
#endif
#if LED3_EXISTS
	gpio_pin_configure_dt(&led3, GPIO_OUTPUT);
	gpio_pin_set_dt(&led3, 0);
#endif
	return 0;
}

SYS_INIT(led_pin_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

static void led_pin_reset(void)
{
	LOG_DBG("led_pin_reset");
	gpio_pin_configure_dt(&led, GPIO_DISCONNECTED);
#if LED0_EXISTS
	gpio_pin_configure_dt(&led0, GPIO_DISCONNECTED);
#endif
#if LED1_EXISTS
	gpio_pin_configure_dt(&led1, GPIO_DISCONNECTED);
#endif
#if LED2_EXISTS
	gpio_pin_configure_dt(&led2, GPIO_DISCONNECTED);
#endif
#if LED3_EXISTS
	gpio_pin_configure_dt(&led3, GPIO_DISCONNECTED);
#endif
}

static void led_suspend(void)
{
	LOG_DBG("led_suspend");
#ifdef PWM_LED_EXISTS
	pm_device_action_run(pwm_led.dev, PM_DEVICE_ACTION_SUSPEND);
#endif
#ifdef PWM_LED1_EXISTS
	pm_device_action_run(pwm_led1.dev, PM_DEVICE_ACTION_SUSPEND);
#endif
#ifdef PWM_LED2_EXISTS
	pm_device_action_run(pwm_led2.dev, PM_DEVICE_ACTION_SUSPEND);
#endif
	led_pin_reset();
	// disable power
#if LED_EN_EXISTS
	gpio_pin_configure_dt(&led_en, GPIO_OUTPUT);
	gpio_pin_set_dt(&led_en, 0);
#endif
}

static void led_resume(void)
{
	LOG_DBG("led_resume");
	// enable power
#if LED_EN_EXISTS
	gpio_pin_configure_dt(&led_en, GPIO_OUTPUT);
	gpio_pin_set_dt(&led_en, 1);
#endif
#ifdef PWM_LED_EXISTS
	pm_device_action_run(pwm_led.dev, PM_DEVICE_ACTION_RESUME);
#endif
#ifdef PWM_LED1_EXISTS
	pm_device_action_run(pwm_led1.dev, PM_DEVICE_ACTION_RESUME);
#endif
#ifdef PWM_LED2_EXISTS
	pm_device_action_run(pwm_led2.dev, PM_DEVICE_ACTION_RESUME);
#endif
	led_pin_init();
}

#ifdef CONFIG_LED_RGB_COLOR
#define LED_RGB_COLOR
#define LED_RG_COLOR
#endif

#if PWM_LED_EXISTS && PWM_LED1_EXISTS && PWM_LED2_EXISTS
#define LED_TRI_COLOR
#else
#undef LED_RGB_COLOR
#undef LED_TRI_COLOR
#if PWM_LED_EXISTS && PWM_LED1_EXISTS
#define LED_DUAL_COLOR
#else
#undef LED_RG_COLOR
#undef LED_DUAL_COLOR
#endif
#endif

#ifdef LED_RGB_COLOR
static int led_pwm_period[8][3] = {
	{CONFIG_LED_DEFAULT_COLOR_R, CONFIG_LED_DEFAULT_COLOR_G, CONFIG_LED_DEFAULT_COLOR_B}, // Default
	{10000, 0, 10000}, // Success (purple)
	{10000, 0, 0}, // Error
	{9000, 0, 9000}, // Charging (purple)
	{10000, 3000, 0}, // Low battery
	{0, 0, 10000}, // Pairing
	{0, 10000, 0}, // Pairing success
	{0, 0, 0}, // Dynamic runtime color
};
#elif defined(LED_TRI_COLOR)
static int led_pwm_period[8][3] = {
	{0, 0, 10000}, // Default
	{10000, 3000, 7000}, // Success (approx purple)
	{10000, 0, 0}, // Error
	{9000, 2000, 6000}, // Charging (approx purple)
	{9000, 4000, 0}, // Low battery
	{0, 0, 10000}, // Pairing
	{0, 10000, 0}, // Pairing success
	{0, 0, 0}, // Dynamic runtime color
};
#elif defined(LED_RG_COLOR)
static int led_pwm_period[8][2] = {
	{CONFIG_LED_DEFAULT_COLOR_R, CONFIG_LED_DEFAULT_COLOR_G}, // Default
	{10000, 2000}, // Success (approx purple)
	{10000, 0}, // Error
	{9000, 1500}, // Charging (approx purple)
	{9000, 3000}, // Low battery
	{0, 10000}, // Pairing
	{0, 10000}, // Pairing success
	{0, 0}, // Dynamic runtime color
};
#elif defined(LED_DUAL_COLOR)
static int led_pwm_period[8][2] = {
	{0, 10000}, // Default
	{10000, 3000}, // Success (approx purple)
	{10000, 0}, // Error
	{9000, 2000}, // Charging (approx purple)
	{9000, 4000}, // Low battery
	{0, 10000}, // Pairing
	{0, 10000}, // Pairing success
	{0, 0}, // Dynamic runtime color
};
#else
static int led_pwm_period[8][1] = {
	{10000}, // Default
	{10000}, // Success
	{10000}, // Error
	{10000}, // Charging
	{10000}, // Low battery
	{10000}, // Pairing
	{10000}, // Pairing success
	{0}, // Dynamic runtime color
};
#endif

static inline int clamp_pwm_value(int value)
{
	if (value < 0)
		return 0;
	if (value > 10000)
		return 10000;
	return value;
}

static void led_set_dynamic_color(int r, int g, int b)
{
#if defined(LED_RGB_COLOR) || defined(LED_TRI_COLOR)
	led_pwm_period[SYS_LED_COLOR_DYNAMIC][0] = clamp_pwm_value(r);
	led_pwm_period[SYS_LED_COLOR_DYNAMIC][1] = clamp_pwm_value(g);
	led_pwm_period[SYS_LED_COLOR_DYNAMIC][2] = clamp_pwm_value(b);
#elif defined(LED_RG_COLOR) || defined(LED_DUAL_COLOR)
	int mixed_r = clamp_pwm_value(r + b / 2);
	int mixed_g = clamp_pwm_value(g + b / 2);
	led_pwm_period[SYS_LED_COLOR_DYNAMIC][0] = mixed_r;
	led_pwm_period[SYS_LED_COLOR_DYNAMIC][1] = mixed_g;
#else
	int max_val = r;
	if (g > max_val)
		max_val = g;
	if (b > max_val)
		max_val = b;
	led_pwm_period[SYS_LED_COLOR_DYNAMIC][0] = clamp_pwm_value(max_val);
#endif
}

// Using brightness and value if PWM is supported, otherwise value is coerced to on/off
// TODO: use computed constants for high/low brightness and color values
static void led_pin_set(enum sys_led_color color, int brightness_pptt, int value_pptt)
{
	LOG_DBG("led_pin_set: color %d, brightness %d, value %d", color, brightness_pptt, value_pptt);
	if (brightness_pptt < 0)
		brightness_pptt = 0;
	else if (brightness_pptt > 10000)
		brightness_pptt = 10000;
	if (value_pptt < 0)
		value_pptt = 0;
	else if (value_pptt > 10000)
		value_pptt = 10000;
#if PWM_LED_EXISTS
	value_pptt = value_pptt * brightness_pptt / 10000;
	// only supporting color if PWM is supported
	pwm_set_pulse_dt(&pwm_led, pwm_led.period / 10000 * (led_pwm_period[color][0] * value_pptt / 10000));
#if PWM_LED1_EXISTS
	pwm_set_pulse_dt(&pwm_led1, pwm_led1.period / 10000 * (led_pwm_period[color][1] * value_pptt / 10000));
#if PWM_LED2_EXISTS
	pwm_set_pulse_dt(&pwm_led2, pwm_led2.period / 10000 * (led_pwm_period[color][2] * value_pptt / 10000));
#endif
#endif
#else
	gpio_pin_set_dt(&led, value_pptt > 5000);
#endif
}
#endif

void set_led(enum sys_led_pattern led_pattern, int priority)
{
	LOG_DBG("set_led: current_led_pattern %d, current_priority %d", current_led_pattern, current_priority);
	LOG_DBG("set_led: pattern %d, priority %d", led_pattern, priority);
#if LED_EXISTS
	if (led_pattern <= SYS_LED_PATTERN_OFF && k_current_get() == led_thread_id)
		led_patterns[current_priority] = led_pattern;
	else
		led_patterns[priority] = led_pattern;
	for (priority = 0; priority < SYS_LED_PATTERN_DEPTH; priority++)
	{
		if (led_patterns[priority] == SYS_LED_PATTERN_OFF)
			continue;
		led_pattern = led_patterns[priority];
		break;
	}
	if (led_pattern == current_led_pattern && led_pattern > SYS_LED_PATTERN_OFF)
		return;
	current_led_pattern = led_pattern;
	current_priority = priority;
	led_pattern_state = 0;
	led_pattern_start_time = k_uptime_get();
	if (current_led_pattern <= SYS_LED_PATTERN_OFF)
	{
		led_suspend();
		k_thread_suspend(led_thread_id);
		LOG_DBG("set_led: suspended led_thread_id");
	}
	else if (k_current_get() != led_thread_id) // do not suspend if called from thread
	{
		k_thread_suspend(led_thread_id);
		LOG_DBG("set_led: suspended led_thread_id");
		led_resume();
		k_thread_resume(led_thread_id);
		k_wakeup(led_thread_id);
		LOG_DBG("set_led: resumed led_thread_id");
	}
	else
	{
		led_resume();
		k_thread_resume(led_thread_id);
		k_wakeup(led_thread_id);
		LOG_DBG("set_led: resumed led_thread_id");
	}
#endif
}

static void led_thread(void)
{
#if !LED_EXISTS
	LOG_WRN("LED GPIO does not exist");
	return;
#else
	while (1)
	{
		LOG_DBG("led_thread: current_led_pattern %d", current_led_pattern);
		switch (current_led_pattern)
		{
		case SYS_LED_PATTERN_ON:
			led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, 10000);
			k_thread_suspend(led_thread_id);
			break;
	case SYS_LED_PATTERN_SHORT: {
		const int pairing_step_time_ms = 10;
		const int pairing_cycle_ms = 500;
		const int pairing_fade_ms = 250;
		const int pairing_half_fade_ms = pairing_fade_ms / 2;
		const int pairing_cycle_steps = pairing_cycle_ms / pairing_step_time_ms;
		int step = led_pattern_state;
		int phase_ms = (step * pairing_step_time_ms) % pairing_cycle_ms;
		int led_value = 0;

		if (phase_ms < pairing_fade_ms) {
			int ramp_ms = phase_ms < pairing_half_fade_ms ? phase_ms : (pairing_fade_ms - phase_ms);
			led_value = ramp_ms * 10000 / pairing_half_fade_ms;
		}

		led_pin_set(SYS_LED_COLOR_PAIRING, 10000, led_value);

		led_pattern_state = (step + 1) % pairing_cycle_steps;
		k_msleep(pairing_step_time_ms);
		break;
	}

		case SYS_LED_PATTERN_LONG:
			led_pattern_state = (led_pattern_state + 1) % 2;
			led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, led_pattern_state * 10000);
			k_msleep(500);
			break;
		case SYS_LED_PATTERN_FLASH:
			led_pattern_state = (led_pattern_state + 1) % 2;
			led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, led_pattern_state * 10000);
			k_msleep(200);
			break;

		case SYS_LED_PATTERN_ONESHOT_POWERON:
			led_pattern_state++;
			led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, !(led_pattern_state % 2) * 10000);
			if (led_pattern_state == 7)
				set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_HIGHEST);
			else
				k_msleep(200);
			break;
		case SYS_LED_PATTERN_ONESHOT_POWEROFF:
			if (led_pattern_state++ > 0)
				led_pin_set(SYS_LED_COLOR_DEFAULT, (202 - led_pattern_state) * 50, (led_pattern_state != 202 ? 10000 : 0));
			else
				led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, 0);
			if (led_pattern_state == 202)
				set_led(SYS_LED_PATTERN_OFF_FORCE, SYS_LED_PRIORITY_HIGHEST);
			else if (led_pattern_state == 1)
				k_msleep(250);
			else
				k_msleep(5);
			break;
		case SYS_LED_PATTERN_ONESHOT_PROGRESS:
			led_pattern_state++;
			led_pin_set(SYS_LED_COLOR_SUCCESS, 10000, !(led_pattern_state % 2) * 10000);
			if (led_pattern_state == 5)
				set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_HIGHEST);
			else
				k_msleep(200);
			break;
		case SYS_LED_PATTERN_ONESHOT_COMPLETE:
			led_pattern_state++;
			led_pin_set(SYS_LED_COLOR_PAIRING_SUCCESS, 10000, !(led_pattern_state % 2) * 10000);
			if (led_pattern_state == 9)
				set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_HIGHEST);
			else
				k_msleep(200);
			break;

		case SYS_LED_PATTERN_ON_PERSIST:
			led_pin_set(SYS_LED_COLOR_SUCCESS, 2000, 10000);
			k_thread_suspend(led_thread_id);
			break;
	case SYS_LED_PATTERN_LONG_PERSIST:
		led_pattern_state = (led_pattern_state + 1) % 2;
		led_pin_set(SYS_LED_COLOR_LOW_BATTERY, 2000, led_pattern_state * 10000);
			k_msleep(500);
			break;
		case SYS_LED_PATTERN_PULSE_PERSIST:
			led_pattern_state = (led_pattern_state + 1) % 1000;
//			float led_value = sinf(led_pattern_state * (M_PI / 1000));
//			led_pin_set(SYS_LED_COLOR_CHARGING, 10000, led_value * 10000);
			int led_value = led_pattern_state > 500 ? 1000 - led_pattern_state : led_pattern_state;
			if (led_value < 200)
				led_value = (led_value) * 30;
			else if (led_value < 300)
				led_value = (led_value - 200) * 20 + 6000;
			else if (led_value < 400)
				led_value = (led_value - 300) * 15 + 8000;
			else
				led_value = (led_value - 400) * 5 + 9500;
			led_pin_set(SYS_LED_COLOR_CHARGING, 10000, led_value);
			k_msleep(5);
			break;
		case SYS_LED_PATTERN_ACTIVE_PERSIST:
		{
			const int cycle_ms = 10000;
			const int fade_ms = 300;
			int64_t elapsed = k_uptime_get() - led_pattern_start_time;
			if (elapsed < 0)
				elapsed = 0;
			int phase = (int)(elapsed % cycle_ms);
			if (phase < fade_ms)
			{
				int half = fade_ms / 2;
				int ramp = phase < half ? phase : (fade_ms - phase);
				int value = half ? ramp * 10000 / half : 10000;
				led_pin_set(SYS_LED_COLOR_SUCCESS, 10000, value);
				k_msleep(10);
			}
			else
			{
				led_pin_set(SYS_LED_COLOR_SUCCESS, 10000, 0);
				int sleep_ms = cycle_ms - phase;
				if (sleep_ms < 10)
					sleep_ms = 10;
				k_msleep(sleep_ms);
			}
			break;
		}

		case SYS_LED_PATTERN_UNCALIBRATED_ALERT:
		{
			const int cycle_ms = 5000;
			const int flash_on_ms = 80;
			const int flash_gap_ms = 60;
			int64_t elapsed = k_uptime_get() - led_pattern_start_time;
			if (elapsed < 0)
				elapsed = 0;
			int phase = (int)(elapsed % cycle_ms);
			int value = 0;
			if (phase < flash_on_ms)
				value = 10000;
			else if (phase < flash_on_ms + flash_gap_ms)
				value = 0;
			else if (phase < flash_on_ms + flash_gap_ms + flash_on_ms)
				value = 10000;
			else
				value = 0;
			led_pin_set(SYS_LED_COLOR_LOW_BATTERY, 10000, value);
			int sleep_ms;
			if (phase < flash_on_ms + flash_gap_ms + flash_on_ms)
				sleep_ms = 10;
			else
			{
				sleep_ms = cycle_ms - phase;
				if (sleep_ms < 20)
					sleep_ms = 20;
			}
			k_msleep(sleep_ms);
			break;
		}

		case SYS_LED_PATTERN_CALIBRATION_PROGRESS:
		{
			const int total_ms = 12000;
			int64_t now = k_uptime_get();
			int64_t raw_elapsed = now - led_pattern_start_time;
			if (raw_elapsed < 0)
				raw_elapsed = 0;
			int64_t limited_elapsed = raw_elapsed > total_ms ? total_ms : raw_elapsed;
			int half_span = total_ms / 2;
			int red = 10000;
			if (limited_elapsed > half_span)
			{
				int decay = (int)((limited_elapsed - half_span) * 10000 / (total_ms - half_span));
				red = clamp_pwm_value(10000 - decay);
			}
			int green = 4000 + (int)(limited_elapsed * 6000 / total_ms);
			if (green > 10000)
				green = 10000;
			led_set_dynamic_color(red, green, 0);

			double progress = (double)limited_elapsed / (double)total_ms;
			if (progress > 1.0)
				progress = 1.0;
			double freq_hz = 0.5 + progress * 99.5;
			if (freq_hz < 0.5)
				freq_hz = 0.5;
			double cycle_ms = 1000.0 / freq_hz;
			double phase = fmod((double)raw_elapsed, cycle_ms);
			double half_cycle = cycle_ms / 2.0;
			double ratio;
			if (half_cycle <= 0.0)
			{
				ratio = 1.0;
			}
			else if (phase < half_cycle)
			{
				ratio = phase / half_cycle;
			}
			else
			{
				ratio = (cycle_ms - phase) / half_cycle;
			}
			if (ratio < 0.0)
				ratio = 0.0;
			if (ratio > 1.0)
				ratio = 1.0;
			int value = (int)(ratio * 10000.0);
			if (value > 10000)
				value = 10000;
			led_pin_set(SYS_LED_COLOR_DYNAMIC, 10000, value);
			int sleep_ms = (int)(cycle_ms / 4.0);
			if (sleep_ms < 1)
				sleep_ms = 1;
			k_msleep(sleep_ms);
			break;
		}

		case SYS_LED_PATTERN_CALIBRATION_SUCCESS:
		{
			int64_t elapsed = k_uptime_get() - led_pattern_start_time;
			if (elapsed < 0)
				elapsed = 0;
			led_set_dynamic_color(10000, 0, 10000);
			if (elapsed < 2000)
			{
				int envelope = 10000 - (int)(elapsed * 10000 / 2000);
				if (envelope < 0)
					envelope = 0;
				double cycle_ms = 50.0; // 20 Hz flash while fading down
				double phase = fmod((double)elapsed, cycle_ms);
				double half_cycle = cycle_ms / 2.0;
				double ratio;
				if (half_cycle <= 0.0)
				{
					ratio = 1.0;
				}
				else if (phase < half_cycle)
				{
					ratio = phase / half_cycle;
				}
				else
				{
					ratio = (cycle_ms - phase) / half_cycle;
				}
				if (ratio < 0.0)
					ratio = 0.0;
				if (ratio > 1.0)
					ratio = 1.0;
				int value = (int)(ratio * envelope);
				led_pin_set(SYS_LED_COLOR_DYNAMIC, 10000, value);
				int sleep_ms = (int)(cycle_ms / 4.0);
				if (sleep_ms < 1)
					sleep_ms = 1;
				k_msleep(sleep_ms);
			}
			else if (elapsed < 4000)
			{
				led_pin_set(SYS_LED_COLOR_DYNAMIC, 10000, 10000);
				k_msleep(10);
			}
			else
			{
				set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
			}
			break;
		}

		case SYS_LED_PATTERN_ERROR_A: // TODO: should this use 20% duty cycle?
			led_pattern_state = (led_pattern_state + 1) % 10;
			led_pin_set(SYS_LED_COLOR_ERROR, 10000, (led_pattern_state < 4 && led_pattern_state % 2) * 10000);
			k_msleep(500);
			break;
		case SYS_LED_PATTERN_ERROR_B:
			led_pattern_state = (led_pattern_state + 1) % 10;
			led_pin_set(SYS_LED_COLOR_ERROR, 10000, (led_pattern_state < 6 && led_pattern_state % 2) * 10000);
			k_msleep(500);
			break;
		case SYS_LED_PATTERN_ERROR_C:
			led_pattern_state = (led_pattern_state + 1) % 10;
			led_pin_set(SYS_LED_COLOR_ERROR, 10000, (led_pattern_state < 8 && led_pattern_state % 2) * 10000);
			k_msleep(500);
			break;
		case SYS_LED_PATTERN_ERROR_D:
			led_pattern_state = (led_pattern_state + 1) % 2;
			led_pin_set(SYS_LED_COLOR_ERROR, 10000, led_pattern_state * 10000);
			k_msleep(500);
			break;

		default:
			LOG_DBG("led_thread: suspending led_thread_id");
			k_thread_suspend(led_thread_id);
		}
	}
#endif
}
