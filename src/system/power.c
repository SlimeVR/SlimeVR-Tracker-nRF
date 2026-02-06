/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include "globals.h"
#include "sensor/sensor.h"
#include "battery.h"
#include "battery_tracker.h"
#include "connection/connection.h"
#include "system.h"
#include "led.h"
#include "connection/esb.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/reboot.h>
#include <hal/nrf_gpio.h>
#include <zephyr/pm/device.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>

#include "power.h"
#include "clock_control.h"

#define DFU_DBL_RESET_MEM 0x20007F7C
#define DFU_DBL_RESET_APP 0x4ee5677e
#define ADAFRUIT_BOOTLOADER CONFIG_BUILD_OUTPUT_UF2

static uint32_t *dbl_reset_mem __attribute__((unused)) = ((uint32_t *)DFU_DBL_RESET_MEM); // retained

enum sys_regulator {
	SYS_REGULATOR_DCDC,
	SYS_REGULATOR_LDO
};

#define BATTERY_SAMPLES 24

static int16_t calibrated_battery_pptt = -1;
static int16_t current_battery_pptt = -1;
static int32_t hysteresis_pptt = -1;
static int32_t average_pptt = -1;
static int16_t last_pptt[BATTERY_SAMPLES - 1] = {[0 ... BATTERY_SAMPLES - 2] = -1};
static int last_pptt_index = 0;
static uint8_t samples = 0;
static bool battery_low = false;

static bool plugged = false;
static bool power_init = false;
static bool device_plugged = false;
static bool device_charged = false;

static bool chg_temp_warn = false;
static int64_t last_valid_temp = -1;

LOG_MODULE_REGISTER(power, LOG_LEVEL_INF);

static void sys_WOM(bool force);
static void sys_system_off(void);
static void sys_system_reboot(void);

static int sys_power_state_request(int id);

static void disable_DFU_thread(void);
K_THREAD_DEFINE(disable_DFU_thread_id, 128, disable_DFU_thread, NULL, NULL, NULL, DISABLE_DFU_THREAD_PRIORITY, 0, 500); // disable DFU if the system is running correctly

static void power_thread(void);
K_THREAD_DEFINE(power_thread_id, 1024, power_thread, NULL, NULL, NULL, POWER_THREAD_PRIORITY, 0, 0);

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, int0_gpios)
#define IMU_INT_EXISTS true
#else
#warning "IMU wake up GPIO does not exist"
#endif
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dcdc_gpios)
#define DCDC_EN_EXISTS true
static const struct gpio_dt_spec dcdc_en = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, dcdc_gpios);
#else
#pragma message "DCDC enable GPIO does not exist"
#endif
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, ldo_gpios)
#define LDO_EN_EXISTS true
static const struct gpio_dt_spec ldo_en = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, ldo_gpios);
#else
#pragma message "LDO enable GPIO does not exist"
#endif
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, chg_en_gpios)
#define CHG_EN_EXISTS true
static const struct gpio_dt_spec chg_en = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, chg_en_gpios);
#else
#pragma message "Charge enable GPIO does not exist"
#endif

#define ADAFRUIT_BOOTLOADER CONFIG_BUILD_OUTPUT_UF2

static void sys_disconnect_interface_pins(void)
{
	// interface pins are disconnected according to devicetree, so only need to disconnect any cs pins
	// int pin already configured by power off
#if DT_SPI_DEV_HAS_CS_GPIOS(DT_NODELABEL(imu_spi))
	uint32_t imu_cs_gpios = DT_SPI_DEV_CS_GPIOS_PIN(DT_NODELABEL(imu_spi));
	LOG_INF("IMU CS GPIO pin: %u", imu_cs_gpios);
	nrf_gpio_cfg_default(imu_cs_gpios);
	LOG_INF("Disconnected IMU CS GPIO");
#endif
#if DT_SPI_DEV_HAS_CS_GPIOS(DT_NODELABEL(mag_spi))
	uint32_t mag_cs_gpios = DT_SPI_DEV_CS_GPIOS_PIN(DT_NODELABEL(mag_spi)));
	LOG_INF("Magnetometer CS GPIO pin: %u", mag_cs_gpios);
	nrf_gpio_cfg_default(mag_cs_gpios);
	LOG_INF("Disconnected Magnetometer CS GPIO");
#endif
/*
	TODO: for promicro, leaving ext_vcc on draws ~50uA, disconnect works, pulldown may be more reliable
	what to do about boards that use ext_vcc? it is not expected to leave on during WOM
*/
}

void sys_interface_suspend(void)
{
#if DT_NODE_HAS_STATUS_OKAY(DT_PARENT(DT_NODELABEL(imu_spi)))
	const struct device *const pm_spi_imu = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(imu_spi)));
	pm_device_action_run(pm_spi_imu, PM_DEVICE_ACTION_SUSPEND);
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_PARENT(DT_NODELABEL(imu)))
	const struct device *const pm_i2c_imu = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(imu)));
	pm_device_action_run(pm_i2c_imu, PM_DEVICE_ACTION_SUSPEND);
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_PARENT(DT_NODELABEL(mag_spi)))
	const struct device *const pm_spi_mag = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(mag_spi)));
	pm_device_action_run(pm_spi_mag, PM_DEVICE_ACTION_SUSPEND);
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_PARENT(DT_NODELABEL(mag)))
	const struct device *const pm_i2c_mag = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(mag)));
	pm_device_action_run(pm_i2c_mag, PM_DEVICE_ACTION_SUSPEND);
#endif
}

void sys_interface_resume(void)
{
#if DT_NODE_HAS_STATUS_OKAY(DT_PARENT(DT_NODELABEL(imu_spi)))
	const struct device *const pm_spi_imu = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(imu_spi)));
	pm_device_action_run(pm_spi_imu, PM_DEVICE_ACTION_RESUME);
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_PARENT(DT_NODELABEL(imu)))
	const struct device *const pm_i2c_imu = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(imu)));
	pm_device_action_run(pm_i2c_imu, PM_DEVICE_ACTION_RESUME);
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_PARENT(DT_NODELABEL(mag_spi)))
	const struct device *const pm_spi_mag = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(mag_spi)));
	pm_device_action_run(pm_spi_mag, PM_DEVICE_ACTION_RESUME);
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_PARENT(DT_NODELABEL(mag)))
	const struct device *const pm_i2c_mag = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(mag)));
	pm_device_action_run(pm_i2c_mag, PM_DEVICE_ACTION_RESUME);
#endif
}

// TODO: the gpio sense is weird, maybe the device will turn back on immediately after shutdown or after (attempting to) enter WOM
// TODO: there should be a better system of how to handle all system_off cases and all the sense pins
// TODO: just changed it make sure to test it thanks

// TODO: should the tracker start again if docking state changes?
// TODO: keep sending battery state while plugged and docked?
// TODO: on some boards there is actual power path, try to use the LED in this case
// TODO: usually charging, i would flash LED but that will drain the battery while it is charging..
// TODO: should not really shut off while plugged in

static void configure_system_off(void)
{
	if (get_status(SYS_STATUS_SENSOR_ERROR))
		LOG_WRN("Entering new power state while sensor error is raised");
	if (get_status(SYS_STATUS_SYSTEM_ERROR))
		LOG_WRN("Entering new power state while system error is raised");
	clock_pre_shutdown();
	main_imu_suspend();
	sensor_shutdown();
	set_led(SYS_LED_PATTERN_OFF_FORCE, SYS_LED_PRIORITY_HIGHEST);
	float actual_clock_rate;
	set_sensor_clock(false, 0, &actual_clock_rate);
	// Configure interrupts
	configure_sense_pins();
}

static void set_regulator(enum sys_regulator regulator)
{
#if DCDC_EN_EXISTS
	bool use_dcdc = regulator == SYS_REGULATOR_DCDC;
	if (use_dcdc)
	{
		gpio_pin_set_dt(&dcdc_en, 1);
		LOG_INF("Enabled DCDC");
	}
#endif
#if LDO_EN_EXISTS
	bool use_ldo = regulator == SYS_REGULATOR_LDO;
	gpio_pin_set_dt(&ldo_en, use_ldo);
	LOG_INF("%s", use_ldo ? "Enabled LDO" : "Disabled LDO");
#endif
#if DCDC_EN_EXISTS
	if (!use_dcdc)
	{
		gpio_pin_set_dt(&dcdc_en, 0);
		LOG_INF("Disabled DCDC");
	}
#endif
}

static int set_charger_enable(bool enable, bool plugged)
{
#if CHG_EN_EXISTS
	static bool last_chg_en = false;
	if (!plugged)
		enable = false; // always disable charger if not plugged
	if (enable != last_chg_en)
	{
		last_chg_en = enable;
		gpio_pin_set_dt(&chg_en, enable);
		LOG_INF("%s", enable ? "Enabled charger" : "Disabled charger");
	}
#elif CONFIG_BATTERY_CHARGER_HAS_NTC
	// charger has implemented NTC and does not have enable pin, can ignore
#else
	static bool err = false;
	if (!enable && plugged)
	{
		if (!err)
			LOG_ERR("Cannot disable charger");
		err = true;
		return -1;
	}
	err = false;
#endif
	return 0;
}

static void wait_for_logging(void)
{
#if CONFIG_LOG_BACKEND_UART
	// only UART backend is disabled usually
	const struct log_backend *uart_backend = log_backend_get_by_name("log_backend_uart");
	if (!uart_backend)
		return;
	bool uart_active = log_backend_is_active(uart_backend);
	if (uart_active)
	{
		LOG_INF("Delayed for UART backend");
		k_msleep(200);
	}
#endif
}

#if IMU_INT_EXISTS
static int64_t system_off_timeout = 0;
#endif

void sys_request_WOM(bool force, bool immediate)
{
	if (immediate)
	{
		sys_WOM(force);
		return;
	}
	if (force)
		sys_power_state_request(2);
	else
		sys_power_state_request(1);
}

void sys_request_system_off(bool immediate)
{
	if (immediate)
	{
		sys_system_off();
		return;
	}
	sys_power_state_request(3);
}

void sys_request_system_reboot(bool immediate)
{
	if (immediate)
	{
		sys_system_reboot();
		return;
	}
	sys_power_state_request(4);
}

static void sys_WOM(bool force) // TODO: if IMU interrupt does not exist what does the system do?
{
	LOG_INF("IMU wake up requested");
#if IMU_INT_EXISTS
	if (CONFIG_0_SETTINGS_READ(CONFIG_0_DELAY_SLEEP_ON_STATUS) && !force && (!esb_ready() || !status_ready())) // Wait for esb to pair in case the user is still trying to pair the device
	{
		if (!system_off_timeout)
			system_off_timeout = k_uptime_get() + 30000; // allow system off after 30 seconds if status errors are still active
		if (k_uptime_get() < system_off_timeout)
		{
			LOG_INF("IMU wake up not available, waiting on ESB/status ready");
			return; // not timed out yet, skip system off
		}
		LOG_INF("ESB/status ready timed out");
		// TODO: this may mean the system never enters system off if sys_request_WOM is not called again after the timeout
	}
	configure_system_off(); // Common subsystem shutdown and prepare sense pins
	sensor_retained_write();
#if CONFIG_WOM_USE_DCDC // In case DCDC is more efficient in the ~10-100uA range
	set_regulator(SYS_REGULATOR_DCDC); // Make sure DCDC is selected
#else
	set_regulator(SYS_REGULATOR_LDO); // Switch to LDO
#endif
	// Set system off
	uint8_t pin_config = sensor_setup_WOM(); // enable WOM feature
	LOG_INF("Configured IMU wake up");
	// Configure WOM interrupt
	uint32_t int0_gpios = NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, int0_gpios);
	LOG_INF("Wake up GPIO pin: %u, config: %u", int0_gpios, pin_config);
	nrf_gpio_cfg_input(int0_gpios, (pin_config >> 4) & 0xF);
	nrf_gpio_cfg_sense_set(int0_gpios, pin_config & 0xF);
	LOG_INF("Configured IMU wake up GPIO");
	LOG_INF("Powering off nRF");
	sys_update_battery_tracker(current_battery_pptt, device_plugged);
//	retained_update();
	wait_for_logging();
#if ADAFRUIT_BOOTLOADER // if using Adafruit bootloader, always skip dfu for next boot
	(*dbl_reset_mem) = DFU_DBL_RESET_APP; // Skip DFU
#endif
	sys_poweroff();
#else
	LOG_WRN("IMU wake up GPIO does not exist");
	LOG_WRN("IMU wake up not available");
#endif
}

static void sys_system_off(void) // TODO: add timeout
{
	LOG_INF("System off requested");
	configure_system_off(); // Common subsystem shutdown and prepare sense pins
	// Clear sensor addresses
	sensor_scan_clear();
	LOG_INF("Requested sensor scan on next boot");
//	sensor_retained_write();
	set_regulator(SYS_REGULATOR_LDO); // Switch to LDO
	// Set system off
#if IMU_INT_EXISTS
	// Configure interrupt pin as it is not used
	uint32_t int0_gpios = NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, int0_gpios);
	LOG_INF("Wake up GPIO pin: %u", int0_gpios);
	nrf_gpio_cfg(int0_gpios, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);
	LOG_INF("Disconnected IMU wake up GPIO");
#endif
	// Disconnect remaining interface pins // TODO: only an improvement during shutdown? causes higher usage in WOM
	sys_disconnect_interface_pins();
	LOG_INF("Powering off nRF");
	sys_update_battery_tracker(current_battery_pptt, device_plugged);
//	retained_update();
	wait_for_logging();
#if ADAFRUIT_BOOTLOADER // if using Adafruit bootloader, always skip dfu for next boot
	(*dbl_reset_mem) = DFU_DBL_RESET_APP; // Skip DFU
#endif
	sys_poweroff();
}

static void sys_system_reboot(void) // TODO: add timeout
{
	LOG_INF("System reboot requested");
	configure_system_off(); // Common subsystem shutdown and prepare sense pins
//	sensor_retained_write();
	// Set system reboot
	LOG_INF("Rebooting nRF");
	sys_update_battery_tracker(current_battery_pptt, device_plugged);
//	retained_update();
	wait_for_logging();
#if ADAFRUIT_BOOTLOADER // if using Adafruit bootloader, always skip dfu for next boot
	(*dbl_reset_mem) = DFU_DBL_RESET_APP; // Skip DFU
#endif
	sys_reboot(SYS_REBOOT_COLD);
}

static int sys_power_state_request(int id)
{
	static int requested = 0;
	switch (id)
	{
	case -1:
		requested = 0;
		return 0;
	case 0:
		return requested;
	default:
		if (requested != 0)
		{
			LOG_ERR("System is already entering a new power state");
			return -1;
		}
		requested = id;
		return 0;
	}
}

bool vin_read(void) // blocking
{
	while (!power_init)
		k_usleep(1); // wait for first battery read
	return plugged;
}

static void disable_DFU_thread(void)
{
#if ADAFRUIT_BOOTLOADER
	(*dbl_reset_mem) = DFU_DBL_RESET_APP; // Skip DFU
#endif
}

static void update_battery(int16_t battery_pptt)
{
	// Plugged state will cause a sudden change in SOC >10%, so reset the sample array
	if (average_pptt >= 0 && NRFX_ABS(battery_pptt - average_pptt) > 1000)
	{
		LOG_INF("Change to battery SOC: %5.2f%% -> %5.2f%%", (double)average_pptt / 100.0, (double)battery_pptt / 100.0);
		memset(last_pptt, -1, sizeof(last_pptt)); // reset array
		samples = 1;
	}

	// Initalize sorted array
	int16_t sorted_pptt[BATTERY_SAMPLES];
	memcpy(sorted_pptt, last_pptt, sizeof(last_pptt));
	sorted_pptt[BATTERY_SAMPLES - 1] = battery_pptt;

	// Now add the last reading to the sample array
	last_pptt[last_pptt_index] = battery_pptt;
	last_pptt_index++;
	last_pptt_index %= BATTERY_SAMPLES - 1;

	// Sort sample array
	for (int i = 1; i < BATTERY_SAMPLES; i++)
	{
		int16_t key = sorted_pptt[i];
		int8_t j = i - 1;
		while (j >= 0 && sorted_pptt[j] > key)
		{
			sorted_pptt[j + 1] = sorted_pptt[j];
			j = j - 1;
		}
		sorted_pptt[j + 1] = key;
	}

	// Average across median 75% of samples
	average_pptt = 0;
	uint8_t valid_samples = 0;
	for (uint8_t i = BATTERY_SAMPLES - (samples - samples / 8); i < (BATTERY_SAMPLES - samples / 8); i++)
	{
		if (sorted_pptt[i] != -1)
		{
			average_pptt += sorted_pptt[i];
			valid_samples++;
		}
	}
	if (valid_samples > 0)
		average_pptt /= valid_samples;
	else
		average_pptt = battery_pptt;

	// Store the average battery level with hysteresis (Effectively 100-10000 -> 1-100%)
	if (average_pptt + 100 < hysteresis_pptt) // Lower bound -100pptt
		hysteresis_pptt = average_pptt + 100;
	else if (average_pptt > hysteresis_pptt) // Upper bound +0pptt
		hysteresis_pptt = average_pptt;

	// 0% to battery tracker will reset it, as >1% to 0% is invalid change
	// Instead, remap 1-100 to 0-100
	current_battery_pptt = (hysteresis_pptt - 100) * 100 / 99;
}

// TODO: this thread is handling reading charging state, battery state, dock state, and setting status/led
// TODO: should be separated to be more clear in its function?
// TODO: call into other thread for handling the system state
static void power_thread(void)
{
	while (1)
	{
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(uart0))
		const struct device *const uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
		pm_device_action_run(uart, PM_DEVICE_ACTION_SUSPEND);
#endif
		int requested = sys_power_state_request(0);
		switch (requested)
		{
		case 1:
			sys_WOM(false);
			break;
		case 2:
			sys_WOM(true);
			break;
		case 3:
			sys_system_off();
			break;
		case 4:
			sys_system_reboot();
			break;
		default:
			break;
		}
		sys_power_state_request(-1); // clear request

		bool docked = dock_read();
		bool charging = chg_read();
		bool charged = stby_read();

		float temp;
		int chg_ret = sensor_get_sensor_temperature(&temp);
		switch (chg_ret)
		{
		case -2:
			last_valid_temp = -1;
			chg_temp_warn = true;
			break;
		case -1:
			if (last_valid_temp == -1)
				last_valid_temp = k_uptime_get();
			if (k_uptime_get() - last_valid_temp > 1000) // valid read timeout
				chg_temp_warn = true;
			break;
		case 0:
			last_valid_temp = k_uptime_get();
			// https://www.batteryuniversity.com/article/bu-410-charging-at-high-and-low-temperatures/
			if (!chg_temp_warn && (temp < 5.f || temp > 45.f)) // this is still safe (hard limit is 0C, but that is dangerous)
				chg_temp_warn = true;
			else if (chg_temp_warn && temp > 10.f && temp < 40.f) // safest range
				chg_temp_warn = false;
		default:
			break;
		}
		chg_ret = set_charger_enable(!chg_temp_warn, device_plugged);
		// chg_ret = -1: outside safe temp range, but charger could not be disabled
		// chg_ret = 0 and chg_temp_warn = true: out of temp range, charger is disabled or already has thermistor

		int battery_mV;
		int16_t battery_pptt = read_batt_mV(&battery_mV);
		if (battery_level_pptt < 0)
			LOG_ERR("Failed to read battery voltage: %d", battery_pptt);
		if (samples < BATTERY_SAMPLES)
			samples++;

		bool abnormal_reading = battery_mV < 100 || battery_mV > 6000;
		bool battery_available = battery_mV > 1500 && !abnormal_reading; // Keep working without the battery connected, otherwise it is obviously too dead to boot system
		bool battery_discharged = battery_available && (average_pptt >= 0 ? average_pptt : battery_pptt) == 0;
		// Separate detection of vin
		if (!plugged && battery_mV > 4300 && !abnormal_reading)
			plugged = true;
		else if ((plugged && battery_mV <= 4250) || abnormal_reading)
			plugged = false;
#ifdef POWER_USBREGSTATUS_VBUSDETECT_Msk
		bool usb_plugged = NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk;
#else
		bool usb_plugged = false;
#endif

		if (!device_plugged && (charging || charged || plugged || usb_plugged))
		{
			device_plugged = true;
			set_status(SYS_STATUS_PLUGGED, true);
		}
		else if (device_plugged && !(charging || charged || plugged || usb_plugged))
		{
			device_plugged = false;
			set_status(SYS_STATUS_PLUGGED, false);
		}

		device_charged = charged; // TODO: timer on device_plugged could be used to infer charged state

		static bool adc_abnormal = false;
		if (!power_init)
		{
			// log battery state once
			if (battery_available)
				LOG_INF("Battery %u%% (%d mV)", battery_pptt / 100, battery_mV);
			else
				LOG_INF("Battery not available (%d mV)", battery_mV);
			if (abnormal_reading)
			{
				LOG_ERR("Battery voltage reading is abnormal");
				adc_abnormal = true;
			}
			set_regulator(SYS_REGULATOR_DCDC); // Switch to DCDC
			power_init = true;
		}

		if ((battery_discharged && !device_plugged) || docked) // TODO: docked may or may not also mean device_plugged due to charging
		{
			if (battery_discharged)
			{
				LOG_WRN("Discharged battery");
				sys_update_battery_tracker(0, device_plugged);
			}
			sys_request_system_off(true);
		}

		// will update average_pptt, and current_battery_pptt
		update_battery(battery_pptt);

		// use estimated remaining runtime or pptt for battery_low
		uint64_t runtime = sys_get_battery_remaining_time_estimate();
		bool runtime_valid = runtime > 0;
		bool runtime_low = k_ticks_to_ms_floor64(runtime) < CONFIG_3_SETTINGS_READ(CONFIG_3_BATTERY_LOW_RUNTIME_THRESHOLD);
		bool pptt_low = current_battery_pptt < 1000;
		if (battery_available && !battery_low && (runtime_valid ? runtime_low : pptt_low))
			battery_low = true;
		else if (!battery_available || (battery_low && (runtime_valid ? !runtime_low : !pptt_low))) // hysteresis already provided
			battery_low = false;

		sys_update_battery_tracker_voltage(battery_mV, device_plugged);
		if (samples == BATTERY_SAMPLES || device_plugged)
			sys_update_battery_tracker(current_battery_pptt, device_plugged);
		calibrated_battery_pptt = sys_get_calibrated_battery_pptt(current_battery_pptt);

		connection_update_battery(battery_available, device_plugged, device_charged, calibrated_battery_pptt, battery_mV);

		if ((adc_abnormal || chg_ret) && !get_status(SYS_STATUS_SYSTEM_ERROR))
			set_status(SYS_STATUS_SYSTEM_ERROR, true);
		else if ((!adc_abnormal && !chg_ret) && get_status(SYS_STATUS_SYSTEM_ERROR))
			set_status(SYS_STATUS_SYSTEM_ERROR, false);

		if (chg_ret)
			set_led(SYS_LED_PATTERN_CRITICAL, SYS_LED_PRIORITY_CRITICAL);
		else
			set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_CRITICAL);

		if (chg_temp_warn && plugged) // don't need to warn if not plugged in
			set_led(SYS_LED_PATTERN_WARNING, SYS_LED_PRIORITY_SYSTEM); // not critical
		else if (charging)
			set_led(SYS_LED_PATTERN_PULSE_PERSIST, SYS_LED_PRIORITY_SYSTEM);
		else if (charged)
			set_led(SYS_LED_PATTERN_ON_PERSIST, SYS_LED_PRIORITY_SYSTEM);
		else if (plugged || usb_plugged)
			set_led(SYS_LED_PATTERN_PULSE_PERSIST, SYS_LED_PRIORITY_SYSTEM);
		else if (battery_low)
			set_led(SYS_LED_PATTERN_LONG_PERSIST, SYS_LED_PRIORITY_SYSTEM);
		else
			set_led(SYS_LED_PATTERN_ACTIVE_PERSIST, SYS_LED_PRIORITY_SYSTEM);
//			set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SYSTEM);

		k_msleep(100);
	}
}
