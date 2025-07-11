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

#include "power.h"

#define DFU_DBL_RESET_MEM 0x20007F7C
#define DFU_DBL_RESET_APP 0x4ee5677e

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

LOG_MODULE_REGISTER(power, LOG_LEVEL_INF);

static void disable_DFU_thread(void);
K_THREAD_DEFINE(disable_DFU_thread_id, 128, disable_DFU_thread, NULL, NULL, NULL, 6, 0, 500); // disable DFU if the system is running correctly

static void power_thread(void);
K_THREAD_DEFINE(power_thread_id, 1024, power_thread, NULL, NULL, NULL, 6, 0, 0);

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

#define ADAFRUIT_BOOTLOADER CONFIG_BUILD_OUTPUT_UF2

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
	// TODO: not calling suspend here, because sensor can call it and stop the system from shutting down since it suspended itself
//	main_imu_suspend(); // TODO: when the thread is suspended, its possibly suspending in the middle of an i2c transaction and this is bad. Instead sensor should be suspended at a different time
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

#if IMU_INT_EXISTS && CONFIG_DELAY_SLEEP_ON_STATUS
static int64_t system_off_timeout = 0;
#endif

void sys_request_WOM(bool force) // TODO: if IMU interrupt does not exist what does the system do?
{
	LOG_INF("IMU wake up requested");
#if IMU_INT_EXISTS
#if CONFIG_DELAY_SLEEP_ON_STATUS
	if (!force && (!esb_ready() || !status_ready())) // Wait for esb to pair in case the user is still trying to pair the device
	{
		if (!system_off_timeout)
			system_off_timeout = k_uptime_get() + 30000; // allow system off after 30 seconds if status errors are still active
		if (k_uptime_get() < system_off_timeout)
		{
			LOG_INF("IMU wake up not available, waiting on ESB/status ready");
			return; // not timed out yet, skip system off
		}
		LOG_INF("ESB/status ready timed out");
		// this may mean the system never enters system off if sys_request_WOM is not called again after the timeout
	}
#endif
	configure_system_off(); // Common subsystem shutdown and prepare sense pins
	sensor_retained_write();
#if WOM_USE_DCDC // In case DCDC is more efficient in the 10-100uA range
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

void sys_request_system_off(void) // TODO: add timeout
{
	LOG_INF("System off requested");
	// TODO: fails, its possible that it is getting stuck at main_imu_suspend
	main_imu_suspend(); // TODO: should be a common shutdown step
	configure_system_off(); // Common subsystem shutdown and prepare sense pins
	// Clear sensor addresses
	sensor_scan_clear();
	LOG_INF("Requested sensor scan on next boot");
//	sensor_retained_write();
	set_regulator(SYS_REGULATOR_LDO); // Switch to LDO
	// Set system off
	LOG_INF("Powering off nRF");
	sys_update_battery_tracker(current_battery_pptt, device_plugged);
//	retained_update();
	wait_for_logging();
#if ADAFRUIT_BOOTLOADER // if using Adafruit bootloader, always skip dfu for next boot
	(*dbl_reset_mem) = DFU_DBL_RESET_APP; // Skip DFU
#endif
	sys_poweroff();
}

void sys_request_system_reboot(void) // TODO: add timeout
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

static void power_thread(void)
{
	while (1)
	{
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(uart0))
		const struct device *const uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
		pm_device_action_run(uart, PM_DEVICE_ACTION_SUSPEND);
#endif
		bool docked = dock_read();
		bool charging = chg_read();
		bool charged = stby_read();

		int battery_mV;
		int16_t battery_pptt = read_batt_mV(&battery_mV);
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
				set_status(SYS_STATUS_SYSTEM_ERROR, true);
			}
			set_regulator(SYS_REGULATOR_DCDC); // Switch to DCDC
			power_init = true;
		}

		if (battery_discharged || docked)
		{
			if (battery_discharged)
			{
				LOG_WRN("Discharged battery");
				sys_update_battery_tracker(0, device_plugged);
			}
			sys_request_system_off();
		}

		if (battery_available && !battery_low && battery_pptt < 1000)
			battery_low = true;
		else if (!battery_available || (battery_low && battery_pptt > 1500)) // hysteresis
			battery_low = false;

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

		sys_update_battery_tracker_voltage(battery_mV, device_plugged);
		if (samples == BATTERY_SAMPLES || device_plugged)
			sys_update_battery_tracker(current_battery_pptt, device_plugged);
		calibrated_battery_pptt = sys_get_calibrated_battery_pptt(current_battery_pptt);

		connection_update_battery(battery_available, device_plugged, calibrated_battery_pptt, battery_mV);

		if (charging)
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
