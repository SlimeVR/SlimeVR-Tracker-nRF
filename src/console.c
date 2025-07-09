#include "globals.h"
#include "system/system.h"
#include "system/battery_tracker.h"
#include "sensor/sensor.h"
#include "sensor/calibration.h"
#include "connection/esb.h"
#include "build_defines.h"

#if CONFIG_USB_DEVICE_STACK
#define USB DT_NODELABEL(usbd)
#define USB_EXISTS (DT_NODE_HAS_STATUS(USB, okay) && CONFIG_UART_CONSOLE)
#endif

#if (USB_EXISTS || CONFIG_RTT_CONSOLE) && CONFIG_USE_SLIMENRF_CONSOLE

#if USB_EXISTS
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/console/console.h>
#include <zephyr/logging/log_ctrl.h>
#else
#include "system/rtt_console.h"
#endif
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/pm/device.h>

#include <ctype.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(console, LOG_LEVEL_INF);

static void usb_init_thread(void);
K_THREAD_DEFINE(usb_init_thread_id, 256, usb_init_thread, NULL, NULL, NULL, 6, 0, 500); // Wait before enabling USB

static void console_thread(void);
static struct k_thread console_thread_id;
static K_THREAD_STACK_DEFINE(console_thread_id_stack, 512);

#define DFU_EXISTS CONFIG_BUILD_OUTPUT_UF2 || CONFIG_BOARD_HAS_NRF5_BOOTLOADER
#define ADAFRUIT_BOOTLOADER CONFIG_BUILD_OUTPUT_UF2
#define NRF5_BOOTLOADER CONFIG_BOARD_HAS_NRF5_BOOTLOADER

#if NRF5_BOOTLOADER
static const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(mag), okay)
#define SENSOR_MAG_EXISTS true
#endif

static const char *meows[] = {
	"Mew",
	"Meww",
	"Meow",
	"Meow meow",
	"Mrrrp",
	"Mrrf",
	"Mreow",
	"Mrrrow",
	"Mrrr",
	"Purr",
	"mew",
	"meww",
	"meow",
	"meow meow",
	"mrrrp",
	"mrrf",
	"mreow",
	"mrrrow",
	"mrrr",
	"purr",
};

static const char *meow_punctuations[] = {
	".",
	"?",
	"!",
	"-",
	"~",
	""
};

static const char *meow_suffixes[] = {
	" :3",
	" :3c",
	" ;3",
	" ;3c",
	" x3",
	" x3c",
	" X3",
	" X3c",
	" >:3",
	" >:3c",
	" >;3",
	" >;3c",
	""
};

static void console_thread_create(void)
{
	k_thread_create(&console_thread_id, console_thread_id_stack, K_THREAD_STACK_SIZEOF(console_thread_id_stack), (k_thread_entry_t)console_thread, NULL, NULL, NULL, 6, 0, K_NO_WAIT);
}

#if USB_EXISTS
static void status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	const struct log_backend *backend = log_backend_get_by_name("log_backend_uart");
	const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	switch (status)
	{
	case USB_DC_CONNECTED:
		set_status(SYS_STATUS_USB_CONNECTED, true);
		pm_device_action_run(cons, PM_DEVICE_ACTION_RESUME);
		log_backend_enable(backend, backend->cb->ctx, CONFIG_LOG_MAX_LEVEL);
		console_thread_create();
		break;
	case USB_DC_DISCONNECTED:
		set_status(SYS_STATUS_USB_CONNECTED, false);
		k_thread_abort(&console_thread_id);
		log_backend_disable(backend);
		pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);
		break;
	default:
		LOG_DBG("status %u unhandled", status);
		break;
	}
}
#endif

static void usb_init_thread(void)
{
#if USB_EXISTS
	usb_enable(status_cb);
#else
	console_thread_create();
#endif
}

static void print_info(void)
{
#if USB_EXISTS
	printk(CONFIG_USB_DEVICE_MANUFACTURER " " CONFIG_USB_DEVICE_PRODUCT "\n");
#endif
	printk(FW_STRING);

	printk("\nBoard: " CONFIG_BOARD "\n");
	printk("SOC: " CONFIG_SOC "\n");
	printk("Target: " CONFIG_BOARD_TARGET "\n");

	printk("\nIMU: %s\n", (retained->imu_addr & 0x7F) != 0x7F ? sensor_get_sensor_imu_name() : "Not searching");
	if (retained->imu_reg != 0xFF)
		printk("Interface: %s\n", (retained->imu_reg & 0x80) ? "SPI" : "I2C");
	printk("Address: 0x%02X%02X\n", retained->imu_addr, retained->imu_reg);

#if SENSOR_MAG_EXISTS
	printk("\nMagnetometer: %s\n", (retained->mag_addr & 0x7F) != 0x7F ? sensor_get_sensor_mag_name() : "Not searching");
	if (retained->mag_reg != 0xFF)
		printk("Interface: %s%s\n", (retained->mag_reg & 0x80) ? "SPI" : "I2C", (retained->mag_addr & 0x80) ? ", external" : "");
	printk("Address: 0x%02X%02X\n", retained->mag_addr, retained->mag_reg);
#endif

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	printk("\nAccelerometer matrix:\n");
	for (int i = 0; i < 3; i++)
		printk("%.5f %.5f %.5f %.5f\n", (double)retained->accBAinv[0][i], (double)retained->accBAinv[1][i], (double)retained->accBAinv[2][i], (double)retained->accBAinv[3][i]);
#else
	printk("\nAccelerometer bias: %.5f %.5f %.5f\n", (double)retained->accelBias[0], (double)retained->accelBias[1], (double)retained->accelBias[2]);
#endif
	printk("Gyroscope bias: %.5f %.5f %.5f\n", (double)retained->gyroBias[0], (double)retained->gyroBias[1], (double)retained->gyroBias[2]);
#if SENSOR_MAG_EXISTS
//	printk("Magnetometer bridge offset: %.5f %.5f %.5f\n", (double)retained->magBias[0], (double)retained->magBias[1], (double)retained->magBias[2]);
	printk("Magnetometer matrix:\n");
	for (int i = 0; i < 3; i++)
		printk("%.5f %.5f %.5f %.5f\n", (double)retained->magBAinv[0][i], (double)retained->magBAinv[1][i], (double)retained->magBAinv[2][i], (double)retained->magBAinv[3][i]);
#endif

	printk("\nFusion: %s\n", sensor_get_sensor_fusion_name());

	bool paired = retained->paired_addr[0];
	printk(paired ? "\nTracker ID: %u\n" : "\nTracker ID: None\n", retained->paired_addr[1]);
	printk("Device address: %012llX\n", *(uint64_t *)NRF_FICR->DEVICEADDR & 0xFFFFFFFFFFFF);
	printk(paired ? "Receiver address: %012llX\n" : "Receiver address: None\n", (*(uint64_t *)&retained->paired_addr[0] >> 16) & 0xFFFFFFFFFFFF);

	int battery_mV = sys_get_valid_battery_mV();
	int16_t calibrated_pptt = sys_get_calibrated_battery_pptt(sys_get_valid_battery_pptt());
	uint64_t unplugged_time = sys_get_last_unplugged_time();
	uint64_t remaining = sys_get_battery_remaining_time_estimate();
	uint64_t runtime = sys_get_battery_runtime_estimate();
	if (battery_mV > 0)
	{
		unplugged_time = k_ticks_to_us_floor64(k_uptime_ticks() - unplugged_time);
		uint32_t hours = unplugged_time / 3600000000;
		unplugged_time %= 3600000000;
		uint8_t minutes = unplugged_time / 60000000;
		if (hours > 0 || minutes > 0)
			printk("\nBattery: %.0f%% (Read %uh %umin ago)\n", (double)calibrated_pptt / 100.0, hours, minutes);
		else
			printk("\nBattery: %.0f%%\n", (double)calibrated_pptt / 100.0);
	}
	else if (unplugged_time == 0)
	{
		printk("\nBattery: Waiting for valid reading\n");
	}
	else
	{
		printk("\nBattery: None\n");
	}
	if (remaining > 0)
	{
		remaining = k_ticks_to_us_floor64(remaining);
		uint32_t hours = remaining / 3600000000;
		remaining %= 3600000000;
		uint8_t minutes = remaining / 60000000;
		printk("Remaining runtime: %uh %umin\n", hours, minutes);
	}
	else
	{
		printk("Remaining runtime: Not available\n");
	}
	if (runtime > 0)
	{
		runtime = k_ticks_to_us_floor64(runtime);
		uint32_t hours = runtime / 3600000000;
		runtime %= 3600000000;
		uint8_t minutes = runtime / 60000000;
		printk("Fully charged runtime: %uh %umin\n", hours, minutes);
	}
	else
	{
		printk("Fully charged runtime: Not available\n");
	}
}

static void print_uptime(const uint64_t ticks, const char *name)
{
	uint64_t uptime = k_ticks_to_us_floor64(ticks);

	uint32_t hours = uptime / 3600000000;
	uptime %= 3600000000;
	uint8_t minutes = uptime / 60000000;
	uptime %= 60000000;
	uint8_t seconds = uptime / 1000000;
	uptime %= 1000000;
	uint16_t milliseconds = uptime / 1000;
	uint16_t microseconds = uptime % 1000;

	printk("%s: %02u:%02u:%02u.%03u,%03u\n", name, hours, minutes, seconds, milliseconds, microseconds);
}

static void print_battery(void)
{
	int adc_mV = sys_get_battery_mV();
	printk("ADC: %d mV\n", adc_mV);

	int battery_mV = sys_get_valid_battery_mV();
	int16_t pptt = sys_get_valid_battery_pptt();
	int16_t calibrated_pptt = sys_get_calibrated_battery_pptt(pptt);
	uint64_t unplugged_time = sys_get_last_unplugged_time();
	if (battery_mV > 0)
		printk("\nBattery: %.2f%% (Raw %.2f%%, %d mV)\n", (double)calibrated_pptt / 100.0, (double)pptt / 100.0, battery_mV);
	else
		printk("\nBattery: None\n");
	if (unplugged_time > 0)
		print_uptime(k_uptime_ticks() - unplugged_time, "Last updated");
	else
		printk("Last updated: Never\n");

	uint64_t runtime = sys_get_battery_runtime_estimate();
	uint64_t runtime_min = sys_get_battery_runtime_min_estimate();
	uint64_t runtime_max = sys_get_battery_runtime_max_estimate();
	uint64_t remaining = sys_get_battery_remaining_time_estimate();
	if (remaining > 0)
		print_uptime(remaining, "\nRemaining runtime");
	else
		printk("Remaining runtime: Not available\n");
	if (runtime > 0)
		print_uptime(runtime, "Fully charged runtime");
	else
		printk("Fully charged runtime: Not available\n");
	if (runtime_min > 0)
		print_uptime(runtime_min, "Minimum runtime");
	else
		printk("Minimum runtime: Not available\n");
	if (runtime_max > 0)
		print_uptime(runtime_max, "Maximum runtime");
	else
		printk("Maximum runtime: Not available\n");

	int16_t last_min = sys_get_last_cycle_min_pptt();
	int16_t last_max = sys_get_last_cycle_max_pptt();
	int16_t last_calibrated_min = sys_get_calibrated_battery_pptt(last_min);
	int16_t last_calibrated_max = sys_get_calibrated_battery_pptt(last_max);
	uint64_t last_runtime = sys_get_last_cycle_runtime();
	if (last_min >= 0 && last_max >= 0 && last_runtime > 0)
	{
		printk("\nLast discharge cycle: %.2f%% -> %.2f%% (Raw %.2f%% -> %.2f%%)\n", (double)last_calibrated_max / 100.0, (double)last_calibrated_min / 100.0, (double)last_max / 100.0, (double)last_min / 100.0);
		print_uptime(last_runtime, "Last cycle runtime");
	}
	else
	{
		printk("\nLast cycle: Not available\n");
	}

	float coverage = sys_get_battery_calibration_coverage();
	int16_t min = sys_get_calibrated_battery_range_min_pptt();
	int16_t max = sys_get_calibrated_battery_range_max_pptt();
	if (min >= 0 && max >= 0)
		printk("\nCalibration: %.0f%% - %.0f%% (%.0f%% coverage, max 95%%)\n", (double)min / 100.0, (double)max / 100.0, (double)coverage * 100.0);
	else
		printk("\nCalibration: None\n");
	printk("Cycle count: ~%.2f\n", (double)sys_get_battery_cycles());
}

static void print_meow(void)
{
	int64_t ticks = k_uptime_ticks();

	ticks %= ARRAY_SIZE(meows) * ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes); // silly number generator
	uint8_t meow = ticks / (ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes));
	ticks %= (ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes));
	uint8_t punctuation = ticks / ARRAY_SIZE(meow_suffixes);
	uint8_t suffix = ticks % ARRAY_SIZE(meow_suffixes);

	printk("%s%s%s\n", meows[meow], meow_punctuations[punctuation], meow_suffixes[suffix]);
}

static void console_thread(void)
{
#if DFU_EXISTS
	if (button_read()) // button held on usb connect, enter DFU
	{
#if ADAFRUIT_BOOTLOADER
		NRF_POWER->GPREGRET = 0x57;
		sys_request_system_reboot();
#endif
#if NRF5_BOOTLOADER
		gpio_pin_configure(gpio_dev, 19, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
#endif
	}
#endif

#if USB_EXISTS
	console_getline_init();
	while (log_data_pending())
		k_usleep(1);
	k_msleep(100);
	printk("*** " CONFIG_USB_DEVICE_MANUFACTURER " " CONFIG_USB_DEVICE_PRODUCT " ***\n");
#endif
	printk(FW_STRING);
	printk("info                         Get device information\n");
	printk("uptime                       Get device uptime\n");
	printk("reboot                       Soft reset the device\n");
	printk("battery                      Get battery information\n");
	printk("scan                         Restart sensor scan\n");
	printk("calibrate                    Calibrate sensor ZRO\n");

	uint8_t command_info[] = "info";
	uint8_t command_uptime[] = "uptime";
	uint8_t command_reboot[] = "reboot";
	uint8_t command_battery[] = "battery";
	uint8_t command_scan[] = "scan";
	uint8_t command_calibrate[] = "calibrate";

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	printk("6-side                       Calibrate 6-side accelerometer\n");

	uint8_t command_6_side[] = "6-side";
#endif

#if SENSOR_MAG_EXISTS
	printk("mag                          Clear magnetometer calibration\n");

	uint8_t command_mag[] = "mag";
#endif

	printk("set <address>                Manually set receiver\n");
	printk("pair                         Enter pairing mode\n");
	printk("clear                        Clear pairing data\n");

	uint8_t command_set[] = "set";
	uint8_t command_pair[] = "pair";
	uint8_t command_clear[] = "clear";

#if DFU_EXISTS
	printk("dfu                          Enter DFU bootloader\n");

	uint8_t command_dfu[] = "dfu";
#endif

	printk("meow                         Meow!\n");

	uint8_t command_meow[] = "meow";

	// debug
	uint8_t command_reset[] = "reset";
	uint8_t command_reset_arg_zro[] = "zro";
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	uint8_t command_reset_arg_acc[] = "acc";
#endif
#if SENSOR_MAG_EXISTS
	uint8_t command_reset_arg_mag[] = "mag";
#endif
	uint8_t command_reset_arg_bat[] = "bat";
	uint8_t command_reset_arg_all[] = "all";

	while (1) {
#if USB_EXISTS
		uint8_t *line = console_getline();
#else
		uint8_t *line = rtt_console_getline();
#endif
		uint8_t *arg = NULL;
		for (uint8_t *p = line; *p; ++p)
		{
			*p = tolower(*p);
			if (*p == ' ' && !arg)
			{
				*p = 0;
				p++;
				*p = tolower(*p);
				if (*p)
					arg = p;
			}
		}

		if (memcmp(line, command_info, sizeof(command_info)) == 0)
		{
			print_info();
		}
		else if (memcmp(line, command_uptime, sizeof(command_uptime)) == 0)
		{
			uint64_t uptime = k_uptime_ticks();
			print_uptime(uptime, "Uptime");
			print_uptime(uptime - retained->uptime_latest + retained->uptime_sum, "Accumulated");
		}
		else if (memcmp(line, command_reboot, sizeof(command_reboot)) == 0)
		{
			sys_request_system_reboot();
		}
		else if (memcmp(line, command_battery, sizeof(command_battery)) == 0)
		{
			print_battery();
		}
		else if (memcmp(line, command_scan, sizeof(command_scan)) == 0)
		{
			sensor_request_scan(true);
		}
		else if (memcmp(line, command_calibrate, sizeof(command_calibrate)) == 0)
		{
			sensor_request_calibration();
		}
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
		else if (memcmp(line, command_6_side, sizeof(command_6_side)) == 0)
		{
			sensor_request_calibration_6_side();
		}
#endif
#if SENSOR_MAG_EXISTS
		else if (memcmp(line, command_mag, sizeof(command_mag)) == 0)
		{
			sensor_calibration_clear_mag(NULL, true);
		}
#endif
		else if (memcmp(line, command_set, sizeof(command_set)) == 0) 
		{
			uint64_t addr = strtoull(arg, NULL, 16);
			uint8_t buf[17];
			snprintk(buf, 17, "%016llx", addr);
			if (addr != 0 && memcmp(buf, arg, 17) == 0)
				esb_set_pair(addr);
			else
				printk("Invalid address\n");
		}
		else if (memcmp(line, command_pair, sizeof(command_pair)) == 0) 
		{
			esb_reset_pair();
		}
		else if (memcmp(line, command_clear, sizeof(command_clear)) == 0) 
		{
			esb_clear_pair();
		}
#if DFU_EXISTS
		else if (memcmp(line, command_dfu, sizeof(command_dfu)) == 0)
		{
#if ADAFRUIT_BOOTLOADER
			NRF_POWER->GPREGRET = 0x57;
			sys_request_system_reboot();
#endif
#if NRF5_BOOTLOADER
			gpio_pin_configure(gpio_dev, 19, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
#endif
		}
#endif
		else if (memcmp(line, command_meow, sizeof(command_meow)) == 0) 
		{
			print_meow();
		}
		else if (memcmp(line, command_reset, sizeof(command_reset)) == 0)
		{
			if (arg && memcmp(arg, command_reset_arg_zro, sizeof(command_reset_arg_zro)) == 0)
			{
				sensor_calibration_clear(NULL, NULL, true);
			}
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
			else if (arg && memcmp(arg, command_reset_arg_acc, sizeof(command_reset_arg_acc)) == 0)
			{
				sensor_calibration_clear_6_side(NULL, true);
			}
#endif
#if SENSOR_MAG_EXISTS
			else if (arg && memcmp(arg, command_reset_arg_mag, sizeof(command_reset_arg_mag)) == 0)
			{
				sensor_calibration_clear_mag(NULL, true);
			}
#endif
			else if (arg && memcmp(arg, command_reset_arg_bat, sizeof(command_reset_arg_bat)) == 0)
			{
				sys_reset_battery_tracker();
			}
			else if (arg && memcmp(arg, command_reset_arg_all, sizeof(command_reset_arg_all)) == 0)
			{
				sys_clear();
			}
			else
			{
				printk("Invalid argument\n");
			}
		}
		else
		{
			printk("Unknown command\n");
		}
	}
}

#endif