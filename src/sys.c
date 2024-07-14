#include "globals.h"
#include "sensor.h"

#include "sys.h"

struct nvs_fs fs;

#define NVS_PARTITION		storage_partition
#define NVS_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(NVS_PARTITION)

LOG_MODULE_REGISTER(sys, 4);

K_THREAD_DEFINE(led_thread_id, 512, led_thread, NULL, NULL, NULL, 6, 0, 0);

void (*extern_main_imu_suspend)(void);

void configure_system_off_WOM()
{
	(*extern_main_imu_suspend)();
	// Configure WOM interrupt
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, int0_gpios), NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, int0_gpios), NRF_GPIO_PIN_SENSE_LOW);
	sensor_retained_write_quat();
	sensor_retained_write_gOff();
	LOG_INF("System off (WOM)");
	// Set system off
	sensor_setup_WOM(); // enable WOM feature
	sys_poweroff();
}

void configure_system_off_chgstat(void){
	(*extern_main_imu_suspend)();
//	// Configure chgstat interrupt
//	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_NOPULL);
//	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, chgstat_gpios), NRF_GPIO_PIN_PULLUP);
//	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, chgstat_gpios), NRF_GPIO_PIN_SENSE_LOW);
	// Configure dock interrupt
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_PULLUP); // Still works
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_SENSE_LOW);
	sensor_retained_write_gOff();
	LOG_INF("System off (chgstat)");
	// Set system off
	sys_poweroff();
}

void configure_system_off_dock(void){
	(*extern_main_imu_suspend)();
	// Configure dock interrupt
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_NOPULL); // Still works
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_SENSE_HIGH);
	sensor_retained_write_gOff();
	LOG_INF("System off (dock)");
	// Set system off
	sys_poweroff();
}

void power_check(void) {
	bool docked = gpio_pin_get_dt(&dock);
	int batt_mV;
	uint32_t batt_pptt = read_batt_mV(&batt_mV);
	LOG_INF("Battery %u%% (%dmV)", batt_pptt/100, batt_mV);
	if (batt_pptt == 0 && !docked) {
		(*extern_main_imu_suspend)();
		sensor_shutdown();
		set_led(SYS_LED_PATTERN_OFF); // Turn off LED
		configure_system_off_chgstat();
	} else if (docked) {
		(*extern_main_imu_suspend)();
		sensor_shutdown();
		set_led(SYS_LED_PATTERN_OFF); // Turn off LED
		configure_system_off_dock(); // usually charging, i would flash LED but that will drain the battery while it is charging..
	}
}

#if DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios) // Alternate button if available to use as "reset key"
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	sys_reboot(SYS_REBOOT_COLD); // treat like pin reset but without pin reset reason
}
#endif

enum sys_led_pattern current_led_pattern;
int led_pattern_state;

const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});
const struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(ZEPHYR_USER_NODE, led_gpios, led0);
const struct pwm_dt_spec pwm_led = PWM_DT_SPEC_GET_OR(LED0_NODE, {0});

void set_led(enum sys_led_pattern led_pattern)
{
	if (led_pattern == SYS_LED_PATTERN_OFF)
	{
		k_thread_suspend(led_thread_id);
		gpio_pin_set_dt(&led, 0);
	}
	else
	{
		current_led_pattern = led_pattern;
		led_pattern_state = 0;
		k_thread_resume(led_thread_id);
	}
}

void led_thread(void)
{
	gpio_pin_configure_dt(&led, GPIO_OUTPUT);
	while (1)
	{
		switch (current_led_pattern)
		{
			case SYS_LED_PATTERN_ON:
				gpio_pin_set_dt(&led, 1);
				k_thread_suspend(led_thread_id);
				break;
			case SYS_LED_PATTERN_SHORT:
				led_pattern_state = (led_pattern_state + 1) % 2;
				gpio_pin_set_dt(&led, led_pattern_state);
				k_msleep(led_pattern_state == 1 ? 100 : 900);
				break;
			case SYS_LED_PATTERN_LONG:
				led_pattern_state = (led_pattern_state + 1) % 2;
				gpio_pin_set_dt(&led, led_pattern_state);
				k_msleep(500);
				break;
			case SYS_LED_PATTERN_ACTIVE:
				led_pattern_state = (led_pattern_state + 1) % 2;
				gpio_pin_set_dt(&led, led_pattern_state);
				k_msleep(led_pattern_state == 1 ? 300 : 9700);
				break;
			case SYS_LED_PATTERN_ONESHOT_POWERON:
				led_pattern_state++;
				gpio_pin_set_dt(&led, led_pattern_state % 2);
				if (led_pattern_state == 6)
					k_thread_suspend(led_thread_id);
				else
					k_msleep(200);
				break;
			case SYS_LED_PATTERN_ONESHOT_POWEROFF:
				if (led_pattern_state++ > 0 && led_pattern_state < 22)
					pwm_set_pulse_dt(&pwm_led, PWM_MSEC(22 - led_pattern_state));
				else
					gpio_pin_set_dt(&led, 0);
				if (led_pattern_state == 22)
					k_thread_suspend(led_thread_id);
				else if (led_pattern_state == 1)
					k_msleep(250);
				else
					k_msleep(50);
				break;
			default:
				k_thread_suspend(led_thread_id);
		}
	}
}

bool ram_validated;
bool ram_retention;
bool nvs_init;

inline void sys_retained_init(void) {
	if (!ram_validated) {
		ram_retention = retained_validate(); // Check ram retention
		ram_validated = true;
	}
}

inline void sys_nvs_init(void) {
	if (!nvs_init) {
		struct flash_pages_info info;
		fs.flash_device = NVS_PARTITION_DEVICE;
		fs.offset = NVS_PARTITION_OFFSET; // Start NVS FS here
		flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
		fs.sector_size = info.size; // Sector size equal to page size
		fs.sector_count = 4U; // 4 sectors
		nvs_mount(&fs);
		nvs_init = true;
	}
}

// read from retained
uint8_t reboot_counter_read(void) {
	sys_retained_init();
	return retained.reboot_counter;
}

// write to retained
void reboot_counter_write(uint8_t reboot_counter) {
	sys_retained_init();
	retained.reboot_counter = reboot_counter;
	retained_update();
}

// read from nvs to retained
void sys_read(void) {
	sys_retained_init();
	// All contents of NVS was stored in RAM to not need initializing NVS often
	if (!ram_retention) { 
		LOG_INF("Invalidated RAM");
		sys_nvs_init();
		nvs_read(&fs, PAIRED_ID, &retained.paired_addr, sizeof(retained.paired_addr));
		nvs_read(&fs, MAIN_ACCEL_BIAS_ID, &retained.accelBias, sizeof(retained.accelBias));
		nvs_read(&fs, MAIN_GYRO_BIAS_ID, &retained.gyroBias, sizeof(retained.gyroBias));
		nvs_read(&fs, MAIN_MAG_BIAS_ID, &retained.magBAinv, sizeof(retained.magBAinv));
		retained_update();
		ram_retention = true;
	} else {
		LOG_INF("Recovered calibration from RAM");
	}
}

// write to retained and nvs
void sys_write(uint16_t id, void *retained_ptr, const void *data, size_t len) {
	sys_retained_init();
	sys_nvs_init();
	memcpy(retained_ptr, data, len);
	nvs_write(&fs, id, data, len);
	retained_update();
}