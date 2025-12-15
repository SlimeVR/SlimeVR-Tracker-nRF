#ifndef SLIMENRF_CONFIG
#define SLIMENRF_CONFIG

#include "retained.h"

#define CONFIG_SETTINGS_COUNT 4

// map of settings area
struct config_settings_data {
	uint16_t config_0_ovrd; // Override enable flags
	uint16_t config_0_settings;
	uint16_t config_1_ovrd;
	uint16_t config_1_settings;
	uint16_t config_2_ovrd;
	int16_t config_2_settings[16];
	uint16_t config_3_ovrd;
	int32_t config_3_settings[16];
};

// because settings read is a macro, need to expose settings struct and defaults
extern struct config_settings_data *config_settings;

// device settings
enum config_0_settings_id {
	CONFIG_0_USER_EXTRA_ACTIONS,
	CONFIG_0_IGNORE_RESET,
	CONFIG_0_USER_SHUTDOWN,
	CONFIG_0_USE_IMU_WAKE_UP,
	CONFIG_0_DELAY_SLEEP_ON_STATUS,
	CONFIG_0_CONNECTION_OVER_HID,
	CONFIG_0_SETTINGS_END
};

// sensor settings
enum config_1_settings_id {
	CONFIG_1_SENSOR_USE_LOW_POWER_2,
	CONFIG_1_USE_IMU_TIMEOUT,
	CONFIG_1_USE_ACTIVE_TIMEOUT,
	CONFIG_1_SENSOR_USE_MAG,
	CONFIG_1_USE_SENSOR_CLOCK,
	CONFIG_1_SENSOR_USE_6_SIDE_CALIBRATION,
	CONFIG_1_SETTINGS_END
};

enum config_2_settings_id {
	CONFIG_2_LED_DEFAULT_COLOR_R,
	CONFIG_2_LED_DEFAULT_COLOR_G,
	CONFIG_2_LED_DEFAULT_COLOR_B,
	CONFIG_2_ACTIVE_TIMEOUT_MODE, // 0: Sleep, 1: Shutdown
	CONFIG_2_SENSOR_ACCEL_ODR,
	CONFIG_2_SENSOR_GYRO_ODR,
	CONFIG_2_SENSOR_ACCEL_FS,
	CONFIG_2_SENSOR_GYRO_FS,
	CONFIG_2_SENSOR_FUSION, // 1: XIO, 3: VQF
	CONFIG_2_RADIO_TX_POWER,
	CONFIG_2_SETTINGS_END
};

enum config_3_settings_id {
	CONFIG_3_CONNECTION_TIMEOUT_DELAY,
	CONFIG_3_SENSOR_LP_TIMEOUT,
	CONFIG_3_IMU_TIMEOUT_RAMP_MIN,
	CONFIG_3_IMU_TIMEOUT_RAMP_MAX,
	CONFIG_3_ACTIVE_TIMEOUT_THRESHOLD,
	CONFIG_3_ACTIVE_TIMEOUT_DELAY,
	CONFIG_3_BATTERY_LOW_RUNTIME_THRESHOLD,
	CONFIG_3_SETTINGS_END
};

extern const uint16_t config_settings_count[];

extern const char *config_settings_names[];

// because settings read is a macro, need to expose settings struct and defaults
extern const bool config_0_settings_defaults[16];
extern const bool config_1_settings_defaults[16];
//extern const int16_t config_2_settings_defaults[16];
//extern const int32_t config_3_settings_defaults[16];

#define CONFIG_0_SETTINGS_READ(id) (config_settings->config_0_settings & (1 << id))
#define CONFIG_1_SETTINGS_READ(id) (config_settings->config_1_settings & (1 << id))
#define CONFIG_2_SETTINGS_READ(id) (config_settings->config_2_settings[id])
#define CONFIG_3_SETTINGS_READ(id) (config_settings->config_3_settings[id])

void config_settings_init(void);

//bool config_0_settings_read(uint16_t id);
//bool config_1_settings_read(uint16_t id);
//int16_t config_2_settings_read(uint16_t id);
//int32_t config_3_settings_read(uint16_t id);

void config_0_settings_write(uint16_t id, bool value);
void config_1_settings_write(uint16_t id, bool value);
void config_2_settings_write(uint16_t id, int16_t value);
void config_3_settings_write(uint16_t id, int32_t value);

void config_settings_reset(uint16_t config, uint16_t id);
void config_settings_reset_all(void);

#endif
