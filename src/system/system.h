#ifndef SLIMENRF_SYSTEM
#define SLIMENRF_SYSTEM

#include "led.h"
#include "power.h"
#include "status.h"

#define RBT_CNT_ID 1
#define PAIRED_ID 2
#define MAIN_ACCEL_BIAS_ID 3
#define MAIN_GYRO_BIAS_ID 4
#define MAIN_MAG_BIAS_ID 5
#define MAIN_ACC_6_BIAS_ID 7

#define BATT_STATS_LAST_RUN_ID 8
#define BATT_STATS_INTERVAL_0 9 // ID 9 to 28
#define BATT_STATS_CURVE_ID 29

#define SETTINGS_ID 30

#define MAIN_SENSOR_DATA_ID 31

void configure_sense_pins(void);

uint8_t reboot_counter_read(void);
void reboot_counter_write(uint8_t reboot_counter);

void sys_write(uint16_t id, void *ptr, const void *data, size_t len);
void sys_read(uint16_t id, void *data, size_t len);
void sys_clear(void);
void sys_nvs_stats(void);

int set_sensor_clock(bool enable, float rate, float *actual_rate);

bool button_read(void);

bool dock_read(void);
bool chg_read(void);
bool stby_read(void);

int sys_user_shutdown(void);
void sys_reset_mode(uint8_t mode);

#endif