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
#ifndef SLIMENRF_SENSOR
#define SLIMENRF_SENSOR

#include "interface.h"

const char *sensor_get_sensor_imu_name(void);
const char *sensor_get_sensor_mag_name(void);
const char *sensor_get_sensor_fusion_name(void);

int sensor_get_sensor_temperature(float *);

int sensor_request_scan(bool force);

void sensor_scan_read(void);
void sensor_scan_write(void);
void sensor_scan_clear(void);

void sensor_retained_read(void);
void sensor_retained_write(void);

void sensor_shutdown(void);
uint8_t sensor_setup_WOM(void);

void sensor_fusion_invalidate(void);

void wait_for_threads(void);
void main_imu_suspend(void);
void main_imu_resume(void);
void main_imu_wakeup(void);
void main_imu_restart(void);

typedef enum {
	SENSOR_EXT_MODE_OFF = 0, // no external sensor
	SENSOR_EXT_MODE_I2C_PASSTHROUGH = 1, // shorting external i2c bus to main i2c bus, mcu controls everything
	SENSOR_EXT_MODE_I2CM_PROXY = 2, // separate buses, imu is as as a proxy / protocol to i2c converter
	SENSOR_EXT_MODE_I2CM_AUTONOMOUS = 4, // separate buses, mag is autonomically driven by imu, data lands into fifo
} sensor_ext_mode_t;

typedef enum {
	DATA_VALID_ACCEL = 1,
	DATA_VALID_GYRO = 2,
	DATA_VALID_MAG = 4,
	DATA_OUTSIDE_FIFO = 8,
	DATA_INVALID = 16,
} sensor_data_attrs_t;

typedef struct sensor_fusion {
	void (*init)(float, float, float); // gyro_time, accel_time, mag_time
	void (*load)(const void *);
	void (*save)(void *);

	void (*update_gyro)(float *, float); // deg/s
	void (*update_accel)(float *, float); // g
	void (*update_mag)(float *, float); // any unit (usually gauss)
	void (*update)(float *, float *, float *, float);

	void (*get_gyro_bias)(float *);
	void (*set_gyro_bias)(float *);

	void (*update_gyro_sanity)(float *, float *);
	int (*get_gyro_sanity)(void);

	void (*get_lin_a)(float *);
	void (*get_quat)(float *);
} sensor_fusion_t;

typedef struct sensor_mag {
	int (*init)(float, float*); // return update time, return 0 if success, 1 if general error
	void (*shutdown)(void);

	int (*update_odr)(float, float*); // change update time, return real one, return 0 if success, 1 if odr is same, -1 if general error
	float (*get_odr)(void); // return real update time

	void (*mag_oneshot)(void); // trigger oneshot if exists
	void (*mag_read)(float[3]); // any unit (usually gauss)
	float (*temp_read)(float[3]); // deg C

	void (*mag_process)(uint8_t*, float[3]); // use if magnetometer is present as an auxiliary sensor, from data read by IMU
	uint8_t ext_min_burst; // minimum supported burst length for external interface
	uint8_t ext_burst; // default supported burst length
	uint8_t ext_dummy_bytes; // required dummy bytes to skip during every read
	uint8_t ext_burst_reg; // first register address to read in burst
} sensor_mag_t;

typedef struct sensor_imu {
	int (*init)(float, float, float, float*, float*); // first float is clock_rate, nonzero means use CLKIN, return update time, return 0 if success, -1 if general error
	void (*shutdown)(void);

	void (*update_fs)(float, float, float*, float*); // return actual range
	int (*update_odr)(float, float, float*, float*); // return actual update time, return 0 if success, 1 if odr is same, -1 if general error

	uint16_t (*data_read)(uint8_t*, uint16_t);
	sensor_data_attrs_t (*data_process)(uint16_t, uint8_t*, float[3], float[3], float[3]); // g, deg/s
	void (*accel_read)(float[3]); // g
	void (*gyro_read)(float[3]); // deg/s
	int (*temp_read)(float*); // deg C, return 0 if success, -1 if error

	uint8_t (*setup_DRDY)(uint16_t);
	uint8_t (*setup_WOM)(void);

	int (*ext_setup)(sensor_ext_mode_t, const sensor_mag_t *mag, uint8_t mag_addr); // mag used for autonomous mode
} sensor_imu_t;

#endif