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
#ifndef SLIMENRF_SENSOR_NONE
#define SLIMENRF_SENSOR_NONE

#include "sensor.h"

int imu_none_init(float clock_rate, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time);
void imu_none_shutdown(void);

int imu_none_update_odr(float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time);

uint16_t imu_none_fifo_read(uint8_t *data, uint16_t len);
int imu_none_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3]);
void imu_none_accel_read(float a[3]);
void imu_none_gyro_read(float g[3]);
int imu_none_temp_read(float *data);

uint8_t imu_none_setup_WOM(void);

int imu_none_ext_setup(enum sensor_ext_mode);

extern const sensor_imu_t sensor_imu_none;

int mag_none_init(float time, float *actual_time);
void mag_none_shutdown(void);

int mag_none_update_odr(float time, float *actual_time);

void mag_none_mag_oneshot(void);
void mag_none_mag_read(float m[3]);
float mag_none_temp_read(float bias[3]);

void mag_none_mag_process(uint8_t *raw_m, float m[3]);

extern const sensor_mag_t sensor_mag_none;

#endif
