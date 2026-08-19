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
#include <zephyr/logging/log.h>

#include "sensor_none.h"

LOG_MODULE_REGISTER(sensor_none, LOG_LEVEL_INF);

int imu_none_init(float clock_rate, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	LOG_DBG("imu_none_init, sensor has no IMU or IMU cannot be initialized");
	return -1;
}

void imu_none_shutdown(void)
{
	LOG_DBG("imu_none_shutdown, sensor has no IMU or IMU cannot be shutdown");
	return;
}

void imu_none_update_fs(float accel_range, float gyro_range, float *accel_actual_range, float *gyro_actual_range)
{
	LOG_DBG("imu_none_update_fs, sensor has no IMU or IMU has no configurable FS");
	return;
}

int imu_none_update_odr(float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	LOG_DBG("imu_none_update_odr, sensor has no IMU or IMU has no configurable ODR");
	return -1;
}

uint16_t imu_none_data_read(uint8_t *data, uint16_t len)
{
	LOG_DBG("imu_none_data_read, sensor has no IMU or IMU has no FIFO");
	return 0;
}

int imu_none_data_process(uint16_t index, uint8_t *data, float a[3], float g[3], float m[3])
{
	LOG_DBG("imu_none_data_process, sensor has no IMU or IMU has no FIFO");
	return -1;
}

void imu_none_accel_read(float a[3])
{
	LOG_DBG("imu_none_accel_read, sensor has no IMU or IMU has no direct data register");
	return;
}

void imu_none_gyro_read(float g[3])
{
	LOG_DBG("imu_none_gyro_read, sensor has no IMU or IMU has no direct data register");
	return;
}

int imu_none_temp_read(float *data)
{
	LOG_DBG("imu_none_temp_read, sensor has no IMU or IMU has no temperature register");
	return -1;
}

uint8_t imu_none_setup_DRDY(uint16_t threshold)
{
	LOG_DBG("imu_none_setup_DRDY, sensor has no IMU or IMU has no FIFO threshold/watermark support");
	return 0;
}

uint8_t imu_none_setup_WOM(void)
{
	LOG_DBG("imu_none_setup_WOM, sensor has no IMU or IMU has no wake up support");
	return 0;
}

int imu_none_ext_setup(enum sensor_ext_mode, const sensor_mag_t *mag, uint8_t mag_addr)
{
	LOG_DBG("imu_none_ext_setup, sensor has no IMU or IMU has no ext support");
	return -1;
}

const sensor_imu_t sensor_imu_none = {
	*imu_none_init,
	*imu_none_shutdown,

	*imu_none_update_fs,
	*imu_none_update_odr,

	*imu_none_data_read,
	*imu_none_data_process,
	*imu_none_accel_read,
	*imu_none_gyro_read,
	*imu_none_temp_read,

	*imu_none_setup_DRDY,
	*imu_none_setup_WOM,
	
	*imu_none_ext_setup
};

int mag_none_init(float time, float *actual_time)
{
	LOG_DBG("mag_none_init, sensor has no magnetometer or magnetometer cannot be initialized");
	return -1;
}

void mag_none_shutdown(void)
{
	LOG_DBG("mag_none_shutdown, sensor has no magnetometer or magnetometer cannot be shutdown");
	return;
}

int mag_none_update_odr(float time, float *actual_time)
{
	LOG_DBG("mag_none_update_odr, sensor has no magnetometer or magnetometer has no configurable ODR");
	return -1;
}

float mag_none_get_odr(void)
{
	return 0.0f;
}

void mag_none_mag_oneshot(void)
{
	LOG_DBG("mag_none_mag_oneshot, sensor has no magnetometer or magnetometer has no oneshot mode");
	return;
}

void mag_none_mag_read(float m[3])
{
	LOG_DBG("mag_none_mag_read, sensor has no magnetometer or magnetometer has no direct data register");
	return;
}

float mag_none_temp_read(float bias[3])
{
	LOG_DBG("mag_none_temp_read, sensor has no magnetometer or magnetometer has no temperature register");
	return 0;
}

void mag_none_mag_process(uint8_t *raw_m, float m[3])
{
	LOG_DBG("mag_none_mag_process, sensor has no magnetometer");
	return;
}

const sensor_mag_t sensor_mag_none = {
	*mag_none_init,
	*mag_none_shutdown,

	*mag_none_update_odr,
	*mag_none_get_odr,

	*mag_none_mag_oneshot,
	*mag_none_mag_read,
	*mag_none_temp_read,

	*mag_none_mag_process,
	UINT8_MAX, UINT8_MAX
};
