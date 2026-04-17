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
#ifndef SLIMENRF_SENSOR_INTERFACE
#define SLIMENRF_SENSOR_INTERFACE

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>

enum sensor_interface_dev
{
	SENSOR_INTERFACE_DEV_IMU,
	SENSOR_INTERFACE_DEV_MAG
};
#define SENSOR_INTERFACE_DEV_COUNT 2

enum sensor_interface_spec
{
	SENSOR_INTERFACE_SPEC_SPI,
	SENSOR_INTERFACE_SPEC_I2C,
	SENSOR_INTERFACE_SPEC_EXT
};

typedef struct sensor_ext_ssi {
	int (*ext_write)(const uint8_t addr, const uint8_t *buf, uint32_t num_bytes);
	int (*ext_write_read)(const uint8_t addr, const void *write_buf, size_t num_write, void *read_buf, size_t num_read);
	uint8_t ext_burst;
} sensor_ext_ssi_t;

void sensor_interface_register_sensor_imu_spi(struct spi_dt_spec *dev);
void sensor_interface_register_sensor_imu_i2c(struct i2c_dt_spec *dev);

void sensor_interface_register_sensor_mag_spi(struct spi_dt_spec *dev);
void sensor_interface_register_sensor_mag_i2c(struct i2c_dt_spec *dev);
int sensor_interface_register_sensor_mag_ext(uint8_t addr, uint8_t min_burst, uint8_t burst);

int sensor_interface_spi_configure(enum sensor_interface_dev dev, uint32_t frequency, uint32_t dummy_reads);
void sensor_interface_ext_configure(const sensor_ext_ssi_t *ext);
const sensor_ext_ssi_t *sensor_interface_ext_get(void);

int ssi_write(enum sensor_interface_dev dev, const uint8_t *buf, uint32_t num_bytes);
int ssi_read(enum sensor_interface_dev dev, uint8_t *buf, uint32_t num_bytes);
int ssi_write_read(enum sensor_interface_dev dev, const void *write_buf, size_t num_write, void *read_buf, size_t num_read);

int ssi_burst_read(enum sensor_interface_dev dev, uint8_t start_addr, uint8_t *buf, uint32_t num_bytes);
int ssi_burst_write(enum sensor_interface_dev dev, uint8_t start_addr, const uint8_t *buf, uint32_t num_bytes);
int ssi_reg_read_byte(enum sensor_interface_dev dev, uint8_t reg_addr, uint8_t *value);
int ssi_reg_write_byte(enum sensor_interface_dev dev, uint8_t reg_addr, uint8_t value);
int ssi_reg_update_byte(enum sensor_interface_dev dev, uint8_t reg_addr, uint8_t mask, uint8_t value);

int ssi_reg_read_interval(enum sensor_interface_dev dev, uint8_t start_addr, uint8_t *buf, uint32_t num_bytes, uint32_t interval);
int ssi_burst_read_interval(enum sensor_interface_dev dev, uint8_t start_addr, uint8_t *buf, uint32_t num_bytes, uint32_t interval);

#endif