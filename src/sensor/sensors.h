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
#ifndef SLIMENRF_SENSOR_SENSORS
#define SLIMENRF_SENSOR_SENSORS

#include <zephyr/kernel.h>

#include "sensor_none.h"

#include "imu/BMI160.h"
#include "imu/BMI270.h"
#include "imu/ICM42688.h"
#include "imu/ICM45686.h"
#include "imu/LSM6DSM.h"
#include "imu/LSM6DSO.h"
#include "imu/LSM6DSV.h"
#include "imu/MPU6050.h"

#include "mag/AK09940.h"
#include "mag/BMM150.h"
#include "mag/BMM350.h"
#include "mag/IST8306.h"
#include "mag/IST8308.h"
#include "mag/LIS2MDL.h"
#include "mag/LIS3MDL.h"
#include "mag/MMC5983MA.h"
#include "mag/QMC5883P.h"
#include "mag/QMC6309.h"

#include "scan.h"
#include "scan_spi.h"
#include "scan_ext.h"
#include "sensor.h"
#include "sensors_enum.h"

const char *dev_imu_names[] = {
	"BMI160",
	"BMI270",
	"BMI323",
	"MPU-6000/MPU-6050",
	"MPU-6500",
	"MPU-9250",
	"ICM-20948",
	"ICM-42688-P/ICM-42688-V",
	"ICM-45686",
	"LSM6DSO16IS/ISM330IS",
	"LSM6DS3",
	"LSM6DS3TR-C/LSM6DSL/LSM6DSM/ISM330DLC",
	"LSM6DSR/ISM330DHCX",
	"LSM6DSO",
	"LSM6DST",
	"LSM6DSV",
	"LSM6DSV16B/ISM330BX"
};
const sensor_imu_t *sensor_imus[] = {
	&sensor_imu_bmi160,
	&sensor_imu_bmi270,
	&sensor_imu_none,
	&sensor_imu_mpu6050, // why did I do this
	&sensor_imu_none, // cardinal sin
	&sensor_imu_none, // cardinal sin
	&sensor_imu_none,
	&sensor_imu_icm42688,
	&sensor_imu_icm45686,
	&sensor_imu_none, // will not implement, does not have FIFO
	&sensor_imu_lsm6dsm, // compatible with driver (unfortunately)
	&sensor_imu_lsm6dsm,
	&sensor_imu_lsm6dso, // compatible with driver
	&sensor_imu_lsm6dso,
	&sensor_imu_none,
	&sensor_imu_lsm6dsv,
	&sensor_imu_lsm6dsv  // compatible with driver
};
const int i2c_dev_imu_addr_count = 2;
const uint8_t i2c_dev_imu_addr[] = {
	2,	0x68,0x69,
	2,	0x6A,0x6B
};
const uint8_t i2c_dev_imu_reg[] = {
	3,	0x00,
		0x72,
		0x75,
	1,	0x0F
};
const uint8_t i2c_dev_imu_id[] = {
	4,	0xEA,0xD1,0x24,0x43, // reg 0x00
	1,	0xE9, // reg 0x72
	5,	0x68,0x70,0x71,0x47,0xDB, // reg 0x75
	8,	0x22,0x69,0x6A,0x6B,0x6C,0x6D,0x70,0x71 // reg 0x0F
};
const int i2c_dev_imu[] = {
	IMU_ICM20948, IMU_BMI160, IMU_BMI270, IMU_BMI323,
	IMU_ICM45686,
	IMU_MPU6050, IMU_MPU6500, IMU_MPU9250, IMU_ICM42688, IMU_ICM42688, // ICM-42688-P, ICM-42688-V
	IMU_ISM330IS, IMU_LSM6DS3, IMU_LSM6DSM, IMU_LSM6DSR, IMU_LSM6DSO, IMU_LSM6DST, IMU_LSM6DSV, IMU_ISM330BX
};

const char *dev_mag_names[] = {
	"HMC5883L",
	"QMC5883L",
	"QMC6309",
	"QMC6310",
	"AK8963",
	"AK09916",
	"AK09940",
	"BMM150",
	"BMM350",
	"IST8306",
	"IST8308",
	"IST8320",
	"IST8321",
	"IIS2MDC/LIS2MDL",
	"QMC5883P",
	"LIS3MDL",
	"MMC34160PJ",
	"MMC3630KJ",
	"MMC5603NJ/MMC5633NJL",
	"MMC5616WA",
	"MMC5983MA"
};
const sensor_mag_t *sensor_mags[] = {
	&sensor_mag_none, // HMC5883 will not implement, too low quality
	&sensor_mag_none, // QMC5883 not implemented
	&sensor_mag_qmc6309,
	&sensor_mag_none, // QMC6310
	&sensor_mag_none, // AK8963
	&sensor_mag_none, // AK09916
	&sensor_mag_ak09940,
	&sensor_mag_bmm150,
	&sensor_mag_bmm350,
	&sensor_mag_ist8306,
	&sensor_mag_ist8308,
	&sensor_mag_none, // IST8320
	&sensor_mag_none, // IST8321
	&sensor_mag_lis2mdl,
	&sensor_mag_qmc5883p, // QMC5883P / HP5883
	&sensor_mag_lis3mdl,
	&sensor_mag_none, // MMC34160
	&sensor_mag_none, // MMC3630
	&sensor_mag_none, // MMC5603/MMC5633
	&sensor_mag_none, // MMC5616
	&sensor_mag_mmc5983ma
};
const int i2c_dev_mag_addr_count = 12;
const uint8_t i2c_dev_mag_addr[] = {
	1,	0x0C,
	1,	0x0D,
	2,	0x0E,0x0F,
	4,	0x10,0x11,0x12,0x13, // why bosch
	4,	0x14,0x15,0x16,0x17,
	1,	0x19,
	1,	0x1C,
	1,	0x1E,
    1,  0x2C,
	1,	0x30,
	1,	0x3C,
	1,	0x7C
};
const uint8_t i2c_dev_mag_reg[] = {
	2,	0x01, // AK09916/AK09940 first
		0x00,
	3,	0x01, // AK09940 first
		0x00,
		0x0D,
	2,	0x01, // AK09940 first
		0x00,
	1,	0x40,
	1,	0x00,
	1,	0x00,
	2,	0x00,
		0x0F,
	3,	0x0A,
		0x0F,
		0x4F,
	1,	0x00,
	3,	0x20,
		0x2F,
		0x39,
	1,	0x00,
	1,	0x00
};
const uint8_t i2c_dev_mag_id[] = {
	2,	0x09,0xA3, // reg 0x01
	2,	0x08,0x48, // reg 0x00
	1,	0xA3, // reg 0x01
	2,	0x08,0x48, // reg 0x00
	1,	0xFF, // reg 0x0D
	1,	0xA3, // reg 0x01
	2,	0x08,0x48, // reg 0x00
	1,	0x32, // reg 0x40
	1,	0x33, // reg 0x00
	3,	0x06,0x20,0x21, // reg 0x00
	1,	0x80, // reg 0x00
	1,	0x3D, // reg 0x0F
	1,	0x48, // reg 0x0A
	1,	0x3D, // reg 0x0F
	1,	0x40, // reg 0x4F
	1,	0x80, // reg 0x00
	1,	0x06, // reg 0x20
	2,	0x0A,0x30, // reg 0x2F
	2,	0x10,0x11, // reg 0x39
	1,	0x80, // reg 0x00
	1,	0x90 // reg 0x00
};
const int i2c_dev_mag[] = {
	MAG_AK09916, MAG_AK09940,
	MAG_IST8308, MAG_AK8963,
	MAG_AK09940,
	MAG_IST8308, MAG_AK8963,
	MAG_QMC5883L,
	MAG_AK09940,
	MAG_IST8308, MAG_AK8963,
	MAG_BMM150,
	MAG_BMM350,
	MAG_IST8306, MAG_IST8320, MAG_IST8321,
	MAG_QMC5883P,
	MAG_QMC6310,
	MAG_LIS3MDL,
	MAG_HMC5883L,
	MAG_LIS3MDL,
	MAG_LIS2MDL,
	MAG_MMC34160PJ,
	MAG_MMC3630KJ, MAG_MMC5983MA,
	MAG_MMC5633NJL, MAG_MMC5616WA,
	MAG_QMC6310,
	MAG_QMC6309
};

int sensor_scan_imu(struct i2c_dt_spec *i2c_dev, uint8_t *i2c_dev_reg)
{
	return sensor_scan_i2c(i2c_dev, i2c_dev_reg, i2c_dev_imu_addr_count, i2c_dev_imu_addr, i2c_dev_imu_reg, i2c_dev_imu_id, i2c_dev_imu);
}

int sensor_scan_mag(struct i2c_dt_spec *i2c_dev, uint8_t *i2c_dev_reg)
{
	return sensor_scan_i2c(i2c_dev, i2c_dev_reg, i2c_dev_mag_addr_count, i2c_dev_mag_addr, i2c_dev_mag_reg, i2c_dev_mag_id, i2c_dev_mag);
}

int sensor_scan_imu_spi(struct spi_dt_spec *bus, uint8_t *spi_dev_reg)
{
	return sensor_scan_spi(bus, spi_dev_reg, i2c_dev_imu_addr_count, i2c_dev_imu_reg, i2c_dev_imu_id, i2c_dev_imu);
}

int sensor_scan_mag_spi(struct spi_dt_spec *bus, uint8_t *spi_dev_reg)
{
	return sensor_scan_spi(bus, spi_dev_reg, i2c_dev_mag_addr_count, i2c_dev_mag_reg, i2c_dev_mag_id, i2c_dev_mag);
}

int sensor_scan_mag_ext(const sensor_ext_ssi_t *ext_ssi, uint16_t *ext_dev_addr, uint8_t *ext_dev_reg)
{
	return sensor_scan_ext(ext_ssi, ext_dev_addr, ext_dev_reg, i2c_dev_mag_addr_count, i2c_dev_mag_addr, i2c_dev_mag_reg, i2c_dev_mag_id, i2c_dev_mag);
}

#endif