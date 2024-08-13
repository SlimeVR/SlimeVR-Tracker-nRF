/* 01/14/2022 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Ladybug default), respectively, and it uses the Ladybug STM32L432 Breakout Board.
  The ICM42688 is a combo sensor with embedded accel and gyro, here used as 6 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/
#include <math.h>

#include <zephyr/drivers/i2c.h>

#include "ICM42688.h"

float _aRes, _gRes;

uint8_t icm_getChipID(struct i2c_dt_spec dev_i2c)
{
	uint8_t temp;
	i2c_reg_read_byte_dt(&dev_i2c, ICM42688_WHO_AM_I, &temp);
	return temp;
}

float icm_getAres(uint8_t Ascale)
{
	switch (Ascale)
	{ // Possible accelerometer scales (and their register bit settings) are:
	case AFS_2G:
		_aRes = 2.0f/32768.0f;
		return _aRes;
		break;
	case AFS_4G:
		_aRes = 4.0f/32768.0f;
		return _aRes;
		break;
	case AFS_8G:
		_aRes = 8.0f/32768.0f;
		return _aRes;
		break;
	case AFS_16G:
		_aRes = 16.0f/32768.0f;
		return _aRes;
		break;
	default: // invalid..
		return _aRes;
	}
}

float icm_getGres(uint8_t Gscale)
{
	switch (Gscale)
	{ // Possible gyro scales (and their register bit settings) are:
	case GFS_15_625DPS:
		_gRes = 15.625f/32768.0f;
		return _gRes;
		break;
	case GFS_31_25DPS:
		_gRes = 31.25f/32768.0f;
		return _gRes;
		break;
	case GFS_62_50DPS:
		_gRes = 62.5f/32768.0f;
		return _gRes;
		break;
	case GFS_125DPS:
		_gRes = 125.0f/32768.0f;
		return _gRes;
		break;
	case GFS_250DPS:
		_gRes = 250.0f/32768.0f;
		return _gRes;
		break;
	case GFS_500DPS:
		_gRes = 500.0f/32768.0f;
		return _gRes;
		break;
	case GFS_1000DPS:
		_gRes = 1000.0f/32768.0f;
		return _gRes;
		break;
	case GFS_2000DPS:
		_gRes = 2000.0f/32768.0f;
		return _gRes;
		break;
	default: // invalid..
		return _gRes;
	}
}

void icm_reset(struct i2c_dt_spec dev_i2c)
{
	// reset device
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_DEVICE_CONFIG, 0x01); // Set bit 0 to 1 to reset ICM42688
	k_msleep(2); // Wait 1 ms for all registers to reset
}

void icm_setup_WOM(struct i2c_dt_spec dev_i2c)
{
	uint8_t temp;
	i2c_reg_read_byte_dt(&dev_i2c, ICM42688_INT_STATUS, &temp); // clear reset done int flag
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_INT_SOURCE0, 0x00); // temporary disable interrupts
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_ACCEL_CONFIG0, AFS_8G << 5 | AODR_200Hz); // set accel ODR and FS
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_PWR_MGMT0, aMode_LP); // set accel and gyro modes
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_INTF_CONFIG1, 0x00); // set low power clock
	k_busy_wait(1000);
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_REG_BANK_SEL, 0x04); // select register bank 4
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_ACCEL_WOM_X_THR, 0x08); // set wake thresholds // 80 x 3.9 mg is ~312 mg
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_ACCEL_WOM_Y_THR, 0x08); // set wake thresholds
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_ACCEL_WOM_Z_THR, 0x08); // set wake thresholds
	k_busy_wait(1000);
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_INT_SOURCE1, 0x07); // enable WOM interrupt
	k_busy_wait(50000);
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_SMD_CONFIG, 0x01); // enable WOM feature
}

// make i2c stuff external? (portability)
// make busy wait and msleep external? (portability)
void icm_init(struct i2c_dt_spec dev_i2c, uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR, uint8_t aMode, uint8_t gMode, bool CLKIN)
{
	icm_getAres(Ascale);
	icm_getGres(Gscale);
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_INT_SOURCE0, 0x00); // temporary disable interrupts
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_PWR_MGMT0, gMode << 2 | aMode); // set accel and gyro modes
	k_busy_wait(250); // wait >200us (datasheet 14.36)
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_ACCEL_CONFIG0, Ascale << 5 | AODR); // set accel ODR and FS
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_GYRO_CONFIG0, Gscale << 5 | GODR); // set gyro ODR and FS
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_GYRO_ACCEL_CONFIG0, 0x44); // set gyro and accel bandwidth to ODR/10
//	k_msleep(50); // 10ms Accel, 30ms Gyro startup
	k_msleep(1); // fuck i dont wanna wait that long
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_FIFO_CONFIG, 0x00); // FIFO bypass mode
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_FSYNC_CONFIG, 0x00); // disable FSYNC
	i2c_reg_update_byte_dt(&dev_i2c, ICM42688_TMST_CONFIG, 0x02, 0x00); // disable FSYNC
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_FIFO_CONFIG1, 0x02); // enable FIFO gyro only
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_FIFO_CONFIG, 1<<6); // begin FIFO stream
}

void icm_accel_read(struct i2c_dt_spec dev_i2c, float a[3])
{
	uint8_t rawAccel[6];
	i2c_burst_read_dt(&dev_i2c, ICM42688_ACCEL_DATA_X1, &rawAccel[0], 6);
	float raw0 = (int16_t)((((int16_t)rawAccel[0]) << 8) | rawAccel[1]);
	float raw1 = (int16_t)((((int16_t)rawAccel[2]) << 8) | rawAccel[3]);
	float raw2 = (int16_t)((((int16_t)rawAccel[4]) << 8) | rawAccel[5]);
	a[0] = raw0 * _aRes;
	a[1] = raw1 * _aRes;
	a[2] = raw2 * _aRes;
}

void icm_gyro_read(struct i2c_dt_spec dev_i2c, float g[3])
{
	uint8_t rawGyro[6];
	i2c_burst_read_dt(&dev_i2c, ICM42688_GYRO_DATA_X1, &rawGyro[0], 6);
	float raw0 = (int16_t)((((int16_t)rawGyro[0]) << 8) | rawGyro[1]);
	float raw1 = (int16_t)((((int16_t)rawGyro[2]) << 8) | rawGyro[3]);
	float raw2 = (int16_t)((((int16_t)rawGyro[4]) << 8) | rawGyro[5]);
	g[0] = raw0 * _gRes;
	g[1] = raw1 * _gRes;
	g[2] = raw2 * _gRes;
}

// need to make this external
void icm_offsetBias(struct i2c_dt_spec dev_i2c, float * dest1, float * dest2)
{
	float rawData[3];
	for (int ii = 0; ii < 500; ii++)
	{
		icm_accel_read(dev_i2c, &rawData[0]);
		dest1[0] += rawData[0];
		dest1[1] += rawData[1];
		dest1[2] += rawData[2];
		icm_gyro_read(dev_i2c, &rawData[0]);
		dest2[0] += rawData[0];
		dest2[1] += rawData[1];
		dest2[2] += rawData[2];
		k_msleep(5);
	}

	dest1[0] /= 500.0f;
	dest1[1] /= 500.0f;
	dest1[2] /= 500.0f;
	dest2[0] /= 500.0f;
	dest2[1] /= 500.0f;
	dest2[2] /= 500.0f;
// need better accel calibration
	if(dest1[0] > 0.8f) {dest1[0] -= 1.0f;} // Remove gravity from the x-axis accelerometer bias calculation
	if(dest1[0] < -0.8f) {dest1[0] += 1.0f;} // Remove gravity from the x-axis accelerometer bias calculation
	if(dest1[1] > 0.8f) {dest1[1] -= 1.0f;} // Remove gravity from the y-axis accelerometer bias calculation
	if(dest1[1] < -0.8f) {dest1[1] += 1.0f;} // Remove gravity from the y-axis accelerometer bias calculation
	if(dest1[2] > 0.8f) {dest1[2] -= 1.0f;} // Remove gravity from the z-axis accelerometer bias calculation
	if(dest1[2] < -0.8f) {dest1[2] += 1.0f;} // Remove gravity from the z-axis accelerometer bias calculation
}

void icm_shutdown(struct i2c_dt_spec dev_i2c)
{
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_DEVICE_CONFIG, 0x01); // Don't need to wait for ICM to finish reset
}

uint16_t icm_fifo_read(struct i2c_dt_spec dev_i2c, uint8_t *data)
{
	uint8_t rawCount[2];
	i2c_burst_read_dt(&dev_i2c, ICM42688_FIFO_COUNTH, &rawCount[0], 2);
	uint16_t count = (uint16_t)(rawCount[0] << 8 | rawCount[1]); // Turn the 16 bits into a unsigned 16-bit value
	//LOG_DBG("IMU packet count: %u", count);
	count += 32; // Add a few read buffer packets (4 ms)
	uint16_t packets = count / 8;								 // Packet size 8 bytes
	uint16_t stco = 0;
	uint8_t addr = ICM42688_FIFO_DATA;
	i2c_write_dt(&dev_i2c, &addr, 1); // Start read buffer
	while (count > 0)
	{
		i2c_read_dt(&dev_i2c, &data[stco], count > 248 ? 248 : count); // Read less than 255 at a time (for nRF52832)
		stco += 248;
		count = count > 248 ? count - 248 : 0;
		//LOG_DBG("IMU packets left: %u", count);
	}
	return count;
}

int icm_fifo_process(uint16_t index, uint8_t *data, float g[3])
{
	index *= 8; // Packet size 8 bytes
	if ((data[index] & 0x80) == 0x80)
		return 1; // Skip empty packets
	// combine into 16 bit values
	float raw[3];
	for (int i = 0; i < 3; i++) { // gx, gy, gz
		raw[i] = (int16_t)((((int16_t)data[index + (i * 2) + 1]) << 8) | data[index + (i * 2) + 2]);
//		raw[i] *= 2000.0f/32768.0f;
		raw[i] *= _gRes;
	}
	if (raw[0] < -32766 || raw[1] < -32766 || raw[2] < -32766)
		return 1; // Skip invalid data
	memcpy(g, raw, sizeof(raw));
	return 0;
}