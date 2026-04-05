/* 01/14/2022 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Ladybug default), respectively, and it uses the Ladybug STM32L432 Breakout Board.
  The ICM42688 is a combo sensor with embedded accel and gyro, here used as 6 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/
#include <math.h>

#include <zephyr/logging/log.h>
#include <hal/nrf_gpio.h>

#include "ICM42688.h"
#include "sensor/sensor_none.h"

#define PACKET_SIZE 20

static const float accel_sensitivity = 16.0f / 32768.0f; // Always 16G
static const float gyro_sensitivity = 2000.0f / 32768.0f; // Always 2000dps

static const float accel_sensitivity_32 = 16.0f / ((uint32_t)2<<30); // 16G forced
static const float gyro_sensitivity_32 = 2000.0f / ((uint32_t)2<<30); // 2000dps forced

static const uint16_t intervals[] = {1, 2, 4, 8, 16, 32, 64, 160, 320, 640, 1280, 2560, 0};
static const uint8_t odrs[] = {AODR_32kHz, AODR_16kHz, AODR_8kHz, AODR_4kHz, AODR_2kHz, AODR_1kHz, AODR_500Hz, AODR_200Hz, AODR_100Hz, AODR_50Hz, AODR_25Hz, AODR_12_5Hz}; // same binary

static uint8_t last_accel_odr = 0xff;
static uint8_t last_gyro_odr = 0xff;
static const float clock_reference = 32000;
static float clock_scale = 1; // ODR is scaled by clock_rate/clock_reference

#define FIFO_MULT 0.00075f // assuming i2c fast mode
#define FIFO_MULT_SPI 0.0001f // ~24MHz

static float fifo_multiplier_factor = FIFO_MULT;
static float fifo_multiplier = 0;

LOG_MODULE_REGISTER(ICM42688, LOG_LEVEL_DBG);

int icm_init(float clock_rate, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	// setup interface for SPI
	if (!sensor_interface_spi_configure(SENSOR_INTERFACE_DEV_IMU, MHZ(24), 0))
		fifo_multiplier_factor = FIFO_MULT_SPI; // SPI mode
	else
		fifo_multiplier_factor = FIFO_MULT; // I2C mode
	int err = 0;
//	ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_INT_SOURCE0, 0x00); // disable default interrupt (RESET_DONE)
	err |= ssi_reg_update_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_INTF_CONFIG0, 0x40, 0x40); // FIFO_COUNT and FIFO_WM use records
	if (clock_rate > 0)
	{
		clock_scale = clock_rate / clock_reference;
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_REG_BANK_SEL, 0x01); // select register bank 1
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_INTF_CONFIG5, 0x04); // use CLKIN (set PIN9_FUNCTION to CLKIN)
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
		err |= ssi_reg_update_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_INTF_CONFIG1, 0x04, 0x04); // use CLKIN (set RTC_MODE to require RTC clock input)
//		ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_INTF_CONFIG1, 0x91 | 0x04); // use CLKIN (set RTC_MODE to require RTC clock input)
	}
	last_accel_odr = 0xff; // reset last odr
	last_gyro_odr = 0xff; // reset last odr
	err |= icm_update_odr(accel_time, gyro_time, accel_actual_time, gyro_actual_time);
	// TODO: I can't remember why bandwidth was set to ODR/10, make sure to test this
//	ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_GYRO_ACCEL_CONFIG0, 0x44); // set gyro and accel bandwidth to ODR/10
//	k_msleep(50); // 10ms Accel, 30ms Gyro startup
	k_msleep(1); // fuck i dont wanna wait that long
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_FIFO_CONFIG1, 0x10); // enable FIFO hires, a+g
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_FIFO_CONFIG, 1<<6); // begin FIFO stream
	if (err)
		LOG_ERR("Communication error");
	return (err < 0 ? err : 0);
}

void icm_shutdown(void)
{
	last_accel_odr = 0xff; // reset last odr
	last_gyro_odr = 0xff; // reset last odr
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_DEVICE_CONFIG, 0x01); // Don't need to wait for ICM to finish reset
	if (err)
		LOG_ERR("Communication error");
}

void icm_update_fs(float accel_range, float gyro_range, float *accel_actual_range, float *gyro_actual_range)
{
	*accel_actual_range = 16; // always 16g in hires
	*gyro_actual_range = 2000; // always 2000dps in hires
}

int icm_update_odr(float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	int interval;
	uint8_t Ascale = AFS_16G; // set highest
	uint8_t Gscale = GFS_2000DPS; // set highest
	uint8_t aMode;
	uint8_t gMode;
	uint8_t AODR = 0;
	uint8_t GODR = 0;

	// Calculate accel
	if (accel_time <= 0 || accel_time == INFINITY) // off, standby interpreted as off
	{
		aMode = aMode_OFF;
		accel_time = 0;
	}
	else
	{
		aMode = aMode_LN;
		interval = accel_time * clock_scale * 32000; // scale clock
		for (int i = 1; i < ARRAY_SIZE(intervals); i++)
		{
			if (intervals[i] && interval >= intervals[i])
				continue;
			AODR = odrs[i - 1];
			accel_time = intervals[i - 1] / 32000.0f;
			break;
		}
	}
	accel_time /= clock_scale; // scale clock

	// Calculate gyro
	if (gyro_time <= 0) // off
	{
		gMode = gMode_OFF;
		gyro_time = 0;
	}
	else if (gyro_time == INFINITY) // standby
	{
		gMode = gMode_SBY;
		gyro_time = 0;
	}
	else
	{
		gMode = gMode_LN;
		interval = gyro_time * clock_scale * 32000; // scale clock
		for (int i = 1; i < ARRAY_SIZE(intervals); i++)
		{
			if (intervals[i] && interval >= intervals[i])
				continue;
			GODR = odrs[i - 1];
			accel_time = intervals[i - 1] / 32000.0f;
			break;
		}
	}
	gyro_time /= clock_scale; // scale clock

	if (last_accel_odr == AODR && last_gyro_odr == GODR) // if both were already configured
		return 1;

	int err = 0;
	// only if the power mode has changed
	if (last_accel_odr == 0xff || last_gyro_odr == 0xff || (last_accel_odr == 0 ? 0 : 1) != (AODR == 0 ? 0 : 1) || (last_gyro_odr == 0 ? 0 : 1) != (GODR == 0 ? 0 : 1))
	{ // TODO: can't tell difference between gyro off and gyro standby
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_PWR_MGMT0, gMode << 2 | aMode); // set accel and gyro modes
		k_busy_wait(250); // wait >200us (datasheet 14.36)
	}
	last_accel_odr = AODR;
	last_gyro_odr = GODR;

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_ACCEL_CONFIG0, Ascale << 5 | AODR); // set accel ODR and FS
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_GYRO_CONFIG0, Gscale << 5 | GODR); // set gyro ODR and FS
	if (err)
		LOG_ERR("Communication error");

	*accel_actual_time = accel_time;
	*gyro_actual_time = gyro_time;

	// extra read packets by ODR time
	if (accel_time == 0 && gyro_time != 0)
		fifo_multiplier = fifo_multiplier_factor / gyro_time;
	else if (accel_time != 0 && gyro_time == 0)
		fifo_multiplier = fifo_multiplier_factor / accel_time;
	else if (gyro_time > accel_time)
		fifo_multiplier = fifo_multiplier_factor / accel_time;
	else if (accel_time > gyro_time)
		fifo_multiplier = fifo_multiplier_factor / gyro_time;
	else
		fifo_multiplier = 0;

	return 0;
}

uint16_t icm_fifo_read(uint8_t *data, uint16_t len)
{
	int err = 0;
	uint16_t total = 0;
	uint16_t packets = UINT16_MAX;
	while (packets > 0 && len >= PACKET_SIZE)
	{
		uint8_t rawCount[2];
		err |= ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, ICM42688_FIFO_COUNTH, &rawCount[0], 2);
		packets = (uint16_t)(rawCount[0] << 8 | rawCount[1]); // Turn the 16 bits into a unsigned 16-bit value
		if (!packets) // nothing to do
			break;
		float extra_read_packets = packets * fifo_multiplier;
		packets += extra_read_packets;
		uint16_t count = packets * PACKET_SIZE;
		uint16_t limit = len / PACKET_SIZE;
		if (packets > limit)
		{
			LOG_WRN("FIFO read buffer limit reached, %d packets dropped", packets - limit);
			packets = limit;
			count = packets * PACKET_SIZE;
		}
		err |= ssi_burst_read_interval(SENSOR_INTERFACE_DEV_IMU, ICM42688_FIFO_DATA, data, count, PACKET_SIZE);
		if (err)
			LOG_ERR("Communication error");
		data += packets * PACKET_SIZE;
		len -= packets * PACKET_SIZE;
		total += packets;
	}
	return total;
}

static const uint8_t invalid[6] = {0x80, 0x00, 0x80, 0x00, 0x80, 0x00};

int icm_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3])
{
	index *= PACKET_SIZE;
	if ((data[index] & 0x80) == 0x80)
		return 1; // Skip empty packets
	if ((data[index] & 0x7F) == 0x7F)
		return 1; // Skip empty packets
	// combine into 20 bit values in 32 bit int
	float a_raw[3] = {0};
	float g_raw[3] = {0};
	if (memcmp(&data[index + 1], invalid, sizeof(invalid))) // valid accel data
	{
		for (int i = 0; i < 3; i++) // accel x, y, z
			a_raw[i] = (int32_t)((((uint32_t)data[index + 1 + (i * 2)]) << 24) | (((uint32_t)data[index + 2 + (i * 2)]) << 16) | (((uint32_t)data[index + 17 + i] & 0xF0) << 8));
	}
	if (memcmp(&data[index + 7], invalid, sizeof(invalid))) // valid gyro data
	{
		for (int i = 0; i < 3; i++) // gyro x, y, z
			g_raw[i] = (int32_t)((((uint32_t)data[index + 7 + (i * 2)]) << 24) | (((uint32_t)data[index + 8 + (i * 2)]) << 16) | (((uint32_t)data[index + 17 + i] & 0x0F) << 12));
	}
	else if (!memcmp(&data[index + 1], invalid, sizeof(invalid))) // Skip invalid data
	{
		return 1;
	}
	for (int i = 0; i < 3; i++) // x, y, z
	{
		a_raw[i] *= accel_sensitivity_32;
		g_raw[i] *= gyro_sensitivity_32;
	}
	memcpy(a, a_raw, sizeof(a_raw));
	memcpy(g, g_raw, sizeof(g_raw));
	return 0;
}

void icm_accel_read(float a[3])
{
	uint8_t rawAccel[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, ICM42688_ACCEL_DATA_X1, &rawAccel[0], 6);
	if (err)
		LOG_ERR("Communication error");
	for (int i = 0; i < 3; i++) // x, y, z
	{
		a[i] = (int16_t)((((uint16_t)rawAccel[i * 2]) << 8) | rawAccel[1 + (i * 2)]);
		a[i] *= accel_sensitivity;
	}
}

void icm_gyro_read(float g[3])
{
	uint8_t rawGyro[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, ICM42688_GYRO_DATA_X1, &rawGyro[0], 6);
	if (err)
		LOG_ERR("Communication error");
	for (int i = 0; i < 3; i++) // x, y, z
	{
		g[i] = (int16_t)((((uint16_t)rawGyro[i * 2]) << 8) | rawGyro[1 + (i * 2)]);
		g[i] *= gyro_sensitivity;
	}
}

int icm_temp_read(float *data)
{
	uint8_t rawTemp[2];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, ICM42688_TEMP_DATA1, &rawTemp[0], 2);
	if (err)
	{
		LOG_ERR("Communication error");
		return -1;
	}
	// Temperature in Degrees Centigrade = (TEMP_DATA / 132.48) + 25
	*data = (int16_t)((((uint16_t)rawTemp[0]) << 8) | rawTemp[1]);
	*data /= 132.48f;
	*data += 25;
	return 0;
}

uint8_t icm_setup_DRDY(uint16_t threshold)
{
	uint8_t buf[2];
	buf[0] = threshold & 0xFF;
	buf[1] = (threshold >> 8) & 0x0F;
	int err = ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM42688_FIFO_CONFIG2, buf, 2);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_INT_SOURCE0, 0x04); // FIFO threshold interrupt
	if (err)
		LOG_ERR("Communication error");
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low
}

uint8_t icm_setup_WOM(void)
{
	uint8_t interrupts;
	int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_INT_STATUS, &interrupts); // clear reset done int flag
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_INT_SOURCE0, 0x00); // disable default interrupt (RESET_DONE)
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_ACCEL_CONFIG0, AFS_8G << 5 | AODR_200Hz); // set accel ODR and FS
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_PWR_MGMT0, aMode_LP); // set accel and gyro modes
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_INTF_CONFIG1, 0x00); // set low power clock
	k_msleep(1);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_REG_BANK_SEL, 0x04); // select register bank 4
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_ACCEL_WOM_X_THR, 0x08); // set wake thresholds // 8 x 3.9 mg is ~31.25 mg
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_ACCEL_WOM_Y_THR, 0x08); // set wake thresholds
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_ACCEL_WOM_Z_THR, 0x08); // set wake thresholds
	k_msleep(1);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_INT_SOURCE1, 0x07); // enable WOM interrupt
	k_msleep(50); // TODO: does this need to be 50ms?
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_SMD_CONFIG, 0x01); // enable WOM feature
	if (err)
		LOG_ERR("Communication error");
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low
}

const sensor_imu_t sensor_imu_icm42688 = {
	*icm_init,
	*icm_shutdown,

	*icm_update_fs,
	*icm_update_odr,

	*icm_fifo_read,
	*icm_fifo_process,
	*icm_accel_read,
	*icm_gyro_read,
	*icm_temp_read,

	*icm_setup_DRDY,
	*icm_setup_WOM,

	*imu_none_ext_setup,
	*imu_none_ext_passthrough
};
