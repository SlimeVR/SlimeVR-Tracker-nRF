#include <math.h>

#include <zephyr/logging/log.h>

#include "LIS3MDL.h"

static const float sensitivity = 1.0 / 3421; // Always 8G (FS = ±8 gauss: 3421 LSB/Gauss)

static uint8_t last_odr = 0xff;
static float last_real_time = 0.0f;

LOG_MODULE_REGISTER(LIS3MDL, LOG_LEVEL_DBG);

int lis3_init(float time, float *actual_time)
{
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, LIS3MDL_CTRL_REG1, 0x80); // enable temp sensor
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, LIS3MDL_CTRL_REG2, FS_8G << 5);
	if (err)
		LOG_ERR("Communication error");
	last_odr = 0xff; // reset last odr
	err |= lis3_update_odr(time, actual_time);
	return (err < 0 ? err : 0);
}

void lis3_shutdown(void)
{
	last_odr = 0xff; // reset last odr
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, LIS3MDL_CTRL_REG2, 0x04);
	if (err)
		LOG_ERR("Communication error");
}

float lis3_get_odr(void)
{
	return last_real_time;
}

int lis3_update_odr(float time, float *actual_time)
{
	int ODR;
	uint8_t OM;
	uint8_t DO;
	uint8_t FAST_ODR;
	uint8_t MD;

	if (time <= 0) // off
	{
		MD = MD_POWER_DOWN;
		ODR = 0;
	}
	else if (time == INFINITY) // oneshot/single
	{
//		MD = MD_SINGLE_CONV;
//		ODR = 0;
		MD = MD_POWER_DOWN; // Single measurement only up to 80Hz, so it is useless
		ODR = 0;
	}
	else
	{
		OM = OM_UHP;
		DO = 0;
		MD = MD_CONTINUOUS_CONV;
		ODR = 1 / time;
	}

	if (MD == MD_POWER_DOWN)
	{
		time = 0; // off
	}
	else if (ODR > 560) // TODO: this sucks
	{
		OM = OM_LP;
		FAST_ODR = 1;
		time = 1.0 / 1000;
	}
	else if (ODR > 300)
	{
		OM = OM_MP;
		FAST_ODR = 1;
		time = 1.0 / 560;
	}
	else if (ODR > 155)
	{
		OM = OM_HP;
		FAST_ODR = 1;
		time = 1.0 / 300;
	}
	else if (ODR > 80)
	{
		OM = OM_UHP;
		FAST_ODR = 1;
		time = 1.0 / 155;
	}
	else if (ODR > 40)
	{
		DO = DO_80Hz;
		time = 1.0 / 80;
	}
	else if (ODR > 20)
	{
		DO = DO_40Hz;
		time = 1.0 / 40;
	}
	else if (ODR > 10)
	{
		DO = DO_20Hz;
		time = 1.0 / 20;
	}
	else if (ODR > 5)
	{
		DO = DO_10Hz;
		time = 1.0 / 10;
	}
	else if (ODR > 2.5)
	{
		DO = DO_5Hz;
		time = 1.0 / 5;
	}
	else if (ODR > 1.25)
	{
		DO = DO_2_5Hz;
		time = 1.0 / 2.5;
	}
	else if (ODR > 0.625)
	{
		DO = DO_1_25Hz;
		time = 1.0 / 1.25;
	}
	else if (ODR > 0)
	{
		DO = DO_0_625Hz;
		time = 1.0 / 0.625;
	}
	else
	{
		DO = 0;
		time = INFINITY;
	}

	uint8_t ctrl = OM << 5 | DO << 2 | FAST_ODR << 1;
	if (last_odr == ctrl)
		return 1;
	else
		last_odr = ctrl;

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, LIS3MDL_CTRL_REG1, 0x80 | ctrl); // temp, X/Y operating mode, and ODR
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, LIS3MDL_CTRL_REG3, MD); // set measurement mode
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, LIS3MDL_CTRL_REG4, OM << 2); // set Z-axis operating mode
	if (err)
		LOG_ERR("Communication error");

	*actual_time = time;
	last_real_time = time;
	return err;
}

void lis3_mag_oneshot(void)
{
	// write MD_SINGLE again to trigger a measurement (not clear in datasheet?)
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, LIS3MDL_CTRL_REG3, MD_SINGLE_CONV);
	if (err)
		LOG_ERR("Communication error");
}

void lis3_mag_read(float m[3])
{
	int err = 0;
	uint8_t status;
	while ((status & 0x03) == MD_SINGLE_CONV) // wait for oneshot to complete
		err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, LIS3MDL_CTRL_REG3, &status);
	uint8_t rawData[6];
	err |= ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, LIS3MDL_OUT_X_L, &rawData[0], 6);
	if (err)
		LOG_ERR("Communication error");
	lis3_mag_process(rawData, m);
}

float lis3_temp_read(float bias[3])
{
	uint8_t rawTemp[2];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, LIS3MDL_TEMP_OUT_L, &rawTemp[0], 2);
	// The output value is expressed as a signed 16-bit byte in two’s complement.
	// The four most significant bits contain a copy of the sign bit.
	// The nominal sensitivity is 8 LSB/°C
	float temp = (int16_t)((((uint16_t)rawTemp[1]) << 8) | rawTemp[0]);
	temp /= 8;
	// No value offset?
	if (err)
		LOG_ERR("Communication error");
	return temp;
}

void lis3_mag_process(uint8_t *raw_m, float m[3])
{
	for (int i = 0; i < 3; i++) // x, y, z
	{
		m[i] = (int16_t)((((uint16_t)raw_m[(i * 2) + 1]) << 8) | raw_m[i * 2]);
		m[i] *= sensitivity;
	}
}

const sensor_mag_t sensor_mag_lis3mdl = {
	*lis3_init,
	*lis3_shutdown,

	*lis3_update_odr,
	*lis3_get_odr,

	*lis3_mag_oneshot,
	*lis3_mag_read,
	*lis3_temp_read,

	*lis3_mag_process,
	6, 6, 0, LIS3MDL_OUT_X_L
};
