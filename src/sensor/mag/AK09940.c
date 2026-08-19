#include <math.h>

#include <zephyr/logging/log.h>

#include "AK09940.h"

static const float sensitivity = 10; // nT/LSB

static uint8_t last_odr = 0xff;
static float last_real_time = 0.0f;
//static uint8_t last_rawTemp = 0xff;
static int64_t oneshot_trigger_time = 0;

LOG_MODULE_REGISTER(AK09940, LOG_LEVEL_DBG);

int ak_init(float time, float *actual_time)
{
	last_odr = 0xff; // reset last odr
	int err = ak_update_odr(time, actual_time);
	return (err < 0 ? err : 0);
}

void ak_shutdown(void)
{
	last_odr = 0xff; // reset last odr
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, AK09940_CNTL4, 0x01);
	if (err)
		LOG_ERR("Communication error");
}

float ak_get_odr(void)
{
	return last_real_time;
}

int ak_update_odr(float time, float *actual_time)
{
	int ODR;
	uint8_t MODE;

	if (time <= 0 || time == INFINITY) // power down mode or single measurement mode
	{
		MODE = MODE_PDM;
		ODR = 0;
	}
	else
	{
		ODR = 1 / time;
	}

	if (time <= 0)
	{
		time = 0; // off
	}
	else if (ODR > 100) // TODO: this sucks
	{ // only up to 200Hz supported with MT_LND2
		MODE = MODE_CMM5_200Hz;
		time = 1.0 / 200;
	}
	else if (ODR > 50)
	{
		MODE = MODE_CMM4_100Hz;
		time = 1.0 / 100;
	}
	else if (ODR > 20)
	{
		MODE = MODE_CMM3_50Hz;
		time = 1.0 / 50;
	}
	else if (ODR > 10)
	{
		MODE = MODE_CMM2_20Hz;
		time = 1.0 / 20;
	}
	else if (ODR > 0)
	{
		MODE = MODE_CMM1_10Hz;
		time = 1.0 / 10;
	}
	else
	{
		MODE = MODE_SMM;
//		time = INFINITY;
		time = 0; // unsure if SMM is working at the needed rate
	}

	if (last_odr == MODE)
		return 1;
	else
		last_odr = MODE;

	if (MODE == MODE_SMM)
		MODE = MODE_PDM; // set PDM, oneshot will set SMM

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, AK09940_CNTL3, MT_LND2 << 5 | MODE);
	if (err)
		LOG_ERR("Communication error");

	*actual_time = time;
	last_real_time = time;
	return err;
}

void ak_mag_oneshot(void)
{
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, AK09940_CNTL3, MT_LND2 << 5 | MODE_SMM); // single measurement mode (does not change MT2)
	oneshot_trigger_time = k_uptime_get();
	if (err)
		LOG_ERR("Communication error");
}

void ak_mag_read(float m[3])
{
	int err = 0;
	uint8_t status = oneshot_trigger_time ? 0x00 : 0x01;
	int64_t timeout = oneshot_trigger_time + 2; // 2ms timeout
	if (k_uptime_get() >= timeout) // already passed timeout
		oneshot_trigger_time = 0;
	while ((~status & 0x01) && k_uptime_get() < timeout) // wait for oneshot to complete or timeout
		err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, AK09940_ST, &status); // polling
	if (oneshot_trigger_time ? k_uptime_get() >= timeout : false)
		LOG_ERR("Read timeout");
	oneshot_trigger_time = 0;
	uint8_t rawData[9];
	err |= ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, AK09940_HXL, &rawData[0], 9);
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, AK09940_ST2, &status); // release protection
	if (err)
		LOG_ERR("Communication error");
	ak_mag_process(rawData, m);
}

float ak_temp_read(float bias[3])
{
	uint8_t rawTemp;
	int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, AK09940_TMPS, &rawTemp);
	// Temperature [˚C] = 30 – (TMPS) / 1.7
	// Measurement data is stored in two’s complement and Little Endian format.
	float temp = (int8_t)rawTemp;
	temp /= -1.7f;
	temp += 30;
	// TODO: see pg.24
	if (err < 0)
		LOG_ERR("Communication error");
	return temp;
}

void ak_mag_process(uint8_t *raw_m, float m[3])
{
	for (int i = 0; i < 3; i++) // x, y, z
	{
		m[i] = (int32_t)((((uint32_t)raw_m[(i * 3) + 2]) << 24) | (((uint32_t)raw_m[(i * 3) + 1]) << 16) | (((uint32_t)raw_m[i * 3]) << 8)) / 256;
		m[i] *= sensitivity; //LSB to nT
		m[i] /= 100000; // nT to gauss
	}
}

const sensor_mag_t sensor_mag_ak09940 = {
	*ak_init,
	*ak_shutdown,

	*ak_update_odr,
	*ak_get_odr,

	*ak_mag_oneshot,
	*ak_mag_read,
	*ak_temp_read,

	*ak_mag_process,
	9, 9, 0, AK09940_HXL
};
