#include <math.h>

#include <zephyr/logging/log.h>

#include "IST8308.h"
#include "IST8306.h" // Common functions
#include "sensor/sensor_none.h"

static const float sensitivity = 0.075; // uT/LSB

static uint8_t last_odr = 0xff;
static float last_real_time = 0.0f;
static int64_t oneshot_trigger_time = 0;

LOG_MODULE_REGISTER(IST8308, LOG_LEVEL_DBG);

int ist8308_init(float time, float *actual_time)
{
	last_odr = 0xff; // reset last odr
//	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, IST8306_ACTR, 0x00); // exit suspend
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, IST8308_CNTL4, DR_200); // set DR
	err |= ist8308_update_odr(time, actual_time);
	return (err < 0 ? err : 0);
}

void ist8308_shutdown(void)
{
	last_odr = 0xff; // reset last odr
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, IST8306_CNTL3, 0x01); // soft reset
//	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, IST8306_ACTR, 0x02); // suspend
	if (err)
		LOG_ERR("Communication error");
}

float ist8308_get_odr(void)
{
	return last_real_time;
}

int ist8308_update_odr(float time, float *actual_time)
{
	int ODR;
	uint8_t NSF;
	uint8_t MODE;
	uint8_t OSR;

	if (time <= 0 || time == INFINITY) // standby mode or single measurement mode
	{
		NSF = NSF_Low; // High Speed
		OSR = OSR_8;
		MODE = MODE_STANDBY;
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
	{
		NSF = NSF_Low; // High Speed
		OSR = OSR_8;
		MODE = MODE_CMM_200Hz;
		time = 1.0 / 200;
	}
	else if (ODR > 50)
	{
		NSF = NSF_Low; // Normal
		OSR = OSR_16;
		MODE = MODE_CMM_100Hz;
		time = 1.0 / 100;
	}
	else if (ODR > 20)
	{
		NSF = NSF_Low; // Normal
		OSR = OSR_16;
		MODE = MODE_CMM_50Hz;
		time = 1.0 / 50;
	}
	else if (ODR > 10)
	{
		NSF = NSF_Low; // Normal
		OSR = OSR_16;
		MODE = MODE_CMM_20Hz;
		time = 1.0 / 20;
	}
	else if (ODR > 0)
	{
		NSF = NSF_Medium; // Low noise
		OSR = OSR_32;
		MODE = MODE_CMM_10Hz;
		time = 1.0 / 10;
	}
	else
	{
		NSF = NSF_Low; // High Speed
		OSR = OSR_8;
		MODE = MODE_SINGLE;
		time = INFINITY;
	}

	if (last_odr == MODE)
		return 1;
	else
		last_odr = MODE;

	if (MODE == MODE_SINGLE)
		MODE = MODE_STANDBY; // set STBY, oneshot will set SMM

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, IST8306_CNTL1, NSF << 5);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, IST8306_CNTL2, MODE);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, IST8306_OSRCNTL, OSR);
	if (err)
		LOG_ERR("Communication error");

	*actual_time = time;
	last_real_time = time;
	return err;
}

void ist8308_mag_oneshot(void)
{
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, IST8306_CNTL2, MODE_SINGLE); // set single measurement mode
	oneshot_trigger_time = k_uptime_get();
	if (err)
		LOG_ERR("Communication error");
}

void ist8308_mag_read(float m[3])
{
	int err = 0;
	uint8_t status = oneshot_trigger_time ? 0x00 : 0x01;
	int64_t timeout = oneshot_trigger_time + 5; // 5ms timeout
	if (k_uptime_get() >= timeout) // already passed timeout
		oneshot_trigger_time = 0;
	while ((~status & 0x01) && k_uptime_get() < timeout) // wait for oneshot to complete or timeout
		err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, IST8306_STAT, &status);
	if (oneshot_trigger_time ? k_uptime_get() >= timeout : false)
		LOG_ERR("Read timeout");
	oneshot_trigger_time = 0;
	uint8_t rawData[6];
	err |= ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, IST8306_DATAXL, &rawData[0], 6);
	if (err)
		LOG_ERR("Communication error");
	ist8308_mag_process(rawData, m);
}

void ist8308_mag_process(uint8_t *raw_m, float m[3])
{
	for (int i = 0; i < 3; i++) // x, y, z
	{
		m[i] = (int16_t)((((uint16_t)raw_m[(i * 2) + 1]) << 8) | raw_m[i * 2]);
		m[i] *= sensitivity; //LSB to uT
		m[i] /= 100; // uT to gauss
	}
}

const sensor_mag_t sensor_mag_ist8308 = {
	*ist8308_init,
	*ist8308_shutdown,

	*ist8308_update_odr,
	*ist8308_get_odr,

	*ist8308_mag_oneshot,
	*ist8308_mag_read,
	*mag_none_temp_read,

	*ist8308_mag_process,
	6, 6, 0, IST8306_DATAXL
};
