/* 09/23/2017 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Ladybug default), respectively, and it uses the Ladybug STM32L432 Breakout Board.
  The MMC5983MA is a low power magnetometer, here used as 3 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/
#include <math.h>

#include <zephyr/logging/log.h>

#include "MMC5983MA.h"

static const float sensitivity = (1.0f / 16384.0f); // mag sensitivity if using 18 bit data (16384 Counts/G)
static const float offset = 131072.0f; // mag range unsigned to signed

static uint8_t last_odr = 0xff;
static float last_time = 0;
static float last_real_time = 0.0f;
static uint8_t last_rawTemp = 0xff;
static int64_t oneshot_trigger_time = 0;
static bool auto_set_reset = true;

LOG_MODULE_REGISTER(MMC5983MA, LOG_LEVEL_DBG);

static void mmc_SET(void);
static void mmc_RESET(void);

int mmc_init(float time, float *actual_time)
{
	// moved here, it is mmc specific
	mmc_SET();													 // "deGauss" magnetometer

	// enable auto set/reset (bit 5 == 1)
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5983MA_CONTROL_0, 0x20);
	if (err)
		LOG_ERR("Communication error");

	last_odr = 0xff; // reset last odr
	err |= mmc_update_odr(time, actual_time);
	return (err < 0 ? err : 0);
}

void mmc_shutdown(void)
{
	// reset device
	last_odr = 0xff; // reset last odr
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5983MA_CONTROL_1, 0x80); // Don't need to wait for MMC to finish reset
	if (err)
		LOG_ERR("Communication error");
}

float mmc_get_odr(void)
{
	return last_real_time;
}

int mmc_update_odr(float time, float *actual_time)
{
	int ODR;
	uint8_t MODR;
	uint8_t MBW;
	uint8_t MSET = MSET_2000; // always use lowest SET/RESET interval
	last_time = time;

	if (time <= 0 || time == INFINITY) // off interpreted as oneshot
		ODR = 0;
	else
		ODR = 1 / time;

	if (ODR > 200) // TODO: this sucks
	{ // 1000Hz*0.5ms/1000ms = 50% active
		MODR = MODR_1000Hz;
		MBW = MBW_800Hz; // Max MBW_800Hz
		time = 1.0 / 1000;
	}
	else if (ODR > 100) // Nominal working state, this should use as low power as possible
	{ // 200Hz*0.5ms/1000ms = 10% active
		MODR = MODR_200Hz;
		MBW = MBW_800Hz; // Max MBW_200Hz, MBW_800Hz is used since the RMS noise is still only 1.2mG
		time = 1.0 / 200;
	}
	else if (ODR > 50)
	{ // 100Hz*2ms/1000ms = 20% active
		MODR = MODR_100Hz;
		MBW = MBW_400Hz; // 0.8mG
		time = 1.0 / 100;
	}
	else if (ODR > 20)
	{ // 50Hz*4ms/1000ms = 20% active
		MODR = MODR_50Hz;
		MBW = MBW_200Hz; // 0.6mG
		time = 1.0 / 50;
	}
	else if (ODR > 10)
	{ // 20Hz*8ms/1000ms = 16% active
		MODR = MODR_20Hz;
		MBW = MBW_100Hz; // 0.4mG
		time = 1.0 / 20;
	}
	else if (ODR > 1)
	{ // 10Hz*8ms/1000ms = 8% active
		MODR = MODR_10Hz;
		MBW = MBW_100Hz;
		time = 1.0 / 10;
	}
	else if (ODR > 0)
	{ // 1Hz*8ms/1000ms = 0.8% active
		MODR = MODR_1Hz;
		MBW = MBW_100Hz;
		time = 1.0 / 1;
	}
	else
	{
		MODR = MODR_ONESHOT;
		MBW = MBW_800Hz;
		time = INFINITY;
	}

	if (last_odr == MODR)
		return 1;
	else
		last_odr = MODR;

	// set magnetometer bandwidth
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5983MA_CONTROL_1, MBW);

	// enable continuous measurement mode (bit 3 == 1), set sample rate
	// enable automatic Set/Reset (bit 7 == 1), set set/reset rate
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5983MA_CONTROL_2, 0x80 | (MSET << 4) | (MODR ? 0x08 : 0) | MODR);
	if (err)
		LOG_ERR("Communication error");

	*actual_time = time;
	last_real_time = time;
	return err;
}

void mmc_mag_oneshot(void)
{
	// enable auto set/reset (bit 5 == 1) and trigger oneshot
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5983MA_CONTROL_0, (auto_set_reset ? 0x20 : 0) | 0x01);
	oneshot_trigger_time = k_uptime_get();
	if (err)
		LOG_ERR("Communication error");
}

void mmc_mag_read(float m[3])
{
	int err = 0;
	uint8_t status = oneshot_trigger_time ? 0x00 : 0x01;
	int64_t timeout = oneshot_trigger_time + 2; // 2ms timeout
	if (k_uptime_get() >= timeout) // already passed timeout
		oneshot_trigger_time = 0;
	while ((~status & 0x01) && k_uptime_get() < timeout) // wait for oneshot to complete or timeout
		err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, MMC5983MA_STATUS, &status);
	if (oneshot_trigger_time ? k_uptime_get() >= timeout : false)
		LOG_ERR("Read timeout");
	oneshot_trigger_time = 0;
	uint8_t rawData[7]; // x/y/z mag register data stored here
	err |= ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, MMC5983MA_XOUT_0, &rawData[0], 7); // Read the 7 raw data registers into data array
	if (err)
		LOG_ERR("Communication error");
	mmc_mag_process(rawData, m);
}

// MMC must trigger the measurement, which will take significant time
// instead, the temperature is read from the last measurement and then another measurement is immediately triggered
float mmc_temp_read(float bias[3])
{
	uint8_t rawTemp;
	int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, MMC5983MA_TOUT, &rawTemp);
	// Temperature output, unsigned format. The range is -75~125°C, about 0.8°C/LSB, 00000000 stands for -75°C
	float temp = rawTemp;
	temp *= 0.8f;
	temp -= 75;

	// USING SET AND RESET TO REMOVE BRIDGE OFFSET in datasheet
	if (last_rawTemp != rawTemp && last_time > 1.0f / 50) // calculate offset at low motion only
	{ // TODO: does the temp register have hysteresis?
		float mPos[3], mNeg[3];
		float actual_time;

		last_rawTemp = rawTemp;
		for (int i = 0; i < 3; i++) // clear stored offset
			bias[i] = 0;

		err |= mmc_update_odr(INFINITY, &actual_time); // set oneshot mode
		auto_set_reset = false; // disable auto set/reset

		mmc_RESET();
		mmc_mag_oneshot();
		mmc_mag_read(mNeg);

		mmc_SET();
		mmc_mag_oneshot();
		mmc_mag_read(mPos);

		for (int i = 0; i < 3; i++) // store bridge offset for future readings
			bias[i] = (mPos[i] + mNeg[i]) / 2; // ((+H + Offset) + (-H + Offset)) / 2

		auto_set_reset = true; // enable auto set/reset
		err |= mmc_update_odr(last_time, &actual_time); // reset odr
	}

	// enable auto set/reset (bit 5 == 1) and trigger measurement
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5983MA_CONTROL_0, 0x20 | 0x02);
	if (err < 0)
		LOG_ERR("Communication error");
	return temp;
}

void mmc_mag_process(uint8_t *raw_m, float m[3])
{
	uint32_t rawMag[3];
	rawMag[0] = (uint32_t)(raw_m[0] << 10 | raw_m[1] << 2 | (raw_m[6] & 0xC0) >> 6); // Turn the 18 bits into a unsigned 32-bit value
	rawMag[1] = (uint32_t)(raw_m[2] << 10 | raw_m[3] << 2 | (raw_m[6] & 0x30) >> 4); // Turn the 18 bits into a unsigned 32-bit value
	rawMag[2] = (uint32_t)(raw_m[4] << 10 | raw_m[5] << 2 | (raw_m[6] & 0x0C) >> 2); // Turn the 18 bits into a unsigned 32-bit value
	for (int i = 0; i < 3; i++) // x, y, z
		m[i] = ((float)rawMag[i] - offset) * sensitivity;
}

static void mmc_SET(void)
{
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5983MA_CONTROL_0, 0x08);
	if (err)
		LOG_ERR("Communication error");
	k_busy_wait(1); // self clearing after 500 ns
}

static void mmc_RESET(void)
{
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5983MA_CONTROL_0, 0x10);
	if (err)
		LOG_ERR("Communication error");
	k_busy_wait(1); // self clearing after 500 ns
}

const sensor_mag_t sensor_mag_mmc5983ma = {
	*mmc_init,
	*mmc_shutdown,

	*mmc_update_odr,
	*mmc_get_odr,

	*mmc_mag_oneshot,
	*mmc_mag_read,
	*mmc_temp_read,

	*mmc_mag_process,
	6, 7, // if only reading 6 bytes, the data will be lower precision
	0, MMC5983MA_XOUT_0
};
